

// Frame scheduler implementation: slot creation/teardown, fence/timeline
// waiting, GPU timestamp harvesting, and slot rotation.

#if defined(ENGINE_RENDER_BACKEND_VULKAN)

#include "vulkan_frames.hpp"
#include "vulkan_device.hpp"

#include <array>
#include <cstdint>
#include <cstdio>
#include <vulkan/vulkan_core.h>

namespace engine
{
namespace render
{

// One command pool/buffer pair per slot plus semaphores and (signaled) slot
// fences. GPU timing needs device timestamp support and a positive period.
bool VulkanFrameScheduler::create(const VulkanDeviceContext &device, std::uint32_t framesInFlight, bool enableGpuTiming)
{
    device_                       = &device;
    const VkDevice vulkanDevice   = device.device();
    gpuTimingAvailable_           = false;
    lastCompletedGpuMilliseconds_ = 0.0;
    timestampValidBits_           = device.profile().timestampValidBits;
    timestampPeriodNanoseconds_   = device.profile().timestampPeriodNanoseconds;
    gpuMilliseconds_.assign(framesInFlight, 0.0);
    if (enableGpuTiming && device.profile().timestampsSupported && timestampValidBits_ != 0u
        && timestampPeriodNanoseconds_ > 0.0f)
    {
        const VkQueryPoolCreateInfo queryInfo{
            .sType              = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO,
            .pNext              = nullptr,
            .flags              = 0,
            .queryType          = VK_QUERY_TYPE_TIMESTAMP,
            .queryCount         = framesInFlight * 2u,
            .pipelineStatistics = 0,
        };
        if (vkCreateQueryPool(vulkanDevice, &queryInfo, nullptr, &timestampQueries_) == VK_SUCCESS)
            gpuTimingAvailable_ = true;
        else
            std::fprintf(stderr, "render module: GPU timestamp query pool unavailable; timing disabled\n");
    }

    // Two timestamps per slot: TOP_OF_PIPE at beginGpuTiming, BOTTOM_OF_PIPE
    // at endGpuTiming; the difference is the frame's GPU duration.
    commandPools_.assign(framesInFlight, VK_NULL_HANDLE);
    commands_.assign(framesInFlight, VK_NULL_HANDLE);
    const VkCommandPoolCreateInfo poolInfo{
        .sType            = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
        .pNext            = nullptr,
        .flags            = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT,
        .queueFamilyIndex = device.graphicsFamily(),
    };
    for (std::uint32_t index = 0; index < framesInFlight; ++index)
    {
        if (vkCreateCommandPool(vulkanDevice, &poolInfo, nullptr, &commandPools_[index]) != VK_SUCCESS)
        {
            std::fprintf(stderr, "render module: could not create command pool for frame slot %u\n", index);
            return false;
        }
        const VkCommandBufferAllocateInfo commandAllocate{
            .sType              = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
            .pNext              = nullptr,
            .commandPool        = commandPools_[index],
            .level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
            .commandBufferCount = 1,
        };
        if (vkAllocateCommandBuffers(vulkanDevice, &commandAllocate, &commands_[index]) != VK_SUCCESS)
        {
            std::fprintf(stderr, "render module: could not allocate command buffer for frame slot %u\n", index);
            return false;
        }
    }

    imageAvailable_.resize(framesInFlight, VK_NULL_HANDLE);
    inFlight_.resize(framesInFlight, VK_NULL_HANDLE);
    frameTimelineValues_.assign(framesInFlight, 0);
    const VkSemaphoreCreateInfo semaphoreInfo{
        .sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO,
        .pNext = nullptr,
        .flags = 0,
    };

    const VkFenceCreateInfo fenceInfo{
        .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO,
        .pNext = nullptr,
        .flags = VK_FENCE_CREATE_SIGNALED_BIT,
    };
    for (std::uint32_t index = 0; index < framesInFlight; ++index)
    {
        if (vkCreateSemaphore(vulkanDevice, &semaphoreInfo, nullptr, &imageAvailable_[index]) != VK_SUCCESS
            || vkCreateFence(vulkanDevice, &fenceInfo, nullptr, &inFlight_[index]) != VK_SUCCESS)
        {
            std::fprintf(stderr, "render module: could not create synchronization objects\n");
            return false;
        }
    }
    return true;
}

void VulkanFrameScheduler::destroy()
{
    if (device_ == nullptr)
    {
        return;
    }
    const VkDevice vulkanDevice = device_->device();
    if (timestampQueries_ != VK_NULL_HANDLE)
    {
        vkDestroyQueryPool(vulkanDevice, timestampQueries_, nullptr);
        timestampQueries_ = VK_NULL_HANDLE;
    }
    gpuTimingAvailable_           = false;
    lastCompletedGpuMilliseconds_ = 0.0;
    gpuMilliseconds_.clear();
    for (VkSemaphore semaphore : imageAvailable_)
    {
        vkDestroySemaphore(vulkanDevice, semaphore, nullptr);
    }
    imageAvailable_.clear();
    for (VkFence fence : inFlight_)
    {
        vkDestroyFence(vulkanDevice, fence, nullptr);
    }
    inFlight_.clear();
    for (VkCommandPool commandPool : commandPools_)
    {
        if (commandPool != VK_NULL_HANDLE)
            vkDestroyCommandPool(vulkanDevice, commandPool, nullptr);
    }
    commandPools_.clear();
    commands_.clear();
    device_ = nullptr;
}

// Waits for the slot's previous submission (timeline value when the device
// semaphore exists, else the slot fence), converts the two harvested
// timestamps into milliseconds (masked to the timestamp's valid bits because
// counters wrap), then resets the slot's command pool for re-recording.
bool VulkanFrameScheduler::waitForFrameSlot()
{
    if (device_ == nullptr || currentFrame_ >= inFlight_.size() || currentFrame_ >= commandPools_.size())
        return false;

    if (device_->timelineSemaphore() != VK_NULL_HANDLE && currentFrame_ < frameTimelineValues_.size())
    {
        const std::uint64_t wv = frameTimelineValues_[currentFrame_];
        if (wv > 0 && !device_->waitTimeline(wv))
            return false;
    }
    else
    {
        if (vkWaitForFences(device_->device(), 1, &inFlight_[currentFrame_], VK_TRUE, UINT64_MAX) != VK_SUCCESS)
            return false;
    }
    if (gpuTimingAvailable_ && timestampQueries_ != VK_NULL_HANDLE)
    {
        std::array<std::uint64_t, 2> timestamps{};
        const VkResult               result = vkGetQueryPoolResults(device_->device(),
            timestampQueries_,
            currentFrame_ * 2u,
            2u,
            sizeof(timestamps),
            timestamps.data(),
            sizeof(std::uint64_t),
            VK_QUERY_RESULT_64_BIT);
        if (result == VK_SUCCESS)
        {
            std::uint64_t elapsed = timestamps[1] - timestamps[0];
            if (timestampValidBits_ < 64u)
                elapsed &= (std::uint64_t{1} << timestampValidBits_) - 1u;
            gpuMilliseconds_[currentFrame_]
                = static_cast<double>(elapsed) * static_cast<double>(timestampPeriodNanoseconds_) / 1'000'000.0;
            lastCompletedGpuMilliseconds_ = gpuMilliseconds_[currentFrame_];
        }
    }

    if (vkResetCommandPool(device_->device(), commandPools_[currentFrame_], 0) != VK_SUCCESS)
    {
        std::fprintf(stderr, "render module: could not reset command pool for frame slot %u\n", currentFrame_);
        return false;
    }
    return true;
}

void VulkanFrameScheduler::beginGpuTiming()
{
    if (!gpuTimingAvailable_ || timestampQueries_ == VK_NULL_HANDLE)
        return;
    const VkCommandBuffer command    = commandBuffer();
    const std::uint32_t   firstQuery = currentFrame_ * 2u;
    vkCmdResetQueryPool(command, timestampQueries_, firstQuery, 2u);
    vkCmdWriteTimestamp(command, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, timestampQueries_, firstQuery);
}

void VulkanFrameScheduler::endGpuTiming()
{
    if (!gpuTimingAvailable_ || timestampQueries_ == VK_NULL_HANDLE)
        return;
    vkCmdWriteTimestamp(
        commandBuffer(), VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, timestampQueries_, currentFrame_ * 2u + 1u);
}

void VulkanFrameScheduler::resetFrameSlotFence()
{
    vkResetFences(device_->device(), 1, &inFlight_[currentFrame_]);
}

void VulkanFrameScheduler::rotate()
{
    currentFrame_ = (currentFrame_ + 1) % framesInFlight();
}

// One fence wait per slot; used at destruction so teardown cannot race the
// GPU.
void VulkanFrameScheduler::waitForAll()
{
    if (device_ == nullptr)
    {
        return;
    }
    for (VkFence fence : inFlight_)
    {
        if (fence != VK_NULL_HANDLE)
        {
            vkWaitForFences(device_->device(), 1, &fence, VK_TRUE, UINT64_MAX);
        }
    }
}

} // namespace render
} // namespace engine

#endif
