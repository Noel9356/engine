// Per-slot frame pacing for the Vulkan backend: N frame slots, each with its
// own command pool/buffer, an image-available semaphore, and an in-flight
// fence. Also owns the GPU timestamp query pool backing the reported
// gpuFrameMilliseconds. Main thread only.

#ifndef ENGINE_RENDER_MODULE_VULKAN_FRAMES_HPP
#define ENGINE_RENDER_MODULE_VULKAN_FRAMES_HPP

#if defined(ENGINE_RENDER_BACKEND_VULKAN)
#include <cstdint>
#include <vector>
#include <vulkan/vulkan.h>
#include <vulkan/vulkan_core.h>

namespace engine
{
namespace render
{

class VulkanDeviceContext;

// Slot rotation model (called from VulkanBackend::beginFrame/endFrame):
//   waitForFrameSlot -> wait this slot's fence/timeline, harvest timestamps,
//                       reset the slot's command pool
//   ... record draws ...
//   resetFrameSlotFence before submit; rotate() advances to the next slot.
class VulkanFrameScheduler
{
public:
    VulkanFrameScheduler() = default;

    // Allocates pools/buffers/semaphores/fences for framesInFlight slots and
    // (when device + config allow) the timestamp query pool.
    bool create(const VulkanDeviceContext &device, std::uint32_t framesInFlight, bool enableGpuTiming);

    void destroy();

    // Blocks until the current slot's previous submission has finished
    // (fence, or timeline value when the device semaphore exists), reads the
    // slot's GPU timestamps, and resets its command pool. Returns false when
    // waiting or the pool reset failed.
    bool waitForFrameSlot();
    void resetFrameSlotFence(); // unfence the slot just before resubmitting

    // GPU timing: vkCmdWriteTimestamp at top/bottom of pipe around the frame,
    // results converted with the device's timestamp period (nanoseconds to
    // milliseconds, masked to the valid bit width).
    void               beginGpuTiming();
    void               endGpuTiming();
    [[nodiscard]] bool gpuTimingAvailable() const noexcept
    {
        return gpuTimingAvailable_;
    }
    [[nodiscard]] double gpuMilliseconds() const noexcept
    {
        return lastCompletedGpuMilliseconds_;
    }

    std::uint32_t currentFrame() const
    {
        return currentFrame_; // index of the slot being recorded
    }

    std::uint32_t framesInFlight() const
    {
        return static_cast<std::uint32_t>(commands_.size());
    }

    VkCommandBuffer commandBuffer() const
    {
        return commands_[currentFrame_]; // valid only between beginFrame and endFrame
    }

    VkSemaphore imageAvailableSemaphore() const
    {
        return imageAvailable_[currentFrame_];
    }

    VkFence inFlightFence() const
    {
        return inFlight_[currentFrame_];
    }

    void rotate();

    void waitForAll();

private:
    const VulkanDeviceContext *device_ = nullptr;

    std::vector<VkCommandPool>   commandPools_;
    std::vector<VkCommandBuffer> commands_;

    std::vector<VkSemaphore>   imageAvailable_;      // per slot: acquire-side semaphore
    std::vector<VkFence>       inFlight_;            // per slot: signals when the slot's GPU work is done
    std::vector<std::uint64_t> frameTimelineValues_; // per slot: timeline value when the device semaphore is used

public:
    std::vector<std::uint64_t> &frameTimelineValues()
    {
        return frameTimelineValues_;
    }

private:
    VkQueryPool         timestampQueries_ = VK_NULL_HANDLE;
    std::vector<double> gpuMilliseconds_;
    double              lastCompletedGpuMilliseconds_ = 0.0;
    std::uint32_t       timestampValidBits_           = 0;
    float               timestampPeriodNanoseconds_   = 0.0f;
    bool                gpuTimingAvailable_           = false;

    std::uint32_t currentFrame_ = 0;
};

} // namespace render
} // namespace engine

#endif

#endif
