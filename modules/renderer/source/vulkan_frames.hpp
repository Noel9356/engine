// Per-slot frame pacing for the Vulkan backend: N frame slots, each with its
// own command pool/buffer, an image-available semaphore, and an in-flight
// fence. Also owns the GPU timestamp query pool backing the reported
// gpuFrameMilliseconds. Main thread only.

#ifndef ENGINE_RENDERER_MODULE_VULKAN_FRAMES_HPP
#define ENGINE_RENDERER_MODULE_VULKAN_FRAMES_HPP

#if defined(ENGINE_RENDERER_BACKEND_VULKAN)
#include <chrono>
#include <cstdint>
#include <vector>
#include <vulkan/vulkan.h>
#include <vulkan/vulkan_core.h>

namespace engine
{
namespace renderer
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

    // Waits (bounded) for the current slot's previous submission to finish
    // (fence, or timeline value when the device semaphore exists), reads the
    // slot's GPU timestamps, and resets its command pool. Returns false when
    // the wait timed out or the pool reset failed. The wait is capped at one
    // second: a wedged or glacially slow driver must never pin the frame loop
    // forever - the caller skips the frame and keeps polling window events,
    // which is what keeps the OS from declaring the window unresponsive.
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

    // The slot's in-flight fence, waiting on which is exactly "this slot's
    // previous submission finished" (used by the deferred-export flush; see
    // vulkan_backend.cpp). Null when slot is out of range.
    VkFence inFlightFenceFor(std::uint32_t slot) const
    {
        return slot < inFlight_.size() ? inFlight_[slot] : VK_NULL_HANDLE;
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

    // Rate-limits the "slot did not retire in time" log so a persistently
    // slow driver produces one line every few seconds instead of a flood.
    std::chrono::steady_clock::time_point lastSlotStallLog_{};

    std::uint32_t currentFrame_ = 0;
};

} // namespace renderer
} // namespace engine

#endif

#endif
