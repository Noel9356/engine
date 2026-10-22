#pragma once

// Ownership policy for the binary semaphores of the render loop.
//
// Two different scopes own the two semaphore families, and mixing them up is
// the classic swapchain-synchronization bug (the validation layer reports it
// as VUID-vkQueueSubmit-pSignalSemaphores-00067: "semaphore is being signaled
// ... but it may still be in use by VkSwapchainKHR ... presented but was not
// re-acquired").
//
//   acquire semaphores        one per FRAME SLOT (frame in flight)
//   render-finished (present) one per SWAPCHAIN IMAGE
//
// Why the scopes differ:
//
//   * A frame slot has at most one outstanding acquire at any time and its own
//     submit is the only waiter, so the acquire signal is always consumed by
//     the same frame slot before the slot is reused. Frame-scoped is correct.
//
//   * A render-finished semaphore is signaled by the submit and consumed by
//     the presentation engine, which may hold it long after vkQueueSubmit and
//     even vkQueuePresentKHR returned. The only deterministic guarantee the
//     spec gives is that a swapchain image cannot be re-acquired before the
//     presentation operation that retired it has consumed the wait. Pairing
//     the semaphore with the image therefore makes re-signaling legal exactly
//     when the image is legally reacquired. Frame-scoped ownership raced here
//     on NVIDIA/Windows (presentation depth 1-2 frames) while software raster
//     drivers (lavapipe) retire presents synchronously and mask the bug -
//     which is why frame_sync_policy_tests.cpp replays the recorded
//     acquisition pattern deterministically instead of relying on a driver.
//
// The index mappings are deliberately trivial; the type exists so the counts
// and the two index expressions have ONE definition that the renderer and the
// regression test share, with the ownership contract written next to it.

#include <cstdint>

namespace engine
{

// The frames-in-flight defaults: the sync policy's construction default, the
// renderer state's configuration default and the [renderer] config fallback
// all state the same number through these constants (the configuration clamp
// uses the limit).
inline constexpr std::uint32_t defaultFramesInFlight = 2;
inline constexpr std::uint32_t maxFramesInFlightLimit = 8;

struct FrameSyncPolicy
{
    std::uint32_t framesInFlight = defaultFramesInFlight;
    std::uint32_t swapchainImageCount = 0;

    void setSwapchainImageCount(std::uint32_t newSwapchainImageCount)
    {
        swapchainImageCount = newSwapchainImageCount;
    }

    // Acquire semaphores are waited and reused by frame slot.
    [[nodiscard]] std::uint32_t acquireSemaphoreCount() const
    {
        return framesInFlight;
    }

    // Render-finished semaphores are indexed by the acquired image index.
    [[nodiscard]] std::uint32_t renderFinishedSemaphoreCount() const
    {
        return swapchainImageCount;
    }

    [[nodiscard]] static constexpr std::uint32_t acquireSemaphoreIndex(std::uint32_t frameSlot)
    {
        return frameSlot;
    }

    [[nodiscard]] static constexpr std::uint32_t renderFinishedSemaphoreIndex(std::uint32_t imageIndex)
    {
        return imageIndex;
    }
};

} // namespace engine
