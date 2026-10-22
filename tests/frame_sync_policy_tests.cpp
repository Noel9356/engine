// Frame synchronization ownership regression tests.
//
// These tests exist because of a field-reported bug that the headless smoke
// run could NOT catch: the renderer used to own its render-finished (present)
// semaphores per frame slot instead of per swapchain image. On NVIDIA/Windows
// (presentation depth 1-2 frames) vkQueueSubmit re-signaled semaphores that
// the presentation engine could still be holding
// (VUID-vkQueueSubmit-pSignalSemaphores-00067), while on lavapipe the smoke
// run passed cleanly because software WSI retires present operations
// synchronously - the "presented but was not re-acquired" state simply never
// occurred. Driver timing therefore decides whether this bug class is visible,
// so the guard here is deterministic: it replays recorded acquisition
// sequences against the ownership policy from frame_sync_policy.hpp and flags
// every re-signal of a semaphore whose presented image has not been
// re-acquired - the exact condition the validation layer checks.

#include "frame_sync_policy.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include <catch2/catch_test_macros.hpp>

namespace
{

// A binary semaphore would be re-signaled while its previous presentation may
// still be pending: the validation layer reports exactly this as
// VUID-vkQueueSubmit-pSignalSemaphores-00067.
struct ReuseViolation
{
    std::size_t frame = 0;
    std::uint32_t semaphore = 0;
    std::uint32_t presentedImage = 0;
    std::uint32_t acquiredImage = 0;
};

// Replays an acquisition sequence against a render-finished semaphore mapping.
// The mapping receives (frame index, acquired image index) and returns the
// semaphore that will be signaled for that frame's submit and waited on by
// that frame's present. A semaphore stays "pending present" from the frame
// that signals it until its presented image appears in the sequence again:
// a swapchain image can only be handed out by vkAcquireNextImageKHR after the
// presentation that retired it consumed the semaphore wait, so re-acquisition
// is the deterministic signal that reusing the semaphore became legal.
std::vector<ReuseViolation> findSemaphoreReuseViolations(
    const std::vector<std::uint32_t>& acquisitionSequence, std::uint32_t semaphoreCount,
    const std::function<std::uint32_t(std::size_t, std::uint32_t)>& renderFinishedIndexFor)
{
    std::vector<ReuseViolation> violations;
    std::vector<std::optional<std::uint32_t>> pendingPresentedImage(semaphoreCount);

    for (std::size_t frame = 0; frame < acquisitionSequence.size(); ++frame)
    {
        const std::uint32_t acquiredImage = acquisitionSequence[frame];

        // Re-acquiring an image means its previous presentation consumed the
        // semaphore wait, retiring that pending-present state.
        for (std::uint32_t semaphore = 0; semaphore < semaphoreCount; ++semaphore)
        {
            if (pendingPresentedImage[semaphore] == acquiredImage)
            {
                pendingPresentedImage[semaphore].reset();
            }
        }

        const std::uint32_t semaphore = renderFinishedIndexFor(frame, acquiredImage);
        if (pendingPresentedImage[semaphore].has_value())
        {
            violations.push_back(ReuseViolation{frame, semaphore, *pendingPresentedImage[semaphore], acquiredImage});
        }
        pendingPresentedImage[semaphore] = acquiredImage;
    }
    return violations;
}

// The historical (buggy) ownership: render-finished semaphores owned by the
// frame slot. Kept here as the counter-example the policy must never regress
// to; findSemaphoreReuseViolations is proven meaningful by the test that
// expects violations from this mapping.
std::uint32_t perFrameRenderFinishedIndex(std::size_t frame, std::uint32_t /*acquiredImage*/,
                                          std::uint32_t framesInFlight)
{
    return static_cast<std::uint32_t>(frame % framesInFlight);
}

std::function<std::uint32_t(std::size_t, std::uint32_t)> perFrameMapping(std::uint32_t framesInFlight)
{
    return [framesInFlight](std::size_t frame, std::uint32_t image)
    {
        return perFrameRenderFinishedIndex(frame, image, framesInFlight);
    };
}

std::function<std::uint32_t(std::size_t, std::uint32_t)> perImageMapping()
{
    return [](std::size_t /*frame*/, std::uint32_t image)
    {
        return engine::FrameSyncPolicy::renderFinishedSemaphoreIndex(image);
    };
}

// Acquisition pattern recorded from the field report on NVIDIA/Windows
// (RTX 4060 Laptop, swapchain with 3 images, 2 frames in flight): the third
// submit re-signaled the semaphore whose last presentation was image 0 while
// images 1 and 2 had been acquired in between.
const std::vector<std::uint32_t> fieldSequenceA{0, 1, 2, 0, 1, 2, 0, 1, 2};
const std::vector<std::uint32_t> fieldSequenceB{0, 1, 2, 1, 2, 0, 2, 0, 1};
// Four-image swapchain (what lavapipe reports) under the same round-robin.
const std::vector<std::uint32_t> fourImageSequence{0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3};

} // namespace

TEST_CASE("render-finished semaphores are owned by the swapchain image", "[renderer][sync]")
{
    // Each recorded sequence is replayed against a pool sized to the
    // swapchain that produced it, mirroring what recreateSwapchain does when
    // the image count changes.
    const std::vector<std::uint32_t> recordedSequences[3] = {fieldSequenceA, fieldSequenceB, fourImageSequence};
    const std::uint32_t sequenceImageCounts[3] = {3, 3, 4};

    engine::FrameSyncPolicy policy{.framesInFlight = 2, .swapchainImageCount = 3};
    for (std::size_t sequenceIndex = 0; sequenceIndex < 3; ++sequenceIndex)
    {
        policy.setSwapchainImageCount(sequenceImageCounts[sequenceIndex]);
        const std::uint32_t semaphoreCount = policy.renderFinishedSemaphoreCount();
        REQUIRE(semaphoreCount == sequenceImageCounts[sequenceIndex]);
        const std::vector<ReuseViolation> violations =
            findSemaphoreReuseViolations(recordedSequences[sequenceIndex], semaphoreCount, perImageMapping());
        INFO("recorded acquisition sequence index " << sequenceIndex);
        REQUIRE(violations.empty());
    }
}

TEST_CASE("acquire semaphores stay owned by the frame slot", "[renderer][sync]")
{
    const engine::FrameSyncPolicy policy{.framesInFlight = 2, .swapchainImageCount = 3};

    REQUIRE(policy.acquireSemaphoreCount() == 2);
    REQUIRE(policy.acquireSemaphoreCount() != 0);
    // Distinct frame slots map to distinct acquire semaphores, so two frames
    // in flight never share one pending acquire.
    REQUIRE(engine::FrameSyncPolicy::acquireSemaphoreIndex(0) !=
            engine::FrameSyncPolicy::acquireSemaphoreIndex(1));
}

TEST_CASE("the historical per-frame ownership violates the recorded sequences", "[renderer][sync]")
{
    // Guards the detector: the same sequences MUST produce violations under
    // the old frame-slot ownership, otherwise these tests could silently stop
    // discriminating the two ownership models.
    const std::vector<ReuseViolation> violationsA =
        findSemaphoreReuseViolations(fieldSequenceA, 2, perFrameMapping(2));
    REQUIRE_FALSE(violationsA.empty());
    // The first violation is the third submit of the field report: frame slot
    // 0 re-signaled while its last presented image 0 was not re-acquired.
    REQUIRE(violationsA.front().frame == 2);
    REQUIRE(violationsA.front().presentedImage == 0);
    REQUIRE(violationsA.front().acquiredImage == 2);

    const std::vector<ReuseViolation> violationsB =
        findSemaphoreReuseViolations(fieldSequenceB, 2, perFrameMapping(2));
    REQUIRE_FALSE(violationsB.empty());

    const std::vector<ReuseViolation> violationsFour =
        findSemaphoreReuseViolations(fourImageSequence, 2, perFrameMapping(2));
    REQUIRE_FALSE(violationsFour.empty());
}

TEST_CASE("swapchain recreation resizes the image-scoped semaphore pool", "[renderer][sync]")
{
    engine::FrameSyncPolicy policy{.framesInFlight = 2, .swapchainImageCount = 3};
    REQUIRE(policy.renderFinishedSemaphoreCount() == 3);

    // A recreated swapchain may hand out a different image count; the pool
    // follows it while frame-slot scoping is untouched.
    policy.setSwapchainImageCount(4);
    REQUIRE(policy.renderFinishedSemaphoreCount() == 4);
    REQUIRE(policy.acquireSemaphoreCount() == 2);

    policy.setSwapchainImageCount(2);
    REQUIRE(policy.renderFinishedSemaphoreCount() == 2);
    REQUIRE(policy.acquireSemaphoreCount() == 2);

    // The index mapping is identity over the image index, so every pool entry
    // is reachable and none is shared between two images.
    for (std::uint32_t image = 0; image < policy.renderFinishedSemaphoreCount(); ++image)
    {
        REQUIRE(engine::FrameSyncPolicy::renderFinishedSemaphoreIndex(image) == image);
    }
}
