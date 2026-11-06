// FrameClock implementation: frame timing and fixed-update accumulation.
//
// advance() measures wall-clock deltas, clamps them (spiral-of-death
// protection) and converts the clamped delta into fixed steps via an
// accumulator; the leftover fraction feeds the interpolation alpha.

#include "engine/kernel/engine_kernel.hpp"
#include "engine/kernel/profiling.hpp"
#include <engine/application/frame_clock.hpp>
#include <engine/kernel/participant_interface.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

namespace engine
{

// Sanitizes options (non-finite or silly values fall back to defaults) and derives
// the fixed step 1/hz (0 disables fixed updates).
FrameClock::FrameClock(FrameClockOptions options)
    : options_(options)
{
    if (!std::isfinite(options_.maxFrameDeltaSeconds) || options_.maxFrameDeltaSeconds <= 0.0)
    {
        options_.maxFrameDeltaSeconds = 0.25;
    }
    options_.maxFrameDeltaSeconds = std::clamp(options_.maxFrameDeltaSeconds, 0.001, 10.0);
    if (!std::isfinite(options_.fixedUpdateHz) || options_.fixedUpdateHz <= 0.0)
    {
        options_.fixedUpdateHz = 0.0;
    }
    else
    {
        options_.fixedUpdateHz = std::min(options_.fixedUpdateHz, 1000.0);
    }
    options_.maxFixedUpdatesPerFrame = std::clamp<std::uint32_t>(options_.maxFixedUpdatesPerFrame, 1, 256);
    fixedDeltaSeconds_               = options_.fixedUpdateHz > 0.0 ? 1.0 / options_.fixedUpdateHz : 0.0;
    reset();
}

void FrameClock::reset(TimePoint now)
{
    lastFrameTime_    = now;
    initialized_      = true;
    elapsedSeconds_   = 0.0;
    fixedAccumulator_ = 0.0;
    nextFrameIndex_   = 0;
}

// Accumulator math: run floor(acc/step) fixed steps (capped), subtract the executed
// steps and drop any remaining backlog so the clock catches up instead of spiraling;
// the remainder drives interpolationAlpha.
FrameTiming FrameClock::advance(TimePoint now)
{
    if (!initialized_)
    {
        reset(now);
    }

    double actualDelta = std::chrono::duration<double>(now - lastFrameTime_).count();
    lastFrameTime_     = now;
    // Non-monotonic or infinite clock reads count as no time.
    if (!std::isfinite(actualDelta) || actualDelta < 0.0)
    {
        actualDelta = 0.0;
    }
    elapsedSeconds_ += actualDelta;
    // Delta beyond the clamp is real time but is not simulated (droppedSimulationSeconds).
    const double frameDelta = std::min(actualDelta, options_.maxFrameDeltaSeconds);

    FrameTiming timing;
    timing.frameIndex        = nextFrameIndex_++;
    timing.deltaSeconds      = frameDelta;
    timing.elapsedSeconds    = elapsedSeconds_;
    timing.fixedDeltaSeconds = fixedDeltaSeconds_;

    if (fixedDeltaSeconds_ > 0.0)
    {
        timing.droppedSimulationSeconds = actualDelta - frameDelta;
        fixedAccumulator_ += frameDelta;
        const double availableSteps = std::floor(fixedAccumulator_ / fixedDeltaSeconds_);
        const double permittedSteps = static_cast<double>(options_.maxFixedUpdatesPerFrame);
        timing.fixedUpdateCount     = static_cast<std::uint32_t>(std::min(availableSteps, permittedSteps));
        fixedAccumulator_ -= static_cast<double>(timing.fixedUpdateCount) * fixedDeltaSeconds_;

        // Backlog beyond the per-frame cap is dropped, keeping only the sub-step remainder.
        if (fixedAccumulator_ >= fixedDeltaSeconds_)
        {
            const double retained = std::fmod(fixedAccumulator_, fixedDeltaSeconds_);
            timing.droppedSimulationSeconds += fixedAccumulator_ - retained;
            fixedAccumulator_ = retained;
        }
        timing.interpolationAlpha = std::clamp(fixedAccumulator_ / fixedDeltaSeconds_, 0.0, 1.0);
    }
    return timing;
}

// One EngineFixedUpdateEvent per accumulated step, each carrying the fixed delta.
void FrameClock::publishFixedUpdates(EngineKernel &kernel, const FrameTiming &timing) const
{
    if (timing.fixedUpdateCount == 0 || fixedDeltaSeconds_ <= 0.0)
    {
        return;
    }

    auto &fixedChannel = kernel.eventBus().channel<EngineFixedUpdateEvent>();
    for (std::uint32_t step = 0; step < timing.fixedUpdateCount; ++step)
    {
        fixedChannel.publish(EngineFixedUpdateEvent{.deltaSeconds = timing.fixedDeltaSeconds});
    }
}

void FrameClock::publishFrameEvent(EngineKernel &kernel, const FrameTiming &timing) const
{
    kernel.eventBus().channel<EngineFrameEvent>().publish(EngineFrameEvent{
        .frameIndex = timing.frameIndex, .deltaSeconds = timing.deltaSeconds, .elapsedSeconds = timing.elapsedSeconds});
}

void FrameClock::publishEvents(EngineKernel &kernel, const FrameTiming &timing) const
{
    ScopedProfileTimer timer(kernel.profiler(), PerformanceMetric::FrameEvents);
    publishFixedUpdates(kernel, timing);
    publishFrameEvent(kernel, timing);
}

} // namespace engine
