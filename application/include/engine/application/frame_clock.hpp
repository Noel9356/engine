// FrameClock: fixed-timestep bookkeeping for the main loop.
//
// advance() turns wall-clock time into a FrameTiming: a clamped variable
// delta, the number of fixed steps to run and the interpolation alpha.

#ifndef ENGINE_APPLICATION_FRAME_CLOCK_HPP
#define ENGINE_APPLICATION_FRAME_CLOCK_HPP

#include <engine/kernel/engine_kernel.hpp>

#include <chrono>
#include <cstdint>

namespace engine
{

struct FrameClockOptions
{
    // Clamp on a single frame's simulated delta (seconds); excess time is dropped, not simulated.
    double maxFrameDeltaSeconds = 0.25;

    // Fixed-step rate (Hz); 0 disables fixed updates, values in (0, 1) are
    // clamped up to 1 Hz at construction.
    double fixedUpdateHz = 0.0;

    // Cap on fixed steps per frame; backlog beyond it is dropped.
    std::uint32_t maxFixedUpdatesPerFrame = 4;
};

// One frame's timing: index, clamped delta, elapsed total, fixed-step plan and leftover.
struct FrameTiming
{
    std::uint64_t frameIndex        = 0;
    double        deltaSeconds      = 0.0;
    double        elapsedSeconds    = 0.0;
    double        fixedDeltaSeconds = 0.0;
    std::uint32_t fixedUpdateCount  = 0;
    // Fraction [0,1) of a fixed step left in the accumulator (render interpolation factor).
    double interpolationAlpha = 0.0;
    // Seconds skipped this frame (delta clamp overshoot plus capped-step backlog).
    double droppedSimulationSeconds = 0.0;
};

class FrameClock final
{
public:
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    explicit FrameClock(FrameClockOptions options = {});

    void                      reset(TimePoint now = Clock::now());
    [[nodiscard]] FrameTiming advance(TimePoint now = Clock::now());

    void publishFixedUpdates(EngineKernel &kernel, const FrameTiming &timing) const;
    void publishFrameEvent(EngineKernel &kernel, const FrameTiming &timing) const;

    void publishEvents(EngineKernel &kernel, const FrameTiming &timing) const;

    [[nodiscard]] const FrameClockOptions &options() const noexcept
    {
        return options_;
    }

private:
    FrameClockOptions options_{};
    TimePoint         lastFrameTime_{};
    bool              initialized_       = false;
    double            elapsedSeconds_    = 0.0;
    double            fixedAccumulator_  = 0.0;
    double            fixedDeltaSeconds_ = 0.0;
    std::uint64_t     nextFrameIndex_    = 0;
};

} // namespace engine

#endif
