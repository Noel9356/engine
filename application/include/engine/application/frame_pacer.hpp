// FramePacer: voluntary frame-rate throttling for the main loop.
//
// pace() blocks until the next deadline (sleep then a short spin) and
// reports how long it waited and by how much it overshot.

#ifndef ENGINE_APPLICATION_FRAME_PACER_HPP
#define ENGINE_APPLICATION_FRAME_PACER_HPP

#include <engine/kernel/profiling.hpp>

#include <chrono>
#include <cstdint>

namespace engine
{

struct FramePacerOptions
{
    double targetFrameRateHz = 0.0;

    // Busy-yield duration (microseconds) ahead of the deadline after the sleep ends; 0 disables spinning.
    std::uint32_t spinWindowMicroseconds = 0;
};

struct FramePacingSample
{
    std::uint64_t waitedNanoseconds    = 0;
    std::uint64_t overshootNanoseconds = 0;
    bool          paced                = false;
};

class FramePacer final
{
public:
    using Clock = std::chrono::steady_clock;

    explicit FramePacer(FramePacerOptions options = {});
    ~FramePacer();

    void                                   setOptions(FramePacerOptions options);
    [[nodiscard]] const FramePacerOptions &options() const noexcept
    {
        return options_;
    }
    // Re-anchors the deadline chain to now (call after long pauses).
    void reset() noexcept
    {
        nextDeadline_ = Clock::now();
        initialized_  = true;
    }

    [[nodiscard]] FramePacingSample pace(PerformanceProfiler *profiler = nullptr);

private:
    FramePacerOptions options_{};
    Clock::time_point nextDeadline_{};
    bool              initialized_           = false;
    bool              timerResolutionRaised_ = false; // Windows only; see frame_pacer.cpp
};

} // namespace engine

#endif
