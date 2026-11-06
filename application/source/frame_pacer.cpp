// FramePacer implementation: target frame-rate throttling.
//
// pace() sleeps until the next frame deadline (sleep then a short yield
// spin) and reports wait/overshoot; a target of 0 disables pacing.

#include "engine/kernel/profiling.hpp"
#include <engine/application/frame_pacer.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <thread>

#if defined(__linux__)
#include <sys/prctl.h>
#endif

#if defined(_WIN32)
// Paced frames quantize to the OS scheduler tick; without a raised timer
// resolution Sleep(1) on Windows can overshoot by up to ~15.6 ms, which lands
// directly in the frame-time P99. winmm's timeBeginPeriod(1) fixes it for
// this process while pacing is active (per-process scope on Windows 10 2004+,
// which is exactly the scope wanted here).
#include <windows.h>
#include <timeapi.h>
#if defined(_MSC_VER)
#pragma comment(lib, "winmm.lib")
#endif
#endif

namespace engine
{

FramePacer::FramePacer(FramePacerOptions options)
{
    setOptions(options);
}

// Options sanitize: rate capped at 1 kHz, spin window at 5 ms; resets the deadline chain.
void FramePacer::setOptions(FramePacerOptions options)
{
    if (!std::isfinite(options.targetFrameRateHz) || options.targetFrameRateHz < 0.0)
        throw std::invalid_argument("target frame rate must be finite and non-negative");
    options.targetFrameRateHz      = std::min(options.targetFrameRateHz, 1000.0);
    options.spinWindowMicroseconds = std::min(options.spinWindowMicroseconds, 5000u);

#if defined(__linux__)
    // Linux default timer slack is 50 us: the kernel may delay every
    // timed sleep by up to that much, which lands straight in the
    // frame-time P99 of paced runs. Ask for the minimum slack (1 ns);
    // best effort - failures just keep the default.
    if (options.targetFrameRateHz > 0.0 && !timerSlackReduced_)
    {
        if (prctl(PR_SET_TIMERSLACK, 1ul, 0ul, 0ul, 0ul) == 0)
            timerSlackReduced_ = true;
    }
#endif

#if defined(_WIN32)
    const bool wantHighResolution = options.targetFrameRateHz > 0.0;
    if (wantHighResolution && !timerResolutionRaised_)
    {
        if (timeBeginPeriod(1u) == TIMERR_NOERROR)
            timerResolutionRaised_ = true;
    }
    else if (!wantHighResolution && timerResolutionRaised_)
    {
        timeEndPeriod(1u);
        timerResolutionRaised_ = false;
    }
#endif

    options_ = options;
    reset();
}

FramePacer::~FramePacer()
{
#if defined(_WIN32)
    if (timerResolutionRaised_)
    {
        timeEndPeriod(1u);
        timerResolutionRaised_ = false;
    }
#endif
}

// The deadline chain advances by whole periods; if the caller is late it jumps forward
// whole periods (no permanent lag). Sleep until just before the deadline, then
// spin/yield for the remainder.
FramePacingSample FramePacer::pace(PerformanceProfiler *profiler)
{
    FramePacingSample sample{};
    if (options_.targetFrameRateHz <= 0.0)
        return sample;

    const auto started = Clock::now();
    const auto period
        = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / options_.targetFrameRateHz));
    if (period <= Clock::duration::zero())
        return sample;

    const auto now = Clock::now();
    if (!initialized_)
    {
        nextDeadline_ = started;
        initialized_  = true;
    }
    nextDeadline_ += period;
    if (nextDeadline_ <= now)
    {
        const auto lateBy = now - nextDeadline_;
        const auto missed = lateBy / period + 1;
        nextDeadline_ += period * missed;
    }

    const auto spinWindow    = std::chrono::microseconds(options_.spinWindowMicroseconds);
    const auto sleepDeadline = nextDeadline_ - std::min<Clock::duration>(spinWindow, period / 2);
    if (Clock::now() < sleepDeadline)
        std::this_thread::sleep_until(sleepDeadline);
    // Spin the remaining window with PAUSE hints: tighter than yield
    // (no scheduler round trip) and keeps the core's pipelines quiet.
    while (Clock::now() < nextDeadline_ && options_.spinWindowMicroseconds != 0)
        engineSpinWaitHint();

    const auto finished = Clock::now();
    sample.paced        = true;
    sample.waitedNanoseconds
        = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(finished - started).count());
    if (finished > nextDeadline_)
    {
        sample.overshootNanoseconds = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(finished - nextDeadline_).count());
    }
    if (profiler != nullptr)
        profiler->record(PerformanceMetric::FramePacing, sample.waitedNanoseconds);
    return sample;
}

} // namespace engine
