// FrameTimeRecorder: fixed-capacity ring of frame periods with a
// percentile (P50/P90/P95/P99/max) summary for the exit log.
//
// Frame P99 is the metric players feel as stutter: the average hides bad
// frames, the 99th percentile names them. The recorder measures start-to-start
// frame periods (covering event pumping, simulation, render submission,
// present and pacing) so the summary reflects the true cadence.

#ifndef ENGINE_DEMO_FRAME_STATS_HPP
#define ENGINE_DEMO_FRAME_STATS_HPP

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace demo
{

class FrameTimeRecorder final
{
public:
    // ~2 minutes of 60 Hz frames; oldest samples are overwritten when the
    // cap is exceeded so the summary tracks recent behaviour.
    static constexpr std::size_t ringCapacity = 8192u;

    // Call once per main-loop iteration, at the top; the period between
    // consecutive calls is the frame time. The first call only anchors.
    void beginFrame()
    {
        const auto now = std::chrono::steady_clock::now();
        if (anchored_)
        {
            const double seconds = std::chrono::duration<double>(now - lastBegin_).count();
            if (seconds > 0.0 && seconds < 10.0) // ignore multi-second pauses (debuggers, suspends)
            {
                samples_[count_ % ringCapacity] = seconds;
                ++count_;
            }
        }
        lastBegin_ = now;
        anchored_  = true;
    }

    // Percentile of the recorded samples, linear-interpolated (ms).
    [[nodiscard]] double percentileMs(double fraction) const
    {
        const std::size_t available = std::min<std::size_t>(count_, ringCapacity);
        if (available == 0)
            return 0.0;
        std::vector<double> sorted;
        sorted.reserve(available);
        const std::size_t start = count_ > ringCapacity ? count_ % ringCapacity : 0;
        for (std::size_t index = 0; index < available; ++index)
            sorted.push_back(samples_[(start + index) % ringCapacity] * 1000.0);
        std::sort(sorted.begin(), sorted.end());
        const double position = fraction * static_cast<double>(available - 1);
        const std::size_t lower  = static_cast<std::size_t>(position);
        const std::size_t upper  = std::min(lower + 1u, available - 1u);
        const double      weight = position - static_cast<double>(lower);
        return sorted[lower] + (sorted[upper] - sorted[lower]) * weight;
    }

    [[nodiscard]] double maximumMs() const
    {
        const std::size_t available = std::min<std::size_t>(count_, ringCapacity);
        double            maximum   = 0.0;
        const std::size_t start     = count_ > ringCapacity ? count_ % ringCapacity : 0;
        for (std::size_t index = 0; index < available; ++index)
            maximum = std::max(maximum, samples_[(start + index) % ringCapacity] * 1000.0);
        return maximum;
    }

    [[nodiscard]] double meanMs() const
    {
        const std::size_t available = std::min<std::size_t>(count_, ringCapacity);
        if (available == 0)
            return 0.0;
        double total = 0.0;
        for (std::size_t index = 0; index < available; ++index)
            total += samples_[index] * 1000.0;
        return total / static_cast<double>(available);
    }

    [[nodiscard]] std::uint64_t sampleCount() const noexcept
    {
        return count_;
    }

    // One-line summary for the exit log; `warmupFrames` samples are dropped
    // from the front of the window first (pipeline/asset warm-up spikes are
    // startup cost, not steady-state stutter). ENGINE_FRAME_WARMUP overrides
    // the default for short benchmark runs.
    void printSummary(std::uint32_t warmupFrames = 60) const
    {
        if (const char *overrideValue = std::getenv("ENGINE_FRAME_WARMUP"); overrideValue != nullptr)
        {
            char         *end    = nullptr;
            const long    parsed = std::strtol(overrideValue, &end, 10);
            if (end != overrideValue && parsed >= 0 && parsed <= 100000)
                warmupFrames = static_cast<std::uint32_t>(parsed);
        }
        const std::uint64_t total     = count_;
        const std::size_t   available = std::min<std::size_t>(count_, ringCapacity);
        if (available == 0)
            return;
        const std::uint64_t warmup   = std::min<std::uint64_t>(warmupFrames, total);
        const std::size_t   measured = static_cast<std::size_t>(total - warmup);
        if (measured == 0)
        {
            std::fprintf(stderr,
                "demo: frame times: %llu sample(s), all warm-up\n",
                static_cast<unsigned long long>(total));
            return;
        }

        // Percentiles over the most recent `measured` samples.
        std::vector<double> sorted;
        sorted.reserve(measured);
        for (std::uint64_t index = warmup; index < total; ++index)
            sorted.push_back(samples_[index % ringCapacity] * 1000.0);
        std::sort(sorted.begin(), sorted.end());
        const auto percentile = [&sorted](double fraction)
        {
            const double      position = fraction * static_cast<double>(sorted.size() - 1);
            const std::size_t lower    = static_cast<std::size_t>(position);
            const std::size_t upper    = std::min(lower + 1u, sorted.size() - 1u);
            const double      weight   = position - static_cast<double>(lower);
            return sorted[lower] + (sorted[upper] - sorted[lower]) * weight;
        };
        const double p50 = percentile(0.50);
        const double p95 = percentile(0.95);
        const double p99 = percentile(0.99);
        const double maximum = sorted.back();
        const double median2 = p50 * 1.5;
        const std::size_t hitches = static_cast<std::size_t>(
            std::count_if(sorted.begin(), sorted.end(), [median2](double value) { return value > median2; }));
        std::fprintf(stderr,
            "demo: frame times ms (n=%llu): p50=%.2f p95=%.2f p99=%.2f max=%.2f mean=%.2f, hitches(>1.5x p50)=%zu\n",
            static_cast<unsigned long long>(measured),
            p50,
            p95,
            p99,
            maximum,
            meanMs(),
            hitches);
    }

private:
    std::array<double, ringCapacity> samples_{};
    std::uint64_t                    count_    = 0;
    std::chrono::steady_clock::time_point lastBegin_{};
    bool                             anchored_ = false;
};

} // namespace demo

#endif
