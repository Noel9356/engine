// Lock-free performance counters shared through ParticipantContext.
//
// Every kernel service and module records nanosecond durations into
// fixed per-metric slots (PerformanceMetric); the application reads
// them for the diagnostics report.

#ifndef ENGINE_KERNEL_PROFILING_HPP
#define ENGINE_KERNEL_PROFILING_HPP

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace engine
{

// Slots hold nanosecond durations recorded via record(); Count is the
// array bound, not a metric.
enum class PerformanceMetric : std::uint8_t
{
    SchedulerTask,
    SchedulerQueueWait,
    AssetRead,
    FrameEvents,
    RenderFrame,
    RenderGraphCompile,
    RenderGraphExecute,
    VulkanUpload,
    FramePacing,
    Count
};

// Nanosecond totals; averageMilliseconds() derives the mean.
struct PerformanceMetricSnapshot
{
    std::uint64_t sampleCount        = 0;
    std::uint64_t totalNanoseconds   = 0;
    std::uint64_t maximumNanoseconds = 0;

    [[nodiscard]] double averageMilliseconds() const noexcept
    {
        return sampleCount == 0 ? 0.0
                                : static_cast<double>(totalNanoseconds) / static_cast<double>(sampleCount) / 1.0e6;
    }
};

// Atomics per metric: record() is wait-free, snapshot() may tear slightly
// (acceptable for diagnostics). Enabled by default.
class PerformanceProfiler final
{
public:
    using Clock = std::chrono::steady_clock;

    void setEnabled(bool enabled) noexcept
    {
        enabled_.store(enabled, std::memory_order_release);
    }

    [[nodiscard]] bool enabled() const noexcept
    {
        return enabled_.load(std::memory_order_acquire);
    }

    // Max update via a CAS loop (relaxed ordering: diagnostics only).
    void record(PerformanceMetric metric, std::uint64_t nanoseconds) noexcept
    {
        if (!enabled())
            return;
        const std::size_t index = static_cast<std::size_t>(metric);
        if (index >= counters_.size())
            return;
        Counter &counter = counters_[index];
        counter.samples.fetch_add(1, std::memory_order_relaxed);
        counter.totalNanoseconds.fetch_add(nanoseconds, std::memory_order_relaxed);
        std::uint64_t maximum = counter.maximumNanoseconds.load(std::memory_order_relaxed);
        while (maximum < nanoseconds
               && !counter.maximumNanoseconds.compare_exchange_weak(
                   maximum, nanoseconds, std::memory_order_relaxed, std::memory_order_relaxed))
        {
        }
    }

    // Reads the three atomics; values may be mutually slightly stale.
    [[nodiscard]] PerformanceMetricSnapshot snapshot(PerformanceMetric metric) const noexcept
    {
        const std::size_t index = static_cast<std::size_t>(metric);
        if (index >= counters_.size())
            return {};
        const Counter &counter = counters_[index];
        return {
            counter.samples.load(std::memory_order_relaxed),
            counter.totalNanoseconds.load(std::memory_order_relaxed),
            counter.maximumNanoseconds.load(std::memory_order_relaxed)};
    }

    void reset() noexcept
    {
        for (Counter &counter : counters_)
        {
            counter.samples.store(0, std::memory_order_relaxed);
            counter.totalNanoseconds.store(0, std::memory_order_relaxed);
            counter.maximumNanoseconds.store(0, std::memory_order_relaxed);
        }
    }

private:
    struct Counter
    {
        std::atomic<std::uint64_t> samples{0};
        std::atomic<std::uint64_t> totalNanoseconds{0};
        std::atomic<std::uint64_t> maximumNanoseconds{0};
    };

    std::array<Counter, static_cast<std::size_t>(PerformanceMetric::Count)> counters_{};
    std::atomic<bool>                                                       enabled_{true};
};

// RAII: records elapsed nanoseconds on scope exit; records nothing when
// the profiler was disabled at construction.
class ScopedProfileTimer final
{
public:
    ScopedProfileTimer(PerformanceProfiler &profiler, PerformanceMetric metric) noexcept
        : profiler_(&profiler),
          metric_(metric),
          active_(profiler.enabled()),
          start_(active_ ? PerformanceProfiler::Clock::now() : PerformanceProfiler::Clock::time_point{})
    {}

    ScopedProfileTimer(const ScopedProfileTimer &)            = delete;
    ScopedProfileTimer &operator=(const ScopedProfileTimer &) = delete;

    ~ScopedProfileTimer()
    {
        if (!active_)
            return;
        const auto elapsed
            = std::chrono::duration_cast<std::chrono::nanoseconds>(PerformanceProfiler::Clock::now() - start_).count();
        profiler_->record(metric_, static_cast<std::uint64_t>(std::max<std::int64_t>(elapsed, 0)));
    }

private:
    PerformanceProfiler                   *profiler_;
    PerformanceMetric                      metric_;
    bool                                   active_ = false;
    PerformanceProfiler::Clock::time_point start_;
};

} // namespace engine

#endif
