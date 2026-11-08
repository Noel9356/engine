// Lock-free performance counters shared through ParticipantContext.
//
// Every kernel service and module records nanosecond durations into
// fixed per-metric slots (PerformanceMetric); the application reads
// them for the diagnostics report.
//
// Beyond count/total/max, each slot keeps a logarithmic histogram
// (HdrHistogram-style: ~1/8-octave bucket precision) so the application can
// read P50/P90/P95/P99/P99.9 estimates instead of deriving tail behaviour
// from a mean or a maximum. record() stays wait-free: one bucket fetch_add
// per sample, no locks, no allocation.

#ifndef ENGINE_KERNEL_PROFILING_HPP
#define ENGINE_KERNEL_PROFILING_HPP

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>

#if defined(_MSC_VER)
#include <intrin.h> // _mm_pause
#endif

// Spin-wait hint: PAUSE on x86/x86_64 (with a plain compiler barrier
// elsewhere). PAUSE keeps spin loops from wasting issue slots and memory-
// order buffers, and measurably lowers wake latency versus a bare yield
// loop in fork/join wait paths.
inline void engineSpinWaitHint() noexcept
{
#if defined(__x86_64__) || defined(__i386__) || defined(_M_X64) || defined(_M_IX86)
#if defined(__GNUC__) || defined(__clang__)
    __builtin_ia32_pause();
#elif defined(_MSC_VER)
    _mm_pause();
#else
    volatile int spinSink = 0;
    (void)spinSink;
#endif
#elif defined(__aarch64__) || defined(_M_ARM64) || defined(__arm__)
#if defined(__GNUC__) || defined(__clang__)
    __builtin_arm_yield();
#else
    volatile int spinSink = 0;
    (void)spinSink;
#endif
#else
    volatile int spinSink = 0;
    (void)spinSink;
#endif
}

namespace engine
{

// Slots hold nanosecond durations recorded via record(); Count is the
// array bound, not a metric. The FrameSlot*/Swapchain*/Upload* metrics name
// the engine's explicit blocking points (frame-slot retirement, swapchain
// acquire/present, upload-fence waits): their histograms attribute P99
// stalls to a wait instead of an execution cost. RenderDrawSort covers the
// draw-order cache (hit gather + rebuild), AssetEviction the asset-store
// cache eviction pass.
enum class PerformanceMetric : std::uint8_t
{
    SchedulerTask,
    SchedulerQueueWait,
    AssetRead,
    AssetEviction,
    FrameEvents,
    RenderFrame,
    RenderGraphCompile,
    RenderGraphExecute,
    VulkanUpload,
    FramePacing,
    FrameSlotWait,
    SwapchainAcquire,
    SwapchainPresent,
    QueueSubmit,
    UploadFenceWait,
    UploadsWait,
    ImageFenceWait,
    ReadbackWait,
    RenderDrawSort,
    Count
};

// Nanosecond totals; averageMilliseconds() derives the mean. The percentile
// fields are histogram estimates (bucket error <= ~6%, plus snapshot tearing
// under concurrent record() calls - acceptable for diagnostics; publish the
// sample count alongside so readers can judge stability).
struct PerformanceMetricSnapshot
{
    std::uint64_t sampleCount        = 0;
    std::uint64_t totalNanoseconds   = 0;
    std::uint64_t maximumNanoseconds = 0;
    std::uint64_t p50Nanoseconds     = 0;
    std::uint64_t p90Nanoseconds     = 0;
    std::uint64_t p95Nanoseconds     = 0;
    std::uint64_t p99Nanoseconds     = 0;
    std::uint64_t p999Nanoseconds    = 0;

    [[nodiscard]] double averageMilliseconds() const noexcept
    {
        return sampleCount == 0 ? 0.0
                                : static_cast<double>(totalNanoseconds) / static_cast<double>(sampleCount) / 1.0e6;
    }
};

// Logarithmic nanosecond histogram: wait-free record() (one bucket
// fetch_add), bounded snapshot cost, and predictable percentile error.
// Bucket layout: values 1..7 ns get one exact bucket each; from 8 ns up,
// every octave [8<<o, 16<<o) splits into 8 sub-buckets by the low mantissa
// bits of v>>o, so two values in one bucket differ by less than ~12.5% and
// percentile estimates carry a bounded, predictable error. Range: 1 ns up to
// 2^44 ns (~4.4 months); anything larger folds into the last bucket.
class PerformanceHistogram final
{
public:
    static constexpr std::size_t kSubBuckets   = 8;                    // sub-buckets per octave
    static constexpr std::size_t kExactBuckets = 7;                    // exact values 1..7 ns
    static constexpr std::size_t kMaxBitWidth  = 44;                   // values below 2^44 ns
    static constexpr std::size_t kMaxOctave    = kMaxBitWidth - 4;     // bit width 4 = first octave
    static constexpr std::size_t kBucketCount  = kExactBuckets + (kMaxOctave + 1) * kSubBuckets;

    void record(std::uint64_t nanoseconds) noexcept
    {
        buckets_[bucketFor(nanoseconds)].fetch_add(1, std::memory_order_relaxed);
    }

    // caller guarantees value >= 1 (record clamps 0 to 1).
    [[nodiscard]] static std::size_t bucketFor(std::uint64_t value) noexcept
    {
        if (value < kExactBuckets + 1)
            return static_cast<std::size_t>(value) - 1u; // exact buckets for 1..7 ns
        const std::uint32_t bitWidth = 64u - static_cast<std::uint32_t>(std::countl_zero(value));
        const std::size_t   octave   = static_cast<std::size_t>(bitWidth) - 4u;
        const std::uint64_t mantissa = (value >> octave) & (kSubBuckets - 1u);
        const std::size_t   bucket   = kExactBuckets + octave * kSubBuckets + static_cast<std::size_t>(mantissa);
        return bucket < kBucketCount ? bucket : kBucketCount - 1u; // >= 2^44 ns folds into the last bucket
    }

    // Lowest value a bucket represents; inverse of bucketFor.
    [[nodiscard]] static std::uint64_t bucketLowerBound(std::size_t bucket) noexcept
    {
        if (bucket < kExactBuckets)
            return static_cast<std::uint64_t>(bucket) + 1u; // bucket b holds exactly the value b+1
        const std::size_t octave   = (bucket - kExactBuckets) / kSubBuckets;
        const std::size_t mantissa = (bucket - kExactBuckets) % kSubBuckets;
        return (static_cast<std::uint64_t>(kSubBuckets) + mantissa) << octave;
    }

    // Walks the buckets (non-atomic; may tear slightly under concurrent
    // record() calls). Returns the estimated value at the requested quantile
    // (0.5 = P50, 0.99 = P99), interpolated inside the crossing bucket so the
    // estimate always stays inside that bucket's value range.
    [[nodiscard]] std::uint64_t percentile(double quantile) const noexcept
    {
        std::uint64_t total = 0;
        for (const auto &bucket : buckets_)
            total += bucket.load(std::memory_order_relaxed);
        if (total == 0)
            return 0;
        const double        clamped = std::clamp(quantile, 0.0, 1.0);
        const std::uint64_t rank    = std::max<std::uint64_t>(
            1u, static_cast<std::uint64_t>(std::ceil(clamped * static_cast<double>(total))));
        std::uint64_t seen = 0;
        for (std::size_t bucket = 0; bucket < kBucketCount; ++bucket)
        {
            const std::uint64_t count = buckets_[bucket].load(std::memory_order_relaxed);
            if (count == 0)
                continue;
            if (seen + count >= rank)
            {
                const std::uint64_t lower = bucketLowerBound(bucket);
                const std::uint64_t upper = bucket < kExactBuckets ? lower + 1u : bucketLowerBound(bucket + 1);
                if (upper <= lower) // defensive: keeps the interpolation in range
                    return lower;
                const double fraction = static_cast<double>(rank - seen) / static_cast<double>(count);
                const double span     = static_cast<double>(upper - lower - 1u);
                return lower + static_cast<std::uint64_t>(span * (fraction > 1.0 ? 1.0 : fraction));
            }
            seen += count;
        }
        return 0;
    }

    void reset() noexcept
    {
        for (auto &bucket : buckets_)
            bucket.store(0, std::memory_order_relaxed);
    }

private:
    std::array<std::atomic<std::uint64_t>, kBucketCount> buckets_{};
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
        counter.histogram.record(nanoseconds == 0 ? 1u : nanoseconds);
    }

    // Reads the counters and walks the histogram; values may be mutually
    // slightly stale.
    [[nodiscard]] PerformanceMetricSnapshot snapshot(PerformanceMetric metric) const noexcept
    {
        const std::size_t index = static_cast<std::size_t>(metric);
        if (index >= counters_.size())
            return {};
        const Counter &counter = counters_[index];
        return {counter.samples.load(std::memory_order_relaxed),
            counter.totalNanoseconds.load(std::memory_order_relaxed),
            counter.maximumNanoseconds.load(std::memory_order_relaxed),
            counter.histogram.percentile(0.50),
            counter.histogram.percentile(0.90),
            counter.histogram.percentile(0.95),
            counter.histogram.percentile(0.99),
            counter.histogram.percentile(0.999)};
    }

    void reset() noexcept
    {
        for (Counter &counter : counters_)
        {
            counter.samples.store(0, std::memory_order_relaxed);
            counter.totalNanoseconds.store(0, std::memory_order_relaxed);
            counter.maximumNanoseconds.store(0, std::memory_order_relaxed);
            counter.histogram.reset();
        }
    }

private:
    struct Counter
    {
        std::atomic<std::uint64_t> samples{0};
        std::atomic<std::uint64_t> totalNanoseconds{0};
        std::atomic<std::uint64_t> maximumNanoseconds{0};
        PerformanceHistogram       histogram{};
    };

    std::array<Counter, static_cast<std::size_t>(PerformanceMetric::Count)> counters_{};
    std::atomic<bool>                                                       enabled_{true};
};

// RAII: records elapsed nanoseconds on scope exit; records nothing when
// the profiler was disabled at construction (or the pointer is null, for
// instrumentation sites that may run without a profiler attached).
class ScopedProfileTimer final
{
public:
    ScopedProfileTimer(PerformanceProfiler &profiler, PerformanceMetric metric) noexcept
        : profiler_(&profiler),
          metric_(metric),
          active_(profiler.enabled()),
          start_(active_ ? PerformanceProfiler::Clock::now() : PerformanceProfiler::Clock::time_point{})
    {}

    ScopedProfileTimer(PerformanceProfiler *profiler, PerformanceMetric metric) noexcept
        : profiler_(profiler),
          metric_(metric),
          active_(profiler != nullptr && profiler->enabled()),
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
