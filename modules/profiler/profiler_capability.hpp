#ifndef ENGINE_MODULES_PROFILER_PROFILER_CAPABILITY_HPP
#define ENGINE_MODULES_PROFILER_PROFILER_CAPABILITY_HPP

// The published capability surface of the profiler module. Every consumer -
// other modules, native mods, the host - includes THIS header, from the
// module that provides the capability: the provider owns and publishes its
// own capability vocabulary, so there is no central contract directory and
// no mirrored copy that could drift. Like every published capability header
// this stays implementation-, platform- and vendor-agnostic; the per-module
// purity guard in the module's CMakeLists enforces that mechanically.
// See mods/README.md, section "Portability".

#include <chrono>
#include <string>
#include <utility>

namespace engine
{

// Capability "profiler": CPU-side instrumentation. Consumers feed two kinds
// of measurements - counters (monotonically growing amounts: bytes read,
// objects spawned, cache hits) and samples (named durations recorded once
// the measured work has completed). The provider owns aggregation and
// reporting: when and where the numbers land is the provider's policy, not
// the caller's, so a replacement implementation can write to a file, a
// network stream or an overlay without one consumer changing.
//
// The surface is deliberately free of any output concept: a consumer never
// asks for a report, it only contributes measurements. That keeps the calls
// cheap enough for per-frame use and keeps the capability portable - nothing
// here names a destination, a format or a thread.
class ProfilerCapability
{
public:
    static constexpr const char *capabilityName = "profiler";

    virtual ~ProfilerCapability() = default;

    // Adds one increment to a named counter. Counters accumulate: repeated
    // calls with the same name sum up. Amounts are doubles so both counts
    // and byte sizes flow through one method; a consumer decides the unit
    // and says it in the name ("assets.bytes_read").
    virtual void addCounterAmount(const std::string &name, double amount) = 0;

    // Records one completed duration under a named sample, in seconds.
    // Repeated calls with the same name accumulate count, total and maximum,
    // which is everything a mean needs. Any positive value is accepted; the
    // provider never interprets what the duration measured.
    virtual void addSampleSeconds(const std::string &name, double seconds) = 0;

    // Measures one scope and records it as a sample on destruction. The
    // clock starts at construction, so the guard covers exactly the lexical
    // scope it is declared in - the standard RAII instrumentation pattern.
    // Construction costs one string copy and one clock read; a hot loop
    // that samples every iteration is better served by timing the batch
    // manually and calling addSampleSeconds once.
    class ScopedSample
    {
    public:
        ScopedSample(ProfilerCapability &profiler, std::string sampleName)
            : profiler_(profiler)
            , sampleName_(std::move(sampleName))
            , started_(std::chrono::steady_clock::now())
        {
        }

        ScopedSample(const ScopedSample &)            = delete;
        ScopedSample &operator=(const ScopedSample &) = delete;

        ~ScopedSample()
        {
            const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - started_;
            profiler_.addSampleSeconds(sampleName_, elapsed.count());
        }

    private:
        ProfilerCapability                   &profiler_;
        std::string                           sampleName_;
        std::chrono::steady_clock::time_point started_;
    };
};

} // namespace engine

#endif // ENGINE_MODULES_PROFILER_PROFILER_CAPABILITY_HPP
