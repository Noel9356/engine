// The profiler_bench native mod: the worked example of consuming a
// brand-new capability without touching the kernel, the host or any other
// module. The package declares its requirements in mod.toml ("logging",
// "profiler"); the kernel activates the provider modules, derives the
// "capability.<name>" grants from the same list and orders this mod after
// its providers. This file includes only the kernel headers and the two
// published capability headers of the providers - the providers themselves
// stay anonymous behind their capability names.
#include "logging_capability.hpp"
#include "profiler_capability.hpp"
#include "engine/kernel/mod_api.hpp"

#include <cstdint>

namespace
{

// A small deterministic workload with a result the compiler cannot fold
// away: enough work to produce a measurable sample, short enough to stay
// invisible in a run's startup time.
double benchWorkload(
    std::uint64_t iterations
)
{
    double accumulator = 0.0;
    for (std::uint64_t index = 1; index <= iterations; ++index)
    {
        accumulator += static_cast<double>((index * 2654435761ull) % 1000ull) / 1000.0;
    }
    return accumulator;
}

// The mod component. Its compiled manifest only needs the name (the package
// manifest mod.toml is authoritative for identity and wiring: required
// capabilities, permissions, ordering); the kernel verifies the agreement
// and refuses a drift with a precise diagnostic.
class ProfilerBenchMod final : public engine::ModInterface
{
public:
    const engine::ComponentManifest &manifest() const override
    {
        static const engine::ComponentManifest compiledManifest = []()
        {
            engine::ComponentManifest manifest;
            manifest.name                 = "profiler_bench";
            manifest.kind                 = engine::ComponentKind::SharedLibraryMod;
            manifest.providedCapabilities = {};
            manifest.requiredCapabilities = {};
            manifest.requiredPermissions  = {};
            return manifest;
        }();
        return compiledManifest;
    }

    void construct(engine::ComponentContext &context) override
    {
        logger   = &context.acquireCapability<engine::LoggerCapability>();
        profiler = &context.acquireCapability<engine::ProfilerCapability>();

        logger->logMessage(engine::LogLevel::Information,
                           "profiler_bench constructed: acquired logging and profiler");
    }

    void start() override
    {
        // Counter: this mod's own run counter, plus one measured workload.
        profiler->addCounterAmount("bench.runs", 1.0);

        {
            engine::ProfilerCapability::ScopedSample sample(*profiler, "bench.start_scope");
            benchResult = benchWorkload(benchIterations);
        }

        logger->logMessage(engine::LogLevel::Information, "profiler_bench started: workload recorded");
    }

    void stop() override
    {
        // Work recorded here still lands in the report: the profiler, as a
        // provider, stops after its consumers.
        logger->logMessage(engine::LogLevel::Information, "profiler_bench stopped");
    }

    // The mod publishes no capability of its own: the hook exists so the
    // kernel can verify every provided capability has an implementation
    // behind it, and the contract stays uniform for capability-providing mods.
    void *capabilityPointerFor(const std::string &capabilityName) override
    {
        (void)capabilityName;
        return nullptr;
    }

private:
    static constexpr std::uint64_t benchIterations = 200000;

    engine::LoggerCapability    *logger   = nullptr;
    engine::ProfilerCapability  *profiler = nullptr;
    double                       benchResult = 0.0;
};

} // namespace

// The one exported symbol of a native mod (engine/kernel/mod_api.hpp): the
// C ABI entry table the kernel resolves by name after loading the library.
extern "C" __attribute__((visibility("default"))) const EngineModEntryTable *engine_query_mod_entry_table()
{
    static const EngineModEntryTable entryTable{
        .interfaceVersion = engine::engineModInterfaceVersion,
        .createMod        = []() -> engine::ModInterface * { return new ProfilerBenchMod(); },
        .destroyMod       = [](engine::ModInterface *mod) { delete mod; }
    };
    return &entryTable;
}
