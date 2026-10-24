#ifndef ENGINE_MODULES_PROFILER_HPP
#define ENGINE_MODULES_PROFILER_HPP

#include "profiler_capability.hpp"
#include "logging_capability.hpp"
#include "engine/kernel/component_interface.hpp"

#include <cstdint>
#include <map>
#include <string>

namespace engine
{

// Module "profiler": provides the ProfilerCapability. Accumulates counters
// and named duration samples over the run and emits one aggregated report
// through the logging capability at shutdown - after every consumer has
// stopped, because the kernel stops components in reverse dependency order
// and the profiler, as a provider, stops last among its consumers.
class ProfilerModule final : public ModuleInterface, public ProfilerCapability
{
public:
    ProfilerModule();

    const ComponentManifest &manifest() const override
    {
        return componentManifest;
    }

    void construct(ComponentContext &context) override;
    void start() override;
    void stop() override;

    void *capabilityPointerFor(const std::string &capabilityName) override;

    void addCounterAmount(const std::string &name, double amount) override;
    void addSampleSeconds(const std::string &name, double seconds) override;

private:
    struct SampleStatistics
    {
        std::uint64_t count       = 0;
        double        totalSeconds = 0.0;
        double        maxSeconds   = 0.0;
    };

    void emitReport();

    ComponentManifest componentManifest;
    LoggerCapability *logger = nullptr;
    LogLevel          reportLevel = LogLevel::Information;
    // std::map, not unordered: the shutdown report is deterministic across
    // runs, which makes before/after comparisons readable.
    std::map<std::string, double>          counters;
    std::map<std::string, SampleStatistics> samples;
};

} // namespace engine

#endif // ENGINE_MODULES_PROFILER_HPP
