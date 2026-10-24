#include "profiler.hpp"

#include "logging_capability.hpp"

#include <cstdio>
#include <utility>

namespace engine
{

namespace
{

LogLevel parseReportLevel(
    const std::string &levelName
)
{
    if (levelName == "trace")
    {
        return LogLevel::Trace;
    }
    if (levelName == "debug")
    {
        return LogLevel::Debug;
    }
    if (levelName == "information")
    {
        return LogLevel::Information;
    }
    if (levelName == "warning")
    {
        return LogLevel::Warning;
    }
    return LogLevel::Information;
}

} // namespace

ProfilerModule::ProfilerModule()
{
    componentManifest.name                 = "profiler";
    componentManifest.kind                 = ComponentKind::Module;
    componentManifest.providedCapabilities = {"profiler"};
    componentManifest.requiredCapabilities = {"logging"};
    componentManifest.requiredPermissions  = {};
    componentManifest.knownConfigKeys      = {"profiler.report_level"};
}

void ProfilerModule::construct(
    ComponentContext &context
)
{
    // The one cross-component touch of the construct phase: the report sink.
    // The requirement is declared in the manifest and in the static plugin
    // descriptor, so the grant ("capability.logging") was derived from the
    // requirement and the provider is ordered before this module.
    logger = &context.acquireCapability<LoggerCapability>();

    const std::string levelName = context.configurationString("report_level").value_or("information");
    reportLevel                 = parseReportLevel(levelName);
}

void ProfilerModule::start()
{
    if (logger != nullptr)
    {
        logger->logMessage(LogLevel::Information, "profiler module started");
    }
}

void ProfilerModule::stop()
{
    emitReport();
}

void ProfilerModule::addCounterAmount(
    const std::string &name, double amount
)
{
    counters[name] += amount;
}

void ProfilerModule::addSampleSeconds(
    const std::string &name, double seconds
)
{
    SampleStatistics &statistics = samples[name];
    statistics.count += 1;
    statistics.totalSeconds += seconds;
    statistics.maxSeconds = statistics.maxSeconds > seconds ? statistics.maxSeconds : seconds;
}

void ProfilerModule::emitReport()
{
    if (logger == nullptr)
    {
        return; // never constructed: nothing was measured, nothing to report
    }

    char line[256];

    std::snprintf(line, sizeof(line), "profiler report: %zu counter(s), %zu sample group(s)",
                  counters.size(), samples.size());
    logger->logMessage(reportLevel, line);

    for (const auto &entry : counters)
    {
        std::snprintf(line, sizeof(line), "  counter '%s': total %.3f", entry.first.c_str(), entry.second);
        logger->logMessage(reportLevel, line);
    }

    for (const auto &entry : samples)
    {
        const SampleStatistics &statistics = entry.second;
        const double            meanSeconds = statistics.count > 0 ? statistics.totalSeconds / static_cast<double>(statistics.count) : 0.0;
        std::snprintf(line, sizeof(line),
                      "  sample '%s': %llu call(s), total %.4f s, mean %.4f s, max %.4f s",
                      entry.first.c_str(),
                      static_cast<unsigned long long>(statistics.count),
                      statistics.totalSeconds,
                      meanSeconds,
                      statistics.maxSeconds);
        logger->logMessage(reportLevel, line);
    }
}

void *ProfilerModule::capabilityPointerFor(
    const std::string &capabilityName
)
{
    if (capabilityName == ProfilerCapability::capabilityName)
    {
        return static_cast<ProfilerCapability *>(this);
    }
    return nullptr;
}

} // namespace engine
