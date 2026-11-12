// EngineKernel lifecycle: validate dry run, construct, start and stop.
//
// validate() runs the whole pipeline without executing participant code;
// construct() re-runs it with trust enforced and builds participants in
// the resolved topological order; start()/stop() walk that order.

#include "capability_provider_index.hpp"
#include "kernel_internals.hpp"

#include "engine/kernel/dependency_resolver.hpp"
#include "engine/kernel/task_scheduler.hpp"
#include <engine/kernel/engine_kernel.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace engine
{

// Runs the resolver with the configured [providers] selections; failures are
// appended to the report.
DependencyResolver::ResolutionResult EngineKernel::resolveGraph(
    const std::vector<ParticipantManifest> &manifests, std::string &report) const
{
    DependencyResolver                         resolver(manifests, configurationStore.providerSelections());
    const DependencyResolver::ResolutionResult resolution = resolver.resolve();
    if (!resolution.successful)
    {
        appendDiagnosticsReport(resolution.diagnostics, report);
    }
    return resolution;
}

void EngineKernel::appendDiagnosticsReport(
    const std::vector<DependencyResolver::ResolutionDiagnostic> &diagnostics, std::string &report) const
{
    report += "engine startup failed with " + std::to_string(diagnostics.size()) + " problem(s):\n";
    for (const DependencyResolver::ResolutionDiagnostic &diagnostic : diagnostics)
    {
        report += "  - " + diagnostic.message + "\n";
    }
}

// Dry run without executing participant code: gather, permission pass, known-key
// registry, then the full graph resolution. Every diagnostic lands in the report.
bool EngineKernel::validate(std::string &report)
{
    report.clear();
    if (!prepareParticipants(report, false))
    {
        report += startupWarnings_;
        return false;
    }

    std::vector<DependencyResolver::ResolutionDiagnostic> permissionDiagnostics;
    for (const auto &[record, permission] : applyPermissionPass())
    {
        permissionDiagnostics.push_back(DependencyResolver::ResolutionDiagnostic{
            .kind    = DependencyResolver::ResolutionDiagnostic::Kind::UnknownCapability,
            .message = describeComponentKind(record->manifest.kind) + " '" + record->name + "' requires permission '"
                     + permission + "' which the configuration denies."});
    }
    if (!permissionDiagnostics.empty())
    {
        appendDiagnosticsReport(permissionDiagnostics, report);
    }

    if (!declareKnownKeyRegistry(report))
    {
        report += startupWarnings_;
        return false;
    }

    if (!report.empty())
    {
        report += startupWarnings_;
        return false;
    }

    report += deferredConfigurationWarnings;
    report += startupWarnings_;

    std::string                                resolutionReport;
    const DependencyResolver::ResolutionResult resolution = resolveGraph(participantManifests(), resolutionReport);
    report += resolutionReport;
    if (!resolution.successful)
    {
        return false;
    }

    for (const std::string &warning : configurationStore.unknownKeyWarnings())
    {
        report += "warning: " + warning + "\n";
    }
    return true;
}

// Constructs participants in providers-first order: script plugins go through their
// resolved host module; every declared capability must come back implemented.
bool EngineKernel::runConstructPhase(const DependencyResolver::ResolutionResult &resolution,
    const std::vector<ParticipantManifest> &manifests, std::string &report)
{
    for (const std::size_t manifestIndex : resolution.constructionOrder)
    {
        const ParticipantManifest &manifest = manifests[manifestIndex];
        ParticipantRecord         *record   = findRecord(manifest.name);
        if (record == nullptr || record->constructed)
        {
            continue;
        }

        if (record->manifest.kind == ParticipantKind::ScriptPlugin)
        {
            ParticipantRecord *hostRecord = resolveScriptHost(*record, report);
            if (hostRecord == nullptr)
            {
                return false;
            }
            if (!hostRecord->constructed)
            {
                report += "- script plugin '" + record->name + "' cannot be hosted: module '" + hostRecord->name
                        + "' did not construct.\n";
                return false;
            }
            ScriptPluginHost *scriptHost = hostRecord->moduleInstance->scriptPluginHost();
            if (scriptHost == nullptr)
            {
                report += "- script plugin '" + record->name + "' cannot be hosted: module '" + hostRecord->name
                        + "' provides the '" + record->scriptPlugin.hostCapability
                        + "' capability but does not host script plugins.\n";
                return false;
            }
            try
            {
                scriptHost->hostScriptPlugin(record->scriptPlugin);
            }
            catch (const std::exception &error)
            {
                report += "- construct failed for script plugin '" + record->name + "': " + error.what() + "\n";
                return false;
            }
            record->constructed = true;
            participantStartOrder.push_back(record);
            continue;
        }

        record->context             = std::make_unique<ParticipantContext>(bus,
            *this,
            permissionSystem_,
            configurationStore,
            record->name,
            record->manifest.kind,
            &capabilityProviderIndex_->demandedCapabilities(),
            mainThreadTaskQueue_,
            taskScheduler_,
            profiler_);
        ParticipantContext &context = *record->context;
        try
        {
            if (record->moduleInstance != nullptr)
            {
                record->moduleInstance->construct(context);
            }
            else if (record->pluginInstance != nullptr)
            {
                record->pluginInstance->construct(context);
            }
        }
        catch (const std::exception &error)
        {
            report += "- construct failed for " + describeComponentKind(record->manifest.kind) + " '" + record->name
                    + "': " + error.what() + "\n";
            return false;
        }

        for (const std::string &capabilityName : record->manifest.providedCapabilities)
        {
            void *capabilityPointer = nullptr;
            if (record->moduleInstance != nullptr)
            {
                capabilityPointer = record->moduleInstance->capabilityPointerFor(capabilityName);
            }
            else if (record->pluginInstance != nullptr)
            {
                capabilityPointer = record->pluginInstance->capabilityPointerFor(capabilityName);
            }
            if (capabilityPointer == nullptr)
            {
                report += "- " + describeComponentKind(record->manifest.kind) + " '" + record->name
                        + "' declares capability '" + capabilityName + "' but returned no implementation.\n";
                return false;
            }
        }
        record->constructed = true;
        participantStartOrder.push_back(record);

        // Only fully constructed participants enter the provider index.
        capabilityProviderIndex_->addProvider(CapabilityProviderEntry{
            .name                 = record->name,
            .kind                 = record->manifest.kind,
            .providedCapabilities = record->manifest.providedCapabilities,
            .moduleInstance       = record->moduleInstance,
            .pluginInstance      = record->pluginInstance});
    }
    return true;
}

void *EngineKernel::resolveCapabilityPointer(
    const std::string &, ParticipantKind requesterKind, const std::string &capabilityName)
{
    return capabilityProviderIndex_->resolveCapabilityPointer(requesterKind, capabilityName);
}

bool EngineKernel::hasSingleCapabilityProvider(const std::string &capabilityName, ParticipantKind requesterKind)
{
    return capabilityProviderIndex_->hasSingleCapabilityProvider(capabilityName, requesterKind);
}

void *EngineKernel::tryResolveCapabilityPointer(
    const std::string &, ParticipantKind requesterKind, const std::string &capabilityName)
{
    return capabilityProviderIndex_->tryResolveCapabilityPointer(requesterKind, capabilityName);
}

std::vector<void *> EngineKernel::resolveEachCapabilityPointer(
    const std::string &, ParticipantKind requesterKind, const std::string &capabilityName)
{
    return capabilityProviderIndex_->resolveEachCapabilityPointer(requesterKind, capabilityName);
}

// Full startup: scheduler options come from [engine], participants are gathered with
// trust enforced and constructed in dependency order, then the provider index
// freezes (it becomes a cache from then on).
bool EngineKernel::construct(std::string &report)
{
    report.clear();
    {
        // [engine] tuning (workers, queue limits, priority aging, fairness) maps onto the
        // scheduler options; unset keys keep the current defaults.
        TaskSchedulerOptions options    = taskScheduler_.options();
        bool                 hasOptions = false;
        if (const auto workers = configurationStore.integerAtPath("engine", "worker_count"); workers && *workers >= 0)
        {
            options.workerCount = static_cast<std::uint32_t>(std::min<std::int64_t>(*workers, 256));
            hasOptions          = true;
        }
        if (const auto maximum = configurationStore.integerAtPath("engine", "max_outstanding_tasks");
            maximum && *maximum >= 0)
        {
            options.maxOutstandingTasks = static_cast<std::size_t>(std::min<std::int64_t>(*maximum, 10'000'000));
            hasOptions                  = true;
        }
        if (const auto workerCap = configurationStore.integerAtPath("engine", "max_worker_count");
            workerCap && *workerCap > 0)
        {
            options.maxWorkerCount = static_cast<std::uint32_t>(std::min<std::int64_t>(*workerCap, 256));
            hasOptions             = true;
        }
        if (const auto aging = configurationStore.integerAtPath("engine", "priority_aging_ms"); aging && *aging >= 0)
        {
            options.priorityAgingThreshold = std::chrono::milliseconds(std::min<std::int64_t>(*aging, 600'000));
            hasOptions                     = true;
        }
        if (const auto fairness = configurationStore.booleanAtPath("engine", "queue_source_fairness"); fairness)
        {
            options.queueSourceFairness = *fairness;
            hasOptions                  = true;
        }
        if (hasOptions)
            taskScheduler_.setOptions(options);
    }
    taskScheduler_.start();
    if (constructed)
    {
        return true;
    }

    // Records gathered by validate (no native code) must be rebuilt for a real construct.
    if (participantsGathered && !nativePluginLibrariesLoaded_)
    {
        stop();
    }

    capabilityProviderIndex_->reset();
    if (!prepareParticipants(report, true))
    {
        return false;
    }

    for (const auto &[record, permission] : applyPermissionPass())
    {
        report += "- " + describeComponentKind(record->manifest.kind) + " '" + record->name + "' requires permission '"
                + permission + "' which the configuration denies.\n";
    }
    if (!report.empty())
    {
        report = "engine startup failed:\n" + report;
        return false;
    }

    const std::vector<ParticipantManifest>     manifests = participantManifests();
    std::string                                resolutionReport;
    const DependencyResolver::ResolutionResult resolution = resolveGraph(manifests, resolutionReport);
    report += resolutionReport;
    if (!resolution.successful)
    {
        return false;
    }

    if (!runConstructPhase(resolution, manifests, report))
    {
        stop();
        report = "engine startup failed:\n" + report;
        return false;
    }

    // After this point provider classification results are cached.
    capabilityProviderIndex_->freeze();
    constructed = true;
    return true;
}

// Starts every constructed participant in construct order; requires a completed
// construct phase and throws otherwise.
void EngineKernel::start()
{
    if (!constructed || started)
    {
        throw std::runtime_error("engine kernel: start requires a completed construct phase");
    }

    taskScheduler_.start();
    for (ParticipantRecord *recordPointer : participantStartOrder)
    {
        ParticipantRecord &record = *recordPointer;
        if (!record.constructed || record.started)
        {
            continue;
        }
        if (record.moduleInstance != nullptr)
        {
            record.moduleInstance->start();
        }
        else if (record.pluginInstance != nullptr)
        {
            record.pluginInstance->start();
        }
        record.started = true;
    }
    started = true;
    bus.publish(EngineStartupCompletedEvent{});
}

// Teardown in reverse start order: deferred bus events flush and the shutdown event
// publishes first, then participants stop and the scheduler/bus wind down.
// Idempotent and safe during unwinding.
void EngineKernel::stop()
{
    capabilityProviderIndex_->reset();
    if (started)
    {
        try
        {
            bus.flushDeferredEvents();
            bus.publish(EngineShutdownStartedEvent{});
        }
        catch (const std::exception &error)
        {
            std::fprintf(stderr, "engine kernel: shutdown event handler failed: %s\n", error.what());
        }
    }

    mainThreadTaskQueue_.clear();

    for (auto iterator = participantStartOrder.rbegin(); iterator != participantStartOrder.rend(); ++iterator)
    {
        ParticipantRecord &record = **iterator;
        if (record.started && record.moduleInstance != nullptr)
        {
            record.moduleInstance->stop();
        }
        if (record.started && record.pluginInstance != nullptr)
        {
            record.pluginInstance->stop();
        }
    }
    for (ParticipantRecord &record : records)
    {
        if (record.pluginEntryTable != nullptr && record.pluginInstance != nullptr)
        {
            record.pluginEntryTable->destroyPlugin(record.pluginInstance);
            record.pluginInstance = nullptr;
        }
        record.started     = false;
        record.constructed = false;
    }
    started     = false;
    constructed = false;

    taskScheduler_.waitIdle();
    taskScheduler_.stop();
    bus.releaseChannels();

    records.clear();
    participantStartOrder.clear();
    participantsGathered = false;
}

// Debug dump of participants and bus subscriptions; when nothing is constructed yet
// it falls back to a fresh graph resolution (reporting resolution failures inline).
std::string EngineKernel::participantGraphReport() const
{
    std::string report;

    std::vector<const ParticipantRecord *> orderedRecords;
    const char                            *orderSource = "start order";
    if (!participantStartOrder.empty())
    {
        orderedRecords.assign(participantStartOrder.begin(), participantStartOrder.end());
    }
    else if (!records.empty())
    {
        std::string                                resolutionReport;
        const DependencyResolver::ResolutionResult resolution = resolveGraph(participantManifests(), resolutionReport);
        orderSource                                           = "resolved construct order (nothing is constructed yet)";
        if (resolution.successful)
        {
            for (const std::size_t manifestIndex : resolution.constructionOrder)
            {
                orderedRecords.push_back(&records[manifestIndex]);
            }
        }
        else
        {
            orderSource = "declared order (graph resolution failed - see --validate)";
            for (const ParticipantRecord &record : records)
            {
                orderedRecords.push_back(&record);
            }
            report += resolutionReport;
        }
    }

    report += "participant graph (" + std::to_string(records.size()) + " participant(s), " + orderSource + "):\n";
    for (const ParticipantRecord *recordPointer : orderedRecords)
    {
        const ParticipantRecord &record = *recordPointer;
        report += "  " + record.name + " (" + describeComponentKind(record.manifest.kind) + " "
                + record.manifest.version + ")\n";

        const auto appendList = [&report](const char *label, const std::vector<std::string> &names)
        {
            report += std::string("    ") + label + ":";
            if (names.empty())
            {
                report += " (none)";
            }
            else
            {
                for (const std::string &name : names)
                {
                    report += " " + name;
                }
            }
            report += "\n";
        };
        appendList("provides", record.manifest.providedCapabilities);
        appendList("requires", record.manifest.requiredCapabilities);
        if (!record.manifest.optionalCapabilities.empty())
        {
            appendList("optional", record.manifest.optionalCapabilities);
        }
        appendList("permissions", record.manifest.requiredPermissions);
        if (!record.scriptPlugin.pluginId.empty())
        {
            report += "    script host: "
                    + (record.scriptHostName.empty() ? std::string("(unresolved)") : record.scriptHostName) + "\n";
        }
        report += std::string("    state: ")
                + (record.started        ? "started"
                    : record.constructed ? "constructed"
                                         : "declared")
                + "\n";
    }

    const std::string subscriptions = bus.subscriptionReport();
    if (!subscriptions.empty())
    {
        report += subscriptions;
    }
    return report;
}

} // namespace engine
