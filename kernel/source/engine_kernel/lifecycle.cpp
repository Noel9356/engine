// EngineKernel lifecycle: validate dry run, construct, start and stop.
//
// validate() runs the whole pipeline without executing participant code;
// construct() re-runs it with trust enforced and builds participants in
// the resolved topological order; start()/stop() walk that order.

#include "capability_provider_index.hpp"
#include "kernel_internals.hpp"

#include <engine/kernel/dependency_resolver.hpp>
#include <engine/kernel/engine_kernel.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/kernel/task_scheduler.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
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
    DependencyResolver                         resolver(manifests, configurationStore_.providerSelections());
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

    for (const std::string &warning : configurationStore_.unknownKeyWarnings())
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
            participantStartOrder_.push_back(static_cast<std::size_t>(record - records_.data()));
            continue;
        }

        record->context             = std::make_unique<ParticipantContext>(bus_,
            *this,
            permissionSystem_,
            configurationStore_,
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
        participantStartOrder_.push_back(static_cast<std::size_t>(record - records_.data()));

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
        // Bounds duplicated from TaskSchedulerOptions::normalise are not
        // re-clamped here; only the ranges normalise does not own (the
        // outstanding-task cap, the aging ceiling) are bounded locally.
        if (const auto workers = configurationStore_.integerAtPath("engine", "worker_count"); workers && *workers >= 0)
        {
            options.workerCount = static_cast<std::uint32_t>(*workers);
            hasOptions          = true;
        }
        if (const auto maximum = configurationStore_.integerAtPath("engine", "max_outstanding_tasks");
            maximum && *maximum >= 0)
        {
            options.maxOutstandingTasks = static_cast<std::size_t>(std::min<std::int64_t>(*maximum, 10'000'000));
            hasOptions                  = true;
        }
        if (const auto workerCap = configurationStore_.integerAtPath("engine", "max_worker_count");
            workerCap && *workerCap > 0)
        {
            options.maxWorkerCount = static_cast<std::uint32_t>(*workerCap);
            hasOptions             = true;
        }
        if (const auto aging = configurationStore_.integerAtPath("engine", "priority_aging_ms"); aging && *aging >= 0)
        {
            options.priorityAgingThreshold = std::chrono::milliseconds(std::min<std::int64_t>(*aging, 600'000));
            hasOptions                     = true;
        }
        if (const auto fairness = configurationStore_.booleanAtPath("engine", "queue_source_fairness"); fairness)
        {
            options.queueSourceFairness = *fairness;
            hasOptions                  = true;
        }
        if (const auto spin = configurationStore_.integerAtPath("engine", "worker_spin_before_park");
            spin && *spin >= 0)
        {
            options.workerSpinBeforePark = static_cast<std::uint32_t>(*spin);
            hasOptions                   = true;
        }
        if (const auto idleSpin = configurationStore_.integerAtPath("engine", "wait_idle_spin_iterations");
            idleSpin && *idleSpin >= 0)
        {
            options.waitIdleSpinIterations = static_cast<std::uint32_t>(*idleSpin);
            hasOptions                     = true;
        }
        if (hasOptions)
            taskScheduler_.setOptions(options);
    }
    taskScheduler_.start();
    if (constructed)
    {
        return true;
    }

    // Records gathered by validate (no native code) must be rebuilt for a real
    // construct. Only the gather state is dropped here: stopping the scheduler
    // and releasing the bus channels would reject tasks and deferred publishes
    // submitted by participant construct() until the start() below restarts
    // them, and nothing constructed exists to tear down at this point.
    if (participantsGathered && !nativePluginLibrariesLoaded_)
    {
        records_.clear();
        participantStartOrder_.clear();
        participantsGathered = false;
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
    for (const std::size_t recordIndex : participantStartOrder_)
    {
        ParticipantRecord &record = records_[recordIndex];
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
    bus_.publish(EngineStartupCompletedEvent{});
}

// Teardown in reverse start order: deferred bus_ events flush and the shutdown event
// publishes first (while the provider index is still populated, so handlers run
// against a fully started engine and may still acquire capabilities), then the
// index resets, participants stop and the scheduler/bus_ wind down. Idempotent
// and safe during unwinding.
void EngineKernel::stop()
{
    if (started)
    {
        try
        {
            bus_.flushDeferredEvents();
            bus_.publish(EngineShutdownStartedEvent{});
        }
        catch (const std::exception &error)
        {
            std::fprintf(stderr, "engine kernel: shutdown event handler failed: %s\n", error.what());
        }
    }

    // Reset the provider index only after the dispatch above: deferred and
    // shutdown handlers execute participant code that may resolve capabilities
    // through ParticipantContext, and with an empty index every resolve throws
    // "no active provider". The reset must still precede the teardown below:
    // the index caches raw ModuleInterface*/PluginInterface* pointers, plugins
    // are destroyed by pluginEntryTable->destroyPlugin and module instances
    // (plus the .so files backing their vtables) only die when records_.clear()
    // unloads the libraries - entries kept past those points would be
    // use-after-free material. No participant code runs between this reset and
    // the last of those pointers going away.
    capabilityProviderIndex_->reset();

    mainThreadTaskQueue_.clear();

    // Drain the scheduler BEFORE any participant stops or is destroyed: a
    // scheduled task can hold a capability pointer into a participant (module
    // instance or native plugin), and waitIdle only guarantees completion
    // when it runs ahead of the teardown that invalidates those pointers.
    // Modules die later still (records_.clear() unloads their libraries), but
    // no participant code runs past this point.
    taskScheduler_.waitIdle();

    for (auto iterator = participantStartOrder_.rbegin(); iterator != participantStartOrder_.rend(); ++iterator)
    {
        ParticipantRecord &record = records_[*iterator];
        if (record.started && record.moduleInstance != nullptr)
        {
            record.moduleInstance->stop();
        }
        if (record.started && record.pluginInstance != nullptr)
        {
            record.pluginInstance->stop();
        }
    }
    for (ParticipantRecord &record : records_)
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

    taskScheduler_.stop();
    bus_.releaseChannels();

    records_.clear();
    participantStartOrder_.clear();
    participantsGathered = false;
}

// Debug dump of participants and bus_ subscriptions; when nothing is constructed yet
// it falls back to a fresh graph resolution (reporting resolution failures inline).
std::string EngineKernel::participantGraphReport() const
{
    std::string report;

    std::vector<const ParticipantRecord *> orderedRecords;
    const char                            *orderSource = "start order";
    if (!participantStartOrder_.empty() && !records_.empty())
    {
        orderedRecords.reserve(participantStartOrder_.size());
        for (const std::size_t recordIndex : participantStartOrder_)
        {
            orderedRecords.push_back(&records_[recordIndex]);
        }
    }
    else if (!records_.empty())
    {
        std::string                                resolutionReport;
        const DependencyResolver::ResolutionResult resolution = resolveGraph(participantManifests(), resolutionReport);
        orderSource                                           = "resolved construct order (nothing is constructed yet)";
        if (resolution.successful)
        {
            for (const std::size_t manifestIndex : resolution.constructionOrder)
            {
                orderedRecords.push_back(&records_[manifestIndex]);
            }
        }
        else
        {
            orderSource = "declared order (graph resolution failed - see --validate)";
            for (const ParticipantRecord &record : records_)
            {
                orderedRecords.push_back(&record);
            }
            report += resolutionReport;
        }
    }

    report += "participant graph (" + std::to_string(records_.size()) + " participant(s), " + orderSource + "):\n";
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

    const std::string subscriptions = bus_.subscriptionReport();
    if (!subscriptions.empty())
    {
        report += subscriptions;
    }
    return report;
}

} // namespace engine
