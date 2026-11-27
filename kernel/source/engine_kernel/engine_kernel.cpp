// EngineKernel construction, staged setters and the application context.
//
// The phase logic lives in engine_kernel/*.cpp; this file holds the
// constructor/destructor, the input setters (which only mark state stale)
// and small accessors.

#include "capability_provider_index.hpp"
#include <engine/kernel/engine_kernel.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include "engine_kernel_internal.hpp"
#include "trust_gate.hpp"

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace engine
{

// Scheduler metrics feed the shared profiler from the start.
EngineKernel::EngineKernel()
{
    trustGate_               = std::make_unique<TrustGate>();
    capabilityProviderIndex_ = std::make_unique<CapabilityProviderIndex>();
    taskScheduler_.setProfiler(&profiler_);
}

// Every setter below clears configurationLoaded/participantsGathered so
// the next validate/construct rebuilds everything.
void EngineKernel::setConfigurationFile(const std::filesystem::path &configurationPath)
{
    configurationFilePath = configurationPath;
    configurationLoaded   = false;
    // Only the document is invalidated: staged command-line overrides (and
    // the known-key registry) must survive later path staging, so the call
    // order of the setters cannot silently drop them (full reset() would).
    configurationStore_.resetForReload();

    participantsGathered = false;
}

// Appends a profile; profiles merge in the order they were added. This is the
// only profile API: the application host stages a fresh kernel per run, so
// "append to an empty list" covers the retired single-profile setter too.
void EngineKernel::addConfigurationProfileFile(const std::filesystem::path &profilePath)
{
    if (profilePath.empty())
        return;

    configurationProfileFilePaths.push_back(profilePath);
    configurationLoaded = false;
    // See setConfigurationFile: keep staged overrides across the staging call.
    configurationStore_.resetForReload();
    participantsGathered = false;
}

void EngineKernel::setSceneFile(const std::filesystem::path &scenePath)
{
    sceneFilePath       = scenePath;
    configurationLoaded = false;
    // See setConfigurationFile: keep staged overrides across the staging call.
    configurationStore_.resetForReload();
    participantsGathered = false;
}

bool EngineKernel::addCommandLineOverride(const std::string &assignment, std::string &error)
{
    return configurationStore_.stageCommandLineOverride(assignment, error);
}

void EngineKernel::setPluginsDirectory(const std::filesystem::path &pluginsDirectory)
{
    explicitPluginsDirectory = pluginsDirectory;
    explicitExtraPluginsDirectories.clear();
    participantsGathered = false;
}

void EngineKernel::addPluginsDirectory(const std::filesystem::path &pluginsDirectory)
{
    explicitExtraPluginsDirectories.push_back(pluginsDirectory);
    participantsGathered = false;
}

void EngineKernel::setApplicationRequirements(const std::vector<std::string> &capabilities)
{
    applicationRequiredCapabilities = capabilities;
    participantsGathered            = false;
}

void EngineKernel::setActivePlugins(const std::vector<std::string> &pluginIds)
{
    applicationActivePlugins = pluginIds;
    participantsGathered      = false;
}

void EngineKernel::clearActivePlugins()
{
    applicationActivePlugins.reset();
    participantsGathered = false;
}

// stop() is idempotent and safe during unwinding.
EngineKernel::~EngineKernel()
{
    stop();
}

// "<app>" gets every permission; its context participates in capability
// resolution like any other participant.
std::unique_ptr<ParticipantContext> EngineKernel::createAppContext()
{
    permissionSystem_.grantAllPermissions(detail::appContextName());
    return std::make_unique<ParticipantContext>(bus_,
        *this,
        permissionSystem_,
        configurationStore_,
        detail::appContextName(),
        ParticipantKind::Application,
        nullptr,
        mainThreadTaskQueue_,
        taskScheduler_,
        profiler_);
}

void EngineKernel::setTrustConsentHandler(TrustConsentHandler handler)
{
    trustGate_->setConsentHandler(std::move(handler));
}

const std::vector<ParticipantTrustAdvisory> &EngineKernel::trustAdvisories() const
{
    return trustGate_->advisories();
}

} // namespace engine
