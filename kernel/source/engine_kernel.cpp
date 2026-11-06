// EngineKernel construction, staged setters and the application context.
//
// The phase logic lives in engine_kernel/*.cpp; this file holds the
// constructor/destructor, the input setters (which only mark state stale)
// and small accessors.

#include "capability_provider_index.hpp"
#include "engine_kernel_internal.hpp"
#include "trust_gate.hpp"
#include <engine/kernel/engine_kernel.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>

#include <filesystem>
#include <memory>
#include <string>
#include <utility>
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
    configurationStore.reset();

    participantsGathered = false;
}

void EngineKernel::setConfigurationProfileFile(const std::filesystem::path &profilePath)
{
    configurationProfileFilePath = profilePath;
    configurationProfileFilePaths.clear();
    configurationLoaded = false;
    configurationStore.reset();
    participantsGathered = false;
}

// Appends a profile; the single-profile slot is drained into the list to
// preserve ordering across both APIs.
void EngineKernel::addConfigurationProfileFile(const std::filesystem::path &profilePath)
{
    if (profilePath.empty())
        return;

    if (!configurationProfileFilePath.empty())
    {
        configurationProfileFilePaths.insert(configurationProfileFilePaths.begin(), configurationProfileFilePath);
        configurationProfileFilePath.clear();
    }
    configurationProfileFilePaths.push_back(profilePath);
    configurationLoaded = false;
    configurationStore.reset();
    participantsGathered = false;
}

void EngineKernel::setSceneFile(const std::filesystem::path &scenePath)
{
    sceneFilePath       = scenePath;
    configurationLoaded = false;
    configurationStore.reset();
    participantsGathered = false;
}

bool EngineKernel::addCommandLineOverride(const std::string &assignment, std::string &error)
{
    return configurationStore.stageCommandLineOverride(assignment, error);
}

void EngineKernel::setGameModsDirectory(const std::filesystem::path &modsDirectory)
{
    explicitGameModsDirectory = modsDirectory;
    explicitExtraGameModsDirectories.clear();
    participantsGathered = false;
}

void EngineKernel::addGameModsDirectory(const std::filesystem::path &modsDirectory)
{
    explicitExtraGameModsDirectories.push_back(modsDirectory);
    participantsGathered = false;
}

void EngineKernel::setApplicationRequirements(const std::vector<std::string> &capabilities)
{
    applicationRequiredCapabilities = capabilities;
    participantsGathered            = false;
}

void EngineKernel::setActiveGameMods(const std::vector<std::string> &modIds)
{
    applicationActiveGameMods = modIds;
    participantsGathered      = false;
}

void EngineKernel::clearActiveGameMods()
{
    applicationActiveGameMods.reset();
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
    return std::make_unique<ParticipantContext>(bus,
        *this,
        permissionSystem_,
        configurationStore,
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
