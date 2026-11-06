// EngineKernel discovery: permission pass and participant gathering.
//
// gatherParticipants assembles every participant record (the <app>
// context, module libraries, game-mod packages), the demand closure picks
// active modules, and the trust gate runs before any native code loads.

#include "../capability_provider_index.hpp"
#include "../engine_kernel_internal.hpp"
#include "../trust_gate.hpp"

#include "engine/kernel/configuration.hpp"
#include "engine/kernel/dependency_resolver.hpp"
#include "engine/kernel/module_descriptor.hpp"
#include "engine/kernel/participant_interface.hpp"
#include <engine/kernel/capability_name.hpp>
#include <engine/kernel/engine_kernel.hpp>
#include <engine/kernel/game_mod_discovery.hpp>
#include <engine/kernel/game_mod_manifest.hpp>
#include <engine/kernel/participant_manifest.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace engine
{

// Re-applies the configured [permissions].denied list, declares each participant's
// effective permissions (manifest permissions plus capability.<name> for every
// capability) and returns the (participant, permission) pairs that end up denied.
std::vector<std::pair<const EngineKernel::ParticipantRecord *, std::string>> EngineKernel::applyPermissionPass()
{
    std::vector<std::pair<const ParticipantRecord *, std::string>> denied;
    for (const auto &denial : configurationStore.permissionDenials())
    {
        permissionSystem_.denyPermission(denial.first, denial.second);
    }
    for (const ParticipantRecord &record : records)
    {
        const std::vector<std::string> grants = detail::effectivePermissions(record.manifest);
        permissionSystem_.declarePermissions(record.name, grants);
        const std::vector<std::string> missing = permissionSystem_.missingPermissions(record.name, grants);
        for (const std::string &permission : missing)
        {
            denied.emplace_back(&record, permission);
        }
    }
    return denied;
}

std::vector<ParticipantManifest> EngineKernel::participantManifests() const
{
    std::vector<ParticipantManifest> manifests;
    manifests.reserve(records.size());
    for (const ParticipantRecord &record : records)
    {
        manifests.push_back(record.manifest);
    }
    return manifests;
}

EngineKernel::ParticipantRecord *EngineKernel::findRecord(const std::string &name)
{
    for (ParticipantRecord &candidate : records)
    {
        if (candidate.name == name)
        {
            return &candidate;
        }
    }
    return nullptr;
}

// Assembles the participant list without constructing anything. With enforceTrust
// false (the validate dry run) no native code runs: modules are declared from
// descriptors only and native mod libraries stay unloaded.
bool EngineKernel::gatherParticipants(std::string &report, bool enforceTrust)
{
    if (participantsGathered)
    {
        return true;
    }

    // A re-gather tears down any previous assembly first.
    if (!records.empty() || !moduleLibraries.empty())
    {
        stop();
    }

    if (!applicationRequiredCapabilities.empty())
    {
        ParticipantRecord applicationRecord{
            .name     = detail::appContextName(),
            .manifest = ParticipantManifest{
                .name                 = detail::appContextName(),
                .kind                 = ParticipantKind::Application,
                .requiredCapabilities = applicationRequiredCapabilities}};
        records.push_back(std::move(applicationRecord));
        permissionSystem_.grantAllPermissions(detail::appContextName());
    }

    trustGate_->clearAdvisories();

    loadModuleLibraries(startupWarnings_);

    gatherGameModPackages(startupWarnings_);

    resolveUndescribedModuleLibraries(report, enforceTrust);

    computeDemandClosure();

    for (const ModuleLibraryRecord &module : moduleLibraries)
    {
        if (module.active)
        {
            trustGate_->appendModuleRiskWarning(module, startupWarnings_);
        }
    }

    if (!trustGate_->evaluate(report, enforceTrust, startupWarnings_, configurationStore))
    {
        return false;
    }

    if (enforceTrust)
    {
        loadNativeGameModLibraries(startupWarnings_);
        nativeGameModLibrariesLoaded_ = true;
        if (!instantiateActiveModules(report))
        {
            return false;
        }
    }
    else
    {
        appendDeclaredModuleRecords();
    }

    // Duplicate participant names would alias in the graph and on the bus; refuse the run.
    for (std::size_t outer = 0; outer < records.size(); ++outer)
    {
        for (std::size_t inner = outer + 1; inner < records.size(); ++inner)
        {
            if (records[outer].name == records[inner].name)
            {
                report += "- participant name '" + records[outer].name
                        + "' is declared twice (a mod id and a module name, or two mods sharing an id); "
                          "participant names must be unique across the whole assembly.\n";
                return false;
            }
        }
    }

    participantsGathered = true;
    return true;
}

// Scans the mods directories for mod.toml packages, applies the active mod set and
// turns each package into a ParticipantRecord; native libraries wait in
// pendingNativeGameModPackages_ until the trust gate passes.
void EngineKernel::gatherGameModPackages(std::string &report)
{
    gameModSceneClaims.clear();
    gameModDefaultsFiles.clear();
    pendingNativeGameModPackages_.clear();
    nativeGameModLibrariesLoaded_ = false;

    std::vector<std::filesystem::path> scanRoots;
    if (!explicitGameModsDirectory.empty() || !explicitExtraGameModsDirectories.empty())
    {
        if (!explicitGameModsDirectory.empty())
        {
            scanRoots.push_back(detail::resolveAgainstExecutableDirectory(explicitGameModsDirectory));
        }
        for (const std::filesystem::path &extraDirectory : explicitExtraGameModsDirectories)
        {
            scanRoots.push_back(detail::resolveAgainstExecutableDirectory(extraDirectory));
        }
    }
    else
    {
        scanRoots.push_back(detail::resolveAgainstExecutableDirectory(configurationStore.modDirectory()));
        for (const std::string &extraDirectory : configurationStore.extraModDirectories())
        {
            scanRoots.push_back(detail::resolveAgainstExecutableDirectory(extraDirectory));
        }
    }

    std::map<std::string, std::filesystem::path> discoveredPackageRoots;
    std::vector<DiscoveredGameModPackage>        packages;
    for (const std::filesystem::path &scanRoot : scanRoots)
    {
        for (DiscoveredGameModPackage &package :
            discoverModPackages(scanRoot, configurationStore.disabledMods(), report))
        {
            const auto previouslySeen = discoveredPackageRoots.find(package.manifest.id);
            if (previouslySeen != discoveredPackageRoots.end())
            {
                report += "- mod '" + package.manifest.id + "' appears in more than one mods directory ('"
                        + previouslySeen->second.string() + "' and '" + scanRoot.string()
                        + "'); mod ids are unique across every scanned directory and the later package is "
                          "dropped.\n";
                continue;
            }
            discoveredPackageRoots.emplace(package.manifest.id, scanRoot);
            packages.push_back(std::move(package));
        }
    }

    // An explicit app set wins over [engine].active_mods; no set at all means every
    // discovered mod activates.
    std::optional<std::vector<std::string>> activeSet = applicationActiveGameMods;
    if (!activeSet)
    {
        activeSet = configurationStore.activeMods();
    }
    if (activeSet)
    {
        std::vector<DiscoveredGameModPackage> activated;
        for (DiscoveredGameModPackage &package : packages)
        {
            if (std::find(activeSet->begin(), activeSet->end(), package.manifest.id) == activeSet->end())
            {
                report += "- mod '" + package.manifest.id
                        + "' was not activated: it is not in the app's active mod set.\n";
                continue;
            }
            activated.push_back(std::move(package));
        }
        packages = std::move(activated);
    }

    for (const DiscoveredGameModPackage &package : packages)
    {
        const GameModManifest &gameModManifest = package.manifest;

        if (!package.sceneFile.empty())
        {
            gameModSceneClaims.push_back(
                GameModSceneClaim{.sceneFile = package.sceneFile, .ownerId = gameModManifest.id});
        }

        // Mod-shipped defaults merge right here (ModDefaults layer, fill-only).
        if (!package.defaultsFile.empty())
        {
            std::string defaultsError;
            if (!configurationStore.mergeDefaultsFile(package.defaultsFile,
                    ConfigProvenance::Layer::ModDefaults,
                    gameModManifest.directory.filename().string(),
                    defaultsError))
            {
                report += "- " + defaultsError + "\n";
                continue;
            }

            gameModDefaultsFiles.emplace_back(package.defaultsFile, gameModManifest.directory.filename().string());
        }

        if (gameModManifest.isNative())
        {
            ParticipantRecord record{
                .name     = gameModManifest.id,
                .manifest = detail::manifestFromModManifest(gameModManifest, ParticipantKind::NativeGameMod)};
            trustGate_->appendModRiskWarning(gameModManifest, true, startupWarnings_);
            pendingNativeGameModPackages_.push_back(package);
            records.push_back(std::move(record));
        }
        else
        {
            ParticipantRecord record{
                .name      = gameModManifest.id,
                .manifest  = detail::manifestFromModManifest(gameModManifest, ParticipantKind::ScriptGameMod),
                .scriptMod = ScriptGameModDescription{
                    .modId                = gameModManifest.id,
                    .scriptPath           = gameModManifest.directory / gameModManifest.entry,
                    .hostCapability       = gameModManifest.hostCapability,
                    .declaredCapabilities = gameModManifest.capabilities,
                    .declaredPermissions  = gameModManifest.permissions,
                    .version              = gameModManifest.version}};
            trustGate_->appendModRiskWarning(gameModManifest, false, startupWarnings_);
            records.push_back(std::move(record));
        }
    }
}

// Fixpoint over demanded capabilities: seed with every participant's required
// capabilities, then let each newly demanded capability activate its providing
// modules and queue those modules' own requirements.
void EngineKernel::computeDemandClosure()
{
    CapabilityProviderIndex &index = *capabilityProviderIndex_;
    index.resetDemandedCapabilities();
    for (ModuleLibraryRecord &module : moduleLibraries)
    {
        module.active = false;
    }

    const std::vector<DependencyResolver::ProviderSelection> selections = configurationStore.providerSelections();

    std::vector<std::string> worklist;
    for (const ParticipantRecord &record : records)
    {
        for (const std::string &capabilityName : record.manifest.requiredCapabilities)
        {
            worklist.push_back(capabilityName);
        }
    }

    const auto providesCapability = [](const std::string &requirement, const auto &provided)
    {
        return std::any_of(provided.begin(),
            provided.end(),
            [&requirement](const std::string &candidate)
            { return capabilityRequirementSatisfiedBy(requirement, candidate); });
    };

    while (!worklist.empty())
    {
        const std::string capabilityName = std::move(worklist.back());
        worklist.pop_back();
        if (!index.demandCapability(capabilityName))
        {
            continue;
        }

        std::vector<std::string> providerNames;
        for (ModuleLibraryRecord &module : moduleLibraries)
        {
            if (providesCapability(capabilityName, module.descriptor.providedCapabilities))
            {
                providerNames.push_back(module.descriptor.name);
            }
        }
        for (const ParticipantRecord &record : records)
        {
            if (providesCapability(capabilityName, record.manifest.providedCapabilities))
            {
                providerNames.push_back(record.name);
            }
        }
        // Several providers: a matching [providers] selection narrows to one; without a
        // selection all stay (the resolver reports the ambiguity later).
        if (providerNames.size() > 1)
        {
            const auto selection = std::find_if(selections.begin(),
                selections.end(),
                [&capabilityName](const DependencyResolver::ProviderSelection &candidate)
                { return candidate.capabilityName == capabilityName; });
            if (selection != selections.end())
            {
                const bool selectionMatches
                    = std::find(providerNames.begin(), providerNames.end(), selection->providerName)
                   != providerNames.end();
                if (selectionMatches)
                {
                    providerNames.assign(1, selection->providerName);
                }
            }
        }

        for (const std::string &providerName : providerNames)
        {
            for (ModuleLibraryRecord &module : moduleLibraries)
            {
                if (module.descriptor.name != providerName || module.active)
                {
                    continue;
                }
                module.active = true;

                // The concrete provided names are demanded too (a family match may need one).
                for (const std::string &providedName : module.descriptor.providedCapabilities)
                {
                    if (capabilityRequirementSatisfiedBy(capabilityName, providedName))
                    {
                        index.demandCapability(providedName);
                    }
                }
                for (const std::string &requiredName : module.descriptor.requiredCapabilities)
                {
                    worklist.push_back(requiredName);
                }
            }
        }
    }
}

} // namespace engine
