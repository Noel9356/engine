// EngineKernel configuration pipeline: layered loading, scene resolution
// and live reload.
//
// loadConfiguration merges engine.toml + module defaults + profiles;
// finishConfiguration applies scene files (assembly or sole mod claim)
// and staged overrides; reloadConfiguration re-runs the pipeline after a
// pre-parse validation, keeping the old configuration on any failure.

#include "engine/kernel/configuration.hpp"
#include <engine/kernel/engine_kernel.hpp>

#include "../engine_kernel_internal.hpp"
#include "../kernel_internals.hpp"
#include "engine/kernel/participant_interface.hpp"

#include <cstddef>
#include <filesystem>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace engine
{

// Idempotent until invalidated; profiles are override-merged AFTER module
// defaults so a profile can re-tune a default but not assembly sections.
bool EngineKernel::loadConfiguration(std::string &report)
{
    if (configurationLoaded)
    {
        return true;
    }
    deferredConfigurationWarnings.clear();
    assemblyDefinesScene = false;
    if (configurationFilePath.empty())
    {
        report += "- no configuration file was set on the kernel.\n";
        return false;
    }
    const std::filesystem::path resolvedPath = detail::resolveAgainstExecutableDirectory(configurationFilePath);
    if (!std::filesystem::exists(resolvedPath))
    {
        report += "- configuration file '" + resolvedPath.string() + "' does not exist.\n";
        return false;
    }

    std::string loadError;
    if (!configurationStore.loadFromFile(resolvedPath, loadError))
    {
        report += "- " + loadError + "\n";
        return false;
    }

    // True when engine.toml itself carries [scene] (mod scenes then lose).
    assemblyDefinesScene = configurationStore.hasSection(ConfigurationStore::sceneSectionName);

    if (!loadModuleDefaults(report))
    {
        return false;
    }
    std::vector<std::filesystem::path> profilePaths = configurationProfileFilePaths;
    if (profilePaths.empty() && !configurationProfileFilePath.empty())
        profilePaths.push_back(configurationProfileFilePath);
    for (const std::filesystem::path &configuredProfilePath : profilePaths)
    {
        const std::filesystem::path profilePath = detail::resolveAgainstExecutableDirectory(configuredProfilePath);
        if (!std::filesystem::exists(profilePath))
        {
            report += "- runtime configuration profile '" + profilePath.string() + "' does not exist.\n";
            return false;
        }
        std::string profileError;
        if (!configurationStore.mergeProfileFile(profilePath, profileError))
        {
            report += "- " + profileError + "\n";
            return false;
        }
    }
    configurationLoaded = true;
    return true;
}

// config/modules/<name>/config.toml, sorted for a deterministic merge
// order; merged as the fill-only ModuleDefaults layer.
bool EngineKernel::loadModuleDefaults(std::string &report)
{
    const std::vector<std::filesystem::path> defaultsFiles = detail::collectModuleDefaultsFiles();

    for (const std::filesystem::path &defaultsFile : defaultsFiles)
    {
        std::string defaultsError;
        if (!configurationStore.mergeDefaultsFile(defaultsFile,
                ConfigProvenance::Layer::ModuleDefaults,
                defaultsFile.parent_path().filename().string(),
                defaultsError))
        {
            report += "- " + defaultsError + "\n";
            return false;
        }
    }
    return true;
}

// Scene precedence: an explicit scene wins, then the single mod-claimed
// scene; multiple claims are ambiguous (error); mod scenes under an
// assembly scene become deferred warnings. Overrides apply last and the
// diagnostics switches take effect.
bool EngineKernel::finishConfiguration(std::string &report)
{
    if (!sceneFilePath.empty())
    {
        const std::filesystem::path resolvedScenePath = detail::resolveAgainstExecutableDirectory(sceneFilePath);
        if (!std::filesystem::exists(resolvedScenePath))
        {
            report += "- scene file '" + resolvedScenePath.string() + "' does not exist.\n";
            return false;
        }

        std::string sceneError;
        if (!configurationStore.applySceneFile(resolvedScenePath, sceneError))
        {
            report += "- " + sceneError + "\n";
            return false;
        }
        // Exactly one mod scene claim: adopt it as the world.
    }
    else if (!assemblyDefinesScene && gameModSceneClaims.size() == 1)
    {
        const GameModSceneClaim &claim = gameModSceneClaims.front();
        if (!std::filesystem::exists(claim.sceneFile))
        {
            report += "- mod '" + claim.ownerId + "' declares scene file '" + claim.sceneFile.string()
                    + "' which does not exist.\n";
            return false;
        }
        std::string sceneError;
        if (!configurationStore.applySceneFile(claim.sceneFile, sceneError))
        {
            report += "- " + sceneError + "\n";
            return false;
        }
        // Several mods claim scenes and no assembly scene: refuse rather than
        // pick one.
    }
    else if (!assemblyDefinesScene && gameModSceneClaims.size() > 1)
    {
        report += "- the world is ambiguous: " + std::to_string(gameModSceneClaims.size())
                + " active mods declare scene files";
        for (const GameModSceneClaim &claim : gameModSceneClaims)
        {
            report += " '" + claim.sceneFile.string() + "' (" + claim.ownerId + ")";
        }
        report += "; give the run a world: a [scene] section in the configuration, "
                  "or disable all but one of these mods.\n";
        return false;
    }
    else if (assemblyDefinesScene && !gameModSceneClaims.empty())
    {
        for (const GameModSceneClaim &claim : gameModSceneClaims)
        {
            deferredConfigurationWarnings += "warning: mod '" + claim.ownerId + "' declares scene file '"
                                           + claim.sceneFile.string()
                                           + "' but the configuration defines its own [scene]; the mod scene is "
                                             "not applied\n";
        }
    }
    std::string overridesError;
    if (!configurationStore.applyCommandLineOverrides(overridesError))
    {
        report += "- " + overridesError + "\n";
        return false;
    }

    // diagnostics.profiling_enabled (default on); the bus latency switch is
    // sticky-on (once enabled it stays enabled).
    profiler_.setEnabled(configurationStore
            .booleanValue(
                ConfigurationStore::diagnosticsSectionName, ConfigurationStore::diagnosticsProfilingEnabledKey)
            .value_or(true));

    bus.setLatencyRecordingEnabled(
        bus.latencyRecordingEnabled()
        || configurationStore
            .booleanValue(ConfigurationStore::diagnosticsSectionName, ConfigurationStore::diagnosticsLatencyReportKey)
            .value_or(false));
    return true;
}

// Overrides apply before gather so [engine].active_mods etc. are current;
// finishConfiguration re-applies them (staging is idempotent).
bool EngineKernel::prepareParticipants(std::string &report, bool enforceTrust)
{
    if (!loadConfiguration(report))
    {
        return false;
    }
    std::string overridesError;
    if (!configurationStore.applyCommandLineOverrides(overridesError))
    {
        report += "- " + overridesError + "\n";
        return false;
    }
    return gatherParticipants(report, enforceTrust) && finishConfiguration(report);
}

// Assembly keys + every participant's manifest keys; malformed keys are
// reported but do not abort.
bool EngineKernel::declareKnownKeyRegistry(std::string &report)
{
    configurationStore.declareAssemblyKnownKeys();
    for (const ParticipantRecord &record : records)
    {
        for (const std::string &configKey : record.manifest.knownConfigKeys)
        {
            if (!configurationStore.declareKnownConfigKey(configKey))
            {
                report += "- " + describeComponentKind(record.manifest.kind) + " '" + record.name
                        + "' declares the malformed configuration key '" + configKey
                        + "'; keys are '<section>.<dotted path>' of bare key segments.\n";
            }
        }
    }
    return report.empty();
}

// Two-pass reload: parse every input file first (a refusal leaves the
// live configuration untouched), then rebuild. Only live keys (non-
// [scene]) count as changed; scene changes need a restart.
bool EngineKernel::reloadConfiguration(std::string &report)
{
    report.clear();
    if (!constructed)
    {
        report = "engine reload refused:\n- reload requires a completed construct phase; before construct the "
                 "configuration is assembled for the first time, not re-assembled\n";
        return false;
    }

    const std::filesystem::path resolvedConfigurationPath
        = detail::resolveAgainstExecutableDirectory(configurationFilePath);
    std::string parseError;
    if (!ConfigurationStore::fileParses(resolvedConfigurationPath, parseError))
    {
        report = "engine reload refused (the running configuration is untouched):\n- " + parseError + "\n";
        return false;
    }
    std::vector<std::filesystem::path> profilePaths = configurationProfileFilePaths;
    if (profilePaths.empty() && !configurationProfileFilePath.empty())
        profilePaths.push_back(configurationProfileFilePath);
    for (const std::filesystem::path &configuredProfilePath : profilePaths)
    {
        const std::filesystem::path profilePath = detail::resolveAgainstExecutableDirectory(configuredProfilePath);
        // Profiles are validated by merging into a throwaway store (stricter
        // than a plain parse: the section/key rules are enforced).
        ConfigurationStore profileValidation;
        if (!profileValidation.mergeProfileFile(profilePath, parseError))
        {
            report = "engine reload refused (the running configuration is untouched):\n- " + parseError + "\n";
            return false;
        }
    }
    for (const std::filesystem::path &defaultsFile : detail::collectModuleDefaultsFiles())
    {
        if (!ConfigurationStore::fileParses(defaultsFile, parseError))
        {
            report = "engine reload refused (the running configuration is untouched):\n- " + parseError + "\n";
            return false;
        }
    }
    for (const auto &[defaultsFile, source] : gameModDefaultsFiles)
    {
        (void)source;
        if (!ConfigurationStore::fileParses(defaultsFile, parseError))
        {
            report = "engine reload refused (the running configuration is untouched):\n- " + parseError + "\n";
            return false;
        }
    }

    std::filesystem::path reloadScenePath;
    if (!sceneFilePath.empty())
    {
        reloadScenePath = detail::resolveAgainstExecutableDirectory(sceneFilePath);
    }
    else if (!assemblyDefinesScene && gameModSceneClaims.size() == 1)
    {
        reloadScenePath = gameModSceneClaims.front().sceneFile;
    }
    if (!reloadScenePath.empty() && !ConfigurationStore::fileParses(reloadScenePath, parseError))
    {
        report = "engine reload refused (the running configuration is untouched):\n- " + parseError + "\n";
        return false;
    }

    // Flattened "type:value" text of every leaf before and after; the diff
    // drives the changed-keys report and the reload event.
    const std::map<std::string, std::string> previousValues = configurationStore.flattenedValueReprs();

    configurationStore.resetForReload();
    configurationLoaded = false;
    std::string reloadError;
    if (!loadConfiguration(reloadError))
    {
        report = "engine reload failed after the pre-parse pass (unexpected; the report names the file):\n- "
               + reloadError + "\n";
        return false;
    }
    if (!configurationStore.applyCommandLineOverrides(reloadError))
    {
        report = "engine reload failed after the pre-parse pass (unexpected):\n- " + reloadError + "\n";
        return false;
    }
    for (const auto &[defaultsFile, source] : gameModDefaultsFiles)
    {
        if (!configurationStore.mergeDefaultsFile(
                defaultsFile, ConfigProvenance::Layer::ModDefaults, source, reloadError))
        {
            report = "engine reload failed after the pre-parse pass (unexpected):\n- " + reloadError + "\n";
            return false;
        }
    }
    if (!finishConfiguration(reloadError))
    {
        report = "engine reload failed after the pre-parse pass (unexpected):\n- " + reloadError + "\n";
        return false;
    }

    const std::map<std::string, std::string> currentValues = configurationStore.flattenedValueReprs();

    // Denials may have changed; re-run the permission pass for the new
    // configuration.
    permissionSystem_.clearDenials();
    applyPermissionPass();

    std::vector<std::string> changedKeys;
    std::size_t              changedCount = 0;
    std::size_t              sceneChanges = 0;
    std::size_t              removedCount = 0;
    // [scene] changes are world content: reported but not applied live.
    const auto isScenePath = [](const std::string &path)
    {
        return path == ConfigurationStore::sceneSectionName
            || path.rfind(std::string(ConfigurationStore::sceneSectionName) + ".", 0) == 0;
    };
    for (const auto &[path, representation] : currentValues)
    {
        (void)representation;
        const auto previous = previousValues.find(path);
        if (previous != previousValues.end() && previous->second == representation)
        {
            continue;
        }
        ++changedCount;
        if (isScenePath(path))
        {
            ++sceneChanges;
            continue;
        }
        changedKeys.push_back(path);
    }
    for (const auto &[path, representation] : previousValues)
    {
        (void)representation;
        if (currentValues.find(path) == currentValues.end())
        {
            ++removedCount;
            if (!isScenePath(path))
            {
                changedKeys.push_back(path);
            }
        }
    }

    report += "engine configuration reloaded: " + std::to_string(changedCount) + " key(s) changed, "
            + std::to_string(removedCount) + " key(s) removed";
    if (sceneChanges > 0)
    {
        report += "; " + std::to_string(sceneChanges)
                + " world content ([scene]) change(s) require a restart and were not applied live";
    }
    report += "\n";
    if (changedKeys.empty())
    {
        report += "no live keys changed; nothing was published\n";
        return true;
    }
    report += "changed keys:";
    for (const std::string &key : changedKeys)
    {
        report += " " + key;
    }
    report += "\n";

    for (const std::string &warning : configurationStore.unknownKeyWarnings())
    {
        report += "warning: " + warning + "\n";
    }

    bus.publish(ConfigurationReloadedEvent{.changedKeys = std::move(changedKeys)});
    return true;
}

} // namespace engine
