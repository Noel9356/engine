// EngineKernel configuration pipeline: layered loading, scene resolution
// and live reload.
//
// loadConfiguration merges engine.toml + module defaults + profiles;
// finishConfiguration applies scene files (assembly or sole plugin claim)
// and staged overrides; reloadConfiguration re-runs the pipeline after a
// pre-parse validation, keeping the old configuration on any failure.

#include <engine/kernel/configuration.hpp>
#include <engine/kernel/engine_kernel.hpp>

#include <engine/kernel/participant_interface.hpp>
#include "engine_kernel_internal.hpp"
#include "kernel_internals.hpp"

#include <toml.hpp>

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
    if (!configurationStore_.loadFromFile(resolvedPath, loadError))
    {
        report += "- " + loadError + "\n";
        return false;
    }

    // True when engine.toml itself carries [scene] (plugin scenes then lose).
    assemblyDefinesScene = configurationStore_.hasSection(ConfigurationStore::sceneSectionName);

    if (!loadModuleDefaults(report))
    {
        return false;
    }
    const std::vector<std::filesystem::path> &profilePaths = configurationProfileFilePaths;
    for (const std::filesystem::path &configuredProfilePath : profilePaths)
    {
        const std::filesystem::path profilePath = detail::resolveAgainstExecutableDirectory(configuredProfilePath);
        if (!std::filesystem::exists(profilePath))
        {
            report += "- runtime configuration profile '" + profilePath.string() + "' does not exist.\n";
            return false;
        }
        std::string profileError;
        if (!configurationStore_.mergeProfileFile(profilePath, profileError))
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
        if (!configurationStore_.mergeDefaultsFile(defaultsFile,
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

// Effective scene file for the current assembly state: the explicit one,
// else the single plugin-claimed scene (multiple claims were rejected when
// the configuration finished). Shared by the finish and reload paths so the
// precedence rule lives in one place.
std::filesystem::path EngineKernel::effectiveSceneFilePath() const
{
    if (!sceneFilePath.empty())
    {
        return detail::resolveAgainstExecutableDirectory(sceneFilePath);
    }
    if (!assemblyDefinesScene && pluginSceneClaims.size() == 1)
    {
        return pluginSceneClaims.front().sceneFile;
    }
    return {};
}

// Scene precedence: an explicit scene wins, then the single plugin-claimed
// scene; multiple claims are ambiguous (error); plugin scenes under an
// assembly scene become deferred warnings. Overrides apply last and the
// diagnostics switches take effect.
bool EngineKernel::finishConfiguration(std::string &report)
{
    if (!sceneFilePath.empty())
    {
        const std::filesystem::path resolvedScenePath = effectiveSceneFilePath();
        if (!std::filesystem::exists(resolvedScenePath))
        {
            report += "- scene file '" + resolvedScenePath.string() + "' does not exist.\n";
            return false;
        }

        std::string sceneError;
        if (!configurationStore_.applySceneFile(resolvedScenePath, sceneError))
        {
            report += "- " + sceneError + "\n";
            return false;
        }
        // Exactly one plugin scene claim: adopt it as the world.
    }
    else if (!assemblyDefinesScene && pluginSceneClaims.size() == 1)
    {
        const PluginSceneClaim &claim = pluginSceneClaims.front();
        if (!std::filesystem::exists(claim.sceneFile))
        {
            report += "- plugin '" + claim.ownerId + "' declares scene file '" + claim.sceneFile.string()
                    + "' which does not exist.\n";
            return false;
        }
        std::string sceneError;
        if (!configurationStore_.applySceneFile(claim.sceneFile, sceneError))
        {
            report += "- " + sceneError + "\n";
            return false;
        }
        // Several plugins claim scenes and no assembly scene: refuse rather than
        // pick one.
    }
    else if (!assemblyDefinesScene && pluginSceneClaims.size() > 1)
    {
        report += "- the world is ambiguous: " + std::to_string(pluginSceneClaims.size())
                + " active plugins declare scene files";
        for (const PluginSceneClaim &claim : pluginSceneClaims)
        {
            report += " '" + claim.sceneFile.string() + "' (" + claim.ownerId + ")";
        }
        report += "; give the run a world: a [scene] section in the configuration, "
                  "or disable all but one of these plugins.\n";
        return false;
    }
    else if (assemblyDefinesScene && !pluginSceneClaims.empty())
    {
        for (const PluginSceneClaim &claim : pluginSceneClaims)
        {
            deferredConfigurationWarnings += "warning: plugin '" + claim.ownerId + "' declares scene file '"
                                           + claim.sceneFile.string()
                                           + "' but the configuration defines its own [scene]; the plugin scene is "
                                             "not applied\n";
        }
    }
    std::string overridesError;
    if (!configurationStore_.applyCommandLineOverrides(overridesError))
    {
        report += "- " + overridesError + "\n";
        return false;
    }

    // diagnostics.profiling_enabled (default on); the bus_ latency switch is
    // sticky-on (once enabled it stays enabled).
    profiler_.setEnabled(configurationStore_
            .booleanValue(
                ConfigurationStore::diagnosticsSectionName, ConfigurationStore::diagnosticsProfilingEnabledKey)
            .value_or(true));

    bus_.setLatencyRecordingEnabled(
        bus_.latencyRecordingEnabled()
        || configurationStore_
            .booleanValue(ConfigurationStore::diagnosticsSectionName, ConfigurationStore::diagnosticsLatencyReportKey)
            .value_or(false));
    return true;
}

// Overrides apply before gather so [engine].active_plugins etc. are current;
// finishConfiguration re-applies them (staging is idempotent).
bool EngineKernel::prepareParticipants(std::string &report, bool enforceTrust)
{
    if (!loadConfiguration(report))
    {
        return false;
    }
    std::string overridesError;
    if (!configurationStore_.applyCommandLineOverrides(overridesError))
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
    configurationStore_.declareAssemblyKnownKeys();
    for (const ParticipantRecord &record : records_)
    {
        for (const std::string &configKey : record.manifest.knownConfigKeys)
        {
            if (!configurationStore_.declareKnownConfigKey(configKey))
            {
                // Malformed keys are reported but do not abort (see the
                // comment above): the participant still runs, its own
                // section reads simply never match the malformed key, and
                // the note joins the run's warning stream.
                report += "warning: " + describeComponentKind(record.manifest.kind) + " '" + record.name
                        + "' declares the malformed configuration key '" + configKey
                        + "'; keys are '<section>.<dotted path>' of bare key segments.\n";
            }
        }
    }
    return true;
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
    const std::vector<std::filesystem::path> &profilePaths = configurationProfileFilePaths;
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
    for (const auto &[defaultsFile, source] : pluginDefaultsFiles)
    {
        (void)source;
        if (!ConfigurationStore::fileParses(defaultsFile, parseError))
        {
            report = "engine reload refused (the running configuration is untouched):\n- " + parseError + "\n";
            return false;
        }
    }

    const std::filesystem::path reloadScenePath = effectiveSceneFilePath();
    if (!reloadScenePath.empty() && !ConfigurationStore::fileParses(reloadScenePath, parseError))
    {
        report = "engine reload refused (the running configuration is untouched):\n- " + parseError + "\n";
        return false;
    }

    // Flattened "type:value" text of every leaf before and after; the diff
    // drives the changed-keys report and the reload event. The live [scene]
    // section is snapshotted here and reinstated after the reload: world
    // content is restart-only by contract, so the reloaded store must keep
    // serving the previous world even when the scene file changed on disk.
    const std::map<std::string, std::string> previousValues = configurationStore_.flattenedValueReprs();
    toml::table                             previousScene  = configurationStore_.takeSceneSection();

    configurationStore_.resetForReload();
    configurationLoaded = false;
    std::string reloadError;
    if (!loadConfiguration(reloadError) || !configurationStore_.applyCommandLineOverrides(reloadError))
    {
        // The pre-parse pass validated every file, so a failure here means a
        // file changed (or was removed) between the pre-parse and the reload.
        // Best-effort restore: rebuild the previous configuration from the
        // same sources so the engine keeps running with a usable store
        // instead of an empty one.
        std::string restoreReport;
        configurationStore_.resetForReload();
        const bool restored = loadConfiguration(restoreReport)
                           && configurationStore_.applyCommandLineOverrides(restoreReport);
        for (const auto &[defaultsFile, source] : pluginDefaultsFiles)
        {
            if (restored
                && !configurationStore_.mergeDefaultsFile(
                    defaultsFile, ConfigProvenance::Layer::PluginDefaults, source, restoreReport))
            {
                break;
            }
        }
        if (restored)
        {
            std::string finishError;
            if (finishConfiguration(finishError))
            {
                // The scene snapshot still holds the live world; put it back
                // so even the restored store keeps serving it.
                configurationStore_.restoreSceneSection(std::move(previousScene));
                configurationLoaded = true;
                report = "engine reload refused (the running configuration is restored):\n- " + reloadError + "\n";
                return false;
            }
        }
        // Nothing to restore into a broken store; the snapshot dies with the
        // reload attempt, matching the previous behavior for its other keys.
        previousScene = toml::table{};
        // The store is not reliably loaded here: keep configurationLoaded
        // false so the next loadConfiguration/validate/construct rebuilds it
        // from the source files instead of serving a partial document for
        // the rest of the run.
        configurationLoaded = false;
        report = "engine reload failed and the previous configuration could not be fully restored:\n- reload error: "
               + reloadError + "\n- restore status: " + (restored ? "partial" : "failed") + "\n";
        return false;
    }
    for (const auto &[defaultsFile, source] : pluginDefaultsFiles)
    {
        if (!configurationStore_.mergeDefaultsFile(
                defaultsFile, ConfigProvenance::Layer::PluginDefaults, source, reloadError))
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

    const std::map<std::string, std::string> currentValues = configurationStore_.flattenedValueReprs();

    // Denials may have changed; re-run the permission pass for the new
    // configuration. The pass returns the (participant, permission) pairs the
    // new [permissions].denied list denies - startup and validate report the
    // same pairs, so a reload that silently dropped them left the operator
    // with no clue which participants now run with denied permissions.
    permissionSystem_.clearDenials();
    for (const auto &[deniedRecord, permission] : applyPermissionPass())
    {
        report += "- " + describeComponentKind(deniedRecord->manifest.kind) + " '" + deniedRecord->name
                + "' now requires permission '" + permission
                + "' which the configuration denies; the denial applies to the running participants.\n";
    }

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

    // Reinstating the snapshot keeps live reads of [scene] on the previous
    // world (the diff above already counted what the new file would change);
    // the reload report tells the operator those changes wait for a restart.
    configurationStore_.restoreSceneSection(std::move(previousScene));

    report += "engine configuration reloaded: " + std::to_string(changedCount) + " key(s) changed, "
            + std::to_string(removedCount) + " key(s) removed";
    if (sceneChanges > 0)
    {
        report += "; " + std::to_string(sceneChanges)
                + " world content ([scene]) change(s) were detected and held back; they take effect after a restart";
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

    for (const std::string &warning : configurationStore_.unknownKeyWarnings())
    {
        report += "warning: " + warning + "\n";
    }

    bus_.publish(ConfigurationReloadedEvent{.changedKeys = std::move(changedKeys)});
    return true;
}

} // namespace engine
