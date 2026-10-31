#include <engine/kernel/engine_kernel.hpp>

#include <engine/kernel/capability_name.hpp>
#include <engine/kernel/filesystem_utilities.hpp>
#include <engine/kernel/mod_api.hpp>
#include <engine/kernel/mod_discovery.hpp>
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/module_manifest.hpp>
#include <engine/kernel/module_plugin_cache.hpp>
#include <engine/kernel/sha256.hpp>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>

namespace engine
{

namespace
{

const std::string &appContextName()
{
    static const std::string name = "<app>";
    return name;
}

constexpr std::size_t broadCapabilityWarningThreshold = 5;

std::filesystem::path resolveAgainstExecutableDirectory(const std::filesystem::path &path)
{
    if (path.is_absolute())
    {
        return path;
    }
    return executableDirectory() / path;
}

// " (version 1.2.3, sha256 <hex>)" pieces for report lines, omitting what is
// unknown.
std::string advisoryIdentitySuffix(const ComponentTrustAdvisory &advisory)
{
    std::string suffix;
    if (!advisory.version.empty())
    {
        suffix += " (version " + advisory.version;
        if (!advisory.sha256.empty())
        {
            suffix += ", sha256 " + advisory.sha256;
        }
        suffix += ")";
    } else if (!advisory.sha256.empty())
    {
        suffix += " (sha256 " + advisory.sha256 + ")";
    }
    return suffix;
}

bool trustBindingMatches(const TrustBinding &binding, const ComponentTrustAdvisory &advisory)
{
    if (binding.name != advisory.name)
    {
        return false;
    }
    if (!binding.version.empty() && binding.version != advisory.version)
    {
        return false;
    }
    // A checksum-pinned binding never matches content without a digest: an
    // unhashable component must not inherit content-bound trust.
    if (!binding.sha256.empty()
        && (advisory.sha256.empty() || !sha256HexDigestsEqual(binding.sha256, advisory.sha256)))
    {
        return false;
    }
    return true;
}

bool anyTrustBindingMatches(
    const std::vector<TrustBinding>    &bindings,
    const ComponentTrustAdvisory       &advisory,
    std::vector<std::string>           &warnings
)
{
    for (const TrustBinding &binding : bindings)
    {
        if (!binding.sha256.empty() && !isValidSha256HexDigest(binding.sha256))
        {
            warnings.push_back("the [trust] entry for '" + binding.name
                             + "' carries a malformed sha256 (must be 64 hexadecimal characters); it is treated "
                               "as name-only trust for this run");
            continue;
        }
        if (trustBindingMatches(binding, advisory))
        {
            return true;
        }
    }
    return false;
}

// Content hash of a native mod: the digest of its entry library, before any
// of its code is loaded. Empty when no library file exists (the load would
// fail later anyway).
std::string nativeModContentHash(const ModManifest &modManifest)
{
    for (const std::filesystem::path &candidate :
         nativeModLibraryCandidates(modManifest.directory, modManifest.entry))
    {
        std::error_code statError;
        if (!std::filesystem::is_regular_file(candidate, statError) || statError)
        {
            continue;
        }
        std::string hashError;
        const std::string digest = Sha256::fileHexDigest(candidate, hashError);
        if (!digest.empty())
        {
            return digest;
        }
    }
    return {};
}

// Content hash of a script mod: the manifest plus every .lua file in the
// package, ordered by relative path, each prefixed with its path and size so
// the digest cannot be rearranged. Scripts are data until the host module
// runs them, so this stays inside the no-code-before-the-gate invariant.
std::string scriptModContentHash(const ModManifest &modManifest)
{
    struct HashedFile
    {
        std::string           relativePath;
        std::filesystem::path path;
    };

    std::vector<HashedFile> files;
    std::error_code         scanError;
    std::filesystem::recursive_directory_iterator iterator(
        modManifest.directory,
        std::filesystem::directory_options::skip_permission_denied,
        scanError
    );
    if (!scanError)
    {
        for (const std::filesystem::directory_entry &entry : iterator)
        {
            std::error_code statError;
            if (statError || !entry.is_regular_file(statError) || statError)
            {
                continue;
            }
            const std::string extension = entry.path().extension().string();
            if (extension != ".lua")
            {
                continue;
            }
            std::string relative = std::filesystem::relative(entry.path(), modManifest.directory, statError).string();
            if (statError)
            {
                relative = entry.path().filename().string();
            }
            files.push_back(HashedFile{std::move(relative), entry.path()});
        }
    }
    std::sort(files.begin(), files.end(), [](const HashedFile &left, const HashedFile &right) {
        return left.relativePath < right.relativePath;
    });

    Sha256     hasher;
    const auto feed = [&hasher](const void *data, std::size_t size) {
        hasher.update(data, size);
    };
    const auto feedText = [&feed](const std::string &text) {
        std::uint64_t size = static_cast<std::uint64_t>(text.size());
        feed(&size, sizeof(size));
        feed(text.data(), text.size());
    };

    feedText("engine-script-mod-hash-v1");
    feedText("mod.toml");
    {
        std::ifstream manifestStream(modManifest.directory / ModManifestLoader::manifestFileName, std::ios::in | std::ios::binary);
        if (manifestStream.is_open())
        {
            std::string manifestBytes((std::istreambuf_iterator<char>(manifestStream)), std::istreambuf_iterator<char>());
            feedText(manifestBytes);
        }
    }
    for (const HashedFile &file : files)
    {
        feedText(file.relativePath);
        std::ifstream scriptStream(file.path, std::ios::in | std::ios::binary);
        if (!scriptStream.is_open())
        {
            return {};
        }
        std::string scriptBytes((std::istreambuf_iterator<char>(scriptStream)), std::istreambuf_iterator<char>());
        feedText(scriptBytes);
    }

    std::uint8_t digest[Sha256::digestSize];
    hasher.finish(digest);
    std::string hex;
    hex.reserve(Sha256::hexDigestSize);
    static const char digits[] = "0123456789abcdef";
    for (const std::uint8_t byte : digest)
    {
        hex.push_back(digits[byte >> 4]);
        hex.push_back(digits[byte & 0x0F]);
    }
    return hex;
}

std::string describeComponentKind(ComponentKind kind)
{
    switch (kind)
    {
    case ComponentKind::Module:
        return "module";
    case ComponentKind::SharedLibraryMod:
        return "shared library mod";
    case ComponentKind::ScriptMod:
        return "script mod";
    case ComponentKind::Application:
        return "application";
    }
    return "component";
}

std::vector<std::string> effectivePermissions(const ComponentManifest &manifest)
{
    std::set<std::string> unique(manifest.requiredPermissions.begin(), manifest.requiredPermissions.end());
    for (const std::string &capabilityName : manifest.requiredCapabilities)
    {
        unique.insert(std::string(capabilityPermissionPrefix) + capabilityName);
    }
    for (const std::string &capabilityName : manifest.optionalCapabilities)
    {
        unique.insert(std::string(capabilityPermissionPrefix) + capabilityName);
    }
    return std::vector<std::string>(unique.begin(), unique.end());
}

bool sameStringSet(
    const std::vector<std::string> &left,
    const std::vector<std::string> &right
)
{
    const std::set<std::string> leftSet(left.begin(), left.end());
    const std::set<std::string> rightSet(right.begin(), right.end());
    return leftSet == rightSet;
}

std::vector<std::filesystem::path> collectModuleDefaultsFiles()
{
    std::vector<std::filesystem::path> moduleDirectories;
    const std::filesystem::path        defaultsDirectory
        = resolveAgainstExecutableDirectory(ConfigurationStore::moduleDefaultsDirectory);
    if (!std::filesystem::exists(defaultsDirectory) || !std::filesystem::is_directory(defaultsDirectory))
    {
        return moduleDirectories;
    }
    for (const std::filesystem::directory_entry &entry : std::filesystem::directory_iterator(defaultsDirectory))
    {
        if (entry.is_directory() && std::filesystem::exists(entry.path() / ConfigurationStore::moduleDefaultsFileName))
        {
            moduleDirectories.push_back(entry.path());
        }
    }
    std::sort(moduleDirectories.begin(), moduleDirectories.end());
    std::vector<std::filesystem::path> defaultsFiles;
    defaultsFiles.reserve(moduleDirectories.size());
    for (const std::filesystem::path &moduleDirectory : moduleDirectories)
    {
        defaultsFiles.push_back(moduleDirectory / ConfigurationStore::moduleDefaultsFileName);
    }
    return defaultsFiles;
}

} // namespace

void EngineKernel::setConfigurationFile(const std::filesystem::path &configurationPath)
{
    configurationFilePath = configurationPath;
    configurationLoaded   = false;
    configurationStore.reset();

    componentsGathered = false;
}

void EngineKernel::setSceneFile(const std::filesystem::path &scenePath)
{
    sceneFilePath       = scenePath;
    configurationLoaded = false;
    configurationStore.reset();
    componentsGathered = false;
}

bool EngineKernel::addCommandLineOverride(
    const std::string &assignment,
    std::string       &error
)
{
    return configurationStore.stageCommandLineOverride(assignment, error);
}

void EngineKernel::setModsDirectory(const std::filesystem::path &modsDirectory)
{
    explicitModsDirectory = modsDirectory;
    explicitExtraModsDirectories.clear();
    componentsGathered = false;
}

void EngineKernel::addModsDirectory(const std::filesystem::path &modsDirectory)
{
    explicitExtraModsDirectories.push_back(modsDirectory);
    componentsGathered = false;
}

void EngineKernel::setApplicationRequirements(const std::vector<std::string> &capabilities)
{
    applicationRequiredCapabilities = capabilities;
    componentsGathered              = false;
}

void EngineKernel::setActiveMods(const std::vector<std::string> &modIds)
{
    applicationActiveMods = modIds;
    componentsGathered    = false;
}

void EngineKernel::clearActiveMods()
{
    applicationActiveMods.reset();
    componentsGathered = false;
}

EngineKernel::~EngineKernel()
{
    stop();
}

std::unique_ptr<ComponentContext> EngineKernel::createAppContext()
{
    permissionSystem_.grantAllPermissions(appContextName());
    return std::make_unique<ComponentContext>(
        bus,
        *this,
        permissionSystem_,
        configurationStore,
        appContextName(),
        ComponentKind::Application,
        nullptr,
        mainThreadScheduler_
    );
}

void EngineKernel::setTrustConsentHandler(TrustConsentHandler handler)
{
    trustConsentHandler_ = std::move(handler);
}

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
    const std::filesystem::path resolvedPath = resolveAgainstExecutableDirectory(configurationFilePath);
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

    assemblyDefinesScene = configurationStore.hasSection(ConfigurationStore::sceneSectionName);

    if (!loadModuleDefaults(report))
    {
        return false;
    }
    configurationLoaded = true;
    return true;
}

bool EngineKernel::loadModuleDefaults(std::string &report)
{
    const std::vector<std::filesystem::path> defaultsFiles = collectModuleDefaultsFiles();

    for (const std::filesystem::path &defaultsFile : defaultsFiles)
    {
        std::string defaultsError;
        if (!configurationStore.mergeDefaultsFile(
                defaultsFile,
                ConfigProvenance::Layer::ModuleDefaults,
                defaultsFile.parent_path().filename().string(),
                defaultsError
            ))
        {
            report += "- " + defaultsError + "\n";
            return false;
        }
    }
    return true;
}

bool EngineKernel::finishConfiguration(std::string &report)
{
    if (!sceneFilePath.empty())
    {
        const std::filesystem::path resolvedScenePath = resolveAgainstExecutableDirectory(sceneFilePath);
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
    } else if (!assemblyDefinesScene && modSceneClaims.size() == 1)
    {
        const ModSceneClaim &claim = modSceneClaims.front();
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
    } else if (!assemblyDefinesScene && modSceneClaims.size() > 1)
    {
        report += "- the world is ambiguous: " + std::to_string(modSceneClaims.size())
                + " active mods declare scene files";
        for (const ModSceneClaim &claim : modSceneClaims)
        {
            report += " '" + claim.sceneFile.string() + "' (" + claim.ownerId + ")";
        }
        report += "; give the run a world: a [scene] section in the configuration, "
                  "or disable all but one of these mods.\n";
        return false;
    } else if (assemblyDefinesScene && !modSceneClaims.empty())
    {
        for (const ModSceneClaim &claim : modSceneClaims)
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

    bus.setLatencyRecordingEnabled(
        bus.latencyRecordingEnabled()
        || configurationStore
            .booleanValue(ConfigurationStore::diagnosticsSectionName, ConfigurationStore::diagnosticsLatencyReportKey)
            .value_or(false)
    );
    return true;
}

bool EngineKernel::prepareComponents(
    std::string &report,
    bool         enforceTrust
)
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
    return gatherComponents(report, enforceTrust) && finishConfiguration(report);
}

std::vector<std::pair<
    const EngineKernel::ComponentRecord *,
    std::string>>
EngineKernel::applyPermissionPass()
{
    std::vector<std::pair<const ComponentRecord *, std::string>> denied;
    for (const auto &denial : configurationStore.permissionDenials())
    {
        permissionSystem_.denyPermission(denial.first, denial.second);
    }
    for (const ComponentRecord &record : records)
    {
        const std::vector<std::string> grants = effectivePermissions(record.manifest);
        permissionSystem_.declarePermissions(record.name, grants);
        const std::vector<std::string> missing = permissionSystem_.missingPermissions(record.name, grants);
        for (const std::string &permission : missing)
        {
            denied.emplace_back(&record, permission);
        }
    }
    return denied;
}

std::vector<ComponentManifest> EngineKernel::componentManifests() const
{
    std::vector<ComponentManifest> manifests;
    manifests.reserve(records.size());
    for (const ComponentRecord &record : records)
    {
        manifests.push_back(record.manifest);
    }
    return manifests;
}

EngineKernel::ComponentRecord *EngineKernel::findRecord(const std::string &name)
{
    for (ComponentRecord &candidate : records)
    {
        if (candidate.name == name)
        {
            return &candidate;
        }
    }
    return nullptr;
}

bool EngineKernel::gatherComponents(
    std::string &report,
    bool         enforceTrust
)
{
    if (componentsGathered)
    {
        return true;
    }

    if (!records.empty() || !modulePlugins.empty())
    {
        stop();
    }

    if (!applicationRequiredCapabilities.empty())
    {
        ComponentRecord applicationRecord;
        applicationRecord.name                          = appContextName();
        applicationRecord.manifest.kind                 = ComponentKind::Application;
        applicationRecord.manifest.name                 = appContextName();
        applicationRecord.manifest.requiredCapabilities = applicationRequiredCapabilities;
        records.push_back(std::move(applicationRecord));
        permissionSystem_.grantAllPermissions(appContextName());
    }

    trustAdvisories_.clear();

    loadModulePlugins(startupWarnings_);

    gatherModPackages(startupWarnings_);

    resolveUndescribedPlugins(report, enforceTrust);

    computeDemandClosure();

    for (const ModulePluginRecord &plugin : modulePlugins)
    {
        if (plugin.active)
        {
            appendModuleRiskWarning(plugin);
        }
    }

    if (!evaluateTrustGate(report, enforceTrust))
    {
        return false;
    }

    if (enforceTrust)
    {
        loadNativeModLibraries(startupWarnings_);
        nativeModLibrariesLoaded_ = true;
        if (!instantiateActiveModules(report))
        {
            return false;
        }
    } else
    {
        // validate() is a dry run: it must never execute plugin code. Declare
        // active modules from their descriptors so the dependency graph can
        // be resolved; the real instantiation happens during construct.
        appendDeclaredModuleRecords();
    }

    for (std::size_t outer = 0; outer < records.size(); ++outer)
    {
        for (std::size_t inner = outer + 1; inner < records.size(); ++inner)
        {
            if (records[outer].name == records[inner].name)
            {
                report += "- component name '" + records[outer].name
                        + "' is declared twice (a mod id and a module plugin name, or two mods sharing an id); "
                          "component names must be unique across the whole assembly.\n";
                return false;
            }
        }
    }

    componentsGathered = true;
    return true;
}

void EngineKernel::gatherModPackages(std::string &report)
{
    modSceneClaims.clear();
    modDefaultsFiles.clear();
    pendingNativeModPackages_.clear();
    nativeModLibrariesLoaded_ = false;

    std::vector<std::filesystem::path> scanRoots;
    if (!explicitModsDirectory.empty() || !explicitExtraModsDirectories.empty())
    {
        if (!explicitModsDirectory.empty())
        {
            scanRoots.push_back(resolveAgainstExecutableDirectory(explicitModsDirectory));
        }
        for (const std::filesystem::path &extraDirectory : explicitExtraModsDirectories)
        {
            scanRoots.push_back(resolveAgainstExecutableDirectory(extraDirectory));
        }
    } else
    {
        scanRoots.push_back(resolveAgainstExecutableDirectory(configurationStore.modDirectory()));
        for (const std::string &extraDirectory : configurationStore.extraModDirectories())
        {
            scanRoots.push_back(resolveAgainstExecutableDirectory(extraDirectory));
        }
    }

    std::map<std::string, std::filesystem::path> discoveredPackageRoots;
    std::vector<DiscoveredModPackage>            packages;
    for (const std::filesystem::path &scanRoot : scanRoots)
    {
        for (DiscoveredModPackage &package : discoverModPackages(scanRoot, configurationStore.disabledMods(), report))
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

    // The app declares the active mod set (API first, then the
    // configuration's engine.active_mods). Mods never activate on their own
    // initiative: an undeclared mod is reported and skipped, not an error.
    std::optional<std::vector<std::string>> activeSet = applicationActiveMods;
    if (!activeSet)
    {
        activeSet = configurationStore.activeMods();
    }
    if (activeSet)
    {
        std::vector<DiscoveredModPackage> activated;
        for (DiscoveredModPackage &package : packages)
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

    for (const DiscoveredModPackage &package : packages)
    {
        const ModManifest &modManifest = package.manifest;

        if (!package.sceneFile.empty())
        {
            ModSceneClaim claim;
            claim.sceneFile = package.sceneFile;
            claim.ownerId   = modManifest.id;
            modSceneClaims.push_back(std::move(claim));
        }

        if (!package.defaultsFile.empty())
        {
            std::string defaultsError;
            if (!configurationStore.mergeDefaultsFile(
                    package.defaultsFile,
                    ConfigProvenance::Layer::ModDefaults,
                    modManifest.directory.filename().string(),
                    defaultsError
                ))
            {
                report += "- " + defaultsError + "\n";
                continue;
            }

            modDefaultsFiles.emplace_back(package.defaultsFile, modManifest.directory.filename().string());
        }

        if (modManifest.isNative())
        {
            ComponentRecord record;
            record.name                          = modManifest.id;
            record.manifest.kind                 = ComponentKind::SharedLibraryMod;
            record.manifest.name                 = modManifest.id;
            record.manifest.version              = modManifest.version;
            record.manifest.providedCapabilities = modManifest.providedCapabilities;
            record.manifest.requiredCapabilities = modManifest.capabilities;
            record.manifest.optionalCapabilities = modManifest.optionalCapabilities;
            record.manifest.requiredPermissions  = modManifest.permissions;
            record.manifest.requiredComponents   = modManifest.requiresComponents;
            record.manifest.knownConfigKeys      = modManifest.configKeys;
            appendModRiskWarning(modManifest, true);
            pendingNativeModPackages_.push_back(package);
            records.push_back(std::move(record));
        } else
        {
            ComponentRecord record;
            record.name                          = modManifest.id;
            record.manifest.kind                 = ComponentKind::ScriptMod;
            record.manifest.name                 = modManifest.id;
            record.manifest.version              = modManifest.version;
            record.manifest.requiredCapabilities = modManifest.capabilities;
            record.manifest.optionalCapabilities = modManifest.optionalCapabilities;
            record.manifest.requiredPermissions  = modManifest.permissions;
            record.manifest.requiredComponents   = modManifest.requiresComponents;
            record.manifest.knownConfigKeys      = modManifest.configKeys;
            record.scriptMod.modId                  = modManifest.id;
            record.scriptMod.scriptPath             = modManifest.directory / modManifest.entry;
            record.scriptMod.hostCapability         = modManifest.hostCapability;
            record.scriptMod.declaredCapabilities   = modManifest.capabilities;
            record.scriptMod.declaredPermissions    = modManifest.permissions;
            record.scriptMod.version                = modManifest.version;
            appendModRiskWarning(modManifest, false);
            records.push_back(std::move(record));
        }
    }
}

void EngineKernel::computeDemandClosure()
{
    demandedCapabilities.clear();
    for (ModulePluginRecord &plugin : modulePlugins)
    {
        plugin.active = false;
    }

    const std::vector<DependencyResolver::ProviderSelection> selections = configurationStore.providerSelections();

    std::vector<std::string> worklist;
    for (const ComponentRecord &record : records)
    {
        for (const std::string &capabilityName : record.manifest.requiredCapabilities)
        {
            worklist.push_back(capabilityName);
        }
    }

    while (!worklist.empty())
    {
        const std::string capabilityName = std::move(worklist.back());
        worklist.pop_back();
        if (demandedCapabilities.find(capabilityName) != demandedCapabilities.end())
        {
            continue;
        }
        demandedCapabilities.insert(capabilityName);

        const auto providesCapability = [&capabilityName](const std::vector<std::string> &provided) {
            return std::any_of(provided.begin(), provided.end(), [&capabilityName](const std::string &candidate) {
                return capabilityRequirementSatisfiedBy(capabilityName, candidate);
            });
        };
        std::vector<std::string> providerNames;
        for (ModulePluginRecord &plugin : modulePlugins)
        {
            if (providesCapability(plugin.providedCapabilities))
            {
                providerNames.push_back(plugin.name);
            }
        }
        for (const ComponentRecord &record : records)
        {
            if (providesCapability(record.manifest.providedCapabilities))
            {
                providerNames.push_back(record.name);
            }
        }
        if (providerNames.size() > 1)
        {
            const auto selection = std::find_if(
                selections.begin(),
                selections.end(),
                [&capabilityName](const DependencyResolver::ProviderSelection &candidate) {
                    return candidate.capabilityName == capabilityName;
                }
            );
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
            for (ModulePluginRecord &plugin : modulePlugins)
            {
                if (plugin.name != providerName || plugin.active)
                {
                    continue;
                }
                plugin.active = true;

                for (const std::string &providedName : plugin.providedCapabilities)
                {
                    if (capabilityRequirementSatisfiedBy(capabilityName, providedName))
                    {
                        demandedCapabilities.insert(providedName);
                    }
                }
                for (const std::string &requiredName : plugin.requiredCapabilities)
                {
                    worklist.push_back(requiredName);
                }
            }
        }
    }
}

void EngineKernel::loadModulePlugins(std::string &report)
{
    modulePlugins.clear();
    undescribedPlugins_.clear();

    std::filesystem::path       modulesDirectory = configurationStore.modulesDirectory();
    const std::filesystem::path resolvedDirectory
        = modulesDirectory.is_absolute() ? modulesDirectory : executableDirectory() / modulesDirectory;

    std::error_code                     scanError;
    std::filesystem::directory_iterator plugin{
        resolvedDirectory, std::filesystem::directory_options::skip_permission_denied, scanError
    };
    if (scanError)
    {
        report += "- module plugins directory '" + resolvedDirectory.string()
                + "' cannot be scanned: " + scanError.message() + "\n";
        return;
    }

    // Candidate plugin libraries live in the modules directory or one of its
    // immediate subdirectories; each is remembered with its directory so a
    // sidecar module.toml can be matched to it.
    std::vector<std::pair<std::filesystem::path, std::filesystem::path>> pluginPaths;
    const auto collectLibraries
        = [&pluginPaths, &scanError](const std::filesystem::directory_iterator &iterator) {
              for (const std::filesystem::directory_entry &entry : iterator)
              {
                  std::error_code statError;
                  if (statError || !entry.is_regular_file())
                  {
                      continue;
                  }
                  const std::filesystem::path libraryPath = entry.path();
                  const std::string           extension   = libraryPath.extension().string();

                  if (extension == ".dll" || extension == ".so")
                  {
                      pluginPaths.emplace_back(libraryPath, libraryPath.parent_path());
                  }
              }
          };
    collectLibraries(plugin);
    for (const std::filesystem::directory_entry &entry : plugin)
    {
        std::error_code statError;
        if (statError || !entry.is_directory())
        {
            continue;
        }
        std::filesystem::directory_iterator nested{
            entry.path(), std::filesystem::directory_options::skip_permission_denied, statError
        };
        if (statError)
        {
            continue;
        }
        collectLibraries(nested);
    }

    std::sort(pluginPaths.begin(), pluginPaths.end());

    std::map<std::string, std::size_t> librariesPerDirectory;
    for (const auto &[libraryPath, directory] : pluginPaths)
    {
        ++librariesPerDirectory[directory.string()];
    }

    // A module.toml beside the library is a data descriptor: it lets the
    // kernel plan the plugin without executing any of its code. Parsed once
    // per directory.
    struct DirectorySidecar
    {
        bool          attempted = false;
        bool          present   = false;
        ModuleManifest manifest;
    };
    std::map<std::string, DirectorySidecar> sidecarsByDirectory;
    const auto sidecarFor = [&](const std::filesystem::path &directory) -> DirectorySidecar & {
        auto [iterator, inserted] = sidecarsByDirectory.try_emplace(directory.string());
        if (inserted)
        {
            iterator->second.attempted = true;
            const std::filesystem::path manifestPath = directory / ModuleManifestLoader::manifestFileName;
            std::error_code             existsError;
            if (std::filesystem::exists(manifestPath, existsError) && !existsError)
            {
                std::string parseError;
                if (ModuleManifestLoader{}.loadFromFile(manifestPath, iterator->second.manifest, parseError))
                {
                    iterator->second.present = true;
                } else
                {
                    report += "- " + parseError + "\n";
                }
            }
        }
        return iterator->second;
    };

    ModulePluginCache descriptorCache;
    descriptorCache.setCacheFile(executableDirectory() / modulePluginCacheFileName);
    descriptorCache.load();
    bool cacheChanged = false;

    for (const auto &[libraryPath, libraryDirectory] : pluginPaths)
    {
        std::error_code      statError;
        const std::uintmax_t sizeBytes = std::filesystem::file_size(libraryPath, statError);
        if (statError)
        {
            report += "- '" + libraryPath.string() + "' cannot be stat-ed: " + statError.message() + "\n";
            continue;
        }
        const std::uintmax_t modificationStamp = static_cast<std::uintmax_t>(
            std::filesystem::last_write_time(libraryPath, statError).time_since_epoch().count()
        );
        if (statError)
        {
            report += "- '" + libraryPath.string() + "' cannot be stat-ed: " + statError.message() + "\n";
            continue;
        }

        std::string hashError;
        const std::string contentSha256 = Sha256::fileHexDigest(libraryPath, hashError);
        if (contentSha256.empty())
        {
            report += "- '" + libraryPath.string() + "' cannot be hashed for the trust gate: " + hashError + "\n";
            continue;
        }

        ModulePluginRecord            record;
        const ModulePluginCacheEntry *remembered = descriptorCache.find(libraryPath, sizeBytes, modificationStamp);
        if (remembered != nullptr)
        {
            record.name                 = remembered->moduleName;
            record.version              = remembered->moduleVersion;
            record.contentSha256        = contentSha256;
            record.libraryPath          = libraryPath;
            record.providedCapabilities = remembered->providedCapabilities;
            record.requiredCapabilities = remembered->requiredCapabilities;
            record.optionalCapabilities = remembered->optionalCapabilities;
            record.declaredPermissions  = remembered->declaredPermissions;
        } else
        {
            // No cached descriptor: the only way to plan this plugin without
            // running its code is a sidecar module.toml. Without one, the
            // plugin stays undescribed and the trust gate decides whether the
            // kernel may execute it to extract a descriptor.
            const DirectorySidecar &sidecar = sidecarFor(libraryDirectory);
            bool                    describedBySidecar = false;
            if (sidecar.present)
            {
                const bool namesThisLibrary
                    = !sidecar.manifest.library.empty() && sidecar.manifest.library == libraryPath.filename().string();
                const bool coversDirectory = sidecar.manifest.library.empty()
                                          && librariesPerDirectory[libraryDirectory.string()] == 1;
                describedBySidecar = namesThisLibrary || coversDirectory;
            }
            if (!describedBySidecar)
            {
                UndescribedPlugin candidate;
                candidate.libraryPath       = libraryPath;
                candidate.sizeBytes         = sizeBytes;
                candidate.modificationStamp = modificationStamp;
                candidate.contentSha256     = contentSha256;
                undescribedPlugins_.push_back(std::move(candidate));
                continue;
            }

            record.name                 = sidecar.manifest.name;
            record.version              = sidecar.manifest.version;
            record.contentSha256        = contentSha256;
            record.libraryPath          = libraryPath;
            record.providedCapabilities = sidecar.manifest.providedCapabilities;
            record.requiredCapabilities = sidecar.manifest.requiredCapabilities;
            record.optionalCapabilities = sidecar.manifest.optionalCapabilities;
            record.declaredPermissions  = sidecar.manifest.declaredPermissions;

            ModulePluginCacheEntry cacheEntry;
            cacheEntry.sizeBytes            = sizeBytes;
            cacheEntry.modificationStamp    = modificationStamp;
            cacheEntry.moduleName           = record.name;
            cacheEntry.moduleVersion        = record.version;
            cacheEntry.contentSha256        = record.contentSha256;
            cacheEntry.providedCapabilities = record.providedCapabilities;
            cacheEntry.requiredCapabilities = record.requiredCapabilities;
            cacheEntry.optionalCapabilities = record.optionalCapabilities;
            cacheEntry.declaredPermissions  = record.declaredPermissions;
            descriptorCache.record(libraryPath, cacheEntry);
            cacheChanged = true;
        }

        if (std::find_if(
                modulePlugins.begin(),
                modulePlugins.end(),
                [&record](const ModulePluginRecord &existing) { return existing.name == record.name; }
            )
            != modulePlugins.end())
        {
            report += "- module '" + record.name + "' is provided by more than one plugin binary; '"
                    + libraryPath.string() + "' is ignored.\n";
            continue;
        }

        modulePlugins.push_back(std::move(record));
    }

    if (cacheChanged)
    {
        descriptorCache.save();
    }
}

void EngineKernel::resolveUndescribedPlugins(std::string &report, bool enforceTrust)
{
    if (undescribedPlugins_.empty())
    {
        return;
    }

    std::vector<ComponentTrustAdvisory> scanAdvisories;
    scanAdvisories.reserve(undescribedPlugins_.size());
    for (const UndescribedPlugin &candidate : undescribedPlugins_)
    {
        ComponentTrustAdvisory advisory;
        advisory.kind    = ComponentKind::Module;
        advisory.name    = candidate.libraryPath.filename().string();
        advisory.sha256  = candidate.contentSha256;
        advisory.reasons.push_back(
            "the kernel has no data descriptor for it (no module.toml beside the library and no cached "
            "descriptor); identifying this plugin requires executing its code"
        );
        scanAdvisories.push_back(std::move(advisory));
    }

    if (!enforceTrust)
    {
        for (const ComponentTrustAdvisory &advisory : scanAdvisories)
        {
            startupWarnings_ += "note: module plugin '" + advisory.name + advisoryIdentitySuffix(advisory)
                              + "' was not scanned: validate never executes plugin code and this plugin has no "
                                "data descriptor; construct will ask for a trust decision, or ship a module.toml "
                                "next to the library.\n";
        }
        undescribedPlugins_.clear();
        return;
    }

    const std::vector<TrustBinding> moduleBindings  = configurationStore.trustedModuleBindings();
    const bool                      trustAllModules = configurationStore.trustAllModules();

    std::vector<std::string> bindingWarnings;
    std::vector<std::size_t> approvedIndexes;
    std::vector<std::size_t> undecidedIndexes;
    for (std::size_t index = 0; index < undescribedPlugins_.size(); ++index)
    {
        if (trustAllModules || anyTrustBindingMatches(moduleBindings, scanAdvisories[index], bindingWarnings))
        {
            approvedIndexes.push_back(index);
        } else
        {
            undecidedIndexes.push_back(index);
        }
    }
    for (const std::string &warning : bindingWarnings)
    {
        startupWarnings_ += "warning: " + warning + "\n";
    }

    if (!undecidedIndexes.empty() && trustConsentHandler_ != nullptr)
    {
        std::vector<ComponentTrustAdvisory> untrusted;
        untrusted.reserve(undecidedIndexes.size());
        for (const std::size_t index : undecidedIndexes)
        {
            untrusted.push_back(scanAdvisories[index]);
        }
        if (trustConsentHandler_(untrusted))
        {
            std::string names;
            for (const ComponentTrustAdvisory &advisory : untrusted)
            {
                if (!names.empty())
                {
                    names += ", ";
                }
                names += "'" + advisory.name + "'";
            }
            startupWarnings_ += "agreement granted at startup: scanning " + names + " for this run.\n";
            approvedIndexes.insert(approvedIndexes.end(), undecidedIndexes.begin(), undecidedIndexes.end());
            undecidedIndexes.clear();
        } else
        {
            startupWarnings_ += "agreement was not granted at startup: no undescribed plugin was scanned.\n";
        }
    }

    ModulePluginCache descriptorCache;
    descriptorCache.setCacheFile(executableDirectory() / modulePluginCacheFileName);
    descriptorCache.load();
    bool cacheChanged = false;

    for (const std::size_t index : approvedIndexes)
    {
        ModulePluginRecord    record;
        ModulePluginCacheEntry cacheEntry;
        if (!scanUndescribedPluginDescriptor(undescribedPlugins_[index], record, cacheEntry, report))
        {
            continue;
        }
        if (std::find_if(
                modulePlugins.begin(),
                modulePlugins.end(),
                [&record](const ModulePluginRecord &existing) { return existing.name == record.name; }
            )
            != modulePlugins.end())
        {
            report += "- module '" + record.name + "' is provided by more than one plugin binary; '"
                    + record.libraryPath.string() + "' is ignored.\n";
            continue;
        }
        descriptorCache.record(record.libraryPath, cacheEntry);
        cacheChanged = true;
        modulePlugins.push_back(std::move(record));
    }
    for (const std::size_t index : undecidedIndexes)
    {
        const ComponentTrustAdvisory &advisory = scanAdvisories[index];
        startupWarnings_ += "note: module plugin '" + advisory.name + advisoryIdentitySuffix(advisory)
                          + "' was not scanned: identifying it would execute its code and no trust decision covers "
                            "it. Ship a module.toml beside the library, or trust it under [trust] (or agree at "
                            "startup); until then the plugin stays unloaded.\n";
    }
    if (cacheChanged)
    {
        descriptorCache.save();
    }
    undescribedPlugins_.clear();
}

bool EngineKernel::scanUndescribedPluginDescriptor(
    const UndescribedPlugin &candidate,
    ModulePluginRecord      &outRecord,
    ModulePluginCacheEntry  &outCacheEntry,
    std::string             &report
)
{
    // Scanning executes code inside the plugin binary; the caller has already
    // obtained the trust decision that authorizes this.
    outRecord                  = ModulePluginRecord{};
    outRecord.libraryPath      = candidate.libraryPath;
    outRecord.contentSha256    = candidate.contentSha256;

    auto        library = std::make_unique<DynamicLibrary>();
    std::string loadError;
    if (!library->load(candidate.libraryPath, loadError))
    {
        report += "- " + loadError + "\n";
        return false;
    }

    const auto query = reinterpret_cast<const EngineModulePluginEntry *(*)()>(
        library->resolveSymbol("engineQueryModulePlugin")
    );
    if (query == nullptr)
    {
        report += "- '" + candidate.libraryPath.string()
                + "' exports no engineQueryModulePlugin entry point; not a module plugin.\n";
        return false;
    }

    const EngineModulePluginEntry *pluginEntry = query();
    if (pluginEntry == nullptr || pluginEntry->abiVersion != engineModulePluginAbiVersion)
    {
        report += "- '" + candidate.libraryPath.string()
                + "' was built against a different module plugin ABI (expected version "
                + std::to_string(engineModulePluginAbiVersion) + "); rebuild the plugin with this kernel.\n";
        return false;
    }
    if (pluginEntry->moduleName == nullptr || pluginEntry->createModule == nullptr)
    {
        report += "- '" + candidate.libraryPath.string() + "' returns an incomplete plugin descriptor.\n";
        return false;
    }

    outRecord.name = pluginEntry->moduleName;
    if (pluginEntry->moduleVersion != nullptr)
    {
        outRecord.version = pluginEntry->moduleVersion;
    }
    outRecord.createModule = pluginEntry->createModule;
    for (std::uint32_t capabilityIndex = 0; capabilityIndex < pluginEntry->providedCapabilityCount; ++capabilityIndex)
    {
        if (pluginEntry->providedCapabilities != nullptr && pluginEntry->providedCapabilities[capabilityIndex] != nullptr)
        {
            outRecord.providedCapabilities.emplace_back(pluginEntry->providedCapabilities[capabilityIndex]);
        }
    }
    for (std::uint32_t capabilityIndex = 0; capabilityIndex < pluginEntry->requiredCapabilityCount; ++capabilityIndex)
    {
        if (pluginEntry->requiredCapabilities != nullptr && pluginEntry->requiredCapabilities[capabilityIndex] != nullptr)
        {
            outRecord.requiredCapabilities.emplace_back(pluginEntry->requiredCapabilities[capabilityIndex]);
        }
    }
    for (std::uint32_t capabilityIndex = 0; capabilityIndex < pluginEntry->optionalCapabilityCount; ++capabilityIndex)
    {
        if (pluginEntry->optionalCapabilities != nullptr && pluginEntry->optionalCapabilities[capabilityIndex] != nullptr)
        {
            outRecord.optionalCapabilities.emplace_back(pluginEntry->optionalCapabilities[capabilityIndex]);
        }
    }
    for (std::uint32_t permissionIndex = 0; permissionIndex < pluginEntry->declaredPermissionCount; ++permissionIndex)
    {
        if (pluginEntry->declaredPermissions != nullptr && pluginEntry->declaredPermissions[permissionIndex] != nullptr)
        {
            outRecord.declaredPermissions.emplace_back(pluginEntry->declaredPermissions[permissionIndex]);
        }
    }

    // The library is released on purpose: the plugin is instantiated later
    // through loadPluginLibrary, after the demand closure chose the active set.
    library->release();

    outCacheEntry.sizeBytes            = candidate.sizeBytes;
    outCacheEntry.modificationStamp    = candidate.modificationStamp;
    outCacheEntry.moduleName           = outRecord.name;
    outCacheEntry.moduleVersion        = outRecord.version;
    outCacheEntry.contentSha256        = outRecord.contentSha256;
    outCacheEntry.providedCapabilities = outRecord.providedCapabilities;
    outCacheEntry.requiredCapabilities = outRecord.requiredCapabilities;
    outCacheEntry.optionalCapabilities = outRecord.optionalCapabilities;
    outCacheEntry.declaredPermissions  = outRecord.declaredPermissions;
    return true;
}

bool EngineKernel::loadPluginLibrary(
    ModulePluginRecord &plugin,
    std::string        &report
)
{
    auto        library = std::make_unique<DynamicLibrary>();
    std::string loadError;
    if (!library->load(plugin.libraryPath, loadError))
    {
        report += "- " + loadError + "\n";
        return false;
    }

    const auto query
        = reinterpret_cast<const EngineModulePluginEntry *(*)()>(library->resolveSymbol("engineQueryModulePlugin"));
    if (query == nullptr)
    {
        report += "- '" + plugin.libraryPath.string()
                + "' exports no engineQueryModulePlugin entry point; not a module plugin.\n";
        return false;
    }

    const EngineModulePluginEntry *pluginEntry = query();
    if (pluginEntry == nullptr || pluginEntry->abiVersion != engineModulePluginAbiVersion)
    {
        report += "- '" + plugin.libraryPath.string()
                + "' was built against a different module plugin ABI (expected version "
                + std::to_string(engineModulePluginAbiVersion) + "); rebuild the plugin with this kernel.\n";
        return false;
    }
    if (pluginEntry->moduleName == nullptr || pluginEntry->createModule == nullptr)
    {
        report += "- '" + plugin.libraryPath.string() + "' returns an incomplete plugin descriptor.\n";
        return false;
    }

    plugin.name = pluginEntry->moduleName;
    if (pluginEntry->moduleVersion != nullptr)
    {
        plugin.version = pluginEntry->moduleVersion;
    }
    plugin.createModule = pluginEntry->createModule;
    plugin.providedCapabilities.clear();
    plugin.requiredCapabilities.clear();
    plugin.optionalCapabilities.clear();
    plugin.declaredPermissions.clear();
    for (std::uint32_t index = 0; index < pluginEntry->providedCapabilityCount; ++index)
    {
        if (pluginEntry->providedCapabilities != nullptr && pluginEntry->providedCapabilities[index] != nullptr)
        {
            plugin.providedCapabilities.emplace_back(pluginEntry->providedCapabilities[index]);
        }
    }
    for (std::uint32_t index = 0; index < pluginEntry->requiredCapabilityCount; ++index)
    {
        if (pluginEntry->requiredCapabilities != nullptr && pluginEntry->requiredCapabilities[index] != nullptr)
        {
            plugin.requiredCapabilities.emplace_back(pluginEntry->requiredCapabilities[index]);
        }
    }
    for (std::uint32_t index = 0; index < pluginEntry->optionalCapabilityCount; ++index)
    {
        if (pluginEntry->optionalCapabilities != nullptr && pluginEntry->optionalCapabilities[index] != nullptr)
        {
            plugin.optionalCapabilities.emplace_back(pluginEntry->optionalCapabilities[index]);
        }
    }
    for (std::uint32_t index = 0; index < pluginEntry->declaredPermissionCount; ++index)
    {
        if (pluginEntry->declaredPermissions != nullptr && pluginEntry->declaredPermissions[index] != nullptr)
        {
            plugin.declaredPermissions.emplace_back(pluginEntry->declaredPermissions[index]);
        }
    }

    plugin.library = std::move(library);
    return true;
}

bool EngineKernel::instantiateActiveModules(std::string &report)
{
    bool successful = true;
    for (ModulePluginRecord &plugin : modulePlugins)
    {
        if (!plugin.active)
        {
            continue;
        }

        if (!plugin.library && !loadPluginLibrary(plugin, report))
        {
            successful = false;
            continue;
        }
        ComponentRecord record;
        record.name          = plugin.name;
        record.manifest.kind = ComponentKind::Module;

        try
        {
            record.ownedModule.reset(plugin.createModule());
        } catch (const std::exception &error)
        {
            report += "- module factory for '" + plugin.name + "' threw: " + error.what() + "\n";
            successful = false;
            continue;
        }
        record.moduleInstance = record.ownedModule.get();
        if (record.ownedModule == nullptr)
        {
            report += "- module factory for '" + plugin.name + "' returned no instance.\n";
            successful = false;
            continue;
        }
        record.manifest = record.moduleInstance->manifest();
        if (record.manifest.name != plugin.name)
        {
            report += "- module '" + plugin.name + "' reports the manifest name '" + record.manifest.name
                    + "'; the descriptor name and the manifest name must match.\n";
            successful = false;
            continue;
        }
        if (!plugin.version.empty() && !record.manifest.version.empty() && plugin.version != record.manifest.version)
        {
            report += "- module '" + plugin.name + "' descriptor version '" + plugin.version
                    + "' differs from its manifest version '" + record.manifest.version
                    + "'; the descriptor and the manifest must agree.\n";
            successful = false;
            continue;
        }
        if (!sameStringSet(record.manifest.providedCapabilities, plugin.providedCapabilities))
        {
            report += "- module '" + plugin.name
                    + "' declares different provided capabilities in its manifest than in its plugin descriptor; "
                      "rebuild the plugin with this kernel.\n";
            successful = false;
            continue;
        }
        if (!sameStringSet(record.manifest.requiredCapabilities, plugin.requiredCapabilities))
        {
            report += "- module '" + plugin.name
                    + "' declares different required capabilities in its manifest than in its plugin descriptor; "
                      "rebuild the plugin with this kernel.\n";
            successful = false;
            continue;
        }
        if (!sameStringSet(record.manifest.optionalCapabilities, plugin.optionalCapabilities))
        {
            report += "- module '" + plugin.name
                    + "' declares different optional capabilities in its manifest than in its plugin descriptor; "
                      "rebuild the plugin with this kernel.\n";
            successful = false;
            continue;
        }

        std::vector<std::string> manifestEnvironmentPermissions;
        for (const std::string &permission : record.manifest.requiredPermissions)
        {
            if (permission.rfind(capabilityPermissionPrefix, 0) != 0)
            {
                manifestEnvironmentPermissions.push_back(permission);
            }
        }
        if (!sameStringSet(manifestEnvironmentPermissions, plugin.declaredPermissions))
        {
            report += "- module '" + plugin.name
                    + "' requests different environment permissions in its manifest than its plugin descriptor "
                      "declares; declare the environment permissions (anything outside the capability.* "
                      "namespace) in the descriptor's declaredPermissions so the trust gate sees them before "
                      "the module is instantiated.\n";
            successful = false;
            continue;
        }
        records.push_back(std::move(record));
    }

    for (ModulePluginRecord &plugin : modulePlugins)
    {
        if (!plugin.active && plugin.library)
        {
            plugin.library->release();
        }
    }
    return successful;
}

void EngineKernel::appendDeclaredModuleRecords()
{
    for (const ModulePluginRecord &plugin : modulePlugins)
    {
        if (!plugin.active)
        {
            continue;
        }
        ComponentRecord record;
        record.name                          = plugin.name;
        record.manifest.kind                 = ComponentKind::Module;
        record.manifest.name                 = plugin.name;
        record.manifest.version              = plugin.version.empty() ? "0.1.0" : plugin.version;
        record.manifest.providedCapabilities = plugin.providedCapabilities;
        record.manifest.requiredCapabilities = plugin.requiredCapabilities;
        record.manifest.optionalCapabilities = plugin.optionalCapabilities;
        record.manifest.requiredPermissions  = plugin.declaredPermissions;
        records.push_back(std::move(record));
    }
}

void EngineKernel::appendModRiskWarning(
    const ModManifest &modManifest,
    bool               isNativeMod
)
{
    std::vector<std::string> reasons;
    if (isNativeMod)
    {
        reasons.emplace_back("it runs native machine code");
    }
    std::vector<std::string> environmentPermissions;
    for (const std::string &permission : modManifest.permissions)
    {
        if (permission.rfind(capabilityPermissionPrefix, 0) != 0)
        {
            environmentPermissions.push_back(permission);
        }
    }
    if (!environmentPermissions.empty())
    {
        std::string joined;
        for (const std::string &permission : environmentPermissions)
        {
            if (!joined.empty())
            {
                joined += ", ";
            }
            joined += "'" + permission + "'";
        }
        reasons.push_back("it requests permissions outside the engine's capability system: " + joined);
    }
    if (modManifest.capabilities.size() >= broadCapabilityWarningThreshold)
    {
        reasons.push_back(
            "it acquires a broad set of " + std::to_string(modManifest.capabilities.size()) + " capabilities"
        );
    }
    if (reasons.empty())
    {
        return;
    }
    ComponentTrustAdvisory advisory;
    advisory.kind    = modManifest.isNative() ? ComponentKind::SharedLibraryMod : ComponentKind::ScriptMod;
    advisory.name    = modManifest.id;
    advisory.version = modManifest.version;
    advisory.sha256  = isNativeMod ? nativeModContentHash(modManifest) : scriptModContentHash(modManifest);
    advisory.reasons = reasons;
    trustAdvisories_.push_back(std::move(advisory));
    std::string joined;
    for (const std::string &reason : reasons)
    {
        if (!joined.empty())
        {
            joined += "; ";
        }
        joined += reason;
    }
    startupWarnings_ += "warning: mod '" + modManifest.id + advisoryIdentitySuffix(trustAdvisories_.back())
                      + " requests elevated trust: " + joined
                      + ". Review this mod before trusting it with sensitive data.\n";
}

void EngineKernel::appendModuleRiskWarning(const ModulePluginRecord &plugin)
{
    std::vector<std::string> reasons;
    std::vector<std::string> environmentPermissions;
    for (const std::string &permission : plugin.declaredPermissions)
    {
        if (permission.rfind(capabilityPermissionPrefix, 0) != 0)
        {
            environmentPermissions.push_back(permission);
        }
    }
    if (!environmentPermissions.empty())
    {
        std::string joined;
        for (const std::string &permission : environmentPermissions)
        {
            if (!joined.empty())
            {
                joined += ", ";
            }
            joined += "'" + permission + "'";
        }
        reasons.push_back("it requests permissions outside the engine's capability system: " + joined);
    }
    const std::size_t acquiredCapabilities = plugin.requiredCapabilities.size() + plugin.optionalCapabilities.size();
    if (acquiredCapabilities >= broadCapabilityWarningThreshold)
    {
        reasons.push_back("it acquires a broad set of " + std::to_string(acquiredCapabilities) + " capabilities");
    }
    if (reasons.empty())
    {
        return;
    }
    ComponentTrustAdvisory advisory;
    advisory.kind    = ComponentKind::Module;
    advisory.name    = plugin.name;
    advisory.version = plugin.version;
    advisory.sha256  = plugin.contentSha256;
    advisory.reasons = reasons;
    trustAdvisories_.push_back(std::move(advisory));
    std::string joined;
    for (const std::string &reason : reasons)
    {
        if (!joined.empty())
        {
            joined += "; ";
        }
        joined += reason;
    }
    startupWarnings_ += "warning: module '" + plugin.name + advisoryIdentitySuffix(trustAdvisories_.back())
                      + " requests elevated trust: " + joined
                      + ". Review this module before trusting it with sensitive data.\n";
}

bool EngineKernel::evaluateTrustGate(
    std::string &report,
    bool         enforce
)
{
    if (trustAdvisories_.empty())
    {
        return true;
    }

    const std::vector<TrustBinding> modBindings    = configurationStore.trustedModBindings();
    const std::vector<TrustBinding> moduleBindings = configurationStore.trustedModuleBindings();
    const bool                      allMods        = configurationStore.trustAllMods();
    const bool                      allModules     = configurationStore.trustAllModules();
    std::vector<std::string>        bindingWarnings;
    const auto                      isTrusted = [&](const ComponentTrustAdvisory &advisory) {
        if (advisory.kind == ComponentKind::Module)
        {
            return allModules || anyTrustBindingMatches(moduleBindings, advisory, bindingWarnings);
        }
        return allMods || anyTrustBindingMatches(modBindings, advisory, bindingWarnings);
    };

    std::vector<ComponentTrustAdvisory> untrusted;
    for (const ComponentTrustAdvisory &advisory : trustAdvisories_)
    {
        if (!isTrusted(advisory))
        {
            untrusted.push_back(advisory);
        }
    }
    for (const std::string &warning : bindingWarnings)
    {
        startupWarnings_ += "warning: " + warning + "\n";
    }
    if (untrusted.empty())
    {
        return true;
    }

    std::string names;
    for (const ComponentTrustAdvisory &advisory : untrusted)
    {
        if (!names.empty())
        {
            names += ", ";
        }
        names += describeComponentKind(advisory.kind) + " '" + advisory.name + "'"
               + advisoryIdentitySuffix(advisory);
    }
    const char *trustInstructions
        = "To proceed without being asked, trust the components in the configuration's [trust] section:\n"
          "  trustMods = [\"<mod id>\"]       (or trustAllMods = true)\n"
          "  trustModules = [\"<module name>\"] (or trustAllModules = true)\n"
          "Bind trust to content so replaced binaries or scripts cannot inherit it:\n"
          "  trustMods = [{ id = \"<mod id>\", version = \"1.2.3\", sha256 = \"<64 hex characters>\" }]\n"
          "  trustModules = [{ name = \"<module name>\", sha256 = \"<64 hex characters>\" }]\n";

    if (!enforce)
    {
        startupWarnings_ += "note: " + std::to_string(untrusted.size())
                          + " component(s) that can be harmful are not covered by the trust configuration (" + names
                          + "); a real run requires trusting them under [trust] or agreeing to them at startup.\n";
        return true;
    }

    if (trustConsentHandler_ != nullptr && trustConsentHandler_(untrusted))
    {
        startupWarnings_ += "agreement granted at startup: proceeding with " + names + " for this run.\n";
        return true;
    }
    if (trustConsentHandler_ != nullptr)
    {
        report += "startup aborted: agreement was not granted for " + names + ".\n";
    } else
    {
        report += "startup aborted: " + std::to_string(untrusted.size()) + " component(s) that can be harmful (" + names
                + ") are not trusted and no agreement could be requested (no interactive consent "
                  "surface is connected).\n";
    }
    report += trustInstructions;
    return false;
}

void EngineKernel::loadNativeModLibraries(std::string &report)
{
    for (const DiscoveredModPackage &package : pendingNativeModPackages_)
    {
        const ModManifest &modManifest = package.manifest;
        NativeModLoad      load;
        if (!loadNativeModEntry(package, load, report))
        {
            auto recordIt
                = std::find_if(records.begin(), records.end(), [&modManifest](const ComponentRecord &candidate) {
                      return candidate.name == modManifest.id
                          && candidate.manifest.kind == ComponentKind::SharedLibraryMod;
                  });
            if (recordIt != records.end())
            {
                records.erase(recordIt);
            }
            continue;
        }
        auto recordIt = std::find_if(records.begin(), records.end(), [&modManifest](const ComponentRecord &candidate) {
            return candidate.name == modManifest.id && candidate.manifest.kind == ComponentKind::SharedLibraryMod;
        });
        if (recordIt == records.end())
        {
            report += "- native mod '" + modManifest.id + "' lost its component record; the package is dropped.\n";
            load.entryTable->destroyMod(load.modInstance);
            continue;
        }
        ComponentRecord        &record           = *recordIt;
        const ComponentManifest compiledManifest = load.modInstance->manifest();
        if (!compiledManifest.providedCapabilities.empty()
            && !sameStringSet(compiledManifest.providedCapabilities, modManifest.providedCapabilities))
        {
            report += "- native mod '" + modManifest.id
                    + "' provides different capabilities in its compiled manifest than mod.toml declares under "
                      "'provides'; declare the provided capabilities in mod.toml (and only there) so the graph "
                      "and the validate dry run see them.\n";
            load.entryTable->destroyMod(load.modInstance);
            records.erase(recordIt);
            continue;
        }
        if (compiledManifest.providedCapabilities.empty() && !modManifest.providedCapabilities.empty())
        {
            report += "- native mod '" + modManifest.id + "' declares 'provides' in mod.toml but its compiled "
                    + "manifest provides nothing; rebuild the mod against this engine.\n";
            load.entryTable->destroyMod(load.modInstance);
            records.erase(recordIt);
            continue;
        }
        record.library       = std::move(load.library);
        record.modEntryTable = load.entryTable;
        record.modInstance   = load.modInstance;
    }
    pendingNativeModPackages_.clear();
}

EngineKernel::ComponentRecord *EngineKernel::resolveScriptHost(
    const ComponentRecord &scriptRecord,
    std::string           &report
)
{
    const std::string capabilityName = scriptRecord.scriptMod.hostCapability;
    const auto        providesHost   = [&capabilityName](const ComponentRecord &record) {
        return record.manifest.kind == ComponentKind::Module
            && std::any_of(
                record.manifest.providedCapabilities.begin(),
                record.manifest.providedCapabilities.end(),
                [&capabilityName](const std::string &candidate) {
                    return capabilityRequirementSatisfiedBy(capabilityName, candidate);
                }
            );
    };
    std::vector<ComponentRecord *> candidates;
    for (ComponentRecord &record : records)
    {
        if (record.manifest.kind != ComponentKind::Module || !record.constructed)
        {
            continue;
        }
        if (providesHost(record))
        {
            candidates.push_back(&record);
        }
    }
    if (candidates.empty())
    {
        report += "- script mod '" + scriptRecord.name + "' cannot be hosted: no active module provides the '"
                + capabilityName + "' capability it names as its host.\n";
        return nullptr;
    }
    if (candidates.size() > 1)
    {
        const std::vector<DependencyResolver::ProviderSelection> selections = configurationStore.providerSelections();
        const auto                                               selection  = std::find_if(
            selections.begin(),
            selections.end(),
            [&capabilityName](const DependencyResolver::ProviderSelection &candidate) {
                return candidate.capabilityName == capabilityName;
            }
        );
        if (selection != selections.end())
        {
            for (ComponentRecord *candidate : candidates)
            {
                if (candidate->name == selection->providerName)
                {
                    return candidate;
                }
            }
        }
        std::string names;
        for (const ComponentRecord *candidate : candidates)
        {
            if (!names.empty())
            {
                names += ", ";
            }
            names += "'" + candidate->name + "'";
        }
        report += "- script mod '" + scriptRecord.name + "' cannot be hosted: " + std::to_string(candidates.size())
                + " active modules provide the '" + capabilityName + "' capability (" + names
                + "); select one under [providers].\n";
        return nullptr;
    }
    return candidates.front();
}

DependencyResolver::ResolutionResult EngineKernel::resolveGraph(
    const std::vector<ComponentManifest> &manifests,
    std::string                          &report
) const
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
    const std::vector<DependencyResolver::ResolutionDiagnostic> &diagnostics,
    std::string                                                 &report
) const
{
    report += "engine startup failed with " + std::to_string(diagnostics.size()) + " problem(s):\n";
    for (const DependencyResolver::ResolutionDiagnostic &diagnostic : diagnostics)
    {
        report += "  - " + diagnostic.message + "\n";
    }
}

bool EngineKernel::validate(std::string &report)
{
    report.clear();
    if (!prepareComponents(report, false))
    {
        report += startupWarnings_;
        return false;
    }

    std::vector<DependencyResolver::ResolutionDiagnostic> permissionDiagnostics;
    for (const auto &[record, permission] : applyPermissionPass())
    {
        DependencyResolver::ResolutionDiagnostic diagnostic;
        diagnostic.kind    = DependencyResolver::ResolutionDiagnostic::Kind::UnknownCapability;
        diagnostic.message = describeComponentKind(record->manifest.kind) + " '" + record->name
                           + "' requires permission '" + permission + "' which the configuration denies.";
        permissionDiagnostics.push_back(std::move(diagnostic));
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
    const DependencyResolver::ResolutionResult resolution = resolveGraph(componentManifests(), resolutionReport);
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

bool EngineKernel::declareKnownKeyRegistry(std::string &report)
{
    configurationStore.declareAssemblyKnownKeys();
    for (const ComponentRecord &record : records)
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

bool EngineKernel::runConstructPhase(
    const DependencyResolver::ResolutionResult &resolution,
    const std::vector<ComponentManifest>       &manifests,
    std::string                                &report
)
{
    for (const std::size_t manifestIndex : resolution.constructionOrder)
    {
        const ComponentManifest &manifest = manifests[manifestIndex];
        ComponentRecord         *record   = findRecord(manifest.name);
        if (record == nullptr || record->constructed)
        {
            continue;
        }

        if (record->manifest.kind == ComponentKind::ScriptMod)
        {
            ComponentRecord *hostRecord = resolveScriptHost(*record, report);
            if (hostRecord == nullptr)
            {
                return false;
            }
            if (!hostRecord->constructed)
            {
                report += "- script mod '" + record->name + "' cannot be hosted: module '" + hostRecord->name
                        + "' did not construct.\n";
                return false;
            }
            ScriptModHost *scriptHost = hostRecord->moduleInstance->scriptModHost();
            if (scriptHost == nullptr)
            {
                report += "- script mod '" + record->name + "' cannot be hosted: module '" + hostRecord->name
                        + "' provides the '" + record->scriptMod.hostCapability
                        + "' capability but does not host script mods.\n";
                return false;
            }
            try
            {
                scriptHost->hostScriptMod(record->scriptMod);
            } catch (const std::exception &error)
            {
                report += "- construct failed for script mod '" + record->name + "': " + error.what() + "\n";
                return false;
            }
            record->constructed = true;
            componentStartOrder.push_back(record);
            continue;
        }

        record->context = std::make_unique<ComponentContext>(
            bus,
            *this,
            permissionSystem_,
            configurationStore,
            record->name,
            record->manifest.kind,
            &demandedCapabilities,
            mainThreadScheduler_
        );
        ComponentContext &context = *record->context;
        try
        {
            if (record->moduleInstance != nullptr)
            {
                record->moduleInstance->construct(context);
            } else if (record->modInstance != nullptr)
            {
                record->modInstance->construct(context);
            }
        } catch (const std::exception &error)
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
            } else if (record->modInstance != nullptr)
            {
                capabilityPointer = record->modInstance->capabilityPointerFor(capabilityName);
            }
            if (capabilityPointer == nullptr)
            {
                report += "- " + describeComponentKind(record->manifest.kind) + " '" + record->name
                        + "' declares capability '" + capabilityName + "' but returned no implementation.\n";
                return false;
            }
        }
        record->constructed = true;
        componentStartOrder.push_back(record);
    }
    return true;
}

void EngineKernel::classifyCapabilityProviders(
    const std::string     &capabilityName,
    CapabilityProviderSet &outSet
)
{
    outSet.exactProviders.clear();
    outSet.memberProviders.clear();
    for (ComponentRecord &record : records)
    {
        if (!record.constructed)
        {
            continue;
        }
        for (const std::string &providedName : record.manifest.providedCapabilities)
        {
            if (providedName == capabilityName)
            {
                outSet.exactProviders.push_back(&record);
            } else if (capabilityRequirementSatisfiedBy(capabilityName, providedName))
            {
                outSet.memberProviders.emplace_back(&record, providedName);

                break;
            }
        }
    }
}

const EngineKernel::CapabilityProviderSet &EngineKernel::capabilityProviderSetFor(const std::string &capabilityName)
{
    if (!capabilityProviderIndexFrozen)
    {
        classifyCapabilityProviders(capabilityName, providerScratch);
        return providerScratch;
    }

    for (CapabilityProviderIndexEntry &entry : capabilityProviderIndex)
    {
        if (entry.capabilityName == capabilityName)
        {
            return entry.providers;
        }
    }

    classifyCapabilityProviders(capabilityName, providerScratch);
    capabilityProviderIndex.push_back(CapabilityProviderIndexEntry{capabilityName, providerScratch});
    return capabilityProviderIndex.back().providers;
}

void *EngineKernel::resolveCapabilityPointer(
    const std::string &,
    ComponentKind      requesterKind,
    const std::string &capabilityName
)
{
    const CapabilityProviderSet &providers       = capabilityProviderSetFor(capabilityName);
    const bool                   moduleRequester = requesterKind == ComponentKind::Module;

    if (providers.exactProviders.size() == 1) [[likely]]
    {
        ComponentRecord *provider = providers.exactProviders.front();
        if (!moduleRequester || provider->manifest.kind == ComponentKind::Module) [[likely]]
        {
            return capabilityPointerOf(*provider, capabilityName);
        }
    } else if (providers.exactProviders.size() > 1)
    {
        std::vector<ComponentRecord *> visibleProviders;
        visibleProviders.reserve(providers.exactProviders.size());
        for (ComponentRecord *provider : providers.exactProviders)
        {
            if (!moduleRequester || provider->manifest.kind == ComponentKind::Module)
            {
                visibleProviders.push_back(provider);
            }
        }
        if (visibleProviders.size() > 1)
        {
            std::string names;
            for (const ComponentRecord *provider : visibleProviders)
            {
                if (!names.empty())
                {
                    names += ", ";
                }
                names += "'" + provider->name + "'";
            }
            throw std::logic_error(
                "capability '" + capabilityName + "' is provided by several components (" + names
                + "); choose one in the configuration under [providers]"
            );
        }
        if (visibleProviders.size() == 1) [[unlikely]]
        {
            return capabilityPointerOf(*visibleProviders.front(), capabilityName);
        }
    }

    if (providers.memberProviders.size() == 1) [[likely]]
    {
        const auto &member = providers.memberProviders.front();
        if (!moduleRequester || member.first->manifest.kind == ComponentKind::Module)
        {
            return capabilityPointerOf(*member.first, member.second);
        }
    } else if (providers.memberProviders.size() > 1)
    {
        std::vector<std::pair<ComponentRecord *, std::string>> visibleMembers;
        visibleMembers.reserve(providers.memberProviders.size());
        for (const auto &member : providers.memberProviders)
        {
            if (!moduleRequester || member.first->manifest.kind == ComponentKind::Module)
            {
                visibleMembers.push_back(member);
            }
        }
        if (visibleMembers.size() > 1)
        {
            std::string names;
            for (const auto &member : visibleMembers)
            {
                if (!names.empty())
                {
                    names += ", ";
                }
                names += "'" + member.first->name + "'";
            }
            throw std::logic_error(
                "capability family '" + capabilityName + "' is ambiguous: " + std::to_string(visibleMembers.size())
                + " implementations are active (" + names + "); choose one in the configuration under [providers]"
            );
        }
        if (visibleMembers.size() == 1) [[unlikely]]
        {
            return capabilityPointerOf(*visibleMembers.front().first, visibleMembers.front().second);
        }
    }

    if (moduleRequester)
    {
        std::string providerNames;
        for (const ComponentRecord *provider : providers.exactProviders)
        {
            if (!providerNames.empty())
            {
                providerNames += ", ";
            }
            providerNames += "'" + provider->name + "'";
        }
        for (const auto &member : providers.memberProviders)
        {
            if (!providerNames.empty())
            {
                providerNames += ", ";
            }
            providerNames += "'" + member.first->name + "'";
        }
        if (!providerNames.empty())
        {
            throw std::logic_error(
                "capability '" + capabilityName + "' is provided only by mods (" + providerNames
                + "); modules can only use capabilities provided by other modules"
            );
        }
    }

    throw std::logic_error(
        "capability '" + capabilityName
        + "' has no active provider; a component acquired it without a "
          "requirement whose provider is available in this run"
    );
}

bool EngineKernel::hasSingleCapabilityProvider(
    const std::string &capabilityName,
    ComponentKind      requesterKind
)
{
    const CapabilityProviderSet &providers       = capabilityProviderSetFor(capabilityName);
    const bool                   moduleRequester = requesterKind == ComponentKind::Module;
    if (!providers.exactProviders.empty())
    {
        if (!moduleRequester)
        {
            return providers.exactProviders.size() == 1;
        }
        std::size_t visibleCount = 0;
        for (const ComponentRecord *provider : providers.exactProviders)
        {
            if (provider->manifest.kind == ComponentKind::Module)
            {
                ++visibleCount;
            }
        }
        return visibleCount == 1;
    }
    if (!moduleRequester)
    {
        return providers.memberProviders.size() == 1;
    }
    std::size_t visibleMembers = 0;
    for (const auto &member : providers.memberProviders)
    {
        if (member.first->manifest.kind == ComponentKind::Module)
        {
            ++visibleMembers;
        }
    }
    return visibleMembers == 1;
}

void *EngineKernel::tryResolveCapabilityPointer(
    const std::string &,
    ComponentKind      requesterKind,
    const std::string &capabilityName
)
{
    const CapabilityProviderSet &providers       = capabilityProviderSetFor(capabilityName);
    const bool                   moduleRequester = requesterKind == ComponentKind::Module;

    std::size_t      visibleExact = 0;
    ComponentRecord *exactOne     = nullptr;
    for (ComponentRecord *provider : providers.exactProviders)
    {
        if (moduleRequester && provider->manifest.kind != ComponentKind::Module)
        {
            continue;
        }
        ++visibleExact;
        exactOne = provider;
    }
    if (visibleExact == 1)
    {
        return capabilityPointerOf(*exactOne, capabilityName);
    }
    if (providers.exactProviders.empty() || visibleExact == 0)
    {
        std::size_t                               visibleMembers = 0;
        std::pair<ComponentRecord *, std::string> memberOne;
        for (const auto &member : providers.memberProviders)
        {
            if (moduleRequester && member.first->manifest.kind != ComponentKind::Module)
            {
                continue;
            }
            ++visibleMembers;
            memberOne = member;
        }
        if (visibleMembers == 1)
        {
            return capabilityPointerOf(*memberOne.first, memberOne.second);
        }
    }
    return nullptr;
}

std::vector<void *> EngineKernel::resolveEachCapabilityPointer(
    const std::string &,
    ComponentKind      requesterKind,
    const std::string &capabilityName
)
{
    const CapabilityProviderSet &providers = capabilityProviderSetFor(capabilityName);

    const bool moduleRequester = requesterKind == ComponentKind::Module;

    std::vector<void *> pointers;
    if (!providers.exactProviders.empty())
    {
        pointers.reserve(providers.exactProviders.size());
        for (ComponentRecord *provider : providers.exactProviders)
        {
            if (moduleRequester && provider->manifest.kind != ComponentKind::Module)
            {
                continue;
            }
            if (provider->moduleInstance != nullptr)
            {
                if (void *capabilityPointer = provider->moduleInstance->capabilityPointerFor(capabilityName);
                    capabilityPointer != nullptr)
                {
                    pointers.push_back(capabilityPointer);
                }
            } else if (provider->modInstance != nullptr)
            {
                if (void *capabilityPointer = provider->modInstance->capabilityPointerFor(capabilityName);
                    capabilityPointer != nullptr)
                {
                    pointers.push_back(capabilityPointer);
                }
            }
        }
        return pointers;
    }
    if (!providers.memberProviders.empty())
    {
        pointers.reserve(providers.memberProviders.size());
        for (const auto &member : providers.memberProviders)
        {
            ComponentRecord *provider = member.first;
            if (moduleRequester && provider->manifest.kind != ComponentKind::Module)
            {
                continue;
            }
            if (provider->moduleInstance != nullptr)
            {
                if (void *capabilityPointer = provider->moduleInstance->capabilityPointerFor(member.second);
                    capabilityPointer != nullptr)
                {
                    pointers.push_back(capabilityPointer);
                }
            } else if (provider->modInstance != nullptr)
            {
                if (void *capabilityPointer = provider->modInstance->capabilityPointerFor(member.second);
                    capabilityPointer != nullptr)
                {
                    pointers.push_back(capabilityPointer);
                }
            }
        }
    }
    return pointers;
}

void *EngineKernel::capabilityPointerOf(
    ComponentRecord   &provider,
    const std::string &capabilityName
)
{
    void *capabilityPointer = nullptr;
    if (provider.moduleInstance != nullptr)
    {
        capabilityPointer = provider.moduleInstance->capabilityPointerFor(capabilityName);
    } else if (provider.modInstance != nullptr)
    {
        capabilityPointer = provider.modInstance->capabilityPointerFor(capabilityName);
    }
    if (capabilityPointer == nullptr)
    {
        throw std::logic_error(
            "component '" + provider.name + "' declares capability '" + capabilityName
            + "' but returned no implementation"
        );
    }
    return capabilityPointer;
}

bool EngineKernel::construct(std::string &report)
{
    report.clear();
    if (constructed)
    {
        return true;
    }

    if (componentsGathered && !nativeModLibrariesLoaded_)
    {
        stop();
    }

    capabilityProviderIndexFrozen = false;
    capabilityProviderIndex.clear();
    if (!prepareComponents(report, true))
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

    const std::vector<ComponentManifest>       manifests = componentManifests();
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

    capabilityProviderIndexFrozen = true;
    constructed                   = true;
    return true;
}

void EngineKernel::start()
{
    if (!constructed || started)
    {
        throw std::runtime_error("engine kernel: start requires a completed construct phase");
    }

    for (ComponentRecord *recordPointer : componentStartOrder)
    {
        ComponentRecord &record = *recordPointer;
        if (!record.constructed || record.started)
        {
            continue;
        }
        if (record.moduleInstance != nullptr)
        {
            record.moduleInstance->start();
        } else if (record.modInstance != nullptr)
        {
            record.modInstance->start();
        }
        record.started = true;
    }
    started = true;
    bus.publish(EngineStartupCompletedEvent{});
}

void EngineKernel::stop()
{
    capabilityProviderIndexFrozen = false;
    capabilityProviderIndex.clear();
    if (started)
    {
        try
        {
            bus.flushDeferredEvents();
            bus.publish(EngineShutdownStartedEvent{});
        } catch (const std::exception &error)
        {
            std::fprintf(stderr, "engine kernel: shutdown event handler failed: %s\n", error.what());
        }
    }

    mainThreadScheduler_.clear();

    for (auto iterator = componentStartOrder.rbegin(); iterator != componentStartOrder.rend(); ++iterator)
    {
        ComponentRecord &record = **iterator;
        if (record.started && record.moduleInstance != nullptr)
        {
            record.moduleInstance->stop();
        }
        if (record.started && record.modInstance != nullptr)
        {
            record.modInstance->stop();
        }
    }
    for (ComponentRecord &record : records)
    {
        if (record.modEntryTable != nullptr && record.modInstance != nullptr)
        {
            record.modEntryTable->destroyMod(record.modInstance);
            record.modInstance = nullptr;
        }
        record.started     = false;
        record.constructed = false;
    }
    started     = false;
    constructed = false;

    bus.releaseChannels();

    records.clear();
    componentStartOrder.clear();
    componentsGathered = false;
}

bool EngineKernel::reloadConfiguration(std::string &report)
{
    report.clear();
    if (!constructed)
    {
        report = "engine reload refused:\n- reload requires a completed construct phase; before construct the "
                 "configuration is assembled for the first time, not re-assembled\n";
        return false;
    }

    const std::filesystem::path resolvedConfigurationPath = resolveAgainstExecutableDirectory(configurationFilePath);
    std::string                 parseError;
    if (!ConfigurationStore::fileParses(resolvedConfigurationPath, parseError))
    {
        report = "engine reload refused (the running configuration is untouched):\n- " + parseError + "\n";
        return false;
    }
    for (const std::filesystem::path &defaultsFile : collectModuleDefaultsFiles())
    {
        if (!ConfigurationStore::fileParses(defaultsFile, parseError))
        {
            report = "engine reload refused (the running configuration is untouched):\n- " + parseError + "\n";
            return false;
        }
    }
    for (const auto &[defaultsFile, source] : modDefaultsFiles)
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
        reloadScenePath = resolveAgainstExecutableDirectory(sceneFilePath);
    } else if (!assemblyDefinesScene && modSceneClaims.size() == 1)
    {
        reloadScenePath = modSceneClaims.front().sceneFile;
    }
    if (!reloadScenePath.empty() && !ConfigurationStore::fileParses(reloadScenePath, parseError))
    {
        report = "engine reload refused (the running configuration is untouched):\n- " + parseError + "\n";
        return false;
    }

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
    for (const auto &[defaultsFile, source] : modDefaultsFiles)
    {
        if (!configurationStore.mergeDefaultsFile(
                defaultsFile, ConfigProvenance::Layer::ModDefaults, source, reloadError
            ))
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

    permissionSystem_.clearDenials();
    applyPermissionPass();

    std::vector<std::string> changedKeys;
    std::size_t              changedCount = 0;
    std::size_t              sceneChanges = 0;
    std::size_t              removedCount = 0;
    const auto               isScenePath  = [](const std::string &path) {
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

    ConfigurationReloadedEvent reloaded;
    reloaded.changedKeys = std::move(changedKeys);
    bus.publish(reloaded);
    return true;
}

std::string EngineKernel::componentGraphReport() const
{
    std::string report;

    std::vector<const ComponentRecord *> orderedRecords;
    const char                          *orderSource = "start order";
    if (!componentStartOrder.empty())
    {
        orderedRecords.assign(componentStartOrder.begin(), componentStartOrder.end());
    } else if (!records.empty())
    {
        std::string                                resolutionReport;
        const DependencyResolver::ResolutionResult resolution = resolveGraph(componentManifests(), resolutionReport);
        orderSource                                           = "resolved construct order (nothing is constructed yet)";
        if (resolution.successful)
        {
            for (const std::size_t manifestIndex : resolution.constructionOrder)
            {
                orderedRecords.push_back(&records[manifestIndex]);
            }
        } else
        {
            orderSource = "declared order (graph resolution failed - see --validate)";
            for (const ComponentRecord &record : records)
            {
                orderedRecords.push_back(&record);
            }
            report += resolutionReport;
        }
    }

    report += "component graph (" + std::to_string(records.size()) + " component(s), " + orderSource + "):\n";
    for (const ComponentRecord *recordPointer : orderedRecords)
    {
        const ComponentRecord &record = *recordPointer;
        report += "  " + record.name + " (" + describeComponentKind(record.manifest.kind) + " "
                + record.manifest.version + ")\n";

        const auto appendList = [&report](const char *label, const std::vector<std::string> &names) {
            report += std::string("    ") + label + ":";
            if (names.empty())
            {
                report += " (none)";
            } else
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
        if (!record.scriptMod.modId.empty())
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
