#include "engine/kernel/engine_kernel.hpp"

#include "engine/kernel/filesystem_utilities.hpp"
#include "engine/kernel/mod_api.hpp"
#include "engine/kernel/mod_discovery.hpp"
#include "engine/kernel/module_api.hpp"

#include <algorithm>
#include <set>
#include <sstream>
#include <stdexcept>

namespace engine
{

namespace
{

const std::string &hostContextName()
{
    static const std::string name = "<host>";
    return name;
}

// A mod that acquires at least this many distinct capabilities is reported as
// a broad-access advisory: the number is a structural signal only, the kernel
// never interprets what the capabilities mean.
constexpr std::size_t broadCapabilityWarningThreshold = 5;

std::filesystem::path resolveAgainstExecutableDirectory(
    const std::filesystem::path &path
)
{
    if (path.is_absolute())
    {
        return path;
    }
    return executableDirectory() / path;
}

std::string describeComponentKind(
    ComponentKind kind
)
{
    switch (kind)
    {
    case ComponentKind::Module:
        return "module";
    case ComponentKind::SharedLibraryMod:
        return "shared library mod";
    case ComponentKind::ScriptMod:
        return "script mod";
    }
    return "component";
}

// Effective permission grants of one component: the permissions it declared
// plus one "capability.<name>" grant per required capability. Requiring a
// capability and being authorized to acquire it are the same statement, so a
// component says it once; the explicit permissions list remains for grants
// that are not capability acquisitions (filesystem.write, or a capability
// acquired lazily without a graph edge). Configuration denials still win
// over every grant, declared or derived.
std::vector<std::string> effectivePermissions(
    const ComponentManifest &manifest
)
{
    std::set<std::string> unique(manifest.requiredPermissions.begin(), manifest.requiredPermissions.end());
    for (const std::string &capabilityName : manifest.requiredCapabilities)
    {
        unique.insert(std::string(capabilityPermissionPrefix) + capabilityName);
    }
    return std::vector<std::string>(unique.begin(), unique.end());
}

// Order-insensitive comparison of a manifest's capability declaration against
// the static descriptor declaration. Used after instantiation: the descriptor
// resolved the demand, the manifest must agree, and drift between them would
// silently break both directions of the contract.
bool sameStringSet(
    const std::vector<std::string> &left, const std::vector<std::string> &right
)
{
    const std::set<std::string> leftSet(left.begin(), left.end());
    const std::set<std::string> rightSet(right.begin(), right.end());
    return leftSet == rightSet;
}

} // namespace

void EngineKernel::setConfigurationFile(
    const std::filesystem::path &configurationPath
)
{
    configurationFilePath = configurationPath;
    configurationLoaded   = false;
    configurationStore.reset();
}

void EngineKernel::setSceneFile(
    const std::filesystem::path &scenePath
)
{
    sceneFilePath       = scenePath;
    configurationLoaded = false;
    configurationStore.reset();
}

bool EngineKernel::addCommandLineOverride(
    const std::string &assignment, std::string &error
)
{
    // Parsed (not applied) immediately: a malformed --set must fail at the
    // composition root with exit code 2, before any backend starts.
    return configurationStore.stageCommandLineOverride(assignment, error);
}

void EngineKernel::setModsDirectory(
    const std::filesystem::path &modsDirectory
)
{
    explicitModsDirectory = modsDirectory;
    componentsGathered    = false;
}

EngineKernel::~EngineKernel()
{
    // Guarantees that components are stopped and released even when a caller
    // forgets the explicit stop phase.
    stop();
}

std::unique_ptr<ComponentContext> EngineKernel::createHostContext()
{
    permissionSystem_.grantAllPermissions(hostContextName());
    return std::make_unique<ComponentContext>(
        bus, capabilityRegistry_, permissionSystem_, configurationStore, hostContextName(), nullptr
    );
}

bool EngineKernel::loadConfiguration(
    std::string &report
)
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
    // The store API takes an error OUT parameter and clears it: the report is
    // an accumulator, so store errors always travel in local strings.
    std::string loadError;
    if (!configurationStore.loadFromFile(resolvedPath, loadError))
    {
        report += "- " + loadError + "\n";
        return false;
    }
    // Recorded from the assembly layer alone, before any defaults merge: the
    // assembly either owns the world ([scene] present) or delegates it to the
    // installed content mod's scene file. Defaults files can never define
    // [scene] (rejected at merge), so the check stays exact.
    assemblyDefinesScene = configurationStore.hasSection(ConfigurationStore::sceneSectionName);
    // Shipped module defaults (config/modules/<name>/module.toml) fill the
    // gaps the assembly left; a broken defaults file is a startup problem.
    if (!loadModuleDefaults(report))
    {
        return false;
    }
    configurationLoaded = true;
    return true;
}

// Scans the runtime module defaults directory: every config/modules/<name>/
// that contains a module.toml contributes that module's shipped defaults as
// the lowest configuration layer. Discovery is sorted by name so behavior
// never depends on filesystem iteration order; when two defaults files
// provide the same key, the alphabetically first module wins (defaults fill
// gaps, they never override).
bool EngineKernel::loadModuleDefaults(
    std::string &report
)
{
    const std::filesystem::path defaultsDirectory = resolveAgainstExecutableDirectory(
        ConfigurationStore::moduleDefaultsDirectory
    );
    if (!std::filesystem::exists(defaultsDirectory) || !std::filesystem::is_directory(defaultsDirectory))
    {
        return true; // no module ships defaults: a valid state
    }

    std::vector<std::filesystem::path> moduleDirectories;
    for (const std::filesystem::directory_entry &entry : std::filesystem::directory_iterator(defaultsDirectory))
    {
        if (entry.is_directory() && std::filesystem::exists(entry.path() / ConfigurationStore::moduleDefaultsFileName))
        {
            moduleDirectories.push_back(entry.path());
        }
    }
    std::sort(moduleDirectories.begin(), moduleDirectories.end());

    for (const std::filesystem::path &moduleDirectory : moduleDirectories)
    {
        std::string defaultsError;
        if (!configurationStore.mergeDefaultsFile(
                moduleDirectory / ConfigurationStore::moduleDefaultsFileName,
                ConfigProvenance::Layer::ModuleDefaults,
                moduleDirectory.filename().string(),
                defaultsError
            ))
        {
            report += "- " + defaultsError + "\n";
            return false;
        }
    }
    return true;
}

// The world content source and the --set overrides: the layers that must
// wait for mod discovery. Precedence, highest first: the --scene flag (the
// composition root's explicit world), the assembly's own [scene] section,
// the one content mod's declared scene file, the components' built-in
// defaults. After the scene is final the staged --set overrides are written
// on top. Then the configuration is complete, and [diagnostics] is applied
// from the fully merged stack - a --set diagnostics.latency_report=true is
// honored exactly like the configuration key.
bool EngineKernel::finishConfiguration(
    std::string &report
)
{
    if (!sceneFilePath.empty())
    {
        const std::filesystem::path resolvedScenePath = resolveAgainstExecutableDirectory(sceneFilePath);
        if (!std::filesystem::exists(resolvedScenePath))
        {
            report += "- scene file '" + resolvedScenePath.string() + "' does not exist.\n";
            return false;
        }
        // The store API takes an error OUT parameter and clears it: never pass
        // the accumulating report straight in, or earlier diagnostics vanish.
        std::string sceneError;
        if (!configurationStore.applySceneFile(resolvedScenePath, sceneError))
        {
            report += "- " + sceneError + "\n";
            return false;
        }
    }
    else if (!assemblyDefinesScene && modSceneClaims.size() == 1)
    {
        // The assembly defines no world of its own and exactly one active
        // content mod claims it: that claim provides the world, through the
        // same wholesale [scene] swap --scene uses, so partial assembly
        // scenes can never merge with mod scenes.
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
    }
    else if (!assemblyDefinesScene && modSceneClaims.size() > 1)
    {
        // The one-world rule, enforced only where the world is actually
        // ambiguous: with no explicit world source and several active
        // content mods, picking one silently would depend on filesystem
        // listing order. Content mods may coexist freely - the run just has
        // to say which world it wants.
        report += "- the world is ambiguous: " + std::to_string(modSceneClaims.size())
                + " active mods declare scene files";
        for (const ModSceneClaim &claim : modSceneClaims)
        {
            report += " '" + claim.sceneFile.string() + "' (" + claim.ownerId + ")";
        }
        report += "; give the run a world with --scene or a [scene] section in the configuration, "
                  "or disable all but one of these mods.\n";
        return false;
    }
    else if (assemblyDefinesScene && !modSceneClaims.empty())
    {
        // Valid state, but the author should know the installed content is
        // not the world this run uses. validate() prints it; a normal run
        // stays quiet.
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
    // Diagnostics are opt-in: the communication latency report is a developer
    // tool and must stay invisible (and free) for normal runs. Applied after
    // every layer is merged so the whole run - module constructs included -
    // is covered once the flag is set; a --latency-report command line flag
    // set before construct is preserved (the flag can only turn recording ON,
    // never off).
    bus.setLatencyRecordingEnabled(
        bus.latencyRecordingEnabled()
        || configurationStore
            .booleanValue(ConfigurationStore::diagnosticsSectionName, ConfigurationStore::diagnosticsLatencyReportKey)
            .value_or(false)
    );
    return true;
}

bool EngineKernel::prepareComponents(
    std::string &report
)
{
    return loadConfiguration(report) && gatherComponents(report) && finishConfiguration(report);
}

std::vector<std::pair<const EngineKernel::ComponentRecord *, std::string>> EngineKernel::applyPermissionPass()
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

EngineKernel::ComponentRecord *EngineKernel::findRecord(
    const std::string &name
)
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

// Builds one record per component. Modules are NOT registered anywhere: their
// plugin binaries describe themselves statically, the demand closure decides
// which of them this run actually needs, and only those are instantiated.
// Mods are the demand roots of the run: every active package becomes a
// component, and its requirements pull the providers it needs.
bool EngineKernel::gatherComponents(
    std::string &report
)
{
    if (componentsGathered)
    {
        return true;
    }

    // Read the module plugin inventory: one static descriptor per binary,
    // no instances created (see loadModulePlugins).
    loadModulePlugins(report);

    // Mod packages discovered in the mods directory, one mod.toml per
    // package. Mods are always active - they are the reason the run exists -
    // and their required capabilities seed the demand closure.
    gatherModPackages(report);

    // Demand closure over opaque capability names: mods' requirements plus
    // the assembly's [engine].required_capabilities, transitively closed
    // through the providers' own requirements. Marks plugin descriptors
    // active; nothing is instantiated yet.
    computeDemandClosure();

    // Instantiate only the plugins the closure marked active; verify each
    // instance manifest against its static descriptor; release the rest.
    if (!instantiateActiveModules(report))
    {
        return false;
    }

    componentsGathered = true;
    return true;
}

// Turns the packages discovered on disk into component records. The
// filesystem scan and the native library loading live in mod_discovery.cpp;
// this phase owns the kernel policy: scene claims, shipped defaults, record
// wiring for native and script mods, and the structural risk advisories the
// user sees before any mod code runs.
void EngineKernel::gatherModPackages(
    std::string &report
)
{
    // Fresh scan state every call: the claims are re-collected from the
    // current packages, never carried over from an earlier gather.
    modSceneClaims.clear();

    std::filesystem::path modsDirectory = explicitModsDirectory;
    if (modsDirectory.empty())
    {
        modsDirectory = configurationStore.modDirectory();
    }
    const std::filesystem::path             resolvedDirectory = resolveAgainstExecutableDirectory(modsDirectory);
    const std::vector<DiscoveredModPackage> packages          = discoverModPackages(
        resolvedDirectory, configurationStore.disabledMods(), report
    );

    for (const DiscoveredModPackage &package : packages)
    {
        const ModManifest &modManifest = package.manifest;

        // World content claims: collected, never resolved here. Content mods
        // may coexist freely; finishConfiguration decides - an explicit world
        // source (--scene or the assembly's [scene]) makes the claims
        // irrelevant, a single claim provides the world, and two or more
        // claims without an explicit source are a startup error (the world
        // would be ambiguous - never a silent pick of whichever package the
        // filesystem listed first).
        if (!package.sceneFile.empty())
        {
            ModSceneClaim claim;
            claim.sceneFile = package.sceneFile;
            claim.ownerId   = modManifest.id;
            modSceneClaims.push_back(std::move(claim));
        }

        // Shipped defaults: an active package may carry config.toml with
        // default values for the sections it reads. Defaults fill only what
        // the assembly left unset; a broken defaults file is a precise
        // startup problem for the package (and drops it, like a broken
        // manifest does).
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
        }

        if (modManifest.isNative())
        {
            ComponentRecord record;
            record.name          = modManifest.id;
            record.manifest.kind = ComponentKind::SharedLibraryMod;
            NativeModLoad load;
            if (!loadNativeModEntry(package, load, report))
            {
                continue;
            }
            record.library       = std::move(load.library);
            record.modEntryTable = load.entryTable;
            record.modInstance   = load.modInstance;
            record.manifest      = record.modInstance->manifest();
            // The package manifest is authoritative for identity and wiring;
            // the compiled manifest only had to agree on the name.
            record.manifest.name                 = modManifest.id;
            record.manifest.requiredCapabilities = modManifest.capabilities;
            record.manifest.requiredPermissions  = modManifest.permissions;
            record.manifest.requiredComponents   = modManifest.requiresComponents;
            record.manifest.knownConfigKeys      = modManifest.configKeys;
            appendModRiskWarning(modManifest, true);
            records.push_back(std::move(record));
        }
        else
        {
            // Script mod: hosted by whichever active module provides the
            // scripting capability (resolved in the construct phase). The
            // record exists so the resolver and the permission system see
            // the script's capabilities and permissions like any other
            // component's.
            ComponentRecord record;
            record.name                          = modManifest.id;
            record.manifest.kind                 = ComponentKind::ScriptMod;
            record.manifest.name                 = modManifest.id;
            record.manifest.requiredCapabilities = modManifest.capabilities;
            record.manifest.requiredPermissions  = modManifest.permissions;
            record.manifest.requiredComponents   = modManifest.requiresComponents;
            record.manifest.knownConfigKeys      = modManifest.configKeys;
            record.scriptMod.modId               = modManifest.id;
            record.scriptMod.scriptPath          = modManifest.directory / modManifest.entry;
            appendModRiskWarning(modManifest, false);
            records.push_back(std::move(record));
        }
    }
}

// Capability demand closure over opaque names. Seeds: every active mod's
// required capabilities plus the assembly's [engine].required_capabilities
// (the composition root's demand, stated as data). Each demanded capability
// activates the module plugins that provide it - all of them when the
// configuration's [providers] selection does not name exactly one - and an
// activated plugin's own requirements join the worklist. Capability names
// are never interpreted: the kernel only matches strings between consumer
// requirements and provider declarations.
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
        // At this point records holds exactly the active mods: the demand
        // roots of the run.
        for (const std::string &capabilityName : record.manifest.requiredCapabilities)
        {
            worklist.push_back(capabilityName);
        }
    }
    for (const std::string &capabilityName : configurationStore.requiredCapabilities())
    {
        worklist.push_back(capabilityName);
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

        // Providers of this capability among the descriptors and the mod
        // records. The [providers] selection narrows the choice when it
        // names one of them; otherwise every provider activates and the
        // resolver reports the ambiguity (or the selection naming an
        // implementation that does not exist).
        std::vector<std::string> providerNames;
        for (ModulePluginRecord &plugin : modulePlugins)
        {
            if (std::find(plugin.providedCapabilities.begin(), plugin.providedCapabilities.end(), capabilityName)
                != plugin.providedCapabilities.end())
            {
                providerNames.push_back(plugin.name);
            }
        }
        for (const ComponentRecord &record : records)
        {
            if (std::find(
                    record.manifest.providedCapabilities.begin(),
                    record.manifest.providedCapabilities.end(),
                    capabilityName
                )
                != record.manifest.providedCapabilities.end())
            {
                providerNames.push_back(record.name);
            }
        }
        if (providerNames.size() > 1)
        {
            const auto selection = std::find_if(
                selections.begin(),
                selections.end(),
                [&capabilityName](const DependencyResolver::ProviderSelection &candidate)
                { return candidate.capabilityName == capabilityName; }
            );
            if (selection != selections.end())
            {
                const bool selectionMatches = std::find(
                                                  providerNames.begin(), providerNames.end(), selection->providerName
                                              )
                                           != providerNames.end();
                if (selectionMatches)
                {
                    providerNames.assign(1, selection->providerName);
                }
                // A selection that matches nothing stays unapplied: every
                // candidate activates and the resolver reports the stale
                // selection with its precise diagnostic.
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
                for (const std::string &requiredName : plugin.requiredCapabilities)
                {
                    worklist.push_back(requiredName);
                }
            }
        }
    }
}

// Runtime module discovery: the plugin seam of the kernel.
//
// Every shared library in the modules directory declares itself through one
// exported entry point (engine/kernel/module_api.hpp): its name, its factory
// and its static capability declaration. Reading a descriptor is uniform -
// dlopen, resolve, version check, copy the declaration - and creating an
// instance is a separate decision the demand closure owns. The composition
// root registers nothing and the kernel registers nothing: the set of
// available modules comes from the runtime layout, and the set of ACTIVE
// modules comes from what this run actually requires.
void EngineKernel::loadModulePlugins(
    std::string &report
)
{
    // Fresh scan every gather: release the libraries of a previous scan so a
    // re-gather never accumulates stale handles.
    modulePlugins.clear();

    std::filesystem::path       modulesDirectory  = configurationStore.modulesDirectory();
    const std::filesystem::path resolvedDirectory = modulesDirectory.is_absolute()
                                                      ? modulesDirectory
                                                      : executableDirectory() / modulesDirectory;

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

    std::vector<std::filesystem::path> pluginPaths;
    for (const std::filesystem::directory_entry &entry : plugin)
    {
        std::error_code statError;
        if (statError || !entry.is_regular_file())
        {
            continue;
        }
        const std::filesystem::path libraryPath = entry.path();
        const std::string           extension   = libraryPath.extension().string();
        // Both shared library extensions are accepted on every platform: the
        // modules directory is a deployment layout, and a plugin tree
        // assembled for one platform must not silently look empty when
        // inspected from the other.
        if (extension == ".dll" || extension == ".so")
        {
            pluginPaths.push_back(libraryPath);
        }
    }
    // Sorted scan order: startup behavior and reports never depend on
    // filesystem iteration order.
    std::sort(pluginPaths.begin(), pluginPaths.end());

    for (const std::filesystem::path &libraryPath : pluginPaths)
    {
        auto        library = std::make_unique<DynamicLibrary>();
        std::string loadError;
        if (!library->load(libraryPath, loadError))
        {
            report += "- " + loadError + "\n";
            continue;
        }

        const auto query = reinterpret_cast<const EngineModulePluginEntry *(*)()>(
            library->resolveSymbol("engineQueryModulePlugin")
        );
        if (query == nullptr)
        {
            report += "- '" + libraryPath.string()
                    + "' exports no engineQueryModulePlugin entry point; not a module plugin.\n";
            continue;
        }

        const EngineModulePluginEntry *pluginEntry = query();
        if (pluginEntry == nullptr || pluginEntry->abiVersion != engineModulePluginAbiVersion)
        {
            report += "- '" + libraryPath.string()
                    + "' was built against a different module plugin ABI (expected version "
                    + std::to_string(engineModulePluginAbiVersion) + "); rebuild the plugin with this kernel.\n";
            continue;
        }
        if (pluginEntry->moduleName == nullptr || pluginEntry->createModule == nullptr)
        {
            report += "- '" + libraryPath.string() + "' returns an incomplete plugin descriptor.\n";
            continue;
        }

        ModulePluginRecord record;
        record.name         = pluginEntry->moduleName;
        record.createModule = pluginEntry->createModule;
        for (std::uint32_t index = 0; index < pluginEntry->providedCapabilityCount; ++index)
        {
            if (pluginEntry->providedCapabilities != nullptr && pluginEntry->providedCapabilities[index] != nullptr)
            {
                record.providedCapabilities.emplace_back(pluginEntry->providedCapabilities[index]);
            }
        }
        for (std::uint32_t index = 0; index < pluginEntry->requiredCapabilityCount; ++index)
        {
            if (pluginEntry->requiredCapabilities != nullptr && pluginEntry->requiredCapabilities[index] != nullptr)
            {
                record.requiredCapabilities.emplace_back(pluginEntry->requiredCapabilities[index]);
            }
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

        record.library = std::move(library);
        modulePlugins.push_back(std::move(record));
    }
}

// Instantiates exactly the plugins the demand closure marked active. The
// instance manifest must agree with the static descriptor (name, provided,
// required): the descriptor decided the demand, so drift would silently
// change what the run computed. Plugins nobody required are released here -
// their binaries are unloaded and they never exist as components.
bool EngineKernel::instantiateActiveModules(
    std::string &report
)
{
    bool successful = true;
    for (const ModulePluginRecord &plugin : modulePlugins)
    {
        if (!plugin.active)
        {
            continue;
        }
        ComponentRecord record;
        record.name          = plugin.name;
        record.manifest.kind = ComponentKind::Module;
        record.ownedModule.reset(plugin.createModule());
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
        records.push_back(std::move(record));
    }

    // Release the descriptors nobody required: their binaries unload now,
    // before any construct phase. A module that never becomes a component
    // never touches a backend.
    for (ModulePluginRecord &plugin : modulePlugins)
    {
        if (!plugin.active && plugin.library)
        {
            plugin.library->release();
        }
    }
    return successful;
}

// Structural risk advisory for one active mod. The signals are generic (they
// never name or interpret a capability): running native machine code,
// requesting permissions outside the capability.* namespace (direct
// environment access such as writing files) and acquiring a broad set of
// capabilities. The engine blocks what it can enforce; the user decides
// whether to trust what it cannot.
void EngineKernel::appendModRiskWarning(
    const ModManifest &modManifest, bool isNativeMod
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
    std::string joined;
    for (const std::string &reason : reasons)
    {
        if (!joined.empty())
        {
            joined += "; ";
        }
        joined += reason;
    }
    startupWarnings_ += "warning: mod '" + modManifest.id + "' requests elevated trust: " + joined
                      + ". Review this mod before trusting it with sensitive data.\n";
}

// Resolves the hosting module of one script mod: the active component that
// provides the scripting capability, honoring the configuration's [providers]
// selection when several implementations are active. The kernel never names
// the host: the script package required the capability, the graph picked the
// provider.
EngineKernel::ComponentRecord *EngineKernel::resolveScriptHost(
    const ComponentRecord &scriptRecord, std::string &report
)
{
    const std::string              capabilityName = ScriptModHost::scriptingCapabilityName;
    std::vector<ComponentRecord *> candidates;
    for (ComponentRecord &record : records)
    {
        // Script hosting is a module surface: mod instances are consumers by
        // contract shape (ModInterface has no ScriptModHost hook).
        if (record.manifest.kind != ComponentKind::Module || !record.constructed)
        {
            continue;
        }
        if (std::find(
                record.manifest.providedCapabilities.begin(), record.manifest.providedCapabilities.end(), capabilityName
            )
            != record.manifest.providedCapabilities.end())
        {
            candidates.push_back(&record);
        }
    }
    if (candidates.empty())
    {
        report += "- script mod '" + scriptRecord.name + "' cannot be hosted: no active module provides the '"
                + capabilityName + "' capability it requires.\n";
        return nullptr;
    }
    if (candidates.size() > 1)
    {
        const std::vector<DependencyResolver::ProviderSelection> selections = configurationStore.providerSelections();
        const auto                                               selection  = std::find_if(
            selections.begin(),
            selections.end(),
            [&capabilityName](const DependencyResolver::ProviderSelection &candidate)
            { return candidate.capabilityName == capabilityName; }
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
    const std::vector<ComponentManifest> &manifests, std::string &report
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
    const std::vector<DependencyResolver::ResolutionDiagnostic> &diagnostics, std::string &report
) const
{
    report += "engine startup failed with " + std::to_string(diagnostics.size()) + " problem(s):\n";
    for (const DependencyResolver::ResolutionDiagnostic &diagnostic : diagnostics)
    {
        report += "  - " + diagnostic.message + "\n";
    }
}

bool EngineKernel::validate(
    std::string &report
)
{
    report.clear();
    if (!prepareComponents(report))
    {
        return false;
    }

    // Permission denials from the configuration are precise startup problems.
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

    // Known-key registry: the assembly's own sections plus every component's
    // declared key surface. A malformed declaration is a startup problem -
    // it would silently weaken the unknown-key warnings otherwise.
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

    if (!report.empty())
    {
        // Non fatal diagnostics accumulated: denied permissions, malformed
        // key declarations.
        return false;
    }

    // Warnings never fail validation and never fail a run: they are appended
    // after the failure check so a warning stays a warning.
    report += deferredConfigurationWarnings;
    report += startupWarnings_;

    std::string                                resolutionReport;
    const DependencyResolver::ResolutionResult resolution = resolveGraph(componentManifests(), resolutionReport);
    report += resolutionReport;
    if (!resolution.successful)
    {
        return false;
    }

    // Unknown-key warnings never fail validation: a key nobody reads is dead
    // configuration (usually a typo), and the report should say so - but the
    // graph above it is fine. Normal runs stay quiet; --validate is the place
    // where the author is looking.
    for (const std::string &warning : configurationStore.unknownKeyWarnings())
    {
        report += "warning: " + warning + "\n";
    }
    return true;
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

        // Script mods have no instance of their own: the record is a proxy in
        // the dependency graph, and the actual script runtime lives in the
        // module that provides the scripting capability. The resolver
        // guarantees that provider constructed first (the script's
        // requirement is a graph edge).
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
                        + "' provides the '" + std::string(ScriptModHost::scriptingCapabilityName)
                        + "' capability but does not host script mods.\n";
                return false;
            }
            try
            {
                scriptHost->hostScriptMod(record->scriptMod);
            }
            catch (const std::exception &error)
            {
                report += "- construct failed for script mod '" + record->name + "': " + error.what() + "\n";
                return false;
            }
            record->constructed = true;
            continue;
        }

        // Heap anchored: the component may cache this context for its whole
        // lifetime (event publishing from frame loops or input callbacks), so
        // it must not live in this loop's stack frame.
        record->context = std::make_unique<ComponentContext>(
            bus, capabilityRegistry_, permissionSystem_, configurationStore, record->name, &demandedCapabilities
        );
        ComponentContext &context = *record->context;
        try
        {
            if (record->moduleInstance != nullptr)
            {
                record->moduleInstance->construct(context);
            }
            else if (record->modInstance != nullptr)
            {
                record->modInstance->construct(context);
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
            else if (record->modInstance != nullptr)
            {
                capabilityPointer = record->modInstance->capabilityPointerFor(capabilityName);
            }
            if (capabilityPointer == nullptr)
            {
                report += "- " + describeComponentKind(record->manifest.kind) + " '" + record->name
                        + "' declares capability '" + capabilityName + "' but returned no implementation.\n";
                return false;
            }
            try
            {
                capabilityRegistry_.registerCapability(capabilityName, record->name, capabilityPointer);
            }
            catch (const std::logic_error &error)
            {
                report += "- " + std::string(error.what()) + "\n";
                return false;
            }
        }
        record->constructed = true;
    }
    return true;
}

bool EngineKernel::construct(
    std::string &report
)
{
    report.clear();
    if (constructed)
    {
        return true;
    }
    if (!prepareComponents(report))
    {
        return false;
    }
    // The same permission pass as validate: configuration denials become
    // precise startup problems before any construct callback runs.
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

    std::string                                resolutionReport;
    const DependencyResolver::ResolutionResult resolution = resolveGraph(componentManifests(), resolutionReport);
    report += resolutionReport;
    if (!resolution.successful)
    {
        return false;
    }

    if (!runConstructPhase(resolution, componentManifests(), report))
    {
        stop();
        report = "engine startup failed:\n" + report;
        return false;
    }
    constructed = true;
    return true;
}

void EngineKernel::start()
{
    if (!constructed || started)
    {
        throw std::runtime_error("engine kernel: start requires a completed construct phase");
    }
    for (ComponentRecord &record : records)
    {
        if (!record.constructed || record.started)
        {
            continue;
        }
        if (record.moduleInstance != nullptr)
        {
            record.moduleInstance->start();
        }
        else if (record.modInstance != nullptr)
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
    if (started)
    {
        bus.publish(EngineShutdownStartedEvent{});
    }
    for (auto iterator = records.rbegin(); iterator != records.rend(); ++iterator)
    {
        ComponentRecord &record = *iterator;
        if (record.started && record.moduleInstance != nullptr)
        {
            record.moduleInstance->stop();
        }
        if (record.started && record.modInstance != nullptr)
        {
            record.modInstance->stop();
        }
        // Shared library mods own their destruction function; destroying them
        // through delete here would use the engine allocator on mod memory.
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

    records.clear();
    componentsGathered = false;
}

} // namespace engine
