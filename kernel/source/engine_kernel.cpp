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

const std::string& hostContextName()
{
    static const std::string name = "<host>";
    return name;
}

std::filesystem::path resolveAgainstExecutableDirectory(const std::filesystem::path& path)
{
    if (path.is_absolute())
    {
        return path;
    }
    return executableDirectory() / path;
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
std::vector<std::string> effectivePermissions(const ComponentManifest& manifest)
{
    std::set<std::string> unique(manifest.requiredPermissions.begin(), manifest.requiredPermissions.end());
    for (const std::string& capabilityName : manifest.requiredCapabilities)
    {
        unique.insert(std::string(capabilityPermissionPrefix) + capabilityName);
    }
    return std::vector<std::string>(unique.begin(), unique.end());
}

} // namespace

void EngineKernel::setConfigurationFile(const std::filesystem::path& configurationPath)
{
    configurationFilePath = configurationPath;
    configurationLoaded = false;
    configurationStore.reset();
}

void EngineKernel::setSceneFile(const std::filesystem::path& scenePath)
{
    sceneFilePath = scenePath;
    configurationLoaded = false;
    configurationStore.reset();
}

bool EngineKernel::addCommandLineOverride(const std::string& assignment, std::string& error)
{
    // Parsed (not applied) immediately: a malformed --set must fail at the
    // composition root with exit code 2, before any backend starts.
    return configurationStore.stageCommandLineOverride(assignment, error);
}

void EngineKernel::setModsDirectory(const std::filesystem::path& modsDirectory)
{
    explicitModsDirectory = modsDirectory;
    componentsGathered = false;
}

void EngineKernel::registerModuleFactory(const std::string& moduleName, ModuleFactory factory)
{
    moduleFactories[moduleName] = std::move(factory);
}

void EngineKernel::registerModFactory(const std::string& modName, ModFactory factory)
{
    modFactories[modName] = std::move(factory);
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
    return std::make_unique<ComponentContext>(bus, capabilityRegistry_, permissionSystem_, configurationStore,
                                              hostContextName());
}

bool EngineKernel::loadConfiguration(std::string& report)
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
bool EngineKernel::loadModuleDefaults(std::string& report)
{
    const std::filesystem::path defaultsDirectory =
        resolveAgainstExecutableDirectory(ConfigurationStore::moduleDefaultsDirectory);
    if (!std::filesystem::exists(defaultsDirectory) || !std::filesystem::is_directory(defaultsDirectory))
    {
        return true; // no module ships defaults: a valid state
    }

    std::vector<std::filesystem::path> moduleDirectories;
    for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(defaultsDirectory))
    {
        if (entry.is_directory() &&
            std::filesystem::exists(entry.path() / ConfigurationStore::moduleDefaultsFileName))
        {
            moduleDirectories.push_back(entry.path());
        }
    }
    std::sort(moduleDirectories.begin(), moduleDirectories.end());

    for (const std::filesystem::path& moduleDirectory : moduleDirectories)
    {
        std::string defaultsError;
        if (!configurationStore.mergeDefaultsFile(moduleDirectory / ConfigurationStore::moduleDefaultsFileName,
                                                  ConfigProvenance::Layer::ModuleDefaults,
                                                  moduleDirectory.filename().string(), defaultsError))
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
bool EngineKernel::finishConfiguration(std::string& report)
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
        const ModSceneClaim& claim = modSceneClaims.front();
        if (!std::filesystem::exists(claim.sceneFile))
        {
            report += "- mod '" + claim.ownerId + "' declares scene file '" + claim.sceneFile.string() +
                      "' which does not exist.\n";
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
        report += "- the world is ambiguous: " + std::to_string(modSceneClaims.size()) +
                  " active mods declare scene files";
        for (const ModSceneClaim& claim : modSceneClaims)
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
        for (const ModSceneClaim& claim : modSceneClaims)
        {
            deferredConfigurationWarnings += "warning: mod '" + claim.ownerId + "' declares scene file '" +
                                             claim.sceneFile.string() +
                                             "' but the configuration defines its own [scene]; the mod scene is "
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
    bus.setLatencyRecordingEnabled(bus.latencyRecordingEnabled() ||
                                   configurationStore
                                       .booleanValue(ConfigurationStore::diagnosticsSectionName,
                                                     ConfigurationStore::diagnosticsLatencyReportKey)
                                       .value_or(false));
    return true;
}

bool EngineKernel::prepareComponents(std::string& report)
{
    return loadConfiguration(report) && gatherComponents(report) && finishConfiguration(report);
}

std::vector<std::pair<const EngineKernel::ComponentRecord*, std::string>> EngineKernel::applyPermissionPass()
{
    std::vector<std::pair<const ComponentRecord*, std::string>> denied;
    for (const auto& denial : configurationStore.permissionDenials())
    {
        permissionSystem_.denyPermission(denial.first, denial.second);
    }
    for (const ComponentRecord& record : records)
    {
        const std::vector<std::string> grants = effectivePermissions(record.manifest);
        permissionSystem_.declarePermissions(record.name, grants);
        const std::vector<std::string> missing = permissionSystem_.missingPermissions(record.name, grants);
        for (const std::string& permission : missing)
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
    for (const ComponentRecord& record : records)
    {
        manifests.push_back(record.manifest);
    }
    return manifests;
}

EngineKernel::ComponentRecord* EngineKernel::findRecord(const std::string& name)
{
    for (ComponentRecord& candidate : records)
    {
        if (candidate.name == name)
        {
            return &candidate;
        }
    }
    return nullptr;
}

// Builds one record per component: module plugins from the modules directory,
// in-process mod factories, shared library mods from the configuration, and
// validates that every module listed in the configuration is actually
// available in this run.
bool EngineKernel::gatherComponents(std::string& report)
{
    if (componentsGathered)
    {
        return true;
    }

    // Runtime module plugins first: every plugin binary in the modules
    // directory registers a factory, exactly like the compile-time
    // registrations the composition root no longer makes. Load failures are
    // reported and skipped; a configured module backed by no factory then
    // reports below through the availability diagnostic.
    loadModulePlugins(report);

    const std::vector<std::string> configuredModules = configurationStore.moduleNames();

    std::vector<std::string> knownModuleNames;
    knownModuleNames.reserve(moduleFactories.size());
    for (const auto& entry : moduleFactories)
    {
        knownModuleNames.push_back(entry.first);
    }

    for (const std::string& moduleName : configuredModules)
    {
        const auto factory = moduleFactories.find(moduleName);
        if (factory == moduleFactories.end())
        {
            continue; // reported below against the compiled module inventory
        }
        ComponentRecord record;
        record.name = moduleName;
        record.manifest.kind = ComponentKind::Module;
        record.ownedModule = factory->second();
        record.moduleInstance = record.ownedModule.get();
        if (record.ownedModule == nullptr)
        {
            report += "- module factory for '" + moduleName + "' returned no instance.\n";
            return false;
        }
        record.manifest = record.moduleInstance->manifest();
        if (record.manifest.name != moduleName)
        {
            report += "- module '" + moduleName + "' reports the manifest name '" + record.manifest.name +
                      "'; the configuration name and the manifest name must match.\n";
            return false;
        }
        records.push_back(std::move(record));
    }

    // Modules requested in the configuration but backed by no factory: the
    // plugin binary is missing from the modules directory (or failed to load
    // above), the module was compiled out through its build option, or the
    // configuration name is wrong.
    for (const std::string& moduleName : configuredModules)
    {
        const bool registered = moduleFactories.find(moduleName) != moduleFactories.end();
        if (!registered)
        {
            report += "- module '" + moduleName +
                      "' is listed in the configuration but is not available in this run (its plugin is missing "
                      "from the modules directory, compiled out through its build option, or the name is wrong).\n";
        }
    }

    // Mod packages discovered in the mods directory, one mod.toml per package.
    gatherModPackages(report);

    // In-process mod factories (tests and statically linked mods).
    for (const auto& entry : modFactories)
    {
        ComponentRecord record;
        record.name = entry.first;
        record.manifest.kind = ComponentKind::SharedLibraryMod;
        record.ownedMod = entry.second();
        record.modInstance = record.ownedMod.get();
        if (record.modInstance == nullptr)
        {
            report += "- mod factory for '" + entry.first + "' returned no instance.\n";
            return false;
        }
        record.manifest = record.modInstance->manifest();
        records.push_back(std::move(record));
    }

    componentsGathered = true;
    return true;
}

// Turns the packages discovered on disk into component records. The
// filesystem scan and the native library loading live in mod_discovery.cpp;
// this phase owns the kernel policy: scene claims, shipped defaults, record
// wiring for native and script mods.
void EngineKernel::gatherModPackages(std::string& report)
{
    // Fresh scan state every call: the claims are re-collected from the
    // current packages, never carried over from an earlier gather.
    modSceneClaims.clear();

    std::filesystem::path modsDirectory = explicitModsDirectory;
    if (modsDirectory.empty())
    {
        modsDirectory = configurationStore.modDirectory();
    }
    const std::filesystem::path resolvedDirectory = resolveAgainstExecutableDirectory(modsDirectory);
    const std::vector<DiscoveredModPackage> packages =
        discoverModPackages(resolvedDirectory, configurationStore.disabledMods(), report);

    for (const DiscoveredModPackage& package : packages)
    {
        const ModManifest& modManifest = package.manifest;

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
            claim.ownerId = modManifest.id;
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
            if (!configurationStore.mergeDefaultsFile(package.defaultsFile, ConfigProvenance::Layer::ModDefaults,
                                                      modManifest.directory.filename().string(), defaultsError))
            {
                report += "- " + defaultsError + "\n";
                continue;
            }
        }

        if (modManifest.isNative())
        {
            ComponentRecord record;
            record.name = modManifest.id;
            record.manifest.kind = ComponentKind::SharedLibraryMod;
            NativeModLoad load;
            if (!loadNativeModEntry(package, load, report))
            {
                continue;
            }
            record.library = std::move(load.library);
            record.modEntryTable = load.entryTable;
            record.modInstance = load.modInstance;
            record.manifest = record.modInstance->manifest();
            // The package manifest is authoritative for identity and wiring;
            // the compiled manifest only had to agree on the name.
            record.manifest.name = modManifest.id;
            record.manifest.requiredCapabilities = modManifest.capabilities;
            record.manifest.requiredPermissions = modManifest.permissions;
            record.manifest.requiredComponents = modManifest.requiresComponents;
            record.manifest.knownConfigKeys = modManifest.configKeys;
            records.push_back(std::move(record));
        }
        else
        {
            // Script mod: hosted by a module through the ScriptModHost
            // contract. The record exists so the resolver and the permission
            // system see the script's capabilities and permissions like any
            // other component's.
            const bool hostAvailable = moduleFactories.find(modManifest.host) != moduleFactories.end();
            if (!hostAvailable)
            {
                report += "- script mod '" + modManifest.id + "' declares host '" + modManifest.host +
                          "' which is not available in this build (compiled out through its build option, "
                          "or the name is wrong).\n";
                continue;
            }
            ComponentRecord record;
            record.name = modManifest.id;
            record.manifest.kind = ComponentKind::ScriptMod;
            record.manifest.name = modManifest.id;
            record.manifest.requiredCapabilities = modManifest.capabilities;
            record.manifest.requiredPermissions = modManifest.permissions;
            record.manifest.requiredComponents = modManifest.requiresComponents;
            record.manifest.knownConfigKeys = modManifest.configKeys;
            record.manifest.requiredComponents.push_back(modManifest.host); // always ordered after its host
            record.scriptHostName = modManifest.host;
            record.scriptMod.modId = modManifest.id;
            record.scriptMod.scriptPath = modManifest.directory / modManifest.entry;
            records.push_back(std::move(record));
        }
    }
}

DependencyResolver::ResolutionResult EngineKernel::resolveGraph(const std::vector<ComponentManifest>& manifests,
                                                                std::string& report) const
{
    DependencyResolver resolver(manifests, configurationStore.providerSelections());
    const DependencyResolver::ResolutionResult resolution = resolver.resolve();
    if (!resolution.successful)
    {
        appendDiagnosticsReport(resolution.diagnostics, report);
    }
    return resolution;
}

void EngineKernel::appendDiagnosticsReport(const std::vector<DependencyResolver::ResolutionDiagnostic>& diagnostics,
                                           std::string& report) const
{
    report += "engine startup failed with " + std::to_string(diagnostics.size()) + " problem(s):\n";
    for (const DependencyResolver::ResolutionDiagnostic& diagnostic : diagnostics)
    {
        report += "  - " + diagnostic.message + "\n";
    }
}

bool EngineKernel::validate(std::string& report)
{
    report.clear();
    if (!prepareComponents(report))
    {
        return false;
    }

    // Permission denials from the configuration are precise startup problems.
    std::vector<DependencyResolver::ResolutionDiagnostic> permissionDiagnostics;
    for (const auto& [record, permission] : applyPermissionPass())
    {
        DependencyResolver::ResolutionDiagnostic diagnostic;
        diagnostic.kind = DependencyResolver::ResolutionDiagnostic::Kind::UnknownCapability;
        diagnostic.message = describeComponentKind(record->manifest.kind) + " '" + record->name +
                             "' requires permission '" + permission + "' which the configuration denies.";
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
    for (const ComponentRecord& record : records)
    {
        for (const std::string& configKey : record.manifest.knownConfigKeys)
        {
            if (!configurationStore.declareKnownConfigKey(configKey))
            {
                report += "- " + describeComponentKind(record.manifest.kind) + " '" + record.name +
                          "' declares the malformed configuration key '" + configKey +
                          "'; keys are '<section>.<dotted path>' of bare key segments.\n";
            }
        }
    }

    if (!report.empty())
    {
        // Non fatal diagnostics accumulated: modules requested in the
        // configuration but compiled out of this build, denied permissions,
        // malformed key declarations.
        return false;
    }

    // Warnings never fail validation and never fail a run: they are appended
    // after the failure check so a warning stays a warning.
    report += deferredConfigurationWarnings;

    std::string resolutionReport;
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
    for (const std::string& warning : configurationStore.unknownKeyWarnings())
    {
        report += "warning: " + warning + "\n";
    }
    return true;
}

bool EngineKernel::runConstructPhase(const DependencyResolver::ResolutionResult& resolution,
                                     const std::vector<ComponentManifest>& manifests, std::string& report)
{
    for (const std::size_t manifestIndex : resolution.constructionOrder)
    {
        const ComponentManifest& manifest = manifests[manifestIndex];
        ComponentRecord* record = findRecord(manifest.name);
        if (record == nullptr || record->constructed)
        {
            continue;
        }

        // Script mods have no instance of their own: the record is a proxy in
        // the dependency graph, and the actual script runtime lives in the
        // hosting module. The resolver guarantees the host constructed first.
        if (record->manifest.kind == ComponentKind::ScriptMod)
        {
            ComponentRecord* hostRecord = findRecord(record->scriptHostName);
            if (hostRecord == nullptr || !hostRecord->constructed)
            {
                report += "- script mod '" + record->name + "' cannot be hosted: module '" +
                          record->scriptHostName + "' did not construct.\n";
                return false;
            }
            ScriptModHost* scriptHost = hostRecord->moduleInstance->scriptModHost();
            if (scriptHost == nullptr)
            {
                report += "- script mod '" + record->name + "' declares host '" + record->scriptHostName +
                          "' which does not host script mods.\n";
                return false;
            }
            try
            {
                scriptHost->hostScriptMod(record->scriptMod);
            }
            catch (const std::exception& error)
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
        record->context = std::make_unique<ComponentContext>(bus, capabilityRegistry_, permissionSystem_,
                                                              configurationStore, record->name);
        ComponentContext& context = *record->context;
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
        catch (const std::exception& error)
        {
            report += "- construct failed for " + describeComponentKind(record->manifest.kind) + " '" +
                      record->name + "': " + error.what() + "\n";
            return false;
        }

        for (const std::string& capabilityName : record->manifest.providedCapabilities)
        {
            void* capabilityPointer = nullptr;
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
                report += "- " + describeComponentKind(record->manifest.kind) + " '" + record->name +
                          "' declares capability '" + capabilityName + "' but returned no implementation.\n";
                return false;
            }
            try
            {
                capabilityRegistry_.registerCapability(capabilityName, record->name, capabilityPointer);
            }
            catch (const std::logic_error& error)
            {
                report += "- " + std::string(error.what()) + "\n";
                return false;
            }
        }
        record->constructed = true;
    }
    return true;
}

bool EngineKernel::construct(std::string& report)
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
    for (const auto& [record, permission] : applyPermissionPass())
    {
        report += "- " + describeComponentKind(record->manifest.kind) + " '" + record->name +
                  "' requires permission '" + permission + "' which the configuration denies.\n";
    }
    if (!report.empty())
    {
        report = "engine startup failed:\n" + report;
        return false;
    }

    std::string resolutionReport;
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
    for (ComponentRecord& record : records)
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
        ComponentRecord& record = *iterator;
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
        record.started = false;
        record.constructed = false;
    }
    started = false;
    constructed = false;

    records.clear();
    componentsGathered = false;
}

} // namespace engine
