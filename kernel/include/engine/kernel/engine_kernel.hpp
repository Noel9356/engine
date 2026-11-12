// EngineKernel public interface: the engine's composition root.
//
// The kernel loads module .so libraries, discovers plugin packages,
// resolves the capability dependency graph, enforces permissions and the
// sha256 trust gate, and owns the event bus, task scheduler and
// main-thread queue that every participant receives.

#ifndef ENGINE_KERNEL_ENGINE_KERNEL_HPP
#define ENGINE_KERNEL_ENGINE_KERNEL_HPP

#include <engine/kernel/configuration.hpp>
#include <engine/kernel/dependency_resolver.hpp>
#include <engine/kernel/dynamic_library.hpp>
#include <engine/kernel/event_bus.hpp>
#include <engine/kernel/plugin_api.hpp>
#include <engine/kernel/plugin_discovery.hpp>
#include <engine/kernel/main_thread_task_queue.hpp>
#include <engine/kernel/module_descriptor.hpp>
#include <engine/kernel/module_descriptor_cache.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/kernel/permission_system.hpp>
#include <engine/kernel/profiling.hpp>
#include <engine/kernel/task_scheduler.hpp>

#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace engine
{

// Published on the event bus after every participant's start() ran.
struct EngineStartupCompletedEvent
{};

// Published before the kernel stops participants; their stop() calls run
// after the dispatch returns.
struct EngineShutdownStartedEvent
{};

// One participant flagged by the trust gate. reasons lists why it needs
// trust; sha256 is the content hash of its binary/scripts (may be empty).
struct ParticipantTrustAdvisory
{
    ParticipantKind kind = ParticipantKind::EngineModule;

    std::string name    = {};
    std::string version = {};

    std::string sha256 = {};

    std::vector<std::string> reasons = {};
};

// Called by the trust gate when untrusted participants would abort
// startup; return true to allow this run only (nothing is persisted).
using TrustConsentHandler = std::function<bool(const std::vector<ParticipantTrustAdvisory> &untrustedAdvisories)>;

// Kernel internals kept out of the public header.
class CapabilityProviderIndex;
class TrustGate;

// Drives the validate/construct/start/stop phases. Configuration setters
// may only be called before construct(); they mark staged state stale so
// the next phase rebuilds from scratch.
class EngineKernel final : public CapabilityResolver
{
public:
    EngineKernel();
    ~EngineKernel();

    EngineKernel(const EngineKernel &)            = delete;
    EngineKernel &operator=(const EngineKernel &) = delete;

    // The setters below only stage inputs and invalidate what they affect;
    // real work happens in validate/construct.
    void setConfigurationFile(const std::filesystem::path &configurationPath);

    void setConfigurationProfileFile(const std::filesystem::path &profilePath);

    void addConfigurationProfileFile(const std::filesystem::path &profilePath);

    void setSceneFile(const std::filesystem::path &scenePath);

    // Stages a SECTION.KEY=VALUE override applied above every config layer;
    // returns false and fills error for malformed assignments.
    bool addCommandLineOverride(const std::string &assignment, std::string &error);

    void setPluginsDirectory(const std::filesystem::path &pluginsDirectory);
    void addPluginsDirectory(const std::filesystem::path &pluginsDirectory);

    // Capabilities the application itself requires; the app joins the graph
    // as a synthetic participant named "<app>".
    void setApplicationRequirements(const std::vector<std::string> &capabilities);

    // Restricts the run to these plugin ids; clearActivePlugins() restores
    // "all discovered plugins".
    void setActivePlugins(const std::vector<std::string> &pluginIds);

    void clearActivePlugins();

    // Dry run: loads configuration, gathers participants, checks permissions
    // and resolves the graph without executing any module code. Appends a
    // human-readable report; returns false when problems were found.
    bool validate(std::string &report);

    // Full startup: enforces the trust gate (may consult the consent
    // handler), loads module libraries and constructs participants in
    // dependency order. Returns false and fills report on failure.
    bool construct(std::string &report);
    // start() runs participant start() in construct order and publishes
    // EngineStartupCompletedEvent; stop() runs stop() in reverse order.
    void start();
    void stop();

    // Hot-reloads engine.toml/profiles/defaults after construct; a failed
    // pre-parse leaves the running configuration untouched.
    bool reloadConfiguration(std::string &report);

    // Human-readable participant listing (provides/requires/permissions/
    // state) plus the event-bus subscription report.
    std::string participantGraphReport() const;

    bool isStarted() const
    {
        return started;
    }

    EventBus &eventBus()
    {
        return bus;
    }

    MainThreadTaskQueue &taskQueue()
    {
        return mainThreadTaskQueue_;
    }

    TaskScheduler &taskScheduler()
    {
        return taskScheduler_;
    }

    PerformanceProfiler &profiler()
    {
        return profiler_;
    }

    // CapabilityResolver implementation; the requester kind decides module-
    // vs-plugin visibility. Throws on ambiguity or missing providers; the try*/
    // has* variants report instead of throwing.
    void *resolveCapabilityPointer(
        const std::string &requesterName, ParticipantKind requesterKind, const std::string &capabilityName) override;

    std::vector<void *> resolveEachCapabilityPointer(
        const std::string &requesterName, ParticipantKind requesterKind, const std::string &capabilityName) override;

    bool hasSingleCapabilityProvider(const std::string &capabilityName, ParticipantKind requesterKind) override;

    void *tryResolveCapabilityPointer(
        const std::string &requesterName, ParticipantKind requesterKind, const std::string &capabilityName) override;

    // Read-only view of the merged configuration (engine.toml, profiles,
    // defaults, overrides).
    const ConfigurationStore &configuration() const
    {
        return configurationStore;
    }

    // Non-fatal notes gathered while loading libraries/plugins (trust warnings,
    // skipped packages).
    const std::string &startupWarnings() const
    {
        return startupWarnings_;
    }

    // Advisories from the last gather; see ParticipantTrustAdvisory.
    const std::vector<ParticipantTrustAdvisory> &trustAdvisories() const;

    void setTrustConsentHandler(TrustConsentHandler handler);

    // Builds the application's own ParticipantContext ("<app>") with every
    // permission granted. Valid after construct().
    std::unique_ptr<ParticipantContext> createAppContext();

private:
    // One loaded participant: its library, manifest, live instance and
    // lifecycle state.
    struct ParticipantRecord
    {
        // Stays loaded while the participant lives (its symbols must resolve).
        std::unique_ptr<DynamicLibrary> library = nullptr;

        std::unique_ptr<ParticipantContext> context = nullptr;
        // Native plugins destroy their instance through this table (never delete).
        const EnginePluginEntryTable *pluginEntryTable = nullptr;
        std::string                    name              = {};
        ParticipantManifest            manifest          = {};
        // Exactly one of moduleInstance/pluginInstance is set for native
        // participants; script plugins own neither.
        ModuleInterface  *moduleInstance  = nullptr;
        PluginInterface *pluginInstance = nullptr;
        // Owning storage for module instances (moduleInstance aliases it).
        std::unique_ptr<ModuleInterface>  ownedModule  = nullptr;
        std::unique_ptr<PluginInterface> ownedPlugin = nullptr;

        // Script plugins only: the hosted-plugin description and resolved host name.
        ScriptPluginDescription scriptPlugin      = {};
        std::string              scriptHostName = {};
        bool                     constructed    = false;
        bool                     started        = false;
    };

    // A scene file claimed by a plugin via plugin.toml "scene"; adopted when the
    // configuration defines no [scene] of its own.
    struct PluginSceneClaim
    {
        std::filesystem::path sceneFile;
        std::string           ownerId;
    };

    // Configuration pipeline (see engine_kernel/configuration.cpp).
    bool loadConfiguration(std::string &report);

    bool loadModuleDefaults(std::string &report);

    // Scene resolution, staged overrides and diagnostics switches.
    bool finishConfiguration(std::string &report);

    // Registers assembly + participant config keys for unknown-key warnings.
    bool declareKnownKeyRegistry(std::string &report);

    // enforceTrust=false is the validate dry run (no module code executes);
    // enforceTrust=true is the construct path.
    bool prepareParticipants(std::string &report, bool enforceTrust);

    // Collects records from modules + plugins; enforceTrust selects dry-run vs
    // real loading.
    bool gatherParticipants(std::string &report, bool enforceTrust);
    // Scans the plugin roots and creates plugin records (no native .so loads yet).
    void gatherPluginPackages(std::string &report);

    // Marks module libraries active when anything in the current (transitive)
    // demand set needs a capability they provide.
    void computeDemandClosure();

    // Scans the modules directory; descriptors come from the cache or
    // module.toml sidecars; no code executes.
    void loadModuleLibraries(std::string &report);

    // Trust-gated scan of libraries without any descriptor.
    void resolveUndescribedModuleLibraries(std::string &report, bool enforceTrust);

    // Queries the library's entry table (executes code); yields a
    // descriptor-only record.
    bool scanUndescribedModuleLibraryDescriptor(const UndescribedModuleLibrary &candidate,
        ModuleLibraryRecord &outRecord, ModuleDescriptorCacheEntry &outCacheEntry, std::string &report);

    // Full load for instantiation; keeps the .so resident.
    bool loadModuleLibrary(ModuleLibraryRecord &module, std::string &report);

    // Creates instances and cross-checks compiled manifests vs descriptors.
    bool instantiateActiveModules(std::string &report);

    // validate path: descriptor-only records, no instances.
    void appendDeclaredModuleRecords();

    // Deferred native plugin loading (after the trust gate passes).
    void loadNativePluginLibraries(std::string &report);

    // Grants each record's effective permissions and returns the (record,
    // permission) pairs the configuration denies under [permissions].
    std::vector<std::pair<const ParticipantRecord *, std::string>> applyPermissionPass();
    std::vector<ParticipantManifest>                               participantManifests() const;
    ParticipantRecord                                             *findRecord(const std::string &name);

    // Finds the module hosting a script plugin; [providers] breaks ambiguity.
    ParticipantRecord *resolveScriptHost(const ParticipantRecord &scriptRecord, std::string &report);

    // Runs the DependencyResolver with the [providers] selections; appends a
    // diagnostics report when resolution fails.
    DependencyResolver::ResolutionResult resolveGraph(
        const std::vector<ParticipantManifest> &manifests, std::string &report) const;
    void appendDiagnosticsReport(
        const std::vector<DependencyResolver::ResolutionDiagnostic> &diagnostics, std::string &report) const;

    // Calls construct() in resolution order (script plugins through their host
    // first) and registers each participant's providers; false aborts the
    // startup.
    bool runConstructPhase(const DependencyResolver::ResolutionResult &resolution,
        const std::vector<ParticipantManifest> &manifests, std::string &report);

    // Staged inputs; see the setters above.
    std::vector<std::string> applicationRequiredCapabilities;

    std::optional<std::vector<std::string>> applicationActivePlugins;

    // Staged paths: engine.toml, optional profiles, scene file, plugin roots.
    std::filesystem::path              configurationFilePath;
    std::filesystem::path              configurationProfileFilePath;
    std::vector<std::filesystem::path> configurationProfileFilePaths;
    std::filesystem::path              sceneFilePath;
    std::filesystem::path              explicitPluginsDirectory;

    std::vector<std::filesystem::path> explicitExtraPluginsDirectories;

    // Scene claims from active plugins (see PluginSceneClaim).
    std::vector<PluginSceneClaim> pluginSceneClaims;

    // Plugin config.toml files merged as the PluginDefaults layer; kept for reload.
    std::vector<std::pair<std::filesystem::path, std::string>> pluginDefaultsFiles;

    // True when engine.toml itself carries a [scene] section.
    bool assemblyDefinesScene = false;

    // Warnings deferred during gather (e.g. ignored plugin scenes), shown once
    // the run is known to succeed.
    std::string deferredConfigurationWarnings;
    std::string startupWarnings_;

    // Native plugin packages held until the trust gate passes; only then are
    // their .so libraries loaded and entry tables queried.
    std::vector<DiscoveredPluginPackage> pendingNativePluginPackages_;

    // Libraries with no module.toml sidecar and no cached descriptor;
    // identifying them would require executing their code.
    std::vector<UndescribedModuleLibrary> undescribedModuleLibraries_;

    // Guards a double load on construct re-entry.
    bool nativePluginLibrariesLoaded_ = false;

    // Core services; references are handed to every ParticipantContext.
    ConfigurationStore configurationStore;
    EventBus           bus;
    PermissionSystem   permissionSystem_;

    MainThreadTaskQueue mainThreadTaskQueue_;
    PerformanceProfiler profiler_;
    TaskScheduler       taskScheduler_;

    // Heap-allocated internals (kept out of the public header).
    std::unique_ptr<TrustGate>               trustGate_;
    std::unique_ptr<CapabilityProviderIndex> capabilityProviderIndex_;

    // Discovered module libraries (descriptor + hash) and live participants.
    std::vector<ModuleLibraryRecord> moduleLibraries;

    std::vector<ParticipantRecord> records;

    // Construct order; stop() iterates it in reverse.
    std::vector<ParticipantRecord *> participantStartOrder;
    // Lifecycle flags; the setters above clear the ones they invalidate.
    bool configurationLoaded  = false;
    bool participantsGathered = false;
    bool constructed          = false;
    bool started              = false;
};

} // namespace engine

#endif
