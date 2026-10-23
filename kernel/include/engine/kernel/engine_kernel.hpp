#ifndef ENGINE_KERNEL_ENGINE_KERNEL_HPP
#define ENGINE_KERNEL_ENGINE_KERNEL_HPP

#include "capability_registry.hpp"
#include "component_interface.hpp"
#include "component_manifest.hpp"
#include "configuration.hpp"
#include "dependency_resolver.hpp"
#include "dynamic_library.hpp"
#include "event_bus.hpp"
#include "mod_api.hpp"
#include "mod_manifest.hpp"
#include "permission_system.hpp"

#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine
{

// Kernel level lifecycle events. They let modules and mods observe the engine
// itself without depending on any other module.
struct EngineStartupCompletedEvent
{
};

struct EngineShutdownStartedEvent
{
};

// Owns the minimal kernel: configuration, component graph resolution, the
// permission system, the capability registry and the event bus, and drives the
// construct / start / stop lifecycle in dependency order.
//
// The kernel deliberately knows nothing about windows, rendering or scripting:
// those are capability providers registered from outside. Startup problems are
// collected into one precise report instead of failing with the first error.
class EngineKernel final
{
public:
    using ModuleFactory = std::function<std::unique_ptr<ModuleInterface>()>;
    using ModFactory    = std::function<std::unique_ptr<ModInterface>()>;

    EngineKernel() = default;
    ~EngineKernel();

    EngineKernel(const EngineKernel &)            = delete;
    EngineKernel &operator=(const EngineKernel &) = delete;

    // Configuration file path, relative to the executable directory when not
    // absolute. Loading happens during validate/construct.
    void setConfigurationFile(const std::filesystem::path &configurationPath);

    // Scene file for --scene: replaces the assembly configuration's [scene]
    // section wholesale once every layer has merged (the file may define only
    // [scene]). Relative paths resolve against the executable directory, like
    // the configuration file.
    void setSceneFile(const std::filesystem::path &scenePath);

    // --set: stages one "section.path=value" override. The assignment is
    // parsed immediately so a malformed --set is a usage error before any
    // backend starts; the value is applied after every configuration layer
    // has merged, because the command line is the highest layer. Returns
    // false with error filled in when the assignment is malformed.
    bool addCommandLineOverride(const std::string &assignment, std::string &error);

    // Directory that is scanned for mod packages (one subdirectory with a
    // mod.toml each), relative to the executable directory when not absolute.
    // Default: "mods". The configuration key [engine].mod_directory overrides
    // this default; an explicit call here overrides both.
    void setModsDirectory(const std::filesystem::path &modsDirectory);

    // Registers a factory for one compiled module. The name must match the
    // module name used in the configuration.
    void registerModuleFactory(const std::string &moduleName, ModuleFactory factory);

    // Registers an in-process mod factory (used by tests and by statically
    // linked mods). Shared library mods come from the configuration instead.
    void registerModFactory(const std::string &modName, ModFactory factory);

    // Dry run: loads the configuration, builds every manifest, resolves the
    // graph and checks permissions. On failure the report contains one line per
    // diagnostic. No backend is initialized and no construct phase runs.
    bool validate(std::string &report);

    // Full startup: validate, then run construct for every component in
    // dependency order, registering provided capabilities as they appear.
    bool construct(std::string &report);

    // Runs start for every component in dependency order and publishes
    // EngineStartupCompletedEvent. Throws std::runtime_error on failure.
    void start();

    // Publishes EngineShutdownStartedEvent, runs stop in reverse order and
    // releases every component, then unloads mod libraries in reverse order.
    void stop();

    bool isStarted() const
    {
        return started;
    }

    EventBus &eventBus()
    {
        return bus;
    }

    CapabilityRegistry &capabilityRegistry()
    {
        return capabilityRegistry_;
    }

    const ConfigurationStore &configuration() const
    {
        return configurationStore;
    }

    // Trusted context for the composition root (the frame loop). The host may
    // acquire every available capability.
    std::unique_ptr<ComponentContext> createHostContext();

private:
    struct ComponentRecord
    {
        // Declaration order is destruction order reversed: the library is
        // declared first so it unloads last, and the context is declared
        // second so it outlives every module and mod instance that may still
        // hold the pointer it received during construct.
        std::unique_ptr<DynamicLibrary> library;
        // The per-component context is heap anchored: components are allowed
        // to cache the reference they get in construct and use it for event
        // publishing for the rest of the engine lifetime (input callbacks,
        // frame events, capture events). A stack-local context here would
        // dangle as soon as the construct phase returns.
        std::unique_ptr<ComponentContext> context;
        const EngineModEntryTable        *modEntryTable = nullptr;
        std::string                       name;
        ComponentManifest                 manifest;
        ModuleInterface                  *moduleInstance = nullptr;
        ModInterface                     *modInstance    = nullptr;
        std::unique_ptr<ModuleInterface>  ownedModule;
        std::unique_ptr<ModInterface>     ownedMod;
        // Script mods have no instance of their own: the hosting module keeps
        // the script runtime. The description is what the kernel hands over.
        ScriptModDescription scriptMod;
        std::string          scriptHostName;
        bool                 constructed = false;
        bool                 started     = false;
    };

    bool loadConfiguration(std::string &report);
    bool loadModuleDefaults(std::string &report);
    // The front-door sequence validate and construct share: load the layered
    // configuration, build the component records, resolve the world source.
    bool prepareComponents(std::string &report);
    // The permission pass both phases run identically: apply the
    // configuration's denials, declare every component's effective grants,
    // and collect the (component, denied permission) pairs for reporting.
    std::vector<std::pair<const ComponentRecord *, std::string>> applyPermissionPass();
    std::vector<ComponentManifest>                               componentManifests() const;
    ComponentRecord                                             *findRecord(const std::string &name);
    // Applies the layers that must wait for mod discovery (the world content
    // source: --scene, or the content mods' declared scene files when the
    // assembly defines no [scene] of its own, then the staged --set
    // overrides), then applies [diagnostics] from the fully merged
    // configuration. Idempotent: the store guards its layers.
    bool finishConfiguration(std::string &report);
    bool gatherComponents(std::string &report);
    void gatherModPackages(std::string &report);
    // Runtime module loading: scans the modules directory and registers one
    // factory per plugin binary (engine/kernel/module_api.hpp). Runs inside
    // gatherComponents, before the configured modules resolve against the
    // factory inventory.
    void loadModulePlugins(std::string &report);
    DependencyResolver::ResolutionResult
         resolveGraph(const std::vector<ComponentManifest> &manifests, std::string &report) const;
    bool runConstructPhase(
        const DependencyResolver::ResolutionResult &resolution,
        const std::vector<ComponentManifest>       &manifests,
        std::string                                &report
    );
    void appendDiagnosticsReport(
        const std::vector<DependencyResolver::ResolutionDiagnostic> &diagnostics, std::string &report
    ) const;

    std::filesystem::path configurationFilePath;
    std::filesystem::path sceneFilePath;
    std::filesystem::path explicitModsDirectory;
    // World content claims the active packages declared: each claim's
    // package-relative scene file resolved against its package directory,
    // plus the owning mod id for diagnostics. Collected during mod
    // discovery; content mods coexist freely - the claims only matter when
    // the run has no explicit world source, and two or more claims then make
    // the world ambiguous (a startup error, never a silent pick).
    struct ModSceneClaim
    {
        std::filesystem::path sceneFile;
        std::string           ownerId;
    };
    std::vector<ModSceneClaim> modSceneClaims;
    // Whether the assembly configuration defined its own [scene] section. The
    // kernel records it right after the assembly layer loads (before any
    // defaults merge): the assembly either owns the world or delegates it.
    bool assemblyDefinesScene = false;
    // validate-only diagnostics that must not fail the run (like the unknown
    // key warnings): printed after the failure check so a warning stays a
    // warning.
    std::string        deferredConfigurationWarnings;
    ConfigurationStore configurationStore;
    EventBus           bus;
    CapabilityRegistry capabilityRegistry_;
    PermissionSystem   permissionSystem_;

    std::unordered_map<std::string, ModuleFactory> moduleFactories;
    std::unordered_map<std::string, ModFactory>    modFactories;

    // The module plugin binaries backing the factories registered by
    // loadModulePlugins. Declared before records so reverse destruction
    // releases every component instance first and unloads the libraries last
    // (a factory's function pointer into a plugin must outlive the instances
    // the factory created).
    std::vector<std::unique_ptr<DynamicLibrary>> modulePluginLibraries;

    std::vector<ComponentRecord> records;
    bool                         configurationLoaded = false;
    bool                         componentsGathered  = false;
    bool                         constructed         = false;
    bool                         started             = false;
};

} // namespace engine

#endif // ENGINE_KERNEL_ENGINE_KERNEL_HPP
