#ifndef ENGINE_KERNEL_ENGINE_KERNEL_HPP
#define ENGINE_KERNEL_ENGINE_KERNEL_HPP

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
#include <unordered_set>
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
// permission system and the event bus, and drives the construct / start /
// stop lifecycle in dependency order.
//
// The kernel deliberately knows nothing about windows, rendering or scripting,
// and it does not know any module by name: modules and mods are anonymous
// capability providers and consumers that declare themselves through static
// descriptors and manifests. The kernel never registers a module, a service
// or a capability: it only mediates. It resolves who requires which capability,
// orders the construct phases, pulls every acquisition straight from the
// declaring provider and enforces permissions. Startup problems are collected
// into one precise report instead of failing with the first error.
class EngineKernel final : public CapabilityResolver
{
public:
    EngineKernel() = default;
    ~EngineKernel();

    EngineKernel(const EngineKernel &)            = delete;
    EngineKernel &operator=(const EngineKernel &) = delete;

    // Configuration file path, relative to the executable directory when not
    // absolute. Loading happens during validate/construct.
    void setConfigurationFile(const std::filesystem::path &configurationPath);

    // Scene file: replaces the assembly configuration's [scene]
    // section wholesale once every layer has merged (the file may define only
    // [scene]). Relative paths resolve against the executable directory, like
    // the configuration file.
    void setSceneFile(const std::filesystem::path &scenePath);

    // Stages one "section.path=value" override. The assignment is
    // parsed immediately so a malformed assignment is a usage error before any
    // backend starts; the value is applied after every configuration layer
    // has merged, because staged overrides are the highest layer. Returns
    // false with error filled in when the assignment is malformed.
    bool addCommandLineOverride(const std::string &assignment, std::string &error);

    // Directory that is scanned for mod packages (one subdirectory with a
    // mod.toml each), relative to the executable directory when not absolute.
    // Default: "mods". The configuration key [engine].mod_directory overrides
    // this default; an explicit call here overrides both.
    void setModsDirectory(const std::filesystem::path &modsDirectory);

    // Dry run: loads the configuration, builds every manifest, resolves the
    // graph and checks permissions. On failure the report contains one line per
    // diagnostic. No backend is initialized and no construct phase runs.
    bool validate(std::string &report);

    // Full startup: validate, then run construct for every component in
    // dependency order, verifying each provided capability has an
    // implementation behind it.
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

    // The pull surface behind ComponentContext::acquireCapability: resolves
    // every request straight from the declaring provider's
    // capabilityPointerFor hook at acquisition time. Nothing is stored.
    void *resolveCapabilityPointer(const std::string &requesterName, const std::string &capabilityName) override;

    const ConfigurationStore &configuration() const
    {
        return configurationStore;
    }

    // Warnings that do not fail a run (dangerous-mod advisories and similar
    // policy notices). validate() appends them to its report; a normal run
    // prints them through this accessor so the user always sees them.
    const std::string &startupWarnings() const
    {
        return startupWarnings_;
    }

    // Trusted context for the composition root (the frame loop). The host may
    // acquire every available capability.
    std::unique_ptr<ComponentContext> createHostContext();

private:
    // One module plugin binary found in the modules directory, described by
    // its own static descriptor (engine/kernel/module_api.hpp). The kernel
    // holds the descriptor WITHOUT creating an instance: only plugins whose
    // provided capabilities the demand closure requires are ever instantiated.
    // The library stays loaded for active plugins so their function pointers
    // and static data stay valid; the kernel releases the libraries of
    // non-required plugins immediately after the closure is computed.
    struct ModulePluginRecord
    {
        std::string name;
        ModuleInterface *(*createModule)() = nullptr;
        std::vector<std::string>        providedCapabilities;
        std::vector<std::string>        requiredCapabilities;
        std::unique_ptr<DynamicLibrary> library;
        bool                            active = false;
    };

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
    // Asks one constructed component for the implementation behind a
    // capability name it declared; shared by the exact and the family path of
    // resolveCapabilityPointer.
    void *capabilityPointerOf(ComponentRecord &provider, const std::string &capabilityName);
    // Scans config/modules/<name>/module.toml shipped defaults; the lowest
    // configuration layer (defaults fill gaps, they never override).
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
    // source: the scene file, or the content mods' declared scene files when the
    // assembly defines no [scene] of its own, then the staged
    // overrides), then applies [diagnostics] from the fully merged
    // configuration. Idempotent: the store guards its layers.
    bool finishConfiguration(std::string &report);
    bool gatherComponents(std::string &report);
    void gatherModPackages(std::string &report);
    // Capability demand: seeds the worklist with every active mod's required
    // capabilities, then activates the module plugins that provide them -
    // transitively, because an activated plugin's own requirements demand
    // further providers. Runs after the mods are gathered and before any
    // module is instantiated; a plugin whose capabilities no mod requires is
    // never instantiated and its binary is released. With no active mods, no
    // module loads at all.
    void computeDemandClosure();
    // Runtime module discovery: scans the modules directory and reads one
    // static descriptor per plugin binary (engine/kernel/module_api.hpp).
    // No instance is created here. Runs inside gatherComponents, before the
    // demand closure decides which descriptors become components.
    void loadModulePlugins(std::string &report);
    // Instantiates exactly the plugins the demand closure marked active,
    // verifies each instance manifest against its static descriptor and
    // turns it into a component record. Returns false (with diagnostics in
    // the report) when a factory fails or a manifest drifts from the
    // descriptor.
    bool instantiateActiveModules(std::string &report);
    // The construct phase both construct() and startup recovery share: walks
    // the resolved order, hosts script mods through their resolved provider
    // and verifies every provided capability has an implementation behind it
    // (nothing is stored - acquisition pulls from the provider at call time).
    bool runConstructPhase(
        const DependencyResolver::ResolutionResult &resolution,
        const std::vector<ComponentManifest>       &manifests,
        std::string                                &report
    );
    // Resolves the hosting module of one script mod: the active component
    // that provides the host capability the package declared, honoring the
    // configuration's [providers] selection when several implementations are
    // active.
    // Returns nullptr (with a diagnostic in the report) when no provider or
    // an ambiguous provider set remains.
    ComponentRecord *resolveScriptHost(const ComponentRecord &scriptRecord, std::string &report);
    // Structural risk advisories for one active mod: native machine code,
    // environment permissions (anything outside the capability.* namespace)
    // and broad capability access. Appends one warning line per mod.
    void appendModRiskWarning(const ModManifest &modManifest, bool isNativeMod);
    DependencyResolver::ResolutionResult
         resolveGraph(const std::vector<ComponentManifest> &manifests, std::string &report) const;
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
    std::string        startupWarnings_;
    ConfigurationStore configurationStore;
    EventBus           bus;
    PermissionSystem   permissionSystem_;

    // The plugin descriptors found in the modules directory. Declared before
    // records so reverse destruction releases every component instance first
    // and unloads the plugin libraries last (a factory's function pointer
    // into a plugin must outlive the instances the factory created).
    std::vector<ModulePluginRecord> modulePlugins;

    // Every capability the demand closure found a consumer for. ComponentContext
    // exposes it to modules ("was my layer capability demanded?"); it carries
    // no meaning the kernel would interpret.
    std::unordered_set<std::string> demandedCapabilities;

    std::vector<ComponentRecord> records;
    // The components that completed the construct phase, in the resolver's
    // dependency order (providers before consumers). start() and stop() walk
    // this sequence - not the discovery-ordered records vector - because the
    // start phase consumes provider work: the renderer needs the window the
    // window module creates during its own start. The records vector holds
    // the plugin scan order, which says nothing about providership.
    std::vector<ComponentRecord *> componentStartOrder;
    bool                           configurationLoaded = false;
    bool                           componentsGathered  = false;
    bool                           constructed         = false;
    bool                           started             = false;
};

} // namespace engine

#endif // ENGINE_KERNEL_ENGINE_KERNEL_HPP
