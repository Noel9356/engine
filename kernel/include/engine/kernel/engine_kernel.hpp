#ifndef ENGINE_KERNEL_ENGINE_KERNEL_HPP
#define ENGINE_KERNEL_ENGINE_KERNEL_HPP

#include <engine/kernel/component_interface.hpp>
#include <engine/kernel/component_manifest.hpp>
#include <engine/kernel/configuration.hpp>
#include <engine/kernel/dependency_resolver.hpp>
#include <engine/kernel/dynamic_library.hpp>
#include <engine/kernel/event_bus.hpp>
#include <engine/kernel/main_thread_scheduler.hpp>
#include <engine/kernel/mod_api.hpp>
#include <engine/kernel/mod_discovery.hpp>
#include <engine/kernel/mod_manifest.hpp>
#include <engine/kernel/permission_system.hpp>

#include <deque>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace engine
{

struct EngineStartupCompletedEvent
{};

struct EngineShutdownStartedEvent
{};

struct ComponentTrustAdvisory
{
    ComponentKind kind = ComponentKind::Module;

    std::string name;

    std::vector<std::string> reasons;
};

class EngineKernel final : public CapabilityResolver
{
public:
    EngineKernel() = default;
    ~EngineKernel();

    EngineKernel(const EngineKernel &)            = delete;
    EngineKernel &operator=(const EngineKernel &) = delete;

    void setConfigurationFile(const std::filesystem::path &configurationPath);

    void setSceneFile(const std::filesystem::path &scenePath);

    bool addCommandLineOverride(
        const std::string &assignment,
        std::string       &error
    );

    void setModsDirectory(const std::filesystem::path &modsDirectory);
    void addModsDirectory(const std::filesystem::path &modsDirectory);

    void setApplicationRequirements(const std::vector<std::string> &capabilities);

    bool validate(std::string &report);

    bool construct(std::string &report);
    void start();
    void stop();

    bool reloadConfiguration(std::string &report);

    std::string componentGraphReport() const;

    bool isStarted() const
    {
        return started;
    }

    EventBus &eventBus()
    {
        return bus;
    }

    void *resolveCapabilityPointer(
        const std::string &requesterName,
        ComponentKind      requesterKind,
        const std::string &capabilityName
    ) override;

    std::vector<void *> resolveEachCapabilityPointer(
        const std::string &requesterName,
        ComponentKind      requesterKind,
        const std::string &capabilityName
    ) override;

    bool hasSingleCapabilityProvider(
        const std::string &capabilityName,
        ComponentKind      requesterKind
    ) override;

    void *tryResolveCapabilityPointer(
        const std::string &requesterName,
        ComponentKind      requesterKind,
        const std::string &capabilityName
    ) override;

    const ConfigurationStore &configuration() const
    {
        return configurationStore;
    }

    const std::string &startupWarnings() const
    {
        return startupWarnings_;
    }

    const std::vector<ComponentTrustAdvisory> &trustAdvisories() const
    {
        return trustAdvisories_;
    }

    using TrustConsentHandler = std::function<bool(const std::vector<ComponentTrustAdvisory> &untrustedAdvisories)>;
    void setTrustConsentHandler(TrustConsentHandler handler);

    MainThreadScheduler &scheduler()
    {
        return mainThreadScheduler_;
    }

    std::unique_ptr<ComponentContext> createAppContext();

private:
    struct ModulePluginRecord
    {
        std::string name;

        std::filesystem::path libraryPath;
        ModuleInterface *(*createModule)() = nullptr;
        std::vector<std::string> providedCapabilities;
        std::vector<std::string> requiredCapabilities;
        std::vector<std::string> optionalCapabilities;

        std::vector<std::string>        declaredPermissions;
        std::unique_ptr<DynamicLibrary> library;
        bool                            active = false;
    };

    struct ComponentRecord
    {
        std::unique_ptr<DynamicLibrary> library;

        std::unique_ptr<ComponentContext> context;
        const EngineModEntryTable        *modEntryTable = nullptr;
        std::string                       name;
        ComponentManifest                 manifest;
        ModuleInterface                  *moduleInstance = nullptr;
        ModInterface                     *modInstance    = nullptr;
        std::unique_ptr<ModuleInterface>  ownedModule;
        std::unique_ptr<ModInterface>     ownedMod;

        ScriptModDescription scriptMod;
        std::string          scriptHostName;
        bool                 constructed = false;
        bool                 started     = false;
    };

    bool loadConfiguration(std::string &report);

    bool declareKnownKeyRegistry(std::string &report);

    void *capabilityPointerOf(
        ComponentRecord   &provider,
        const std::string &capabilityName
    );

    struct CapabilityProviderSet
    {
        std::vector<ComponentRecord *> exactProviders;

        std::vector<std::pair<ComponentRecord *, std::string>> memberProviders;
    };

    void classifyCapabilityProviders(
        const std::string     &capabilityName,
        CapabilityProviderSet &outSet
    );

    const CapabilityProviderSet &capabilityProviderSetFor(const std::string &capabilityName);

    bool loadModuleDefaults(std::string &report);

    bool prepareComponents(
        std::string &report,
        bool         enforceTrust
    );

    std::vector<std::pair<
        const ComponentRecord *,
        std::string>>
                                   applyPermissionPass();
    std::vector<ComponentManifest> componentManifests() const;
    ComponentRecord               *findRecord(const std::string &name);

    bool finishConfiguration(std::string &report);
    bool gatherComponents(
        std::string &report,
        bool         enforceTrust
    );
    void gatherModPackages(std::string &report);

    void loadNativeModLibraries(std::string &report);

    void computeDemandClosure();

    void loadModulePlugins(std::string &report);

    bool loadPluginLibrary(
        ModulePluginRecord &plugin,
        std::string        &report
    );

    bool instantiateActiveModules(std::string &report);

    bool runConstructPhase(
        const DependencyResolver::ResolutionResult &resolution,
        const std::vector<ComponentManifest>       &manifests,
        std::string                                &report
    );

    ComponentRecord *resolveScriptHost(
        const ComponentRecord &scriptRecord,
        std::string           &report
    );

    void appendModRiskWarning(
        const ModManifest &modManifest,
        bool               isNativeMod
    );

    void appendModuleRiskWarning(const ModulePluginRecord &plugin);

    bool evaluateTrustGate(
        std::string &report,
        bool         enforce
    );
    DependencyResolver::ResolutionResult resolveGraph(
        const std::vector<ComponentManifest> &manifests,
        std::string                          &report
    ) const;
    void appendDiagnosticsReport(
        const std::vector<DependencyResolver::ResolutionDiagnostic> &diagnostics,
        std::string                                                 &report
    ) const;

    std::vector<std::string> applicationRequiredCapabilities;

    std::filesystem::path configurationFilePath;
    std::filesystem::path sceneFilePath;
    std::filesystem::path explicitModsDirectory;

    std::vector<std::filesystem::path> explicitExtraModsDirectories;

    struct ModSceneClaim
    {
        std::filesystem::path sceneFile;
        std::string           ownerId;
    };
    std::vector<ModSceneClaim> modSceneClaims;

    std::vector<std::pair<std::filesystem::path, std::string>> modDefaultsFiles;

    bool assemblyDefinesScene = false;

    std::string deferredConfigurationWarnings;
    std::string startupWarnings_;

    std::vector<ComponentTrustAdvisory> trustAdvisories_;
    TrustConsentHandler                 trustConsentHandler_;

    std::vector<DiscoveredModPackage> pendingNativeModPackages_;

    bool               nativeModLibrariesLoaded_ = false;
    ConfigurationStore configurationStore;
    EventBus           bus;
    PermissionSystem   permissionSystem_;

    MainThreadScheduler mainThreadScheduler_;

    std::vector<ModulePluginRecord> modulePlugins;

    std::unordered_set<std::string, CapabilityNameHash, std::equal_to<>> demandedCapabilities;

    CapabilityProviderSet providerScratch;

    struct CapabilityProviderIndexEntry
    {
        std::string           capabilityName;
        CapabilityProviderSet providers;
    };

    std::deque<CapabilityProviderIndexEntry> capabilityProviderIndex;
    bool                                     capabilityProviderIndexFrozen = false;

    std::vector<ComponentRecord> records;

    std::vector<ComponentRecord *> componentStartOrder;
    bool                           configurationLoaded = false;
    bool                           componentsGathered  = false;
    bool                           constructed         = false;
    bool                           started             = false;
};

} // namespace engine

#endif
