// ConfigurationStore: layered TOML configuration for the whole engine.
//
// Layers merge onto one document: engine.toml, runtime profiles, scene
// file, module defaults, plugin defaults and staged command-line overrides
// (last wins). The store tracks which layer supplied each key
// (provenance), which keys are known (unknown-key warnings) and offers
// typed readers for sections and dotted paths.

#ifndef ENGINE_KERNEL_CONFIGURATION_HPP
#define ENGINE_KERNEL_CONFIGURATION_HPP

#include <engine/kernel/dependency_resolver.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace toml
{
inline namespace v3
{
class table;
class node;
} // namespace v3
} // namespace toml

namespace engine
{

class ProvenanceTracker;
class KnownKeyRegistry;
class CommandLineOverrides;

// One [trust] entry; empty version/sha256 fields act as wildcards
// (name-only trust).
struct TrustBinding
{
    std::string name    = {};
    std::string version = {};

    std::string sha256 = {};
};

// Which layer supplied an effective value (see layerName()).
struct ConfigProvenance
{
    // Effective precedence, highest first: CommandLine, Profile, SceneFile,
    // EngineConfig, then the defaults layers (ModuleDefaults/PluginDefaults only
    // fill keys no higher layer set).
    enum class Layer
    {
        CommandLine,
        EngineConfig,
        SceneFile,
        ModuleDefaults,
        PluginDefaults,
        Profile
    };

    Layer layer = Layer::EngineConfig;

    std::string source;

    std::string layerName() const;
};

// Parses and merges the configuration layers and answers typed queries.
// Not thread-safe: owned by the kernel, touched from the main thread.
class ConfigurationStore final
{
public:
    // Both ship as "config.toml": modules in config/modules/<name>/, plugins in
    // their package directory.
    static constexpr const char *moduleDefaultsFileName = "config.toml";
    static constexpr const char *pluginDefaultsFileName    = "config.toml";

    // Sections owned by the assembly ([scene] carries world content);
    // defaults files may never define these.
    static constexpr const char *engineSectionName      = "engine";
    static constexpr const char *providersSectionName   = "providers";
    static constexpr const char *permissionsSectionName = "permissions";
    static constexpr const char *diagnosticsSectionName = "diagnostics";
    static constexpr const char *sceneSectionName       = "scene";
    static constexpr const char *trustSectionName       = "trust";

    // [trust]: id/name lists (plain strings or {id,version,sha256} tables)
    // plus the trustAllPlugins/trustAllModules blanket switches.
    static constexpr const char *trustPluginsKey       = "trustPlugins";
    static constexpr const char *trustAllPluginsKey    = "trustAllPlugins";
    static constexpr const char *trustModulesKey    = "trustModules";
    static constexpr const char *trustAllModulesKey = "trustAllModules";

    // [engine]: active_plugins restricts the run to these plugin ids;
    // disabled_plugins excludes packages from discovery entirely.
    static constexpr const char *engineActivePluginsKey   = "active_plugins";
    static constexpr const char *engineDisabledPluginsKey = "disabled_plugins";

    static constexpr const char *diagnosticsLatencyReportKey    = "latency_report";
    static constexpr const char *diagnosticsFrameHistoryKey     = "frame_history";
    static constexpr const char *diagnosticsGraphReportKey      = "graph_report";
    static constexpr const char *diagnosticsProfilingEnabledKey = "profiling_enabled";

    // [engine] tuning keys read by the application layer (pacing, fixed step,
    // reload polling) and the task scheduler (worker_count and friends).
    static constexpr const char *engineMaxFpsKey                  = "max_fps";
    static constexpr const char *engineIdlePollMsKey              = "idle_poll_ms";
    static constexpr const char *engineConfigReloadPollSecondsKey = "config_reload_poll_seconds";
    static constexpr const char *engineFixedUpdateHzKey           = "fixed_update_hz";
    static constexpr const char *engineMaxFixedUpdateStepsKey     = "max_fixed_update_steps";
    static constexpr const char *engineMaxFrameDeltaSecondsKey    = "max_frame_delta_seconds";
    static constexpr const char *engineQuitAfterSecondsKey        = "quit_after_seconds";

    static constexpr const char *engineExtraPluginDirectoriesKey = "extra_plugin_directories";

    // Scanned for per-module defaults (config/modules/<module>/config.toml).
    static constexpr const char *moduleDefaultsDirectory = "config/modules";

    ConfigurationStore();
    ~ConfigurationStore();

    ConfigurationStore(const ConfigurationStore &)            = delete;
    ConfigurationStore &operator=(const ConfigurationStore &) = delete;

    // Drops everything, including staged overrides and the known-key registry.
    void reset();

    // Reload variant: keeps staged overrides and known keys, drops content.
    void resetForReload();

    // Parses engine.toml and merges it as overrides (its keys win over the
    // defaults merged later). Returns false and fills errorMessage on parse
    // failure.
    bool loadFromFile(const std::filesystem::path &path, std::string &errorMessage);

    // Merges a module/plugin defaults file; only fills keys still unset. layer
    // must be ModuleDefaults or PluginDefaults. Rejects assembly-owned sections,
    // [scene] and bare top-level values.
    bool mergeDefaultsFile(const std::filesystem::path &path, ConfigProvenance::Layer layer, const std::string &source,
        std::string &errorMessage);

    // Merges a runtime profile as overrides, restricted to the whitelisted
    // [engine]/[diagnostics] tuning keys; profiles cannot touch scene,
    // providers, permissions or trust.
    bool mergeProfileFile(const std::filesystem::path &path, std::string &errorMessage);

    // Replaces the store's [scene] section with the scene file's content.
    bool applySceneFile(const std::filesystem::path &path, std::string &errorMessage);

    // Validates and queues a KEY=VALUE assignment; applyCommandLineOverrides
    // writes it into the document as the CommandLine layer.
    bool stageCommandLineOverride(const std::string &assignment, std::string &errorMessage);

    // Applies every staged override (idempotent; safe to call again).
    bool applyCommandLineOverrides(std::string &errorMessage);

    // Snapshot of every leaf value as "type:value" text, used to diff
    // configurations across a reload.
    std::map<std::string, std::string> flattenedValueReprs() const;

    // Static pre-parse check so a reload can refuse before touching live
    // state.
    static bool fileParses(const std::filesystem::path &path, std::string &errorMessage);

    bool isLoaded() const;

    bool hasSection(const std::string &sectionName) const;

    // [engine] plugin_directory / modules_directory; "plugins"/"modules" defaults,
    // resolved against the executable.
    std::string pluginsDirectory() const;

    std::vector<std::string> extraPluginsDirectories() const;

    std::string              modulesDirectory() const;
    std::vector<std::string> disabledPlugins() const;

    // nullopt when unset (all discovered plugins run); otherwise the allow-list.
    std::optional<std::vector<std::string>> activePlugins() const;

    // [providers] entries as capability -> participant-name choices.
    std::vector<DependencyResolver::ProviderSelection> providerSelections() const;

    // [permissions] denied = [["participant", "permission"], ...].
    std::vector<std::pair<std::string, std::string>> permissionDenials() const;

    // [trust] readers; the *Bindings variants also parse {id/version/sha256}
    // table entries (sha256 binds trust to specific content).
    std::vector<std::string> trustedPlugins() const;
    std::vector<std::string> trustedModules() const;

    bool trustAllPlugins() const;
    bool trustAllModules() const;

    // Binding form of the [trust] lists (see trustedPlugins).
    std::vector<TrustBinding> trustedPluginBindings() const;

    std::vector<TrustBinding> trustedModuleBindings() const;

    // Typed lookups within one section; nullopt when absent or of a different
    // type.
    std::optional<std::string>  stringValue(const std::string &sectionName, const std::string &key) const;
    std::optional<std::int64_t> integerValue(const std::string &sectionName, const std::string &key) const;
    std::optional<bool>         booleanValue(const std::string &sectionName, const std::string &key) const;
    std::optional<double>       floatingValue(const std::string &sectionName, const std::string &key) const;

    // Dotted-path lookups ("a.b.c", optional "[n]" array indexing); return
    // nullopt/empty on any mismatch. floatingAtPath also accepts integers.
    std::optional<std::string>  stringAtPath(const std::string &sectionName, const std::string &path) const;
    std::vector<std::string>    stringArrayAtPath(const std::string &sectionName, const std::string &path) const;
    std::optional<std::int64_t> integerAtPath(const std::string &sectionName, const std::string &path) const;
    std::optional<bool>         booleanAtPath(const std::string &sectionName, const std::string &path) const;
    std::optional<double>       floatingAtPath(const std::string &sectionName, const std::string &path) const;
    std::vector<double>         floatingArrayAtPath(const std::string &sectionName, const std::string &path) const;

    // 0 when the path does not resolve to an array.
    std::size_t arraySizeAtPath(const std::string &sectionName, const std::string &path) const;

    std::vector<std::string> tableKeysAtPath(const std::string &sectionName, const std::string &path) const;

    // Provenance of section.path; nullopt when unknown.
    std::optional<ConfigProvenance> provenanceFor(const std::string &sectionName, const std::string &path) const;

    // Human-readable "path <- layer (source)" dump.
    std::string provenanceReport() const;

    // Registers a "section.path" (or bare "section") key participants read;
    // supports a trailing ".*" wildcard. Returns false for malformed keys.
    bool declareKnownConfigKey(const std::string &dottedKeyPath);

    // Declares the keys the engine itself reads.
    void declareAssemblyKnownKeys();

    // Values no participant declared and no defaults file shipped; surfaced
    // after validate/construct/reload.
    std::vector<std::string> unknownKeyWarnings() const;

private:
    const toml::table *sectionTable(const std::string &sectionName) const;

    std::vector<std::string>  trustSectionStringArray(const std::string &key) const;
    std::vector<TrustBinding> trustSectionBindings(const std::string &key) const;
    bool                      trustSectionBoolean(const std::string &key) const;

    static std::string leafValueRepr(const toml::node &value);

    static void flattenNode(
        const toml::node &subtree, const std::string &path, std::map<std::string, std::string> &out);

    void mergeTableAsOverride(toml::table &destination, const toml::table &source, const std::string &prefix,
        const ConfigProvenance &provenance, ProvenanceTracker &provenanceTracker);

    void mergeTableAsDefaults(toml::table &destination, const toml::table &source, const std::string &prefix,
        const ConfigProvenance &provenance, ProvenanceTracker &provenanceTracker);

    // The merged document; a raw owning pointer because toml::table is only
    // forward-declared here (deleted in the destructor).
    toml::table *parsedTable = nullptr;

    // Helpers implemented in configuration.cpp.
    std::unique_ptr<ProvenanceTracker>    provenanceTracker;
    std::unique_ptr<KnownKeyRegistry>     knownKeyRegistry;
    std::unique_ptr<CommandLineOverrides> commandLineOverrides;
};

} // namespace engine

#endif
