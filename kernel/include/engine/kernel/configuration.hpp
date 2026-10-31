#ifndef ENGINE_KERNEL_CONFIGURATION_HPP
#define ENGINE_KERNEL_CONFIGURATION_HPP

#include "./dependency_resolver.hpp"

#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <set>
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

struct ConfigProvenance
{
    enum class Layer
    {
        CommandLine,
        EngineConfig,
        SceneFile,
        ModuleDefaults,
        ModDefaults
    };

    Layer layer = Layer::EngineConfig;

    std::string source;

    std::string layerName() const;
};

class ConfigurationStore final
{
public:
    static constexpr const char *moduleDefaultsFileName = "config.toml";
    static constexpr const char *modDefaultsFileName    = "config.toml";

    static constexpr const char *engineSectionName      = "engine";
    static constexpr const char *providersSectionName   = "providers";
    static constexpr const char *permissionsSectionName = "permissions";
    static constexpr const char *diagnosticsSectionName = "diagnostics";
    static constexpr const char *sceneSectionName       = "scene";
    static constexpr const char *trustSectionName       = "trust";

    static constexpr const char *trustModsKey       = "trustMods";
    static constexpr const char *trustAllModsKey    = "trustAllMods";
    static constexpr const char *trustModulesKey    = "trustModules";
    static constexpr const char *trustAllModulesKey = "trustAllModules";

    static constexpr const char *diagnosticsLatencyReportKey = "latency_report";
    static constexpr const char *diagnosticsFrameHistoryKey  = "frame_history";
    static constexpr const char *diagnosticsGraphReportKey   = "graph_report";

    static constexpr const char *engineMaxFpsKey                  = "max_fps";
    static constexpr const char *engineIdlePollMsKey              = "idle_poll_ms";
    static constexpr const char *engineConfigReloadPollSecondsKey = "config_reload_poll_seconds";
    static constexpr const char *engineFixedUpdateHzKey           = "fixed_update_hz";
    static constexpr const char *engineMaxFixedUpdateStepsKey     = "max_fixed_update_steps";
    static constexpr const char *engineQuitAfterSecondsKey        = "quit_after_seconds";

    static constexpr const char *engineExtraModDirectoriesKey = "extra_mod_directories";

    static constexpr const char *moduleDefaultsDirectory = "config/modules";

    ConfigurationStore();
    ~ConfigurationStore();

    ConfigurationStore(const ConfigurationStore &)            = delete;
    ConfigurationStore &operator=(const ConfigurationStore &) = delete;

    void reset();

    void resetForReload();

    bool loadFromFile(
        const std::filesystem::path &path,
        std::string                 &errorMessage
    );

    bool mergeDefaultsFile(
        const std::filesystem::path &path,
        ConfigProvenance::Layer      layer,
        const std::string           &source,
        std::string                 &errorMessage
    );

    bool applySceneFile(
        const std::filesystem::path &path,
        std::string                 &errorMessage
    );

    bool stageCommandLineOverride(
        const std::string &assignment,
        std::string       &errorMessage
    );

    bool applyCommandLineOverrides(std::string &errorMessage);

    std::map<
        std::string,
        std::string>
    flattenedValueReprs() const;

    static bool fileParses(
        const std::filesystem::path &path,
        std::string                 &errorMessage
    );

    bool isLoaded() const;

    bool hasSection(const std::string &sectionName) const;

    std::string modDirectory() const;

    std::vector<std::string> extraModDirectories() const;

    std::string              modulesDirectory() const;
    std::vector<std::string> disabledMods() const;

    std::vector<DependencyResolver::ProviderSelection> providerSelections() const;

    std::vector<std::pair<
        std::string,
        std::string>>
    permissionDenials() const;

    std::vector<std::string> trustedMods() const;
    bool                     trustAllMods() const;
    std::vector<std::string> trustedModules() const;
    bool                     trustAllModules() const;

    std::optional<std::string> stringValue(
        const std::string &sectionName,
        const std::string &key
    ) const;
    std::optional<std::int64_t> integerValue(
        const std::string &sectionName,
        const std::string &key
    ) const;
    std::optional<bool> booleanValue(
        const std::string &sectionName,
        const std::string &key
    ) const;
    std::optional<double> floatingValue(
        const std::string &sectionName,
        const std::string &key
    ) const;

    std::optional<std::string> stringAtPath(
        const std::string &sectionName,
        const std::string &path
    ) const;
    std::vector<std::string> stringArrayAtPath(
        const std::string &sectionName,
        const std::string &path
    ) const;
    std::optional<std::int64_t> integerAtPath(
        const std::string &sectionName,
        const std::string &path
    ) const;
    std::optional<bool> booleanAtPath(
        const std::string &sectionName,
        const std::string &path
    ) const;
    std::optional<double> floatingAtPath(
        const std::string &sectionName,
        const std::string &path
    ) const;
    std::vector<double> floatingArrayAtPath(
        const std::string &sectionName,
        const std::string &path
    ) const;

    std::size_t arraySizeAtPath(
        const std::string &sectionName,
        const std::string &path
    ) const;

    std::vector<std::string> tableKeysAtPath(
        const std::string &sectionName,
        const std::string &path
    ) const;

    std::optional<ConfigProvenance> provenanceFor(
        const std::string &sectionName,
        const std::string &path
    ) const;

    std::string provenanceReport() const;

    bool declareKnownConfigKey(const std::string &dottedKeyPath);

    void declareAssemblyKnownKeys();

    std::vector<std::string> unknownKeyWarnings() const;

private:
    const toml::table *sectionTable(const std::string &sectionName) const;

    std::vector<std::string> trustSectionStringArray(const std::string &key) const;
    bool                     trustSectionBoolean(const std::string &key) const;

    static std::string leafValueRepr(const toml::node &value);

    static void flattenNode(
        const toml::node  &subtree,
        const std::string &path,
        std::map<
            std::string,
            std::string> &out
    );

    void mergeTableAsOverride(
        toml::table            &destination,
        const toml::table      &source,
        const std::string      &prefix,
        const ConfigProvenance &provenance
    );

    void mergeTableAsDefaults(
        toml::table            &destination,
        const toml::table      &source,
        const std::string      &prefix,
        const ConfigProvenance &provenance
    );

    void recordProvenance(
        const toml::node       &subtree,
        const std::string      &prefix,
        const ConfigProvenance &provenance
    );

    void eraseProvenanceUnder(const std::string &prefix);

    toml::table                            *parsedTable = nullptr;
    std::map<std::string, ConfigProvenance> provenanceByPath;

    struct StagedOverride
    {
        std::vector<std::string> pathSegments;

        std::unique_ptr<toml::table> parsedValue;
        bool                         isPlainString = false;
        std::string                  plainStringValue;
    };
    std::vector<StagedOverride> stagedOverrides;

    std::map<std::string, std::set<std::string>> knownKeys;
};

} // namespace engine

#endif
