#ifndef ENGINE_KERNEL_CONFIGURATION_HPP
#define ENGINE_KERNEL_CONFIGURATION_HPP

#include "dependency_resolver.hpp"

#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

// Forward declaration matching toml++'s real shape: the type lives in the
// inline namespace toml::v3 and is re-exported under toml. Opening v3 as a
// regular namespace here would collide with the inline definition inside
// <toml.hpp>.
namespace toml
{
inline namespace v3
{
class table;
class node;
}
}

namespace engine
{

// Where one effective configuration value came from. Recorded for every leaf
// key while the layered configuration is assembled, surfaced by the
// provenance report (--provenance) and by the unknown-key warnings of
// --validate ("unknown key 'x', from '...'" only means something when the
// source layer is named).
struct ConfigProvenance
{
    // Layers in decreasing precedence: the command line wins, the assembly
    // file wins over everything that is shipped, shipped defaults fill only
    // what nobody above them provided.
    enum class Layer
    {
        CommandLine,    // --set key=value
        EngineConfig,   // config/engine.toml (or --config <path>)
        SceneFile,      // --scene <path> (wholesale [scene] swap)
        ModuleDefaults, // config/modules/<name>/module.toml
        ModDefaults     // <mod package>/config.toml
    };

    Layer layer = Layer::EngineConfig;

    // File path, mod package directory name, module name or "--set".
    std::string source;

    std::string layerName() const;
};

// Loads and merges the layered engine configuration. The store owns the
// effective merged table; components read their own section through their
// ComponentContext. Paths inside the configuration are resolved relative to
// the executable directory by the kernel, not here.
//
// The layers, assembled in the order the kernel drives them (highest
// precedence last):
//
//   1. loadFromFile            the assembly file (config/engine.toml or
//                              --config <path>). Its values are final unless
//                              --set overrides them.
//   2. mergeDefaultsFile       shipped defaults, scanned by the kernel: one
//                              module.toml per module directory, one
//                              config.toml per active mod package. Defaults
//                              FILL: a key the assembly already provided is
//                              never touched, tables merge recursively,
//                              arrays and sub-tables are inserted only when
//                              absent.
//   3. applySceneFile          --scene <path>: replaces the [scene] section
//                              wholesale. The file may define only [scene].
//   4. applyCommandLineOverrides  the staged --set assignments, in the order
//                              they were given. Values use TOML syntax; a
//                              bare word becomes a literal string.
//
// Arrays and array-of-tables rows ([[scene.objects]]) are always replaced
// wholesale by a higher layer, never merged element-wise. There is no
// hot reload: the stack is assembled once, before any component constructs.
class ConfigurationStore final
{
public:
    // File names the kernel looks for when scanning shipped defaults.
    static constexpr const char* moduleDefaultsFileName = "module.toml";
    static constexpr const char* modDefaultsFileName = "config.toml";

    // Section names shared by the kernel, the host and the authority policy.
    // Named once because each is spelled at several sites (assembly-authority
    // checks, typed accessors, the host's readers) and a misspelling would
    // silently read the wrong section.
    static constexpr const char* engineSectionName = "engine";
    static constexpr const char* providersSectionName = "providers";
    static constexpr const char* permissionsSectionName = "permissions";
    static constexpr const char* diagnosticsSectionName = "diagnostics";
    static constexpr const char* leakTestSectionName = "leak_test";
    static constexpr const char* sceneSectionName = "scene";

    // [leak_test] keys: the host's render-QA contract. The kernel declares
    // them known so shipped defaults cannot own them; the host reads them.
    static constexpr const char* leakTestMaxLuminanceKey = "max_luminance";
    static constexpr const char* leakTestRegionKey = "region";
    static constexpr const char* leakTestLitRegionKey = "lit_region";
    static constexpr const char* leakTestMinLitLuminanceKey = "min_lit_luminance";

    // [diagnostics] keys.
    static constexpr const char* diagnosticsLatencyReportKey = "latency_report";

    // Directory scanned for per-module shipped defaults, relative to the
    // executable directory (one module.toml per subdirectory).
    static constexpr const char* moduleDefaultsDirectory = "config/modules";

    // Declared here, defined in the .cpp: the staged-override vector holds a
    // unique_ptr to the forward-declared TOML table, so every special member
    // that can destroy it must instantiate where the type is complete.
    ConfigurationStore();
    ~ConfigurationStore();

    ConfigurationStore(const ConfigurationStore&) = delete;
    ConfigurationStore& operator=(const ConfigurationStore&) = delete;

    // Drops every layer, staged override and known key; the store returns to
    // its just-constructed state. The kernel calls this when the composition
    // root points it at a new configuration or scene file.
    void reset();

    // Layer 1: parses and merges the assembly file. On failure the effective
    // table stays empty and errorMessage holds the parser diagnostic including
    // source location.
    bool loadFromFile(const std::filesystem::path& path, std::string& errorMessage);

    // Layer 2: merges one shipped defaults file (module.toml or config.toml).
    // Defaults fill gaps only, as described in the class comment. Assembly and
    // host authority sections ([engine], [providers], [permissions],
    // [diagnostics], [leak_test]) are rejected with a precise error: a shipped
    // package must not be able to grow a module list or deny permissions.
    // The [scene] section is rejected for the same authority reason: world
    // content has a dedicated channel (a mod's declared scene file or the
    // assembly's [scene], replaced wholesale by --scene). A defaults file that
    // could drip scene keys into configurations that do not define them would
    // silently change OTHER configurations' worlds (a sealed-room test config
    // that does not name a sky gradient must not inherit one from an installed
    // content package).
    // layer selects ModuleDefaults or ModDefaults; source names the module or
    // package for the provenance report.
    bool mergeDefaultsFile(const std::filesystem::path& path, ConfigProvenance::Layer layer,
                           const std::string& source, std::string& errorMessage);

    // Layer 3: replaces the [scene] section with the file's [scene] section
    // wholesale. The file may define ONLY [scene] (sub-tables and
    // array-of-tables rows included); any other top-level key, a non-table
    // [scene] or a missing [scene] is a precise error.
    bool applySceneFile(const std::filesystem::path& path, std::string& errorMessage);

    // Layer 4, staging: parses one "section.path=value" assignment now, so a
    // malformed --set is a usage error before any backend starts. The value
    // side is parsed as TOML (numbers, booleans, quoted strings, arrays,
    // inline tables); anything that does not parse as a TOML value becomes a
    // literal string, so --set window.title=My Title works without quotes.
    bool stageCommandLineOverride(const std::string& assignment, std::string& errorMessage);

    // Layer 4, application: writes the staged assignments into the effective
    // table in staging order, creating intermediate tables as needed.
    // Traversing through an existing non-table value is a precise error.
    bool applyCommandLineOverrides(std::string& errorMessage);

    bool isLoaded() const;

    // Whether the section exists in the effective merged table. The kernel
    // uses it to decide whether the assembly provided its own world ([scene])
    // or delegates content to the installed content mod's scene file.
    bool hasSection(const std::string& sectionName) const;

    // [engine] section. Module names must match the factories registered by
    // the module plugins (or by a host that links modules directly); mod
    // packages are discovered in modDirectory and skipped when their id is in
    // disabledMods.
    std::vector<std::string> moduleNames() const;
    std::string modDirectory() const;
    // Directory scanned for module plugin binaries, relative to the
    // executable directory when not absolute. Default: "modules".
    std::string modulesDirectory() const;
    std::vector<std::string> disabledMods() const;

    // [providers] capability name -> selected provider name.
    std::vector<DependencyResolver::ProviderSelection> providerSelections() const;

    // [permissions] denied = [[component, permission], ...].
    std::vector<std::pair<std::string, std::string>> permissionDenials() const;

    // Typed value access into one component section, for example
    // stringValue("window", "title"). Returns empty when the section or the
    // key is absent; the owning component applies its own default.
    std::optional<std::string> stringValue(const std::string& sectionName, const std::string& key) const;
    std::optional<std::int64_t> integerValue(const std::string& sectionName, const std::string& key) const;
    std::optional<bool> booleanValue(const std::string& sectionName, const std::string& key) const;
    std::optional<double> floatingValue(const std::string& sectionName, const std::string& key) const;

    // Dotted-path access into one section, for example floatingAtPath(
    // "scene", "sun.intensity") or nested array-of-table rows such as
    // "objects[0].position". Shared world content ([scene]) and host-only
    // sections live outside any single component's own section; these helpers
    // keep that readable without exposing the TOML parser to modules.
    std::optional<std::string> stringAtPath(const std::string& sectionName, const std::string& path) const;
    std::vector<std::string> stringArrayAtPath(const std::string& sectionName, const std::string& path) const;
    std::optional<std::int64_t> integerAtPath(const std::string& sectionName, const std::string& path) const;
    std::optional<bool> booleanAtPath(const std::string& sectionName, const std::string& path) const;
    std::optional<double> floatingAtPath(const std::string& sectionName, const std::string& path) const;
    std::vector<double> floatingArrayAtPath(const std::string& sectionName, const std::string& path) const;
    // Number of elements of an array at the path (array-of-tables rows or
    // plain arrays); 0 when absent.
    std::size_t arraySizeAtPath(const std::string& sectionName, const std::string& path) const;

    // ---- provenance -----------------------------------------------------------------

    // The layer and source that provided the value at the given path, for
    // example provenanceFor("scene", "sun.intensity") after
    // --set scene.sun.intensity=2. Arrays are tracked at the array's path,
    // not per element. Empty when the key is absent.
    std::optional<ConfigProvenance> provenanceFor(const std::string& sectionName, const std::string& path) const;

    // One "<path> <- <layer (source)>" line per effective value, sorted by
    // path. Empty string when nothing is loaded.
    std::string provenanceReport() const;

    // ---- known keys -----------------------------------------------------------------

    // Declares one configuration key a component reads, as
    // "<section>.<dotted path>" ("renderer.exposure", "scene.camera.fov").
    // A declaration without a dot ("providers") marks the whole section as
    // known: its keys are dynamic (capability names) and cannot be enumerated.
    // Array-of-tables rows are covered by declaring the array's path
    // ("scene.objects"). Returns false (and declares nothing) when the key is
    // malformed; the kernel reports malformed manifest declarations as
    // startup problems.
    bool declareKnownConfigKey(const std::string& dottedKeyPath);

    // The assembly- and host-owned sections the kernel itself reads:
    // [engine], [providers], [permissions], [diagnostics] and [leak_test].
    void declareAssemblyKnownKeys();

    // One diagnostic per effective key that no component declares and no
    // shipped default owns. Warnings only: --validate prints them and still
    // succeeds, because dead keys are noise, not breakage. Keys that only a
    // shipped defaults file provided are exempt - a package owns the keys it
    // ships.
    std::vector<std::string> unknownKeyWarnings() const;

private:
    const toml::table* sectionTable(const std::string& sectionName) const;

    // Deep merge where every source leaf or subtree REPLACES the destination
    // (tables merge recursively, arrays and scalars overwrite wholesale).
    void mergeTableAsOverride(toml::table& destination, const toml::table& source, const std::string& prefix,
                              const ConfigProvenance& provenance);

    // Deep merge where the source FILLS: keys the destination already has are
    // never touched; recursion continues into tables present on both sides.
    void mergeTableAsDefaults(toml::table& destination, const toml::table& source, const std::string& prefix,
                              const ConfigProvenance& provenance);

    // Records provenance for every leaf of the subtree (tables recurse;
    // arrays and scalars are leaves), keyed by dotted path.
    void recordProvenance(const toml::node& subtree, const std::string& prefix, const ConfigProvenance& provenance);

    // Drops provenance entries at the path and everywhere below it, used when
    // a higher layer replaces a subtree wholesale.
    void eraseProvenanceUnder(const std::string& prefix);

    toml::table* parsedTable = nullptr;
    std::map<std::string, ConfigProvenance> provenanceByPath;

    struct StagedOverride
    {
        std::vector<std::string> pathSegments;
        // Unique pointer keeps toml++ out of this header (the type is only
        // forward declared here); the destructor instantiates in the .cpp.
        std::unique_ptr<toml::table> parsedValue; // holds the key "value" when isPlainString is false
        bool isPlainString = false;
        std::string plainStringValue;
    };
    std::vector<StagedOverride> stagedOverrides;

    // section name -> set of known dotted key paths ("" = the whole section
    // is known: every key in it is dynamic).
    std::map<std::string, std::set<std::string>> knownKeys;
};

} // namespace engine

#endif // ENGINE_KERNEL_CONFIGURATION_HPP
