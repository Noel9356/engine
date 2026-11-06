// ConfigurationStore implementation: the layered TOML configuration.
//
// engine.toml, module/mod defaults, profiles, scene files and staged
// overrides merge in precedence order; provenance records the winning
// layer per leaf and undeclared keys become warnings.

#include <cstdint>
#include <engine/kernel/configuration.hpp>
#include <engine/kernel/dependency_resolver.hpp>

#include <toml.hpp>

#include <charconv>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace engine
{

namespace
{

// Visits every leaf path ("section.sub.key") of a TOML table in document order.
template <typename Visit>
void walkLeafPaths(const toml::table &table, const std::string &prefix, Visit &&visit)
{
    for (auto &&[key, value] : table)
    {
        const std::string path = prefix.empty() ? std::string(key.str()) : prefix + "." + std::string(key.str());
        if (const auto *subtable = value.as_table())
        {
            walkLeafPaths(*subtable, path, visit);
        }
        else
        {
            visit(path);
        }
    }
}

// A key segment is bare: letters, digits, '_' or '-' only (dots separate segments).
bool isValidKeySegment(const std::string &segment)
{
    if (segment.empty())
    {
        return false;
    }
    for (const char character : segment)
    {
        const bool allowed = (character >= 'a' && character <= 'z') || (character >= 'A' && character <= 'Z')
                          || (character >= '0' && character <= '9') || character == '_' || character == '-';
        if (!allowed)
        {
            return false;
        }
    }
    return true;
}

std::string trimmed(const std::string &text)
{
    const std::size_t begin = text.find_first_not_of(" \t");
    if (begin == std::string::npos)
    {
        return std::string();
    }
    const std::size_t end = text.find_last_not_of(" \t");
    return text.substr(begin, end - begin + 1);
}

// Sections only the engine configuration and the app may define; defaults and
// profiles are refused when they try to write them.
bool isAssemblyOwnedSection(const std::string &sectionName)
{
    return sectionName == ConfigurationStore::engineSectionName
        || sectionName == ConfigurationStore::providersSectionName
        || sectionName == ConfigurationStore::permissionsSectionName
        || sectionName == ConfigurationStore::diagnosticsSectionName
        || sectionName == ConfigurationStore::trustSectionName;
}

std::string parseErrorDiagnostic(const std::filesystem::path &path, const toml::parse_error &error)
{
    std::string message
        = std::string("configuration parse error in '") + path.string() + "': " + std::string(error.description());
    message += " (line " + std::to_string(error.source().begin.line) + ", column "
             + std::to_string(error.source().begin.column) + ")";
    return message;
}

} // namespace

// Maps each leaf path to the ConfigProvenance of the layer that last set it.
// eraseUnder drops the exact path plus everything below it (the map ordering
// bounds the subtree scan).
class ProvenanceTracker final
{
public:
    void record(const toml::node &subtree, const std::string &prefix, const ConfigProvenance &provenance)
    {
        if (const auto *table = subtree.as_table())
        {
            for (auto &&[key, value] : *table)
            {
                const std::string path
                    = prefix.empty() ? std::string(key.str()) : prefix + "." + std::string(key.str());
                record(value, path, provenance);
            }
            return;
        }
        byPath[prefix] = provenance;
    }

    void eraseUnder(const std::string &prefix)
    {
        byPath.erase(prefix);
        const std::string firstSubtreeKey = prefix + ".";
        const std::string afterSubtreeKey = prefix + "/";
        for (auto iterator = byPath.lower_bound(firstSubtreeKey);
            iterator != byPath.end() && iterator->first < afterSubtreeKey;)
        {
            iterator = byPath.erase(iterator);
        }
    }

    void assign(const std::string &path, const ConfigProvenance &provenance)
    {
        byPath[path] = provenance;
    }

    void clear()
    {
        byPath.clear();
    }

    const std::map<std::string, ConfigProvenance> &all() const
    {
        return byPath;
    }

private:
    std::map<std::string, ConfigProvenance> byPath;
};

// Declared config keys per section ("" means the whole section).
// unknownKeyWarnings reports document keys declared by nobody; keys supplied by
// the shipped defaults layers are exempt (they are not hand-written).
class KnownKeyRegistry final
{
public:
    // Accepts "<section>.<path>" of bare segments with an optional trailing ".*"
    // wildcard; false on malformed keys.
    bool declare(const std::string &dottedKeyPath)
    {
        if (dottedKeyPath.empty())
        {
            return false;
        }

        const std::size_t wildcard  = dottedKeyPath.rfind(".*");
        const std::string validated = wildcard != std::string::npos && wildcard + 2 == dottedKeyPath.size()
                                        ? dottedKeyPath.substr(0, wildcard)
                                        : dottedKeyPath;
        const std::size_t separator = validated.find('.');
        if (separator == std::string::npos)
        {
            if (!isValidKeySegment(dottedKeyPath))
            {
                return false;
            }
            knownKeys[dottedKeyPath].insert("");
            return true;
        }
        const std::string section = validated.substr(0, separator);
        const std::string path    = validated.substr(separator + 1);
        if (!isValidKeySegment(section) || path.empty() || path.front() == '.' || path.back() == '.'
            || path.find("..") != std::string::npos)
        {
            return false;
        }
        std::size_t position = 0;
        while (position < path.size())
        {
            const std::size_t dot = path.find('.', position);
            const std::string segment
                = path.substr(position, dot == std::string::npos ? std::string::npos : dot - position);
            position = (dot == std::string::npos) ? path.size() : dot + 1;
            if (!isValidKeySegment(segment))
            {
                return false;
            }
        }
        knownKeys[section].insert(dottedKeyPath.substr(separator + 1));
        return true;
    }

    void clear()
    {
        knownKeys.clear();
    }

    std::vector<std::string> unknownKeyWarnings(
        const toml::table &document, const std::map<std::string, ConfigProvenance> &provenanceByPath) const
    {
        std::vector<std::string> warnings;
        for (auto &&[sectionKey, sectionNode] : document)
        {
            const std::string section(sectionKey.str());
            const auto       *sectionTableNode = sectionNode.as_table();
            if (sectionTableNode == nullptr)
            {
                warnings.push_back("unknown configuration value '" + section
                                   + "' (a bare top-level value is read by nobody; use '[" + section
                                   + "] key = value' instead)");
                continue;
            }
            const auto knownSection    = knownKeys.find(section);
            const bool sectionDeclared = knownSection != knownKeys.end();
            if (sectionDeclared && knownSection->second.count("") > 0)
            {
                continue;
            }
            walkLeafPaths(*sectionTableNode,
                section,
                [&](const std::string &leafPath)
                {
                    const std::string relativePath
                        = leafPath.size() > section.size() ? leafPath.substr(section.size() + 1) : leafPath;
                    if (sectionDeclared && knownSection->second.count(relativePath) > 0)
                    {
                        return;
                    }

                    bool coveredByWildcard = false;
                    if (sectionDeclared)
                    {
                        for (const std::string &known : knownSection->second)
                        {
                            const std::size_t wildcard = known.rfind(".*");
                            if (wildcard != std::string::npos && wildcard + 2 == known.size()
                                && relativePath.rfind(known.substr(0, wildcard), 0) == 0
                                && (relativePath.size() == wildcard || relativePath[wildcard] == '.'))
                            {
                                coveredByWildcard = true;
                                break;
                            }
                        }
                    }
                    if (coveredByWildcard)
                    {
                        return;
                    }
                    const auto provenanceEntry = provenanceByPath.find(leafPath);
                    if (provenanceEntry != provenanceByPath.end()
                        && (provenanceEntry->second.layer == ConfigProvenance::Layer::ModuleDefaults
                            || provenanceEntry->second.layer == ConfigProvenance::Layer::ModDefaults))
                    {
                        return;
                    }
                    const std::string source = provenanceEntry != provenanceByPath.end()
                                                 ? "'" + provenanceEntry->second.source + "'"
                                                 : "an unknown layer";
                    if (sectionDeclared)
                    {
                        warnings.push_back("unknown configuration key '" + leafPath + "' (from " + source + ")");
                    }
                    else
                    {
                        warnings.push_back("unknown configuration key '" + leafPath + "' (from " + source
                                           + "; no participant declares keys in '[" + section + "]')");
                    }
                });
        }
        return warnings;
    }

private:
    std::map<std::string, std::set<std::string>> knownKeys;
};

// Staged "--set key=value" assignments. Values are parsed as TOML literals at
// staging time (falling back to a plain string); apply() navigates the document,
// creating intermediate tables, and rewrites provenance for what it replaces.
class CommandLineOverrides final
{
public:
    bool stage(const std::string &assignment, std::string &errorMessage)
    {
        errorMessage.clear();
        const std::size_t separator = assignment.find('=');
        if (separator == std::string::npos)
        {
            errorMessage = "override assignment expects <key>=<value>, got '" + assignment + "' (no '=')";
            return false;
        }
        const std::string key      = trimmed(assignment.substr(0, separator));
        const std::string rawValue = trimmed(assignment.substr(separator + 1));
        if (key.empty() || key.front() == '.' || key.back() == '.' || key.find("..") != std::string::npos)
        {
            errorMessage = "override key '" + key
                         + "' is not a dotted path of bare key segments (example: scene.sun.intensity=2.0)";
            return false;
        }

        StagedOverride staged;
        std::size_t    position = 0;
        while (position < key.size())
        {
            const std::size_t dot = key.find('.', position);
            std::string segment   = key.substr(position, dot == std::string::npos ? std::string::npos : dot - position);
            position              = (dot == std::string::npos) ? key.size() : dot + 1;
            if (!isValidKeySegment(segment))
            {
                errorMessage = "override key '" + key
                             + "' is not a dotted path of bare key segments (example: scene.sun.intensity=2.0)";
                return false;
            }
            staged.pathSegments.push_back(std::move(segment));
        }

        bool parsedAsToml = false;
        if (!rawValue.empty())
        {
            try
            {
                // Parse through a synthetic table so any TOML scalar/array/table literal is accepted.
                staged.parsedValue = std::make_unique<toml::table>(toml::parse("value = " + rawValue));
                parsedAsToml       = true;
            }
            catch (const toml::parse_error &)
            {
                parsedAsToml = false;
            }
            catch (const std::exception &)
            {
                parsedAsToml = false;
            }
        }
        staged.isPlainString    = !parsedAsToml;
        staged.plainStringValue = rawValue;
        stagedOverrides.push_back(std::move(staged));
        return true;
    }

    bool apply(toml::table &document, ProvenanceTracker &provenance, std::string &errorMessage)
    {
        errorMessage.clear();
        const ConfigProvenance provenanceLayer{
            .layer = ConfigProvenance::Layer::CommandLine, .source = "staged override"};
        for (const StagedOverride &staged : stagedOverrides)
        {
            std::string fullPath;
            for (const std::string &segment : staged.pathSegments)
            {
                fullPath += (fullPath.empty() ? "" : ".") + segment;
            }

            toml::table *current = &document;
            std::string  navigated;
            bool         navigationFailed = false;
            for (std::size_t index = 0; index + 1 < staged.pathSegments.size(); ++index)
            {
                const std::string &segment = staged.pathSegments[index];
                navigated += (navigated.empty() ? "" : ".") + segment;
                toml::node *existing = current->get(segment);
                if (existing == nullptr)
                {
                    current->insert_or_assign(segment, toml::table{});
                    existing = current->get(segment);
                }
                toml::table *next = (existing == nullptr) ? nullptr : existing->as_table();
                if (next == nullptr)
                {
                    errorMessage     = "override " + fullPath + ": '" + navigated + "' is not a table";
                    navigationFailed = true;
                    break;
                }
                current = next;
            }
            if (navigationFailed)
            {
                return false;
            }

            const std::string &lastSegment = staged.pathSegments.back();
            if (staged.isPlainString)
            {
                current->insert_or_assign(lastSegment, staged.plainStringValue);
                provenance.eraseUnder(fullPath);
                provenance.assign(fullPath, provenanceLayer);
            }
            else
            {
                const toml::node *valueNode = staged.parsedValue->get("value");
                if (valueNode == nullptr)
                {
                    errorMessage = "override " + fullPath + ": the staged value is missing (internal error)";
                    return false;
                }
                current->insert_or_assign(lastSegment, *valueNode);
                provenance.eraseUnder(fullPath);
                provenance.record(*valueNode, fullPath, provenanceLayer);
            }
        }

        return true;
    }

    void clear()
    {
        stagedOverrides.clear();
    }

private:
    struct StagedOverride
    {
        std::vector<std::string> pathSegments;

        std::unique_ptr<toml::table> parsedValue;
        bool                         isPlainString = false;
        std::string                  plainStringValue;
    };

    std::vector<StagedOverride> stagedOverrides;
};

std::string ConfigProvenance::layerName() const
{
    switch (layer)
    {
    case Layer::CommandLine:
        return "staged override";
    case Layer::EngineConfig:
        return "engine configuration";
    case Layer::Profile:
        return "runtime profile";
    case Layer::SceneFile:
        return "scene file";
    case Layer::ModuleDefaults:
        return "module defaults";
    case Layer::ModDefaults:
        return "mod defaults";
    }
    return "unknown layer";
}

ConfigurationStore::ConfigurationStore()
{
    provenanceTracker    = std::make_unique<ProvenanceTracker>();
    knownKeyRegistry     = std::make_unique<KnownKeyRegistry>();
    commandLineOverrides = std::make_unique<CommandLineOverrides>();
}

ConfigurationStore::~ConfigurationStore()
{
    delete parsedTable;
    parsedTable = nullptr;
}

// Full invalidation: staged overrides and known keys are dropped too.
void ConfigurationStore::reset()
{
    delete parsedTable;
    parsedTable = nullptr;
    provenanceTracker->clear();
    commandLineOverrides->clear();
    knownKeyRegistry->clear();
}

// Reload keeps staged overrides and known keys; only the document and provenance reset.
void ConfigurationStore::resetForReload()
{
    delete parsedTable;
    parsedTable = nullptr;
    provenanceTracker->clear();
}

// Parses the engine configuration into the live document (override merge: the file
// wins over anything already present).
bool ConfigurationStore::loadFromFile(const std::filesystem::path &path, std::string &errorMessage)
{
    errorMessage.clear();
    try
    {
        toml::table parsed = toml::parse_file(path.string());
        if (parsedTable == nullptr)
        {
            parsedTable = new toml::table();
        }
        const ConfigProvenance provenance{.layer = ConfigProvenance::Layer::EngineConfig, .source = path.string()};
        mergeTableAsOverride(*parsedTable, parsed, "", provenance, *provenanceTracker);
        return true;
    }
    catch (const toml::parse_error &error)
    {
        errorMessage = parseErrorDiagnostic(path, error);
        return false;
    }
    catch (const std::exception &error)
    {
        errorMessage = std::string("configuration error for '") + path.string() + "': " + error.what();
        return false;
    }
}

// Fill-only merge of a shipped defaults file. Defaults may define only non-assembly
// sections and never [scene], so shipped data cannot override world content or policy.
bool ConfigurationStore::mergeDefaultsFile(const std::filesystem::path &path, ConfigProvenance::Layer layer,
    const std::string &source, std::string &errorMessage)
{
    errorMessage.clear();
    if (layer != ConfigProvenance::Layer::ModuleDefaults && layer != ConfigProvenance::Layer::ModDefaults)
    {
        errorMessage = "shipped defaults '" + path.string()
                     + "': only the module defaults or mod defaults layer may merge a defaults file";
        return false;
    }
    toml::table parsed;
    try
    {
        parsed = toml::parse_file(path.string());
    }
    catch (const toml::parse_error &error)
    {
        errorMessage = parseErrorDiagnostic(path, error);
        return false;
    }
    catch (const std::exception &error)
    {
        errorMessage = std::string("configuration error for '") + path.string() + "': " + error.what();
        return false;
    }

    for (auto &&[key, value] : parsed)
    {
        const std::string sectionName(key.str());
        if (isAssemblyOwnedSection(sectionName))
        {
            errorMessage = "shipped defaults file '" + path.string() + "' must not define the '" + sectionName
                         + "' section: assembly and app sections belong to the engine configuration and the app";
            return false;
        }
        if (sectionName == sceneSectionName)
        {
            errorMessage = "shipped defaults file '" + path.string()
                         + "' must not define the 'scene' section: world content belongs to the assembly's "
                           "[scene] or to a mod's declared scene file (mod.toml scene = ...), never to a "
                           "shipped defaults file";
            return false;
        }
        if (value.as_table() == nullptr)
        {
            errorMessage = "shipped defaults file '" + path.string() + "' must only define sections; '" + sectionName
                         + "' is a bare value";
            return false;
        }
    }

    if (parsedTable == nullptr)
    {
        parsedTable = new toml::table();
    }
    const ConfigProvenance provenance{.layer = layer, .source = source};
    mergeTableAsDefaults(*parsedTable, parsed, "", provenance, *provenanceTracker);
    return true;
}

// Merges a runtime profile as an override layer after an allow-list check: profiles
// may tune known engine/diagnostics keys only and must not define scene, providers,
// permissions or trust sections.
bool ConfigurationStore::mergeProfileFile(const std::filesystem::path &path, std::string &errorMessage)
{
    errorMessage.clear();
    toml::table parsed;
    try
    {
        parsed = toml::parse_file(path.string());
    }
    catch (const toml::parse_error &error)
    {
        errorMessage = parseErrorDiagnostic(path, error);
        return false;
    }
    catch (const std::exception &error)
    {
        errorMessage = std::string("configuration profile error for '") + path.string() + "': " + error.what();
        return false;
    }

    static const std::set<std::string> engineTuningKeys = {
        "max_fps",
        "worker_count",
        "max_worker_count",
        "max_outstanding_tasks",
        "priority_aging_ms",
        "queue_source_fairness",
        "idle_poll_ms",
        "config_reload_poll_seconds",
        "fixed_update_hz",
        "max_fixed_update_steps",
        "max_frame_delta_seconds",
        "quit_after_seconds",
    };
    static const std::set<std::string> diagnosticKeys = {
        diagnosticsLatencyReportKey,
        diagnosticsFrameHistoryKey,
        diagnosticsGraphReportKey,
        diagnosticsProfilingEnabledKey,
    };

    for (auto &&[key, value] : parsed)
    {
        const std::string  sectionName(key.str());
        const toml::table *section = value.as_table();
        if (section == nullptr)
        {
            errorMessage = "configuration profile '" + path.string() + "' must define tables; '" + sectionName
                         + "' is a bare value";
            return false;
        }
        if (sectionName == sceneSectionName || sectionName == providersSectionName
            || sectionName == permissionsSectionName || sectionName == trustSectionName)
        {
            errorMessage = "configuration profile '" + path.string() + "' must not define the '" + sectionName
                         + "' section; profiles cannot select scenes, providers, permissions, or trust policy";
            return false;
        }
        if (sectionName == engineSectionName || sectionName == diagnosticsSectionName)
        {
            bool        invalidKeyFound = false;
            std::string invalidKey;
            walkLeafPaths(*section,
                sectionName,
                [&](const std::string &dottedPath)
                {
                    const std::size_t separator = dottedPath.find('.');
                    const std::string leafKey
                        = separator == std::string::npos ? dottedPath : dottedPath.substr(separator + 1);
                    const bool allowed = sectionName == engineSectionName ? engineTuningKeys.contains(leafKey)
                                                                          : diagnosticKeys.contains(leafKey);
                    if (!allowed && !invalidKeyFound)
                    {
                        invalidKeyFound = true;
                        invalidKey      = dottedPath;
                    }
                });
            if (invalidKeyFound)
            {
                errorMessage = "configuration profile '" + path.string()
                             + "' cannot override protected or unknown key '" + invalidKey + "'";
                return false;
            }
        }
    }

    if (parsedTable == nullptr)
    {
        parsedTable = new toml::table();
    }
    const ConfigProvenance provenance{.layer = ConfigProvenance::Layer::Profile, .source = path.string()};
    mergeTableAsOverride(*parsedTable, parsed, "", provenance, *provenanceTracker);
    return true;
}

// Replaces the whole [scene] section (world content is swapped, not merged); the
// file may define nothing but [scene].
bool ConfigurationStore::applySceneFile(const std::filesystem::path &path, std::string &errorMessage)
{
    errorMessage.clear();
    toml::table parsed;
    try
    {
        parsed = toml::parse_file(path.string());
    }
    catch (const toml::parse_error &error)
    {
        errorMessage = parseErrorDiagnostic(path, error);
        return false;
    }
    catch (const std::exception &error)
    {
        errorMessage = std::string("configuration error for '") + path.string() + "': " + error.what();
        return false;
    }

    const toml::table *sceneTable = nullptr;
    for (auto &&[key, value] : parsed)
    {
        const std::string topKey(key.str());
        if (topKey != sceneSectionName)
        {
            errorMessage
                = "scene file '" + path.string() + "' may only define the [scene] section; found '" + topKey + "'";
            return false;
        }
        if (value.as_table() == nullptr)
        {
            errorMessage = "scene file '" + path.string() + "' must define [scene] as a table";
            return false;
        }
        sceneTable = value.as_table();
    }
    if (sceneTable == nullptr)
    {
        errorMessage = "scene file '" + path.string() + "' does not define a [scene] section";
        return false;
    }

    if (parsedTable == nullptr)
    {
        parsedTable = new toml::table();
    }

    parsedTable->erase(sceneSectionName);
    provenanceTracker->eraseUnder(sceneSectionName);
    const ConfigProvenance provenance{.layer = ConfigProvenance::Layer::SceneFile, .source = path.string()};
    parsedTable->insert_or_assign(sceneSectionName, *sceneTable);
    provenanceTracker->record(*sceneTable, sceneSectionName, provenance);
    return true;
}

bool ConfigurationStore::stageCommandLineOverride(const std::string &assignment, std::string &errorMessage)
{
    return commandLineOverrides->stage(assignment, errorMessage);
}

bool ConfigurationStore::applyCommandLineOverrides(std::string &errorMessage)
{
    if (parsedTable == nullptr)
    {
        parsedTable = new toml::table();
    }
    return commandLineOverrides->apply(*parsedTable, *provenanceTracker, errorMessage);
}

// Recursive override merge: source tables merge into destination tables, everything
// else replaces the destination value and its provenance subtree.
void ConfigurationStore::mergeTableAsOverride(toml::table &destination, const toml::table &source,
    const std::string &prefix, const ConfigProvenance &provenance, ProvenanceTracker &provenanceTracker)
{
    for (auto &&[key, value] : source)
    {
        const std::string keyString(key.str());
        const std::string path        = prefix.empty() ? keyString : prefix + "." + keyString;
        const auto       *sourceTable = value.as_table();
        if (sourceTable != nullptr)
        {
            auto *destinationTable = destination.get_as<toml::table>(keyString);
            if (destinationTable != nullptr)
            {
                mergeTableAsOverride(*destinationTable, *sourceTable, path, provenance, provenanceTracker);
                continue;
            }
        }

        destination.insert_or_assign(keyString, value);
        provenanceTracker.eraseUnder(path);
        provenanceTracker.record(value, path, provenance);
    }
}

// Recursive fill-only merge: only keys missing from the destination are inserted,
// so defaults never override the engine configuration.
void ConfigurationStore::mergeTableAsDefaults(toml::table &destination, const toml::table &source,
    const std::string &prefix, const ConfigProvenance &provenance, ProvenanceTracker &provenanceTracker)
{
    for (auto &&[key, value] : source)
    {
        const std::string keyString(key.str());
        const std::string path     = prefix.empty() ? keyString : prefix + "." + keyString;
        toml::node       *existing = destination.get(keyString);
        if (existing == nullptr)
        {
            destination.insert_or_assign(keyString, value);
            provenanceTracker.record(value, path, provenance);
            continue;
        }
        if (const auto *sourceTable = value.as_table())
        {
            if (auto *destinationTable = existing->as_table())
            {
                mergeTableAsDefaults(*destinationTable, *sourceTable, path, provenance, provenanceTracker);
            }
        }
    }
}

std::optional<ConfigProvenance> ConfigurationStore::provenanceFor(
    const std::string &sectionName, const std::string &path) const
{
    const auto &byPath = provenanceTracker->all();
    const auto  entry  = byPath.find(sectionName + "." + path);
    if (entry == byPath.end())
    {
        return std::nullopt;
    }
    return entry->second;
}

// Human-readable "path <- layer" dump for diagnostics.
std::string ConfigurationStore::provenanceReport() const
{
    const auto &byPath = provenanceTracker->all();
    if (byPath.empty())
    {
        return std::string();
    }
    std::string report = "configuration provenance (the layer that provided each effective value):\n";
    for (const auto &[path, provenance] : byPath)
    {
        report += "  " + path + " <- " + provenance.layerName();
        if (provenance.layer != ConfigProvenance::Layer::CommandLine)
        {
            report += " '" + provenance.source + "'";
        }
        report += "\n";
    }
    return report;
}

bool ConfigurationStore::declareKnownConfigKey(const std::string &dottedKeyPath)
{
    return knownKeyRegistry->declare(dottedKeyPath);
}

// Registers the engine-owned sections and keys so they never warn as unknown.
void ConfigurationStore::declareAssemblyKnownKeys()
{
    declareKnownConfigKey("engine.mod_directory");
    declareKnownConfigKey("engine.modules_directory");
    declareKnownConfigKey("engine.disabled_mods");
    declareKnownConfigKey(std::string(engineSectionName) + "." + engineActiveModsKey);
    declareKnownConfigKey(std::string(engineSectionName) + "." + engineExtraModDirectoriesKey);

    declareKnownConfigKey("engine.max_fps");
    declareKnownConfigKey("engine.worker_count");
    declareKnownConfigKey("engine.max_worker_count");
    declareKnownConfigKey("engine.max_outstanding_tasks");
    declareKnownConfigKey("engine.priority_aging_ms");
    declareKnownConfigKey("engine.queue_source_fairness");
    declareKnownConfigKey("engine.idle_poll_ms");
    declareKnownConfigKey(std::string(engineSectionName) + "." + engineFixedUpdateHzKey);
    declareKnownConfigKey(std::string(engineSectionName) + "." + engineMaxFixedUpdateStepsKey);
    declareKnownConfigKey(std::string(engineSectionName) + "." + engineMaxFrameDeltaSecondsKey);
    declareKnownConfigKey(providersSectionName);
    declareKnownConfigKey("permissions.denied");
    declareKnownConfigKey(std::string(diagnosticsSectionName) + "." + diagnosticsLatencyReportKey);

    declareKnownConfigKey(std::string(diagnosticsSectionName) + "." + diagnosticsFrameHistoryKey);

    declareKnownConfigKey(std::string(diagnosticsSectionName) + "." + diagnosticsGraphReportKey);
    declareKnownConfigKey(std::string(diagnosticsSectionName) + "." + diagnosticsProfilingEnabledKey);

    declareKnownConfigKey(std::string(engineSectionName) + "." + engineConfigReloadPollSecondsKey);
    declareKnownConfigKey(std::string(engineSectionName) + "." + engineQuitAfterSecondsKey);

    declareKnownConfigKey(trustSectionName);
}

std::vector<std::string> ConfigurationStore::unknownKeyWarnings() const
{
    if (parsedTable == nullptr)
    {
        return {};
    }
    return knownKeyRegistry->unknownKeyWarnings(*parsedTable, provenanceTracker->all());
}

// Type-tagged text ("s:", "i:", "f:", ...) so reload diffs detect type changes too.
std::string ConfigurationStore::leafValueRepr(const toml::node &value)
{
    if (const auto *stringValue = value.as_string())
    {
        return "s:" + stringValue->get();
    }
    if (const auto *integerValue = value.as_integer())
    {
        return "i:" + std::to_string(integerValue->get());
    }
    if (const auto *floatingValue = value.as_floating_point())
    {
        std::ostringstream text;
        text << std::setprecision(17) << floatingValue->get();
        return "f:" + text.str();
    }
    if (const auto *booleanValue = value.as_boolean())
    {
        return booleanValue->get() ? "b:1" : "b:0";
    }
    if (const auto *dateValue = value.as_date())
    {
        std::ostringstream text;
        text << dateValue->get();
        return "d:" + text.str();
    }
    if (const auto *timeValue = value.as_time())
    {
        std::ostringstream text;
        text << timeValue->get();
        return "t:" + text.str();
    }
    if (const auto *dateTimeValue = value.as_date_time())
    {
        std::ostringstream text;
        text << dateTimeValue->get();
        return "dt:" + text.str();
    }
    return "?:";
}

// Flattens the document to "path -> repr": tables recurse, arrays of tables recurse
// with [i] path segments, other arrays collapse into one "a:N:[...]" value.
void ConfigurationStore::flattenNode(
    const toml::node &subtree, const std::string &path, std::map<std::string, std::string> &out)
{
    if (const auto *table = subtree.as_table())
    {
        for (auto &&[key, value] : *table)
        {
            const std::string childPath = path.empty() ? std::string(key.str()) : path + "." + std::string(key.str());
            flattenNode(value, childPath, out);
        }
        return;
    }
    if (const auto *array = subtree.as_array())
    {
        bool allTables = !array->empty();
        for (const toml::node &element : *array)
        {
            if (element.as_table() == nullptr)
            {
                allTables = false;
                break;
            }
        }
        if (allTables)
        {
            for (std::size_t index = 0; index < array->size(); ++index)
            {
                flattenNode(*array->get(index), path + "[" + std::to_string(index) + "]", out);
            }
            return;
        }
        std::string repr = "a:" + std::to_string(array->size()) + ":[";
        for (std::size_t index = 0; index < array->size(); ++index)
        {
            if (index != 0)
            {
                repr += ",";
            }
            repr += leafValueRepr(*array->get(index));
        }
        out[path] = repr + "]";
        return;
    }
    out[path] = leafValueRepr(subtree);
}

std::map<std::string, std::string> ConfigurationStore::flattenedValueReprs() const
{
    std::map<std::string, std::string> flattened;
    if (parsedTable != nullptr)
    {
        flattenNode(*parsedTable, std::string(), flattened);
    }
    return flattened;
}

// Parse-only probe for the reload pre-check; nothing is stored.
bool ConfigurationStore::fileParses(const std::filesystem::path &path, std::string &errorMessage)
{
    errorMessage.clear();
    try
    {
        const toml::table parsed = toml::parse_file(path.string());
        (void)parsed;
        return true;
    }
    catch (const toml::parse_error &error)
    {
        errorMessage = parseErrorDiagnostic(path, error);
        return false;
    }
    catch (const std::exception &error)
    {
        errorMessage = std::string("configuration error for '") + path.string() + "': " + error.what();
        return false;
    }
}

bool ConfigurationStore::isLoaded() const
{
    return parsedTable != nullptr;
}

bool ConfigurationStore::hasSection(const std::string &sectionName) const
{
    return sectionTable(sectionName) != nullptr;
}

std::string ConfigurationStore::modDirectory() const
{
    std::string directory = "mods";
    if (parsedTable == nullptr)
    {
        return directory;
    }
    const toml::table *engineTable = parsedTable->get_as<toml::table>(engineSectionName);
    if (engineTable == nullptr)
    {
        return directory;
    }
    if (const auto *value = engineTable->get_as<std::string>("mod_directory"))
    {
        directory = value->get();
    }
    return directory;
}

std::vector<std::string> ConfigurationStore::extraModDirectories() const
{
    std::vector<std::string> directories;
    if (parsedTable == nullptr)
    {
        return directories;
    }
    const toml::table *engineTable = parsedTable->get_as<toml::table>(engineSectionName);
    if (engineTable == nullptr)
    {
        return directories;
    }
    if (const toml::array *extraArray = engineTable->get_as<toml::array>(engineExtraModDirectoriesKey))
    {
        extraArray->for_each(
            [&directories](const toml::value<std::string> &element) { directories.push_back(element.get()); });
    }
    return directories;
}

std::string ConfigurationStore::modulesDirectory() const
{
    std::string directory = "modules";
    if (parsedTable == nullptr)
    {
        return directory;
    }
    const toml::table *engineTable = parsedTable->get_as<toml::table>(engineSectionName);
    if (engineTable == nullptr)
    {
        return directory;
    }
    if (const auto *value = engineTable->get_as<std::string>("modules_directory"))
    {
        directory = value->get();
    }
    return directory;
}

std::vector<std::string> ConfigurationStore::disabledMods() const
{
    std::vector<std::string> ids;
    if (parsedTable == nullptr)
    {
        return ids;
    }
    const toml::table *engineTable = parsedTable->get_as<toml::table>(engineSectionName);
    if (engineTable == nullptr)
    {
        return ids;
    }
    if (const toml::array *disabledArray = engineTable->get_as<toml::array>(engineDisabledModsKey))
    {
        disabledArray->for_each([&ids](const toml::value<std::string> &element) { ids.push_back(element.get()); });
    }
    return ids;
}

std::optional<std::vector<std::string>> ConfigurationStore::activeMods() const
{
    if (parsedTable == nullptr)
    {
        return std::nullopt;
    }
    const toml::table *engineTable = parsedTable->get_as<toml::table>(engineSectionName);
    if (engineTable == nullptr)
    {
        return std::nullopt;
    }
    const toml::array *activeArray = engineTable->get_as<toml::array>(engineActiveModsKey);
    if (activeArray == nullptr)
    {
        return std::nullopt;
    }
    std::vector<std::string> ids;
    activeArray->for_each([&ids](const toml::value<std::string> &element) { ids.push_back(element.get()); });
    return ids;
}

// [providers] entries map capability name -> participant name (string values only).
std::vector<DependencyResolver::ProviderSelection> ConfigurationStore::providerSelections() const
{
    std::vector<DependencyResolver::ProviderSelection> selections;
    if (parsedTable == nullptr)
    {
        return selections;
    }
    const toml::table *providersTable = parsedTable->get_as<toml::table>(providersSectionName);
    if (providersTable == nullptr)
    {
        return selections;
    }
    for (const auto &entry : *providersTable)
    {
        if (const auto providerNode = entry.second.as_string())
        {
            selections.push_back(DependencyResolver::ProviderSelection{
                .capabilityName = std::string(entry.first.str()), .providerName = providerNode->get()});
        }
    }
    return selections;
}

bool ConfigurationStore::trustSectionBoolean(const std::string &key) const
{
    if (parsedTable == nullptr)
    {
        return false;
    }
    const toml::table *trustTable = parsedTable->get_as<toml::table>(trustSectionName);
    if (trustTable == nullptr)
    {
        return false;
    }
    if (const auto *value = trustTable->get_as<bool>(key))
    {
        return value->get();
    }
    return false;
}

// [trust] entries are plain names ("mods = [\"id\"]") or tables with id/version/sha256;
// both become TrustBindings (nameless tables are skipped).
std::vector<TrustBinding> ConfigurationStore::trustSectionBindings(const std::string &key) const
{
    std::vector<TrustBinding> bindings;
    if (parsedTable == nullptr)
    {
        return bindings;
    }
    const toml::table *trustTable = parsedTable->get_as<toml::table>(trustSectionName);
    if (trustTable == nullptr)
    {
        return bindings;
    }
    const toml::array *valueArray = trustTable->get_as<toml::array>(key);
    if (valueArray == nullptr)
    {
        return bindings;
    }
    for (const toml::node &element : *valueArray)
    {
        if (const auto *nameValue = element.as_string())
        {
            bindings.push_back(TrustBinding{.name = nameValue->get()});
            continue;
        }
        const auto *bindingTable = element.as_table();
        if (bindingTable == nullptr)
        {
            continue;
        }
        TrustBinding binding;
        if (const toml::node *nameNode = bindingTable->get("id"); nameNode != nullptr)
        {
            if (const auto *nameString = nameNode->as_string())
            {
                binding.name = nameString->get();
            }
        }
        if (binding.name.empty())
        {
            if (const toml::node *nameNode = bindingTable->get("name"); nameNode != nullptr)
            {
                if (const auto *nameString = nameNode->as_string())
                {
                    binding.name = nameString->get();
                }
            }
        }
        if (const toml::node *versionNode = bindingTable->get("version"); versionNode != nullptr)
        {
            if (const auto *versionString = versionNode->as_string())
            {
                binding.version = versionString->get();
            }
        }
        if (const toml::node *shaNode = bindingTable->get("sha256"); shaNode != nullptr)
        {
            if (const auto *shaString = shaNode->as_string())
            {
                binding.sha256 = shaString->get();
            }
        }
        if (!binding.name.empty())
        {
            bindings.push_back(std::move(binding));
        }
    }
    return bindings;
}

std::vector<std::string> ConfigurationStore::trustSectionStringArray(const std::string &key) const
{
    std::vector<std::string> values;
    const toml::table       *trustTable = sectionTable(trustSectionName);
    if (trustTable == nullptr)
    {
        return values;
    }
    if (const toml::array *valueArray = trustTable->get_as<toml::array>(key))
    {
        valueArray->for_each([&values](const toml::value<std::string> &element) { values.push_back(element.get()); });
    }
    return values;
}

std::vector<std::string> ConfigurationStore::trustedMods() const
{
    return trustSectionStringArray(trustModsKey);
}

bool ConfigurationStore::trustAllMods() const
{
    return trustSectionBoolean(trustAllModsKey);
}

std::vector<std::string> ConfigurationStore::trustedModules() const
{
    return trustSectionStringArray(trustModulesKey);
}

bool ConfigurationStore::trustAllModules() const
{
    return trustSectionBoolean(trustAllModulesKey);
}

std::vector<TrustBinding> ConfigurationStore::trustedModBindings() const
{
    return trustSectionBindings(trustModsKey);
}

std::vector<TrustBinding> ConfigurationStore::trustedModuleBindings() const
{
    return trustSectionBindings(trustModulesKey);
}

std::vector<std::pair<std::string, std::string>> ConfigurationStore::permissionDenials() const
{
    std::vector<std::pair<std::string, std::string>> denials;
    if (parsedTable == nullptr)
    {
        return denials;
    }
    const toml::table *permissionsTable = parsedTable->get_as<toml::table>(permissionsSectionName);
    if (permissionsTable == nullptr)
    {
        return denials;
    }
    const toml::array *deniedArray = permissionsTable->get_as<toml::array>("denied");
    if (deniedArray == nullptr)
    {
        return denials;
    }
    deniedArray->for_each(
        [&denials](const toml::array &pairArray)
        {
            if (pairArray.size() != 2)
            {
                return;
            }
            const auto participantNode = pairArray.get_as<std::string>(0);
            const auto permissionNode  = pairArray.get_as<std::string>(1);
            if (participantNode != nullptr && permissionNode != nullptr)
            {
                denials.emplace_back(participantNode->get(), permissionNode->get());
            }
        });
    return denials;
}

const toml::table *ConfigurationStore::sectionTable(const std::string &sectionName) const
{
    if (parsedTable == nullptr)
    {
        return nullptr;
    }
    return parsedTable->get_as<toml::table>(sectionName);
}

std::optional<std::string> ConfigurationStore::stringValue(const std::string &sectionName, const std::string &key) const
{
    const toml::table *section = sectionTable(sectionName);
    if (section == nullptr)
    {
        return std::nullopt;
    }
    if (const auto value = section->get_as<std::string>(key))
    {
        return value->get();
    }
    return std::nullopt;
}

std::optional<std::int64_t> ConfigurationStore::integerValue(
    const std::string &sectionName, const std::string &key) const
{
    const toml::table *section = sectionTable(sectionName);
    if (section == nullptr)
    {
        return std::nullopt;
    }
    if (const auto value = section->get_as<std::int64_t>(key))
    {
        return value->get();
    }
    return std::nullopt;
}

std::optional<bool> ConfigurationStore::booleanValue(const std::string &sectionName, const std::string &key) const
{
    const toml::table *section = sectionTable(sectionName);
    if (section == nullptr)
    {
        return std::nullopt;
    }
    if (const auto value = section->get_as<bool>(key))
    {
        return value->get();
    }
    return std::nullopt;
}

std::optional<double> ConfigurationStore::floatingValue(const std::string &sectionName, const std::string &key) const
{
    const toml::table *section = sectionTable(sectionName);
    if (section == nullptr)
    {
        return std::nullopt;
    }
    if (const auto value = section->get_as<double>(key))
    {
        return value->get();
    }
    return std::nullopt;
}

namespace
{

// Walks a dotted path with optional "key[N]" array indexing; null when any step is
// missing or malformed.
const toml::node *resolvePath(const toml::table *section, const std::string &path)
{
    if (section == nullptr)
    {
        return nullptr;
    }
    const toml::node *current = section;
    std::string_view  remaining(path);
    std::size_t       position = 0;
    while (position < path.size())
    {
        const std::size_t      dot = path.find('.', position);
        const std::string_view step
            = remaining.substr(position, dot == std::string::npos ? std::string::npos : dot - position);
        position = (dot == std::string::npos) ? path.size() : dot + 1;
        if (step.empty())
        {
            return nullptr;
        }

        std::size_t      arrayIndex = 0;
        bool             hasIndex   = false;
        std::string_view key        = step;
        if (const std::size_t bracket = step.find('['); bracket != std::string_view::npos)
        {
            if (step.back() != ']')
            {
                return nullptr;
            }
            const std::string_view indexText = step.substr(bracket + 1, step.size() - bracket - 2);

            const auto parse = std::from_chars(indexText.data(), indexText.data() + indexText.size(), arrayIndex);
            if (indexText.empty() || parse.ec != std::errc{} || parse.ptr != indexText.data() + indexText.size())
            {
                return nullptr;
            }
            key      = step.substr(0, bracket);
            hasIndex = true;
        }
        const auto *table = current->as_table();
        if (table == nullptr)
        {
            return nullptr;
        }
        const toml::node *next = table->get(key);
        if (next == nullptr)
        {
            return nullptr;
        }
        if (hasIndex)
        {
            const auto *array = next->as_array();
            if (array == nullptr || arrayIndex >= array->size())
            {
                return nullptr;
            }
            next = array->get(arrayIndex);
        }
        current = next;
    }
    return current;
}

} // namespace

std::optional<std::string> ConfigurationStore::stringAtPath(
    const std::string &sectionName, const std::string &path) const
{
    const toml::node *node = resolvePath(sectionTable(sectionName), path);
    if (const auto *value = node == nullptr ? nullptr : node->as_string())
    {
        return value->get();
    }
    return std::nullopt;
}

std::vector<std::string> ConfigurationStore::stringArrayAtPath(
    const std::string &sectionName, const std::string &path) const
{
    std::vector<std::string> values;
    const toml::node        *node = resolvePath(sectionTable(sectionName), path);
    if (const auto *array = node == nullptr ? nullptr : node->as_array())
    {
        array->for_each([&values](const toml::value<std::string> &element) { values.push_back(element.get()); });
    }
    return values;
}

std::vector<std::string> ConfigurationStore::tableKeysAtPath(
    const std::string &sectionName, const std::string &path) const
{
    std::vector<std::string> keys;
    const toml::node        *node = resolvePath(sectionTable(sectionName), path);
    if (const auto *table = node == nullptr ? nullptr : node->as_table())
    {
        table->for_each([&keys](const toml::key &key, const toml::node &) { keys.emplace_back(key.str()); });
    }
    return keys;
}

std::optional<std::int64_t> ConfigurationStore::integerAtPath(
    const std::string &sectionName, const std::string &path) const
{
    const toml::node *node = resolvePath(sectionTable(sectionName), path);
    if (const auto *value = node == nullptr ? nullptr : node->as_integer())
    {
        return value->get();
    }
    return std::nullopt;
}

std::optional<bool> ConfigurationStore::booleanAtPath(const std::string &sectionName, const std::string &path) const
{
    const toml::node *node = resolvePath(sectionTable(sectionName), path);
    if (const auto *value = node == nullptr ? nullptr : node->as_boolean())
    {
        return value->get();
    }
    return std::nullopt;
}

// Floating lookup also accepts a stored integer (TOML whole numbers may be integers).
std::optional<double> ConfigurationStore::floatingAtPath(const std::string &sectionName, const std::string &path) const
{
    const toml::node *node = resolvePath(sectionTable(sectionName), path);
    if (const auto *value = node == nullptr ? nullptr : node->as_floating_point())
    {
        return value->get();
    }

    if (const auto *integer = node == nullptr ? nullptr : node->as_integer())
    {
        return static_cast<double>(integer->get());
    }
    return std::nullopt;
}

std::vector<double> ConfigurationStore::floatingArrayAtPath(
    const std::string &sectionName, const std::string &path) const
{
    std::vector<double> values;
    const toml::node   *node = resolvePath(sectionTable(sectionName), path);
    if (const auto *array = node == nullptr ? nullptr : node->as_array())
    {
        array->for_each([&values](const toml::value<double> &element) { values.push_back(element.get()); });
    }
    return values;
}

std::size_t ConfigurationStore::arraySizeAtPath(const std::string &sectionName, const std::string &path) const
{
    const toml::node *node = resolvePath(sectionTable(sectionName), path);
    if (const auto *array = node == nullptr ? nullptr : node->as_array())
    {
        return array->size();
    }
    return 0;
}

} // namespace engine
