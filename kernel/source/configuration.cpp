#include "engine/kernel/configuration.hpp"

#include <toml.hpp>

namespace engine
{

namespace
{

// Walks every leaf path of one table. Sub-tables recurse; scalars AND arrays
// are leaves - an array-of-tables row is authored wholesale, so its inner keys
// never need individual treatment ("scene.objects" covers every row's keys).
template <typename Visit>
void walkLeafPaths(
    const toml::table &table, const std::string &prefix, Visit &&visit
)
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

// TOML bare key characters, shared by --set paths and known-key declarations.
bool isValidKeySegment(
    const std::string &segment
)
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

std::string trimmed(
    const std::string &text
)
{
    const std::size_t begin = text.find_first_not_of(" \t");
    if (begin == std::string::npos)
    {
        return std::string();
    }
    const std::size_t end = text.find_last_not_of(" \t");
    return text.substr(begin, end - begin + 1);
}

// Sections a shipped defaults file must never define: they belong to the
// assembly ([engine], [providers], [permissions]) or to the host and kernel
// ([diagnostics], [leak_test]). A package that could grow the module list or
// deny permissions would break the authority model the kernel is built on.
bool isAssemblyOwnedSection(
    const std::string &sectionName
)
{
    return sectionName == ConfigurationStore::engineSectionName
        || sectionName == ConfigurationStore::providersSectionName
        || sectionName == ConfigurationStore::permissionsSectionName
        || sectionName == ConfigurationStore::diagnosticsSectionName
        || sectionName == ConfigurationStore::leakTestSectionName;
}

std::string parseErrorDiagnostic(
    const std::filesystem::path &path, const toml::parse_error &error
)
{
    std::string message = std::string("configuration parse error in '") + path.string()
                        + "': " + std::string(error.description());
    message += " (line " + std::to_string(error.source().begin.line) + ", column "
             + std::to_string(error.source().begin.column) + ")";
    return message;
}

} // namespace

std::string ConfigProvenance::layerName() const
{
    switch (layer)
    {
    case Layer::CommandLine:
        return "command line (--set)";
    case Layer::EngineConfig:
        return "engine configuration";
    case Layer::SceneFile:
        return "scene file";
    case Layer::ModuleDefaults:
        return "module defaults";
    case Layer::ModDefaults:
        return "mod defaults";
    }
    return "unknown layer";
}

ConfigurationStore::ConfigurationStore() = default;

ConfigurationStore::~ConfigurationStore()
{
    delete parsedTable;
    parsedTable = nullptr;
}

void ConfigurationStore::reset()
{
    delete parsedTable;
    parsedTable = nullptr;
    provenanceByPath.clear();
    stagedOverrides.clear();
    knownKeys.clear();
}

// ---- layered assembly ----------------------------------------------------------

bool ConfigurationStore::loadFromFile(
    const std::filesystem::path &path, std::string &errorMessage
)
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
        mergeTableAsOverride(*parsedTable, parsed, "", provenance);
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

bool ConfigurationStore::mergeDefaultsFile(
    const std::filesystem::path &path,
    ConfigProvenance::Layer      layer,
    const std::string           &source,
    std::string                 &errorMessage
)
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
                         + "' section: assembly and host sections belong to the engine configuration and the host";
            return false;
        }
        if (sectionName == sceneSectionName)
        {
            // World content has a dedicated channel: a mod's declared scene
            // file or the assembly's [scene] section, replaced wholesale by
            // --scene. A defaults file would drip its scene keys into every
            // configuration that does not define the same keys - including
            // sealed-room test configurations that rely on an untouched
            // zero sky - so it is rejected instead of merged.
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
        // The kernel always loads the assembly layer first; the store still
        // tolerates a defaults-only composition (tests rely on it).
        parsedTable = new toml::table();
    }
    const ConfigProvenance provenance{.layer = layer, .source = source};
    mergeTableAsDefaults(*parsedTable, parsed, "", provenance);
    return true;
}

bool ConfigurationStore::applySceneFile(
    const std::filesystem::path &path, std::string &errorMessage
)
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
            errorMessage = "scene file '" + path.string() + "' may only define the [scene] section; found '" + topKey
                         + "'";
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
    // Wholesale swap: the scene file owns the world content. Assembly-side
    // [scene] keys do not leak into the swapped world - a scene author who
    // wants an assembly-side key copies it into the scene file. This is what
    // keeps [[scene.objects]] predictable: the objects the run uses are the
    // objects the scene file lists, never a merge with the demo room.
    parsedTable->erase(sceneSectionName);
    eraseProvenanceUnder(sceneSectionName);
    const ConfigProvenance provenance{.layer = ConfigProvenance::Layer::SceneFile, .source = path.string()};
    parsedTable->insert_or_assign(sceneSectionName, *sceneTable);
    recordProvenance(*sceneTable, sceneSectionName, provenance);
    return true;
}

bool ConfigurationStore::stageCommandLineOverride(
    const std::string &assignment, std::string &errorMessage
)
{
    errorMessage.clear();
    const std::size_t separator = assignment.find('=');
    if (separator == std::string::npos)
    {
        errorMessage = "--set expects <key>=<value>, got '" + assignment + "' (no '=')";
        return false;
    }
    const std::string key      = trimmed(assignment.substr(0, separator));
    const std::string rawValue = trimmed(assignment.substr(separator + 1));
    if (key.empty() || key.front() == '.' || key.back() == '.' || key.find("..") != std::string::npos)
    {
        errorMessage = "--set key '" + key
                     + "' is not a dotted path of bare key segments (example: scene.sun.intensity=2.0)";
        return false;
    }

    StagedOverride staged;
    std::size_t    position = 0;
    while (position < key.size())
    {
        const std::size_t dot     = key.find('.', position);
        std::string       segment = key.substr(position, dot == std::string::npos ? std::string::npos : dot - position);
        position                  = (dot == std::string::npos) ? key.size() : dot + 1;
        if (!isValidKeySegment(segment))
        {
            errorMessage = "--set key '" + key
                         + "' is not a dotted path of bare key segments (example: scene.sun.intensity=2.0)";
            return false;
        }
        staged.pathSegments.push_back(std::move(segment));
    }

    // TOML value syntax first (numbers, booleans, quoted strings, arrays,
    // inline tables); a raw value that TOML rejects becomes a literal string,
    // so --set window.title=My Room works without quotes.
    bool parsedAsToml = false;
    if (!rawValue.empty())
    {
        try
        {
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

bool ConfigurationStore::applyCommandLineOverrides(
    std::string &errorMessage
)
{
    errorMessage.clear();
    if (parsedTable == nullptr)
    {
        parsedTable = new toml::table();
    }
    const ConfigProvenance provenance{.layer = ConfigProvenance::Layer::CommandLine, .source = "--set"};
    for (const StagedOverride &staged : stagedOverrides)
    {
        std::string fullPath;
        for (const std::string &segment : staged.pathSegments)
        {
            fullPath += (fullPath.empty() ? "" : ".") + segment;
        }

        // Navigate (and create) the intermediate tables.
        toml::table *current = parsedTable;
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
                errorMessage     = "--set " + fullPath + ": '" + navigated + "' is not a table";
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
            eraseProvenanceUnder(fullPath);
            provenanceByPath[fullPath] = provenance;
        }
        else
        {
            const toml::node *valueNode = staged.parsedValue->get("value");
            if (valueNode == nullptr)
            {
                errorMessage = "--set " + fullPath + ": the staged value is missing (internal error)";
                return false;
            }
            current->insert_or_assign(lastSegment, *valueNode);
            eraseProvenanceUnder(fullPath);
            recordProvenance(*valueNode, fullPath, provenance);
        }
    }
    stagedOverrides.clear();
    return true;
}

// ---- merge helpers ---------------------------------------------------------------

void ConfigurationStore::mergeTableAsOverride(
    toml::table &destination, const toml::table &source, const std::string &prefix, const ConfigProvenance &provenance
)
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
                mergeTableAsOverride(*destinationTable, *sourceTable, path, provenance);
                continue;
            }
        }
        // Replace wholesale (toml++ deep-copies nodes on insert) and re-record:
        // the new subtree's leaves now come from this layer.
        destination.insert_or_assign(keyString, value);
        eraseProvenanceUnder(path);
        recordProvenance(value, path, provenance);
    }
}

void ConfigurationStore::mergeTableAsDefaults(
    toml::table &destination, const toml::table &source, const std::string &prefix, const ConfigProvenance &provenance
)
{
    for (auto &&[key, value] : source)
    {
        const std::string keyString(key.str());
        const std::string path     = prefix.empty() ? keyString : prefix + "." + keyString;
        toml::node       *existing = destination.get(keyString);
        if (existing == nullptr)
        {
            destination.insert_or_assign(keyString, value);
            recordProvenance(value, path, provenance);
            continue;
        }
        if (const auto *sourceTable = value.as_table())
        {
            if (auto *destinationTable = existing->as_table())
            {
                mergeTableAsDefaults(*destinationTable, *sourceTable, path, provenance);
            }
        }
        // else: the key exists on both sides - a higher layer owns it and the
        // default stays invisible.
    }
}

void ConfigurationStore::recordProvenance(
    const toml::node &subtree, const std::string &prefix, const ConfigProvenance &provenance
)
{
    if (const auto *table = subtree.as_table())
    {
        for (auto &&[key, value] : *table)
        {
            const std::string path = prefix.empty() ? std::string(key.str()) : prefix + "." + std::string(key.str());
            recordProvenance(value, path, provenance);
        }
        return;
    }
    provenanceByPath[prefix] = provenance;
}

void ConfigurationStore::eraseProvenanceUnder(
    const std::string &prefix
)
{
    for (auto iterator = provenanceByPath.begin(); iterator != provenanceByPath.end();)
    {
        const std::string &path    = iterator->first;
        const bool         matches = path == prefix
                                  || (path.size() > prefix.size() && path.compare(0, prefix.size(), prefix) == 0
                                      && path[prefix.size()] == '.');
        if (matches)
        {
            iterator = provenanceByPath.erase(iterator);
        }
        else
        {
            ++iterator;
        }
    }
}

// ---- provenance ------------------------------------------------------------------

std::optional<ConfigProvenance> ConfigurationStore::provenanceFor(
    const std::string &sectionName, const std::string &path
) const
{
    const auto entry = provenanceByPath.find(sectionName + "." + path);
    if (entry == provenanceByPath.end())
    {
        return std::nullopt;
    }
    return entry->second;
}

std::string ConfigurationStore::provenanceReport() const
{
    if (provenanceByPath.empty())
    {
        return std::string();
    }
    std::string report = "configuration provenance (the layer that provided each effective value):\n";
    for (const auto &[path, provenance] : provenanceByPath)
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

// ---- known keys --------------------------------------------------------------------

bool ConfigurationStore::declareKnownConfigKey(
    const std::string &dottedKeyPath
)
{
    if (dottedKeyPath.empty())
    {
        return false;
    }
    const std::size_t separator = dottedKeyPath.find('.');
    if (separator == std::string::npos)
    {
        // No dot: the whole section is known. Used for sections whose keys are
        // dynamic ([providers] maps capability names to providers).
        if (!isValidKeySegment(dottedKeyPath))
        {
            return false;
        }
        knownKeys[dottedKeyPath].insert("");
        return true;
    }
    const std::string section = dottedKeyPath.substr(0, separator);
    const std::string path    = dottedKeyPath.substr(separator + 1);
    if (!isValidKeySegment(section) || path.empty() || path.front() == '.' || path.back() == '.'
        || path.find("..") != std::string::npos)
    {
        return false;
    }
    std::size_t position = 0;
    while (position < path.size())
    {
        const std::size_t dot     = path.find('.', position);
        const std::string segment = path.substr(
            position, dot == std::string::npos ? std::string::npos : dot - position
        );
        position = (dot == std::string::npos) ? path.size() : dot + 1;
        if (!isValidKeySegment(segment))
        {
            return false;
        }
    }
    knownKeys[section].insert(path);
    return true;
}

void ConfigurationStore::declareAssemblyKnownKeys()
{
    // [engine]: the assembly itself. [providers]: capability names are
    // dynamic, the whole section is declared. [permissions] and
    // [diagnostics]: read by the kernel. [leak_test]: read by the host frame
    // loop (--leak-check).
    declareKnownConfigKey("engine.mod_directory");
    declareKnownConfigKey("engine.modules_directory");
    declareKnownConfigKey("engine.disabled_mods");
    declareKnownConfigKey(providersSectionName);
    declareKnownConfigKey("permissions.denied");
    declareKnownConfigKey(std::string(diagnosticsSectionName) + "." + diagnosticsLatencyReportKey);
    declareKnownConfigKey(std::string(leakTestSectionName) + "." + leakTestMaxLuminanceKey);
    declareKnownConfigKey(std::string(leakTestSectionName) + "." + leakTestRegionKey);
    declareKnownConfigKey(std::string(leakTestSectionName) + "." + leakTestLitRegionKey);
    declareKnownConfigKey(std::string(leakTestSectionName) + "." + leakTestMinLitLuminanceKey);
}

std::vector<std::string> ConfigurationStore::unknownKeyWarnings() const
{
    std::vector<std::string> warnings;
    if (parsedTable == nullptr)
    {
        return warnings;
    }
    for (auto &&[sectionKey, sectionNode] : *parsedTable)
    {
        const std::string section(sectionKey.str());
        const auto       *sectionTableNode = sectionNode.as_table();
        if (sectionTableNode == nullptr)
        {
            warnings.push_back(
                "unknown configuration value '" + section + "' (a bare top-level value is read by nobody; use '["
                + section + "] key = value' instead)"
            );
            continue;
        }
        const auto knownSection    = knownKeys.find(section);
        const bool sectionDeclared = knownSection != knownKeys.end();
        if (sectionDeclared && knownSection->second.count("") > 0)
        {
            continue; // the whole section is known: its keys are dynamic
        }
        walkLeafPaths(
            *sectionTableNode,
            section,
            [&](const std::string &leafPath)
            {
                // The walk yields full paths ("engine.mod_directory"); the
                // registry stores section-relative paths
                // ("mod_directory").
                const std::string relativePath = leafPath.size() > section.size() ? leafPath.substr(section.size() + 1)
                                                                                  : leafPath;
                if (sectionDeclared && knownSection->second.count(relativePath) > 0)
                {
                    return;
                }
                const auto provenanceEntry = provenanceByPath.find(leafPath);
                if (provenanceEntry != provenanceByPath.end()
                    && (provenanceEntry->second.layer == ConfigProvenance::Layer::ModuleDefaults
                        || provenanceEntry->second.layer == ConfigProvenance::Layer::ModDefaults))
                {
                    return; // a shipped defaults file owns the keys it provides
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
                    warnings.push_back(
                        "unknown configuration key '" + leafPath + "' (from " + source
                        + "; no component declares keys in '[" + section + "]')"
                    );
                }
            }
        );
    }
    return warnings;
}

// ---- queries -----------------------------------------------------------------------

bool ConfigurationStore::isLoaded() const
{
    return parsedTable != nullptr;
}

bool ConfigurationStore::hasSection(
    const std::string &sectionName
) const
{
    return sectionTable(sectionName) != nullptr;
}

std::string ConfigurationStore::modDirectory() const
{
    // The default matches the runtime layout the build assembles; a value in
    // the configuration relocates the search without touching the kernel.
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

std::string ConfigurationStore::modulesDirectory() const
{
    // The default matches the runtime layout the build assembles; a value in
    // the configuration relocates the scan without touching the kernel.
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
    if (const toml::array *disabledArray = engineTable->get_as<toml::array>("disabled_mods"))
    {
        disabledArray->for_each([&ids](const toml::value<std::string> &element) { ids.push_back(element.get()); });
    }
    return ids;
}

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
            DependencyResolver::ProviderSelection selection;
            selection.capabilityName = std::string(entry.first.str());
            selection.providerName   = providerNode->get();
            selections.push_back(std::move(selection));
        }
    }
    return selections;
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
            const auto componentNode  = pairArray.get_as<std::string>(0);
            const auto permissionNode = pairArray.get_as<std::string>(1);
            if (componentNode != nullptr && permissionNode != nullptr)
            {
                denials.emplace_back(componentNode->get(), permissionNode->get());
            }
        }
    );
    return denials;
}

const toml::table *ConfigurationStore::sectionTable(
    const std::string &sectionName
) const
{
    if (parsedTable == nullptr)
    {
        return nullptr;
    }
    return parsedTable->get_as<toml::table>(sectionName);
}

std::optional<std::string> ConfigurationStore::stringValue(
    const std::string &sectionName, const std::string &key
) const
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
    const std::string &sectionName, const std::string &key
) const
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

std::optional<bool> ConfigurationStore::booleanValue(
    const std::string &sectionName, const std::string &key
) const
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

std::optional<double> ConfigurationStore::floatingValue(
    const std::string &sectionName, const std::string &key
) const
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

// Resolves a dotted path with optional [index] steps against one section.
// Returns null when any step is missing, so callers can apply their defaults.
const toml::node *resolvePath(
    const toml::table *section, const std::string &path
)
{
    if (section == nullptr)
    {
        return nullptr;
    }
    const toml::node *current  = section;
    std::size_t       position = 0;
    while (position < path.size())
    {
        const std::size_t dot  = path.find('.', position);
        std::string       step = path.substr(position, dot == std::string::npos ? std::string::npos : dot - position);
        position               = (dot == std::string::npos) ? path.size() : dot + 1;
        if (step.empty())
        {
            return nullptr;
        }
        // Trailing [index] on the step descends into the array element.
        std::size_t arrayIndex = 0;
        bool        hasIndex   = false;
        if (const std::size_t bracket = step.find('['); bracket != std::string::npos)
        {
            if (step.back() != ']')
            {
                return nullptr;
            }
            const std::string indexText = step.substr(bracket + 1, step.size() - bracket - 2);
            if (indexText.empty() || indexText.find_first_not_of("0123456789") != std::string::npos)
            {
                return nullptr;
            }
            arrayIndex = static_cast<std::size_t>(std::stoull(indexText));
            step       = step.substr(0, bracket);
            hasIndex   = true;
        }
        const auto *table = current->as_table();
        if (table == nullptr)
        {
            return nullptr;
        }
        const toml::node *next = table->get(step);
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
    const std::string &sectionName, const std::string &path
) const
{
    const toml::node *node = resolvePath(sectionTable(sectionName), path);
    if (const auto *value = node == nullptr ? nullptr : node->as_string())
    {
        return value->get();
    }
    return std::nullopt;
}

std::vector<std::string> ConfigurationStore::stringArrayAtPath(
    const std::string &sectionName, const std::string &path
) const
{
    std::vector<std::string> values;
    const toml::node        *node = resolvePath(sectionTable(sectionName), path);
    if (const auto *array = node == nullptr ? nullptr : node->as_array())
    {
        array->for_each([&values](const toml::value<std::string> &element) { values.push_back(element.get()); });
    }
    return values;
}

std::optional<std::int64_t> ConfigurationStore::integerAtPath(
    const std::string &sectionName, const std::string &path
) const
{
    const toml::node *node = resolvePath(sectionTable(sectionName), path);
    if (const auto *value = node == nullptr ? nullptr : node->as_integer())
    {
        return value->get();
    }
    return std::nullopt;
}

std::optional<bool> ConfigurationStore::booleanAtPath(
    const std::string &sectionName, const std::string &path
) const
{
    const toml::node *node = resolvePath(sectionTable(sectionName), path);
    if (const auto *value = node == nullptr ? nullptr : node->as_boolean())
    {
        return value->get();
    }
    return std::nullopt;
}

std::optional<double> ConfigurationStore::floatingAtPath(
    const std::string &sectionName, const std::string &path
) const
{
    const toml::node *node = resolvePath(sectionTable(sectionName), path);
    if (const auto *value = node == nullptr ? nullptr : node->as_floating_point())
    {
        return value->get();
    }
    // Integer-valued configuration is accepted for floating keys, so authors
    // can write intensity = 2 instead of intensity = 2.0.
    if (const auto *integer = node == nullptr ? nullptr : node->as_integer())
    {
        return static_cast<double>(integer->get());
    }
    return std::nullopt;
}

std::vector<double> ConfigurationStore::floatingArrayAtPath(
    const std::string &sectionName, const std::string &path
) const
{
    std::vector<double> values;
    const toml::node   *node = resolvePath(sectionTable(sectionName), path);
    if (const auto *array = node == nullptr ? nullptr : node->as_array())
    {
        array->for_each([&values](const toml::value<double> &element) { values.push_back(element.get()); });
    }
    return values;
}

std::size_t ConfigurationStore::arraySizeAtPath(
    const std::string &sectionName, const std::string &path
) const
{
    const toml::node *node = resolvePath(sectionTable(sectionName), path);
    if (const auto *array = node == nullptr ? nullptr : node->as_array())
    {
        return array->size();
    }
    return 0;
}

} // namespace engine
