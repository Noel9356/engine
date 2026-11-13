// Recipe serialization for the character module.
//
// Assembly recipes are versioned, declarative TOML documents: the parts,
// their structural attachments (with authored connector correction offsets),
// the parameter values, and the assembly root placement. Recipes are the
// character storage format; presets are just recipes shipped as content.
//
// Format (recipe format version 1):
//
//   format_version = 1
//   profile        = "hybrid_biped"        # optional metadata
//
//   [root]
//   translation = [0.0, 0.0, 0.0]          # assembly root placement
//   rotation    = [0.0, 0.0, 0.0, 1.0]     # quaternion x, y, z, w
//
//   [[parts]]
//   name  = "torso"
//   asset = "torso_humanoid"
//
//   [[connections]]
//   parent           = "torso"
//   parent_connector = "spine.top"
//   child            = "head"
//   child_connector  = "skull.base"
//   offset_translation = [0.0, 0.0, 0.0]   # optional authored correction
//   offset_rotation    = [0.0, 0.0, 0.0, 1.0]
//
//   [parameters]
//   "body.height" = 1.0
//
// Floats are written with full round-trip precision and every loaded value is
// resolved and checked before the assembly is touched, so a rejected recipe
// leaves the current character untouched.

#include "math_utils.hpp"
#include "records.hpp"
#include "registry.hpp"

#include <toml.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine::character
{
namespace internal
{
namespace
{

// Numbers are accepted as TOML integers or floats; anything else keeps the
// fallback.
double readNumber(const toml::node *node, double fallback)
{
    if (node == nullptr)
    {
        return fallback;
    }
    if (const auto *value = node->as_floating_point(); value != nullptr)
    {
        return value->get();
    }
    if (const auto *value = node->as_integer(); value != nullptr)
    {
        return static_cast<double>(value->get());
    }
    return fallback;
}

// True when node holds an integral TOML value (integers and whole floats).
bool readIntegral(const toml::node *node, std::uint64_t &outValue)
{
    if (node == nullptr)
    {
        return false;
    }
    if (const auto *value = node->as_integer(); value != nullptr)
    {
        if (value->get() < 0)
        {
            return false;
        }
        outValue = static_cast<std::uint64_t>(value->get());
        return true;
    }
    if (const auto *value = node->as_floating_point(); value != nullptr)
    {
        const double number = value->get();
        if (!std::isfinite(number) || number < 0.0 || number != std::floor(number)
            || number > static_cast<double>(std::numeric_limits<std::uint64_t>::max()))
        {
            return false;
        }
        outValue = static_cast<std::uint64_t>(number);
        return true;
    }
    return false;
}

glm::vec3 readVector3(const toml::node *node, const glm::vec3 &fallback)
{
    if (node == nullptr)
    {
        return fallback;
    }
    const auto *array = node->as_array();
    if (array == nullptr || array->size() < 3)
    {
        return fallback;
    }
    const glm::vec3 value(static_cast<float>(readNumber(array->get(0), fallback.x)),
        static_cast<float>(readNumber(array->get(1), fallback.y)),
        static_cast<float>(readNumber(array->get(2), fallback.z)));
    if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z))
    {
        return fallback;
    }
    return value;
}

glm::quat readQuaternion(const toml::node *node, const glm::quat &fallback)
{
    if (node == nullptr)
    {
        return fallback;
    }
    const auto *array = node->as_array();
    if (array == nullptr || array->size() < 4)
    {
        return fallback;
    }
    const glm::quat rotation(static_cast<float>(readNumber(array->get(3), fallback.w)),
        static_cast<float>(readNumber(array->get(0), fallback.x)),
        static_cast<float>(readNumber(array->get(1), fallback.y)),
        static_cast<float>(readNumber(array->get(2), fallback.z)));
    const float     length = glm::length(rotation);
    if (!std::isfinite(length) || length < 1.0e-8f)
    {
        return fallback;
    }
    return rotation / length;
}

std::string readString(const toml::node *node)
{
    if (node == nullptr)
    {
        return {};
    }
    return node->value_or(std::string{});
}

// Minimal TOML basic-string escaping for the writer: quotes, backslashes,
// and every control character (short control escapes when defined, unicode
// escapes otherwise) so instance names can never break out of the string.
std::string tomlString(const std::string &value)
{
    std::string escaped;
    escaped.reserve(value.size() + 2);
    escaped += '"';
    for (const char character : value)
    {
        const unsigned char code = static_cast<unsigned char>(character);
        switch (character)
        {
        case '"':
            escaped += "\\\"";
            break;
        case '\\':
            escaped += "\\\\";
            break;
        case '\n':
            escaped += "\\n";
            break;
        case '\r':
            escaped += "\\r";
            break;
        case '\t':
            escaped += "\\t";
            break;
        case '\b':
            escaped += "\\b";
            break;
        case '\f':
            escaped += "\\f";
            break;
        default:
            if (code < 0x20 || code == 0x7f)
            {
                static constexpr char digits[] = "0123456789ABCDEF";
                escaped += "\\u00";
                escaped += digits[(code >> 4) & 0xF];
                escaped += digits[code & 0xF];
            }
            else
            {
                escaped += character;
            }
            break;
        }
    }
    escaped += '"';
    return escaped;
}

// Full float round-trip precision: max_digits10 for the type, so a
// save/load cycle reproduces the stored values bit-exactly.
void writeFloat(std::ostream &stream, float value)
{
    char buffer[64] = {};
    std::snprintf(buffer, sizeof(buffer), "%.9g", static_cast<double>(value));
    stream << buffer;
}

void writeVector3(std::ostream &stream, const char *key, const glm::vec3 &value)
{
    stream << key << " = [";
    writeFloat(stream, value.x);
    stream << ", ";
    writeFloat(stream, value.y);
    stream << ", ";
    writeFloat(stream, value.z);
    stream << "]\n";
}

void writeQuaternion(std::ostream &stream, const char *key, const glm::quat &value)
{
    stream << key << " = [";
    writeFloat(stream, value.x);
    stream << ", ";
    writeFloat(stream, value.y);
    stream << ", ";
    writeFloat(stream, value.z);
    stream << ", ";
    writeFloat(stream, value.w);
    stream << "]\n";
}

} // namespace

bool saveRecipeToFile(const Registry &registry, const CharacterRecord &record, const std::string &filePath,
    std::string &errorMessage)
{
    std::ostringstream stream;
    stream << "# Character assembly recipe (engine character module).\n";
    stream << "# Format " << recipeFormatVersion << ": parts, structural attachments, parameters.\n\n";
    stream << "format_version = " << recipeFormatVersion << "\n";
    if (!record.profileId.empty())
    {
        stream << "profile = " << tomlString(record.profileId) << "\n";
    }
    stream << "\n[root]\n";
    writeVector3(stream, "translation", record.rootTranslation);
    writeQuaternion(stream, "rotation", record.rootRotation);

    stream << "\n";
    for (const PartInstance &part : record.parts)
    {
        stream << "[[parts]]\n";
        stream << "name  = " << tomlString(part.name) << "\n";
        stream << "asset = " << tomlString(part.partId) << "\n\n";
    }

    for (const ConnectionRecord &connection : record.connections)
    {
        const auto parentIndex = record.instanceIndicesById.find(connection.parent);
        const auto childIndex  = record.instanceIndicesById.find(connection.child);
        if (parentIndex == record.instanceIndicesById.end() || childIndex == record.instanceIndicesById.end())
        {
            continue;
        }
        const PartInstance &parent = record.parts[parentIndex->second];
        const PartInstance &child  = record.parts[childIndex->second];
        stream << "[[connections]]\n";
        stream << "parent           = " << tomlString(parent.name) << "\n";
        stream << "parent_connector = " << tomlString(connection.parentConnector) << "\n";
        stream << "child            = " << tomlString(child.name) << "\n";
        stream << "child_connector  = " << tomlString(connection.childConnector) << "\n";
        writeVector3(stream, "offset_translation", connection.offsetTranslation);
        writeQuaternion(stream, "offset_rotation", connection.offsetRotation);
        stream << "\n";
    }

    stream << "[parameters]\n";
    // Deterministic (sorted) order for stable diffs across saves.
    std::set<std::string> parameterIds;
    for (const ParameterRecord &parameter : registry.parameters)
    {
        parameterIds.insert(parameter.id);
    }
    for (const std::string &parameterId : parameterIds)
    {
        const auto index = registry.parameterIndices.find(parameterId);
        if (index == registry.parameterIndices.end())
        {
            continue;
        }
        const float value = index->second < record.parameterValues.size()
            ? record.parameterValues[index->second]
            : registry.parameters[index->second].defaultV;
        stream << tomlString(parameterId) << " = ";
        writeFloat(stream, value);
        stream << "\n";
    }

    std::ofstream file(filePath, std::ios::binary | std::ios::trunc);
    if (!file)
    {
        errorMessage = "cannot open '" + filePath + "' for writing";
        return false;
    }
    file << stream.str();
    file.flush();
    if (!file)
    {
        errorMessage = "failed writing '" + filePath + "'";
        return false;
    }
    return true;
}

bool loadRecipeFromFile(
    const Registry &registry, const Budgets &budgets, CharacterRecord &record, const std::string &filePath,
    std::string &errorMessage)
{
    toml::table document;
    try
    {
        document = toml::parse_file(filePath);
    }
    catch (const toml::parse_error &error)
    {
        errorMessage = std::string("parse failed: ") + error.what();
        return false;
    }
    catch (const std::exception &error)
    {
        errorMessage = std::string("parse failed: ") + error.what();
        return false;
    }

    if (const toml::node *versionNode = document.get("format_version"))
    {
        std::uint64_t version = 0;
        if (!readIntegral(versionNode, version) || version != recipeFormatVersion)
        {
            errorMessage = "unsupported recipe format version (expected "
                         + std::to_string(recipeFormatVersion) + ")";
            return false;
        }
    }

    // Content replacement happens only after every lookup and check
    // succeeded, so a bad recipe leaves the current character untouched.
    const std::string profileId = readString(document.get("profile"));
    if (!profileId.empty() && registry.profileIndices.count(profileId) == 0)
    {
        errorMessage = "recipe references unregistered profile '" + profileId + "'";
        return false;
    }

    struct LoadedPart
    {
        std::string       name;
        const PartRecord *asset = nullptr;
        InstanceId        id    = invalidInstance;
    };
    std::vector<LoadedPart> loadedParts;
    if (const toml::node *partsNode = document.get("parts"))
    {
        const auto *parts = partsNode->as_array();
        if (parts == nullptr)
        {
            errorMessage = "'parts' must be an array of tables";
            return false;
        }
        if (parts->size() > budgets.maximumPartsPerCharacter)
        {
            errorMessage = "recipe declares " + std::to_string(parts->size()) + " parts but the per-assembly budget "
                         "allows " + std::to_string(budgets.maximumPartsPerCharacter);
            return false;
        }
        loadedParts.reserve(parts->size());
        for (const toml::node &element : *parts)
        {
            const auto *entry = element.as_table();
            if (entry == nullptr)
            {
                errorMessage = "every [[parts]] entry must be a table";
                return false;
            }
            LoadedPart part;
            part.name                 = readString(entry->get("name"));
            const std::string assetId = readString(entry->get("asset"));
            if (part.name.empty() || assetId.empty())
            {
                errorMessage = "every [[parts]] entry needs 'name' and 'asset'";
                return false;
            }
            const auto found = registry.partIndices.find(assetId);
            if (found == registry.partIndices.end())
            {
                errorMessage = "recipe references unregistered part '" + assetId + "'";
                return false;
            }
            part.asset = &registry.parts[found->second];
            loadedParts.push_back(std::move(part));
        }
    }

    struct LoadedConnection
    {
        std::string parentName;
        std::string parentConnector;
        std::string childName;
        std::string childConnector;
        glm::vec3   offsetTranslation{0.0f};
        glm::quat   offsetRotation{1.0f, 0.0f, 0.0f, 0.0f};
    };
    std::vector<LoadedConnection> loadedConnections;
    if (const toml::node *connectionsNode = document.get("connections"))
    {
        const auto *connections = connectionsNode->as_array();
        if (connections == nullptr)
        {
            errorMessage = "'connections' must be an array of tables";
            return false;
        }
        if (connections->size() > budgets.maximumConnectionsPerCharacter)
        {
            errorMessage = "recipe declares " + std::to_string(connections->size())
                         + " connections but the per-assembly budget allows "
                         + std::to_string(budgets.maximumConnectionsPerCharacter);
            return false;
        }
        loadedConnections.reserve(connections->size());
        for (const toml::node &element : *connections)
        {
            const auto *entry = element.as_table();
            if (entry == nullptr)
            {
                errorMessage = "every [[connections]] entry must be a table";
                return false;
            }
            LoadedConnection connection;
            connection.parentName      = readString(entry->get("parent"));
            connection.parentConnector = readString(entry->get("parent_connector"));
            connection.childName       = readString(entry->get("child"));
            connection.childConnector  = readString(entry->get("child_connector"));
            if (connection.parentName.empty() || connection.parentConnector.empty() || connection.childName.empty()
                || connection.childConnector.empty())
            {
                errorMessage = "every [[connections]] entry needs 'parent', 'parent_connector', 'child', "
                               "and 'child_connector'";
                return false;
            }
            connection.offsetTranslation = readVector3(entry->get("offset_translation"), connection.offsetTranslation);
            connection.offsetRotation    = readQuaternion(entry->get("offset_rotation"), connection.offsetRotation);
            loadedConnections.push_back(std::move(connection));
        }
    }

    std::unordered_map<std::string, float> loadedParameters;
    if (const toml::node *parametersNode = document.get("parameters"))
    {
        const auto *parameters = parametersNode->as_table();
        if (parameters == nullptr)
        {
            errorMessage = "'parameters' must be a table";
            return false;
        }
        for (const auto &entry : *parameters)
        {
            const std::string parameterId = std::string(entry.first);
            const auto        definition  = registry.parameterIndices.find(parameterId);
            if (definition == registry.parameterIndices.end())
            {
                errorMessage = "recipe references unregistered parameter '" + parameterId + "'";
                return false;
            }
            const double value = readNumber(&entry.second, 0.0);
            if (!std::isfinite(value))
            {
                errorMessage = "parameter '" + parameterId + "' has a non-finite value";
                return false;
            }
            const ParameterRecord &definitionRecord = registry.parameters[definition->second];
            loadedParameters.emplace(
                parameterId, std::clamp(static_cast<float>(value), definitionRecord.minimum, definitionRecord.maximum));
        }
    }

    glm::vec3 rootTranslation(0.0f);
    glm::quat rootRotation(1.0f, 0.0f, 0.0f, 0.0f);
    if (const toml::node *rootNode = document.get("root"))
    {
        if (const auto *root = rootNode->as_table())
        {
            rootTranslation = readVector3(root->get("translation"), rootTranslation);
            rootRotation    = readQuaternion(root->get("rotation"), rootRotation);
        }
    }

    // Name uniqueness before anything else is resolved.
    std::set<std::string> instanceNames;
    for (const LoadedPart &part : loadedParts)
    {
        if (!instanceNames.insert(part.name).second)
        {
            errorMessage = "duplicate instance name '" + part.name + "'";
            return false;
        }
    }

    // Resolve connection names against the loaded parts, and the referenced
    // connectors and role pairs against the part assets - the same eager
    // checks connect() applies, so a recipe that connect() would have
    // rejected never reaches the assembly.
    std::unordered_map<std::string, const LoadedPart *> partsByName;
    partsByName.reserve(loadedParts.size() * 2);
    for (const LoadedPart &part : loadedParts)
    {
        partsByName.emplace(part.name, &part);
    }
    struct LoadedConnectionTarget
    {
        const LoadedPart      *parent            = nullptr;
        const LoadedPart      *child             = nullptr;
        const ConnectorRecord *parentConnector   = nullptr;
        const ConnectorRecord *childConnector    = nullptr;
        glm::vec3              offsetTranslation{0.0f};
        glm::quat              offsetRotation{1.0f, 0.0f, 0.0f, 0.0f};
    };
    std::vector<LoadedConnectionTarget> resolvedConnections;
    resolvedConnections.reserve(loadedConnections.size());
    for (const LoadedConnection &connection : loadedConnections)
    {
        LoadedConnectionTarget resolved;
        const auto             parent = partsByName.find(connection.parentName);
        const auto             child  = partsByName.find(connection.childName);
        if (parent == partsByName.end() || child == partsByName.end())
        {
            errorMessage = "connection references unknown instance name";
            return false;
        }
        resolved.parent = parent->second;
        resolved.child  = child->second;
        resolved.parentConnector = findConnector(*resolved.parent->asset, connection.parentConnector);
        resolved.childConnector  = findConnector(*resolved.child->asset, connection.childConnector);
        if (resolved.parentConnector == nullptr)
        {
            errorMessage = "parent part '" + resolved.parent->asset->id + "' has no connector '"
                         + connection.parentConnector + "'";
            return false;
        }
        if (resolved.childConnector == nullptr)
        {
            errorMessage = "child part '" + resolved.child->asset->id + "' has no connector '"
                         + connection.childConnector + "'";
            return false;
        }
        if (resolveCompatibilityRule(registry, *resolved.parentConnector, *resolved.childConnector) == nullptr)
        {
            errorMessage = "no compatibility rule allows role pair ('" + resolved.parentConnector->role + "', '"
                         + resolved.childConnector->role + "')";
            return false;
        }
        resolved.offsetTranslation = connection.offsetTranslation;
        resolved.offsetRotation    = connection.offsetRotation;
        resolvedConnections.push_back(std::move(resolved));
    }

    // Commit: rebuild the assembly content.
    record.parts.clear();
    record.connections.clear();
    record.instancesByName.clear();
    record.instanceIndicesById.clear();
    record.profileId = profileId;

    InstanceId nextInstanceId = 1;
    for (LoadedPart &part : loadedParts)
    {
        PartInstance instance;
        instance.id     = nextInstanceId++;
        instance.name   = part.name;
        instance.partId = part.asset->id;
        instance.part   = *part.asset;
        record.instancesByName.emplace(instance.name, instance.id);
        part.id = instance.id;
        record.parts.push_back(std::move(instance));
    }
    record.nextInstanceId = nextInstanceId;
    for (std::size_t index = 0; index < record.parts.size(); ++index)
    {
        record.instanceIndicesById.emplace(record.parts[index].id, static_cast<std::uint32_t>(index));
    }
    for (const LoadedConnectionTarget &connection : resolvedConnections)
    {
        ConnectionRecord connectionRecord;
        connectionRecord.parent            = connection.parent->id;
        connectionRecord.parentConnector   = connection.parentConnector->id;
        connectionRecord.child             = connection.child->id;
        connectionRecord.childConnector    = connection.childConnector->id;
        connectionRecord.offsetTranslation = connection.offsetTranslation;
        connectionRecord.offsetRotation    = connection.offsetRotation;
        record.connections.push_back(std::move(connectionRecord));
    }

    // Parameters: start from the registry defaults, then apply the recipe.
    record.parameterValues.clear();
    record.parameterValues.reserve(registry.parameters.size());
    for (const ParameterRecord &parameter : registry.parameters)
    {
        record.parameterValues.push_back(parameter.defaultV);
    }
    for (const auto &entry : loadedParameters)
    {
        const auto index = registry.parameterIndices.find(entry.first);
        if (index != registry.parameterIndices.end())
        {
            record.parameterValues[index->second] = entry.second;
        }
    }

    record.rootTranslation = rootTranslation;
    record.rootRotation    = rootRotation;
    record.boundsValid     = false;
    return true;
}

} // namespace internal
} // namespace engine::character
