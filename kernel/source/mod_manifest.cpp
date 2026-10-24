#include "engine/kernel/mod_manifest.hpp"

#include <toml.hpp>

#include <algorithm>

namespace engine
{

namespace
{

std::string describeLocation(
    const std::filesystem::path &manifestPath
)
{
    return "mod manifest '" + manifestPath.string() + "'";
}

bool readRequiredString(
    const toml::table           &table,
    const std::string           &key,
    std::string                 &out,
    const std::filesystem::path &manifestPath,
    std::string                 &errorMessage
)
{
    const toml::node *value = table.get(key);
    if (value == nullptr)
    {
        errorMessage = describeLocation(manifestPath) + " is missing the required key '" + key + "'";
        return false;
    }
    const auto *stringValue = value->as_string();
    if (stringValue == nullptr)
    {
        errorMessage = describeLocation(manifestPath) + " key '" + key + "' must be a string";
        return false;
    }
    out = stringValue->get();
    return true;
}

bool readOptionalString(
    const toml::table           &table,
    const std::string           &key,
    std::string                 &out,
    const std::filesystem::path &manifestPath,
    std::string                 &errorMessage
)
{
    const toml::node *value = table.get(key);
    if (value == nullptr)
    {
        return true; // absent keys keep their defaults
    }
    const auto *stringValue = value->as_string();
    if (stringValue == nullptr)
    {
        errorMessage = describeLocation(manifestPath) + " key '" + key + "' must be a string";
        return false;
    }
    out = stringValue->get();
    return true;
}

bool readStringArray(
    const toml::table           &table,
    const std::string           &key,
    std::vector<std::string>    &out,
    const std::filesystem::path &manifestPath,
    std::string                 &errorMessage
)
{
    const toml::node *value = table.get(key);
    if (value == nullptr)
    {
        return true; // absent arrays stay empty
    }
    const auto *arrayValue = value->as_array();
    if (arrayValue == nullptr)
    {
        errorMessage = describeLocation(manifestPath) + " key '" + key + "' must be an array of strings";
        return false;
    }
    for (const toml::node &element : *arrayValue)
    {
        const auto *elementString = element.as_string();
        if (elementString == nullptr)
        {
            errorMessage = describeLocation(manifestPath) + " key '" + key + "' must contain only strings";
            return false;
        }
        out.push_back(elementString->get());
    }
    return true;
}

bool isValidModId(
    const std::string &id
)
{
    if (id.empty())
    {
        return false;
    }
    for (const char character : id)
    {
        const bool allowed = (character >= 'a' && character <= 'z') || (character >= 'A' && character <= 'Z')
                          || (character >= '0' && character <= '9') || character == '_';
        if (!allowed)
        {
            return false;
        }
    }
    return true;
}

// config_keys entries are "<section>.<dotted path>" of TOML bare key segments;
// the same rule the kernel's known-key registry enforces. Validated here so a
// malformed entry names the manifest and the entry instead of a kernel report.
bool isValidConfigKey(
    const std::string &configKey
)
{
    const std::size_t separator = configKey.find('.');
    if (separator == std::string::npos || separator == 0 || separator + 1 >= configKey.size()
        || configKey.find("..") != std::string::npos)
    {
        return false;
    }
    std::size_t position = 0;
    while (position < configKey.size())
    {
        const std::size_t dot     = configKey.find('.', position);
        const std::string segment = configKey.substr(
            position, dot == std::string::npos ? std::string::npos : dot - position
        );
        position = (dot == std::string::npos) ? configKey.size() : dot + 1;
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
    }
    return true;
}

} // namespace

bool ModManifestLoader::loadFromFile(
    const std::filesystem::path &manifestPath, ModManifest &outManifest, std::string &errorMessage
) const
{
    errorMessage.clear();
    outManifest = ModManifest{};

    toml::table table;
    try
    {
        table = toml::parse_file(manifestPath.string());
    }
    catch (const toml::parse_error &error)
    {
        errorMessage = describeLocation(manifestPath) + " could not be parsed: " + std::string(error.description())
                     + " (line " + std::to_string(error.source().begin.line) + ", column "
                     + std::to_string(error.source().begin.column) + ")";
        return false;
    }
    catch (const std::exception &error)
    {
        errorMessage = describeLocation(manifestPath) + " could not be read: " + error.what();
        return false;
    }

    if (!readRequiredString(table, "id", outManifest.id, manifestPath, errorMessage))
    {
        return false;
    }
    if (!isValidModId(outManifest.id))
    {
        errorMessage = describeLocation(manifestPath) + " has id '" + outManifest.id
                     + "'; ids may only contain letters, digits and underscores";
        return false;
    }
    if (!readOptionalString(table, "version", outManifest.version, manifestPath, errorMessage)
        || !readRequiredString(table, "type", outManifest.type, manifestPath, errorMessage)
        || !readRequiredString(table, "entry", outManifest.entry, manifestPath, errorMessage)
        || !readOptionalString(table, "display_name", outManifest.displayName, manifestPath, errorMessage)
        || !readOptionalString(table, "description", outManifest.description, manifestPath, errorMessage)
        || !readOptionalString(table, "author", outManifest.author, manifestPath, errorMessage)
        || !readOptionalString(table, "scene", outManifest.scene, manifestPath, errorMessage))
    {
        return false;
    }
    if (table.get("host") != nullptr)
    {
        // Removed field: a mod never names a module. The host is declared as a
        // capability requirement, so the dependency graph picks the provider.
        errorMessage = describeLocation(manifestPath) + " declares the removed key 'host'; a mod never names a "
                     + "module. Declare the capability the host must provide as 'host_capability' (and list it "
                       "under 'capabilities')";
        return false;
    }
    if (!readStringArray(table, "capabilities", outManifest.capabilities, manifestPath, errorMessage)
        || !readStringArray(table, "optional_capabilities", outManifest.optionalCapabilities, manifestPath, errorMessage)
        || !readStringArray(table, "permissions", outManifest.permissions, manifestPath, errorMessage)
        || !readStringArray(table, "requires", outManifest.requiresComponents, manifestPath, errorMessage)
        || !readStringArray(table, "config_keys", outManifest.configKeys, manifestPath, errorMessage))
    {
        return false;
    }
    if (outManifest.isScript())
    {
        // The host is a capability requirement, never a module name: the mod
        // declares which ONE of its required capabilities the host module must
        // provide, and the kernel resolves that provider like any other
        // requirement. Native mods never name a host.
        if (!readOptionalString(table, "host_capability", outManifest.hostCapability, manifestPath, errorMessage))
        {
            return false;
        }
        if (outManifest.hostCapability.empty())
        {
            errorMessage = describeLocation(manifestPath) + " is a script mod but declares no 'host_capability'; "
                         + "name the required capability whose provider hosts the script runtime";
            return false;
        }
        if (std::find(outManifest.capabilities.begin(), outManifest.capabilities.end(), outManifest.hostCapability)
            == outManifest.capabilities.end())
        {
            errorMessage = describeLocation(manifestPath) + " declares host_capability '" + outManifest.hostCapability
                         + "' which is not in its 'capabilities' list; the host "
                           "capability must also be a required capability so the provider is ordered first";
            return false;
        }
    }
    for (const std::string &configKey : outManifest.configKeys)
    {
        if (!isValidConfigKey(configKey))
        {
            errorMessage = describeLocation(manifestPath) + " config_keys entry '" + configKey
                         + "' must be a '<section>.<dotted path>' of bare key segments "
                           "(example: 'example_mod.level')";
            return false;
        }
    }

    if (const toml::node *apiValue = table.get("engine_api"))
    {
        const auto *apiInteger = apiValue->as_integer();
        if (apiInteger == nullptr)
        {
            errorMessage = describeLocation(manifestPath) + " key 'engine_api' must be an integer";
            return false;
        }
        outManifest.engineApi = static_cast<std::uint32_t>(apiInteger->get());
    }

    if (!outManifest.isNative() && !outManifest.isScript())
    {
        errorMessage = describeLocation(manifestPath) + " has type '" + outManifest.type
                     + "'; expected 'native' or 'script'";
        return false;
    }
    if (outManifest.engineApi != engineModInterfaceVersion)
    {
        errorMessage = describeLocation(manifestPath) + " declares engine_api " + std::to_string(outManifest.engineApi)
                     + " but this engine provides " + std::to_string(engineModInterfaceVersion)
                     + "; update the mod or the engine";
        return false;
    }
    if (outManifest.entry.empty())
    {
        errorMessage = describeLocation(manifestPath) + " has an empty 'entry'";
        return false;
    }
    if (outManifest.isNative() && outManifest.entry.find('/') != std::string::npos)
    {
        errorMessage = describeLocation(manifestPath) + " native entry '" + outManifest.entry
                     + "' must be a library base name without path separators (the loader applies the "
                       "platform prefix and suffix)";
        return false;
    }
    if (!outManifest.scene.empty())
    {
        // The scene file is resolved against the package directory; a
        // declaration that escapes the package (absolute path or a '..'
        // climb) would let one package claim content outside itself.
        const std::filesystem::path declaredScenePath(outManifest.scene);
        if (declaredScenePath.is_absolute() || outManifest.scene.find("..") != std::string::npos)
        {
            errorMessage = describeLocation(manifestPath) + " scene '" + outManifest.scene
                         + "' must be a file path inside the mod package (relative, no '..')";
            return false;
        }
    }

    if (outManifest.displayName.empty())
    {
        outManifest.displayName = outManifest.id;
    }
    outManifest.directory = manifestPath.parent_path();
    return true;
}

} // namespace engine
