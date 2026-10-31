#include <engine/kernel/module_manifest.hpp>

#include <toml.hpp>

#include <algorithm>

namespace engine
{

namespace
{

std::string describeLocation(const std::filesystem::path &manifestPath)
{
    return "module manifest '" + manifestPath.string() + "'";
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
        return true;
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
        return true;
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

bool isValidModuleName(const std::string &name)
{
    if (name.empty())
    {
        return false;
    }
    for (const char character : name)
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

} // namespace

bool ModuleManifestLoader::loadFromFile(
    const std::filesystem::path &manifestPath,
    ModuleManifest              &outManifest,
    std::string                 &errorMessage
) const
{
    errorMessage.clear();
    outManifest = ModuleManifest{};

    toml::table table;
    try
    {
        table = toml::parse_file(manifestPath.string());
    } catch (const toml::parse_error &error)
    {
        errorMessage = describeLocation(manifestPath) + " could not be parsed: " + std::string(error.description())
                     + " (line " + std::to_string(error.source().begin.line) + ", column "
                     + std::to_string(error.source().begin.column) + ")";
        return false;
    } catch (const std::exception &error)
    {
        errorMessage = describeLocation(manifestPath) + " could not be read: " + error.what();
        return false;
    }

    if (!readRequiredString(table, "name", outManifest.name, manifestPath, errorMessage))
    {
        return false;
    }
    if (!isValidModuleName(outManifest.name))
    {
        errorMessage = describeLocation(manifestPath) + " has name '" + outManifest.name
                     + "'; module names may only contain letters, digits and underscores";
        return false;
    }
    if (!readOptionalString(table, "version", outManifest.version, manifestPath, errorMessage)
        || !readOptionalString(table, "library", outManifest.library, manifestPath, errorMessage))
    {
        return false;
    }
    if (!readStringArray(table, "provides", outManifest.providedCapabilities, manifestPath, errorMessage)
        || !readStringArray(table, "requires", outManifest.requiredCapabilities, manifestPath, errorMessage)
        || !readStringArray(table, "optional_capabilities", outManifest.optionalCapabilities, manifestPath, errorMessage)
        || !readStringArray(table, "permissions", outManifest.declaredPermissions, manifestPath, errorMessage))
    {
        return false;
    }

    if (const toml::node *abiValue = table.get("abi"))
    {
        const auto *abiInteger = abiValue->as_integer();
        if (abiInteger == nullptr)
        {
            errorMessage = describeLocation(manifestPath) + " key 'abi' must be an integer";
            return false;
        }
        outManifest.abi = static_cast<std::uint32_t>(abiInteger->get());
    }
    if (outManifest.abi != engineModulePluginAbiVersion)
    {
        errorMessage = describeLocation(manifestPath) + " declares abi " + std::to_string(outManifest.abi)
                     + " but this kernel provides " + std::to_string(engineModulePluginAbiVersion)
                     + "; update the module manifest or the kernel";
        return false;
    }

    if (!outManifest.library.empty()
        && (outManifest.library.find('/') != std::string::npos
            || outManifest.library.find('\\') != std::string::npos))
    {
        errorMessage = describeLocation(manifestPath) + " library '" + outManifest.library
                     + "' must be a file name without path separators";
        return false;
    }
    return true;
}

} // namespace engine
