// GameModManifestLoader: mod.toml parsing and validation.
//
// One manifest per mod package: identity, type (native|script), entry,
// capabilities/permissions and the script host capability. Everything is
// validated here so discovery can trust the result.

#include <engine/kernel/game_mod_manifest.hpp>

#include "manifest_toml_readers.hpp"

#include <toml.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <engine/kernel/game_mod_api.hpp>
#include <exception>
#include <filesystem>
#include <limits>
#include <string>

namespace engine
{

namespace
{


bool isPackageRelativeFilePath(const std::string &value, const std::filesystem::path &packageDirectory)
{
    if (value.empty() || value.find('\0') != std::string::npos)
        return false;

    const std::filesystem::path relativePath(value);
    if (relativePath.is_absolute() || relativePath.has_root_name() || relativePath.has_root_directory())
        return false;

    std::error_code error;
    const std::filesystem::path packageRoot = std::filesystem::weakly_canonical(packageDirectory, error);
    if (error || packageRoot.empty())
        return false;

    const std::filesystem::path resolved = std::filesystem::weakly_canonical(packageRoot / relativePath, error);
    if (error || resolved.empty())
        return false;

    const std::filesystem::path containedPath = resolved.lexically_relative(packageRoot);
    if (containedPath.empty() || containedPath.is_absolute() || containedPath == ".")
        return false;
    return *containedPath.begin() != "..";
}

bool isValidModId(const std::string &id)
{
    if (id.empty() || id.size() > 128u)
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

// "<section>.<dotted path>" of bare key segments (no empty or doubled dots).
bool isValidConfigKey(const std::string &configKey)
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
        const std::size_t dot = configKey.find('.', position);
        const std::string segment
            = configKey.substr(position, dot == std::string::npos ? std::string::npos : dot - position);
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

// Parses and fully validates mod.toml; on success fills the manifest and records
// the package directory.
bool GameModManifestLoader::loadFromFile(
    const std::filesystem::path &manifestPath, GameModManifest &outManifest, std::string &errorMessage) const
{
    using manifest_readers::describeLocation;
    using manifest_readers::readOptionalString;
    using manifest_readers::readRequiredString;
    using manifest_readers::readStringArray;

    constexpr const char *manifestLabel = "mod manifest";

    errorMessage.clear();
    outManifest = GameModManifest{};

    constexpr std::uintmax_t maximumManifestBytes = 1024ull * 1024ull;
    std::error_code filesystemError;
    const std::uintmax_t manifestSize = std::filesystem::file_size(manifestPath, filesystemError);
    if (filesystemError || manifestSize > maximumManifestBytes)
    {
        errorMessage = describeLocation(manifestLabel, manifestPath)
                     + (filesystemError ? " could not be read: " + filesystemError.message()
                                        : " exceeds the 1 MiB manifest size limit");
        return false;
    }

    toml::table table;
    try
    {
        table = toml::parse_file(manifestPath.string());
    }
    catch (const toml::parse_error &error)
    {
        errorMessage = describeLocation(manifestLabel, manifestPath) + " could not be parsed: "
                     + std::string(error.description()) + " (line " + std::to_string(error.source().begin.line)
                     + ", column " + std::to_string(error.source().begin.column) + ")";
        return false;
    }
    catch (const std::exception &error)
    {
        errorMessage = describeLocation(manifestLabel, manifestPath) + " could not be read: " + error.what();
        return false;
    }

    if (!readRequiredString(table, "id", outManifest.id, manifestLabel, manifestPath, errorMessage))
    {
        return false;
    }
    if (!isValidModId(outManifest.id))
    {
        errorMessage = describeLocation(manifestLabel, manifestPath) + " has id '" + outManifest.id
                     + "'; ids may only contain letters, digits and underscores";
        return false;
    }
    if (!readOptionalString(table, "version", outManifest.version, manifestLabel, manifestPath, errorMessage)
        || !readRequiredString(table, "type", outManifest.type, manifestLabel, manifestPath, errorMessage)
        || !readRequiredString(table, "entry", outManifest.entry, manifestLabel, manifestPath, errorMessage)
        || !readOptionalString(
            table, "display_name", outManifest.displayName, manifestLabel, manifestPath, errorMessage)
        || !readOptionalString(table, "description", outManifest.description, manifestLabel, manifestPath, errorMessage)
        || !readOptionalString(table, "author", outManifest.author, manifestLabel, manifestPath, errorMessage)
        || !readOptionalString(table, "scene", outManifest.scene, manifestLabel, manifestPath, errorMessage))
    {
        return false;
    }
    if (table.get("host") != nullptr)
    {
        errorMessage = describeLocation(manifestLabel, manifestPath)
                     + " declares the removed key 'host'; a mod never names a "
                     + "module. Declare the capability the host must provide as 'host_capability' (and list it "
                       "under 'capabilities')";
        return false;
    }
    if (!readStringArray(table, "capabilities", outManifest.capabilities, manifestLabel, manifestPath, errorMessage)
        || !readStringArray(
            table, "provides", outManifest.providedCapabilities, manifestLabel, manifestPath, errorMessage)
        || !readStringArray(
            table, "optional_capabilities", outManifest.optionalCapabilities, manifestLabel, manifestPath, errorMessage)
        || !readStringArray(table, "permissions", outManifest.permissions, manifestLabel, manifestPath, errorMessage)
        || !readStringArray(
            table, "requires", outManifest.requiredParticipants, manifestLabel, manifestPath, errorMessage)
        || !readStringArray(table, "config_keys", outManifest.configKeys, manifestLabel, manifestPath, errorMessage))
    {
        return false;
    }
    if (!outManifest.providedCapabilities.empty() && outManifest.isScript())
    {
        errorMessage = describeLocation(manifestLabel, manifestPath)
                     + " is a script mod and declares 'provides'; script mods "
                     + "cannot provide capabilities (a native mod publishes its provided capabilities through "
                       "the compiled manifest, declared here so the graph sees them)";
        return false;
    }
    if (outManifest.isScript())
    {
        if (!readOptionalString(
                table, "host_capability", outManifest.hostCapability, manifestLabel, manifestPath, errorMessage))
        {
            return false;
        }
        if (outManifest.hostCapability.empty())
        {
            errorMessage = describeLocation(manifestLabel, manifestPath)
                         + " is a script mod but declares no 'host_capability'; "
                         + "name the required capability whose provider hosts the script runtime";
            return false;
        }
        if (std::find(outManifest.capabilities.begin(), outManifest.capabilities.end(), outManifest.hostCapability)
            == outManifest.capabilities.end())
        {
            errorMessage = describeLocation(manifestLabel, manifestPath) + " declares host_capability '"
                         + outManifest.hostCapability
                         + "' which is not in its 'capabilities' list; the host "
                           "capability must also be a required capability so the provider is ordered first";
            return false;
        }
    }
    for (const std::string &configKey : outManifest.configKeys)
    {
        if (!isValidConfigKey(configKey))
        {
            errorMessage = describeLocation(manifestLabel, manifestPath) + " config_keys entry '" + configKey
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
            errorMessage = describeLocation(manifestLabel, manifestPath) + " key 'engine_api' must be an integer";
            return false;
        }
        const std::int64_t engineApi = apiInteger->get();
        if (engineApi < 0 || static_cast<std::uint64_t>(engineApi) > std::numeric_limits<std::uint32_t>::max())
        {
            errorMessage = describeLocation(manifestLabel, manifestPath)
                         + " key 'engine_api' must be an unsigned 32-bit integer";
            return false;
        }
        outManifest.engineApi = static_cast<std::uint32_t>(engineApi);
    }

    if (!outManifest.isNative() && !outManifest.isScript())
    {
        errorMessage = describeLocation(manifestLabel, manifestPath) + " has type '" + outManifest.type
                     + "'; expected 'native' or 'script'";
        return false;
    }
    if (outManifest.engineApi != engineGameModInterfaceVersion)
    {
        errorMessage = describeLocation(manifestLabel, manifestPath) + " declares engine_api "
                     + std::to_string(outManifest.engineApi) + " but this engine provides "
                     + std::to_string(engineGameModInterfaceVersion) + "; update the mod or the engine";
        return false;
    }
    if (outManifest.entry.empty())
    {
        errorMessage = describeLocation(manifestLabel, manifestPath) + " has an empty 'entry'";
        return false;
    }

    if (outManifest.isNative()
        && (outManifest.entry.find('/') != std::string::npos || outManifest.entry.find('\\') != std::string::npos))
    {
        errorMessage = describeLocation(manifestLabel, manifestPath) + " native entry '" + outManifest.entry
                     + "' must be a library base name without path separators (the loader applies the "
                       "platform prefix and suffix)";
        return false;
    }
    const std::filesystem::path packageDirectory = manifestPath.parent_path();
    if (outManifest.isScript())
    {
        const std::filesystem::path scriptPath(outManifest.entry);
        if (scriptPath.extension() != ".lua"
            || !isPackageRelativeFilePath(outManifest.entry, packageDirectory))
        {
            errorMessage = describeLocation(manifestLabel, manifestPath) + " script entry '" + outManifest.entry
                         + "' must name a Lua file inside the mod package";
            return false;
        }
    }
    if (!outManifest.scene.empty() && !isPackageRelativeFilePath(outManifest.scene, packageDirectory))
    {
        errorMessage = describeLocation(manifestLabel, manifestPath) + " scene '" + outManifest.scene
                     + "' must name a file inside the mod package";
        return false;
    }

    if (outManifest.displayName.empty())
    {
        outManifest.displayName = outManifest.id;
    }
    outManifest.directory = packageDirectory;
    return true;
}

} // namespace engine
