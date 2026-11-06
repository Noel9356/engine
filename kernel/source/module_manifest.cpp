// ModuleManifestLoader: module.toml sidecar parsing and validation.
//
// A sidecar gives a module library a data descriptor (name, version,
// capabilities, permissions) without executing its code; abi must match
// the kernel's module ABI.

#include <engine/kernel/module_manifest.hpp>

#include "manifest_toml_readers.hpp"

#include <toml.hpp>

#include <cstdint>
#include <engine/kernel/module_api.hpp>
#include <exception>
#include <filesystem>
#include <string>

namespace engine
{
namespace
{

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

// Parses module.toml into a ModuleManifest; "library" may be empty (the sidecar
// then describes the directory's single library).
bool ModuleManifestLoader::loadFromFile(
    const std::filesystem::path &manifestPath, ModuleManifest &outManifest, std::string &errorMessage) const
{
    using manifest_readers::describeLocation;
    using manifest_readers::readOptionalString;
    using manifest_readers::readRequiredString;
    using manifest_readers::readStringArray;

    constexpr const char *manifestLabel = "module manifest";

    errorMessage.clear();
    outManifest = ModuleManifest{};

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

    if (!readRequiredString(table, "name", outManifest.descriptor.name, manifestLabel, manifestPath, errorMessage))
    {
        return false;
    }
    if (!isValidModuleName(outManifest.descriptor.name))
    {
        errorMessage = describeLocation(manifestLabel, manifestPath) + " has name '" + outManifest.descriptor.name
                     + "'; module names may only contain letters, digits and underscores";
        return false;
    }
    if (!readOptionalString(table, "version", outManifest.descriptor.version, manifestLabel, manifestPath, errorMessage)
        || !readOptionalString(table, "library", outManifest.library, manifestLabel, manifestPath, errorMessage))
    {
        return false;
    }
    if (!readStringArray(
            table, "provides", outManifest.descriptor.providedCapabilities, manifestLabel, manifestPath, errorMessage)
        || !readStringArray(
            table, "requires", outManifest.descriptor.requiredCapabilities, manifestLabel, manifestPath, errorMessage)
        || !readStringArray(table,
            "optional_capabilities",
            outManifest.descriptor.optionalCapabilities,
            manifestLabel,
            manifestPath,
            errorMessage)
        || !readStringArray(table,
            "permissions",
            outManifest.descriptor.declaredPermissions,
            manifestLabel,
            manifestPath,
            errorMessage))
    {
        return false;
    }

    if (const toml::node *abiValue = table.get("abi"))
    {
        const auto *abiInteger = abiValue->as_integer();
        if (abiInteger == nullptr)
        {
            errorMessage = describeLocation(manifestLabel, manifestPath) + " key 'abi' must be an integer";
            return false;
        }
        outManifest.abi = static_cast<std::uint32_t>(abiInteger->get());
    }
    if (outManifest.abi != engineModuleAbiVersion)
    {
        errorMessage = describeLocation(manifestLabel, manifestPath) + " declares abi "
                     + std::to_string(outManifest.abi) + " but this kernel provides "
                     + std::to_string(engineModuleAbiVersion) + "; update the module manifest or the kernel";
        return false;
    }

    if (!outManifest.library.empty()
        && (outManifest.library.find('/') != std::string::npos || outManifest.library.find('\\') != std::string::npos))
    {
        errorMessage = describeLocation(manifestLabel, manifestPath) + " library '" + outManifest.library
                     + "' must be a file name without path separators";
        return false;
    }
    return true;
}

} // namespace engine
