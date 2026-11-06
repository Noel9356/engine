// Shared TOML field readers for the manifest loaders (module.toml, mod.toml).
//
// Each reader type-checks one key and produces a precise error message
// naming the manifest and the key on failure.

#ifndef ENGINE_KERNEL_MANIFEST_TOML_READERS_HPP
#define ENGINE_KERNEL_MANIFEST_TOML_READERS_HPP

#include <toml.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace engine
{

namespace manifest_readers
{

inline std::string describeLocation(const char *manifestLabel, const std::filesystem::path &manifestPath)
{
    return std::string(manifestLabel) + " '" + manifestPath.string() + "'";
}

// Required key: missing or wrong type fails.
inline bool readRequiredString(const toml::table &table, const std::string &key, std::string &out,
    const char *manifestLabel, const std::filesystem::path &manifestPath, std::string &errorMessage)
{
    const toml::node *value = table.get(key);
    if (value == nullptr)
    {
        errorMessage = describeLocation(manifestLabel, manifestPath) + " is missing the required key '" + key + "'";
        return false;
    }
    const auto *stringValue = value->as_string();
    if (stringValue == nullptr)
    {
        errorMessage = describeLocation(manifestLabel, manifestPath) + " key '" + key + "' must be a string";
        return false;
    }
    out = stringValue->get();
    return true;
}

// Optional key: absent is fine, wrong type fails.
inline bool readOptionalString(const toml::table &table, const std::string &key, std::string &out,
    const char *manifestLabel, const std::filesystem::path &manifestPath, std::string &errorMessage)
{
    const toml::node *value = table.get(key);
    if (value == nullptr)
    {
        return true;
    }
    const auto *stringValue = value->as_string();
    if (stringValue == nullptr)
    {
        errorMessage = describeLocation(manifestLabel, manifestPath) + " key '" + key + "' must be a string";
        return false;
    }
    out = stringValue->get();
    return true;
}

// Optional array-of-strings key; wrong type or element type fails.
inline bool readStringArray(const toml::table &table, const std::string &key, std::vector<std::string> &out,
    const char *manifestLabel, const std::filesystem::path &manifestPath, std::string &errorMessage)
{
    const toml::node *value = table.get(key);
    if (value == nullptr)
    {
        return true;
    }
    const auto *arrayValue = value->as_array();
    if (arrayValue == nullptr)
    {
        errorMessage = describeLocation(manifestLabel, manifestPath) + " key '" + key + "' must be an array of strings";
        return false;
    }
    for (const toml::node &element : *arrayValue)
    {
        const auto *elementString = element.as_string();
        if (elementString == nullptr)
        {
            errorMessage
                = describeLocation(manifestLabel, manifestPath) + " key '" + key + "' must contain only strings";
            return false;
        }
        out.push_back(elementString->get());
    }
    return true;
}

} // namespace manifest_readers

} // namespace engine

#endif
