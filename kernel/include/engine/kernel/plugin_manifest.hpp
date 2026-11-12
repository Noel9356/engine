// plugin.toml: the manifest of one plugin package (plugins/<id>/plugin.toml).
//
// A package is either type = "native" (entry names a .so loaded through
// the C ABI in plugin_api.hpp) or type = "script" (entry names a
// script file hosted by the module providing host_capability).

#ifndef ENGINE_KERNEL_PLUGIN_MANIFEST_HPP
#define ENGINE_KERNEL_PLUGIN_MANIFEST_HPP

#include <engine/kernel/plugin_api.hpp>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace engine
{

// Parsed plugin.toml. type is "native" (a .so) or "script" (e.g. Lua);
// entry is the library base name or the script file.
struct PluginManifest
{
    std::string id;
    std::string version = "0.1.0";
    // Defaults to the engine's current interface version; checked on load.
    std::uint32_t engineApi = enginePluginInterfaceVersion;
    std::string   type;
    std::string   entry;
    std::string   displayName;
    std::string   description;
    std::string   author;
    // Capabilities the plugin requires at runtime.
    std::vector<std::string> capabilities;

    // Native plugins only: what the plugin publishes; must match its compiled
    // manifest (checked when the .so loads). Script plugins cannot provide.
    std::vector<std::string> providedCapabilities;

    std::vector<std::string> optionalCapabilities;

    // Script plugins only: the capability whose provider module hosts the
    // script (must also appear under capabilities).
    std::string              hostCapability;
    std::vector<std::string> permissions;
    std::vector<std::string> requiredParticipants;

    // "<section>.<dotted path>" keys the plugin reads; registered for the
    // unknown-key warning pass.
    std::vector<std::string> configKeys;

    // Optional scene file inside the package; applied when the assembly
    // defines no [scene] of its own.
    std::string scene;

    // Filled by the loader: the package root (plugin.toml's parent).
    std::filesystem::path directory;

    bool isNative() const
    {
        return type == "native";
    }

    bool isScript() const
    {
        return type == "script";
    }
};

// Validates ids (up to 128 ASCII characters), types, script-host rules and path safety; fills
// directory.
class PluginManifestLoader final
{
public:
    static constexpr const char *manifestFileName = "plugin.toml";

    // Parses and validates one manifest; outManifest is reset first and, on
    // success, also carries directory (manifestPath's parent). Returns false
    // with a human-readable errorMessage on any problem.
    bool loadFromFile(
        const std::filesystem::path &manifestPath, PluginManifest &outManifest, std::string &errorMessage) const;
};

} // namespace engine

#endif
