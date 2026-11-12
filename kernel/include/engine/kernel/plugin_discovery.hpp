// Plugin package discovery and native plugin loading.
//
// Scans plugins directories for packages (plugins/<id>/plugin.toml) and, when
// asked, loads a native plugin's .so and creates its instance through the
// entry table.

#ifndef ENGINE_KERNEL_PLUGIN_DISCOVERY_HPP
#define ENGINE_KERNEL_PLUGIN_DISCOVERY_HPP

#include <engine/kernel/dynamic_library.hpp>
#include <engine/kernel/plugin_api.hpp>
#include <engine/kernel/plugin_manifest.hpp>
#include <engine/kernel/participant_interface.hpp>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace engine
{

// A validated package plus the optional scene/defaults files found
// inside it.
struct DiscoveredPluginPackage
{
    PluginManifest manifest;

    // Empty when plugin.toml declares no scene.
    std::filesystem::path sceneFile;

    // Package config.toml merged as the PluginDefaults layer (set only when
    // the file exists).
    std::filesystem::path defaultsFile;
};

// Deterministic order (sorted directories); disabled ids and duplicate
// ids are skipped with a report line.
std::vector<DiscoveredPluginPackage> discoverPluginPackages(
    const std::filesystem::path &pluginsDirectory, const std::vector<std::string> &disabledIds, std::string &report);

// Platform library names for an entry base name ("libX.so"/"X.so",
// "X.dll"/"libX.dll"), in preference order.
std::vector<std::filesystem::path> nativePluginLibraryCandidates(
    const std::filesystem::path &packageDirectory, const std::string &entry);

// Out-parameter bundle of loadNativePluginEntry; the caller owns the library
// and must destroyPlugin(pluginInstance) through entryTable when done.
struct NativePluginLoad
{
    std::unique_ptr<DynamicLibrary> library;
    PluginInterface               *pluginInstance = nullptr;
    const EnginePluginEntryTable  *entryTable      = nullptr;
};

// Loads the package's library, checks the interface version and the
// compiled manifest name; on failure report explains and load stays
// cleared.
bool loadNativePluginEntry(const DiscoveredPluginPackage &package, NativePluginLoad &load, std::string &report);

} // namespace engine

#endif
