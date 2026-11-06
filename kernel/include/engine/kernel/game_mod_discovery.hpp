// Game-mod package discovery and native mod loading.
//
// Scans mods directories for packages (mods/<id>/mod.toml) and, when
// asked, loads a native mod's .so and creates its instance through the
// entry table.

#ifndef ENGINE_KERNEL_GAME_MOD_DISCOVERY_HPP
#define ENGINE_KERNEL_GAME_MOD_DISCOVERY_HPP

#include <engine/kernel/dynamic_library.hpp>
#include <engine/kernel/game_mod_api.hpp>
#include <engine/kernel/game_mod_manifest.hpp>
#include <engine/kernel/participant_interface.hpp>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace engine
{

// A validated package plus the optional scene/defaults files found
// inside it.
struct DiscoveredGameModPackage
{
    GameModManifest manifest;

    // Empty when mod.toml declares no scene.
    std::filesystem::path sceneFile;

    // Package config.toml merged as the ModDefaults layer (set only when
    // the file exists).
    std::filesystem::path defaultsFile;
};

// Deterministic order (sorted directories); disabled ids and duplicate
// ids are skipped with a report line.
std::vector<DiscoveredGameModPackage> discoverModPackages(
    const std::filesystem::path &modsDirectory, const std::vector<std::string> &disabledIds, std::string &report);

// Platform library names for an entry base name ("libX.so"/"X.so",
// "X.dll"/"libX.dll"), in preference order.
std::vector<std::filesystem::path> nativeModLibraryCandidates(
    const std::filesystem::path &packageDirectory, const std::string &entry);

// Out-parameter bundle of loadNativeModEntry; the caller owns the library
// and must destroyMod(gameModInstance) through entryTable when done.
struct NativeGameModLoad
{
    std::unique_ptr<DynamicLibrary> library;
    GameModInterface               *gameModInstance = nullptr;
    const EngineGameModEntryTable  *entryTable      = nullptr;
};

// Loads the package's library, checks the interface version and the
// compiled manifest name; on failure report explains and load stays
// cleared.
bool loadNativeModEntry(const DiscoveredGameModPackage &package, NativeGameModLoad &load, std::string &report);

} // namespace engine

#endif
