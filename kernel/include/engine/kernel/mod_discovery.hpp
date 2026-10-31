#ifndef ENGINE_KERNEL_MOD_DISCOVERY_HPP
#define ENGINE_KERNEL_MOD_DISCOVERY_HPP

#include <engine/kernel/dynamic_library.hpp>
#include <engine/kernel/mod_api.hpp>
#include <engine/kernel/mod_manifest.hpp>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace engine
{

struct DiscoveredModPackage
{
    ModManifest manifest;

    std::filesystem::path sceneFile;

    std::filesystem::path defaultsFile;
};

std::vector<DiscoveredModPackage> discoverModPackages(
    const std::filesystem::path    &modsDirectory,
    const std::vector<std::string> &disabledIds,
    std::string                    &report
);

// The library file names a native mod may use for its 'entry', in preference
// order. The trust gate uses this to hash the mod's binary content before any
// native code is loaded; the loader uses the same list when it loads.
std::vector<std::filesystem::path> nativeModLibraryCandidates(
    const std::filesystem::path &packageDirectory,
    const std::string           &entry
);

struct NativeModLoad
{
    std::unique_ptr<DynamicLibrary> library;
    ModInterface                   *modInstance = nullptr;
    const EngineModEntryTable      *entryTable  = nullptr;
};

bool loadNativeModEntry(
    const DiscoveredModPackage &package,
    NativeModLoad              &load,
    std::string                &report
);

} // namespace engine

#endif
