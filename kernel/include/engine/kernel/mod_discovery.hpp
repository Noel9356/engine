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
