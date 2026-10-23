#ifndef ENGINE_KERNEL_MOD_DISCOVERY_HPP
#define ENGINE_KERNEL_MOD_DISCOVERY_HPP

// Mod package discovery: what is on disk, decoupled from what the kernel
// does with it. The kernel's gather phase consumes these free functions and
// keeps every policy decision (component records, scene claims, defaults
// merging, lifecycle) in engine_kernel.cpp.

#include "engine/kernel/dynamic_library.hpp"
#include "engine/kernel/mod_api.hpp"
#include "engine/kernel/mod_manifest.hpp"

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace engine
{

// One mod package found on disk, before the kernel turns it into a component
// record: the parsed manifest plus the package-relative files the kernel
// consumes later (the world-content scene file, the shipped defaults).
struct DiscoveredModPackage
{
    ModManifest manifest;
    // The manifest's declared scene file resolved against the package
    // directory; empty when the manifest declares none.
    std::filesystem::path sceneFile;
    // The package's shipped defaults file (config.toml); empty when absent.
    std::filesystem::path defaultsFile;
};

// Scans one mods directory: every subdirectory that contains a mod.toml is a
// mod package. A directory without a manifest is not a mod and is skipped
// silently; a directory with a broken or conflicting manifest is a precise
// startup problem (one report line, the package dropped). Discovery is sorted
// by path so behavior never depends on filesystem iteration order. Packages
// whose id is in disabledIds are skipped on purpose and not reported; a
// duplicate id is reported and the later package dropped. A missing or
// non-directory path is a valid state (no packages). Returns the active
// packages in discovery order.
std::vector<DiscoveredModPackage> discoverModPackages(
    const std::filesystem::path &modsDirectory, const std::vector<std::string> &disabledIds, std::string &report
);

// One native package's loaded state. The library handle must outlive the mod
// instance (the owner destroys in reverse declaration order); the instance is
// destroyed through the entry table's destroyMod, never delete.
struct NativeModLoad
{
    std::unique_ptr<DynamicLibrary> library;
    ModInterface                   *modInstance = nullptr;
    const EngineModEntryTable      *entryTable  = nullptr;
};

// Loads one native package's shared library, resolves its entry table,
// creates the mod instance and checks the compiled manifest agrees with the
// package manifest on identity. Returns false with one report line when the
// package cannot be used; the package is then dropped instead of failing the
// whole startup, so the report stays one line per problem.
bool loadNativeModEntry(const DiscoveredModPackage &package, NativeModLoad &load, std::string &report);

} // namespace engine

#endif // ENGINE_KERNEL_MOD_DISCOVERY_HPP
