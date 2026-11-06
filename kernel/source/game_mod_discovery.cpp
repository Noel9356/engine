// Game-mod package discovery: mods/*/mod.toml scanning and native loading.
//
// discoverModPackages collects candidate packages (with scene and defaults
// sidecars); loadNativeModEntry dlopens a native mod's library, checks its
// ABI and returns the entry table plus instance for the kernel.

#include "engine/kernel/dynamic_library.hpp"

#include <algorithm>
#include <engine/kernel/configuration.hpp>
#include <engine/kernel/game_mod_api.hpp>
#include <engine/kernel/game_mod_discovery.hpp>
#include <engine/kernel/game_mod_manifest.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <filesystem>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace engine
{

// Every subdirectory with a mod.toml is a package; ids must be unique and not
// disabled. Directory order is sorted for determinism.
std::vector<DiscoveredGameModPackage> discoverModPackages(
    const std::filesystem::path &modsDirectory, const std::vector<std::string> &disabledIds, std::string &report)
{
    std::vector<DiscoveredGameModPackage> packages;
    if (!std::filesystem::exists(modsDirectory) || !std::filesystem::is_directory(modsDirectory))
    {
        return packages;
    }

    std::vector<std::filesystem::path> packageDirectories;
    for (const std::filesystem::directory_entry &entry : std::filesystem::directory_iterator(modsDirectory))
    {
        if (!entry.is_directory())
        {
            continue;
        }
        if (!std::filesystem::exists(entry.path() / GameModManifestLoader::manifestFileName))
        {
            report += "- directory '" + entry.path().string() + "' inside the mods directory has no "
                    + std::string(GameModManifestLoader::manifestFileName)
                    + "; every subdirectory of a mods directory is a mod package, so this one is skipped."
                      " Add the manifest or move the directory out of the mods tree.\n";
            continue;
        }
        packageDirectories.push_back(entry.path());
    }
    std::sort(packageDirectories.begin(), packageDirectories.end());

    GameModManifestLoader manifestLoader;
    std::set<std::string> seenIds;
    for (const std::filesystem::path &packageDirectory : packageDirectories)
    {
        DiscoveredGameModPackage package;
        std::string              manifestError;
        if (!manifestLoader.loadFromFile(
                packageDirectory / GameModManifestLoader::manifestFileName, package.manifest, manifestError))
        {
            report += "- " + manifestError + "\n";
            continue;
        }
        if (std::find(disabledIds.begin(), disabledIds.end(), package.manifest.id) != disabledIds.end())
        {
            continue;
        }
        if (!seenIds.insert(package.manifest.id).second)
        {
            report += "- duplicate mod id '" + package.manifest.id + "' in '" + packageDirectory.string()
                    + "'; mod ids must be unique across the mods directory.\n";
            continue;
        }

        if (!package.manifest.scene.empty())
        {
            package.sceneFile = packageDirectory / package.manifest.scene;
        }
        const std::filesystem::path defaultsPath = packageDirectory / ConfigurationStore::modDefaultsFileName;
        if (std::filesystem::exists(defaultsPath))
        {
            package.defaultsFile = defaultsPath;
        }
        packages.push_back(std::move(package));
    }
    return packages;
}

// Platform library names tried in order for a native mod entry (a base name).
std::vector<std::filesystem::path> nativeModLibraryCandidates(
    const std::filesystem::path &packageDirectory, const std::string &entry)
{
    std::vector<std::filesystem::path> libraryCandidates;
#if defined(_WIN32)
    libraryCandidates.push_back(packageDirectory / (entry + ".dll"));
    libraryCandidates.push_back(packageDirectory / ("lib" + entry + ".dll"));
#else
    libraryCandidates.push_back(packageDirectory / ("lib" + entry + ".so"));
    libraryCandidates.push_back(packageDirectory / (entry + ".so"));
#endif
    return libraryCandidates;
}

// dlopens the mod library, verifies its entry table and interface version, then
// creates the instance; the compiled manifest name must agree with mod.toml.
// Failures destroy anything already created.
bool loadNativeModEntry(const DiscoveredGameModPackage &package, NativeGameModLoad &load, std::string &report)
{
    const std::filesystem::path packageDirectory = package.manifest.directory;
    const GameModManifest      &gameModManifest  = package.manifest;

    std::filesystem::path libraryPath;
    for (const std::filesystem::path &candidate : nativeModLibraryCandidates(packageDirectory, gameModManifest.entry))
    {
        if (std::filesystem::exists(candidate))
        {
            libraryPath = candidate;
            break;
        }
    }
    if (libraryPath.empty())
    {
        std::string expectedNames;
        for (const std::filesystem::path &candidate :
            nativeModLibraryCandidates(packageDirectory, gameModManifest.entry))
        {
            expectedNames += (expectedNames.empty() ? "" : ", ") + candidate.filename().string();
        }
        report += "- native mod '" + gameModManifest.id + "' has no library file in '" + packageDirectory.string()
                + "' (looked for " + expectedNames + "); build the mod first.\n";
        return false;
    }

    load.library = std::make_unique<DynamicLibrary>();
    std::string loadError;
    if (!load.library->load(libraryPath, loadError))
    {
        report += "- " + loadError + "\n";
        return false;
    }
    using ModEntryQueryFunction = const EngineGameModEntryTable *(*)();
    const auto queryFunction
        = reinterpret_cast<ModEntryQueryFunction>(load.library->resolveSymbol(engineGameModEntryTableSymbol));
    if (queryFunction == nullptr)
    {
        report += "- shared library mod '" + libraryPath.string() + "' does not export "
                + std::string(engineGameModEntryTableSymbol) + ".\n";
        return false;
    }
    load.entryTable = queryFunction();
    if (load.entryTable == nullptr)
    {
        report += "- shared library mod '" + libraryPath.string() + "' returned no entry table.\n";
        return false;
    }
    if (load.entryTable->interfaceVersion != engineGameModInterfaceVersion)
    {
        report += "- shared library mod '" + libraryPath.string() + "' reports interface version "
                + std::to_string(load.entryTable->interfaceVersion) + " but the engine provides version "
                + std::to_string(engineGameModInterfaceVersion) + "; rebuild the mod against this engine.\n";
        return false;
    }
    load.gameModInstance = load.entryTable->createMod();
    if (load.gameModInstance == nullptr)
    {
        report += "- shared library mod '" + libraryPath.string() + "' created no mod instance.\n";
        return false;
    }

    const ParticipantManifest compiledManifest = load.gameModInstance->manifest();
    if (!compiledManifest.name.empty() && compiledManifest.name != gameModManifest.id)
    {
        report += "- native mod '" + gameModManifest.id + "' reports the compiled manifest name '"
                + compiledManifest.name + "'; the compiled manifest and mod.toml must agree.\n";
        load.entryTable->destroyMod(load.gameModInstance);
        load.gameModInstance = nullptr;
        return false;
    }
    return true;
}

} // namespace engine
