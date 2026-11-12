// Game-plugin package discovery: plugins/*/plugin.toml scanning and native loading.
//
// discoverPluginPackages collects candidate packages (with scene and defaults
// sidecars); loadNativePluginEntry dlopens a native plugin's library, checks its
// ABI and returns the entry table plus instance for the kernel.

#include "engine/kernel/dynamic_library.hpp"

#include <algorithm>
#include <engine/kernel/configuration.hpp>
#include <engine/kernel/plugin_api.hpp>
#include <engine/kernel/plugin_discovery.hpp>
#include <engine/kernel/plugin_manifest.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <filesystem>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace engine
{

// Every subdirectory with a plugin.toml is a package; ids must be unique and not
// disabled. Directory order is sorted for determinism.
std::vector<DiscoveredPluginPackage> discoverPluginPackages(
    const std::filesystem::path &pluginsDirectory, const std::vector<std::string> &disabledIds, std::string &report)
{
    std::vector<DiscoveredPluginPackage> packages;
    if (!std::filesystem::exists(pluginsDirectory) || !std::filesystem::is_directory(pluginsDirectory))
    {
        return packages;
    }

    std::vector<std::filesystem::path> packageDirectories;
    for (const std::filesystem::directory_entry &entry : std::filesystem::directory_iterator(pluginsDirectory))
    {
        if (!entry.is_directory())
        {
            continue;
        }
        if (!std::filesystem::exists(entry.path() / PluginManifestLoader::manifestFileName))
        {
            report += "- directory '" + entry.path().string() + "' inside the plugins directory has no "
                    + std::string(PluginManifestLoader::manifestFileName)
                    + "; every subdirectory of a plugins directory is a plugin package, so this one is skipped."
                      " Add the manifest or move the directory out of the plugins tree.\n";
            continue;
        }
        packageDirectories.push_back(entry.path());
    }
    std::sort(packageDirectories.begin(), packageDirectories.end());

    PluginManifestLoader manifestLoader;
    std::set<std::string> seenIds;
    for (const std::filesystem::path &packageDirectory : packageDirectories)
    {
        DiscoveredPluginPackage package;
        std::string              manifestError;
        if (!manifestLoader.loadFromFile(
                packageDirectory / PluginManifestLoader::manifestFileName, package.manifest, manifestError))
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
            report += "- duplicate plugin id '" + package.manifest.id + "' in '" + packageDirectory.string()
                    + "'; plugin ids must be unique across the plugins directory.\n";
            continue;
        }

        if (!package.manifest.scene.empty())
        {
            package.sceneFile = packageDirectory / package.manifest.scene;
        }
        const std::filesystem::path defaultsPath = packageDirectory / ConfigurationStore::pluginDefaultsFileName;
        if (std::filesystem::exists(defaultsPath))
        {
            package.defaultsFile = defaultsPath;
        }
        packages.push_back(std::move(package));
    }
    return packages;
}

// Platform library names tried in order for a native plugin entry (a base name).
std::vector<std::filesystem::path> nativePluginLibraryCandidates(
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

// dlopens the plugin library, verifies its entry table and interface version, then
// creates the instance; the compiled manifest name must agree with plugin.toml.
// Failures destroy anything already created.
bool loadNativePluginEntry(const DiscoveredPluginPackage &package, NativePluginLoad &load, std::string &report)
{
    const std::filesystem::path packageDirectory = package.manifest.directory;
    const PluginManifest      &pluginManifest  = package.manifest;

    std::filesystem::path libraryPath;
    for (const std::filesystem::path &candidate : nativePluginLibraryCandidates(packageDirectory, pluginManifest.entry))
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
            nativePluginLibraryCandidates(packageDirectory, pluginManifest.entry))
        {
            expectedNames += (expectedNames.empty() ? "" : ", ") + candidate.filename().string();
        }
        report += "- native plugin '" + pluginManifest.id + "' has no library file in '" + packageDirectory.string()
                + "' (looked for " + expectedNames + "); build the plugin first.\n";
        return false;
    }

    load.library = std::make_unique<DynamicLibrary>();
    std::string loadError;
    if (!load.library->load(libraryPath, loadError))
    {
        report += "- " + loadError + "\n";
        return false;
    }
    using PluginEntryQueryFunction = const EnginePluginEntryTable *(*)();
    const auto queryFunction
        = reinterpret_cast<PluginEntryQueryFunction>(load.library->resolveSymbol(enginePluginEntryTableSymbol));
    if (queryFunction == nullptr)
    {
        report += "- shared library plugin '" + libraryPath.string() + "' does not export "
                + std::string(enginePluginEntryTableSymbol) + ".\n";
        return false;
    }
    load.entryTable = queryFunction();
    if (load.entryTable == nullptr)
    {
        report += "- shared library plugin '" + libraryPath.string() + "' returned no entry table.\n";
        return false;
    }
    if (load.entryTable->interfaceVersion != enginePluginInterfaceVersion)
    {
        report += "- shared library plugin '" + libraryPath.string() + "' reports interface version "
                + std::to_string(load.entryTable->interfaceVersion) + " but the engine provides version "
                + std::to_string(enginePluginInterfaceVersion) + "; rebuild the plugin against this engine.\n";
        return false;
    }
    load.pluginInstance = load.entryTable->createPlugin();
    if (load.pluginInstance == nullptr)
    {
        report += "- shared library plugin '" + libraryPath.string() + "' created no plugin instance.\n";
        return false;
    }

    const ParticipantManifest compiledManifest = load.pluginInstance->manifest();
    if (!compiledManifest.name.empty() && compiledManifest.name != pluginManifest.id)
    {
        report += "- native plugin '" + pluginManifest.id + "' reports the compiled manifest name '"
                + compiledManifest.name + "'; the compiled manifest and plugin.toml must agree.\n";
        load.entryTable->destroyPlugin(load.pluginInstance);
        load.pluginInstance = nullptr;
        return false;
    }
    return true;
}

} // namespace engine
