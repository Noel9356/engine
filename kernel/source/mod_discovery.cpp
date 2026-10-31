#include <engine/kernel/mod_discovery.hpp>

#include <engine/kernel/configuration.hpp>

#include <algorithm>
#include <set>

namespace engine
{

std::vector<DiscoveredModPackage> discoverModPackages(
    const std::filesystem::path    &modsDirectory,
    const std::vector<std::string> &disabledIds,
    std::string                    &report
)
{
    std::vector<DiscoveredModPackage> packages;
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
        if (!std::filesystem::exists(entry.path() / ModManifestLoader::manifestFileName))
        {
            report += "- directory '" + entry.path().string() + "' inside the mods directory has no "
                    + std::string(ModManifestLoader::manifestFileName)
                    + "; every subdirectory of a mods directory is a mod package, so this one is skipped."
                      " Add the manifest or move the directory out of the mods tree.\n";
            continue;
        }
        packageDirectories.push_back(entry.path());
    }
    std::sort(packageDirectories.begin(), packageDirectories.end());

    ModManifestLoader     manifestLoader;
    std::set<std::string> seenIds;
    for (const std::filesystem::path &packageDirectory : packageDirectories)
    {
        DiscoveredModPackage package;
        std::string          manifestError;
        if (!manifestLoader.loadFromFile(
                packageDirectory / ModManifestLoader::manifestFileName, package.manifest, manifestError
            ))
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

std::vector<std::filesystem::path> nativeModLibraryCandidates(
    const std::filesystem::path &packageDirectory,
    const std::string           &entry
)
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

bool loadNativeModEntry(
    const DiscoveredModPackage &package,
    NativeModLoad              &load,
    std::string                &report
)
{
    const std::filesystem::path packageDirectory = package.manifest.directory;
    const ModManifest          &modManifest      = package.manifest;

    std::filesystem::path libraryPath;
    for (const std::filesystem::path &candidate : nativeModLibraryCandidates(packageDirectory, modManifest.entry))
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
             nativeModLibraryCandidates(packageDirectory, modManifest.entry))
        {
            expectedNames += (expectedNames.empty() ? "" : ", ") + candidate.filename().string();
        }
        report += "- native mod '" + modManifest.id + "' has no library file in '" + packageDirectory.string()
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
    using ModEntryQueryFunction = const EngineModEntryTable *(*)();
    const auto queryFunction
        = reinterpret_cast<ModEntryQueryFunction>(load.library->resolveSymbol(engineModEntryTableSymbol));
    if (queryFunction == nullptr)
    {
        report += "- shared library mod '" + libraryPath.string() + "' does not export "
                + std::string(engineModEntryTableSymbol) + ".\n";
        return false;
    }
    load.entryTable = queryFunction();
    if (load.entryTable == nullptr)
    {
        report += "- shared library mod '" + libraryPath.string() + "' returned no entry table.\n";
        return false;
    }
    if (load.entryTable->interfaceVersion != engineModInterfaceVersion)
    {
        report += "- shared library mod '" + libraryPath.string() + "' reports interface version "
                + std::to_string(load.entryTable->interfaceVersion) + " but the engine provides version "
                + std::to_string(engineModInterfaceVersion) + "; rebuild the mod against this engine.\n";
        return false;
    }
    load.modInstance = load.entryTable->createMod();
    if (load.modInstance == nullptr)
    {
        report += "- shared library mod '" + libraryPath.string() + "' created no mod instance.\n";
        return false;
    }

    const ComponentManifest compiledManifest = load.modInstance->manifest();
    if (!compiledManifest.name.empty() && compiledManifest.name != modManifest.id)
    {
        report += "- native mod '" + modManifest.id + "' reports the compiled manifest name '" + compiledManifest.name
                + "'; the compiled manifest and mod.toml must agree.\n";
        load.entryTable->destroyMod(load.modInstance);
        load.modInstance = nullptr;
        return false;
    }
    return true;
}

} // namespace engine
