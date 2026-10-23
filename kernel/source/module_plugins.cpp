// Runtime module loading: the plugin seam of the kernel.
//
// Every shared library in the modules directory declares itself through one
// exported entry point (engine/kernel/module_api.hpp). Loading a plugin is
// therefore uniform - dlopen, resolve, version check, register a factory -
// and the composition root no longer links any module: the set of available
// modules comes from the runtime layout, not from the link line. Load
// failures and ABI mismatches are reported and skipped here; a module listed
// in the configuration but backed by no factory then fails through the regular
// availability diagnostic in gatherComponents, so both compiled-out and
// failed-to-load plugins surface through one reporting path.

#include "engine/kernel/engine_kernel.hpp"
#include "engine/kernel/filesystem_utilities.hpp"
#include "engine/kernel/module_api.hpp"

#include <filesystem>
#include <memory>
#include <unordered_map>
#include <vector>

namespace engine
{

namespace
{

// Both shared library extensions are accepted on every platform: the modules
// directory is a deployment layout, and a plugin tree assembled for one
// platform must not silently look empty when inspected from the other.
bool isModulePluginFile(
    const std::filesystem::path &path
)
{
    const std::string extension = path.extension().string();
    return extension == ".dll" || extension == ".so";
}

} // namespace

// Scans the modules directory and registers one module factory per plugin.
// The libraries stay loaded for the kernel lifetime: factories handed out
// here are plain function pointers into the plugin binaries, and the member
// declaration order unloads them after the component records are released.
void EngineKernel::loadModulePlugins(
    std::string &report
)
{
    std::filesystem::path       modulesDirectory  = configurationStore.modulesDirectory();
    const std::filesystem::path resolvedDirectory = modulesDirectory.is_absolute()
                                                      ? modulesDirectory
                                                      : executableDirectory() / modulesDirectory;

    std::error_code                     scanError;
    std::filesystem::directory_iterator plugin{
        resolvedDirectory, std::filesystem::directory_options::skip_permission_denied, scanError
    };
    if (scanError)
    {
        report += "- module plugins directory '" + resolvedDirectory.string()
                + "' cannot be scanned: " + scanError.message() + "\n";
        return;
    }

    for (const std::filesystem::directory_entry &entry : plugin)
    {
        std::error_code statError;
        if (statError || !entry.is_regular_file())
        {
            continue;
        }
        const std::filesystem::path libraryPath = entry.path();
        if (!isModulePluginFile(libraryPath))
        {
            continue;
        }

        auto        library = std::make_unique<DynamicLibrary>();
        std::string loadError;
        if (!library->load(libraryPath, loadError))
        {
            report += "- " + loadError + "\n";
            continue;
        }

        const auto query = reinterpret_cast<const EngineModulePluginEntry *(*)()>(
            library->resolveSymbol("engineQueryModulePlugin")
        );
        if (query == nullptr)
        {
            report += "- '" + libraryPath.string()
                    + "' exports no engineQueryModulePlugin entry point; not a module plugin.\n";
            continue;
        }

        const EngineModulePluginEntry *pluginEntry = query();
        if (pluginEntry == nullptr || pluginEntry->abiVersion != engineModulePluginAbiVersion)
        {
            report += "- '" + libraryPath.string()
                    + "' was built against a different module plugin ABI (expected version "
                    + std::to_string(engineModulePluginAbiVersion) + "); rebuild the plugin with this kernel.\n";
            continue;
        }
        if (pluginEntry->moduleName == nullptr || pluginEntry->createModule == nullptr)
        {
            report += "- '" + libraryPath.string() + "' returns an incomplete plugin descriptor.\n";
            continue;
        }

        const std::string moduleName = pluginEntry->moduleName;
        if (moduleFactories.find(moduleName) != moduleFactories.end())
        {
            report += "- module '" + moduleName + "' is provided by more than one plugin binary; '"
                    + libraryPath.string() + "' is ignored.\n";
            continue;
        }

        ModuleInterface *(*createModule)() = pluginEntry->createModule;
        moduleFactories.emplace(
            moduleName, [createModule]() { return std::unique_ptr<ModuleInterface>(createModule()); }
        );
        modulePluginLibraries.push_back(std::move(library));
    }
}

} // namespace engine
