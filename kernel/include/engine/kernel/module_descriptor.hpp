// Module descriptors: capability/permission metadata for module libraries,
// sourced from module.toml sidecars, the descriptor cache or the entry
// table.

#ifndef ENGINE_KERNEL_MODULE_DESCRIPTOR_HPP
#define ENGINE_KERNEL_MODULE_DESCRIPTOR_HPP

#include <engine/kernel/dynamic_library.hpp>
#include <engine/kernel/participant_interface.hpp>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace engine
{

// Static self-description of one module library.
struct ModuleDescriptor
{
    std::string name;

    // Default when the source omits it; must match the compiled manifest.
    std::string version = "0.1.0";

    // Capabilities the module implements (dotted names may bind specific
    // implementations).
    std::vector<std::string> providedCapabilities;

    std::vector<std::string> requiredCapabilities;

    // Consumed when available; never fail the graph.
    std::vector<std::string> optionalCapabilities;

    // Non-capability (environment) permissions; capability.* entries come
    // from the requirement lists and are not re-declared here.
    std::vector<std::string> declaredPermissions;
};

// A discovered module .so plus everything needed to load it later.
struct ModuleLibraryRecord
{
    ModuleDescriptor descriptor;

    // Hex digest of the library file; feeds the trust gate.
    std::string contentSha256;

    std::filesystem::path libraryPath;

    // Entry-table factory; valid only while the library stays loaded.
    ModuleInterface *(*createModule)() = nullptr;

    // Non-null once loaded (instantiateActiveModules); released again for
    // modules that turn out to be inactive.
    std::unique_ptr<DynamicLibrary> library;

    // Set by the demand closure: something in this run needs this module.
    bool active = false;
};

// A library with no descriptor source (no module.toml, no cache entry).
// Loading it to query the entry table would execute its code, so the
// trust gate decides first.
struct UndescribedModuleLibrary
{
    std::filesystem::path libraryPath;

    // Cache validation stamps (bytes, filesystem mtime tick count).
    std::uintmax_t sizeBytes         = 0;
    std::uintmax_t modificationStamp = 0;

    std::string contentSha256;
};

} // namespace engine

#endif
