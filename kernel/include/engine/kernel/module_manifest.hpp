#ifndef ENGINE_KERNEL_MODULE_MANIFEST_HPP
#define ENGINE_KERNEL_MODULE_MANIFEST_HPP

#include <engine/kernel/module_api.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace engine
{

// Data-only descriptor for a module plugin. A module.toml lets the kernel
// plan (dependency graph, demand closure, trust gate) without executing any
// plugin code: the invariant is that no component code runs before the trust
// gate, so descriptors must come from data, not from calling into the binary.
//
// A module.toml sits next to the library it describes (same directory). When
// a directory holds several plugin libraries, it must name its library with
// the optional 'library' key.
struct ModuleManifest
{
    std::string name;

    std::string version = "0.1.0";

    std::vector<std::string> providedCapabilities;

    std::vector<std::string> requiredCapabilities;

    std::vector<std::string> optionalCapabilities;

    std::vector<std::string> declaredPermissions;

    // Optional: the library file (filename only) this manifest describes.
    std::string library;

    // Optional: the plugin ABI the manifest was written for.
    std::uint32_t abi = engineModulePluginAbiVersion;
};

class ModuleManifestLoader final
{
public:
    static constexpr const char *manifestFileName = "module.toml";

    bool loadFromFile(
        const std::filesystem::path &manifestPath,
        ModuleManifest              &outManifest,
        std::string                 &errorMessage
    ) const;
};

} // namespace engine

#endif
