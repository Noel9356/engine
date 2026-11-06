// module.toml loader: the data descriptor shipped beside a module library.
//
// A manifest lets the kernel identify a module without executing its code
// (no trust decision needed); its descriptor must match what the library's
// entry table reports.

#ifndef ENGINE_KERNEL_MODULE_MANIFEST_HPP
#define ENGINE_KERNEL_MODULE_MANIFEST_HPP

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/module_descriptor.hpp>

#include <cstdint>
#include <filesystem>
#include <string>

namespace engine
{

// Parsed module.toml; abi must equal engineModuleAbiVersion.
struct ModuleManifest
{
    ModuleDescriptor descriptor;

    // Optional: names the .so this manifest describes (empty = the only
    // library in the directory).
    std::string library;

    std::uint32_t abi = engineModuleAbiVersion;
};

class ModuleManifestLoader final
{
public:
    static constexpr const char *manifestFileName = "module.toml";

    // Returns false with a human-readable errorMessage on any problem.
    bool loadFromFile(
        const std::filesystem::path &manifestPath, ModuleManifest &outManifest, std::string &errorMessage) const;
};

} // namespace engine

#endif
