#ifndef ENGINE_KERNEL_MOD_MANIFEST_HPP
#define ENGINE_KERNEL_MOD_MANIFEST_HPP

#include <engine/kernel/mod_api.hpp>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace engine
{

struct ModManifest
{
    std::string              id;
    std::string              version   = "0.1.0";
    std::uint32_t            engineApi = engineModInterfaceVersion;
    std::string              type;
    std::string              entry;
    std::string              displayName;
    std::string              description;
    std::string              author;
    std::vector<std::string> capabilities;

    std::vector<std::string> providedCapabilities;

    std::vector<std::string> optionalCapabilities;

    std::string              hostCapability;
    std::vector<std::string> permissions;
    std::vector<std::string> requiresComponents;

    std::vector<std::string> configKeys;

    std::string scene;

    std::filesystem::path directory;

    bool isNative() const
    {
        return type == "native";
    }

    bool isScript() const
    {
        return type == "script";
    }
};

class ModManifestLoader final
{
public:
    static constexpr const char *manifestFileName = "mod.toml";

    bool loadFromFile(
        const std::filesystem::path &manifestPath,
        ModManifest                 &outManifest,
        std::string                 &errorMessage
    ) const;
};

} // namespace engine

#endif
