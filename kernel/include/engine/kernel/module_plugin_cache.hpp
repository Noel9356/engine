#ifndef ENGINE_KERNEL_MODULE_PLUGIN_CACHE_HPP
#define ENGINE_KERNEL_MODULE_PLUGIN_CACHE_HPP

#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine
{

inline constexpr const char *modulePluginCacheFileName = "module_plugins.cache";

struct ModulePluginCacheEntry
{
    std::uintmax_t sizeBytes         = 0;
    std::uintmax_t modificationStamp = 0;

    std::string              moduleName;
    std::string              moduleVersion;
    std::string              contentSha256;
    std::vector<std::string> providedCapabilities;
    std::vector<std::string> requiredCapabilities;
    std::vector<std::string> optionalCapabilities;

    std::vector<std::string> declaredPermissions;
};

class ModulePluginCache final
{
public:
    ModulePluginCache() = default;

    ModulePluginCache(const ModulePluginCache &)            = delete;
    ModulePluginCache &operator=(const ModulePluginCache &) = delete;

    void setCacheFile(const std::filesystem::path &cacheFilePath)
    {
        cacheFile = cacheFilePath;
    }

    void load();

    const ModulePluginCacheEntry *find(
        const std::filesystem::path &libraryPath,
        std::uintmax_t               sizeBytes,
        std::uintmax_t               modificationStamp
    ) const;

    void record(
        const std::filesystem::path  &libraryPath,
        const ModulePluginCacheEntry &entry
    );

    void save() const;

private:
    std::filesystem::path                                   cacheFile;
    std::unordered_map<std::string, ModulePluginCacheEntry> entries;
};

} // namespace engine

#endif
