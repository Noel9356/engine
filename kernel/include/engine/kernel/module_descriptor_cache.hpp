// Disk cache of module descriptors (sidecar replacement).
//
// Describing a module normally needs module.toml beside the library; the
// cache stores descriptors extracted from libraries the user already
// approved for scanning, keyed by path and validated by size/mtime.

#ifndef ENGINE_KERNEL_MODULE_DESCRIPTOR_CACHE_HPP
#define ENGINE_KERNEL_MODULE_DESCRIPTOR_CACHE_HPP

#include <engine/kernel/module_descriptor.hpp>

#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>

namespace engine
{

// Lives next to the executable.
inline constexpr const char *moduleDescriptorCacheFileName = "module_descriptors.cache";

// One cached library: validation stamps (bytes, mtime tick count), the
// content hash and the descriptor its entry table reported.
struct ModuleDescriptorCacheEntry
{
    std::uintmax_t sizeBytes         = 0;
    std::uintmax_t modificationStamp = 0;

    std::string contentSha256;

    ModuleDescriptor descriptor;
};

// Plain-text "format 3" records; entries whose size/mtime no longer
// match are ignored by find().
class ModuleDescriptorCache final
{
public:
    ModuleDescriptorCache() = default;

    ModuleDescriptorCache(const ModuleDescriptorCache &)            = delete;
    ModuleDescriptorCache &operator=(const ModuleDescriptorCache &) = delete;

    void setCacheFile(const std::filesystem::path &cacheFilePath)
    {
        cacheFile = cacheFilePath;
    }

    // Silently ignores a missing, empty or unparseable cache file.
    void load();

    // Returns the cached descriptor only when size and mtime match exactly.
    const ModuleDescriptorCacheEntry *find(
        const std::filesystem::path &libraryPath, std::uintmax_t sizeBytes, std::uintmax_t modificationStamp) const;

    // Adds or overwrites the in-memory entry for libraryPath; persisted by
    // save().
    void record(const std::filesystem::path &libraryPath, const ModuleDescriptorCacheEntry &entry);

    // Atomic write (temp file + rename); no-op when empty or unwritable.
    void save() const;

private:
    std::filesystem::path                                       cacheFile;
    std::unordered_map<std::string, ModuleDescriptorCacheEntry> entries;
};

} // namespace engine

#endif
