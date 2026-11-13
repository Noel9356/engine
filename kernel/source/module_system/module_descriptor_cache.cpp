// ModuleDescriptorCache: on-disk cache of scanned module descriptors.
//
// Remembers descriptors keyed by library path + size + mtime so a scanned
// library stays described without re-scanning. Text format "format 3",
// rewritten atomically via a temporary file.

#include <engine/kernel/module_descriptor_cache.hpp>
#include <engine/kernel/sha256.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <ios>
#include <limits>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace engine
{

namespace
{

bool splitCacheLine(const std::string &line, std::string &key, std::string &value)
{
    const std::size_t split = line.find(' ');
    if (split == std::string::npos)
    {
        key   = line;
        value = std::string();
        return !key.empty();
    }
    key   = line.substr(0, split);
    value = line.substr(split + 1);
    return true;
}

// Overflow-checked decimal parse for the size/mtime fields.
bool parseDecimal(const std::string &text, std::uintmax_t &out)
{
    if (text.empty())
    {
        return false;
    }
    std::uintmax_t parsed = 0;
    for (const char digit : text)
    {
        if (digit < '0' || digit > '9')
        {
            return false;
        }
        const std::uintmax_t digitValue = static_cast<std::uintmax_t>(digit - '0');
        const std::uintmax_t maximum    = std::numeric_limits<std::uintmax_t>::max();
        if (parsed > (maximum - digitValue) / 10u)
        {
            return false;
        }
        parsed = parsed * 10u + digitValue;
    }
    out = parsed;
    return true;
}

} // namespace

// Reads "format 3" records; a record counts only with a path and a module name.
// Unparsable lines are skipped silently (the cache is advisory).
void ModuleDescriptorCache::load()
{
    entries.clear();
    if (cacheFile.empty())
    {
        return;
    }
    constexpr std::uintmax_t maximumCacheBytes = 64ull * 1024ull * 1024ull;
    std::error_code          sizeError;
    const std::uintmax_t     cacheSize = std::filesystem::file_size(cacheFile, sizeError);
    if (sizeError || cacheSize > maximumCacheBytes)
    {
        return;
    }
    std::ifstream cacheStream(cacheFile, std::ios::in | std::ios::binary);
    if (!cacheStream.is_open())
    {
        return;
    }

    std::string line;
    if (!std::getline(cacheStream, line) || line != "format 3")
    {
        return;
    }

    constexpr std::size_t maximumCacheEntries = 65'536u;
    constexpr std::size_t maximumCacheLineBytes = 4096u;
    constexpr std::size_t maximumDescriptorValues = 256u;
    ModuleDescriptorCacheEntry pending;
    std::string                pendingPath;
    bool                       recordOpen = false;
    while (std::getline(cacheStream, line))
    {
        if (line.size() > maximumCacheLineBytes)
        {
            entries.clear();
            return;
        }
        std::string key;
        std::string value;
        if (!splitCacheLine(line, key, value))
        {
            continue;
        }
        if (key == "record")
        {
            if (recordOpen && !pendingPath.empty() && !pending.descriptor.name.empty())
            {
                if (entries.size() >= maximumCacheEntries)
                {
                    entries.clear();
                    return;
                }
                entries.emplace(pendingPath, pending);
            }
            pending     = ModuleDescriptorCacheEntry{};
            pendingPath = std::string();
            recordOpen  = true;
            continue;
        }
        if (!recordOpen)
        {
            continue;
        }
        if (key == "size")
        {
            (void)parseDecimal(value, pending.sizeBytes);
        }
        else if (key == "mtime")
        {
            (void)parseDecimal(value, pending.modificationStamp);
        }
        else if (key == "path")
        {
            if (value.size() <= maximumCacheLineBytes)
            {
                pendingPath = value;
            }
        }
        else if (key == "name")
        {
            pending.descriptor.name = value;
        }
        else if (key == "version")
        {
            pending.descriptor.version = value;
        }
        else if (key == "sha256")
        {
            if (value.size() == 64u)
            {
                pending.contentSha256 = value;
            }
        }
        else if (key == "provided")
        {
            if (!value.empty())
            {
                if (pending.descriptor.providedCapabilities.size() >= maximumDescriptorValues)
                {
                    entries.clear();
                    return;
                }
                pending.descriptor.providedCapabilities.push_back(value);
            }
        }
        else if (key == "required")
        {
            if (!value.empty())
            {
                if (pending.descriptor.requiredCapabilities.size() >= maximumDescriptorValues)
                {
                    entries.clear();
                    return;
                }
                pending.descriptor.requiredCapabilities.push_back(value);
            }
        }
        else if (key == "optional")
        {
            if (!value.empty())
            {
                if (pending.descriptor.optionalCapabilities.size() >= maximumDescriptorValues)
                {
                    entries.clear();
                    return;
                }
                pending.descriptor.optionalCapabilities.push_back(value);
            }
        }
        else if (key == "permission")
        {
            if (!value.empty())
            {
                if (pending.descriptor.declaredPermissions.size() >= maximumDescriptorValues)
                {
                    entries.clear();
                    return;
                }
                pending.descriptor.declaredPermissions.push_back(value);
            }
        }
    }
    if (recordOpen && !pendingPath.empty() && !pending.descriptor.name.empty())
    {
        if (entries.size() >= maximumCacheEntries)
        {
            entries.clear();
            return;
        }
        entries.emplace(pendingPath, pending);
    }
}

// A hit requires size and mtime to match; the stored sha256 is for advisories, not
// compared here.
const ModuleDescriptorCacheEntry *ModuleDescriptorCache::find(const std::filesystem::path &libraryPath,
    std::uintmax_t sizeBytes, std::uintmax_t modificationStamp, const std::string &contentSha256) const
{
    const ModuleDescriptorCacheEntry *entry = findByStamps(libraryPath, sizeBytes, modificationStamp);
    if (entry == nullptr)
    {
        return nullptr;
    }
    // Case-insensitive so cached lowercase and freshly-computed lowercase (or any
    // case in [trust] bindings) compare equal.
    if (!sha256HexDigestsEqual(entry->contentSha256, contentSha256))
    {
        return nullptr;
    }
    return entry;
}

// Fast path: trust size+mtime as the validity signal and return the cached sha256
// alongside the descriptor so the caller can skip re-hashing unchanged binaries.
const ModuleDescriptorCacheEntry *ModuleDescriptorCache::findByStamps(
    const std::filesystem::path &libraryPath, std::uintmax_t sizeBytes, std::uintmax_t modificationStamp) const
{
    const auto found = entries.find(libraryPath.string());
    if (found == entries.end())
    {
        return nullptr;
    }
    const ModuleDescriptorCacheEntry &entry = found->second;
    if (entry.sizeBytes != sizeBytes || entry.modificationStamp != modificationStamp)
    {
        return nullptr;
    }
    return &entry;
}

void ModuleDescriptorCache::record(const std::filesystem::path &libraryPath, const ModuleDescriptorCacheEntry &entry)
{
    entries.insert_or_assign(libraryPath.string(), entry);
}

// Writes to "<cache>.tmp" then renames (atomic swap); entries are sorted by path for
// a stable, diff-friendly file.
void ModuleDescriptorCache::save() const
{
    if (cacheFile.empty() || entries.empty())
    {
        return;
    }

    std::vector<const std::pair<const std::string, ModuleDescriptorCacheEntry> *> ordered;
    ordered.reserve(entries.size());
    for (const auto &entry : entries)
    {
        ordered.push_back(&entry);
    }
    std::sort(ordered.begin(),
        ordered.end(),
        [](const std::pair<const std::string, ModuleDescriptorCacheEntry>  *left,
            const std::pair<const std::string, ModuleDescriptorCacheEntry> *right)
        { return left->first < right->first; });

    const std::filesystem::path temporaryFile = cacheFile.string() + ".tmp";
    {
        std::ofstream cacheStream(temporaryFile, std::ios::out | std::ios::binary | std::ios::trunc);
        if (!cacheStream.is_open())
        {
            return;
        }
        cacheStream << "format 3\n";
        for (const auto *entry : ordered)
        {
            cacheStream << "record\n";
            cacheStream << "size " << entry->second.sizeBytes << "\n";
            cacheStream << "mtime " << entry->second.modificationStamp << "\n";
            cacheStream << "path " << entry->first << "\n";
            cacheStream << "name " << entry->second.descriptor.name << "\n";
            cacheStream << "version " << entry->second.descriptor.version << "\n";
            cacheStream << "sha256 " << entry->second.contentSha256 << "\n";
            for (const std::string &capability : entry->second.descriptor.providedCapabilities)
            {
                cacheStream << "provided " << capability << "\n";
            }
            for (const std::string &capability : entry->second.descriptor.requiredCapabilities)
            {
                cacheStream << "required " << capability << "\n";
            }
            for (const std::string &capability : entry->second.descriptor.optionalCapabilities)
            {
                cacheStream << "optional " << capability << "\n";
            }
            for (const std::string &permission : entry->second.descriptor.declaredPermissions)
            {
                cacheStream << "permission " << permission << "\n";
            }
        }
        if (!cacheStream.good())
        {
            cacheStream.close();
            std::error_code discardError;
            (void)std::filesystem::remove(temporaryFile, discardError);
            return;
        }
    }
    std::error_code renameError;
    std::filesystem::rename(temporaryFile, cacheFile, renameError);
    if (renameError)
    {
        std::error_code discardError;
        (void)std::filesystem::remove(temporaryFile, discardError);
    }
}

} // namespace engine
