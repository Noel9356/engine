#include <engine/kernel/module_plugin_cache.hpp>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <utility>

namespace engine
{

namespace
{

bool splitCacheLine(
    const std::string &line,
    std::string       &key,
    std::string       &value
)
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

bool parseDecimal(
    const std::string &text,
    std::uintmax_t    &out
)
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
        const std::uintmax_t next       = parsed * 10 + digitValue;
        if (next < parsed)
        {
            return false;
        }
        parsed = next;
    }
    out = parsed;
    return true;
}

} // namespace

void ModulePluginCache::load()
{
    entries.clear();
    if (cacheFile.empty())
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

    ModulePluginCacheEntry pending;
    std::string            pendingPath;
    bool                   recordOpen = false;
    while (std::getline(cacheStream, line))
    {
        std::string key;
        std::string value;
        if (!splitCacheLine(line, key, value))
        {
            continue;
        }
        if (key == "record")
        {
            if (recordOpen && !pendingPath.empty() && !pending.moduleName.empty())
            {
                entries.emplace(pendingPath, pending);
            }
            pending     = ModulePluginCacheEntry{};
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
        } else if (key == "mtime")
        {
            (void)parseDecimal(value, pending.modificationStamp);
        } else if (key == "path")
        {
            pendingPath = value;
        } else if (key == "name")
        {
            pending.moduleName = value;
        } else if (key == "version")
        {
            pending.moduleVersion = value;
        } else if (key == "sha256")
        {
            pending.contentSha256 = value;
        } else if (key == "provided")
        {
            if (!value.empty())
            {
                pending.providedCapabilities.push_back(value);
            }
        } else if (key == "required")
        {
            if (!value.empty())
            {
                pending.requiredCapabilities.push_back(value);
            }
        } else if (key == "optional")
        {
            if (!value.empty())
            {
                pending.optionalCapabilities.push_back(value);
            }
        } else if (key == "permission")
        {
            if (!value.empty())
            {
                pending.declaredPermissions.push_back(value);
            }
        }
    }
    if (recordOpen && !pendingPath.empty() && !pending.moduleName.empty())
    {
        entries.emplace(pendingPath, pending);
    }
}

const ModulePluginCacheEntry *ModulePluginCache::find(
    const std::filesystem::path &libraryPath,
    std::uintmax_t               sizeBytes,
    std::uintmax_t               modificationStamp
) const
{
    const auto found = entries.find(libraryPath.string());
    if (found == entries.end())
    {
        return nullptr;
    }
    const ModulePluginCacheEntry &entry = found->second;

    if (entry.sizeBytes != sizeBytes || entry.modificationStamp != modificationStamp)
    {
        return nullptr;
    }
    return &entry;
}

void ModulePluginCache::record(
    const std::filesystem::path  &libraryPath,
    const ModulePluginCacheEntry &entry
)
{
    entries.insert_or_assign(libraryPath.string(), entry);
}

void ModulePluginCache::save() const
{
    if (cacheFile.empty() || entries.empty())
    {
        return;
    }

    std::vector<const std::pair<const std::string, ModulePluginCacheEntry> *> ordered;
    ordered.reserve(entries.size());
    for (const auto &entry : entries)
    {
        ordered.push_back(&entry);
    }
    std::sort(
        ordered.begin(),
        ordered.end(),
        [](const std::pair<const std::string, ModulePluginCacheEntry>  *left,
            const std::pair<const std::string, ModulePluginCacheEntry> *right) { return left->first < right->first; }
    );

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
            cacheStream << "name " << entry->second.moduleName << "\n";
            cacheStream << "version " << entry->second.moduleVersion << "\n";
            cacheStream << "sha256 " << entry->second.contentSha256 << "\n";
            for (const std::string &capability : entry->second.providedCapabilities)
            {
                cacheStream << "provided " << capability << "\n";
            }
            for (const std::string &capability : entry->second.requiredCapabilities)
            {
                cacheStream << "required " << capability << "\n";
            }
            for (const std::string &capability : entry->second.optionalCapabilities)
            {
                cacheStream << "optional " << capability << "\n";
            }
            for (const std::string &permission : entry->second.declaredPermissions)
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
