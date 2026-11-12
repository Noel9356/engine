// EngineKernel-internal helpers: the <app> participant name, executable-
// relative path resolution, permission derivation and the module-defaults
// directory scan shared by the lifecycle and configuration phases.

#ifndef ENGINE_KERNEL_ENGINE_KERNEL_INTERNAL_HPP
#define ENGINE_KERNEL_ENGINE_KERNEL_INTERNAL_HPP

#include "engine/kernel/participant_manifest.hpp"
#include <engine/kernel/configuration.hpp>
#include <engine/kernel/engine_kernel.hpp>
#include <engine/kernel/executable_directory.hpp>
#include <engine/kernel/plugin_manifest.hpp>
#include <engine/kernel/permission_system.hpp>

#include <algorithm>
#include <filesystem>
#include <set>
#include <string>
#include <vector>

namespace engine::detail
{

// Reserved name of the application's own participant ("<app>").
inline const std::string &appContextName()
{
    static const std::string name = "<app>";
    return name;
}

// Relative paths are interpreted against the executable directory, not the cwd.
inline std::filesystem::path resolveAgainstExecutableDirectory(const std::filesystem::path &path)
{
    return path.is_absolute() ? path : executableDirectory() / path;
}

// Manifest permissions plus capability.<name> for every (optionally) required
// capability.
inline std::vector<std::string> effectivePermissions(const ParticipantManifest &manifest)
{
    std::set<std::string> unique(manifest.requiredPermissions.begin(), manifest.requiredPermissions.end());
    for (const std::string &capabilityName : manifest.requiredCapabilities)
    {
        unique.insert(std::string(capabilityPermissionPrefix) + capabilityName);
    }
    for (const std::string &capabilityName : manifest.optionalCapabilities)
    {
        unique.insert(std::string(capabilityPermissionPrefix) + capabilityName);
    }
    return {unique.begin(), unique.end()};
}

inline bool sameStringSet(const std::vector<std::string> &left, const std::vector<std::string> &right)
{
    return std::set<std::string>(left.begin(), left.end()) == std::set<std::string>(right.begin(), right.end());
}

// config/modules/*/config.toml paths, sorted for a deterministic merge order.
inline std::vector<std::filesystem::path> collectModuleDefaultsFiles()
{
    const std::filesystem::path defaultsDirectory
        = resolveAgainstExecutableDirectory(ConfigurationStore::moduleDefaultsDirectory);
    std::vector<std::filesystem::path> moduleDirectories;
    if (!std::filesystem::exists(defaultsDirectory) || !std::filesystem::is_directory(defaultsDirectory))
    {
        return {};
    }
    for (const std::filesystem::directory_entry &entry : std::filesystem::directory_iterator(defaultsDirectory))
    {
        if (entry.is_directory() && std::filesystem::exists(entry.path() / ConfigurationStore::moduleDefaultsFileName))
        {
            moduleDirectories.push_back(entry.path());
        }
    }
    std::sort(moduleDirectories.begin(), moduleDirectories.end());
    std::vector<std::filesystem::path> defaultsFiles;
    defaultsFiles.reserve(moduleDirectories.size());
    for (const std::filesystem::path &moduleDirectory : moduleDirectories)
    {
        defaultsFiles.push_back(moduleDirectory / ConfigurationStore::moduleDefaultsFileName);
    }
    return defaultsFiles;
}

// Maps a plugin.toml manifest onto a ParticipantManifest (script plugins provide nothing;
// their host module does).
inline ParticipantManifest manifestFromPluginManifest(const PluginManifest &pluginManifest, ParticipantKind kind)
{
    ParticipantManifest manifest;
    manifest.name                 = pluginManifest.id;
    manifest.kind                 = kind;
    manifest.version              = pluginManifest.version;
    manifest.providedCapabilities = pluginManifest.providedCapabilities;
    manifest.requiredCapabilities = pluginManifest.capabilities;
    manifest.optionalCapabilities = pluginManifest.optionalCapabilities;
    manifest.requiredPermissions  = pluginManifest.permissions;
    manifest.requiredParticipants = pluginManifest.requiredParticipants;
    manifest.knownConfigKeys      = pluginManifest.configKeys;
    if (kind == ParticipantKind::ScriptPlugin)
    {
        manifest.providedCapabilities.clear();
    }
    return manifest;
}

} // namespace engine::detail

#endif
