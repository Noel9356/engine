#ifndef ENGINE_MODULES_HOT_RELOAD_HOT_RELOAD_CAPABILITY_HPP
#define ENGINE_MODULES_HOT_RELOAD_HOT_RELOAD_CAPABILITY_HPP

// The published capability surface of the hot reload module. Every consumer -
// other modules, native mods, the host - includes THIS header, from the
// module that provides the capability. Like every published capability header
// this stays implementation-, platform- and vendor-agnostic.

#include <cstdint>
#include <string>

namespace engine
{

// What kind of file changed. The value decides who can react: the renderer
// reloads shaders, the scripting module reloads the named mod's script,
// configuration changes are logged (an assembly restart is the honest reload
// path for configuration).
namespace hot_reload_kind
{
inline constexpr std::uint32_t shader      = 0; // a compiled .spv under runtime shaders
inline constexpr std::uint32_t script      = 1; // a mod's .lua (data carries the package id)
inline constexpr std::uint32_t manifest    = 2; // a mod.toml (restart required to apply)
inline constexpr std::uint32_t scene       = 3; // a scene.toml (restart, or re-import via scripts)
inline constexpr std::uint32_t config      = 4; // an engine/mod .toml (restart required)
} // namespace hot_reload_kind

// Published on the MAIN thread when a watched file changed on disk. The
// relativePath is engine-relative (the executable directory as root), the
// form every consumer resolves paths in.
struct HotReloadEvent
{
    std::uint32_t kind         = 0;
    std::string   relativePath;
    std::string   modId; // non-empty for script/manifest/scene events
};

// Capability "hot_reload": filesystem watching for the development loop.
//
// The module polls watched trees on a worker thread (the kernel's
// AsyncTaskDispatcher bridges the results back onto the main thread), so the
// frame loop only ever sees a cheap drain. Consumers subscribe to the event
// channel and decide what a change means for them; the module itself only
// watches and reports.
class HotReloadCapability
{
public:
    static constexpr const char *capabilityName = "hot_reload";

    virtual ~HotReloadCapability() = default;

    // Adds one file or one directory (recursively, matching the watcher's
    // extension set) to the watch set at runtime. Returns false when the
    // path does not exist. The standard trees (runtime shaders, mod
    // packages, configuration) are watched automatically; this call exists
    // for content that lives elsewhere.
    virtual bool watchPath(
        const std::string &engineRelativePath
    ) = 0;

    virtual std::uint32_t watchedFileCount() const = 0;
};

} // namespace engine

#endif // ENGINE_MODULES_HOT_RELOAD_HOT_RELOAD_CAPABILITY_HPP
