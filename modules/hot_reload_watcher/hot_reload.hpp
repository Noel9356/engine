#ifndef ENGINE_MODULES_HOT_RELOAD_HPP
#define ENGINE_MODULES_HOT_RELOAD_HPP

#include "hot_reload_capability.hpp"
#include "logging_capability.hpp"
#include "renderer_capability.hpp"
#include "engine/kernel/async_dispatcher.hpp"
#include "engine/kernel/component_interface.hpp"
#include "engine/kernel/event_bus.hpp"

#include <chrono>
#include <filesystem>
#include <map>
#include <mutex>
#include <string>

namespace engine
{

// Module "hot_reload_watcher": filesystem watching for the development loop.
//
// Shape: a worker thread (the kernel's AsyncTaskDispatcher) polls the watch
// set's mtimes every [hot_reload].watch_interval_ms; each poll's result
// comes back as MAIN-thread completions (drained on the frame event), which
// is where the HotReloadEvent is published - the bus stays single threaded,
// the frame loop pays one drain call.
//
// What is watched automatically (all engine-relative):
//   runtime/shaders/*.spv   -> kind shader   (the renderer re-creates its pipelines)
//   mods/<id>/*.lua         -> kind script   (the scripting module reloads the mod)
//   mods/<id>/mod.toml      -> kind manifest (logged; restart applies)
//   mods/<id>/scene.toml    -> kind scene    (logged; restart, or re-import)
//   config/**/*.toml        -> kind config   (logged; restart applies)
//
// Consumers: the renderer (shaders) and the scripting module (scripts)
// subscribe to the bus channel themselves - the module publishes, it does
// not know who reacts.
class HotReloadModule final : public ModuleInterface, public HotReloadCapability
{
public:
    HotReloadModule();
    ~HotReloadModule() override;

    const ComponentManifest &manifest() const override
    {
        return componentManifest;
    }

    void construct(ComponentContext &context) override;
    void start() override;
    void stop() override;

    void *capabilityPointerFor(const std::string &capabilityName) override;

    // HotReloadCapability.
    bool           watchPath(const std::string &engineRelativePath) override;
    std::uint32_t  watchedFileCount() const override;

private:
    struct WatchedFile
    {
        std::filesystem::path    absolutePath;
        std::uint32_t            kind = 0;
        std::string              modId;
        std::filesystem::file_time_type lastWriteTime{};
    };

    void buildDefaultWatchSet();
    void scanOnce();   // runs on the WORKER thread
    void forwardFrameBegin(const FrameBeginEvent &event);

    ComponentManifest   componentManifest;
    ComponentContext   *boundContext = nullptr;
    LoggerCapability   *logger       = nullptr;

    AsyncTaskDispatcher dispatcher;
    EventSubscription   frameBeginSubscription;
    EventChannel<FrameBeginEvent> *frameBeginChannel = nullptr;

    // The watch set. Written on the main thread (construct/start/watchPath);
    // read by the worker during scans. The mutex guards exactly this
    // hand-off - the only lock in the module.
    mutable std::mutex            watchMutex;
    std::map<std::string, WatchedFile> watchedFiles; // keyed by engine-relative path
    std::chrono::milliseconds     watchInterval{250};

    // One-time diagnostics.
    bool startupLogged = false;
};

} // namespace engine

#endif // ENGINE_MODULES_HOT_RELOAD_HPP
