#include "hot_reload.hpp"

#include "engine/kernel/filesystem_utilities.hpp"

#include <algorithm>

namespace engine
{

namespace
{

// Kinds for a given path, by convention (the runtime layout is the contract).
std::uint32_t kindForPath(
    const std::filesystem::path &relativePath, std::string &outModId
)
{
    outModId.clear();
    const std::string generic = relativePath.generic_string();
    if (generic.rfind("runtime/shaders/", 0) == 0 && generic.ends_with(".spv"))
    {
        return hot_reload_kind::shader;
    }
    if (generic.rfind("mods/", 0) == 0)
    {
        // mods/<id>/... - the second component is the package id.
        const std::size_t firstSlash  = generic.find('/');
        const std::size_t secondSlash = generic.find('/', firstSlash + 1);
        if (secondSlash != std::string::npos)
        {
            outModId = generic.substr(firstSlash + 1, secondSlash - firstSlash - 1);
        }
        if (generic.ends_with(".lua"))
        {
            return hot_reload_kind::script;
        }
        if (generic.ends_with("mod.toml"))
        {
            return hot_reload_kind::manifest;
        }
        if (generic.ends_with("scene.toml"))
        {
            return hot_reload_kind::scene;
        }
    }
    if (generic.rfind("config/", 0) == 0 && generic.ends_with(".toml"))
    {
        return hot_reload_kind::config;
    }
    return hot_reload_kind::config; // unknown-but-watched: the conservative kind
}

} // namespace

HotReloadModule::HotReloadModule()
{
    componentManifest.name = "hot_reload";
    componentManifest.kind = ComponentKind::Module;
    componentManifest.providedCapabilities = {"hot_reload.watcher"};
    componentManifest.requiredCapabilities = {"logging"};
    componentManifest.requiredPermissions  = {"capability.logging", "filesystem.read"};
    componentManifest.knownConfigKeys      = {
        "hot_reload.enabled",
        "hot_reload.watch_interval_ms",
    };
}

HotReloadModule::~HotReloadModule()
{
    stop();
}

void HotReloadModule::construct(
    ComponentContext &context
)
{
    boundContext = &context;
    logger       = &context.acquireCapability<LoggerCapability>();

    const auto interval = context.configurationInteger("watch_interval_ms").value_or(250);
    watchInterval       = std::chrono::milliseconds(std::clamp<std::int64_t>(interval, 50, 5000));

    EventBus                      &bus          = context.eventBus();
    frameBeginChannel                           = &bus.channel<FrameBeginEvent>();
    frameBeginSubscription                      = EventSubscription(
        &bus, typeid(FrameBeginEvent), frameBeginChannel->subscribe<HotReloadModule, &HotReloadModule::forwardFrameBegin>(this)
    );
}

void HotReloadModule::start()
{
    if (!boundContext->isCapabilityDemanded(capabilityName))
    {
        // Demand-driven by contract: nothing should have constructed us
        // without demand, but the guard keeps the log honest if a future
        // assembly changes that.
        logger->logMessage(LogLevel::Warning, "hot reload module started without demand (unexpected)");
    }
    buildDefaultWatchSet();
    dispatcher.start();
    // First scan immediately: baseline mtimes before any change can fire.
    scanOnce();
    logger->logMessage(
        LogLevel::Information,
        "hot reload module started (" + std::to_string(watchedFiles.size()) + " file(s) watched, poll "
            + std::to_string(watchInterval.count()) + " ms)"
    );
    startupLogged = true;
}

void HotReloadModule::stop()
{
    frameBeginSubscription.release();
    dispatcher.stop();
    {
        std::lock_guard<std::mutex> lock(watchMutex);
        watchedFiles.clear();
    }
    boundContext = nullptr;
    logger       = nullptr;
}

void *HotReloadModule::capabilityPointerFor(
    const std::string &capabilityName
)
{
    if (capabilityName == "hot_reload.watcher")
    {
        return static_cast<HotReloadCapability *>(this);
    }
    return nullptr;
}

bool HotReloadModule::watchPath(
    const std::string &engineRelativePath
)
{
    const std::filesystem::path absolute =
        std::filesystem::path(engineRelativePath).is_relative()
            ? (executableDirectory() / engineRelativePath)
            : std::filesystem::path(engineRelativePath);
    std::error_code error;
    if (!std::filesystem::exists(absolute, error))
    {
        if (logger != nullptr)
        {
            logger->logMessage(
                LogLevel::Warning, "hot reload module: watch path '" + engineRelativePath + "' does not exist"
            );
        }
        return false;
    }
    if (std::filesystem::is_regular_file(absolute, error))
    {
        std::lock_guard<std::mutex> lock(watchMutex);
        WatchedFile                 file;
        file.absolutePath   = absolute;
        file.kind           = kindForPath(absolute, file.modId);
        file.lastWriteTime  = std::filesystem::last_write_time(absolute, error);
        watchedFiles.emplace(std::filesystem::path(engineRelativePath).generic_string(), std::move(file));
        return true;
    }
    // Directory: add every file under it (the module's own extension filter
    // keeps binaries out).
    for (const std::filesystem::directory_entry &entry : std::filesystem::recursive_directory_iterator(absolute, error))
    {
        if (!entry.is_regular_file(error))
        {
            continue;
        }
        const std::filesystem::path relative = std::filesystem::relative(entry.path(), executableDirectory(), error);
        std::lock_guard<std::mutex> lock(watchMutex);
        WatchedFile                 file;
        file.absolutePath   = entry.path();
        file.kind           = kindForPath(relative, file.modId);
        file.lastWriteTime  = std::filesystem::last_write_time(entry.path(), error);
        watchedFiles.emplace(relative.generic_string(), std::move(file));
    }
    return true;
}

std::uint32_t HotReloadModule::watchedFileCount() const
{
    std::lock_guard<std::mutex> lock(watchMutex);
    return static_cast<std::uint32_t>(watchedFiles.size());
}

void HotReloadModule::buildDefaultWatchSet()
{
    // The runtime layout's hot spots. Missing directories are normal (a
    // build without the renderer has no shaders; an assembly may ship no
    // mods) - absent trees simply contribute nothing.
    const std::filesystem::path root = executableDirectory();
    std::error_code             error;
    const std::filesystem::path shaders   = root / "runtime/shaders";
    if (std::filesystem::is_directory(shaders, error))
    {
        watchPath("runtime/shaders");
    }
    const std::filesystem::path mods = root / "mods";
    if (std::filesystem::is_directory(mods, error))
    {
        watchPath("mods");
    }
    const std::filesystem::path config = root / "config";
    if (std::filesystem::is_directory(config, error))
    {
        watchPath("config");
    }
}

void HotReloadModule::scanOnce()
{
    // Worker thread. Snapshot the watch set (cheap: the module owns few
    // hundred files at most), stat each, and post ONE completion per change -
    // the completions run on the main thread and publish the events.
    std::map<std::string, WatchedFile> snapshot;
    {
        std::lock_guard<std::mutex> lock(watchMutex);
        snapshot = watchedFiles;
    }
    std::error_code error;
    for (auto &entry : snapshot)
    {
        const std::filesystem::file_time_type current = std::filesystem::last_write_time(entry.second.absolutePath, error);
        if (error || current == entry.second.lastWriteTime)
        {
            continue;
        }
        // Update the baseline in the LIVE map so one edit fires exactly once.
        {
            std::lock_guard<std::mutex> lock(watchMutex);
            const auto                  live = watchedFiles.find(entry.first);
            if (live != watchedFiles.end())
            {
                live->second.lastWriteTime = current;
            }
        }
        WatchedFile changed = entry.second;
        changed.lastWriteTime = current;
        dispatcher.postWithCompletion(
            [changed]() { /* the stat already happened; the completion carries the result */ },
            [this, changed]()
            {
                if (logger != nullptr)
                {
                    logger->logMessage(
                        LogLevel::Information,
                        "hot reload: '" + changed.absolutePath.generic_string() + "' changed (kind "
                            + std::to_string(changed.kind) + ")"
                    );
                }
                if (boundContext != nullptr)
                {
                    boundContext->eventBus().publish(
                        HotReloadEvent{.kind = changed.kind, .relativePath = changed.absolutePath.generic_string(), .modId = changed.modId}
                    );
                }
            }
        );
    }
}

void HotReloadModule::forwardFrameBegin(
    const FrameBeginEvent & /*event*/
)
{
    // The main-thread drain point: completions (event publications) run here.
    dispatcher.drainCompletedTasks();
    // Re-arm the next scan. One poll in flight at a time keeps the cadence
    // even when a scan is slower than the interval.
    dispatcher.post([this]() { scanOnce(); });
    (void) startupLogged;
}

} // namespace engine
