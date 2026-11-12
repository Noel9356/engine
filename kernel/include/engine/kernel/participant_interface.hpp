// Participant-facing kernel API.
//
// ParticipantContext is handed to every module, plugin and the
// application; it is the only door to capabilities (permission-checked),
// configuration, the event bus and the schedulers. ModuleInterface and
// PluginInterface are the entry points implemented by modules and
// native plugins.

#ifndef ENGINE_KERNEL_PARTICIPANT_INTERFACE_HPP
#define ENGINE_KERNEL_PARTICIPANT_INTERFACE_HPP

#include <engine/kernel/event_bus.hpp>
#include <engine/kernel/main_thread_task_queue.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/kernel/permission_system.hpp>
#include <engine/kernel/profiling.hpp>
#include <engine/kernel/task_scheduler.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

namespace engine
{

// Transparent hash so unordered_set<string> lookups also accept
// string_view.
struct CapabilityNameHash
{
    using is_transparent = void;

    std::size_t operator()(std::string_view text) const noexcept
    {
        return std::hash<std::string_view>{}(text);
    }
};

class ConfigurationStore;

// Everything a script-plugin host (e.g. script_runtime) needs to load a
// type = "script" plugin package.
struct ScriptPluginDescription
{
    std::string           pluginId;
    std::filesystem::path scriptPath;

    // Capability naming the module that will host this script plugin.
    std::string hostCapability;

    std::vector<std::string> declaredCapabilities;

    std::vector<std::string> declaredPermissions;

    std::string version;
};

// Implemented by modules that host script plugins (returned from
// ModuleInterface::scriptPluginHost).
class ScriptPluginHost
{
public:
    virtual ~ScriptPluginHost() = default;

    virtual void hostScriptPlugin(const ScriptPluginDescription &description) = 0;
};

// Thrown by acquireCapability when the permission system blocks access.
class ParticipantAccessError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

// Published after a live reload; changedKeys lists the live keys that
// changed or were removed ([scene] changes need a restart and are
// excluded).
struct ConfigurationReloadedEvent
{
    std::vector<std::string> changedKeys;
};

// Published once per frame by the application's FrameClock; deltaSeconds
// is clamped by max_frame_delta_seconds, elapsedSeconds is the unclamped
// total.
struct EngineFrameEvent
{
    std::uint64_t frameIndex     = 0;
    double        deltaSeconds   = 0.0;
    double        elapsedSeconds = 0.0;
};

// Published 0..max_fixed_update_steps times per frame with the constant
// fixed timestep (seconds) so simulation stays deterministic.
struct EngineFixedUpdateEvent
{
    double deltaSeconds = 0.0;
};

// Published by the application layer whenever a key press is pumped from the
// window event queue. The kernel never interprets keys itself: script runtime
// forwards the event to hosted script plugins as on_key_pressed(keyName).
struct WindowKeyEvent
{
    std::uint32_t keyCode = 0;

    // Stable lowercase key name (for example "escape"); owned by the publisher
    // and only guaranteed to live for the duration of the dispatch.
    const char *keyName = nullptr;
};

// Implemented by EngineKernel: maps capability names to provider-owned
// void* interfaces after checking participant visibility.
class CapabilityResolver
{
public:
    virtual ~CapabilityResolver() = default;

    // Throws std::logic_error when the capability is ambiguous or has no
    // visible provider; tryResolveCapabilityPointer returns nullptr instead.
    virtual void *resolveCapabilityPointer(
        const std::string &requesterName, ParticipantKind requesterKind, const std::string &capabilityName) = 0;

    // Every visible provider of the capability (extension points), in
    // provider registration order; empty when none.
    virtual std::vector<void *> resolveEachCapabilityPointer(
        const std::string &requesterName, ParticipantKind requesterKind, const std::string &capabilityName) = 0;

    // True when exactly one visible provider exists (unambiguous acquire).
    virtual bool hasSingleCapabilityProvider(const std::string &capabilityName, ParticipantKind requesterKind) = 0;

    virtual void *tryResolveCapabilityPointer(
        const std::string &requesterName, ParticipantKind requesterKind, const std::string &capabilityName) = 0;
};

// The handle every participant receives in construct(). References stay
// valid for the kernel's lifetime; calls are main-thread only unless
// documented otherwise.
class ParticipantContext final
{
public:
    ParticipantContext(EventBus &eventBus, CapabilityResolver &capabilityResolver, PermissionSystem &permissionSystem,
        const ConfigurationStore &configuration, std::string participantName, ParticipantKind participantKind,
        const std::unordered_set<std::string, CapabilityNameHash, std::equal_to<>> *demandedCapabilities,
        MainThreadTaskQueue &mainThreadScheduler, TaskScheduler &taskScheduler, PerformanceProfiler &profiler);

    EventBus &eventBus()
    {
        return eventBus_;
    }

    MainThreadTaskQueue &taskQueue()
    {
        return mainThreadTaskQueue_;
    }

    TaskScheduler &taskScheduler()
    {
        return taskScheduler_;
    }

    PerformanceProfiler &profiler()
    {
        return profiler_;
    }

    // Permission-checked capability acquisition: verifies the synthetic
    // permission "capability.<name>", then resolves through the kernel.
    // Throws ParticipantAccessError when the manifest never declared it.
    void *acquireCapability(const std::string &capabilityName) const
    {
        permissionScratch_.assign(capabilityPermissionPrefix);
        permissionScratch_ += capabilityName;
        if (!permissionSystem_.isGranted(participantName_, permissionScratch_))
        {
            const std::string permissionName = permissionScratch_;
            throw ParticipantAccessError("participant '" + participantName_
                                         + "' was blocked by the permission system while acquiring '" + capabilityName
                                         + "': the permission '" + permissionName
                                         + "' was not granted. Declare the requirement in the manifest.");
        }
        return capabilityResolver_.resolveCapabilityPointer(participantName_, participantKind_, capabilityName);
    }

    // Typed overload: CapabilityType must expose a static capabilityName.
    template <typename CapabilityType>
    CapabilityType &acquireCapability() const
    {
        void *capabilityPointer = acquireCapability(std::string(CapabilityType::capabilityName));
        return *static_cast<CapabilityType *>(capabilityPointer);
    }

    // Same permission check, then every visible provider (extension model).
    std::vector<void *> acquireEachCapability(const std::string &capabilityName) const
    {
        permissionScratch_.assign(capabilityPermissionPrefix);
        permissionScratch_ += capabilityName;
        if (!permissionSystem_.isGranted(participantName_, permissionScratch_))
        {
            const std::string permissionName = permissionScratch_;
            throw ParticipantAccessError("participant '" + participantName_
                                         + "' was blocked by the permission system while acquiring '" + capabilityName
                                         + "': the permission '" + permissionName
                                         + "' was not granted. Declare the requirement in the manifest.");
        }
        return capabilityResolver_.resolveEachCapabilityPointer(participantName_, participantKind_, capabilityName);
    }

    template <typename CapabilityType>
    std::vector<CapabilityType *> acquireEachCapability() const
    {
        std::vector<void *> capabilityPointers = acquireEachCapability(std::string(CapabilityType::capabilityName));
        std::vector<CapabilityType *> typed;
        typed.reserve(capabilityPointers.size());
        for (void *capabilityPointer : capabilityPointers)
        {
            typed.push_back(static_cast<CapabilityType *>(capabilityPointer));
        }
        return typed;
    }

    // Non-throwing variant: nullptr when the permission is denied or no
    // provider exists.
    void *tryAcquireCapability(const std::string &capabilityName) const
    {
        permissionScratch_.assign(capabilityPermissionPrefix);
        permissionScratch_ += capabilityName;
        if (!permissionSystem_.isGranted(participantName_, permissionScratch_))
        {
            return nullptr;
        }
        return capabilityResolver_.tryResolveCapabilityPointer(participantName_, participantKind_, capabilityName);
    }

    template <typename CapabilityType>
    CapabilityType *tryAcquireCapability() const
    {
        void *capabilityPointer = tryAcquireCapability(std::string(CapabilityType::capabilityName));
        return capabilityPointer != nullptr ? static_cast<CapabilityType *>(capabilityPointer) : nullptr;
    }

    // True when any active participant's demand closure includes this
    // capability (lets providers skip work nobody consumes).
    bool isCapabilityDemanded(std::string_view capabilityName) const
    {
        return demandedCapabilities_ != nullptr
            && demandedCapabilities_->find(capabilityName) != demandedCapabilities_->end();
    }

    // True when the permission is granted and exactly one provider exists.
    bool isCapabilityAvailable(const std::string &capabilityName) const
    {
        permissionScratch_.assign(capabilityPermissionPrefix);
        permissionScratch_ += capabilityName;
        if (!permissionSystem_.isGranted(participantName_, permissionScratch_))
        {
            return false;
        }
        return capabilityResolver_.hasSingleCapabilityProvider(capabilityName, participantKind_);
    }

    // Reads the participant's own [section] (keyed by participant name) in
    // the merged configuration; modules and plugins get namespaced config for
    // free.
    std::optional<std::string>  configurationString(const std::string &key) const;
    std::optional<std::int64_t> configurationInteger(const std::string &key) const;
    std::optional<bool>         configurationBoolean(const std::string &key) const;
    std::optional<double>       configurationFloating(const std::string &key) const;

    // Explicit-section variants (cross-section reads).
    std::optional<std::string> configurationStringInSection(const std::string &section, const std::string &path) const;

    std::vector<std::string> configurationStringArrayInSection(
        const std::string &section, const std::string &path) const;

    std::optional<std::int64_t> configurationIntegerInSection(
        const std::string &section, const std::string &path) const;

    std::optional<bool> configurationBooleanInSection(const std::string &section, const std::string &path) const;

    std::optional<double> configurationFloatingInSection(const std::string &section, const std::string &path) const;

    std::vector<double> configurationFloatingArrayInSection(const std::string &section, const std::string &path) const;

    std::size_t configurationArraySizeInSection(const std::string &section, const std::string &path) const;

    std::vector<std::string> configurationTableKeysInSection(const std::string &section, const std::string &path) const;

private:
    // demandedCapabilities_ is shared with the CapabilityProviderIndex demand
    // closure; permissionScratch_ avoids an allocation per permission check.
    EventBus                 &eventBus_;
    CapabilityResolver       &capabilityResolver_;
    PermissionSystem         &permissionSystem_;
    const ConfigurationStore &configuration_;
    std::string               participantName_;

    ParticipantKind participantKind_ = ParticipantKind::Application;
    const std::unordered_set<std::string, CapabilityNameHash, std::equal_to<>> *demandedCapabilities_ = nullptr;
    MainThreadTaskQueue                                                        &mainThreadTaskQueue_;
    TaskScheduler                                                              &taskScheduler_;
    PerformanceProfiler                                                        &profiler_;

    mutable std::string permissionScratch_;
};

// Native module entry point: one instance per .so, created via
// engineQueryModuleEntryTable (see module_api.hpp).
class ModuleInterface
{
public:
    virtual ~ModuleInterface() = default;

    // Must agree with the module descriptor (name, version and capability
    // sets are cross-checked at load).
    virtual const ParticipantManifest &manifest() const = 0;

    virtual void construct(ParticipantContext &context) = 0;
    virtual void start()                                = 0;
    virtual void stop()                                 = 0;

    // Raw interface for one provided capability; nullptr means the module
    // declines to provide it right now (the kernel reports the mismatch).
    virtual void *capabilityPointerFor(const std::string &capabilityName) = 0;

    // Non-null when the module hosts script plugins.
    virtual ScriptPluginHost *scriptPluginHost()
    {
        return nullptr;
    }
};

// Native plugin entry point; instances are created/destroyed through
// the plugin .so's entry table (see plugin_api.hpp).
class PluginInterface
{
public:
    virtual ~PluginInterface() = default;

    virtual const ParticipantManifest &manifest() const = 0;

    virtual void construct(ParticipantContext &context) = 0;
    virtual void start()                                = 0;
    virtual void stop()                                 = 0;

    virtual void *capabilityPointerFor(const std::string &capabilityName) = 0;
};

} // namespace engine

#endif
