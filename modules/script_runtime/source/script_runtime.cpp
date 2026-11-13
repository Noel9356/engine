// script_runtime module implementation.
//
// Provides the "script_runtime" capability: sandboxed Lua 5.5 runtimes with
// memory and instruction budgets, plus hosting for type = "script" plugins
// (see ScriptPluginHost). When ENGINE_SCRIPT_RUNTIME_BACKEND_LUA is not
// defined every capability entry fails gracefully and script plugins cannot be
// hosted.
//
// The Lua-facing primitives (custom allocator, instruction-count hook,
// native-function bridge, library whitelist, global-path helpers) live in
// lua_sandbox.hpp / lua_sandbox.cpp. This file owns the runtime map, the
// capability binding, the plugin host, and the engine event forwarding.

#include <engine/kernel/engine_kernel.hpp>
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/script_runtime/capabilities.hpp>
#include <engine/util/hardware_info.hpp>

#include "lua_sandbox.hpp"

#include <algorithm>
#include <atomic>
#include <climits>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#if defined(ENGINE_SCRIPT_RUNTIME_BACKEND_LUA)
extern "C"
{
#include <lua.h>
}
#endif

namespace engine::script_runtime
{
namespace
{

using sandbox::copyText;
using sandbox::RuntimeInitializerRecord;
using sandbox::setError;

#if defined(ENGINE_SCRIPT_RUNTIME_BACKEND_LUA)
using sandbox::classifyError;
using sandbox::instructionHook;
using sandbox::invokeNativeFunction;
using sandbox::luaAllocator;
using sandbox::luaHashSeed;
using sandbox::NativeFunctionRecord;
using sandbox::openLibraries;
using sandbox::pushGlobalPath;
using sandbox::resetLastError;
using sandbox::RuntimeRecord;
using sandbox::setExecutionBudget;
using sandbox::setGlobalPath;
using sandbox::stackError;
#endif

class ScriptRuntimeModule final : public ModuleInterface, public ScriptPluginHost
{
public:
    ScriptRuntimeModule()
        : manifest_(makeManifest())
    {
        assignCapabilityPointers();
    }
    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    void construct(ParticipantContext &context) override
    {
        context_ = &context;
        startupSubscription_
            = context.eventBus()
                  .channel<EngineStartupCompletedEvent>()
                  .subscribe<ScriptRuntimeModule, &ScriptRuntimeModule::onEngineStartupCompleted>(this);
        frameSubscription_       = context.eventBus()
                                       .channel<EngineFrameEvent>()
                                       .subscribe<ScriptRuntimeModule, &ScriptRuntimeModule::onEngineFrame>(this);
        fixedUpdateSubscription_ = context.eventBus()
                                       .channel<EngineFixedUpdateEvent>()
                                       .subscribe<ScriptRuntimeModule, &ScriptRuntimeModule::onEngineFixedUpdate>(this);
        shutdownSubscription_
            = context.eventBus()
                  .channel<EngineShutdownStartedEvent>()
                  .subscribe<ScriptRuntimeModule, &ScriptRuntimeModule::onEngineShutdownStarted>(this);
        keySubscription_ = context.eventBus()
                               .channel<WindowKeyEvent>()
                               .subscribe<ScriptRuntimeModule, &ScriptRuntimeModule::onWindowKey>(this);

        // Non-memory keys (instruction budget, script size, runtime/native
        // caps) are always read from configuration: they bound logic rather
        // than memory and stay effective regardless of auto_config.
        applyNonMemoryConfiguration(context);

        // Memory ceilings are either derived from installed RAM (auto_config)
        // or read from the manual keys.
        if (context.configurationBooleanInSection("script_runtime", "auto_config").value_or(false))
        {
            applyHardwareDerivedMemoryLimits();
        }
        else
        {
            applyManualMemoryConfiguration(context);
        }
    }

    void start() override
    {}
    void stop() override
    {
        if (context_ != nullptr)
        {
            context_->eventBus().channel<EngineStartupCompletedEvent>().unsubscribe(startupSubscription_);
            context_->eventBus().channel<EngineFrameEvent>().unsubscribe(frameSubscription_);
            context_->eventBus().channel<EngineFixedUpdateEvent>().unsubscribe(fixedUpdateSubscription_);
            context_->eventBus().channel<EngineShutdownStartedEvent>().unsubscribe(shutdownSubscription_);
            context_->eventBus().channel<WindowKeyEvent>().unsubscribe(keySubscription_);
            context_ = nullptr;
        }
        {
            std::lock_guard lock(hostedMutex_);
            hostedPlugins_.clear();
        }
        {
            std::lock_guard lock(initializersMutex_);
            initializers_.clear();
        }
#if defined(ENGINE_SCRIPT_RUNTIME_BACKEND_LUA)
        std::lock_guard lock(runtimesMutex_);
        runtimes_.clear();
#endif
        dispatchBatch_.clear();
        dispatchBatch_.shrink_to_fit();
    }
    void *capabilityPointerFor(const std::string &name) override
    {
        return name == ScriptRuntimeCapability::capabilityName ? &capability_ : nullptr;
    }

    ScriptPluginHost *scriptPluginHost() override
    {
        return this;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "script_runtime",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"script_runtime"}};
    }

    // --- configuration ---------------------------------------------------------

    void applyNonMemoryConfiguration(ParticipantContext &context)
    {
        if (const auto value = context.configurationIntegerInSection("script_runtime", "instruction_limit");
            value && *value >= 1000)
        {
            defaultDescription_.instructionLimit = static_cast<std::uint64_t>(*value);
        }
        if (const auto value = context.configurationIntegerInSection("script_runtime", "maximum_script_size_mb");
            value && *value >= 1 && *value <= 256)
        {
            defaultDescription_.maximumScriptBytes = static_cast<std::uint64_t>(*value) * 1024ull * 1024ull;
        }
        if (const auto value = context.configurationIntegerInSection("script_runtime", "maximum_runtimes");
            value && *value >= 1 && *value <= 4096)
        {
            maximumRuntimes_ = static_cast<std::size_t>(*value);
        }
        if (const auto value = context.configurationIntegerInSection("script_runtime", "maximum_native_functions");
            value && *value >= 1 && *value <= 4096)
        {
            maximumNativeFunctions_ = static_cast<std::size_t>(*value);
        }
        if (const auto value = context.configurationIntegerInSection("script_runtime", "maximum_native_arguments");
            value && *value >= 1 && *value <= 32)
        {
            maximumNativeArguments_ = static_cast<std::uint32_t>(*value);
        }
        if (const auto value = context.configurationIntegerInSection("script_runtime", "maximum_native_results");
            value && *value >= 1 && *value <= 16)
        {
            maximumNativeResults_ = static_cast<std::uint32_t>(*value);
        }
        if (const auto value = context.configurationIntegerInSection("script_runtime", "maximum_runtime_initializers");
            value && *value >= 1 && *value <= 4096)
        {
            maximumRuntimeInitializers_ = static_cast<std::size_t>(*value);
        }
        if (const auto value = context.configurationIntegerInSection("script_runtime", "instruction_hook_granularity");
            value && *value >= 1 && *value <= 100000)
        {
            instructionHookGranularity_ = static_cast<std::uint32_t>(*value);
        }
        if (const auto value = context.configurationIntegerInSection("script_runtime", "wall_clock_limit_milliseconds");
            value && *value >= 0)
        {
            defaultDescription_.wallClockLimitMilliseconds = static_cast<std::uint64_t>(*value);
        }
        // Optional: override the default library mask. Bits outside the
        // allowed set are silently dropped at runtime-create time.
        if (const auto value = context.configurationIntegerInSection("script_runtime", "default_enabled_libraries");
            value && *value >= 0)
        {
            constexpr std::uint32_t allowedLibraries = ScriptLibraryMath | ScriptLibraryString | ScriptLibraryTable
                                                     | ScriptLibraryUtf8 | ScriptLibraryCoroutine;
            defaultDescription_.enabledLibraries     = static_cast<std::uint32_t>(*value) & allowedLibraries;
        }
    }

    void applyManualMemoryConfiguration(ParticipantContext &context)
    {
        if (const auto value = context.configurationIntegerInSection("script_runtime", "memory_limit_mb");
            value && *value >= 1 && *value <= 4096)
        {
            defaultDescription_.memoryLimitBytes = static_cast<std::uint64_t>(*value) * 1024ull * 1024ull;
        }
        if (const auto value = context.configurationIntegerInSection("script_runtime", "maximum_total_memory_mb");
            value && *value >= 1 && *value <= 65536)
        {
            maximumTotalMemoryBytes_ = static_cast<std::uint64_t>(*value) * 1024ull * 1024ull;
        }
    }

    // auto_config = true: per-runtime memory gets ~1.6% of installed RAM
    // (16..256 MiB), the module-wide total ~12.5% (128..4096 MiB). Unknown
    // RAM keeps the built-in ceilings (the conservative fallback). Every
    // derived value stays inside the manual ranges, so switching
    // auto_config off and hand-writing the derived numbers reproduces the
    // same behavior exactly.
    void applyHardwareDerivedMemoryLimits()
    {
        constexpr std::uint64_t mib = 1024ull * 1024ull;
        constexpr std::uint64_t gib = 1024ull * mib;
        const std::uint64_t     ram = util::physicalMemoryBytes();
        if (ram == 0u)
        {
            std::fprintf(stderr,
                "script_runtime: auto_config is on; installed RAM unknown, keeping built-in limits; "
                "other tuning keys in [script_runtime] are ignored\n");
            return;
        }
        defaultDescription_.memoryLimitBytes = std::clamp<std::uint64_t>(ram / 64u, 16u * mib, 256u * mib);
        maximumTotalMemoryBytes_             = std::clamp<std::uint64_t>(ram / 8u, 128u * mib, 4096u * mib);
        std::fprintf(stderr,
            "script_runtime: auto_config is on; memory limits derived from %.1f GiB RAM "
            "(memory_limit_mb=%llu, maximum_total_memory_mb=%llu); "
            "other tuning keys in [script_runtime] are ignored\n",
            static_cast<double>(ram) / static_cast<double>(gib),
            (unsigned long long)(defaultDescription_.memoryLimitBytes / mib),
            (unsigned long long)(maximumTotalMemoryBytes_ / mib));
    }

    // --- script plugin hosting ------------------------------------------------
    //
    // The kernel hands every type = "script" plugin to this module during the
    // construct phase (see ScriptPluginHost). Hosting a plugin means running its
    // entry file in a fresh sandboxed Lua runtime and forwarding engine events
    // to the callback functions the script chose to define:
    //
    //   on_construct()                      after the entry file loaded
    //   on_start()                          EngineStartupCompletedEvent
    //   on_frame(delta, elapsed)            EngineFrameEvent
    //   on_fixed_update(delta)              EngineFixedUpdateEvent
    //   on_key_pressed(name)                WindowKeyEvent
    //   on_shutdown()                       EngineShutdownStartedEvent

    struct HostedPlugin
    {
        std::string         pluginId;
        ScriptRuntimeHandle runtime = invalidScriptRuntime;
    };

    ScriptRuntimeHandle hostedRuntimeImpl(const char *pluginId)
    {
        constexpr std::size_t maximumPluginIdBytes = 128;
        if (pluginId == nullptr)
            return invalidScriptRuntime;
        const auto *terminator = static_cast<const char *>(std::memchr(pluginId, '\0', maximumPluginIdBytes + 1u));
        if (terminator == nullptr || terminator == pluginId)
            return invalidScriptRuntime;
        const std::string_view requestedId(pluginId, static_cast<std::size_t>(terminator - pluginId));
        std::lock_guard        lock(hostedMutex_);
        for (const HostedPlugin &plugin : hostedPlugins_)
            if (plugin.pluginId == requestedId)
                return plugin.runtime;
        return invalidScriptRuntime;
    }

#if defined(ENGINE_SCRIPT_RUNTIME_BACKEND_LUA)

    // Calls a global function if the plugin defined one. Absent callbacks are
    // the normal opt-in case, not errors. Runtime failures are printed once per
    // distinct message so a broken on_frame cannot flood stderr.
    void callOptionalGlobal(ScriptRuntimeHandle handle, const char *name, std::initializer_list<double> numbers = {},
        const char *text = nullptr)
    {
        auto runtime = findRuntime(handle);
        if (!runtime)
            return;
        std::lock_guard lock(runtime->mutex);
        lua_State      *state = runtime->state;
        // pushGlobalPath restores the stack on failure; only pop when it
        // actually pushed something.
        const bool found = pushGlobalPath(state, name);
        if (!found || !lua_isfunction(state, -1))
        {
            if (found)
                lua_pop(state, 1);
            return;
        }
        for (double number : numbers)
            lua_pushnumber(state, number);
        if (text != nullptr)
            lua_pushstring(state, text);
        resetLastError(*runtime);
        setExecutionBudget(*runtime);
        const int argumentCount = static_cast<int>(numbers.size()) + (text != nullptr ? 1 : 0);
        const int result        = lua_pcall(state, argumentCount, 0, 0);
        lua_sethook(state, nullptr, 0, 0);
        if (result != LUA_OK)
        {
            const std::string_view message = stackError(state);
            if (runtime->lastCallbackError != message)
            {
                try
                {
                    runtime->lastCallbackError.assign(message);
                }
                catch (...)
                {}
                std::fprintf(stderr,
                    "script_runtime: callback '%s' failed: %.*s\n",
                    name,
                    static_cast<int>(std::min<std::size_t>(message.size(), static_cast<std::size_t>(INT_MAX))),
                    message.data());
            }
            lua_pop(state, 1);
        }
    }
#else
    void callOptionalGlobal(
        ScriptRuntimeHandle, const char *, std::initializer_list<double> = {}, const char * = nullptr)
    {}
#endif

    void dispatchToHostedPlugins(
        const char *callback, std::initializer_list<double> numbers = {}, const char *text = nullptr)
    {
        // Reuse a per-module buffer so the per-frame dispatch path does not
        // allocate. The vector is cleared (not freed) at the start of each
        // call so the capacity stabilises after the first frame.
        dispatchBatch_.clear();
        try
        {
            std::lock_guard lock(hostedMutex_);
            dispatchBatch_.reserve(hostedPlugins_.size());
            for (const HostedPlugin &plugin : hostedPlugins_)
                dispatchBatch_.push_back(plugin.runtime);
        }
        catch (...)
        {
            return;
        }
        for (const ScriptRuntimeHandle runtime : dispatchBatch_)
            callOptionalGlobal(runtime, callback, numbers, text);
    }

    void onEngineStartupCompleted(const EngineStartupCompletedEvent &)
    {
        dispatchToHostedPlugins("on_start");
    }

    void onEngineFrame(const EngineFrameEvent &event)
    {
        dispatchToHostedPlugins("on_frame", {event.deltaSeconds, event.elapsedSeconds});
    }

    void onEngineFixedUpdate(const EngineFixedUpdateEvent &event)
    {
        dispatchToHostedPlugins("on_fixed_update", {event.deltaSeconds});
    }

    void onEngineShutdownStarted(const EngineShutdownStartedEvent &)
    {
        dispatchToHostedPlugins("on_shutdown");
    }

    void onWindowKey(const WindowKeyEvent &event)
    {
        dispatchToHostedPlugins("on_key_pressed", {}, event.keyName);
    }

    void hostScriptPlugin(const ScriptPluginDescription &description) override
    {
#if defined(ENGINE_SCRIPT_RUNTIME_BACKEND_LUA)
        ScriptError               error = ScriptError::None;
        std::array<char, 512>     errorBuffer{};
        const ScriptRuntimeHandle runtime = createRuntimeImpl(nullptr, &error);
        if (runtime == invalidScriptRuntime)
        {
            throw std::runtime_error("script plugin '" + description.pluginId
                                     + "' could not create a Lua runtime (script_runtime error "
                                     + std::to_string(static_cast<std::uint32_t>(error)) + ")");
        }
        setGlobalStringImpl(runtime, "engine_plugin_id", description.pluginId.c_str());
        setGlobalStringImpl(runtime, "engine_plugin_version", description.version.c_str());
        setGlobalStringImpl(runtime, "engine_plugin_path", description.scriptPath.string().c_str());
        if (!executeFileImpl(runtime,
                description.scriptPath.string().c_str(),
                &error,
                errorBuffer.data(),
                static_cast<std::uint32_t>(errorBuffer.size()),
                nullptr))
        {
            destroyRuntimeImpl(runtime);
            throw std::runtime_error("script plugin '" + description.pluginId + "' failed to run '"
                                     + description.scriptPath.string() + "': " + std::string(errorBuffer.data()));
        }
        callOptionalGlobal(runtime, "on_construct");
        try
        {
            std::lock_guard lock(hostedMutex_);
            hostedPlugins_.push_back(HostedPlugin{.pluginId = description.pluginId, .runtime = runtime});
        }
        catch (...)
        {
            destroyRuntimeImpl(runtime);
            throw std::runtime_error("script plugin '" + description.pluginId + "' could not be registered");
        }
#else
        (void)description;
        throw std::runtime_error("script_runtime: the Lua backend is disabled; script plugins cannot be hosted");
#endif
    }

    // --- capability implementation -------------------------------------------

    void assignCapabilityPointers()
    {
        capability_.self = this;
#define ENGINE_SCRIPT_RUNTIME_BIND(field, method)                                                                      \
    capability_.field = [](void *self, auto... args) -> decltype(auto)                                                 \
    { return static_cast<ScriptRuntimeModule *>(self)->method(args...); }
        ENGINE_SCRIPT_RUNTIME_BIND(createRuntime, createRuntimeImpl);
        ENGINE_SCRIPT_RUNTIME_BIND(destroyRuntime, destroyRuntimeImpl);
        ENGINE_SCRIPT_RUNTIME_BIND(runtimeInfo, runtimeInfoImpl);
        ENGINE_SCRIPT_RUNTIME_BIND(executeString, executeStringImpl);
        ENGINE_SCRIPT_RUNTIME_BIND(executeFile, executeFileImpl);
        ENGINE_SCRIPT_RUNTIME_BIND(callFunction, callFunctionImpl);
        ENGINE_SCRIPT_RUNTIME_BIND(registerNativeFunction, registerNativeFunctionImpl);
        ENGINE_SCRIPT_RUNTIME_BIND(setGlobalNumber, setGlobalNumberImpl);
        ENGINE_SCRIPT_RUNTIME_BIND(getGlobalNumber, getGlobalNumberImpl);
        ENGINE_SCRIPT_RUNTIME_BIND(setGlobalString, setGlobalStringImpl);
        ENGINE_SCRIPT_RUNTIME_BIND(getGlobalString, getGlobalStringImpl);
        ENGINE_SCRIPT_RUNTIME_BIND(collectGarbage, collectGarbageImpl);
        ENGINE_SCRIPT_RUNTIME_BIND(hostedRuntime, hostedRuntimeImpl);
        ENGINE_SCRIPT_RUNTIME_BIND(registerRuntimeInitializer, registerRuntimeInitializerImpl);
        ENGINE_SCRIPT_RUNTIME_BIND(unregisterRuntimeInitializer, unregisterRuntimeInitializerImpl);
#undef ENGINE_SCRIPT_RUNTIME_BIND
    }

#if defined(ENGINE_SCRIPT_RUNTIME_BACKEND_LUA)
    ScriptRuntimeInitializerHandle registerRuntimeInitializerImpl(ScriptRuntimeInitializer initializer, void *userData)
    {
        if (initializer == nullptr)
            return invalidScriptRuntimeInitializer;

        std::lock_guard initializerLock(initializersMutex_);
        if (initializers_.size() >= maximumRuntimeInitializers_ || nextInitializer_ == invalidScriptRuntimeInitializer
            || nextInitializer_ == std::numeric_limits<ScriptRuntimeInitializerHandle>::max())
            return invalidScriptRuntimeInitializer;

        const ScriptRuntimeInitializerHandle handle = nextInitializer_++;
        try
        {
            initializers_.push_back(RuntimeInitializerRecord{handle, initializer, userData});
        }
        catch (...)
        {
            return invalidScriptRuntimeInitializer;
        }

        // Apply newly registered bridges to existing runtimes as well. The
        // initializer mutex serializes this with createRuntimeImpl so a
        // runtime cannot miss an initializer or receive it twice. A runtime
        // that has been destroyed between collecting the handle and invoking
        // the initializer is skipped (findRuntime returns null inside the
        // callback) rather than failing the whole registration.
        std::vector<ScriptRuntimeHandle> handles;
        try
        {
            std::lock_guard runtimeLock(runtimesMutex_);
            handles.reserve(runtimes_.size());
            for (const auto &[runtimeHandle, runtime] : runtimes_)
            {
                (void)runtime;
                handles.push_back(runtimeHandle);
            }
        }
        catch (...)
        {
            initializers_.pop_back();
            return invalidScriptRuntimeInitializer;
        }

        for (const ScriptRuntimeHandle runtime : handles)
        {
            bool initialized = false;
            try
            {
                initialized = initializer(userData, runtime);
            }
            catch (...)
            {
                initialized = false;
            }
            if (!initialized)
            {
                // The runtime may have been destroyed concurrently; verify
                // before rolling back so a transient target does not undo an
                // otherwise valid registration.
                std::lock_guard runtimeLock(runtimesMutex_);
                if (runtimes_.find(runtime) != runtimes_.end())
                {
                    initializers_.pop_back();
                    return invalidScriptRuntimeInitializer;
                }
            }
        }
        return handle;
    }

    bool unregisterRuntimeInitializerImpl(ScriptRuntimeInitializerHandle handle)
    {
        if (handle == invalidScriptRuntimeInitializer)
            return false;
        std::lock_guard lock(initializersMutex_);
        const auto      found = std::find_if(initializers_.begin(),
            initializers_.end(),
            [handle](const RuntimeInitializerRecord &record) { return record.handle == handle; });
        if (found == initializers_.end())
            return false;
        initializers_.erase(found);
        return true;
    }

    std::shared_ptr<RuntimeRecord> findRuntime(ScriptRuntimeHandle handle)
    {
        std::lock_guard lock(runtimesMutex_);
        const auto      found = runtimes_.find(handle);
        return found == runtimes_.end() ? nullptr : found->second;
    }

    ScriptRuntimeHandle createRuntimeImpl(const ScriptRuntimeDescription *description, ScriptError *error)
    {
        setError(error, ScriptError::None);
        ScriptRuntimeDescription settings = description != nullptr ? *description : defaultDescription_;
        constexpr std::uint32_t  allowedLibraries
            = ScriptLibraryMath | ScriptLibraryString | ScriptLibraryTable | ScriptLibraryUtf8 | ScriptLibraryCoroutine;
        if (settings.memoryLimitBytes < 1024u * 1024u || settings.memoryLimitBytes > (1ull << 40u)
            || settings.instructionLimit < 1000u || settings.maximumScriptBytes == 0
            || settings.maximumScriptBytes > settings.memoryLimitBytes
            || settings.maximumScriptBytes >= static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())
            || settings.maximumScriptBytes > static_cast<std::uint64_t>(std::numeric_limits<std::streamsize>::max())
            || (settings.enabledLibraries & ~allowedLibraries) != 0)
        {
            setError(error, ScriptError::InvalidArgument);
            return invalidScriptRuntime;
        }
        std::lock_guard                initializerLock(initializersMutex_);
        std::shared_ptr<RuntimeRecord> runtime;
        try
        {
            runtime = std::make_shared<RuntimeRecord>(settings);
        }
        catch (...)
        {
            setError(error, ScriptError::MemoryLimitExceeded);
            return invalidScriptRuntime;
        }
        runtime->globalMemoryUsed       = &totalMemoryUsed_;
        runtime->globalMemoryLimitBytes = maximumTotalMemoryBytes_;
        runtime->hookGranularity        = instructionHookGranularity_;
        runtime->state                  = lua_newstate(luaAllocator, runtime.get(), luaHashSeed());
        if (runtime->state == nullptr)
        {
            setError(error, ScriptError::MemoryLimitExceeded);
            return invalidScriptRuntime;
        }
        *static_cast<RuntimeRecord **>(lua_getextraspace(runtime->state)) = runtime.get();
        openLibraries(runtime->state, settings.enabledLibraries);

        ScriptRuntimeHandle handle = invalidScriptRuntime;
        {
            std::lock_guard runtimeLock(runtimesMutex_);
            if (runtimes_.size() >= maximumRuntimes_ || nextHandle_ == invalidScriptRuntime
                || nextHandle_ == std::numeric_limits<ScriptRuntimeHandle>::max())
            {
                setError(error, ScriptError::CapacityExceeded);
                return invalidScriptRuntime;
            }
            handle = nextHandle_++;
            try
            {
                runtimes_.emplace(handle, runtime);
            }
            catch (...)
            {
                setError(error, ScriptError::MemoryLimitExceeded);
                return invalidScriptRuntime;
            }
        }

        for (const RuntimeInitializerRecord &initializer : initializers_)
        {
            bool initialized = false;
            try
            {
                initialized = initializer.callback != nullptr && initializer.callback(initializer.userData, handle);
            }
            catch (...)
            {
                initialized = false;
            }
            if (!initialized)
            {
                std::lock_guard runtimeLock(runtimesMutex_);
                runtimes_.erase(handle);
                setError(error, ScriptError::RuntimeFailed);
                return invalidScriptRuntime;
            }
        }
        return handle;
    }

    void destroyRuntimeImpl(ScriptRuntimeHandle handle)
    {
        std::lock_guard lock(runtimesMutex_);
        runtimes_.erase(handle);
    }

    bool runtimeInfoImpl(ScriptRuntimeHandle handle, ScriptRuntimeInfo *output)
    {
        auto runtime = findRuntime(handle);
        if (!runtime || output == nullptr)
            return false;
        std::lock_guard lock(runtime->mutex);
        output->memoryUsedBytes    = runtime->memoryUsed;
        output->memoryLimitBytes   = runtime->description.memoryLimitBytes;
        output->instructionLimit   = runtime->description.instructionLimit;
        output->maximumScriptBytes = runtime->description.maximumScriptBytes;
        return true;
    }

    bool executeStringImpl(ScriptRuntimeHandle handle, const char *source, const char *chunkName, ScriptError *error,
        char *errorText, std::uint32_t errorCapacity, std::uint32_t *errorRequired)
    {
        setError(error, ScriptError::None);
        if (source == nullptr)
        {
            setError(error, ScriptError::InvalidArgument);
            return copyText("source is null", errorText, errorCapacity, errorRequired);
        }
        auto runtime = findRuntime(handle);
        if (!runtime)
        {
            setError(error, ScriptError::InvalidHandle);
            return copyText("invalid script runtime", errorText, errorCapacity, errorRequired);
        }
        std::lock_guard   lock(runtime->mutex);
        const std::size_t maximumBytes = static_cast<std::size_t>(runtime->description.maximumScriptBytes);
        const auto       *terminator   = static_cast<const char *>(std::memchr(source, '\0', maximumBytes + 1u));
        if (terminator == nullptr)
        {
            setError(error, ScriptError::CapacityExceeded);
            return copyText("script exceeds configured size limit", errorText, errorCapacity, errorRequired);
        }
        const std::size_t     length                = static_cast<std::size_t>(terminator - source);
        constexpr std::size_t maximumChunkNameBytes = 16384;
        if (chunkName != nullptr && std::memchr(chunkName, '\0', maximumChunkNameBytes + 1u) == nullptr)
        {
            setError(error, ScriptError::InvalidArgument);
            return copyText("chunk name exceeds configured size limit", errorText, errorCapacity, errorRequired);
        }
        lua_State *state = runtime->state;
        resetLastError(*runtime);
        setExecutionBudget(*runtime);
        const int loaded = luaL_loadbufferx(state, source, length, chunkName != nullptr ? chunkName : "=(engine)", "t");
        if (loaded != LUA_OK)
        {
            const std::string_view message = stackError(state);
            const ScriptError      code    = classifyError(*runtime, ScriptError::CompilationFailed);
            setError(error, code);
            lua_sethook(state, nullptr, 0, 0);
            const bool copied = copyText(message, errorText, errorCapacity, errorRequired);
            lua_pop(state, 1);
            return copied;
        }
        const int executed = lua_pcall(state, 0, 0, 0);
        lua_sethook(state, nullptr, 0, 0);
        if (executed != LUA_OK)
        {
            const std::string_view message = stackError(state);
            const ScriptError      code    = classifyError(*runtime, ScriptError::RuntimeFailed);
            setError(error, code);
            const bool copied = copyText(message, errorText, errorCapacity, errorRequired);
            lua_pop(state, 1);
            return copied;
        }
        if (errorRequired != nullptr)
            *errorRequired = 1;
        if (errorText != nullptr && errorCapacity != 0)
            errorText[0] = '\0';
        return true;
    }

    bool executeFileImpl(ScriptRuntimeHandle handle, const char *filePath, ScriptError *error, char *errorText,
        std::uint32_t errorCapacity, std::uint32_t *errorRequired)
    {
        setError(error, ScriptError::None);
        constexpr std::size_t maximumPathBytes = 16384;
        if (filePath == nullptr)
        {
            setError(error, ScriptError::InvalidArgument);
            return copyText("file path is empty", errorText, errorCapacity, errorRequired);
        }
        const auto *pathTerminator = static_cast<const char *>(std::memchr(filePath, '\0', maximumPathBytes + 1u));
        if (pathTerminator == nullptr || pathTerminator == filePath)
        {
            setError(error, ScriptError::InvalidArgument);
            return copyText("file path is empty or too long", errorText, errorCapacity, errorRequired);
        }
        auto runtime = findRuntime(handle);
        if (!runtime)
        {
            setError(error, ScriptError::InvalidHandle);
            return copyText("invalid script runtime", errorText, errorCapacity, errorRequired);
        }
        std::error_code filesystemError;
        const auto      fileSize = std::filesystem::file_size(filePath, filesystemError);
        if (filesystemError)
        {
            setError(error, ScriptError::IoFailed);
            return copyText("unable to read script file size", errorText, errorCapacity, errorRequired);
        }
        if (fileSize > runtime->description.maximumScriptBytes
            || fileSize > static_cast<std::uintmax_t>(std::numeric_limits<std::size_t>::max())
            || fileSize > static_cast<std::uintmax_t>(std::numeric_limits<std::streamsize>::max()))
        {
            setError(error, ScriptError::CapacityExceeded);
            return copyText("script exceeds configured size limit", errorText, errorCapacity, errorRequired);
        }
        std::ifstream stream(std::filesystem::path(filePath), std::ios::binary);
        if (!stream)
        {
            setError(error, ScriptError::IoFailed);
            return copyText("unable to open script file", errorText, errorCapacity, errorRequired);
        }
        std::string source(static_cast<std::size_t>(fileSize), '\0');
        if (!source.empty())
        {
            stream.read(source.data(), static_cast<std::streamsize>(source.size()));
            if (stream.gcount() != static_cast<std::streamsize>(source.size()))
            {
                setError(error, ScriptError::IoFailed);
                return copyText("unable to read complete script file", errorText, errorCapacity, errorRequired);
            }
        }
        char trailingByte = 0;
        if (stream.get(trailingByte))
        {
            setError(error, ScriptError::IoFailed);
            return copyText("script file changed while being read", errorText, errorCapacity, errorRequired);
        }
        if (!stream.eof())
        {
            setError(error, ScriptError::IoFailed);
            return copyText("unable to finish reading script file", errorText, errorCapacity, errorRequired);
        }
        if (source.find('\0') != std::string::npos)
        {
            setError(error, ScriptError::CompilationFailed);
            return copyText("script file contains a null byte", errorText, errorCapacity, errorRequired);
        }
        return executeStringImpl(handle, source.c_str(), filePath, error, errorText, errorCapacity, errorRequired);
    }

    bool callFunctionImpl(ScriptRuntimeHandle handle, const char *functionName, ScriptError *error, char *errorText,
        std::uint32_t errorCapacity, std::uint32_t *errorRequired)
    {
        setError(error, ScriptError::None);
        if (functionName == nullptr || functionName[0] == '\0')
        {
            setError(error, ScriptError::InvalidArgument);
            return copyText("function name is empty", errorText, errorCapacity, errorRequired);
        }
        auto runtime = findRuntime(handle);
        if (!runtime)
        {
            setError(error, ScriptError::InvalidHandle);
            return copyText("invalid script runtime", errorText, errorCapacity, errorRequired);
        }
        std::lock_guard lock(runtime->mutex);
        lua_State      *state = runtime->state;
        if (!pushGlobalPath(state, functionName))
        {
            setError(error, ScriptError::TypeMismatch);
            return copyText("global value is not a function", errorText, errorCapacity, errorRequired);
        }
        if (!lua_isfunction(state, -1))
        {
            lua_pop(state, 1);
            setError(error, ScriptError::TypeMismatch);
            return copyText("global value is not a function", errorText, errorCapacity, errorRequired);
        }
        resetLastError(*runtime);
        setExecutionBudget(*runtime);
        const int result = lua_pcall(state, 0, 0, 0);
        lua_sethook(state, nullptr, 0, 0);
        if (result != LUA_OK)
        {
            const std::string_view message = stackError(state);
            const ScriptError      code    = classifyError(*runtime, ScriptError::RuntimeFailed);
            setError(error, code);
            const bool copied = copyText(message, errorText, errorCapacity, errorRequired);
            lua_pop(state, 1);
            return copied;
        }
        if (errorRequired != nullptr)
            *errorRequired = 1;
        if (errorText != nullptr && errorCapacity != 0)
            errorText[0] = '\0';
        return true;
    }

    bool registerNativeFunctionImpl(
        ScriptRuntimeHandle handle, const char *name, ScriptNativeFunction callback, void *userData)
    {
        auto runtime = findRuntime(handle);
        if (!runtime || name == nullptr || name[0] == '\0' || callback == nullptr)
            return false;
        std::lock_guard lock(runtime->mutex);
        if (runtime->nativeFunctions.size() >= maximumNativeFunctions_)
            return false;
        std::unique_ptr<NativeFunctionRecord> function;
        try
        {
            function = std::make_unique<NativeFunctionRecord>();
        }
        catch (...)
        {
            return false;
        }
        function->callback                    = callback;
        function->userData                    = userData;
        function->maximumArguments            = maximumNativeArguments_;
        function->maximumResults              = maximumNativeResults_;
        NativeFunctionRecord *functionPointer = function.get();
        try
        {
            runtime->nativeFunctions.push_back(std::move(function));
        }
        catch (...)
        {
            return false;
        }
        lua_pushlightuserdata(runtime->state, functionPointer);
        lua_pushcclosure(runtime->state, invokeNativeFunction, 1);
        const int functionIndex = lua_absindex(runtime->state, -1);
        if (!setGlobalPath(runtime->state, name, functionIndex))
        {
            lua_pop(runtime->state, 1);
            runtime->nativeFunctions.pop_back();
            return false;
        }
        lua_pop(runtime->state, 1);
        return true;
    }

    bool setGlobalNumberImpl(ScriptRuntimeHandle handle, const char *name, double value)
    {
        auto runtime = findRuntime(handle);
        if (!runtime || name == nullptr || name[0] == '\0' || !std::isfinite(value))
            return false;
        std::lock_guard lock(runtime->mutex);
        lua_pushnumber(runtime->state, value);
        const int  valueIndex = lua_absindex(runtime->state, -1);
        const bool assigned   = setGlobalPath(runtime->state, name, valueIndex);
        lua_pop(runtime->state, 1);
        return assigned;
    }

    bool getGlobalNumberImpl(ScriptRuntimeHandle handle, const char *name, double *output)
    {
        auto runtime = findRuntime(handle);
        if (!runtime || name == nullptr || output == nullptr)
            return false;
        std::lock_guard lock(runtime->mutex);
        if (!pushGlobalPath(runtime->state, name))
            return false;
        if (!lua_isnumber(runtime->state, -1))
        {
            lua_pop(runtime->state, 1);
            return false;
        }
        *output = lua_tonumber(runtime->state, -1);
        lua_pop(runtime->state, 1);
        return true;
    }

    bool setGlobalStringImpl(ScriptRuntimeHandle handle, const char *name, const char *value)
    {
        auto runtime = findRuntime(handle);
        if (!runtime || name == nullptr || name[0] == '\0' || value == nullptr)
            return false;
        std::lock_guard   lock(runtime->mutex);
        const std::size_t maximumBytes = static_cast<std::size_t>(runtime->description.maximumScriptBytes);
        const auto       *terminator   = static_cast<const char *>(std::memchr(value, '\0', maximumBytes + 1u));
        if (terminator == nullptr)
            return false;
        lua_pushlstring(runtime->state, value, static_cast<std::size_t>(terminator - value));
        const int  valueIndex = lua_absindex(runtime->state, -1);
        const bool assigned   = setGlobalPath(runtime->state, name, valueIndex);
        lua_pop(runtime->state, 1);
        return assigned;
    }

    bool getGlobalStringImpl(
        ScriptRuntimeHandle handle, const char *name, char *output, std::uint32_t capacity, std::uint32_t *outRequired)
    {
        auto runtime = findRuntime(handle);
        if (!runtime || name == nullptr)
            return false;
        std::lock_guard lock(runtime->mutex);
        if (!pushGlobalPath(runtime->state, name))
            return false;
        std::size_t length = 0;
        const char *value  = lua_tolstring(runtime->state, -1, &length);
        if (value == nullptr || length >= UINT32_MAX)
        {
            lua_pop(runtime->state, 1);
            return false;
        }
        const bool copied = copyText(std::string_view(value, length), output, capacity, outRequired);
        lua_pop(runtime->state, 1);
        return copied;
    }

    bool collectGarbageImpl(ScriptRuntimeHandle handle)
    {
        auto runtime = findRuntime(handle);
        if (!runtime)
            return false;
        std::lock_guard lock(runtime->mutex);
        lua_gc(runtime->state, LUA_GCCOLLECT);
        return true;
    }
#else
    ScriptRuntimeHandle createRuntimeImpl(const ScriptRuntimeDescription *, ScriptError *error)
    {
        setError(error, ScriptError::RuntimeFailed);
        return invalidScriptRuntime;
    }
    void destroyRuntimeImpl(ScriptRuntimeHandle)
    {}
    bool runtimeInfoImpl(ScriptRuntimeHandle, ScriptRuntimeInfo *)
    {
        return false;
    }
    bool executeStringImpl(
        ScriptRuntimeHandle, const char *, const char *, ScriptError *error, char *, std::uint32_t, std::uint32_t *)
    {
        setError(error, ScriptError::RuntimeFailed);
        return false;
    }
    bool executeFileImpl(ScriptRuntimeHandle, const char *, ScriptError *error, char *, std::uint32_t, std::uint32_t *)
    {
        setError(error, ScriptError::RuntimeFailed);
        return false;
    }
    bool callFunctionImpl(ScriptRuntimeHandle, const char *, ScriptError *error, char *, std::uint32_t, std::uint32_t *)
    {
        setError(error, ScriptError::RuntimeFailed);
        return false;
    }
    bool registerNativeFunctionImpl(ScriptRuntimeHandle, const char *, ScriptNativeFunction, void *)
    {
        return false;
    }
    bool setGlobalNumberImpl(ScriptRuntimeHandle, const char *, double)
    {
        return false;
    }
    bool getGlobalNumberImpl(ScriptRuntimeHandle, const char *, double *)
    {
        return false;
    }
    bool setGlobalStringImpl(ScriptRuntimeHandle, const char *, const char *)
    {
        return false;
    }
    bool getGlobalStringImpl(ScriptRuntimeHandle, const char *, char *, std::uint32_t, std::uint32_t *)
    {
        return false;
    }
    bool collectGarbageImpl(ScriptRuntimeHandle)
    {
        return false;
    }
    ScriptRuntimeInitializerHandle registerRuntimeInitializerImpl(ScriptRuntimeInitializer, void *)
    {
        return invalidScriptRuntimeInitializer;
    }
    bool unregisterRuntimeInitializerImpl(ScriptRuntimeInitializerHandle)
    {
        return false;
    }
#endif

    ParticipantManifest      manifest_;
    ScriptRuntimeCapability  capability_{};
    ScriptRuntimeDescription defaultDescription_{};

    ParticipantContext *context_                 = nullptr;
    SubscriptionToken   startupSubscription_     = invalidSubscriptionToken;
    SubscriptionToken   frameSubscription_       = invalidSubscriptionToken;
    SubscriptionToken   fixedUpdateSubscription_ = invalidSubscriptionToken;
    SubscriptionToken   shutdownSubscription_    = invalidSubscriptionToken;
    SubscriptionToken   keySubscription_         = invalidSubscriptionToken;

    std::mutex                            hostedMutex_;
    std::vector<HostedPlugin>             hostedPlugins_;
    std::mutex                            initializersMutex_;
    std::vector<RuntimeInitializerRecord> initializers_;
    ScriptRuntimeInitializerHandle        nextInitializer_ = 1;
#if defined(ENGINE_SCRIPT_RUNTIME_BACKEND_LUA)
    std::mutex                                                              runtimesMutex_;
    std::unordered_map<ScriptRuntimeHandle, std::shared_ptr<RuntimeRecord>> runtimes_;
    ScriptRuntimeHandle                                                     nextHandle_ = 1;
#endif
    std::size_t                maximumRuntimes_         = 256;
    std::size_t                maximumNativeFunctions_  = 256;
    std::uint64_t              maximumTotalMemoryBytes_ = 1024ull * 1024ull * 1024ull;
    std::atomic<std::uint64_t> totalMemoryUsed_{0};
    std::uint32_t              maximumNativeArguments_     = 32;
    std::uint32_t              maximumNativeResults_       = 16;
    std::size_t                maximumRuntimeInitializers_ = 64;
    std::uint32_t              instructionHookGranularity_ = 1000;

    // Reused by dispatchToHostedPlugins to avoid a per-frame allocation.
    std::vector<ScriptRuntimeHandle> dispatchBatch_;
};

} // namespace
} // namespace engine::script_runtime

extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"script_runtime"};
    static const engine::EngineModuleEntryTable entry      = {
        .abiVersion    = engine::engineModuleAbiVersion,
        .moduleName    = "script_runtime",
        .moduleVersion = "0.1.0",
        .createModule = []() -> engine::ModuleInterface      *{ return new engine::script_runtime::ScriptRuntimeModule(); },
        .providedCapabilities    = provided,
        .providedCapabilityCount = 1,
        .requiredCapabilities    = nullptr,
        .requiredCapabilityCount = 0,
        .optionalCapabilities    = nullptr,
        .optionalCapabilityCount = 0,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0};
    return &entry;
}
