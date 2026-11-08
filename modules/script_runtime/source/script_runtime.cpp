// script_runtime module implementation.
//
// Provides the "script_runtime" capability: sandboxed Lua 5.5 runtimes with
// memory and instruction budgets, plus hosting for type = "script" game mods
// (see ScriptGameModHost). When ENGINE_SCRIPT_RUNTIME_BACKEND_LUA is not
// defined every capability entry fails gracefully and script mods cannot be
// hosted.

#include <engine/kernel/engine_kernel.hpp>
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/script_runtime/capabilities.hpp>
#include <engine/util/hardware_info.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#if defined(ENGINE_SCRIPT_RUNTIME_BACKEND_LUA)
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <utility>

extern "C"
{
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
}
#endif

namespace engine::script_runtime
{
namespace
{

struct RuntimeInitializerRecord
{
    ScriptRuntimeInitializerHandle handle   = invalidScriptRuntimeInitializer;
    ScriptRuntimeInitializer       callback = nullptr;
    void                          *userData = nullptr;
};

// Writes an error code through an optional out-parameter.
void setError(ScriptError *output, ScriptError value)
{
    if (output != nullptr)
        *output = value;
}

#if defined(ENGINE_SCRIPT_RUNTIME_BACKEND_LUA)

// Copies text into a caller-owned buffer. Reports the required size (including
// the terminating NUL) through outRequired even when the buffer is too small,
// so callers can retry with a larger allocation. This is the return convention
// of the errorText outputs across the capability: on a failed execute/call the
// boolean return only reports usage problems, the ScriptError out-parameter
// reports the runtime failure itself.
bool copyText(const std::string &text, char *output, std::uint32_t capacity, std::uint32_t *outRequired)
{
    if (text.size() >= UINT32_MAX)
        return false;
    const auto required = static_cast<std::uint32_t>(text.size() + 1u);
    if (outRequired != nullptr)
        *outRequired = required;
    if (output == nullptr || capacity < required)
        return false;
    std::memcpy(output, text.c_str(), required);
    return true;
}

// Resolve a dotted global path without invoking Lua metamethods. The function
// leaves the final value on the stack; missing or non-table namespace segments
// restore the original stack top and return false.
bool pushGlobalPath(lua_State *state, const char *name)
{
    if (state == nullptr || name == nullptr || name[0] == '\0')
        return false;
    const std::string_view path(name);
    if (path.find('.') == std::string_view::npos)
    {
        lua_getglobal(state, name);
        return true;
    }

    const int base = lua_gettop(state);
    lua_pushglobaltable(state);
    int parent = lua_absindex(state, -1);
    std::size_t begin = 0;
    while (begin < path.size())
    {
        const std::size_t separator = path.find('.', begin);
        const std::size_t end = separator == std::string_view::npos ? path.size() : separator;
        if (end == begin)
        {
            lua_settop(state, base);
            return false;
        }
        lua_pushlstring(state, path.data() + begin, end - begin);
        lua_rawget(state, parent);
        if (separator == std::string_view::npos)
        {
            lua_remove(state, parent);
            return true;
        }
        if (!lua_istable(state, -1))
        {
            lua_settop(state, base);
            return false;
        }
        lua_remove(state, parent);
        parent = lua_absindex(state, -1);
        begin = separator + 1u;
    }
    lua_settop(state, base);
    return false;
}

// Assign a value to a global path. Namespace tables are created lazily; an
// existing non-table namespace is treated as a collision, not overwritten.
bool setGlobalPath(lua_State *state, const char *name, int valueIndex)
{
    if (state == nullptr || name == nullptr || name[0] == '\0')
        return false;
    const std::string_view path(name);
    valueIndex = lua_absindex(state, valueIndex);
    if (path.find('.') == std::string_view::npos)
    {
        lua_pushvalue(state, valueIndex);
        lua_setglobal(state, name);
        return true;
    }

    const int base = lua_gettop(state);
    lua_pushglobaltable(state);
    int parent = lua_absindex(state, -1);
    std::size_t begin = 0;
    for (;;)
    {
        const std::size_t separator = path.find('.', begin);
        const std::size_t end = separator == std::string_view::npos ? path.size() : separator;
        if (end == begin)
        {
            lua_settop(state, base);
            return false;
        }
        lua_pushlstring(state, path.data() + begin, end - begin);
        if (separator == std::string_view::npos)
        {
            lua_pushvalue(state, valueIndex);
            lua_rawset(state, parent);
            lua_pop(state, 1);
            return true;
        }

        lua_rawget(state, parent);
        if (lua_isnil(state, -1))
        {
            lua_pop(state, 1);
            lua_newtable(state);
            lua_pushlstring(state, path.data() + begin, end - begin);
            lua_pushvalue(state, -2);
            lua_rawset(state, parent);
        }
        else if (!lua_istable(state, -1))
        {
            lua_settop(state, base);
            return false;
        }
        lua_remove(state, parent);
        parent = lua_absindex(state, -1);
        begin = separator + 1u;
    }
}

// One registered native function. The record is owned by the runtime so the
// Lua closure's upvalue pointer stays valid for the lifetime of the state.
struct NativeFunctionRecord
{
    ScriptNativeFunction callback         = nullptr;
    void                *userData         = nullptr;
    std::uint32_t        maximumArguments = 32;
    std::uint32_t        maximumResults   = 16;
};

// Everything that belongs to one created runtime. Shared pointers allow the
// capability layer and the mod host to reference the same record; the mutex
// serializes all use of the lua_State.
struct RuntimeRecord
{
    explicit RuntimeRecord(const ScriptRuntimeDescription &settings)
        : description(settings)
    {}
    ~RuntimeRecord()
    {
        if (state != nullptr)
            lua_close(state);
    }

    ScriptRuntimeDescription                           description;
    lua_State                                         *state = nullptr;
    mutable std::mutex                                 mutex;
    std::uint64_t                                      memoryUsed             = 0;
    std::uint64_t                                      instructionsRemaining  = 0;
    std::atomic<std::uint64_t>                        *globalMemoryUsed       = nullptr;
    std::uint64_t                                      globalMemoryLimitBytes = 0;
    std::vector<std::unique_ptr<NativeFunctionRecord>> nativeFunctions;
    std::string                                        lastCallbackError;
};

// Per-process hash seed for Lua's string table (Lua seeds itself; this keeps
// the engine's own allocations unpredictable as well).
unsigned int luaHashSeed() noexcept
{
    try
    {
        std::random_device randomDevice;
        return randomDevice();
    }
    catch (...)
    {
        static std::atomic<std::uint32_t> fallbackCounter{0u};
        const auto    now     = static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
        const auto    counter = fallbackCounter.fetch_add(1u, std::memory_order_relaxed);
        std::uint64_t value   = now ^ (static_cast<std::uint64_t>(counter) << 32u);
        value ^= value >> 30u;
        value *= 0xbf58476d1ce4e5b9ull;
        value ^= value >> 27u;
        value *= 0x94d049bb133111ebull;
        value ^= value >> 31u;
        return static_cast<unsigned int>(value ^ (value >> 32u));
    }
}

// Custom allocator backing every Lua allocation. Enforces two budgets: the
// per-runtime limit (description.memoryLimitBytes) and the module-wide limit
// (globalMemoryLimitBytes, shared atomically across all runtimes). A growth
// beyond either budget returns null, which Lua surfaces as a memory error.
void *luaAllocator(void *userData, void *pointer, std::size_t oldSize, std::size_t newSize)
{
    auto               *runtime       = static_cast<RuntimeRecord *>(userData);
    const std::uint64_t actualOldSize = pointer == nullptr ? 0u : static_cast<std::uint64_t>(oldSize);
    if (newSize == 0)
    {
        std::free(pointer);
        runtime->memoryUsed -= std::min(runtime->memoryUsed, actualOldSize);
        if (runtime->globalMemoryUsed != nullptr)
            runtime->globalMemoryUsed->fetch_sub(actualOldSize, std::memory_order_acq_rel);
        return nullptr;
    }

    const std::uint64_t requestedSize = static_cast<std::uint64_t>(newSize);
    const bool          growing       = requestedSize > actualOldSize;
    const std::uint64_t growth        = growing ? requestedSize - actualOldSize : 0u;
    if (growing)
    {
        if (growth > runtime->description.memoryLimitBytes
                         - std::min(runtime->memoryUsed, runtime->description.memoryLimitBytes))
            return nullptr;
        if (runtime->globalMemoryUsed != nullptr)
        {
            std::uint64_t current = runtime->globalMemoryUsed->load(std::memory_order_relaxed);
            for (;;)
            {
                if (growth > runtime->globalMemoryLimitBytes - std::min(current, runtime->globalMemoryLimitBytes))
                    return nullptr;
                if (runtime->globalMemoryUsed->compare_exchange_weak(
                        current, current + growth, std::memory_order_acq_rel, std::memory_order_relaxed))
                    break;
            }
        }
    }

    void *replacement = std::realloc(pointer, newSize);
    if (replacement == nullptr)
    {
        if (growing && runtime->globalMemoryUsed != nullptr)
            runtime->globalMemoryUsed->fetch_sub(growth, std::memory_order_acq_rel);
        return nullptr;
    }

    if (growing)
        runtime->memoryUsed += growth;
    else if (requestedSize < actualOldSize)
    {
        const std::uint64_t released = actualOldSize - requestedSize;
        runtime->memoryUsed -= std::min(runtime->memoryUsed, released);
        if (runtime->globalMemoryUsed != nullptr)
            runtime->globalMemoryUsed->fetch_sub(released, std::memory_order_acq_rel);
    }
    return replacement;
}

// Count hook that turns description.instructionLimit into a hard wall-clock
// independent budget: past the limit the VM is stopped with a Lua error.
// Granularity is 1000 instructions per hook call to keep the overhead low.
void instructionHook(lua_State *state, lua_Debug *)
{
    auto         **slot    = static_cast<RuntimeRecord **>(lua_getextraspace(state));
    RuntimeRecord *runtime = *slot;
    if (runtime == nullptr)
        return;
    constexpr std::uint64_t hookGranularity = 1000;
    if (runtime->instructionsRemaining <= hookGranularity)
    {
        runtime->instructionsRemaining = 0;
        luaL_error(state, "instruction limit exceeded");
        return;
    }
    runtime->instructionsRemaining -= hookGranularity;
}

// Bridge between Lua closures and ScriptNativeFunction callbacks: marshals
// Lua values to ScriptValue (nil/boolean/number/string only), invokes the
// callback, and pushes the results back. Any violation (bad types, limits,
// exceptions) becomes a Lua error instead of crossing into C++.
int invokeNativeFunction(lua_State *state)
{
    auto *function = static_cast<NativeFunctionRecord *>(lua_touserdata(state, lua_upvalueindex(1)));
    if (function == nullptr || function->callback == nullptr)
        return luaL_error(state, "native function is unavailable");
    constexpr std::uint32_t hardMaximumArguments = 32;
    constexpr std::uint32_t hardMaximumResults   = 16;
    const std::uint32_t     maximumArguments     = function->maximumArguments;
    const std::uint32_t     maximumResults       = function->maximumResults;
    const int               luaArgumentCount     = lua_gettop(state);
    if (maximumArguments > hardMaximumArguments || maximumResults > hardMaximumResults || luaArgumentCount < 0
        || static_cast<std::uint32_t>(luaArgumentCount) > maximumArguments)
        return luaL_error(state, "native function argument limit exceeded");
    std::array<ScriptValue, hardMaximumArguments> arguments{};
    std::array<ScriptValue, hardMaximumResults>   results{};
    for (int index = 0; index < luaArgumentCount; ++index)
    {
        ScriptValue &value      = arguments[static_cast<std::size_t>(index)];
        const int    stackIndex = index + 1;
        switch (lua_type(state, stackIndex))
        {
        case LUA_TNIL:
            value.type = ScriptValueType::Nil;
            break;
        case LUA_TBOOLEAN:
            value.type         = ScriptValueType::Boolean;
            value.booleanValue = lua_toboolean(state, stackIndex) != 0;
            break;
        case LUA_TNUMBER:
            value.type        = ScriptValueType::Number;
            value.numberValue = lua_tonumber(state, stackIndex);
            break;
        case LUA_TSTRING: {
            std::size_t length = 0;
            value.stringData   = lua_tolstring(state, stackIndex, &length);
            if (length > UINT32_MAX)
                return luaL_error(state, "native function string argument too large");
            value.stringSize = static_cast<std::uint32_t>(length);
            value.type       = ScriptValueType::String;
            break;
        }
        default:
            return luaL_error(state, "native functions accept only nil, boolean, number, and string arguments");
        }
    }
    std::uint32_t resultCount = 0;
    bool          succeeded   = false;
    try
    {
        succeeded = function->callback(function->userData,
            arguments.data(),
            static_cast<std::uint32_t>(luaArgumentCount),
            results.data(),
            maximumResults,
            &resultCount);
    }
    catch (...)
    {
        return luaL_error(state, "native function threw an exception");
    }
    if (!succeeded)
        return luaL_error(state, "native function returned failure");
    if (resultCount > maximumResults)
        return luaL_error(state, "native function result limit exceeded");
    for (std::uint32_t index = 0; index < resultCount; ++index)
    {
        const ScriptValue &value = results[index];
        switch (value.type)
        {
        case ScriptValueType::Nil:
            lua_pushnil(state);
            break;
        case ScriptValueType::Boolean:
            lua_pushboolean(state, value.booleanValue);
            break;
        case ScriptValueType::Number:
            if (!std::isfinite(value.numberValue))
                return luaL_error(state, "native function returned a non-finite number");
            lua_pushnumber(state, value.numberValue);
            break;
        case ScriptValueType::String:
            if (value.stringSize != 0 && value.stringData == nullptr)
                return luaL_error(state, "native function returned an invalid string");
            lua_pushlstring(state, value.stringData != nullptr ? value.stringData : "", value.stringSize);
            break;
        default:
            return luaL_error(state, "native function returned an unsupported value type");
        }
    }
    return static_cast<int>(resultCount);
}

// Arms the instruction budget for one execute/call. The hook is removed again
// by the caller once the protected call finished.
void setExecutionBudget(RuntimeRecord &runtime)
{
    runtime.instructionsRemaining = runtime.description.instructionLimit;
    lua_sethook(runtime.state, instructionHook, LUA_MASKCOUNT, 1000);
}

std::string stackError(lua_State *state)
{
    const char *message = lua_tostring(state, -1);
    return message != nullptr ? message : "Lua execution failed without an error message";
}

void removeGlobal(lua_State *state, const char *name)
{
    lua_pushnil(state);
    lua_setglobal(state, name);
}

// Opens the base library plus the whitelisted standard libraries and removes
// the ones that would let a script reach the host filesystem or control the
// VM (dofile, loadfile, collectgarbage). No io/os library is ever opened.
void openLibraries(lua_State *state, std::uint32_t libraries)
{
    luaL_requiref(state, LUA_GNAME, luaopen_base, 1);
    lua_pop(state, 1);
    removeGlobal(state, "dofile");
    removeGlobal(state, "loadfile");
    removeGlobal(state, "collectgarbage");
    if ((libraries & ScriptLibraryMath) != 0)
    {
        luaL_requiref(state, LUA_MATHLIBNAME, luaopen_math, 1);
        lua_pop(state, 1);
    }
    if ((libraries & ScriptLibraryString) != 0)
    {
        luaL_requiref(state, LUA_STRLIBNAME, luaopen_string, 1);
        lua_pop(state, 1);
    }
    if ((libraries & ScriptLibraryTable) != 0)
    {
        luaL_requiref(state, LUA_TABLIBNAME, luaopen_table, 1);
        lua_pop(state, 1);
    }
    if ((libraries & ScriptLibraryUtf8) != 0)
    {
        luaL_requiref(state, LUA_UTF8LIBNAME, luaopen_utf8, 1);
        lua_pop(state, 1);
    }
}

#endif

class ScriptRuntimeModule final : public ModuleInterface, public ScriptGameModHost
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

        // auto_config = true: the script memory ceilings derive from
        // installed RAM and the matching [script_runtime] keys have no
        // effect. Everything else (instruction budget, script size,
        // runtimes/native-function caps) bounds logic, not memory, and keeps
        // its built-in default in both modes.
        if (context.configurationBooleanInSection("script_runtime", "auto_config").value_or(false))
        {
            applyHardwareDerivedMemoryLimits();
        }
        else
        {
        if (const auto value = context.configurationIntegerInSection("script_runtime", "memory_limit_mb");
            value && *value >= 1 && *value <= 4096)
            defaultDescription_.memoryLimitBytes = static_cast<std::uint64_t>(*value) * 1024ull * 1024ull;
        if (const auto value = context.configurationIntegerInSection("script_runtime", "instruction_limit");
            value && *value >= 1000)
            defaultDescription_.instructionLimit = static_cast<std::uint64_t>(*value);
        if (const auto value = context.configurationIntegerInSection("script_runtime", "maximum_script_size_mb");
            value && *value >= 1 && *value <= 256)
            defaultDescription_.maximumScriptBytes = static_cast<std::uint64_t>(*value) * 1024ull * 1024ull;
        if (const auto value = context.configurationIntegerInSection("script_runtime", "maximum_runtimes");
            value && *value >= 1 && *value <= 4096)
            maximumRuntimes_ = static_cast<std::size_t>(*value);
        if (const auto value = context.configurationIntegerInSection("script_runtime", "maximum_native_functions");
            value && *value >= 1 && *value <= 4096)
            maximumNativeFunctions_ = static_cast<std::size_t>(*value);
        if (const auto value = context.configurationIntegerInSection("script_runtime", "maximum_total_memory_mb");
            value && *value >= 1 && *value <= 65536)
            maximumTotalMemoryBytes_ = static_cast<std::uint64_t>(*value) * 1024ull * 1024ull;
        if (const auto value = context.configurationIntegerInSection("script_runtime", "maximum_native_arguments");
            value && *value >= 1 && *value <= 32)
            maximumNativeArguments_ = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("script_runtime", "maximum_native_results");
            value && *value >= 1 && *value <= 16)
            maximumNativeResults_ = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("script_runtime", "maximum_runtime_initializers");
            value && *value >= 1 && *value <= 4096)
            maximumRuntimeInitializers_ = static_cast<std::size_t>(*value);
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
            hostedMods_.clear();
        }
        {
            std::lock_guard lock(initializersMutex_);
            initializers_.clear();
        }
#if defined(ENGINE_SCRIPT_RUNTIME_BACKEND_LUA)
        std::lock_guard lock(runtimesMutex_);
        runtimes_.clear();
#endif
    }
    void *capabilityPointerFor(const std::string &name) override
    {
        return name == ScriptRuntimeCapability::capabilityName ? &capability_ : nullptr;
    }

    ScriptGameModHost *scriptModHost() override
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

    // --- script mod hosting -------------------------------------------------
    //
    // The kernel hands every type = "script" mod to this module during the
    // construct phase (see ScriptGameModHost). Hosting a mod means running its
    // entry file in a fresh sandboxed Lua runtime and forwarding engine events
    // to the callback functions the script chose to define:
    //
    //   on_construct()                      after the entry file loaded
    //   on_start()                          EngineStartupCompletedEvent
    //   on_frame(delta, elapsed)            EngineFrameEvent
    //   on_fixed_update(delta)              EngineFixedUpdateEvent
    //   on_key_pressed(name)                WindowKeyEvent
    //   on_shutdown()                       EngineShutdownStartedEvent

    struct HostedMod
    {
        std::string         modId;
        ScriptRuntimeHandle runtime = invalidScriptRuntime;
    };

    ScriptRuntimeHandle hostedRuntimeImpl(const char *modId)
    {
        if (modId == nullptr)
            return invalidScriptRuntime;
        std::lock_guard lock(hostedMutex_);
        for (const HostedMod &mod : hostedMods_)
            if (mod.modId == modId)
                return mod.runtime;
        return invalidScriptRuntime;
    }

#if defined(ENGINE_SCRIPT_RUNTIME_BACKEND_LUA)

    // Calls a global function if the mod defined one. Absent callbacks are the
    // normal opt-in case, not errors. Runtime failures are printed once per
    // distinct message so a broken on_frame cannot flood stderr.
    void callOptionalGlobal(ScriptRuntimeHandle handle, const char *name, std::initializer_list<double> numbers = {},
        const char *text = nullptr)
    {
        auto runtime = findRuntime(handle);
        if (!runtime)
            return;
        std::lock_guard lock(runtime->mutex);
        lua_State      *state = runtime->state;
        lua_getglobal(state, name);
        if (!lua_isfunction(state, -1))
        {
            lua_pop(state, 1);
            return;
        }
        for (double number : numbers)
            lua_pushnumber(state, number);
        if (text != nullptr)
            lua_pushstring(state, text);
        setExecutionBudget(*runtime);
        const int argumentCount = static_cast<int>(numbers.size()) + (text != nullptr ? 1 : 0);
        const int result        = lua_pcall(state, argumentCount, 0, 0);
        lua_sethook(state, nullptr, 0, 0);
        if (result != LUA_OK)
        {
            const std::string message = stackError(state);
            lua_pop(state, 1);
            if (runtime->lastCallbackError != message)
            {
                runtime->lastCallbackError = message;
                std::fprintf(stderr, "script_runtime: callback '%s' failed: %s\n", name, message.c_str());
            }
        }
    }
#else
    void callOptionalGlobal(
        ScriptRuntimeHandle, const char *, std::initializer_list<double> = {}, const char * = nullptr)
    {}
#endif

    void dispatchToHostedMods(
        const char *callback, std::initializer_list<double> numbers = {}, const char *text = nullptr)
    {
        std::lock_guard lock(hostedMutex_);
        for (const HostedMod &mod : hostedMods_)
            callOptionalGlobal(mod.runtime, callback, numbers, text);
    }

    void onEngineStartupCompleted(const EngineStartupCompletedEvent &)
    {
        dispatchToHostedMods("on_start");
    }

    void onEngineFrame(const EngineFrameEvent &event)
    {
        dispatchToHostedMods("on_frame", {event.deltaSeconds, event.elapsedSeconds});
    }

    void onEngineFixedUpdate(const EngineFixedUpdateEvent &event)
    {
        dispatchToHostedMods("on_fixed_update", {event.deltaSeconds});
    }

    void onEngineShutdownStarted(const EngineShutdownStartedEvent &)
    {
        dispatchToHostedMods("on_shutdown");
    }

    void onWindowKey(const WindowKeyEvent &event)
    {
        dispatchToHostedMods("on_key_pressed", {}, event.keyName);
    }

    void hostScriptGameMod(const ScriptGameModDescription &description) override
    {
#if defined(ENGINE_SCRIPT_RUNTIME_BACKEND_LUA)
        ScriptError               error = ScriptError::None;
        std::array<char, 512>     errorBuffer{};
        const ScriptRuntimeHandle runtime = createRuntimeImpl(nullptr, &error);
        if (runtime == invalidScriptRuntime)
        {
            throw std::runtime_error("script mod '" + description.modId
                                     + "' could not create a Lua runtime (script_runtime error "
                                     + std::to_string(static_cast<std::uint32_t>(error)) + ")");
        }
        setGlobalStringImpl(runtime, "engine_mod_id", description.modId.c_str());
        setGlobalStringImpl(runtime, "engine_mod_version", description.version.c_str());
        setGlobalStringImpl(runtime, "engine_mod_path", description.scriptPath.string().c_str());
        if (!executeFileImpl(runtime,
                description.scriptPath.string().c_str(),
                &error,
                errorBuffer.data(),
                static_cast<std::uint32_t>(errorBuffer.size()),
                nullptr))
        {
            destroyRuntimeImpl(runtime);
            throw std::runtime_error("script mod '" + description.modId + "' failed to run '"
                                     + description.scriptPath.string() + "': " + std::string(errorBuffer.data()));
        }
        callOptionalGlobal(runtime, "on_construct");
        std::lock_guard lock(hostedMutex_);
        hostedMods_.push_back(HostedMod{.modId = description.modId, .runtime = runtime});
#else
        (void)description;
        throw std::runtime_error("script_runtime: the Lua backend is disabled; script mods cannot be hosted");
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
    ScriptRuntimeInitializerHandle registerRuntimeInitializerImpl(
        ScriptRuntimeInitializer initializer, void *userData)
    {
        if (initializer == nullptr)
            return invalidScriptRuntimeInitializer;

        std::lock_guard initializerLock(initializersMutex_);
        if (initializers_.size() >= maximumRuntimeInitializers_
            || nextInitializer_ == invalidScriptRuntimeInitializer
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
        // runtime cannot miss an initializer or receive it twice.
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
                initializers_.pop_back();
                return invalidScriptRuntimeInitializer;
            }
        }
        return handle;
    }

    bool unregisterRuntimeInitializerImpl(ScriptRuntimeInitializerHandle handle)
    {
        if (handle == invalidScriptRuntimeInitializer)
            return false;
        std::lock_guard lock(initializersMutex_);
        const auto found = std::find_if(initializers_.begin(), initializers_.end(),
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
            = ScriptLibraryMath | ScriptLibraryString | ScriptLibraryTable | ScriptLibraryUtf8;
        if (settings.memoryLimitBytes < 1024u * 1024u || settings.memoryLimitBytes > (1ull << 40u)
            || settings.instructionLimit < 1000u || settings.maximumScriptBytes == 0
            || settings.maximumScriptBytes > settings.memoryLimitBytes
            || (settings.enabledLibraries & ~allowedLibraries) != 0)
        {
            setError(error, ScriptError::InvalidArgument);
            return invalidScriptRuntime;
        }
        std::lock_guard initializerLock(initializersMutex_);
        auto runtime = std::make_shared<RuntimeRecord>(settings);
        runtime->globalMemoryUsed = &totalMemoryUsed_;
        runtime->globalMemoryLimitBytes = maximumTotalMemoryBytes_;
        runtime->state = lua_newstate(luaAllocator, runtime.get(), luaHashSeed());
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
        const std::size_t length = std::strlen(source);
        if (length > runtime->description.maximumScriptBytes)
        {
            setError(error, ScriptError::CapacityExceeded);
            return copyText("script exceeds configured size limit", errorText, errorCapacity, errorRequired);
        }
        lua_State *state = runtime->state;
        setExecutionBudget(*runtime);
        const int loaded = luaL_loadbufferx(state, source, length, chunkName != nullptr ? chunkName : "=(engine)", "t");
        if (loaded != LUA_OK)
        {
            const std::string message = stackError(state);
            lua_pop(state, 1);
            const ScriptError code = message.find("memory") != std::string::npos ? ScriptError::MemoryLimitExceeded
                                                                                 : ScriptError::CompilationFailed;
            setError(error, code);
            lua_sethook(state, nullptr, 0, 0);
            return copyText(message, errorText, errorCapacity, errorRequired);
        }
        const int executed = lua_pcall(state, 0, 0, 0);
        lua_sethook(state, nullptr, 0, 0);
        if (executed != LUA_OK)
        {
            const std::string message = stackError(state);
            lua_pop(state, 1);
            const ScriptError code = message.find("instruction limit exceeded") != std::string::npos
                                       ? ScriptError::InstructionLimitExceeded
                                       : (message.find("memory") != std::string::npos ? ScriptError::MemoryLimitExceeded
                                                                                      : ScriptError::RuntimeFailed);
            setError(error, code);
            return copyText(message, errorText, errorCapacity, errorRequired);
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
        if (filePath == nullptr || filePath[0] == '\0')
        {
            setError(error, ScriptError::InvalidArgument);
            return copyText("file path is empty", errorText, errorCapacity, errorRequired);
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
        if (fileSize > runtime->description.maximumScriptBytes)
        {
            setError(error, ScriptError::CapacityExceeded);
            return copyText("script exceeds configured size limit", errorText, errorCapacity, errorRequired);
        }
        std::ifstream stream(filePath, std::ios::binary);
        if (!stream)
        {
            setError(error, ScriptError::IoFailed);
            return copyText("unable to open script file", errorText, errorCapacity, errorRequired);
        }
        std::string source(static_cast<std::size_t>(fileSize), '\0');
        if (!source.empty())
            stream.read(source.data(), static_cast<std::streamsize>(source.size()));
        if (!stream && !source.empty())
        {
            setError(error, ScriptError::IoFailed);
            return copyText("unable to read script file", errorText, errorCapacity, errorRequired);
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
        setExecutionBudget(*runtime);
        const int result = lua_pcall(state, 0, 0, 0);
        lua_sethook(state, nullptr, 0, 0);
        if (result != LUA_OK)
        {
            const std::string message = stackError(state);
            lua_pop(state, 1);
            setError(error,
                message.find("instruction limit exceeded") != std::string::npos ? ScriptError::InstructionLimitExceeded
                                                                                : ScriptError::RuntimeFailed);
            return copyText(message, errorText, errorCapacity, errorRequired);
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
        auto function                         = std::make_unique<NativeFunctionRecord>();
        function->callback                    = callback;
        function->userData                    = userData;
        function->maximumArguments            = maximumNativeArguments_;
        function->maximumResults              = maximumNativeResults_;
        NativeFunctionRecord *functionPointer = function.get();
        lua_pushlightuserdata(runtime->state, functionPointer);
        lua_pushcclosure(runtime->state, invokeNativeFunction, 1);
        const int functionIndex = lua_absindex(runtime->state, -1);
        if (!setGlobalPath(runtime->state, name, functionIndex))
        {
            lua_pop(runtime->state, 1);
            return false;
        }
        lua_pop(runtime->state, 1);
        runtime->nativeFunctions.push_back(std::move(function));
        return true;
    }

    bool setGlobalNumberImpl(ScriptRuntimeHandle handle, const char *name, double value)
    {
        auto runtime = findRuntime(handle);
        if (!runtime || name == nullptr || name[0] == '\0' || !std::isfinite(value))
            return false;
        std::lock_guard lock(runtime->mutex);
        lua_pushnumber(runtime->state, value);
        const int valueIndex = lua_absindex(runtime->state, -1);
        const bool assigned = setGlobalPath(runtime->state, name, valueIndex);
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
        std::lock_guard lock(runtime->mutex);
        lua_pushstring(runtime->state, value);
        const int valueIndex = lua_absindex(runtime->state, -1);
        const bool assigned = setGlobalPath(runtime->state, name, valueIndex);
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
        const bool copied = copyText(std::string(value, length), output, capacity, outRequired);
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

    std::mutex             hostedMutex_;
    std::vector<HostedMod> hostedMods_;
    std::mutex initializersMutex_;
    std::vector<RuntimeInitializerRecord> initializers_;
    ScriptRuntimeInitializerHandle nextInitializer_ = 1;
#if defined(ENGINE_SCRIPT_RUNTIME_BACKEND_LUA)
    std::mutex                                                              runtimesMutex_;
    std::unordered_map<ScriptRuntimeHandle, std::shared_ptr<RuntimeRecord>> runtimes_;
    ScriptRuntimeHandle                                                     nextHandle_ = 1;
#endif
    std::size_t                maximumRuntimes_         = 256;
    std::size_t                maximumNativeFunctions_  = 256;
    std::uint64_t              maximumTotalMemoryBytes_ = 1024ull * 1024ull * 1024ull;
    std::atomic<std::uint64_t> totalMemoryUsed_{0};
    std::uint32_t              maximumNativeArguments_ = 32;
    std::uint32_t              maximumNativeResults_   = 16;
    std::size_t                maximumRuntimeInitializers_ = 64;

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
