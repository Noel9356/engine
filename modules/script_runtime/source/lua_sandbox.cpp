// Implementation of the Lua sandboxing primitives declared in lua_sandbox.hpp.
//
// The capability implementation and the plugin host both link against this TU
// so the Lua-facing helpers (allocator, instruction hook, native-function
// bridge, library whitelist) live in one place. The capability binding in
// script_runtime.cpp only orchestrates: it owns the runtime map and forwards
// to these helpers.

#include "lua_sandbox.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <string_view>

#if defined(ENGINE_SCRIPT_RUNTIME_BACKEND_LUA)
extern "C"
{
#include <lualib.h>
}
#endif

namespace engine::script_runtime::sandbox
{

void setError(ScriptError *output, ScriptError value) noexcept
{
    if (output != nullptr)
        *output = value;
}

bool copyText(std::string_view text, char *output, std::uint32_t capacity, std::uint32_t *outRequired) noexcept
{
    if (text.size() >= UINT32_MAX)
        return false;
    const auto required = static_cast<std::uint32_t>(text.size() + 1u);
    if (outRequired != nullptr)
        *outRequired = required;
    if (output == nullptr || capacity < required)
        return false;
    if (!text.empty())
        std::memcpy(output, text.data(), text.size());
    output[text.size()] = '\0';
    return true;
}

#if defined(ENGINE_SCRIPT_RUNTIME_BACKEND_LUA)

namespace
{

// Maximum byte length of a global path or chunk name accepted by the helpers.
// Keeps the memchr-based terminator scan bounded.
constexpr std::size_t maximumGlobalPathBytes = 4096;

// Hard upper bounds for the native-function argument/result arrays. The
// configurable per-runtime caps in NativeFunctionRecord can be lower but never
// higher than these.
constexpr std::uint32_t hardMaximumArguments = 32;
constexpr std::uint32_t hardMaximumResults   = 16;

} // namespace

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

void *luaAllocator(void *userData, void *pointer, std::size_t oldSize, std::size_t newSize) noexcept
{
    auto               *runtime       = static_cast<RuntimeRecord *>(userData);
    const std::uint64_t actualOldSize = pointer == nullptr ? 0u : static_cast<std::uint64_t>(oldSize);
    if (newSize == 0)
    {
        std::free(pointer);
        runtime->memoryUsed -= std::min(runtime->memoryUsed, actualOldSize);
        if (runtime->globalMemoryUsed != nullptr)
        {
            std::uint64_t       current  = runtime->globalMemoryUsed->load(std::memory_order_relaxed);
            const std::uint64_t released = std::min(current, actualOldSize);
            while (released > 0
                   && !runtime->globalMemoryUsed->compare_exchange_weak(
                       current, current - released, std::memory_order_acq_rel, std::memory_order_relaxed))
            {
                current = std::min(current, runtime->globalMemoryUsed->load(std::memory_order_relaxed));
            }
        }
        return nullptr;
    }

    const std::uint64_t requestedSize = static_cast<std::uint64_t>(newSize);
    const bool          growing       = requestedSize > actualOldSize;
    const std::uint64_t growth        = growing ? requestedSize - actualOldSize : 0u;
    if (growing)
    {
        if (growth > runtime->description.memoryLimitBytes
                         - std::min(runtime->memoryUsed, runtime->description.memoryLimitBytes))
        {
            runtime->lastErrorCategory = LastErrorCategory::MemoryLimit;
            return nullptr;
        }
        if (runtime->globalMemoryUsed != nullptr)
        {
            std::uint64_t current = runtime->globalMemoryUsed->load(std::memory_order_relaxed);
            for (;;)
            {
                if (growth > runtime->globalMemoryLimitBytes - std::min(current, runtime->globalMemoryLimitBytes))
                {
                    runtime->lastErrorCategory = LastErrorCategory::MemoryLimit;
                    return nullptr;
                }
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
        {
            std::uint64_t       current  = runtime->globalMemoryUsed->load(std::memory_order_relaxed);
            const std::uint64_t rollback = std::min(current, growth);
            while (rollback > 0
                   && !runtime->globalMemoryUsed->compare_exchange_weak(
                       current, current - rollback, std::memory_order_acq_rel, std::memory_order_relaxed))
            {
                current = std::min(current, runtime->globalMemoryUsed->load(std::memory_order_relaxed));
            }
        }
        return nullptr;
    }

    if (growing)
        runtime->memoryUsed += growth;
    else if (requestedSize < actualOldSize)
    {
        const std::uint64_t released = actualOldSize - requestedSize;
        runtime->memoryUsed -= std::min(runtime->memoryUsed, released);
        if (runtime->globalMemoryUsed != nullptr)
        {
            std::uint64_t       current   = runtime->globalMemoryUsed->load(std::memory_order_relaxed);
            const std::uint64_t decrement = std::min(current, released);
            while (decrement > 0
                   && !runtime->globalMemoryUsed->compare_exchange_weak(
                       current, current - decrement, std::memory_order_acq_rel, std::memory_order_relaxed))
            {
                current = std::min(current, runtime->globalMemoryUsed->load(std::memory_order_relaxed));
            }
        }
    }
    return replacement;
}

// True when the runtime's optional wall-clock budget has expired. Checked at
// every hook and native-call boundary; the budget bounds whole executions
// (including native callback time the instruction counter cannot see).
bool wallClockExpired(const RuntimeRecord &runtime) noexcept
{
    return runtime.wallClockDeadlineEnabled && std::chrono::steady_clock::now() >= runtime.wallClockDeadline;
}

void armWallClockBudget(RuntimeRecord &runtime) noexcept
{
    if (runtime.description.wallClockLimitMilliseconds == 0)
    {
        runtime.wallClockDeadlineEnabled = false;
        return;
    }
    const std::uint64_t milliseconds = runtime.description.wallClockLimitMilliseconds;
    runtime.wallClockDeadline
        = std::chrono::steady_clock::now()
          + std::chrono::milliseconds(std::min<std::uint64_t>(milliseconds, std::numeric_limits<std::int64_t>::max()));
    runtime.wallClockDeadlineEnabled = true;
}

void instructionHook(lua_State *state, lua_Debug * /*debug*/) noexcept
{
    auto         **slot    = static_cast<RuntimeRecord **>(lua_getextraspace(state));
    RuntimeRecord *runtime = *slot;
    if (runtime == nullptr)
        return;
    if (wallClockExpired(*runtime))
    {
        runtime->lastErrorCategory = LastErrorCategory::WallClockLimit;
        luaL_error(state, "wall-clock limit exceeded");
        return;
    }
    // The hook fires every `hookGranularity` VM instructions (the clamped count
    // passed to lua_sethook in setExecutionBudget). Decrement by the same
    // amount so the budget tracks the configured instruction limit accurately.
    const std::uint32_t granularity = runtime->hookGranularity != 0 ? runtime->hookGranularity : 1000;
    if (runtime->instructionsRemaining < granularity)
    {
        runtime->instructionsRemaining = 0;
        runtime->lastErrorCategory     = LastErrorCategory::InstructionLimit;
        luaL_error(state, "instruction limit exceeded");
        return;
    }
    runtime->instructionsRemaining -= granularity;
}

int invokeNativeFunction(lua_State *state) noexcept
{
    auto *function = static_cast<NativeFunctionRecord *>(lua_touserdata(state, lua_upvalueindex(1)));
    if (function == nullptr || function->callback == nullptr)
        return luaL_error(state, "native function is unavailable");
    const std::uint32_t maximumArguments = function->maximumArguments;
    const std::uint32_t maximumResults   = function->maximumResults;
    const int           luaArgumentCount = lua_gettop(state);
    if (maximumArguments > hardMaximumArguments || maximumResults > hardMaximumResults || luaArgumentCount < 0
        || static_cast<std::uint32_t>(luaArgumentCount) > maximumArguments)
    {
        return luaL_error(state, "native function argument limit exceeded");
    }
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
        auto         **slot    = static_cast<RuntimeRecord **>(lua_getextraspace(state));
        RuntimeRecord *runtime = slot != nullptr ? *slot : nullptr;
        succeeded = function->callback(function->userData,
            arguments.data(),
            static_cast<std::uint32_t>(luaArgumentCount),
            results.data(),
            maximumResults,
            &resultCount);
        // Native callback time is invisible to the count hook; a wall-clock
        // budget is re-checked as soon as the callback returns so an execution
        // cannot extend its budget through repeated native calls.
        if (succeeded && runtime != nullptr && wallClockExpired(*runtime))
        {
            runtime->lastErrorCategory = LastErrorCategory::WallClockLimit;
            return luaL_error(state, "wall-clock limit exceeded");
        }
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

void setExecutionBudget(RuntimeRecord &runtime) noexcept
{
    runtime.instructionsRemaining = runtime.description.instructionLimit;
    armWallClockBudget(runtime);
    // The count hook fires every `count` VM instructions. Clamp the configured
    // granularity against the budget: a hook interval larger than a fraction
    // of the limit would overshoot the budget by up to one interval before the
    // first check. One quarter keeps the hook overhead negligible for large
    // budgets while bounding the overshoot to 25%.
    const std::uint32_t configuredGranularity = runtime.hookGranularity != 0 ? runtime.hookGranularity : 1000;
    const std::uint64_t granularityCeiling = std::max<std::uint64_t>(1u, runtime.description.instructionLimit / 4u);
    const std::uint32_t granularity        = static_cast<std::uint32_t>(
        std::min<std::uint64_t>(configuredGranularity, std::min<std::uint64_t>(granularityCeiling, std::numeric_limits<std::uint32_t>::max())));
    lua_sethook(runtime.state,
        instructionHook,
        LUA_MASKCOUNT,
        static_cast<int>(std::min<std::uint64_t>(granularity, std::numeric_limits<int>::max())));
}

void resetLastError(RuntimeRecord &runtime) noexcept
{
    runtime.lastErrorCategory = LastErrorCategory::None;
}

std::string_view stackError(lua_State *state) noexcept
{
    std::size_t length  = 0;
    const char *message = lua_tolstring(state, -1, &length);
    return message != nullptr ? std::string_view(message, length)
                              : std::string_view("Lua execution failed without an error message");
}

ScriptError classifyError(RuntimeRecord &runtime, ScriptError fallback) noexcept
{
    switch (runtime.lastErrorCategory)
    {
    case LastErrorCategory::MemoryLimit:
        return ScriptError::MemoryLimitExceeded;
    case LastErrorCategory::InstructionLimit:
        return ScriptError::InstructionLimitExceeded;
    case LastErrorCategory::WallClockLimit:
        return ScriptError::WallClockLimitExceeded;
    default:
        return fallback;
    }
}

bool pushGlobalPath(lua_State *state, const char *name) noexcept
{
    if (state == nullptr || name == nullptr)
        return false;
    const auto *terminator = static_cast<const char *>(std::memchr(name, '\0', maximumGlobalPathBytes + 1u));
    if (terminator == nullptr || terminator == name)
        return false;
    const std::string_view path(name, static_cast<std::size_t>(terminator - name));
    if (path.find('.') == std::string_view::npos)
    {
        lua_pushglobaltable(state);
        lua_pushstring(state, name);
        lua_rawget(state, -2);
        lua_remove(state, -2);
        return true;
    }

    const int base = lua_gettop(state);
    lua_pushglobaltable(state);
    int         parent = lua_absindex(state, -1);
    std::size_t begin  = 0;
    while (begin < path.size())
    {
        const std::size_t separator = path.find('.', begin);
        const std::size_t end       = separator == std::string_view::npos ? path.size() : separator;
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
        begin  = separator + 1u;
    }
    lua_settop(state, base);
    return false;
}

bool setGlobalPath(lua_State *state, const char *name, int valueIndex) noexcept
{
    if (state == nullptr || name == nullptr)
        return false;
    const auto *terminator = static_cast<const char *>(std::memchr(name, '\0', maximumGlobalPathBytes + 1u));
    if (terminator == nullptr || terminator == name)
        return false;
    const std::string_view path(name, static_cast<std::size_t>(terminator - name));
    valueIndex = lua_absindex(state, valueIndex);
    if (path.find('.') == std::string_view::npos)
    {
        lua_pushglobaltable(state);
        lua_pushstring(state, name);
        lua_pushvalue(state, valueIndex);
        lua_rawset(state, -3);
        lua_pop(state, 1);
        return true;
    }

    const int base = lua_gettop(state);
    lua_pushglobaltable(state);
    int         parent = lua_absindex(state, -1);
    std::size_t begin  = 0;
    for (;;)
    {
        const std::size_t separator = path.find('.', begin);
        const std::size_t end       = separator == std::string_view::npos ? path.size() : separator;
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
        begin  = separator + 1u;
    }
}

namespace
{

void removeGlobal(lua_State *state, const char *name) noexcept
{
    lua_pushnil(state);
    lua_setglobal(state, name);
}

// Removes a field from the table on top of the stack. Used to strip
// string.dump after the string library has been opened.
void removeField(lua_State *state, const char *name) noexcept
{
    lua_pushnil(state);
    lua_setfield(state, -2, name);
}

} // namespace

void openLibraries(lua_State *state, std::uint32_t libraries) noexcept
{
    luaL_requiref(state, LUA_GNAME, luaopen_base, 1);
    // Strip base-library functions that would let a script reach the host
    // filesystem, control the VM, or bypass the text-only chunk restriction
    // enforced by luaL_loadbufferx.
    removeGlobal(state, "dofile");
    removeGlobal(state, "loadfile");
    removeGlobal(state, "collectgarbage");
    removeGlobal(state, "load");
    lua_pop(state, 1);
    if ((libraries & ScriptLibraryMath) != 0)
    {
        luaL_requiref(state, LUA_MATHLIBNAME, luaopen_math, 1);
        lua_pop(state, 1);
    }
    if ((libraries & ScriptLibraryString) != 0)
    {
        luaL_requiref(state, LUA_STRLIBNAME, luaopen_string, 1);
        // string.dump exposes function bytecode, which combined with any
        // binary-chunk loader would evade the text-only chunk guard. With
        // `load` removed there is no built-in consumer, but the dump is still
        // an information leak; remove it as well.
        removeField(state, "dump");
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
    if ((libraries & ScriptLibraryCoroutine) != 0)
    {
        luaL_requiref(state, LUA_COLIBNAME, luaopen_coroutine, 1);
        lua_pop(state, 1);
    }
}

#endif // ENGINE_SCRIPT_RUNTIME_BACKEND_LUA

} // namespace engine::script_runtime::sandbox
