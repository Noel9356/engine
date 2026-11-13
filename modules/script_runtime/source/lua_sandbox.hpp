// Private Lua sandboxing primitives for the script_runtime module.
//
// Owns the Lua-facing types and helpers shared between the capability
// implementation and the plugin host: the runtime record, the native
// function bridge, the custom allocator, the instruction-count hook, and
// the global-path helpers. Everything here is internal to the module and
// must not be included from outside the script_runtime source directory.

#ifndef ENGINE_SCRIPT_RUNTIME_LUA_SANDBOX_HPP
#define ENGINE_SCRIPT_RUNTIME_LUA_SANDBOX_HPP

#include <engine/script_runtime/capabilities.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#if defined(ENGINE_SCRIPT_RUNTIME_BACKEND_LUA)
extern "C"
{
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
}
#endif

namespace engine::script_runtime::sandbox
{

// One registered runtime initializer: the host callback plus its handle and
// user-data. The handle is opaque to the sandbox; the module owns the vector
// and the next-handle counter.
struct RuntimeInitializerRecord
{
    ScriptRuntimeInitializerHandle handle   = invalidScriptRuntimeInitializer;
    ScriptRuntimeInitializer       callback = nullptr;
    void                          *userData = nullptr;
};

// Writes an error code through an optional out-parameter.
void setError(ScriptError *output, ScriptError value) noexcept;

// Copies text into a caller-owned buffer. Reports the required size (including
// the terminating NUL) through outRequired even when the buffer is too small,
// so callers can retry with a larger allocation. This is the return convention
// of the errorText outputs across the capability: on a failed execute/call the
// boolean return only reports usage problems, the ScriptError out-parameter
// reports the runtime failure itself.
bool copyText(std::string_view text, char *output, std::uint32_t capacity, std::uint32_t *outRequired) noexcept;

#if defined(ENGINE_SCRIPT_RUNTIME_BACKEND_LUA)

// Records the category of the most recent Lua failure so the capability can
// classify errors without parsing Lua's message text. Set by the allocator
// (MemoryLimit), the instruction hook (InstructionLimit, WallClockLimit), and
// the native-function bridge (WallClockLimit); cleared at the start of each
// execute/call by resetLastError().
enum class LastErrorCategory : std::uint8_t
{
    None = 0,
    MemoryLimit,
    InstructionLimit,
    WallClockLimit,
};

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
// capability layer and the plugin host to reference the same record; the mutex
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

    RuntimeRecord(const RuntimeRecord &)            = delete;
    RuntimeRecord &operator=(const RuntimeRecord &) = delete;

    ScriptRuntimeDescription                           description;
    lua_State                                         *state = nullptr;
    mutable std::mutex                                 mutex;
    std::uint64_t                                      memoryUsed             = 0;
    std::uint64_t                                      instructionsRemaining  = 0;
    std::uint32_t                                      hookGranularity        = 1000;
    std::atomic<std::uint64_t>                        *globalMemoryUsed       = nullptr;
    std::uint64_t                                      globalMemoryLimitBytes = 0;
    // Optional wall-clock budget (description.wallClockLimitMilliseconds > 0);
    // armed before each execute/call and checked by the instruction hook and
    // the native-function bridge.
    std::chrono::steady_clock::time_point              wallClockDeadline{};
    bool                                               wallClockDeadlineEnabled = false;
    LastErrorCategory                                  lastErrorCategory      = LastErrorCategory::None;
    std::vector<std::unique_ptr<NativeFunctionRecord>> nativeFunctions;
    std::string                                        lastCallbackError;
};

// Per-process hash seed for Lua's string table (Lua seeds itself; this keeps
// the engine's own allocations unpredictable as well).
unsigned int luaHashSeed() noexcept;

// Custom allocator backing every Lua allocation. Enforces two budgets: the
// per-runtime limit (description.memoryLimitBytes) and the module-wide limit
// (globalMemoryLimitBytes, shared atomically across all runtimes). A growth
// beyond either budget returns null, which Lua surfaces as a memory error.
void *luaAllocator(void *userData, void *pointer, std::size_t oldSize, std::size_t newSize) noexcept;

// Count hook that turns description.instructionLimit into a hard budget: past
// the limit the VM is stopped with a Lua error. Granularity is configurable to
// trade overhead against precision.
void instructionHook(lua_State *state, lua_Debug *debug) noexcept;

// Bridge between Lua closures and ScriptNativeFunction callbacks: marshals Lua
// values to ScriptValue (nil/boolean/number/string only), invokes the callback,
// and pushes the results back. Any violation (bad types, limits, exceptions)
// becomes a Lua error instead of crossing into C++.
int invokeNativeFunction(lua_State *state) noexcept;

// Arms the instruction budget for one execute/call. The hook is removed again
// by the caller once the protected call finished.
void setExecutionBudget(RuntimeRecord &runtime) noexcept;

// Resets the per-call error category. Call at the start of every execute/call
// so a stale classification from a previous run does not leak.
void resetLastError(RuntimeRecord &runtime) noexcept;

// Returns the message on top of the Lua stack without copying. The caller must
// pop or use the result before any other Lua call.
std::string_view stackError(lua_State *state) noexcept;

// Classifies a Lua failure into a ScriptError using the runtime's tagged error
// category first, then falling back to the supplied default for syntax errors
// and other untagged failures.
ScriptError classifyError(RuntimeRecord &runtime, ScriptError fallback) noexcept;

// Resolve a dotted global path without invoking Lua metamethods. The function
// leaves the final value on the stack; missing or non-table namespace segments
// restore the original stack top and return false.
bool pushGlobalPath(lua_State *state, const char *name) noexcept;

// Assign a value to a global path. Namespace tables are created lazily; an
// existing non-table namespace is treated as a collision, not overwritten.
bool setGlobalPath(lua_State *state, const char *name, int valueIndex) noexcept;

// Opens the base library plus the whitelisted standard libraries and removes
// the ones that would let a script reach the host filesystem, control the VM,
// or evade the text-only chunk restriction (load, dofile, loadfile,
// collectgarbage, string.dump). No io/os/package/debug library is ever opened.
void openLibraries(lua_State *state, std::uint32_t libraries) noexcept;

#endif // ENGINE_SCRIPT_RUNTIME_BACKEND_LUA

} // namespace engine::script_runtime::sandbox

#endif // ENGINE_SCRIPT_RUNTIME_LUA_SANDBOX_HPP
