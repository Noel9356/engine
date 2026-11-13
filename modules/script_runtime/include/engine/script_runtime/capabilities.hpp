// Public capability surface of the script_runtime module.
//
// The capability manages isolated Lua runtimes. Data crossing the boundary is
// limited to four primitive value types; everything else (files, globals,
// native functions) goes through explicit calls so the host stays in control
// of what a script can reach.

#ifndef ENGINE_SCRIPT_RUNTIME_CAPABILITIES_HPP
#define ENGINE_SCRIPT_RUNTIME_CAPABILITIES_HPP

#include <cstdint>

namespace engine::script_runtime
{

// Script runtime capability ABI. Version 2 added hostedRuntime(); version 3
// adds module runtime initializers so optional modules can install Lua bridges.
inline constexpr std::uint32_t scriptRuntimeCapabilityAbi = 3u;

using ScriptRuntimeHandle = std::uint32_t;
// Handles are never reused; 0 is reserved so "no runtime" stays unambiguous.
inline constexpr ScriptRuntimeHandle invalidScriptRuntime = 0;

using ScriptRuntimeInitializerHandle                                            = std::uint64_t;
inline constexpr ScriptRuntimeInitializerHandle invalidScriptRuntimeInitializer = 0;

// Called once for every runtime, including runtimes created before registration.
// Initializers may register native functions through ScriptRuntimeCapability.
using ScriptRuntimeInitializer = bool (*)(void *userData, ScriptRuntimeHandle runtime) noexcept;

// Coarse failure classification for execute/call functions. Their calling
// convention: the boolean return reports usage problems (bad handles, bad
// arguments), the ScriptError out-parameter reports compilation/runtime
// failures.
enum class ScriptError : std::uint32_t
{
    None = 0,
    InvalidHandle,
    InvalidArgument,
    CapacityExceeded,
    MemoryLimitExceeded,
    InstructionLimitExceeded,
    CompilationFailed,
    RuntimeFailed,
    IoFailed,
    TypeMismatch
};

// The only value types a script and native code exchange.
enum class ScriptValueType : std::uint32_t
{
    Nil = 0,
    Boolean,
    Number,
    String
};

// A primitive value exchanged with a script. Strings are borrowed: stringData
// points into Lua-owned memory (arguments) or caller memory (results) and is
// only valid for the duration of the call.
struct ScriptValue
{
    ScriptValueType type         = ScriptValueType::Nil;
    bool            booleanValue = false;
    double          numberValue  = 0.0;
    const char     *stringData   = nullptr;
    std::uint32_t   stringSize   = 0;
};

using ScriptNativeFunction = bool (*)(void *userData, const ScriptValue *arguments, std::uint32_t argumentCount,
    ScriptValue *results, std::uint32_t resultCapacity, std::uint32_t *outResultCount);

enum ScriptLibrary : std::uint32_t
{
    ScriptLibraryMath      = 1u << 0u,
    ScriptLibraryString    = 1u << 1u,
    ScriptLibraryTable     = 1u << 2u,
    ScriptLibraryUtf8      = 1u << 3u,
    ScriptLibraryCoroutine = 1u << 4u
};

struct ScriptRuntimeDescription
{
    std::uint64_t memoryLimitBytes   = 64ull * 1024ull * 1024ull;
    std::uint64_t instructionLimit   = 10'000'000ull;
    std::uint64_t maximumScriptBytes = 4ull * 1024ull * 1024ull;
    std::uint32_t enabledLibraries   = ScriptLibraryMath | ScriptLibraryString | ScriptLibraryTable | ScriptLibraryUtf8;
};

struct ScriptRuntimeInfo
{
    std::uint64_t memoryUsedBytes    = 0;
    std::uint64_t memoryLimitBytes   = 0;
    std::uint64_t instructionLimit   = 0;
    std::uint64_t maximumScriptBytes = 0;
};

struct ScriptRuntimeCapability
{
    static constexpr const char *capabilityName = "script_runtime";
    std::uint32_t                abi            = scriptRuntimeCapabilityAbi;
    void                        *self           = nullptr;

    ScriptRuntimeHandle (*createRuntime)(void *, const ScriptRuntimeDescription *, ScriptError *);
    void (*destroyRuntime)(void *, ScriptRuntimeHandle);
    bool (*runtimeInfo)(void *, ScriptRuntimeHandle, ScriptRuntimeInfo *);
    bool (*executeString)(void *, ScriptRuntimeHandle, const char *source, const char *chunkName, ScriptError *,
        char *errorText, std::uint32_t errorCapacity, std::uint32_t *errorRequired);
    bool (*executeFile)(void *, ScriptRuntimeHandle, const char *filePath, ScriptError *, char *errorText,
        std::uint32_t errorCapacity, std::uint32_t *errorRequired);
    bool (*callFunction)(void *, ScriptRuntimeHandle, const char *functionName, ScriptError *, char *errorText,
        std::uint32_t errorCapacity, std::uint32_t *errorRequired);
    bool (*registerNativeFunction)(void *, ScriptRuntimeHandle, const char *name, ScriptNativeFunction, void *userData);

    ScriptRuntimeHandle (*hostedRuntime)(void *, const char *pluginId);
    bool (*setGlobalNumber)(void *, ScriptRuntimeHandle, const char *name, double value);
    bool (*getGlobalNumber)(void *, ScriptRuntimeHandle, const char *name, double *value);
    bool (*setGlobalString)(void *, ScriptRuntimeHandle, const char *name, const char *value);
    bool (*getGlobalString)(void *, ScriptRuntimeHandle, const char *name, char *output, std::uint32_t capacity,
        std::uint32_t *outRequired);
    bool (*collectGarbage)(void *, ScriptRuntimeHandle);

    // Optional module hooks. Registering an initializer also applies it to
    // runtimes that already exist; unregister prevents future applications.
    ScriptRuntimeInitializerHandle (*registerRuntimeInitializer)(
        void *, ScriptRuntimeInitializer initializer, void *userData);
    bool (*unregisterRuntimeInitializer)(void *, ScriptRuntimeInitializerHandle initializer);
};

// Optional-module helper for installing a bridge initializer when a compatible
// script runtime provider is present.
inline bool canRegisterRuntimeInitializer(const ScriptRuntimeCapability *capability) noexcept
{
    return capability != nullptr && capability->abi == scriptRuntimeCapabilityAbi
        && capability->registerRuntimeInitializer != nullptr;
}

inline ScriptRuntimeInitializerHandle tryRegisterRuntimeInitializer(
    ScriptRuntimeCapability *capability, ScriptRuntimeInitializer initializer, void *userData)
{
    if (!canRegisterRuntimeInitializer(capability) || initializer == nullptr)
    {
        return invalidScriptRuntimeInitializer;
    }
    return capability->registerRuntimeInitializer(capability->self, initializer, userData);
}

// Clears the local handle even when the provider has already shut down or the
// initializer was removed by the runtime.
inline void unregisterRuntimeInitializer(
    ScriptRuntimeCapability *capability, ScriptRuntimeInitializerHandle &initializer)
{
    if (initializer == invalidScriptRuntimeInitializer)
    {
        return;
    }
    if (capability != nullptr && capability->abi == scriptRuntimeCapabilityAbi
        && capability->unregisterRuntimeInitializer != nullptr)
    {
        capability->unregisterRuntimeInitializer(capability->self, initializer);
    }
    initializer = invalidScriptRuntimeInitializer;
}

} // namespace engine::script_runtime

#endif
