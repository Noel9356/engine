#ifndef ENGINE_KERNEL_MODULE_API_HPP
#define ENGINE_KERNEL_MODULE_API_HPP

#include "engine/kernel/component_interface.hpp"

#include <cstdint>

namespace engine
{

// ABI version of the module plugin entry point. The kernel refuses any plugin
// whose reported version differs: a rebuilt kernel combined with stale plugin
// binaries from an older interface must fail with a precise diagnostic, never
// crash inside a mismatched vtable.
//
// Version 2 added the static capability declaration (providedCapabilities /
// requiredCapabilities): the kernel resolves the capability demand closure
// from this metadata BEFORE instantiating anything, so a module whose
// capabilities nobody requires is never created and its binary is released.
inline constexpr std::uint32_t engineModulePluginAbiVersion = 2u;

// The descriptor every module plugin returns from its single exported entry
// point. Plain data and plain function pointers: nothing that crosses the
// library boundary assumes more than one heap for the whole process (one CRT
// on Windows, a shared libstdc++ on POSIX), which this build guarantees.
struct EngineModulePluginEntry
{
    // Must equal engineModulePluginAbiVersion at the time of loading.
    std::uint32_t abiVersion;
    // The module's own name, exactly what the module's manifest reports.
    const char *moduleName;
    // Creates one module instance. The kernel owns the returned object and
    // releases it with delete: one process, one heap.
    ModuleInterface *(*createModule)();
    // Static capability declaration, read before any instantiation. Each
    // array holds plain string literals with static storage duration (or is
    // null when the count is zero). The kernel verifies, after creating an
    // instance, that the instance manifest agrees with this declaration: a
    // mismatch is a precise startup problem, never silent drift.
    const char *const *providedCapabilities;
    std::uint32_t      providedCapabilityCount;
    const char *const *requiredCapabilities;
    std::uint32_t      requiredCapabilityCount;
};

} // namespace engine

// The one exported symbol of a module plugin. Every module binary defines it
// through its module_plugin.cpp translation unit; the kernel resolves it by
// name after dlopen and never links a plugin's import library.
#if defined(_WIN32)
#define ENGINE_MODULE_EXPORT extern "C" __declspec(dllexport)
#else
#define ENGINE_MODULE_EXPORT extern "C" __attribute__((visibility("default")))
#endif

// Returns the static plugin descriptor of this module binary. The pointed-to
// object must have static storage duration.
ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin();

#endif // ENGINE_KERNEL_MODULE_API_HPP
