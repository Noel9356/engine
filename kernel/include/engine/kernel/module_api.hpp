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
inline constexpr std::uint32_t engineModulePluginAbiVersion = 1u;

// The descriptor every module plugin returns from its single exported entry
// point. Plain data and plain function pointers: nothing that crosses the
// library boundary assumes more than one heap for the whole process (one CRT
// on Windows, a shared libstdc++ on POSIX), which this build guarantees.
struct EngineModulePluginEntry
{
    // Must equal engineModulePluginAbiVersion at the time of loading.
    std::uint32_t abiVersion;
    // The configuration name of the module ("window", "renderer"), exactly
    // what the composition root used to pass to registerModuleFactory.
    const char *moduleName;
    // Creates one module instance. The kernel owns the returned object and
    // releases it with delete: one process, one heap.
    ModuleInterface *(*createModule)();
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
