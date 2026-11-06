// C ABI boundary for native engine modules (.so entry table).
//
// A module library exports engineQueryModuleEntryTable returning this
// table; strings and arrays must stay valid for the library's lifetime.

#ifndef ENGINE_KERNEL_MODULE_API_HPP
#define ENGINE_KERNEL_MODULE_API_HPP

#include <engine/kernel/participant_interface.hpp>

#include <cstdint>

namespace engine
{

// Bump when the kernel-participant ABI changes; mismatched libraries are
// refused with a rebuild hint.
inline constexpr std::uint32_t engineModuleAbiVersion = 8u;

// Filled by the module author; the counts match the array lengths.
struct EngineModuleEntryTable
{
    // Must equal engineModuleAbiVersion; the kernel refuses the library
    // otherwise.
    std::uint32_t abiVersion;

    const char *moduleName;

    const char *moduleVersion;

    ModuleInterface *(*createModule)();

    const char *const *providedCapabilities;
    std::uint32_t      providedCapabilityCount;
    const char *const *requiredCapabilities;
    std::uint32_t      requiredCapabilityCount;

    const char *const *optionalCapabilities;
    std::uint32_t      optionalCapabilityCount;

    const char *const *declaredPermissions;
    std::uint32_t      declaredPermissionCount;
};

} // namespace engine

#if defined(_WIN32)
#define ENGINE_MODULE_EXPORT __declspec(dllexport)
#else
#define ENGINE_MODULE_EXPORT __attribute__((visibility("default")))
#endif

// Every module .so must export this symbol with C linkage.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable();

#endif
