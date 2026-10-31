#ifndef ENGINE_KERNEL_MODULE_API_HPP
#define ENGINE_KERNEL_MODULE_API_HPP

#include <engine/kernel/component_interface.hpp>

#include <cstdint>

namespace engine
{

inline constexpr std::uint32_t engineModulePluginAbiVersion = 7u;

struct EngineModulePluginEntry
{
    std::uint32_t abiVersion;

    const char *moduleName;

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
#define ENGINE_MODULE_EXPORT extern "C" __declspec(dllexport)
#else
#define ENGINE_MODULE_EXPORT extern "C" __attribute__((visibility("default")))
#endif

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin();

#endif
