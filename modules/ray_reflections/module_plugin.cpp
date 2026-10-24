// The module plugin seam of 'ray_reflections': one exported entry point that
// hands the kernel this binary's static descriptor (engine/kernel/module_api.hpp).
// The descriptor carries the module's name, its factory and its static
// capability declaration; the kernel reads all three without creating an
// instance, and only instantiates the module when the demand closure requires
// one of its capabilities.
#include "engine/kernel/module_api.hpp"

#include <iterator>

#include "ray_reflections.hpp"

namespace
{
const char *const ray_reflectionsProvidedCapabilities[] = {"indirect_specular"};
const char *const ray_reflectionsRequiredCapabilities[] = {"logging", "renderer"};
// "lighting" is used when a provider is active and reflections carry
// emissive + sky without one (see the module manifest).
const char *const ray_reflectionsOptionalCapabilities[] = {"lighting"};

engine::ModuleInterface *createray_reflectionsPluginModule()
{
    return new engine::RayReflectionsModule();
}

const engine::EngineModulePluginEntry ray_reflectionsPluginEntry{
    .abiVersion              = engine::engineModulePluginAbiVersion,
    .moduleName              = "ray_reflections",
    .createModule            = &createray_reflectionsPluginModule,
    .providedCapabilities    = ray_reflectionsProvidedCapabilities,
    .providedCapabilityCount = std::size(ray_reflectionsProvidedCapabilities),
    .requiredCapabilities    = ray_reflectionsRequiredCapabilities,
    .requiredCapabilityCount = std::size(ray_reflectionsRequiredCapabilities),
    .optionalCapabilities    = ray_reflectionsOptionalCapabilities,
    .optionalCapabilityCount = std::size(ray_reflectionsOptionalCapabilities)
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &ray_reflectionsPluginEntry;
}
