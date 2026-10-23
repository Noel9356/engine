// The module plugin seam of 'ray_reflections': one exported entry point that
// hands the kernel this binary's static descriptor (engine/kernel/module_api.hpp).
// The descriptor carries the module's name, its factory and its static
// capability declaration; the kernel reads all three without creating an
// instance, and only instantiates the module when the demand closure requires
// one of its capabilities.
#include "engine/kernel/module_api.hpp"

#include "ray_reflections.hpp"

namespace
{
const char *const ray_reflectionsProvidedCapabilities[] = {"indirect_specular"};
const char *const ray_reflectionsRequiredCapabilities[] = {"logging", "renderer"};

engine::ModuleInterface *createray_reflectionsPluginModule()
{
    return new engine::RayReflectionsModule();
}

const engine::EngineModulePluginEntry ray_reflectionsPluginEntry{
    engine::engineModulePluginAbiVersion,
    "ray_reflections",
    &createray_reflectionsPluginModule,
    ray_reflectionsProvidedCapabilities,
    1,
    ray_reflectionsRequiredCapabilities,
    2
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &ray_reflectionsPluginEntry;
}
