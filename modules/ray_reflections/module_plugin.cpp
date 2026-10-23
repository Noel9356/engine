// The module plugin seam of 'ray_reflections': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp). The
// kernel dlopens the library from the modules directory and registers the
// factory under the descriptor's module name.
#include "engine/kernel/module_api.hpp"

#include "ray_reflections.hpp"

namespace
{
engine::ModuleInterface *createray_reflectionsPluginModule()
{
    return new engine::RayReflectionsModule();
}

const engine::EngineModulePluginEntry ray_reflectionsPluginEntry{
    engine::engineModulePluginAbiVersion, "ray_reflections", &createray_reflectionsPluginModule
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &ray_reflectionsPluginEntry;
}
