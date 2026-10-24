// The module plugin seam of 'physics_builtin': one exported entry point that hands
// the kernel this binary's static descriptor (engine/kernel/module_api.hpp).
#include "engine/kernel/module_api.hpp"

#include "physics.hpp"

namespace
{
const char *const physicsProvidedCapabilities[] = {"physics.builtin"};
const char *const physicsRequiredCapabilities[] = {"logging", "scene", "bvh"};

engine::ModuleInterface *createphysicsPluginModule()
{
    return new engine::PhysicsModule();
}

const engine::EngineModulePluginEntry physicsPluginEntry{
    .abiVersion              = engine::engineModulePluginAbiVersion,
    .moduleName              = "physics_builtin",
    .createModule            = &createphysicsPluginModule,
    .providedCapabilities    = physicsProvidedCapabilities,
    .providedCapabilityCount = 1,
    .requiredCapabilities    = physicsRequiredCapabilities,
    .requiredCapabilityCount = 2
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &physicsPluginEntry;
}
