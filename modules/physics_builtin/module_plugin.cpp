// The module plugin seam of 'physics_builtin': one exported entry point that hands
// the kernel this binary's static descriptor (engine/kernel/module_api.hpp).
#include "engine/kernel/module_api.hpp"

#include <iterator>

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
    .moduleName              = "physics",
    .createModule            = &createphysicsPluginModule,
    .providedCapabilities    = physicsProvidedCapabilities,
    .providedCapabilityCount = std::size(physicsProvidedCapabilities),
    .requiredCapabilities    = physicsRequiredCapabilities,
    .requiredCapabilityCount = std::size(physicsRequiredCapabilities),
    .optionalCapabilities    = nullptr,
    .optionalCapabilityCount = 0
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &physicsPluginEntry;
}
