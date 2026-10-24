// The module plugin seam of 'animation_tween': one exported entry point that hands
// the kernel this binary's static descriptor (engine/kernel/module_api.hpp).
#include "engine/kernel/module_api.hpp"

#include "animation.hpp"

namespace
{
const char *const animationProvidedCapabilities[] = {"animation.tweens"};
const char *const animationRequiredCapabilities[] = {"logging", "scene"};

engine::ModuleInterface *createanimationPluginModule()
{
    return new engine::AnimationModule();
}

const engine::EngineModulePluginEntry animationPluginEntry{
    .abiVersion              = engine::engineModulePluginAbiVersion,
    .moduleName              = "animation_tween",
    .createModule            = &createanimationPluginModule,
    .providedCapabilities    = animationProvidedCapabilities,
    .providedCapabilityCount = 1,
    .requiredCapabilities    = animationRequiredCapabilities,
    .requiredCapabilityCount = 2
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &animationPluginEntry;
}
