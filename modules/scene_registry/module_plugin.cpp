// The module plugin seam of 'scene_registry': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp).
#include "engine/kernel/module_api.hpp"

#include <iterator>

#include "scene.hpp"

namespace
{
const char *const sceneProvidedCapabilities[] = {"scene.registry"};
const char *const sceneRequiredCapabilities[] = {"logging"};

engine::ModuleInterface *createscenePluginModule()
{
    return new engine::SceneModule();
}

const engine::EngineModulePluginEntry scenePluginEntry{
    .abiVersion              = engine::engineModulePluginAbiVersion,
    .moduleName              = "scene",
    .createModule            = &createscenePluginModule,
    .providedCapabilities    = sceneProvidedCapabilities,
    .providedCapabilityCount = std::size(sceneProvidedCapabilities),
    .requiredCapabilities    = sceneRequiredCapabilities,
    .requiredCapabilityCount = std::size(sceneRequiredCapabilities),
    .optionalCapabilities    = nullptr,
    .optionalCapabilityCount = 0
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &scenePluginEntry;
}
