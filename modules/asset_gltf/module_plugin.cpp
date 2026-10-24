// The module plugin seam of 'asset_gltf': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp).
#include "engine/kernel/module_api.hpp"

#include "asset.hpp"

namespace
{
const char *const assetProvidedCapabilities[] = {"asset.gltf"};
const char *const assetRequiredCapabilities[] = {"logging", "scene"};

engine::ModuleInterface *createassetPluginModule()
{
    return new engine::AssetModule();
}

const engine::EngineModulePluginEntry assetPluginEntry{
    .abiVersion              = engine::engineModulePluginAbiVersion,
    .moduleName              = "asset_gltf",
    .createModule            = &createassetPluginModule,
    .providedCapabilities    = assetProvidedCapabilities,
    .providedCapabilityCount = 1,
    .requiredCapabilities    = assetRequiredCapabilities,
    .requiredCapabilityCount = 2
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &assetPluginEntry;
}
