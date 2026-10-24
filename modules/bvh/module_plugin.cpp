// The module plugin seam of 'bvh': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp).
#include "engine/kernel/module_api.hpp"

#include "bvh.hpp"

namespace
{
const char *const bvhProvidedCapabilities[] = {"bvh.binary"};
const char *const bvhRequiredCapabilities[] = {"logging"};

engine::ModuleInterface *createbvhPluginModule()
{
    return new engine::BvhModule();
}

const engine::EngineModulePluginEntry bvhPluginEntry{
    .abiVersion              = engine::engineModulePluginAbiVersion,
    .moduleName              = "bvh",
    .createModule            = &createbvhPluginModule,
    .providedCapabilities    = bvhProvidedCapabilities,
    .providedCapabilityCount = 1,
    .requiredCapabilities    = bvhRequiredCapabilities,
    .requiredCapabilityCount = 1
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &bvhPluginEntry;
}
