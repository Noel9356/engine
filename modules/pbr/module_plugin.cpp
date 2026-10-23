// The module plugin seam of 'pbr': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp). The
// descriptor carries the module's name, its factory and its static capability
// declaration; the kernel reads all three without creating an instance, and
// only instantiates the module when the demand closure requires one of its
// capabilities.
#include "engine/kernel/module_api.hpp"

#include "pbr.hpp"

namespace
{
const char *const pbrProvidedCapabilities[] = {"lighting", "direct_diffuse"};
const char *const pbrRequiredCapabilities[] = {"logging", "renderer"};

engine::ModuleInterface *createpbrPluginModule()
{
    return new engine::PbrLightingModule();
}

const engine::EngineModulePluginEntry pbrPluginEntry{
    engine::engineModulePluginAbiVersion,
    "pbr",
    &createpbrPluginModule,
    pbrProvidedCapabilities,
    2,
    pbrRequiredCapabilities,
    2
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &pbrPluginEntry;
}
