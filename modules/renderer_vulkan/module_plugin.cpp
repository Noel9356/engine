// The module plugin seam of 'renderer_vulkan': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp). The
// descriptor carries the module's name, its factory and its static capability
// declaration; the kernel reads all three without creating an instance, and
// only instantiates the module when the demand closure requires one of its
// capabilities.
#include "engine/kernel/module_api.hpp"

#include "renderer.hpp"

namespace
{
const char *const rendererProvidedCapabilities[] = {"renderer.vulkan", "renderer.vulkan.overlay"};
const char *const rendererRequiredCapabilities[] = {"logging", "window"};

engine::ModuleInterface *createrendererPluginModule()
{
    return new engine::RendererModule();
}

const engine::EngineModulePluginEntry rendererPluginEntry{
    .abiVersion              = engine::engineModulePluginAbiVersion,
    .moduleName              = "renderer",
    .createModule            = &createrendererPluginModule,
    .providedCapabilities    = rendererProvidedCapabilities,
    .providedCapabilityCount = 2,
    .requiredCapabilities    = rendererRequiredCapabilities,
    .requiredCapabilityCount = 2
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &rendererPluginEntry;
}
