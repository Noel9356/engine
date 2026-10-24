// The module plugin seam of 'ui_imgui': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp).
#include "engine/kernel/module_api.hpp"

#include "ui.hpp"

namespace
{
const char *const uiProvidedCapabilities[] = {"ui.imgui"};
const char *const uiRequiredCapabilities[] = {"logging", "renderer.vulkan.overlay", "window", "input", "scene"};

engine::ModuleInterface *createuiPluginModule()
{
    return new engine::UiModule();
}

const engine::EngineModulePluginEntry uiPluginEntry{
    .abiVersion              = engine::engineModulePluginAbiVersion,
    .moduleName              = "ui_imgui",
    .createModule            = &createuiPluginModule,
    .providedCapabilities    = uiProvidedCapabilities,
    .providedCapabilityCount = 1,
    .requiredCapabilities    = uiRequiredCapabilities,
    .requiredCapabilityCount = 5
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &uiPluginEntry;
}
