// The module plugin seam of 'ui_imgui': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp).
#include "engine/kernel/module_api.hpp"

#include <iterator>

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
    .moduleName              = "ui",
    .createModule            = &createuiPluginModule,
    .providedCapabilities    = uiProvidedCapabilities,
    .providedCapabilityCount = std::size(uiProvidedCapabilities),
    .requiredCapabilities    = uiRequiredCapabilities,
    .requiredCapabilityCount = std::size(uiRequiredCapabilities),
    .optionalCapabilities    = nullptr,
    .optionalCapabilityCount = 0
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &uiPluginEntry;
}
