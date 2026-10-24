// The module plugin seam of 'input_glfw': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp). The
// descriptor carries the module's name, its factory and its static capability
// declaration; the kernel reads all three without creating an instance, and
// only instantiates the module when the demand closure requires one of its
// capabilities.
#include "engine/kernel/module_api.hpp"

#include "input.hpp"

namespace
{
const char *const inputProvidedCapabilities[] = {"input.glfw"};
const char *const inputRequiredCapabilities[] = {"logging", "window"};

engine::ModuleInterface *createinputPluginModule()
{
    return new engine::InputModule();
}

const engine::EngineModulePluginEntry inputPluginEntry{
    .abiVersion              = engine::engineModulePluginAbiVersion,
    .moduleName              = "input_glfw",
    .createModule            = &createinputPluginModule,
    .providedCapabilities    = inputProvidedCapabilities,
    .providedCapabilityCount = 1,
    .requiredCapabilities    = inputRequiredCapabilities,
    .requiredCapabilityCount = 2
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &inputPluginEntry;
}
