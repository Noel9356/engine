// The module plugin seam of 'window': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp). The
// descriptor carries the module's name, its factory and its static capability
// declaration; the kernel reads all three without creating an instance, and
// only instantiates the module when the demand closure requires one of its
// capabilities.
#include "engine/kernel/module_api.hpp"

#include "window.hpp"

namespace
{
const char *const windowProvidedCapabilities[] = {"window.glfw"};
const char *const windowRequiredCapabilities[] = {"logging"};

engine::ModuleInterface *createwindowPluginModule()
{
    return new engine::WindowModule();
}

const engine::EngineModulePluginEntry windowPluginEntry{
    engine::engineModulePluginAbiVersion,
    "window",
    &createwindowPluginModule,
    windowProvidedCapabilities,
    1,
    windowRequiredCapabilities,
    1
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &windowPluginEntry;
}
