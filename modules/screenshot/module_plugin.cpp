// The module plugin seam of 'screenshot': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp). The
// kernel dlopens the library from the modules directory and registers the
// factory under the descriptor's module name.
#include "engine/kernel/module_api.hpp"

#include "screenshot.hpp"

namespace
{
engine::ModuleInterface* createscreenshotPluginModule()
{
    return new engine::ScreenshotModule();
}

const engine::EngineModulePluginEntry screenshotPluginEntry{
    engine::engineModulePluginAbiVersion, "screenshot", &createscreenshotPluginModule};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry* engineQueryModulePlugin()
{
    return &screenshotPluginEntry;
}
