// The module plugin seam of 'screenshot_png': one exported entry point that hands
// the kernel this binary's static descriptor (engine/kernel/module_api.hpp).
// The descriptor carries the module's name, its factory and its static
// capability declaration; the kernel reads all three without creating an
// instance, and only instantiates the module when the demand closure requires
// one of its capabilities.
#include "engine/kernel/module_api.hpp"

#include <iterator>

#include "screenshot.hpp"

namespace
{
const char *const screenshotProvidedCapabilities[] = {"screenshot"};
const char *const screenshotRequiredCapabilities[] = {"logging", "renderer"};

engine::ModuleInterface *createscreenshotPluginModule()
{
    return new engine::ScreenshotModule();
}

const engine::EngineModulePluginEntry screenshotPluginEntry{
    .abiVersion              = engine::engineModulePluginAbiVersion,
    .moduleName              = "screenshot",
    .createModule            = &createscreenshotPluginModule,
    .providedCapabilities    = screenshotProvidedCapabilities,
    .providedCapabilityCount = std::size(screenshotProvidedCapabilities),
    .requiredCapabilities    = screenshotRequiredCapabilities,
    .requiredCapabilityCount = std::size(screenshotRequiredCapabilities),
    .optionalCapabilities    = nullptr,
    .optionalCapabilityCount = 0
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &screenshotPluginEntry;
}
