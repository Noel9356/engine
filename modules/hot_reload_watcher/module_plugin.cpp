// The module plugin seam of 'hot_reload_watcher': one exported entry point that hands
// the kernel this binary's static descriptor (engine/kernel/module_api.hpp).
#include "engine/kernel/module_api.hpp"

#include "hot_reload.hpp"

namespace
{
const char *const hotReloadProvidedCapabilities[] = {"hot_reload.watcher"};
const char *const hotReloadRequiredCapabilities[] = {"logging"};

engine::ModuleInterface *createhot_reloadPluginModule()
{
    return new engine::HotReloadModule();
}

const engine::EngineModulePluginEntry hotReloadPluginEntry{
    .abiVersion              = engine::engineModulePluginAbiVersion,
    .moduleName              = "hot_reload_watcher",
    .createModule            = &createhot_reloadPluginModule,
    .providedCapabilities    = hotReloadProvidedCapabilities,
    .providedCapabilityCount = 1,
    .requiredCapabilities    = hotReloadRequiredCapabilities,
    .requiredCapabilityCount = 1
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &hotReloadPluginEntry;
}
