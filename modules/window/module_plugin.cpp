// The module plugin seam of 'window': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp). The
// kernel dlopens the library from the modules directory and registers the
// factory under the descriptor's module name.
#include "engine/kernel/module_api.hpp"

#include "window.hpp"

namespace
{
engine::ModuleInterface *createwindowPluginModule()
{
    return new engine::WindowModule();
}

const engine::EngineModulePluginEntry windowPluginEntry{
    engine::engineModulePluginAbiVersion, "window", &createwindowPluginModule
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &windowPluginEntry;
}
