// The module plugin seam of 'renderer': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp). The
// kernel dlopens the library from the modules directory and registers the
// factory under the descriptor's module name.
#include "engine/kernel/module_api.hpp"

#include "renderer.hpp"

namespace
{
engine::ModuleInterface *createrendererPluginModule()
{
    return new engine::RendererModule();
}

const engine::EngineModulePluginEntry rendererPluginEntry{
    engine::engineModulePluginAbiVersion, "renderer", &createrendererPluginModule
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &rendererPluginEntry;
}
