// The module plugin seam of 'input': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp). The
// kernel dlopens the library from the modules directory and registers the
// factory under the descriptor's module name.
#include "engine/kernel/module_api.hpp"

#include "input.hpp"

namespace
{
engine::ModuleInterface *createinputPluginModule()
{
    return new engine::InputModule();
}

const engine::EngineModulePluginEntry inputPluginEntry{
    engine::engineModulePluginAbiVersion, "input", &createinputPluginModule
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &inputPluginEntry;
}
