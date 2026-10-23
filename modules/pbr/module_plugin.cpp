// The module plugin seam of 'pbr': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp). The
// kernel dlopens the library from the modules directory and registers the
// factory under the descriptor's module name.
#include "engine/kernel/module_api.hpp"

#include "pbr.hpp"

namespace
{
engine::ModuleInterface* createpbrPluginModule()
{
    return new engine::PbrLightingModule();
}

const engine::EngineModulePluginEntry pbrPluginEntry{
    engine::engineModulePluginAbiVersion, "pbr", &createpbrPluginModule};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry* engineQueryModulePlugin()
{
    return &pbrPluginEntry;
}
