// The module plugin seam of 'radiance_cascades': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp). The
// kernel dlopens the library from the modules directory and registers the
// factory under the descriptor's module name.
#include "engine/kernel/module_api.hpp"

#include "radiance_cascades.hpp"

namespace
{
engine::ModuleInterface* createradiance_cascadesPluginModule()
{
    return new engine::RadianceCascadesModule();
}

const engine::EngineModulePluginEntry radiance_cascadesPluginEntry{
    engine::engineModulePluginAbiVersion, "radiance_cascades", &createradiance_cascadesPluginModule};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry* engineQueryModulePlugin()
{
    return &radiance_cascadesPluginEntry;
}
