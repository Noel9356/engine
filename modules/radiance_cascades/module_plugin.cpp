// The module plugin seam of 'radiance_cascades': one exported entry point that
// hands the kernel this binary's static descriptor (engine/kernel/module_api.hpp).
// The descriptor carries the module's name, its factory and its static
// capability declaration; the kernel reads all three without creating an
// instance, and only instantiates the module when the demand closure requires
// one of its capabilities.
#include "engine/kernel/module_api.hpp"

#include "radiance_cascades.hpp"

namespace
{
const char *const radiance_cascadesProvidedCapabilities[] = {"indirect_diffuse"};
const char *const radiance_cascadesRequiredCapabilities[] = {"logging", "renderer"};

engine::ModuleInterface *createradiance_cascadesPluginModule()
{
    return new engine::RadianceCascadesModule();
}

const engine::EngineModulePluginEntry radiance_cascadesPluginEntry{
    .abiVersion              = engine::engineModulePluginAbiVersion,
    .moduleName              = "radiance_cascades",
    .createModule            = &createradiance_cascadesPluginModule,
    .providedCapabilities    = radiance_cascadesProvidedCapabilities,
    .providedCapabilityCount = 1,
    .requiredCapabilities    = radiance_cascadesRequiredCapabilities,
    .requiredCapabilityCount = 2
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &radiance_cascadesPluginEntry;
}
