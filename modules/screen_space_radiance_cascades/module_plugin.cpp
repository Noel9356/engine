// The module plugin seam of 'screen_space_radiance_cascades': one exported entry point that
// hands the kernel this binary's static descriptor (engine/kernel/module_api.hpp).
// The descriptor carries the module's name, its factory and its static
// capability declaration; the kernel reads all three without creating an
// instance, and only instantiates the module when the demand closure requires
// one of its capabilities.
#include "engine/kernel/module_api.hpp"

#include "screen_space_radiance_cascades.hpp"

namespace
{
const char *const screen_space_radiance_cascadesProvidedCapabilities[] = {"indirect_diffuse"};
const char *const screen_space_radiance_cascadesRequiredCapabilities[] = {"logging", "renderer"};
// "lighting" is used when a provider is active and the pass degrades to
// emissive + sky transport without one (see the module manifest).
const char *const screen_space_radiance_cascadesOptionalCapabilities[] = {"lighting"};

engine::ModuleInterface *createscreen_space_radiance_cascadesPluginModule()
{
    return new engine::ScreenSpaceRadianceCascadesModule();
}

const engine::EngineModulePluginEntry screen_space_radiance_cascadesPluginEntry{
    .abiVersion              = engine::engineModulePluginAbiVersion,
    .moduleName              = "screen_space_radiance_cascades",
    .createModule            = &createscreen_space_radiance_cascadesPluginModule,
    .providedCapabilities    = screen_space_radiance_cascadesProvidedCapabilities,
    .providedCapabilityCount = 1,
    .requiredCapabilities    = screen_space_radiance_cascadesRequiredCapabilities,
    .requiredCapabilityCount = 2,
    .optionalCapabilities    = screen_space_radiance_cascadesOptionalCapabilities,
    .optionalCapabilityCount = 1
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &screen_space_radiance_cascadesPluginEntry;
}
