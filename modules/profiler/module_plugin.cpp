// The module plugin seam of 'profiler': one exported entry point that hands
// the kernel this binary's static descriptor (engine/kernel/module_api.hpp).
// The descriptor carries the module's name, its factory and its static
// capability declaration; the kernel reads all three without creating an
// instance, and only instantiates the module when the demand closure requires
// one of its capabilities. The descriptor name must equal the name the
// module's manifest reports - the kernel verifies the agreement at startup
// and refuses a drifting plugin with a precise diagnostic.
#include "engine/kernel/module_api.hpp"

#include <iterator>

#include "profiler.hpp"

namespace
{
const char *const profilerProvidedCapabilities[] = {"profiler"};
const char *const profilerRequiredCapabilities[] = {"logging"};

engine::ModuleInterface *createprofilerPluginModule()
{
    return new engine::ProfilerModule();
}

const engine::EngineModulePluginEntry profilerPluginEntry{
    .abiVersion              = engine::engineModulePluginAbiVersion,
    .moduleName              = "profiler",
    .createModule            = &createprofilerPluginModule,
    .providedCapabilities    = profilerProvidedCapabilities,
    .providedCapabilityCount = std::size(profilerProvidedCapabilities),
    .requiredCapabilities    = profilerRequiredCapabilities,
    .requiredCapabilityCount = std::size(profilerRequiredCapabilities),
    .optionalCapabilities    = nullptr,
    .optionalCapabilityCount = 0
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &profilerPluginEntry;
}
