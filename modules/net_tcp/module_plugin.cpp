// The module plugin seam of 'net_tcp': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp).
#include "engine/kernel/module_api.hpp"

#include <iterator>

#include "net.hpp"

namespace
{
const char *const netProvidedCapabilities[] = {"net.tcp"};
const char *const netRequiredCapabilities[] = {"logging"};

engine::ModuleInterface *createnetPluginModule()
{
    return new engine::NetModule();
}

const engine::EngineModulePluginEntry netPluginEntry{
    .abiVersion              = engine::engineModulePluginAbiVersion,
    .moduleName              = "net",
    .createModule            = &createnetPluginModule,
    .providedCapabilities    = netProvidedCapabilities,
    .providedCapabilityCount = std::size(netProvidedCapabilities),
    .requiredCapabilities    = netRequiredCapabilities,
    .requiredCapabilityCount = std::size(netRequiredCapabilities),
    .optionalCapabilities    = nullptr,
    .optionalCapabilityCount = 0
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &netPluginEntry;
}
