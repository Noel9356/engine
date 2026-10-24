// The module plugin seam of 'persistence_toml': one exported entry point that
// hands the kernel this binary's static descriptor (engine/kernel/module_api.hpp).
#include "engine/kernel/module_api.hpp"

#include "persistence.hpp"

namespace
{
const char *const persistenceProvidedCapabilities[] = {"persistence.toml"};
const char *const persistenceRequiredCapabilities[] = {"logging", "scene"};

engine::ModuleInterface *createpersistencePluginModule()
{
    return new engine::PersistenceModule();
}

const engine::EngineModulePluginEntry persistencePluginEntry{
    .abiVersion              = engine::engineModulePluginAbiVersion,
    .moduleName              = "persistence_toml",
    .createModule            = &createpersistencePluginModule,
    .providedCapabilities    = persistenceProvidedCapabilities,
    .providedCapabilityCount = 1,
    .requiredCapabilities    = persistenceRequiredCapabilities,
    .requiredCapabilityCount = 2
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &persistencePluginEntry;
}
