// The module plugin seam of 'logging': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp). The
// kernel dlopens the library from the modules directory and registers the
// factory under the descriptor's module name.
#include "engine/kernel/module_api.hpp"

#include "logging.hpp"

namespace
{
engine::ModuleInterface* createloggingPluginModule()
{
    return new engine::LoggingModule();
}

const engine::EngineModulePluginEntry loggingPluginEntry{
    engine::engineModulePluginAbiVersion, "logging", &createloggingPluginModule};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry* engineQueryModulePlugin()
{
    return &loggingPluginEntry;
}
