// The module plugin seam of 'lua_scripting': one exported entry point that
// hands the kernel this binary's static descriptor (engine/kernel/module_api.hpp).
// The descriptor carries the module's name, its factory and its static
// capability declaration; the kernel reads all three without creating an
// instance, and only instantiates the module when the demand closure requires
// one of its capabilities - for this module, that happens exactly when a
// script mod requires the scripting capability it provides.
#include "engine/kernel/module_api.hpp"

#include "lua_scripting.hpp"

namespace
{
const char *const lua_scriptingProvidedCapabilities[] = {"scripting.lua"};
const char *const lua_scriptingRequiredCapabilities[] = {"logging", "window"};

engine::ModuleInterface *createlua_scriptingPluginModule()
{
    return new engine::LuaScriptingModule();
}

const engine::EngineModulePluginEntry lua_scriptingPluginEntry{
    .abiVersion              = engine::engineModulePluginAbiVersion,
    .moduleName              = "lua_scripting",
    .createModule            = &createlua_scriptingPluginModule,
    .providedCapabilities    = lua_scriptingProvidedCapabilities,
    .providedCapabilityCount = 1,
    .requiredCapabilities    = lua_scriptingRequiredCapabilities,
    .requiredCapabilityCount = 2
};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry *engineQueryModulePlugin()
{
    return &lua_scriptingPluginEntry;
}
