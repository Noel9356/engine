// The module plugin seam of 'lua_scripting': one exported entry point that hands the
// kernel this binary's static descriptor (engine/kernel/module_api.hpp). The
// kernel dlopens the library from the modules directory and registers the
// factory under the descriptor's module name.
#include "engine/kernel/module_api.hpp"

#include "lua_scripting.hpp"

namespace
{
engine::ModuleInterface* createlua_scriptingPluginModule()
{
    return new engine::LuaScriptingModule();
}

const engine::EngineModulePluginEntry lua_scriptingPluginEntry{
    engine::engineModulePluginAbiVersion, "lua_scripting", &createlua_scriptingPluginModule};
} // namespace

ENGINE_MODULE_EXPORT const engine::EngineModulePluginEntry* engineQueryModulePlugin()
{
    return &lua_scriptingPluginEntry;
}
