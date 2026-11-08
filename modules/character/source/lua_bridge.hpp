// Lua bridge of the character module.
//
// Exposes the assembly operations to script runtimes as "character.*"
// natives, following the same runtime-initializer pattern as the camera and
// input_actions modules. Values crossing the boundary are the four primitive
// script types; handles travel as numbers.

#ifndef ENGINE_CHARACTER_LUA_BRIDGE_HPP
#define ENGINE_CHARACTER_LUA_BRIDGE_HPP

#include <engine/character/capabilities.hpp>
#include <engine/script_runtime/capabilities.hpp>

#include <mutex>

namespace engine::character
{

// Shared state handed to every native function: the capability table plus the
// mutex guarding it. The script runtime dispatches on the main thread, but
// the bridge does not assume it.
struct CharacterLuaBridgeContext
{
    script_runtime::ScriptRuntimeCapability *scriptRuntime = nullptr;
    CharacterCapability                     *character     = nullptr;
    // Recursive: bridge natives lock it and then call capability entries
    // that lock the same mutex.
    std::recursive_mutex *mutex = nullptr;
};

// Runtime initializer registered with script_runtime: installs the
// "character.*" natives into every runtime (existing and future). The void*
// userData is a CharacterLuaBridgeContext.
bool initializeCharacterLuaRuntime(void *userData, script_runtime::ScriptRuntimeHandle runtime) noexcept;

} // namespace engine::character

#endif
