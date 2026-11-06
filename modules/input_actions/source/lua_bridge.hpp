#ifndef ENGINE_INPUT_ACTIONS_LUA_BRIDGE_HPP
#define ENGINE_INPUT_ACTIONS_LUA_BRIDGE_HPP

#include <engine/input_actions/capabilities.hpp>
#include <engine/script_runtime/capabilities.hpp>

#include <mutex>

namespace engine::input_actions
{

struct InputActionsLuaBridgeContext
{
    script_runtime::ScriptRuntimeCapability *scriptRuntime = nullptr;
    InputActionsCapability *inputActions = nullptr;
    std::mutex apiMutex;
};

bool initializeInputActionsLuaRuntime(
    void *userData, script_runtime::ScriptRuntimeHandle runtime) noexcept;

} // namespace engine::input_actions

#endif
