#ifndef ENGINE_CAMERA_LUA_BRIDGE_HPP
#define ENGINE_CAMERA_LUA_BRIDGE_HPP

#include <engine/camera/capabilities.hpp>
#include <engine/script_runtime/capabilities.hpp>

#include <mutex>

namespace engine::camera
{

struct CameraLuaBridgeContext
{
    script_runtime::ScriptRuntimeCapability *scriptRuntime = nullptr;
    CameraCapability *camera = nullptr;
    std::mutex apiMutex;
};

bool initializeCameraLuaRuntime(void *userData, script_runtime::ScriptRuntimeHandle runtime) noexcept;

} // namespace engine::camera

#endif
