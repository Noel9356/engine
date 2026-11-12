#include "lua_bridge.hpp"

#include <engine/script_runtime/native_values.hpp>

#include <cstdint>
#include <string_view>

namespace engine::camera
{
namespace
{
using script_runtime::ScriptNativeFunction;
using script_runtime::ScriptRuntimeHandle;
using script_runtime::ScriptValue;
using script_runtime::ScriptValueType;
using script_runtime::native::boolean;
using script_runtime::native::floatValue;
using script_runtime::native::resultBoolean;
using script_runtime::native::resultNil;
using script_runtime::native::resultNumber;
using script_runtime::native::stringValue;
using script_runtime::native::uint32Value;
using Bridge = CameraLuaBridgeContext;

CameraControlMode parseMode(std::string_view value, bool &valid)
{
    valid = true;
    if (value == "fixed") return CameraControlMode::Fixed;
    if (value == "orbit") return CameraControlMode::Orbit;
    if (value == "fly") return CameraControlMode::Fly;
    valid = false;
    return CameraControlMode::Fly;
}

ProjectionType parseProjection(std::string_view value, bool &valid)
{
    valid = true;
    if (value == "perspective") return ProjectionType::Perspective;
    if (value == "orthographic") return ProjectionType::Orthographic;
    valid = false;
    return ProjectionType::Perspective;
}

bool createCamera(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    if (count > 1 || capacity < 1)
        return false;

    CameraParameters parameters{};
    bool hasMode = false;
    CameraControlMode mode = CameraControlMode::Fly;
    if (count == 1)
    {
        std::string_view name;
        bool valid = false;
        if (!stringValue(args, count, 0, name))
            return false;
        mode = parseMode(name, valid);
        if (!valid)
            return false;
        hasMode = true;
    }

    std::lock_guard lock(bridge.apiMutex);
    if (bridge.camera == nullptr)
        return false;
    CameraError error = CameraError::None;
    const CameraHandle handle = bridge.camera->createCamera(bridge.camera->self, &parameters, &error);
    if (handle == invalidCamera)
        resultNil(results, 0);
    else
    {
        if (hasMode)
            bridge.camera->setControlMode(bridge.camera->self, handle, mode);
        resultNumber(results, 0, handle);
    }
    *outCount = 1;
    return true;
}

bool destroyCamera(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *,
    std::uint32_t, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::uint32_t handle = 0;
    if (count != 1 || !uint32Value(args, count, 0, handle))
        return false;
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.camera == nullptr)
        return false;
    bridge.camera->destroyCamera(bridge.camera->self, handle);
    *outCount = 0;
    return true;
}

bool setMode(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *,
    std::uint32_t, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::uint32_t handle = 0;
    std::string_view name;
    bool valid = false;
    if (count != 2 || !uint32Value(args, count, 0, handle)
        || !stringValue(args, count, 1, name))
        return false;
    const CameraControlMode mode = parseMode(name, valid);
    if (!valid)
        return false;
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.camera == nullptr)
        return false;
    bridge.camera->setControlMode(bridge.camera->self, handle, mode);
    *outCount = 0;
    return true;
}

bool lookAt(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *,
    std::uint32_t, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::uint32_t handle = 0;
    float values[9]{};
    if (count != 10 || !uint32Value(args, count, 0, handle))
        return false;
    for (std::uint32_t i = 0; i < 9; ++i)
        if (!floatValue(args, count, i + 1, values[i]))
            return false;
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.camera == nullptr)
        return false;
    bridge.camera->setLookAt(bridge.camera->self, handle, values, values + 3, values + 6);
    *outCount = 0;
    return true;
}

bool setProjection(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *,
    std::uint32_t, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::uint32_t handle = 0;
    std::string_view name;
    float vertical = 0.0f;
    float nearZ = 0.0f;
    float farZ = 0.0f;
    bool valid = false;
    if (count != 5 || !uint32Value(args, count, 0, handle)
        || !stringValue(args, count, 1, name) || !floatValue(args, count, 2, vertical)
        || !floatValue(args, count, 3, nearZ) || !floatValue(args, count, 4, farZ))
        return false;
    const ProjectionType type = parseProjection(name, valid);
    if (!valid)
        return false;
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.camera == nullptr)
        return false;
    bridge.camera->setProjection(bridge.camera->self, handle, type, vertical, nearZ, farZ);
    *outCount = 0;
    return true;
}

bool updateCamera(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *,
    std::uint32_t, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::uint32_t handle = 0;
    CameraInputState input{};
    float deltaSeconds = 0.0f;
    if (count != 11 || !uint32Value(args, count, 0, handle)
        || !floatValue(args, count, 1, input.moveForward) || !floatValue(args, count, 2, input.moveRight)
        || !floatValue(args, count, 3, input.moveUp) || !floatValue(args, count, 4, input.lookHorizontal)
        || !floatValue(args, count, 5, input.lookVertical) || !floatValue(args, count, 6, input.zoom)
        || !floatValue(args, count, 7, input.mouseDeltaX) || !floatValue(args, count, 8, input.mouseDeltaY)
        || !boolean(args, count, 9, input.sprint) || !floatValue(args, count, 10, deltaSeconds))
        return false;
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.camera == nullptr)
        return false;
    bridge.camera->update(bridge.camera->self, handle, &input, deltaSeconds);
    *outCount = 0;
    return true;
}

bool viewPosition(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::uint32_t handle = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    if (count != 3 || capacity < 3 || !uint32Value(args, count, 0, handle)
        || !uint32Value(args, count, 1, width) || !uint32Value(args, count, 2, height))
        return false;
    ViewState state{};
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.camera == nullptr)
        return false;
    bridge.camera->viewState(bridge.camera->self, handle, width, height, &state);
    for (std::uint32_t i = 0; i < 3; ++i)
        resultNumber(results, i, state.position[i]);
    *outCount = 3;
    return true;
}

bool viewPose(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::uint32_t handle = 0;
    if (count != 1 || capacity < 12 || !uint32Value(args, count, 0, handle))
        return false;
    CameraPose pose{};
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.camera == nullptr)
        return false;
    if (!bridge.camera->cameraPose(bridge.camera->self, handle, &pose))
        return false;
    const float *vectors[4] = {pose.position, pose.forward, pose.up, pose.right};
    std::uint32_t result = 0;
    for (const float *vector : vectors)
        for (std::uint32_t i = 0; i < 3; ++i)
            resultNumber(results, result++, vector[i]);
    *outCount = 12;
    return true;
}

bool setOrbitTarget(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *,
    std::uint32_t, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::uint32_t handle = 0;
    float target[3]{};
    if (count != 4 || !uint32Value(args, count, 0, handle)
        || !floatValue(args, count, 1, target[0]) || !floatValue(args, count, 2, target[1])
        || !floatValue(args, count, 3, target[2]))
        return false;
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.camera == nullptr)
        return false;
    bridge.camera->setOrbitTarget(bridge.camera->self, handle, target);
    *outCount = 0;
    return true;
}

bool worldToViewport(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::uint32_t handle = 0, width = 0, height = 0;
    float world[3]{};
    if (count != 6 || capacity < 3 || !uint32Value(args, count, 0, handle)
        || !uint32Value(args, count, 1, width) || !uint32Value(args, count, 2, height)
        || !floatValue(args, count, 3, world[0]) || !floatValue(args, count, 4, world[1])
        || !floatValue(args, count, 5, world[2]))
        return false;
    float x = 0.0f, y = 0.0f, depth = 0.0f;
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.camera == nullptr)
        return false;
    const bool ok = bridge.camera->worldToViewport(bridge.camera->self, handle, width, height, world, &x, &y, &depth);
    if (ok)
    {
        resultNumber(results, 0, x);
        resultNumber(results, 1, y);
        resultNumber(results, 2, depth);
    }
    else
    {
        resultNil(results, 0);
        resultNil(results, 1);
        resultNil(results, 2);
    }
    *outCount = 3;
    return true;
}

bool viewportToWorldRay(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::uint32_t handle = 0, width = 0, height = 0;
    float x = 0.0f, y = 0.0f;
    if (count != 5 || capacity < 6 || !uint32Value(args, count, 0, handle)
        || !uint32Value(args, count, 1, width) || !uint32Value(args, count, 2, height)
        || !floatValue(args, count, 3, x) || !floatValue(args, count, 4, y))
        return false;
    float origin[3]{}, direction[3]{};
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.camera == nullptr)
        return false;
    const bool ok = bridge.camera->viewportToWorldRay(
        bridge.camera->self, handle, width, height, x, y, origin, direction);
    for (std::uint32_t index = 0; index < 6; ++index)
    {
        if (ok)
            resultNumber(results, index, index < 3 ? origin[index] : direction[index - 3]);
        else
            resultNil(results, index);
    }
    *outCount = 6;
    return true;
}

bool setControlSettings(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::uint32_t handle = 0;
    CameraControlSettings settings{};
    float *fields[] = {&settings.moveSpeed, &settings.lookSpeed, &settings.mouseSensitivity,
        &settings.orbitDistance, &settings.orbitZoomSpeed, &settings.sprintMultiplier,
        &settings.maxDeltaSeconds, &settings.pitchLimitDegrees};
    if (count != 9 || capacity < 1 || !uint32Value(args, count, 0, handle))
        return false;
    for (std::uint32_t i = 0; i < 8; ++i)
        if (!floatValue(args, count, i + 1, *fields[i]))
            return false;
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.camera == nullptr)
        return false;
    const bool ok = bridge.camera->setControlSettings(bridge.camera->self, handle, &settings);
    resultBoolean(results, 0, ok);
    *outCount = 1;
    return true;
}

struct Binding
{
    const char *name;
    ScriptNativeFunction function;
};

constexpr Binding bindings[] = {
    {"camera.create", createCamera},
    {"camera.destroy", destroyCamera},
    {"camera.set_mode", setMode},
    {"camera.look_at", lookAt},
    {"camera.set_orbit_target", setOrbitTarget},
    {"camera.set_projection", setProjection},
    {"camera.update", updateCamera},
    {"camera.view_position", viewPosition},
    {"camera.view_pose", viewPose},
    {"camera.world_to_viewport", worldToViewport},
    {"camera.viewport_to_world_ray", viewportToWorldRay},
    {"camera.set_control_settings", setControlSettings},
};
} // namespace

bool initializeCameraLuaRuntime(void *userData, ScriptRuntimeHandle runtime) noexcept
{
    auto *context = static_cast<CameraLuaBridgeContext *>(userData);
    if (context == nullptr || context->scriptRuntime == nullptr || context->camera == nullptr)
        return false;
    try
    {
        for (const Binding &binding : bindings)
            if (!context->scriptRuntime->registerNativeFunction(
                    context->scriptRuntime->self, runtime, binding.name, binding.function, context))
                return false;
        return true;
    }
    catch (...)
    {
        return false;
    }
}
} // namespace engine::camera
