#include "lua_bridge.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string_view>

namespace engine::camera
{
namespace
{
using script_runtime::ScriptNativeFunction;
using script_runtime::ScriptRuntimeHandle;
using script_runtime::ScriptValue;
using script_runtime::ScriptValueType;
using Bridge = CameraLuaBridgeContext;

bool number(const ScriptValue *args, std::uint32_t count, std::uint32_t index, double &out)
{
    if (index >= count || args[index].type != ScriptValueType::Number || !std::isfinite(args[index].numberValue))
        return false;
    out = args[index].numberValue;
    return true;
}

bool uint32Value(const ScriptValue *args, std::uint32_t count, std::uint32_t index, std::uint32_t &out)
{
    double value = 0.0;
    if (!number(args, count, index, value) || value < 0.0
        || value > static_cast<double>(std::numeric_limits<std::uint32_t>::max()) || std::floor(value) != value)
        return false;
    out = static_cast<std::uint32_t>(value);
    return true;
}

bool floatValue(const ScriptValue *args, std::uint32_t count, std::uint32_t index, float &out)
{
    double value = 0.0;
    if (!number(args, count, index, value) || value < -std::numeric_limits<float>::max()
        || value > std::numeric_limits<float>::max())
        return false;
    out = static_cast<float>(value);
    return true;
}

bool boolean(const ScriptValue *args, std::uint32_t count, std::uint32_t index, bool &out)
{
    if (index >= count || args[index].type != ScriptValueType::Boolean)
        return false;
    out = args[index].booleanValue;
    return true;
}

bool stringValue(const ScriptValue *args, std::uint32_t count, std::uint32_t index, std::string_view &out)
{
    if (index >= count || args[index].type != ScriptValueType::String || args[index].stringData == nullptr)
        return false;
    out = std::string_view(args[index].stringData, args[index].stringSize);
    return true;
}

void resultNumber(ScriptValue *out, std::uint32_t index, double value)
{
    out[index].type = ScriptValueType::Number;
    out[index].numberValue = value;
}

void resultBoolean(ScriptValue *out, std::uint32_t index, bool value)
{
    out[index].type = ScriptValueType::Boolean;
    out[index].booleanValue = value;
}

void resultNil(ScriptValue *out, std::uint32_t index)
{
    out[index] = {};
    out[index].type = ScriptValueType::Nil;
}

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
    {"camera.set_projection", setProjection},
    {"camera.update", updateCamera},
    {"camera.view_position", viewPosition},
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
