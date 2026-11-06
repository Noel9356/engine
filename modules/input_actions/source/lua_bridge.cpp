#include "lua_bridge.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>

namespace engine::input_actions
{
namespace
{
using script_runtime::ScriptNativeFunction;
using script_runtime::ScriptRuntimeHandle;
using script_runtime::ScriptValue;
using script_runtime::ScriptValueType;
using Bridge = InputActionsLuaBridgeContext;

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

bool int32Value(const ScriptValue *args, std::uint32_t count, std::uint32_t index, std::int32_t &out)
{
    double value = 0.0;
    if (!number(args, count, index, value) || value < std::numeric_limits<std::int32_t>::min()
        || value > std::numeric_limits<std::int32_t>::max() || std::floor(value) != value)
        return false;
    out = static_cast<std::int32_t>(value);
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

bool parseActionType(std::string_view value, InputActionType &out)
{
    if (value == "button") out = InputActionType::Button;
    else if (value == "axis1d") out = InputActionType::Axis1D;
    else if (value == "axis2d") out = InputActionType::Axis2D;
    else return false;
    return true;
}

bool parseCombineMode(std::string_view value, InputActionCombineMode &out)
{
    if (value == "sum") out = InputActionCombineMode::Sum;
    else if (value == "maximum") out = InputActionCombineMode::MaximumMagnitude;
    else if (value == "last") out = InputActionCombineMode::LastNonZero;
    else return false;
    return true;
}

bool parseControl(std::string_view value, InputControlKind &out)
{
    if (value == "key") out = InputControlKind::Key;
    else if (value == "mouse_button") out = InputControlKind::MouseButton;
    else if (value == "gamepad_button") out = InputControlKind::GamepadButton;
    else if (value == "gamepad_axis") out = InputControlKind::GamepadAxis;
    else if (value == "mouse_delta_x") out = InputControlKind::MouseDeltaX;
    else if (value == "mouse_delta_y") out = InputControlKind::MouseDeltaY;
    else if (value == "mouse_wheel_x") out = InputControlKind::MouseWheelX;
    else if (value == "mouse_wheel_y") out = InputControlKind::MouseWheelY;
    else return false;
    return true;
}

bool parseEventKind(std::string_view value, InputEventKind &out)
{
    if (value == "key") out = InputEventKind::Key;
    else if (value == "mouse_button") out = InputEventKind::MouseButton;
    else if (value == "gamepad_button") out = InputEventKind::GamepadButton;
    else if (value == "gamepad_axis") out = InputEventKind::GamepadAxis;
    else if (value == "mouse_delta") out = InputEventKind::MouseDelta;
    else if (value == "mouse_wheel") out = InputEventKind::MouseWheel;
    else if (value == "focus_lost") out = InputEventKind::FocusLost;
    else return false;
    return true;
}

bool createContext(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.inputActions == nullptr)
        return false;
    std::string_view name;
    std::int32_t priority = 0;
    bool enabled = true;
    bool consume = false;
    if (count != 4 || capacity < 1 || !stringValue(args, count, 0, name)
        || !int32Value(args, count, 1, priority) || !boolean(args, count, 2, enabled)
        || !boolean(args, count, 3, consume))
        return false;
    std::string nameStorage(name);
    InputContextDescription description{};
    description.name = nameStorage.c_str();
    description.priority = priority;
    description.enabled = enabled;
    description.consumeWhenActive = consume;
    const InputContextHandle handle
        = bridge.inputActions->createContext(bridge.inputActions->self, &description);
    if (handle == invalidInputContext) resultNil(results, 0);
    else resultNumber(results, 0, handle);
    *outCount = 1;
    return true;
}

bool destroyContext(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *,
    std::uint32_t, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.inputActions == nullptr)
        return false;
    std::uint32_t handle = 0;
    if (count != 1 || !uint32Value(args, count, 0, handle))
        return false;
    bridge.inputActions->destroyContext(bridge.inputActions->self, handle);
    *outCount = 0;
    return true;
}

bool setContextEnabled(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.inputActions == nullptr)
        return false;
    std::uint32_t handle = 0;
    bool enabled = false;
    if (count != 2 || capacity < 1 || !uint32Value(args, count, 0, handle)
        || !boolean(args, count, 1, enabled))
        return false;
    resultBoolean(results, 0, bridge.inputActions->setContextEnabled(bridge.inputActions->self, handle, enabled));
    *outCount = 1;
    return true;
}

bool createAction(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.inputActions == nullptr)
        return false;
    std::uint32_t context = 0;
    std::string_view name;
    std::string_view typeName;
    std::string_view combineName;
    float threshold = 0.5f;
    float smoothing = 0.0f;
    if (count != 6 || capacity < 1 || !uint32Value(args, count, 0, context)
        || !stringValue(args, count, 1, name) || !stringValue(args, count, 2, typeName)
        || !floatValue(args, count, 3, threshold) || !stringValue(args, count, 4, combineName)
        || !floatValue(args, count, 5, smoothing))
        return false;
    InputActionDescription description{};
    std::string nameStorage(name);
    description.name = nameStorage.c_str();
    if (!parseActionType(typeName, description.type) || !parseCombineMode(combineName, description.combineMode))
        return false;
    description.activationThreshold = threshold;
    description.smoothingSeconds = smoothing;
    const InputActionHandle handle
        = bridge.inputActions->createAction(bridge.inputActions->self, context, &description);
    if (handle == invalidInputAction) resultNil(results, 0);
    else resultNumber(results, 0, handle);
    *outCount = 1;
    return true;
}

bool removeAction(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.inputActions == nullptr)
        return false;
    std::uint32_t context = 0;
    std::uint32_t action = 0;
    if (count != 2 || capacity < 1 || !uint32Value(args, count, 0, context)
        || !uint32Value(args, count, 1, action))
        return false;
    resultBoolean(results, 0, bridge.inputActions->removeAction(bridge.inputActions->self, context, action));
    *outCount = 1;
    return true;
}

bool bindAction(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.inputActions == nullptr)
        return false;
    std::uint32_t context = 0;
    std::uint32_t action = 0;
    std::string_view control;
    InputBinding binding{};
    if (count != 10 || capacity < 1 || !uint32Value(args, count, 0, context)
        || !uint32Value(args, count, 1, action) || !stringValue(args, count, 2, control)
        || !uint32Value(args, count, 3, binding.device) || !uint32Value(args, count, 4, binding.code)
        || !uint32Value(args, count, 5, binding.component) || !floatValue(args, count, 6, binding.scale)
        || !floatValue(args, count, 7, binding.deadzone) || !floatValue(args, count, 8, binding.responseExponent)
        || !boolean(args, count, 9, binding.invert) || !parseControl(control, binding.control))
        return false;
    resultBoolean(results, 0, bridge.inputActions->bindAction(bridge.inputActions->self, context, action, &binding));
    *outCount = 1;
    return true;
}

bool clearBindings(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.inputActions == nullptr)
        return false;
    std::uint32_t context = 0;
    std::uint32_t action = 0;
    if (count != 2 || capacity < 1 || !uint32Value(args, count, 0, context)
        || !uint32Value(args, count, 1, action))
        return false;
    resultBoolean(results, 0, bridge.inputActions->clearBindings(bridge.inputActions->self, context, action));
    *outCount = 1;
    return true;
}

bool feedEvent(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.inputActions == nullptr)
        return false;
    std::string_view kindName;
    InputEvent event{};
    if (count != 6 || capacity < 1 || !stringValue(args, count, 0, kindName)
        || !uint32Value(args, count, 1, event.device) || !uint32Value(args, count, 2, event.code)
        || !floatValue(args, count, 3, event.value) || !floatValue(args, count, 4, event.secondaryValue)
        || !boolean(args, count, 5, event.down) || !parseEventKind(kindName, event.kind))
        return false;
    resultBoolean(results, 0, bridge.inputActions->feedEvent(bridge.inputActions->self, &event));
    *outCount = 1;
    return true;
}

bool advanceFrame(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *,
    std::uint32_t, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.inputActions == nullptr)
        return false;
    float deltaSeconds = 0.0f;
    if (count != 1 || !floatValue(args, count, 0, deltaSeconds))
        return false;
    bridge.inputActions->advanceFrame(bridge.inputActions->self, deltaSeconds);
    *outCount = 0;
    return true;
}

bool actionState(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *results,
    std::uint32_t capacity, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.inputActions == nullptr)
        return false;
    std::uint32_t context = 0;
    std::uint32_t action = 0;
    if (count != 2 || capacity < 5 || !uint32Value(args, count, 0, context)
        || !uint32Value(args, count, 1, action))
        return false;
    InputActionState state{};
    if (!bridge.inputActions->actionState(bridge.inputActions->self, context, action, &state))
    {
        resultNil(results, 0);
        *outCount = 1;
        return true;
    }
    resultNumber(results, 0, state.value[0]);
    resultNumber(results, 1, state.value[1]);
    resultBoolean(results, 2, state.down);
    resultBoolean(results, 3, state.pressed);
    resultBoolean(results, 4, state.released);
    *outCount = 5;
    return true;
}

bool resetDevice(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *,
    std::uint32_t, std::uint32_t *outCount)
{
    auto &bridge = *static_cast<Bridge *>(userData);
    std::lock_guard lock(bridge.apiMutex);
    if (bridge.inputActions == nullptr)
        return false;
    std::uint32_t device = 0;
    if (count != 1 || !uint32Value(args, count, 0, device))
        return false;
    bridge.inputActions->resetDeviceState(bridge.inputActions->self, device);
    *outCount = 0;
    return true;
}

struct Binding
{
    const char *name;
    ScriptNativeFunction function;
};

constexpr Binding bindings[] = {
    {"input_actions.create_context", createContext},
    {"input_actions.destroy_context", destroyContext},
    {"input_actions.set_context_enabled", setContextEnabled},
    {"input_actions.create_action", createAction},
    {"input_actions.remove_action", removeAction},
    {"input_actions.bind_action", bindAction},
    {"input_actions.clear_bindings", clearBindings},
    {"input_actions.feed_event", feedEvent},
    {"input_actions.advance_frame", advanceFrame},
    {"input_actions.action_state", actionState},
    {"input_actions.reset_device", resetDevice},
};
} // namespace

bool initializeInputActionsLuaRuntime(void *userData, ScriptRuntimeHandle runtime) noexcept
{
    auto *context = static_cast<InputActionsLuaBridgeContext *>(userData);
    if (context == nullptr || context->scriptRuntime == nullptr || context->inputActions == nullptr)
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
} // namespace engine::input_actions
