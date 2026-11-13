#include "frame_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace engine::input_actions
{
namespace internal
{

float FrameEvaluator::readBinding(const InputBinding &binding)
{
    const DeviceState *devicePtr = devices_.find(binding.device);
    if (devicePtr == nullptr)
        return 0.0f;
    const DeviceState &device = *devicePtr;
    float              value  = 0.0f;
    switch (binding.control)
    {
    case InputControlKind::Key: {
        const auto found = device.keys.find(binding.code);
        value            = found != device.keys.end() && found->second ? 1.0f : 0.0f;
        break;
    }
    case InputControlKind::MouseButton: {
        const auto found = device.mouseButtons.find(binding.code);
        value            = found != device.mouseButtons.end() && found->second ? 1.0f : 0.0f;
        break;
    }
    case InputControlKind::GamepadButton: {
        const auto found = device.gamepadButtons.find(binding.code);
        value            = found != device.gamepadButtons.end() && found->second ? 1.0f : 0.0f;
        break;
    }
    case InputControlKind::GamepadAxis: {
        const auto found = device.gamepadAxes.find(binding.code);
        value            = found != device.gamepadAxes.end() ? found->second : 0.0f;
        break;
    }
    case InputControlKind::MouseDeltaX:
        value = device.mouseDelta[0];
        break;
    case InputControlKind::MouseDeltaY:
        value = device.mouseDelta[1];
        break;
    case InputControlKind::MouseWheelX:
        value = device.mouseWheel[0];
        break;
    case InputControlKind::MouseWheelY:
        value = device.mouseWheel[1];
        break;
    case InputControlKind::MousePositionX:
        value = device.mousePosition[0];
        break;
    case InputControlKind::MousePositionY:
        value = device.mousePosition[1];
        break;
    case InputControlKind::TouchContact: {
        const TouchState *touch = devices_.findTouch(device, binding.code);
        value = touch != nullptr && (touch->down || touch->pressedThisFrame) ? 1.0f : 0.0f;
        break;
    }
    case InputControlKind::TouchPositionX: {
        const TouchState *touch = devices_.findTouch(device, binding.code);
        value                   = touch != nullptr ? touch->x : 0.0f;
        break;
    }
    case InputControlKind::TouchPositionY: {
        const TouchState *touch = devices_.findTouch(device, binding.code);
        value                   = touch != nullptr ? touch->y : 0.0f;
        break;
    }
    case InputControlKind::TouchDeltaX: {
        const TouchState *touch = devices_.findTouch(device, binding.code);
        value                   = touch != nullptr ? touch->deltaX : 0.0f;
        break;
    }
    case InputControlKind::TouchDeltaY: {
        const TouchState *touch = devices_.findTouch(device, binding.code);
        value                   = touch != nullptr ? touch->deltaY : 0.0f;
        break;
    }
    case InputControlKind::TouchPressure: {
        const TouchState *touch = devices_.findTouch(device, binding.code);
        value                   = touch != nullptr ? touch->pressure : 0.0f;
        break;
    }
    }
    if (binding.control == InputControlKind::GamepadAxis)
    {
        // deadzone < 0 means "use the module default".
        const float deadzone = binding.deadzone >= 0.0f ? binding.deadzone : config_.defaultDeadzone;
        if (deadzone > 0.0f)
        {
            if (std::abs(value) <= deadzone)
                value = 0.0f;
            else
                value = std::copysign((std::abs(value) - deadzone) / (1.0f - deadzone), value);
        }
    }
    if (binding.invert)
        value = -value;
    value *= binding.scale;
    if (binding.responseExponent != 1.0f && value != 0.0f)
        value = std::copysign(std::pow(std::abs(value), binding.responseExponent), value);
    return value;
}

void FrameEvaluator::advanceFrame(float deltaSeconds)
{
    // Non-finite or non-positive deltas skip time-based effects.
    const float frameDelta = std::isfinite(deltaSeconds) && deltaSeconds > 0.0f
                                 ? std::min(deltaSeconds, config_.maximumFrameDeltaSeconds)
                                 : 0.0f;
    contexts_.refreshOrder();
    consumedControls_.clear();
    for (ContextRecord *context : contexts_.orderedContexts())
    {
        pendingConsumption_.clear();
        bool contextActive = false;
        for (auto &[id, action] : context->actions)
        {
            (void)id;
            const bool       wasDown = action.state.down;
            InputActionState next{};
            activeControls_.clear();
            if (context->enabled)
            {
                for (const InputBinding &binding : action.bindings)
                {
                    ControlKey key{binding.control, binding.device, binding.code};
                    // Wildcard touch bindings resolve to one concrete contact
                    // id per frame so the same active contact is consumed for
                    // every lower-priority context (concrete or wildcard).
                    if (binding.code == anyTouchContact && isTouchControl(binding.control))
                    {
                        const DeviceState *device = devices_.find(binding.device);
                        if (device != nullptr)
                            (void)devices_.findTouch(*device, anyTouchContact, &key.code);
                    }
                    if (consumedControls_.contains(key))
                        continue;
                    const float value          = readBinding(binding);
                    float      &componentValue = next.value[binding.component];
                    switch (action.combineMode)
                    {
                    case InputActionCombineMode::Sum:
                        componentValue += value;
                        break;
                    case InputActionCombineMode::MaximumMagnitude:
                        if (std::abs(value) > std::abs(componentValue))
                            componentValue = value;
                        break;
                    case InputActionCombineMode::LastNonZero:
                        if (value != 0.0f)
                            componentValue = value;
                        break;
                    }
                    if (value != 0.0f)
                        activeControls_.push_back(key);
                }
                next.value[0] = std::clamp(next.value[0], -1.0f, 1.0f);
                next.value[1] = std::clamp(next.value[1], -1.0f, 1.0f);
                if (action.smoothingSeconds > 0.0f && action.type != InputActionType::Button && frameDelta > 0.0f)
                {
                    // Exponential smoothing toward the raw value; the time
                    // constant is smoothingSeconds.
                    const float alpha = 1.0f - std::exp(-frameDelta / action.smoothingSeconds);
                    for (std::size_t component = 0; component < 2; ++component)
                        next.value[component]
                            = action.state.value[component] + (next.value[component] - action.state.value[component]) * alpha;
                }
                // Buttons threshold on |x|; Axis2D thresholds on the Euclidean
                // magnitude (up to ~1.41 on the diagonal).
                const float magnitude = action.type == InputActionType::Axis2D
                                            ? std::sqrt(next.value[0] * next.value[0] + next.value[1] * next.value[1])
                                            : std::abs(next.value[0]);
                next.down     = magnitude >= action.threshold;
                next.pressed  = next.down && !wasDown;
                next.released = !next.down && wasDown;
            }
            action.state = next;
            // Collect this context's active controls; they only become
            // consumed when the context opts in and actually fired.
            if (next.down)
            {
                contextActive = true;
                pendingConsumption_.insert(pendingConsumption_.end(), activeControls_.begin(), activeControls_.end());
            }
        }
        if (context->enabled && context->consumeWhenActive && contextActive)
            consumedControls_.insert(pendingConsumption_.begin(), pendingConsumption_.end());
    }
    devices_.resetFrameAccumulators();
}

} // namespace internal
} // namespace engine::input_actions
