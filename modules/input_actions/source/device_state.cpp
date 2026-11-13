#include "device_state.hpp"

#include <cmath>
#include <cstdint>
#include <utility>

namespace engine::input_actions
{
namespace internal
{

namespace
{
// Clamps the magnitude of an accumulated mouse delta/wheel value to keep a
// runaway producer from producing NaNs via float overflow. The +/-1e6 bound
// matches the documented cap.
constexpr float kAccumulatorCap = 1.0e6f;
} // namespace

template <typename Map, typename Value>
bool DeviceStateRegistry::upsertControl(DeviceState &device, Map &controls, std::uint32_t code, Value value)
{
    const auto item = controls.find(code);
    if (item == controls.end())
    {
        if (device.controlCount >= config_.maximumControlsPerDevice)
            return false;
        try
        {
            controls.emplace(code, value);
        }
        catch (...)
        {
            return false;
        }
        ++device.controlCount;
    }
    else
    {
        item->second = value;
    }
    return true;
}

bool DeviceStateRegistry::feedEvent(const InputEvent &event)
{
    if (!std::isfinite(event.value) || !std::isfinite(event.secondaryValue) || !std::isfinite(event.tertiaryValue)
        || static_cast<std::uint32_t>(event.kind) > static_cast<std::uint32_t>(InputEventKind::MousePosition))
        return false;

    if (event.kind == InputEventKind::FocusLost)
    {
        const auto found = devices_.find(event.device);
        if (found == devices_.end())
            return true;
        DeviceState &device   = found->second;
        device.controlCount   = 0;
        device.keys.clear();
        device.mouseButtons.clear();
        device.gamepadButtons.clear();
        device.gamepadAxes.clear();
        device.touches.clear();
        device.mouseDelta[0] = device.mouseDelta[1] = 0.0f;
        device.mouseWheel[0] = device.mouseWheel[1] = 0.0f;
        return true;
    }

    auto found = devices_.find(event.device);
    if (found == devices_.end())
    {
        if (devices_.size() >= config_.maximumDevices)
            return false;
        try
        {
            found = devices_.try_emplace(event.device).first;
        }
        catch (...)
        {
            return false;
        }
    }
    DeviceState &device = found->second;

    switch (event.kind)
    {
    case InputEventKind::Key:
        return upsertControl(device, device.keys, event.code, event.down);
    case InputEventKind::MouseButton:
        return upsertControl(device, device.mouseButtons, event.code, event.down);
    case InputEventKind::GamepadButton:
        return upsertControl(device, device.gamepadButtons, event.code, event.down);
    case InputEventKind::GamepadAxis:
        // Analog values are clamped to the conventional [-1, 1] range; stick
        // axes are signed and trigger axes are unipolar [0, 1].
        return upsertControl(device, device.gamepadAxes, event.code, std::clamp(event.value, -1.0f, 1.0f));
    case InputEventKind::MouseDelta:
        device.mouseDelta[0] = std::clamp(device.mouseDelta[0] + event.value, -kAccumulatorCap, kAccumulatorCap);
        device.mouseDelta[1] = std::clamp(device.mouseDelta[1] + event.secondaryValue, -kAccumulatorCap, kAccumulatorCap);
        return true;
    case InputEventKind::MouseWheel:
        device.mouseWheel[0] = std::clamp(device.mouseWheel[0] + event.value, -kAccumulatorCap, kAccumulatorCap);
        device.mouseWheel[1] = std::clamp(device.mouseWheel[1] + event.secondaryValue, -kAccumulatorCap, kAccumulatorCap);
        return true;
    case InputEventKind::MousePosition:
        // Absolute cursor position; stored verbatim (caller-chosen units).
        device.mousePosition[0] = event.value;
        device.mousePosition[1] = event.secondaryValue;
        return true;
    case InputEventKind::TouchBegan:
    case InputEventKind::TouchMoved:
    case InputEventKind::TouchEnded:
    case InputEventKind::TouchCancelled:
        return handleTouch(device, event);
    case InputEventKind::FocusLost:
        // Handled by the early return above; unreachable here.
        break;
    }
    return false;
}

bool DeviceStateRegistry::handleTouch(DeviceState &device, const InputEvent &event)
{
    if (event.code == anyTouchContact || event.value < 0.0f || event.value > 1.0f || event.secondaryValue < 0.0f
        || event.secondaryValue > 1.0f || event.tertiaryValue < 0.0f || event.tertiaryValue > 1.0f)
        return false;
    auto touch = device.touches.find(event.code);
    if (event.kind == InputEventKind::TouchBegan)
    {
        if (touch != device.touches.end() && touch->second.down)
            return false;
        if (touch == device.touches.end() && device.controlCount >= config_.maximumControlsPerDevice)
            return false;
        TouchState state{};
        state.x                  = event.value;
        state.y                  = event.secondaryValue;
        state.pressure           = event.tertiaryValue;
        state.down               = true;
        state.pressedThisFrame   = true;
        state.sequence           = ++device.touchSequence;
        if (state.sequence == 0)
            state.sequence = ++device.touchSequence;
        if (touch == device.touches.end())
        {
            try
            {
                device.touches.emplace(event.code, state);
            }
            catch (...)
            {
                return false;
            }
            ++device.controlCount;
        }
        else
        {
            touch->second = state;
        }
        return true;
    }
    if (touch == device.touches.end() || !touch->second.down)
        return false;
    TouchState &state = touch->second;
    if (event.kind == InputEventKind::TouchMoved)
    {
        state.deltaX += event.value - state.x;
        state.deltaY += event.secondaryValue - state.y;
    }
    state.x         = event.value;
    state.y         = event.secondaryValue;
    state.pressure  = event.tertiaryValue;
    state.sequence  = ++device.touchSequence;
    if (state.sequence == 0)
        state.sequence = ++device.touchSequence;
    if (event.kind == InputEventKind::TouchEnded || event.kind == InputEventKind::TouchCancelled)
        state.down = false;
    return true;
}

void DeviceStateRegistry::clearTouchDevices()
{
    for (auto &[deviceId, device] : devices_)
    {
        if ((deviceId & touchDeviceIdBase) != 0)
        {
            device.controlCount -= device.touches.size();
            device.touches.clear();
        }
    }
}

void DeviceStateRegistry::resetFrameAccumulators()
{
    for (auto &[id, device] : devices_)
    {
        (void)id;
        device.mouseDelta[0]    = device.mouseDelta[1] = 0.0f;
        device.mouseWheel[0]    = device.mouseWheel[1] = 0.0f;
        for (auto touch = device.touches.begin(); touch != device.touches.end();)
        {
            touch->second.deltaX            = 0.0f;
            touch->second.deltaY            = 0.0f;
            touch->second.pressedThisFrame  = false;
            if (!touch->second.down)
            {
                touch = device.touches.erase(touch);
                --device.controlCount;
            }
            else
            {
                ++touch;
            }
        }
    }
}

const TouchState *DeviceStateRegistry::findTouch(
    const DeviceState &device, std::uint32_t contactId, std::uint32_t *outContactId) const
{
    if (contactId != anyTouchContact)
    {
        const auto found = device.touches.find(contactId);
        if (outContactId != nullptr)
            *outContactId = found != device.touches.end() ? found->first : anyTouchContact;
        return found != device.touches.end() ? &found->second : nullptr;
    }
    const TouchState *selected   = nullptr;
    std::uint32_t     selectedId = anyTouchContact;
    for (const auto &[id, touch] : device.touches)
    {
        if (selected == nullptr || (touch.down && !selected->down)
            || (touch.down == selected->down && touch.sequence > selected->sequence))
        {
            selected   = &touch;
            selectedId = id;
        }
    }
    if (outContactId != nullptr)
        *outContactId = selectedId;
    return selected;
}

} // namespace internal
} // namespace engine::input_actions
