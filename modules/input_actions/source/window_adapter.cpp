#include "window_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace engine::input_actions
{
namespace internal
{

namespace
{
// Clamps a double into float range without UB at the float extremes.
float clampToFloat(double value) noexcept
{
    return static_cast<float>(std::clamp(value,
        -static_cast<double>(std::numeric_limits<float>::max()),
        static_cast<double>(std::numeric_limits<float>::max())));
}
} // namespace

bool feedWindowEvent(
    DeviceStateRegistry &devices, window::WindowHandle windowHandle, const window::WindowEvent &event)
{
    if (windowHandle == window::invalidWindow)
        return false;

    InputEvent input{};
    input.device = keyboardMouseDevice;
    switch (event.kind)
    {
    case window::WindowEventKind::KeyPressed:
    case window::WindowEventKind::KeyReleased:
        input.kind = InputEventKind::Key;
        input.code = static_cast<std::uint32_t>(event.key);
        input.down = event.kind == window::WindowEventKind::KeyPressed;
        return input.code != static_cast<std::uint32_t>(window::Key::Unknown) && devices.feedEvent(input);
    case window::WindowEventKind::MouseButtonPressed:
    case window::WindowEventKind::MouseButtonReleased:
        input.kind = InputEventKind::MouseButton;
        input.code = static_cast<std::uint32_t>(event.mouseButton);
        input.down = event.kind == window::WindowEventKind::MouseButtonPressed;
        return input.code < 8u && devices.feedEvent(input);
    case window::WindowEventKind::CursorMoved: {
        // CursorMoved emits both the per-frame delta (accumulated until
        // advanceFrame) and the absolute position so consumers can bind either.
        input.kind            = InputEventKind::MouseDelta;
        input.value           = clampToFloat(event.cursorDeltaX);
        input.secondaryValue  = clampToFloat(event.cursorDeltaY);
        if (!devices.feedEvent(input))
            return false;
        input.kind            = InputEventKind::MousePosition;
        input.value           = clampToFloat(event.cursorX);
        input.secondaryValue  = clampToFloat(event.cursorY);
        return devices.feedEvent(input);
    }
    case window::WindowEventKind::MouseScrolled:
        input.kind           = InputEventKind::MouseWheel;
        input.value          = clampToFloat(event.scrollX);
        input.secondaryValue = clampToFloat(event.scrollY);
        return devices.feedEvent(input);
    case window::WindowEventKind::FocusLost:
        input.kind = InputEventKind::FocusLost;
        if (!devices.feedEvent(input))
            return false;
        // A lost window focus can prevent a native host from receiving
        // touch-end callbacks. Drop all forwarded contacts to avoid stuck touches.
        devices.clearTouchDevices();
        return true;
    case window::WindowEventKind::GamepadConnected:
        return true;
    case window::WindowEventKind::GamepadDisconnected:
        devices.reset(gamepadDeviceIdBase | (event.gamepadId & 0x3fffffffu));
        return true;
    case window::WindowEventKind::GamepadButtonPressed:
    case window::WindowEventKind::GamepadButtonReleased:
        input.device = gamepadDeviceIdBase | (event.gamepadId & 0x3fffffffu);
        input.kind   = InputEventKind::GamepadButton;
        input.code   = event.gamepadControl;
        input.down   = event.kind == window::WindowEventKind::GamepadButtonPressed;
        return input.code < 15u && devices.feedEvent(input);
    case window::WindowEventKind::GamepadAxisMoved:
        input.device = gamepadDeviceIdBase | (event.gamepadId & 0x3fffffffu);
        input.kind   = InputEventKind::GamepadAxis;
        input.code   = event.gamepadControl;
        input.value  = event.gamepadValue;
        return input.code < 6u && devices.feedEvent(input);
    case window::WindowEventKind::TouchBegan:
    case window::WindowEventKind::TouchMoved:
    case window::WindowEventKind::TouchEnded:
    case window::WindowEventKind::TouchCancelled:
        if (event.touchDevice > 0x7fffffffu)
            return false;
        input.device         = touchDeviceIdBase | event.touchDevice;
        input.code           = event.touchId;
        input.value          = event.touchX;
        input.secondaryValue = event.touchY;
        input.tertiaryValue  = event.touchPressure;
        switch (event.kind)
        {
        case window::WindowEventKind::TouchBegan: input.kind = InputEventKind::TouchBegan; break;
        case window::WindowEventKind::TouchMoved: input.kind = InputEventKind::TouchMoved; break;
        case window::WindowEventKind::TouchEnded: input.kind = InputEventKind::TouchEnded; break;
        case window::WindowEventKind::TouchCancelled: input.kind = InputEventKind::TouchCancelled; break;
        default: return false;
        }
        return devices.feedEvent(input);
    default:
        // CloseRequested, Resized, FramebufferResized, FocusGained, Minimized,
        // Restored, TextInput, FilesDropped: not device input.
        return true;
    }
}

} // namespace internal
} // namespace engine::input_actions
