// Translates window::WindowEvent into InputEvent(s) and feeds them to a
// DeviceStateRegistry. Unrelated window events are accepted and ignored so
// callers can drain an entire window queue through one call.
//
// Key/mouse events are routed to keyboardMouseDevice, gamepad events to
// gamepadDeviceIdBase | GLFW joystick id, and touch events to
// touchDeviceIdBase | native touch device id. A FocusLost event additionally
// clears touch contacts on every touch-surface device so a host that misses
// touch-end callbacks cannot leave stuck touches.

#ifndef ENGINE_INPUT_ACTIONS_WINDOW_ADAPTER_HPP
#define ENGINE_INPUT_ACTIONS_WINDOW_ADAPTER_HPP

#include "device_state.hpp"

#include <engine/input_actions/capabilities.hpp>
#include <engine/window/capabilities.hpp>

namespace engine::input_actions
{
namespace internal
{

// Forwards one WindowEvent into the device registry. Returns true unless the
// event itself is malformed (null pointers, invalid handle, oversized touch
// device id). Unrelated window events return true and are ignored.
bool feedWindowEvent(DeviceStateRegistry &devices, window::WindowHandle windowHandle, const window::WindowEvent &event);

} // namespace internal
} // namespace engine::input_actions

#endif // ENGINE_INPUT_ACTIONS_WINDOW_ADAPTER_HPP
