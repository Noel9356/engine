// Input event sink.
//
// Forwards mouse, touch, wheel, key, gamepad, text and focus events into
// ImGui's IO queue. Events take effect on the next NewFrame. Non-finite or
// out-of-range input is silently dropped; touch maps to ImGui's primary
// mouse pointer (first contact owns the pointer until ended or cancelled).

#include "ui_backend.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace engine
{
namespace ui
{
namespace backend
{

#if defined(ENGINE_UI_BACKEND_IMGUI)

void addMousePosition(UiModuleState &state, UiContextHandle handle, float x, float y)
{
    UiContextRecord *record = findContext(state, handle);
    if (record == nullptr || !std::isfinite(x) || !std::isfinite(y))
        return;
    makeCurrent(*record);
    ImGui::GetIO().AddMousePosEvent(x, y);
}

void addMouseButton(UiModuleState &state, UiContextHandle handle, std::uint32_t button, bool down)
{
    UiContextRecord *record = findContext(state, handle);
    if (record == nullptr || button >= ImGuiMouseButton_COUNT)
        return;
    makeCurrent(*record);
    ImGui::GetIO().AddMouseButtonEvent(static_cast<int>(button), down);
}

void addTouch(UiModuleState &state, UiContextHandle handle, std::uint32_t touchId, UiTouchPhase phase,
    float x, float y)
{
    UiContextRecord *record = findContext(state, handle);
    if (record == nullptr || static_cast<std::uint32_t>(phase) > static_cast<std::uint32_t>(UiTouchPhase::Cancelled)
        || !std::isfinite(x) || !std::isfinite(y))
        return;
    makeCurrent(*record);
    ImGuiIO &io = ImGui::GetIO();
    switch (phase)
    {
    case UiTouchPhase::Began:
        if (record->touchPointerActive)
            return;
        record->touchPointerActive = true;
        record->touchPointerId     = touchId;
        io.AddMousePosEvent(x, y);
        io.AddMouseButtonEvent(0, true);
        break;
    case UiTouchPhase::Moved:
        if (record->touchPointerActive && record->touchPointerId == touchId)
            io.AddMousePosEvent(x, y);
        break;
    case UiTouchPhase::Ended:
    case UiTouchPhase::Cancelled:
        if (record->touchPointerActive && record->touchPointerId == touchId)
        {
            io.AddMousePosEvent(x, y);
            io.AddMouseButtonEvent(0, false);
            record->touchPointerActive = false;
        }
        break;
    }
}

void addMouseWheel(UiModuleState &state, UiContextHandle handle, float horizontal, float vertical)
{
    UiContextRecord *record = findContext(state, handle);
    if (record == nullptr || !std::isfinite(horizontal) || !std::isfinite(vertical))
        return;
    makeCurrent(*record);
    ImGui::GetIO().AddMouseWheelEvent(horizontal, vertical);
}

void addKey(UiModuleState &state, UiContextHandle handle, UiKey key, bool down)
{
    UiContextRecord *record = findContext(state, handle);
    const ImGuiKey   mapped = nativeKey(key);
    if (record == nullptr || mapped == ImGuiKey_None)
        return;
    makeCurrent(*record);
    ImGui::GetIO().AddKeyEvent(mapped, down);
}

void addGamepadButton(UiModuleState &state, UiContextHandle handle, UiGamepadButton button, bool down)
{
    UiContextRecord *record = findContext(state, handle);
    const ImGuiKey   mapped = nativeGamepadButton(button);
    if (record == nullptr || mapped == ImGuiKey_None)
        return;
    makeCurrent(*record);
    ImGui::GetIO().AddKeyEvent(mapped, down);
}

// Analog axes are split into ImGui's positive/negative directional keys;
// the value is clamped to [-1, 1] and triggers only drive the positive key.
void addGamepadAxis(UiModuleState &state, UiContextHandle handle, UiGamepadAxis axis, float value)
{
    UiContextRecord *record = findContext(state, handle);
    if (record == nullptr || !std::isfinite(value))
        return;
    makeCurrent(*record);
    ImGuiIO    &io       = ImGui::GetIO();
    const float amount   = std::clamp(value, -1.0f, 1.0f);
    const auto  sendAxis = [&io](ImGuiKey positive, ImGuiKey negative, float position)
    {
        io.AddKeyAnalogEvent(positive, position > 0.0f, std::max(position, 0.0f));
        io.AddKeyAnalogEvent(negative, position < 0.0f, std::max(-position, 0.0f));
    };
    switch (axis)
    {
    case UiGamepadAxis::LeftStickX:
        sendAxis(ImGuiKey_GamepadLStickRight, ImGuiKey_GamepadLStickLeft, amount);
        break;
    case UiGamepadAxis::LeftStickY:
        sendAxis(ImGuiKey_GamepadLStickDown, ImGuiKey_GamepadLStickUp, amount);
        break;
    case UiGamepadAxis::RightStickX:
        sendAxis(ImGuiKey_GamepadRStickRight, ImGuiKey_GamepadRStickLeft, amount);
        break;
    case UiGamepadAxis::RightStickY:
        sendAxis(ImGuiKey_GamepadRStickDown, ImGuiKey_GamepadRStickUp, amount);
        break;
    case UiGamepadAxis::LeftTrigger:
        io.AddKeyAnalogEvent(ImGuiKey_GamepadL2, amount > 0.0f, std::max(amount, 0.0f));
        break;
    case UiGamepadAxis::RightTrigger:
        io.AddKeyAnalogEvent(ImGuiKey_GamepadR2, amount > 0.0f, std::max(amount, 0.0f));
        break;
    }
}

// Text input as a Unicode codepoint; surrogates and values above 0x10FFFF
// are rejected.
void addText(UiModuleState &state, UiContextHandle handle, std::uint32_t codepoint)
{
    UiContextRecord *record = findContext(state, handle);
    if (record == nullptr || codepoint == 0 || codepoint > 0x10FFFFu
        || (codepoint >= 0xD800u && codepoint <= 0xDFFFu))
        return;
    makeCurrent(*record);
    ImGui::GetIO().AddInputCharacter(codepoint);
}

void addFocus(UiModuleState &state, UiContextHandle handle, bool focused)
{
    UiContextRecord *record = findContext(state, handle);
    if (record == nullptr)
        return;
    makeCurrent(*record);
    ImGuiIO &io = ImGui::GetIO();
    if (!focused && record->touchPointerActive)
    {
        io.AddMouseButtonEvent(0, false);
        record->touchPointerActive = false;
    }
    io.AddFocusEvent(focused);
}

#endif // ENGINE_UI_BACKEND_IMGUI

} // namespace backend
} // namespace ui
} // namespace engine
