// Internal declarations shared by the input_actions translation units.
//
// The public capability surface lives in
// engine/input_actions/capabilities.hpp; everything below is implementation
// detail. The module decomposes into four cooperating pieces:
//
//   - InputActionsConfiguration: a plain aggregate of the cap values parsed
//     from ParticipantContext, plus a free loader. Header-only.
//   - DeviceStateRegistry: per-device control snapshots, event ingestion,
//     touch-contact lookup, per-frame delta reset.
//   - ContextRegistry: contexts and their actions/bindings, handle allocation,
//     ordered iteration, name uniqueness.
//   - FrameEvaluator: the per-frame pipeline (binding reads, combine modes,
//     shaping, smoothing, threshold, consumption).
//
// input_actions.cpp wires these into the ModuleInterface implementation,
// serializes entry points with one mutex, and binds the C capability vtable.

#ifndef ENGINE_INPUT_ACTIONS_INTERNAL_HPP
#define ENGINE_INPUT_ACTIONS_INTERNAL_HPP

#include <engine/input_actions/capabilities.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace engine::input_actions
{
namespace internal
{

// Per-touch contact snapshot maintained by DeviceStateRegistry. Positions and
// pressure are normalized to [0, 1]; deltas are per-frame coordinate
// differences reset by advanceFrame.
struct TouchState
{
    float        x                  = 0.0f;
    float        y                  = 0.0f;
    float        deltaX             = 0.0f;
    float        deltaY             = 0.0f;
    float        pressure           = 1.0f;
    std::uint64_t sequence          = 0;
    bool         down               = false;
    bool         pressedThisFrame   = false;
};

// Identifies a control (kind + device + code) for consumption tracking. The
// control kind is part of the key so the same device/code pair can be both
// pressed-as-button and read-as-axis without aliasing.
struct ControlKey
{
    InputControlKind control = InputControlKind::Key;
    std::uint32_t    device  = 0;
    std::uint32_t    code    = 0;
    friend bool      operator==(const ControlKey &, const ControlKey &) = default;
};

// Fibonacci-style mixing hash for ControlKey. Adequate for the small sets the
// per-frame consumption tracking produces; numbers chosen from the golden ratio.
struct ControlKeyHash
{
    std::size_t operator()(const ControlKey &key) const noexcept
    {
        std::size_t value = static_cast<std::size_t>(key.control);
        value             = value * 0x9e3779b1u + key.device;
        value             = value * 0x9e3779b1u + key.code;
        return value;
    }
};

// True for controls whose raw value is analog (continuous in [-1, 1] or [0, 1])
// rather than digital (0 or 1). Used by binding validation to reject analog
// controls on Button actions.
constexpr bool isAnalogControl(InputControlKind control) noexcept
{
    return control == InputControlKind::GamepadAxis || control == InputControlKind::TouchPositionX
        || control == InputControlKind::TouchPositionY || control == InputControlKind::TouchDeltaX
        || control == InputControlKind::TouchDeltaY || control == InputControlKind::TouchPressure
        || control == InputControlKind::MousePositionX || control == InputControlKind::MousePositionY;
}

// True for the touch-family controls; used to resolve wildcard contact ids
// during the per-frame consumption pass.
constexpr bool isTouchControl(InputControlKind control) noexcept
{
    return control >= InputControlKind::TouchContact && control <= InputControlKind::TouchPressure;
}

} // namespace internal
} // namespace engine::input_actions

#endif // ENGINE_INPUT_ACTIONS_INTERNAL_HPP
