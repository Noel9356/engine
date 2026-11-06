// Public capability surface of the input_actions module.
//
// Consumers obtain this C struct via ParticipantContext::acquireCapability to
// define virtual input actions (buttons/axes), bind them to device controls,
// feed raw device events, and read per-frame action state.

#ifndef ENGINE_INPUT_ACTIONS_CAPABILITIES_HPP
#define ENGINE_INPUT_ACTIONS_CAPABILITIES_HPP

#include <cstdint>

namespace engine::input_actions
{

// ABI version consumers must match before calling any entry point.
inline constexpr std::uint32_t inputActionsCapabilityAbi = 2u;
// Handles are module-wide ids starting at 1; 0 (invalid*) means "none".
using InputContextHandle                                = std::uint32_t;
using InputActionHandle                                 = std::uint32_t;
inline constexpr InputContextHandle invalidInputContext = 0;
inline constexpr InputActionHandle  invalidInputAction  = 0;

// Kind of raw event fed via feedEvent. Key/MouseButton/GamepadButton carry
// press state in `down`; GamepadAxis carries `value` in [-1, 1]; MouseDelta
// and MouseWheel carry X in `value`, Y in `secondaryValue`, and accumulate
// until the next advanceFrame; FocusLost wipes a device's stored state.
enum class InputEventKind : std::uint32_t
{
    Key = 0,
    MouseButton,
    GamepadButton,
    GamepadAxis,
    MouseDelta,
    MouseWheel,
    FocusLost
};

// What a binding reads from device state. The first four map to stored
// button/axis state; MouseDeltaX/Y and MouseWheelX/Y select one component of
// the accumulated per-frame values.
enum class InputControlKind : std::uint32_t
{
    Key = 0,
    MouseButton,
    GamepadButton,
    GamepadAxis,
    MouseDeltaX,
    MouseDeltaY,
    MouseWheelX,
    MouseWheelY
};

// Action output shape: Button is digital (down flag only), Axis1D uses
// component 0, Axis2D uses components 0 (X) and 1 (Y).
enum class InputActionType : std::uint32_t
{
    Button = 0,
    Axis1D,
    Axis2D
};

// How multiple bindings merge into one component: Sum adds, MaximumMagnitude
// keeps the value with the largest absolute value, LastNonZero keeps the
// most recent nonzero value.
enum class InputActionCombineMode : std::uint32_t
{
    Sum = 0,
    MaximumMagnitude,
    LastNonZero
};

// One raw device event. `device` is a producer-chosen device id (state is
// created per id on demand); `code` is the kind-specific control code;
// `down` is only meaningful for the button kinds.
struct InputEvent
{
    InputEventKind kind           = InputEventKind::Key;
    std::uint32_t  device         = 0;
    std::uint32_t  code           = 0;
    float          value          = 0.0f;
    float          secondaryValue = 0.0f;
    bool           down           = false;
};

// Context creation parameters. Higher-`priority` contexts evaluate (and may
// consume controls) first; consumeWhenActive hides this context's active
// controls from every lower-priority context for the frame.
struct InputContextDescription
{
    const char  *name              = nullptr;
    std::int32_t priority          = 0;
    bool         enabled           = true;
    bool         consumeWhenActive = false;
};

// Action definition. activationThreshold is the |value| (or the 2D magnitude)
// at which the action counts as down; smoothingSeconds applies an exponential
// filter to axis actions and is ignored for Button.
struct InputActionDescription
{
    const char            *name                = nullptr;
    InputActionType        type                = InputActionType::Button;
    float                  activationThreshold = 0.5f;
    InputActionCombineMode combineMode         = InputActionCombineMode::Sum;
    float                  smoothingSeconds    = 0.0f;
};

// One control-to-action mapping. component selects the Axis2D target (0 or
// 1); scale and invert reshape the raw value; deadzone applies only to
// GamepadAxis (-1 means the module default); responseExponent applies a
// power curve |v|^e and must lie in (0, 8].
struct InputBinding
{
    InputControlKind control          = InputControlKind::Key;
    std::uint32_t    device           = 0;
    std::uint32_t    code             = 0;
    std::uint32_t    component        = 0;
    float            scale            = 1.0f;
    float            deadzone         = -1.0f;
    float            responseExponent = 1.0f;
    bool             invert           = false;
};

// Per-frame result: value holds one or two components, down is the threshold
// test, and pressed/released are the down edges detected on this frame.
struct InputActionState
{
    float value[2] = {0.0f, 0.0f};
    bool  down     = false;
    bool  pressed  = false;
    bool  released = false;
};

// Capability vtable handed to consumers; every function takes `self` first.
// Frame flow: feedEvent as events arrive, advanceFrame once per frame, then
// actionState per action. The module has no internal locks, so drive the
// whole capability from a single thread.
struct InputActionsCapability
{
    static constexpr const char *capabilityName = "input_actions";
    std::uint32_t                abi            = inputActionsCapabilityAbi;
    void                        *self           = nullptr;

    // Returns invalidInputContext when the context cap is reached.
    InputContextHandle (*createContext)(void *, const InputContextDescription *);
    void (*destroyContext)(void *, InputContextHandle);
    bool (*setContextEnabled)(void *, InputContextHandle, bool);
    // Rejects duplicate names, invalid type/combine enums, thresholds outside
    // [0, 1], and smoothing above the configured cap.
    InputActionHandle (*createAction)(void *, InputContextHandle, const InputActionDescription *);
    bool (*removeAction)(void *, InputContextHandle, InputActionHandle);
    // The component must match the action type; Button actions cannot bind
    // analog axes. Appends the binding (up to the configured cap).
    bool (*bindAction)(void *, InputContextHandle, InputActionHandle, const InputBinding *);
    bool (*clearBindings)(void *, InputContextHandle, InputActionHandle);
    // Updates stored device state; false for malformed events or when the
    // device/control caps refuse a new device or control.
    bool (*feedEvent)(void *, const InputEvent *);
    // Re-evaluates every action for the frame; deltaSeconds is clamped to the
    // configured maximum (and non-positive deltas skip time-based effects).
    void (*advanceFrame)(void *, float deltaSeconds);
    // Copies the state computed by the last advanceFrame.
    bool (*actionState)(void *, InputContextHandle, InputActionHandle, InputActionState *);
    // Copies the action name including the terminating NUL; reports the
    // required size via outRequired.
    bool (*copyActionName)(void *, InputContextHandle, InputActionHandle, char *, std::uint32_t, std::uint32_t *);
    // Forgets everything about a device, also freeing its device slot.
    void (*resetDeviceState)(void *, std::uint32_t device);
};

} // namespace engine::input_actions

#endif
