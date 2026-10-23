#ifndef ENGINE_CAPABILITIES_INPUT_CONTRACT_HPP
#define ENGINE_CAPABILITIES_INPUT_CONTRACT_HPP

// Mod-facing contract: implementation-, platform- and vendor-agnostic.
// Key codes carry stable values (see the enum below); see mods/README.md,
// section "Portability".

#include <cstdint>
#include <string>

namespace engine
{

// Keyboard codes with stable values. The enum intentionally matches the GLFW
// key identifiers so the current input implementation passes platform values
// through without a translation table; a different backend implementation
// would map its native codes onto these values instead.
enum class KeyCode : std::int32_t
{
    Unknown      = -1,
    Space        = 32,
    Apostrophe   = 39,
    Comma        = 44,
    Minus        = 45,
    Period       = 46,
    Slash        = 47,
    Key0         = 48,
    Key1         = 49,
    Key2         = 50,
    Key3         = 51,
    Key4         = 52,
    Key5         = 53,
    Key6         = 54,
    Key7         = 55,
    Key8         = 56,
    Key9         = 57,
    Semicolon    = 59,
    Equal        = 61,
    A            = 65,
    B            = 66,
    C            = 67,
    D            = 68,
    E            = 69,
    F            = 70,
    G            = 71,
    H            = 72,
    I            = 73,
    J            = 74,
    K            = 75,
    L            = 76,
    M            = 77,
    N            = 78,
    O            = 79,
    P            = 80,
    Q            = 81,
    R            = 82,
    S            = 83,
    T            = 84,
    U            = 85,
    V            = 86,
    W            = 87,
    X            = 88,
    Y            = 89,
    Z            = 90,
    LeftBracket  = 91,
    Backslash    = 92,
    RightBracket = 93,
    GraveAccent  = 96,
    Escape       = 256,
    Enter        = 257,
    Tab          = 258,
    Backspace    = 259,
    Insert       = 260,
    Delete       = 261,
    Right        = 262,
    Left         = 263,
    Down         = 264,
    Up           = 265,
    PageUp       = 266,
    PageDown     = 267,
    Home         = 268,
    End          = 269,
    CapsLock     = 280,
    ScrollLock   = 281,
    NumLock      = 282,
    PrintScreen  = 283,
    Pause        = 284,
    F1           = 290,
    F2           = 291,
    F3           = 292,
    F4           = 293,
    F5           = 294,
    F6           = 295,
    F7           = 296,
    F8           = 297,
    F9           = 298,
    F10          = 299,
    F11          = 300,
    F12          = 301,
    LeftShift    = 340,
    LeftControl  = 341,
    LeftAlt      = 342,
    LeftSuper    = 343,
    RightShift   = 344,
    RightControl = 345,
    RightAlt     = 346,
    RightSuper   = 347,
    Menu         = 348
};

// Returns the stable name ("A", "F12", "LeftShift") for diagnostics and script
// bridges, or "Unknown" for unrecognized codes.
const char *keyCodeName(KeyCode keyCode);

// Parses a configuration key name into a code. Returns KeyCode::Unknown when
// the name is not recognized.
KeyCode keyCodeFromName(const std::string &keyName);

// Published by the input module when a key goes from released to pressed.
struct KeyPressedEvent
{
    KeyCode keyCode = KeyCode::Unknown;
};

// Published by the input module when a key goes from pressed to released.
struct KeyReleasedEvent
{
    KeyCode keyCode = KeyCode::Unknown;
};

// Mouse buttons with stable values (the enum intentionally matches GLFW).
enum class MouseButton : std::int32_t
{
    Left   = 0,
    Right  = 1,
    Middle = 2
};

// Capability "input": keyboard and mouse state. Continuous devices (cursor,
// wheel) are consumed as per-frame DELTAS: the provider accumulates platform
// events and every consumer query drains the accumulator, so each frame reads
// exactly the motion that arrived since the last query - no event subscription
// needed on the hot path.
class InputCapability
{
public:
    static constexpr const char *capabilityName = "input";

    virtual ~InputCapability() = default;

    // Current polled state of one key on the main window.
    virtual bool isKeyDown(KeyCode keyCode) = 0;

    // Current polled state of one mouse button on the main window.
    virtual bool isMouseButtonDown(MouseButton button) = 0;

    // Drains the accumulated cursor motion since the last call. The deltas
    // are in platform cursor units (screen pixels on current backends), x
    // grows right, y grows down. Consuming an empty accumulator yields 0, 0.
    virtual void consumeMouseDelta(double &deltaX, double &deltaY) = 0;

    // Drains the accumulated wheel motion since the last call. One notch is
    // typically 1.0 (positive = away from the user).
    virtual void consumeScrollDelta(double &deltaX, double &deltaY) = 0;

    // Captures (disables and hides) or releases the cursor on the main
    // window. While captured, platforms deliver raw relative motion; the
    // module clears any pending delta around the transition so no jump is
    // reported. Inactive without a window.
    virtual void setCursorCaptured(bool captured) = 0;

    // Whether the cursor is currently captured.
    virtual bool isCursorCaptured() const = 0;
};

} // namespace engine

#endif // ENGINE_CAPABILITIES_INPUT_CONTRACT_HPP
