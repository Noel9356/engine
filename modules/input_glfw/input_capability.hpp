#ifndef ENGINE_MODULES_INPUT_INPUT_CAPABILITY_HPP
#define ENGINE_MODULES_INPUT_INPUT_CAPABILITY_HPP

// The published capability surface of the input module. Every consumer -
// other modules, native mods, the host - includes THIS header, from the
// module that provides the capability: the provider owns and publishes its
// own capability vocabulary, so there is no central contract directory and
// no mirrored copy that could drift.
//
// This must stay implementation-, platform- and vendor-agnostic. Key codes
// carry stable values (see the enum below); see mods/README.md, section
// "Portability". The key name mapping is capability metadata, so it is
// published here as inline functions: every consumer binary embeds the table
// it uses and no plugin depends on another plugin's symbols at load time.

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

// Published when the platform delivered a text character (the text-input
// path: layout-aware, modifiers applied - "Shift+a" arrives as 'A', not as
// two key events). The codepoint is Unicode; scripts receive it as a number
// (and can convert with the Lua utf8 library).
struct TextInputEvent
{
    std::uint32_t codepoint = 0;
};

// Published when a gamepad was plugged in / unplugged (the first gamepad;
// connection state is re-polled lazily, so the events may lag the hardware
// event by one query).
struct GamepadConnectedEvent
{
};

struct GamepadDisconnectedEvent
{
};

// Gamepad buttons with stable values (the enum intentionally matches the
// GLFW gamepad button order; a different backend maps onto these).
enum class GamepadButton : std::int32_t
{
    A            = 0,
    B            = 1,
    X            = 2,
    Y            = 3,
    LeftBumper   = 4,
    RightBumper  = 5,
    Back         = 6,
    Start        = 7,
    Guide        = 8,
    LeftStick    = 9,
    RightStick   = 10,
    DpadUp       = 11,
    DpadRight    = 12,
    DpadDown     = 13,
    DpadLeft     = 14
};

// Gamepad axes with stable values. Sticks are -1..1 (y grows DOWN on the
// standard layout, matching cursor conventions); triggers 0..1.
enum class GamepadAxis : std::int32_t
{
    LeftX        = 0,
    LeftY        = 1,
    RightX       = 2,
    RightY       = 3,
    LeftTrigger  = 4,
    RightTrigger = 5
};

namespace detail
{

struct GamepadNameEntry
{
    const char    *name;
    std::int32_t   code;
    bool           isButton; // true: button table, false: axis table
};

inline const GamepadNameEntry *gamepadNameEntries()
{
    static constexpr GamepadNameEntry entries[] = {
        {.name = "A", .code = 0, .isButton = true},
        {.name = "B", .code = 1, .isButton = true},
        {.name = "X", .code = 2, .isButton = true},
        {.name = "Y", .code = 3, .isButton = true},
        {.name = "LeftBumper", .code = 4, .isButton = true},
        {.name = "RightBumper", .code = 5, .isButton = true},
        {.name = "Back", .code = 6, .isButton = true},
        {.name = "Start", .code = 7, .isButton = true},
        {.name = "Guide", .code = 8, .isButton = true},
        {.name = "LeftStick", .code = 9, .isButton = true},
        {.name = "RightStick", .code = 10, .isButton = true},
        {.name = "DpadUp", .code = 11, .isButton = true},
        {.name = "DpadRight", .code = 12, .isButton = true},
        {.name = "DpadDown", .code = 13, .isButton = true},
        {.name = "DpadLeft", .code = 14, .isButton = true},
        {.name = "LeftX", .code = 0, .isButton = false},
        {.name = "LeftY", .code = 1, .isButton = false},
        {.name = "RightX", .code = 2, .isButton = false},
        {.name = "RightY", .code = 3, .isButton = false},
        {.name = "LeftTrigger", .code = 4, .isButton = false},
        {.name = "RightTrigger", .code = 5, .isButton = false},
    };
    return entries;
}

} // namespace detail

// Parses a gamepad control name ("A", "DpadUp", "LeftTrigger") into its
// button/axis code. Returns false when the name is not recognized.
inline bool gamepadControlFromName(
    const std::string &name, std::int32_t &outCode, bool &outIsButton
)
{
    for (const detail::GamepadNameEntry *entry = detail::gamepadNameEntries(); entry->name != nullptr; ++entry)
    {
        if (name == entry->name)
        {
            outCode     = entry->code;
            outIsButton = entry->isButton;
            return true;
        }
    }
    return false;
}

// Mouse buttons with stable values (the enum intentionally matches GLFW).
enum class MouseButton : std::int32_t
{
    Left   = 0,
    Right  = 1,
    Middle = 2
};

namespace detail
{

// The stable key name table: capability metadata, written once, next to the
// capability it belongs to. Designated initializers keep every row's meaning
// in the row itself.
struct KeyCodeNameEntry
{
    const char *name;
    KeyCode     keyCode;
};

inline const KeyCodeNameEntry *keyCodeNameEntries()
{
    static constexpr KeyCodeNameEntry entries[] = {
        {.name = "Space", .keyCode = KeyCode::Space},
        {.name = "Apostrophe", .keyCode = KeyCode::Apostrophe},
        {.name = "Comma", .keyCode = KeyCode::Comma},
        {.name = "Minus", .keyCode = KeyCode::Minus},
        {.name = "Period", .keyCode = KeyCode::Period},
        {.name = "Slash", .keyCode = KeyCode::Slash},
        {.name = "0", .keyCode = KeyCode::Key0},
        {.name = "1", .keyCode = KeyCode::Key1},
        {.name = "2", .keyCode = KeyCode::Key2},
        {.name = "3", .keyCode = KeyCode::Key3},
        {.name = "4", .keyCode = KeyCode::Key4},
        {.name = "5", .keyCode = KeyCode::Key5},
        {.name = "6", .keyCode = KeyCode::Key6},
        {.name = "7", .keyCode = KeyCode::Key7},
        {.name = "8", .keyCode = KeyCode::Key8},
        {.name = "9", .keyCode = KeyCode::Key9},
        {.name = "Semicolon", .keyCode = KeyCode::Semicolon},
        {.name = "Equal", .keyCode = KeyCode::Equal},
        {.name = "A", .keyCode = KeyCode::A},
        {.name = "B", .keyCode = KeyCode::B},
        {.name = "C", .keyCode = KeyCode::C},
        {.name = "D", .keyCode = KeyCode::D},
        {.name = "E", .keyCode = KeyCode::E},
        {.name = "F", .keyCode = KeyCode::F},
        {.name = "G", .keyCode = KeyCode::G},
        {.name = "H", .keyCode = KeyCode::H},
        {.name = "I", .keyCode = KeyCode::I},
        {.name = "J", .keyCode = KeyCode::J},
        {.name = "K", .keyCode = KeyCode::K},
        {.name = "L", .keyCode = KeyCode::L},
        {.name = "M", .keyCode = KeyCode::M},
        {.name = "N", .keyCode = KeyCode::N},
        {.name = "O", .keyCode = KeyCode::O},
        {.name = "P", .keyCode = KeyCode::P},
        {.name = "Q", .keyCode = KeyCode::Q},
        {.name = "R", .keyCode = KeyCode::R},
        {.name = "S", .keyCode = KeyCode::S},
        {.name = "T", .keyCode = KeyCode::T},
        {.name = "U", .keyCode = KeyCode::U},
        {.name = "V", .keyCode = KeyCode::V},
        {.name = "W", .keyCode = KeyCode::W},
        {.name = "X", .keyCode = KeyCode::X},
        {.name = "Y", .keyCode = KeyCode::Y},
        {.name = "Z", .keyCode = KeyCode::Z},
        {.name = "LeftBracket", .keyCode = KeyCode::LeftBracket},
        {.name = "Backslash", .keyCode = KeyCode::Backslash},
        {.name = "RightBracket", .keyCode = KeyCode::RightBracket},
        {.name = "GraveAccent", .keyCode = KeyCode::GraveAccent},
        {.name = "Escape", .keyCode = KeyCode::Escape},
        {.name = "Enter", .keyCode = KeyCode::Enter},
        {.name = "Tab", .keyCode = KeyCode::Tab},
        {.name = "Backspace", .keyCode = KeyCode::Backspace},
        {.name = "Insert", .keyCode = KeyCode::Insert},
        {.name = "Delete", .keyCode = KeyCode::Delete},
        {.name = "Right", .keyCode = KeyCode::Right},
        {.name = "Left", .keyCode = KeyCode::Left},
        {.name = "Down", .keyCode = KeyCode::Down},
        {.name = "Up", .keyCode = KeyCode::Up},
        {.name = "PageUp", .keyCode = KeyCode::PageUp},
        {.name = "PageDown", .keyCode = KeyCode::PageDown},
        {.name = "Home", .keyCode = KeyCode::Home},
        {.name = "End", .keyCode = KeyCode::End},
        {.name = "CapsLock", .keyCode = KeyCode::CapsLock},
        {.name = "ScrollLock", .keyCode = KeyCode::ScrollLock},
        {.name = "NumLock", .keyCode = KeyCode::NumLock},
        {.name = "PrintScreen", .keyCode = KeyCode::PrintScreen},
        {.name = "Pause", .keyCode = KeyCode::Pause},
        {.name = "F1", .keyCode = KeyCode::F1},
        {.name = "F2", .keyCode = KeyCode::F2},
        {.name = "F3", .keyCode = KeyCode::F3},
        {.name = "F4", .keyCode = KeyCode::F4},
        {.name = "F5", .keyCode = KeyCode::F5},
        {.name = "F6", .keyCode = KeyCode::F6},
        {.name = "F7", .keyCode = KeyCode::F7},
        {.name = "F8", .keyCode = KeyCode::F8},
        {.name = "F9", .keyCode = KeyCode::F9},
        {.name = "F10", .keyCode = KeyCode::F10},
        {.name = "F11", .keyCode = KeyCode::F11},
        {.name = "F12", .keyCode = KeyCode::F12},
        {.name = "LeftShift", .keyCode = KeyCode::LeftShift},
        {.name = "LeftControl", .keyCode = KeyCode::LeftControl},
        {.name = "LeftAlt", .keyCode = KeyCode::LeftAlt},
        {.name = "LeftSuper", .keyCode = KeyCode::LeftSuper},
        {.name = "RightShift", .keyCode = KeyCode::RightShift},
        {.name = "RightControl", .keyCode = KeyCode::RightControl},
        {.name = "RightAlt", .keyCode = KeyCode::RightAlt},
        {.name = "RightSuper", .keyCode = KeyCode::RightSuper},
        {.name = "Menu", .keyCode = KeyCode::Menu},
    };
    return entries;
}

} // namespace detail

// Returns the stable name ("A", "F12", "LeftShift") for diagnostics and script
// bridges, or "Unknown" for unrecognized codes.
inline const char *keyCodeName(
    KeyCode keyCode
)
{
    for (const detail::KeyCodeNameEntry *entry = detail::keyCodeNameEntries(); entry->name != nullptr; ++entry)
    {
        if (entry->keyCode == keyCode)
        {
            return entry->name;
        }
    }
    return "Unknown";
}

// Parses a configuration key name into a code. Returns KeyCode::Unknown when
// the name is not recognized.
inline KeyCode keyCodeFromName(
    const std::string &keyName
)
{
    for (const detail::KeyCodeNameEntry *entry = detail::keyCodeNameEntries(); entry->name != nullptr; ++entry)
    {
        if (keyName == entry->name)
        {
            return entry->keyCode;
        }
    }
    return KeyCode::Unknown;
}

// Capability "input": keyboard and mouse state. Continuous devices (cursor,
// wheel) are consumed as per-frame DELTAS: the provider accumulates platform
// events and every consumer query drains the accumulator, so each frame reads
// exactly the motion that arrived since the last query - no event subscription
// needed on the hot path.
class InputCapability
{
public:
    // The implementation-agnostic family name of this capability. Consumers
    // acquire through it; the module's plugin descriptor carries the
    // implementation-bound member name ("input.glfw").
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

    // Current cursor position on the main window, in the same platform
    // cursor units as the deltas (screen pixels), origin at the top left.
    // Unchanged by the delta consumers: absolute position and motion are
    // independent accumulators, so an immediate mode UI reading position
    // does not race the camera consuming motion.
    virtual void cursorPosition(double &positionX, double &positionY) = 0;

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

    // ---- gamepad -----------------------------------------------------------------

    // Whether a gamepad is currently connected (the system's first gamepad;
    // connection state changes publish GamepadConnectedEvent /
    // GamepadDisconnectedEvent on the bus).
    virtual bool isGamepadConnected() = 0;

    // Current polled state of one gamepad button. False without a gamepad.
    virtual bool isGamepadButtonDown(GamepadButton button) = 0;

    // Current value of one gamepad axis (sticks -1..1, triggers 0..1).
    // 0.0 without a gamepad.
    virtual float gamepadAxis(GamepadAxis axis) = 0;

    // ---- action mapping ----------------------------------------------------------

    // Bind-by-name input: the assembly (or a mod's shipped defaults) maps
    // action names onto device bindings in [input.actions]
    //
    //   [input.actions]
    //   jump = "Space, gamepad:A"
    //   fire = "mouse:left, gamepad:RightTrigger"
    //
    // and consumers query by MEANING, never by device: actionStrength("jump")
    // is 1.0 while any bound control is held (analog for trigger bindings),
    // 0.0 otherwise. Unknown action names return 0.0 - a script asking for an
    // unconfigured action gets "not pressed", and the startup log names the
    // actions that resolved to no binding at all.
    virtual float actionStrength(const std::string &actionName) = 0;
};

} // namespace engine

#endif // ENGINE_MODULES_INPUT_INPUT_CAPABILITY_HPP
