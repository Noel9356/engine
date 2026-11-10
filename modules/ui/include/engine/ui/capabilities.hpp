// Public capability surface of the ui module.
//
// The module wraps Dear ImGui behind a C struct of function pointers: context
// management, an input event sink, an immediate-mode widget set, and draw data
// a renderer can consume without knowing anything about ImGui.

#ifndef ENGINE_UI_CAPABILITIES_HPP
#define ENGINE_UI_CAPABILITIES_HPP

#include <cstdint>

namespace engine
{
namespace ui
{

// Capability ABI. Bump when the UiCapability layout changes so consumers can
// reject mismatched providers. History: 1 -> 2 added single-primary-pointer
// touch input; 2 -> 3 added numeric and selection widgets.
inline constexpr std::uint32_t uiCapabilityAbi = 3u;

// Opaque handles. 0 (invalidUiContext) means "none"; font handle 0 is the
// built-in default font created with every context.
using UiContextHandle = std::uint32_t;
using UiFontHandle    = std::uint32_t;
using UiTextureHandle = std::uint64_t;

inline constexpr UiContextHandle invalidUiContext = 0;
inline constexpr UiFontHandle    defaultUiFont    = 0;

// Error codes returned through out-parameters of fallible calls. WrongThread
// enforces the single-owner-thread policy, FrameAlreadyActive/FrameNotActive
// guard the beginFrame/endFrame pairing, CapacityExceeded caps draw data.
enum class UiError : std::uint32_t
{
    None = 0,
    BackendUnavailable,
    CreationFailed,
    InvalidHandle,
    WrongThread,
    InvalidDescription,
    FrameAlreadyActive,
    FrameNotActive,
    CapacityExceeded
};

// Keyboard keys the host forwards via addKey, named after the GLFW/ImGui
// layout (D0..D9 digits, A..Z letters, F1..F12, keypad block, modifiers).
// Keys with no ImGui counterpart are ignored by the backend.
enum class UiKey : std::uint32_t
{
    None,
    Tab,
    Left,
    Right,
    Up,
    Down,
    PageUp,
    PageDown,
    Home,
    End,
    Insert,
    Delete,
    Backspace,
    Space,
    Enter,
    Escape,
    D0,
    D1,
    D2,
    D3,
    D4,
    D5,
    D6,
    D7,
    D8,
    D9,
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L,
    M,
    N,
    O,
    P,
    Q,
    R,
    S,
    T,
    U,
    V,
    W,
    X,
    Y,
    Z,
    F1,
    F2,
    F3,
    F4,
    F5,
    F6,
    F7,
    F8,
    F9,
    F10,
    F11,
    F12,
    Apostrophe,
    Comma,
    Minus,
    Period,
    Slash,
    Semicolon,
    Equal,
    LeftBracket,
    Backslash,
    RightBracket,
    GraveAccent,
    CapsLock,
    ScrollLock,
    NumLock,
    PrintScreen,
    Pause,
    Keypad0,
    Keypad1,
    Keypad2,
    Keypad3,
    Keypad4,
    Keypad5,
    Keypad6,
    Keypad7,
    Keypad8,
    Keypad9,
    KeypadDecimal,
    KeypadDivide,
    KeypadMultiply,
    KeypadSubtract,
    KeypadAdd,
    KeypadEnter,
    KeypadEqual,
    LeftControl,
    LeftShift,
    LeftAlt,
    LeftSuper,
    RightControl,
    RightShift,
    RightAlt,
    RightSuper
};

// Platform-neutral gamepad buttons, mapped onto ImGui's gamepad keys.
enum class UiGamepadButton : std::uint32_t
{
    FaceLeft = 0,
    FaceRight,
    FaceUp,
    FaceDown,
    DpadLeft,
    DpadRight,
    DpadUp,
    DpadDown,
    LeftShoulder,
    RightShoulder,
    LeftTrigger,
    RightTrigger,
    LeftStick,
    RightStick,
    Start,
    Back
};

// Analog axes in [-1, 1]; addGamepadAxis splits each stick axis into
// ImGui's positive/negative directional keys, triggers drive one key.
enum class UiGamepadAxis : std::uint32_t
{
    LeftStickX = 0,
    LeftStickY,
    RightStickX,
    RightStickY,
    LeftTrigger,
    RightTrigger
};

// Bit flags for beginWindow, mirroring the ImGui window flags; combine with
// bitwise or.
enum class UiWindowFlags : std::uint32_t
{
    None             = 0,
    NoTitleBar       = 1u << 0,
    NoResize         = 1u << 1,
    NoMove           = 1u << 2,
    NoScrollbar      = 1u << 3,
    NoCollapse       = 1u << 4,
    AlwaysAutoResize = 1u << 5,
    NoBackground     = 1u << 6,
    NoSavedSettings  = 1u << 7,
    MenuBar          = 1u << 8,
    NoBringToFront   = 1u << 9,
    NoNavigation     = 1u << 10
};

// Style color slots for pushStyleColor (ImGui's palette: text, backgrounds,
// frames, buttons, headers, sliders).
enum class UiColor : std::uint32_t
{
    Text = 0,
    TextDisabled,
    WindowBackground,
    ChildBackground,
    PopupBackground,
    Border,
    FrameBackground,
    FrameHovered,
    FrameActive,
    TitleBackground,
    TitleBackgroundActive,
    Button,
    ButtonHovered,
    ButtonActive,
    Header,
    HeaderHovered,
    HeaderActive,
    CheckMark,
    SliderGrab,
    SliderGrabActive
};

// Scalar style variables for pushStyleVariable: alpha factors and rounding
// /border sizes (pixels).
enum class UiStyleVariable : std::uint32_t
{
    Alpha = 0,
    DisabledAlpha,
    WindowRounding,
    WindowBorderSize,
    ChildRounding,
    FrameRounding,
    FrameBorderSize,
    PopupRounding,
    PopupBorderSize,
    ScrollbarRounding,
    GrabRounding,
    TabRounding
};

// Creation options for a context. reserve*/maximum* bound the per-frame draw
// data (vertex/index/command counts); exceeding a maximum fails endFrame with
// CapacityExceeded. globalScale scales the style and the font size.
struct UiContextDescription
{
    float globalScale        = 1.0f;
    bool  keyboardNavigation = true;
    bool  gamepadNavigation  = false;

    std::uint32_t reserveVertices = 4096;
    std::uint32_t reserveIndices  = 8192;
    std::uint32_t reserveCommands = 256;
    std::uint32_t maximumVertices = 1'048'576;
    std::uint32_t maximumIndices  = 2'097'152;
    std::uint32_t maximumCommands = 65'536;
};

// Per-frame parameters for beginFrame: display size in pixels, framebuffer
// scale (device pixels per display pixel), deltaSeconds for animations.
struct UiFrameDescription
{
    float displayWidth      = 0.0f;
    float displayHeight     = 0.0f;
    float framebufferScaleX = 1.0f;
    float framebufferScaleY = 1.0f;
    float deltaSeconds      = 1.0f / 60.0f;
};

// One triangle vertex of the draw data: position in display pixels, uv in
// texture coordinates, color as the backend's packed RGBA word.
struct UiVertex
{
    float         position[2] = {};
    float         uv[2]       = {};
    std::uint32_t color       = 0;
};

// One indexed draw: elementCount indices from indexOffset/vertexOffset,
// scissored to clipRectangle {x0, y0, x1, y1}. texture is a renderer-owned
// handle (registered with setFontTexture); resetRenderState asks for a reset.
struct UiDrawCommand
{
    std::uint32_t   indexOffset      = 0;
    std::uint32_t   vertexOffset     = 0;
    std::uint32_t   elementCount     = 0;
    float           clipRectangle[4] = {};
    UiTextureHandle texture          = 0;
    bool            resetRenderState = false;
};

// Renderable output of one frame, valid from endFrame until the next
// beginFrame. vertices/indices/commands point into context-owned buffers.
struct UiDrawData
{
    const UiVertex      *vertices     = nullptr;
    std::uint32_t        vertexCount  = 0;
    const std::uint32_t *indices      = nullptr;
    std::uint32_t        indexCount   = 0;
    const UiDrawCommand *commands     = nullptr;
    std::uint32_t        commandCount = 0;

    float displayPosition[2]  = {};
    float displaySize[2]      = {};
    float framebufferScale[2] = {1.0f, 1.0f};
};

// Font atlas pixels (RGBA). The buffer is owned by the font system and stays
// valid until fonts change; upload it once and register with setFontTexture.
struct UiFontAtlas
{
    const std::uint8_t *pixels        = nullptr;
    std::uint32_t       width         = 0;
    std::uint32_t       height        = 0;
    std::uint32_t       bytesPerPixel = 4;
};

// Which input streams the UI currently wants for itself; hosts use it to
// decide whether game input should be suppressed.
struct UiCaptureState
{
    bool mouse     = false;
    bool keyboard  = false;
    bool textInput = false;
};

// Touch is mapped to ImGui's primary mouse pointer. The first contact owns
// the pointer until it ends or is cancelled; concurrent contacts are ignored
// by the UI sink (they remain available to input_actions for gesture logic).
enum class UiTouchPhase : std::uint32_t
{
    Began = 0,
    Moved,
    Ended,
    Cancelled
};

// The "ui" capability: a C struct of function pointers; self is the first
// argument of every call. Contexts are bound to their creating thread, widget
// calls only act inside an active frame, and a false beginWindow still needs
// a matching endWindow.
struct UiCapability
{
    static constexpr const char *capabilityName = "ui";

    std::uint32_t abi  = uiCapabilityAbi;
    void         *self = nullptr;

    UiContextHandle (*createContext)(void *self, const UiContextDescription *description, UiError *error);
    void (*destroyContext)(void *self, UiContextHandle context);
    bool (*beginFrame)(void *self, UiContextHandle context, const UiFrameDescription *description, UiError *error);
    bool (*endFrame)(void *self, UiContextHandle context, UiError *error);
    const UiDrawData *(*drawData)(void *self, UiContextHandle context);
    bool (*captureState)(void *self, UiContextHandle context, UiCaptureState *outState);

    void (*addMousePosition)(void *self, UiContextHandle context, float x, float y);
    void (*addMouseButton)(void *self, UiContextHandle context, std::uint32_t button, bool down);
    void (*addTouch)(void *self, UiContextHandle context, std::uint32_t touchId, UiTouchPhase phase,
        float x, float y);
    void (*addMouseWheel)(void *self, UiContextHandle context, float horizontal, float vertical);
    void (*addKey)(void *self, UiContextHandle context, UiKey key, bool down);
    void (*addGamepadButton)(void *self, UiContextHandle context, UiGamepadButton button, bool down);
    void (*addGamepadAxis)(void *self, UiContextHandle context, UiGamepadAxis axis, float value);
    void (*addText)(void *self, UiContextHandle context, std::uint32_t codepoint);
    void (*addFocus)(void *self, UiContextHandle context, bool focused);

    bool (*beginWindow)(void *self, UiContextHandle context, const char *name, bool *open, UiWindowFlags flags);
    void (*endWindow)(void *self, UiContextHandle context);
    void (*text)(void *self, UiContextHandle context, const char *value);
    void (*textColored)(
        void *self, UiContextHandle context, float red, float green, float blue, float alpha, const char *value);
    void (*separator)(void *self, UiContextHandle context);
    void (*sameLine)(void *self, UiContextHandle context);
    void (*spacing)(void *self, UiContextHandle context);
    bool (*button)(void *self, UiContextHandle context, const char *label, float width, float height);
    bool (*checkbox)(void *self, UiContextHandle context, const char *label, bool *value);
    bool (*sliderFloat)(void *self, UiContextHandle context, const char *label, float *value, float minimum,
        float maximum, const char *format);
    bool (*inputText)(void *self, UiContextHandle context, const char *label, char *buffer, std::uint32_t bufferSize);
    bool (*inputInt)(void *self, UiContextHandle context, const char *label, std::int32_t *value,
        std::int32_t step, std::int32_t fastStep);
    bool (*inputFloat)(void *self, UiContextHandle context, const char *label, float *value,
        const char *format);
    bool (*sliderInt)(void *self, UiContextHandle context, const char *label, std::int32_t *value,
        std::int32_t minimum, std::int32_t maximum, const char *format);
    bool (*dragFloat)(void *self, UiContextHandle context, const char *label, float *value,
        float speed, float minimum, float maximum, const char *format);
    bool (*combo)(void *self, UiContextHandle context, const char *label, std::int32_t *currentItem,
        const char *const *items, std::uint32_t itemCount);
    bool (*selectable)(void *self, UiContextHandle context, const char *label, bool *selected,
        float width, float height);
    bool (*radioButton)(void *self, UiContextHandle context, const char *label, std::int32_t *value,
        std::int32_t buttonValue);
    void (*progressBar)(
        void *self, UiContextHandle context, float fraction, float width, float height, const char *overlay);
    void (*image)(void *self, UiContextHandle context, UiTextureHandle texture, float width, float height, float uv0x,
        float uv0y, float uv1x, float uv1y);

    // Must be called outside an active frame: it rebuilds the font atlas.
    bool (*addFontFromFile)(
        void *self, UiContextHandle context, const char *filePath, float pixelSize, UiFontHandle *outFont);
    bool (*pushFont)(void *self, UiContextHandle context, UiFontHandle font);
    void (*popFont)(void *self, UiContextHandle context);
    bool (*fontAtlas)(void *self, UiContextHandle context, UiFontAtlas *outAtlas);
    bool (*setFontTexture)(void *self, UiContextHandle context, UiTextureHandle texture);

    bool (*pushStyleColor)(
        void *self, UiContextHandle context, UiColor color, float red, float green, float blue, float alpha);
    void (*popStyleColor)(void *self, UiContextHandle context, std::uint32_t count);
    bool (*pushStyleVariable)(void *self, UiContextHandle context, UiStyleVariable variable, float value);
    void (*popStyleVariable)(void *self, UiContextHandle context, std::uint32_t count);
    void (*setNextWindowPosition)(void *self, UiContextHandle context, float x, float y, float pivotX, float pivotY);
    void (*setNextWindowSize)(void *self, UiContextHandle context, float width, float height);
};

} // namespace ui
} // namespace engine

#endif
