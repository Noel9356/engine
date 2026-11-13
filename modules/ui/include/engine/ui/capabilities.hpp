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
// touch input; 2 -> 3 added numeric and selection widgets; 3 -> 4 added
// layout primitives, popups, menus, tabs, tables, tooltips, tree nodes,
// color edits, vector sliders/drags, plots, font-from-memory, cursor control,
// and window state queries.
inline constexpr std::uint32_t uiCapabilityAbi = 4u;

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
// bitwise or. NoNavigation combines NoNavInputs and NoNavFocus.
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
    NoNavigation     = 1u << 10,
    HorizontalScrollbar = 1u << 11,
    NoFocusOnAppearing  = 1u << 12,
    NoNavInputs         = 1u << 13,
    NoNavFocus          = 1u << 14
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
    BorderShadow,
    FrameBackground,
    FrameHovered,
    FrameActive,
    TitleBackground,
    TitleBackgroundActive,
    TitleBackgroundCollapsed,
    MenuBarBackground,
    ScrollbarBackground,
    ScrollbarGrab,
    ScrollbarGrabHovered,
    ScrollbarGrabActive,
    CheckMark,
    SliderGrab,
    SliderGrabActive,
    Button,
    ButtonHovered,
    ButtonActive,
    Header,
    HeaderHovered,
    HeaderActive,
    Separator,
    SeparatorHovered,
    SeparatorActive,
    ResizeGrip,
    ResizeGripHovered,
    ResizeGripActive,
    Tab,
    TabHovered,
    TabActive,
    TabUnfocused,
    TabUnfocusedActive,
    PlotLines,
    PlotLinesHovered,
    PlotHistogram,
    PlotHistogramHovered,
    TableHeaderBackground,
    TableBorderStrong,
    TableBorderLight,
    TableRowBackground,
    TableRowBackgroundAlt,
    TextSelectedBackground,
    DragDropTarget,
    NavHighlight,
    NavWindowingHighlight,
    NavWindowingDimBackground,
    ModalWindowDimBackground
};

// Scalar style variables for pushStyleVariable. Restricted to the float
// members of ImGuiStyle so the single-argument push API matches; ImVec2
// members (WindowPadding, ItemSpacing, etc.) are not exposed here.
enum class UiStyleVariable : std::uint32_t
{
    Alpha = 0,
    DisabledAlpha,
    WindowRounding,
    WindowBorderSize,
    ChildRounding,
    ChildBorderSize,
    PopupRounding,
    PopupBorderSize,
    FrameRounding,
    FrameBorderSize,
    IndentSpacing,
    ScrollbarSize,
    ScrollbarRounding,
    GrabRounding,
    GrabMinSize,
    TabRounding,
    TabBorderSize,
    TabBarBorderSize,
    TabBarOverlineSize,
    ImageBorderSize,
    TableAngledHeadersAngle,
    SeparatorTextBorderSize
};

// Style applied to every new context unless the host overrides it later.
enum class UiStyle : std::uint32_t
{
    Dark    = 0,
    Light,
    Classic
};

// Bit flags for beginChild: borders and resizable / auto-resizing children.
enum class UiChildFlags : std::uint32_t
{
    None              = 0,
    Border            = 1u << 0,
    AlwaysUseWindowPadding = 1u << 1,
    ResizeX           = 1u << 2,
    ResizeY           = 1u << 3,
    AutoResizeX       = 1u << 4,
    AutoResizeY       = 1u << 5,
    AlwaysAutoResize  = 1u << 6,
    FrameStyle        = 1u << 7,
    NavFlattened      = 1u << 8
};

// Bit flags for treeNodeEx / collapsingHeader.
enum class UiTreeNodeFlags : std::uint32_t
{
    None                    = 0,
    Selected                = 1u << 0,
    Framed                  = 1u << 1,
    AllowItemOverlap        = 1u << 2,
    NoTreePushOnOpen        = 1u << 3,
    NoAutoOpenOnLog         = 1u << 4,
    DefaultOpen             = 1u << 5,
    OpenOnDoubleClick       = 1u << 6,
    OpenOnArrow             = 1u << 7,
    Leaf                    = 1u << 8,
    Bullet                  = 1u << 9,
    FramePadding            = 1u << 10,
    SpanAvailWidth          = 1u << 11,
    SpanFullWidth           = 1u << 12,
    SpanAllColumns          = 1u << 13,
    NavLeftJumpsBackHere    = 1u << 14,
    CollapsingHeader        = 1u << 15
};

// Bit flags for selectableEx.
enum class UiSelectableFlags : std::uint32_t
{
    None                = 0,
    NoAutoClosePopups   = 1u << 0,
    SpanAllColumns      = 1u << 1,
    AllowDoubleClick    = 1u << 2,
    Disabled            = 1u << 3,
    AllowItemOverlap    = 1u << 4
};

// Bit flags for openPopup / beginPopup: mouse button triggers and overlay
// behaviour. MouseButtonLeft/Right/Middle are mouse-button masks for the
// openPopup-style helpers.
enum class UiPopupFlags : std::uint32_t
{
    None                       = 0,
    MouseButtonLeft            = 1u << 0,
    MouseButtonRight           = 1u << 1,
    MouseButtonMiddle          = 1u << 2,
    MouseButtonMaskLeftRight   = (1u << 0) | (1u << 1),
    MouseButtonMaskLeftMiddleRight = (1u << 0) | (1u << 1) | (1u << 2),
    NoOpenOverExistingPopup    = 1u << 3,
    NoOpenOverItems            = 1u << 4,
    AnyPopupId                 = 1u << 5,
    AnyPopupLevel              = 1u << 6,
    AnyPopup                   = (1u << 5) | (1u << 6)
};

// Bit flags for beginTabBar.
enum class UiTabBarFlags : std::uint32_t
{
    None                      = 0,
    Reorderable               = 1u << 0,
    AutoSelectNewTabs         = 1u << 1,
    TabListPopupButton        = 1u << 2,
    NoCloseWithMiddleMouseButton = 1u << 3,
    NoTabListScrollingButtons   = 1u << 4,
    NoTooltip                  = 1u << 5,
    DrawSelectedOverline      = 1u << 6,
    FittingPolicyResizeDown   = 1u << 7,
    FittingPolicyScroll       = 1u << 8,
    FittingPolicyMask         = (1u << 7) | (1u << 8),
    FittingPolicyDefault      = 1u << 7
};

// Bit flags for beginTabItem.
enum class UiTabItemFlags : std::uint32_t
{
    None                       = 0,
    UnsavedDocument            = 1u << 0,
    SetSelected                = 1u << 1,
    NoCloseWithMiddleMouseButton = 1u << 2,
    NoPushId                   = 1u << 3,
    NoTooltip                  = 1u << 4,
    NoReorder                  = 1u << 5,
    Leading                    = 1u << 6,
    Trailing                   = 1u << 7
};

// Bit flags for beginTable.
enum class UiTableFlags : std::uint32_t
{
    None                              = 0,
    Resizable                         = 1u << 0,
    Reorderable                       = 1u << 1,
    Hideable                          = 1u << 2,
    Sortable                          = 1u << 3,
    NoSavedSettings                   = 1u << 4,
    ContextMenuInBody                 = 1u << 5,
    RowBackground                     = 1u << 6,
    BordersInnerHorizontal            = 1u << 7,
    BordersOuterHorizontal            = 1u << 8,
    BordersInnerVertical              = 1u << 9,
    BordersOuterVertical              = 1u << 10,
    BordersHorizontal                 = (1u << 7) | (1u << 8),
    BordersVertical                   = (1u << 9) | (1u << 10),
    BordersInner                      = (1u << 7) | (1u << 9),
    BordersOuter                      = (1u << 8) | (1u << 10),
    Borders                           = (1u << 7) | (1u << 8) | (1u << 9) | (1u << 10),
    NoBordersInBody                   = 1u << 11,
    NoBordersInBodyUntilResize        = 1u << 12,
    SizingFixedFit                    = 1u << 13,
    SizingFixedSame                   = 1u << 14,
    SizingStretchProp                 = 1u << 15,
    SizingStretchSame                 = 1u << 16,
    NoHostExtendX                     = 1u << 17,
    NoHostExtendY                     = 1u << 18,
    NoKeepColumnsVisible              = 1u << 19,
    PreciseWidths                     = 1u << 20,
    NoClip                            = 1u << 21,
    PadOuterX                         = 1u << 22,
    NoPadOuterX                       = 1u << 23,
    NoPadInnerX                       = 1u << 24,
    ScrollX                           = 1u << 25,
    ScrollY                           = 1u << 26,
    SortMulti                         = 1u << 27,
    SortTristate                      = 1u << 28,
    SizingMask                        = (1u << 13) | (1u << 14) | (1u << 15) | (1u << 16)
};

// Bit flags for tableSetupColumn.
enum class UiTableColumnFlags : std::uint32_t
{
    None                       = 0,
    DefaultSort                = 1u << 0,
    IndentEnable               = 1u << 1,
    NoHeaderLabel              = 1u << 2,
    NoHeaderWidth              = 1u << 3,
    NoHide                     = 1u << 4,
    NoClip                     = 1u << 5,
    NoSort                     = 1u << 6,
    NoSortAscending            = 1u << 7,
    NoSortDescending           = 1u << 8,
    NoReorder                  = 1u << 9,
    NoResize                   = 1u << 10,
    WidthFixed                 = 1u << 11,
    WidthStretch               = 1u << 12,
    IsEnabled                  = 1u << 13,
    IsVisible                  = 1u << 14,
    IsSorted                   = 1u << 15,
    IsHovered                  = 1u << 16
};

// Bit flags for tableNextRow.
enum class UiTableRowFlags : std::uint32_t
{
    None    = 0,
    Headers = 1u << 0
};

// Bit flags for colorEdit3 / colorEdit4 / colorButton.
enum class UiColorEditFlags : std::uint32_t
{
    None                       = 0,
    NoAlpha                    = 1u << 1,
    NoPicker                   = 1u << 2,
    NoOptions                  = 1u << 3,
    NoSmallPreview             = 1u << 4,
    NoInputs                   = 1u << 5,
    NoTooltip                  = 1u << 6,
    NoDragDrop                 = 1u << 7,
    NoBorder                   = 1u << 8,
    AlphaBar                   = 1u << 9,
    AlphaPreview               = 1u << 10,
    AlphaPreviewHalf           = 1u << 11,
    DisplayRGB                 = 1u << 12,
    DisplayHSV                 = 1u << 13,
    DisplayHex                 = 1u << 14,
    Uint8                      = 1u << 15,
    Float                      = 1u << 16,
    PickerHueBar               = 1u << 17,
    PickerHueWheel             = 1u << 18,
    InputRGB                   = 1u << 19,
    InputHSV                   = 1u << 20,
    DefaultOptions             = (1u << 9) | (1u << 12) | (1u << 15) | (1u << 17) | (1u << 19)
};

// Mouse cursor shapes for setMouseCursor.
enum class UiMouseCursor : std::uint32_t
{
    None         = 0,
    Arrow,
    TextInput,
    ResizeAll,
    ResizeNS,
    ResizeEW,
    ResizeNESW,
    ResizeNWSE,
    Hand,
    NotAllowed
};

// Table background target for tableSetBgColor.
enum class UiTableBgTarget : std::uint32_t
{
    None      = 0,
    RowBg0    = 1,
    RowBg1    = 2,
    CellBg    = 3
};

// Creation options for a context. reserve*/maximum* bound the per-frame draw
// data (vertex/index/command counts); exceeding a maximum fails endFrame with
// CapacityExceeded. globalScale scales the style and the font size.
// iniFilename, when non-null and non-empty, overrides the module-level ini
// path; the caller owns the storage and it must outlive the context.
// defaultFontSize sizes the built-in default font (0 keeps ImGui's 13 px).
// initialStyle applies StyleColorsDark/Light/Classic after creation.
struct UiContextDescription
{
    float         globalScale        = 1.0f;
    bool          keyboardNavigation = true;
    bool          gamepadNavigation  = false;
    const char   *iniFilename        = nullptr;
    float         defaultFontSize    = 0.0f;
    UiStyle       initialStyle       = UiStyle::Dark;

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
    bool (*selectableEx)(void *self, UiContextHandle context, const char *label, const bool *selected,
        float width, float height, UiSelectableFlags flags);
    bool (*radioButton)(void *self, UiContextHandle context, const char *label, std::int32_t *value,
        std::int32_t buttonValue);
    void (*progressBar)(
        void *self, UiContextHandle context, float fraction, float width, float height, const char *overlay);
    void (*image)(void *self, UiContextHandle context, UiTextureHandle texture, float width, float height, float uv0x,
        float uv0y, float uv1x, float uv1y);
    void (*bulletText)(void *self, UiContextHandle context, const char *value);

    // Layout primitives.
    bool (*beginChild)(void *self, UiContextHandle context, const char *name, float width, float height,
        UiChildFlags flags);
    void (*endChild)(void *self, UiContextHandle context);
    void (*beginGroup)(void *self, UiContextHandle context);
    void (*endGroup)(void *self, UiContextHandle context);
    void (*indent)(void *self, UiContextHandle context, float width);
    void (*unindent)(void *self, UiContextHandle context, float width);
    void (*newLine)(void *self, UiContextHandle context);
    void (*dummy)(void *self, UiContextHandle context, float width, float height);

    // Tree nodes and collapsing headers.
    bool (*treeNode)(void *self, UiContextHandle context, const char *label);
    bool (*treeNodeEx)(void *self, UiContextHandle context, const char *label, UiTreeNodeFlags flags);
    void (*treePop)(void *self, UiContextHandle context);
    void (*setNextItemOpen)(void *self, UiContextHandle context, bool open);
    bool (*collapsingHeader)(void *self, UiContextHandle context, const char *label, UiTreeNodeFlags flags);
    bool (*collapsingHeaderBool)(void *self, UiContextHandle context, const char *label, bool *open,
        UiTreeNodeFlags flags);

    // Disabled sections (greys out contained widgets).
    void (*beginDisabled)(void *self, UiContextHandle context, bool disabled);
    void (*endDisabled)(void *self, UiContextHandle context);

    // Popups and modal dialogs.
    void (*openPopup)(void *self, UiContextHandle context, const char *name, UiPopupFlags flags);
    void (*openPopupId)(void *self, UiContextHandle context, std::uint32_t identifier, UiPopupFlags flags);
    bool (*beginPopup)(void *self, UiContextHandle context, const char *name, UiPopupFlags flags);
    bool (*beginPopupModal)(void *self, UiContextHandle context, const char *name, bool *open, UiPopupFlags flags);
    bool (*beginPopupContextItem)(void *self, UiContextHandle context, const char *name, UiPopupFlags flags);
    bool (*beginPopupContextWindow)(void *self, UiContextHandle context, const char *name, UiPopupFlags flags);
    bool (*beginPopupContextVoid)(void *self, UiContextHandle context, const char *name, UiPopupFlags flags);
    void (*endPopup)(void *self, UiContextHandle context);
    void (*closeCurrentPopup)(void *self, UiContextHandle context);
    bool (*isPopupOpen)(void *self, UiContextHandle context, const char *name, UiPopupFlags flags);

    // Menu bars and items.
    bool (*beginMenuBar)(void *self, UiContextHandle context);
    void (*endMenuBar)(void *self, UiContextHandle context);
    bool (*beginMenu)(void *self, UiContextHandle context, const char *label, bool enabled);
    void (*endMenu)(void *self, UiContextHandle context);
    bool (*menuItem)(void *self, UiContextHandle context, const char *label, const char *shortcut,
        bool *selected, bool enabled);
    bool (*menuItemActivated)(void *self, UiContextHandle context, const char *label, const char *shortcut,
        bool enabled);

    // Tab bars and items.
    bool (*beginTabBar)(void *self, UiContextHandle context, const char *name, UiTabBarFlags flags);
    void (*endTabBar)(void *self, UiContextHandle context);
    bool (*beginTabItem)(void *self, UiContextHandle context, const char *label, bool *open, UiTabItemFlags flags);
    void (*endTabItem)(void *self, UiContextHandle context);
    void (*setTabItemClosed)(void *self, UiContextHandle context, const char *label);

    // Tables.
    bool (*beginTable)(void *self, UiContextHandle context, const char *name, std::uint32_t columns,
        UiTableFlags flags, float outerSizeX, float outerSizeY, float innerWidth);
    void (*endTable)(void *self, UiContextHandle context);
    void (*tableNextRow)(void *self, UiContextHandle context, UiTableRowFlags flags, float minRowHeight);
    bool (*tableNextColumn)(void *self, UiContextHandle context);
    bool (*tableSetColumnIndex)(void *self, UiContextHandle context, std::uint32_t columnIndex);
    void (*tableSetupColumn)(void *self, UiContextHandle context, const char *label, UiTableColumnFlags flags,
        float initWidthOrWeight, std::uint32_t identifier);
    void (*tableSetupScrollFreeze)(void *self, UiContextHandle context, std::uint32_t columns, std::uint32_t rows);
    void (*tableHeadersRow)(void *self, UiContextHandle context);
    void (*tableHeader)(void *self, UiContextHandle context, const char *label);
    std::int32_t (*tableColumnCount)(void *self, UiContextHandle context);
    std::int32_t (*tableColumnIndex)(void *self, UiContextHandle context);
    std::int32_t (*tableRowIndex)(void *self, UiContextHandle context);
    const char *(*tableColumnName)(void *self, UiContextHandle context, std::int32_t columnIndex);
    bool (*tableColumnIsEnabled)(void *self, UiContextHandle context, std::int32_t columnIndex);
    bool (*tableColumnIsSorted)(void *self, UiContextHandle context, std::int32_t columnIndex);
    void (*tableSetBgColor)(void *self, UiContextHandle context, UiTableBgTarget target, std::uint32_t color);

    // Tooltips.
    void (*setTooltip)(void *self, UiContextHandle context, const char *value);
    void (*beginTooltip)(void *self, UiContextHandle context);
    void (*endTooltip)(void *self, UiContextHandle context);

    // Vector sliders/drags.
    bool (*sliderFloat2)(void *self, UiContextHandle context, const char *label, float values[2],
        float minimum, float maximum, const char *format);
    bool (*sliderFloat3)(void *self, UiContextHandle context, const char *label, float values[3],
        float minimum, float maximum, const char *format);
    bool (*sliderFloat4)(void *self, UiContextHandle context, const char *label, float values[4],
        float minimum, float maximum, const char *format);
    bool (*sliderInt2)(void *self, UiContextHandle context, const char *label, std::int32_t values[2],
        std::int32_t minimum, std::int32_t maximum, const char *format);
    bool (*sliderInt3)(void *self, UiContextHandle context, const char *label, std::int32_t values[3],
        std::int32_t minimum, std::int32_t maximum, const char *format);
    bool (*sliderInt4)(void *self, UiContextHandle context, const char *label, std::int32_t values[4],
        std::int32_t minimum, std::int32_t maximum, const char *format);
    bool (*dragFloat2)(void *self, UiContextHandle context, const char *label, float values[2],
        float speed, float minimum, float maximum, const char *format);
    bool (*dragFloat3)(void *self, UiContextHandle context, const char *label, float values[3],
        float speed, float minimum, float maximum, const char *format);
    bool (*dragInt2)(void *self, UiContextHandle context, const char *label, std::int32_t values[2],
        float speed, std::int32_t minimum, std::int32_t maximum, const char *format);
    bool (*dragInt3)(void *self, UiContextHandle context, const char *label, std::int32_t values[3],
        float speed, std::int32_t minimum, std::int32_t maximum, const char *format);

    // Color editors and swatches.
    bool (*colorEdit3)(void *self, UiContextHandle context, const char *label,
        float values[3], UiColorEditFlags flags);
    bool (*colorEdit4)(void *self, UiContextHandle context, const char *label,
        float values[4], UiColorEditFlags flags);
    bool (*colorButton)(void *self, UiContextHandle context, const char *descriptionId,
        const float values[4], UiColorEditFlags flags, float width, float height);

    // Plot helpers for debug overlays (caller-owned sample arrays).
    void (*plotLines)(void *self, UiContextHandle context, const char *label, const float *values,
        std::uint32_t valueCount, float valueMinimum, float valueMaximum, const char *overlay,
        float graphWidth, float graphHeight);
    void (*plotHistogram)(void *self, UiContextHandle context, const char *label, const float *values,
        std::uint32_t valueCount, float valueMinimum, float valueMaximum, const char *overlay,
        float graphWidth, float graphHeight);

    // Item/window state queries.
    bool (*isItemHovered)(void *self, UiContextHandle context);
    bool (*isItemActive)(void *self, UiContextHandle context);
    bool (*isItemFocused)(void *self, UiContextHandle context);
    bool (*isItemClicked)(void *self, UiContextHandle context, std::uint32_t button);
    bool (*isItemVisible)(void *self, UiContextHandle context);
    bool (*isItemActivated)(void *self, UiContextHandle context);
    bool (*isItemDeactivated)(void *self, UiContextHandle context);
    bool (*isItemDeactivatedAfterEdit)(void *self, UiContextHandle context);
    bool (*isAnyItemHovered)(void *self, UiContextHandle context);
    bool (*isAnyItemActive)(void *self, UiContextHandle context);
    bool (*isWindowAppearing)(void *self, UiContextHandle context);
    bool (*isWindowCollapsed)(void *self, UiContextHandle context);
    bool (*isWindowFocused)(void *self, UiContextHandle context);
    bool (*isWindowHovered)(void *self, UiContextHandle context);

    // Window placement queries.
    float (*windowWidth)(void *self, UiContextHandle context);
    float (*windowHeight)(void *self, UiContextHandle context);
    float (*windowPositionX)(void *self, UiContextHandle context);
    float (*windowPositionY)(void *self, UiContextHandle context);
    void (*setNextWindowFocus)(void *self, UiContextHandle context);
    void (*setNextWindowCollapsed)(void *self, UiContextHandle context, bool collapsed);
    void (*setWindowFocus)(void *self, UiContextHandle context, const char *name);
    void (*setWindowCollapsedNamed)(void *self, UiContextHandle context, const char *name, bool collapsed);

    // Mouse cursor control (ImGui-side, not OS-side).
    void (*setMouseCursor)(void *self, UiContextHandle context, UiMouseCursor cursor);
    UiMouseCursor (*mouseCursor)(void *self, UiContextHandle context);

    // Fonts. Must be called outside an active frame: it rebuilds the font atlas.
    bool (*addFontFromFile)(
        void *self, UiContextHandle context, const char *filePath, float pixelSize, UiFontHandle *outFont);
    bool (*addFontFromMemoryTTF)(
        void *self, UiContextHandle context, const void *data, std::uint32_t dataSize, float pixelSize,
        UiFontHandle *outFont);
    bool (*pushFont)(void *self, UiContextHandle context, UiFontHandle font);
    void (*popFont)(void *self, UiContextHandle context);
    bool (*fontAtlas)(void *self, UiContextHandle context, UiFontAtlas *outAtlas);
    bool (*setFontTexture)(void *self, UiContextHandle context, UiTextureHandle texture);

    // Style stack and window placement.
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
