// Public capability surface of the window module.
//
// Four capabilities: "window" (window lifecycle, events, keyboard/cursor
// input), "window.vulkan" (Vulkan instance extensions and native surface
// handles), "window.glfw" (raw GLFW access), and "window.headless" (software
// surface export for tests and CI).

#ifndef ENGINE_WINDOW_CAPABILITIES_HPP
#define ENGINE_WINDOW_CAPABILITIES_HPP

#include <cstddef>
#include <cstdint>

namespace engine
{
namespace window
{

// Capability ABI shared by all four capability structs. Bump on layout
// changes so consumers can reject mismatched providers. History:
// 9 -> 10 added mouse button state and button events.
// 10 -> 11 added cursor movement, scroll, and Unicode text-input events.
// 11 -> 12 adds standardized gamepad events and a platform-neutral touch ingress; 12 -> 13 adds clipboard text access; 13 -> 14 adds dropped-file batches.
inline constexpr std::uint32_t windowCapabilityAbi = 14u;

// Opaque window identifier; 0 (invalidWindow) means "none".
using WindowHandle = std::uint32_t;

inline constexpr WindowHandle invalidWindow = 0;
inline constexpr std::uint32_t invalidTouchId = 0xffffffffu;
inline constexpr std::uint32_t maximumTouchContactsPerWindow = 256u;

// Error codes for fallible calls. Unsupported marks operations the active
// backend cannot provide (for example pixel locks outside the headless
// backend).
enum class WindowError : std::uint32_t
{
    None = 0,

    BackendUnavailable,

    CreationFailed,

    InvalidHandle,

    Unsupported
};

// Graphics API a window can present to: None (no surface) or Vulkan (GLFW
// windows, and headless windows when built with Vulkan support).
enum class SurfaceApi : std::uint32_t
{
    None = 0,
    Vulkan
};

// Subset of keys the module reports (translated from GLFW). Values must stay
// below 64: key state is tracked in a 64-bit mask (see keyStateMask), and the
// static_assert below enforces that.
enum class Key : std::uint32_t
{
    Unknown = 0,
    Escape,
    Space,
    W,
    A,
    S,
    D,
    Left,
    Right,
    Up,
    Down,

    LeftShift,
    LeftControl,

    Q,
    E,
    R,
    F,
    Z,
    X,
    C,
    V,
    B,
    N,
    M,
    T,
    G,
    Y,
    U,
    I,
    O,
    P,
    J,
    K,
    L,
    H,
    Num0,
    Num1,
    Num2,
    Num3,
    Num4,
    Num5,
    Num6,
    Num7,
    Num8,
    Num9,
    Tab,
    Enter,
    Backspace,
    LeftAlt,
    RightShift,
    RightControl,
    RightAlt
};

static_assert(static_cast<std::uint32_t>(Key::RightAlt) < 64, "Window key masks require Key values below 64");

// Normal shows the OS cursor; Captured hides and confines it (raw motion
// where the platform supports it) for camera-style controls.
enum class CursorMode : std::uint32_t
{
    Normal = 0,
    Captured
};

// Mouse buttons reported by isMouseButtonDown and the mouse button events.
// Values match the GLFW button ordering.
enum class MouseButton : std::uint32_t
{
    Left = 0,
    Right,
    Middle,
    Back,
    Forward,
    Button6,
    Button7,
    Button8
};

// Standard gamepad layout follows GLFW's standardized Xbox-style mapping.
enum class GamepadButton : std::uint32_t
{
    A = 0,
    B,
    X,
    Y,
    LeftBumper,
    RightBumper,
    Back,
    Start,
    Guide,
    LeftThumb,
    RightThumb,
    DpadUp,
    DpadRight,
    DpadDown,
    DpadLeft
};

// Stick axes are signed [-1, 1]; trigger axes are unipolar [0, 1] with 0
// meaning released. The GLFW backend normalizes its raw trigger values.
enum class GamepadAxis : std::uint32_t
{
    LeftX = 0,
    LeftY,
    RightX,
    RightY,
    LeftTrigger,
    RightTrigger
};

// Event kinds queued per window and drained with popEvent(s).
// Resize and cursor events may be coalesced and scroll events accumulated
// (see queueWindowEvent); size fields are in pixels.
enum class WindowEventKind : std::uint32_t
{
    CloseRequested = 0,

    Resized,

    FramebufferResized,

    FocusGained,
    FocusLost,
    Minimized,
    Restored,
    KeyPressed,
    KeyReleased,
    MouseButtonPressed,
    MouseButtonReleased,
    CursorMoved,
    MouseScrolled,
    TextInput,
    GamepadConnected,
    GamepadDisconnected,
    GamepadButtonPressed,
    GamepadButtonReleased,
    GamepadAxisMoved,
    TouchBegan,
    TouchMoved,
    TouchEnded,
    TouchCancelled,
    FilesDropped
};

// One queued window event. Only the fields relevant to kind are filled:
// width/height for Resized, framebufferWidth/Height for
// FramebufferResized, key for key events, mouseButton for mouse button
// events.
struct WindowEvent
{
    WindowEventKind kind = WindowEventKind::CloseRequested;

    std::uint32_t width  = 0;
    std::uint32_t height = 0;

    std::uint32_t framebufferWidth  = 0;
    std::uint32_t framebufferHeight = 0;

    Key key = Key::Unknown;

    MouseButton mouseButton = MouseButton::Left;

    double cursorX = 0.0;
    double cursorY = 0.0;
    double scrollX = 0.0;
    double scrollY = 0.0;
    std::uint32_t codepoint = 0;

    // Relative cursor motion accumulated when adjacent CursorMoved events
    // coalesce. The first event after creation/focus gain has zero delta.
    double cursorDeltaX = 0.0;
    double cursorDeltaY = 0.0;

    // Standardized gamepad controls. `gamepadId` is the GLFW joystick id
    // (0..GLFW_JOYSTICK_LAST); stick axes are [-1, 1], trigger axes [0, 1],
    // and down is meaningful for buttons.
    std::uint32_t gamepadId = 0;
    std::uint32_t gamepadControl = 0;
    float gamepadValue = 0.0f;
    bool gamepadDown = false;

    // Touch coordinates are normalized to the drawable window area [0, 1].
    // touchId identifies one contact on the source touch device.
    std::uint32_t touchDevice = 0;
    std::uint32_t touchId = 0;
    float touchX = 0.0f;
    float touchY = 0.0f;
    float touchPressure = 1.0f;
    float touchDeltaX = 0.0f;
    float touchDeltaY = 0.0f;

    // FilesDropped identifies a retained batch. Paths are copied separately
    // through copyDroppedFiles and remain valid until releaseDroppedFiles.
    std::uint64_t fileDropBatchId = 0;
    std::uint32_t fileDropCount = 0;
};

// A platform adapter (Android, iOS, SDL, etc.) submits touch contacts through
// WindowCapability::submitTouchEvent. Coordinates and pressure are normalized
// to [0, 1], with the origin at the top-left. Each window tracks at most `maximumTouchContactsPerWindow`
// simultaneous contacts. A contact id is stable from Began to Ended or
// Cancelled and may be reused only after it ends. `invalidTouchId` is reserved.
enum class TouchPhase : std::uint32_t
{
    Began = 0,
    Moved,
    Ended,
    Cancelled
};

struct TouchEventDescription
{
    std::uint32_t device = 0;
    std::uint32_t touchId = 0;
    TouchPhase phase = TouchPhase::Began;
    float x = 0.0f;
    float y = 0.0f;
    float pressure = 1.0f;
};

// Creation parameters. The has* flags distinguish "caller set this" from the
// struct default so module configuration only fills genuinely unset fields;
// size limits of 0 mean "no limit".
struct WindowDescription
{
    const char *title = nullptr;

    std::uint32_t width     = 0;
    std::uint32_t height    = 0;
    bool          resizable = true;

    bool         hasPosition = false;
    std::int32_t x           = 0;
    std::int32_t y           = 0;

    std::uint32_t minimumWidth  = 0;
    std::uint32_t minimumHeight = 0;
    std::uint32_t maximumWidth  = 0;
    std::uint32_t maximumHeight = 0;

    bool visible   = true;
    bool decorated = true;
    bool floating  = false;

    bool hasResizable = false;
    bool hasVisible   = false;
    bool hasDecorated = false;
    bool hasFloating  = false;

    // Queue capacity per window. Only consulted when hasEventQueueCapacity
    // is set (0 there means unbounded); otherwise the configured
    // [window] event_queue_capacity default applies.
    std::uint32_t eventQueueCapacity    = 4096;
    bool          hasEventQueueCapacity = false;

    bool ignoreConfiguredLimits = false;
};

// CPU-side view of a headless window's surface: 8-bit RGBA with
// strideBytes = width * 4. The buffer is owned by the window and valid
// between lock/unlock (resizes invalidate it).
struct SurfacePixels
{
    std::uint8_t *rgba        = nullptr;
    std::uint32_t width       = 0;
    std::uint32_t height      = 0;
    std::size_t   strideBytes = 0;
};

// Platform window handles for creating native Vulkan surfaces (Win32/X11);
// members for other platforms are null.
struct NativeWindowHandles
{
    void *win32Hwnd      = nullptr;
    void *win32Hinstance = nullptr;

    void *x11Window  = nullptr;
    void *x11Display = nullptr;
};

// The primary "window" capability: window lifecycle, a per-window event
// queue, keyboard state, and cursor control. self is the first argument of
// every call. pollEvents pumps the OS queue once per frame; events are then
// drained per window with popEvent/popEvents.
struct WindowCapability
{
    static constexpr const char *capabilityName = "window";

    std::uint32_t abi = windowCapabilityAbi;

    void *self = nullptr;

    const char *(*activeBackendName)(void *self);

    SurfaceApi (*surfaceApi)(void *self, WindowHandle window);

    WindowHandle (*createWindow)(void *self, const WindowDescription &description, WindowError *error);
    void (*destroyWindow)(void *self, WindowHandle window);

    bool (*shouldClose)(void *self, WindowHandle window);
    void (*requestClose)(void *self, WindowHandle window);

    void (*setTitle)(void *self, WindowHandle window, const char *title);

    bool (*setSize)(void *self, WindowHandle window, std::uint32_t width, std::uint32_t height);

    bool (*setPosition)(void *self, WindowHandle window, std::int32_t x, std::int32_t y);
    void (*getSize)(void *self, WindowHandle window, std::uint32_t *outWidth, std::uint32_t *outHeight);
    void (*getFramebufferSize)(void *self, WindowHandle window, std::uint32_t *outWidth, std::uint32_t *outHeight);

    void (*pollEvents)(void *self);
    bool (*popEvent)(void *self, WindowHandle window, WindowEvent *outEvent);

    std::uint32_t (*popEvents)(void *self, WindowHandle window, WindowEvent *outEvents, std::uint32_t capacity);

    bool (*isKeyDown)(void *self, WindowHandle window, Key key);

    // 64-bit mask with bit (1 << Key) set for pressed keys; the result is
    // the stored mask ANDed with requestedMask (0 selects nothing).
    std::uint64_t (*keyStateMask)(void *self, WindowHandle window, std::uint64_t requestedMask);

    // Live mouse button state, mirrored from button events the same way
    // isKeyDown mirrors key events. Always false on backends without a
    // pointing device (headless).
    bool (*isMouseButtonDown)(void *self, WindowHandle window, MouseButton button);

    // Headless backend only: maps the software RGBA buffer (fails otherwise).
    bool (*lockSurfacePixels)(void *self, WindowHandle window, SurfacePixels *outPixels);
    void (*unlockSurfacePixels)(void *self, WindowHandle window);

    void (*setCursorMode)(void *self, WindowHandle window, CursorMode mode);

    bool (*cursorPosition)(void *self, WindowHandle window, double *outX, double *outY);

    // Copies UTF-8 clipboard text. Text is capped at 1 MiB; copyClipboardText
    // writes the required buffer size including NUL to outRequired and performs
    // no partial copy when the supplied buffer is too small.
    bool (*setClipboardText)(void *self, WindowHandle window, const char *text);
    bool (*copyClipboardText)(void *self, WindowHandle window, char *output, std::uint32_t capacity,
        std::uint32_t *outRequired);

    // Platform input adapters use this to forward native multitouch contacts
    // into the same per-window event queue used by GLFW. It is intentionally
    // backend-independent so mobile hosts do not need to emulate mouse input.
    bool (*submitTouchEvent)(void *self, WindowHandle window, const TouchEventDescription *event);

    // A FilesDropped event exposes a batch id and count. copyDroppedFiles
    // writes NUL-separated UTF-8 paths (including each trailing NUL) and
    // reports the required byte count; it never performs a partial copy.
    // The batch must be released after use. Platform adapters may use
    // submitFileDropEvent to forward their native drop callback.
    bool (*copyDroppedFiles)(void *self, WindowHandle window, std::uint64_t batchId,
        char *output, std::uint32_t capacity, std::uint32_t *outRequired);
    bool (*releaseDroppedFiles)(void *self, WindowHandle window, std::uint64_t batchId);
    bool (*submitFileDropEvent)(void *self, WindowHandle window,
        const char *const *paths, std::uint32_t count);
};

// Secondary "window.vulkan" capability for render modules: the instance
// extensions to enable and the native handles a VkSurfaceKHR needs.
struct VulkanSurfaceCapability
{
    static constexpr const char *capabilityName = "window.vulkan";

    std::uint32_t abi = windowCapabilityAbi;

    void *self = nullptr;

    const char *const *(*requiredInstanceExtensions)(void *self, std::uint32_t *outCount);

    bool (*nativeSurfaceHandles)(void *self, WindowHandle window, NativeWindowHandles *outHandles);
};

// Secondary "window.glfw" capability for consumers that talk to GLFW
// directly: the version string and the raw GLFWwindow pointer.
struct GlfwWindowCapability
{
    static constexpr const char *capabilityName = "window.glfw";

    std::uint32_t abi = windowCapabilityAbi;

    void *self = nullptr;

    const char *(*glfwVersion)(void *self);

    void *(*glfwWindowPointer)(void *self, WindowHandle window);

    bool (*nativeHandles)(void *self, WindowHandle window, NativeWindowHandles *outHandles);
};

// Secondary "window.headless" capability: writes the software surface to a
// binary PPM (P6) file, used for CI snapshot checks.
struct HeadlessWindowCapability
{
    static constexpr const char *capabilityName = "window.headless";

    std::uint32_t abi = windowCapabilityAbi;

    void *self = nullptr;

    bool (*exportSurfacePpm)(void *self, WindowHandle window, const char *filePath, WindowError *error);
};

} // namespace window
} // namespace engine

#endif
