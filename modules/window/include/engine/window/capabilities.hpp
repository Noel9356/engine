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
// changes so consumers can reject mismatched providers.
inline constexpr std::uint32_t windowCapabilityAbi = 9u;

// Opaque window identifier; 0 (invalidWindow) means "none".
using WindowHandle = std::uint32_t;

inline constexpr WindowHandle invalidWindow = 0;

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

// Event kinds queued per window and drained with popEvent(s).
// Resized/FramebufferResized events are coalesced (see queueWindowEvent);
// size fields are in pixels.
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
    KeyReleased
};

// One queued window event. Only the fields relevant to kind are filled:
// width/height for Resized, framebufferWidth/Height for
// FramebufferResized, key for key events.
struct WindowEvent
{
    WindowEventKind kind = WindowEventKind::CloseRequested;

    std::uint32_t width  = 0;
    std::uint32_t height = 0;

    std::uint32_t framebufferWidth  = 0;
    std::uint32_t framebufferHeight = 0;

    Key key = Key::Unknown;
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

    // Headless backend only: maps the software RGBA buffer (fails otherwise).
    bool (*lockSurfacePixels)(void *self, WindowHandle window, SurfacePixels *outPixels);
    void (*unlockSurfacePixels)(void *self, WindowHandle window);

    void (*setCursorMode)(void *self, WindowHandle window, CursorMode mode);

    bool (*cursorPosition)(void *self, WindowHandle window, double *outX, double *outY);
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
