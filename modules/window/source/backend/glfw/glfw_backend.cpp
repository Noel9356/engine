// GLFW implementation of the window module's WindowBackend: window creation
// with hints, callback wiring into the per-window event queues, key mapping,
// cursor modes, runtime window control, and Vulkan surface support
// (VK_KHR_surface plus the platform surface extension).

#if defined(ENGINE_WINDOW_BACKEND_GLFW)

#include "glfw_backend.hpp"
#include "glfw_key_map.hpp"

#define GLFW_INCLUDE_NONE
#if defined(_WIN32)
#define GLFW_EXPOSE_NATIVE_WIN32
#elif defined(__linux__)
#define GLFW_EXPOSE_NATIVE_X11
#endif
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

#if defined(_WIN32)
#include <windows.h>
#endif

#undef None

namespace engine
{
namespace window
{

namespace
{

using glfw_keys::mapEngineKey;
using glfw_keys::mapGlfwKey;

// Bit for a Key in the 64-bit key state mask; 0 for Unknown/out-of-range.
std::uint64_t keyMaskBit(Key key)
{
    const std::uint32_t value = static_cast<std::uint32_t>(key);
    return value > 0 && value < 64 ? (std::uint64_t{1} << value) : 0;
}

// Bit for a MouseButton in the mouse button state mask; 0 when out of range.
std::uint8_t mouseButtonMaskBit(MouseButton button)
{
    const std::uint32_t value = static_cast<std::uint32_t>(button);
    return value < 8 ? static_cast<std::uint8_t>(std::uint8_t{1} << value) : std::uint8_t{0};
}

// Rebuilds the held-mouse-button mask from GLFW's live button state. Used at
// creation and on focus gain: while unfocused, button events are missed.
// GLFW exposes eight buttons; every tracked slot is probed.
void synchronizeMouseButtonState(GLFWwindow *window, WindowRecord &record)
{
    std::uint8_t mask = 0;
    constexpr std::uint32_t maximumButton = static_cast<std::uint32_t>(MouseButton::Button8);
    for (std::uint32_t value = 0; value <= maximumButton; ++value)
    {
        if (glfwGetMouseButton(window, static_cast<int>(value)) == GLFW_PRESS)
        {
            mask |= mouseButtonMaskBit(static_cast<MouseButton>(value));
        }
    }
    record.mouseButtonStateMask = mask;
}

// Rebuilds the pressed-key mask from GLFW's live key state. Used at creation
// and on focus gain: while unfocused, key events are missed entirely.
void synchronizeKeyState(GLFWwindow *window, WindowRecord &record)
{
    std::uint64_t       mask       = 0;
    const std::uint32_t maximumKey = static_cast<std::uint32_t>(Key::Slash);
    for (std::uint32_t value = 1; value <= maximumKey; ++value)
    {
        const Key key     = static_cast<Key>(value);
        const int glfwKey = mapEngineKey(key);
        if (glfwKey != GLFW_KEY_UNKNOWN && glfwGetKey(window, glfwKey) == GLFW_PRESS)
        {
            mask |= keyMaskBit(key);
        }
    }
    record.keyStateMask = mask;
}

// GLFW callbacks: resolve the WindowRecord from the window's user pointer
// and translate the event into the record's queue. They fire inside
// glfwPollEvents()/glfwWaitEvents(), i.e. during the module's event pump.

// Close button pressed: queue CloseRequested (the window stays alive).
void glfwCloseCallback(GLFWwindow *glfwWindow)
{
    auto *record = static_cast<WindowRecord *>(glfwGetWindowUserPointer(glfwWindow));
    if (record != nullptr)
    {
        queueWindowEvent(*record, WindowEvent{.kind = WindowEventKind::CloseRequested});
    }
}

// Size <= 0 means minimized; otherwise update the record and queue Resized.
void glfwSizeCallback(GLFWwindow *glfwWindow, int width, int height)
{
    auto *record = static_cast<WindowRecord *>(glfwGetWindowUserPointer(glfwWindow));
    if (record == nullptr)
    {
        return;
    }
    if (width <= 0 || height <= 0)
    {
        queueWindowEvent(*record, WindowEvent{.kind = WindowEventKind::Minimized});
        return;
    }
    record->width  = static_cast<std::uint32_t>(width);
    record->height = static_cast<std::uint32_t>(height);
    queueWindowEvent(
        *record, WindowEvent{.kind = WindowEventKind::Resized, .width = record->width, .height = record->height});
}

// Swapchain-size change: positive sizes update the record and queue
// FramebufferResized.
void glfwFramebufferSizeCallback(GLFWwindow *glfwWindow, int width, int height)
{
    auto *record = static_cast<WindowRecord *>(glfwGetWindowUserPointer(glfwWindow));
    if (record == nullptr || width <= 0 || height <= 0)
    {
        return;
    }
    record->framebufferWidth  = static_cast<std::uint32_t>(width);
    record->framebufferHeight = static_cast<std::uint32_t>(height);
    queueWindowEvent(*record,
        WindowEvent{
            .kind              = WindowEventKind::FramebufferResized,
            .framebufferWidth  = record->framebufferWidth,
            .framebufferHeight = record->framebufferHeight});
}

// Focus gain re-syncs the key mask (missed events are compensated), focus
// loss clears it so no key sticks. Mouse buttons are re-synced/cleared the
// same way.
void glfwFocusCallback(GLFWwindow *glfwWindow, int focused)
{
    auto *record = static_cast<WindowRecord *>(glfwGetWindowUserPointer(glfwWindow));
    if (record != nullptr)
    {
        if (focused == GLFW_TRUE)
        {
            synchronizeKeyState(glfwWindow, *record);
            synchronizeMouseButtonState(glfwWindow, *record);
            record->cursorPositionValid = false;
        }
        else
        {
            record->keyStateMask         = 0;
            record->mouseButtonStateMask = 0;
            record->cursorPositionValid = false;
            cancelActiveTouches(*record);
        }
        queueWindowEvent(
            *record, WindowEvent{.kind = focused ? WindowEventKind::FocusGained : WindowEventKind::FocusLost});
    }
}

// Minimize/restore transitions.
void glfwIconifyCallback(GLFWwindow *glfwWindow, int iconified)
{
    auto *record = static_cast<WindowRecord *>(glfwGetWindowUserPointer(glfwWindow));
    if (record != nullptr)
    {
        record->iconified = iconified == GLFW_TRUE;
        queueWindowEvent(
            *record, WindowEvent{.kind = iconified ? WindowEventKind::Minimized : WindowEventKind::Restored});
    }
}

// Press/release updates the key mask and queues a key event; repeat actions
// are ignored.
void glfwKeyCallback(GLFWwindow *glfwWindow, int key, int, int action, int)
{
    auto *record = static_cast<WindowRecord *>(glfwGetWindowUserPointer(glfwWindow));
    if (record == nullptr)
    {
        return;
    }
    if (action != GLFW_PRESS && action != GLFW_RELEASE)
    {
        return;
    }

    const Key           mappedKey = mapGlfwKey(key);
    const std::uint64_t bit       = keyMaskBit(mappedKey);
    if (action == GLFW_PRESS)
    {
        record->keyStateMask |= bit;
    }
    else
    {
        record->keyStateMask &= ~bit;
    }
    queueWindowEvent(*record,
        WindowEvent{
            .kind = action == GLFW_PRESS ? WindowEventKind::KeyPressed : WindowEventKind::KeyReleased,
            .key  = mappedKey});
}

// Press/release updates the mouse button mask and queues a button event.
void glfwMouseButtonCallback(GLFWwindow *glfwWindow, int button, int action, int)
{
    auto *record = static_cast<WindowRecord *>(glfwGetWindowUserPointer(glfwWindow));
    if (record == nullptr)
    {
        return;
    }
    if (action != GLFW_PRESS && action != GLFW_RELEASE)
    {
        return;
    }
    if (button < 0 || button > 7)
    {
        return;
    }

    const MouseButton mappedButton = static_cast<MouseButton>(static_cast<std::uint32_t>(button));
    const std::uint8_t bit         = mouseButtonMaskBit(mappedButton);
    if (action == GLFW_PRESS)
    {
        record->mouseButtonStateMask |= bit;
    }
    else
    {
        record->mouseButtonStateMask &= static_cast<std::uint8_t>(~bit);
    }
    queueWindowEvent(*record,
        WindowEvent{
            .kind        = action == GLFW_PRESS ? WindowEventKind::MouseButtonPressed
                                 : WindowEventKind::MouseButtonReleased,
            .mouseButton = mappedButton});
}

// Cursor coordinates use GLFW's logical window coordinate system.
void glfwCursorPositionCallback(GLFWwindow *glfwWindow, double x, double y)
{
    auto *record = static_cast<WindowRecord *>(glfwGetWindowUserPointer(glfwWindow));
    if (record == nullptr)
        return;

    const double deltaX = record->cursorPositionValid ? x - record->lastCursorX : 0.0;
    const double deltaY = record->cursorPositionValid ? y - record->lastCursorY : 0.0;
    record->lastCursorX = x;
    record->lastCursorY = y;
    record->cursorPositionValid = true;
    queueWindowEvent(*record,
        WindowEvent{.kind = WindowEventKind::CursorMoved,
            .cursorX = x,
            .cursorY = y,
            .cursorDeltaX = deltaX,
            .cursorDeltaY = deltaY});
}

// Scroll offsets are accumulated when consecutive scroll callbacks arrive
// before the application drains the event queue.
void glfwScrollCallback(GLFWwindow *glfwWindow, double xOffset, double yOffset)
{
    auto *record = static_cast<WindowRecord *>(glfwGetWindowUserPointer(glfwWindow));
    if (record != nullptr)
        queueWindowEvent(*record, WindowEvent{.kind = WindowEventKind::MouseScrolled, .scrollX = xOffset, .scrollY = yOffset});
}

// GLFW delivers all dropped paths for one user action in a single callback.
// The window record owns a bounded copy so the application's event lifetime
// does not depend on GLFW's callback buffer lifetime.
void glfwDropCallback(GLFWwindow *glfwWindow, int pathCount, const char **paths)
{
    auto *record = static_cast<WindowRecord *>(glfwGetWindowUserPointer(glfwWindow));
    if (record == nullptr || pathCount <= 0)
        return;
    queueDroppedFileBatch(*record, paths, static_cast<std::uint32_t>(pathCount));
}

// GLFW supplies Unicode code points, not key codes; preserve the scalar value
// so text entry and international input do not depend on keyboard layout.
void glfwCharacterCallback(GLFWwindow *glfwWindow, unsigned int codepoint)
{
    auto *record = static_cast<WindowRecord *>(glfwGetWindowUserPointer(glfwWindow));
    if (record != nullptr && codepoint <= 0x10ffffu && !(codepoint >= 0xd800u && codepoint <= 0xdfffu))
        queueWindowEvent(*record, WindowEvent{.kind = WindowEventKind::TextInput, .codepoint = codepoint});
}

// Tracks whether glfwInit succeeded (GLFW state is process-global).
bool initialized = false;

} // namespace

const char *glfwBackendVersionString()
{
    return glfwGetVersionString();
}

// GLFW windows present through Vulkan only (GLFW_CLIENT_API is NO_API).
SurfaceApi GlfwWindowBackend::surfaceApi(const WindowRecord &) const
{
    return SurfaceApi::Vulkan;
}

bool GlfwWindowBackend::initialize()
{
    if (initialized)
    {
        return true;
    }
    if (glfwInit() != GLFW_TRUE)
    {
        return false;
    }
    initialized = true;
    return true;
}

void GlfwWindowBackend::shutdown()
{
    windows_.clear();
    gamepads_ = {};
    if (initialized)
    {
        glfwTerminate();
        initialized = false;
    }
}

// Creates the OS window with rendering disabled (NO_API), applies the
// description as hints, installs the callbacks, seeds the key mask when the
// window is created focused, and reads the initial framebuffer size.
bool GlfwWindowBackend::createWindow(WindowRecord &record, const WindowDescription &description)
{
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, description.resizable ? GLFW_TRUE : GLFW_FALSE);
    glfwWindowHint(GLFW_VISIBLE, description.visible ? GLFW_TRUE : GLFW_FALSE);
    glfwWindowHint(GLFW_DECORATED, description.decorated ? GLFW_TRUE : GLFW_FALSE);
    glfwWindowHint(GLFW_FLOATING, description.floating ? GLFW_TRUE : GLFW_FALSE);
    glfwWindowHint(GLFW_MAXIMIZED, description.maximized ? GLFW_TRUE : GLFW_FALSE);
    glfwWindowHint(GLFW_FOCUS_ON_SHOW, description.focusOnShow ? GLFW_TRUE : GLFW_FALSE);

    GLFWwindow *glfwWindow
        = glfwCreateWindow((int)record.width, (int)record.height, record.title.c_str(), nullptr, nullptr);
    if (glfwWindow == nullptr)
    {
        return false;
    }

    record.platformWindow = glfwWindow;
    try
    {
        windows_.push_back(&record);
    }
    catch (...)
    {
        glfwDestroyWindow(glfwWindow);
        record.platformWindow = nullptr;
        return false;
    }
    if (description.hasPosition)
    {
        glfwSetWindowPos(glfwWindow, description.x, description.y);
    }
    if (description.minimumWidth != 0 || description.minimumHeight != 0 || description.maximumWidth != 0
        || description.maximumHeight != 0)
    {
        glfwSetWindowSizeLimits(glfwWindow,
            description.minimumWidth != 0 ? static_cast<int>(std::min(description.minimumWidth,
                                                static_cast<std::uint32_t>(std::numeric_limits<int>::max())))
                                          : GLFW_DONT_CARE,
            description.minimumHeight != 0 ? static_cast<int>(std::min(description.minimumHeight,
                                                 static_cast<std::uint32_t>(std::numeric_limits<int>::max())))
                                           : GLFW_DONT_CARE,
            description.maximumWidth != 0 ? static_cast<int>(std::min(description.maximumWidth,
                                                static_cast<std::uint32_t>(std::numeric_limits<int>::max())))
                                          : GLFW_DONT_CARE,
            description.maximumHeight != 0 ? static_cast<int>(std::min(description.maximumHeight,
                                                 static_cast<std::uint32_t>(std::numeric_limits<int>::max())))
                                           : GLFW_DONT_CARE);
    }

    glfwSetWindowUserPointer(glfwWindow, &record);
    glfwSetWindowCloseCallback(glfwWindow, glfwCloseCallback);
    glfwSetWindowSizeCallback(glfwWindow, glfwSizeCallback);
    glfwSetFramebufferSizeCallback(glfwWindow, glfwFramebufferSizeCallback);
    glfwSetWindowFocusCallback(glfwWindow, glfwFocusCallback);
    glfwSetWindowIconifyCallback(glfwWindow, glfwIconifyCallback);
    glfwSetKeyCallback(glfwWindow, glfwKeyCallback);
    glfwSetMouseButtonCallback(glfwWindow, glfwMouseButtonCallback);
    glfwSetCursorPosCallback(glfwWindow, glfwCursorPositionCallback);
    glfwSetScrollCallback(glfwWindow, glfwScrollCallback);
    glfwSetCharCallback(glfwWindow, glfwCharacterCallback);
    glfwSetDropCallback(glfwWindow, glfwDropCallback);
    if (glfwGetWindowAttrib(glfwWindow, GLFW_FOCUSED) == GLFW_TRUE)
    {
        synchronizeKeyState(glfwWindow, record);
        synchronizeMouseButtonState(glfwWindow, record);
    }

    // Track the requested state; getWindowAttributes prefers GLFW's live
    // attributes whenever they are available.
    record.visible   = glfwGetWindowAttrib(glfwWindow, GLFW_VISIBLE) == GLFW_TRUE;
    record.iconified = glfwGetWindowAttrib(glfwWindow, GLFW_ICONIFIED) == GLFW_TRUE;
    record.maximized = glfwGetWindowAttrib(glfwWindow, GLFW_MAXIMIZED) == GLFW_TRUE;
    record.opacity   = std::clamp(description.hasOpacity ? description.opacity : 1.0f, 0.0f, 1.0f);
    if (description.hasOpacity)
    {
        glfwSetWindowOpacity(glfwWindow, record.opacity);
    }

    int framebufferWidth  = 0;
    int framebufferHeight = 0;
    glfwGetFramebufferSize(glfwWindow, &framebufferWidth, &framebufferHeight);
    record.framebufferWidth  = framebufferWidth > 0 ? (std::uint32_t)framebufferWidth : record.width;
    record.framebufferHeight = framebufferHeight > 0 ? (std::uint32_t)framebufferHeight : record.height;
    return true;
}

void GlfwWindowBackend::destroyWindow(WindowRecord &record)
{
    windows_.erase(std::remove(windows_.begin(), windows_.end(), &record), windows_.end());
    if (record.platformWindow != nullptr)
    {
        glfwDestroyWindow(static_cast<GLFWwindow *>(record.platformWindow));
        record.platformWindow = nullptr;
    }
}

bool GlfwWindowBackend::shouldClose(const WindowRecord &record) const
{
    if (record.platformWindow != nullptr)
    {
        return glfwWindowShouldClose(static_cast<GLFWwindow *>(record.platformWindow)) == GLFW_TRUE;
    }
    return record.closeRequested;
}

void GlfwWindowBackend::requestClose(WindowRecord &record)
{
    if (record.platformWindow != nullptr)
    {
        glfwSetWindowShouldClose(static_cast<GLFWwindow *>(record.platformWindow), GLFW_TRUE);
        return;
    }
    record.closeRequested = true;
}

void GlfwWindowBackend::setTitle(WindowRecord &record, const char *title)
{
    if (title == nullptr)
    {
        return;
    }
    record.title = title;
    if (record.platformWindow != nullptr)
    {
        glfwSetWindowTitle(static_cast<GLFWwindow *>(record.platformWindow), record.title.c_str());
    }
}

bool GlfwWindowBackend::setSize(WindowRecord &record, std::uint32_t width, std::uint32_t height)
{
    if (record.platformWindow == nullptr || width == 0 || height == 0
        || width > static_cast<std::uint32_t>(std::numeric_limits<int>::max())
        || height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()))
    {
        return false;
    }
    glfwSetWindowSize(
        static_cast<GLFWwindow *>(record.platformWindow), static_cast<int>(width), static_cast<int>(height));
    return true;
}

bool GlfwWindowBackend::setPosition(WindowRecord &record, std::int32_t x, std::int32_t y)
{
    if (record.platformWindow == nullptr)
    {
        return false;
    }
    glfwSetWindowPos(static_cast<GLFWwindow *>(record.platformWindow), x, y);
    return true;
}

bool GlfwWindowBackend::getPosition(const WindowRecord &record, std::int32_t *outX, std::int32_t *outY) const
{
    if (record.platformWindow == nullptr || outX == nullptr || outY == nullptr)
    {
        return false;
    }
    int x = 0;
    int y = 0;
    glfwGetWindowPos(static_cast<GLFWwindow *>(record.platformWindow), &x, &y);
    *outX = static_cast<std::int32_t>(x);
    *outY = static_cast<std::int32_t>(y);
    return true;
}

void GlfwWindowBackend::setOpacity(WindowRecord &record, float opacity)
{
    record.opacity = std::clamp(opacity, 0.0f, 1.0f);
    if (record.platformWindow != nullptr)
    {
        glfwSetWindowOpacity(static_cast<GLFWwindow *>(record.platformWindow), record.opacity);
    }
}

void GlfwWindowBackend::setVisible(WindowRecord &record, bool visible)
{
    record.visible = visible;
    if (record.platformWindow == nullptr)
    {
        return;
    }
    GLFWwindow *window = static_cast<GLFWwindow *>(record.platformWindow);
    if (visible)
    {
        glfwShowWindow(window);
    }
    else
    {
        glfwHideWindow(window);
    }
}

void GlfwWindowBackend::setResizable(WindowRecord &record, bool resizable)
{
    if (record.platformWindow != nullptr)
    {
        glfwSetWindowAttrib(static_cast<GLFWwindow *>(record.platformWindow), GLFW_RESIZABLE,
            resizable ? GLFW_TRUE : GLFW_FALSE);
    }
}

void GlfwWindowBackend::setDecorated(WindowRecord &record, bool decorated)
{
    if (record.platformWindow != nullptr)
    {
        glfwSetWindowAttrib(static_cast<GLFWwindow *>(record.platformWindow), GLFW_DECORATED,
            decorated ? GLFW_TRUE : GLFW_FALSE);
    }
}

void GlfwWindowBackend::setFloating(WindowRecord &record, bool floating)
{
    if (record.platformWindow != nullptr)
    {
        glfwSetWindowAttrib(static_cast<GLFWwindow *>(record.platformWindow), GLFW_FLOATING,
            floating ? GLFW_TRUE : GLFW_FALSE);
    }
}

void GlfwWindowBackend::focus(const WindowRecord &record)
{
    if (record.platformWindow != nullptr)
    {
        glfwFocusWindow(static_cast<GLFWwindow *>(record.platformWindow));
    }
}

void GlfwWindowBackend::requestAttention(const WindowRecord &record)
{
    if (record.platformWindow != nullptr)
    {
        glfwRequestWindowAttention(static_cast<GLFWwindow *>(record.platformWindow));
    }
}

void GlfwWindowBackend::iconify(WindowRecord &record)
{
    record.iconified = true;
    if (record.platformWindow != nullptr)
    {
        glfwIconifyWindow(static_cast<GLFWwindow *>(record.platformWindow));
    }
}

void GlfwWindowBackend::restore(WindowRecord &record)
{
    record.iconified = false;
    record.maximized = false;
    if (record.platformWindow != nullptr)
    {
        glfwRestoreWindow(static_cast<GLFWwindow *>(record.platformWindow));
    }
}

void GlfwWindowBackend::maximize(WindowRecord &record)
{
    record.maximized = true;
    if (record.platformWindow != nullptr)
    {
        glfwMaximizeWindow(static_cast<GLFWwindow *>(record.platformWindow));
    }
}

bool GlfwWindowBackend::getWindowAttributes(const WindowRecord &record, WindowAttributes *outAttributes) const
{
    if (record.platformWindow == nullptr || outAttributes == nullptr)
    {
        return false;
    }
    GLFWwindow *window = static_cast<GLFWwindow *>(record.platformWindow);

    *outAttributes                    = {};
    outAttributes->focused            = glfwGetWindowAttrib(window, GLFW_FOCUSED) == GLFW_TRUE;
    outAttributes->visible            = glfwGetWindowAttrib(window, GLFW_VISIBLE) == GLFW_TRUE;
    outAttributes->iconified          = glfwGetWindowAttrib(window, GLFW_ICONIFIED) == GLFW_TRUE;
    outAttributes->maximized          = glfwGetWindowAttrib(window, GLFW_MAXIMIZED) == GLFW_TRUE;
    outAttributes->resizable          = glfwGetWindowAttrib(window, GLFW_RESIZABLE) == GLFW_TRUE;
    outAttributes->decorated          = glfwGetWindowAttrib(window, GLFW_DECORATED) == GLFW_TRUE;
    outAttributes->floating           = glfwGetWindowAttrib(window, GLFW_FLOATING) == GLFW_TRUE;
    outAttributes->hover              = glfwGetWindowAttrib(window, GLFW_HOVERED) == GLFW_TRUE;
    outAttributes->opacity            = glfwGetWindowOpacity(window);
    glfwGetWindowContentScale(window, &outAttributes->contentScaleX, &outAttributes->contentScaleY);
    return true;
}

bool GlfwWindowBackend::getContentScale(const WindowRecord &record, float *outX, float *outY) const
{
    if (record.platformWindow == nullptr || outX == nullptr || outY == nullptr)
    {
        return false;
    }
    glfwGetWindowContentScale(static_cast<GLFWwindow *>(record.platformWindow), outX, outY);
    return true;
}

bool GlfwWindowBackend::rawMouseMotionSupported() const
{
    return glfwRawMouseMotionSupported() == GLFW_TRUE;
}

bool GlfwWindowBackend::setClipboardText(const WindowRecord &record, const char *text)
{
    if (record.platformWindow == nullptr || text == nullptr)
        return false;
    std::size_t length = 0;
    while (length <= maximumClipboardTextBytes && text[length] != '\0')
        ++length;
    if (length > maximumClipboardTextBytes || !isValidUtf8(text, length))
        return false;
    glfwSetClipboardString(static_cast<GLFWwindow *>(record.platformWindow), text);
    return true;
}

bool GlfwWindowBackend::copyClipboardText(const WindowRecord &record, const char **outText) const
{
    if (record.platformWindow == nullptr || outText == nullptr)
        return false;
    const char *text = glfwGetClipboardString(static_cast<GLFWwindow *>(record.platformWindow));
    if (text == nullptr)
        return false;
    std::size_t length = 0;
    while (length <= maximumClipboardTextBytes && text[length] != '\0')
        ++length;
    if (length > maximumClipboardTextBytes || !isValidUtf8(text, length))
        return false;
    *outText = text;
    return true;
}

void GlfwWindowBackend::pollEvents()
{
    glfwPollEvents();
    pollGamepads();
}

void GlfwWindowBackend::waitEvents(double timeoutSeconds)
{
    if (timeoutSeconds > 0.0)
    {
        glfwWaitEventsTimeout(timeoutSeconds);
    }
    else
    {
        glfwWaitEvents();
    }
    pollGamepads();
}

void GlfwWindowBackend::configureGamepads(const GamepadPollingOptions &options)
{
    gamepadOptions_ = options;
}

void GlfwWindowBackend::queueGamepadEvent(const WindowEvent &event)
{
    for (WindowRecord *record : windows_)
    {
        if (record != nullptr)
            queueWindowEvent(*record, event);
    }
}

// GLFW exposes standardized gamepad states through joystick polling rather
// than per-control callbacks. Poll once per engine event pump, compare against
// the previous snapshot, and publish only changed controls. Axis changes are
// coalesced per device/control by queueWindowEvent to bound queue growth.
void GlfwWindowBackend::pollGamepads()
{
    if (!gamepadOptions_.enabled)
    {
        return;
    }

    constexpr int buttonCount = GLFW_GAMEPAD_BUTTON_LAST + 1;
    constexpr int axisCount = GLFW_GAMEPAD_AXIS_LAST + 1;
    static_assert(buttonCount == 15);
    static_assert(axisCount == 6);

    for (int joystick = GLFW_JOYSTICK_1; joystick <= GLFW_JOYSTICK_LAST; ++joystick)
    {
        const std::size_t index = static_cast<std::size_t>(joystick - GLFW_JOYSTICK_1);
        GamepadSnapshot &previous = gamepads_[index];
        GLFWgamepadstate state{};
        const bool connected = glfwJoystickPresent(joystick) == GLFW_TRUE
            && glfwJoystickIsGamepad(joystick) == GLFW_TRUE
            && glfwGetGamepadState(joystick, &state) == GLFW_TRUE;

        if (!connected)
        {
            if (previous.connected)
            {
                queueGamepadEvent(WindowEvent{.kind = WindowEventKind::GamepadDisconnected,
                    .gamepadId = static_cast<std::uint32_t>(joystick)});
                previous = {};
            }
            continue;
        }

        if (!previous.connected)
        {
            previous = {};
            previous.connected = true;
            queueGamepadEvent(WindowEvent{.kind = WindowEventKind::GamepadConnected,
                .gamepadId = static_cast<std::uint32_t>(joystick)});
        }

        for (int button = 0; button < buttonCount; ++button)
        {
            const unsigned char down = state.buttons[button] == GLFW_PRESS ? 1u : 0u;
            if (down == previous.buttons[static_cast<std::size_t>(button)])
                continue;
            previous.buttons[static_cast<std::size_t>(button)] = down;
            queueGamepadEvent(WindowEvent{
                .kind = down != 0 ? WindowEventKind::GamepadButtonPressed : WindowEventKind::GamepadButtonReleased,
                .gamepadId = static_cast<std::uint32_t>(joystick),
                .gamepadControl = static_cast<std::uint32_t>(button),
                .gamepadDown = down != 0});
        }

        for (int axis = 0; axis < axisCount; ++axis)
        {
            const float rawValue = std::clamp(state.axes[axis], -1.0f, 1.0f);
            // GLFW exposes trigger axes in [-1, 1] with the released state at
            // -1. Convert them to the engine's conventional [0, 1] trigger
            // range; stick axes remain signed.
            const float value = axis >= static_cast<int>(GamepadAxis::LeftTrigger)
                ? (rawValue + 1.0f) * 0.5f
                : rawValue;
            float &oldValue = previous.axes[static_cast<std::size_t>(axis)];
            if (std::abs(value - oldValue) <= gamepadOptions_.axisEpsilon)
                continue;
            oldValue = value;
            queueGamepadEvent(WindowEvent{.kind = WindowEventKind::GamepadAxisMoved,
                .gamepadId = static_cast<std::uint32_t>(joystick),
                .gamepadControl = static_cast<std::uint32_t>(axis),
                .gamepadValue = value});
        }
    }
}

// Normal/Hidden/Captured map to GLFW_CURSOR normal/hidden/disabled; captured
// also enables raw mouse motion where the platform supports it (the support
// check keeps unsupported systems off GLFW's error path).
void GlfwWindowBackend::setCursorMode(const WindowRecord &record, CursorMode mode)
{
    if (record.platformWindow == nullptr)
    {
        return;
    }
    GLFWwindow *window = static_cast<GLFWwindow *>(record.platformWindow);

    int glfwMode = GLFW_CURSOR_NORMAL;
    if (mode == CursorMode::Hidden)
    {
        glfwMode = GLFW_CURSOR_HIDDEN;
    }
    else if (mode == CursorMode::Captured)
    {
        glfwMode = GLFW_CURSOR_DISABLED;
    }
    glfwSetInputMode(window, GLFW_CURSOR, glfwMode);

#if defined(GLFW_RAW_MOUSE_MOTION)

    const bool captureRaw = mode == CursorMode::Captured && glfwRawMouseMotionSupported() == GLFW_TRUE;
    if (captureRaw || glfwGetInputMode(window, GLFW_RAW_MOUSE_MOTION) == GLFW_TRUE)
    {
        glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, captureRaw ? GLFW_TRUE : GLFW_FALSE);
    }
#endif
}

bool GlfwWindowBackend::cursorPosition(const WindowRecord &record, double *outX, double *outY) const
{
    if (record.platformWindow == nullptr || outX == nullptr || outY == nullptr)
    {
        return false;
    }
    glfwGetCursorPos(static_cast<GLFWwindow *>(record.platformWindow), outX, outY);
    return true;
}

const char *const *GlfwWindowBackend::requiredInstanceExtensions(std::uint32_t *outCount) const
{
#if defined(_WIN32) || defined(__linux__)
    if (initialized)
    {
#if defined(_WIN32)
        static const char *const extensions[] = {"VK_KHR_surface", "VK_KHR_win32_surface"};
#else
        static const char *const extensions[] = {"VK_KHR_surface", "VK_KHR_xlib_surface"};
#endif
        if (outCount != nullptr)
        {
            *outCount = 2;
        }
        return extensions;
    }
#else
    (void)outCount;
#endif
    return nullptr;
}

bool GlfwWindowBackend::nativeSurfaceHandles(const WindowRecord &record, NativeWindowHandles *outHandles) const
{
    if (record.platformWindow == nullptr || outHandles == nullptr)
    {
        return false;
    }

    GLFWwindow *glfwWindow = static_cast<GLFWwindow *>(record.platformWindow);
#if defined(_WIN32)

    outHandles->win32Hwnd      = glfwGetWin32Window(glfwWindow);
    outHandles->win32Hinstance = GetModuleHandleW(nullptr);
#elif defined(__linux__)
    outHandles->x11Window  = reinterpret_cast<void *>(static_cast<std::uintptr_t>(glfwGetX11Window(glfwWindow)));
    outHandles->x11Display = reinterpret_cast<void *>(glfwGetX11Display());
#endif
    return true;
}

} // namespace window
} // namespace engine

#endif
