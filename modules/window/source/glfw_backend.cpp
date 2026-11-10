// GLFW implementation of the window module's WindowBackend: window creation
// with hints, callback wiring into the per-window event queues, key mapping,
// cursor modes, and Vulkan surface support (VK_KHR_surface plus the platform
// surface extension).

#if defined(ENGINE_WINDOW_BACKEND_GLFW)

#include "glfw_backend.hpp"

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

// GLFW key code -> engine Key (Unknown for keys the module does not track).
Key mapGlfwKey(int key)
{
    switch (key)
    {
    case GLFW_KEY_ESCAPE:
        return Key::Escape;
    case GLFW_KEY_SPACE:
        return Key::Space;
    case GLFW_KEY_W:
        return Key::W;
    case GLFW_KEY_A:
        return Key::A;
    case GLFW_KEY_S:
        return Key::S;
    case GLFW_KEY_D:
        return Key::D;
    case GLFW_KEY_LEFT:
        return Key::Left;
    case GLFW_KEY_RIGHT:
        return Key::Right;
    case GLFW_KEY_UP:
        return Key::Up;
    case GLFW_KEY_DOWN:
        return Key::Down;
    case GLFW_KEY_LEFT_SHIFT:
        return Key::LeftShift;
    case GLFW_KEY_LEFT_CONTROL:
        return Key::LeftControl;
    case GLFW_KEY_Q:
        return Key::Q;
    case GLFW_KEY_E:
        return Key::E;
    case GLFW_KEY_R:
        return Key::R;
    case GLFW_KEY_F:
        return Key::F;
    case GLFW_KEY_Z:
        return Key::Z;
    case GLFW_KEY_X:
        return Key::X;
    case GLFW_KEY_C:
        return Key::C;
    case GLFW_KEY_V:
        return Key::V;
    case GLFW_KEY_B:
        return Key::B;
    case GLFW_KEY_N:
        return Key::N;
    case GLFW_KEY_M:
        return Key::M;
    case GLFW_KEY_T:
        return Key::T;
    case GLFW_KEY_G:
        return Key::G;
    case GLFW_KEY_Y:
        return Key::Y;
    case GLFW_KEY_U:
        return Key::U;
    case GLFW_KEY_I:
        return Key::I;
    case GLFW_KEY_O:
        return Key::O;
    case GLFW_KEY_P:
        return Key::P;
    case GLFW_KEY_J:
        return Key::J;
    case GLFW_KEY_K:
        return Key::K;
    case GLFW_KEY_L:
        return Key::L;
    case GLFW_KEY_H:
        return Key::H;
    case GLFW_KEY_0:
        return Key::Num0;
    case GLFW_KEY_1:
        return Key::Num1;
    case GLFW_KEY_2:
        return Key::Num2;
    case GLFW_KEY_3:
        return Key::Num3;
    case GLFW_KEY_4:
        return Key::Num4;
    case GLFW_KEY_5:
        return Key::Num5;
    case GLFW_KEY_6:
        return Key::Num6;
    case GLFW_KEY_7:
        return Key::Num7;
    case GLFW_KEY_8:
        return Key::Num8;
    case GLFW_KEY_9:
        return Key::Num9;
    case GLFW_KEY_TAB:
        return Key::Tab;
    case GLFW_KEY_ENTER:
        return Key::Enter;
    case GLFW_KEY_BACKSPACE:
        return Key::Backspace;
    case GLFW_KEY_LEFT_ALT:
        return Key::LeftAlt;
    case GLFW_KEY_RIGHT_SHIFT:
        return Key::RightShift;
    case GLFW_KEY_RIGHT_CONTROL:
        return Key::RightControl;
    case GLFW_KEY_RIGHT_ALT:
        return Key::RightAlt;
    default:
        return Key::Unknown;
    }
}

// Engine Key -> GLFW key code (GLFW_KEY_UNKNOWN for Key::Unknown).
int mapEngineKey(Key key)
{
    switch (key)
    {
    case Key::Escape:
        return GLFW_KEY_ESCAPE;
    case Key::Space:
        return GLFW_KEY_SPACE;
    case Key::W:
        return GLFW_KEY_W;
    case Key::A:
        return GLFW_KEY_A;
    case Key::S:
        return GLFW_KEY_S;
    case Key::D:
        return GLFW_KEY_D;
    case Key::Left:
        return GLFW_KEY_LEFT;
    case Key::Right:
        return GLFW_KEY_RIGHT;
    case Key::Up:
        return GLFW_KEY_UP;
    case Key::Down:
        return GLFW_KEY_DOWN;
    case Key::LeftShift:
        return GLFW_KEY_LEFT_SHIFT;
    case Key::LeftControl:
        return GLFW_KEY_LEFT_CONTROL;
    case Key::Q:
        return GLFW_KEY_Q;
    case Key::E:
        return GLFW_KEY_E;
    case Key::R:
        return GLFW_KEY_R;
    case Key::F:
        return GLFW_KEY_F;
    case Key::Z:
        return GLFW_KEY_Z;
    case Key::X:
        return GLFW_KEY_X;
    case Key::C:
        return GLFW_KEY_C;
    case Key::V:
        return GLFW_KEY_V;
    case Key::B:
        return GLFW_KEY_B;
    case Key::N:
        return GLFW_KEY_N;
    case Key::M:
        return GLFW_KEY_M;
    case Key::T:
        return GLFW_KEY_T;
    case Key::G:
        return GLFW_KEY_G;
    case Key::Y:
        return GLFW_KEY_Y;
    case Key::U:
        return GLFW_KEY_U;
    case Key::I:
        return GLFW_KEY_I;
    case Key::O:
        return GLFW_KEY_O;
    case Key::P:
        return GLFW_KEY_P;
    case Key::J:
        return GLFW_KEY_J;
    case Key::K:
        return GLFW_KEY_K;
    case Key::L:
        return GLFW_KEY_L;
    case Key::H:
        return GLFW_KEY_H;
    case Key::Num0:
        return GLFW_KEY_0;
    case Key::Num1:
        return GLFW_KEY_1;
    case Key::Num2:
        return GLFW_KEY_2;
    case Key::Num3:
        return GLFW_KEY_3;
    case Key::Num4:
        return GLFW_KEY_4;
    case Key::Num5:
        return GLFW_KEY_5;
    case Key::Num6:
        return GLFW_KEY_6;
    case Key::Num7:
        return GLFW_KEY_7;
    case Key::Num8:
        return GLFW_KEY_8;
    case Key::Num9:
        return GLFW_KEY_9;
    case Key::Tab:
        return GLFW_KEY_TAB;
    case Key::Enter:
        return GLFW_KEY_ENTER;
    case Key::Backspace:
        return GLFW_KEY_BACKSPACE;
    case Key::LeftAlt:
        return GLFW_KEY_LEFT_ALT;
    case Key::RightShift:
        return GLFW_KEY_RIGHT_SHIFT;
    case Key::RightControl:
        return GLFW_KEY_RIGHT_CONTROL;
    case Key::RightAlt:
        return GLFW_KEY_RIGHT_ALT;
    case Key::Unknown:
        break;
    }
    return GLFW_KEY_UNKNOWN;
}

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
void synchronizeMouseButtonState(GLFWwindow *window, WindowRecord &record)
{
    std::uint8_t        mask       = 0;
    constexpr std::uint32_t maximumButton = static_cast<std::uint32_t>(MouseButton::Forward);
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
    const std::uint32_t maximumKey = static_cast<std::uint32_t>(Key::RightAlt);
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
// glfwPollEvents(), i.e. during the module's pollEvents capability call.

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

    const MouseButton mappedButton  = static_cast<MouseButton>(static_cast<std::uint32_t>(button));
    const std::uint8_t bit          = mouseButtonMaskBit(mappedButton);
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

bool GlfwWindowBackend::copyClipboardText(const WindowRecord &record, std::string &output) const
{
    if (record.platformWindow == nullptr)
        return false;
    const char *text = glfwGetClipboardString(static_cast<GLFWwindow *>(record.platformWindow));
    if (text == nullptr)
        return false;
    std::size_t length = 0;
    while (length <= maximumClipboardTextBytes && text[length] != '\0')
        ++length;
    if (length > maximumClipboardTextBytes || !isValidUtf8(text, length))
        return false;
    try
    {
        output.assign(text, length);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

void GlfwWindowBackend::pollEvents()
{
    glfwPollEvents();
    pollGamepads();
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
            if (std::abs(value - oldValue) <= 0.0005f)
                continue;
            oldValue = value;
            queueGamepadEvent(WindowEvent{.kind = WindowEventKind::GamepadAxisMoved,
                .gamepadId = static_cast<std::uint32_t>(joystick),
                .gamepadControl = static_cast<std::uint32_t>(axis),
                .gamepadValue = value});
        }
    }
}

// Normal/Captured map to GLFW_CURSOR normal/disabled; captured also enables
// raw mouse motion where the platform supports it.
void GlfwWindowBackend::setCursorMode(const WindowRecord &record, CursorMode mode)
{
    if (record.platformWindow == nullptr)
    {
        return;
    }
    GLFWwindow *window   = static_cast<GLFWwindow *>(record.platformWindow);
    const int   glfwMode = mode == CursorMode::Captured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL;
    glfwSetInputMode(window, GLFW_CURSOR, glfwMode);
#if defined(GLFW_RAW_MOUSE_MOTION)

    glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, mode == CursorMode::Captured ? GLFW_TRUE : GLFW_FALSE);
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
