#include "input.hpp"

#include "engine/capabilities/window_contract.hpp"

#include <GLFW/glfw3.h>

#include <stdexcept>

namespace engine
{

namespace
{

constexpr std::int32_t pressedAction = GLFW_PRESS;
constexpr std::int32_t releasedAction = GLFW_RELEASE;

const char* keyCodeNameTable(KeyCode keyCode)
{
    switch (keyCode)
    {
    case KeyCode::Space: return "Space";
    case KeyCode::Apostrophe: return "Apostrophe";
    case KeyCode::Comma: return "Comma";
    case KeyCode::Minus: return "Minus";
    case KeyCode::Period: return "Period";
    case KeyCode::Slash: return "Slash";
    case KeyCode::Key0: return "0";
    case KeyCode::Key1: return "1";
    case KeyCode::Key2: return "2";
    case KeyCode::Key3: return "3";
    case KeyCode::Key4: return "4";
    case KeyCode::Key5: return "5";
    case KeyCode::Key6: return "6";
    case KeyCode::Key7: return "7";
    case KeyCode::Key8: return "8";
    case KeyCode::Key9: return "9";
    case KeyCode::Semicolon: return "Semicolon";
    case KeyCode::Equal: return "Equal";
    case KeyCode::A: return "A";
    case KeyCode::B: return "B";
    case KeyCode::C: return "C";
    case KeyCode::D: return "D";
    case KeyCode::E: return "E";
    case KeyCode::F: return "F";
    case KeyCode::G: return "G";
    case KeyCode::H: return "H";
    case KeyCode::I: return "I";
    case KeyCode::J: return "J";
    case KeyCode::K: return "K";
    case KeyCode::L: return "L";
    case KeyCode::M: return "M";
    case KeyCode::N: return "N";
    case KeyCode::O: return "O";
    case KeyCode::P: return "P";
    case KeyCode::Q: return "Q";
    case KeyCode::R: return "R";
    case KeyCode::S: return "S";
    case KeyCode::T: return "T";
    case KeyCode::U: return "U";
    case KeyCode::V: return "V";
    case KeyCode::W: return "W";
    case KeyCode::X: return "X";
    case KeyCode::Y: return "Y";
    case KeyCode::Z: return "Z";
    case KeyCode::LeftBracket: return "LeftBracket";
    case KeyCode::Backslash: return "Backslash";
    case KeyCode::RightBracket: return "RightBracket";
    case KeyCode::GraveAccent: return "GraveAccent";
    case KeyCode::Escape: return "Escape";
    case KeyCode::Enter: return "Enter";
    case KeyCode::Tab: return "Tab";
    case KeyCode::Backspace: return "Backspace";
    case KeyCode::Insert: return "Insert";
    case KeyCode::Delete: return "Delete";
    case KeyCode::Right: return "Right";
    case KeyCode::Left: return "Left";
    case KeyCode::Down: return "Down";
    case KeyCode::Up: return "Up";
    case KeyCode::PageUp: return "PageUp";
    case KeyCode::PageDown: return "PageDown";
    case KeyCode::Home: return "Home";
    case KeyCode::End: return "End";
    case KeyCode::CapsLock: return "CapsLock";
    case KeyCode::ScrollLock: return "ScrollLock";
    case KeyCode::NumLock: return "NumLock";
    case KeyCode::PrintScreen: return "PrintScreen";
    case KeyCode::Pause: return "Pause";
    case KeyCode::F1: return "F1";
    case KeyCode::F2: return "F2";
    case KeyCode::F3: return "F3";
    case KeyCode::F4: return "F4";
    case KeyCode::F5: return "F5";
    case KeyCode::F6: return "F6";
    case KeyCode::F7: return "F7";
    case KeyCode::F8: return "F8";
    case KeyCode::F9: return "F9";
    case KeyCode::F10: return "F10";
    case KeyCode::F11: return "F11";
    case KeyCode::F12: return "F12";
    case KeyCode::LeftShift: return "LeftShift";
    case KeyCode::LeftControl: return "LeftControl";
    case KeyCode::LeftAlt: return "LeftAlt";
    case KeyCode::LeftSuper: return "LeftSuper";
    case KeyCode::RightShift: return "RightShift";
    case KeyCode::RightControl: return "RightControl";
    case KeyCode::RightAlt: return "RightAlt";
    case KeyCode::RightSuper: return "RightSuper";
    case KeyCode::Menu: return "Menu";
    default: return nullptr;
    }
}

struct KeyCodeNameEntry
{
    const char* name;
    KeyCode keyCode;
};

const KeyCodeNameEntry* keyCodeNameEntries()
{
    static const KeyCodeNameEntry entries[] = {
        {"Space", KeyCode::Space},       {"Apostrophe", KeyCode::Apostrophe},
        {"Comma", KeyCode::Comma},       {"Minus", KeyCode::Minus},
        {"Period", KeyCode::Period},     {"Slash", KeyCode::Slash},
        {"0", KeyCode::Key0},            {"1", KeyCode::Key1},
        {"2", KeyCode::Key2},            {"3", KeyCode::Key3},
        {"4", KeyCode::Key4},            {"5", KeyCode::Key5},
        {"6", KeyCode::Key6},            {"7", KeyCode::Key7},
        {"8", KeyCode::Key8},            {"9", KeyCode::Key9},
        {"Semicolon", KeyCode::Semicolon}, {"Equal", KeyCode::Equal},
        {"A", KeyCode::A},               {"B", KeyCode::B},
        {"C", KeyCode::C},               {"D", KeyCode::D},
        {"E", KeyCode::E},               {"F", KeyCode::F},
        {"G", KeyCode::G},               {"H", KeyCode::H},
        {"I", KeyCode::I},               {"J", KeyCode::J},
        {"K", KeyCode::K},               {"L", KeyCode::L},
        {"M", KeyCode::M},               {"N", KeyCode::N},
        {"O", KeyCode::O},               {"P", KeyCode::P},
        {"Q", KeyCode::Q},               {"R", KeyCode::R},
        {"S", KeyCode::S},               {"T", KeyCode::T},
        {"U", KeyCode::U},               {"V", KeyCode::V},
        {"W", KeyCode::W},               {"X", KeyCode::X},
        {"Y", KeyCode::Y},               {"Z", KeyCode::Z},
        {"LeftBracket", KeyCode::LeftBracket}, {"Backslash", KeyCode::Backslash},
        {"RightBracket", KeyCode::RightBracket}, {"GraveAccent", KeyCode::GraveAccent},
        {"Escape", KeyCode::Escape},     {"Enter", KeyCode::Enter},
        {"Tab", KeyCode::Tab},           {"Backspace", KeyCode::Backspace},
        {"Insert", KeyCode::Insert},     {"Delete", KeyCode::Delete},
        {"Right", KeyCode::Right},       {"Left", KeyCode::Left},
        {"Down", KeyCode::Down},         {"Up", KeyCode::Up},
        {"PageUp", KeyCode::PageUp},     {"PageDown", KeyCode::PageDown},
        {"Home", KeyCode::Home},         {"End", KeyCode::End},
        {"CapsLock", KeyCode::CapsLock}, {"ScrollLock", KeyCode::ScrollLock},
        {"NumLock", KeyCode::NumLock},   {"PrintScreen", KeyCode::PrintScreen},
        {"Pause", KeyCode::Pause},       {"F1", KeyCode::F1},
        {"F2", KeyCode::F2},             {"F3", KeyCode::F3},
        {"F4", KeyCode::F4},             {"F5", KeyCode::F5},
        {"F6", KeyCode::F6},             {"F7", KeyCode::F7},
        {"F8", KeyCode::F8},             {"F9", KeyCode::F9},
        {"F10", KeyCode::F10},           {"F11", KeyCode::F11},
        {"F12", KeyCode::F12},           {"LeftShift", KeyCode::LeftShift},
        {"LeftControl", KeyCode::LeftControl}, {"LeftAlt", KeyCode::LeftAlt},
        {"LeftSuper", KeyCode::LeftSuper}, {"RightShift", KeyCode::RightShift},
        {"RightControl", KeyCode::RightControl}, {"RightAlt", KeyCode::RightAlt},
        {"RightSuper", KeyCode::RightSuper}, {"Menu", KeyCode::Menu},
        {nullptr, KeyCode::Unknown}, // sentinel
    };
    return entries;
}

} // namespace

const char* keyCodeName(KeyCode keyCode)
{
    return keyCodeNameTable(keyCode);
}

KeyCode keyCodeFromName(const std::string& keyName)
{
    for (const KeyCodeNameEntry* entry = keyCodeNameEntries(); entry->name != nullptr; ++entry)
    {
        if (keyName == entry->name)
        {
            return entry->keyCode;
        }
    }
    return KeyCode::Unknown;
}

InputModule::InputModule()
{
    componentManifest.name = "input";
    componentManifest.kind = ComponentKind::Module;
    componentManifest.providedCapabilities = {"input"};
    componentManifest.requiredCapabilities = {"logging", "window"};
    componentManifest.requiredPermissions = {"capability.logging", "capability.window"};
}

InputModule::~InputModule()
{
    stop();
}

void InputModule::construct(ComponentContext& context)
{
    boundContext = &context;
    logger = &context.acquireCapability<LoggerCapability>();
    windowCapability = &context.acquireCapability<WindowCapability>();
}

void InputModule::start()
{
    const std::vector<WindowIdentifier> identifiers = windowCapability->windowIdentifiers();
    if (identifiers.empty())
    {
        logger->logMessage(LogLevel::Warning, "input module started without any window to attach to");
        return;
    }
    for (WindowIdentifier identifier : identifiers)
    {
        GLFWwindow* window = static_cast<GLFWwindow*>(windowCapability->nativeWindowHandle(identifier));
        if (window != nullptr)
        {
            // The input module owns the GLFW window user pointer: GLFW offers
            // exactly one per window and the window module never uses it.
            glfwSetWindowUserPointer(window, this);
            glfwSetKeyCallback(window, &InputModule::keyCallback);
            glfwSetCursorPosCallback(window, &InputModule::cursorPositionCallback);
            glfwSetScrollCallback(window, &InputModule::scrollCallback);
        }
    }
    cursorPositionKnown = false;
    logger->logMessage(LogLevel::Information, "input module started");
}

void InputModule::stop()
{
    // stop must tolerate being reached before construct (dry validation
    // creates instances only to read their manifests) and after a previous
    // stop (kernel teardown destroys instances that were never started).
    if (windowCapability != nullptr)
    {
        for (WindowIdentifier identifier : windowCapability->windowIdentifiers())
        {
            GLFWwindow* window = static_cast<GLFWwindow*>(windowCapability->nativeWindowHandle(identifier));
            if (window != nullptr)
            {
                glfwSetKeyCallback(window, nullptr);
                glfwSetCursorPosCallback(window, nullptr);
                glfwSetScrollCallback(window, nullptr);
                // Leave a released cursor behind: a teardown that keeps the
                // window alive must not strand the user's pointer.
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
                glfwSetWindowUserPointer(window, nullptr);
            }
        }
    }
    pendingMouseX = 0.0;
    pendingMouseY = 0.0;
    pendingScrollX = 0.0;
    pendingScrollY = 0.0;
    cursorCaptured = false;
    cursorPositionKnown = false;
    boundContext = nullptr;
    windowCapability = nullptr;
    logger = nullptr;
}

void* InputModule::capabilityPointerFor(const std::string& capabilityName)
{
    if (capabilityName == InputCapability::capabilityName)
    {
        return static_cast<InputCapability*>(this);
    }
    return nullptr;
}

bool InputModule::isKeyDown(KeyCode keyCode)
{
    const std::vector<WindowIdentifier> identifiers = windowCapability->windowIdentifiers();
    if (identifiers.empty())
    {
        return false;
    }
    GLFWwindow* window = static_cast<GLFWwindow*>(windowCapability->nativeWindowHandle(identifiers.front()));
    if (window == nullptr)
    {
        return false;
    }
    return glfwGetKey(window, static_cast<int>(keyCode)) == GLFW_PRESS;
}

bool InputModule::isMouseButtonDown(MouseButton button)
{
    const std::vector<WindowIdentifier> identifiers = windowCapability->windowIdentifiers();
    if (identifiers.empty())
    {
        return false;
    }
    GLFWwindow* window = static_cast<GLFWwindow*>(windowCapability->nativeWindowHandle(identifiers.front()));
    if (window == nullptr)
    {
        return false;
    }
    return glfwGetMouseButton(window, static_cast<int>(button)) == GLFW_PRESS;
}

void InputModule::consumeMouseDelta(double& deltaX, double& deltaY)
{
    deltaX = pendingMouseX;
    deltaY = pendingMouseY;
    pendingMouseX = 0.0;
    pendingMouseY = 0.0;
}

void InputModule::consumeScrollDelta(double& deltaX, double& deltaY)
{
    deltaX = pendingScrollX;
    deltaY = pendingScrollY;
    pendingScrollX = 0.0;
    pendingScrollY = 0.0;
}

void InputModule::setCursorCaptured(bool captured)
{
    const std::vector<WindowIdentifier> identifiers = windowCapability == nullptr
                                                         ? std::vector<WindowIdentifier>{}
                                                         : windowCapability->windowIdentifiers();
    if (identifiers.empty())
    {
        return;
    }
    GLFWwindow* window = static_cast<GLFWwindow*>(windowCapability->nativeWindowHandle(identifiers.front()));
    if (window == nullptr || captured == cursorCaptured)
    {
        return;
    }
    // Disabled hides and locks the cursor; raw motion (when the platform
    // supports it) reports unaccelerated deltas, which is what a fly camera
    // wants. The pending delta is cleared around the transition: mode changes
    // can emit a cursor position event with a large jump.
    glfwSetInputMode(window, GLFW_CURSOR, captured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
    if (glfwRawMouseMotionSupported() == GLFW_TRUE)
    {
        glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, captured ? GLFW_TRUE : GLFW_FALSE);
    }
    pendingMouseX = 0.0;
    pendingMouseY = 0.0;
    cursorPositionKnown = false;
    cursorCaptured = captured;
}

bool InputModule::isCursorCaptured() const
{
    return cursorCaptured;
}

void InputModule::cursorPositionCallback(GLFWwindow* window, double x, double y)
{
    InputModule* module = static_cast<InputModule*>(glfwGetWindowUserPointer(window));
    if (module == nullptr)
    {
        return;
    }
    if (!module->cursorPositionKnown)
    {
        // Absolute position without a previous one: not motion.
        module->cursorPositionKnown = true;
        module->lastCursorX = x;
        module->lastCursorY = y;
        return;
    }
    module->pendingMouseX += x - module->lastCursorX;
    module->pendingMouseY += y - module->lastCursorY;
    module->lastCursorX = x;
    module->lastCursorY = y;
}

void InputModule::scrollCallback(GLFWwindow* window, double x, double y)
{
    InputModule* module = static_cast<InputModule*>(glfwGetWindowUserPointer(window));
    if (module == nullptr)
    {
        return;
    }
    module->pendingScrollX += x;
    module->pendingScrollY += y;
}

void InputModule::keyCallback(GLFWwindow* window, std::int32_t platformKeyCode, std::int32_t scanCode,
                              std::int32_t action, std::int32_t modifiers)
{
    (void)scanCode;
    (void)modifiers;

    InputModule* module = static_cast<InputModule*>(glfwGetWindowUserPointer(window));
    if (module == nullptr || module->boundContext == nullptr)
    {
        return;
    }

    const KeyCode keyCode = static_cast<KeyCode>(platformKeyCode);
    if (action == pressedAction)
    {
        KeyPressedEvent event;
        event.keyCode = keyCode;
        module->boundContext->eventBus().publish(event);
    }
    else if (action == releasedAction)
    {
        KeyReleasedEvent event;
        event.keyCode = keyCode;
        module->boundContext->eventBus().publish(event);
    }
}

} // namespace engine
