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

// keyCodeName/keyCodeFromName live in the kernel (source/
// input_contract_names.cpp): they are input-contract metadata shared by the
// screenshot trigger configuration and the Lua script bridge, and across
// module plugins every shared library resolves contract symbols from the
// kernel it links - a module-local definition strands the other plugins.

} // namespace

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
