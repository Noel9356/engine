#include "input.hpp"

#include "window_capability.hpp"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <stdexcept>

namespace engine
{

namespace
{

constexpr std::int32_t pressedAction  = GLFW_PRESS;
constexpr std::int32_t releasedAction = GLFW_RELEASE;

// keyCodeName/keyCodeFromName are published with the input capability header
// (input_capability.hpp) as inline functions: capability metadata lives with
// the capability, and every consumer binary embeds the table it uses, so no
// plugin depends on another plugin's symbols at load time.

} // namespace

InputModule::InputModule()
{
    componentManifest.name = "input";
    componentManifest.kind = ComponentKind::Module;
    // The implementation-bound member name: this module is the GLFW
    // implementation of the "input" family (see window.glfw).
    componentManifest.providedCapabilities = {"input.glfw"};
    componentManifest.requiredCapabilities = {"logging", "window"};
    componentManifest.requiredPermissions  = {"capability.logging", "capability.window"};
    // The action map's keys are authored freely by the assembly (the array's
    // path is what the known-key registry sees; rows are dynamic data).
    componentManifest.knownConfigKeys      = {"input.actions"};
}

InputModule::~InputModule()
{
    stop();
}

void InputModule::construct(
    ComponentContext &context
)
{
    boundContext     = &context;
    logger           = &context.acquireCapability<LoggerCapability>();
    windowCapability = &context.acquireCapability<WindowCapability>();
    loadActionMap();
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
        GLFWwindow *window = static_cast<GLFWwindow *>(windowCapability->nativeWindowHandle(identifier));
        if (window != nullptr)
        {
            // The input module owns the GLFW window user pointer: GLFW offers
            // exactly one per window and the window module never uses it.
            glfwSetWindowUserPointer(window, this);
            glfwSetKeyCallback(window, &InputModule::keyCallback);
            glfwSetCursorPosCallback(window, &InputModule::cursorPositionCallback);
            glfwSetScrollCallback(window, &InputModule::scrollCallback);
            glfwSetCharCallback(window, &InputModule::characterCallback);
        }
    }
    cursorPositionKnown = false;
    // Gamepad hot-plug events fire through GLFW's global callback (no window
    // involved); the first poll seeds the connection cache.
    glfwSetJoystickCallback(&InputModule::gamepadConnectionCallback);
    if (pollGamepad())
    {
        logger->logMessage(LogLevel::Information, "input module started (gamepad connected)");
    }
    else
    {
        logger->logMessage(LogLevel::Information, "input module started");
    }
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
            GLFWwindow *window = static_cast<GLFWwindow *>(windowCapability->nativeWindowHandle(identifier));
            if (window != nullptr)
            {
                glfwSetKeyCallback(window, nullptr);
                glfwSetCursorPosCallback(window, nullptr);
                glfwSetScrollCallback(window, nullptr);
                glfwSetCharCallback(window, nullptr);
                // Leave a released cursor behind: a teardown that keeps the
                // window alive must not strand the user's pointer.
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
                glfwSetWindowUserPointer(window, nullptr);
            }
        }
    }
    pendingMouseX       = 0.0;
    pendingMouseY       = 0.0;
    pendingScrollX      = 0.0;
    pendingScrollY      = 0.0;
    cursorCaptured      = false;
    cursorPositionKnown = false;
    glfwSetJoystickCallback(nullptr);
    gamepadConnectedCache = false;
    actions.clear();
    boundContext        = nullptr;
    windowCapability    = nullptr;
    logger              = nullptr;
}

void *InputModule::capabilityPointerFor(
    const std::string &capabilityName
)
{
    if (capabilityName == "input.glfw")
    {
        return static_cast<InputCapability *>(this);
    }
    return nullptr;
}

bool InputModule::isKeyDown(
    KeyCode keyCode
)
{
    const std::vector<WindowIdentifier> identifiers = windowCapability->windowIdentifiers();
    if (identifiers.empty())
    {
        return false;
    }
    GLFWwindow *window = static_cast<GLFWwindow *>(windowCapability->nativeWindowHandle(identifiers.front()));
    if (window == nullptr)
    {
        return false;
    }
    return glfwGetKey(window, static_cast<int>(keyCode)) == GLFW_PRESS;
}

bool InputModule::isMouseButtonDown(
    MouseButton button
)
{
    const std::vector<WindowIdentifier> identifiers = windowCapability->windowIdentifiers();
    if (identifiers.empty())
    {
        return false;
    }
    GLFWwindow *window = static_cast<GLFWwindow *>(windowCapability->nativeWindowHandle(identifiers.front()));
    if (window == nullptr)
    {
        return false;
    }
    return glfwGetMouseButton(window, static_cast<int>(button)) == GLFW_PRESS;
}

void InputModule::consumeMouseDelta(
    double &deltaX, double &deltaY
)
{
    deltaX        = pendingMouseX;
    deltaY        = pendingMouseY;
    pendingMouseX = 0.0;
    pendingMouseY = 0.0;
}

void InputModule::consumeScrollDelta(
    double &deltaX, double &deltaY
)
{
    deltaX         = pendingScrollX;
    deltaY         = pendingScrollY;
    pendingScrollX = 0.0;
    pendingScrollY = 0.0;
}

void InputModule::setCursorCaptured(
    bool captured
)
{
    const std::vector<WindowIdentifier> identifiers = windowCapability == nullptr
                                                        ? std::vector<WindowIdentifier>{}
                                                        : windowCapability->windowIdentifiers();
    if (identifiers.empty())
    {
        return;
    }
    GLFWwindow *window = static_cast<GLFWwindow *>(windowCapability->nativeWindowHandle(identifiers.front()));
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
    pendingMouseX       = 0.0;
    pendingMouseY       = 0.0;
    cursorPositionKnown = false;
    cursorCaptured      = captured;
}

bool InputModule::isCursorCaptured() const
{
    return cursorCaptured;
}

bool InputModule::isGamepadConnected()
{
    return pollGamepad();
}

bool InputModule::isGamepadButtonDown(
    GamepadButton button
)
{
    if (!pollGamepad())
    {
        return false;
    }
    const std::size_t index = static_cast<std::size_t>(button);
    return index < gamepadButtons.size() && gamepadButtons[index] != 0;
}

float InputModule::gamepadAxis(
    GamepadAxis axis
)
{
    if (!pollGamepad())
    {
        return 0.0f;
    }
    const std::size_t index = static_cast<std::size_t>(axis);
    return index < gamepadAxes.size() ? gamepadAxes[index] : 0.0f;
}

float InputModule::actionStrength(
    const std::string &actionName
)
{
    const auto found = actions.find(actionName);
    if (found == actions.end())
    {
        return 0.0f; // an unconfigured action reads as "not pressed"
    }
    float strength = 0.0f;
    for (const ActionBinding &binding : found->second)
    {
        float candidate = 0.0f;
        switch (binding.kind)
        {
            case ActionBinding::Kind::Key:
                candidate = isKeyDown(static_cast<KeyCode>(binding.code)) ? 1.0f : 0.0f;
                break;
            case ActionBinding::Kind::MouseButton:
                candidate = isMouseButtonDown(static_cast<MouseButton>(binding.code)) ? 1.0f : 0.0f;
                break;
            case ActionBinding::Kind::GamepadButton:
                candidate = isGamepadButtonDown(static_cast<GamepadButton>(binding.code)) ? 1.0f : 0.0f;
                break;
            case ActionBinding::Kind::GamepadAxis:
                candidate = std::clamp(gamepadAxis(static_cast<GamepadAxis>(binding.code)), 0.0f, 1.0f);
                break;
        }
        strength = std::max(strength, candidate);
    }
    return strength;
}

void InputModule::loadActionMap()
{
    if (boundContext == nullptr)
    {
        return;
    }
    const std::vector<std::string> actionNames = boundContext->configurationTableKeysInSection("input", "actions");
    for (const std::string &actionName : actionNames)
    {
        const std::string value = boundContext->configurationStringInSection("input", "actions." + actionName).value_or("");
        std::vector<ActionBinding> bindings;
        // One action = one comma-separated binding string ("Space, gamepad:A").
        std::size_t position = 0;
        while (position < value.size())
        {
            std::size_t comma   = value.find(',', position);
            if (comma == std::string::npos)
            {
                comma = value.size();
            }
            std::string token = value.substr(position, comma - position);
            position          = comma + 1;
            // Trim.
            const auto isSpace = [](char character) { return character == ' ' || character == '\t'; };
            while (!token.empty() && isSpace(token.front()))
            {
                token.erase(token.begin());
            }
            while (!token.empty() && isSpace(token.back()))
            {
                token.pop_back();
            }
            if (token.empty())
            {
                continue;
            }
            ActionBinding binding;
            if (token.rfind("gamepad:", 0) == 0)
            {
                std::int32_t code    = -1;
                bool         isButton = false;
                if (gamepadControlFromName(token.substr(8), code, isButton))
                {
                    binding.kind = isButton ? ActionBinding::Kind::GamepadButton : ActionBinding::Kind::GamepadAxis;
                    binding.code = code;
                    bindings.push_back(binding);
                }
                else if (logger != nullptr)
                {
                    logger->logMessage(
                        LogLevel::Warning,
                        "input module: action '" + actionName + "' names unknown gamepad control '" + token.substr(8) + "'"
                    );
                }
            }
            else if (token.rfind("mouse:", 0) == 0)
            {
                const std::string buttonName = token.substr(6);
                if (buttonName == "left")
                {
                    binding.kind = ActionBinding::Kind::MouseButton;
                    binding.code = static_cast<std::int32_t>(MouseButton::Left);
                    bindings.push_back(binding);
                }
                else if (buttonName == "right")
                {
                    binding.kind = ActionBinding::Kind::MouseButton;
                    binding.code = static_cast<std::int32_t>(MouseButton::Right);
                    bindings.push_back(binding);
                }
                else if (buttonName == "middle")
                {
                    binding.kind = ActionBinding::Kind::MouseButton;
                    binding.code = static_cast<std::int32_t>(MouseButton::Middle);
                    bindings.push_back(binding);
                }
            }
            else
            {
                const KeyCode keyCode = keyCodeFromName(token);
                if (keyCode != KeyCode::Unknown)
                {
                    binding.kind = ActionBinding::Kind::Key;
                    binding.code = static_cast<std::int32_t>(keyCode);
                    bindings.push_back(binding);
                }
                else if (logger != nullptr)
                {
                    logger->logMessage(
                        LogLevel::Warning,
                        "input module: action '" + actionName + "' names unknown key '" + token + "'"
                    );
                }
            }
        }
        if (bindings.empty())
        {
            if (logger != nullptr)
            {
                logger->logMessage(
                    LogLevel::Warning,
                    "input module: action '" + actionName + "' resolved to no usable binding; it always reads 0"
                );
            }
            continue;
        }
        actions.emplace(actionName, std::move(bindings));
    }
    if (!actions.empty() && logger != nullptr)
    {
        std::string names;
        for (const auto &entry : actions)
        {
            if (!names.empty())
            {
                names += ", ";
            }
            names += entry.first;
        }
        logger->logMessage(LogLevel::Information, "input module: actions mapped: " + names);
    }
}

bool InputModule::pollGamepad()
{
    if (windowCapability == nullptr)
    {
        return false;
    }
    GLFWgamepadstate state;
    const bool       connected = glfwGetGamepadState(GLFW_JOYSTICK_1, &state) == GLFW_TRUE;
    if (connected)
    {
        for (std::size_t axis = 0; axis < gamepadAxes.size(); ++axis)
        {
            gamepadAxes[axis] = state.axes[axis];
        }
        for (std::size_t button = 0; button < gamepadButtons.size(); ++button)
        {
            gamepadButtons[button] = state.buttons[button];
        }
    }
    if (connected != gamepadConnectedCache && boundContext != nullptr)
    {
        gamepadConnectedCache = connected;
        if (connected)
        {
            boundContext->eventBus().publish(GamepadConnectedEvent{});
        }
        else
        {
            boundContext->eventBus().publish(GamepadDisconnectedEvent{});
        }
    }
    return connected;
}

void InputModule::characterCallback(
    GLFWwindow *window, unsigned int codepoint
)
{
    InputModule *module = static_cast<InputModule *>(glfwGetWindowUserPointer(window));
    if (module == nullptr || module->boundContext == nullptr)
    {
        return;
    }
    TextInputEvent event;
    event.codepoint = codepoint;
    module->boundContext->eventBus().publish(event);
}

void InputModule::gamepadConnectionCallback(
    int /*joystickId*/, int /*event*/
)
{
    // The per-query poll publishes the connect/disconnect events (it has the
    // module pointer); this callback only exists so GLFW sees a listener and
    // keeps its internal state warm. State is re-polled lazily.
}

void InputModule::cursorPositionCallback(
    GLFWwindow *window, double x, double y
)
{
    InputModule *module = static_cast<InputModule *>(glfwGetWindowUserPointer(window));
    if (module == nullptr)
    {
        return;
    }
    if (!module->cursorPositionKnown)
    {
        // Absolute position without a previous one: not motion.
        module->cursorPositionKnown = true;
        module->lastCursorX         = x;
        module->lastCursorY         = y;
        return;
    }
    module->pendingMouseX += x - module->lastCursorX;
    module->pendingMouseY += y - module->lastCursorY;
    module->lastCursorX = x;
    module->lastCursorY = y;
}

void InputModule::scrollCallback(
    GLFWwindow *window, double x, double y
)
{
    InputModule *module = static_cast<InputModule *>(glfwGetWindowUserPointer(window));
    if (module == nullptr)
    {
        return;
    }
    module->pendingScrollX += x;
    module->pendingScrollY += y;
}

void InputModule::keyCallback(
    GLFWwindow *window, std::int32_t platformKeyCode, std::int32_t scanCode, std::int32_t action, std::int32_t modifiers
)
{
    (void)scanCode;
    (void)modifiers;

    InputModule *module = static_cast<InputModule *>(glfwGetWindowUserPointer(window));
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
