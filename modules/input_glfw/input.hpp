#ifndef ENGINE_MODULES_INPUT_HPP
#define ENGINE_MODULES_INPUT_HPP

#include "input_capability.hpp"
#include "logging_capability.hpp"
#include "window_capability.hpp"
#include "engine/kernel/component_interface.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

struct GLFWwindow;

namespace engine
{

// Module "input_glfw": provides the InputCapability and translates platform keyboard
// callbacks into KeyPressedEvent and KeyReleasedEvent on the event bus. The
// module attaches its callbacks to windows created by the window module, which
// demonstrates one module consuming the capability of another without any
// shared implementation detail.
class InputModule final : public ModuleInterface, public InputCapability
{
public:
    InputModule();
    ~InputModule() override;

    const ComponentManifest &manifest() const override
    {
        return componentManifest;
    }

    void construct(ComponentContext &context) override;
    void start() override;
    void stop() override;

    void *capabilityPointerFor(const std::string &capabilityName) override;

    bool isKeyDown(KeyCode keyCode) override;
    bool isMouseButtonDown(MouseButton button) override;
    void consumeMouseDelta(double &deltaX, double &deltaY) override;
    void cursorPosition(double &positionX, double &positionY) override;
    void consumeScrollDelta(double &deltaX, double &deltaY) override;
    void setCursorCaptured(bool captured) override;
    bool isCursorCaptured() const override;

    bool  isGamepadConnected() override;
    bool  isGamepadButtonDown(GamepadButton button) override;
    float gamepadAxis(GamepadAxis axis) override;
    float actionStrength(const std::string &actionName) override;

private:
    static void keyCallback(
        GLFWwindow  *window,
        std::int32_t platformKeyCode,
        std::int32_t scanCode,
        std::int32_t action,
        std::int32_t modifiers
    );
    static void cursorPositionCallback(GLFWwindow *window, double x, double y);
    static void scrollCallback(GLFWwindow *window, double x, double y);
    static void characterCallback(GLFWwindow *window, unsigned int codepoint);
    static void gamepadConnectionCallback(int joystickId, int event);

    // Polls the first gamepad; publishes connect/disconnect events on state
    // flips and caches the GLFW state for the query calls.
    bool        pollGamepad();

    ComponentManifest componentManifest;
    ComponentContext *boundContext     = nullptr;
    LoggerCapability *logger           = nullptr;
    WindowCapability *windowCapability = nullptr;

    // Continuous input accumulators: platform callbacks add, frame consumers
    // drain. Guarded by the main-thread-only GLFW contract (callbacks, poll
    // and the frame loop all run on the caller's thread), so no mutex.
    double pendingMouseX  = 0.0;
    double pendingMouseY  = 0.0;
    double pendingScrollX = 0.0;
    double pendingScrollY = 0.0;
    bool   cursorCaptured = false;
    // First cursor callback after attaching has no previous position: the
    // platform hands us an absolute position, not motion.
    bool   cursorPositionKnown = false;
    double lastCursorX         = 0.0;
    double lastCursorY         = 0.0;

    // Gamepad state: refreshed on every query (the GLFW gamepad API polls the
    // device synchronously); the connection cache flips publish the bus
    // events. GLFW_GAMEPAD_STATE - the struct lives in glfw3.h, so the
    // member is kept as the raw button/axis arrays to avoid leaking GLFW
    // types through this header.
    bool                          gamepadConnectedCache = false;
    std::array<float, 6>          gamepadAxes{};
    std::array<unsigned char, 15> gamepadButtons{};

    // Action map ([input.actions]): action name -> device bindings.
    struct ActionBinding
    {
        enum class Kind
        {
            Key,
            MouseButton,
            GamepadButton,
            GamepadAxis // analog: triggers contribute their 0..1 value
        };
        Kind        kind    = Kind::Key;
        std::int32_t code   = 0;
    };
    std::unordered_map<std::string, std::vector<ActionBinding>> actions;
    void loadActionMap();
};

} // namespace engine

#endif // ENGINE_MODULES_INPUT_HPP
