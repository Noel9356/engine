#ifndef ENGINE_MODULES_INPUT_HPP
#define ENGINE_MODULES_INPUT_HPP

#include "input_capability.hpp"
#include "logging_capability.hpp"
#include "window_capability.hpp"
#include "engine/kernel/component_interface.hpp"

#include <cstdint>
#include <string>

struct GLFWwindow;

namespace engine
{

// Module "input": provides the InputCapability and translates platform keyboard
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
    void consumeScrollDelta(double &deltaX, double &deltaY) override;
    void setCursorCaptured(bool captured) override;
    bool isCursorCaptured() const override;

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
};

} // namespace engine

#endif // ENGINE_MODULES_INPUT_HPP
