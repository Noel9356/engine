// sandbox_input.hpp - window-to-demo input bridge: camera input sampling,
// mouse grab picking state, and the keyboard action queue. Camera creation
// and the physics responses stay in main.cpp and the physics capabilities.
#pragma once

#include <engine/camera/capabilities.hpp>
#include <engine/window/capabilities.hpp>

#include <cstdint>
#include <vector>

namespace demo::physics_sandbox
{

enum class SandboxAction : std::uint32_t
{
    None = 0,
    SpawnBox,
    SpawnSphere,
    SpawnCapsule,
    Blast,
    ToggleWind,
    ResetScene,
    TogglePause,
    CycleFixedRate,
    Quit
};

class SandboxInput final
{
public:
    SandboxInput(const engine::window::WindowCapability &windowCapability, engine::window::WindowHandle windowHandle,
        bool windowedBackend);

    void handleWindowEvent(const engine::window::WindowEvent &event);

    // One frame's camera sample (zero input when the backend has no input).
    engine::camera::CameraInputState sampleCamera();

    // Pops the queued actions (one per keyboard press event).
    std::vector<SandboxAction> drainActions();

    // Grab state: cursor position in pixels plus the press/release edges.
    bool grabPressed() const
    {
        return grabPressed_;
    }
    bool grabReleased() const
    {
        return grabReleased_;
    }
    bool grabActive() const
    {
        return grabActive_;
    }
    void clearGrabEdges()
    {
        grabPressed_  = false;
        grabReleased_ = false;
    }
    void setGrabActive(bool active)
    {
        grabActive_ = active;
    }
    double cursorX() const
    {
        return cursorX_;
    }
    double cursorY() const
    {
        return cursorY_;
    }

    [[nodiscard]] bool cursorCaptured() const noexcept
    {
        return cursorCaptured_;
    }

private:
    void setCursorCaptured(bool captured);

    const engine::window::WindowCapability *window_          = nullptr;
    engine::window::WindowHandle            windowHandle_    = engine::window::invalidWindow;
    bool                                    windowedBackend_ = false;
    bool                                    cursorCaptured_  = false;
    bool                                    haveLastCursor_  = false;
    double                                  lastCursorX_     = 0.0;
    double                                  lastCursorY_     = 0.0;
    double                                  pendingDeltaX_   = 0.0;
    double                                  pendingDeltaY_   = 0.0;

    bool   grabPressed_  = false;
    bool   grabReleased_ = false;
    bool   grabActive_   = false;
    double cursorX_      = 0.0;
    double cursorY_      = 0.0;

    std::vector<SandboxAction> actions_;
};

} // namespace demo::physics_sandbox
