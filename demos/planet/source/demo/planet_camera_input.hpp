// Window-to-camera input bridge for the planet demo.
//
// Keeps cursor capture, keyboard sampling and mouse-delta bookkeeping out of
// the demo's application loop. Camera creation, pose selection and rendering
// remain owned by their respective systems.
#ifndef ENGINE_DEMO_PLANET_CAMERA_INPUT_HPP
#define ENGINE_DEMO_PLANET_CAMERA_INPUT_HPP

#include <engine/camera/capabilities.hpp>
#include <engine/window/capabilities.hpp>

namespace demo::planet_demo
{

class PlanetCameraInput final
{
public:
    PlanetCameraInput(const engine::window::WindowCapability &windowCapability,
        engine::window::WindowHandle windowHandle, bool windowedBackend, bool captureInitially);

    void handleWindowEvent(const engine::window::WindowEvent &event);

    // Returns one frame's input sample. When the backend has no input support,
    // or the automatic --orbit path owns the camera, this returns zero input.
    engine::camera::CameraInputState sample(bool enabled = true);

    [[nodiscard]] bool cursorCaptured() const noexcept
    {
        return cursorCaptured_;
    }

private:
    void setCursorCaptured(bool captured);

    const engine::window::WindowCapability *window_ = nullptr;
    engine::window::WindowHandle windowHandle_ = engine::window::invalidWindow;
    bool windowedBackend_ = false;
    bool cursorToggleEnabled_ = false;
    bool cursorCaptured_ = false;
    bool haveLastCursor_ = false;
    double lastCursorX_ = 0.0;
    double lastCursorY_ = 0.0;
};

} // namespace demo::planet_demo

#endif
