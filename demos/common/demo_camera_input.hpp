// Shared interactive-camera input for the demos: cursor capture with the
// LeftAlt toggle, raw mouse deltas into CameraInputState::mouseDeltaX/Y (the
// camera module's degrees-per-pixel look path), and the WASD/Space/Ctrl
// movement axes with Shift sprint. The planet demo's ground/macro views and
// the lighting demo's free camera both drive from here so the feel is
// identical across the demos.

#pragma once

#include <engine/camera/capabilities.hpp>
#include <engine/window/capabilities.hpp>

#include <algorithm>
#include <cmath>

namespace demo
{
namespace harness
{

class DemoCameraInput
{
public:
    // captureInitially grabs the cursor on windowed backends (the raw-motion
    // deltas mouselook needs); headless backends never capture.
    DemoCameraInput(const engine::window::WindowCapability &windowCapability,
        engine::window::WindowHandle windowHandle, bool windowedBackend, bool captureInitially)
        : window_(&windowCapability)
        , windowHandle_(windowHandle)
        , windowedBackend_(windowedBackend)
        , cursorToggleEnabled_(windowedBackend && captureInitially && windowCapability.setCursorMode != nullptr)
    {
        setCursorCaptured(cursorToggleEnabled_);
    }

    void setCursorCaptured(bool captured)
    {
        cursorCaptured_ = windowedBackend_ && captured;
        haveLastCursor_ = false;
        if (window_ != nullptr && window_->setCursorMode != nullptr)
        {
            window_->setCursorMode(window_->self, windowHandle_,
                cursorCaptured_ ? engine::window::CursorMode::Captured : engine::window::CursorMode::Normal);
        }
    }

    [[nodiscard]] bool cursorCaptured() const noexcept
    {
        return cursorCaptured_;
    }

    // LeftAlt toggles the cursor capture (release the pointer to leave the
    // window); feed every drained window event through here.
    void handleWindowEvent(const engine::window::WindowEvent &event)
    {
        if (cursorToggleEnabled_ && event.kind == engine::window::WindowEventKind::KeyPressed
            && event.key == engine::window::Key::LeftAlt)
        {
            setCursorCaptured(!cursorCaptured_);
        }
    }

    // The per-frame input sample: keyboard axes plus the raw cursor deltas
    // (clamped to a conservative range so a cursor warp or focus transition
    // cannot throw the camera). enabled=false returns the axes alone.
    [[nodiscard]] engine::camera::CameraInputState sample(bool enabled = true) const
    {
        engine::camera::CameraInputState input{};
        if (!enabled || !windowedBackend_ || window_ == nullptr || window_->isKeyDown == nullptr)
        {
            haveLastCursor_ = false;
            return input;
        }

        const auto keyDown = [&](engine::window::Key key)
        { return window_->isKeyDown(window_->self, windowHandle_, key); };

        input.moveForward = static_cast<float>(keyDown(engine::window::Key::W))
                          - static_cast<float>(keyDown(engine::window::Key::S));
        input.moveRight = static_cast<float>(keyDown(engine::window::Key::D))
                        - static_cast<float>(keyDown(engine::window::Key::A));
        input.moveUp = static_cast<float>(keyDown(engine::window::Key::Space))
                     - static_cast<float>(keyDown(engine::window::Key::LeftControl));
        input.lookHorizontal = static_cast<float>(keyDown(engine::window::Key::Right))
                             - static_cast<float>(keyDown(engine::window::Key::Left));
        input.lookVertical = static_cast<float>(keyDown(engine::window::Key::Up))
                           - static_cast<float>(keyDown(engine::window::Key::Down));
        input.sprint = keyDown(engine::window::Key::LeftShift);

        if (!cursorCaptured_ || window_->cursorPosition == nullptr)
        {
            haveLastCursor_ = false;
            return input;
        }

        double cursorX = 0.0;
        double cursorY = 0.0;
        if (!window_->cursorPosition(window_->self, windowHandle_, &cursorX, &cursorY)
            || !std::isfinite(cursorX) || !std::isfinite(cursorY))
        {
            haveLastCursor_ = false;
            return input;
        }

        if (haveLastCursor_)
        {
            // Cursor warps and focus transitions can produce large deltas.
            // Clamp them to a conservative per-frame range.
            input.mouseDeltaX = static_cast<float>(std::clamp(cursorX - lastCursorX_, -200.0, 200.0));
            input.mouseDeltaY = static_cast<float>(std::clamp(cursorY - lastCursorY_, -200.0, 200.0));
        }
        lastCursorX_  = cursorX;
        lastCursorY_  = cursorY;
        haveLastCursor_ = true;
        return input;
    }

private:
    const engine::window::WindowCapability *window_ = nullptr;
    engine::window::WindowHandle            windowHandle_{};
    bool                                    windowedBackend_     = false;
    bool                                    cursorToggleEnabled_ = false;
    bool                                    cursorCaptured_      = false;
    mutable double lastCursorX_    = 0.0;
    mutable double lastCursorY_    = 0.0;
    mutable bool   haveLastCursor_ = false;
};

} // namespace harness
} // namespace demo
