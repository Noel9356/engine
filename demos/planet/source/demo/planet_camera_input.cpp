#include "planet_camera_input.hpp"

#include <algorithm>
#include <cmath>

namespace demo::planet_demo
{
namespace window = engine::window;
namespace camera = engine::camera;

PlanetCameraInput::PlanetCameraInput(const window::WindowCapability &windowCapability,
    window::WindowHandle windowHandle, bool windowedBackend, bool captureInitially)
    : window_(&windowCapability)
    , windowHandle_(windowHandle)
    , windowedBackend_(windowedBackend)
    , cursorToggleEnabled_(windowedBackend && captureInitially && windowCapability.setCursorMode != nullptr)
{
    setCursorCaptured(cursorToggleEnabled_);
}

void PlanetCameraInput::setCursorCaptured(bool captured)
{
    cursorCaptured_ = windowedBackend_ && captured;
    haveLastCursor_ = false;
    if (window_ != nullptr && window_->setCursorMode != nullptr)
    {
        window_->setCursorMode(window_->self, windowHandle_,
            cursorCaptured_ ? window::CursorMode::Captured : window::CursorMode::Normal);
    }
}

void PlanetCameraInput::handleWindowEvent(const window::WindowEvent &event)
{
    if (cursorToggleEnabled_ && event.kind == window::WindowEventKind::KeyPressed
        && event.key == window::Key::LeftAlt)
    {
        setCursorCaptured(!cursorCaptured_);
    }
}

camera::CameraInputState PlanetCameraInput::sample(bool enabled)
{
    camera::CameraInputState input{};
    if (!enabled || !windowedBackend_ || window_ == nullptr || window_->isKeyDown == nullptr)
    {
        haveLastCursor_ = false;
        return input;
    }

    const auto keyDown = [&](window::Key key)
    { return window_->isKeyDown(window_->self, windowHandle_, key); };

    input.moveForward = static_cast<float>(keyDown(window::Key::W))
                      - static_cast<float>(keyDown(window::Key::S));
    input.moveRight = static_cast<float>(keyDown(window::Key::D))
                    - static_cast<float>(keyDown(window::Key::A));
    input.moveUp = static_cast<float>(keyDown(window::Key::Space))
                 - static_cast<float>(keyDown(window::Key::LeftControl));
    input.lookHorizontal = static_cast<float>(keyDown(window::Key::Right))
                         - static_cast<float>(keyDown(window::Key::Left));
    input.lookVertical = static_cast<float>(keyDown(window::Key::Up))
                       - static_cast<float>(keyDown(window::Key::Down));
    input.sprint = keyDown(window::Key::LeftShift);

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
        // Cursor warps and focus transitions can produce large deltas. Clamp
        // them to the same conservative range used by the Lumen demo.
        input.mouseDeltaX = static_cast<float>(std::clamp(cursorX - lastCursorX_, -200.0, 200.0));
        input.mouseDeltaY = static_cast<float>(std::clamp(cursorY - lastCursorY_, -200.0, 200.0));
    }
    lastCursorX_ = cursorX;
    lastCursorY_ = cursorY;
    haveLastCursor_ = true;
    return input;
}

} // namespace demo::planet_demo
