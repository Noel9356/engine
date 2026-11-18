// sandbox_input.cpp - window event translation for the physics sandbox:
// camera deltas, cursor capture toggling, grab edges, and keyboard actions.
#include "sandbox_input.hpp"

#include "demo_harness.hpp"

#include <algorithm>

namespace demo::physics_sandbox
{
namespace
{

constexpr double kMaximumCursorDeltaPixels = 64.0;

SandboxAction actionForKey(engine::window::Key key)
{
    switch (key)
    {
    case engine::window::Key::Num1:
        return SandboxAction::SpawnBox;
    case engine::window::Key::Num2:
        return SandboxAction::SpawnSphere;
    case engine::window::Key::Num3:
        return SandboxAction::SpawnCapsule;
    case engine::window::Key::B:
        return SandboxAction::Blast;
    case engine::window::Key::W:
        return SandboxAction::ToggleWind;
    case engine::window::Key::R:
        return SandboxAction::ResetScene;
    case engine::window::Key::P:
        return SandboxAction::TogglePause;
    case engine::window::Key::V:
        return SandboxAction::CycleFixedRate;
    case engine::window::Key::Escape:
        return SandboxAction::Quit;
    default:
        return SandboxAction::None;
    }
}

} // namespace

SandboxInput::SandboxInput(const engine::window::WindowCapability &windowCapability,
    engine::window::WindowHandle windowHandle, bool windowedBackend)
    : window_(&windowCapability),
      windowHandle_(windowHandle),
      windowedBackend_(windowedBackend)
{
    if (windowedBackend_)
    {
        setCursorCaptured(true);
    }
}

void SandboxInput::setCursorCaptured(bool captured)
{
    if (window_ == nullptr || !windowedBackend_)
    {
        return;
    }
    cursorCaptured_ = captured;
    window_->setCursorMode(window_->self,
        windowHandle_,
        captured ? engine::window::CursorMode::Captured : engine::window::CursorMode::Normal);
    haveLastCursor_ = false;
}

void SandboxInput::handleWindowEvent(const engine::window::WindowEvent &event)
{
    switch (event.kind)
    {
    case engine::window::WindowEventKind::CursorMoved: {
        cursorX_ = event.cursorX;
        cursorY_ = event.cursorY;
        if (haveLastCursor_)
        {
            double deltaX = event.cursorX - lastCursorX_;
            double deltaY = event.cursorY - lastCursorY_;
            demo::harness::clampCursorDelta(deltaX, deltaY, kMaximumCursorDeltaPixels);
            pendingDeltaX_ += deltaX;
            pendingDeltaY_ += deltaY;
        }
        lastCursorX_    = event.cursorX;
        lastCursorY_    = event.cursorY;
        haveLastCursor_ = true;
        break;
    }
    case engine::window::WindowEventKind::KeyPressed: {
        const SandboxAction action = actionForKey(event.key);
        if (action != SandboxAction::None)
        {
            actions_.push_back(action);
        }
        if (event.key == engine::window::Key::Space)
        {
            grabActive_ = !grabActive_;
            if (grabActive_)
            {
                grabPressed_ = true;
            }
            else
            {
                grabReleased_ = true;
            }
        }
        break;
    }
    case engine::window::WindowEventKind::MouseButtonPressed:
        if (event.mouseButton == engine::window::MouseButton::Left)
        {
            grabActive_  = true;
            grabPressed_ = true;
        }
        break;
    case engine::window::WindowEventKind::MouseButtonReleased:
        if (event.mouseButton == engine::window::MouseButton::Left)
        {
            grabActive_   = false;
            grabReleased_ = true;
        }
        break;
    default:
        break;
    }
}

engine::camera::CameraInputState SandboxInput::sampleCamera()
{
    engine::camera::CameraInputState sample;
    if (!windowedBackend_)
    {
        pendingDeltaX_ = 0.0;
        pendingDeltaY_ = 0.0;
        return sample;
    }
    sample.mouseDeltaX = static_cast<float>(pendingDeltaX_);
    sample.mouseDeltaY = static_cast<float>(pendingDeltaY_);
    pendingDeltaX_     = 0.0;
    pendingDeltaY_     = 0.0;
    if (window_ != nullptr)
    {
        sample.moveForward = window_->isKeyDown(window_->self, windowHandle_, engine::window::Key::Up)   ? 1.0f
                           : window_->isKeyDown(window_->self, windowHandle_, engine::window::Key::Down) ? -1.0f
                                                                                                         : 0.0f;
        sample.moveRight   = window_->isKeyDown(window_->self, windowHandle_, engine::window::Key::Right) ? 1.0f
                           : window_->isKeyDown(window_->self, windowHandle_, engine::window::Key::Left)  ? -1.0f
                                                                                                          : 0.0f;
        sample.zoom        = window_->isKeyDown(window_->self, windowHandle_, engine::window::Key::E) ? -1.0f
                           : window_->isKeyDown(window_->self, windowHandle_, engine::window::Key::Q) ? 1.0f
                                                                                                      : 0.0f;
    }
    return sample;
}

std::vector<SandboxAction> SandboxInput::drainActions()
{
    std::vector<SandboxAction> actions;
    actions.swap(actions_);
    return actions;
}

} // namespace demo::physics_sandbox
