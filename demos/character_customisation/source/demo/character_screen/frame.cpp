#include "../character_screen_internal.hpp"
#include "../geometry.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <utility>

namespace demo
{
using namespace screen_detail;

void CharacterScreen::handleWindowEvent(const WindowEvent &event)
{
    if (ui_ == nullptr || uiContext_ == engine::ui::invalidUiContext)
    {
        return;
    }
    if (event.kind == WindowEventKind::KeyPressed || event.kind == WindowEventKind::KeyReleased)
    {
        const UiKey uiKey = mapWindowKey(event.key);
        if (uiKey != UiKey::None)
        {
            ui_->addKey(ui_->self, uiContext_, uiKey, event.kind == WindowEventKind::KeyPressed);
        }
    }
    else if (event.kind == WindowEventKind::MouseButtonPressed || event.kind == WindowEventKind::MouseButtonReleased)
    {
        const std::uint32_t button = static_cast<std::uint32_t>(event.mouseButton);
        if (button < 3)
        {
            mouseButtons_[button] = event.kind == WindowEventKind::MouseButtonPressed;
            ui_->addMouseButton(ui_->self, uiContext_, button, mouseButtons_[button]);
        }
    }
    else if (event.kind == WindowEventKind::FocusLost)
    {
        orbitDragging_ = false;
    }
}

// ---- per-frame -------------------------------------------------------------------

void CharacterScreen::frame(double deltaSeconds, double elapsedSeconds)
{
    if (!created_ || character_ == nullptr || characterHandle_ == engine::character::invalidCharacter)
    {
        return;
    }

    // Mouse position + button backstop poll (covers backends that only
    // expose live state, and recovers from missed events).
    double     cursorX    = 0.0;
    double     cursorY    = 0.0;
    const bool haveCursor = window_->cursorPosition(window_->self, windowHandle_, &cursorX, &cursorY)
                         && std::isfinite(cursorX) && std::isfinite(cursorY);
    if (haveCursor)
    {
        ui_->addMousePosition(ui_->self, uiContext_, static_cast<float>(cursorX), static_cast<float>(cursorY));
    }
    for (std::uint32_t button = 0; button < 3; ++button)
    {
        const bool down = window_->isMouseButtonDown(window_->self, windowHandle_, static_cast<MouseButton>(button));
        if (down != mouseButtons_[button])
        {
            mouseButtons_[button] = down;
            ui_->addMouseButton(ui_->self, uiContext_, button, down);
        }
    }

    // Animation drivers, then a single evaluation pass per frame: drivers
    // edit connection offsets, evaluate() re-derives every part transform
    // and the bounds from the attachment tree. Validation is event-driven
    // (structure/parameter changes), not per-frame.
    updateAnimationDrivers(deltaSeconds, elapsedSeconds);
    evaluateCharacter();

    // Camera: orbit by dragging the viewport (never the panel), zoom with
    // Space/Ctrl, arrow keys orbit too.
    engine::camera::CameraInputState cameraInput{};
    const std::uint64_t              keys = window_->keyStateMask(window_->self,
        windowHandle_,
        characterScreenKeyMaskBit(Key::Space) | characterScreenKeyMaskBit(Key::LeftControl) | characterScreenKeyMaskBit(Key::Left) | characterScreenKeyMaskBit(Key::Right)
            | characterScreenKeyMaskBit(Key::Up) | characterScreenKeyMaskBit(Key::Down));
    cameraInput.zoom                      = static_cast<float>((keys & characterScreenKeyMaskBit(Key::Space)) != 0)
                                          - static_cast<float>((keys & characterScreenKeyMaskBit(Key::LeftControl)) != 0);
    cameraInput.lookHorizontal            = static_cast<float>((keys & characterScreenKeyMaskBit(Key::Right)) != 0)
                                          - static_cast<float>((keys & characterScreenKeyMaskBit(Key::Left)) != 0);
    cameraInput.lookVertical              = static_cast<float>((keys & characterScreenKeyMaskBit(Key::Up)) != 0)
                                          - static_cast<float>((keys & characterScreenKeyMaskBit(Key::Down)) != 0);

    const bool dragAllowed = mouseButtons_[0] && !uiWantsMouse_;
    if (dragAllowed && !orbitDragging_ && haveCursor)
    {
        orbitDragging_ = true;
    }
    if (orbitDragging_ && (!mouseButtons_[0] || uiWantsMouse_))
    {
        orbitDragging_ = false;
    }
    if (orbitDragging_ && haveCursor && haveLastCursor_)
    {
        cameraInput.mouseDeltaX = static_cast<float>(std::clamp(cursorX - lastCursorX_, -200.0, 200.0));
        cameraInput.mouseDeltaY = static_cast<float>(std::clamp(cursorY - lastCursorY_, -200.0, 200.0));
    }
    lastCursorX_    = cursorX;
    lastCursorY_    = cursorY;
    haveLastCursor_ = haveCursor;

    // Keep the orbit radius proportional to the character bounds - but only
    // when the user did not request explicit framing (the CLI
    // --orbit-distance/--camera-target-y must survive, or junction
    // close-ups get re-framed away after one update).
    if (!userCameraFraming_)
    {
        float boundsCenter[3] = {0.0f, 1.0f, 0.0f};
        float boundsRadius    = 1.0f;
        if (character_->bounds(character_->self, characterHandle_, boundsCenter, &boundsRadius))
        {
            camera_->setOrbitDistance(camera_->self, cameraHandle_, std::clamp(boundsRadius * 2.4f, 1.8f, 9.0f));
        }
    }
    camera_->update(camera_->self, cameraHandle_, &cameraInput, deltaSeconds);
    std::uint32_t framebufferWidthForCamera  = 0;
    std::uint32_t framebufferHeightForCamera = 0;
    window_->getFramebufferSize(window_->self, windowHandle_, &framebufferWidthForCamera, &framebufferHeightForCamera);
    engine::camera::ViewState viewState;
    camera_->viewState(camera_->self, cameraHandle_, framebufferWidthForCamera, framebufferHeightForCamera, &viewState);
    engine::renderer::CameraMatrices cameraMatrices;
    std::memcpy(cameraMatrices.view, viewState.view, sizeof(cameraMatrices.view));
    std::memcpy(cameraMatrices.projection, viewState.projection, sizeof(cameraMatrices.projection));
    std::memcpy(cameraMatrices.position, viewState.position, sizeof(cameraMatrices.position));
    render_->setCamera(render_->self, rendererHandle_, &cameraMatrices);

    // Renderer frame was begun by main(): draw the 3D scene first, then the
    // overlay panel, and finish the UI frame during submission. The sky and
    // sun draw last so their occluded pixels depth-reject instead of
    // shading (the CPU rasterizer's budget is shaded pixels).
    stage_.drawFloor();
    drawCharacter();
    stage_.drawSkyAndSun();

    // UI frame.
    std::uint32_t displayWidth      = 0;
    std::uint32_t displayHeight     = 0;
    std::uint32_t framebufferWidth  = 0;
    std::uint32_t framebufferHeight = 0;
    window_->getSize(window_->self, windowHandle_, &displayWidth, &displayHeight);
    window_->getFramebufferSize(window_->self, windowHandle_, &framebufferWidth, &framebufferHeight);
    const float framebufferScaleX
        = displayWidth != 0 ? static_cast<float>(framebufferWidth) / static_cast<float>(displayWidth) : 1.0f;
    const float framebufferScaleY
        = displayHeight != 0 ? static_cast<float>(framebufferHeight) / static_cast<float>(displayHeight) : 1.0f;
    const UiFrameDescription uiFrame{
        .displayWidth      = static_cast<float>(displayWidth),
        .displayHeight     = static_cast<float>(displayHeight),
        .framebufferScaleX = framebufferScaleX,
        .framebufferScaleY = framebufferScaleY,
        .deltaSeconds      = static_cast<float>(std::clamp(deltaSeconds, 1.0e-3, 0.25))};
    if (!ui_->beginFrame(ui_->self, uiContext_, &uiFrame, nullptr))
    {
        return;
    }
    drawPanel(deltaSeconds);
    drawStatsOverlay();
    submitOverlay();
}

void CharacterScreen::updateAnimationDrivers(double deltaSeconds, double elapsedSeconds)
{
    (void)deltaSeconds;
    // Tail sway: the reptile tail is one rigid piece riding its root
    // connection offset, so the sway is a yaw oscillation of that offset
    // (the module re-derives the attached part's transform from the
    // connectors). The naga lower body sways slower with the whole body.
    const float nagaSway = animateTail_ && tailSwaySpeed_ > 0.01f
        ? std::sin(static_cast<float>(elapsedSeconds) * 0.9f * tailSwaySpeed_) * 0.08f * tailSwaySpeed_
        : 0.0f;
    const struct
    {
        const char *name;
        float       phase;
        float       amplitude;
    } tails[] = {
        {"tail", 0.0f, 0.16f},
        {"naga_body", 1.1f, 0.0f}, // naga uses nagaSway below
    };
    for (const auto &entry : tails)
    {
        const InstanceId instance = instanceByName(entry.name);
        if (instance == engine::character::invalidInstance)
        {
            continue;
        }
        const float amplitude = entry.amplitude > 0.0f ? entry.amplitude : std::fabs(nagaSway);
        float sway = 0.0f;
        if (animateTail_ && tailSwaySpeed_ > 0.01f)
        {
            sway = std::sin(static_cast<float>(elapsedSeconds) * 1.7f * tailSwaySpeed_ + entry.phase) * amplitude
                 * tailSwaySpeed_;
        }
        float rotation[4];
        setYawRotation(rotation, sway);
        character_->setConnectionOffset(character_->self, characterHandle_, instance, nullptr, rotation);
    }

    // Wing flap: rotate the connection offsets around the forward axis; the
    // right wing's socket is mirrored, so the sign flips. Up to three pairs
    // flap with a phase offset per pair (lower pairs trail the upper pair).
    for (int slot = 1; slot <= wingPairs_; ++slot)
    {
        const float flap = animateWings_
            ? std::sin(static_cast<float>(elapsedSeconds) * 2.4f - 0.55f * static_cast<float>(slot - 1)) * 0.15f
            : 0.0f;
        const std::string leftName = "wing" + std::to_string(slot) + "_left";
        const std::string rightName = "wing" + std::to_string(slot) + "_right";
        const InstanceId wingLeft = instanceByName(leftName);
        if (wingLeft != engine::character::invalidInstance)
        {
            float rotation[4];
            setRollRotation(rotation, flap);
            character_->setConnectionOffset(character_->self, characterHandle_, wingLeft, nullptr, rotation);
        }
        const InstanceId wingRight = instanceByName(rightName);
        if (wingRight != engine::character::invalidInstance)
        {
            float rotation[4];
            setRollRotation(rotation, -flap);
            character_->setConnectionOffset(character_->self, characterHandle_, wingRight, nullptr, rotation);
        }
    }
}


} // namespace demo
