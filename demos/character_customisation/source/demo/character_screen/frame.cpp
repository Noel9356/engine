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
    else if (event.kind == WindowEventKind::MouseScrolled)
    {
        // Wheel scrolling feeds the UI (hovered window scrolls).
        ui_->addMouseWheel(ui_->self, uiContext_, static_cast<float>(event.scrollX),
            static_cast<float>(event.scrollY));
    }
    else if (event.kind == WindowEventKind::FocusLost)
    {
        orbitDragging_  = false;
        middleScrolling_ = false;
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

    // Middle-mouse grab scrolling: holding the middle button and dragging
    // scrolls the hovered panel window (the same convention as the viewport
    // orbit, so either hand can drive the panel).
    if (mouseButtons_[2] && haveCursor)
    {
        if (!middleScrolling_)
        {
            middleScrolling_ = true;
            middleLastX_     = cursorX;
            middleLastY_     = cursorY;
        }
        const double deltaX = cursorX - middleLastX_;
        const double deltaY = cursorY - middleLastY_;
        middleLastX_ = cursorX;
        middleLastY_ = cursorY;
        if (std::fabs(deltaX) > 0.5 || std::fabs(deltaY) > 0.5)
        {
            ui_->addMouseWheel(ui_->self, uiContext_, static_cast<float>(deltaX * 0.02),
                static_cast<float>(-deltaY * 0.02));
        }
    }
    else
    {
        middleScrolling_ = false;
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
    // Tail sway: the tail is one rigid piece riding its root connection
    // offset, so the sway is a yaw oscillation of that offset (the module
    // re-derives the attached part's transform from the connectors).
    if (tailPresent_)
    {
        const InstanceId tail = instanceByName("tail");
        if (tail != engine::character::invalidInstance && animateTail_ && tailSwaySpeed_ > 0.01f)
        {
            const float sway = std::sin(static_cast<float>(elapsedSeconds) * 1.7f * tailSwaySpeed_) * 0.14f
                             * tailSwaySpeed_;
            float rotation[4];
            setYawRotation(rotation, sway);
            character_->setConnectionOffset(character_->self, characterHandle_, tail, nullptr, rotation);
        }
    }

    // Wing flap: rotate the connection offsets around the forward axis; the
    // right wing's socket is mirrored, so the sign flips. The flap COMPOSES
    // with the authored outward roll: both rotations are rolls about the
    // same axis, so composition is an angle sum.
    for (const char *wingName : {"wing_left", "wing_right"})
    {
        const InstanceId wing = instanceByName(wingName);
        if (wing == engine::character::invalidInstance)
        {
            continue;
        }
        const float flap = animateWings_
            ? std::sin(static_cast<float>(elapsedSeconds) * 2.4f) * 0.15f
            : 0.0f;
        const float authoredRoll = wingName[4] == 'l' ? -8.0f : 8.0f;
        float rotation[4];
        setRollRotation(rotation, glm::radians(authoredRoll + flap));
        character_->setConnectionOffset(character_->self, characterHandle_, wing, nullptr, rotation);
    }
}

bool CharacterScreen::renderPartPreviews(const std::string &previewDirectory,
    const std::function<bool(const std::string &)> &exportPpm)
{
    // Render one standalone frame per registered part: neutral three-quarter
    // camera framed on the part's bounds, the stage sky behind it, and a
    // PPM surface export per part. The capture host converts the PPM files
    // to PNG thumbnails for the panel (assets/previews/<partId>.png).
    if (!created_ || character_ == nullptr)
    {
        return false;
    }
    std::printf("part-previews: rendering part thumbnails into %s\n", previewDirectory.c_str());
    const std::vector<std::string> slots = {"head", "ears", "tail", "wings", "torso", "arms", "legs",
        "garment", "quadruped"};
    std::uint32_t rendered = 0;
    for (const std::string &slot : slots)
    {
        for (const std::string &partId : content_.partsInSlot(slot))
        {
            const PartRenderInfo *info = content_.renderInfoForPart(partId);
            if (info == nullptr)
            {
                continue;
            }
            const MeshHandle mesh = content_.meshForGeometry(info->geometryId);
            if (mesh == engine::renderer::invalidMesh)
            {
                continue;
            }
            // Frame the part's own geometry bounds.
            float boundsCenter[3] = {0.0f, 0.0f, 0.0f};
            float boundsRadius    = 0.3f;
            if (!content_.geometryBounds(info->geometryId, boundsCenter, &boundsRadius))
            {
                continue;
            }
            const float targetY = boundsCenter[1];
            // Tight framing: the bounding sphere fills most of the tile.
            const float distance = std::clamp(boundsRadius * 1.85f, 0.35f, 2.8f);
            configureCamera(38.0f, 10.0f, distance, targetY);
            // Push the camera's view state into the renderer (configureCamera
            // only sets the camera; the renderer needs the matrices).
            std::uint32_t previewWidth  = 0;
            std::uint32_t previewHeight = 0;
            window_->getFramebufferSize(window_->self, windowHandle_, &previewWidth, &previewHeight);
            engine::camera::ViewState previewView;
            camera_->viewState(camera_->self, cameraHandle_, previewWidth, previewHeight, &previewView);
            engine::renderer::CameraMatrices previewMatrices;
            std::memcpy(previewMatrices.view, previewView.view, sizeof(previewMatrices.view));
            std::memcpy(previewMatrices.projection, previewView.projection, sizeof(previewMatrices.projection));
            std::memcpy(previewMatrices.position, previewView.position, sizeof(previewMatrices.position));
            render_->setCamera(render_->self, rendererHandle_, &previewMatrices);
            render_->beginFrame(render_->self, rendererHandle_, nullptr);
            stage_.drawSkyAndSun();
            const float identity[16] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f};
            engine::renderer::MeshMaterial material = info->material;
            render_->drawMesh(render_->self, rendererHandle_, mesh, identity, &material);
            render_->endFrame(render_->self, rendererHandle_);
            const std::string ppmPath = previewDirectory + "/" + partId + ".ppm";
            if (exportPpm(ppmPath))
            {
                ++rendered;
            }
        }
    }
    std::printf("part-previews: %u thumbnails rendered\n", rendered);
    return rendered > 0;
}

} // namespace demo
