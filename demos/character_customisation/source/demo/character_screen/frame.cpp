#include "../character_screen_internal.hpp"
#include "../geometry.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

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
        ui_->addMouseWheel(ui_->self, uiContext_, static_cast<float>(event.scrollX), static_cast<float>(event.scrollY));
    }
    else if (event.kind == WindowEventKind::FocusLost)
    {
        orbitDragging_   = false;
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
        middleLastX_        = cursorX;
        middleLastY_        = cursorY;
        if (std::fabs(deltaX) > 0.5 || std::fabs(deltaY) > 0.5)
        {
            ui_->addMouseWheel(
                ui_->self, uiContext_, static_cast<float>(deltaX * 0.02), static_cast<float>(-deltaY * 0.02));
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
        characterScreenKeyMaskBit(Key::Space) | characterScreenKeyMaskBit(Key::LeftControl)
            | characterScreenKeyMaskBit(Key::Left) | characterScreenKeyMaskBit(Key::Right)
            | characterScreenKeyMaskBit(Key::Up) | characterScreenKeyMaskBit(Key::Down));
    cameraInput.zoom           = static_cast<float>((keys & characterScreenKeyMaskBit(Key::Space)) != 0)
                               - static_cast<float>((keys & characterScreenKeyMaskBit(Key::LeftControl)) != 0);
    cameraInput.lookHorizontal = static_cast<float>((keys & characterScreenKeyMaskBit(Key::Right)) != 0)
                               - static_cast<float>((keys & characterScreenKeyMaskBit(Key::Left)) != 0);
    cameraInput.lookVertical   = static_cast<float>((keys & characterScreenKeyMaskBit(Key::Up)) != 0)
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

    // The assembly's world box, rebuilt every frame: it drives both the
    // automatic framing (below) and the contact-shadow footprint. Every
    // instance's part bounds ride its instance transform.
    glm::vec3           worldMin(std::numeric_limits<float>::max());
    glm::vec3           worldMax(std::numeric_limits<float>::lowest());
    bool                haveExtents = false;
    const std::uint32_t count       = character_->instanceCount(character_->self, characterHandle_);
    for (std::uint32_t index = 0; index < count; ++index)
    {
        const InstanceId id     = character_->instanceId(character_->self, characterHandle_, index);
        const char      *partId = nullptr;
        if (!character_->instanceInfo(character_->self, characterHandle_, id, nullptr, &partId, nullptr)
            || partId == nullptr)
        {
            continue;
        }
        const PartRenderInfo *renderInfo = content_.renderInfoForPart(partId);
        if (renderInfo == nullptr)
        {
            continue;
        }
        float minimum[3] = {};
        float maximum[3] = {};
        if (!content_.geometryExtents(renderInfo->geometryId, minimum, maximum))
        {
            continue;
        }
        float matrix[16] = {};
        if (!character_->instanceTransform(character_->self, characterHandle_, id, matrix))
        {
            continue;
        }
        const glm::mat4 world = glm::make_mat4(matrix);
        for (int corner = 0; corner < 8; ++corner)
        {
            const glm::vec4 local(corner & 1 ? maximum[0] : minimum[0],
                corner & 2 ? maximum[1] : minimum[1],
                corner & 3 ? maximum[2] : minimum[2],
                1.0f);
            const glm::vec3 cornerWorld(world * local);
            worldMin    = glm::min(worldMin, cornerWorld);
            worldMax    = glm::max(worldMax, cornerWorld);
            haveExtents = true;
        }
    }

    // Keep the orbit radius proportional to the character bounds - but only
    // when the user did not request explicit framing (the CLI
    // --orbit-distance/--camera-target-y must survive, or junction
    // close-ups get re-framed away after one update).
    if (!userCameraFraming_ && haveExtents)
    {
        // Frame the assembly's world-space box in VIEW space: project the
        // box corners with the current view, measure how much camera
        // retreat along the view axis each corner needs to sit inside a
        // filled frustum, and take the maximum. (Translating the camera
        // along its forward axis leaves every corner's view-space x/y
        // unchanged, so the needed retreat is exact.) A sphere around a
        // long, flat animal overshoots its silhouette; this measures the
        // real silhouette. The orbit TARGET is the box centre itself -
        // long-tailed taurs centre far behind their torso, and targeting
        // the origin pushed those characters off-frame. With the studio
        // panel sharing the frame the usable viewport is only the panel's
        // left side, so the fit measures THAT frustum and the target rides
        // the camera's right axis until the character centres in it.
        const glm::vec3 center    = (worldMin + worldMax) * 0.5f;
        std::uint32_t   fitWidth  = 0;
        std::uint32_t   fitHeight = 0;
        window_->getFramebufferSize(window_->self, windowHandle_, &fitWidth, &fitHeight);
        const float frameWidth  = static_cast<float>(fitWidth);
        const float usableWidth = std::max(frameWidth - (panelVisible_ ? 430.0f : 0.0f), 0.35f * frameWidth);
        engine::camera::ViewState fullView{};
        engine::camera::ViewState usableView{};
        camera_->viewState(camera_->self, cameraHandle_, fitWidth, fitHeight, &fullView);
        camera_->viewState(
            camera_->self, cameraHandle_, static_cast<std::uint32_t>(usableWidth), fitHeight, &usableView);
        const glm::mat4 view           = glm::make_mat4(usableView.view);
        const glm::mat4 projection     = glm::make_mat4(usableView.projection);
        const float     tangentV       = std::fabs(projection[1][1]) > 1e-6f ? 1.0f / projection[1][1] : 1.0f;
        const float     tangentH       = std::fabs(projection[0][0]) > 1e-6f ? 1.0f / projection[0][0] : 1.0f;
        const glm::mat4 fullProjection = glm::make_mat4(fullView.projection);
        const float     fullTangentH   = std::fabs(fullProjection[0][0]) > 1e-6f ? 1.0f / fullProjection[0][0] : 1.0f;
        const glm::vec3 eye            = glm::make_vec3(usableView.position);
        const glm::vec3 forward        = center - eye;
        const float     eyeToCenter    = std::max(glm::length(forward), 0.1f);
        const glm::vec3 forwardDir     = forward / eyeToCenter;
        const glm::vec3 right          = glm::normalize(glm::cross(forwardDir, glm::vec3(0.0f, 1.0f, 0.0f)));
        const float     usableFraction = usableWidth / std::max(frameWidth, 1.0f);
        const glm::vec3 shiftedTarget  = center + right * ((1.0f - usableFraction) * eyeToCenter * fullTangentH);
        const float     target[3]      = {shiftedTarget.x, shiftedTarget.y, shiftedTarget.z};
        camera_->setOrbitTarget(camera_->self, cameraHandle_, target);

        // The signed retreat each corner needs (negative = the camera may
        // approach); the binding corner decides.
        float           retreat = -std::numeric_limits<float>::max();
        constexpr float kFill   = 0.72f;
        for (int corner = 0; corner < 8; ++corner)
        {
            const glm::vec4 local(corner & 1 ? worldMax.x : worldMin.x,
                corner & 2 ? worldMax.y : worldMin.y,
                corner & 3 ? worldMax.z : worldMin.z,
                1.0f);
            const glm::vec4 viewCorner = view * local;
            const float     needed
                = std::max(std::fabs(viewCorner.x) / (tangentH * kFill), std::fabs(viewCorner.y) / (tangentV * kFill));
            const float depth = std::max(-viewCorner.z, 0.05f);
            retreat           = std::max(retreat, needed - depth);
        }
        if (retreat == -std::numeric_limits<float>::max())
        {
            retreat = 0.0f;
        }
        const float currentDistance = std::max(glm::length(eye - center), 0.1f);
        // The floor follows the silhouette: a rat quadruped needs a
        // sub-metre orbit, a winged biped several metres - a fixed 1.2 m
        // minimum left every small animal lost in its frame.
        camera_->setOrbitDistance(camera_->self, cameraHandle_, std::clamp(currentDistance + retreat, 0.35f, 9.0f));
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
    // The fur LOD measures part distance from the camera eye; capture it for
    // drawCharacter.
    std::memcpy(lastCameraEye_, viewState.position, sizeof(lastCameraEye_));
    render_->setCamera(render_->self, rendererHandle_, &cameraMatrices);

    // Renderer frame was begun by main(): draw the 3D scene first, then the
    // overlay panel, and finish the UI frame during submission. The sky and
    // sun draw last so their occluded pixels depth-reject instead of
    // shading (the CPU rasterizer's budget is shaded pixels).
    stage_.drawFloor();
    if (haveExtents)
    {
        // Ground the silhouette: a soft gradient disc under the character's
        // ground footprint (the horizontal extent of the world box).
        const glm::vec3 footprint = worldMax - worldMin;
        const glm::vec3 center    = (worldMin + worldMax) * 0.5f;
        const float     radius    = std::clamp(0.5f * std::max(footprint.x, footprint.z) + 0.06f, 0.22f, 1.8f);
        stage_.drawContactShadow(center.x, center.z, radius);
    }
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
    if (!hideHud_)
    {
        drawStatsOverlay();
    }
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
            const float sway
                = std::sin(static_cast<float>(elapsedSeconds) * 1.7f * tailSwaySpeed_) * 0.14f * tailSwaySpeed_;
            float rotation[4];
            setYawRotation(rotation, sway);
            character_->setConnectionOffset(character_->self, characterHandle_, tail, nullptr, rotation);
        }
    }

    // Wing flap: the wings arch through the connection's pitch (up and back)
    // with a small outward roll; the flap swings the roll angle, composing
    // with the authored arch exactly like the static connection.
    for (const char *wingName : {"wing_left", "wing_right"})
    {
        const InstanceId wing = instanceByName(wingName);
        if (wing == engine::character::invalidInstance)
        {
            continue;
        }
        const float     flap = animateWings_ ? std::sin(static_cast<float>(elapsedSeconds) * 2.4f) * 0.15f : 0.0f;
        const float     authoredRoll     = glm::radians(wingName[4] == 'l' ? -8.0f : 8.0f + glm::degrees(flap));
        const glm::quat roll             = glm::angleAxis(authoredRoll, glm::vec3(0.0f, 0.0f, 1.0f));
        const glm::quat pitch            = glm::angleAxis(glm::radians(-105.0f), glm::vec3(1.0f, 0.0f, 0.0f));
        const glm::quat rotation         = pitch * roll;
        const float     rotationArray[4] = {rotation.x, rotation.y, rotation.z, rotation.w};
        character_->setConnectionOffset(character_->self, characterHandle_, wing, nullptr, rotationArray);
    }
}

} // namespace demo
