#ifndef ENGINE_MODULES_RENDERER_CAMERA_CONTROLLER_HPP
#define ENGINE_MODULES_RENDERER_CAMERA_CONTROLLER_HPP

// Free-fly camera controller for the renderer module. Header only and
// Vulkan-free on purpose: it integrates keyboard and mouse state (through the
// mod-facing input contract) into the [scene.camera] state the renderer
// uploads every frame.
//
// Binding (all rebindable in the sense that the keys live in ONE function):
//   W / S           fly forward / back along the view direction
//   A / D           strafe left / right
//   Space / E       up        (world +Y)
//   LeftCtrl / Q    down      (world -Y)
//   LeftShift       speed boost (fly_boost multiplier)
//   mouse           look (yaw/pitch) while the cursor is captured
//   wheel           fly speed * 1.15 per notch (clamped)
//   F               toggle cursor capture
//   Left button     recapture the cursor
//   Escape          release the cursor
//
// The controller starts from the configured position/look_at and derives a
// yaw/pitch pair from that direction, so a configuration authored as a look
// target keeps its framing until the first input arrives.

#include "engine/capabilities/input_contract.hpp"
#include "engine/capabilities/logging_contract.hpp"
#include "scene_description.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace engine
{

class FlyCameraController
{
public:
    // Adopts the configured camera: position and yaw/pitch from
    // position -> look_at, speed knobs from the camera block.
    void syncFromCamera(const SceneCamera& camera)
    {
        position_ = camera.position;
        flySpeed_ = camera.flySpeed;
        flyBoost_ = camera.flyBoost;
        lookSensitivity_ = camera.lookSensitivity;

        float forward[3] = {camera.lookAt[0] - camera.position[0], camera.lookAt[1] - camera.position[1],
                            camera.lookAt[2] - camera.position[2]};
        const float length = std::sqrt(forward[0] * forward[0] + forward[1] * forward[1] + forward[2] * forward[2]);
        if (length > 1e-6f)
        {
            forward[0] /= length;
            forward[1] /= length;
            forward[2] /= length;
        }
        // Convention: yaw wraps the world Y axis starting from -Z, pitch is
        // positive upwards. forward = (cos(pitch) sin(yaw), sin(pitch),
        // -cos(pitch) cos(yaw)); the inverse below degenerates only for a
        // straight-up/down look_at, which the pitch clamp makes impossible to
        // re-enter afterwards.
        pitch_ = std::asin(std::clamp(forward[1], -1.0f, 1.0f));
        yaw_ = std::atan2(forward[0], -forward[2]);
    }

    // Writes the current fly state back into the camera block the renderer
    // uploads: position moves, look_at tracks the yaw/pitch direction.
    void applyToCamera(SceneCamera& camera) const
    {
        const float forwardX = std::cos(pitch_) * std::sin(yaw_);
        const float forwardY = std::sin(pitch_);
        const float forwardZ = -std::cos(pitch_) * std::cos(yaw_);
        camera.position = position_;
        camera.lookAt = {position_[0] + forwardX, position_[1] + forwardY, position_[2] + forwardZ};
    }

    // One frame of input integration. input may be null (capability absent):
    // the controller then keeps the camera exactly where it is, so the rest
    // of the engine cannot tell. dt must be > 0.
    void update(InputCapability* input, double deltaTime, LoggerCapability& logger)
    {
        if (input == nullptr || deltaTime <= 0.0)
        {
            return;
        }

        handleCaptureKeys(input, logger);

        if (input->isCursorCaptured())
        {
            double deltaX = 0.0;
            double deltaY = 0.0;
            input->consumeMouseDelta(deltaX, deltaY);
            // Screen x grows right -> turn right; screen y grows down -> look
            // up when the cursor moves up. Raw motion where available.
            yaw_ += static_cast<float>(deltaX) * lookSensitivity_;
            pitch_ -= static_cast<float>(deltaY) * lookSensitivity_;
            pitch_ = std::clamp(pitch_, -maxPitch, maxPitch);
        }
        else
        {
            // Not captured: drain and drop so a click-to-recapture does not
            // inherit stale motion.
            double deltaX = 0.0;
            double deltaY = 0.0;
            input->consumeMouseDelta(deltaX, deltaY);
        }

        double scrollY = 0.0;
        double scrollX = 0.0;
        input->consumeScrollDelta(scrollX, scrollY);
        if (scrollY != 0.0)
        {
            flySpeed_ =
                std::clamp(flySpeed_ * std::pow(flyWheelSpeedFactor, static_cast<float>(scrollY)), minSpeed, maxSpeed);
        }

        const float forwardX = std::cos(pitch_) * std::sin(yaw_);
        const float forwardY = std::sin(pitch_);
        const float forwardZ = -std::cos(pitch_) * std::cos(yaw_);
        // Right = forward x worldUp (level, independent of pitch: a strafe
        // should not tilt with the look direction).
        float rightX = -forwardZ;
        float rightZ = forwardX;
        const float rightLength = std::sqrt(rightX * rightX + rightZ * rightZ);
        if (rightLength > 1e-6f)
        {
            rightX /= rightLength;
            rightZ /= rightLength;
        }

        float moveX = 0.0f;
        float moveY = 0.0f;
        float moveZ = 0.0f;
        if (input->isKeyDown(KeyCode::W))
        {
            moveX += forwardX;
            moveY += forwardY;
            moveZ += forwardZ;
        }
        if (input->isKeyDown(KeyCode::S))
        {
            moveX -= forwardX;
            moveY -= forwardY;
            moveZ -= forwardZ;
        }
        if (input->isKeyDown(KeyCode::D))
        {
            moveX += rightX;
            moveZ += rightZ;
        }
        if (input->isKeyDown(KeyCode::A))
        {
            moveX -= rightX;
            moveZ -= rightZ;
        }
        if (input->isKeyDown(KeyCode::Space) || input->isKeyDown(KeyCode::E))
        {
            moveY += 1.0f;
        }
        if (input->isKeyDown(KeyCode::LeftControl) || input->isKeyDown(KeyCode::Q))
        {
            moveY -= 1.0f;
        }

        const float moveLength = std::sqrt(moveX * moveX + moveY * moveY + moveZ * moveZ);
        if (moveLength > 1e-6f)
        {
            const float speed = flySpeed_ * (input->isKeyDown(KeyCode::LeftShift) ? flyBoost_ : 1.0f);
            const float scale = speed * static_cast<float>(deltaTime) / moveLength;
            position_[0] += moveX * scale;
            position_[1] += moveY * scale;
            position_[2] += moveZ * scale;
        }
    }

private:
    static constexpr float maxPitch = 1.55334f; // just under 89 degrees
    static constexpr float minSpeed = 0.1f;
    static constexpr float maxSpeed = 200.0f;

    void handleCaptureKeys(InputCapability* input, LoggerCapability& logger)
    {
        if (togglePending_ && !input->isKeyDown(KeyCode::F))
        {
            togglePending_ = false;
        }
        if (!togglePending_ && input->isKeyDown(KeyCode::F))
        {
            togglePending_ = true;
            const bool captured = !input->isCursorCaptured();
            input->setCursorCaptured(captured);
            logger.logMessage(LogLevel::Information,
                              std::string("camera: cursor ") + (captured ? "captured (mouse look active, Esc releases)"
                                                                          : "released"));
        }
        if (input->isKeyDown(KeyCode::Escape) && input->isCursorCaptured())
        {
            input->setCursorCaptured(false);
        }
        if (!input->isCursorCaptured() && input->isMouseButtonDown(MouseButton::Left))
        {
            input->setCursorCaptured(true);
        }
    }

    std::array<float, 3> position_{};
    float yaw_ = 0.0f;
    float pitch_ = 0.0f;
    float flySpeed_ = defaultFlySpeed;
    float flyBoost_ = defaultFlyBoost;
    float lookSensitivity_ = defaultLookSensitivity;
    bool togglePending_ = false;
};

} // namespace engine

#endif // ENGINE_MODULES_RENDERER_CAMERA_CONTROLLER_HPP
