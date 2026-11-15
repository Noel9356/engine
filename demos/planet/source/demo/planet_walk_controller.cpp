// First-person surface walk controller implementation (see the header).

#include <engine/camera/capabilities.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/walker.hpp>
#include "planet_walk_controller.hpp"

#include <engine/planet/types.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>

namespace demo::planet_demo
{
namespace planet = engine::planet;

namespace
{
// Pitch clamp mirroring the camera module's limit (just under straight up/down).
constexpr double kPitchLimitRadians = 1.5533430342748532; // 89 degrees

glm::dvec3 toGlm(const planet::Vec3 &v)
{
    return glm::dvec3(v.x, v.y, v.z);
}

planet::Vec3 toPlanet(const glm::dvec3 &v)
{
    return planet::Vec3{v.x, v.y, v.z};
}

} // namespace

PlanetWalkController::PlanetWalkController(const planet::PlanetGenerationParams &params,
    const glm::dvec3 &planetCenter)
    : params_(params)
    , planetCenter_(planetCenter)
    , field_(params)
    , walker_(params, field_)
{
}

void PlanetWalkController::spawn(const planet::Vec3 &unitDirection)
{
    walker_.placeOnSurface(unitDirection);

    // Look along the horizon tangent at the spawn point (world +x projected
    // onto the tangent plane; falls back to +z where +x is radial).
    const glm::dvec3 up = toGlm(walker_.state().up);
    glm::dvec3       tangent = glm::dvec3(1.0, 0.0, 0.0) - up * glm::dot(glm::dvec3(1.0, 0.0, 0.0), up);
    if (glm::length(tangent) < 0.5)
    {
        tangent = glm::dvec3(0.0, 0.0, 1.0) - up * glm::dot(glm::dvec3(0.0, 0.0, 1.0), up);
    }
    viewForward_ = glm::normalize(tangent);
    autoWalkYaw_ = 0.0;
    clampPitch();
}

void PlanetWalkController::clampPitch()
{
    // viewForward_ is kept orthogonal to the current up; its angle below the
    // tangent plane is the pitch. The clamp keeps the view out of the ground
    // and the sky, matching the camera module's pitch limit.
    const glm::dvec3 upAxis   = up();
    const double     alongUp  = glm::dot(viewForward_, upAxis);
    const glm::dvec3 tangent  = viewForward_ - upAxis * alongUp;
    const double     tangentLength = glm::length(tangent);
    if (tangentLength < 1.0e-9)
    {
        // Look direction collapsed onto the pole of the local frame: rebuild
        // a level tangent.
        glm::dvec3 fallback = glm::dvec3(1.0, 0.0, 0.0) - upAxis * upAxis.x;
        if (glm::length(fallback) < 0.5)
        {
            fallback = glm::dvec3(0.0, 0.0, 1.0) - upAxis * upAxis.z;
        }
        viewForward_ = glm::normalize(fallback);
        return;
    }
    const double pitch = std::atan2(-alongUp, tangentLength);
    const double clamped = std::clamp(pitch, -kPitchLimitRadians, kPitchLimitRadians);
    if (clamped != pitch)
    {
        const glm::dvec3 level = tangent / tangentLength;
        viewForward_ = level * std::cos(clamped) - upAxis * std::sin(clamped);
    }
}

void PlanetWalkController::update(const engine::camera::CameraInputState &input, bool enabled, double deltaSeconds)
{
    planet::WalkIntent intent{};

    const glm::dvec3 upBefore = up();
    if (autoWalk_)
    {
        // Deterministic capture path: walk forward with a slow constant
        // turn, pitch level with the horizon.
        autoWalkYaw_ += 0.03 * deltaSeconds;
        const double c = std::cos(autoWalkYaw_);
        const double s = std::sin(autoWalkYaw_);
        const glm::dvec3 level = glm::normalize(viewForward_ - upBefore * glm::dot(viewForward_, upBefore));
        const glm::dvec3 right = glm::cross(level, upBefore);
        viewForward_ = glm::normalize(level * c + right * s);
        intent.forward = 1.0;
        intent.heading = toPlanet(viewForward_);
        intent.sprint  = false;
        intent.jump    = false;
    }
    else if (enabled)
    {
        // Mouse look: yaw rotates the view around the walker's up, pitch
        // around the strafe axis.
        const double yawDelta   = -static_cast<double>(input.mouseDeltaX) * 0.0022;
        const double pitchDelta = -static_cast<double>(input.mouseDeltaY) * 0.0022;
        if (yawDelta != 0.0 || pitchDelta != 0.0)
        {
            const glm::dvec3 level = viewForward_ - upBefore * glm::dot(viewForward_, upBefore);
            if (glm::length(level) > 1.0e-9)
            {
                const glm::dvec3 right   = glm::cross(glm::normalize(level), upBefore);
                const glm::dvec3 yawed   = glm::rotate(glm::dmat4(1.0), yawDelta, upBefore) * glm::dvec4(viewForward_, 0.0);
                const glm::dvec3 pitched = glm::rotate(glm::dmat4(1.0), pitchDelta, right) * glm::dvec4(yawed, 0.0);
                if (glm::length(pitched) > 1.0e-9)
                {
                    viewForward_ = glm::normalize(pitched);
                }
            }
        }
        // Keyboard look (arrows) for windowless remotes: same axes.
        if (input.lookHorizontal != 0.0f || input.lookVertical != 0.0f)
        {
            const glm::dvec3 level = viewForward_ - upBefore * glm::dot(viewForward_, upBefore);
            if (glm::length(level) > 1.0e-9)
            {
                const glm::dvec3 right = glm::cross(glm::normalize(level), upBefore);
                const double speed = 1.8 * deltaSeconds;
                const glm::dvec3 yawed   = glm::rotate(glm::dmat4(1.0), -speed * input.lookHorizontal, upBefore) * glm::dvec4(viewForward_, 0.0);
                const glm::dvec3 pitched = glm::rotate(glm::dmat4(1.0), speed * input.lookVertical, right) * glm::dvec4(yawed, 0.0);
                if (glm::length(pitched) > 1.0e-9)
                {
                    viewForward_ = glm::normalize(pitched);
                }
            }
        }
        clampPitch();

        intent.forward = std::clamp(static_cast<double>(input.moveForward), -1.0, 1.0);
        intent.right   = std::clamp(static_cast<double>(input.moveRight), -1.0, 1.0);
        intent.jump    = input.moveUp > 0.5f; // Space requests a jump from the ground
        intent.sprint  = input.sprint;
        intent.heading = toPlanet(viewForward_);
    }

    walker_.update(deltaSeconds, intent);
}

glm::dvec3 PlanetWalkController::eye() const
{
    return toGlm(state().eyePosition) + planetCenter_;
}

glm::dvec3 PlanetWalkController::target() const
{
    return eye() + viewForward_ * 40.0;
}

} // namespace demo::planet_demo
