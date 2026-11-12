// Stateless orientation math shared by the camera records.
//
// Orientation lives in a reference frame derived from the up vector: the
// forward basis is +Z projected onto the plane perpendicular to up (falling
// back to +X when the two are nearly parallel), and the right basis is their
// cross product. Yaw is the angle around up measured from the forward basis,
// pitch the elevation, so a changed up vector never corrupts the pose.

#ifndef ENGINE_CAMERA_ORIENTATION_HPP
#define ENGINE_CAMERA_ORIENTATION_HPP

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>

namespace engine
{
namespace camera
{

inline bool finiteVector(const glm::vec3 &value) noexcept
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

// Basis derived from an up vector: forward = +Z projected onto the plane
// perpendicular to up, right = cross(up, forward); both normalized. A
// non-finite or near-zero up falls back to +Y.
struct OrientationBasis
{
    glm::vec3 up      = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 forward = glm::vec3(0.0f, 0.0f, 1.0f);
    glm::vec3 right   = glm::vec3(1.0f, 0.0f, 0.0f);
};

inline OrientationBasis orientationBasisFromUp(const glm::vec3 &up) noexcept
{
    glm::vec3 unitUp = up;
    if (finiteVector(up))
    {
        const float lengthSquared = glm::dot(up, up);
        if (std::isfinite(lengthSquared) && lengthSquared > 1e-12f)
        {
            unitUp = up * glm::inversesqrt(lengthSquared);
        }
        else
        {
            unitUp = glm::vec3(0.0f, 1.0f, 0.0f);
        }
    }
    else
    {
        unitUp = glm::vec3(0.0f, 1.0f, 0.0f);
    }

    const glm::vec3 preferredForward(0.0f, 0.0f, 1.0f);
    glm::vec3       projected = preferredForward - unitUp * glm::dot(preferredForward, unitUp);
    if (glm::dot(projected, projected) <= 1e-8f)
    {
        const glm::vec3 fallback(1.0f, 0.0f, 0.0f);
        projected = fallback - unitUp * glm::dot(fallback, unitUp);
    }
    OrientationBasis basis;
    basis.up      = unitUp;
    basis.forward = glm::normalize(projected);
    basis.right   = glm::normalize(glm::cross(unitUp, basis.forward));
    return basis;
}

// Direction <-> (yaw, pitch) conversion in a basis: yaw is the angle around
// up from the forward basis, pitch the elevation (clamped to the pitch
// limit). direction must be finite and unit-length.
inline void yawPitchFromDirection(const glm::vec3 &direction, const OrientationBasis &basis, float pitchLimitDegrees,
    float &outYawDegrees, float &outPitchDegrees) noexcept
{
    const float     vertical = std::clamp(glm::dot(direction, basis.up), -1.0f, 1.0f);
    const glm::vec3 planar   = direction - basis.up * vertical;
    outYawDegrees            = glm::degrees(std::atan2(glm::dot(planar, basis.right), glm::dot(planar, basis.forward)));
    outPitchDegrees
        = std::clamp(glm::degrees(std::asin(vertical)), -pitchLimitDegrees, pitchLimitDegrees);
}

inline glm::vec3 directionFromYawPitch(float yawDegrees, float pitchDegrees, const OrientationBasis &basis) noexcept
{
    const float yawRadians   = glm::radians(yawDegrees);
    const float pitchRadians = glm::radians(pitchDegrees);
    const float planarScale  = std::cos(pitchRadians);
    return basis.forward * (std::cos(yawRadians) * planarScale)
         + basis.right * (std::sin(yawRadians) * planarScale) + basis.up * std::sin(pitchRadians);
}

// Horizontal strafe-right direction for a yaw: the forward basis rotated by
// the yaw, so it stays perpendicular to the view while the pitch only tilts
// the view direction itself. Equals normalize(cross(viewDirection, up)).
inline glm::vec3 rightDirectionFromYaw(float yawDegrees, const OrientationBasis &basis) noexcept
{
    const float yawRadians = glm::radians(yawDegrees);
    return basis.forward * std::sin(yawRadians) - basis.right * std::cos(yawRadians);
}

} // namespace camera
} // namespace engine

#endif
