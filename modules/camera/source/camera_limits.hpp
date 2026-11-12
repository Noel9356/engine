// Shared validation ranges and small numeric helpers for the camera module.
//
// One home for the numeric bounds every input path enforces - configuration
// keys, creation parameters, per-camera setters, and projection clamping - so
// the ranges cannot drift apart between those paths.

#ifndef ENGINE_CAMERA_LIMITS_HPP
#define ENGINE_CAMERA_LIMITS_HPP

#include <cmath>

namespace engine
{
namespace camera
{
namespace limits
{

// Hard FOV clamp (in degrees) applied to every camera, whatever its source:
// configuration, createCamera parameters, or setProjection.
inline constexpr float minimumFovYDegrees        = 0.1f;
inline constexpr float maximumFovYDegrees        = 179.0f;

// Upper bounds for the per-camera tuning fields and their configuration
// keys; every field must also be finite and positive.
inline constexpr float maximumMoveSpeed                   = 1.0e6f;
inline constexpr float maximumLookSpeed                   = 3600.0f;
inline constexpr float maximumMouseSensitivity            = 100.0f;
inline constexpr float maximumOrbitDistance               = 1.0e8f;
inline constexpr float maximumOrbitZoomSpeed              = 1.0e6f;
inline constexpr float maximumSprintMultiplier            = 100.0f;
inline constexpr float maximumDeltaSeconds                = 5.0f;
inline constexpr float pitchLimitUpperBoundDegrees        = 90.0f;

// Upper bounds for the projection planes and the orthographic height.
inline constexpr float maximumNearZ               = 1.0e6f;
inline constexpr float maximumFarZ                = 1.0e9f;
inline constexpr float maximumOrthographicHeight  = 1.0e8f;

// Lower bounds applied when projection values are clamped after adoption.
inline constexpr float minimumNearZ              = 1e-4f;
inline constexpr float minimumFarAboveNearZ      = 1e-3f;
inline constexpr float minimumOrthographicHeight = 1e-4f;

// Orbit radius floor applied while zooming (world units).
inline constexpr float minimumOrbitZoomDistance = 0.5f;

// Smallest orbit radius derived from a pose; below it the configured default
// is used instead.
inline constexpr float minimumPoseDerivedOrbitDistance = 1e-4f;

// Adopts value into destination when it is finite and inside (0, maximum].
inline bool adoptPositiveFinite(float value, float maximum, float &destination) noexcept
{
    if (!std::isfinite(value) || value <= 0.0f || value > maximum)
    {
        return false;
    }
    destination = value;
    return true;
}

} // namespace limits
} // namespace camera
} // namespace engine

#endif
