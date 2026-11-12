// Configuration handling for the camera module.
//
// CameraDefaults is the single set of defaults new cameras start from: the
// built-in fallbacks, replaced by whatever valid [camera] keys the
// configuration provides. loadCameraDefaults re-reads the section, so the
// same path serves construct() and configuration hot-reload.

#ifndef ENGINE_CAMERA_CONFIGURATION_HPP
#define ENGINE_CAMERA_CONFIGURATION_HPP

#include <engine/camera/capabilities.hpp>
#include <engine/kernel/participant_interface.hpp>

#include <cstddef>

namespace engine
{
namespace camera
{

// Built-in fallbacks used when [camera] configuration is missing or invalid.
inline constexpr float builtInFovYDegrees            = 55.0f;
inline constexpr float builtInNearZ                  = 0.1f;
inline constexpr float builtInFarZ                   = 100.0f;
inline constexpr float builtInOrthographicHeight     = 10.0f;
inline constexpr float builtInMoveSpeed              = 4.0f;
inline constexpr float builtInLookSpeed              = 60.0f;
inline constexpr float builtInMouseSensitivity       = 0.11f;
inline constexpr float builtInOrbitDistance          = 6.0f;
inline constexpr float builtInSprintMultiplier       = 3.0f;
inline constexpr float builtInMaximumDeltaSeconds    = 0.25f;
inline constexpr float builtInPitchLimitDegrees      = 89.0f;
inline constexpr std::size_t builtInMaximumCameras   = 256u;
inline constexpr std::size_t maximumConfiguredCameras = 65536u;

// Defaults handed to every newly created camera; refreshed from the [camera]
// configuration section. orbitZoomSpeed tracks moveSpeed unless the
// orbit_zoom_speed key overrides it.
struct CameraDefaults
{
    CameraControlMode mode                = CameraControlMode::Fly;
    float             moveSpeed           = builtInMoveSpeed;
    float             lookSpeed           = builtInLookSpeed;
    float             mouseSensitivity    = builtInMouseSensitivity;
    float             orbitZoomSpeed      = builtInMoveSpeed;
    float             orbitDistance       = builtInOrbitDistance;
    float             fovYDegrees         = builtInFovYDegrees;
    float             nearZ               = builtInNearZ;
    float             farZ                = builtInFarZ;
    float             orthographicHeight  = builtInOrthographicHeight;
    float             sprintMultiplier    = builtInSprintMultiplier;
    float             maximumDeltaSeconds = builtInMaximumDeltaSeconds;
    float             pitchLimitDegrees   = builtInPitchLimitDegrees;
    std::size_t       maximumCameras      = builtInMaximumCameras;
};

// Resets defaults to the built-ins, then adopts every valid [camera] key.
// Values outside their documented range are reported on stderr and keep the
// built-in default.
void loadCameraDefaults(ParticipantContext &context, CameraDefaults &defaults);

} // namespace camera
} // namespace engine

#endif
