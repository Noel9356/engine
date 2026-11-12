// Loads the [camera] configuration section into CameraDefaults.
//
// Keys outside their documented range are reported on stderr and leave the
// built-in default in place; nothing is clamped silently. The plain
// positive-bounded keys share one table-driven reader, while mode, fov,
// pitch limit, camera capacity, and the far/near plane relationship keep
// their dedicated checks.

#include "camera_configuration.hpp"
#include "camera_limits.hpp"

#include <algorithm>
#include <cstdio>
#include <optional>
#include <string>

namespace engine
{
namespace camera
{
namespace
{

// Reads a key that must be finite and inside (0, maximum]; invalid present
// values warn on stderr and keep the current default.
void readPositiveKey(ParticipantContext &context, const char *key, float &destination, double maximum)
{
    const std::optional<double> value = context.configurationFloatingInSection("camera", key);
    if (!value.has_value())
    {
        return;
    }
    if (!std::isfinite(*value) || *value <= 0.0 || *value > maximum)
    {
        std::fprintf(stderr,
            "camera module: ignoring [camera] %s; expected a finite value in (0, %.3g]\n",
            key,
            maximum);
        return;
    }
    destination = static_cast<float>(*value);
}

// The plain positive-bounded [camera] keys and where they land.
struct FloatingKey
{
    const char        *key;
    float CameraDefaults::*field;
    double             maximum;
};

constexpr FloatingKey floatingKeys[] = {
    {"move_speed", &CameraDefaults::moveSpeed, limits::maximumMoveSpeed},
    {"look_speed", &CameraDefaults::lookSpeed, limits::maximumLookSpeed},
    {"mouse_sensitivity", &CameraDefaults::mouseSensitivity, limits::maximumMouseSensitivity},
    {"near_z", &CameraDefaults::nearZ, limits::maximumNearZ},
    {"far_z", &CameraDefaults::farZ, limits::maximumFarZ},
    {"orthographic_height", &CameraDefaults::orthographicHeight, limits::maximumOrthographicHeight},
    {"orbit_distance", &CameraDefaults::orbitDistance, limits::maximumOrbitDistance},
    {"sprint_multiplier", &CameraDefaults::sprintMultiplier, limits::maximumSprintMultiplier},
    {"max_delta_seconds", &CameraDefaults::maximumDeltaSeconds, limits::maximumDeltaSeconds},
};

} // namespace

void loadCameraDefaults(ParticipantContext &context, CameraDefaults &defaults)
{
    defaults = CameraDefaults{};

    if (const auto value = context.configurationIntegerInSection("camera", "maximum_cameras");
        value && *value > 0 && static_cast<std::size_t>(*value) <= maximumConfiguredCameras)
    {
        defaults.maximumCameras = static_cast<std::size_t>(*value);
    }

    const std::optional<std::string> mode = context.configurationStringInSection("camera", "mode");
    if (mode.has_value())
    {
        if (*mode == "orbit")
        {
            defaults.mode = CameraControlMode::Orbit;
        }
        else if (*mode == "fly")
        {
            defaults.mode = CameraControlMode::Fly;
        }
        else if (*mode == "fixed")
        {
            defaults.mode = CameraControlMode::Fixed;
        }
        else
        {
            std::fprintf(stderr, "camera module: unknown [camera] mode '%s'; using \"fly\"\n", mode->c_str());
        }
    }

    for (const FloatingKey &entry : floatingKeys)
    {
        readPositiveKey(context, entry.key, defaults.*entry.field, entry.maximum);
    }
    // orbit_zoom_speed tracks the resolved move_speed unless configured.
    defaults.orbitZoomSpeed = defaults.moveSpeed;
    readPositiveKey(context, "orbit_zoom_speed", defaults.orbitZoomSpeed, limits::maximumOrbitZoomSpeed);

    const std::optional<double> fov = context.configurationFloatingInSection("camera", "fov_degrees");
    if (fov.has_value())
    {
        if (std::isfinite(*fov) && *fov > limits::minimumFovYDegrees && *fov < limits::maximumFovYDegrees)
        {
            defaults.fovYDegrees = static_cast<float>(*fov);
        }
        else
        {
            std::fprintf(stderr,
                "camera module: ignoring [camera] fov_degrees; expected a finite value in (0.1, 179)\n");
        }
    }

    const std::optional<double> pitchLimit = context.configurationFloatingInSection("camera", "pitch_limit_degrees");
    if (pitchLimit.has_value())
    {
        if (std::isfinite(*pitchLimit) && *pitchLimit > 0.0 && *pitchLimit < limits::pitchLimitUpperBoundDegrees)
        {
            defaults.pitchLimitDegrees = static_cast<float>(*pitchLimit);
        }
        else
        {
            std::fprintf(stderr, "camera module: ignoring [camera] pitch_limit_degrees; expected (0, 90)\n");
        }
    }

    // far must exceed near; otherwise restore the built-in far plane (or
    // near + 1 when the configured far is usable but too close).
    if (defaults.farZ <= defaults.nearZ)
    {
        std::fprintf(stderr, "camera module: [camera] far_z must exceed near_z; using the built-in far plane\n");
        defaults.farZ = std::max(builtInFarZ, defaults.nearZ + 1.0f);
    }
}

} // namespace camera
} // namespace engine
