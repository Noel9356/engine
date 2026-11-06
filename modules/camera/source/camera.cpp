// camera module implementation.
//
// Owns camera records behind the "camera" capability: fixed/orbit/fly control,
// yaw/pitch orientation derived from a reference frame around the up vector,
// and lazily cached lookAt / perspective-or-orthographic matrices built with
// glm. Defaults come from the [camera] config section and hot-reload when the
// kernel publishes ConfigurationReloadedEvent.

#include <engine/camera/capabilities.hpp>
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <new>
#include <optional>
#include <string>
#include <unordered_map>

namespace engine
{
namespace camera
{
namespace
{

// Built-in fallbacks used when [camera] configuration is missing or invalid.
float defaultFovYDegrees        = 55.0f;
float defaultNearZ              = 0.1f;
float defaultFarZ               = 100.0f;
float defaultOrthographicHeight = 10.0f;
float defaultMoveSpeed          = 4.0f;
float defaultLookSpeed          = 60.0f;

float defaultMouseSensitivity = 0.11f;

float defaultSprintMultiplier    = 3.0f;
float defaultMaximumDeltaSeconds = 0.25f;
float defaultPitchLimitDegrees   = 89.0f;

// Hard FOV clamp (in degrees) applied to every camera, whatever its source:
// configuration, createCamera parameters, or setProjection.
float minimumFovYDegrees = 0.1f;
float maximumFovYDegrees = 179.0f;

// One camera. Orientation is stored as yaw/pitch (degrees) in the reference
// frame derived from the up vector, so a changed up does not corrupt the pose.
// The mutable cache members are rebuilt lazily by viewState; orbitDistance is
// only meaningful in Orbit mode.
struct CameraRecord
{
    CameraHandle id = invalidCamera;

    CameraControlMode mode = CameraControlMode::Fly;

    glm::vec3 target        = glm::vec3(0.0f);
    float     orbitDistance = 6.0f; // radius around target in Orbit mode

    glm::vec3 position     = glm::vec3(0.0f, 0.0f, 6.0f);
    float     yawDegrees   = 0.0f;
    float     pitchDegrees = 20.0f;

    glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
    // Basis derived from up: forward = +Z projected onto the up plane.
    glm::vec3 referenceForward = glm::vec3(0.0f, 0.0f, 1.0f);
    glm::vec3 referenceRight   = glm::vec3(1.0f, 0.0f, 0.0f);

    ProjectionType projectionType     = ProjectionType::Perspective;
    float          fovYDegrees        = defaultFovYDegrees;
    float          orthographicHeight = 10.0f;
    float          nearZ              = defaultNearZ;
    float          farZ               = defaultFarZ;

    float moveSpeed         = defaultMoveSpeed;
    float lookSpeed         = defaultLookSpeed;
    float mouseSensitivity  = defaultMouseSensitivity;
    float orbitZoomSpeed    = defaultMoveSpeed;
    float sprintMultiplier  = defaultSprintMultiplier;
    float maxDeltaSeconds   = defaultMaximumDeltaSeconds;
    float pitchLimitDegrees = defaultPitchLimitDegrees;

    mutable ViewState     cachedViewState{};
    mutable glm::mat4     cachedView{1.0f};
    mutable glm::mat4     cachedProjection{1.0f};
    mutable std::uint32_t cachedViewportWidth  = 0;
    mutable std::uint32_t cachedViewportHeight = 0;
    mutable bool          viewDirty            = true;
    mutable bool          projectionDirty      = true;
};

// Builds a stable forward/right basis for an up vector: projects +Z onto the
// plane perpendicular to up, falling back to +X when up is (nearly) parallel
// to +Z, then normalizes both.
void referenceFrame(const glm::vec3 &up, glm::vec3 &forward, glm::vec3 &right)
{
    const glm::vec3 preferredForward(0.0f, 0.0f, 1.0f);
    glm::vec3       projected = preferredForward - up * glm::dot(preferredForward, up);
    if (glm::dot(projected, projected) <= 1e-8f)
    {
        const glm::vec3 fallback(1.0f, 0.0f, 0.0f);
        projected = fallback - up * glm::dot(fallback, up);
    }
    forward = glm::normalize(projected);
    right   = glm::normalize(glm::cross(up, forward));
}

void updateReferenceFrame(CameraRecord &record)
{
    referenceFrame(record.up, record.referenceForward, record.referenceRight);
}

// Direction <-> (yaw, pitch) conversion in the record's reference frame: yaw
// is the angle around up from referenceForward, pitch the elevation (clamped
// to the record's pitch limit).
void yawPitchFromDirection(
    const glm::vec3 &direction, const CameraRecord &record, float &outYawDegrees, float &outPitchDegrees)
{
    const float     vertical = std::clamp(glm::dot(direction, record.up), -1.0f, 1.0f);
    const glm::vec3 planar   = direction - record.up * vertical;
    outYawDegrees
        = glm::degrees(std::atan2(glm::dot(planar, record.referenceRight), glm::dot(planar, record.referenceForward)));
    outPitchDegrees
        = std::clamp(glm::degrees(std::asin(vertical)), -record.pitchLimitDegrees, record.pitchLimitDegrees);
}

glm::vec3 directionFromYawPitch(float yawDegrees, float pitchDegrees, const CameraRecord &record)
{
    const float yawRadians   = glm::radians(yawDegrees);
    const float pitchRadians = glm::radians(pitchDegrees);
    const float planarScale  = std::cos(pitchRadians);
    return record.referenceForward * (std::cos(yawRadians) * planarScale)
         + record.referenceRight * (std::sin(yawRadians) * planarScale) + record.up * std::sin(pitchRadians);
}

bool finiteVector(const glm::vec3 &value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

// Implements the camera capability. Not thread-safe: the camera map and the
// records (including their caches) are accessed without locks.
class CameraModule final : public ModuleInterface
{
public:
    CameraModule()
        : manifest_(makeManifest())
    {
        cameras_.reserve(16);
        assignCapabilityPointers();
    }

    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    void construct(ParticipantContext &context) override
    {
        // Reads [camera] defaults and subscribes for configuration hot-reload.
        context_ = &context;
        readConfiguration();
        reloadSubscription_ = context.eventBus()
                                  .channel<ConfigurationReloadedEvent>()
                                  .subscribe<CameraModule, &CameraModule::onConfigurationReloaded>(this);
    }

    void start() override
    {}

    void stop() override
    {
        cameras_.clear();
        if (reloadSubscription_ != 0 && context_ != nullptr)
        {
            context_->eventBus().channel<ConfigurationReloadedEvent>().unsubscribe(reloadSubscription_);
            reloadSubscription_ = 0;
        }
        context_ = nullptr;
    }

    void *capabilityPointerFor(const std::string &capabilityName) override
    {
        if (capabilityName == CameraCapability::capabilityName)
        {
            return &capability_;
        }
        return nullptr;
    }

private:
    // Loads the [camera] section into the default* fields. Values outside
    // their documented range are reported on stderr and keep the built-in
    // default; readPositive is the shared validator for keys that must be
    // finite and positive.
    void readConfiguration()
    {
        if (context_ == nullptr)
        {
            return;
        }
        ParticipantContext &context = *context_;
        defaultMode_                = CameraControlMode::Fly;
        defaultMoveSpeed_           = defaultMoveSpeed;
        defaultLookSpeed_           = defaultLookSpeed;
        defaultMouseSensitivity_    = defaultMouseSensitivity;
        defaultOrbitZoomSpeed_      = defaultMoveSpeed;
        defaultFovY_                = defaultFovYDegrees;
        defaultNearZ_               = defaultNearZ;
        defaultFarZ_                = defaultFarZ;
        defaultOrthographicHeight_  = defaultOrthographicHeight;
        defaultOrbitDistance_       = 6.0f;
        defaultSprintMultiplier_    = defaultSprintMultiplier;
        defaultMaximumDeltaSeconds_ = defaultMaximumDeltaSeconds;
        defaultPitchLimitDegrees_   = defaultPitchLimitDegrees;

        const std::optional<std::string> mode = context.configurationStringInSection("camera", "mode");
        if (mode.has_value())
        {
            if (*mode == "orbit")
            {
                defaultMode_ = CameraControlMode::Orbit;
            }
            else if (*mode == "fly")
            {
                defaultMode_ = CameraControlMode::Fly;
            }
            else if (*mode == "fixed")
            {
                defaultMode_ = CameraControlMode::Fixed;
            }
            else
            {
                std::fprintf(stderr, "camera module: unknown [camera] mode '%s'; using \"fly\"\n", mode->c_str());
            }
        }
        const std::optional<double> moveSpeed = context.configurationFloatingInSection("camera", "move_speed");
        if (moveSpeed.has_value() && std::isfinite(*moveSpeed) && *moveSpeed > 0.0 && *moveSpeed <= 1.0e6)
        {
            defaultMoveSpeed_ = static_cast<float>(*moveSpeed);
        }
        const std::optional<double> lookSpeed = context.configurationFloatingInSection("camera", "look_speed");
        if (lookSpeed.has_value() && std::isfinite(*lookSpeed) && *lookSpeed > 0.0 && *lookSpeed <= 3600.0)
        {
            defaultLookSpeed_ = static_cast<float>(*lookSpeed);
        }
        const std::optional<double> mouseSensitivity
            = context.configurationFloatingInSection("camera", "mouse_sensitivity");
        if (mouseSensitivity.has_value() && std::isfinite(*mouseSensitivity) && *mouseSensitivity > 0.0
            && *mouseSensitivity <= 100.0)
        {
            defaultMouseSensitivity_ = static_cast<float>(*mouseSensitivity);
        }
        const std::optional<double> fov = context.configurationFloatingInSection("camera", "fov_degrees");
        if (fov.has_value() && std::isfinite(*fov) && *fov > minimumFovYDegrees && *fov < maximumFovYDegrees)
        {
            defaultFovY_ = static_cast<float>(*fov);
        }
        const auto readPositive = [&context](const char *key, float &destination, double maximum)
        {
            const std::optional<double> value = context.configurationFloatingInSection("camera", key);
            if (value.has_value() && std::isfinite(*value) && *value > 0.0 && *value <= maximum)
            {
                destination = static_cast<float>(*value);
            }
            else if (value.has_value())
            {
                std::fprintf(stderr,
                    "camera module: ignoring [camera] %s; expected a finite value in (0, %.3g]\n",
                    key,
                    maximum);
            }
        };
        defaultOrbitZoomSpeed_ = defaultMoveSpeed_;
        readPositive("orbit_zoom_speed", defaultOrbitZoomSpeed_, 1.0e6);
        readPositive("near_z", defaultNearZ_, 1.0e6);
        readPositive("far_z", defaultFarZ_, 1.0e9);
        readPositive("orthographic_height", defaultOrthographicHeight_, 1.0e8);
        readPositive("orbit_distance", defaultOrbitDistance_, 1.0e8);
        readPositive("sprint_multiplier", defaultSprintMultiplier_, 100.0);
        readPositive("max_delta_seconds", defaultMaximumDeltaSeconds_, 5.0);

        const std::optional<double> pitchLimit
            = context.configurationFloatingInSection("camera", "pitch_limit_degrees");
        if (pitchLimit.has_value() && std::isfinite(*pitchLimit) && *pitchLimit > 0.0 && *pitchLimit < 90.0)
        {
            defaultPitchLimitDegrees_ = static_cast<float>(*pitchLimit);
        }
        else if (pitchLimit.has_value())
        {
            std::fprintf(stderr, "camera module: ignoring [camera] pitch_limit_degrees; expected (0, 90)\n");
        }
        // far must exceed near; otherwise restore the built-in far plane (or
        // near + 1 when the configured far is usable but too close).
        if (defaultFarZ_ <= defaultNearZ_)
        {
            std::fprintf(stderr, "camera module: [camera] far_z must exceed near_z; using the built-in far plane\n");
            defaultFarZ_ = std::max(defaultFarZ, defaultNearZ_ + 1.0f);
        }
    }

    // Re-reads defaults when any changed key starts with "camera."; cameras
    // that already exist keep their current settings.
    void onConfigurationReloaded(const ConfigurationReloadedEvent &event)
    {
        const bool changed = std::any_of(event.changedKeys.begin(),
            event.changedKeys.end(),
            [](const std::string &key) { return key.rfind("camera.", 0) == 0; });
        if (changed)
        {
            readConfiguration();
        }
    }

    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "camera",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"camera"},
            .requiredCapabilities = {}};
    }

    CameraRecord *recordFor(CameraHandle camera)
    {
        const auto found = cameras_.find(camera);
        return found != cameras_.end() ? &found->second : nullptr;
    }

    // Coerces projection values into hard limits: finite, FOV in
    // [0.1, 179] degrees, near in [1e-4, 1e6], far > near (up to 1e9),
    // ortho height in [1e-4, 1e8].
    static void clampProjection(CameraRecord &record)
    {
        if (!std::isfinite(record.fovYDegrees))
        {
            record.fovYDegrees = defaultFovYDegrees;
        }
        if (!std::isfinite(record.nearZ))
        {
            record.nearZ = defaultNearZ;
        }
        if (!std::isfinite(record.farZ))
        {
            record.farZ = defaultFarZ;
        }
        if (!std::isfinite(record.orthographicHeight))
        {
            record.orthographicHeight = 10.0f;
        }
        record.fovYDegrees        = std::clamp(record.fovYDegrees, minimumFovYDegrees, maximumFovYDegrees);
        record.nearZ              = std::clamp(record.nearZ, 1e-4f, 1e6f);
        record.farZ               = std::clamp(record.farZ, record.nearZ + 1e-3f, 1e9f);
        record.orthographicHeight = std::clamp(record.orthographicHeight, 1e-4f, 1e8f);
    }

    // Builds a camera from the defaults, then applies every valid field from
    // parameters (vectors are adopted when flagged or non-zero; invalid
    // scalars keep the default). The initial yaw/pitch looks from position at
    // target; the up vector is rejected when degenerate.
    CameraHandle createCameraImpl(const CameraParameters *parameters, CameraError *error)
    {
        CameraRecord record;
        record.id                 = nextCameraId_;
        record.mode               = defaultMode_;
        record.target             = glm::vec3(0.0f, 0.0f, 0.0f);
        record.position           = glm::vec3(0.0f, 2.0f, defaultOrbitDistance_);
        record.orbitDistance      = defaultOrbitDistance_;
        record.up                 = glm::vec3(0.0f, 1.0f, 0.0f);
        record.fovYDegrees        = defaultFovY_;
        record.nearZ              = defaultNearZ_;
        record.farZ               = defaultFarZ_;
        record.orthographicHeight = defaultOrthographicHeight_;
        record.moveSpeed          = defaultMoveSpeed_;
        record.lookSpeed          = defaultLookSpeed_;
        record.mouseSensitivity   = defaultMouseSensitivity_;
        record.orbitZoomSpeed     = defaultOrbitZoomSpeed_;
        record.sprintMultiplier   = defaultSprintMultiplier_;
        record.maxDeltaSeconds    = defaultMaximumDeltaSeconds_;
        record.pitchLimitDegrees  = defaultPitchLimitDegrees_;

        if (parameters != nullptr)
        {
            if (parameters->hasTarget || parameters->target[0] != 0.0f || parameters->target[1] != 0.0f
                || parameters->target[2] != 0.0f)
            {
                const glm::vec3 value = glm::make_vec3(parameters->target);
                if (finiteVector(value))
                    record.target = value;
            }
            if (parameters->hasPosition || parameters->position[0] != 0.0f || parameters->position[1] != 0.0f
                || parameters->position[2] != 0.0f)
            {
                const glm::vec3 value = glm::make_vec3(parameters->position);
                if (finiteVector(value))
                    record.position = value;
            }
            if (parameters->hasUp || parameters->up[0] != 0.0f || parameters->up[1] != 0.0f
                || parameters->up[2] != 0.0f)
            {
                const glm::vec3 value = glm::make_vec3(parameters->up);
                if (finiteVector(value))
                    record.up = value;
            }
            if (std::isfinite(parameters->fovYDegrees) && parameters->fovYDegrees > 0.0f)
            {
                record.fovYDegrees = parameters->fovYDegrees;
            }
            if (std::isfinite(parameters->nearZ) && parameters->nearZ > 0.0f)
            {
                record.nearZ = parameters->nearZ;
            }
            if (std::isfinite(parameters->farZ) && parameters->farZ > 0.0f)
            {
                record.farZ = parameters->farZ;
            }
            if (parameters->projectionType == ProjectionType::Perspective
                || parameters->projectionType == ProjectionType::Orthographic)
            {
                record.projectionType = parameters->projectionType;
            }
            if (std::isfinite(parameters->orthographicHeight) && parameters->orthographicHeight > 0.0f)
            {
                record.orthographicHeight = parameters->orthographicHeight;
            }
            if (std::isfinite(parameters->moveSpeed) && parameters->moveSpeed > 0.0f && parameters->moveSpeed <= 1.0e6f)
            {
                record.moveSpeed = parameters->moveSpeed;
            }
            if (std::isfinite(parameters->lookSpeed) && parameters->lookSpeed > 0.0f
                && parameters->lookSpeed <= 3600.0f)
            {
                record.lookSpeed = parameters->lookSpeed;
            }
            if (std::isfinite(parameters->mouseSensitivity) && parameters->mouseSensitivity > 0.0f
                && parameters->mouseSensitivity <= 100.0f)
            {
                record.mouseSensitivity = parameters->mouseSensitivity;
            }
            if (std::isfinite(parameters->orbitZoomSpeed) && parameters->orbitZoomSpeed > 0.0f
                && parameters->orbitZoomSpeed <= 1.0e6f)
            {
                record.orbitZoomSpeed = parameters->orbitZoomSpeed;
            }
            if (std::isfinite(parameters->sprintMultiplier) && parameters->sprintMultiplier > 0.0f
                && parameters->sprintMultiplier <= 100.0f)
            {
                record.sprintMultiplier = parameters->sprintMultiplier;
            }
            if (std::isfinite(parameters->maxDeltaSeconds) && parameters->maxDeltaSeconds > 0.0f
                && parameters->maxDeltaSeconds <= 5.0f)
            {
                record.maxDeltaSeconds = parameters->maxDeltaSeconds;
            }
            if (std::isfinite(parameters->pitchLimitDegrees) && parameters->pitchLimitDegrees > 0.0f
                && parameters->pitchLimitDegrees < 90.0f)
            {
                record.pitchLimitDegrees = parameters->pitchLimitDegrees;
            }
            if (std::isfinite(parameters->orbitDistance) && parameters->orbitDistance > 0.0f
                && parameters->orbitDistance <= 1.0e8f)
            {
                record.orbitDistance = parameters->orbitDistance;
            }
        }
        // Reject a near-zero-length up vector by restoring +Y.
        const float upLength2 = glm::dot(record.up, record.up);
        if (!std::isfinite(upLength2) || upLength2 <= 1e-12f)
        {
            record.up = glm::vec3(0.0f, 1.0f, 0.0f);
        }
        else
        {
            record.up *= glm::inversesqrt(upLength2);
        }
        updateReferenceFrame(record);
        clampProjection(record);

        // Seed yaw/pitch from the position -> target direction, when the two
        // points are distinct.
        const glm::vec3 eyeToTarget        = record.target - record.position;
        const float     eyeToTargetLength2 = glm::dot(eyeToTarget, eyeToTarget);
        if (finiteVector(eyeToTarget) && std::isfinite(eyeToTargetLength2) && eyeToTargetLength2 > 1e-12f)
        {
            yawPitchFromDirection(
                eyeToTarget * glm::inversesqrt(eyeToTargetLength2), record, record.yawDegrees, record.pitchDegrees);
        }

        // Orbit radius: explicit parameter, else |position - target| when a
        // pose was given, else the configured default.
        if (record.mode == CameraControlMode::Orbit)
        {
            const bool explicitOrbitDistance
                = parameters != nullptr && std::isfinite(parameters->orbitDistance) && parameters->orbitDistance > 0.0f;
            const bool explicitPose
                = parameters != nullptr
               && (parameters->hasPosition || parameters->hasTarget || parameters->position[0] != 0.0f
                   || parameters->position[1] != 0.0f || parameters->position[2] != 0.0f
                   || parameters->target[0] != 0.0f || parameters->target[1] != 0.0f || parameters->target[2] != 0.0f);
            if (explicitOrbitDistance)
            {
                record.orbitDistance = parameters->orbitDistance;
            }
            else if (explicitPose)
            {
                const float distance = glm::length(record.position - record.target);
                record.orbitDistance
                    = std::isfinite(distance) && distance >= 1e-4f ? std::min(distance, 1.0e8f) : defaultOrbitDistance_;
            }
            else
            {
                record.orbitDistance = defaultOrbitDistance_;
            }
        }

        // Ids start at 1; the sentinel 0 means the 32-bit counter wrapped.
        if (nextCameraId_ == invalidCamera)
        {
            reportError(error, CameraError::CreationFailed);
            return invalidCamera;
        }
        const CameraHandle id = nextCameraId_++;
        try
        {
            cameras_.emplace(id, record);
        }
        catch (const std::bad_alloc &)
        {
            reportError(error, CameraError::CreationFailed);
            return invalidCamera;
        }
        reportError(error, CameraError::None);
        return id;
    }

    void destroyCameraImpl(CameraHandle camera)
    {
        cameras_.erase(camera);
    }

    void setLookAtImpl(CameraHandle camera, const float *eye, const float *target, const float *up)
    {
        CameraRecord *record = recordFor(camera);
        if (record == nullptr || eye == nullptr || target == nullptr || up == nullptr)
        {
            return;
        }

        // Degenerate inputs (NaNs, zero-length direction/up, parallel up and
        // direction) are ignored rather than reported; the pose is untouched.
        const glm::vec3 newEye           = glm::make_vec3(eye);
        const glm::vec3 newTarget        = glm::make_vec3(target);
        const glm::vec3 newUp            = glm::make_vec3(up);
        const glm::vec3 direction        = newTarget - newEye;
        const float     directionLength2 = glm::dot(direction, direction);
        const float     upLength2        = glm::dot(newUp, newUp);
        const glm::vec3 upCrossDirection = glm::cross(direction, newUp);
        const float     basisLength2     = glm::dot(upCrossDirection, upCrossDirection);
        if (!finiteVector(newEye) || !finiteVector(newTarget) || !finiteVector(newUp) || directionLength2 <= 1e-12f
            || upLength2 <= 1e-12f || basisLength2 <= 1e-12f || !std::isfinite(directionLength2)
            || !std::isfinite(upLength2) || !std::isfinite(basisLength2))
        {
            return;
        }

        record->position = newEye;
        record->target   = newTarget;
        record->up       = newUp * glm::inversesqrt(upLength2);
        updateReferenceFrame(*record);
        yawPitchFromDirection(
            direction * glm::inversesqrt(directionLength2), *record, record->yawDegrees, record->pitchDegrees);
        record->orbitDistance = std::min(std::sqrt(directionLength2), 1.0e8f);
        record->viewDirty     = true;
    }

    void setControlModeImpl(CameraHandle camera, CameraControlMode mode)
    {
        CameraRecord *record = recordFor(camera);
        if (record == nullptr
            || (mode != CameraControlMode::Fixed && mode != CameraControlMode::Orbit && mode != CameraControlMode::Fly))
        {
            return;
        }

        if (mode == CameraControlMode::Fly && record->mode == CameraControlMode::Orbit)
        {
            // Leaving Orbit for Fly intentionally keeps the current pose.
        }
        // Entering Orbit from Fly re-derives the orbit radius from the pose.
        if (mode == CameraControlMode::Orbit && record->mode == CameraControlMode::Fly)
        {
            record->orbitDistance = std::clamp(glm::length(record->position - record->target), 1e-4f, 1.0e8f);
        }
        record->mode      = mode;
        record->viewDirty = true;
    }

    void setOrbitDistanceImpl(CameraHandle camera, float distance)
    {
        CameraRecord *record = recordFor(camera);
        if (record == nullptr || !std::isfinite(distance) || distance <= 0.0f)
        {
            return;
        }
        // In Orbit mode the eye is re-placed at the new radius along the
        // current view direction.
        record->orbitDistance = std::min(distance, 1.0e8f);
        if (record->mode == CameraControlMode::Orbit)
        {
            record->position
                = record->target
                - directionFromYawPitch(record->yawDegrees, record->pitchDegrees, *record) * record->orbitDistance;
        }
        record->viewDirty = true;
    }

    void setMovementSpeedImpl(CameraHandle camera, float unitsPerSecond)
    {
        CameraRecord *record = recordFor(camera);
        if (record == nullptr || !std::isfinite(unitsPerSecond) || unitsPerSecond <= 0.0f)
        {
            return;
        }
        record->moveSpeed = std::min(unitsPerSecond, 1.0e6f);
    }

    void setLookSpeedImpl(CameraHandle camera, float degreesPerSecond)
    {
        CameraRecord *record = recordFor(camera);
        if (record == nullptr || !std::isfinite(degreesPerSecond) || degreesPerSecond <= 0.0f)
        {
            return;
        }
        record->lookSpeed = std::min(degreesPerSecond, 3600.0f);
    }

    void setMouseSensitivityImpl(CameraHandle camera, float degreesPerPixel)
    {
        CameraRecord *record = recordFor(camera);
        if (record == nullptr || !std::isfinite(degreesPerPixel) || degreesPerPixel <= 0.0f)
        {
            return;
        }
        record->mouseSensitivity = std::min(degreesPerPixel, 100.0f);
    }

    void setProjectionImpl(CameraHandle camera, ProjectionType type, float verticalScale, float nearZ, float farZ)
    {
        CameraRecord *record = recordFor(camera);
        if (record == nullptr || (type != ProjectionType::Perspective && type != ProjectionType::Orthographic))
        {
            return;
        }

        // Apply on a copy and commit only when something actually changed, so
        // equal calls do not invalidate the projection cache.
        CameraRecord updated   = *record;
        updated.projectionType = type;
        if (std::isfinite(verticalScale) && verticalScale > 0.0f)
        {
            if (type == ProjectionType::Perspective)
            {
                updated.fovYDegrees = verticalScale;
            }
            else
            {
                updated.orthographicHeight = verticalScale;
            }
        }
        if (std::isfinite(nearZ) && nearZ > 0.0f)
        {
            updated.nearZ = nearZ;
        }
        if (std::isfinite(farZ) && farZ > 0.0f)
        {
            updated.farZ = farZ;
        }

        clampProjection(updated);
        if (updated.projectionType != record->projectionType || updated.fovYDegrees != record->fovYDegrees
            || updated.orthographicHeight != record->orthographicHeight || updated.nearZ != record->nearZ
            || updated.farZ != record->farZ)
        {
            record->projectionType     = updated.projectionType;
            record->fovYDegrees        = updated.fovYDegrees;
            record->orthographicHeight = updated.orthographicHeight;
            record->nearZ              = updated.nearZ;
            record->farZ               = updated.farZ;
            record->projectionDirty    = true;
        }
    }

    // All-or-nothing: every field is validated before any is stored; pitch is
    // re-clamped and an orbit eye re-placed to match the new settings.
    bool setControlSettingsImpl(CameraHandle camera, const CameraControlSettings *settings)
    {
        CameraRecord *record = recordFor(camera);
        if (record == nullptr || settings == nullptr || !std::isfinite(settings->moveSpeed)
            || settings->moveSpeed <= 0.0f || settings->moveSpeed > 1.0e6f || !std::isfinite(settings->lookSpeed)
            || settings->lookSpeed <= 0.0f || settings->lookSpeed > 3600.0f
            || !std::isfinite(settings->mouseSensitivity) || settings->mouseSensitivity <= 0.0f
            || settings->mouseSensitivity > 100.0f || !std::isfinite(settings->orbitDistance)
            || settings->orbitDistance <= 0.0f || settings->orbitDistance > 1.0e8f
            || !std::isfinite(settings->orbitZoomSpeed) || settings->orbitZoomSpeed <= 0.0f
            || settings->orbitZoomSpeed > 1.0e6f || !std::isfinite(settings->sprintMultiplier)
            || settings->sprintMultiplier <= 0.0f || settings->sprintMultiplier > 100.0f
            || !std::isfinite(settings->maxDeltaSeconds) || settings->maxDeltaSeconds <= 0.0f
            || settings->maxDeltaSeconds > 5.0f || !std::isfinite(settings->pitchLimitDegrees)
            || settings->pitchLimitDegrees <= 0.0f || settings->pitchLimitDegrees >= 90.0f)
        {
            return false;
        }
        record->moveSpeed         = settings->moveSpeed;
        record->lookSpeed         = settings->lookSpeed;
        record->mouseSensitivity  = settings->mouseSensitivity;
        record->orbitDistance     = settings->orbitDistance;
        record->orbitZoomSpeed    = settings->orbitZoomSpeed;
        record->sprintMultiplier  = settings->sprintMultiplier;
        record->maxDeltaSeconds   = settings->maxDeltaSeconds;
        record->pitchLimitDegrees = settings->pitchLimitDegrees;
        record->pitchDegrees = std::clamp(record->pitchDegrees, -record->pitchLimitDegrees, record->pitchLimitDegrees);
        if (record->mode == CameraControlMode::Orbit)
        {
            record->position
                = record->target
                - directionFromYawPitch(record->yawDegrees, record->pitchDegrees, *record) * record->orbitDistance;
        }
        record->viewDirty = true;
        return true;
    }

    // Per-frame integration. Axes are clamped to [-1, 1]; a non-finite or
    // oversized deltaSeconds (pause, debugger break) is clamped so the camera
    // cannot jump. Orbit re-places the eye on the orbit sphere after any
    // rotation/zoom; Fly moves along view/right/up and rotates from axis or
    // mouse input.
    void updateImpl(CameraHandle camera, const CameraInputState *input, double deltaSeconds)
    {
        CameraRecord *record = recordFor(camera);
        if (record == nullptr || record->mode == CameraControlMode::Fixed)
            return;

        const CameraInputState  empty{};
        const CameraInputState &sample = input != nullptr ? *input : empty;
        // Clamp an input axis to [-1, 1]; non-finite values count as 0.
        const auto axis
            = [](float value) noexcept { return std::isfinite(value) ? std::clamp(value, -1.0f, 1.0f) : 0.0f; };
        const float moveForward    = axis(sample.moveForward);
        const float moveRight      = axis(sample.moveRight);
        const float moveUp         = axis(sample.moveUp);
        const float lookHorizontal = axis(sample.lookHorizontal);
        const float lookVertical   = axis(sample.lookVertical);
        const float zoom           = axis(sample.zoom);
        // Mouse deltas are magnitude-clamped to 10000 px (divide, clamp,
        // multiply back) to survive garbage input without changing units.
        const float mouseDeltaX = axis(sample.mouseDeltaX / 10000.0f) * 10000.0f;
        const float mouseDeltaY = axis(sample.mouseDeltaY / 10000.0f) * 10000.0f;

        const glm::vec3 previousPosition      = record->position;
        const float     previousYaw           = record->yawDegrees;
        const float     previousPitch         = record->pitchDegrees;
        const float     previousOrbitDistance = record->orbitDistance;

        const double safeDelta    = std::isfinite(deltaSeconds)
                                      ? std::clamp(deltaSeconds, 0.0, static_cast<double>(record->maxDeltaSeconds))
                                      : 0.0;
        const float  seconds      = static_cast<float>(safeDelta);
        const float  lookRotation = record->lookSpeed * seconds;
        const float  moveDistance = record->moveSpeed * (sample.sprint ? record->sprintMultiplier : 1.0f) * seconds;

        // Orbit: the move axes double as rotation input, zoom changes the
        // radius (hard floor 0.5); yaw wraps at 360 and pitch is clamped.
        if (record->mode == CameraControlMode::Orbit)
        {
            const float orbitYaw   = axis(lookHorizontal + moveRight);
            const float orbitPitch = axis(lookVertical + moveForward);
            record->yawDegrees -= orbitYaw * lookRotation;
            record->pitchDegrees += orbitPitch * lookRotation;
            record->orbitDistance
                = std::clamp(record->orbitDistance - zoom * record->orbitZoomSpeed * seconds, 0.5f, 1.0e8f);
            record->yawDegrees = std::remainder(record->yawDegrees, 360.0f);
            record->pitchDegrees
                = std::clamp(record->pitchDegrees, -record->pitchLimitDegrees, record->pitchLimitDegrees);
        }
        else
        {
            record->yawDegrees -= lookHorizontal * lookRotation;
            record->pitchDegrees += lookVertical * lookRotation;
            record->yawDegrees -= mouseDeltaX * record->mouseSensitivity;
            record->pitchDegrees -= mouseDeltaY * record->mouseSensitivity;
            record->yawDegrees = std::remainder(record->yawDegrees, 360.0f);
            record->pitchDegrees
                = std::clamp(record->pitchDegrees, -record->pitchLimitDegrees, record->pitchLimitDegrees);

            if (moveForward != 0.0f || moveRight != 0.0f || moveUp != 0.0f)
            {
                glm::vec3 movement(0.0f);
                if (moveForward != 0.0f)
                    movement += directionFromYawPitch(record->yawDegrees, record->pitchDegrees, *record) * moveForward;
                if (moveRight != 0.0f)
                {
                    // Strafe right = forward basis rotated by yaw, so it stays
                    // perpendicular to the view.
                    const float     yawRadians     = glm::radians(record->yawDegrees);
                    const glm::vec3 rightDirection = record->referenceForward * std::sin(yawRadians)
                                                   - record->referenceRight * std::cos(yawRadians);
                    movement += rightDirection * moveRight;
                }
                if (moveUp != 0.0f)
                    movement += record->up * moveUp;
                const float movementLength2 = glm::dot(movement, movement);
                if (movementLength2 > 1e-12f)
                    record->position += movement * (moveDistance * glm::inversesqrt(movementLength2));
            }
        }

        // Keep the eye on the orbit sphere after any rotation or zoom.
        if (record->mode == CameraControlMode::Orbit
            && (record->yawDegrees != previousYaw || record->pitchDegrees != previousPitch
                || record->orbitDistance != previousOrbitDistance))
        {
            const glm::vec3 viewDirection = directionFromYawPitch(record->yawDegrees, record->pitchDegrees, *record);
            record->position              = record->target - viewDirection * record->orbitDistance;
        }
        if (record->position != previousPosition || record->yawDegrees != previousYaw
            || record->pitchDegrees != previousPitch || record->orbitDistance != previousOrbitDistance)
            record->viewDirty = true;
    }

    // Lazily rebuilds the cached view matrix (when the pose changed) and the
    // projection (when the type/fov/planes changed or the viewport differs
    // from the cached size); a zero viewport falls back to aspect 1.
    void viewStateImpl(
        CameraHandle camera, std::uint32_t viewportWidth, std::uint32_t viewportHeight, ViewState *outState) const
    {
        const auto found = cameras_.find(camera);
        if (found == cameras_.end() || outState == nullptr)
        {
            return;
        }
        CameraRecord &record = found->second;
        if (record.viewDirty)
        {
            record.cachedView = glm::lookAt(record.position,
                record.position + directionFromYawPitch(record.yawDegrees, record.pitchDegrees, record),
                record.up);
            std::memcpy(
                record.cachedViewState.view, glm::value_ptr(record.cachedView), sizeof(record.cachedViewState.view));
            record.cachedViewState.position[0] = record.position.x;
            record.cachedViewState.position[1] = record.position.y;
            record.cachedViewState.position[2] = record.position.z;
            record.viewDirty                   = false;
        }

        if (record.cachedViewportWidth != viewportWidth || record.cachedViewportHeight != viewportHeight)
        {
            record.cachedViewportWidth  = viewportWidth;
            record.cachedViewportHeight = viewportHeight;
            record.projectionDirty      = true;
        }
        if (record.projectionDirty)
        {
            const float aspect = viewportWidth != 0 && viewportHeight != 0
                                   ? static_cast<float>(viewportWidth) / static_cast<float>(viewportHeight)
                                   : 1.0f;
            record.cachedProjection
                = record.projectionType == ProjectionType::Orthographic
                    ? glm::ortho(-record.orthographicHeight * aspect * 0.5f,
                          record.orthographicHeight * aspect * 0.5f,
                          -record.orthographicHeight * 0.5f,
                          record.orthographicHeight * 0.5f,
                          record.nearZ,
                          record.farZ)
                    : glm::perspective(glm::radians(record.fovYDegrees), aspect, record.nearZ, record.farZ);
            std::memcpy(record.cachedViewState.projection,
                glm::value_ptr(record.cachedProjection),
                sizeof(record.cachedViewState.projection));
            record.projectionDirty = false;
        }
        *outState = record.cachedViewState;
    }

    static void reportError(CameraError *error, CameraError value)
    {
        if (error != nullptr)
        {
            *error = value;
        }
    }

    // Wires each capability entry to its Impl member through a forwarding
    // lambda; the void* self is this CameraModule instance.
    void assignCapabilityPointers()
    {
        capability_.self         = this;
        capability_.createCamera = [](void *self, const CameraParameters *parameters, CameraError *error)
        { return static_cast<CameraModule *>(self)->createCameraImpl(parameters, error); };
        capability_.destroyCamera
            = [](void *self, CameraHandle camera) { static_cast<CameraModule *>(self)->destroyCameraImpl(camera); };
        capability_.setLookAt
            = [](void *self, CameraHandle camera, const float eye[3], const float target[3], const float up[3])
        { static_cast<CameraModule *>(self)->setLookAtImpl(camera, eye, target, up); };
        capability_.setControlMode = [](void *self, CameraHandle camera, CameraControlMode mode)
        { static_cast<CameraModule *>(self)->setControlModeImpl(camera, mode); };
        capability_.setOrbitDistance = [](void *self, CameraHandle camera, float distance)
        { static_cast<CameraModule *>(self)->setOrbitDistanceImpl(camera, distance); };
        capability_.setMovementSpeed = [](void *self, CameraHandle camera, float unitsPerSecond)
        { static_cast<CameraModule *>(self)->setMovementSpeedImpl(camera, unitsPerSecond); };
        capability_.setLookSpeed = [](void *self, CameraHandle camera, float degreesPerSecond)
        { static_cast<CameraModule *>(self)->setLookSpeedImpl(camera, degreesPerSecond); };
        capability_.setMouseSensitivity = [](void *self, CameraHandle camera, float degreesPerPixel)
        { static_cast<CameraModule *>(self)->setMouseSensitivityImpl(camera, degreesPerPixel); };
        capability_.setControlSettings = [](void *self, CameraHandle camera, const CameraControlSettings *settings)
        { return static_cast<CameraModule *>(self)->setControlSettingsImpl(camera, settings); };
        capability_.setProjection
            = [](void *self, CameraHandle camera, ProjectionType type, float verticalScale, float nearZ, float farZ)
        { static_cast<CameraModule *>(self)->setProjectionImpl(camera, type, verticalScale, nearZ, farZ); };
        capability_.update = [](void *self, CameraHandle camera, const CameraInputState *input, double deltaSeconds)
        { static_cast<CameraModule *>(self)->updateImpl(camera, input, deltaSeconds); };
        capability_.viewState
            = [](void *self, CameraHandle camera, std::uint32_t width, std::uint32_t height, ViewState *outState)
        { static_cast<CameraModule *>(self)->viewStateImpl(camera, width, height, outState); };
    }

    ParticipantContext *context_            = nullptr;
    SubscriptionToken   reloadSubscription_ = 0;

    // Live cameras; mutable because viewStateImpl rebuilds per-record caches
    // on const access.
    mutable std::unordered_map<CameraHandle, CameraRecord> cameras_;
    // Ids start at 1; 0 is the invalid sentinel.
    CameraHandle nextCameraId_ = 1;

    // Defaults for new cameras; refreshed from [camera] config in
    // readConfiguration() (construct and hot-reload).
    CameraControlMode defaultMode_                = CameraControlMode::Fly;
    float             defaultMoveSpeed_           = defaultMoveSpeed;
    float             defaultLookSpeed_           = defaultLookSpeed;
    float             defaultMouseSensitivity_    = defaultMouseSensitivity;
    float             defaultOrbitZoomSpeed_      = defaultMoveSpeed;
    float             defaultFovY_                = defaultFovYDegrees;
    float             defaultNearZ_               = defaultNearZ;
    float             defaultFarZ_                = defaultFarZ;
    float             defaultOrthographicHeight_  = defaultOrthographicHeight;
    float             defaultOrbitDistance_       = 6.0f;
    float             defaultSprintMultiplier_    = defaultSprintMultiplier;
    float             defaultMaximumDeltaSeconds_ = defaultMaximumDeltaSeconds;
    float             defaultPitchLimitDegrees_   = defaultPitchLimitDegrees;

    ParticipantManifest manifest_;

    CameraCapability capability_{};
};

} // namespace
} // namespace camera

using camera::CameraModule;

} // namespace engine

// Module entry table: the one exported symbol the kernel loads. Declares the
// module name/version and the single provided "camera" capability; no required
// or optional capabilities, no declared permissions.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const providedCapabilities[] = {"camera"};

    static const engine::EngineModuleEntryTable entry = {
        .abiVersion              = engine::engineModuleAbiVersion,
        .moduleName              = "camera",
        .moduleVersion           = "0.1.0",
        .createModule            = []() -> engine::ModuleInterface            *{ return new engine::camera::CameraModule(); },
        .providedCapabilities    = providedCapabilities,
        .providedCapabilityCount = 1,
        .requiredCapabilities    = nullptr,
        .requiredCapabilityCount = 0,
        .optionalCapabilities    = nullptr,
        .optionalCapabilityCount = 0,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0,
    };
    return &entry;
}
