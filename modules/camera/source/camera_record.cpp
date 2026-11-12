// CameraRecord implementation: creation from parameters, pose and projection
// mutation, per-frame integration, and the lazily rebuilt matrix caches.

#include "camera_record.hpp"
#include "camera_limits.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace engine
{
namespace camera
{
namespace
{

// Coerces projection values into hard limits: finite, FOV in [0.1, 179]
// degrees, near in [1e-4, 1e6], far above near (up to 1e9), ortho height in
// [1e-4, 1e8]. Non-finite inputs restore the built-in fallbacks.
void clampProjectionValues(float &fovYDegrees, float &nearZ, float &farZ, float &orthographicHeight) noexcept
{
    if (!std::isfinite(fovYDegrees))
    {
        fovYDegrees = builtInFovYDegrees;
    }
    if (!std::isfinite(nearZ))
    {
        nearZ = builtInNearZ;
    }
    if (!std::isfinite(farZ))
    {
        farZ = builtInFarZ;
    }
    if (!std::isfinite(orthographicHeight))
    {
        orthographicHeight = builtInOrthographicHeight;
    }
    fovYDegrees        = std::clamp(fovYDegrees, limits::minimumFovYDegrees, limits::maximumFovYDegrees);
    nearZ              = std::clamp(nearZ, limits::minimumNearZ, limits::maximumNearZ);
    farZ               = std::clamp(farZ, nearZ + limits::minimumFarAboveNearZ, limits::maximumFarZ);
    orthographicHeight = std::clamp(
        orthographicHeight, limits::minimumOrthographicHeight, limits::maximumOrthographicHeight);
}

} // namespace

CameraRecord CameraRecord::fromParameters(const CameraParameters *parameters, const CameraDefaults &defaults)
{
    CameraRecord record;
    record.mode_               = defaults.mode;
    record.target_             = glm::vec3(0.0f);
    record.position_           = glm::vec3(0.0f, 2.0f, defaults.orbitDistance);
    record.orbitDistance_      = defaults.orbitDistance;
    record.fovYDegrees_        = defaults.fovYDegrees;
    record.nearZ_              = defaults.nearZ;
    record.farZ_               = defaults.farZ;
    record.orthographicHeight_ = defaults.orthographicHeight;
    record.moveSpeed_          = defaults.moveSpeed;
    record.lookSpeed_          = defaults.lookSpeed;
    record.mouseSensitivity_   = defaults.mouseSensitivity;
    record.orbitZoomSpeed_     = defaults.orbitZoomSpeed;
    record.sprintMultiplier_   = defaults.sprintMultiplier;
    record.maxDeltaSeconds_    = defaults.maximumDeltaSeconds;
    record.pitchLimitDegrees_  = defaults.pitchLimitDegrees;

    glm::vec3 up(0.0f, 1.0f, 0.0f);

    if (parameters != nullptr)
    {
        // Vectors are adopted when flagged with has* or non-zero, and only
        // when finite; invalid vectors keep the default.
        const auto adoptVector
            = [](bool flagged, const float values[3], glm::vec3 &destination) noexcept
        {
            if (flagged || values[0] != 0.0f || values[1] != 0.0f || values[2] != 0.0f)
            {
                const glm::vec3 value = glm::make_vec3(values);
                if (finiteVector(value))
                {
                    destination = value;
                }
            }
        };
        adoptVector(parameters->hasTarget, parameters->target, record.target_);
        adoptVector(parameters->hasPosition, parameters->position, record.position_);
        adoptVector(parameters->hasUp, parameters->up, up);

        // Projection scalars are adopted when positive and finite; the hard
        // clamps below coerce them into range.
        if (std::isfinite(parameters->fovYDegrees) && parameters->fovYDegrees > 0.0f)
        {
            record.fovYDegrees_ = parameters->fovYDegrees;
        }
        if (std::isfinite(parameters->nearZ) && parameters->nearZ > 0.0f)
        {
            record.nearZ_ = parameters->nearZ;
        }
        if (std::isfinite(parameters->farZ) && parameters->farZ > 0.0f)
        {
            record.farZ_ = parameters->farZ;
        }
        if (parameters->projectionType == ProjectionType::Perspective
            || parameters->projectionType == ProjectionType::Orthographic)
        {
            record.projectionType_ = parameters->projectionType;
        }
        if (std::isfinite(parameters->orthographicHeight) && parameters->orthographicHeight > 0.0f)
        {
            record.orthographicHeight_ = parameters->orthographicHeight;
        }

        // Tuning scalars are adopted within the ranges setControlSettings
        // enforces; anything else keeps the default.
        limits::adoptPositiveFinite(parameters->moveSpeed, limits::maximumMoveSpeed, record.moveSpeed_);
        limits::adoptPositiveFinite(parameters->lookSpeed, limits::maximumLookSpeed, record.lookSpeed_);
        limits::adoptPositiveFinite(
            parameters->mouseSensitivity, limits::maximumMouseSensitivity, record.mouseSensitivity_);
        limits::adoptPositiveFinite(parameters->orbitZoomSpeed, limits::maximumOrbitZoomSpeed, record.orbitZoomSpeed_);
        limits::adoptPositiveFinite(
            parameters->sprintMultiplier, limits::maximumSprintMultiplier, record.sprintMultiplier_);
        limits::adoptPositiveFinite(parameters->maxDeltaSeconds, limits::maximumDeltaSeconds, record.maxDeltaSeconds_);
        if (std::isfinite(parameters->pitchLimitDegrees) && parameters->pitchLimitDegrees > 0.0f
            && parameters->pitchLimitDegrees < limits::pitchLimitUpperBoundDegrees)
        {
            record.pitchLimitDegrees_ = parameters->pitchLimitDegrees;
        }
    }

    record.basis_ = orientationBasisFromUp(up);
    clampProjectionValues(record.fovYDegrees_, record.nearZ_, record.farZ_, record.orthographicHeight_);

    // Seed yaw/pitch from the position -> target direction, when the two
    // points are distinct.
    const glm::vec3 eyeToTarget        = record.target_ - record.position_;
    const float     eyeToTargetLength2 = glm::dot(eyeToTarget, eyeToTarget);
    if (finiteVector(eyeToTarget) && std::isfinite(eyeToTargetLength2) && eyeToTargetLength2 > 1e-12f)
    {
        yawPitchFromDirection(eyeToTarget * glm::inversesqrt(eyeToTargetLength2),
            record.basis_,
            record.pitchLimitDegrees_,
            record.yawDegrees_,
            record.pitchDegrees_);
    }

    // Orbit radius: explicit parameter, else |position - target| when a pose
    // was given, else the configured default. The eye then rides the sphere.
    if (record.mode_ == CameraControlMode::Orbit)
    {
        const bool explicitOrbitDistance
            = parameters != nullptr
              && limits::adoptPositiveFinite(
                  parameters->orbitDistance, limits::maximumOrbitDistance, record.orbitDistance_);
        if (!explicitOrbitDistance)
        {
            const bool explicitPose
                = parameters != nullptr
                  && (parameters->hasPosition || parameters->hasTarget || parameters->position[0] != 0.0f
                      || parameters->position[1] != 0.0f || parameters->position[2] != 0.0f
                      || parameters->target[0] != 0.0f || parameters->target[1] != 0.0f
                      || parameters->target[2] != 0.0f);
            if (explicitPose)
            {
                const float distance = glm::length(record.position_ - record.target_);
                record.orbitDistance_ = std::isfinite(distance) && distance >= limits::minimumPoseDerivedOrbitDistance
                                           ? std::min(distance, limits::maximumOrbitDistance)
                                           : defaults.orbitDistance;
            }
            else
            {
                record.orbitDistance_ = defaults.orbitDistance;
            }
        }
        record.placeOrbitEye();
    }
    return record;
}

bool CameraRecord::setLookAt(const glm::vec3 &eye, const glm::vec3 &target, const glm::vec3 &up)
{
    // Degenerate inputs (NaNs, zero-length direction/up, parallel up and
    // direction) leave the pose untouched.
    const glm::vec3 direction        = target - eye;
    const float     directionLength2 = glm::dot(direction, direction);
    const float     upLength2        = glm::dot(up, up);
    const glm::vec3 upCrossDirection = glm::cross(direction, up);
    const float     basisLength2     = glm::dot(upCrossDirection, upCrossDirection);
    if (!finiteVector(eye) || !finiteVector(target) || !finiteVector(up) || directionLength2 <= 1e-12f
        || upLength2 <= 1e-12f || basisLength2 <= 1e-12f || !std::isfinite(directionLength2)
        || !std::isfinite(upLength2) || !std::isfinite(basisLength2))
    {
        return false;
    }

    position_     = eye;
    target_       = target;
    basis_        = orientationBasisFromUp(up * glm::inversesqrt(upLength2));
    yawPitchFromDirection(direction * glm::inversesqrt(directionLength2),
        basis_,
        pitchLimitDegrees_,
        yawDegrees_,
        pitchDegrees_);
    orbitDistance_ = std::min(std::sqrt(directionLength2), limits::maximumOrbitDistance);
    viewDirty_     = true;
    return true;
}

void CameraRecord::setOrbitTarget(const glm::vec3 &target)
{
    if (!finiteVector(target))
    {
        return;
    }
    target_ = target;
    if (mode_ == CameraControlMode::Orbit)
    {
        placeOrbitEye();
    }
}

void CameraRecord::setControlMode(CameraControlMode mode)
{
    if (mode != CameraControlMode::Fixed && mode != CameraControlMode::Orbit && mode != CameraControlMode::Fly)
    {
        return;
    }
    if (mode == mode_)
    {
        return;
    }
    // Entering Orbit re-derives the radius from the current pose; other
    // transitions keep the pose as is.
    if (mode == CameraControlMode::Orbit)
    {
        orbitDistance_ = std::clamp(glm::length(position_ - target_),
            limits::minimumPoseDerivedOrbitDistance,
            limits::maximumOrbitDistance);
    }
    mode_ = mode;
}

void CameraRecord::setOrbitDistance(float distance)
{
    if (!std::isfinite(distance) || distance <= 0.0f)
    {
        return;
    }
    // In Orbit mode the eye is re-placed at the new radius along the current
    // view direction.
    orbitDistance_ = std::min(distance, limits::maximumOrbitDistance);
    if (mode_ == CameraControlMode::Orbit)
    {
        placeOrbitEye();
    }
}

void CameraRecord::setMovementSpeed(float unitsPerSecond)
{
    limits::adoptPositiveFinite(unitsPerSecond, limits::maximumMoveSpeed, moveSpeed_);
}

void CameraRecord::setLookSpeed(float degreesPerSecond)
{
    limits::adoptPositiveFinite(degreesPerSecond, limits::maximumLookSpeed, lookSpeed_);
}

void CameraRecord::setMouseSensitivity(float degreesPerPixel)
{
    limits::adoptPositiveFinite(degreesPerPixel, limits::maximumMouseSensitivity, mouseSensitivity_);
}

bool CameraRecord::setControlSettings(const CameraControlSettings &settings)
{
    // Validate every field into next* locals; commit only when all of them
    // are in range.
    float nextMoveSpeed = moveSpeed_;
    if (!limits::adoptPositiveFinite(settings.moveSpeed, limits::maximumMoveSpeed, nextMoveSpeed))
    {
        return false;
    }
    float nextLookSpeed = lookSpeed_;
    if (!limits::adoptPositiveFinite(settings.lookSpeed, limits::maximumLookSpeed, nextLookSpeed))
    {
        return false;
    }
    float nextMouseSensitivity = mouseSensitivity_;
    if (!limits::adoptPositiveFinite(settings.mouseSensitivity, limits::maximumMouseSensitivity, nextMouseSensitivity))
    {
        return false;
    }
    float nextOrbitDistance = orbitDistance_;
    if (!limits::adoptPositiveFinite(settings.orbitDistance, limits::maximumOrbitDistance, nextOrbitDistance))
    {
        return false;
    }
    float nextOrbitZoomSpeed = orbitZoomSpeed_;
    if (!limits::adoptPositiveFinite(settings.orbitZoomSpeed, limits::maximumOrbitZoomSpeed, nextOrbitZoomSpeed))
    {
        return false;
    }
    float nextSprintMultiplier = sprintMultiplier_;
    if (!limits::adoptPositiveFinite(settings.sprintMultiplier, limits::maximumSprintMultiplier, nextSprintMultiplier))
    {
        return false;
    }
    float nextMaxDeltaSeconds = maxDeltaSeconds_;
    if (!limits::adoptPositiveFinite(settings.maxDeltaSeconds, limits::maximumDeltaSeconds, nextMaxDeltaSeconds))
    {
        return false;
    }
    if (!std::isfinite(settings.pitchLimitDegrees) || settings.pitchLimitDegrees <= 0.0f
        || settings.pitchLimitDegrees >= limits::pitchLimitUpperBoundDegrees)
    {
        return false;
    }
    const float nextPitchLimitDegrees = settings.pitchLimitDegrees;

    moveSpeed_         = nextMoveSpeed;
    lookSpeed_         = nextLookSpeed;
    mouseSensitivity_  = nextMouseSensitivity;
    orbitDistance_     = nextOrbitDistance;
    orbitZoomSpeed_    = nextOrbitZoomSpeed;
    sprintMultiplier_  = nextSprintMultiplier;
    maxDeltaSeconds_   = nextMaxDeltaSeconds;
    pitchLimitDegrees_ = nextPitchLimitDegrees;
    const float clampedPitch = std::clamp(pitchDegrees_, -pitchLimitDegrees_, pitchLimitDegrees_);
    if (clampedPitch != pitchDegrees_)
    {
        pitchDegrees_ = clampedPitch;
        viewDirty_    = true;
    }
    if (mode_ == CameraControlMode::Orbit)
    {
        placeOrbitEye();
    }
    return true;
}

void CameraRecord::setProjection(ProjectionType type, float verticalScale, float nearZ, float farZ)
{
    if (type != ProjectionType::Perspective && type != ProjectionType::Orthographic)
    {
        return;
    }

    // Validate into locals and commit only when something actually changed,
    // so equal calls do not invalidate the projection cache.
    float nextFovYDegrees        = fovYDegrees_;
    float nextOrthographicHeight = orthographicHeight_;
    float nextNearZ              = nearZ_;
    float nextFarZ               = farZ_;
    if (std::isfinite(verticalScale) && verticalScale > 0.0f)
    {
        if (type == ProjectionType::Perspective)
        {
            nextFovYDegrees = verticalScale;
        }
        else
        {
            nextOrthographicHeight = verticalScale;
        }
    }
    if (std::isfinite(nearZ) && nearZ > 0.0f)
    {
        nextNearZ = nearZ;
    }
    if (std::isfinite(farZ) && farZ > 0.0f)
    {
        nextFarZ = farZ;
    }

    clampProjectionValues(nextFovYDegrees, nextNearZ, nextFarZ, nextOrthographicHeight);
    if (type != projectionType_ || nextFovYDegrees != fovYDegrees_ || nextOrthographicHeight != orthographicHeight_
        || nextNearZ != nearZ_ || nextFarZ != farZ_)
    {
        projectionType_     = type;
        fovYDegrees_        = nextFovYDegrees;
        orthographicHeight_ = nextOrthographicHeight;
        nearZ_              = nextNearZ;
        farZ_               = nextFarZ;
        projectionDirty_    = true;
    }
}

void CameraRecord::integrate(const CameraInputState *input, double deltaSeconds)
{
    if (mode_ == CameraControlMode::Fixed)
    {
        return;
    }

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

    const glm::vec3 previousPosition = position_;
    const float     previousYaw      = yawDegrees_;
    const float     previousPitch    = pitchDegrees_;

    const double safeDelta = std::isfinite(deltaSeconds)
                                 ? std::clamp(deltaSeconds, 0.0, static_cast<double>(maxDeltaSeconds_))
                                 : 0.0;
    const float seconds      = static_cast<float>(safeDelta);
    const float lookRotation = lookSpeed_ * seconds;

    // Orbit: the move axes double as rotation input, zoom changes the radius
    // (hard floor 0.5); yaw wraps at 360 and pitch is clamped, then the eye
    // is kept on the orbit sphere.
    if (mode_ == CameraControlMode::Orbit)
    {
        const float orbitYaw   = axis(lookHorizontal + moveRight);
        const float orbitPitch = axis(lookVertical + moveForward);
        yawDegrees_ -= orbitYaw * lookRotation;
        pitchDegrees_ += orbitPitch * lookRotation;
        orbitDistance_ = std::clamp(orbitDistance_ - zoom * orbitZoomSpeed_ * seconds,
            limits::minimumOrbitZoomDistance,
            limits::maximumOrbitDistance);
        yawDegrees_   = std::remainder(yawDegrees_, 360.0f);
        pitchDegrees_ = std::clamp(pitchDegrees_, -pitchLimitDegrees_, pitchLimitDegrees_);
        placeOrbitEye();
    }
    else
    {
        yawDegrees_ -= lookHorizontal * lookRotation;
        pitchDegrees_ += lookVertical * lookRotation;
        yawDegrees_ -= mouseDeltaX * mouseSensitivity_;
        pitchDegrees_ -= mouseDeltaY * mouseSensitivity_;
        yawDegrees_   = std::remainder(yawDegrees_, 360.0f);
        pitchDegrees_ = std::clamp(pitchDegrees_, -pitchLimitDegrees_, pitchLimitDegrees_);

        if (moveForward != 0.0f || moveRight != 0.0f || moveUp != 0.0f)
        {
            const float moveDistance = moveSpeed_ * (sample.sprint ? sprintMultiplier_ : 1.0f) * seconds;
            glm::vec3   movement(0.0f);
            if (moveForward != 0.0f)
            {
                movement += directionFromYawPitch(yawDegrees_, pitchDegrees_, basis_) * moveForward;
            }
            if (moveRight != 0.0f)
            {
                movement += rightDirectionFromYaw(yawDegrees_, basis_) * moveRight;
            }
            if (moveUp != 0.0f)
            {
                movement += basis_.up * moveUp;
            }
            const float movementLength2 = glm::dot(movement, movement);
            if (movementLength2 > 1e-12f)
            {
                position_ += movement * (moveDistance * glm::inversesqrt(movementLength2));
            }
        }
    }

    if (position_ != previousPosition || yawDegrees_ != previousYaw || pitchDegrees_ != previousPitch)
    {
        viewDirty_ = true;
    }
}

void CameraRecord::fillPose(CameraPose &outPose) const
{
    const glm::vec3 forward = directionFromYawPitch(yawDegrees_, pitchDegrees_, basis_);
    const glm::vec3 right   = rightDirectionFromYaw(yawDegrees_, basis_);
    outPose.position[0]     = position_.x;
    outPose.position[1]     = position_.y;
    outPose.position[2]     = position_.z;
    outPose.forward[0]      = forward.x;
    outPose.forward[1]      = forward.y;
    outPose.forward[2]      = forward.z;
    outPose.up[0]           = basis_.up.x;
    outPose.up[1]           = basis_.up.y;
    outPose.up[2]           = basis_.up.z;
    outPose.right[0]        = right.x;
    outPose.right[1]        = right.y;
    outPose.right[2]        = right.z;
}

const ViewState &CameraRecord::viewState(std::uint32_t viewportWidth, std::uint32_t viewportHeight) const
{
    ensureMatrices(viewportWidth, viewportHeight);
    return cachedViewState_;
}

bool CameraRecord::worldToViewport(std::uint32_t viewportWidth, std::uint32_t viewportHeight,
    const glm::vec3 &worldPosition, float &outX, float &outY, float &outDepth) const
{
    if (viewportWidth == 0 || viewportHeight == 0 || !finiteVector(worldPosition))
    {
        return false;
    }
    ensureMatrices(viewportWidth, viewportHeight);
    const glm::vec4 clip = cachedViewProjection_ * glm::vec4(worldPosition, 1.0f);
    if (!std::isfinite(clip.x) || !std::isfinite(clip.y) || !std::isfinite(clip.z) || !std::isfinite(clip.w)
        || clip.w <= 1.0e-7f)
    {
        return false;
    }
    const glm::vec3 ndc   = glm::vec3(clip) / clip.w;
    const float     x     = (ndc.x + 1.0f) * 0.5f * static_cast<float>(viewportWidth);
    const float     y     = (1.0f - ndc.y) * 0.5f * static_cast<float>(viewportHeight);
    const float     depth = ndc.z * 0.5f + 0.5f;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(depth))
    {
        return false;
    }
    outX     = x;
    outY     = y;
    outDepth = depth;
    return true;
}

bool CameraRecord::viewportToWorldRay(std::uint32_t viewportWidth, std::uint32_t viewportHeight, float viewportX,
    float viewportY, glm::vec3 &outOrigin, glm::vec3 &outDirection) const
{
    if (viewportWidth == 0 || viewportHeight == 0 || !std::isfinite(viewportX) || !std::isfinite(viewportY))
    {
        return false;
    }
    ensureMatrices(viewportWidth, viewportHeight);
    // The inverse is cached alongside the matrices, so repeated picking does
    // not pay for a matrix inversion per query.
    if (inverseDirty_)
    {
        cachedInverseViewProjection_ = glm::inverse(cachedViewProjection_);
        inverseDirty_                = false;
    }
    const float ndcX = 2.0f * viewportX / static_cast<float>(viewportWidth) - 1.0f;
    const float ndcY = 1.0f - 2.0f * viewportY / static_cast<float>(viewportHeight);
    const glm::vec4 nearHomogeneous = cachedInverseViewProjection_ * glm::vec4(ndcX, ndcY, -1.0f, 1.0f);
    const glm::vec4 farHomogeneous  = cachedInverseViewProjection_ * glm::vec4(ndcX, ndcY, 1.0f, 1.0f);
    if (!std::isfinite(nearHomogeneous.w) || !std::isfinite(farHomogeneous.w)
        || std::abs(nearHomogeneous.w) <= 1.0e-7f || std::abs(farHomogeneous.w) <= 1.0e-7f)
    {
        return false;
    }
    const glm::vec3 nearPoint = glm::vec3(nearHomogeneous) / nearHomogeneous.w;
    const glm::vec3 farPoint  = glm::vec3(farHomogeneous) / farHomogeneous.w;
    const glm::vec3 origin    = projectionType_ == ProjectionType::Perspective ? position_ : nearPoint;
    const glm::vec3 delta
        = projectionType_ == ProjectionType::Perspective ? farPoint - origin : farPoint - nearPoint;
    const float lengthSquared = glm::dot(delta, delta);
    if (!finiteVector(origin) || !finiteVector(delta) || !std::isfinite(lengthSquared) || lengthSquared <= 1.0e-12f)
    {
        return false;
    }
    outOrigin    = origin;
    outDirection = delta * glm::inversesqrt(lengthSquared);
    return true;
}

void CameraRecord::ensureMatrices(std::uint32_t viewportWidth, std::uint32_t viewportHeight) const
{
    if (cachedViewportWidth_ != viewportWidth || cachedViewportHeight_ != viewportHeight)
    {
        cachedViewportWidth_  = viewportWidth;
        cachedViewportHeight_ = viewportHeight;
        projectionDirty_      = true;
    }
    if (viewDirty_)
    {
        cachedView_ = glm::lookAt(position_,
            position_ + directionFromYawPitch(yawDegrees_, pitchDegrees_, basis_),
            basis_.up);
        std::memcpy(cachedViewState_.view, glm::value_ptr(cachedView_), sizeof(cachedViewState_.view));
        cachedViewState_.position[0] = position_.x;
        cachedViewState_.position[1] = position_.y;
        cachedViewState_.position[2] = position_.z;
        viewDirty_                   = false;
        combinedDirty_               = true;
    }
    if (projectionDirty_)
    {
        const float aspect = viewportWidth != 0 && viewportHeight != 0
                                 ? static_cast<float>(viewportWidth) / static_cast<float>(viewportHeight)
                                 : 1.0f;
        cachedProjection_ = projectionType_ == ProjectionType::Orthographic
                                ? glm::ortho(-orthographicHeight_ * aspect * 0.5f,
                                      orthographicHeight_ * aspect * 0.5f,
                                      -orthographicHeight_ * 0.5f,
                                      orthographicHeight_ * 0.5f,
                                      nearZ_,
                                      farZ_)
                                : glm::perspective(glm::radians(fovYDegrees_), aspect, nearZ_, farZ_);
        std::memcpy(cachedViewState_.projection,
            glm::value_ptr(cachedProjection_),
            sizeof(cachedViewState_.projection));
        projectionDirty_ = false;
        combinedDirty_   = true;
    }
    if (combinedDirty_)
    {
        cachedViewProjection_ = cachedProjection_ * cachedView_;
        std::memcpy(cachedViewState_.viewProjection,
            glm::value_ptr(cachedViewProjection_),
            sizeof(cachedViewState_.viewProjection));
        combinedDirty_ = false;
        inverseDirty_  = true;
    }
}

void CameraRecord::placeOrbitEye() noexcept
{
    const glm::vec3 viewDirection = directionFromYawPitch(yawDegrees_, pitchDegrees_, basis_);
    const glm::vec3 placed        = target_ - viewDirection * orbitDistance_;
    if (placed != position_)
    {
        position_  = placed;
        viewDirty_ = true;
    }
}

} // namespace camera
} // namespace engine
