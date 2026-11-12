// One camera: pose, tuning, projection, and lazily rebuilt matrix caches.
//
// The record owns its invariants. Orientation lives as yaw/pitch in the
// reference frame derived from the up vector; in Orbit mode the eye rides the
// orbit sphere around the target at the current radius; view, projection,
// combined view-projection, and inverse matrices rebuild only when their
// inputs change. Mutating entry points validate their inputs and leave the
// record untouched when the input is unusable.

#ifndef ENGINE_CAMERA_RECORD_HPP
#define ENGINE_CAMERA_RECORD_HPP

#include "camera_configuration.hpp"
#include "camera_orientation.hpp"
#include <engine/camera/capabilities.hpp>

#include <glm/glm.hpp>

#include <cstdint>

namespace engine
{
namespace camera
{

class CameraRecord
{
public:
    CameraRecord() = default;

    // Builds a record from the defaults, then applies every valid field from
    // parameters (vectors are adopted when flagged with has* or non-zero;
    // invalid scalars keep the default). A near-zero-length up vector falls
    // back to +Y, the initial yaw/pitch looks from position at target, the
    // projection values are coerced into their hard limits, and an Orbit-mode
    // record resolves its radius (explicit parameter, else the
    // |position - target| distance when a pose was given, else the default)
    // and places the eye on the orbit sphere.
    static CameraRecord fromParameters(const CameraParameters *parameters, const CameraDefaults &defaults);

    // Repositions from eye/target/up and re-derives yaw/pitch; the pose is
    // untouched for degenerate inputs (non-finite values, zero-length
    // direction or up, parallel up and direction). Returns whether the pose
    // changed.
    bool setLookAt(const glm::vec3 &eye, const glm::vec3 &target, const glm::vec3 &up);

    // Moves the orbit pivot. In Orbit mode the eye travels with the pivot
    // (yaw/pitch/radius are preserved); otherwise the target is stored for a
    // later switch to Orbit. Ignored for a non-finite target.
    void setOrbitTarget(const glm::vec3 &target);

    // Entering Orbit re-derives the orbit radius from the current pose;
    // other transitions keep the pose as is. Invalid modes are ignored.
    void setControlMode(CameraControlMode mode);

    // Sets the orbit radius and, in Orbit mode, re-places the eye along the
    // current view direction. Ignored for non-finite or non-positive values.
    void setOrbitDistance(float distance);

    // Tuning setters; non-finite or non-positive values (or values above the
    // field's bound) are ignored.
    void setMovementSpeed(float unitsPerSecond);
    void setLookSpeed(float degreesPerSecond);
    void setMouseSensitivity(float degreesPerPixel);

    // All-or-nothing: every field is validated before any is stored; pitch is
    // re-clamped and an orbit eye re-placed to match the new settings.
    // Returns false (nothing stored) when any field is out of range.
    bool setControlSettings(const CameraControlSettings &settings);

    // verticalScale is the vertical FOV in degrees for Perspective or the
    // orthographic height for Orthographic; non-finite or non-positive values
    // keep the current ones. Equal calls do not invalidate cached matrices.
    void setProjection(ProjectionType type, float verticalScale, float nearZ, float farZ);

    // Per-frame integration; input may be null, which counts as no input.
    // Axes are clamped to [-1, 1]; a non-finite or oversized deltaSeconds
    // (pause, debugger break) is clamped so the camera cannot jump. Orbit
    // re-places the eye on the orbit sphere after any rotation/zoom; Fly
    // moves along view/right/up and rotates from axis or mouse input. Fixed
    // ignores all input.
    void integrate(const CameraInputState *input, double deltaSeconds);

    // Fills outPose with the world-space eye position, view direction, up
    // vector, and strafe-right direction.
    void fillPose(CameraPose &outPose) const;

    // Returns the cached ViewState for the viewport size, rebuilding matrices
    // lazily when their inputs changed or the viewport differs from the
    // cached size; a zero viewport falls back to aspect 1.
    const ViewState &viewState(std::uint32_t viewportWidth, std::uint32_t viewportHeight) const;

    // Projects a world-space point to top-left-origin viewport pixel
    // coordinates; outDepth maps OpenGL NDC [-1, 1] to [0, 1]. Returns false
    // for a zero viewport, a non-finite point, or a point on/behind the
    // perspective eye plane.
    bool worldToViewport(std::uint32_t viewportWidth, std::uint32_t viewportHeight, const glm::vec3 &worldPosition,
        float &outX, float &outY, float &outDepth) const;

    // Builds a normalized world-space ray from a top-left-origin viewport
    // pixel coordinate. Perspective rays originate at the eye; orthographic
    // rays originate on the near plane. Returns false for a zero viewport,
    // non-finite pixel coordinates, or a degenerate ray.
    bool viewportToWorldRay(std::uint32_t viewportWidth, std::uint32_t viewportHeight, float viewportX, float viewportY,
        glm::vec3 &outOrigin, glm::vec3 &outDirection) const;

    [[nodiscard]] CameraControlMode controlMode() const noexcept
    {
        return mode_;
    }

private:
    // Rebuilds the cached view and/or projection matrices when dirty, then
    // the combined view-projection (and flags its inverse stale). Shared by
    // every matrix consumer so one record does the work once per change.
    void ensureMatrices(std::uint32_t viewportWidth, std::uint32_t viewportHeight) const;

    // Re-places the eye on the orbit sphere from the current
    // target/yaw/pitch/radius.
    void placeOrbitEye() noexcept;

    CameraControlMode mode_ = CameraControlMode::Fly;

    glm::vec3 target_        = glm::vec3(0.0f);
    float     orbitDistance_ = builtInOrbitDistance;

    glm::vec3 position_     = glm::vec3(0.0f, 2.0f, builtInOrbitDistance);
    float     yawDegrees_   = 0.0f;
    float     pitchDegrees_ = 20.0f;

    OrientationBasis basis_{};

    ProjectionType projectionType_     = ProjectionType::Perspective;
    float          fovYDegrees_        = builtInFovYDegrees;
    float          orthographicHeight_ = builtInOrthographicHeight;
    float          nearZ_              = builtInNearZ;
    float          farZ_               = builtInFarZ;

    float moveSpeed_         = builtInMoveSpeed;
    float lookSpeed_         = builtInLookSpeed;
    float mouseSensitivity_  = builtInMouseSensitivity;
    float orbitZoomSpeed_    = builtInMoveSpeed;
    float sprintMultiplier_  = builtInSprintMultiplier;
    float maxDeltaSeconds_   = builtInMaximumDeltaSeconds;
    float pitchLimitDegrees_ = builtInPitchLimitDegrees;

    // Caches rebuilt lazily by ensureMatrices; mutable so the const query
    // entry points can refresh them.
    mutable ViewState     cachedViewState_{};
    mutable glm::mat4     cachedView_{1.0f};
    mutable glm::mat4     cachedProjection_{1.0f};
    mutable glm::mat4     cachedViewProjection_{1.0f};
    mutable glm::mat4     cachedInverseViewProjection_{1.0f};
    mutable std::uint32_t cachedViewportWidth_  = 0;
    mutable std::uint32_t cachedViewportHeight_ = 0;
    mutable bool          viewDirty_            = true;
    mutable bool          projectionDirty_      = true;
    mutable bool          combinedDirty_        = true;
    mutable bool          inverseDirty_         = true;
};

} // namespace camera
} // namespace engine

#endif
