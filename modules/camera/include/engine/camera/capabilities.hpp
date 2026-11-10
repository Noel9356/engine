// Public capability surface of the camera module.
//
// Declares the POD types and function-pointer table consumers receive through
// ParticipantContext::acquireCapability for the "camera" capability (ABI 9):
// handle-based cameras with fixed, orbit, and fly control modes that produce
// view/projection matrices for a caller-supplied viewport size.

#ifndef ENGINE_CAMERA_CAPABILITIES_HPP
#define ENGINE_CAMERA_CAPABILITIES_HPP

#include <cstdint>

namespace engine
{
namespace camera
{

// ABI version of this capability struct; consumers must match it.
// ABI history: 9 added per-camera pitch/control limits; 10 added viewport
// projection and viewport-to-world ray queries.
inline constexpr std::uint32_t cameraCapabilityAbi = 10u;

// Opaque camera handle; 0 (invalidCamera) means "none". Handles start at 1.
using CameraHandle                          = std::uint32_t;
inline constexpr CameraHandle invalidCamera = 0;

// Status codes written through optional CameraError out-parameters. Only
// createCamera reports errors; the other calls fail silently on bad handles.
enum class CameraError : std::uint32_t
{
    None = 0,
    CreationFailed,
    InvalidHandle
};

// Control model: Fixed ignores all input, Orbit circles a target point at a
// set distance, Fly moves freely through the world.
enum class CameraControlMode : std::uint32_t
{
    Fixed = 0,
    Orbit,
    Fly
};

// Projection: Perspective is driven by fovYDegrees, Orthographic by
// orthographicHeight (the vertical half-extent of the view volume).
enum class ProjectionType : std::uint32_t
{
    Perspective = 0,
    Orthographic
};

// Per-frame input sample for update(). Movement/look axes are clamped to
// [-1, 1] by the module; mouse deltas are in pixels; sprint multiplies fly
// speed by the sprint multiplier.
struct CameraInputState
{
    float moveForward    = 0.0f;
    float moveRight      = 0.0f;
    float moveUp         = 0.0f;
    float lookHorizontal = 0.0f;
    float lookVertical   = 0.0f;
    float zoom           = 0.0f;
    float mouseDeltaX    = 0.0f;
    float mouseDeltaY    = 0.0f;
    bool  sprint         = false;
};

// Tuning for update(). setControlSettings validates every field (finite and
// positive, pitch limit below 90 degrees) and rejects the whole struct if any
// field is out of range.
struct CameraControlSettings
{
    float moveSpeed         = 4.0f;
    float lookSpeed         = 60.0f;
    float mouseSensitivity  = 0.11f;
    float orbitDistance     = 6.0f;
    float orbitZoomSpeed    = 4.0f;
    float sprintMultiplier  = 3.0f;
    float maxDeltaSeconds   = 0.25f;
    float pitchLimitDegrees = 89.0f;
};

// Creation-time parameters; zero/unset members fall back to module defaults
// (from the [camera] config section). The has* flags let a caller pass an
// explicit all-zero vector that would otherwise be treated as "unset".
struct CameraParameters
{
    float position[3] = {0.0f, 0.0f, 0.0f};

    float target[3] = {0.0f, 0.0f, -1.0f};

    float up[3] = {0.0f, 1.0f, 0.0f};

    // fovYDegrees is the vertical FOV for Perspective; orthographicHeight is
    // the vertical half-extent for Orthographic. Distances/planes in world
    // units; fovY is degrees.
    float fovYDegrees = 0.0f;

    float nearZ = 0.0f;
    float farZ  = 0.0f;

    ProjectionType projectionType = ProjectionType::Perspective;

    float orthographicHeight = 0.0f;

    // All fields default to 0 = "use the module default"; positive finite
    // values are adopted (within the same ranges setControlSettings enforces).
    float moveSpeed         = 0.0f;
    float lookSpeed         = 0.0f;
    float mouseSensitivity  = 0.0f;
    float orbitDistance     = 0.0f;
    float orbitZoomSpeed    = 0.0f;
    float sprintMultiplier  = 0.0f;
    float maxDeltaSeconds   = 0.0f;
    float pitchLimitDegrees = 0.0f;

    bool hasPosition = false;
    bool hasTarget   = false;
    bool hasUp       = false;
};

// Render-ready matrices: column-major 4x4 view and projection (16 floats
// each) plus the world-space eye position.
struct ViewState
{
    float view[16] = {};

    float projection[16] = {};

    float position[3] = {0.0f, 0.0f, 0.0f};
};

// Function-pointer table for the "camera" capability. Calls are not
// thread-safe (no internal locking); route them through one thread, typically
// the main/simulation thread.
struct CameraCapability
{
    static constexpr const char *capabilityName = "camera";

    std::uint32_t abi = cameraCapabilityAbi;

    void *self = nullptr;

    CameraHandle (*createCamera)(void *self, const CameraParameters *parameters, CameraError *error);
    void (*destroyCamera)(void *self, CameraHandle camera);

    // Repositions the camera from eye/target/up and re-derives yaw/pitch;
    // silently ignored for degenerate inputs (zero-length direction or up,
    // parallel up and direction, non-finite values).
    void (*setLookAt)(void *self, CameraHandle camera, const float eye[3], const float target[3], const float up[3]);

    // Entering Orbit from Fly re-derives the orbit radius from the current
    // pose; other transitions keep the pose as is.
    void (*setControlMode)(void *self, CameraHandle camera, CameraControlMode mode);

    void (*setOrbitDistance)(void *self, CameraHandle camera, float distance);

    void (*setMovementSpeed)(void *self, CameraHandle camera, float unitsPerSecond);
    void (*setLookSpeed)(void *self, CameraHandle camera, float degreesPerSecond);

    void (*setMouseSensitivity)(void *self, CameraHandle camera, float degreesPerPixel);

    bool (*setControlSettings)(void *self, CameraHandle camera, const CameraControlSettings *settings);

    // verticalScale is the vertical FOV in degrees for Perspective or the
    // orthographic height for Orthographic; invalid values keep the current
    // ones (a no-op does not invalidate cached matrices).
    void (*setProjection)(
        void *self, CameraHandle camera, ProjectionType type, float verticalScale, float nearZ, float farZ);

    // Advances the camera by deltaSeconds (clamped to maxDeltaSeconds); input
    // may be null, which counts as no input.
    void (*update)(void *self, CameraHandle camera, const CameraInputState *input, double deltaSeconds);

    // Fills outState with cached matrices, rebuilding them when dirty; safe
    // to call every frame with the current viewport size.
    void (*viewState)(void *self, CameraHandle camera, std::uint32_t viewportWidth, std::uint32_t viewportHeight,
        ViewState *outState);

    // Projects a world-space point to top-left-origin viewport pixel
    // coordinates. outDepth is mapped from OpenGL NDC [-1, 1] to [0, 1].
    // Returns false for an invalid camera, zero viewport, null pointers, or a
    // point on/behind the perspective eye plane. Coordinates may lie outside
    // the viewport when the point is outside the view frustum.
    bool (*worldToViewport)(void *self, CameraHandle camera, std::uint32_t viewportWidth,
        std::uint32_t viewportHeight, const float worldPosition[3], float *outX, float *outY, float *outDepth);

    // Builds a normalized world-space ray from a top-left-origin viewport
    // pixel coordinate. Perspective rays originate at the camera eye;
    // orthographic rays originate on the near plane.
    bool (*viewportToWorldRay)(void *self, CameraHandle camera, std::uint32_t viewportWidth,
        std::uint32_t viewportHeight, float viewportX, float viewportY, float outOrigin[3], float outDirection[3]);
};

} // namespace camera
} // namespace engine

#endif
