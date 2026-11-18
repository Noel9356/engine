// capabilities.hpp - public C capability surface of the physics_rigid module.
//
// Declares the plain-data description/state types plus the function-pointer
// struct consumers receive through ParticipantContext::acquireCapability for
// the "physics_rigid" capability. Handles are std::uint32_t, start at 1, are
// never reused, and 0 means "none". All positions, extents and velocities are
// in meters, seconds, and radians; vectors are right-handed with +Y up by
// convention (gravity points down in the default configuration).
//
// Frame-rate independence contract: advance a world through `step` exactly
// once per fixed update with a constant deltaSeconds, or drive it through
// `simulate`, which accumulates wall time and advances the world in whole
// fixed steps at the configured fixed rate. Never feed a per-frame variable
// delta into `step`; deterministic stepping is the module's core guarantee.

#pragma once

#include <cstdint>

namespace engine::physics_rigid
{

// Increment when any public capability structure or function-table layout
// changes; consumers verify it after acquisition.
inline constexpr std::uint32_t physicsRigidCapabilityAbi = 1u;

using WorldHandle = std::uint32_t;
using BodyHandle  = std::uint32_t;
using JointHandle = std::uint32_t;

inline constexpr WorldHandle invalidWorld = 0;
inline constexpr BodyHandle  invalidBody  = 0;
inline constexpr JointHandle invalidJoint = 0;

// Error codes written through optional PhysicsError out-parameters. Out
// parameters are left untouched on success paths that do not set them
// explicitly; treat boolean return values as authoritative first.
enum class PhysicsError : std::uint32_t
{
    None = 0,

    InvalidHandle,      // unknown world/body/joint handle
    InvalidDescription, // null pointers, non-finite values, negative extents, wrong shape fields
    CapacityExceeded,   // configured world/body/joint budget exhausted
    Unsupported,        // request valid in principle but not provided (e.g. plane bodies must be static)
    SimulationFailure   // non-finite state entered during a step; the world is left unmodified
};

// Body motion model. Static bodies never integrate and carry infinite mass;
// kinematic bodies integrate their velocities but ignore forces and contacts
// (they push dynamic bodies without being pushed); dynamic bodies respond to
// gravity, forces, impulses, and contacts.
enum class BodyKind : std::uint32_t
{
    Static = 0,
    Dynamic,
    Kinematic
};

// Collision shapes. A plane is a static half-space defined in world space
// (x dot normal = offset); its body transform is ignored and the body must be
// Static. Sphere, box, and capsule shapes are defined around the body origin
// in the body's local frame (a capsule segment runs along local +Y).
enum class ShapeKind : std::uint32_t
{
    Sphere = 0,
    Box,
    Capsule,
    Plane
};

struct ShapeDescription
{
    ShapeKind kind = ShapeKind::Sphere;

    double sphereRadius = 0.5;

    double boxHalfExtents[3] = {0.5, 0.5, 0.5};

    double capsuleRadius     = 0.25;
    double capsuleHalfLength = 0.5; // half length of the inner cylinder segment along local +Y

    double planeNormal[3] = {0.0, 1.0, 0.0}; // world-space plane normal (normalized on entry)
    double planeOffset    = 0.0;             // world-space signed distance: x dot normal = offset
};

struct MaterialDescription
{
    double friction    = 0.6; // Coulomb coefficient, clamped to [0, 4]
    double restitution = 0.0; // bounce factor, clamped to [0, 1]
};

// Double-precision vector; positions and directions in world or local space
// depending on the field that carries it.
struct Vec3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// x/y/z/w quaternion; normalized on entry wherever a rotation is stored.
struct Quaternion
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
};

// Body placement: translation plus rotation.
struct RigidTransform
{
    Vec3       position;
    Quaternion rotation;
};

struct BodyDescription
{
    ShapeDescription shape;

    RigidTransform transform;

    BodyKind kind = BodyKind::Dynamic;

    // Mass source: density > 0 computes mass and inertia from the shape
    // volume; otherwise the explicit mass (if positive) is used with a
    // shape-derived inertia. Values that produce a non-positive mass are
    // rejected with InvalidDescription.
    double density = 1000.0;
    double mass    = 0.0;

    MaterialDescription material;

    double linearDamping  = 0.0;
    double angularDamping = 0.0;

    bool sleepingEnabled = true;
    bool gravityEnabled  = true;

    // Collision filter: two bodies collide when
    // (a.categoryBits & b.maskBits) != 0 and (b.categoryBits & a.maskBits) != 0.
    std::uint32_t categoryBits = 1u;
    std::uint32_t maskBits     = 0xFFFFFFFFu;

    // Opaque caller value echoed by copyBodyDescription for application-side
    // bookkeeping (the module never interprets it).
    std::uint64_t userTag = 0;
};

// Runtime snapshot of one body: placement, velocities, and sleep state.
struct BodyState
{
    RigidTransform transform;

    Vec3 linearVelocity;
    Vec3 angularVelocity;

    bool awake = true;
};

// Joint kinds. A Distance joint keeps the two world-space anchor points at a
// fixed distance. A BallSocket keeps them coincident, optionally limited by a
// swing cone around the A-side axis and a twist range about it. A Hinge keeps
// them coincident and removes swing entirely (rotation only about the hinge
// axis), optionally with angle limits and a velocity motor.
enum class JointKind : std::uint32_t
{
    Distance = 0,
    BallSocket,
    Hinge
};

struct JointDescription
{
    JointKind kind = JointKind::Distance;

    BodyHandle bodyA = invalidBody;
    BodyHandle bodyB = invalidBody;

    // Anchor offsets in each body's local frame (world anchors derive from
    // the body transforms at creation; they are re-derived every step).
    Vec3 anchorLocalA;
    Vec3 anchorLocalB;

    // Distance: rest length; <= 0 adopts the current anchor separation.
    double distance = 0.0;

    // Hinge: rotation axis in body A's local frame (normalized on entry);
    // body B's rotation is constrained to spin about this shared axis.
    Vec3 axisLocalA = {0.0, 1.0, 0.0};

    // Angle limits. Hinge: fold range about the hinge axis in radians,
    // measured against the relative frame captured at creation, valid range
    // within (-pi, pi) with lower < upper. BallSocket: coneHalfAngleRadians
    // bounds the swing of the B-side anchor arm away from axisLocalA and
    // twistHalfAngleRadians bounds the rotation about it; both are measured
    // against the frames captured at creation.
    bool   limitsEnabled         = false;
    double limitLowerRadians     = 0.0;
    double limitUpperRadians     = 0.0;
    double coneHalfAngleRadians  = 3.141592653589793;
    double twistHalfAngleRadians = 3.141592653589793;

    // Hinge motor: drives the relative angular velocity about the hinge axis
    // toward motorTargetVelocity, never exceeding maximumMotorTorque.
    bool   motorEnabled        = false;
    double motorTargetVelocity = 0.0;
    double maximumMotorTorque  = 0.0;
};

struct WorldDescription
{
    Vec3 gravity = {0.0, -9.81, 0.0};

    // When set, the module's configured [physics_rigid] gravity overrides the
    // gravity field above.
    bool useConfiguredGravity = false;
};

struct WorldInfo
{
    std::uint32_t bodyCount  = 0;
    std::uint32_t jointCount = 0;
    std::uint32_t planeCount = 0;
};

// Cumulative counters since world creation plus the previous step's cost.
struct WorldStatistics
{
    std::uint32_t bodyCount          = 0;
    std::uint32_t staticBodyCount    = 0;
    std::uint32_t kinematicBodyCount = 0;
    std::uint32_t awakeBodyCount     = 0;
    std::uint32_t sleepingBodyCount  = 0;

    std::uint32_t jointCount        = 0;
    std::uint32_t contactPairCount  = 0;
    std::uint32_t contactPointCount = 0;

    std::uint64_t stepsSimulated       = 0;
    double        lastStepMilliseconds = 0.0;
};

// Closest ray hit. The normal always faces the ray origin (opposes direction).
struct RayHit
{
    BodyHandle body = invalidBody;

    Vec3   point;
    Vec3   normal;
    double distance = 0.0;
};

// The "physics_rigid" capability. Calls are main-thread only unless the
// consumer serializes them; the module takes no internal locks. `self` is the
// first argument of every call. Unknown handles are reported through the
// PhysicsError out-parameter where present and are otherwise ignored.
struct PhysicsRigidCapability
{
    static constexpr const char *capabilityName = "physics_rigid";

    std::uint32_t abi = physicsRigidCapabilityAbi;

    void *self = nullptr;

    WorldHandle (*createWorld)(void *self, const WorldDescription *description, PhysicsError *error);
    void (*destroyWorld)(void *self, WorldHandle world);

    bool (*setWorldGravity)(void *self, WorldHandle world, const double gravity[3]);
    bool (*copyWorldGravity)(void *self, WorldHandle world, double outGravity[3]);

    BodyHandle (*createBody)(void *self, WorldHandle world, const BodyDescription *description, PhysicsError *error);
    void (*destroyBody)(void *self, WorldHandle world, BodyHandle body);

    std::uint32_t (*bodyCount)(void *self, WorldHandle world);
    bool (*copyBodyHandles)(
        void *self, WorldHandle world, BodyHandle *outHandles, std::uint32_t capacity, std::uint32_t *outRequired);
    bool (*copyBodyDescription)(void *self, WorldHandle world, BodyHandle body, BodyDescription *outDescription);
    bool (*copyBodyState)(void *self, WorldHandle world, BodyHandle body, BodyState *outState);

    // Teleports a body and (optionally) wakes it. Zeroes velocities when the
    // body is asleep so a repositioned body does not keep stale momentum.
    bool (*setBodyTransform)(
        void *self, WorldHandle world, BodyHandle body, const RigidTransform *transform, bool wake);

    // Switches a body between Static, Dynamic, and Kinematic; mass properties
    // are recomputed and the body wakes.
    bool (*setBodyKind)(void *self, WorldHandle world, BodyHandle body, BodyKind kind);

    bool (*setBodyLinearVelocity)(void *self, WorldHandle world, BodyHandle body, const double velocity[3]);
    bool (*setBodyAngularVelocity)(void *self, WorldHandle world, BodyHandle body, const double velocity[3]);
    bool (*setBodyMaterial)(void *self, WorldHandle world, BodyHandle body, const MaterialDescription *material);
    bool (*setBodyAwake)(void *self, WorldHandle world, BodyHandle body, bool awake);

    // Force/impulse accumulation: forces persist for exactly one step and are
    // cleared afterwards; impulses are applied immediately. Non-finite values
    // are rejected.
    bool (*applyForceAtPoint)(
        void *self, WorldHandle world, BodyHandle body, const double force[3], const double worldPoint[3]);
    bool (*applyTorque)(void *self, WorldHandle world, BodyHandle body, const double torque[3]);
    bool (*applyImpulseAtPoint)(
        void *self, WorldHandle world, BodyHandle body, const double impulse[3], const double worldPoint[3]);
    bool (*applyAngularImpulse)(void *self, WorldHandle world, BodyHandle body, const double impulse[3]);

    JointHandle (*createJoint)(void *self, WorldHandle world, const JointDescription *description, PhysicsError *error);
    void (*destroyJoint)(void *self, WorldHandle world, JointHandle joint);
    bool (*copyJointDescription)(void *self, WorldHandle world, JointHandle joint, JointDescription *outDescription);

    // Enables or re-tunes a hinge motor; accepted for any joint kind (only
    // hinges act on it).
    bool (*setJointMotor)(
        void *self, WorldHandle world, JointHandle joint, bool enabled, double targetVelocity, double maximumTorque);

    // Advances the world by exactly one fixed step of fixedDeltaSeconds.
    // The delta must stay constant across a running simulation; values outside
    // (0, 0.1] are rejected. This is the deterministic stepping primitive.
    bool (*step)(void *self, WorldHandle world, double fixedDeltaSeconds);

    // Accumulator convenience: adds elapsedSeconds to the world's internal
    // time budget and runs whole fixed steps at the configured fixed rate,
    // returning the number of steps executed. Backlog beyond
    // maximum_steps_per_simulate is dropped. Elapsed values that are negative
    // or non-finite run zero steps.
    std::uint32_t (*simulate)(void *self, WorldHandle world, double elapsedSeconds);

    // Closest-hit ray cast against every body and plane. direction must be
    // non-zero (it is normalized internally); hits register when distance is
    // in (0, maximumDistance].
    bool (*rayCast)(void *self, WorldHandle world, const double origin[3], const double direction[3],
        double maximumDistance, RayHit *outHit);

    // Deepest-overlap sphere query against every body and plane. On hit,
    // writes the direction to move the sphere out of the obstacle (unit
    // normal), the penetration depth, and the responsible body (planes report
    // the world's plane-carrying body handle or invalidBody when the plane
    // has none). Used by consumers that resolve their own particles against
    // rigid geometry.
    bool (*collideSphere)(void *self, WorldHandle world, const double center[3], double radius, double outNormal[3],
        double *outDepth, BodyHandle *outBody);

    bool (*worldStatistics)(void *self, WorldHandle world, WorldStatistics *outStatistics);
};

} // namespace engine::physics_rigid
