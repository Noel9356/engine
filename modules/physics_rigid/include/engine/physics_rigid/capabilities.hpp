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
//
// ABI history:
//   1 - initial surface (worlds, bodies, distance/ball-socket/hinge joints,
//       rayCast, collideSphere, statistics).
//   2 - appended MaterialDescription.rollingResistance,
//       BodyDescription.continuousCollisionEnabled, JointKind::Prismatic and
//       JointKind::Fixed, the Distance-joint spring fields, the SweepHit
//       record, and the setBodyContinuousCollision / setJointSpring /
//       sphereCast entry points. Everything from ABI 1 keeps its slot and
//       meaning; consumers check `abi >= 2` before touching the additions.
//   3 - appended BodyMassProperties with the copyBodyMassProperties entry
//       point, and the rayCastIgnoring entry point (ground probes that must
//       skip a controller's own rig). Everything from ABI 2 keeps its slot
//       and meaning; consumers check `abi >= 3` before touching the
//       additions.
//   4 - appended ShapeKind::Cylinder with its ShapeDescription fields, the
//       trigger-volume surface (TriggerDescription, TriggerEventKind,
//       TriggerEvent, the createTrigger / destroyTrigger /
//       copyTriggerDescription / setTriggerTransform / setTriggerEnabled /
//       triggerEventCount / copyTriggerEvents / clearTriggerEvents /
//       triggerOverlapCount / copyTriggerOverlaps entry points), and the
//       WorldStatistics trigger counters. Everything from ABI 3 keeps its
//       slot and meaning; consumers check `abi >= 4` before touching the
//       additions.
//   5 - appended the batched query surface: the SphereOverlapQuery /
//       SphereOverlapResult records with the collideSpheres entry point for
//       consumers that resolve many spheres per step (particle pools, cloth
//       and rope nodes, fluid particles), and the copyBodyStates /
//       copyBodyDescriptions entry points for consumers that drive many
//       bodies per step (force fields, buoyancy). Everything from ABI 4
//       keeps its slot and meaning; consumers check `abi >= 5` before
//       touching the additions.
inline constexpr std::uint32_t physicsRigidCapabilityAbi = 5u;

using WorldHandle  = std::uint32_t;
using BodyHandle   = std::uint32_t;
using JointHandle  = std::uint32_t;
using TriggerHandle = std::uint32_t;

inline constexpr WorldHandle   invalidWorld   = 0;
inline constexpr BodyHandle    invalidBody    = 0;
inline constexpr JointHandle   invalidJoint   = 0;
inline constexpr TriggerHandle invalidTrigger = 0;

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
// Static. Sphere, box, capsule, and cylinder shapes are defined around the
// body origin in the body's local frame (a capsule segment and a cylinder
// axis both run along local +Y).
enum class ShapeKind : std::uint32_t
{
    Sphere = 0,
    Box,
    Capsule,
    Plane,
    Cylinder
};

struct ShapeDescription
{
    ShapeKind kind = ShapeKind::Sphere;

    double sphereRadius = 0.5;

    double boxHalfExtents[3] = {0.5, 0.5, 0.5};

    double capsuleRadius     = 0.25;
    double capsuleHalfLength = 0.5; // half length of the inner cylinder segment along local +Y

    double cylinderRadius    = 0.5;
    double cylinderHalfHeight = 0.5; // half height of the cylinder along local +Y

    double planeNormal[3] = {0.0, 1.0, 0.0}; // world-space plane normal (normalized on entry)
    double planeOffset    = 0.0;             // world-space signed distance: x dot normal = offset
};

struct MaterialDescription
{
    double friction    = 0.6; // Coulomb coefficient, clamped to [0, 4]
    double restitution = 0.0; // bounce factor, clamped to [0, 1]

    // Rolling resistance coefficient (clamped to [0, 1]): the fraction of the
    // contact normal load converted into a torque that opposes rolling about
    // the contact tangents. Only curved shapes (spheres, capsules) roll, so
    // only their contacts spend this budget; flat-faced shapes ignore it.
    double rollingResistance = 0.0;
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

    // Continuous collision detection: when set, the body is swept from its
    // pre-integration to its post-integration transform every substep and
    // clamped at the first contact with non-dynamic geometry (static bodies,
    // kinematic bodies, planes), so fast movers cannot tunnel through thin
    // obstacles. Dynamic-vs-dynamic pairs stay on the speculative-contact
    // margin path.
    bool continuousCollisionEnabled = false;
};

// Runtime snapshot of one body: placement, velocities, and sleep state.
struct BodyState
{
    RigidTransform transform;

    Vec3 linearVelocity;
    Vec3 angularVelocity;

    bool awake = true;
};

// Mass properties of one dynamic body: total mass and the principal-axis
// inertia diagonal in the body's local frame (zero for static and kinematic
// bodies). Controllers derive per-joint gain bounds from these.
struct BodyMassProperties
{
    double mass = 0.0;
    Vec3  inertiaLocal = {}; // principal-axis diagonal, body frame
};

// Joint kinds. A Distance joint keeps the two world-space anchor points at a
// fixed distance, optionally through a frequency/damping spring instead of a
// hard constraint. A BallSocket keeps them coincident, optionally limited by a
// swing cone around the A-side axis and a twist range about it. A Hinge keeps
// them coincident and removes swing entirely (rotation only about the hinge
// axis), optionally with angle limits and a velocity motor. A Prismatic keeps
// them coincident along the plane perpendicular to the A-side axis and lets
// them slide only along that axis, with optional translation limits and a
// linear velocity motor. A Fixed welds the two bodies into one rigid assembly
// (all relative motion removed against the frame captured at creation).
enum class JointKind : std::uint32_t
{
    Distance = 0,
    BallSocket,
    Hinge,
    Prismatic,
    Fixed
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
    // toward motorTargetVelocity, never exceeding maximumMotorTorque. The
    // prismatic motor reuses the same fields as a linear drive: it tracks
    // motorTargetVelocity along axisLocalA (meters per second) within the
    // same torque budget interpreted as a maximum force.
    bool   motorEnabled        = false;
    double motorTargetVelocity = 0.0;
    double maximumMotorTorque  = 0.0;

    // Distance-joint spring: when enabled, the distance is maintained by a
    // frequency/damping spring (soft constraint) instead of a hard row.
    // springFrequencyHz is the target oscillation frequency and
    // springDampingRatio the dimensionless damping (0 = undamped, 1 = fully
    // damped, > 1 = overdamped); 0 frequency falls back to the hard row.
    bool   springEnabled       = false;
    double springFrequencyHz   = 0.0;
    double springDampingRatio  = 0.0;
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

    std::uint32_t triggerCount             = 0;
    std::uint32_t pendingTriggerEventCount = 0;
};

// Closest ray hit. The normal always faces the ray origin (opposes direction).
struct RayHit
{
    BodyHandle body = invalidBody;

    Vec3   point;
    Vec3   normal;
    double distance = 0.0;
};

// Closest shape-cast (sweep) hit for a sphere moved from `start` to
// `start + delta`. The normal faces back along the sweep (opposes delta); the
// distance is the travelled length at the first contact.
struct SweepHit
{
    BodyHandle body = invalidBody;

    Vec3   point;
    Vec3   normal;
    double distance = 0.0;
};

// A trigger volume: a non-solid shape region that reports which bodies enter
// or leave it. Triggers participate in no contact solving; they are evaluated
// once per step against the solid bodies (static, kinematic, and dynamic
// alike). A trigger is either fixed in world space or attached to a body and
// follows that body's transform every step.
struct TriggerDescription
{
    // Any solid shape (planes are rejected); defined around the trigger
    // origin in the trigger's local frame.
    ShapeDescription shape;

    // World placement for unattached triggers (normalized on entry).
    RigidTransform transform;

    // When set to a valid body, the trigger follows that body every step and
    // `transform` above is interpreted as the body-local offset pose;
    // `copyTriggerDescription` echoes it through `transform` regardless.
    BodyHandle attachedBody = invalidBody;

    // Collision filter with the same two-way rule as bodies:
    // (trigger.categoryBits & body.maskBits) != 0 and
    // (body.categoryBits & trigger.maskBits) != 0.
    std::uint32_t categoryBits = 1u;
    std::uint32_t maskBits     = 0xFFFFFFFFu;

    bool enabled = true;

    // Opaque caller value echoed by copyTriggerDescription.
    std::uint64_t userTag = 0;
};

// Overlap transition kinds reported for one trigger/body pair. Enter fires
// when the pair starts overlapping between two consecutive evaluations; Exit
// fires when it stops overlapping (also when either side is destroyed while
// overlapping).
enum class TriggerEventKind : std::uint32_t
{
    Enter = 0,
    Exit
};

// One trigger/body overlap transition. Events accumulate in a per-world
// queue; consumers poll them with triggerEventCount / copyTriggerEvents and
// drain with clearTriggerEvents.
struct TriggerEvent
{
    TriggerEventKind kind = TriggerEventKind::Enter;

    TriggerHandle trigger = invalidTrigger;
    BodyHandle    body    = invalidBody;
};

// One sphere for the batched overlap query below.
struct SphereOverlapQuery
{
    double center[3] = {0.0, 0.0, 0.0};
    double radius    = 0.0;
};

// One batched overlap result, written in the input order of the queries.
// The normal is the unit direction to move the sphere out of the obstacle
// and the depth is the penetration distance, exactly as collideSphere
// reports them for a single sphere.
struct SphereOverlapResult
{
    bool       hit       = false;
    double     normal[3] = {0.0, 0.0, 0.0};
    double     depth     = 0.0;
    BodyHandle body      = invalidBody;
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
    bool (*copyBodyMassProperties)(
        void *self, WorldHandle world, BodyHandle body, BodyMassProperties *outMass);

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

    // Closest ray hit that skips the listed bodies (a character controller
    // probing the ground under its own rig); null/zero list casts against
    // everything.
    bool (*rayCastIgnoring)(void *self, WorldHandle world, const double origin[3], const double direction[3],
        double maximumDistance, const BodyHandle *ignoredBodies, std::uint32_t ignoredBodyCount, RayHit *outHit);

    // Deepest-overlap sphere query against every body and plane. On hit,
    // writes the direction to move the sphere out of the obstacle (unit
    // normal), the penetration depth, and the responsible body (planes report
    // the world's plane-carrying body handle or invalidBody when the plane
    // has none). Used by consumers that resolve their own particles against
    // rigid geometry.
    bool (*collideSphere)(void *self, WorldHandle world, const double center[3], double radius, double outNormal[3],
        double *outDepth, BodyHandle *outBody);

    bool (*worldStatistics)(void *self, WorldHandle world, WorldStatistics *outStatistics);

    // ---- ABI 2 additions ----------------------------------------------------

    // Enables or disables per-body continuous collision detection at runtime;
    // accepted for any body kind (only dynamic bodies sweep).
    bool (*setBodyContinuousCollision)(void *self, WorldHandle world, BodyHandle body, bool enabled);

    // Re-tunes a Distance joint's spring at runtime (the same validation as
    // creation applies; only Distance joints act on it).
    bool (*setJointSpring)(
        void *self, WorldHandle world, JointHandle joint, bool enabled, double frequencyHz, double dampingRatio);

    // Closest-hit sphere sweep against every body and plane: moves a sphere
    // of the given radius from start toward start + delta (delta may be any
    // finite vector) and reports the first contact. Radius must be positive.
    bool (*sphereCast)(void *self, WorldHandle world, const double start[3], const double delta[3], double radius,
        SweepHit *outHit);

    // ---- ABI 4 additions ----------------------------------------------------

    // Creates a trigger volume in the world (planes rejected; attached-body
    // handles are validated). Trigger handles are monotonic and never reused.
    TriggerHandle (*createTrigger)(void *self, WorldHandle world, const TriggerDescription *description, PhysicsError *error);
    void (*destroyTrigger)(void *self, WorldHandle world, TriggerHandle trigger);

    // Creation-time description echo (shape, placement or body-local offset
    // pose, filter bits, userTag).
    bool (*copyTriggerDescription)(void *self, WorldHandle world, TriggerHandle trigger, TriggerDescription *outDescription);

    // Moves an unattached trigger's world pose or an attached trigger's
    // body-local offset pose; takes effect from the next evaluation.
    bool (*setTriggerTransform)(void *self, WorldHandle world, TriggerHandle trigger, const RigidTransform *transform);
    bool (*setTriggerEnabled)(void *self, WorldHandle world, TriggerHandle trigger, bool enabled);

    // Pending overlap-event queue for the world. Events accumulate in
    // evaluation order (trigger creation order, then body order inside each
    // evaluation) and are never reordered; oldest events are dropped with the
    // overflow flag set when the configured event budget is exceeded.
    bool (*triggerEventCount)(void *self, WorldHandle world, std::uint32_t *outCount, bool *outOverflowed);
    bool (*copyTriggerEvents)(void *self, WorldHandle world, std::uint32_t firstIndex, TriggerEvent *outEvents,
        std::uint32_t capacity, std::uint32_t *outCopied);
    bool (*clearTriggerEvents)(void *self, WorldHandle world);

    // Bodies currently overlapping one trigger, ordered by body handle
    // ascending (static, kinematic, and dynamic bodies alike; sleeping
    // dynamics stay listed). copyTriggerOverlaps writes up to capacity
    // handles and reports the required count through outRequired.
    bool (*triggerOverlapCount)(void *self, WorldHandle world, TriggerHandle trigger, std::uint32_t *outCount);
    bool (*copyTriggerOverlaps)(void *self, WorldHandle world, TriggerHandle trigger, BodyHandle *outBodies,
        std::uint32_t capacity, std::uint32_t *outRequired);

    // ---- ABI 5 additions -------------------------------------------------------

    // Deepest-overlap sphere query for a whole batch in one call: each query
    // resolves exactly as collideSphere does for a single sphere, with the
    // world's solid bodies pre-filtered through their shape bounds before
    // the narrow-phase test. Null query or result arrays (with a non-zero
    // count) process zero queries; the return value is the number of
    // queries processed.
    std::uint32_t (*collideSpheres)(void *self, WorldHandle world, const SphereOverlapQuery *queries,
        std::uint32_t queryCount, SphereOverlapResult *outResults);

    // Batched body reads: one call per world instead of one call per body
    // for consumers that drive many bodies every step (force fields,
    // buoyancy). Entries for unknown handles stay default-constructed; the
    // return value reports how many entries resolved to a live body.
    std::uint32_t (*copyBodyStates)(void *self, WorldHandle world, const BodyHandle *bodies, std::uint32_t bodyCount,
        BodyState *outStates);
    std::uint32_t (*copyBodyDescriptions)(void *self, WorldHandle world, const BodyHandle *bodies,
        std::uint32_t bodyCount, BodyDescription *outDescriptions);
};

} // namespace engine::physics_rigid
