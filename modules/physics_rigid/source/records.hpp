// records.hpp - the module's plain-data runtime records: bodies, joints,
// persistent contact manifolds, and worlds. World owns dense body and joint
// arrays plus handle-to-index maps; handles are monotonic and never reused.

#pragma once

#include "shapes.hpp"
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/util/step_accumulator.hpp>

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace engine::physics_rigid
{

// One rigid body record. Static and kinematic bodies carry zero inverse mass
// and inertia; only dynamic bodies integrate forces.
struct Body
{
    BodyHandle handle = invalidBody;

    Shape    shape;
    BodyKind kind = BodyKind::Dynamic;

    RigidTransform transform;

    math::Vec3 linearVelocity;
    math::Vec3 angularVelocity;

    math::Vec3 forceAccumulator;
    math::Vec3 torqueAccumulator;

    MaterialDescription material;

    double     mass                = 0.0;
    double     inverseMass         = 0.0;
    math::Vec3 inertiaLocal        = {}; // principal-axis inertia in the body frame
    math::Vec3 inverseInertiaLocal = {};

    // Effective rolling radius for the rolling-resistance contact rows: the
    // sphere radius, the capsule or cylinder radius, or 0 for shapes that
    // cannot roll (boxes, planes). Derived from the shape at creation.
    double rollingRadius = 0.0;

    // Continuous collision detection switch (mirrors BodyDescription; only
    // dynamic bodies sweep).
    bool continuousCollisionEnabled = false;

    // Mass source remembered for recomputation after kind changes: density
    // above zero wins over the explicit mass (mirrors BodyDescription).
    double densitySource      = 0.0;
    double explicitMassSource = 0.0;

    double linearDamping  = 0.0;
    double angularDamping = 0.0;

    bool sleepingEnabled = true;
    bool gravityEnabled  = true;
    bool awake           = true;

    double sleepTimer = 0.0;

    std::uint32_t categoryBits = 1u;
    std::uint32_t maskBits     = 0xFFFFFFFFu;

    std::uint64_t userTag = 0;

    math::Vec3 aabbMinimum;
    math::Vec3 aabbMaximum;
};

// One joint record. The stored description keeps the caller's configuration;
// the reference rotation captures the relative body frame at creation so
// limits and twist angles are measured against the assembly pose.
struct Joint
{
    JointHandle handle = invalidJoint;

    JointDescription description;

    math::Vec3 axisLocalA = {}; // normalized hinge axis in body A's frame
    double     restLength = 0.0;

    // Relative rotation (qA^-1 * qB) captured at creation; limit angles are
    // offsets from this frame.
    math::Quaternion referenceRelativeRotation;

    // Warm-start storage per constraint row. Rows acting along different
    // axes keep separate accumulators so warm starting never re-applies one
    // row's impulse along another's axis.
    math::Vec3 ballSocketImpulse;       // three anchor rows
    double     antiSwingImpulse1 = 0.0; // hinge anti-swing rows
    double     antiSwingImpulse2 = 0.0;
    double     limitImpulse      = 0.0; // hinge limit row / ball-socket twist limit row
    double     swingLimitImpulse = 0.0; // ball-socket cone (swing) limit row
    double     motorImpulse      = 0.0; // hinge / prismatic motor row
    double     distanceImpulse   = 0.0; // distance joint row (also the spring's accumulator)

    // Prismatic translation limits and Fixed weld rows.
    double sliderLimitImpulse         = 0.0;              // prismatic translation limit row
    double sliderTranslationImpulse[2] = {0.0, 0.0};      // prismatic perpendicular anchor rows
    double prismaticTwistImpulse      = 0.0;              // prismatic twist-lock row
    double weldImpulses[3]            = {0.0, 0.0, 0.0};  // fixed joint angular rows
    double sliderRestDistance         = 0.0;              // prismatic anchor separation along the axis at creation
};

// One trigger volume: a non-solid shape region that reports overlap
// transitions for the world's solid bodies. Triggers never enter the
// broadphase or solver; they are evaluated once per step against body
// transforms, so their cost is one overlap test per (trigger, body) pair.
struct Trigger
{
    TriggerHandle handle = invalidTrigger;

    Shape shape;

    // Composed world pose (the description pose for unattached triggers, or
    // the attached body's transform composed with the local offset pose).
    RigidTransform transform;

    // Body-local offset pose applied when attachedBody is valid.
    RigidTransform localTransform;

    BodyHandle attachedBody = invalidBody;

    std::uint32_t categoryBits = 1u;
    std::uint32_t maskBits     = 0xFFFFFFFFu;

    bool          enabled = true;
    std::uint64_t userTag = 0;

    // Bodies overlapping as of the last evaluation, sorted ascending so the
    // enter/exit diff against the previous set is deterministic.
    std::vector<BodyHandle> overlapping;

    math::Vec3 aabbMinimum;
    math::Vec3 aabbMaximum;
};

// One world-space contact point carried between steps for warm starting.
struct PersistedContactPoint
{
    math::Vec3 anchorA;  // world contact point on body A at the last solve
    math::Vec3 anchorB;  // world contact point on body B at the last solve
    math::Vec3 tangent1; // friction basis persisted with the impulses
    math::Vec3 tangent2;
    double     normalImpulse   = 0.0;
    double     tangentImpulse1 = 0.0;
    double     tangentImpulse2 = 0.0;
};

// Persistent contact manifold keyed by an unordered body pair. Manifolds not
// refreshed by narrowphase are dropped after a few steps.
struct PersistedManifold
{
    std::uint64_t         pairKey    = 0;
    std::size_t           bodyIndexA = 0;
    std::size_t           bodyIndexB = 0;
    math::Vec3            normal; // from B toward A
    PersistedContactPoint points[4];
    std::uint32_t         pointCount = 0;
    std::uint32_t         ageSteps   = 0;
};

inline std::uint64_t makePairKey(std::uint32_t handleA, std::uint32_t handleB)
{
    return handleA < handleB ? (static_cast<std::uint64_t>(handleA) << 32) | handleB
                             : (static_cast<std::uint64_t>(handleB) << 32) | handleA;
}

// Solver and integration settings shared by every world; read once in
// construct() from the [physics_rigid] configuration section.
struct SimulationSettings
{
    double        fixedRateHz             = 60.0;
    std::uint32_t maximumStepsPerSimulate = 8;

    // Internal canonical substep target (seconds). A step whose delta exceeds
    // it is split into equal internal substeps, so callers at coarse rates
    // integrate on the same canonical step the fine rates use. Zero disables
    // splitting. maximumSubsteps bounds how many substeps one step may split
    // into; a delta whose required count exceeds the bound integrates at a
    // coarser canonical step instead of spending an unbounded pass count.
    double        substepMaximumDelta = 1.0 / 120.0;
    std::uint32_t maximumSubsteps    = 16;

    std::uint32_t velocityIterations = 20;

    // Contact position solver: penetration is corrected by moving transforms
    // directly (split impulse scheme), never by injecting bias velocities, so
    // stacking cannot bounce. The velocity solve only enforces the
    // restitution target; the position pass removes the remaining overlap.
    std::uint32_t positionIterations = 4;

    // Speculative contact margin (meters): shapes within this distance count
    // as touching, so resting contacts persist instead of detaching and
    // re-impacting every step. The velocity solve closes speculative gaps
    // within the step, which also removes impact-impulse overshoot.
    double contactMargin = 0.02;

    double contactSlop                  = 0.005;
    double positionCorrectionFactor     = 0.4;
    double maximumLinearCorrection      = 0.4;
    double restitutionVelocityThreshold = 0.3;

    // Joint rows keep Baumgarte bias velocities (joints are stiff enough that
    // the bias never injects visible energy).
    double jointBaumgarteFactor = 0.2;

    double maximumLinearVelocity  = 500.0;
    double maximumAngularVelocity = 200.0;

    double linearSleepTolerance  = 0.08;
    double angularSleepTolerance = 0.14;
    double timeToSleep           = 0.6;

    // Continuous collision detection: relative speed (meters per second)
    // above which a CCD-enabled dynamic body is swept against non-dynamic
    // geometry instead of relying on the speculative-contact margin alone.
    double ccdVelocityThreshold = 1.5;

    double spatialHashCellSize = 2.0;

    bool warmStarting = true;

    std::uint32_t maximumWorlds  = 16;
    std::uint32_t maximumBodies  = 100000;
    std::uint32_t maximumJoints  = 50000;
    std::uint32_t maximumTriggers = 256;

    // Pending overlap-event queue bound per world; the oldest events are
    // dropped with the overflow flag set once the bound is exceeded.
    std::uint32_t maximumTriggerEvents = 1024;

    math::Vec3 gravity = {0.0, -9.81, 0.0};
};

// One simulation world: bodies, joints, persistent manifolds, broadphase
// scratch, and the accumulate-and-step bookkeeping used by simulate().
struct World
{
    WorldHandle handle = invalidWorld;

    SimulationSettings settings; // copied from the module configuration at creation

    math::Vec3 gravity = {0.0, -9.81, 0.0};

    std::vector<Body>                           bodies;
    std::unordered_map<BodyHandle, std::size_t> bodyIndexByHandle;
    std::vector<BodyHandle>                     planeBodyHandles; // bodies whose shape is a plane

    std::vector<Joint>                           joints;
    std::unordered_map<JointHandle, std::size_t> jointIndexByHandle;

    std::vector<Trigger>                           triggers;
    std::unordered_map<TriggerHandle, std::size_t> triggerIndexByHandle;

    // Overlap transitions accumulated by evaluateTriggers; consumers drain
    // them through the capability's event entry points.
    std::vector<TriggerEvent> pendingTriggerEvents;
    bool                      triggerEventOverflow = false;

    std::unordered_map<std::uint64_t, PersistedManifold> manifolds;

    std::vector<std::pair<std::size_t, std::size_t>> broadphasePairs;

    // Fixed-step budget bookkeeping for simulate().
    engine::util::StepAccumulator accumulator;

    std::uint64_t stepsSimulated       = 0;
    double        lastStepMilliseconds = 0.0;
};

} // namespace engine::physics_rigid
