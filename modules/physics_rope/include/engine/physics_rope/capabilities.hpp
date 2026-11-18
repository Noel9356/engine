// capabilities.hpp - public C capability surface of the physics_rope module.
//
// XPBD rope and cable simulation: a chain of particles held together by
// stretch distance constraints and two-apart bending constraints, stepped
// with fixed-timestep substepping. Ropes collide against a physics_rigid
// world's bodies and planes through that module's sphere-overlap query, and
// either end can be pinned to a world position or attached to a rigid body's
// local anchor (cranes, cables, tow lines, chain links). Consumers acquire
// it via ParticipantContext::acquireCapability<RopeCapability>(); the module
// declares physics_rigid as a required capability.

#pragma once

#include <engine/physics_rigid/capabilities.hpp>

#include <cstdint>

namespace engine::physics_rope
{

// Increment when any public capability structure or function-table layout
// changes; consumers verify it after acquisition.
//
// ABI history:
//   1 - initial surface (rope creation and destruction, node state copies,
//       pins and body attachments, wind, fixed-step XPBD stepping,
//       statistics).
//   2 - appended RopeDescription.breakStrain,
//       RopeStatistics.brokenSegmentCount, RopeStatistics.maximumTension,
//       and the copyRopeTension / setRopeLength entry points. Everything
//       from ABI 1 keeps its slot and meaning; consumers check `abi >= 2`
//       before touching the additions.
inline constexpr std::uint32_t physicsRopeCapabilityAbi = 2u;

using RopeHandle = std::uint32_t;

inline constexpr RopeHandle invalidRope = 0;

enum class RopeError : std::uint32_t
{
    None = 0,

    InvalidHandle,      // unknown rope handle or node index
    InvalidDescription, // null pointers, non-finite values, degenerate chain
    CapacityExceeded,   // configured rope or node budget exhausted
    SimulationFailure   // non-finite state entered during a step; the rope is left unmodified
};

struct RopeDescription
{
    // Number of chain segments; nodes = segments + 1. At least 1.
    std::uint32_t segments = 16;

    // Total rest length (meters) split evenly across the segments.
    double restLength = 4.0;

    // Rope thickness: contact radius of one node against rigid geometry and
    // the visual thickness consumers render.
    double nodeRadius = 0.08;

    double nodeMass = 0.4;

    // XPBD compliances (inverse stiffness; 0 = perfectly rigid). Stretch
    // keeps the rope's length; bending (two-apart neighbors) controls how
    // easily the chain kinks (higher compliance = floppier cable).
    double stretchCompliance = 0.0;
    double bendingCompliance = 0.001;

    // Global velocity damping per second (0 = none).
    double damping = 0.05;

    // Tangential velocity kept per second while a node touches geometry
    // (0 = full stick, 1 = frictionless).
    double friction = 0.6;

    // Node collision against the rigid world (planes and solid bodies).
    bool collideWithWorld = true;

    // Placement: a straight chain from start to end, node 0 at `start`
    // extending toward `end`. When both ends are pinned, node 0 and the last
    // node hold those world positions until re-pinned or attached.
    double start[3] = {0.0, 4.0, 0.0};
    double end[3]   = {0.0, 0.0, 0.0};

    // End handling. A pinned end keeps its world position; an attached end
    // follows the attached body's local anchor each substep. Both false =
    // free rope (falls, drapes, collides).
    bool pinStart      = false;
    bool pinEnd        = false;
    bool attachStartToBody = false;
    bool attachEndToBody   = false;

    // Bodies + local anchor offsets for the attached ends (world anchor
    // derives from the body transform every substep).
    engine::physics_rigid::BodyHandle startBody = engine::physics_rigid::invalidBody;
    engine::physics_rigid::BodyHandle endBody   = engine::physics_rigid::invalidBody;
    double startLocalAnchor[3] = {0.0, 0.0, 0.0};
    double endLocalAnchor[3]   = {0.0, 0.0, 0.0};

    // Maximum tolerated elongation ratio of a segment beyond its rest length
    // before that segment's stretch constraint breaks permanently (stretch
    // only; compression never snaps; 0 = breaking disabled). Broken segments
    // stop solving and the rope continues as independent pieces sharing the
    // node pool; pins and body attachments are untouched.
    double breakStrain = 0.0;
};

struct RopeInfo
{
    std::uint32_t segments        = 0;
    std::uint32_t nodeCount       = 0;
    std::uint32_t constraintCount = 0;
};

struct RopeStatistics
{
    std::uint64_t stepsSimulated       = 0;
    std::uint64_t substepsExecuted     = 0;
    std::uint32_t lastContactCount     = 0;
    double        lastStepMilliseconds = 0.0;

    // ABI 2 additions: the cumulative number of segments broken permanently
    // by strain, and the largest per-segment tension (force units) of the
    // most recent completed step.
    std::uint32_t brokenSegmentCount = 0;
    double        maximumTension     = 0.0;
};

// The "physics_rope" capability. Calls are main-thread only; the module
// takes no internal locks. Positions and velocities are world-space meters
// and meters per second; all stepping happens in whole fixed steps with
// internal XPBD substeps, so results depend on the step count only.
struct RopeCapability
{
    static constexpr const char *capabilityName = "physics_rope";

    std::uint32_t abi = physicsRopeCapabilityAbi;

    void *self = nullptr;

    RopeHandle (*createRope)(
        void *self, engine::physics_rigid::WorldHandle world, const RopeDescription *description, RopeError *error);
    void (*destroyRope)(void *self, RopeHandle rope);

    bool (*ropeInfo)(void *self, RopeHandle rope, RopeInfo *outInfo);

    // Node state copies: positions and velocities are packed x,y,z per node
    // in chain order (node 0 first).
    bool (*copyNodePositions)(
        void *self, RopeHandle rope, double *outPositions, std::uint32_t capacity, std::uint32_t *outRequired);
    bool (*copyNodeVelocities)(
        void *self, RopeHandle rope, double *outVelocities, std::uint32_t capacity, std::uint32_t *outRequired);

    // Node placement control. setNodePinned pins a node at its current
    // position (or unpins it); pinNodeToPosition pins at an explicit world
    // point; attachNodeToBody follows the body's local anchor each substep
    // and detachNode releases any pin or attachment. Attached nodes carry no
    // mass (the rigid body drives them).
    bool (*setNodePinned)(void *self, RopeHandle rope, std::uint32_t nodeIndex, bool pinned);
    bool (*pinNodeToPosition)(void *self, RopeHandle rope, std::uint32_t nodeIndex, const double position[3]);
    bool (*attachNodeToBody)(void *self, RopeHandle rope, std::uint32_t nodeIndex,
        engine::physics_rigid::BodyHandle body, const double localAnchor[3]);
    bool (*detachNode)(void *self, RopeHandle rope, std::uint32_t nodeIndex);
    bool (*isNodeAttached)(void *self, RopeHandle rope, std::uint32_t nodeIndex, bool *outAttached);

    // Uniform wind acceleration (meters per second squared) added to every
    // free node's velocity each substep; consumers can bridge a wind field's
    // local sample here per step.
    bool (*setWindAcceleration)(void *self, RopeHandle rope, const double wind[3]);

    // Advances the rope by exactly one fixed step of fixedDeltaSeconds
    // (internal XPBD substeps per the module configuration). Keep the delta
    // constant across a running simulation.
    bool (*step)(void *self, RopeHandle rope, double fixedDeltaSeconds);

    // Accumulator convenience mirroring the rigid module's simulate: adds
    // elapsedSeconds and runs whole fixed steps at the configured fixed rate,
    // returning the number of steps executed.
    std::uint32_t (*simulate)(void *self, RopeHandle rope, double elapsedSeconds);

    bool (*ropeStatistics)(void *self, RopeHandle rope, RopeStatistics *outStatistics);

    // Per-segment tension readout (ABI 2): one value per segment (segments
    // = nodeCount - 1) for the most recent completed step, in force units —
    // tension_i is the step's summed per-substep |lambda| for segment i
    // divided by fixedDeltaSeconds^2 (lambda carries mass-times-length
    // units, so the division yields newtons). Zero before the first
    // completed step; broken segments stop accumulating once released. Same
    // copy-with-capacity idiom as copyNodePositions: outRequired receives
    // `segments`, and capacity below that returns false without copying.
    bool (*copyRopeTension)(
        void *self, RopeHandle rope, double *outTension, std::uint32_t capacity, std::uint32_t *outRequired);

    // Reeling (ABI 2): sets the rope's total rest length immediately — every
    // segment's rest length becomes length / segmentCount, and the two-apart
    // bend constraints follow at twice that (the relation creation builds).
    // Validates finite length > 0; nothing is woken or warmed (the module has
    // no sleep and XPBD has no warm start), nodes respond from the next step
    // on.
    bool (*setRopeLength)(void *self, RopeHandle rope, double length);
};

} // namespace engine::physics_rope
