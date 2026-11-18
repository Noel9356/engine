// capabilities.hpp - public C capability surface of the physics_character
// module.
//
// Character physics over a physics_rigid world and a physics_articulated
// figure: humanoid ragdoll construction (a standard ten-part template
// scaled to the character), physical animation (per-joint pose targets
// driven through the articulated PD drives), active ragdoll control (a
// mode switch and drive-strength blend between full pose tracking and free
// ragdoll, with an upright assist torque on the torso), secondary motion
// (jiggle chains: spring-damper bone chains excited by root acceleration
// and gravity), and procedural foot placement (a two-foot planter with
// ground ray casts, plant/stride logic, and a two-bone IK solve). Consumers
// acquire it via ParticipantContext::acquireCapability<CharacterCapability>();
// the module declares physics_rigid and physics_articulated as required
// capabilities.

#pragma once

#include <engine/physics_articulated/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>

#include <cstdint>

namespace engine::physics_character
{

// Increment when any public capability structure or function-table layout
// changes; consumers verify it after acquisition.
inline constexpr std::uint32_t physicsCharacterCapabilityAbi = 2u;

using CharacterHandle    = std::uint32_t;
using JiggleChainHandle  = std::uint32_t;
using FootPlanterHandle  = std::uint32_t;

inline constexpr CharacterHandle   invalidCharacter   = 0;
inline constexpr JiggleChainHandle invalidJiggleChain = 0;
inline constexpr FootPlanterHandle invalidFootPlanter = 0;

enum class CharacterError : std::uint32_t
{
    None = 0,

    InvalidHandle,      // unknown character, chain, or planter handle
    InvalidDescription, // null pointers, non-finite values, non-positive scales
    CapacityExceeded,   // configured character, chain, or planter budget exhausted
    Unsupported         // request not applicable (e.g. pose target on the root link)
};

// Standard humanoid ragdoll template parameters. The parts (in fixed link
// order: pelvis, torso, head, upper/left arm, lower/left arm, upper/right
// arm, lower/right arm, upper/left leg, lower/left leg, upper/right leg,
// lower/right leg) form an articulated figure with ball-socket shoulders
// and hips, hinge elbows and knees, and limited ball-socket joints for the
// neck and waist.
struct RagdollParameters
{
    // Root (pelvis) placement; the figure hangs around this point.
    double position[3] = {0.0, 1.0, 0.0};

    double rotation[4] = {0.0, 0.0, 0.0, 1.0}; // x,y,z,w; normalized on entry

    // Overall scale (1.0 = a 1.8 meter human).
    double scale = 1.0;

    // Total mass distributed across the parts by proportional weights.
    double totalMass = 75.0;

    double friction = 0.6;

    // When true the pelvis (root) is a static body: an anchored mannequin or
    // training dummy whose limbs the drives pose while the root holds the
    // placement. When false the whole ragdoll is dynamic.
    bool rootAnchored = false;

    bool sleepingEnabled = true;
};

struct RagdollInfo
{
    engine::physics_articulated::FigureHandle figure = engine::physics_articulated::invalidFigure;
    std::uint32_t                             partCount = 0;
};

// Control mode of an active ragdoll character. Kinematic tracks the pose
// targets at full strength; Blended scales every drive by the drive
// strength (a fall destabilizes into a ragdoll as the strength drains);
// Dynamic disables drives and the upright assist (a free ragdoll).
enum class CharacterMode : std::uint32_t
{
    Dynamic = 0,
    Blended,
    Kinematic
};

struct CharacterDescription
{
    // The physics_rigid world the figure lives in (upright assist torques
    // apply through it).
    engine::physics_rigid::WorldHandle world = engine::physics_rigid::invalidWorld;

    // The articulated figure backing the character (the humanoid ragdoll
    // template or any figure built by the consumer).
    engine::physics_articulated::FigureHandle figure = engine::physics_articulated::invalidFigure;

    // Link index whose body receives the upright assist torque.
    std::uint32_t torsoLinkIndex = 1;

    // Upright assist torque budget (newton-meters) at full strength.
    double uprightTorque = 220.0;
};

// One jiggle chain: a parent-relative bone stack with spring-damper angular
// offsets excited by root acceleration and gravity (secondary motion for
// hair, tails, pouches, antennas).
struct JiggleChainDescription
{
    // Bones in parent order; bone 0 hangs from the chain root.
    std::uint32_t boneCount = 4;

    // Rest offset of each bone in its parent's frame, packed x,y,z per bone
    // (boneCount entries).
    const double *restOffsets = nullptr;

    double stiffness = 90.0;   // spring pull toward the rest frame
    double damping   = 9.0;    // angular velocity damping

    // Fraction of gravity that pulls the bone tips down (0 = none).
    double gravityInfluence = 0.6;

    // Hard clamp on the per-bone angular offset (radians).
    double maximumAngleRadians = 0.7;
};

// One foot of a procedural foot planter.
struct FootPlanterDescription
{
    // Two-bone leg lengths (meters).
    double legUpperLength = 0.45;
    double legLowerLength = 0.45;

    // Hip anchor in the root's local frame (between the legs).
    double hipAnchor[3] = {0.0, 0.9, 0.0};

    // Foot rest offsets in the root's local frame, left then right.
    double footOffsets[2][3] = {{-0.15, 0.0, 0.1}, {0.15, 0.0, 0.1}};

    double footRadius = 0.06;

    // Vertical step the planter will climb without replanting to a new
    // target (meters); a taller ledge forces a replant.
    double maxStepHeight = 0.3;

    // Horizontal distance between the planted point and the desired point
    // that triggers a replant (meters).
    double strideThreshold = 0.45;

    // Ray cast start height above the desired point (meters).
    double rayCastUp = 1.0;

    // Forward prediction: the desired point leads the root by velocity
    // times this many seconds.
    double leadTime = 0.12;

    // Bodies the ground probes skip (the character's own rig, so a cast
    // through a leg reaches the ground instead of the limb). Null or zero
    // count probes against everything.
    const engine::physics_rigid::BodyHandle *ignoredBodies = nullptr;
    std::uint32_t                            ignoredBodyCount = 0;
};

// One foot state after an update: the planted world point (or the desired
// point while stepping), the ground normal, the knee direction (world,
// pointing forward from the hip toward the knee bend), the knee bend angle,
// and how far the hip would have to drop to keep the foot reachable.
struct FootState
{
    bool   planted = false;
    double footPosition[3]   = {0.0, 0.0, 0.0};
    double footNormal[3]     = {0.0, 1.0, 0.0};
    double kneeDirection[3]  = {0.0, 0.0, 1.0};
    double kneeBendRadians   = 0.0;
    double hipDropMeters     = 0.0;
};

struct CharacterStatistics
{
    std::uint32_t characterCount   = 0;
    std::uint32_t jiggleChainCount = 0;
    std::uint32_t footPlanterCount = 0;
};

// The "physics_character" capability. Calls are main-thread only; the
// module takes no internal locks and forwards body work to the rigid and
// articulated capabilities.
struct CharacterCapability
{
    static constexpr const char *capabilityName = "physics_character";

    std::uint32_t abi = physicsCharacterCapabilityAbi;

    void *self = nullptr;

    // ---- ragdoll construction ----------------------------------------------

    RagdollInfo (*createHumanoidRagdoll)(void *self, engine::physics_rigid::WorldHandle world,
        const RagdollParameters *parameters, CharacterError *error);

    // ---- active ragdoll / physical animation --------------------------------

    CharacterHandle (*createCharacter)(void *self, const CharacterDescription *description, CharacterError *error);
    void (*destroyCharacter)(void *self, CharacterHandle character);

    bool (*setCharacterMode)(void *self, CharacterHandle character, CharacterMode mode);
    bool (*setCharacterDriveStrength)(void *self, CharacterHandle character, double strength);

    // Physical animation: per-joint pose target with PD gains (the target
    // angle is measured against the assembly frame like the articulated
    // drives). linkIndex refers to the child link; the root reports
    // Unsupported.
    bool (*setCharacterJointTarget)(void *self, CharacterHandle character, std::uint32_t linkIndex,
        double targetAngleRadians);
    bool (*setCharacterJointGains)(void *self, CharacterHandle character, std::uint32_t linkIndex, double stiffness,
        double damping, double maximumTorque);

    // Per-fixed-update control pass: pushes the pose targets and drive
    // strengths into the articulated drives, applies the pose assist torque
    // pairs on the enabled ball-socket targets (the swing degrees of freedom
    // the twist drives do not cover), and applies the upright assist torque
    // to the torso body (Kinematic and Blended modes only).
    bool (*updateCharacter)(void *self, CharacterHandle character, double fixedDeltaSeconds);

    // ---- secondary motion ---------------------------------------------------

    JiggleChainHandle (*createJiggleChain)(void *self, const JiggleChainDescription *description, CharacterError *error);
    void (*destroyJiggleChain)(void *self, JiggleChainHandle chain);

    // Advances the chain one step from the root's current transform and
    // velocity. Call once per fixed update.
    bool (*updateJiggleChain)(void *self, JiggleChainHandle chain, const engine::physics_rigid::RigidTransform *root,
        const double rootVelocity[3], double fixedDeltaSeconds);

    // Bone world rotations: packed x,y,z,w per bone in chain order (the
    // parent-space spring offsets already composed with the rest frames).
    bool (*copyChainBoneRotations)(void *self, JiggleChainHandle chain, double *outRotations, std::uint32_t capacity,
        std::uint32_t *outRequired);

    // ---- procedural foot placement -------------------------------------------

    FootPlanterHandle (*createFootPlanter)(void *self, engine::physics_rigid::WorldHandle world,
        const FootPlanterDescription *description, CharacterError *error);
    void (*destroyFootPlanter)(void *self, FootPlanterHandle planter);

    // Per-fixed-update placement pass from the root transform and velocity.
    bool (*updateFootPlanter)(void *self, FootPlanterHandle planter, const engine::physics_rigid::RigidTransform *root,
        const double rootVelocity[3], double fixedDeltaSeconds);

    // Foot results: exactly two entries, left then right.
    bool (*copyFootStates)(void *self, FootPlanterHandle planter, FootState *outStates, std::uint32_t capacity,
        std::uint32_t *outRequired);

    bool (*characterStatistics)(void *self, CharacterStatistics *outStatistics);
};

} // namespace engine::physics_character
