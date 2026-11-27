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
// and gravity), procedural foot placement (a two-foot planter with
// ground ray casts, plant/stride logic, and a two-bone IK solve), hand
// placement (a two-hand planter with two-bone arm IK toward consumer
// targets), a procedural lean controller, and a push helper that turns the
// character's motion into impulses on nearby dynamic bodies. Consumers
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
//
// ABI history:
//   1 - initial surface (humanoid ragdoll construction, active ragdoll
//       characters, jiggle chains, the foot planter, statistics).
//   2 - appended per-joint gains through setCharacterJointGains and the
//       reworked update pass (pose assist torque pairs on the ball-socket
//       targets). Everything from ABI 1 keeps its slot and meaning.
//   3 - appended the hand planter (HandPlanterDescription, HandState, the
//       createHandPlanter / destroyHandPlanter / setHandTarget /
//       updateHandPlanter / copyHandStates entry points), the lean
//       controller (LeanControllerDescription, LeanState, the
//       createLeanController / destroyLeanController /
//       updateLeanController entry points), and the applyCharacterPush
//       entry point. Everything from ABI 2 keeps its slot and meaning;
//       consumers check `abi >= 3` before touching the additions.
//   4 - appended the simulation-LOD surface: the setCharacterImportance /
//       setJiggleChainImportance / setFootPlanterImportance /
//       setLeanControllerImportance entry points and the
//       CharacterStatistics.lodSkippedSteps counter. Everything from ABI 3
//       keeps its slot and meaning; consumers check `abi >= 4` before
//       touching the additions.
inline constexpr std::uint32_t physicsCharacterCapabilityAbi = 4u;

using CharacterHandle    = std::uint32_t;
using JiggleChainHandle  = std::uint32_t;
using FootPlanterHandle  = std::uint32_t;
using HandPlanterHandle  = std::uint32_t;
using LeanHandle         = std::uint32_t;

inline constexpr CharacterHandle   invalidCharacter     = 0;
inline constexpr JiggleChainHandle invalidJiggleChain   = 0;
inline constexpr FootPlanterHandle invalidFootPlanter   = 0;
inline constexpr HandPlanterHandle invalidHandPlanter   = 0;
inline constexpr LeanHandle        invalidLeanController = 0;

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

// Two-hand planter: two-bone arm IK from root-local shoulders toward
// consumer-supplied world targets (walls, ledges, objects). Hands without a
// set target rest at the root-local rest offsets; targets beyond full arm
// extension clamp to the reach and report the leftover stretch.
struct HandPlanterDescription
{
    // Two-bone arm lengths (meters).
    double armUpperLength = 0.30;
    double armLowerLength = 0.28;

    // Shoulder anchors in the root's local frame, left then right.
    double shoulderOffsets[2][3] = {{-0.23, 0.33, 0.0}, {0.23, 0.33, 0.0}};

    // Hand rest offsets in the root's local frame, left then right (the
    // default targets before setHandTarget overrides them).
    double handOffsets[2][3] = {{-0.28, 0.05, 0.05}, {0.28, 0.05, 0.05}};

    // Elbow pole direction in the root's local frame (normalized on entry):
    // the world direction the elbows bend toward.
    double elbowDirection[3] = {0.0, -0.5, -1.0};
};

// One hand state after an update: whether the target sat within full arm
// extension, the solved (clamped) hand position, the world elbow pole
// direction, the elbow bend angle, and how far the target lies beyond reach.
struct HandState
{
    bool   withinReach = false;
    double handPosition[3]    = {0.0, 0.0, 0.0};
    double elbowDirection[3]  = {0.0, 0.0, -1.0};
    double elbowBendRadians   = 0.0;
    double stretchMeters      = 0.0;
};

// Procedural lean: a spring-damper that tilts the character's up axis
// toward its lateral acceleration, producing a runner-style lean the
// consumer composes on top of the root rotation.
struct LeanControllerDescription
{
    // Hard clamp on the lean angle (radians).
    double maximumLeanRadians = 0.35;

    // Spring toward the target lean angle and velocity damping.
    double stiffness = 12.0;
    double damping   = 6.0;

    // Lean radians per meter per second squared of lateral acceleration.
    double accelerationGain = 0.06;
};

// One lean update result: the signed lean magnitude around the world lean
// axis (horizontal, perpendicular to the acceleration) and the composed
// world-space rotation offset to apply after the root rotation.
struct LeanState
{
    double leanRadians  = 0.0;
    double leanAxis[3]  = {1.0, 0.0, 0.0};
    double leanRotation[4] = {0.0, 0.0, 0.0, 1.0}; // x,y,z,w
};

struct CharacterStatistics
{
    std::uint32_t characterCount   = 0;
    std::uint32_t jiggleChainCount = 0;
    std::uint32_t footPlanterCount = 0;
    std::uint32_t handPlanterCount = 0;
    std::uint32_t leanControllerCount = 0;

    // Simulation LOD: update passes skipped so far across every driven
    // record (a frozen band, or a staggered cadence that was not due on a
    // step).
    std::uint64_t lodSkippedSteps = 0;
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

    // Per-fixed-update control pass: computes the pose error torques directly on the
    // enabled joint targets (both hinge and ball-socket kinds), applies
    // them as equal-and-opposite torque pairs, and applies the upright assist torque
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

    // ---- hand placement (ABI 3 additions) -----------------------------------

    HandPlanterHandle (*createHandPlanter)(void *self, const HandPlanterDescription *description, CharacterError *error);
    void (*destroyHandPlanter)(void *self, HandPlanterHandle planter);

    // Sets the world-space reach target for one hand (index 0 = left, 1 =
    // right); the target persists across updates until changed.
    bool (*setHandTarget)(void *self, HandPlanterHandle planter, std::uint32_t handIndex, const double target[3]);

    // Per-fixed-update IK pass from the root transform (targets stay as set).
    bool (*updateHandPlanter)(void *self, HandPlanterHandle planter, const engine::physics_rigid::RigidTransform *root);

    // Hand results: exactly two entries, left then right.
    bool (*copyHandStates)(void *self, HandPlanterHandle planter, HandState *outStates, std::uint32_t capacity,
        std::uint32_t *outRequired);

    // ---- procedural lean (ABI 3 additions) ----------------------------------

    LeanHandle (*createLeanController)(void *self, const LeanControllerDescription *description, CharacterError *error);
    void (*destroyLeanController)(void *self, LeanHandle lean);

    // Per-fixed-update pass from the root transform and velocity; writes the
    // lean magnitude, axis, and the composed world rotation offset.
    bool (*updateLeanController)(void *self, LeanHandle lean, const engine::physics_rigid::RigidTransform *root,
        const double rootVelocity[3], double fixedDeltaSeconds, LeanState *outState);

    // ---- physical pushing (ABI 3 additions) ----------------------------------

    // Applies an immediate impulse to every dynamic body whose position sits
    // inside the sphere (center, radius): impulse = pushVelocity * forceScale
    // at the body position. Bodies are visited in the world's body order, so
    // the result is deterministic for a given world state. Call once per
    // fixed update while the character moves.
    bool (*applyCharacterPush)(void *self, engine::physics_rigid::WorldHandle world, const double center[3],
        double radius, const double pushVelocity[3], double forceScale);

    bool (*characterStatistics)(void *self, CharacterStatistics *outStatistics);

    // ---- simulation LOD (ABI 4 additions) ------------------------------------

    // Writes the record's importance score in [0, 1]; the module resolves
    // the fidelity band (with demotion hysteresis, the shared ladder every
    // simulation module uses) and drives the record on a staggered cadence.
    // A due character update scales its PD torques by the interval so the
    // impulse per wall time matches the full-rate schedule (the Cached band
    // stops driving entirely, so the ragdoll goes passive); a due jiggle or
    // lean pass integrates interval x the caller's delta; a due foot-planter
    // pass runs its ground probes, and a skipped pass keeps the last planted
    // feet. Hand planters are pure per-call math and always run full.
    // Non-finite scores report failure.
    bool (*setCharacterImportance)(void *self, CharacterHandle character, double importance);
    bool (*setJiggleChainImportance)(void *self, JiggleChainHandle chain, double importance);
    bool (*setFootPlanterImportance)(void *self, FootPlanterHandle planter, double importance);
    bool (*setLeanControllerImportance)(void *self, LeanHandle controller, double importance);
};

} // namespace engine::physics_character
