// records.hpp - the physics_character module's plain-data runtime records:
// the humanoid ragdoll template, character/jiggle/planter/lean records, and
// the shared CharacterSettings.

#pragma once

#include <engine/physics_articulated/capabilities.hpp>
#include <engine/physics_character/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>
#include <engine/util/simulation_lod.hpp>
#include <engine/util/step_accumulator.hpp>

#include <cstdint>
#include <vector>

namespace engine::physics_character
{
namespace math = engine::physics_rigid::math;

// Semi-implicit Euler stability margins for the module's spring-damper
// integrators (jiggle chains, lean controllers): the discrete solve stays
// stable while h^2 k < 4 and h c < 2, so the gains clamp to these fractions
// of the bounds (matching the foliage module's bend-spring margins). The
// clamps absorb the enlarged step of a reduced-cadence LOD pass, which
// would otherwise destabilize a strongly tuned spring.
constexpr double kSpringStiffnessStabilityFactor = 3.2; // 0.8 x the h^2 k < 4 bound
constexpr double kSpringDampingStabilityFactor  = 1.8; // 0.9 x the h c < 2 bound

struct HumanoidPart
{
    std::int32_t parentIndex;
    const char * name;

    engine::physics_rigid::ShapeKind shapeKind;
    double boxHalf[3];         // box
    double capsuleRadius;      // capsule
    double capsuleHalfLength;  // capsule
    double sphereRadius;       // sphere

    double relativePosition[3]; // in the parent's frame

    // Joint to the parent.
    bool   hinge;                 // false = ball-socket
    double jointAnchorChild[3];
    double jointAnchorParent[3];
    double hingeAxis[3];
    bool   limits;
    double limitLower;
    double limitUpper;
    double coneHalfAngle;
    double twistHalfAngle;

    double massFraction;
};

const HumanoidPart kHumanoidParts[] = {
    // pelvis
    {-1, "pelvis", engine::physics_rigid::ShapeKind::Box, {0.14, 0.09, 0.10}, 0.0, 0.0, 0.0,
        {0.0, 0.0, 0.0}, false, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, false, 0.0, 0.0, 0.0, 0.0, 0.14},
    // torso (waist ball-socket)
    {0, "torso", engine::physics_rigid::ShapeKind::Box, {0.16, 0.17, 0.10}, 0.0, 0.0, 0.0,
        {0.0, 0.35, 0.0}, false, {0.0, -0.23, 0.0}, {0.0, 0.12, 0.0}, {0.0, 1.0, 0.0}, true, 0.0, 0.0, 0.4, 0.4, 0.28},
    // head (neck ball-socket)
    {1, "head", engine::physics_rigid::ShapeKind::Sphere, {0.0, 0.0, 0.0}, 0.0, 0.0, 0.10,
        {0.0, 0.39, 0.0}, false, {0.0, -0.17, 0.0}, {0.0, 0.22, 0.0}, {0.0, 1.0, 0.0}, true, 0.0, 0.0, 0.6, 0.6, 0.08},
    // upper arm L (shoulder ball-socket)
    {1, "upper_arm_l", engine::physics_rigid::ShapeKind::Capsule, {0.0, 0.0, 0.0}, 0.044, 0.11, 0.0,
        {-0.23, -0.02, 0.0}, false, {0.0, 0.16, 0.0}, {-0.23, 0.14, 0.0}, {1.0, 0.0, 0.0}, true, 0.0, 0.0, 1.2, 0.5, 0.03},
    // lower arm L (elbow hinge)
    {3, "lower_arm_l", engine::physics_rigid::ShapeKind::Capsule, {0.0, 0.0, 0.0}, 0.04, 0.105, 0.0,
        {0.0, -0.315, 0.0}, true, {0.0, 0.155, 0.0}, {0.0, -0.16, 0.0}, {1.0, 0.0, 0.0}, true, -2.0, 0.1, 0.0, 0.0, 0.02},
    // upper arm R
    {1, "upper_arm_r", engine::physics_rigid::ShapeKind::Capsule, {0.0, 0.0, 0.0}, 0.044, 0.11, 0.0,
        {0.23, -0.02, 0.0}, false, {0.0, 0.16, 0.0}, {0.23, 0.14, 0.0}, {1.0, 0.0, 0.0}, true, 0.0, 0.0, 1.2, 0.5, 0.03},
    // lower arm R (elbow hinge)
    {5, "lower_arm_r", engine::physics_rigid::ShapeKind::Capsule, {0.0, 0.0, 0.0}, 0.04, 0.105, 0.0,
        {0.0, -0.315, 0.0}, true, {0.0, 0.155, 0.0}, {0.0, -0.16, 0.0}, {1.0, 0.0, 0.0}, true, -2.0, 0.1, 0.0, 0.0, 0.02},
    // upper leg L (hip ball-socket)
    {0, "upper_leg_l", engine::physics_rigid::ShapeKind::Capsule, {0.0, 0.0, 0.0}, 0.06, 0.165, 0.0,
        {-0.09, -0.315, 0.0}, false, {0.0, 0.225, 0.0}, {-0.09, -0.09, 0.0}, {1.0, 0.0, 0.0}, true, 0.0, 0.0, 1.1, 0.5, 0.10},
    // lower leg L (knee hinge)
    {7, "lower_leg_l", engine::physics_rigid::ShapeKind::Capsule, {0.0, 0.0, 0.0}, 0.05, 0.152, 0.0,
        {0.0, -0.427, 0.0}, true, {0.0, 0.202, 0.0}, {0.0, -0.225, 0.0}, {1.0, 0.0, 0.0}, true, -2.2, 0.05, 0.0, 0.0, 0.06},
    // upper leg R
    {0, "upper_leg_r", engine::physics_rigid::ShapeKind::Capsule, {0.0, 0.0, 0.0}, 0.06, 0.165, 0.0,
        {0.09, -0.315, 0.0}, false, {0.0, 0.225, 0.0}, {0.09, -0.09, 0.0}, {1.0, 0.0, 0.0}, true, 0.0, 0.0, 1.1, 0.5, 0.10},
    // lower leg R (knee hinge)
    {9, "lower_leg_r", engine::physics_rigid::ShapeKind::Capsule, {0.0, 0.0, 0.0}, 0.05, 0.152, 0.0,
        {0.0, -0.427, 0.0}, true, {0.0, 0.202, 0.0}, {0.0, -0.225, 0.0}, {1.0, 0.0, 0.0}, true, -2.2, 0.05, 0.0, 0.0, 0.06},
};

constexpr std::uint32_t kHumanoidPartCount = sizeof(kHumanoidParts) / sizeof(kHumanoidParts[0]);
// Sum of the humanoid template's mass fractions, derived from the table so
// the normalization and the parts can never drift apart.
constexpr double sumHumanoidMassFractions()
{
    double total = 0.0;
    for (const HumanoidPart &part : kHumanoidParts)
    {
        total += part.massFraction;
    }
    return total;
}
constexpr double kHumanoidMassFractionSum = sumHumanoidMassFractions();

struct CharacterRecord
{
    CharacterHandle handle = invalidCharacter;

    engine::physics_articulated::FigureHandle figure = engine::physics_articulated::invalidFigure;
    engine::physics_rigid::WorldHandle        world  = engine::physics_rigid::invalidWorld;

    std::uint32_t torsoLinkIndex = 1;
    engine::physics_rigid::BodyHandle torsoBody = engine::physics_rigid::invalidBody;

    CharacterMode mode            = CharacterMode::Blended;
    double        driveStrength   = 1.0;
    double        uprightTorque   = 220.0;

    // Per-link pose targets and gains (physical animation).
    struct JointTarget
    {
        bool   enabled            = false;
        double targetAngleRadians = 0.0;
        double stiffness          = 120.0;
        double damping            = 14.0;
        double maximumTorque      = 60.0;
    };
    std::vector<JointTarget> joints;

    // Simulation LOD: the consumer-written importance score resolves the
    // fidelity band; a due update scales its PD torques by the cadence
    // interval (the impulse per wall time matches the full-rate schedule)
    // and the Cached band stops driving entirely.
    double                           importance = 1.0;
    engine::util::SimulationFidelity lodBand    = engine::util::SimulationFidelity::Full;
    engine::util::SimulationCadence  cadence{};
    std::uint64_t                    lodSkippedSteps = 0;
};

struct JiggleBoneState
{
    double offsetVector[3] = {0.0, 0.0, 0.0}; // rotation vector in the parent bone frame
    double angularVelocity[3] = {0.0, 0.0, 0.0};
};

struct JiggleChainRecord
{
    JiggleChainHandle handle = invalidJiggleChain;

    JiggleChainDescription description{};

    std::vector<std::array<double, 3>> restOffsets; // validated copies
    std::vector<JiggleBoneState>       bones;
    std::vector<math::Quaternion>      worldRotations; // output cache

    math::Vec3 previousRootVelocity{};
    bool       hasPreviousVelocity = false;

    // Simulation LOD: a due pass integrates interval x the caller's delta
    // (a spring system, so the enlarged step keeps the wall-time pacing);
    // the Cached band freezes the chain.
    double                           importance = 1.0;
    engine::util::SimulationFidelity lodBand    = engine::util::SimulationFidelity::Full;
    engine::util::SimulationCadence  cadence{};
    std::uint64_t                    lodSkippedSteps = 0;
};

struct FootRuntime
{
    bool   planted = false;
    math::Vec3 plantPosition{};
    math::Vec3 plantNormal{0.0, 1.0, 0.0};
    math::Vec3 kneeDirection{0.0, 0.0, 1.0};
    double kneeBendRadians = 0.0;
    double hipDropMeters   = 0.0;
};

struct FootPlanterRecord
{
    FootPlanterHandle handle = invalidFootPlanter;

    engine::physics_rigid::WorldHandle world = engine::physics_rigid::invalidWorld;

    FootPlanterDescription description{};

    std::vector<engine::physics_rigid::BodyHandle> ignoredBodies; // owned copy of the description's list

    FootRuntime feet[2];

    // Simulation LOD: a due pass runs the ground probes; a skipped pass
    // keeps the last planted feet, and the Cached band freezes them.
    double                           importance = 1.0;
    engine::util::SimulationFidelity lodBand    = engine::util::SimulationFidelity::Full;
    engine::util::SimulationCadence  cadence{};
    std::uint64_t                    lodSkippedSteps = 0;
};

struct HandRuntime
{
    bool       hasTarget  = false; // setHandTarget overrides the root-local rest offset
    math::Vec3 target{};
    math::Vec3 handPosition{};
    math::Vec3 elbowDirection{0.0, 0.0, -1.0};
    double     elbowBendRadians = 0.0;
    double     stretchMeters    = 0.0;
    bool       withinReach      = false;
};

struct HandPlanterRecord
{
    HandPlanterHandle handle = invalidHandPlanter;

    HandPlanterDescription description{};

    HandRuntime hands[2];
};

struct LeanRecord
{
    LeanHandle handle = invalidLeanController;

    LeanControllerDescription description{};

    math::Quaternion leanRotation{0.0, 0.0, 0.0, 1.0};
    double           leanAngleRadians = 0.0;
    math::Vec3       leanAxis{1.0, 0.0, 0.0};
    double           angularVelocity = 0.0;
    math::Vec3       previousVelocity{};
    bool             hasPreviousVelocity = false;

    // Simulation LOD: a due pass integrates interval x the caller's delta;
    // the Cached band holds the last lean.
    double                           importance = 1.0;
    engine::util::SimulationFidelity lodBand    = engine::util::SimulationFidelity::Full;
    engine::util::SimulationCadence  cadence{};
    std::uint64_t                    lodSkippedSteps = 0;
};

struct CharacterSettings
{
    std::uint32_t maximumCharacters      = 32;
    std::uint32_t maximumJiggleChains    = 256;
    std::uint32_t maximumFootPlanters    = 64;
    std::uint32_t maximumHandPlanters    = 64;
    std::uint32_t maximumLeanControllers = 64;

    // Secondary-motion gravity magnitude (meters per second squared): the
    // jiggle chains' virtual force scales this by the chain's
    // gravityInfluence factor. Configurable because a chain may ride a
    // character whose world uses non-default gravity.
    double jiggleGravityMagnitude = 9.81;

    // Simulation LOD: the shared ladder plus the per-band update intervals
    // (0 freezes that band, which is the Cached default).
    engine::util::SimulationLodSettings lod{};
};

} // namespace engine::physics_character
