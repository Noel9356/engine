// capabilities.hpp - public C capability surface of the physics_articulated
// module.
//
// Articulated figures are trees of rigid links (created and simulated inside
// a physics_rigid world) connected by hinge or ball-socket joints with angle
// limits, plus per-joint PD drives mapped onto the rigid module's velocity
// motors. Consumers acquire it via
// ParticipantContext::acquireCapability<ArticulatedCapability>(); the module
// declares physics_rigid as a required capability, so the kernel constructs
// the rigid core first and grants the capability permission. All figures
// live in caller-created physics_rigid worlds, which means ray casts, sphere
// queries, and per-link force application work through the rigid capability
// with the link body handles reported here.

#pragma once

#include <engine/physics_rigid/capabilities.hpp>

#include <cstdint>

namespace engine::physics_articulated
{

// Increment when any public capability structure or function-table layout
// changes; consumers verify it after acquisition.
inline constexpr std::uint32_t physicsArticulatedCapabilityAbi = 1u;

using FigureHandle = std::uint32_t;

inline constexpr FigureHandle invalidFigure = 0;

enum class ArticulatedError : std::uint32_t
{
    None = 0,

    InvalidHandle,      // unknown figure or link index
    InvalidDescription, // null pointers, non-finite values, cyclic or out-of-range parents
    CapacityExceeded,   // configured figure/link budget exhausted or no collision category left
    Unsupported         // request not applicable to the joint kind (e.g. driving a root link)
};

// Connection between a link and its parent. Hinge removes swing (rotation
// about the shared axis only); BallSocket keeps the anchors coincident with
// optional cone/twist limits.
enum class ArticulatedJointKind : std::uint32_t
{
    Hinge = 0,
    BallSocket
};

// PD drive for one joint, mapped onto the rigid module's hinge motor:
// updateDrives measures the joint angle against the assembly frame, derives
// a velocity target from the stiffness/damping gains, and clamps it to
// maximumVelocity.
struct JointDrive
{
    bool   enabled            = false;
    double stiffness          = 0.0;  // proportional gain, torque-equivalent per radian
    double damping            = 0.0;  // derivative gain, torque-equivalent per radian/second
    double targetAngleRadians = 0.0;  // hinge twist target relative to the assembly frame
    double maximumVelocity    = 8.0;  // rad/s clamp on the derived velocity target
    double maximumTorque      = 50.0; // rigid motor torque budget
};

// One link of a figure: a rigid body shape plus the joint that connects it
// to its parent. relativeTransform places the link in the parent's frame
// (world frame for the root); the joint anchors are expressed in each link's
// local frame and must coincide in world space at the assembly pose.
struct LinkDescription
{
    std::int32_t parentIndex = -1; // index into the figure's link array; -1 for the root

    engine::physics_rigid::ShapeDescription shape;

    engine::physics_rigid::RigidTransform relativeTransform;

    // Mass source: density > 0 wins; otherwise the explicit mass is used
    // with a shape-derived inertia (mirrors BodyDescription).
    double density = 1000.0;
    double mass    = 0.0;

    engine::physics_rigid::MaterialDescription material;

    ArticulatedJointKind jointToParent = ArticulatedJointKind::Hinge;

    engine::physics_rigid::Vec3 jointAnchorLocalChild  = {0.0, 0.0, 0.0};
    engine::physics_rigid::Vec3 jointAnchorLocalParent = {0.0, 0.0, 0.0};

    // Hinge rotation axis in the parent link's local frame.
    engine::physics_rigid::Vec3 hingeAxisLocalParent = {0.0, 1.0, 0.0};

    // Hinge fold limits (relative to the assembly pose) or ball-socket
    // cone/twist limits, forwarded to the rigid joint.
    bool   limitsEnabled         = false;
    double limitLowerRadians     = -0.5;
    double limitUpperRadians     = 0.5;
    double coneHalfAngleRadians  = 3.141592653589793;
    double twistHalfAngleRadians = 3.141592653589793;
};

struct FigureDescription
{
    const LinkDescription *links     = nullptr;
    std::uint32_t          linkCount = 0;

    // When false, links of the same figure never collide with each other
    // (adjacent links always overlap at their joints); each figure reserves
    // one collision category bit for this and collides with everything else.
    bool selfCollision = false;

    // When true the root link is a dynamic body; when false the root is
    // static (a pendulum or ragdoll anchored to the world).
    bool rootDynamic = true;

    bool sleepingEnabled = true;
};

struct FigureInfo
{
    std::uint32_t linkCount  = 0;
    std::uint32_t jointCount = 0;
};

// Measured joint state relative to the assembly frame. Hinge joints fill
// angleRadians (twist about the hinge axis); ball-socket joints fill swing
// and twist. Angles are signed radians.
struct FigureJointState
{
    double angleRadians = 0.0;
    double swingRadians = 0.0;
    double twistRadians = 0.0;
};

struct ArticulatedFigureStatistics
{
    std::uint32_t linkCount        = 0;
    std::uint32_t activeDriveCount = 0;
    std::uint32_t categoryBit      = 0;
};

// The "physics_articulated" capability. Calls are main-thread only; the
// module holds no locks and forwards every body-level operation to the
// physics_rigid capability it required.
struct ArticulatedCapability
{
    static constexpr const char *capabilityName = "physics_articulated";

    std::uint32_t abi = physicsArticulatedCapabilityAbi;

    void *self = nullptr;

    FigureHandle (*createFigure)(void *self, engine::physics_rigid::WorldHandle world,
        const FigureDescription *description, ArticulatedError *error);
    void (*destroyFigure)(void *self, FigureHandle figure);

    bool (*figureInfo)(void *self, FigureHandle figure, FigureInfo *outInfo);
    bool (*figureStatistics)(void *self, FigureHandle figure, ArticulatedFigureStatistics *outStatistics);

    // Rigid body handle of one link (rendering, grabbing, forces through the
    // rigid capability).
    bool (*copyLinkBodyHandles)(void *self, FigureHandle figure, engine::physics_rigid::BodyHandle *outHandles,
        std::uint32_t capacity, std::uint32_t *outRequired);
    bool (*linkBodyHandle)(
        void *self, FigureHandle figure, std::uint32_t linkIndex, engine::physics_rigid::BodyHandle *outHandle);

    // Per-joint drive configuration; linkIndex refers to the child link (the
    // root link has no joint and reports Unsupported).
    bool (*setJointDrive)(void *self, FigureHandle figure, std::uint32_t linkIndex, const JointDrive *drive);
    bool (*copyJointDrive)(void *self, FigureHandle figure, std::uint32_t linkIndex, JointDrive *outDrive);

    // Advances every enabled drive one fixed step: measures angles, derives
    // motor velocity targets, and pushes them into the rigid joints. Call
    // once per fixed update before stepping the rigid world (or after; the
    // motor applies at the next solve either way).
    bool (*updateDrives)(void *self, FigureHandle figure, double fixedDeltaSeconds);

    // Measured joint state relative to the assembly frame.
    bool (*copyJointState)(void *self, FigureHandle figure, std::uint32_t linkIndex, FigureJointState *outState);
};

} // namespace engine::physics_articulated
