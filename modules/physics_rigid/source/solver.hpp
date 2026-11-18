// solver.hpp - sequential impulse constraint solving. Contact and joint
// constraints are prepared once per step (anchors, effective masses, bias
// velocities, warm-start impulses) and then iterated in a deterministic
// order. Position drift is stabilized through Baumgarte bias velocities; warm
// starting reuses the previous step's accumulated impulses.

#pragma once

#include "records.hpp"
#include "shapes.hpp"

#include <utility>
#include <vector>

namespace engine::physics_rigid
{

// Per-step solver context.
struct StepContext
{
    double deltaTime        = 0.0;
    double inverseDeltaTime = 0.0;
};

// Scratch contact constraint for one persistent manifold.
struct ContactConstraint
{
    std::size_t bodyIndexA   = 0;
    std::size_t bodyIndexB   = 0;
    double      inverseMassA = 0.0;
    double      inverseMassB = 0.0;
    math::Mat3  inverseInertiaA;
    math::Mat3  inverseInertiaB;

    struct Point
    {
        math::Vec3             rA; // anchor relative to body A's center of mass
        math::Vec3             rB;
        math::Vec3             localAnchorA; // anchors in each body's local frame, for the
        math::Vec3             localAnchorB; // position solver's post-integration anchors
        math::Vec3             normal;       // from B toward A
        math::Vec3             tangent1;
        math::Vec3             tangent2;
        double                 separation              = 0.0;
        double                 restitutionVelocity     = 0.0;
        double                 speculativeTarget       = 0.0; // gap-closing target for speculative contacts
        double                 speculativeLowerImpulse = 0.0; // accumulated clamp: never open the gap faster
        double                 normalMass              = 0.0;
        double                 tangentMass1            = 0.0;
        double                 tangentMass2            = 0.0;
        double                 friction                = 0.0;
        double                 rollingResistance       = 0.0; // torque budget coefficient (0 = off)
        double                 rollingRadius           = 0.0; // effective contact radius of the rolling body
        double                 rollingImpulse1         = 0.0; // per-step rolling-resistance accumulators
        double                 rollingImpulse2         = 0.0;
        PersistedContactPoint *persisted               = nullptr; // impulse + tangent storage in the world
    };

    Point         points[4];
    std::uint32_t pointCount = 0;
};

// Scratch joint constraint for one joint record.
struct JointConstraint
{
    std::size_t bodyIndexA = 0;
    std::size_t bodyIndexB = 0;
    std::size_t jointIndex = 0;
    JointKind   kind       = JointKind::Distance;

    double     inverseMassA = 0.0;
    double     inverseMassB = 0.0;
    math::Mat3 inverseInertiaA;
    math::Mat3 inverseInertiaB;

    math::Vec3 rA; // world anchor offsets from the body centers
    math::Vec3 rB;

    // Distance joint.
    math::Vec3 distanceAxisWorld; // unit vector from A's anchor toward B's
    double     currentLength = 0.0;
    double     distanceMass  = 0.0;

    // Distance-joint spring (soft constraint, Box2D-style bias/gamma); zero
    // gamma selects the hard row. The soft solve's effective mass folds the
    // gamma term in (softMass = 1 / (1/m + gamma)), which is what keeps the
    // implicit-damping scheme stable.
    double springGamma    = 0.0;
    double springBias     = 0.0;
    double springSoftMass = 0.0;

    // Ball-socket / hinge anchor rows (orthonormal world basis).
    math::Vec3 anchorBasis[3];
    double     anchorMass[3] = {0.0, 0.0, 0.0};

    // Shared angular vocabulary (hinge and limited ball-socket).
    math::Vec3 hingeAxisWorld;
    math::Vec3 antiSwingBasis[2];
    double     antiSwingMass[2] = {0.0, 0.0};
    double     twistAngle       = 0.0; // radians, relative to the assembly frame
    double     swingAngle       = 0.0;
    math::Vec3 swingAxisWorld; // unit axis of the current swing tilt
    double     limitMass = 0.0;
    double     motorMass = 0.0;

    // Prismatic: translation rows act on the two axes perpendicular to the
    // slider axis (anchorBasis[0]/[1] with anchorBasis[2] = slider axis);
    // swing is locked by anti-swing rows and twist by an extra row about the
    // slider axis. Translation limits and the linear motor act along the axis.
    math::Vec3 sliderAxisWorld;
    double     sliderDisplacement = 0.0; // anchor separation along the axis, relative to creation
    double     sliderMass         = 0.0; // translation limit row mass along the axis
    double     twistMass          = 0.0; // twist-lock row mass about the axis

    // Fixed weld: three angular rows on an orthonormal basis with the
    // relative-frame error measured per row (world frame).
    math::Vec3 weldBasis[3];
    double     weldMass[3] = {0.0, 0.0, 0.0};
    math::Vec3 weldError;
};

// World-frame inverse inertia tensor R * diag(1/I) * R^T; the zero matrix
// for static and kinematic bodies. Shared with force integration.
math::Mat3 computeWorldInverseInertia(const Body &body);

// Builds the contact constraints for every touching broadphase pair,
// refreshing the world's persistent manifolds (point matching + warm-start
// impulse carry-over). Pairs without a dynamic body are skipped.
void prepareContactConstraints(
    World &world, const StepContext &context, std::vector<ContactConstraint> &outConstraints);

// Builds one constraint per joint; bodies that are both non-dynamic are
// skipped.
void prepareJointConstraints(World &world, const StepContext &context, std::vector<JointConstraint> &outConstraints);

// Applies the persisted impulses of the previous step (no-op when warm
// starting is disabled).
void warmStartContacts(World &world, std::vector<ContactConstraint> &constraints, bool warmStarting);
void warmStartJoints(World &world, std::vector<JointConstraint> &constraints, bool warmStarting);

// One sequential-impulse iteration over all constraints (contacts first,
// joints second; each in fixed order).
void solveContactVelocityIteration(
    World &world, std::vector<ContactConstraint> &constraints, const StepContext &context);
void solveJointVelocityIteration(World &world, std::vector<JointConstraint> &constraints, const StepContext &context);

// One position-solver iteration: drives the remaining contact penetration out
// by moving transforms directly (no velocity change), which removes overlap
// without injecting the bounce energy a Baumgarte bias would.
void solveContactPositionIteration(
    World &world, std::vector<ContactConstraint> &constraints, const StepContext &context);

} // namespace engine::physics_rigid
