// solver.cpp - sequential impulse solving for contacts and joints. The
// contact block solver handles the normal constraint with Baumgarte and
// restitution bias velocities plus a two-tangent friction cone; joints
// provide distance, ball-socket (with optional cone/twist limits), and hinge
// (with angle limits and a velocity motor) rows. All impulses accumulate in
// the world-owned records so warm starting is exact.

#include "solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace engine::physics_rigid
{
namespace
{

inline double componentAxis(const math::Vec3 &vector, std::uint32_t axis)
{
    return (&vector.x)[axis];
}

} // namespace

// World-frame inverse inertia tensor R * diag(1/I) * R^T; the zero matrix
// for static and kinematic bodies.
math::Mat3 computeWorldInverseInertia(const Body &body)
{
    math::Mat3 inverseInertia;
    if (body.inverseMass == 0.0)
    {
        // Static and kinematic bodies cannot rotate: the zero tensor, not
        // the matrix's identity default.
        for (double &element : inverseInertia.elements)
        {
            element = 0.0;
        }
        return inverseInertia;
    }
    const math::Mat3 rotation = math::makeRotationMatrix(body.transform.rotation);
    for (std::uint32_t row = 0; row < 3; ++row)
    {
        for (std::uint32_t column = 0; column < 3; ++column)
        {
            double sum = 0.0;
            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                sum += rotation.elements[row * 3 + axis] * componentAxis(body.inverseInertiaLocal, axis)
                     * rotation.elements[column * 3 + axis];
            }
            inverseInertia.elements[row * 3 + column] = sum;
        }
    }
    return inverseInertia;
}

inline math::Vec3 velocityAt(const Body &body, const math::Vec3 &offsetFromCenter)
{
    return body.linearVelocity + math::cross(body.angularVelocity, offsetFromCenter);
}

// Applies an impulse to A (+) and the opposite to B (-) at the shared anchor.
void applyPairImpulse(Body &bodyA, Body &bodyB, const math::Mat3 &inverseInertiaA, const math::Mat3 &inverseInertiaB,
    const math::Vec3 &rA, const math::Vec3 &rB, const math::Vec3 &impulse)
{
    bodyA.linearVelocity += impulse * bodyA.inverseMass;
    bodyA.angularVelocity += math::multiplyMatrixVector(inverseInertiaA, math::cross(rA, impulse));
    bodyB.linearVelocity -= impulse * bodyB.inverseMass;
    bodyB.angularVelocity -= math::multiplyMatrixVector(inverseInertiaB, math::cross(rB, impulse));
}

// Applies equal-and-opposite angular-only impulses about one axis.
void applyAngularPairImpulse(Body &bodyA, Body &bodyB, const math::Mat3 &inverseInertiaA,
    const math::Mat3 &inverseInertiaB, const math::Vec3 &axis, double impulse)
{
    bodyA.angularVelocity -= math::multiplyMatrixVector(inverseInertiaA, axis) * impulse;
    bodyB.angularVelocity += math::multiplyMatrixVector(inverseInertiaB, axis) * impulse;
}

// Orthogonal tangents for a unit normal: the world axis least aligned with
// the normal gives a stable basis without degeneracy.
void makeTangentBasis(const math::Vec3 &normal, math::Vec3 &outTangent1, math::Vec3 &outTangent2)
{
    math::Vec3 seed{1.0, 0.0, 0.0};
    double     smallestAlignment = std::abs(normal.x);
    if (std::abs(normal.y) < smallestAlignment)
    {
        seed              = math::Vec3{0.0, 1.0, 0.0};
        smallestAlignment = std::abs(normal.y);
    }
    if (std::abs(normal.z) < smallestAlignment)
    {
        seed = math::Vec3{0.0, 0.0, 1.0};
    }
    outTangent1 = math::normalized(math::cross(seed, normal));
    outTangent2 = math::cross(normal, outTangent1);
}

// Scalar effective mass along a Jacobian (linear + angular rows); zero when
// the pair has no movable mass along it.
double effectiveMass(const math::Mat3 &inverseInertiaA, const math::Mat3 &inverseInertiaB, const math::Vec3 &rA,
    const math::Vec3 &rB, double inverseMassA, double inverseMassB, const math::Vec3 &axis)
{
    const math::Vec3 angularA = math::cross(rA, axis);
    const math::Vec3 angularB = math::cross(rB, axis);
    const double     angular  = math::dot(math::multiplyMatrixVector(inverseInertiaA, angularA), angularA)
                              + math::dot(math::multiplyMatrixVector(inverseInertiaB, angularB), angularB);
    const double     total    = inverseMassA + inverseMassB + angular;
    return total > 1.0e-12 ? 1.0 / total : 0.0;
}

// Scalar effective mass for angular-only rows.
double effectiveAngularMass(
    const math::Mat3 &inverseInertiaA, const math::Mat3 &inverseInertiaB, const math::Vec3 &axis)
{
    const double total = math::dot(math::multiplyMatrixVector(inverseInertiaA, axis), axis)
                       + math::dot(math::multiplyMatrixVector(inverseInertiaB, axis), axis);
    return total > 1.0e-12 ? 1.0 / total : 0.0;
}

// Applies a positional angular displacement to a body's rotation:
// q += 0.5 * dtheta * q, then renormalizes (dtheta is a rotation vector).
void applyAngularDisplacement(Body &body, const math::Vec3 &angularDelta)
{
    const math::Quaternion deltaQuaternion{angularDelta.x * 0.5, angularDelta.y * 0.5, angularDelta.z * 0.5, 0.0};
    body.transform.rotation = math::normalize(body.transform.rotation + deltaQuaternion * body.transform.rotation);
}

// Orthonormal basis whose third vector is the given direction (or a fallback
// world axis for degenerate inputs).
void makeAnchorBasis(const math::Vec3 &direction, math::Vec3 basis[3])
{
    basis[2] = math::lengthSquared(direction) > 1.0e-12 ? math::normalized(direction) : math::Vec3{1.0, 0.0, 0.0};
    math::Vec3 seed = std::abs(basis[2].x) < 0.9 ? math::Vec3{1.0, 0.0, 0.0} : math::Vec3{0.0, 1.0, 0.0};
    basis[0]        = math::normalized(math::cross(seed, basis[2]));
    basis[1]        = math::cross(basis[2], basis[0]);
}

void prepareContactConstraints(World &world, const StepContext &context, std::vector<ContactConstraint> &outConstraints)
{
    outConstraints.clear();
    const SimulationSettings &settings = world.settings;

    // Deterministic order: sort the refreshed manifolds by their pair key.
    std::vector<PersistedManifold *> sortedManifolds;
    sortedManifolds.reserve(world.manifolds.size());
    for (auto &entry : world.manifolds)
    {
        if (entry.second.pointCount > 0)
        {
            sortedManifolds.push_back(&entry.second);
        }
    }
    std::sort(sortedManifolds.begin(),
        sortedManifolds.end(),
        [](const PersistedManifold *left, const PersistedManifold *right) { return left->pairKey < right->pairKey; });

    for (PersistedManifold *manifold : sortedManifolds)
    {
        Body &bodyA = world.bodies[manifold->bodyIndexA];
        Body &bodyB = world.bodies[manifold->bodyIndexB];
        if (bodyA.kind != BodyKind::Dynamic && bodyB.kind != BodyKind::Dynamic)
        {
            continue; // no response to solve for
        }

        ContactConstraint constraint;
        constraint.bodyIndexA      = manifold->bodyIndexA;
        constraint.bodyIndexB      = manifold->bodyIndexB;
        constraint.inverseMassA    = bodyA.inverseMass;
        constraint.inverseMassB    = bodyB.inverseMass;
        constraint.inverseInertiaA = computeWorldInverseInertia(bodyA);
        constraint.inverseInertiaB = computeWorldInverseInertia(bodyB);

        const double friction    = std::sqrt(bodyA.material.friction * bodyB.material.friction);
        const double restitution = std::max(bodyA.material.restitution, bodyB.material.restitution);

        constraint.pointCount = manifold->pointCount;
        for (std::uint32_t pointIndex = 0; pointIndex < manifold->pointCount; ++pointIndex)
        {
            PersistedContactPoint    &persisted = manifold->points[pointIndex];
            ContactConstraint::Point &point     = constraint.points[pointIndex];
            point.persisted                     = &persisted;
            point.normal                        = manifold->normal;
            makeTangentBasis(manifold->normal, point.tangent1, point.tangent2);
            point.rA           = persisted.anchorA - bodyA.transform.position;
            point.rB           = persisted.anchorB - bodyB.transform.position;
            point.separation   = math::dot(persisted.anchorA - persisted.anchorB, manifold->normal);
            point.friction     = friction;
            point.localAnchorA = math::inverseRotate(bodyA.transform.rotation, point.rA);
            point.localAnchorB = math::inverseRotate(bodyB.transform.rotation, point.rB);

            // The velocity solve targets restitution plus the speculative
            // gap: a positive separation must close within this step, which
            // stops approaching bodies before they touch and removes impact
            // overshoot. Penetration itself is the position solver's job.
            const math::Vec3 relativeVelocity = velocityAt(bodyA, point.rA) - velocityAt(bodyB, point.rB);
            const double     separatingSpeed  = math::dot(relativeVelocity, point.normal);
            point.restitutionVelocity
                = separatingSpeed < -settings.restitutionVelocityThreshold ? -restitution * separatingSpeed : 0.0;
            point.speculativeTarget = point.separation > 0.0
                                        ? std::max(point.restitutionVelocity, -point.separation / context.deltaTime)
                                        : point.restitutionVelocity;
            point.speculativeLowerImpulse
                = point.separation > 0.0 ? -point.normalMass * point.separation / context.deltaTime : 0.0;

            point.normalMass   = effectiveMass(constraint.inverseInertiaA,
                constraint.inverseInertiaB,
                point.rA,
                point.rB,
                constraint.inverseMassA,
                constraint.inverseMassB,
                point.normal);
            point.tangentMass1 = effectiveMass(constraint.inverseInertiaA,
                constraint.inverseInertiaB,
                point.rA,
                point.rB,
                constraint.inverseMassA,
                constraint.inverseMassB,
                point.tangent1);
            point.tangentMass2 = effectiveMass(constraint.inverseInertiaA,
                constraint.inverseInertiaB,
                point.rA,
                point.rB,
                constraint.inverseMassA,
                constraint.inverseMassB,
                point.tangent2);
        }
        outConstraints.push_back(constraint);
    }
}

void prepareJointConstraints(World &world, const StepContext &context, std::vector<JointConstraint> &outConstraints)
{
    (void)context;
    (void)world;
    for (std::size_t jointIndex = 0; jointIndex < world.joints.size(); ++jointIndex)
    {
        const Joint &joint         = world.joints[jointIndex];
        const auto   bodyAIterator = world.bodyIndexByHandle.find(joint.description.bodyA);
        const auto   bodyBIterator = world.bodyIndexByHandle.find(joint.description.bodyB);
        if (bodyAIterator == world.bodyIndexByHandle.end() || bodyBIterator == world.bodyIndexByHandle.end())
        {
            continue;
        }
        const std::size_t indexA = bodyAIterator->second;
        const std::size_t indexB = bodyBIterator->second;
        Body             &bodyA  = world.bodies[indexA];
        Body             &bodyB  = world.bodies[indexB];
        if (bodyA.kind != BodyKind::Dynamic && bodyB.kind != BodyKind::Dynamic)
        {
            continue;
        }

        JointConstraint constraint;
        constraint.bodyIndexA      = indexA;
        constraint.bodyIndexB      = indexB;
        constraint.jointIndex      = jointIndex;
        constraint.kind            = joint.description.kind;
        constraint.inverseMassA    = bodyA.inverseMass;
        constraint.inverseMassB    = bodyB.inverseMass;
        constraint.inverseInertiaA = computeWorldInverseInertia(bodyA);
        constraint.inverseInertiaB = computeWorldInverseInertia(bodyB);

        const math::Vec3 anchorWorldA
            = bodyA.transform.position + math::rotate(bodyA.transform.rotation, joint.description.anchorLocalA);
        const math::Vec3 anchorWorldB
            = bodyB.transform.position + math::rotate(bodyB.transform.rotation, joint.description.anchorLocalB);
        constraint.rA = anchorWorldA - bodyA.transform.position;
        constraint.rB = anchorWorldB - bodyB.transform.position;

        const math::Quaternion relativeRotation = math::conjugate(bodyA.transform.rotation) * bodyB.transform.rotation;

        if (constraint.kind == JointKind::Distance)
        {
            const math::Vec3 separation  = anchorWorldB - anchorWorldA;
            constraint.currentLength     = math::length(separation);
            constraint.distanceAxisWorld = constraint.currentLength > 1.0e-9
                                             ? separation * (1.0 / constraint.currentLength)
                                             : math::Vec3{0.0, 1.0, 0.0};
            constraint.distanceMass      = effectiveMass(constraint.inverseInertiaA,
                constraint.inverseInertiaB,
                constraint.rA,
                constraint.rB,
                constraint.inverseMassA,
                constraint.inverseMassB,
                constraint.distanceAxisWorld);
        }
        else
        {
            makeAnchorBasis(anchorWorldB - anchorWorldA, constraint.anchorBasis);
            for (std::uint32_t axisIndex = 0; axisIndex < 3; ++axisIndex)
            {
                constraint.anchorMass[axisIndex] = effectiveMass(constraint.inverseInertiaA,
                    constraint.inverseInertiaB,
                    constraint.rA,
                    constraint.rB,
                    constraint.inverseMassA,
                    constraint.inverseMassB,
                    constraint.anchorBasis[axisIndex]);
            }
            constraint.hingeAxisWorld = math::rotate(bodyA.transform.rotation, joint.axisLocalA);
            constraint.twistAngle     = math::twistAngleRadians(relativeRotation, joint.axisLocalA);
            constraint.swingAngle     = math::swingAngleRadians(relativeRotation, joint.axisLocalA);

            // Anti-swing basis: two axes orthogonal to the hinge axis.
            {
                math::Vec3 swingBasis[3];
                makeAnchorBasis(constraint.hingeAxisWorld, swingBasis);
                constraint.antiSwingBasis[0] = swingBasis[0];
                constraint.antiSwingBasis[1] = swingBasis[1];
            }
            constraint.antiSwingMass[0] = effectiveAngularMass(
                constraint.inverseInertiaA, constraint.inverseInertiaB, constraint.antiSwingBasis[0]);
            constraint.antiSwingMass[1] = effectiveAngularMass(
                constraint.inverseInertiaA, constraint.inverseInertiaB, constraint.antiSwingBasis[1]);

            // Swing direction: the local swing tilt of body B relative to the
            // hinge axis, expressed in world space.
            {
                const math::Quaternion twist = math::twistComponent(relativeRotation, joint.axisLocalA);
                const math::Quaternion swing = math::conjugate(twist) * relativeRotation;
                const math::Vec3       swingLocal{swing.x, swing.y, swing.z};
                constraint.swingAxisWorld = math::lengthSquared(swingLocal) > 1.0e-12
                                              ? math::rotate(bodyA.transform.rotation, math::normalized(swingLocal))
                                              : math::Vec3{0.0, 1.0, 0.0};
            }

            constraint.limitMass = effectiveAngularMass(
                constraint.inverseInertiaA, constraint.inverseInertiaB, constraint.hingeAxisWorld);
            constraint.motorMass = constraint.limitMass;
        }

        outConstraints.push_back(constraint);
    }
}

void warmStartContacts(World &world, std::vector<ContactConstraint> &constraints, bool warmStarting)
{
    if (!warmStarting)
    {
        return;
    }
    for (ContactConstraint &constraint : constraints)
    {
        Body &bodyA = world.bodies[constraint.bodyIndexA];
        Body &bodyB = world.bodies[constraint.bodyIndexB];
        for (std::uint32_t pointIndex = 0; pointIndex < constraint.pointCount; ++pointIndex)
        {
            const ContactConstraint::Point &point     = constraint.points[pointIndex];
            const PersistedContactPoint    &persisted = *point.persisted;
            const math::Vec3                impulse   = point.normal * persisted.normalImpulse
                                                      + persisted.tangent1 * persisted.tangentImpulse1
                                                      + persisted.tangent2 * persisted.tangentImpulse2;
            applyPairImpulse(
                bodyA, bodyB, constraint.inverseInertiaA, constraint.inverseInertiaB, point.rA, point.rB, impulse);
        }
    }
}

void warmStartJoints(World &world, std::vector<JointConstraint> &constraints, bool warmStarting)
{
    if (!warmStarting)
    {
        return;
    }
    for (JointConstraint &constraint : constraints)
    {
        Joint &joint = world.joints[constraint.jointIndex];
        Body  &bodyA = world.bodies[constraint.bodyIndexA];
        Body  &bodyB = world.bodies[constraint.bodyIndexB];
        switch (constraint.kind)
        {
        case JointKind::Distance:
            applyPairImpulse(bodyA,
                bodyB,
                constraint.inverseInertiaA,
                constraint.inverseInertiaB,
                constraint.rA,
                constraint.rB,
                constraint.distanceAxisWorld * joint.distanceImpulse);
            break;
        case JointKind::BallSocket:
        case JointKind::Hinge: {
            applyPairImpulse(bodyA,
                bodyB,
                constraint.inverseInertiaA,
                constraint.inverseInertiaB,
                constraint.rA,
                constraint.rB,
                joint.ballSocketImpulse);
            if (constraint.kind == JointKind::Hinge)
            {
                applyAngularPairImpulse(bodyA,
                    bodyB,
                    constraint.inverseInertiaA,
                    constraint.inverseInertiaB,
                    constraint.antiSwingBasis[0],
                    joint.antiSwingImpulse1);
                applyAngularPairImpulse(bodyA,
                    bodyB,
                    constraint.inverseInertiaA,
                    constraint.inverseInertiaB,
                    constraint.antiSwingBasis[1],
                    joint.antiSwingImpulse2);
                if (joint.description.limitsEnabled)
                {
                    applyAngularPairImpulse(bodyA,
                        bodyB,
                        constraint.inverseInertiaA,
                        constraint.inverseInertiaB,
                        constraint.hingeAxisWorld,
                        joint.limitImpulse);
                }
                if (joint.description.motorEnabled)
                {
                    applyAngularPairImpulse(bodyA,
                        bodyB,
                        constraint.inverseInertiaA,
                        constraint.inverseInertiaB,
                        constraint.hingeAxisWorld,
                        joint.motorImpulse);
                }
            }
            else if (joint.description.limitsEnabled)
            {
                // Ball-socket limits: the limit impulse resists swing and
                // twist violations along their measured axes.
                applyAngularPairImpulse(bodyA,
                    bodyB,
                    constraint.inverseInertiaA,
                    constraint.inverseInertiaB,
                    constraint.swingAxisWorld,
                    joint.limitImpulse);
                applyAngularPairImpulse(bodyA,
                    bodyB,
                    constraint.inverseInertiaA,
                    constraint.inverseInertiaB,
                    constraint.hingeAxisWorld,
                    joint.limitImpulse);
            }
            break;
        }
        default:
            break;
        }
    }
}

void solveContactVelocityIteration(
    World &world, std::vector<ContactConstraint> &constraints, const StepContext &context)
{
    (void)context;
    for (ContactConstraint &constraint : constraints)
    {
        Body &bodyA = world.bodies[constraint.bodyIndexA];
        Body &bodyB = world.bodies[constraint.bodyIndexB];
        for (std::uint32_t pointIndex = 0; pointIndex < constraint.pointCount; ++pointIndex)
        {
            ContactConstraint::Point &point     = constraint.points[pointIndex];
            PersistedContactPoint    &persisted = *point.persisted;

            // Normal constraint (restitution target only), then the friction
            // cone bounded by the accumulated normal impulse.
            {
                const math::Vec3 relativeVelocity = velocityAt(bodyA, point.rA) - velocityAt(bodyB, point.rB);
                const double     separatingSpeed  = math::dot(relativeVelocity, point.normal);
                double           impulseLambda    = point.normalMass * (point.speculativeTarget - separatingSpeed);
                const double     oldImpulse       = persisted.normalImpulse;
                const double     newImpulse       = std::max(oldImpulse + impulseLambda, point.speculativeLowerImpulse);
                impulseLambda                     = newImpulse - oldImpulse;
                persisted.normalImpulse           = newImpulse;
                applyPairImpulse(bodyA,
                    bodyB,
                    constraint.inverseInertiaA,
                    constraint.inverseInertiaB,
                    point.rA,
                    point.rB,
                    point.normal * impulseLambda);
            }
            for (std::uint32_t tangentIndex = 0; tangentIndex < 2; ++tangentIndex)
            {
                const math::Vec3 &tangent  = tangentIndex == 0 ? point.tangent1 : point.tangent2;
                double &accumulatedImpulse = tangentIndex == 0 ? persisted.tangentImpulse1 : persisted.tangentImpulse2;
                const double tangentMass   = tangentIndex == 0 ? point.tangentMass1 : point.tangentMass2;
                if (tangentMass == 0.0)
                {
                    continue;
                }
                const math::Vec3 relativeVelocity = velocityAt(bodyA, point.rA) - velocityAt(bodyB, point.rB);
                const double     slidingSpeed     = math::dot(relativeVelocity, tangent);
                double           impulseLambda    = -tangentMass * slidingSpeed;
                const double     maximum          = point.friction * persisted.normalImpulse;
                const double     oldImpulse       = accumulatedImpulse;
                const double     newImpulse       = std::clamp(oldImpulse + impulseLambda, -maximum, maximum);
                impulseLambda                     = newImpulse - oldImpulse;
                accumulatedImpulse                = newImpulse;
                applyPairImpulse(bodyA,
                    bodyB,
                    constraint.inverseInertiaA,
                    constraint.inverseInertiaB,
                    point.rA,
                    point.rB,
                    tangent * impulseLambda);
            }
        }
    }
}

void solveJointVelocityIteration(World &world, std::vector<JointConstraint> &constraints, const StepContext &context)
{
    const SimulationSettings &settings         = world.settings;
    const double              inverseDeltaTime = context.inverseDeltaTime;
    for (JointConstraint &constraint : constraints)
    {
        Joint &joint = world.joints[constraint.jointIndex];
        Body  &bodyA = world.bodies[constraint.bodyIndexA];
        Body  &bodyB = world.bodies[constraint.bodyIndexB];

        if (constraint.kind == JointKind::Distance)
        {
            const math::Vec3 relativeVelocity = velocityAt(bodyA, constraint.rA) - velocityAt(bodyB, constraint.rB);
            const double     axisSpeed        = math::dot(relativeVelocity, constraint.distanceAxisWorld);
            const double     lengthError      = constraint.currentLength - joint.restLength;
            const double     bias
                = -(settings.jointBaumgarteFactor * inverseDeltaTime) * std::clamp(lengthError, -0.2, 0.2);
            double impulseLambda = constraint.distanceMass > 0.0 ? -(axisSpeed + bias) * constraint.distanceMass : 0.0;
            const double oldImpulse = joint.distanceImpulse;
            const double newImpulse = oldImpulse + impulseLambda;
            joint.distanceImpulse   = newImpulse;
            impulseLambda           = newImpulse - oldImpulse;
            applyPairImpulse(bodyA,
                bodyB,
                constraint.inverseInertiaA,
                constraint.inverseInertiaB,
                constraint.rA,
                constraint.rB,
                constraint.distanceAxisWorld * impulseLambda);
            continue;
        }

        // Anchor coincidence rows (ball-socket and hinge).
        {
            const math::Vec3 anchorError
                = (bodyB.transform.position + math::rotate(bodyB.transform.rotation, joint.description.anchorLocalB))
                - (bodyA.transform.position + math::rotate(bodyA.transform.rotation, joint.description.anchorLocalA));
            for (std::uint32_t axisIndex = 0; axisIndex < 3; ++axisIndex)
            {
                const math::Vec3 &axis = constraint.anchorBasis[axisIndex];
                if (constraint.anchorMass[axisIndex] == 0.0)
                {
                    continue;
                }
                const math::Vec3 relativeVelocity = velocityAt(bodyA, constraint.rA) - velocityAt(bodyB, constraint.rB);
                const double     axisSpeed        = math::dot(relativeVelocity, axis);
                const double     error            = math::dot(anchorError, axis);
                const double bias = -(settings.jointBaumgarteFactor * inverseDeltaTime) * std::clamp(error, -0.2, 0.2);
                double       impulseLambda = -(axisSpeed + bias) * constraint.anchorMass[axisIndex];
                math::Vec3   rowImpulse    = axis * impulseLambda;
                joint.ballSocketImpulse += rowImpulse;
                applyPairImpulse(bodyA,
                    bodyB,
                    constraint.inverseInertiaA,
                    constraint.inverseInertiaB,
                    constraint.rA,
                    constraint.rB,
                    rowImpulse);
            }
        }

        if (constraint.kind == JointKind::Hinge)
        {
            // Anti-swing rows: zero relative angular velocity across the two
            // axes orthogonal to the hinge axis, with Baumgarte bias driving
            // the measured swing tilt back to zero.
            for (std::uint32_t axisIndex = 0; axisIndex < 2; ++axisIndex)
            {
                const math::Vec3 &axis = constraint.antiSwingBasis[axisIndex];
                if (constraint.antiSwingMass[axisIndex] == 0.0)
                {
                    continue;
                }
                const double swingComponent = math::dot(constraint.swingAxisWorld, axis) * constraint.swingAngle;
                const double relativeSpeed  = math::dot(bodyB.angularVelocity - bodyA.angularVelocity, axis);
                const double bias
                    = (settings.jointBaumgarteFactor * inverseDeltaTime) * std::clamp(swingComponent, -0.2, 0.2);
                double       impulseLambda      = -(relativeSpeed + bias) * constraint.antiSwingMass[axisIndex];
                double      &accumulatedImpulse = axisIndex == 0 ? joint.antiSwingImpulse1 : joint.antiSwingImpulse2;
                const double oldImpulse         = accumulatedImpulse;
                accumulatedImpulse              = oldImpulse + impulseLambda;
                impulseLambda                   = accumulatedImpulse - oldImpulse;
                applyAngularPairImpulse(
                    bodyA, bodyB, constraint.inverseInertiaA, constraint.inverseInertiaB, axis, impulseLambda);
            }

            // Motor row: track the target angular velocity about the hinge.
            if (joint.description.motorEnabled && constraint.motorMass > 0.0)
            {
                const double relativeSpeed
                    = math::dot(bodyB.angularVelocity - bodyA.angularVelocity, constraint.hingeAxisWorld);
                const double speedError     = relativeSpeed - joint.description.motorTargetVelocity;
                double       impulseLambda  = -constraint.motorMass * speedError;
                const double maximumImpulse = joint.description.maximumMotorTorque * context.deltaTime;
                const double oldImpulse     = joint.motorImpulse;
                const double newImpulse     = std::clamp(oldImpulse + impulseLambda, -maximumImpulse, maximumImpulse);
                impulseLambda               = newImpulse - oldImpulse;
                joint.motorImpulse          = newImpulse;
                applyAngularPairImpulse(bodyA,
                    bodyB,
                    constraint.inverseInertiaA,
                    constraint.inverseInertiaB,
                    constraint.hingeAxisWorld,
                    impulseLambda);
            }

            // Angle limits about the hinge axis.
            if (joint.description.limitsEnabled && constraint.limitMass > 0.0)
            {
                const double lower = joint.description.limitLowerRadians;
                const double upper = joint.description.limitUpperRadians;
                const double angle = constraint.twistAngle;
                if (angle < lower || angle > upper)
                {
                    const double violation = angle < lower ? angle - lower : angle - upper;
                    const double relativeSpeed
                        = math::dot(bodyB.angularVelocity - bodyA.angularVelocity, constraint.hingeAxisWorld);
                    const double bias
                        = (settings.jointBaumgarteFactor * inverseDeltaTime) * std::clamp(violation, -0.2, 0.2);
                    double impulseLambda = -(relativeSpeed + bias) * constraint.limitMass;
                    if (angle < lower)
                    {
                        impulseLambda = std::max(impulseLambda, 0.0);
                    }
                    else
                    {
                        impulseLambda = std::min(impulseLambda, 0.0);
                    }
                    const double oldImpulse = joint.limitImpulse;
                    joint.limitImpulse      = oldImpulse + impulseLambda;
                    impulseLambda           = joint.limitImpulse - oldImpulse;
                    applyAngularPairImpulse(bodyA,
                        bodyB,
                        constraint.inverseInertiaA,
                        constraint.inverseInertiaB,
                        constraint.hingeAxisWorld,
                        impulseLambda);
                }
                else
                {
                    joint.limitImpulse = 0.0;
                }
            }
        }
        else if (joint.description.limitsEnabled)
        {
            // Ball-socket cone/twist limits.
            if (constraint.limitMass > 0.0)
            {
                const double coneViolation = constraint.swingAngle - joint.description.coneHalfAngleRadians;
                if (coneViolation > 0.0)
                {
                    const double relativeSpeed
                        = math::dot(bodyB.angularVelocity - bodyA.angularVelocity, constraint.swingAxisWorld);
                    const double bias
                        = (settings.jointBaumgarteFactor * inverseDeltaTime) * std::clamp(coneViolation, 0.0, 0.2);
                    double impulseLambda    = -(relativeSpeed + bias) * constraint.limitMass;
                    impulseLambda           = std::min(impulseLambda, 0.0);
                    const double oldImpulse = joint.limitImpulse;
                    joint.limitImpulse      = oldImpulse + impulseLambda;
                    impulseLambda           = joint.limitImpulse - oldImpulse;
                    applyAngularPairImpulse(bodyA,
                        bodyB,
                        constraint.inverseInertiaA,
                        constraint.inverseInertiaB,
                        constraint.swingAxisWorld,
                        impulseLambda);
                }
                const double twistHalf = joint.description.twistHalfAngleRadians;
                if (constraint.twistAngle < -twistHalf || constraint.twistAngle > twistHalf)
                {
                    const double violation = constraint.twistAngle > twistHalf ? constraint.twistAngle - twistHalf
                                                                               : constraint.twistAngle + twistHalf;
                    const double relativeSpeed
                        = math::dot(bodyB.angularVelocity - bodyA.angularVelocity, constraint.hingeAxisWorld);
                    const double bias
                        = (settings.jointBaumgarteFactor * inverseDeltaTime) * std::clamp(violation, -0.2, 0.2);
                    double impulseLambda    = -(relativeSpeed + bias) * constraint.limitMass;
                    impulseLambda           = constraint.twistAngle > twistHalf ? std::max(impulseLambda, 0.0)
                                                                                : std::min(impulseLambda, 0.0);
                    const double oldImpulse = joint.limitImpulse;
                    joint.limitImpulse      = oldImpulse + impulseLambda;
                    impulseLambda           = joint.limitImpulse - oldImpulse;
                    applyAngularPairImpulse(bodyA,
                        bodyB,
                        constraint.inverseInertiaA,
                        constraint.inverseInertiaB,
                        constraint.hingeAxisWorld,
                        impulseLambda);
                }
            }
        }
    }
}

void solveContactPositionIteration(
    World &world, std::vector<ContactConstraint> &constraints, const StepContext &context)
{
    (void)context;
    const SimulationSettings &settings = world.settings;
    for (ContactConstraint &constraint : constraints)
    {
        Body &bodyA = world.bodies[constraint.bodyIndexA];
        Body &bodyB = world.bodies[constraint.bodyIndexB];
        for (std::uint32_t pointIndex = 0; pointIndex < constraint.pointCount; ++pointIndex)
        {
            const ContactConstraint::Point &point = constraint.points[pointIndex];

            // Current anchors from the moved transforms, with refreshed
            // lever arms (earlier iterations in this pass move the bodies).
            const math::Vec3 rA      = math::rotate(bodyA.transform.rotation, point.localAnchorA);
            const math::Vec3 rB      = math::rotate(bodyB.transform.rotation, point.localAnchorB);
            const math::Vec3 anchorA = bodyA.transform.position + rA;
            const math::Vec3 anchorB = bodyB.transform.position + rB;
            // The pass corrects toward minus the slop: resting bodies keep
            // contact-slope penetration, so manifolds persist and the
            // velocity constraint keeps holding instead of detaching.
            const double C = math::dot(anchorA - anchorB, point.normal) + settings.contactSlop;
            if (C >= 0.0)
            {
                continue;
            }

            // Positional displacement along the normal; the effective mass
            // reuses the velocity solve's scalar (the Jacobian is unchanged),
            // so the displacement carries meters as its unit.
            double positionalLambda = -point.normalMass * C * settings.positionCorrectionFactor;
            positionalLambda
                = std::clamp(positionalLambda, -settings.maximumLinearCorrection, settings.maximumLinearCorrection);
            if (positionalLambda == 0.0)
            {
                continue;
            }
            const math::Vec3 displacement = point.normal * positionalLambda;

            bodyA.transform.position += displacement * bodyA.inverseMass;
            bodyB.transform.position -= displacement * bodyB.inverseMass;
            const math::Vec3 angularA
                = math::multiplyMatrixVector(constraint.inverseInertiaA, math::cross(rA, displacement));
            const math::Vec3 angularB
                = math::multiplyMatrixVector(constraint.inverseInertiaB, math::cross(rB, displacement));
            applyAngularDisplacement(bodyA, angularA);
            applyAngularDisplacement(bodyB, angularB * -1.0);
        }
    }
}

} // namespace engine::physics_rigid
