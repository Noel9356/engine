// world.cpp - the fixed-step pipeline and query API. Every step runs the
// same deterministic sequence: force integration, spatial-hash broadphase,
// narrowphase with persistent-manifold warm starting, sequential impulse
// iterations over contacts and joints, position integration, island-based
// sleeping, and statistics. Queries walk all bodies with AABB prechecks so
// they stay exact for any scene size the configured budgets allow.

#include "world.hpp"

#include "broadphase.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace engine::physics_rigid
{

// Helpers defined further down; destroyBodyFromWorld reports trigger exits.
bool destroyTriggerFromWorld(World &world, TriggerHandle handle);
void reportTriggerExit(World &world, Trigger &trigger, BodyHandle body);

namespace
{

constexpr double kEpsilon          = 1.0e-9;
constexpr double kMaximumStepDelta = 0.1;

void clampVelocity(math::Vec3 &velocity, double maximumMagnitude)
{
    if (maximumMagnitude <= 0.0)
    {
        return;
    }
    const double magnitudeSquared = math::lengthSquared(velocity);
    if (magnitudeSquared > maximumMagnitude * maximumMagnitude)
    {
        velocity = velocity * (maximumMagnitude / std::sqrt(magnitudeSquared));
    }
}

// Island union-find with path halving.
std::size_t islandFind(std::vector<std::size_t> &parent, std::size_t node)
{
    while (parent[node] != node)
    {
        parent[node] = parent[parent[node]];
        node         = parent[node];
    }
    return node;
}

void islandUnion(std::vector<std::size_t> &parent, std::size_t left, std::size_t right)
{
    const std::size_t rootLeft  = islandFind(parent, left);
    const std::size_t rootRight = islandFind(parent, right);
    if (rootLeft != rootRight)
    {
        parent[rootLeft] = rootRight;
    }
}

// Resolves the simulation-LOD iteration budget tables into effective
// per-band budgets: a zero entry derives from the configured iteration
// count (Full keeps it, each lower band halves, floored at one) and every
// entry clamps to [1, configured]. Called once per world creation, so the
// per-step solver reads plain clamped numbers.
void resolveLodIterationBudgets(SimulationSettings &settings)
{
    const std::uint32_t derivedVelocity[engine::util::kSimulationFidelityBandCount] = {1u,
        1u,
        settings.velocityIterations / 4u < 1u ? 1u : settings.velocityIterations / 4u,
        settings.velocityIterations / 2u < 1u ? 1u : settings.velocityIterations / 2u,
        settings.velocityIterations};
    const std::uint32_t derivedPosition[engine::util::kSimulationFidelityBandCount] = {1u,
        1u,
        settings.positionIterations / 4u < 1u ? 1u : settings.positionIterations / 4u,
        settings.positionIterations / 2u < 1u ? 1u : settings.positionIterations / 2u,
        settings.positionIterations};
    for (std::uint32_t band = 0; band < engine::util::kSimulationFidelityBandCount; ++band)
    {
        const std::uint32_t velocityBudget = settings.lodVelocityIterations[band] != 0u
                ? settings.lodVelocityIterations[band]
                : derivedVelocity[band];
        const std::uint32_t positionBudget = settings.lodPositionIterations[band] != 0u
                ? settings.lodPositionIterations[band]
                : derivedPosition[band];
        settings.lodVelocityIterations[band] = velocityBudget > settings.velocityIterations
                || velocityBudget < 1u
            ? settings.velocityIterations
            : velocityBudget;
        settings.lodPositionIterations[band] = positionBudget > settings.positionIterations || positionBudget < 1u
            ? settings.positionIterations
            : positionBudget;
    }
}

} // namespace

void StateSnapshot::capture(const World &world)
{
    positions.resize(world.bodies.size());
    rotations.resize(world.bodies.size());
    linearVelocities.resize(world.bodies.size());
    angularVelocities.resize(world.bodies.size());
    for (std::size_t index = 0; index < world.bodies.size(); ++index)
    {
        positions[index]         = world.bodies[index].transform.position;
        rotations[index]         = world.bodies[index].transform.rotation;
        linearVelocities[index]  = world.bodies[index].linearVelocity;
        angularVelocities[index] = world.bodies[index].angularVelocity;
    }
}

void StateSnapshot::restore(World &world) const
{
    for (std::size_t index = 0; index < world.bodies.size(); ++index)
    {
        world.bodies[index].transform.position = positions[index];
        world.bodies[index].transform.rotation = rotations[index];
        world.bodies[index].linearVelocity     = linearVelocities[index];
        world.bodies[index].angularVelocity    = angularVelocities[index];
    }
}

void recomputeMassProperties(Body &body)
{
    if (body.kind != BodyKind::Dynamic)
    {
        body.mass                = 0.0;
        body.inverseMass         = 0.0;
        body.inertiaLocal        = math::Vec3{};
        body.inverseInertiaLocal = math::Vec3{};
        return;
    }
    double     mass = 0.0;
    math::Vec3 inertia{};
    const bool computed = body.densitySource > 0.0
                            ? computeMassProperties(body.shape, body.densitySource, 0.0, mass, inertia)
                            : computeMassProperties(body.shape, 0.0, body.explicitMassSource, mass, inertia);
    if (!computed || !(mass > 0.0))
    {
        return; // keep the previous properties; creation validated them
    }
    body.mass                = mass;
    body.inverseMass         = 1.0 / mass;
    body.inertiaLocal        = inertia;
    body.inverseInertiaLocal = math::Vec3{
        inertia.x > 0.0 ? 1.0 / inertia.x : 0.0,
        inertia.y > 0.0 ? 1.0 / inertia.y : 0.0,
        inertia.z > 0.0 ? 1.0 / inertia.z : 0.0};
}

void refreshBodyAabb(Body &body)
{
    computeShapeAabb(body.shape, body.transform, body.aabbMinimum, body.aabbMaximum);
}

bool validateBodyDescription(const World &world, const SimulationSettings &settings, const BodyDescription &description,
    Body &outBody, PhysicsError *error)
{
    const auto setFailure = [&](PhysicsError value)
    {
        if (error != nullptr)
        {
            *error = value;
        }
        return false;
    };

    if (world.bodies.size() >= settings.maximumBodies)
    {
        return setFailure(PhysicsError::CapacityExceeded);
    }
    Body body;
    if (!makeShape(description.shape, body.shape))
    {
        return setFailure(PhysicsError::InvalidDescription);
    }
    if (body.shape.kind == ShapeKind::Plane && description.kind != BodyKind::Static)
    {
        return setFailure(PhysicsError::Unsupported);
    }
    if (!math::isFinite(description.transform.position) || !math::isFinite(description.transform.rotation))
    {
        return setFailure(PhysicsError::InvalidDescription);
    }
    body.transform.position = description.transform.position;
    body.transform.rotation = math::normalize(description.transform.rotation);
    if (math::dot(body.transform.rotation, body.transform.rotation) < 1.0e-12)
    {
        return setFailure(PhysicsError::InvalidDescription);
    }
    body.kind                 = description.kind;
    // Material finiteness first: std::clamp keeps NaN (both comparisons are
    // false), so a non-finite material reaching the clamp would poison every
    // contact this body takes part in (the solver mixes friction with a
    // square root). The setBodyMaterial entry point checks finiteness; the
    // creation path must match it.
    if (!std::isfinite(description.material.friction) || !std::isfinite(description.material.restitution)
        || !std::isfinite(description.material.rollingResistance))
    {
        return setFailure(PhysicsError::InvalidDescription);
    }
    body.material             = description.material;
    body.material.friction    = std::clamp(body.material.friction, 0.0, 4.0);
    body.material.restitution = std::clamp(body.material.restitution, 0.0, 1.0);
    body.material.rollingResistance = std::clamp(body.material.rollingResistance, 0.0, 1.0);
    if (!std::isfinite(description.linearDamping) || description.linearDamping < 0.0
        || !std::isfinite(description.angularDamping) || description.angularDamping < 0.0)
    {
        return setFailure(PhysicsError::InvalidDescription);
    }
    body.linearDamping   = description.linearDamping;
    body.angularDamping  = description.angularDamping;
    body.sleepingEnabled = description.sleepingEnabled;
    body.gravityEnabled  = description.gravityEnabled;
    body.continuousCollisionEnabled = description.continuousCollisionEnabled;
    body.categoryBits    = description.categoryBits != 0u ? description.categoryBits : 1u;
    body.maskBits        = description.maskBits;
    body.userTag         = description.userTag;
    body.sleepTimer      = 0.0;

    // Only dynamic bodies carry mass; static and kinematic bodies stay at
    // zero inverse mass regardless of the description's density fields.
    if (body.kind == BodyKind::Dynamic)
    {
        double     mass = 0.0;
        math::Vec3 inertia{};
        if (description.density > 0.0)
        {
            if (!std::isfinite(description.density)
                || !computeMassProperties(body.shape, description.density, 0.0, mass, inertia))
            {
                return setFailure(PhysicsError::InvalidDescription);
            }
        }
        else if (std::isfinite(description.mass) && description.mass > 0.0
                 && computeMassProperties(body.shape, 0.0, description.mass, mass, inertia))
        {
            // explicit mass with shape-derived inertia
        }
        else
        {
            return setFailure(PhysicsError::InvalidDescription);
        }
        body.mass                = mass;
        body.inverseMass         = mass > 0.0 ? 1.0 / mass : 0.0;
        body.inertiaLocal        = inertia;
        body.inverseInertiaLocal = math::Vec3{
            inertia.x > 0.0 ? 1.0 / inertia.x : 0.0,
            inertia.y > 0.0 ? 1.0 / inertia.y : 0.0,
            inertia.z > 0.0 ? 1.0 / inertia.z : 0.0};
        body.densitySource      = description.density > 0.0 ? description.density : 0.0;
        body.explicitMassSource = description.density > 0.0 ? 0.0 : description.mass;
    }
    body.awake = body.kind != BodyKind::Static;

    // Rolling resistance acts on the curved shapes only; the effective radius
    // is the contact distance from the rolling axis.
    body.rollingRadius = body.shape.kind == ShapeKind::Sphere      ? body.shape.sphereRadius
                         : body.shape.kind == ShapeKind::Capsule ? body.shape.capsuleRadius
                         : body.shape.kind == ShapeKind::Cylinder ? body.shape.cylinderRadius
                                                                 : 0.0;

    refreshBodyAabb(body);
    outBody = body;
    if (error != nullptr)
    {
        *error = PhysicsError::None;
    }
    return true;
}

// One integration pass at the given (already validated) delta: the whole
// pipeline without statistics or substep bookkeeping. Divergence (non-finite
// state) only reports failure here; the snapshot rollback belongs to the
// caller (stepWorld), which captures once for the whole step so a failed
// substep cannot leave earlier substeps committed.
bool stepPipeline(World &world, double fixedDeltaSeconds, StepScratch &scratch)
{
    const SimulationSettings &settings = world.settings;
    StepContext               context{fixedDeltaSeconds, 1.0 / fixedDeltaSeconds};

    // Simulation LOD: every awake dynamic body resolves its fidelity band
    // from the importance score (promotions immediate, demotions behind the
    // hysteresis margin). Sleeping bodies keep their band; it applies again
    // when they wake. With lod_enabled off every body stays Full.
    if (settings.lod.enabled)
    {
        for (Body &body : world.bodies)
        {
            if (body.kind != BodyKind::Dynamic)
            {
                continue;
            }
            body.lodBand = engine::util::simulationFidelityForImportance(
                settings.lod.thresholds, body.importance, body.lodBand);
        }
    }

    // Force and gravity integration for awake dynamic bodies.
    for (Body &body : world.bodies)
    {
        if (body.kind != BodyKind::Dynamic || !body.awake)
        {
            continue;
        }
        if (body.gravityEnabled)
        {
            body.linearVelocity += world.gravity * fixedDeltaSeconds;
        }
        if (math::lengthSquared(body.forceAccumulator) > 0.0)
        {
            body.linearVelocity += body.forceAccumulator * (body.inverseMass * fixedDeltaSeconds);
        }
        if (math::lengthSquared(body.torqueAccumulator) > 0.0)
        {
            const math::Mat3 inverseInertia = computeWorldInverseInertia(body);
            body.angularVelocity
                += math::multiplyMatrixVector(inverseInertia, body.torqueAccumulator) * fixedDeltaSeconds;
        }
        if (body.linearDamping > 0.0)
        {
            body.linearVelocity *= 1.0 / (1.0 + fixedDeltaSeconds * body.linearDamping);
        }
        if (body.angularDamping > 0.0)
        {
            body.angularVelocity *= 1.0 / (1.0 + fixedDeltaSeconds * body.angularDamping);
        }
        clampVelocity(body.linearVelocity, settings.maximumLinearVelocity);
        clampVelocity(body.angularVelocity, settings.maximumAngularVelocity);
    }

    // Kinematic bodies integrate their velocities without forces.
    for (Body &body : world.bodies)
    {
        if (body.kind == BodyKind::Kinematic)
        {
            clampVelocity(body.linearVelocity, settings.maximumLinearVelocity);
            clampVelocity(body.angularVelocity, settings.maximumAngularVelocity);
        }
    }

    // Broadphase over solid bodies; planes join narrowphase separately.
    collectBroadphasePairs(world, settings.spatialHashCellSize, scratch.broadphase, scratch.pairs);
    world.broadphasePairs = scratch.pairs;

    // Narrowphase into the persistent manifold map.
    for (const auto &pair : scratch.pairs)
    {
        Body           &bodyA = world.bodies[pair.first];
        Body           &bodyB = world.bodies[pair.second];
        ContactManifold fresh;
        const bool      touching
            = collideShapes(bodyA.shape, bodyA.transform, bodyB.shape, bodyB.transform, settings.contactMargin, fresh);
        if (!touching)
        {
            continue;
        }
        const std::uint64_t pairKey   = makePairKey(bodyA.handle, bodyB.handle);
        PersistedManifold  &persisted = world.manifolds[pairKey];

        // Match fresh points to persisted points for impulse carry-over.
        PersistedContactPoint carried[4];
        std::uint32_t         carriedCount = persisted.pointCount;
        for (std::uint32_t pointIndex = 0; pointIndex < carriedCount; ++pointIndex)
        {
            carried[pointIndex] = persisted.points[pointIndex];
        }
        persisted.pairKey        = pairKey;
        persisted.bodyIndexA     = pair.first;
        persisted.bodyIndexB     = pair.second;
        persisted.normal         = fresh.normal;
        persisted.ageSteps       = 0;
        persisted.pointCount     = 0;
        const double matchRadius = std::max(settings.contactSlop * 4.0, 0.02);
        // The match test compares squared distances, so the threshold is
        // squared as well: comparing the squared distance against the linear
        // radius would match anchors up to sqrt(matchRadius) meters apart
        // (sqrt(0.02) ~ 0.14 m at the defaults) and leak one contact point's
        // impulses into a different point.
        const double matchRadiusSquared = matchRadius * matchRadius;
        for (std::uint32_t freshIndex = 0; freshIndex < fresh.pointCount; ++freshIndex)
        {
            PersistedContactPoint target;
            target.anchorA            = fresh.points[freshIndex].pointA;
            target.anchorB            = fresh.points[freshIndex].pointB;
            target.normalImpulse      = 0.0;
            target.tangentImpulse1    = 0.0;
            target.tangentImpulse2    = 0.0;
            double       bestDistance = matchRadiusSquared;
            std::int32_t bestCarried  = -1;
            for (std::uint32_t candidateIndex = 0; candidateIndex < carriedCount; ++candidateIndex)
            {
                const double distanceA = math::lengthSquared(carried[candidateIndex].anchorA - target.anchorA);
                const double distanceB = math::lengthSquared(carried[candidateIndex].anchorB - target.anchorB);
                const double distance  = std::max(distanceA, distanceB);
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    bestCarried  = static_cast<std::int32_t>(candidateIndex);
                }
            }
            if (bestCarried >= 0)
            {
                target.normalImpulse   = carried[bestCarried].normalImpulse;
                target.tangentImpulse1 = carried[bestCarried].tangentImpulse1;
                target.tangentImpulse2 = carried[bestCarried].tangentImpulse2;
                target.tangent1        = carried[bestCarried].tangent1;
                target.tangent2        = carried[bestCarried].tangent2;
                carried[bestCarried]   = carried[--carriedCount];
            }
            else
            {
                target.tangent1 = math::Vec3{};
                target.tangent2 = math::Vec3{};
            }
            persisted.points[persisted.pointCount++] = target;
        }
    }

    // Plane narrowphase: every solid body against every plane, merged into
    // the same persistent-manifold store with canonical pair ordering.
    for (const BodyHandle planeHandle : world.planeBodyHandles)
    {
        const auto planeIndexIterator = world.bodyIndexByHandle.find(planeHandle);
        if (planeIndexIterator == world.bodyIndexByHandle.end())
        {
            continue;
        }
        const Body &planeBody = world.bodies[planeIndexIterator->second];
        for (std::size_t bodyIndex = 0; bodyIndex < world.bodies.size(); ++bodyIndex)
        {
            Body &body = world.bodies[bodyIndex];
            if (body.shape.kind == ShapeKind::Plane || body.kind == BodyKind::Static || !body.awake)
            {
                continue;
            }
            // Cheap swept-AABB rejection before the shape narrowphase: a
            // body whose velocity-extended bounds cannot reach the plane's
            // speculative margin this step skips the full collision test.
            // The bounds come from the previous step's refresh, so the
            // per-axis velocity extension keeps the test conservative.
            {
                math::Vec3 sweptMinimum = body.aabbMinimum;
                math::Vec3 sweptMaximum = body.aabbMaximum;
                const math::Vec3 travel = body.linearVelocity * fixedDeltaSeconds;
                for (std::uint32_t axis = 0; axis < 3; ++axis)
                {
                    const double component = math::component(travel, axis);
                    if (component > 0.0)
                    {
                        math::component(sweptMaximum, axis) += component;
                    }
                    else
                    {
                        math::component(sweptMinimum, axis) += component;
                    }
                }
                const math::Vec3 halfExtent
                    = (sweptMaximum - sweptMinimum) * 0.5;
                const math::Vec3 center = (sweptMaximum + sweptMinimum) * 0.5;
                const double extentAlongNormal = std::abs(planeBody.shape.planeNormal.x) * halfExtent.x
                    + std::abs(planeBody.shape.planeNormal.y) * halfExtent.y
                    + std::abs(planeBody.shape.planeNormal.z) * halfExtent.z;
                const double centerDistance = math::dot(center, planeBody.shape.planeNormal) - planeBody.shape.planeOffset;
                if (centerDistance - extentAlongNormal > settings.contactMargin)
                {
                    continue;
                }
            }
            ContactManifold fresh;
            if (!collidePlaneSolid(planeBody.shape.planeNormal,
                    planeBody.shape.planeOffset,
                    body.shape,
                    body.transform,
                    settings.contactMargin,
                    fresh))
            {
                continue;
            }
            // The solver convention requires the manifold normal to point
            // from body B toward body A of the canonical pair. When the plane
            // takes the A slot (it is created first, so index 0), the
            // collidePlaneSolid result - which reports plane-as-B normals -
            // must flip; when the solid body takes the A slot the result
            // already matches.
            const std::size_t planeIndex = planeIndexIterator->second;
            if (planeIndex < bodyIndex)
            {
                fresh.normal = fresh.normal * -1.0;
                for (std::uint32_t pointIndex = 0; pointIndex < fresh.pointCount; ++pointIndex)
                {
                    std::swap(fresh.points[pointIndex].pointA, fresh.points[pointIndex].pointB);
                }
            }
            const std::uint64_t pairKey   = makePairKey(planeBody.handle, body.handle);
            PersistedManifold  &persisted = world.manifolds[pairKey];
            persisted.pairKey             = pairKey;
            persisted.bodyIndexA          = std::min(planeIndex, bodyIndex);
            persisted.bodyIndexB          = std::max(planeIndex, bodyIndex);
            persisted.normal              = fresh.normal;
            persisted.ageSteps            = 0;
            persisted.pointCount          = 0;
            for (std::uint32_t pointIndex = 0; pointIndex < fresh.pointCount; ++pointIndex)
            {
                PersistedContactPoint target;
                target.anchorA                           = fresh.points[pointIndex].pointA;
                target.anchorB                           = fresh.points[pointIndex].pointB;
                target.tangent1                          = math::Vec3{};
                target.tangent2                          = math::Vec3{};
                persisted.points[persisted.pointCount++] = target;
            }
        }
    }

    // Age and drop stale manifolds (pairs that separated).
    for (auto manifoldIterator = world.manifolds.begin(); manifoldIterator != world.manifolds.end();)
    {
        PersistedManifold &persisted = manifoldIterator->second;
        if (persisted.ageSteps == 0)
        {
            ++persisted.ageSteps;
            ++manifoldIterator;
        }
        else
        {
            manifoldIterator = world.manifolds.erase(manifoldIterator);
        }
    }

    // Constraint preparation and warm starting.
    prepareContactConstraints(world, context, scratch.contacts);
    prepareJointConstraints(world, context, scratch.jointConstraints);
    warmStartContacts(world, scratch.contacts, settings.warmStarting);
    warmStartJoints(world, scratch.jointConstraints, settings.warmStarting);

    // Velocity iterations: contacts then joints, in fixed order. Each
    // constraint carries its simulation-LOD iteration budget and drops out
    // of the passes once the budget is spent, so reduced-fidelity islands
    // stop paying for iterations they no longer refine.
    for (std::uint32_t iteration = 0; iteration < settings.velocityIterations; ++iteration)
    {
        solveContactVelocityIteration(world, scratch.contacts, context, iteration);
        solveJointVelocityIteration(world, scratch.jointConstraints, context, iteration);
    }

    // Position iterations remove the remaining contact penetration directly
    // (split impulse scheme - no bias velocities, no bounce), on the same
    // per-constraint budgets.
    for (std::uint32_t iteration = 0; iteration < settings.positionIterations; ++iteration)
    {
        solveContactPositionIteration(world, scratch.contacts, context, iteration);
    }

    // Persist the solved tangent basis so next step's warm start applies the
    // same frame, then integrate positions.
    for (const ContactConstraint &constraint : scratch.contacts)
    {
        for (std::uint32_t pointIndex = 0; pointIndex < constraint.pointCount; ++pointIndex)
        {
            const ContactConstraint::Point &point = constraint.points[pointIndex];
            point.persisted->tangent1             = point.tangent1;
            point.persisted->tangent2             = point.tangent2;
        }
    }

    // Continuous collision detection: capture the pre-integration transforms
    // of the CCD-enabled dynamic bodies so the post-integration sweep can
    // clamp them at the first contact with non-dynamic geometry.
    std::vector<math::Vec3> &ccdPreviousPositions = scratch.ccdPreviousPositions;
    ccdPreviousPositions.clear();
    for (const Body &body : world.bodies)
    {
        if (body.kind == BodyKind::Dynamic && body.awake && body.continuousCollisionEnabled)
        {
            ccdPreviousPositions.push_back(body.transform.position);
        }
    }

    for (Body &body : world.bodies)
    {
        if (body.kind == BodyKind::Static)
        {
            body.forceAccumulator  = math::Vec3{};
            body.torqueAccumulator = math::Vec3{};
            continue;
        }
        if (body.kind == BodyKind::Kinematic || body.awake)
        {
            body.transform.position += body.linearVelocity * fixedDeltaSeconds;
            const math::Vec3       angular = body.angularVelocity * fixedDeltaSeconds;
            const math::Quaternion delta{angular.x * 0.5, angular.y * 0.5, angular.z * 0.5, 0.0};
            body.transform.rotation = math::normalize(body.transform.rotation + (delta * body.transform.rotation));
            refreshBodyAabb(body);
        }
        body.forceAccumulator  = math::Vec3{};
        body.torqueAccumulator = math::Vec3{};
    }

    // CCD sweep for the fast movers: the body's bounding-sphere proxy is cast
    // from its pre-integration position to the integrated one against every
    // static/kinematic solid body and plane; a hit clamps the position at the
    // contact (minus the speculative margin) and cancels the approach speed
    // with the body's restitution, so nothing tunnels through thin geometry.
    {
        std::size_t ccdIndex = 0;
        for (Body &body : world.bodies)
        {
            if (body.kind != BodyKind::Dynamic || !body.awake || !body.continuousCollisionEnabled)
            {
                continue;
            }
            const math::Vec3 previousPosition = ccdPreviousPositions[ccdIndex++];
            const math::Vec3 delta            = body.transform.position - previousPosition;
            const double     travelled        = math::length(delta);
            const double     proxyRadius      = computeShapeBoundingRadius(body.shape);
            if (travelled <= settings.contactMargin || proxyRadius <= 0.0
                || travelled < settings.ccdVelocityThreshold * fixedDeltaSeconds)
            {
                continue;
            }
            const math::Vec3 direction = delta * (1.0 / travelled);
            math::Vec3       sweptMinimum{previousPosition.x - proxyRadius,
                previousPosition.y - proxyRadius,
                previousPosition.z - proxyRadius};
            math::Vec3       sweptMaximum{previousPosition.x + proxyRadius,
                previousPosition.y + proxyRadius,
                previousPosition.z + proxyRadius};
            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                const double componentValue = math::component(delta, axis);
                if (componentValue > 0.0)
                {
                    math::component(sweptMaximum, axis) += componentValue;
                }
                else
                {
                    math::component(sweptMinimum, axis) += componentValue;
                }
            }

            double     hitDistance = travelled;
            math::Vec3 hitNormal;
            bool       hit         = false;
            for (const Body &obstacle : world.bodies)
            {
                if (&obstacle == &body || obstacle.kind == BodyKind::Dynamic || obstacle.shape.kind == ShapeKind::Plane)
                {
                    continue;
                }
                if (obstacle.aabbMinimum.x > sweptMaximum.x || obstacle.aabbMaximum.x < sweptMinimum.x
                    || obstacle.aabbMinimum.y > sweptMaximum.y || obstacle.aabbMaximum.y < sweptMinimum.y
                    || obstacle.aabbMinimum.z > sweptMaximum.z || obstacle.aabbMaximum.z < sweptMinimum.z)
                {
                    continue;
                }
                double     distance = 0.0;
                math::Vec3 normal;
                if (sweepSphereSolid(previousPosition,
                        proxyRadius,
                        delta,
                        obstacle.shape,
                        obstacle.transform,
                        distance,
                        normal)
                    && distance < hitDistance)
                {
                    hitDistance = distance;
                    hitNormal   = normal;
                    hit         = true;
                }
            }
            for (const BodyHandle planeHandle : world.planeBodyHandles)
            {
                const auto planeIterator = world.bodyIndexByHandle.find(planeHandle);
                if (planeIterator == world.bodyIndexByHandle.end())
                {
                    continue;
                }
                const Body &planeBody = world.bodies[planeIterator->second];
                double     distance   = 0.0;
                math::Vec3 normal;
                if (sweepSpherePlane(previousPosition,
                        proxyRadius,
                        delta,
                        planeBody.shape.planeNormal,
                        planeBody.shape.planeOffset,
                        distance,
                        normal)
                    && distance < hitDistance)
                {
                    hitDistance = distance;
                    hitNormal   = normal;
                    hit         = true;
                }
            }
            if (hit)
            {
                const double clampedTravel = std::max(hitDistance - settings.contactMargin, 0.0);
                body.transform.position    = previousPosition + direction * clampedTravel;
                const double approachSpeed = math::dot(body.linearVelocity, hitNormal);
                if (approachSpeed < 0.0)
                {
                    const double restitutionSpeed = -approachSpeed > settings.restitutionVelocityThreshold
                                                      ? -body.material.restitution * approachSpeed
                                                      : 0.0;
                    body.linearVelocity += hitNormal * (restitutionSpeed - approachSpeed);
                }
                refreshBodyAabb(body);
            }
        }
    }

    // Non-finite guard: divergence fails this substep (and thereby the whole
    // step, which stepWorld rolls back with its pre-step snapshot).
    bool finite = true;
    for (const Body &body : world.bodies)
    {
        if (!math::isFinite(body.transform.position) || !math::isFinite(body.linearVelocity)
            || !math::isFinite(body.angularVelocity) || !std::isfinite(body.transform.rotation.x)
            || !std::isfinite(body.transform.rotation.y) || !std::isfinite(body.transform.rotation.z)
            || !std::isfinite(body.transform.rotation.w))
        {
            finite = false;
            break;
        }
    }
    if (!finite)
    {
        return false;
    }

    // Trigger evaluation against the integrated transforms; overlap events
    // accumulate for the consumer's next poll. A rolled-back (non-finite)
    // step above never reaches this point, so events always describe a
    // committed step.
    evaluateTriggers(world, scratch.triggerOverlapScratch);

    // Island-based sleeping over the contact and joint graphs.
    scratch.islandParent.resize(world.bodies.size());
    for (std::size_t index = 0; index < world.bodies.size(); ++index)
    {
        scratch.islandParent[index] = index;
    }
    for (const auto &pair : scratch.pairs)
    {
        const Body &bodyA = world.bodies[pair.first];
        const Body &bodyB = world.bodies[pair.second];
        if (bodyA.kind == BodyKind::Dynamic && bodyB.kind == BodyKind::Dynamic)
        {
            islandUnion(scratch.islandParent, pair.first, pair.second);
        }
    }
    for (const Joint &joint : world.joints)
    {
        const auto iteratorA = world.bodyIndexByHandle.find(joint.description.bodyA);
        const auto iteratorB = world.bodyIndexByHandle.find(joint.description.bodyB);
        if (iteratorA == world.bodyIndexByHandle.end() || iteratorB == world.bodyIndexByHandle.end())
        {
            continue;
        }
        const Body &bodyA = world.bodies[iteratorA->second];
        const Body &bodyB = world.bodies[iteratorB->second];
        if (bodyA.kind == BodyKind::Dynamic && bodyB.kind == BodyKind::Dynamic)
        {
            islandUnion(scratch.islandParent, iteratorA->second, iteratorB->second);
        }
    }
    // Sleep timers accumulate only when the whole island is below tolerance.
    // The per-root flags live in scratch vectors indexed by body index (every
    // body index is a union-find root slot), and the touched-roots list resets
    // them without hashing or per-step allocations.
    {
        scratch.islandBelowByRoot.assign(world.bodies.size(), 0u); // 0 untouched, 1 all below, 2 any above
        scratch.islandTouchedRoots.clear();
        for (std::size_t index = 0; index < world.bodies.size(); ++index)
        {
            const Body &body = world.bodies[index];
            if (body.kind != BodyKind::Dynamic)
            {
                continue;
            }
            const std::size_t root = islandFind(scratch.islandParent, index);
            const bool        below = math::lengthSquared(body.linearVelocity)
                                      < settings.linearSleepTolerance * settings.linearSleepTolerance
                                 && math::lengthSquared(body.angularVelocity)
                                        < settings.angularSleepTolerance * settings.angularSleepTolerance;
            const std::uint8_t flag = below ? 1u : 2u;
            if (scratch.islandBelowByRoot[root] == 0u)
            {
                scratch.islandTouchedRoots.push_back(root);
            }
            if (flag == 2u || scratch.islandBelowByRoot[root] == 0u)
            {
                scratch.islandBelowByRoot[root] = flag;
            }
        }
        for (std::size_t index = 0; index < world.bodies.size(); ++index)
        {
            Body &body = world.bodies[index];
            if (body.kind != BodyKind::Dynamic)
            {
                continue;
            }
            const std::size_t root = islandFind(scratch.islandParent, index);
            const bool        allBelow = scratch.islandBelowByRoot[root] == 1u;
            if (allBelow)
            {
                body.sleepTimer += fixedDeltaSeconds;
                if (body.sleepingEnabled && body.sleepTimer >= settings.timeToSleep)
                {
                    body.awake           = false;
                    body.linearVelocity  = math::Vec3{};
                    body.angularVelocity = math::Vec3{};
                }
            }
            else
            {
                body.sleepTimer = 0.0;
                body.awake      = true;
            }
        }
        for (const std::size_t root : scratch.islandTouchedRoots)
        {
            scratch.islandBelowByRoot[root] = 0u;
        }
    }

    // Clear stale manifolds left over from bodies destroyed this step.
    for (auto manifoldIterator = world.manifolds.begin(); manifoldIterator != world.manifolds.end();)
    {
        if (world.bodyIndexByHandle.find(static_cast<BodyHandle>(manifoldIterator->second.pairKey >> 32))
                == world.bodyIndexByHandle.end()
            || world.bodyIndexByHandle.find(static_cast<BodyHandle>(manifoldIterator->second.pairKey & 0xFFFFFFFFull))
                   == world.bodyIndexByHandle.end())
        {
            manifoldIterator = world.manifolds.erase(manifoldIterator);
        }
        else
        {
            ++manifoldIterator;
        }
    }

    return true;
}

bool stepWorld(World &world, double fixedDeltaSeconds, StepScratch &scratch)
{
    const auto stepStart = std::chrono::steady_clock::now();
    if (!(fixedDeltaSeconds > 0.0) || fixedDeltaSeconds > kMaximumStepDelta || !std::isfinite(fixedDeltaSeconds))
    {
        return false;
    }
    // Canonical substepping: coarse caller deltas split into equal internal
    // passes so integration runs on the same step everywhere (deterministic
    // in the delta alone). A diverging substep fails the whole step. The
    // substep count is bounded by the configured maximum, so a coarse delta
    // beyond the bound integrates at a coarser canonical step instead of
    // spending an unbounded pass count.
    std::uint32_t substeps = 1;
    if (world.settings.substepMaximumDelta > 0.0 && fixedDeltaSeconds > world.settings.substepMaximumDelta)
    {
        const double rawCount = std::ceil(fixedDeltaSeconds / world.settings.substepMaximumDelta);
        const double bounded  = std::min(rawCount, static_cast<double>(std::max(world.settings.maximumSubsteps, 1u)));
        substeps              = static_cast<std::uint32_t>(std::max(bounded, 1.0));
    }
    const double substepDelta = fixedDeltaSeconds / static_cast<double>(substeps);
    // Whole-step atomicity: the snapshot is captured once before the first
    // substep and restored if ANY substep fails, so a diverging substep
    // cannot leave its predecessors' integration committed. A per-substep
    // capture would only roll back the failing substep, leaving the world
    // half-stepped despite the failed return.
    scratch.snapshot.capture(world);
    for (std::uint32_t substep = 0; substep < substeps; ++substep)
    {
        if (!stepPipeline(world, substepDelta, scratch))
        {
            scratch.snapshot.restore(world);
            return false;
        }
    }
    world.stepsSimulated += 1;
    world.lastStepMilliseconds
        = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - stepStart).count();
    return true;
}

// Removes a body, its joints, and every manifold that references it; wakes
// nothing (the caller wakes partners when a grabbed body disappears).
bool destroyBodyFromWorld(World &world, BodyHandle handle)
{
    const auto iterator = world.bodyIndexByHandle.find(handle);
    if (iterator == world.bodyIndexByHandle.end())
    {
        return false;
    }
    const std::size_t index = iterator->second;
    world.bodyIndexByHandle.erase(iterator);
    const BodyHandle movedHandle = world.bodies.back().handle;
    if (index + 1 != world.bodies.size())
    {
        world.bodies[index]                  = world.bodies.back();
        world.bodyIndexByHandle[movedHandle] = index;
    }
    world.bodies.pop_back();

    for (std::size_t jointIndex = 0; jointIndex < world.joints.size();)
    {
        if (world.joints[jointIndex].description.bodyA == handle
            || world.joints[jointIndex].description.bodyB == handle)
        {
            destroyJointFromWorld(world, world.joints[jointIndex].handle);
            jointIndex = 0; // index map compaction invalidates positions; restart the scan
            continue;
        }
        ++jointIndex;
    }
    for (auto manifoldIterator = world.manifolds.begin(); manifoldIterator != world.manifolds.end();)
    {
        const std::uint32_t keyA = static_cast<std::uint32_t>(manifoldIterator->second.pairKey >> 32);
        const std::uint32_t keyB = static_cast<std::uint32_t>(manifoldIterator->second.pairKey & 0xFFFFFFFFull);
        if (keyA == handle || keyB == handle)
        {
            manifoldIterator = world.manifolds.erase(manifoldIterator);
        }
        else
        {
            ++manifoldIterator;
        }
    }
    for (std::size_t planeIndex = 0; planeIndex < world.planeBodyHandles.size(); ++planeIndex)
    {
        if (world.planeBodyHandles[planeIndex] == handle)
        {
            world.planeBodyHandles[planeIndex] = world.planeBodyHandles.back();
            world.planeBodyHandles.pop_back();
            break;
        }
    }

    // Triggers attached to the body die with it (their pending events are
    // dropped); every other trigger reports the body's exit.
    for (std::size_t triggerIndex = 0; triggerIndex < world.triggers.size();)
    {
        Trigger &trigger = world.triggers[triggerIndex];
        if (trigger.attachedBody == handle)
        {
            destroyTriggerFromWorld(world, trigger.handle);
            triggerIndex = 0; // index map compaction invalidates positions; restart the scan
            continue;
        }
        ++triggerIndex;
    }
    for (Trigger &trigger : world.triggers)
    {
        const auto overlapIterator = std::find(trigger.overlapping.begin(), trigger.overlapping.end(), handle);
        if (overlapIterator != trigger.overlapping.end())
        {
            trigger.overlapping.erase(overlapIterator);
            reportTriggerExit(world, trigger, handle);
        }
    }
    return true;
}

// Removes one joint record.
bool destroyJointFromWorld(World &world, JointHandle handle)
{
    const auto iterator = world.jointIndexByHandle.find(handle);
    if (iterator == world.jointIndexByHandle.end())
    {
        return false;
    }
    const std::size_t index = iterator->second;
    world.jointIndexByHandle.erase(iterator);
    if (index + 1 != world.joints.size())
    {
        world.joints[index]                                  = world.joints.back();
        world.jointIndexByHandle[world.joints[index].handle] = index;
    }
    world.joints.pop_back();
    return true;
}

// Appends one overlap transition, evicting the oldest event with the
// overflow flag set once the configured queue bound is exceeded.
void appendTriggerEvent(World &world, const TriggerEvent &event)
{
    if (world.settings.maximumTriggerEvents == 0)
    {
        world.triggerEventOverflow = true;
        return;
    }
    while (world.pendingTriggerEvents.size() >= world.settings.maximumTriggerEvents)
    {
        world.pendingTriggerEvents.erase(world.pendingTriggerEvents.begin());
        world.triggerEventOverflow = true;
    }
    world.pendingTriggerEvents.push_back(event);
}

// Reports one body leaving one trigger (destruction path).
void reportTriggerExit(World &world, Trigger &trigger, BodyHandle body)
{
    TriggerEvent event;
    event.kind    = TriggerEventKind::Exit;
    event.trigger = trigger.handle;
    event.body    = body;
    appendTriggerEvent(world, event);
}

bool validateTriggerDescription(
    const World &world, const TriggerDescription &description, Trigger &outTrigger, PhysicsError *error)
{
    const auto setFailure = [&](PhysicsError value)
    {
        if (error != nullptr)
        {
            *error = value;
        }
        return false;
    };
    if (world.triggers.size() >= world.settings.maximumTriggers)
    {
        return setFailure(PhysicsError::CapacityExceeded);
    }
    Trigger trigger;
    if (!makeShape(description.shape, trigger.shape) || trigger.shape.kind == ShapeKind::Plane)
    {
        return setFailure(PhysicsError::InvalidDescription);
    }
    if (!math::isFinite(description.transform.position))
    {
        return setFailure(PhysicsError::InvalidDescription);
    }
    trigger.transform.position = description.transform.position;
    trigger.transform.rotation = math::normalize(description.transform.rotation);
    if (math::dot(trigger.transform.rotation, trigger.transform.rotation) < 1.0e-12)
    {
        return setFailure(PhysicsError::InvalidDescription);
    }
    if (description.attachedBody != invalidBody
        && world.bodyIndexByHandle.find(description.attachedBody) == world.bodyIndexByHandle.end())
    {
        return setFailure(PhysicsError::InvalidHandle);
    }
    trigger.localTransform = trigger.transform;
    trigger.attachedBody   = description.attachedBody;
    trigger.categoryBits   = description.categoryBits != 0u ? description.categoryBits : 1u;
    trigger.maskBits       = description.maskBits;
    trigger.enabled        = description.enabled;
    trigger.userTag        = description.userTag;
    computeShapeAabb(trigger.shape, trigger.transform, trigger.aabbMinimum, trigger.aabbMaximum);
    outTrigger = trigger;
    if (error != nullptr)
    {
        *error = PhysicsError::None;
    }
    return true;
}

bool destroyTriggerFromWorld(World &world, TriggerHandle handle)
{
    const auto iterator = world.triggerIndexByHandle.find(handle);
    if (iterator == world.triggerIndexByHandle.end())
    {
        return false;
    }
    const std::size_t index = iterator->second;
    world.triggerIndexByHandle.erase(iterator);
    if (index + 1 != world.triggers.size())
    {
        world.triggers[index]                                    = world.triggers.back();
        world.triggerIndexByHandle[world.triggers[index].handle] = index;
    }
    world.triggers.pop_back();
    // A destroyed trigger stops reporting: its pending transitions are
    // dropped so consumers never observe events from a dead handle.
    for (std::size_t eventIndex = 0; eventIndex < world.pendingTriggerEvents.size();)
    {
        if (world.pendingTriggerEvents[eventIndex].trigger == handle)
        {
            world.pendingTriggerEvents.erase(world.pendingTriggerEvents.begin()
                + static_cast<std::ptrdiff_t>(eventIndex));
        }
        else
        {
            ++eventIndex;
        }
    }
    return true;
}

void evaluateTriggers(World &world, std::vector<BodyHandle> &overlapScratch)
{
    // Compose attached poses and refresh the trigger AABBs first, so a
    // moving trigger sees one consistent transform for the whole pass.
    for (Trigger &trigger : world.triggers)
    {
        if (trigger.attachedBody != invalidBody)
        {
            const auto bodyIterator = world.bodyIndexByHandle.find(trigger.attachedBody);
            if (bodyIterator != world.bodyIndexByHandle.end())
            {
                const Body &body = world.bodies[bodyIterator->second];
                trigger.transform.position
                    = body.transform.position + math::rotate(body.transform.rotation, trigger.localTransform.position);
                trigger.transform.rotation
                    = math::normalize(body.transform.rotation * trigger.localTransform.rotation);
            }
        }
        computeShapeAabb(trigger.shape, trigger.transform, trigger.aabbMinimum, trigger.aabbMaximum);
    }

    for (Trigger &trigger : world.triggers)
    {
        if (!trigger.enabled)
        {
            for (const BodyHandle body : trigger.overlapping)
            {
                reportTriggerExit(world, trigger, body);
            }
            trigger.overlapping.clear();
            continue;
        }

        overlapScratch.clear();
        for (std::size_t bodyIndex = 0; bodyIndex < world.bodies.size(); ++bodyIndex)
        {
            const Body &body = world.bodies[bodyIndex];
            if (body.shape.kind == ShapeKind::Plane)
            {
                continue;
            }
            if ((trigger.categoryBits & body.maskBits) == 0u || (body.categoryBits & trigger.maskBits) == 0u)
            {
                continue;
            }
            if (body.aabbMinimum.x > trigger.aabbMaximum.x || body.aabbMaximum.x < trigger.aabbMinimum.x
                || body.aabbMinimum.y > trigger.aabbMaximum.y || body.aabbMaximum.y < trigger.aabbMinimum.y
                || body.aabbMinimum.z > trigger.aabbMaximum.z || body.aabbMaximum.z < trigger.aabbMinimum.z)
            {
                continue;
            }
            ContactManifold manifold;
            if (collideShapes(trigger.shape, trigger.transform, body.shape, body.transform, 0.0, manifold))
            {
                overlapScratch.push_back(body.handle);
            }
        }
        std::sort(overlapScratch.begin(), overlapScratch.end());

        // Linear diff over the ascending handle sets: one walk reports the
        // exits and enters in deterministic order.
        std::size_t previousIndex = 0;
        std::size_t currentIndex  = 0;
        while (previousIndex < trigger.overlapping.size() || currentIndex < overlapScratch.size())
        {
            if (previousIndex < trigger.overlapping.size()
                && (currentIndex >= overlapScratch.size()
                    || trigger.overlapping[previousIndex] < overlapScratch[currentIndex]))
            {
                reportTriggerExit(world, trigger, trigger.overlapping[previousIndex]);
                ++previousIndex;
            }
            else if (currentIndex < overlapScratch.size()
                     && (previousIndex >= trigger.overlapping.size()
                         || overlapScratch[currentIndex] < trigger.overlapping[previousIndex]))
            {
                TriggerEvent event;
                event.kind    = TriggerEventKind::Enter;
                event.trigger = trigger.handle;
                event.body    = overlapScratch[currentIndex];
                appendTriggerEvent(world, event);
                ++currentIndex;
            }
            else
            {
                ++previousIndex;
                ++currentIndex;
            }
        }
        trigger.overlapping = overlapScratch;
    }
}

bool rayCastWorldIgnoring(const World &world, const math::Vec3 &origin, const math::Vec3 &direction,
    double maximumDistance, const BodyHandle *ignoredBodies, std::uint32_t ignoredBodyCount, RayHit &outHit)
{
    const double directionLength = math::length(direction);
    if (directionLength <= kEpsilon || maximumDistance <= 0.0)
    {
        return false;
    }
    const math::Vec3 normalizedDirection = direction * (1.0 / directionLength);
    const double     scaledMaximum       = maximumDistance * directionLength;
    bool             hit                 = false;
    double           bestDistance        = scaledMaximum;
    RayHit           bestHit;
    for (const Body &body : world.bodies)
    {
        bool skipBody = false;
        for (std::uint32_t ignoredIndex = 0; ignoredIndex < ignoredBodyCount; ++ignoredIndex)
        {
            if (body.handle == ignoredBodies[ignoredIndex])
            {
                skipBody = true;
                break;
            }
        }
        if (skipBody)
        {
            continue;
        }
        if (body.shape.kind == ShapeKind::Plane)
        {
            double distance = 0.0;
            if (rayCastPlane(
                    body.shape.planeNormal, body.shape.planeOffset, origin, normalizedDirection, bestDistance, distance)
                && distance < bestDistance)
            {
                bestDistance     = distance;
                bestHit.body     = body.handle;
                bestHit.distance = distance / directionLength;
                bestHit.point    = origin + normalizedDirection * distance;
                bestHit.normal   = body.shape.planeNormal
                                 * (math::dot(body.shape.planeNormal, normalizedDirection) > 0.0 ? -1.0 : 1.0);
                hit              = true;
            }
            continue;
        }
        double distance = 0.0;
        bool   touches  = false;
        switch (body.shape.kind)
        {
        case ShapeKind::Sphere:
            touches = rayCastSphere(
                body.transform, body.shape.sphereRadius, origin, normalizedDirection, bestDistance, distance);
            break;
        case ShapeKind::Box:
            touches = rayCastBox(
                body.transform, body.shape.boxHalfExtents, origin, normalizedDirection, bestDistance, distance);
            break;
        case ShapeKind::Capsule:
            touches = rayCastCapsule(body.transform,
                body.shape.capsuleRadius,
                body.shape.capsuleHalfLength,
                origin,
                normalizedDirection,
                bestDistance,
                distance);
            break;
        case ShapeKind::Cylinder:
            touches = rayCastCylinder(body.transform,
                body.shape.cylinderRadius,
                body.shape.cylinderHalfHeight,
                origin,
                normalizedDirection,
                bestDistance,
                distance);
            break;
        case ShapeKind::Plane:
        default:
            break;
        }
        if (touches && distance < bestDistance)
        {
            bestDistance     = distance;
            bestHit.body     = body.handle;
            bestHit.distance = distance / directionLength;
            bestHit.point    = origin + normalizedDirection * distance;
            // Normal from the shape surface toward the ray origin.
            const math::Vec3 surfaceDelta = bestHit.point - body.transform.position;
            math::Vec3       candidate    = math::normalized(surfaceDelta);
            if (body.shape.kind == ShapeKind::Box)
            {
                const math::Vec3 localPoint = math::inverseRotate(body.transform.rotation, surfaceDelta);
                const double     components[3]
                    = {std::abs(localPoint.x) / body.shape.boxHalfExtents[0],
                        std::abs(localPoint.y) / body.shape.boxHalfExtents[1],
                        std::abs(localPoint.z) / body.shape.boxHalfExtents[2]};
                std::uint32_t faceAxis = components[0] > components[1] ? (components[0] > components[2] ? 0u : 2u)
                                                                       : (components[1] > components[2] ? 1u : 2u);
                math::Vec3    localNormal{};
                math::component(localNormal, faceAxis)
                    = math::component(localPoint, faceAxis) >= 0.0 ? 1.0 : -1.0;
                candidate                  = math::rotate(body.transform.rotation, localNormal);
            }
            else if (body.shape.kind == ShapeKind::Capsule)
            {
                math::Vec3 endpointA;
                math::Vec3 endpointB;
                computeCapsuleSegment(body.shape, body.transform, endpointA, endpointB);
                const math::Vec3 axis          = math::normalized(endpointB - endpointA);
                const double     segmentLength = math::length(endpointB - endpointA);
                const double projection = std::clamp(math::dot(bestHit.point - endpointA, axis), 0.0, segmentLength);
                candidate               = math::normalized(bestHit.point - (endpointA + axis * projection));
            }
            else if (body.shape.kind == ShapeKind::Cylinder)
            {
                // The projection math needs the UNIT axis: the rotated local
                // +Y half-height vector has length h, so its raw dot products
                // scale by h, the clamp bounds stop measuring meters, and the
                // cap-normal fallback would return a length-h vector.
                const math::Vec3 axis
                    = math::normalized(math::rotate(body.transform.rotation, math::Vec3{0.0, body.shape.cylinderHalfHeight, 0.0}));
                const double projection = std::clamp(math::dot(bestHit.point - body.transform.position, axis),
                    -body.shape.cylinderHalfHeight,
                    body.shape.cylinderHalfHeight);
                math::Vec3 axisPoint   = body.transform.position + axis * projection;
                math::Vec3 radial      = bestHit.point - axisPoint;
                const double radialLength = math::length(radial);
                candidate = radialLength > kEpsilon && radialLength > body.shape.cylinderRadius * 0.5
                    ? radial * (1.0 / radialLength)
                    : axis * (projection >= 0.0 ? 1.0 : -1.0);
            }
            bestHit.normal = math::dot(candidate, normalizedDirection) > 0.0 ? candidate * -1.0 : candidate;
            hit            = true;
        }
    }
    if (!hit)
    {
        return false;
    }
    outHit = bestHit;
    return true;
}

// Unfiltered closest hit: the plain ray cast against every body.
bool rayCastWorld(
    const World &world, const math::Vec3 &origin, const math::Vec3 &direction, double maximumDistance, RayHit &outHit)
{
    return rayCastWorldIgnoring(world, origin, direction, maximumDistance, nullptr, 0, outHit);
}

bool sphereCastWorld(const World &world, const math::Vec3 &start, const math::Vec3 &delta, double radius, SweepHit &outHit)
{
    if (!(radius > 0.0) || math::lengthSquared(delta) <= kEpsilon)
    {
        return false;
    }
    const double length = math::length(delta);
    math::Vec3   direction = delta * (1.0 / length);
    bool         hit       = false;
    double       bestDistance = length;
    math::Vec3   bestNormal;
    BodyHandle   bestBody  = invalidBody;
    for (const Body &body : world.bodies)
    {
        if (body.shape.kind == ShapeKind::Plane)
        {
            double     distance = 0.0;
            math::Vec3 normal;
            if (sweepSpherePlane(start, radius, delta, body.shape.planeNormal, body.shape.planeOffset, distance, normal)
                && distance < bestDistance)
            {
                bestDistance = distance;
                bestNormal   = normal;
                bestBody     = body.handle;
                hit          = true;
            }
            continue;
        }
        double     distance = 0.0;
        math::Vec3 normal;
        if (sweepSphereSolid(start, radius, delta, body.shape, body.transform, distance, normal)
            && distance < bestDistance)
        {
            bestDistance = distance;
            bestNormal   = normal;
            bestBody     = body.handle;
            hit          = true;
        }
    }
    if (!hit)
    {
        return false;
    }
    SweepHit result;
    result.body     = bestBody;
    result.distance = bestDistance;
    result.point    = start + direction * bestDistance;
    result.normal   = bestNormal;
    outHit          = result;
    return true;
}

bool collideSphereWorld(const World &world, const math::Vec3 &center, double radius, math::Vec3 &outNormal,
    double &outDepth, BodyHandle &outBody)
{
    if (!(radius > 0.0) || !math::isFinite(center))
    {
        return false;
    }
    bool       hit       = false;
    double     bestDepth = 0.0;
    math::Vec3 bestNormal;
    BodyHandle bestBody = invalidBody;
    for (const Body &body : world.bodies)
    {
        if (body.shape.kind == ShapeKind::Plane)
        {
            math::Vec3 normal;
            double     depth = 0.0;
            if (collideSpherePlane(center, radius, body.shape.planeNormal, body.shape.planeOffset, normal, depth)
                && depth > bestDepth)
            {
                bestDepth  = depth;
                bestNormal = normal;
                bestBody   = body.handle;
                hit        = true;
            }
            continue;
        }
        // The shape AABB is an exact world-space bound, so a sphere that
        // overlaps the shape always passes this test; skipping it prunes the
        // narrow-phase call for far bodies.
        if (!sphereMayOverlapShape(center, radius, body.aabbMinimum, body.aabbMaximum))
        {
            continue;
        }
        math::Vec3 normal;
        double     depth = 0.0;
        if (collideSphereSolid(center, radius, body.shape, body.transform, normal, depth) && depth > bestDepth)
        {
            bestDepth  = depth;
            bestNormal = normal;
            bestBody   = body.handle;
            hit        = true;
        }
    }
    if (!hit)
    {
        return false;
    }
    outNormal = bestNormal;
    outDepth  = bestDepth;
    outBody   = bestBody;
    return true;
}

std::uint32_t collideSpheresWorld(const World &world, const SphereOverlapQuery *queries, std::uint32_t queryCount,
    SphereOverlapResult *outResults)
{
    if (queries == nullptr || outResults == nullptr)
    {
        return 0u;
    }
    std::uint32_t processed = 0;
    for (std::uint32_t index = 0; index < queryCount; ++index)
    {
        const SphereOverlapQuery &query = queries[index];
        SphereOverlapResult       result;
        math::Vec3                normal;
        double                    depth = 0.0;
        BodyHandle                body  = invalidBody;
        if (collideSphereWorld(world, math::Vec3{query.center[0], query.center[1], query.center[2]}, query.radius,
                normal, depth, body))
        {
            result.hit   = true;
            result.normal[0] = normal.x;
            result.normal[1] = normal.y;
            result.normal[2] = normal.z;
            result.depth = depth;
            result.body  = body;
        }
        outResults[index] = result;
        processed         = index + 1;
    }
    return processed;
}

void fillWorldStatistics(const World &world, WorldStatistics &outStatistics)
{
    WorldStatistics statistics;
    statistics.bodyCount = static_cast<std::uint32_t>(world.bodies.size());
    for (const Body &body : world.bodies)
    {
        if (body.kind == BodyKind::Static)
        {
            ++statistics.staticBodyCount;
        }
        else if (body.kind == BodyKind::Kinematic)
        {
            ++statistics.kinematicBodyCount;
        }
        if (body.kind == BodyKind::Dynamic)
        {
            if (body.awake)
            {
                ++statistics.awakeBodyCount;
                if (body.lodBand != engine::util::SimulationFidelity::Full)
                {
                    ++statistics.lodReducedBodyCount;
                }
            }
            else
            {
                ++statistics.sleepingBodyCount;
            }
        }
    }
    statistics.jointCount       = static_cast<std::uint32_t>(world.joints.size());
    statistics.contactPairCount = static_cast<std::uint32_t>(world.manifolds.size());
    for (const auto &entry : world.manifolds)
    {
        statistics.contactPointCount += entry.second.pointCount;
    }
    statistics.triggerCount             = static_cast<std::uint32_t>(world.triggers.size());
    statistics.pendingTriggerEventCount = static_cast<std::uint32_t>(world.pendingTriggerEvents.size());
    statistics.stepsSimulated       = world.stepsSimulated;
    statistics.lastStepMilliseconds = world.lastStepMilliseconds;
    outStatistics                   = statistics;
}

World *createWorld(std::vector<World> &worlds, std::unordered_map<WorldHandle, std::size_t> &indexByHandle,
    WorldHandle &nextHandle, const SimulationSettings &settings, const WorldDescription &description)
{
    World world;
    world.settings = settings;
    world.gravity  = description.useConfiguredGravity ? settings.gravity : description.gravity;
    resolveLodIterationBudgets(world.settings);
    if (nextHandle == invalidWorld)
    {
        return nullptr; // handle space exhausted
    }
    world.handle = nextHandle;
    nextHandle   = nextHandle == std::numeric_limits<WorldHandle>::max() ? invalidWorld : nextHandle + 1;
    worlds.push_back(std::move(world));
    indexByHandle.emplace(worlds.back().handle, worlds.size() - 1);
    return &worlds.back();
}

} // namespace engine::physics_rigid
