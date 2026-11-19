// Shared XPBD particle-system vocabulary for the engine's simulation
// modules.
//
// Cloth, rope, and similar flat-xyz particle systems repeat the same three
// solver pieces: the XPBD distance constraint, the end-of-substep velocity
// reconstruction with contact response, and the batched sphere-overlap
// projection against the rigid world. This header keeps one canonical
// implementation of each so the modules behave identically where their
// models overlap. It is header-only and dependency-free apart from the
// physics_rigid capability vocabulary and the standard library; the caller
// owns its records, scratch buffers, and policy (tearing, breaking,
// attachment kinds, sleep).

#pragma once

#include <engine/physics_rigid/capabilities.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace engine::util
{

// One XPBD distance constraint solve over a flat xyz position array.
// particleA/particleB index particles (offsets are index * 3); the inverse
// masses passed in already reflect the caller's attachment policy (0 for
// pinned or body-attached particles). Returns the constraint's lambda so
// callers can accumulate tension or strain statistics; a degenerate pair
// (no movable mass, or coincident particles) returns 0 and leaves the
// state untouched.
//
// The compliance term follows the XPBD formulation: the correction divides
// by (inverseMassSum + compliance / dt^2), so `compliance` is a physical
// inverse stiffness in meters per newton and the solve stays stable at any
// substep count.
inline double solveXpbdDistance(double *positions, std::uint32_t particleA, std::uint32_t particleB,
    double inverseMassA, double inverseMassB, double restLength, double compliance, double substepDelta)
{
    const double weightSum = inverseMassA + inverseMassB;
    if (weightSum <= 0.0)
    {
        return 0.0;
    }
    const std::size_t offsetA = static_cast<std::size_t>(particleA) * 3u;
    const std::size_t offsetB = static_cast<std::size_t>(particleB) * 3u;
    const double      deltaX  = positions[offsetB] - positions[offsetA];
    const double      deltaY  = positions[offsetB + 1] - positions[offsetA + 1];
    const double      deltaZ  = positions[offsetB + 2] - positions[offsetA + 2];
    const double      length  = std::sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
    if (length <= 1.0e-12)
    {
        return 0.0;
    }
    const double complianceScaled = compliance / (substepDelta * substepDelta);
    const double lambda           = (restLength - length) / (weightSum + complianceScaled);
    const double scaleX           = deltaX / length * lambda;
    const double scaleY           = deltaY / length * lambda;
    const double scaleZ           = deltaZ / length * lambda;
    positions[offsetA] -= inverseMassA * scaleX;
    positions[offsetA + 1] -= inverseMassA * scaleY;
    positions[offsetA + 2] -= inverseMassA * scaleZ;
    positions[offsetB] += inverseMassB * scaleX;
    positions[offsetB + 1] += inverseMassB * scaleY;
    positions[offsetB + 2] += inverseMassB * scaleZ;
    return lambda;
}

// Rebuilds one particle's velocity from the substep's position delta, then
// applies the contact response and the speed clamp in the same pass:
//
//  1. velocity = (position - previousPosition) / substepDelta (the PBD
//     velocity reconstruction);
//  2. when contactNormal is non-null, the velocity component along the
//     normal (the approach into the obstacle) is removed;
//  3. when in contact, the tangential remainder scales by
//     frictionRetention - the caller passes friction^(substepDelta), the
//     per-second "sliding velocity kept" factor raised to the contact
//     time, so the same wall-clock contact removes the same tangential
//     speed at any step rate;
//  4. the result clamps to maximumVelocity so solver noise cannot
//     escalate into a diverging state.
inline void finalizeParticleVelocity(double *velocity, const double *position, const double *previousPosition,
    double inverseSubstepDelta, const double *contactNormal, double frictionRetention, double maximumVelocity)
{
    double velocityX = (position[0] - previousPosition[0]) * inverseSubstepDelta;
    double velocityY = (position[1] - previousPosition[1]) * inverseSubstepDelta;
    double velocityZ = (position[2] - previousPosition[2]) * inverseSubstepDelta;
    if (contactNormal != nullptr)
    {
        const double normalVelocity
            = contactNormal[0] * velocityX + contactNormal[1] * velocityY + contactNormal[2] * velocityZ;
        velocityX -= contactNormal[0] * normalVelocity;
        velocityY -= contactNormal[1] * normalVelocity;
        velocityZ -= contactNormal[2] * normalVelocity;
        velocityX *= frictionRetention;
        velocityY *= frictionRetention;
        velocityZ *= frictionRetention;
    }
    const double speedSquared = velocityX * velocityX + velocityY * velocityY + velocityZ * velocityZ;
    if (speedSquared > maximumVelocity * maximumVelocity)
    {
        const double scale = maximumVelocity / std::sqrt(speedSquared);
        velocityX *= scale;
        velocityY *= scale;
        velocityZ *= scale;
    }
    velocity[0] = velocityX;
    velocity[1] = velocityY;
    velocity[2] = velocityZ;
}

// Builds one sphere-overlap query per particle the caller does not skip
// (pinned particles, body-attached nodes, collision-disabled systems),
// issues one batched collideSpheres call against the rigid world, projects
// every hit particle out along the push-out normal, and writes the hit
// normal through outContactNormals (offset particle * 3) and 1 through
// outContactFlags (offset particle) when those pointers are non-null - the
// caller clears both first so per-substep contact state stays fresh.
// Positions are read for the queries and written for the projection (the
// push-out mutates the array in place). Returns the number of hits. The
// three scratch vectors are caller-owned and reused across substeps so the
// batched path allocates nothing after warm-up.
template <typename SkipFn>
inline std::uint32_t projectParticleSphereOverlaps(engine::physics_rigid::PhysicsRigidCapability *rigid,
    engine::physics_rigid::WorldHandle world, double *positions, std::size_t particleCount, double radius,
    SkipFn &&skip, std::vector<engine::physics_rigid::SphereOverlapQuery> &queries,
    std::vector<std::size_t> &querySlots, std::vector<engine::physics_rigid::SphereOverlapResult> &results,
    double *outContactNormals, std::uint8_t *outContactFlags)
{
    if (rigid == nullptr)
    {
        return 0u;
    }
    queries.clear();
    querySlots.clear();
    for (std::size_t particle = 0; particle < particleCount; ++particle)
    {
        if (skip(particle))
        {
            continue;
        }
        engine::physics_rigid::SphereOverlapQuery query;
        const std::size_t offset = particle * 3u;
        query.center[0]          = positions[offset];
        query.center[1]          = positions[offset + 1];
        query.center[2]          = positions[offset + 2];
        query.radius             = radius;
        querySlots.push_back(particle);
        queries.push_back(query);
    }
    if (queries.empty())
    {
        return 0u;
    }
    results.resize(queries.size());
    rigid->collideSpheres(rigid->self, world, queries.data(), static_cast<std::uint32_t>(queries.size()),
        results.data());
    std::uint32_t hits = 0u;
    for (std::size_t entry = 0; entry < querySlots.size(); ++entry)
    {
        const engine::physics_rigid::SphereOverlapResult &result = results[entry];
        if (!result.hit || result.depth <= 0.0)
        {
            continue;
        }
        const std::size_t offset = querySlots[entry] * 3u;
        if (outContactNormals != nullptr)
        {
            outContactNormals[offset]     = result.normal[0];
            outContactNormals[offset + 1] = result.normal[1];
            outContactNormals[offset + 2] = result.normal[2];
        }
        if (outContactFlags != nullptr)
        {
            outContactFlags[querySlots[entry]] = 1u;
        }
        positions[offset] += result.normal[0] * result.depth;
        positions[offset + 1] += result.normal[1] * result.depth;
        positions[offset + 2] += result.normal[2] * result.depth;
        ++hits;
    }
    return hits;
}

} // namespace engine::util
