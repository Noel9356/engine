// sph_step.cpp - the SPH fixed-step pipeline. Every substep rebuilds the
// shared sorted-bucket spatial hash, collects the neighbor lists once into a
// cache shared by both passes, estimates densities with the poly6 kernel,
// accumulates pressure (spiky gradient) and viscosity (Laplacian) forces,
// integrates, and resolves particles against rigid geometry through the
// batched sphere-overlap query plus the optional bounds box. The kernel
// normalizations follow Muller et al. 2003: poly6 carries 315 / (64 pi h^9)
// so the density estimate lands in kilograms per cubic meter, and the
// viscosity force divides by the neighbor density alone.

#include "sph_step.hpp"

#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

namespace engine::physics_fluid
{
namespace
{

namespace math = engine::physics_rigid::math;

// Resolves every particle against the rigid world in one batched sphere
// query per substep instead of one capability call per particle. Returns the
// substep's hit count; the step accumulates the counts so the statistic
// reports the whole step. The query and result scratch live in the record
// and are reused across substeps.
std::uint32_t resolveFluidCollisions(FluidRecord &fluid, engine::physics_rigid::PhysicsRigidCapability *rigid)
{
    if (rigid == nullptr)
    {
        return 0u;
    }
    const std::size_t particleCount = fluid.positions.size() / 3;
    fluid.collisionQueries.clear();
    fluid.collisionQueries.reserve(particleCount);
    for (std::size_t particle = 0; particle < particleCount; ++particle)
    {
        engine::physics_rigid::SphereOverlapQuery query;
        const std::size_t offset = particle * 3;
        query.center[0] = fluid.positions[offset];
        query.center[1] = fluid.positions[offset + 1];
        query.center[2] = fluid.positions[offset + 2];
        query.radius    = fluid.description.particleRadius;
        fluid.collisionQueries.push_back(query);
    }
    if (fluid.collisionQueries.empty())
    {
        return 0u;
    }
    fluid.collisionResults.resize(fluid.collisionQueries.size());
    const std::uint32_t resolvedCollisions = rigid->collideSpheres(rigid->self,
        fluid.world,
        fluid.collisionQueries.data(),
        static_cast<std::uint32_t>(fluid.collisionQueries.size()),
        fluid.collisionResults.data());
    if (resolvedCollisions < fluid.collisionQueries.size())
    {
        // The provider wrote nothing (the rigid world is gone). resize()
        // keeps the previous substep's contents, and re-reading stale hits
        // would drift positions by their old depths every substep, so every
        // entry is neutralized before the walk.
        for (engine::physics_rigid::SphereOverlapResult &result : fluid.collisionResults)
        {
            result = engine::physics_rigid::SphereOverlapResult{};
        }
    }
    std::uint32_t hits = 0u;
    for (std::size_t particle = 0; particle < particleCount; ++particle)
    {
        const engine::physics_rigid::SphereOverlapResult &result = fluid.collisionResults[particle];
        if (!result.hit || result.depth <= 0.0)
        {
            continue;
        }
        const std::size_t offset = particle * 3;
        ++hits;
        // Project the particle out along the push-out direction and
        // remove the approaching velocity part (inelastic contact).
        fluid.positions[offset] += result.normal[0] * result.depth;
        fluid.positions[offset + 1] += result.normal[1] * result.depth;
        fluid.positions[offset + 2] += result.normal[2] * result.depth;
        const double approach = result.normal[0] * fluid.velocities[offset]
                                + result.normal[1] * fluid.velocities[offset + 1]
                                + result.normal[2] * fluid.velocities[offset + 2];
        if (approach < 0.0)
        {
            fluid.velocities[offset] -= result.normal[0] * approach;
            fluid.velocities[offset + 1] -= result.normal[1] * approach;
            fluid.velocities[offset + 2] -= result.normal[2] * approach;
        }
    }
    return hits;
}


// Poly6 kernel W(r, h) for 0 <= r <= h: 315 / (64 pi h^9) (h^2 - r^2)^3. The
// h^9 normalization keeps the kernel's units at 1/m^3, so a density estimate
// built from it reads kilograms per cubic meter and the equation of state
// responds to realistic rest densities.
inline double poly6Kernel(double rSquared, double h)
{
    const double hSquared = h * h;
    if (rSquared >= hSquared)
    {
        return 0.0;
    }
    const double hCubed     = hSquared * h;
    const double hToTheNinth = hCubed * hCubed * hCubed;
    const double difference = hSquared - rSquared;
    return 315.0 / (64.0 * math::kPi * hToTheNinth) * difference * difference * difference;
}

// Spiky kernel gradient magnitude factor: -45 / (pi h^6) * (h - r)^2, times
// the radial direction for the full gradient.
inline math::Vec3 spikyGradient(const math::Vec3 &direction, double r, double h)
{
    if (r >= h || r < 1.0e-9)
    {
        return math::Vec3{};
    }
    const double magnitude
        = -45.0 / (math::kPi * h * h * h * h * h * h) * (h - r) * (h - r);
    return direction * magnitude;
}

// Viscosity Laplacian: 45 / (pi h^6) * (h - r).
inline double viscosityLaplacian(double r, double h)
{
    if (r >= h)
    {
        return 0.0;
    }
    return 45.0 / (math::kPi * h * h * h * h * h * h) * (h - r);
}

// Builds the per-substep neighbor cache: one flat structure both SPH passes
// read (positions do not change between the density and force passes, so the
// neighbor lists collect once instead of twice per substep). The shared hash
// caps the candidate walk at the configured neighbor budget, so a
// degenerately dense cluster costs a bounded list instead of an unbounded
// one.
void buildNeighborCache(FluidRecord &fluid, const FluidSettings &settings)
{
    const std::size_t particleCount = fluid.positions.size() / 3;
    fluid.neighborStarts.assign(particleCount + 1, 0u);
    fluid.neighborCounts.assign(particleCount, 0u);
    fluid.neighborIndices.clear();
    fluid.neighborCandidates.resize(
        std::min<std::size_t>(settings.maximumNeighborsPerParticle, particleCount + 1u));
    for (std::size_t particle = 0; particle < particleCount; ++particle)
    {
        const std::uint32_t collected = fluid.particleHash.collectNeighborCandidates(fluid.positions.data(),
            particle,
            fluid.neighborCandidates.data(),
            settings.maximumNeighborsPerParticle);
        fluid.neighborCounts[particle]    = collected;
        fluid.neighborStarts[particle + 1] = fluid.neighborStarts[particle] + collected;
        for (std::uint32_t candidate = 0; candidate < collected; ++candidate)
        {
            fluid.neighborIndices.push_back(fluid.neighborCandidates[candidate]);
        }
    }
}

void computeDensities(FluidRecord &fluid)
{
    const std::size_t particleCount = fluid.positions.size() / 3;
    const double      h             = fluid.description.interactionRadius;
    for (std::size_t particle = 0; particle < particleCount; ++particle)
    {
        const double x = fluid.positions[particle * 3];
        const double y = fluid.positions[particle * 3 + 1];
        const double z = fluid.positions[particle * 3 + 2];
        double density = fluid.description.particleMass * poly6Kernel(0.0, h);
        for (std::uint32_t slot = fluid.neighborStarts[particle]; slot < fluid.neighborStarts[particle + 1]; ++slot)
        {
            const std::uint32_t neighbor = fluid.neighborIndices[slot];
            const double deltaX = fluid.positions[static_cast<std::size_t>(neighbor) * 3] - x;
            const double deltaY = fluid.positions[static_cast<std::size_t>(neighbor) * 3 + 1] - y;
            const double deltaZ = fluid.positions[static_cast<std::size_t>(neighbor) * 3 + 2] - z;
            density += fluid.description.particleMass * poly6Kernel(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ, h);
        }
        fluid.densities[particle]  = density;
        fluid.pressures[particle]  = fluid.description.gasStiffness * std::max(density - fluid.description.restDensity, 0.0);
    }
}

void accumulateForces(FluidRecord &fluid)
{
    const std::size_t particleCount = fluid.positions.size() / 3;
    const double      h             = fluid.description.interactionRadius;
    for (std::size_t particle = 0; particle < particleCount; ++particle)
    {
        const math::Vec3 position
            = math::Vec3{fluid.positions[particle * 3], fluid.positions[particle * 3 + 1], fluid.positions[particle * 3 + 2]};
        const math::Vec3 velocity = math::Vec3{
            fluid.velocities[particle * 3], fluid.velocities[particle * 3 + 1], fluid.velocities[particle * 3 + 2]};
        math::Vec3 pressureForce;
        math::Vec3 viscosityForce;
        for (std::uint32_t slot = fluid.neighborStarts[particle]; slot < fluid.neighborStarts[particle + 1]; ++slot)
        {
            const std::uint32_t neighbor = fluid.neighborIndices[slot];
            const std::size_t offset = static_cast<std::size_t>(neighbor) * 3;
            const math::Vec3 delta   = position
                                     - math::Vec3{fluid.positions[offset], fluid.positions[offset + 1], fluid.positions[offset + 2]};
            const double radiusSquared = math::lengthSquared(delta);
            const double radius        = std::sqrt(radiusSquared);
            if (radius >= h || radius < 1.0e-9)
            {
                continue;
            }
            const math::Vec3 direction = delta * (1.0 / radius);
            const double neighborDensity = fluid.densities[neighbor] > 1.0e-9 ? fluid.densities[neighbor] : 1.0e-9;

            // Pressure: symmetric -m (p_i + p_j) / (2 rho_j) grad W.
            pressureForce = pressureForce
                            + spikyGradient(direction, radius, h)
                                  * (-fluid.description.particleMass
                                        * (fluid.pressures[particle] + fluid.pressures[neighbor])
                                        / (2.0 * neighborDensity));

            // Viscosity: m mu (v_j - v_i) / rho_j lap W (a single density
            // division, per the symmetric Muller formulation).
            const math::Vec3 neighborVelocity = math::Vec3{
                fluid.velocities[offset], fluid.velocities[offset + 1], fluid.velocities[offset + 2]};
            viscosityForce = viscosityForce
                             + (neighborVelocity - velocity)
                                   * (fluid.description.viscosity * fluid.description.particleMass / neighborDensity
                                      * viscosityLaplacian(radius, h));
        }
        math::Vec3 acceleration = pressureForce + viscosityForce;
        acceleration.y += fluid.gravity[1];
        acceleration.x += fluid.gravity[0];
        acceleration.z += fluid.gravity[2];
        fluid.accelerations[particle * 3]     = acceleration.x;
        fluid.accelerations[particle * 3 + 1] = acceleration.y;
        fluid.accelerations[particle * 3 + 2] = acceleration.z;
    }
}

void integrateSubstep(FluidRecord &fluid, const FluidSettings &settings, double substepDelta)
{
    const std::size_t particleCount = fluid.positions.size() / 3;
    const double      dampingFactor = 1.0 / (1.0 + substepDelta * fluid.description.damping);
    const double      maximum       = settings.maximumVelocity;
    for (std::size_t particle = 0; particle < particleCount; ++particle)
    {
        const std::size_t offset = particle * 3;
        fluid.velocities[offset] += fluid.accelerations[offset] * substepDelta;
        fluid.velocities[offset + 1] += fluid.accelerations[offset + 1] * substepDelta;
        fluid.velocities[offset + 2] += fluid.accelerations[offset + 2] * substepDelta;
        fluid.velocities[offset] *= dampingFactor;
        fluid.velocities[offset + 1] *= dampingFactor;
        fluid.velocities[offset + 2] *= dampingFactor;
        const double speedSquared = fluid.velocities[offset] * fluid.velocities[offset]
                                    + fluid.velocities[offset + 1] * fluid.velocities[offset + 1]
                                    + fluid.velocities[offset + 2] * fluid.velocities[offset + 2];
        if (speedSquared > maximum * maximum)
        {
            const double scale = maximum / std::sqrt(speedSquared);
            fluid.velocities[offset] *= scale;
            fluid.velocities[offset + 1] *= scale;
            fluid.velocities[offset + 2] *= scale;
        }
        fluid.positions[offset] += fluid.velocities[offset] * substepDelta;
        fluid.positions[offset + 1] += fluid.velocities[offset + 1] * substepDelta;
        fluid.positions[offset + 2] += fluid.velocities[offset + 2] * substepDelta;
    }
}

void resolveBounds(FluidRecord &fluid, const FluidSettings &settings)
{
    if (!fluid.description.useBounds)
    {
        return;
    }
    const std::size_t particleCount = fluid.positions.size() / 3;
    for (std::size_t particle = 0; particle < particleCount; ++particle)
    {
        const std::size_t offset = particle * 3;
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            const double minimum = fluid.description.boundsMinimum[axis] + fluid.description.particleRadius;
            const double maximum = fluid.description.boundsMaximum[axis] - fluid.description.particleRadius;
            if (fluid.positions[offset + axis] < minimum)
            {
                fluid.positions[offset + axis] = minimum;
                fluid.velocities[offset + axis] *= -settings.boundsRestitution;
            }
            else if (fluid.positions[offset + axis] > maximum)
            {
                fluid.positions[offset + axis] = maximum;
                fluid.velocities[offset + axis] *= -settings.boundsRestitution;
            }
        }
    }
}

} // namespace

bool stepFluid(FluidRecord &fluid, const FluidSettings &settings,
    engine::physics_rigid::PhysicsRigidCapability *rigid, double fixedDeltaSeconds, std::uint32_t substeps,
    std::uint32_t interval)
{
    const auto stepStart = std::chrono::steady_clock::now();
    // The sanity cap applies PER MERGED FIXED STEP: a simulation-LOD due
    // step legitimately integrates `interval` accumulated fixed deltas in
    // one pass, so the accepted delta scales with the merge count (capping
    // the merged delta at the single-step bound would freeze every
    // Animation-band fluid at the default 60 Hz).
    const double maximumDelta = 0.1 * static_cast<double>(interval > 0u ? interval : 1u);
    if (!(fixedDeltaSeconds > 0.0) || fixedDeltaSeconds > maximumDelta || !std::isfinite(fixedDeltaSeconds))
    {
        return false;
    }
    const double        substepDelta  = fixedDeltaSeconds / static_cast<double>(substeps);
    const std::size_t   particleCount = fluid.positions.size() / 3;
    // The contact statistic covers the whole step: every substep's hit
    // count accumulates into the one value the statistics publish.
    fluid.lastContactCount = 0;
    // Pre-step copies for the rollback path: a diverged step restores
    // this state and reports failure, so the caller sees an untouched
    // fluid.
    fluid.preStepPositions.resize(fluid.positions.size());
    fluid.preStepVelocities.resize(fluid.velocities.size());
    std::copy(fluid.positions.begin(), fluid.positions.end(), fluid.preStepPositions.begin());
    std::copy(fluid.velocities.begin(), fluid.velocities.end(), fluid.preStepVelocities.begin());
    for (std::uint32_t substep = 0; substep < substeps; ++substep)
    {
        fluid.particleHash.rebuild(fluid.positions.data(), particleCount, fluid.description.interactionRadius);
        buildNeighborCache(fluid, settings);
        computeDensities(fluid);
        accumulateForces(fluid);
        integrateSubstep(fluid, settings, substepDelta);
        resolveBounds(fluid, settings);
        fluid.lastContactCount += resolveFluidCollisions(fluid, rigid);
    }
    double densitySum = 0.0;
    for (double density : fluid.densities)
    {
        densitySum += density;
    }
    fluid.lastAverageDensity = particleCount > 0 ? densitySum / static_cast<double>(particleCount) : 0.0;
    bool finite = true;
    for (const double value : fluid.positions)
    {
        if (!std::isfinite(value))
        {
            finite = false;
            break;
        }
    }
    for (const double value : fluid.velocities)
    {
        if (!std::isfinite(value))
        {
            finite = false;
            break;
        }
    }
    if (!finite)
    {
        std::copy(fluid.preStepPositions.begin(), fluid.preStepPositions.end(), fluid.positions.begin());
        std::copy(fluid.preStepVelocities.begin(), fluid.preStepVelocities.end(), fluid.velocities.begin());
        return false;
    }
    fluid.stepsSimulated += 1;
    fluid.substepsExecuted += substeps;
    const auto stepEnd          = std::chrono::steady_clock::now();
    fluid.lastStepMilliseconds = std::chrono::duration<double, std::milli>(stepEnd - stepStart).count();
    return true;
}

} // namespace engine::physics_fluid
