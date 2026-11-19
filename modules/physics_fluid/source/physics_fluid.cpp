// physics_fluid module implementation.
//
// Smoothed-particle hydrodynamics over a physics_rigid world. Every substep
// rebuilds the spatial hash, collects the neighbor lists once into a cache
// shared by both passes, estimates densities with the poly6 kernel,
// accumulates pressure (spiky gradient) and viscosity (Laplacian) forces,
// integrates, and resolves particles against rigid geometry through the
// batched sphere-overlap query plus the optional bounds box. Stepping
// happens in whole fixed steps with internal substeps, so the simulation is
// independent of frame rate; simulation LOD cuts the substep count for
// unimportant fluids without touching the fixed step size.

#include "records.hpp"
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/physics_fluid/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>
#include <engine/util/module_error.hpp>
#include <engine/util/simulation_lod.hpp>
#include <engine/util/step_accumulator.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine::physics_fluid
{
namespace
{

namespace math = engine::physics_rigid::math;

// Poly6 kernel W(r, h) for 0 <= r <= h.
inline double poly6Kernel(double rSquared, double h)
{
    const double hSquared = h * h;
    if (rSquared >= hSquared)
    {
        return 0.0;
    }
    const double difference = hSquared - rSquared;
    return 315.0 / (64.0 * math::kPi * hSquared * hSquared * hSquared) * difference * difference * difference;
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
// neighbor lists collect once instead of twice per substep).
void collectParticleNeighbors(const FluidRecord &fluid, std::uint32_t particleIndex,
    std::vector<std::uint32_t> &outNeighbors);

void buildNeighborCache(FluidRecord &fluid)
{
    const std::size_t particleCount = fluid.positions.size() / 3;
    fluid.neighborStarts.assign(particleCount + 1, 0u);
    fluid.neighborCounts.assign(particleCount, 0u);
    fluid.neighborIndices.clear();
    static thread_local std::vector<std::uint32_t> collected;
    for (std::size_t particle = 0; particle < particleCount; ++particle)
    {
        collectParticleNeighbors(fluid, static_cast<std::uint32_t>(particle), collected);
        fluid.neighborCounts[particle] = static_cast<std::uint32_t>(collected.size());
        fluid.neighborStarts[particle + 1] = fluid.neighborStarts[particle] + fluid.neighborCounts[particle];
        fluid.neighborIndices.insert(fluid.neighborIndices.end(), collected.begin(), collected.end());
    }
}

void rebuildParticleHash(FluidRecord &fluid, double cellSize)
{
    const std::size_t particleCount = fluid.positions.size() / 3;
    fluid.hashCellSize = cellSize;
    fluid.sortedIndices.resize(particleCount);
    // Hash cell coordinates into a bucket id; bounds-free modulo hashing so
    // unbounded fluids work too.
    std::unordered_map<std::uint64_t, std::uint32_t> cellCounts;
    cellCounts.reserve(particleCount * 2);
    std::vector<std::uint64_t> cellIds(particleCount);
    for (std::size_t particle = 0; particle < particleCount; ++particle)
    {
        const double x = fluid.positions[particle * 3];
        const double y = fluid.positions[particle * 3 + 1];
        const double z = fluid.positions[particle * 3 + 2];
        const std::int64_t cx = static_cast<std::int64_t>(std::floor(x / cellSize));
        const std::int64_t cy = static_cast<std::int64_t>(std::floor(y / cellSize));
        const std::int64_t cz = static_cast<std::int64_t>(std::floor(z / cellSize));
        const std::uint64_t id = (static_cast<std::uint64_t>(cx) * 73856093ull)
                                 ^ (static_cast<std::uint64_t>(cy) * 19349663ull)
                                 ^ (static_cast<std::uint64_t>(cz) * 83492791ull);
        cellIds[particle]    = id;
        fluid.sortedIndices[particle] = static_cast<std::uint32_t>(particle);
        ++cellCounts[id];
    }
    // Stable counting sort by cell id (deterministic order).
    std::vector<std::uint32_t> cellStarts;
    cellStarts.reserve(cellCounts.size());
    std::vector<std::uint64_t> cellOrder;
    cellOrder.reserve(cellCounts.size());
    for (const auto &entry : cellCounts)
    {
        cellOrder.push_back(entry.first);
    }
    std::sort(cellOrder.begin(), cellOrder.end());
    cellStarts.reserve(cellOrder.size());
    std::uint32_t running = 0;
    for (const std::uint64_t id : cellOrder)
    {
        cellStarts.push_back(running);
        running += cellCounts[id];
    }
    fluid.cellBuckets.clear();
    fluid.cellBuckets.reserve(cellStarts.size() * 2);
    std::unordered_map<std::uint64_t, std::uint32_t> cellSlot;
    cellSlot.reserve(cellOrder.size());
    for (std::size_t index = 0; index < cellOrder.size(); ++index)
    {
        cellSlot[cellOrder[index]] = static_cast<std::uint32_t>(index);
    }
    std::vector<std::uint32_t> fill = cellStarts;
    std::vector<std::uint32_t> sorted(particleCount);
    for (std::size_t particle = 0; particle < particleCount; ++particle)
    {
        const auto slot = cellSlot.find(cellIds[particle]);
        sorted[fill[slot->second]++] = fluid.sortedIndices[particle];
    }
    fluid.sortedIndices = std::move(sorted);
    fluid.cellBuckets   = cellStarts;
    // Remember the cell ordering for neighbor lookup.
    fluid.hashCellIds.assign(cellOrder.begin(), cellOrder.end());
    fluid.hashCellCounts.resize(cellOrder.size());
    for (std::size_t index = 0; index < cellOrder.size(); ++index)
    {
        const std::uint32_t start = cellStarts[index];
        const std::uint32_t end
            = index + 1 < cellStarts.size() ? cellStarts[index + 1] : static_cast<std::uint32_t>(particleCount);
        fluid.hashCellCounts[index] = end - start;
    }
}

void collectParticleNeighbors(const FluidRecord &fluid, std::uint32_t particleIndex,
    std::vector<std::uint32_t> &outNeighbors)
{
    outNeighbors.clear();
    const std::size_t particleCount = fluid.positions.size() / 3;
    if (particleIndex >= particleCount || fluid.hashCellIds.empty())
    {
        return;
    }
    const double cellSize = fluid.hashCellSize;
    const double x = fluid.positions[static_cast<std::size_t>(particleIndex) * 3];
    const double y = fluid.positions[static_cast<std::size_t>(particleIndex) * 3 + 1];
    const double z = fluid.positions[static_cast<std::size_t>(particleIndex) * 3 + 2];
    const std::int64_t cx = static_cast<std::int64_t>(std::floor(x / cellSize));
    const std::int64_t cy = static_cast<std::int64_t>(std::floor(y / cellSize));
    const std::int64_t cz = static_cast<std::int64_t>(std::floor(z / cellSize));
    for (std::int64_t ox = -1; ox <= 1; ++ox)
    {
        for (std::int64_t oy = -1; oy <= 1; ++oy)
        {
            for (std::int64_t oz = -1; oz <= 1; ++oz)
            {
                const std::uint64_t id = (static_cast<std::uint64_t>(cx + ox) * 73856093ull)
                                         ^ (static_cast<std::uint64_t>(cy + oy) * 19349663ull)
                                         ^ (static_cast<std::uint64_t>(cz + oz) * 83492791ull);
                const auto slot = std::lower_bound(
                    fluid.hashCellIds.begin(), fluid.hashCellIds.end(), id);
                if (slot == fluid.hashCellIds.end() || *slot != id)
                {
                    continue;
                }
                const std::size_t cellIndex = static_cast<std::size_t>(slot - fluid.hashCellIds.begin());
                const std::uint32_t start   = fluid.cellBuckets[cellIndex];
                const std::uint32_t count   = fluid.hashCellCounts[cellIndex];
                for (std::uint32_t entry = 0; entry < count; ++entry)
                {
                    const std::uint32_t candidate = fluid.sortedIndices[start + entry];
                    if (candidate != particleIndex)
                    {
                        outNeighbors.push_back(candidate);
                    }
                }
            }
        }
    }
}

class FluidModule final : public ModuleInterface
{
public:
    FluidModule()
        : manifest_(makeManifest())
    {
        assignCapabilityPointers();
    }

    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    void construct(ParticipantContext &context) override
    {
        if (const auto value = context.configurationIntegerInSection("physics_fluid", "maximum_fluids");
            value && *value > 0 && *value <= 4096)
            settings_.maximumFluids = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_fluid", "maximum_particles");
            value && *value > 0 && *value <= 10'000'000)
            settings_.maximumParticles = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_fluid", "maximum_particles_per_fluid");
            value && *value > 0 && *value <= 1'000'000)
            settings_.maximumParticlesPerFluid = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_fluid", "substeps_per_step");
            value && *value > 0 && *value <= 64)
            settings_.substepsPerStep = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_fluid", "maximum_steps_per_simulate");
            value && *value > 0 && *value <= 1000)
            settings_.maximumStepsPerSimulate = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationFloatingInSection("physics_fluid", "fixed_rate_hz");
            value && *value >= 1.0 && *value <= 1000.0 && std::isfinite(*value))
            settings_.fixedRateHz = *value;
        if (const auto value = context.configurationFloatingInSection("physics_fluid", "maximum_velocity");
            value && *value >= 1.0 && *value <= 1000.0 && std::isfinite(*value))
            settings_.maximumVelocity = *value;
        if (const auto value = context.configurationFloatingInSection("physics_fluid", "bounds_restitution");
            value && *value >= 0.0 && *value <= 1.0 && std::isfinite(*value))
            settings_.boundsRestitution = *value;
        readLodSettings(context);

        rigid_ = context.tryAcquireCapability<engine::physics_rigid::PhysicsRigidCapability>();
        if (rigid_ != nullptr && rigid_->abi != engine::physics_rigid::physicsRigidCapabilityAbi)
        {
            rigid_ = nullptr;
        }
    }

    void start() override
    {}

    void stop() override
    {
        fluids_.clear();
        fluidIndexByHandle_.clear();
        nextFluidHandle_ = 1;
        totalParticles_  = 0;
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == FluidCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "physics_fluid",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"physics_fluid"},
            .requiredCapabilities = {"physics_rigid"}};
    }

    void readLodSettings(ParticipantContext &context)
    {
        settings_.lod = engine::util::readSimulationLodSettings(context, "physics_fluid");
    }

    static void setError(FluidError *output, FluidError value) noexcept
    {
        engine::util::setModuleError(output, value);
    }

    FluidRecord *findFluid(FluidHandle handle)
    {
        const auto iterator = fluidIndexByHandle_.find(handle);
        return iterator != fluidIndexByHandle_.end() ? &fluids_[iterator->second] : nullptr;
    }

    // ---- SPH stepping -------------------------------------------------------

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
                const double     neighborDensity = fluid.densities[neighbor] > 1.0e-9 ? fluid.densities[neighbor] : 1.0e-9;
                const double ownDensity    = fluid.densities[particle] > 1.0e-9 ? fluid.densities[particle] : 1.0e-9;

                // Pressure: symmetric -m (p_i + p_j) / (2 rho_j) grad W.
                pressureForce = pressureForce
                                + spikyGradient(direction, radius, h)
                                      * (-fluid.description.particleMass
                                            * (fluid.pressures[particle] + fluid.pressures[neighbor])
                                            / (2.0 * neighborDensity));

                // Viscosity: m mu (v_j - v_i) / rho_j lap W.
                const math::Vec3 neighborVelocity = math::Vec3{
                    fluid.velocities[offset], fluid.velocities[offset + 1], fluid.velocities[offset + 2]};
                viscosityForce = viscosityForce
                                 + (neighborVelocity - velocity)
                                       * (fluid.description.viscosity * fluid.description.particleMass / neighborDensity
                                          * viscosityLaplacian(radius, h) / ownDensity);
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

    void integrateSubstep(FluidRecord &fluid, double substepDelta)
    {
        const std::size_t particleCount = fluid.positions.size() / 3;
        const double      dampingFactor = 1.0 / (1.0 + substepDelta * fluid.description.damping);
        const double      maximum       = settings_.maximumVelocity;
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

    void resolveBounds(FluidRecord &fluid)
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
                    fluid.velocities[offset + axis] *= -settings_.boundsRestitution;
                }
                else if (fluid.positions[offset + axis] > maximum)
                {
                    fluid.positions[offset + axis] = maximum;
                    fluid.velocities[offset + axis] *= -settings_.boundsRestitution;
                }
            }
        }
    }

    // Resolves every particle against the rigid world in one batched sphere
    // query per substep instead of one capability call per particle.
    void resolveCollisions(FluidRecord &fluid)
    {
        fluid.lastContactCount = 0;
        if (rigid_ == nullptr)
        {
            return;
        }
        const std::size_t particleCount = fluid.positions.size() / 3;
        collisionQueries_.clear();
        collisionQueries_.reserve(particleCount);
        for (std::size_t particle = 0; particle < particleCount; ++particle)
        {
            engine::physics_rigid::SphereOverlapQuery query;
            const std::size_t offset = particle * 3;
            query.center[0] = fluid.positions[offset];
            query.center[1] = fluid.positions[offset + 1];
            query.center[2] = fluid.positions[offset + 2];
            query.radius    = fluid.description.particleRadius;
            collisionQueries_.push_back(query);
        }
        if (collisionQueries_.empty())
        {
            return;
        }
        collisionResults_.resize(collisionQueries_.size());
        rigid_->collideSpheres(rigid_->self, fluid.world, collisionQueries_.data(),
            static_cast<std::uint32_t>(collisionQueries_.size()), collisionResults_.data());
        for (std::size_t particle = 0; particle < particleCount; ++particle)
        {
            const engine::physics_rigid::SphereOverlapResult &result = collisionResults_[particle];
            if (!result.hit || result.depth <= 0.0)
            {
                continue;
            }
            const std::size_t offset = particle * 3;
            fluid.lastContactCount += 1;
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
    }

    bool stepFluid(FluidRecord &fluid, double fixedDeltaSeconds, std::uint32_t substeps)
    {
        const auto stepStart = std::chrono::steady_clock::now();
        if (!(fixedDeltaSeconds > 0.0) || fixedDeltaSeconds > 0.1 || !std::isfinite(fixedDeltaSeconds))
        {
            return false;
        }
        const double        substepDelta  = fixedDeltaSeconds / static_cast<double>(substeps);
        const std::size_t   particleCount = fluid.positions.size() / 3;
        // Pre-step copies for the rollback path: a diverged step restores
        // this state and reports failure, so the caller sees an untouched
        // fluid.
        fluid.preStepPositions.resize(fluid.positions.size());
        fluid.preStepVelocities.resize(fluid.velocities.size());
        std::copy(fluid.positions.begin(), fluid.positions.end(), fluid.preStepPositions.begin());
        std::copy(fluid.velocities.begin(), fluid.velocities.end(), fluid.preStepVelocities.begin());
        for (std::uint32_t substep = 0; substep < substeps; ++substep)
        {
            rebuildParticleHash(fluid, fluid.description.interactionRadius);
            buildNeighborCache(fluid);
            computeDensities(fluid);
            accumulateForces(fluid);
            integrateSubstep(fluid, substepDelta);
            resolveBounds(fluid);
            resolveCollisions(fluid);
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
        const auto stepEnd            = std::chrono::steady_clock::now();
        fluid.lastStepMilliseconds = std::chrono::duration<double, std::milli>(stepEnd - stepStart).count();
        return true;
    }

    // ---- capability implementations ---------------------------------------

    FluidHandle createFluidImpl(
        engine::physics_rigid::WorldHandle world, const FluidDescription *description, FluidError *error)
    {
        setError(error, FluidError::None);
        if (rigid_ == nullptr || description == nullptr)
        {
            setError(error, FluidError::InvalidDescription);
            return invalidFluid;
        }
        if (fluids_.size() >= settings_.maximumFluids || totalParticles_ >= settings_.maximumParticles)
        {
            setError(error, FluidError::CapacityExceeded);
            return invalidFluid;
        }
        if (!std::isfinite(description->interactionRadius) || description->interactionRadius <= 0.0
            || !std::isfinite(description->particleRadius) || description->particleRadius <= 0.0
            || !std::isfinite(description->particleMass) || description->particleMass <= 0.0
            || !std::isfinite(description->restDensity) || description->restDensity <= 0.0
            || !std::isfinite(description->gasStiffness) || description->gasStiffness < 0.0
            || !std::isfinite(description->viscosity) || description->viscosity < 0.0
            || !std::isfinite(description->damping) || description->damping < 0.0
            || (description->useBounds
                && (!std::isfinite(description->boundsMinimum[0]) || !std::isfinite(description->boundsMinimum[1])
                    || !std::isfinite(description->boundsMinimum[2]) || !std::isfinite(description->boundsMaximum[0])
                    || !std::isfinite(description->boundsMaximum[1]) || !std::isfinite(description->boundsMaximum[2])
                    || description->boundsMinimum[0] >= description->boundsMaximum[0]
                    || description->boundsMinimum[1] >= description->boundsMaximum[1]
                    || description->boundsMinimum[2] >= description->boundsMaximum[2])))
        {
            setError(error, FluidError::InvalidDescription);
            return invalidFluid;
        }
        if (nextFluidHandle_ == invalidFluid)
        {
            setError(error, FluidError::CapacityExceeded);
            return invalidFluid;
        }
        FluidRecord fluid;
        fluid.handle = nextFluidHandle_;
        nextFluidHandle_
            = nextFluidHandle_ == std::numeric_limits<FluidHandle>::max() ? invalidFluid : nextFluidHandle_ + 1;
        fluid.world       = world;
        fluid.description = *description;
        if (!rigid_->copyWorldGravity(rigid_->self, world, fluid.gravity))
        {
            fluid.gravity[0] = 0.0;
            fluid.gravity[1] = -9.81;
            fluid.gravity[2] = 0.0;
        }
        // Simulation LOD: the stagger phase derives from the registration
        // ordinal, so fluids sharing an interval spread their work across
        // steps.
        fluid.cadence.phase = static_cast<std::uint32_t>(registrationOrdinal_++);
        fluids_.push_back(std::move(fluid));
        fluidIndexByHandle_.emplace(fluids_.back().handle, fluids_.size() - 1);
        return fluids_.back().handle;
    }

    void destroyFluidImpl(FluidHandle handle)
    {
        const auto iterator = fluidIndexByHandle_.find(handle);
        if (iterator == fluidIndexByHandle_.end())
        {
            return;
        }
        totalParticles_ -= fluids_[iterator->second].positions.size() / 3;
        const std::size_t index = iterator->second;
        fluidIndexByHandle_.erase(iterator);
        if (index + 1 != fluids_.size())
        {
            fluids_[index]                                   = fluids_.back();
            fluidIndexByHandle_[fluids_[index].handle]       = index;
        }
        fluids_.pop_back();
    }

    bool fluidInfoImpl(FluidHandle handle, FluidInfo *outInfo)
    {
        FluidRecord *fluid = findFluid(handle);
        if (fluid == nullptr || outInfo == nullptr)
        {
            return false;
        }
        outInfo->particleCount     = static_cast<std::uint32_t>(fluid->positions.size() / 3);
        outInfo->interactionRadius = fluid->description.interactionRadius;
        return true;
    }

    std::uint32_t addParticlesImpl(FluidHandle handle, const double *positions, const double *velocities,
        std::uint32_t count, FluidError *error)
    {
        setError(error, FluidError::None);
        FluidRecord *fluid = findFluid(handle);
        if (fluid == nullptr || (positions == nullptr && count > 0))
        {
            setError(error, FluidError::InvalidHandle);
            return 0u;
        }
        const std::uint32_t current = static_cast<std::uint32_t>(fluid->positions.size() / 3);
        if (current + count > settings_.maximumParticlesPerFluid
            || totalParticles_ + count > settings_.maximumParticles)
        {
            setError(error, FluidError::CapacityExceeded);
            return 0u;
        }
        for (std::uint32_t particle = 0; particle < count; ++particle)
        {
            const std::size_t source = static_cast<std::size_t>(particle) * 3;
            if (!std::isfinite(positions[source]) || !std::isfinite(positions[source + 1])
                || !std::isfinite(positions[source + 2])
                || (velocities != nullptr
                    && (!std::isfinite(velocities[source]) || !std::isfinite(velocities[source + 1])
                        || !std::isfinite(velocities[source + 2]))))
            {
                setError(error, FluidError::InvalidDescription);
                return particle;
            }
            fluid->positions.push_back(positions[source]);
            fluid->positions.push_back(positions[source + 1]);
            fluid->positions.push_back(positions[source + 2]);
            if (velocities != nullptr)
            {
                fluid->velocities.push_back(velocities[source]);
                fluid->velocities.push_back(velocities[source + 1]);
                fluid->velocities.push_back(velocities[source + 2]);
            }
            else
            {
                fluid->velocities.push_back(0.0);
                fluid->velocities.push_back(0.0);
                fluid->velocities.push_back(0.0);
            }
        }
        const std::size_t particleCount = fluid->positions.size() / 3;
        fluid->densities.assign(particleCount, fluid->description.restDensity);
        fluid->pressures.assign(particleCount, 0.0);
        fluid->accelerations.assign(particleCount * 3, 0.0);
        totalParticles_ += count;
        return current;
    }

    bool copyParticlePositionsImpl(
        FluidHandle handle, double *outPositions, std::uint32_t capacity, std::uint32_t *outRequired)
    {
        FluidRecord *fluid = findFluid(handle);
        if (fluid == nullptr || outPositions == nullptr)
        {
            return false;
        }
        const std::uint32_t count = static_cast<std::uint32_t>(fluid->positions.size());
        if (outRequired != nullptr)
        {
            *outRequired = count;
        }
        if (capacity < count)
        {
            return false;
        }
        std::copy(fluid->positions.begin(), fluid->positions.end(), outPositions);
        return true;
    }

    bool copyParticleVelocitiesImpl(
        FluidHandle handle, double *outVelocities, std::uint32_t capacity, std::uint32_t *outRequired)
    {
        FluidRecord *fluid = findFluid(handle);
        if (fluid == nullptr || outVelocities == nullptr)
        {
            return false;
        }
        const std::uint32_t count = static_cast<std::uint32_t>(fluid->velocities.size());
        if (outRequired != nullptr)
        {
            *outRequired = count;
        }
        if (capacity < count)
        {
            return false;
        }
        std::copy(fluid->velocities.begin(), fluid->velocities.end(), outVelocities);
        return true;
    }

    // Internal substep count for one fidelity band: lower bands keep the
    // fixed step size and cut the substep count instead (the small-steps
    // XPBD schedule tolerates the coarser solve).
    std::uint32_t substepsForBand(engine::util::SimulationFidelity band) const
    {
        return engine::util::simulationLodSubstepsForBand(band, settings_.substepsPerStep);
    }

    bool stepImpl(FluidHandle handle, double fixedDeltaSeconds)
    {
        FluidRecord *fluid = findFluid(handle);
        if (fluid == nullptr)
        {
            return false;
        }
        // Simulation LOD: resolve the band and the staggered cadence; a
        // due step keeps the fixed delta and spends the band's substep
        // budget (constrained systems never enlarge the step). Cached
        // fluids freeze. Without lod_enabled every fluid runs Full every
        // step.
        if (engine::util::simulationLodResolvePass(fluid->lodBand,
                fluid->cadence,
                settings_.lod.thresholds,
                settings_.lod.intervals,
                settings_.lod.enabled,
                fluid->importance,
                &fluid->lodSkippedSteps)
            == 0u)
        {
            return true;
        }
        return stepFluid(*fluid, fixedDeltaSeconds, substepsForBand(fluid->lodBand));
    }

    std::uint32_t simulateImpl(FluidHandle handle, double elapsedSeconds)
    {
        FluidRecord *fluid = findFluid(handle);
        if (fluid == nullptr)
        {
            return 0u;
        }
        const double fixedDelta = 1.0 / settings_.fixedRateHz;
        return fluid->accumulator.drain(fixedDelta, settings_.maximumStepsPerSimulate, elapsedSeconds,
            [this, handle, fixedDelta]() { return stepImpl(handle, fixedDelta); });
    }

    bool setFluidImportanceImpl(FluidHandle handle, double importance)
    {
        FluidRecord *fluid = findFluid(handle);
        if (fluid == nullptr || !std::isfinite(importance))
        {
            return false;
        }
        fluid->importance = engine::util::simulationImportanceClamped(importance);
        return true;
    }

    bool fluidStatisticsImpl(FluidHandle handle, FluidStatistics *outStatistics)
    {
        FluidRecord *fluid = findFluid(handle);
        if (fluid == nullptr || outStatistics == nullptr)
        {
            return false;
        }
        outStatistics->stepsSimulated       = fluid->stepsSimulated;
        outStatistics->substepsExecuted     = fluid->substepsExecuted;
        outStatistics->lastContactCount     = fluid->lastContactCount;
        outStatistics->lastAverageDensity   = fluid->lastAverageDensity;
        outStatistics->lastStepMilliseconds = fluid->lastStepMilliseconds;
        outStatistics->lodSkippedSteps      = fluid->lodSkippedSteps;
        return true;
    }

    void assignCapabilityPointers()
    {
        capability_.abi  = physicsFluidCapabilityAbi;
        capability_.self = this;
#define ENGINE_PHYSICS_FLUID_BIND(field, method)                                                                       \
    capability_.field                                                                                                  \
        = [](void *self, auto... args) -> decltype(auto) { return static_cast<FluidModule *>(self)->method(args...); }
        ENGINE_PHYSICS_FLUID_BIND(createFluid, createFluidImpl);
        ENGINE_PHYSICS_FLUID_BIND(destroyFluid, destroyFluidImpl);
        ENGINE_PHYSICS_FLUID_BIND(fluidInfo, fluidInfoImpl);
        ENGINE_PHYSICS_FLUID_BIND(addParticles, addParticlesImpl);
        ENGINE_PHYSICS_FLUID_BIND(copyParticlePositions, copyParticlePositionsImpl);
        ENGINE_PHYSICS_FLUID_BIND(copyParticleVelocities, copyParticleVelocitiesImpl);
        ENGINE_PHYSICS_FLUID_BIND(step, stepImpl);
        ENGINE_PHYSICS_FLUID_BIND(simulate, simulateImpl);
        ENGINE_PHYSICS_FLUID_BIND(fluidStatistics, fluidStatisticsImpl);
        ENGINE_PHYSICS_FLUID_BIND(setFluidImportance, setFluidImportanceImpl);
#undef ENGINE_PHYSICS_FLUID_BIND
    }

    ParticipantManifest manifest_;
    FluidCapability     capability_{};

    engine::physics_rigid::PhysicsRigidCapability *rigid_ = nullptr;

    FluidSettings settings_;

    std::vector<FluidRecord>                     fluids_;
    std::unordered_map<FluidHandle, std::size_t> fluidIndexByHandle_;
    FluidHandle                                  nextFluidHandle_ = 1;
    std::size_t                                  totalParticles_  = 0;

    // Per-substep neighbor scratch.
    std::vector<std::uint32_t> neighbors_;

    // Reused scratch for the batched sphere-overlap queries.
    std::vector<engine::physics_rigid::SphereOverlapQuery>  collisionQueries_;
    std::vector<engine::physics_rigid::SphereOverlapResult> collisionResults_;

    // Registration ordinal for LOD stagger phases.
    std::uint64_t registrationOrdinal_ = 0;
};

} // namespace
} // namespace engine::physics_fluid

// Module entry point queried by the kernel after dlopen: one .so providing
// the physics_fluid capability and requiring physics_rigid.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"physics_fluid"};
    static const char *const                    required[] = {"physics_rigid"};
    static const engine::EngineModuleEntryTable entry{
        .abiVersion           = engine::engineModuleAbiVersion,
        .moduleName           = "physics_fluid",
        .moduleVersion        = "0.1.0",
        .createModule         = []() -> engine::ModuleInterface         *{ return new engine::physics_fluid::FluidModule(); },
        .providedCapabilities = provided,
        .providedCapabilityCount = 1,
        .requiredCapabilities    = required,
        .requiredCapabilityCount = 1,
        .optionalCapabilities    = nullptr,
        .optionalCapabilityCount = 0,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0};
    return &entry;
}
