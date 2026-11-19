// physics_cloth module implementation.
//
// XPBD cloth over a physics_rigid world: particles integrate velocities,
// constraints solve in Gauss-Seidel order with compliance-scaled corrections
// (one walk per substep by the "small steps" schedule, or solver_iterations
// walks when configured), constraints tear when stretched past their
// tolerance, overlapping particle pairs repel each other through a
// deterministic spatial hash, and particles resolve against the rigid
// world's bodies and planes through the batched sphere-overlap query. Wind
// is either a uniform acceleration or, for cloths flagged aerodynamicWind, a
// per-face drag-and-lift response against the far-field wind velocity.
// Stepping happens in whole fixed steps with internal substeps, so the
// simulation is independent of frame rate; contact friction retains the
// documented share of tangential velocity per second of contact. Settled
// cloths stop integrating until an interaction - or an obstacle overlapping
// the resting surface - wakes them, and simulation LOD reduces the internal
// substep count for unimportant cloths without touching the fixed step
// size.

#include "records.hpp"
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/physics_cloth/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>
#include <engine/util/module_error.hpp>
#include <engine/util/simulation_lod.hpp>
#include <engine/util/step_accumulator.hpp>
#include <engine/util/xpbd_particles.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine::physics_cloth
{
namespace
{

namespace math = engine::physics_rigid::math;

class ClothModule final : public ModuleInterface
{
public:
    ClothModule()
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
        if (const auto value = context.configurationIntegerInSection("physics_cloth", "maximum_cloths");
            value && *value > 0 && *value <= 4096)
            settings_.maximumCloths = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_cloth", "maximum_particles");
            value && *value > 0 && *value <= 10'000'000)
            settings_.maximumParticles = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_cloth", "maximum_particles_per_cloth");
            value && *value > 0 && *value <= 1'000'000)
            settings_.maximumParticlesPerCloth = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_cloth", "substeps_per_step");
            value && *value > 0 && *value <= 64)
            settings_.substepsPerStep = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("physics_cloth", "maximum_steps_per_simulate");
            value && *value > 0 && *value <= 1000)
            settings_.maximumStepsPerSimulate = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationFloatingInSection("physics_cloth", "fixed_rate_hz");
            value && *value >= 1.0 && *value <= 1000.0 && std::isfinite(*value))
            settings_.fixedRateHz = *value;
        if (const auto value = context.configurationFloatingInSection("physics_cloth", "maximum_velocity");
            value && *value > 0.0 && *value <= 1000.0 && std::isfinite(*value))
            settings_.maximumVelocity = *value;
        if (const auto value = context.configurationFloatingInSection("physics_cloth", "sleep_velocity_threshold");
            value && *value > 0.0 && *value <= 100.0 && std::isfinite(*value))
            settings_.sleepVelocityThreshold = *value;
        if (const auto value = context.configurationFloatingInSection("physics_cloth", "time_to_sleep");
            value && *value >= 0.0 && *value <= 60.0 && std::isfinite(*value))
            settings_.timeToSleep = *value;
        readLodSettings(context);
        readSolverSettings(context);

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
        cloths_.clear();
        clothIndexByHandle_.clear();
        nextClothHandle_ = 1;
        totalParticles_  = 0;
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == ClothCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "physics_cloth",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"physics_cloth"},
            .requiredCapabilities = {"physics_rigid"}};
    }

    void readLodSettings(ParticipantContext &context)
    {
        settings_.lod = engine::util::readSimulationLodSettings(context, "physics_cloth");
    }

    void readSolverSettings(ParticipantContext &context)
    {
        if (const auto value = context.configurationIntegerInSection("physics_cloth", "solver_iterations");
            value && *value > 0 && *value <= 16)
            settings_.solverIterations = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationFloatingInSection("physics_cloth", "wind_drag_coefficient");
            value && *value >= 0.0 && *value <= 100.0 && std::isfinite(*value))
            settings_.windDragCoefficient = *value;
        if (const auto value = context.configurationFloatingInSection("physics_cloth", "wind_lift_coefficient");
            value && *value >= 0.0 && *value <= 100.0 && std::isfinite(*value))
            settings_.windLiftCoefficient = *value;
    }

    static void setError(ClothError *output, ClothError value) noexcept
    {
        engine::util::setModuleError(output, value);
    }

    ClothRecord *findCloth(ClothHandle handle)
    {
        const auto iterator = clothIndexByHandle_.find(handle);
        return iterator != clothIndexByHandle_.end() ? &cloths_[iterator->second] : nullptr;
    }

    std::size_t particleOffset(const ClothRecord &cloth, std::uint32_t column, std::uint32_t row) const
    {
        return (static_cast<std::size_t>(row) * cloth.columns + column) * 3u;
    }

    std::uint32_t particleIndexOf(const ClothRecord &cloth, std::uint32_t column, std::uint32_t row) const
    {
        return static_cast<std::uint32_t>(static_cast<std::size_t>(row) * cloth.columns + column);
    }

    // ---- XPBD stepping -----------------------------------------------------

    void integrateSubstep(ClothRecord &cloth, double substepDelta)
    {
        // The aerodynamic wind acts as a far-field wind VELOCITY against the
        // per-face drag-and-lift model (evaluated against the pre-integration
        // velocities and merged into them before the position advance). The
        // uniform per-particle acceleration below covers the plain-wind case
        // only: with the per-face model active the same vector must not also
        // ride the loop as a raw acceleration, or the surface response
        // doubles.
        if (cloth.aerodynamicWind)
        {
            applyAerodynamicWind(cloth, substepDelta);
        }
        const double windAccelerationX = cloth.aerodynamicWind ? 0.0 : cloth.windAcceleration[0];
        const double windAccelerationY = cloth.aerodynamicWind ? 0.0 : cloth.windAcceleration[1];
        const double windAccelerationZ = cloth.aerodynamicWind ? 0.0 : cloth.windAcceleration[2];
        const std::size_t particleCount = cloth.positions.size() / 3;
        const double      dampingFactor = 1.0 / (1.0 + substepDelta * cloth.damping);
        for (std::size_t particle = 0; particle < particleCount; ++particle)
        {
            const std::size_t offset = particle * 3;
            if (cloth.pinned[particle] != 0u)
            {
                cloth.velocities[offset]            = 0.0;
                cloth.velocities[offset + 1]        = 0.0;
                cloth.velocities[offset + 2]        = 0.0;
                cloth.positions[offset]             = cloth.pinPositions[offset];
                cloth.positions[offset + 1]         = cloth.pinPositions[offset + 1];
                cloth.positions[offset + 2]         = cloth.pinPositions[offset + 2];
                cloth.previousPositions[offset]     = cloth.positions[offset];
                cloth.previousPositions[offset + 1] = cloth.positions[offset + 1];
                cloth.previousPositions[offset + 2] = cloth.positions[offset + 2];
                continue;
            }
            cloth.previousPositions[offset]     = cloth.positions[offset];
            cloth.previousPositions[offset + 1] = cloth.positions[offset + 1];
            cloth.previousPositions[offset + 2] = cloth.positions[offset + 2];
            cloth.velocities[offset] += (cloth.gravity[0] + windAccelerationX) * substepDelta;
            cloth.velocities[offset + 1] += (cloth.gravity[1] + windAccelerationY) * substepDelta;
            cloth.velocities[offset + 2] += (cloth.gravity[2] + windAccelerationZ) * substepDelta;
            cloth.velocities[offset] *= dampingFactor;
            cloth.velocities[offset + 1] *= dampingFactor;
            cloth.velocities[offset + 2] *= dampingFactor;

            // Drag spring toward the grab target (one target per cloth: two
            // grabbed cloths never interfere through module state).
            if (cloth.dragActive && particle == cloth.drag.particleIndex)
            {
                const double errorX  = cloth.dragTarget[0] - cloth.positions[offset];
                const double errorY  = cloth.dragTarget[1] - cloth.positions[offset + 1];
                const double errorZ  = cloth.dragTarget[2] - cloth.positions[offset + 2];
                double accelerationX = cloth.drag.stiffness * errorX - cloth.drag.damping * cloth.velocities[offset];
                double accelerationY
                    = cloth.drag.stiffness * errorY - cloth.drag.damping * cloth.velocities[offset + 1];
                double accelerationZ
                    = cloth.drag.stiffness * errorZ - cloth.drag.damping * cloth.velocities[offset + 2];
                const double magnitude = std::sqrt(
                    accelerationX * accelerationX + accelerationY * accelerationY + accelerationZ * accelerationZ);
                if (magnitude > cloth.drag.maximumAcceleration)
                {
                    const double scale = cloth.drag.maximumAcceleration / magnitude;
                    accelerationX *= scale;
                    accelerationY *= scale;
                    accelerationZ *= scale;
                }
                cloth.velocities[offset] += accelerationX * substepDelta;
                cloth.velocities[offset + 1] += accelerationY * substepDelta;
                cloth.velocities[offset + 2] += accelerationZ * substepDelta;
            }

            cloth.positions[offset] += cloth.velocities[offset] * substepDelta;
            cloth.positions[offset + 1] += cloth.velocities[offset + 1] * substepDelta;
            cloth.positions[offset + 2] += cloth.velocities[offset + 2] * substepDelta;
        }
    }

    // Per-face aerodynamic wind (the Baraff-Witkin model): the per-cloth wind
    // vector is the far-field wind velocity (meters per second), and each
    // grid quad's triangle decomposes the face's relative velocity into a
    // normal-drag share (a1) and two lift shares (a2, a3) evaluated on the
    // triangle's in-plane cross products; the scalar response applies along
    // the triangle normal, split evenly over the triangle's free particles.
    // The wind pushes the surface and makes it flutter instead of sliding
    // every particle uniformly; while this model is active the wind vector
    // never doubles as a uniform acceleration (see integrateSubstep).
    // Deterministic: faces iterate in grid order, pinned particles receive
    // no velocity, and degenerate (zero-area or non-finite) faces are
    // skipped.
    void applyAerodynamicWind(ClothRecord &cloth, double substepDelta)
    {
        if (cloth.faces.empty())
        {
            return;
        }
        for (const ClothFace &face : cloth.faces)
        {
            // Triangle (A, B, D) of the quad: one triangle per cell keeps the
            // force O(cells); the neighboring cell's mirrored triangle covers
            // the shared diagonal's other half.
            const std::size_t   offsets[3]  = { static_cast<std::size_t>(face.particleA) * 3,
                                                static_cast<std::size_t>(face.particleB) * 3,
                                                static_cast<std::size_t>(face.particleD) * 3 };
            const std::uint32_t particles[3] = { face.particleA, face.particleB, face.particleD };

            // Face velocity: the average of the triangle's free particles.
            double        faceVelocity[3] = { 0.0, 0.0, 0.0 };
            std::uint32_t freeCount       = 0;
            for (const std::size_t offset : offsets)
            {
                if (cloth.pinned[offset / 3] == 0u)
                {
                    faceVelocity[0] += cloth.velocities[offset];
                    faceVelocity[1] += cloth.velocities[offset + 1];
                    faceVelocity[2] += cloth.velocities[offset + 2];
                    ++freeCount;
                }
            }
            if (freeCount == 0)
            {
                continue;
            }
            const double inverseFree = 1.0 / static_cast<double>(freeCount);
            faceVelocity[0] *= inverseFree;
            faceVelocity[1] *= inverseFree;
            faceVelocity[2] *= inverseFree;

            // Relative wind against the moving face.
            const double relative[3] = { cloth.windAcceleration[0] - faceVelocity[0],
                                         cloth.windAcceleration[1] - faceVelocity[1],
                                         cloth.windAcceleration[2] - faceVelocity[2] };

            // Triangle edges and normal from the CURRENT positions, so a
            // crumpled face pushes along its crumpled normal.
            const double edge1[3] = { cloth.positions[offsets[1]] - cloth.positions[offsets[0]],
                                      cloth.positions[offsets[1] + 1] - cloth.positions[offsets[0] + 1],
                                      cloth.positions[offsets[1] + 2] - cloth.positions[offsets[0] + 2] };
            const double edge2[3] = { cloth.positions[offsets[2]] - cloth.positions[offsets[0]],
                                      cloth.positions[offsets[2] + 1] - cloth.positions[offsets[0] + 1],
                                      cloth.positions[offsets[2] + 2] - cloth.positions[offsets[0] + 2] };
            const double normalX2 = edge1[1] * edge2[2] - edge1[2] * edge2[1];
            const double normalY2 = edge1[2] * edge2[0] - edge1[0] * edge2[2];
            const double normalZ2 = edge1[0] * edge2[1] - edge1[1] * edge2[0];
            const double normalLength
                = std::sqrt(normalX2 * normalX2 + normalY2 * normalY2 + normalZ2 * normalZ2);
            if (!(normalLength > 1.0e-12) || !std::isfinite(normalLength))
            {
                continue; // degenerate or non-finite face: no wind response
            }
            const double inverseNormalLength = 1.0 / normalLength;
            const double area                = 0.5 * normalLength;
            const double normal[3]           = { normalX2 * inverseNormalLength,
                                                 normalY2 * inverseNormalLength,
                                                 normalZ2 * inverseNormalLength };

            // In-plane unit tangents: e1 normalized, and normal x e1 (unit
            // automatically, because both factors are unit and orthogonal).
            const double edge1LengthSquared
                = edge1[0] * edge1[0] + edge1[1] * edge1[1] + edge1[2] * edge1[2];
            if (!(edge1LengthSquared > 1.0e-24) || !std::isfinite(edge1LengthSquared))
            {
                continue;
            }
            const double edge1InverseLength = 1.0 / std::sqrt(edge1LengthSquared);
            const double tangent1[3]        = { edge1[0] * edge1InverseLength,
                                                edge1[1] * edge1InverseLength,
                                                edge1[2] * edge1InverseLength };
            const double tangent2[3]        = { normal[1] * tangent1[2] - normal[2] * tangent1[1],
                                                normal[2] * tangent1[0] - normal[0] * tangent1[2],
                                                normal[0] * tangent1[1] - normal[1] * tangent1[0] };

            // Baraff-Witkin scalar response: normal drag a1 (n . v) plus the
            // two lift shares a2/a3 evaluated on the in-plane projections of
            // the cross products (n . (v x t1) and n . (v x t2)).
            const double dotNV      = normal[0] * relative[0] + normal[1] * relative[1] + normal[2] * relative[2];
            const double crossT1[3] = { relative[1] * tangent1[2] - relative[2] * tangent1[1],
                                        relative[2] * tangent1[0] - relative[0] * tangent1[2],
                                        relative[0] * tangent1[1] - relative[1] * tangent1[0] };
            const double crossT2[3] = { relative[1] * tangent2[2] - relative[2] * tangent2[1],
                                        relative[2] * tangent2[0] - relative[0] * tangent2[2],
                                        relative[0] * tangent2[1] - relative[1] * tangent2[0] };
            const double lift1      = normal[0] * crossT1[0] + normal[1] * crossT1[1] + normal[2] * crossT1[2];
            const double lift2      = normal[0] * crossT2[0] + normal[1] * crossT2[1] + normal[2] * crossT2[2];
            const double response   = settings_.windDragCoefficient * dotNV
                                    + settings_.windLiftCoefficient * (lift1 + lift2);
            if (response == 0.0)
            {
                continue;
            }

            // Acceleration along the normal, split evenly over the triangle's
            // free particles: the scale carries the face area (resolution
            // changes keep the same material response) and the per-particle
            // mass share.
            const double acceleration
                = -response * area / (3.0 * cloth.particleMass * static_cast<double>(freeCount));
            for (std::uint32_t corner = 0; corner < 3; ++corner)
            {
                const std::size_t particleIndex = particles[corner];
                if (cloth.pinned[particleIndex] != 0u)
                {
                    continue;
                }
                const std::size_t offset = static_cast<std::size_t>(particleIndex) * 3;
                cloth.velocities[offset] += normal[0] * acceleration * substepDelta;
                cloth.velocities[offset + 1] += normal[1] * acceleration * substepDelta;
                cloth.velocities[offset + 2] += normal[2] * acceleration * substepDelta;
            }
        }
    }

    void solveConstraints(ClothRecord &cloth, double substepDelta, bool tearingCheckEnabled)
    {
        const double inverseDeltaTimeSquared = 1.0 / (substepDelta * substepDelta);
        const bool   tearingEnabled          = tearingCheckEnabled && cloth.tearStrain > 0.0;
        const double stretchLimitFactor      = 1.0 + cloth.tearStrain;
        for (DistanceConstraint &constraint : cloth.constraints)
        {
            const std::size_t offsetA = static_cast<std::size_t>(constraint.particleA) * 3;
            const std::size_t offsetB = static_cast<std::size_t>(constraint.particleB) * 3;
            const double      deltaX  = cloth.positions[offsetB] - cloth.positions[offsetA];
            const double      deltaY  = cloth.positions[offsetB + 1] - cloth.positions[offsetA + 1];
            const double      deltaZ  = cloth.positions[offsetB + 2] - cloth.positions[offsetA + 2];
            const double length = std::sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
            // Tearing: check before solving, every substep, in the fixed
            // constraint order. Only stretched constraints break (never
            // compressed), a broken constraint is skipped from then on, and
            // the count accumulates once per torn constraint. Tearing is a
            // normal outcome and never triggers the step's failure path.
            if (tearingEnabled && !constraint.broken && length > constraint.restLength * stretchLimitFactor)
            {
                constraint.broken             = true;
                cloth.brokenConstraintCount += 1;
            }
            if (constraint.broken)
            {
                continue;
            }
            const double inverseMassA
                = cloth.pinned[constraint.particleA] != 0u ? 0.0 : cloth.inverseMasses[constraint.particleA];
            const double inverseMassB
                = cloth.pinned[constraint.particleB] != 0u ? 0.0 : cloth.inverseMasses[constraint.particleB];
            const double weightSum = inverseMassA + inverseMassB;
            if (weightSum <= 0.0)
            {
                continue;
            }
            if (length <= 1.0e-12)
            {
                continue;
            }
            const double complianceScaled = constraint.compliance * inverseDeltaTimeSquared;
            double       lambda           = (-length + constraint.restLength) / (weightSum + complianceScaled);
            // XPBD accumulates lambda per constraint per substep; starting
            // from zero each substep is equivalent for one iteration.
            const double scaleX = deltaX / length * lambda;
            const double scaleY = deltaY / length * lambda;
            const double scaleZ = deltaZ / length * lambda;
            cloth.positions[offsetA] -= inverseMassA * scaleX;
            cloth.positions[offsetA + 1] -= inverseMassA * scaleY;
            cloth.positions[offsetA + 2] -= inverseMassA * scaleZ;
            cloth.positions[offsetB] += inverseMassB * scaleX;
            cloth.positions[offsetB + 1] += inverseMassB * scaleY;
            cloth.positions[offsetB + 2] += inverseMassB * scaleZ;
        }
    }

    // Cell coordinate for the self-collision hash: clamps the scaled
    // position before the integer cast so huge or non-finite positions
    // cannot overflow (the step's finite check rejects them afterwards; the
    // hash stays well-defined meanwhile).
    static std::int64_t selfCollisionCellCoordinate(double scaled)
    {
        if (!std::isfinite(scaled))
        {
            return 0;
        }
        constexpr double extent = 1.0e9;
        scaled                  = std::min(std::max(scaled, -extent), extent);
        return static_cast<std::int64_t>(std::floor(scaled));
    }

    // Bounds-free cell-id hash (the same primes the fluid module uses); two
    // distinct cells colliding into one id merely merge their candidate
    // lists, which the distance check tolerates deterministically.
    static std::uint64_t selfCollisionCellId(std::int64_t cellX, std::int64_t cellY, std::int64_t cellZ)
    {
        return (static_cast<std::uint64_t>(cellX) * 73856093ull)
               ^ (static_cast<std::uint64_t>(cellY) * 19349663ull)
               ^ (static_cast<std::uint64_t>(cellZ) * 83492791ull);
    }

    // Repels particle pairs closer than twice the self-collision radius.
    // Runs after the constraint iterations of each substep and before the
    // rigid-world resolution, so geometry keeps the final positional word.
    // Pinned particles join the hash as infinite-mass obstacles (inverse
    // mass 0, they never move). The hash is a flat, sorted bucket structure
    // with no map iteration anywhere: cells hash into 64-bit ids, the (id,
    // particle) pairs sort by id with an index tiebreak (so every bucket
    // holds ascending particle indices), and both the particle walk and the
    // 3x3x3 neighbor-cell scan follow fixed orders, which keeps pair
    // discovery and correction order deterministic.
    void resolveSelfCollisions(ClothRecord &cloth)
    {
        if (cloth.selfCollisionRadius <= 0.0)
        {
            return;
        }
        const std::size_t particleCount = cloth.positions.size() / 3;
        if (particleCount < 2)
        {
            return;
        }
        const double minimumDistance        = 2.0 * cloth.selfCollisionRadius;
        const double minimumDistanceSquared = minimumDistance * minimumDistance;
        const double cellSize               = minimumDistance; // cell edge >= 2 x radius

        cloth.selfCollisionEntries.resize(particleCount);
        for (std::size_t particle = 0; particle < particleCount; ++particle)
        {
            const std::size_t offset = particle * 3;
            const std::int64_t cellX = selfCollisionCellCoordinate(cloth.positions[offset] / cellSize);
            const std::int64_t cellY = selfCollisionCellCoordinate(cloth.positions[offset + 1] / cellSize);
            const std::int64_t cellZ = selfCollisionCellCoordinate(cloth.positions[offset + 2] / cellSize);
            cloth.selfCollisionEntries[particle].cellId   = selfCollisionCellId(cellX, cellY, cellZ);
            cloth.selfCollisionEntries[particle].particle = static_cast<std::uint32_t>(particle);
        }
        std::sort(cloth.selfCollisionEntries.begin(), cloth.selfCollisionEntries.end(),
            [](const SelfCollisionEntry &a, const SelfCollisionEntry &b)
            { return a.cellId != b.cellId ? a.cellId < b.cellId : a.particle < b.particle; });

        // Unique ascending cell ids and the start offset of each bucket;
        // a bucket ends where the next one starts (the last runs to the end).
        cloth.selfCollisionCells.clear();
        cloth.selfCollisionCellStarts.clear();
        for (std::size_t entry = 0; entry < particleCount; ++entry)
        {
            if (entry == 0 || cloth.selfCollisionEntries[entry].cellId != cloth.selfCollisionCells.back())
            {
                cloth.selfCollisionCells.push_back(cloth.selfCollisionEntries[entry].cellId);
                cloth.selfCollisionCellStarts.push_back(static_cast<std::uint32_t>(entry));
            }
        }

        for (std::uint32_t particle = 0; particle < particleCount; ++particle)
        {
            const std::size_t  offsetI = static_cast<std::size_t>(particle) * 3;
            const std::int64_t cellX   = selfCollisionCellCoordinate(cloth.positions[offsetI] / cellSize);
            const std::int64_t cellY   = selfCollisionCellCoordinate(cloth.positions[offsetI + 1] / cellSize);
            const std::int64_t cellZ   = selfCollisionCellCoordinate(cloth.positions[offsetI + 2] / cellSize);
            for (std::int64_t offsetX = -1; offsetX <= 1; ++offsetX)
            {
                for (std::int64_t offsetY = -1; offsetY <= 1; ++offsetY)
                {
                    for (std::int64_t offsetZ = -1; offsetZ <= 1; ++offsetZ)
                    {
                        const std::uint64_t neighborId
                            = selfCollisionCellId(cellX + offsetX, cellY + offsetY, cellZ + offsetZ);
                        const auto slot = std::lower_bound(
                            cloth.selfCollisionCells.begin(), cloth.selfCollisionCells.end(), neighborId);
                        if (slot == cloth.selfCollisionCells.end() || *slot != neighborId)
                        {
                            continue;
                        }
                        const std::size_t cellIndex  = static_cast<std::size_t>(slot - cloth.selfCollisionCells.begin());
                        const std::uint32_t start    = cloth.selfCollisionCellStarts[cellIndex];
                        const std::uint32_t end      = cellIndex + 1 < cloth.selfCollisionCellStarts.size()
                                                          ? cloth.selfCollisionCellStarts[cellIndex + 1]
                                                          : static_cast<std::uint32_t>(particleCount);
                        for (std::uint32_t entry = start; entry < end; ++entry)
                        {
                            const std::uint32_t other = cloth.selfCollisionEntries[entry].particle;
                            if (other <= particle)
                            {
                                continue; // each pair is handled once, with j > i
                            }
                            const std::size_t offsetJ = static_cast<std::size_t>(other) * 3;
                            const double deltaX = cloth.positions[offsetJ] - cloth.positions[offsetI];
                            const double deltaY = cloth.positions[offsetJ + 1] - cloth.positions[offsetI + 1];
                            const double deltaZ = cloth.positions[offsetJ + 2] - cloth.positions[offsetI + 2];
                            const double distanceSquared
                                = deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ;
                            if (distanceSquared >= minimumDistanceSquared || distanceSquared <= 1.0e-12)
                            {
                                continue;
                            }
                            const double inverseMassI
                                = cloth.pinned[particle] != 0u ? 0.0 : cloth.inverseMasses[particle];
                            const double inverseMassJ
                                = cloth.pinned[other] != 0u ? 0.0 : cloth.inverseMasses[other];
                            const double weightSum = inverseMassI + inverseMassJ;
                            if (weightSum <= 0.0)
                            {
                                continue;
                            }
                            // XPBD positional correction along the pair axis,
                            // split by inverse mass: pinned particles never
                            // move, equal masses split the push in half.
                            const double distance = std::sqrt(distanceSquared);
                            const double scale    = (minimumDistance - distance) / (distance * weightSum);
                            cloth.positions[offsetI] -= inverseMassI * deltaX * scale;
                            cloth.positions[offsetI + 1] -= inverseMassI * deltaY * scale;
                            cloth.positions[offsetI + 2] -= inverseMassI * deltaZ * scale;
                            cloth.positions[offsetJ] += inverseMassJ * deltaX * scale;
                            cloth.positions[offsetJ + 1] += inverseMassJ * deltaY * scale;
                            cloth.positions[offsetJ + 2] += inverseMassJ * deltaZ * scale;
                        }
                    }
                }
            }
        }
    }

    // Resolves every free particle against the rigid world in one batched
    // sphere query per substep instead of one capability call per particle
    // (the shared engine::util batched projection). The caller clears the
    // per-particle contact scratch; the helper writes flags and normals for
    // the hits and pushes the positions out of the obstacles.
    void resolveCollisions(ClothRecord &cloth)
    {
        if (rigid_ == nullptr)
        {
            return;
        }
        const std::size_t particleCount = cloth.positions.size() / 3;
        for (std::size_t particle = 0; particle < particleCount; ++particle)
        {
            cloth.contactFlags[particle] = 0u;
        }
        engine::util::projectParticleSphereOverlaps(rigid_,
            cloth.world,
            cloth.positions.data(),
            particleCount,
            cloth.particleRadius,
            [&cloth](std::size_t particle) { return cloth.pinned[particle] != 0u; },
            collisionQueries_,
            collisionSlotIndices_,
            collisionResults_,
            cloth.contactNormals.data(),
            cloth.contactFlags.data());
    }

    void finalizeSubstepVelocities(ClothRecord &cloth, double substepDelta, double frictionRetention)
    {
        const std::size_t particleCount    = cloth.positions.size() / 3;
        const double      inverseDeltaTime = 1.0 / substepDelta;
        for (std::size_t particle = 0; particle < particleCount; ++particle)
        {
            const std::size_t offset = particle * 3;
            if (cloth.pinned[particle] != 0u)
            {
                continue;
            }
            // The shared finalize re-derives the velocity from the position
            // delta, removes the approaching velocity along the contact
            // normal, scales the tangential remainder by the retention (the
            // per-second "sliding velocity kept" factor raised to the
            // contact time, rate-independent by construction), and clamps
            // to the module's configured maximum.
            engine::util::finalizeParticleVelocity(&cloth.velocities[offset],
                &cloth.positions[offset],
                &cloth.previousPositions[offset],
                inverseDeltaTime,
                cloth.contactFlags[particle] != 0u ? &cloth.contactNormals[offset] : nullptr,
                frictionRetention,
                settings_.maximumVelocity);
        }
    }

    bool stepCloth(ClothRecord &cloth, double fixedDeltaSeconds, std::uint32_t substeps)
    {
        const auto stepStart = std::chrono::steady_clock::now();
        if (!(fixedDeltaSeconds > 0.0) || fixedDeltaSeconds > 0.1 || !std::isfinite(fixedDeltaSeconds))
        {
            return false;
        }
        const double        substepDelta = fixedDeltaSeconds / static_cast<double>(substeps);
        // Friction retention per contact substep: the capability documents the
        // tangential velocity kept per SECOND of contact, so the per-substep
        // multiplier is friction^(delta) - rate-independent by construction.
        const double frictionRetention = cloth.friction > 0.0 ? std::pow(cloth.friction, substepDelta) : 0.0;
        // Pre-step copies for the rollback path: a diverged step restores
        // this state and reports failure, so the caller sees an untouched
        // cloth.
        cloth.preStepPositions.resize(cloth.positions.size());
        cloth.preStepVelocities.resize(cloth.velocities.size());
        std::copy(cloth.positions.begin(), cloth.positions.end(), cloth.preStepPositions.begin());
        std::copy(cloth.velocities.begin(), cloth.velocities.end(), cloth.preStepVelocities.begin());
        for (std::uint32_t substep = 0; substep < substeps; ++substep)
        {
            integrateSubstep(cloth, substepDelta);
            // The constraint walk repeats solverIterations times per substep;
            // tearing is judged on the first pass only, so the breakage
            // criterion stays the stretch at the substep's start (the
            // small-steps schedule is the iterations = 1 default).
            for (std::uint32_t iteration = 0; iteration < settings_.solverIterations; ++iteration)
            {
                solveConstraints(cloth, substepDelta, iteration == 0);
            }
            resolveSelfCollisions(cloth);
            resolveCollisions(cloth);
            finalizeSubstepVelocities(cloth, substepDelta, frictionRetention);
        }
        cloth.lastContactCount = static_cast<std::uint32_t>(
            std::count(cloth.contactFlags.begin(), cloth.contactFlags.end(), std::uint8_t{1}));
        bool finite = true;
        for (const double value : cloth.positions)
        {
            if (!std::isfinite(value))
            {
                finite = false;
                break;
            }
        }
        for (const double value : cloth.velocities)
        {
            if (!std::isfinite(value))
            {
                finite = false;
                break;
            }
        }
        if (!finite)
        {
            std::copy(cloth.preStepPositions.begin(), cloth.preStepPositions.end(), cloth.positions.begin());
            std::copy(cloth.preStepVelocities.begin(), cloth.preStepVelocities.end(), cloth.velocities.begin());
            cloth.sleepTimer = 0.0;
            return false;
        }
        cloth.stepsSimulated += 1;
        cloth.substepsExecuted += substeps;
        const auto stepEnd         = std::chrono::steady_clock::now();
        cloth.lastStepMilliseconds = std::chrono::duration<double, std::milli>(stepEnd - stepStart).count();
        return true;
    }

    // Settled-cloth bookkeeping after an executed step: the sleep timer
    // advances while every free particle stays below the threshold, and the
    // cloth sleeps (velocities zeroed) after the configured quiet period.
    void updateSleepState(ClothRecord &cloth, double fixedDeltaSeconds)
    {
        const double thresholdSquared = settings_.sleepVelocityThreshold * settings_.sleepVelocityThreshold;
        bool quiet = true;
        for (std::size_t particle = 0; particle < cloth.positions.size() / 3; ++particle)
        {
            if (cloth.pinned[particle] != 0u)
            {
                continue;
            }
            const std::size_t offset = particle * 3;
            const double speedSquared = cloth.velocities[offset] * cloth.velocities[offset]
                + cloth.velocities[offset + 1] * cloth.velocities[offset + 1]
                + cloth.velocities[offset + 2] * cloth.velocities[offset + 2];
            if (speedSquared >= thresholdSquared)
            {
                quiet = false;
                break;
            }
        }
        if (quiet)
        {
            cloth.sleepTimer += fixedDeltaSeconds;
            if (cloth.sleepingEnabled && cloth.sleepTimer >= settings_.timeToSleep)
            {
                cloth.sleeping = true;
                for (double &velocity : cloth.velocities)
                {
                    velocity = 0.0;
                }
            }
        }
        else
        {
            cloth.sleepTimer = 0.0;
            cloth.sleeping   = false;
        }
    }

    // ---- capability implementations ---------------------------------------

    ClothHandle createClothImpl(
        engine::physics_rigid::WorldHandle world, const ClothDescription *description, ClothError *error)
    {
        setError(error, ClothError::None);
        if (rigid_ == nullptr)
        {
            setError(error, ClothError::InvalidHandle);
            return invalidCloth;
        }
        if (description == nullptr || description->columns < 2 || description->rows < 2)
        {
            setError(error, ClothError::InvalidDescription);
            return invalidCloth;
        }
        const std::size_t particleCount = static_cast<std::size_t>(description->columns) * description->rows;
        if (particleCount > settings_.maximumParticlesPerCloth
            || totalParticles_ + particleCount > settings_.maximumParticles
            || cloths_.size() >= settings_.maximumCloths)
        {
            setError(error, ClothError::CapacityExceeded);
            return invalidCloth;
        }
        if (!math::isFinite(description->transform.position) || !std::isfinite(description->spacing)
            || description->spacing <= 0.0 || !std::isfinite(description->particleMass)
            || description->particleMass <= 0.0 || !std::isfinite(description->stretchCompliance)
            || description->stretchCompliance < 0.0 || !std::isfinite(description->shearCompliance)
            || description->shearCompliance < 0.0 || !std::isfinite(description->bendCompliance)
            || description->bendCompliance < 0.0 || !std::isfinite(description->damping) || description->damping < 0.0
            || !std::isfinite(description->friction) || description->friction < 0.0 || description->friction > 1.0
            || !std::isfinite(description->tearStrain) || description->tearStrain < 0.0
            || !std::isfinite(description->selfCollisionRadius) || description->selfCollisionRadius < 0.0)
        {
            setError(error, ClothError::InvalidDescription);
            return invalidCloth;
        }

        ClothRecord cloth;
        cloth.world        = world;
        cloth.columns      = description->columns;
        cloth.rows         = description->rows;
        cloth.spacing      = description->spacing;
        cloth.particleMass = description->particleMass;
        // Capturing the gravity doubles as the world-handle validation: a
        // world the rigid module does not know is rejected here instead of
        // silently falling back to the default gravity.
        if (!rigid_->copyWorldGravity(rigid_->self, world, cloth.gravity))
        {
            setError(error, ClothError::InvalidHandle);
            return invalidCloth;
        }
        cloth.particleRadius
            = description->particleRadius > 0.0 ? description->particleRadius : description->spacing * 0.25;
        cloth.friction          = description->friction;
        cloth.damping           = description->damping;
        cloth.stretchCompliance = description->stretchCompliance;
        cloth.shearCompliance   = description->shearCompliance;
        cloth.bendCompliance    = description->bendCompliance;
        cloth.tearStrain          = description->tearStrain;
        cloth.selfCollisionRadius = description->selfCollisionRadius;
        cloth.aerodynamicWind     = description->aerodynamicWind;

        cloth.positions.resize(particleCount * 3);
        cloth.velocities.assign(particleCount * 3, 0.0);
        cloth.previousPositions.resize(particleCount * 3);
        cloth.inverseMasses.assign(particleCount, 1.0 / description->particleMass);
        cloth.pinned.assign(particleCount, 0u);
        cloth.pinPositions.resize(particleCount * 3);
        cloth.contactFlags.assign(particleCount, 0u);
        cloth.contactNormals.resize(particleCount * 3);

        const math::Mat3 rotation = math::makeRotationMatrix(description->transform.rotation);
        for (std::uint32_t row = 0; row < description->rows; ++row)
        {
            for (std::uint32_t column = 0; column < description->columns; ++column)
            {
                const std::size_t index = particleOffset(cloth, column, row);
                const math::Vec3  local{
                    static_cast<double>(column) * description->spacing,
                    0.0,
                    static_cast<double>(row) * description->spacing};
                const math::Vec3 world = description->transform.position + math::multiplyMatrixVector(rotation, local);
                cloth.positions[index] = world.x;
                cloth.positions[index + 1]         = world.y;
                cloth.positions[index + 2]         = world.z;
                cloth.previousPositions[index]     = world.x;
                cloth.previousPositions[index + 1] = world.y;
                cloth.previousPositions[index + 2] = world.z;
                cloth.pinPositions[index]          = world.x;
                cloth.pinPositions[index + 1]      = world.y;
                cloth.pinPositions[index + 2]      = world.z;
            }
        }

        // Structural edges (grid neighbors), shear edges (diagonals), and
        // bend edges (two apart along each axis).
        const auto addConstraint = [&](std::uint32_t a, std::uint32_t b, double compliance)
        {
            DistanceConstraint constraint;
            constraint.particleA      = a;
            constraint.particleB      = b;
            constraint.compliance     = compliance;
            const std::size_t offsetA = static_cast<std::size_t>(a) * 3;
            const std::size_t offsetB = static_cast<std::size_t>(b) * 3;
            constraint.restLength
                = std::sqrt(std::pow(cloth.positions[offsetB] - cloth.positions[offsetA], 2)
                            + std::pow(cloth.positions[offsetB + 1] - cloth.positions[offsetA + 1], 2)
                            + std::pow(cloth.positions[offsetB + 2] - cloth.positions[offsetA + 2], 2));
            cloth.constraints.push_back(constraint);
        };
        // Grid quads for the aerodynamic wind model: one entry per cell,
        // stored in row-major order so the per-substep wind walk is a flat,
        // deterministic loop. Built for every cloth (cheap: one quad per
        // cell); the model consumes them only when configured.
        cloth.faces.reserve((static_cast<std::size_t>(description->rows) - 1)
                            * (static_cast<std::size_t>(description->columns) - 1));
        for (std::uint32_t row = 0; row + 1 < description->rows; ++row)
        {
            for (std::uint32_t column = 0; column + 1 < description->columns; ++column)
            {
                ClothFace face;
                face.particleA = particleIndexOf(cloth, column, row);
                face.particleB = particleIndexOf(cloth, column + 1, row);
                face.particleC = particleIndexOf(cloth, column, row + 1);
                face.particleD = particleIndexOf(cloth, column + 1, row + 1);
                cloth.faces.push_back(face);
            }
        }
        for (std::uint32_t row = 0; row < description->rows; ++row)
        {
            for (std::uint32_t column = 0; column < description->columns; ++column)
            {
                const std::uint32_t index = particleIndexOf(cloth, column, row);
                if (column + 1 < description->columns)
                {
                    addConstraint(index,
                        particleIndexOf(cloth, column + 1, row),
                        description->stretchCompliance);
                }
                if (row + 1 < description->rows)
                {
                    addConstraint(index,
                        particleIndexOf(cloth, column, row + 1),
                        description->stretchCompliance);
                }
                if (column + 1 < description->columns && row + 1 < description->rows)
                {
                    addConstraint(index,
                        particleIndexOf(cloth, column + 1, row + 1),
                        description->shearCompliance);
                    addConstraint(particleIndexOf(cloth, column + 1, row),
                        particleIndexOf(cloth, column, row + 1),
                        description->shearCompliance);
                }
                if (column + 2 < description->columns)
                {
                    addConstraint(index,
                        particleIndexOf(cloth, column + 2, row),
                        description->bendCompliance);
                }
                if (row + 2 < description->rows)
                {
                    addConstraint(index,
                        particleIndexOf(cloth, column, row + 2),
                        description->bendCompliance);
                }
            }
        }

        for (std::uint32_t pinnedIndex = 0; pinnedIndex < description->pinnedCount; ++pinnedIndex)
        {
            const std::uint32_t particle = description->pinnedParticles[pinnedIndex];
            if (particle >= particleCount)
            {
                setError(error, ClothError::InvalidDescription);
                return invalidCloth;
            }
            cloth.pinned[particle]        = 1u;
            cloth.inverseMasses[particle] = 0.0;
        }

        if (nextClothHandle_ == invalidCloth)
        {
            setError(error, ClothError::CapacityExceeded);
            return invalidCloth;
        }
        cloth.handle = nextClothHandle_;
        nextClothHandle_
            = nextClothHandle_ == std::numeric_limits<ClothHandle>::max() ? invalidCloth : nextClothHandle_ + 1;
        // Simulation LOD: the stagger phase derives from the registration
        // ordinal, so cloths sharing an interval spread their work across
        // steps.
        cloth.sleepingEnabled = description->sleepingEnabled;
        cloth.cadence.phase   = static_cast<std::uint32_t>(registrationOrdinal_++);
        totalParticles_ += particleCount;
        cloths_.push_back(std::move(cloth));
        clothIndexByHandle_.emplace(cloths_.back().handle, cloths_.size() - 1);
        return cloths_.back().handle;
    }

    void destroyClothImpl(ClothHandle handle)
    {
        const auto iterator = clothIndexByHandle_.find(handle);
        if (iterator == clothIndexByHandle_.end())
        {
            return;
        }
        totalParticles_ -= cloths_[iterator->second].positions.size() / 3;
        const std::size_t index = iterator->second;
        clothIndexByHandle_.erase(iterator);
        if (index + 1 != cloths_.size())
        {
            std::swap(cloths_[index], cloths_.back());
            clothIndexByHandle_[cloths_[index].handle] = index;
        }
        cloths_.pop_back();
    }

    bool clothInfoImpl(ClothHandle handle, ClothInfo *outInfo)
    {
        ClothRecord *cloth = findCloth(handle);
        if (cloth == nullptr || outInfo == nullptr)
        {
            return false;
        }
        outInfo->columns         = cloth->columns;
        outInfo->rows            = cloth->rows;
        outInfo->particleCount   = static_cast<std::uint32_t>(cloth->positions.size() / 3);
        outInfo->constraintCount = static_cast<std::uint32_t>(cloth->constraints.size());
        return true;
    }

    bool copyParticlePositionsImpl(
        ClothHandle handle, double *outPositions, std::uint32_t capacity, std::uint32_t *outRequired)
    {
        ClothRecord *cloth = findCloth(handle);
        if (cloth == nullptr || outPositions == nullptr)
        {
            return false;
        }
        const std::uint32_t count = static_cast<std::uint32_t>(cloth->positions.size());
        if (outRequired != nullptr)
        {
            *outRequired = count;
        }
        if (capacity < count)
        {
            return false;
        }
        std::copy(cloth->positions.begin(), cloth->positions.end(), outPositions);
        return true;
    }

    bool copyParticleVelocitiesImpl(
        ClothHandle handle, double *outVelocities, std::uint32_t capacity, std::uint32_t *outRequired)
    {
        ClothRecord *cloth = findCloth(handle);
        if (cloth == nullptr || outVelocities == nullptr)
        {
            return false;
        }
        const std::uint32_t count = static_cast<std::uint32_t>(cloth->velocities.size());
        if (outRequired != nullptr)
        {
            *outRequired = count;
        }
        if (capacity < count)
        {
            return false;
        }
        std::copy(cloth->velocities.begin(), cloth->velocities.end(), outVelocities);
        return true;
    }

    bool setParticlePinnedImpl(ClothHandle handle, std::uint32_t particleIndex, bool pinned)
    {
        ClothRecord *cloth = findCloth(handle);
        if (cloth == nullptr || particleIndex >= cloth->pinned.size())
        {
            return false;
        }
        const std::size_t offset = static_cast<std::size_t>(particleIndex) * 3;
        cloth->sleeping          = false;
        cloth->sleepTimer        = 0.0;
        if (pinned && cloth->pinned[particleIndex] == 0u)
        {
            cloth->pinned[particleIndex]        = 1u;
            cloth->inverseMasses[particleIndex] = 0.0;
            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                cloth->pinPositions[offset + axis] = cloth->positions[offset + axis];
            }
        }
        else if (!pinned && cloth->pinned[particleIndex] != 0u)
        {
            cloth->pinned[particleIndex]        = 0u;
            cloth->inverseMasses[particleIndex] = cloth->particleMass > 0.0 ? 1.0 / cloth->particleMass : 1.0;
        }
        return true;
    }

    bool pinParticleToPositionImpl(ClothHandle handle, std::uint32_t particleIndex, const double position[3])
    {
        ClothRecord *cloth = findCloth(handle);
        if (cloth == nullptr || particleIndex >= cloth->pinned.size() || position == nullptr)
        {
            return false;
        }
        if (!std::isfinite(position[0]) || !std::isfinite(position[1]) || !std::isfinite(position[2]))
        {
            return false;
        }
        // Moving a pin is an interaction: a settled cloth must wake so the
        // moved pin pulls the surface along instead of holding the old pose.
        cloth->sleeping   = false;
        cloth->sleepTimer = 0.0;
        cloth->pinned[particleIndex]        = 1u;
        cloth->inverseMasses[particleIndex] = 0.0;
        const std::size_t offset            = static_cast<std::size_t>(particleIndex) * 3;
        cloth->pinPositions[offset]         = position[0];
        cloth->pinPositions[offset + 1]     = position[1];
        cloth->pinPositions[offset + 2]     = position[2];
        return true;
    }

    bool isParticlePinnedImpl(ClothHandle handle, std::uint32_t particleIndex, bool *outPinned)
    {
        ClothRecord *cloth = findCloth(handle);
        if (cloth == nullptr || particleIndex >= cloth->pinned.size() || outPinned == nullptr)
        {
            return false;
        }
        *outPinned = cloth->pinned[particleIndex] != 0u;
        return true;
    }

    bool beginParticleDragImpl(ClothHandle handle, const ClothParticleDrag *drag)
    {
        ClothRecord *cloth = findCloth(handle);
        if (cloth == nullptr || drag == nullptr || drag->particleIndex >= cloth->pinned.size())
        {
            return false;
        }
        if (!std::isfinite(drag->stiffness) || !std::isfinite(drag->damping)
            || !std::isfinite(drag->maximumAcceleration) || drag->stiffness < 0.0 || drag->damping < 0.0
            || drag->maximumAcceleration < 0.0)
        {
            return false;
        }
        cloth->drag         = *drag;
        cloth->dragActive   = true;
        cloth->sleeping     = false;
        cloth->sleepTimer   = 0.0;
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            cloth->dragTarget[axis] = cloth->positions[static_cast<std::size_t>(drag->particleIndex) * 3 + axis];
        }
        return true;
    }

    bool setParticleDragTargetImpl(ClothHandle handle, const double target[3])
    {
        ClothRecord *cloth = findCloth(handle);
        if (cloth == nullptr || !cloth->dragActive || target == nullptr)
        {
            return false;
        }
        if (!std::isfinite(target[0]) || !std::isfinite(target[1]) || !std::isfinite(target[2]))
        {
            return false;
        }
        cloth->dragTarget[0] = target[0];
        cloth->dragTarget[1] = target[1];
        cloth->dragTarget[2] = target[2];
        return true;
    }

    bool endParticleDragImpl(ClothHandle handle)
    {
        ClothRecord *cloth = findCloth(handle);
        if (cloth == nullptr)
        {
            return false;
        }
        cloth->dragActive = false;
        return true;
    }

    bool setWindAccelerationImpl(ClothHandle handle, const double wind[3])
    {
        ClothRecord *cloth = findCloth(handle);
        if (cloth == nullptr || wind == nullptr)
        {
            return false;
        }
        if (!std::isfinite(wind[0]) || !std::isfinite(wind[1]) || !std::isfinite(wind[2]))
        {
            return false;
        }
        cloth->sleeping   = false;
        cloth->sleepTimer = 0.0;
        cloth->windAcceleration[0] = wind[0];
        cloth->windAcceleration[1] = wind[1];
        cloth->windAcceleration[2] = wind[2];
        return true;
    }

    // Internal substep count for one fidelity band: lower bands keep the
    // fixed step size and cut the substep count instead (the small-steps
    // XPBD schedule tolerates the coarser solve).
    std::uint32_t substepsForBand(engine::util::SimulationFidelity band) const
    {
        return engine::util::simulationLodSubstepsForBand(band, settings_.substepsPerStep);
    }

    // Contact probe for sleeping cloths: resolves the same batched sphere
    // query the live path uses but WITHOUT touching positions, and reports
    // whether any obstacle overlaps the resting surface. Running one query
    // per due step is the price of never letting a moving rigid body tunnel
    // through settled cloth; the query is the module's cheapest surface
    // (the rigid world's broadphase answers it without narrow-phase work).
    bool clothSleepingContactWake(ClothRecord *cloth)
    {
        if (rigid_ == nullptr)
        {
            return false;
        }
        const std::size_t particleCount = cloth->positions.size() / 3;
        collisionQueries_.clear();
        for (std::size_t particle = 0; particle < particleCount; ++particle)
        {
            if (cloth->pinned[particle] != 0u)
            {
                continue;
            }
            engine::physics_rigid::SphereOverlapQuery query;
            const std::size_t offset = particle * 3;
            query.center[0]          = cloth->positions[offset];
            query.center[1]          = cloth->positions[offset + 1];
            query.center[2]          = cloth->positions[offset + 2];
            query.radius             = cloth->particleRadius;
            collisionQueries_.push_back(query);
        }
        if (collisionQueries_.empty())
        {
            return false;
        }
        collisionResults_.resize(collisionQueries_.size());
        rigid_->collideSpheres(rigid_->self, cloth->world, collisionQueries_.data(),
            static_cast<std::uint32_t>(collisionQueries_.size()), collisionResults_.data());
        for (const engine::physics_rigid::SphereOverlapResult &result : collisionResults_)
        {
            if (result.hit && result.depth > 0.0)
            {
                return true;
            }
        }
        return false;
    }

    bool stepImpl(ClothHandle handle, double fixedDeltaSeconds)
    {
        ClothRecord *cloth = findCloth(handle);
        if (cloth == nullptr)
        {
            return false;
        }
        // Simulation LOD: resolve the band and the staggered cadence; a
        // due step keeps the fixed delta and spends the band's substep
        // budget (constrained systems never enlarge the step). Cached
        // cloths freeze. Without lod_enabled every cloth runs Full every
        // step.
        if (engine::util::simulationLodResolvePass(cloth->lodBand,
                cloth->cadence,
                settings_.lod.thresholds,
                settings_.lod.intervals,
                settings_.lod.enabled,
                cloth->importance,
                &cloth->lodSkippedSteps)
            == 0u)
        {
            return true;
        }
        // A settled cloth holds its state until drag, pin, or wind changes
        // wake it (those setters clear the sleeping flag) - or until the
        // collision probe below finds an obstacle overlapping the resting
        // surface, so a moving body or a newly spawned box wakes the cloth
        // instead of tunneling through it.
        if (cloth->sleeping)
        {
            if (!clothSleepingContactWake(cloth))
            {
                return true;
            }
            cloth->sleeping   = false;
            cloth->sleepTimer = 0.0;
        }
        if (!stepCloth(*cloth, fixedDeltaSeconds, substepsForBand(cloth->lodBand)))
        {
            return false;
        }
        updateSleepState(*cloth, fixedDeltaSeconds);
        return true;
    }

    std::uint32_t simulateImpl(ClothHandle handle, double elapsedSeconds)
    {
        ClothRecord *cloth = findCloth(handle);
        if (cloth == nullptr)
        {
            return 0u;
        }
        const double fixedDelta = 1.0 / settings_.fixedRateHz;
        return cloth->accumulator.drain(fixedDelta, settings_.maximumStepsPerSimulate, elapsedSeconds,
            [this, handle, fixedDelta]() { return stepImpl(handle, fixedDelta); });
    }

    bool setClothImportanceImpl(ClothHandle handle, double importance)
    {
        ClothRecord *cloth = findCloth(handle);
        if (cloth == nullptr || !std::isfinite(importance))
        {
            return false;
        }
        cloth->importance = engine::util::simulationImportanceClamped(importance);
        return true;
    }

    bool clothStatisticsImpl(ClothHandle handle, ClothStatistics *outStatistics)
    {
        ClothRecord *cloth = findCloth(handle);
        if (cloth == nullptr || outStatistics == nullptr)
        {
            return false;
        }
        outStatistics->stepsSimulated        = cloth->stepsSimulated;
        outStatistics->substepsExecuted      = cloth->substepsExecuted;
        outStatistics->lastContactCount      = cloth->lastContactCount;
        outStatistics->brokenConstraintCount = cloth->brokenConstraintCount;
        outStatistics->lastStepMilliseconds  = cloth->lastStepMilliseconds;
        outStatistics->lodSkippedSteps       = cloth->lodSkippedSteps;
        outStatistics->sleeping              = cloth->sleeping;
        return true;
    }

    // Bilinear interpolation of the particle grid at normalized coordinates
    // (u along columns, v along rows, clamped into [0, 1]; the fractional
    // cell index uses the last full segment so u = 1 lands exactly on the
    // final particle column). The normal is the cross product of the two
    // grid-direction tangents — the central differences of the surrounding
    // particles blended across the sampled cell — ordered so a flat grid
    // (u along +X, v along +Z) faces +Y like the degenerate fallback.
    bool sampleClothSurfaceImpl(ClothHandle handle, double u, double v, double outPosition[3], double outNormal[3])
    {
        ClothRecord *cloth = findCloth(handle);
        if (cloth == nullptr || outPosition == nullptr || outNormal == nullptr)
        {
            return false;
        }
        if (!std::isfinite(u) || !std::isfinite(v))
        {
            return false;
        }
        const double fractionalColumn
            = std::min(std::max(u, 0.0), 1.0) * static_cast<double>(cloth->columns - 1);
        const double fractionalRow = std::min(std::max(v, 0.0), 1.0) * static_cast<double>(cloth->rows - 1);
        std::uint32_t column = static_cast<std::uint32_t>(fractionalColumn);
        std::uint32_t row    = static_cast<std::uint32_t>(fractionalRow);
        if (column > cloth->columns - 2)
        {
            column = cloth->columns - 2;
        }
        if (row > cloth->rows - 2)
        {
            row = cloth->rows - 2;
        }
        const double tx = fractionalColumn - static_cast<double>(column);
        const double ty = fractionalRow - static_cast<double>(row);

        const std::size_t offset00 = particleOffset(*cloth, column, row);
        const std::size_t offset10 = particleOffset(*cloth, column + 1, row);
        const std::size_t offset01 = particleOffset(*cloth, column, row + 1);
        const std::size_t offset11 = particleOffset(*cloth, column + 1, row + 1);

        double position[3] = {0.0, 0.0, 0.0};
        double tangentU[3] = {0.0, 0.0, 0.0};
        double tangentV[3] = {0.0, 0.0, 0.0};
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            const double p00 = cloth->positions[offset00 + axis];
            const double p10 = cloth->positions[offset10 + axis];
            const double p01 = cloth->positions[offset01 + axis];
            const double p11 = cloth->positions[offset11 + axis];
            position[axis] = (1.0 - tx) * ((1.0 - ty) * p00 + ty * p01) + tx * ((1.0 - ty) * p10 + ty * p11);
            tangentU[axis] = (1.0 - ty) * (p10 - p00) + ty * (p11 - p01);
            tangentV[axis] = (1.0 - tx) * (p01 - p00) + tx * (p11 - p10);
        }
        const double normalX = tangentV[1] * tangentU[2] - tangentV[2] * tangentU[1];
        const double normalY = tangentV[2] * tangentU[0] - tangentV[0] * tangentU[2];
        const double normalZ = tangentV[0] * tangentU[1] - tangentV[1] * tangentU[0];
        const double normalLength = std::sqrt(normalX * normalX + normalY * normalY + normalZ * normalZ);
        double normal[3]          = {0.0, 1.0, 0.0};
        if (normalLength > 1.0e-12)
        {
            const double inverseLength = 1.0 / normalLength;
            normal[0]                  = normalX * inverseLength;
            normal[1]                  = normalY * inverseLength;
            normal[2]                  = normalZ * inverseLength;
        }
        if (!std::isfinite(position[0]) || !std::isfinite(position[1]) || !std::isfinite(position[2])
            || !std::isfinite(normal[0]) || !std::isfinite(normal[1]) || !std::isfinite(normal[2]))
        {
            return false;
        }
        outPosition[0] = position[0];
        outPosition[1] = position[1];
        outPosition[2] = position[2];
        outNormal[0]   = normal[0];
        outNormal[1]   = normal[1];
        outNormal[2]   = normal[2];
        return true;
    }

    void assignCapabilityPointers()
    {
        capability_.abi  = physicsClothCapabilityAbi;
        capability_.self = this;
#define ENGINE_PHYSICS_CLOTH_BIND(field, method)                                                                       \
    capability_.field                                                                                                  \
        = [](void *self, auto... args) -> decltype(auto) { return static_cast<ClothModule *>(self)->method(args...); }
        ENGINE_PHYSICS_CLOTH_BIND(createCloth, createClothImpl);
        ENGINE_PHYSICS_CLOTH_BIND(destroyCloth, destroyClothImpl);
        ENGINE_PHYSICS_CLOTH_BIND(clothInfo, clothInfoImpl);
        ENGINE_PHYSICS_CLOTH_BIND(copyParticlePositions, copyParticlePositionsImpl);
        ENGINE_PHYSICS_CLOTH_BIND(copyParticleVelocities, copyParticleVelocitiesImpl);
        ENGINE_PHYSICS_CLOTH_BIND(setParticlePinned, setParticlePinnedImpl);
        ENGINE_PHYSICS_CLOTH_BIND(pinParticleToPosition, pinParticleToPositionImpl);
        ENGINE_PHYSICS_CLOTH_BIND(isParticlePinned, isParticlePinnedImpl);
        ENGINE_PHYSICS_CLOTH_BIND(beginParticleDrag, beginParticleDragImpl);
        ENGINE_PHYSICS_CLOTH_BIND(setParticleDragTarget, setParticleDragTargetImpl);
        ENGINE_PHYSICS_CLOTH_BIND(endParticleDrag, endParticleDragImpl);
        ENGINE_PHYSICS_CLOTH_BIND(setWindAcceleration, setWindAccelerationImpl);
        ENGINE_PHYSICS_CLOTH_BIND(step, stepImpl);
        ENGINE_PHYSICS_CLOTH_BIND(simulate, simulateImpl);
        ENGINE_PHYSICS_CLOTH_BIND(clothStatistics, clothStatisticsImpl);
        ENGINE_PHYSICS_CLOTH_BIND(sampleClothSurface, sampleClothSurfaceImpl);
        ENGINE_PHYSICS_CLOTH_BIND(setClothImportance, setClothImportanceImpl);
#undef ENGINE_PHYSICS_CLOTH_BIND
    }

    ParticipantManifest manifest_;
    ClothCapability     capability_{};

    engine::physics_rigid::PhysicsRigidCapability *rigid_ = nullptr;

    ClothSettings settings_;

    std::vector<ClothRecord>                     cloths_;
    std::unordered_map<ClothHandle, std::size_t> clothIndexByHandle_;
    ClothHandle                                  nextClothHandle_ = 1;
    std::size_t                                  totalParticles_  = 0;

    // Reused scratch for the batched sphere-overlap queries.
    std::vector<std::size_t>                               collisionSlotIndices_;
    std::vector<engine::physics_rigid::SphereOverlapQuery>  collisionQueries_;
    std::vector<engine::physics_rigid::SphereOverlapResult> collisionResults_;

    // Registration ordinal for LOD stagger phases.
    std::uint64_t registrationOrdinal_ = 0;
};

} // namespace

} // namespace engine::physics_cloth

// Module entry point queried by the kernel after dlopen: one .so providing
// the physics_cloth capability and requiring physics_rigid.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"physics_cloth"};
    static const char *const                    required[] = {"physics_rigid"};
    static const engine::EngineModuleEntryTable entry{
        .abiVersion           = engine::engineModuleAbiVersion,
        .moduleName           = "physics_cloth",
        .moduleVersion        = "0.1.0",
        .createModule         = []() -> engine::ModuleInterface         *{ return new engine::physics_cloth::ClothModule(); },
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
