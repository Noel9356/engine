// records.hpp - the physics_fluid module's plain-data runtime records: SPH
// fluids (particles, per-particle density/pressure, spatial hash scratch)
// and the shared FluidSettings.

#pragma once

#include <engine/physics_fluid/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/util/particle_spatial_hash.hpp>
#include <engine/util/simulation_lod.hpp>
#include <engine/util/step_accumulator.hpp>

#include <cstdint>
#include <vector>

namespace engine::physics_fluid
{

struct FluidRecord
{
    FluidHandle handle = invalidFluid;

    engine::physics_rigid::WorldHandle world = engine::physics_rigid::invalidWorld;

    FluidDescription description{};

    std::vector<double> positions;  // x,y,z per particle
    std::vector<double> velocities; // x,y,z per particle
    std::vector<double> densities;  // per particle
    std::vector<double> pressures;  // per particle
    std::vector<double> accelerations; // x,y,z per particle (per substep)

    // Spatial hash rebuilt per substep through the shared sorted-bucket
    // structure, plus the per-particle candidate scratch the neighbor cache
    // fills from (reused buffers; a warmed-up record rebuilds without
    // allocating).
    engine::util::ParticleSpatialHash particleHash;
    std::vector<std::uint32_t>        neighborCandidates;

    // Per-substep neighbor cache shared by the density and force passes:
    // positions do not change between them, so the neighbor lists collect
    // once per substep and both passes read the same flat structure.
    std::vector<std::uint32_t> neighborStarts;  // per particle: start into neighborIndices
    std::vector<std::uint32_t> neighborCounts;  // per particle
    std::vector<std::uint32_t> neighborIndices; // flat neighbor ids

    // Reused scratch for the batched sphere-overlap queries against the
    // rigid world (one batch per substep).
    std::vector<engine::physics_rigid::SphereOverlapQuery>  collisionQueries;
    std::vector<engine::physics_rigid::SphereOverlapResult> collisionResults;

    double gravity[3] = {0.0, -9.81, 0.0}; // read from the rigid world at creation

    std::uint64_t stepsSimulated   = 0;
    std::uint64_t substepsExecuted = 0;
    std::uint32_t lastContactCount = 0;
    double        lastAverageDensity = 0.0;
    double        lastStepMilliseconds = 0.0;
    std::uint64_t lodSkippedSteps  = 0;

    // Fixed-step budget bookkeeping for simulate().
    engine::util::StepAccumulator accumulator;

    // Simulation LOD: importance-driven fidelity band and staggered cadence
    // (the phase derives from the registration ordinal).
    double                           importance = 1.0;
    engine::util::SimulationFidelity lodBand    = engine::util::SimulationFidelity::Full;
    engine::util::SimulationCadence  cadence{};

    // Pre-step copies of the particle state, restored when a step diverges
    // into non-finite values (reused buffers; capture is one copy per step).
    std::vector<double> preStepPositions;
    std::vector<double> preStepVelocities;
};

struct FluidSettings
{
    double        fixedRateHz             = 60.0;
    std::uint32_t maximumStepsPerSimulate = 8;
    std::uint32_t substepsPerStep         = 4;
    std::uint32_t maximumFluids           = 16;
    std::uint32_t maximumParticles        = 500000;
    std::uint32_t maximumParticlesPerFluid = 50000;
    double        maximumVelocity         = 30.0; // stability clamp (meters per second)

    // Neighbor budget per particle per substep: the SPH passes read at most
    // this many neighbors, so a degenerately dense cluster cannot grow the
    // per-particle neighbor lists (and the force pass) without bound.
    std::uint32_t maximumNeighborsPerParticle = 128;

    // Bounds restitution: the velocity factor a particle keeps along the
    // axis it hits the bounds box on (a small bounce).
    double boundsRestitution = 0.3;

    // Simulation LOD: fidelity bands reduce the internal substep count
    // (Full = substepsPerStep, Simplified = half, Procedural = quarter,
    // Animation = one); a due step at interval N integrates N fixed deltas
    // in one pass, so a reduced-cadence fluid keeps advancing at wall time
    // (the interval table says how many fixed steps each band updates on; 0
    // freezes that band, which is the Cached default).
    engine::util::SimulationLodSettings lod{};
};

} // namespace engine::physics_fluid
