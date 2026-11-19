// records.hpp - the physics_fluid module's plain-data runtime records: SPH
// fluids (particles, per-particle density/pressure, spatial hash scratch)
// and the shared FluidSettings.

#pragma once

#include <engine/physics_fluid/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
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

    // Spatial hash scratch rebuilt per substep: cell start offsets into the
    // sorted particle index list, plus the ordered cell ids and counts.
    std::vector<std::uint32_t> cellBuckets;
    std::vector<std::uint32_t> sortedIndices;
    std::vector<std::uint64_t> hashCellIds;
    std::vector<std::uint32_t> hashCellCounts;
    double hashCellSize = 0.0;
    double hashOrigin[3] = {0.0, 0.0, 0.0};

    // Per-substep neighbor cache shared by the density and force passes:
    // positions do not change between them, so the neighbor lists collect
    // once per substep and both passes read the same flat structure.
    std::vector<std::uint32_t> neighborStarts;  // per particle: start into neighborIndices
    std::vector<std::uint32_t> neighborCounts;  // per particle
    std::vector<std::uint32_t> neighborIndices; // flat neighbor ids

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

    // Bounds restitution: the velocity factor a particle keeps along the
    // axis it hits the bounds box on (a small bounce).
    double boundsRestitution = 0.3;

    // Simulation LOD: fidelity bands keep the fixed step size and reduce the
    // internal substep count instead (Full = substepsPerStep, Simplified =
    // half, Procedural = quarter, Animation = one); the interval table says
    // how many fixed steps each band updates on (0 freezes that band, which
    // is the Cached default).
    engine::util::SimulationLodSettings lod{};
};

} // namespace engine::physics_fluid
