// records.hpp - the physics_fluid module's plain-data runtime records: SPH
// fluids (particles, per-particle density/pressure, spatial hash scratch)
// and the shared FluidSettings.

#pragma once

#include <engine/physics_fluid/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>

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
    std::vector<std::uint64_t> hashCellIds_;
    std::vector<std::uint32_t> hashCellCounts_;
    double hashCellSize = 0.0;
    double hashOrigin[3] = {0.0, 0.0, 0.0};

    double gravity[3] = {0.0, -9.81, 0.0}; // read from the rigid world at creation

    std::uint64_t stepsSimulated   = 0;
    std::uint64_t substepsExecuted = 0;
    std::uint32_t lastContactCount = 0;
    double        lastAverageDensity = 0.0;
    double        lastStepMilliseconds = 0.0;

    double simulatedTimeBudget = 0.0;
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
};

} // namespace engine::physics_fluid
