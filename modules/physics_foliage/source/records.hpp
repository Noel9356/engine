// records.hpp - the physics_foliage module's plain-data runtime records: the
// per-instance bend state, one field record, and the shared FoliageSettings.

#pragma once

#include <engine/physics_foliage/capabilities.hpp>
#include <engine/physics_fields/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/util/simulation_lod.hpp>
#include <engine/util/step_accumulator.hpp>

#include <cstdint>
#include <vector>

namespace engine::physics_foliage
{

struct InstanceState
{
    FoliageInstance description{};

    // Bend state: horizontal displacement of the tip (x, z) with its
    // velocity, integrated by the spring-damper.
    double bend[2]         = {0.0, 0.0};
    double bendVelocity[2] = {0.0, 0.0};

    // Amortized wind sample: refreshed when the instance's sample slot
    // comes due (stride-spread by instance index); the cached velocity
    // drives the bend target between refreshes.
    double cachedWind[3] = {0.0, 0.0, 0.0};
    bool   windCached    = false;
};

struct FieldRecord
{
    FieldHandle handle = invalidField;

    std::uint32_t windFieldHandle = 0; // physics_fields wind handle, 0 = ambient breeze

    FoliageFieldDescription description{};

    std::vector<InstanceState> instances;

    double swayClock = 0.0;

    std::uint64_t disturbanceTotal = 0;

    // Simulation LOD: importance-driven fidelity band and staggered
    // cadence (the phase derives from the registration ordinal).
    double                           importance = 1.0;
    engine::util::SimulationFidelity lodBand    = engine::util::SimulationFidelity::Full;
    engine::util::SimulationCadence  cadence{};
    std::uint64_t                    lodSkippedSteps = 0;
};

struct FoliageSettings
{
    double        fixedRateHz              = 60.0;
    std::uint32_t maximumStepsPerSimulate  = 8;
    std::uint32_t maximumFields            = 64;
    std::uint32_t maximumInstances         = 2000000;
    std::uint32_t maximumInstancesPerField = 500000;

    // Sway amplitude factor (meters per second of target oscillation at
    // full wind speed and bend scale).
    double swayAmplitude = 0.15;

    // Wind sample stride: instance wind samples refresh every Nth step
    // (phase-spread by instance index); 1 samples every step.
    std::uint32_t windSampleStride = 1;

    // Simulation LOD: fields below the Full band update on a staggered
    // cadence; the per-band interval says how many fixed steps one due
    // step covers (0 freezes that band, which is the Cached default).
    engine::util::SimulationLodSettings lod{};
};

} // namespace engine::physics_foliage
