// records.hpp - the physics_fields module's plain-data runtime records:
// wind fields, force volumes, and the shared FieldsSettings.

#pragma once

#include <engine/physics_fields/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/util/simulation_lod.hpp>

#include <cstdint>
#include <vector>

namespace engine::physics_fields
{
struct WindFieldRecord
{
    WindHandle              handle = 0;
    engine::physics_rigid::WorldHandle world = 0;
    WindFieldDescription    description{};
    double                  direction[3] = {1.0, 0.0, 0.0}; // normalized

    // Simulation LOD: importance-driven fidelity band and staggered cadence
    // (the phase derives from the registration ordinal). Due passes scale
    // the applied force by the interval so the total impulse per wall time
    // stays constant.
    double                           importance = 1.0;
    engine::util::SimulationFidelity lodBand    = engine::util::SimulationFidelity::Full;
    engine::util::SimulationCadence  cadence{};
    std::uint64_t                    lodSkippedSteps = 0;
};

} // namespace engine::physics_fields
