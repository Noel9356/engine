// sph_step.hpp - the SPH fixed-step pipeline for one fluid record: neighbor
// cache build, density estimation, force accumulation, integration, bounds
// and rigid collision resolution, and the whole-step driver with its
// non-finite rollback. The module class in physics_fluid.cpp binds these
// operations into the capability table.

#pragma once

#include "records.hpp"

#include <cstdint>

namespace engine::physics_fluid
{

// Runs one fixed step of `substeps` internal substeps over one fluid.
// Returns false and restores the pre-step state when the step would produce
// non-finite positions or velocities; statistics counters advance only for
// committed steps.
bool stepFluid(
    FluidRecord &fluid,
    const FluidSettings &settings,
    engine::physics_rigid::PhysicsRigidCapability *rigid,
    double fixedDeltaSeconds,
    std::uint32_t substeps);

} // namespace engine::physics_fluid
