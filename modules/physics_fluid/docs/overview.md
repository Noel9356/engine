# physics_fluid module

physics_fluid provides smoothed-particle hydrodynamics (SPH): a particle fluid using the classic Muller 2003 force set (poly6 density estimation, spiky-gradient pressure, Laplacian viscosity) over a spatial hash, stepped with fixed-timestep substeps, colliding against a physics_rigid world's bodies and planes through that module's sphere-overlap query and optionally confined to an axis-aligned bounds box for tanks and containers.

## Capability

The module provides the `physics_fluid` capability (ABI version constant `physicsFluidCapabilityAbi = 1`, see `include/engine/physics_fluid/capabilities.hpp`) and requires the `physics_rigid` capability; the kernel constructs the rigid core first and grants the permission from the manifest's `requires` list. Fluid handles are `std::uint32_t`, start at 1, are never reused, and `0` (`invalidFluid`) means "none". A fluid is created empty and populated through `addParticles`; positions and velocities are world-space meters and meters per second packed `x, y, z` per particle in insertion order, masses are kilograms, and densities kilograms per cubic meter. The kernel radius `interactionRadius` is the SPH smoothing length `h` — neighbors within `h` exchange pressure and viscosity, roughly twice the particle spacing.

### Frame-rate independence contract

Like the rigid module, a fluid advances in whole fixed steps only. `step(fluid, fixedDeltaSeconds)` performs exactly one step (values in (0, 0.1] are accepted) and must be called with a constant delta; `simulate(fluid, elapsedSeconds)` accumulates wall time and runs whole fixed steps at the configured `fixed_rate_hz`, dropping backlog beyond `maximum_steps_per_simulate`. Each step internally runs `substeps_per_step` SPH substeps — hash rebuild, density, forces, integration, collision — so the result depends only on the step count.

| Group | Functions | Purpose |
| --- | --- | --- |
| Lifetime | `createFluid(world, description, outError)`, `destroyFluid(fluid)` | Validates the description, adopts the target world's gravity, and registers an empty fluid; destruction releases its particles back to the module budget. |
| Population | `addParticles(fluid, positions, velocities, count, outError)` | Appends particles (packed x, y, z; a null velocities pointer starts them at rest) and returns the first appended particle index. |
| Inspection | `fluidInfo`, `fluidStatistics` | Particle count and kernel radius, plus the cumulative step, substep, contact, average-density, and timing counters. |
| State copies | `copyParticlePositions`, `copyParticleVelocities` | Packed particle state in insertion order; `outRequired` receives the needed buffer length in doubles (three per particle). |
| Stepping | `step(fluid, fixedDeltaSeconds)`, `simulate(fluid, elapsedSeconds)` | The fixed-step primitive and the accumulator convenience; see the contract above. |

**Error convention:** `createFluid` and `addParticles` fill the `FluidError` out-parameter (`InvalidDescription` for a null description, an unavailable rigid capability, non-finite or non-positive radii, mass, or rest density, negative stiffness, viscosity, or damping, inverted bounds, or non-finite particle input — `addParticles` then returns the particles appended so far; `CapacityExceeded` beyond the fluid or particle budgets; `InvalidHandle` for unknown fluid handles or a null position array with a nonzero count). `step` reports failure through its `false` return when the delta is outside (0, 0.1] or any particle position becomes non-finite.

## Simulation model

**Spatial hash and density.** Every substep rebuilds the spatial hash with cell size equal to the interaction radius: cell coordinates hash into 64-bit bucket ids (bounds-free, so unbounded fluids work), a stable counting sort by cell id keeps neighbor order deterministic, and neighbor queries scan the 3 x 3 x 3 cell block around each particle. Density is estimated with the poly6 kernel: each particle sums its own mass over the kernel at zero distance plus every neighbor's mass within `h`. Pressure follows the equation of state `p = gasStiffness x (density - restDensity)` clamped at zero, so compressed regions push apart and the fluid never pulls.

**Forces.** The classic Muller 2003 pair forces run over the same neighbor lists: pressure applies the symmetric form `-m (p_i + p_j) / (2 rho_j) grad W` with the spiky kernel gradient, viscosity applies `m mu (v_j - v_i) / rho_j lap W` with the Laplacian kernel (scaled by the particle's own inverse density), and the world gravity captured at creation is added to every particle. Densities floor at a tiny epsilon in the force denominators so sparse neighborhoods stay well-behaved.

**Integration.** Velocities integrate the accumulated acceleration by the substep delta, then scale by the damping factor `1 / (1 + dt x damping)`, then clamp to `maximum_velocity` (the module configuration's stability bound); positions advance by the clamped velocity. Because each substep uses `fixedDeltaSeconds / substeps_per_step`, the substep count is the integration-resolution dial and the result stays independent of the call cadence.

**Collision and bounds.** After integration each particle resolves against its surroundings: when `useBounds` is set, each axis clamps to the bounds box inset by `particleRadius` and the offending velocity component reverses at 30% strength (`x -0.3`); then every particle queries the rigid world through `collideSphere` with `particleRadius` and is projected out along the push-out normal by the overlap depth, removing only the approaching velocity component (an inelastic contact that preserves sliding). The per-step contact count and the average density land in the statistics.

## Provided / required / optional capabilities

From `module.toml` and the entry table at the bottom of `source/physics_fluid.cpp` (they agree):

- **Provided:** `physics_fluid`.
- **Required:** `physics_rigid`.
- **Optional:** none.
- **Declared permissions:** none.

## Architecture

| File | Purpose |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capability, required capability, declared `config_keys`. |
| `config.toml` | Default values for the `[physics_fluid]` configuration section (see `configuration.md`). |
| `CMakeLists.txt` | Builds `engine_module_physics_fluid` (a `.so` under `runtime/modules/physics_fluid/`) and copies `module.toml` next to it. |
| `include/engine/physics_fluid/capabilities.hpp` | Public capability types: ABI constant, fluid handle, description, info and statistics, capability struct. |
| `source/records.hpp` | Fluid record (particle arrays, per-particle density and pressure, spatial-hash scratch) and module settings. |
| `source/physics_fluid.cpp` | Module class (lifecycle, configuration, SPH kernels, spatial hash, substep pipeline, collision and bounds resolution, capability binding) and entry table. |
| `include/engine/physics_rigid/math.hpp` | The rigid module's public math vocabulary used in the force accumulation. |

## How it works

**Creation.** `createFluid` validates finiteness and positivity (`interactionRadius`, `particleRadius`, `particleMass`, and `restDensity` above zero; `gasStiffness`, `viscosity`, and `damping` at or above zero; bounds finite with `boundsMinimum` below `boundsMaximum` on every axis when `useBounds` is set), checks the fluid and particle budgets, reads the target world's gravity through `copyWorldGravity` (falling back to (0, -9.81, 0) if the world does not answer), and publishes the handle. Particles are appended with `addParticles`, which validates each packed position and optional velocity for finiteness and on success resizes the density, pressure, and acceleration arrays (densities start at rest density).

**Stepping.** `stepFluid` runs the substep loop described above and reports failure (returning false) when any particle position becomes non-finite; the failing step does not advance the statistics counters. The counters accumulate steps, substeps, the contact count of the previous step, the previous step's average density, and the previous step's wall-clock cost. `simulate` mirrors the rigid module's accumulator: whole steps at the configured rate, budget clamp, and zero-budget reset when the step cap trips so a stalled consumer never accumulates debt.

**Interaction with rigid bodies.** Fluid reads nothing from the rigid module beyond the world gravity and the `collideSphere` query, so any body kind works the same: static ground planes catch falling fluid, dynamic boxes pushed through a pool displace it, and kinematic carriers slosh it around. Rigid bodies never feel fluid back-reaction — fluid is one-way coupled by design (see configuration.md for the cost trade-off this represents).

## Threading and lifetime

The module has no locks; serialize calls from one thread. Particle arrays live in the fluid record and die with it: `destroyFluid` releases the arrays and returns the particle budget; `stop()` clears every fluid and resets the handle counters. Handles are monotonic and never reused. The `collideSphere` queries are read-only against the rigid world, so fluid steps may run concurrently with rigid `step` calls of a different world only if the consumer guarantees the worlds are distinct; in the common single-world case the consumer steps the rigid world and every fluid sequentially within one fixed update.

A consumer creates a bounded fluid, fills it with two particles, and steps it:

```cpp
engine::physics_fluid::FluidCapability &fluidCapability =
    context.acquireCapability<engine::physics_fluid::FluidCapability>();
if (fluidCapability.abi != engine::physics_fluid::physicsFluidCapabilityAbi)
{
    return;
}
engine::physics_fluid::FluidDescription description{};
description.interactionRadius = 0.35;
description.useBounds = true; // tank walls at the description's bounds box
const engine::physics_fluid::FluidHandle fluid =
    fluidCapability.createFluid(fluidCapability.self, rigidWorld, &description, nullptr);

double positions[6] = {0.0, 1.0, 0.0, 0.2, 1.4, 0.1}; // packed x,y,z per particle
fluidCapability.addParticles(fluidCapability.self, fluid, positions, nullptr, 2, nullptr);
// per fixed step:
fluidCapability.step(fluidCapability.self, fluid, fixedDeltaSeconds);
```
