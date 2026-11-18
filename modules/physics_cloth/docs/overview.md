# physics_cloth module

physics_cloth provides cloth simulation: XPBD particle grids held together by structural, shear, and bending distance constraints, stepped with fixed-timestep substepping, colliding against a physics_rigid world's bodies and planes. Cloth drapes over static scenery, wraps around moving rigid bodies, and reacts to wind, pinning, and mouse-style dragging, while the rigid capability's sphere-overlap query supplies every contact.

## Capability

The module provides the `physics_cloth` capability (ABI version constant `physicsClothCapabilityAbi = 1`, see `include/engine/physics_cloth/capabilities.hpp`) and requires the `physics_rigid` capability; the kernel constructs the rigid core first and grants the permission from the manifest's `requires` list. Cloth handles are `std::uint32_t`, start at 1, are never reused, and `0` (`invalidCloth`) means "none". Positions and velocities are world-space meters and meters per second packed `x, y, z` per particle in row-major grid order.

### Frame-rate independence contract

Like the rigid module, cloth advances in whole fixed steps only. `step(cloth, fixedDeltaSeconds)` performs exactly one step (values in (0, 0.1] are accepted) and must be called with a constant delta; `simulate(cloth, elapsedSeconds)` accumulates wall time and runs whole fixed steps at the configured `fixed_rate_hz`, dropping backlog beyond `maximum_steps_per_simulate`. Each step internally runs `substeps_per_step` XPBD substeps — the small-steps schedule that keeps constraint stiffness iteration-independent — so the result depends only on the step count.

| Group | Functions | Purpose |
| --- | --- | --- |
| Lifetime | `createCloth(world, description, outError)`, `destroyCloth(cloth)` | Builds the particle grid and the structural, shear, and bending constraint sets against the given rigid world; the cloth adopts that world's gravity. |
| Inspection | `clothInfo`, `copyParticlePositions`, `copyParticleVelocities`, `clothStatistics` | Grid dimensions, constraint count, packed particle state, and the cumulative step, substep, contact, and timing counters. |
| Pinning | `setParticlePinned`, `pinParticleToPosition`, `isParticlePinned` | Pins a particle to its creation position or an explicit target; pinned particles carry no mass and follow their pin exactly. |
| Dragging | `beginParticleDrag`, `setParticleDragTarget`, `endParticleDrag` | A soft spring (stiffness, damping, acceleration clamp) pulls one particle toward a target each substep — the mouse-grab entry point. |
| Wind | `setWindAcceleration` | Uniform wind acceleration (meters per second squared) added to every particle each substep. |
| Stepping | `step(cloth, fixedDeltaSeconds)`, `simulate(cloth, elapsedSeconds)` | The fixed-step primitive and the accumulator convenience; see the contract above. |

**Error convention:** `createCloth` fills the `ClothError` out-parameter (`InvalidDescription` for degenerate grids or non-finite values, `CapacityExceeded` beyond the cloth or particle budgets, `SimulationFailure` when the module could not acquire the rigid capability). `step` reports `SimulationFailure`-style failure through its `false` return when any particle enters a non-finite state; the step is aborted and the cloth state stays untouched.

## Simulation model

**Grid and constraints.** `createCloth` places a `columns x rows` particle grid in the description transform's local XZ plane and builds three constraint families: structural edges along both grid axes (stretch compliance, rigid by default), the two diagonals of every quad (shear compliance), and edges spanning two particles along each axis (bend compliance — the distance-based bending model that resists folding without dihedral calculations). Rest lengths come from the created grid; compliances are the XPBD `alpha` values from the description.

**XPBD stepping.** Each substep: velocities integrate gravity, wind, damping, and the drag spring; positions advance (previous positions recorded); distance constraints solve once in construction order with the compliance-scaled correction `deltaLambda = (-length + restLength) / (inverseMassSum + alpha / dt^2)`; particles then resolve against the rigid world; and finally velocities are recomputed from the position change. Because each substep uses `fixedDeltaSeconds / substeps_per_step`, doubling the substep count halves the per-substep displacement while the XPBD compliance scaling keeps stiffness constant — the time-step independence the compliance term exists for.

**Collision.** Per substep, every free particle queries the rigid world through `collideSphere` with the particle radius (the description's, or `spacing x 0.25` when unset) and is projected out along the deepest overlap normal. The final velocity pass splits the particle's velocity into normal and tangential components at contacted particles: the approach velocity is removed and the tangential part scales by the friction factor (the sliding velocity kept per substep). Cloth therefore slides realistically over ramps and gets carried by moving rigid bodies without any cloth-specific solver state inside the rigid module.

**Pinning and dragging.** Pinned particles skip integration and constraints entirely and sit exactly at their pin positions; unpinning restores their mass. The drag spring applies a clamped acceleration toward the target (`stiffness x error - damping x velocity`), so a grabbed particle trails the pointer with tunable elasticity, and frame-rate independence holds because the spring runs per fixed substep.

## Provided / required / optional capabilities

From `module.toml` and the entry table at the bottom of `source/physics_cloth.cpp` (they agree):

- **Provided:** `physics_cloth`.
- **Required:** `physics_rigid`.
- **Optional:** none.
- **Declared permissions:** none.

## Architecture

| File | Purpose |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capability, required capability, declared `config_keys`. |
| `config.toml` | Default values for the `[physics_cloth]` configuration section (see `configuration.md`). |
| `CMakeLists.txt` | Builds `engine_module_physics_cloth` (a `.so` under `runtime/modules/physics_cloth/`) and copies `module.toml` next to it. |
| `include/engine/physics_cloth/capabilities.hpp` | Public capability types: ABI constant, cloth handles, description, drag configuration, info and statistics, capability struct. |
| `source/records.hpp` | Cloth record (particles, constraints, contact scratch) and module settings. |
| `source/physics_cloth.cpp` | Module class (lifecycle, configuration, grid construction, XPBD stepping, collision resolution, capability binding) and entry table. |
| `include/engine/physics_rigid/math.hpp` | The rigid module's public math vocabulary used for grid placement. |

## How it works

**Creation.** `createCloth` validates the grid (at least 2x2 particles), the spacing, mass, compliances, damping, and friction ranges, reads the target world's gravity through `copyWorldGravity`, builds the particle arrays, and generates the constraint list: two structural passes (row and column neighbors), four shear diagonals per quad as two constraints each, and the two-apart bend neighbors along both axes. Pinned indices are validated against the particle count and flagged before the handle is published.

**Stepping.** `stepCloth` runs the substep loop described above and aborts (returning false without mutating state beyond the substep in progress) when any particle position becomes non-finite. The statistics counters accumulate steps, substeps, the contact count of the previous step, and the previous step's wall-clock cost. `simulate` mirrors the rigid module's accumulator: whole steps at the configured rate, budget clamp, and zero-budget reset when the step cap trips so a stalled consumer never accumulates debt.

**Interaction with rigid bodies.** Cloth reads nothing from the rigid module beyond the world gravity and the `collideSphere` query, so any body kind works the same: static ground planes catch falling cloth, dynamic boxes pushed through a sheet deform it, and kinematic carriers drag it along. Rigid bodies never feel cloth back-reaction — cloth is one-way coupled by design (see configuration.md for the cost trade-off this represents).

## Threading and lifetime

The module has no locks; serialize calls from one thread. Particle arrays live in the cloth record and die with it: `destroyCloth` releases the arrays and returns the particle budget; `stop()` clears every cloth and resets the handle counters. Handles are monotonic and never reused. The `collideSphere` queries are read-only against the rigid world, so cloth steps may run concurrently with rigid `step` calls of a different world only if the consumer guarantees the worlds are distinct; in the common single-world case the consumer steps the rigid world and every cloth sequentially within one fixed update.

A consumer creates a hanging sheet pinned along its top row and grabs a particle:

```cpp
engine::physics_cloth::ClothCapability &clothCapability =
    context.acquireCapability<engine::physics_cloth::ClothCapability>();
if (clothCapability.abi != engine::physics_cloth::physicsClothCapabilityAbi)
{
    return;
}
std::uint32_t pins[4] = {0, 7, 56, 63}; // top row of an 8x8 sheet
engine::physics_cloth::ClothDescription description{};
description.columns = 8;
description.rows = 8;
description.pinnedParticles = pins;
description.pinnedCount = 4;
const engine::physics_cloth::ClothHandle cloth =
    clothCapability.createCloth(clothCapability.self, rigidWorld, &description, nullptr);

engine::physics_cloth::ClothParticleDrag drag{};
drag.particleIndex = 40;
drag.stiffness = 300.0;
clothCapability.beginParticleDrag(clothCapability.self, cloth, &drag);
clothCapability.setParticleDragTarget(clothCapability.self, cloth, targetPoint);
// per fixed step:
clothCapability.step(clothCapability.self, cloth, fixedDeltaSeconds);
```
