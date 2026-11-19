# physics_cloth module

physics_cloth provides cloth simulation: XPBD particle grids held together by structural, shear, and bending distance constraints, stepped with fixed-timestep substepping, colliding against a physics_rigid world's bodies and planes, tearing constraints that stretch past a strain tolerance, repelling overlapping particle pairs through a deterministic self-collision pass, and sampling the deformed surface through bilinear grid interpolation for rendering. Cloth drapes over static scenery, wraps around moving rigid bodies, and reacts to wind (a uniform acceleration, or a per-face aerodynamic drag-and-lift response for sheets flagged aerodynamic), pinning, and mouse-style dragging, while the rigid capability's sphere-overlap query supplies every contact.

## Capability

The module provides the `physics_cloth` capability (ABI version constant `physicsClothCapabilityAbi = 4`; the header's ABI history comment records what each bump appended, tearing, self-collision, and surface sampling arrived in ABI 2, the simulation-LOD and sleep surfaces — `setClothImportance`, the `lodSkippedSteps` and `sleeping` statistics, and the implemented `sleepingEnabled` behavior — arrived in ABI 3, and the per-cloth aerodynamic-wind flag `ClothDescription.aerodynamicWind` arrived in ABI 4; consumers check `abi >= 4` before touching that addition, while everything from ABI 3 keeps its slot and meaning) and requires the `physics_rigid` capability; the kernel constructs the rigid core first and grants the permission from the manifest's `requires` list. Cloth handles are `std::uint32_t`, start at 1, are never reused, and `0` (`invalidCloth`) means "none". Positions and velocities are world-space meters and meters per second packed `x, y, z` per particle in row-major grid order.

### Frame-rate independence contract

Like the rigid module, cloth advances in whole fixed steps only. `step(cloth, fixedDeltaSeconds)` performs exactly one step (values in (0, 0.1] are accepted) and must be called with a constant delta; `simulate(cloth, elapsedSeconds)` accumulates wall time and runs whole fixed steps at the configured `fixed_rate_hz`, dropping backlog beyond `maximum_steps_per_simulate`. Each step internally runs `substeps_per_step` XPBD substeps — the small-steps schedule that keeps constraint stiffness iteration-independent — so the result depends only on the step count.

| Group | Functions | Purpose |
| --- | --- | --- |
| Lifetime | `createCloth(world, description, outError)`, `destroyCloth(cloth)` | Builds the particle grid and the structural, shear, and bending constraint sets against the given rigid world; the cloth adopts that world's gravity. |
| Inspection | `clothInfo`, `copyParticlePositions`, `copyParticleVelocities`, `clothStatistics` | Grid dimensions, constraint count, packed particle state, and the cumulative step, substep, contact, timing, and torn-constraint counters. |
| Pinning | `setParticlePinned`, `pinParticleToPosition`, `isParticlePinned` | Pins a particle to its creation position or an explicit target; pinned particles carry no mass and follow their pin exactly. |
| Dragging | `beginParticleDrag`, `setParticleDragTarget`, `endParticleDrag` | A soft spring (stiffness, damping, acceleration clamp) pulls one particle toward a target each substep — the mouse-grab entry point. |
| Wind | `setWindAcceleration` | Uniform wind acceleration (meters per second squared) added to every particle each substep. |
| Sampling | `sampleClothSurface(cloth, u, v, outPosition, outNormal)` | Bilinear interpolation of the deformed particle grid at normalized surface coordinates plus the tangent cross-product normal; a read-only render-mesh query. |
| Stepping | `step(cloth, fixedDeltaSeconds)`, `simulate(cloth, elapsedSeconds)` | The fixed-step primitive and the accumulator convenience; see the contract above. |

**Error convention:** `createCloth` fills the `ClothError` out-parameter (`InvalidDescription` for degenerate grids or non-finite values, `CapacityExceeded` beyond the cloth or particle budgets, `SimulationFailure` when the module could not acquire the rigid capability). `step` reports `SimulationFailure`-style failure through its `false` return when any particle enters a non-finite state; the step is aborted and the cloth state stays untouched. Tearing is a normal outcome, never a failure: a constraint that breaks is skipped silently and reported only through `ClothStatistics.brokenConstraintCount`.

## Simulation model

**Grid and constraints.** `createCloth` places a `columns x rows` particle grid in the description transform's local XZ plane and builds three constraint families: structural edges along both grid axes (stretch compliance, rigid by default), the two diagonals of every quad (shear compliance), and edges spanning two particles along each axis (bend compliance — the distance-based bending model that resists folding without dihedral calculations). Rest lengths come from the created grid; compliances are the XPBD `alpha` values from the description.

**XPBD stepping.** Each substep: velocities integrate gravity, wind, damping, and the drag spring; positions advance (previous positions recorded); distance constraints solve in construction order with the compliance-scaled correction `deltaLambda = (-length + restLength) / (inverseMassSum + alpha / dt^2)`, each constraint first checking the tear tolerance when tearing is enabled; when self-collision is enabled, the overlapping-pair repulsion pass runs after the constraint iterations; particles then resolve against the rigid world; and finally velocities are recomputed from the position change. The constraint walk repeats `solver_iterations` times per substep with the XPBD multiplier accumulating across the passes (the accumulator resets at substep start, so one iteration per substep reduces to plain XPBD; tearing is judged on the first pass only, so the breakage criterion stays the stretch at the substep's start); the default of 1 is the small-steps schedule, where stiffness comes from the substep count. Because each substep uses `fixedDeltaSeconds / substeps_per_step`, doubling the substep count halves the per-substep displacement while the XPBD compliance scaling keeps stiffness constant — the time-step independence the compliance term exists for.

**Wind.** Two wind models share the `setWindAcceleration` vector per cloth. The default treats it as a uniform acceleration (meters per second squared) added to every free particle each substep. A cloth created with `ClothDescription.aerodynamicWind` treats it instead as the far-field wind velocity (meters per second): every grid face decomposes into its two triangles, and each triangle decomposes its relative velocity (wind minus the face's average particle velocity) into a normal-drag share scaled by `wind_drag_coefficient` and two lift shares scaled by `wind_lift_coefficient` (the Baraff-Witkin per-triangle decomposition). The triangle's response — its force along the current normal, scaled by the triangle area so the material response stays invariant under resolution changes — splits evenly over its free particles, and the sheet catches the wind, billows, and flutters instead of sliding uniformly. Face normals come from the current positions, so a crumpled face pushes along its crumpled normal; degenerate (zero-area or non-finite) faces are skipped, faces and their triangles iterate in grid order, and pinned particles receive no wind velocity, which keeps the model deterministic.

**Collision.** Per substep, every free particle takes part in one batched `collideSpheres` query against the rigid world with the particle radius (the description's, or `spacing x 0.25` when unset) and is projected out along the deepest overlap normal. The final velocity pass splits the particle's velocity into normal and tangential components at contacted particles: the approach velocity is removed and the tangential part scales by the friction retention — `friction^substepDelta`, the per-second "sliding velocity kept" factor the capability documents, so the same wall-clock contact time removes the same tangential speed at any step rate. Cloth therefore slides realistically over ramps and gets carried by moving rigid bodies without any cloth-specific solver state inside the rigid module.

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
| `include/engine/physics_cloth/capabilities.hpp` | Public capability types: ABI constant and history, cloth handles, description (including tearing and self-collision), drag configuration, info and statistics, capability struct with the surface sampler. |
| `source/records.hpp` | Cloth record (particles, constraints and tear state, contact and self-collision scratch) and module settings. |
| `source/physics_cloth.cpp` | Module class (lifecycle, configuration, grid construction, XPBD stepping, tearing, self-collision, collision resolution, surface sampling, capability binding) and entry table. |
| `include/engine/physics_rigid/math.hpp` | The rigid module's public math vocabulary used for grid placement. |

## How it works

**Creation.** `createCloth` validates the grid (at least 2x2 particles), the spacing, mass, compliances, damping, and friction ranges, reads the target world's gravity through `copyWorldGravity`, builds the particle arrays, and generates the constraint list: two structural passes (row and column neighbors), four shear diagonals per quad as two constraints each, and the two-apart bend neighbors along both axes. Pinned indices are validated against the particle count and flagged before the handle is published.

**Stepping.** `stepCloth` runs the substep loop described above and aborts (returning false without mutating state beyond the substep in progress) when any particle position becomes non-finite. The statistics counters accumulate steps, substeps, the contact count of the previous step, the previous step's wall-clock cost, and the constraints torn so far. `simulate` mirrors the rigid module's accumulator: whole steps at the configured rate, budget clamp, and zero-budget reset when the step cap trips so a stalled consumer never accumulates debt.

**Pinning and dragging.** Pinned particles skip integration and constraints entirely and sit exactly at their pin positions; unpinning restores their mass. The drag spring applies a clamped acceleration toward the target (`stiffness x error - damping x velocity`), so a grabbed particle trails the pointer with tunable elasticity, and frame-rate independence holds because the spring runs per fixed substep. The grab target lives in the cloth's own record, so two simultaneously dragged sheets follow their own targets instead of sharing one module-level point.

**Tearing.** When `ClothDescription.tearStrain` is positive, the constraint solve checks every distance constraint before solving it, every first solver pass: a constraint whose current length exceeds `restLength x (1 + tearStrain)` is marked broken and skipped from then on, while compressed constraints never tear. The check runs first in the fixed construction order — before the solvability skips — so breakage depends only on the stretch itself and nothing else, making it fully deterministic; structural, shear, and bend constraints tear alike, a torn constraint never triggers the non-finite failure path, and `ClothStatistics.brokenConstraintCount` reports the cumulative total per cloth.

**Self-collision.** When `ClothDescription.selfCollisionRadius` is positive, each substep runs one self-collision pass after the constraint iterations: a uniform spatial hash of the current particle positions (cell edge exactly `2 x radius`) is rebuilt into a flat sorted bucket structure — no map iteration anywhere — so each bucket holds ascending particle indices, and particles walk in index order while scanning the 3 x 3 x 3 neighbor-cell block in a fixed offset order, which keeps pair discovery deterministic. Every pair closer than `2 x radius` (handled once, with the higher index second) receives an XPBD-style positional correction split by inverse mass along the pair axis; pinned particles join the hash as infinite-mass obstacles that never move, and the velocity pass turns the corrections into contact velocities naturally.

**Surface sampling.** `sampleClothSurface` bilinearly interpolates the particle grid at normalized coordinates (`u` along columns, `v` along rows, clamped into [0, 1]) and returns the blended position plus a normal built from the cross product of the two grid-direction tangents — the central differences of the surrounding particles blended across the sampled cell, ordered so a flat grid faces +Y like the degenerate-case fallback. It returns false for unknown handles, non-finite coordinates, or non-finite results, so a renderer can light and skin the deformed cloth without copying particle arrays first.

**Interaction with rigid bodies.** Cloth reads nothing from the rigid module beyond the world gravity and the batched `collideSpheres` query, so any body kind works the same: static ground planes catch falling cloth, dynamic boxes pushed through a sheet deform it, and kinematic carriers drag it along. Rigid bodies never feel cloth back-reaction — cloth is one-way coupled by design (see configuration.md for the cost trade-off this represents).

## Settled cloth and simulation LOD

A cloth with `sleepingEnabled` (the description default) tracks the largest free-particle speed after every step: while every free particle stays below `sleep_velocity_threshold` the cloth accumulates quiet time, and after `time_to_sleep` seconds it sleeps — velocities zeroed, integration skipped — until a drag target, a pin change, or a wind acceleration wakes it. A settled cloth is never a blind spot for the scene's dynamics: every due step still resolves the same batched sphere-overlap query the live path uses (read-only, positions untouched), and any obstacle overlapping the resting surface wakes the cloth, so a moving rigid body or a newly spawned box deforms settled fabric instead of tunneling through it. The `sleeping` flag rides the statistics so consumers can show or skip settled cloth.

Simulation LOD gives every cloth an importance score (`setClothImportance`, ABI 3) that resolves through the five-band fidelity ladder with demotion hysteresis. Bands below Full reduce the internal substep count — the small-steps XPBD schedule keeps stiffness through the compliance scaling, so Simplified runs half the configured substeps per step, Procedural a quarter, and Animation one — while a due step at a reduced cadence integrates the accumulated fixed deltas in one pass, so the cloth keeps advancing at wall time; Cached freezes the cloth until the score rises again. With `lod_enabled` on, cloths also update on a staggered cadence (the phase comes from the registration ordinal, one due step every `lod_interval_*` steps); the statistics carry `lodSkippedSteps` so a consumer can watch the population settle.

World collision resolves through one batched sphere-overlap query per substep against the rigid capability's `collideSpheres` surface instead of one call per particle, and a step whose state would turn non-finite rolls back to its pre-step snapshot and reports `SimulationFailure`, leaving the cloth untouched.

## Threading and lifetime

The module has no locks; serialize calls from one thread. `sampleClothSurface` reads the particle arrays without mutating them but follows the same serialization rule. Particle arrays live in the cloth record and die with it: `destroyCloth` releases the arrays and returns the particle budget; `stop()` clears every cloth and resets the handle counters. Handles are monotonic and never reused. The batched `collideSpheres` queries are read-only against the rigid world, so cloth steps may run concurrently with rigid `step` calls of a different world only if the consumer guarantees the worlds are distinct; in the common single-world case the consumer steps the rigid world and every cloth sequentially within one fixed update.

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
