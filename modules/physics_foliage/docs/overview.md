# physics_foliage module

physics_foliage provides procedural foliage deformation: grass blades, bushes, and tree crowns are instances that bend under wind through a per-instance spring-damper (a wind-derived bend target plus a phase-offset sway cycle), get pushed away by radial disturbances such as footsteps, passing vehicles, or blasts and spring back, and carry per-instance stiffness so young shoots and thick trunks react differently. Wind is sampled from the physics_fields capability when a field binds a wind handle and from the configured ambient breeze otherwise; every result advances in whole fixed steps, so results depend on the step count only.

## Capability

The module provides the `physics_foliage` capability (ABI version constant `physicsFoliageCapabilityAbi = 2`, see `include/engine/physics_foliage/capabilities.hpp`; the simulation-LOD surface — `setFieldImportance` and the `lodSkippedSteps` counter — arrived in ABI 2, and consumers check `abi >= 2` before touching those additions) and declares no required capabilities (the bend springs integrate their own state, so no kernel ordering constraint applies); `physics_fields` is declared optional and acquired with `tryAcquireCapability` in `construct()`, and a pointer whose `abi` does not equal `physicsFieldsCapabilityAbi = 1` is dropped so the module falls back to its ambient breeze. Field handles are `std::uint32_t` (`FieldHandle`), start at 1, are never reused, and `0` (`invalidField`) means "none". Positions are world-space meters and wind velocities are meters per second; each instance's internal bend state is a horizontal tip displacement in meters (packed `x, z`), and the published `FoliageBend` carries a unit horizontal direction (`bendDirectionX`, `bendDirectionZ`) plus `bendAngleRadians` — consumers compose the instance's model matrix as `translate(position) * rotate(axis, angle)` with `axis` the horizontal axis perpendicular to the bend direction.

### Fixed-step contract

`step(fixedDeltaSeconds)` advances every field one step and accepts deltas in (0, 0.1] that must stay constant across a running simulation; `simulate(elapsedSeconds)` accumulates wall time and runs whole fixed steps at the configured `fixed_rate_hz`, returning the number of steps executed, dropping backlog beyond `maximum_steps_per_simulate`, and resetting the accumulator when the step cap trips so a stalled consumer never accumulates debt. Each step integrates every instance's spring once (semi-implicit Euler), so results depend only on the step count.

| Group | Functions | Purpose |
| --- | --- | --- |
| Lifetime | `createField(self, description, windFieldHandle, outError)`, `destroyField(self, field)` | Validates and copies the instance array into a field record; `windFieldHandle` binds a physics_fields wind handle for wind sampling while `0` selects the configured ambient breeze. |
| Results | `copyBends(self, field, outBends, capacity, outFilled, outRequired)` | Fills up to `capacity` `FoliageBend` records in instance order; `outRequired` receives the instance count and `outFilled` the number written. |
| Disturbance | `applyRadialDisturbance(self, field, position, radius, strength)` | Injects bend velocity into instances within `radius` of the point (a footstep, a passing vehicle, a blast) with a linear falloff scaled by per-instance flexibility. |
| Stepping | `step(self, fixedDeltaSeconds)`, `simulate(self, elapsedSeconds)` | The fixed-step primitive and the accumulator convenience; see the contract above. |
| Statistics | `foliageStatistics(self, outStatistics)` | Field count, module-wide instance count, and the cumulative disturbance counter. |

**Error convention:** `createField` fills the `FoliageError` out-parameter (`InvalidDescription` for a null description, missing instance data when `instanceCount > 0`, or non-finite and out-of-range values — `swayFrequency` and `bendScale` must be ≥ 0, `springStiffness` > 0, `springDamping` ≥ 0, and instances need positive heights and stiffness in [0, 1]; `CapacityExceeded` beyond `maximum_fields`, `maximum_instances_per_field`, or the shared `maximum_instances` budget, or once the handle counter reaches the `std::uint32_t` limit). The remaining calls report failure through their `bool` return — unknown handles, null output buffers, or invalid disturbance arguments return `false` — and `step` returns `false` when any instance bend state becomes non-finite, the case the `SimulationFailure` value documents in the error enum.

## Simulation model

**Wind sampling.** Each step samples one wind velocity per instance at its base position: when the fields capability was acquired and the field was created with a non-zero `windFieldHandle`, the sample comes from `sampleWindVelocity` at the instance's base position with the field's own sway clock, so the fields module's gust cycle and spatial turbulence modulate the foliage; otherwise — no fields capability, `windFieldHandle == 0`, or a wind handle the fields module no longer knows — the sample is the configured ambient breeze: the `ambient_wind_direction` vector normalized to horizontal `x, z` and scaled by `ambient_wind_speed` (normalized once at configuration read, not per instance per step), or zero wind when the configured direction is degenerate.

**Bend springs.** Every instance carries a 2D horizontal bend vector with a velocity, integrated by a spring-damper (semi-implicit Euler at the fixed step). The wind target's magnitude is `windSpeed * (1 - stiffness) * bendScale * height²` — stiffer instances bend less and taller instances lever further — and a phase-offset sway adds `sin(phaseSeed + swayClock * swayFrequency * 2π) * 0.15 * windSpeed * (1 - stiffness) * bendScale * swayScale` along the wind direction, producing traveling wave patterns across the field instead of unison motion; the sway clock doubles as the time input passed to physics_fields sampling, keeping both wind sources on one time base. The spring accelerations are `(target - bend) * springStiffness - bendVelocity * springDamping`, and the bend length is clamped to half the instance height after integration: foliage bends, never folds.

**Disturbances (trampling response).** `applyRadialDisturbance` measures the horizontal distance from the disturbance point to every instance's base; instances within `radius` receive bend velocity along the away-from-center direction scaled by `strength * (1 - distance/radius)` and by the instance flexibility `1 - stiffness`, so stiff trunks barely move while young shoots whip away, and the regular spring-damper then springs them back over subsequent steps. Disturbances inject velocity rather than position, so the response composes with whatever wind is currently blowing.

## Provided / required / optional capabilities

From `module.toml` and the entry table at the bottom of `source/physics_foliage.cpp` (they agree):

- **Provided:** `physics_foliage`.
- **Required:** none.
- **Optional:** `physics_fields` — acquired with `tryAcquireCapability` in `construct()` and kept only when its `abi` matches `physicsFieldsCapabilityAbi = 1`; when present, fields created with a wind handle sample gusts and spatial turbulence through `sampleWindVelocity`, and when absent every field falls back to the configured ambient breeze (a constant horizontal direction and speed), so a scene without the fields module still gets wind-driven motion.
- **Declared permissions:** none.

## Architecture

| File | Purpose |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capability, required and optional capabilities, declared `config_keys`. |
| `config.toml` | Default values for the `[physics_foliage]` configuration section (see `configuration.md`). |
| `CMakeLists.txt` | Builds `engine_module_physics_foliage` (a `.so` under `runtime/modules/physics_foliage/`) and copies `module.toml` next to it. |
| `include/engine/physics_foliage/capabilities.hpp` | Public capability types: ABI constant, field handles, instance and field descriptions, bend results, statistics, capability struct. |
| `source/records.hpp` | Field and instance records (bend state, cached wind) and module settings. |
| `source/physics_foliage.cpp` | Module class (lifecycle, configuration, wind sampling, bend-spring integration, disturbance application, capability binding) and entry table. |
| `include/engine/physics_fields/capabilities.hpp` | The optional wind capability's public surface; `sampleWindVelocity` is the one entry point the stepping path calls when a wind handle is bound. |

## How it works

**Creation.** `createField` checks the description and instance pointers, the budgets (field count, per-field instances, module-wide instances), the field-level ranges, and every instance's values before publishing a handle; instances are copied into per-instance state records with zeroed bend vectors, so callers may free their arrays immediately. Destroying a field releases its instances from the module-wide budget and refreshes the statistics counters.

**Stepping.** `step` walks every field: one wind sample per instance, the target and sway computation, one semi-implicit Euler integration of the bend springs, and the half-height clamp, returning `false` without further steps when any bend state becomes non-finite; `simulate` runs the accumulator described above; `foliageStatistics` reports the live counts plus the cumulative disturbance count.

**Reading results.** `copyBends` converts each bend vector into the publish form: the unit horizontal direction (zeroed when the bend is shorter than a numerical epsilon) and `atan2(bendLength, height)` as the bend angle, which is exactly the rotation consumers feed into their per-instance model matrices.

**No body simulation.** The module creates no rigid bodies and applies no forces: instances are lightweight per-instance state rather than simulated bodies, and the only cross-module call in the stepping path is the optional `sampleWindVelocity`, so foliage cost scales with instance count alone.

## Simulation LOD and amortized wind sampling

Every field carries an importance score (`setFieldImportance`, ABI 2) resolved through the five-band fidelity ladder with demotion hysteresis. With `lod_enabled` on, fields below Full update on a staggered cadence phase-spread by registration order, and one due step covers the whole interval of wall time — the bend springs integrate with interval x the caller's delta, and the spring gains clamp to the semi-implicit-Euler stability bound of that enlarged step (`h x damping < 2`, `h^2 x stiffness < 4`), so an aggressive interval trades sway speed for stability instead of diverging. Cached fields freeze until the score rises again, and `lodSkippedSteps` counts the skipped field-steps. Independent of the cadence, the per-instance wind samples amortize over `wind_sample_stride`: each instance refreshes its cached sample when its stride slot comes due (phase-spread by instance index), so a two-million-instance field calls the wind capability a fraction of the time while the bend target stays a bounded number of steps stale. An instance whose bend state turns non-finite resets upright and the step reports failure.

## Threading and lifetime

The module has no locks; calls are main-thread only and must be serialized from one thread. Field records and their per-instance arrays live in the module and die with it: `destroyField` releases the arrays and returns the instance budget, and `stop()` clears every field, resets the handle counter, and zeroes the statistics. Nothing is shared with the fields module — wind samples are copied values, so the two modules never alias memory.

A consumer plants a small grass patch, drives it from the ambient breeze, tramples it, and reads the bends:

```cpp
engine::physics_foliage::FoliageCapability &foliageCapability =
    context.acquireCapability<engine::physics_foliage::FoliageCapability>();
if (foliageCapability.abi != engine::physics_foliage::physicsFoliageCapabilityAbi)
{
    return;
}
engine::physics_foliage::FoliageInstance blades[2]{};
blades[0].position[0] = -0.2;
blades[0].height      = 0.35;
blades[0].stiffness   = 0.2;
blades[0].phaseSeed   = 0.7;
blades[1].position[0] = 0.4;
blades[1].height      = 0.3;
blades[1].stiffness   = 0.25;
blades[1].phaseSeed   = 2.1;
engine::physics_foliage::FoliageFieldDescription description{};
description.instances     = blades;
description.instanceCount = 2;
const engine::physics_foliage::FieldHandle field = foliageCapability.createField(
    foliageCapability.self, &description, 0 /* 0 = ambient breeze */, nullptr);

const double footPoint[3] = {-0.1, 0.0, 0.2};
foliageCapability.applyRadialDisturbance(foliageCapability.self, field, footPoint, 0.6, 2.5);
// per fixed step:
foliageCapability.step(foliageCapability.self, fixedDeltaSeconds);
engine::physics_foliage::FoliageBend bends[2];
std::uint32_t filled   = 0;
std::uint32_t required = 0;
foliageCapability.copyBends(foliageCapability.self, field, bends, 2, &filled, &required);
```
