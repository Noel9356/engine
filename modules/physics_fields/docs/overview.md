# physics_fields module

physics_fields provides environmental and external force fields over a physics_rigid world: an analytic wind field (base direction and speed, gusts, spatial turbulence) sampled for effects and applied to bodies as quadratic aerodynamic drag, one-shot radial impulse blasts, and placed force volumes (uniform directional acceleration and radial magnetic-style attraction/repulsion), all driven through the rigid module's apply entry points so fields compose deterministically with any other force source and never integrate state themselves.

## Capability

The module provides the `physics_fields` capability (ABI version constant `physicsFieldsCapabilityAbi = 1`, see `include/engine/physics_fields/capabilities.hpp`) and requires the `physics_rigid` capability; the kernel constructs the rigid core first and grants the permission from the manifest's `requires` list. Wind and force-volume handles are `std::uint32_t`, start at 1, are never reused, and `0` (`invalidWind`, `invalidForceVolume`) means "none". Positions are world-space meters, wind velocities meters per second, accelerations meters per second squared, and impulses newton-seconds; vectors are plain `double` triples packed `x, y, z`. The module owns no simulation loop of its own: the consumer calls the apply entry points once per fixed update, immediately before stepping the rigid world, and every force lands through the rigid capability's `applyForceAtPoint` or `applyImpulseAtPoint`.

| Group | Functions | Purpose |
| --- | --- | --- |
| Wind fields | `createWindField(world, description, outError)`, `destroyWindField(wind)`, `sampleWindVelocity(wind, position, timeSeconds, outVelocity)` | Registers and releases an analytic wind field bound to a rigid world and evaluates its velocity (meters per second) at any world point and simulation time. |
| Wind drag | `applyWindDrag(wind, timeSeconds, fixedDeltaSeconds)` | Applies quadratic aerodynamic drag from the wind field to every awake dynamic body of the field's world; one call per fixed update before stepping the rigid world. |
| Force volumes | `createForceVolume(world, description, outError)`, `destroyForceVolume(volume)`, `setForceVolume(volume, description)`, `applyForceVolumes(fixedDeltaSeconds)` | Places, releases, and re-tunes (same validation as creation) uniform or radial volumes and applies every enabled volume of its world to the dynamic bodies currently inside the region. |
| Blasts | `applyRadialImpulse(world, position, impulseMagnitude, radius, upwardBias)` | One-shot radial impulse to every awake dynamic body of the given world within the radius. |
| Statistics | `fieldsStatistics(outStatistics)` | Live wind and volume counts plus the cumulative drag and volume application counters. |

**Error convention:** `createWindField` and `createForceVolume` fill the `FieldsError` out-parameter (`InvalidDescription` for null pointers — including an unavailable rigid capability — non-finite values, out-of-range strengths, a degenerate wind direction, or non-positive volume extents, `CapacityExceeded` beyond the configured wind or volume budgets). The boolean entry points (`sampleWindVelocity`, `applyWindDrag`, `setForceVolume`, `applyForceVolumes`, `applyRadialImpulse`) report unknown handles and invalid inputs through their `false` return; the `InvalidHandle` and `SimulationFailure` enum values name the unknown-handle and rejected-by-the-rigid-world cases for consumers that map failures onto the enum.

## Simulation model

**Wind fields.** A wind field is an analytic velocity function of position and time only, so sampling is stateless, deterministic, and repeatable. The velocity is the normalized base direction times `baseSpeed x max(1 + gust + turbulence, 0)` — clamped at zero so gusts never reverse the flow. The gust term is `gustStrength x (0.6 x sin(cyclePhase) + 0.4 x noise)`, where the cycle phase advances over `gustPeriodSeconds` and the noise samples a front coordinate that travels downwind at the base speed, so gust fronts drift across the field. The turbulence term blends two octaves (0.65 and 0.35 weights, the second at 2.7x frequency) of the module's deterministic 3D value noise (integer lattice hashing with cubic-smooth interpolation, no state) sampled at the point on the `turbulenceWavelength` grid and drifting downwind with time, scaled by `turbulenceStrength`.

**Aerodynamic drag.** `applyWindDrag` collects the awake bodies of the wind's world through the rigid capability, samples the wind at each body's center, forms the relative air velocity (wind minus body velocity), and applies the quadratic drag law `0.5 x air_density x dragCoefficient x area x relativeSpeed x wind_drag_scale` along the relative direction as a force at the body center. Cross-section and drag coefficient are documented per-shape approximations: spheres use `pi x radius^2` with 0.47, boxes the average face area with 1.05, capsules `pi x radius^2 + 2 x radius x length` with 0.82; planes and other zero-area shapes receive no drag, and bodies already riding the wind (relative speed below `1.0e-6`) are skipped.

**Force volumes.** A Uniform volume applies the description acceleration to every dynamic body whose center stays inside the axis-aligned box (per-axis `|offset| <= halfExtents`), scaled by the body's mass — `density x closed-form shape volume` for density-described bodies, else the body's mass. A Radial volume pulls (negative `radialStrength`) or pushes (positive) along the away-from-center direction with the bounded inverse-square falloff `(1 - distance/radius) / max(distance^2, 1)` inside the influence radius; radius 0 or strength 0 keeps the volume inactive, and disabled volumes are skipped entirely. Forces apply at body centers through the rigid module's `applyForceAtPoint` and are counted in the statistics when the rigid world accepts them.

**Radial impulse blasts.** `applyRadialImpulse` is a one-shot that needs no persistent record: every awake dynamic body of the given world within `radius` receives `impulseMagnitude x (1 - distance/radius)` along the away-from-center direction, with the `upwardBias` fraction added to the direction's +Z component before normalization, through the rigid module's `applyImpulseAtPoint`.

**Composition.** The module owns no simulation state: wind fields are analytic functions, volumes are placement records, and every application drives forces and impulses through the physics_rigid capability inside the consumer's fixed update, so fields compose deterministically with any other force source and rigid integration remains the single mover of bodies.

## Provided / required / optional capabilities

From `module.toml` and the entry table at the bottom of `source/physics_fields.cpp` (they agree):

- **Provided:** `physics_fields`.
- **Required:** `physics_rigid`.
- **Optional:** none.
- **Declared permissions:** none.

## Architecture

| File | Purpose |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capability, required capability, declared `config_keys`. |
| `config.toml` | Default values for the `[physics_fields]` configuration section (see `configuration.md`). |
| `CMakeLists.txt` | Builds `engine_module_physics_fields` (a `.so` under `runtime/modules/physics_fields/`) and copies `module.toml` next to it. |
| `include/engine/physics_fields/capabilities.hpp` | Public capability types: ABI constant, wind and force-volume handles, wind field and force volume descriptions, statistics, capability struct. |
| `source/physics_fields.cpp` | Module class (lifecycle, configuration, wind evaluation, drag and volume application, impulse blasts, capability binding) and entry table. |
| `include/engine/physics_rigid/capabilities.hpp` | The rigid module's public capability surface: world and body handles, body state and description copies, and the `applyForceAtPoint` / `applyImpulseAtPoint` entry points every field force flows through. |

## How it works

**Creation.** `createWindField` and `createForceVolume` validate and copy the description (the wind direction is normalized once and stored), assign monotonic handles, and keep handle-index maps for lookup; the live counts surface in the statistics. Body collection is shared by all apply entry points: the pass asks the rigid world for its body count, copies the handle list, then copies each body's state and description, keeping only awake bodies for the force loop.

**Application path.** `applyWindDrag` and `applyForceVolumes` belong at the top of the consumer's fixed update, before stepping the rigid world: forces are applied at body centers through `applyForceAtPoint`, and each accepted application bumps `dragApplications` or `volumeApplications`. `applyRadialImpulse` delivers its impulses through `applyImpulseAtPoint` the moment it is called. `sampleWindVelocity` is pure and allocation-free, so effects code can sample the same wind the physics feels.

**Statistics.** `fieldsStatistics` copies the `FieldsStatistics` struct: the live `windFieldCount` and `forceVolumeCount` (maintained on create and destroy) and the cumulative `dragApplications` and `volumeApplications`.

**Interaction with rigid bodies.** Fields read nothing from the rigid module beyond body enumeration and state copies, and write nothing back beyond forces and impulses, so any body kind works the same: only awake bodies are visited (static scenery stays put), dynamic boxes catch the wind, and kinematic carriers respond to volumes and blasts. Rigid integration stays the single source of motion — fields is a pure force producer, one-way coupled by design (see configuration.md for the cost trade-off this represents).

## Threading and lifetime

The module has no locks; serialize calls from one thread. Wind and volume records live in module vectors and die with the module: `destroyWindField` and `destroyForceVolume` erase the record (swap-with-back keeps the vectors dense) and refresh the live counts; `stop()` clears every record, resets the handle counters to 1, and zeroes the statistics. Handles are monotonic and never reused. The apply entry points read the rigid world's body state through the rigid capability, so they belong in the same fixed update as the rigid world's own step, sequenced before it; those reads are read-only, so apply calls may run concurrently with rigid `step` calls of a different world only if the consumer guarantees the worlds are distinct.

A consumer creates a gusty crosswind and a repulsor volume and drives both from its fixed update:

```cpp
engine::physics_fields::FieldsCapability &fieldsCapability =
    context.acquireCapability<engine::physics_fields::FieldsCapability>();
if (fieldsCapability.abi != engine::physics_fields::physicsFieldsCapabilityAbi)
{
    return;
}
engine::physics_fields::WindFieldDescription windDescription{};
windDescription.direction[0] = 0.0;
windDescription.direction[2] = 1.0;
windDescription.baseSpeed = 8.0;
windDescription.gustStrength = 0.4;
const engine::physics_fields::WindHandle wind =
    fieldsCapability.createWindField(fieldsCapability.self, rigidWorld, &windDescription, nullptr);

engine::physics_fields::ForceVolumeDescription repulsor{};
repulsor.kind = engine::physics_fields::ForceVolumeDescription::Kind::Radial;
repulsor.center[1] = 2.0;
repulsor.radius = 12.0;
repulsor.radialStrength = 6.0; // positive pushes bodies away from the center
const engine::physics_fields::ForceVolumeHandle volume =
    fieldsCapability.createForceVolume(fieldsCapability.self, rigidWorld, &repulsor, nullptr);

// per fixed update, before stepping the rigid world:
fieldsCapability.applyWindDrag(fieldsCapability.self, wind, simulationTimeSeconds, fixedDeltaSeconds);
fieldsCapability.applyForceVolumes(fieldsCapability.self, fixedDeltaSeconds);
```
