# physics_water configuration

All keys live in the `[physics_water]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. `module.toml` mirrors every key under `config_keys` for documentation; the kernel validates known keys for engine sections and plugin manifests only, so unknown `[physics_water]` keys are silently accepted.

Every value is read once in `construct()`; the module does not follow configuration reloads. Keys are listed alphabetically.

| Key | Type | Valid range / values | Default | Effect |
| --- | --- | --- | --- | --- |
| `physics_water.angular_drag_coefficient` | number | [0, 100] | `3.0` | Angular damping scale for submerged bodies: torque = `coefficient x submergedFraction x mass x angularSpeed x 0.01`, opposing the spin. |
| `physics_water.gerstner_iterations` | integer | 1 - 32 | `4` | Fixed-point iterations of the displaced-height inverse mapping: more iterations sharpen the height recovery of steep Gerstner surfaces at proportional query cost. |
| `physics_water.normal_sample_epsilon` | number | (0, 1] | `0.05` | Central-difference offset in meters for the surface normal: small relative to the wave curvature, large enough to stay above the height function's own precision. |
| `physics_water.buoyancy_scale` | number | [0, 100] | `1.0` | Global multiplier on the displaced-volume term of the buoyancy force; `0` disables buoyancy while surface queries keep working. |
| `physics_water.quadratic_drag_coefficient` | number | [0, 100] | `1.8` | Quadratic water drag coefficient: force = `0.5 x water_density x coefficient x dragArea x submergedFraction x speed^2` opposite the velocity. |
| `physics_water.maximum_oceans` | integer | 1 - 4096 | `16` | Maximum concurrently existing oceans; `createOcean` reports `CapacityExceeded` beyond it. |
| `physics_water.maximum_registered_bodies` | integer | 1 - 100000 | `4096` | Largest number of bodies one ocean accepts for buoyancy registration; `registerBody` reports `CapacityExceeded` beyond it. |
| `physics_water.lod_animation_threshold` | number | (0, 1] | `0.1` | Fidelity-ladder boundary between Animation and Cached; scores below it freeze oceans. |
| `physics_water.lod_demotion_margin` | number | [0, 1) | `0.8` | Demotion hysteresis fraction. |
| `physics_water.lod_enabled` | boolean | on/off | `false` | Simulation LOD master switch: oceans below Full run their ripple and buoyancy passes on a staggered cadence whose due passes cover the whole interval of wall time. |
| `physics_water.lod_full_threshold` | number | (0, 1] | `0.75` | Importance at or above which an ocean runs the Full band. |
| `physics_water.lod_interval_animation` | integer | 0 - 65536 | `8` | Fixed steps one due pass covers for Animation-band oceans; `0` freezes them. |
| `physics_water.lod_interval_cached` | integer | 0 - 65536 | `0` | `0` freezes Cached-band oceans (the default). |
| `physics_water.lod_interval_procedural` | integer | 0 - 65536 | `4` | Fixed steps one due pass covers for Procedural-band oceans. |
| `physics_water.lod_interval_simplified` | integer | 0 - 65536 | `2` | Fixed steps one due pass covers for Simplified-band oceans. |
| `physics_water.lod_procedural_threshold` | number | (0, 1] | `0.25` | Boundary between Simplified and Procedural. |
| `physics_water.lod_simplified_threshold` | number | (0, 1] | `0.5` | Boundary between Full and Simplified. |
| `physics_water.ripple_retire_envelope` | number | (0, 1) | `0.001` | A ripple retires when its exponential envelope decays below this amplitude fraction of its initial strength. |
| `physics_water.wake_submerged_fraction` | number | (0, 1] | `0.95` | Bodies at or above this submersion fraction emit no wakes. |
| `physics_water.maximum_ripples_per_ocean` | integer | 1 - 4096 | `64` | Ripple slot count fixed per ocean at creation; `addRipple` recycles the oldest slot once the budget is full. |
| `physics_water.ripple_attenuation` | number | [0, 100] | `0.35` | Ripple amplitude falloff per meter of ring radius (`exp(-radius x attenuation)`). |
| `physics_water.ripple_decay` | number | (0, 100] | `1.2` | Exponential decay per second of ripple amplitude (`exp(-age x decay)`); ripples retire below the `ripple_retire_envelope` fraction of their initial amplitude. |
| `physics_water.ripple_speed` | number | (0, 100] | `4.0` | Ring front expansion speed in meters per second. |
| `physics_water.ripple_wavelength` | number | (0.0001, 1000] | `1.6` | Ripple crest spacing in meters: the band width around the ring front and the cosine period of the ring. |
| `physics_water.wake_spacing_meters` | number | (0, 1000] | `1.5` | Travelled horizontal distance one registered body accumulates between wake ripples in `stepRipples`; the accumulator resets each time a wake spawns, discarding any excess. |
| `physics_water.wake_speed_threshold` | number | (0, 100] | `1.2` | Minimum horizontal speed in meters per second for a registered body to emit wake ripples. |
| `physics_water.wake_strength_scale` | number | [0, 10] | `0.35` | Wake ripple amplitude per meter per second of body speed; `0` disables automatic wake emission. |
| `physics_water.wave_gravity` | number | (0.1, 100] | `9.81` | Deep-water dispersion reference gravity `omega = sqrt(g x k)`: Earth's 9.81 by default, independent of the scene gravity the buoyancy pass reads per world. |
| `physics_water.water_density` | number | (0, 20000] | `1000.0` | Water density in kilograms per cubic meter, the density factor of both the buoyancy force and the quadratic drag law. |

## Out-of-range and invalid values

Out-of-range numeric values, missing values, and non-finite values are ignored and the built-in default stays active. Budgets are enforced when oceans and registrations are created: `createOcean` reports `CapacityExceeded` beyond `maximum_oceans` and `registerBody` beyond `maximum_registered_bodies`; `maximum_ripples_per_ocean` fixes each ocean's slot array at creation, and `addRipple` recycles the oldest slot instead of failing when full. Lowering a configured limit affects new oceans; it does not evict existing ones. `stop()` releases all oceans and resets the handle and statistics counters. The wake keys follow the same rule: an out-of-range value leaves the built-in default active, so a `wake_speed_threshold` of `0` (below the open lower bound) does not turn every drifting body into a wake source, and a `wake_strength_scale` above 10 falls back to `0.35` instead of amplifying rings without bound.

## Where the settings show up

- The four force keys (`water_density`, `buoyancy_scale`, `quadratic_drag_coefficient`, `angular_drag_coefficient`) retune the entire buoyancy pass without touching any ocean description or registration; the default `1000.0` is fresh water in kilograms per cubic meter.
- The ripple keys (`ripple_speed`, `ripple_decay`, `ripple_attenuation`, `ripple_wavelength`) parameterize every ring's expansion, decay, radial falloff, and crest spacing, including the retirement threshold `stepRipples` applies; see [overview.md](overview.md) for the ripple model.
- The budget keys bound the per-ocean slot arrays and registration lists, and the live counts surface in `WaterStatistics` (`oceanCount`, `registeredBodies`, `activeRipples`).
- The wake keys (`wake_speed_threshold`, `wake_spacing_meters`, `wake_strength_scale`) drive the automatic ripple emission inside `stepRipples`: the speed gate, the travelled distance between rings, and the amplitude per meter per second of speed. Bodies whose `querySubmersion`-style submerged fraction reaches `wake_submerged_fraction` emit nothing; see [overview.md](overview.md) for the full wake rule.
