# physics_foliage configuration

All keys live in the `[physics_foliage]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. `module.toml` mirrors every key under `config_keys` for documentation; the kernel validates known keys for engine sections and plugin manifests only, so unknown `[physics_foliage]` keys are silently accepted.

Every value is read once in `construct()`; the module does not follow configuration reloads. Keys are listed alphabetically.

| Key | Type | Valid range / values | Default | Effect |
| --- | --- | --- | --- | --- |
| `physics_foliage.ambient_wind_direction` | number[2] | two finite numbers | `[1.0, 0.0]` | Horizontal (x, z) direction of the fallback ambient breeze, normalized at sampling time; a near-zero vector yields zero wind. Used only when a field samples without a bound wind handle. |
| `physics_foliage.ambient_wind_speed` | number | [0, 100] | `0.0` | Speed (meters per second) of the ambient breeze; `0.0` keeps foliage still unless a field binds a physics_fields wind handle. |
| `physics_foliage.fixed_rate_hz` | number | [1, 1000] | `60.0` | Fixed step rate (steps per second) used by `simulate`'s accumulator. `step` callers choose their own delta and must keep it constant. |
| `physics_foliage.maximum_fields` | integer | 1 - 4096 | `64` | Maximum concurrently existing fields. |
| `physics_foliage.maximum_instances` | integer | 1 - 10000000 | `2000000` | Module-wide instance budget shared by all fields. |
| `physics_foliage.maximum_instances_per_field` | integer | 1 - 1000000 | `500000` | Largest instance count accepted by one `createField` call. |
| `physics_foliage.maximum_steps_per_simulate` | integer | 1 - 1000 | `8` | Cap on fixed steps executed by one `simulate` call; backlog beyond the cap is dropped and the accumulator resets. |
| `physics_foliage.lod_animation_threshold` | number | (0, 1] | `0.1` | Fidelity-ladder boundary between Animation and Cached; scores below it freeze fields. |
| `physics_foliage.lod_demotion_margin` | number | [0, 1) | `0.8` | Demotion hysteresis fraction. |
| `physics_foliage.lod_enabled` | boolean | on/off | `false` | Simulation LOD master switch: fields below Full update on a staggered cadence whose due steps cover the whole interval of wall time. |
| `physics_foliage.lod_full_threshold` | number | (0, 1] | `0.75` | Importance at or above which a field runs the Full band. |
| `physics_foliage.lod_interval_animation` | integer | 0 - 65536 | `8` | Fixed steps one due step covers for Animation-band fields; `0` freezes them. |
| `physics_foliage.lod_interval_cached` | integer | 0 - 65536 | `0` | `0` freezes Cached-band fields (the default). |
| `physics_foliage.lod_interval_procedural` | integer | 0 - 65536 | `4` | Fixed steps one due step covers for Procedural-band fields. |
| `physics_foliage.lod_interval_simplified` | integer | 0 - 65536 | `2` | Fixed steps one due step covers for Simplified-band fields. |
| `physics_foliage.lod_procedural_threshold` | number | (0, 1] | `0.25` | Boundary between Simplified and Procedural. |
| `physics_foliage.lod_simplified_threshold` | number | (0, 1] | `0.5` | Boundary between Full and Simplified. |
| `physics_foliage.sway_amplitude` | number | [0, 100] | `0.15` | Sway amplitude factor (meters per second of target oscillation at full wind speed and bend scale). |
| `physics_foliage.wind_sample_stride` | integer | 1 - 65536 | `1` | Instance wind samples refresh every Nth step, phase-spread by instance index; the cached sample drives the bend target between refreshes. `1` samples every step. |

## Out-of-range and invalid values

Out-of-range numeric values, missing values, and non-finite values are ignored and the built-in default stays active; `ambient_wind_direction` is accepted only when it carries exactly two finite components. Budgets are enforced when fields are created: `createField` reports `CapacityExceeded` beyond `maximum_fields`, `maximum_instances_per_field`, or the shared `maximum_instances` budget. Lowering a configured limit affects new fields; it does not evict existing ones. `stop()` releases all fields and resets the handle counters and statistics.

## Where the settings show up

- The per-step cost scales with the total instance count (one wind sample, one target-and-sway computation, and one spring integration per instance per step), so the instance budgets are the direct cost dials; see [overview.md](overview.md) for the stepping path.
- The ambient breeze keys matter only when the physics_fields capability is absent or a field was created with `windFieldHandle = 0`; fields bound to a wind handle ignore them.
- The accumulator keys (`fixed_rate_hz`, `maximum_steps_per_simulate`) mirror the rigid and cloth modules' contract so consumers can drive every simulation from one FrameClock loop.
