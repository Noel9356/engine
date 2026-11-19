# physics_cloth configuration

All keys live in the `[physics_cloth]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. `module.toml` mirrors every key under `config_keys` for documentation; the kernel validates known keys for engine sections and plugin manifests only, so unknown `[physics_cloth]` keys are silently accepted.

Every value is read once in `construct()`; the module does not follow configuration reloads. Keys are listed alphabetically.

| Key | Type | Valid range / values | Default | Effect |
| --- | --- | --- | --- | --- |
| `physics_cloth.fixed_rate_hz` | number | [1, 1000] | `60.0` | Fixed step rate (steps per second) used by `simulate`'s accumulator. `step` callers choose their own delta and must keep it constant. |
| `physics_cloth.maximum_cloths` | integer | 1 - 4096 | `64` | Maximum concurrently existing cloths. |
| `physics_cloth.maximum_particles` | integer | 1 - 10000000 | `200000` | Module-wide particle budget shared by all cloths. |
| `physics_cloth.maximum_particles_per_cloth` | integer | 1 - 1000000 | `65536` | Largest particle count accepted by one `createCloth` call (a 256 x 256 grid fits). |
| `physics_cloth.maximum_steps_per_simulate` | integer | 1 - 1000 | `8` | Cap on fixed steps executed by one `simulate` call; backlog beyond the cap is dropped. |
| `physics_cloth.lod_animation_threshold` | number | (0, 1] | `0.1` | Fidelity-ladder boundary between Animation and Cached; scores below it freeze cloths. Must stay below `lod_procedural_threshold`. |
| `physics_cloth.lod_demotion_margin` | number | [0, 1) | `0.8` | Demotion hysteresis fraction; a cloth keeps its band until its importance falls below the band's entry threshold times this margin. |
| `physics_cloth.lod_enabled` | boolean | on/off | `false` | Simulation LOD master switch: bands below Full reduce the internal XPBD substep count, and a due step integrates the accumulated fixed deltas in one pass, so the cloth keeps advancing at wall time. |
| `physics_cloth.lod_full_threshold` | number | (0, 1] | `0.75` | Importance at or above which a cloth runs the Full band. Must stay above `lod_simplified_threshold`. |
| `physics_cloth.lod_interval_animation` | integer | 0 - 65536 | `8` | Fixed steps between updates for Animation-band cloths; `0` freezes them. |
| `physics_cloth.lod_interval_cached` | integer | 0 - 65536 | `0` | `0` freezes Cached-band cloths (the default). |
| `physics_cloth.lod_interval_procedural` | integer | 0 - 65536 | `4` | Fixed steps between updates for Procedural-band cloths. |
| `physics_cloth.lod_interval_simplified` | integer | 0 - 65536 | `2` | Fixed steps between updates for Simplified-band cloths. |
| `physics_cloth.lod_procedural_threshold` | number | (0, 1] | `0.25` | Boundary between Simplified and Procedural. |
| `physics_cloth.lod_simplified_threshold` | number | (0, 1] | `0.5` | Boundary between Full and Simplified. |
| `physics_cloth.maximum_velocity` | number | (0, 1000] | `30.0` | Clamp applied to the per-substep particle velocities (meters per second), keeping solver noise from escalating. |
| `physics_cloth.sleep_velocity_threshold` | number | (0, 100] | `0.05` | A cloth whose free particles all stay below this speed (meters per second) accumulates quiet time toward the sleep state. |
| `physics_cloth.time_to_sleep` | number | [0, 60] | `0.6` | Quiet time (seconds) after which a cloth with `sleepingEnabled` sleeps until a drag, pin, or wind change — or an obstacle overlapping the resting surface — wakes it. |
| `physics_cloth.solver_iterations` | integer | 1 - 16 | `1` | Constraint solve passes per XPBD substep. The default of 1 is the small-steps schedule (stiffness comes from the substep count); larger values repeat the Gauss-Seidel walk for stiffer solves at the same substep count. Tearing is judged on the first pass only. |
| `physics_cloth.substeps_per_step` | integer | 1 - 64 | `8` | XPBD substeps per fixed step. More substeps stiffen the effective solver at proportional cost; the compliance scaling keeps material behavior constant across values. |
| `physics_cloth.wind_drag_coefficient` | number | [0, 100] | `1.5` | Aerodynamic-wind drag scale (a1): the share of each face's relative wind velocity that pushes along the face normal. Applies to cloths created with `ClothDescription.aerodynamicWind`; the uniform model ignores it. |
| `physics_cloth.wind_lift_coefficient` | number | [0, 100] | `0.2` | Aerodynamic-wind lift scale (a2 = a3): the two in-plane lift shares of the Baraff-Witkin per-face decomposition; this term is what turns a steady wind into flutter. Applies to cloths created with `ClothDescription.aerodynamicWind`. |

## Out-of-range and invalid values

Out-of-range numeric values, missing values, and non-finite values are ignored and the built-in default stays active. Budgets are enforced when cloths are created: `createCloth` reports `CapacityExceeded` beyond `maximum_cloths`, `maximum_particles_per_cloth`, or the shared `maximum_particles` budget. Lowering a configured limit affects new cloths; it does not evict existing ones. `stop()` releases all cloths and resets the handle counters.

## Where the settings show up

- The step cost scales with `particles x substeps_per_step` (constraint iterations are one per substep by the XPBD small-steps schedule), so the substep key is the direct quality-versus-cost dial; see [overview.md](overview.md) for the stepping path.
- The particle budgets bound the dense arrays: positions, velocities, previous positions, inverse masses, pin flags, and contact scratch are per-particle vectors sized at creation.
- The accumulator keys (`fixed_rate_hz`, `maximum_steps_per_simulate`) mirror the rigid module's contract so consumers can drive both simulations from one FrameClock loop.
