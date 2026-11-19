# physics_particles configuration

All keys live in the `[physics_particles]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. `module.toml` mirrors every key under `config_keys` for documentation; the kernel validates known keys for engine sections and plugin manifests only, so unknown `[physics_particles]` keys are silently accepted.

Every value is read once in `construct()`; the module does not follow configuration reloads. Keys are listed alphabetically.

| Key | Type | Valid range / values | Default | Effect |
| --- | --- | --- | --- | --- |
| `physics_particles.fixed_rate_hz` | number | [1, 1000] | `60.0` | Fixed step rate (steps per second) used by `simulate`'s accumulator. `step` callers choose their own delta and must keep it constant. |
| `physics_particles.insect_steer_gain` | number | (0, 1000] | `8.0` | Proportional gain of the insect wander controller (acceleration per meter per second of velocity error toward the hash-derived hover target). |
| `physics_particles.insect_wander_radius` | number | (0, 1000] | `1.5` | Radius of the soft pull that keeps each insect near its spawn (or burst) point; beyond it the wander target gains an inward correction. |
| `physics_particles.leaf_settle_seconds` | number | (0, 60] | `0.8` | How long a leaf particle holds its impact contact before expiring. |
| `physics_particles.lod_animation_threshold` | number | (0, 1] | `0.1` | Fidelity-ladder boundary between Animation and Cached; scores below it freeze emitters. Must stay below `lod_procedural_threshold`. |
| `physics_particles.lod_demotion_margin` | number | [0, 1) | `0.8` | Demotion hysteresis fraction: an emitter keeps its band until its importance falls below the band's entry threshold times this margin. |
| `physics_particles.lod_enabled` | boolean | on/off | `false` | Simulation LOD master switch. When off, every emitter runs the Full band every step; when on, emitters below Full update on a staggered cadence whose due steps cover the whole interval of wall time. |
| `physics_particles.lod_full_threshold` | number | (0, 1] | `0.75` | Importance at or above which an emitter runs the Full band (every step). Must stay above `lod_simplified_threshold`. |
| `physics_particles.lod_interval_animation` | integer | 0 - 65536 | `8` | Fixed steps one due step covers for Animation-band emitters; `0` freezes them. |
| `physics_particles.lod_interval_cached` | integer | 0 - 65536 | `0` | `0` freezes Cached-band emitters (the default); a non-zero value turns the Cached band into a slow cadence instead. |
| `physics_particles.lod_interval_procedural` | integer | 0 - 65536 | `4` | Fixed steps one due step covers for Procedural-band emitters. |
| `physics_particles.lod_interval_simplified` | integer | 0 - 65536 | `2` | Fixed steps one due step covers for Simplified-band emitters. |
| `physics_particles.lod_procedural_threshold` | number | (0, 1] | `0.25` | Boundary between Simplified and Procedural; must stay between the Simplified and Animation thresholds. |
| `physics_particles.lod_simplified_threshold` | number | (0, 1] | `0.5` | Boundary between Full and Simplified; must stay below `lod_full_threshold`. |
| `physics_particles.maximum_emitters` | integer | 1 - 4096 | `128` | Maximum concurrently existing emitters. |
| `physics_particles.maximum_particles` | integer | 1 - 10000000 | `1000000` | Module-wide particle budget across all emitters; each emitter's own pool is additionally bounded by `maximum_particles_per_emitter`. |
| `physics_particles.maximum_particles_per_emitter` | integer | 1 - 1000000 | `100000` | Largest pool one emitter may hold; `createEmitter` and `setEmitterDescription` reject descriptions above it and spawn growth stops there. |
| `physics_particles.maximum_steps_per_simulate` | integer | 1 - 1000 | `8` | Cap on fixed steps executed by one `simulate` call; backlog beyond the cap is dropped. |

## Out-of-range and invalid values

Out-of-range numeric values, missing values, and non-finite values are ignored and the built-in default stays active. The emitter budget is enforced at creation (`createEmitter` reports `CapacityExceeded` beyond `maximum_emitters`), and the per-emitter pool bound is enforced against descriptions (`InvalidDescription` above `maximum_particles_per_emitter`); a shrink through `setEmitterDescription` truncates the pool in place. Lowering a configured limit affects new emitters; it does not evict existing ones. `stop()` releases all emitters and resets the handle counters and statistics.

## Where the settings show up

- The step cost scales with the pooled particles (spawn, integrate, and collision passes sweep every emitter's pool each fixed step), so the per-emitter pool bound is the main cost dial; see [overview.md](overview.md) for the stepping path.
- The pool bound sizes each emitter's slot array, reserved at creation and resized by `setEmitterDescription`; the module-wide budget states the ceiling all pools share.
- The accumulator keys (`fixed_rate_hz`, `maximum_steps_per_simulate`) mirror the rigid module's contract so consumers can drive the rigid world and every emitter from one FrameClock loop.
- The LOD keys (`lod_enabled`, the four thresholds, the demotion margin, and the per-band intervals) shape the staggered cadence: emitters below Full integrate one due step with interval x the caller's delta, so the emission rate and lifetime pacing stay wall-time exact while the per-step cost drops; see the overview's simulation-LOD section.
- The behavior keys (`leaf_settle_seconds`, `insect_wander_radius`, `insect_steer_gain`) tune the leaves' contact hold and the insect wander controller without code changes.
