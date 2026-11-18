# physics_particles configuration

All keys live in the `[physics_particles]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. `module.toml` mirrors every key under `config_keys` for documentation; the kernel validates known keys for engine sections and plugin manifests only, so unknown `[physics_particles]` keys are silently accepted.

Every value is read once in `construct()`; the module does not follow configuration reloads. Keys are listed alphabetically.

| Key | Type | Valid range / values | Default | Effect |
| --- | --- | --- | --- | --- |
| `physics_particles.fixed_rate_hz` | number | [1, 1000] | `60.0` | Fixed step rate (steps per second) used by `simulate`'s accumulator. `step` callers choose their own delta and must keep it constant. |
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
