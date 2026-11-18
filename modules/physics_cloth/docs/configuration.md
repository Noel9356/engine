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
| `physics_cloth.substeps_per_step` | integer | 1 - 64 | `8` | XPBD substeps per fixed step. More substeps stiffen the effective solver at proportional cost; the compliance scaling keeps material behavior constant across values. |

## Out-of-range and invalid values

Out-of-range numeric values, missing values, and non-finite values are ignored and the built-in default stays active. Budgets are enforced when cloths are created: `createCloth` reports `CapacityExceeded` beyond `maximum_cloths`, `maximum_particles_per_cloth`, or the shared `maximum_particles` budget. Lowering a configured limit affects new cloths; it does not evict existing ones. `stop()` releases all cloths and resets the handle counters.

## Where the settings show up

- The step cost scales with `particles x substeps_per_step` (constraint iterations are one per substep by the XPBD small-steps schedule), so the substep key is the direct quality-versus-cost dial; see [overview.md](overview.md) for the stepping path.
- The particle budgets bound the dense arrays: positions, velocities, previous positions, inverse masses, pin flags, and contact scratch are per-particle vectors sized at creation.
- The accumulator keys (`fixed_rate_hz`, `maximum_steps_per_simulate`) mirror the rigid module's contract so consumers can drive both simulations from one FrameClock loop.
