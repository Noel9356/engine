# physics_fluid configuration

All keys live in the `[physics_fluid]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. `module.toml` mirrors every key under `config_keys` for documentation; the kernel validates known keys for engine sections and plugin manifests only, so unknown `[physics_fluid]` keys are silently accepted.

Every value is read once in `construct()`; the module does not follow configuration reloads. Keys are listed alphabetically.

| Key | Type | Valid range / values | Default | Effect |
| --- | --- | --- | --- | --- |
| `physics_fluid.fixed_rate_hz` | number | [1, 1000] | `60.0` | Fixed step rate (steps per second) used by `simulate`'s accumulator. `step` callers choose their own delta and must keep it constant. |
| `physics_fluid.maximum_fluids` | integer | 1 - 4096 | `16` | Maximum concurrently existing fluids. |
| `physics_fluid.maximum_particles` | integer | 1 - 10000000 | `500000` | Module-wide particle budget shared by all fluids. |
| `physics_fluid.maximum_particles_per_fluid` | integer | 1 - 1000000 | `50000` | Largest particle count one fluid accepts (checked per `addParticles` call and in total). |
| `physics_fluid.maximum_steps_per_simulate` | integer | 1 - 1000 | `8` | Cap on fixed steps executed by one `simulate` call; backlog beyond the cap is dropped. |
| `physics_fluid.maximum_velocity` | number | [1, 1000] | `30.0` | Stability clamp in meters per second: every substep rescales any faster particle's velocity down to this magnitude. |
| `physics_fluid.substeps_per_step` | integer | 1 - 64 | `4` | SPH substeps per fixed step (spatial-hash rebuild, density, forces, integration, and collision run per substep). More substeps shorten the integration interval at proportional cost. |

## Out-of-range and invalid values

Out-of-range numeric values, missing values, and non-finite values are ignored and the built-in default stays active. Budgets are enforced when fluids are created and populated: `createFluid` reports `CapacityExceeded` beyond `maximum_fluids` or once the shared `maximum_particles` budget is fully consumed, and `addParticles` reports `CapacityExceeded` beyond `maximum_particles_per_fluid` or the shared budget. Lowering a configured limit affects new fluids and appends; it does not evict existing particles. `stop()` releases all fluids and resets the handle counters.

## Where the settings show up

- The step cost scales with `particles x substeps_per_step` (a full spatial-hash rebuild, density pass, force pass, and integration run per substep), so the substep key is the direct quality-versus-cost dial; see [overview.md](overview.md) for the stepping path.
- `maximum_velocity` is applied during every substep's integration as the speed clamp that keeps pressure spikes bounded.
- The particle budgets bound the dense arrays: positions, velocities, densities, pressures, and accelerations are per-particle vectors sized at creation and append time, alongside the spatial-hash scratch (sorted indices and cell buckets) rebuilt per substep.
- The accumulator keys (`fixed_rate_hz`, `maximum_steps_per_simulate`) mirror the rigid module's contract so consumers can drive the rigid world and every fluid from one FrameClock loop.
