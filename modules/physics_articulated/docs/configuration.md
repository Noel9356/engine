# physics_articulated configuration

All keys live in the `[physics_articulated]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. `module.toml` mirrors every key under `config_keys` for documentation; the kernel validates known keys for engine sections and plugin manifests only, so unknown `[physics_articulated]` keys are silently accepted.

Every value is read once in `construct()`; the module does not follow configuration reloads. Keys are listed alphabetically.

| Key | Type | Valid range / values | Default | Effect |
| --- | --- | --- | --- | --- |
| `physics_articulated.maximum_drive_torque` | number | [1, 1000000] | `10000.0` | Upper bound on the torque budget `setJointDrive` accepts for one drive. |
| `physics_articulated.maximum_drive_velocity` | number | [1, 10000] | `100.0` | Upper bound (radians per second) on the velocity limit `setJointDrive` accepts; the drive's own `maximumVelocity` is clamped into this range before each `updateDrives`. |
| `physics_articulated.maximum_figures` | integer | 1 - 4096 | `256` | Maximum concurrently existing figures. |
| `physics_articulated.lod_animation_threshold` | number | (0, 1] | `0.1` | Importance score at which the Animation band begins; scores below it sit in Cached. Must stay strictly below `lod_procedural_threshold`. |
| `physics_articulated.lod_demotion_margin` | number | [0, 1) | `0.8` | Hysteresis fraction: a figure keeps its current band until its score falls below that band's entry threshold times this margin. |
| `physics_articulated.lod_enabled` | boolean | (boolean) | `false` | Turns the simulation-LOD ladder on; with the switch off every figure refreshes its drives every step whatever its score. |
| `physics_articulated.lod_full_threshold` | number | (0, 1] | `0.75` | Importance score at which the Full band begins. |
| `physics_articulated.lod_interval_animation` | integer | 0 - 65536 | `8` | Fixed steps between drive refreshes for the Animation band; 0 freezes the band's refreshes. |
| `physics_articulated.lod_interval_cached` | integer | 0 - 65536 | `0` | Fixed steps between drive refreshes for the Cached band; 0 freezes the band's refreshes. |
| `physics_articulated.lod_interval_procedural` | integer | 0 - 65536 | `4` | Fixed steps between drive refreshes for the Procedural band. |
| `physics_articulated.lod_interval_simplified` | integer | 0 - 65536 | `2` | Fixed steps between drive refreshes for the Simplified band. |
| `physics_articulated.lod_procedural_threshold` | number | (0, 1] | `0.25` | Importance score at which the Procedural band begins. |
| `physics_articulated.lod_simplified_threshold` | number | (0, 1] | `0.5` | Importance score at which the Simplified band begins. |
| `physics_articulated.maximum_links_per_figure` | integer | 1 - 4096 | `128` | Largest link count accepted by `createFigure`. |

## Out-of-range and invalid values

Out-of-range numeric values, missing values, and non-finite values are ignored and the built-in default stays active. Budgets are enforced when figures are created: `createFigure` reports `CapacityExceeded` beyond `maximum_figures`, `maximum_links_per_figure`, or when the collision-category pool is exhausted. Lowering a configured limit affects new figures; it does not evict existing ones. `stop()` releases all figures and resets the handle counters.

## Where the settings show up

- The link budget bounds the memory a single figure can reserve in the rigid world (one body and up to one joint per link live in the rigid core's own budgets).
- The drive clamps bound the authority a drive can exercise over a figure regardless of what a consumer requests, which keeps scripted or remote-controlled pose targets from destabilizing the rigid solver. See [overview.md](overview.md) for the drive control law and the collision-category pool.
