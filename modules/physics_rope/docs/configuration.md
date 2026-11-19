# physics_rope configuration

All keys live in the `[physics_rope]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. `module.toml` mirrors every key under `config_keys` for documentation; the kernel validates known keys for engine sections and plugin manifests only, so unknown `[physics_rope]` keys are silently accepted.

Every value is read once in `construct()`; the module does not follow configuration reloads. Keys are listed alphabetically.

| Key | Type | Valid range / values | Default | Effect |
| --- | --- | --- | --- | --- |
| `physics_rope.fixed_rate_hz` | number | [1, 1000] | `60.0` | Fixed step rate (steps per second) used by `simulate`'s accumulator. `step` callers choose their own delta and must keep it constant. |
| `physics_rope.maximum_nodes` | integer | 1 - 10000000 | `200000` | Module-wide node budget shared by all ropes. |
| `physics_rope.maximum_nodes_per_rope` | integer | 1 - 100000 | `4096` | Largest node count (`segments + 1`) accepted by one `createRope` call. |
| `physics_rope.maximum_ropes` | integer | 1 - 4096 | `256` | Maximum concurrently existing ropes. |
| `physics_rope.maximum_steps_per_simulate` | integer | 1 - 1000 | `8` | Cap on fixed steps executed by one `simulate` call; backlog beyond the cap is dropped. |
| `physics_rope.lod_animation_threshold` | number | (0, 1] | `0.1` | Fidelity-ladder boundary between Animation and Cached; scores below it freeze ropes. |
| `physics_rope.lod_demotion_margin` | number | [0, 1) | `0.8` | Demotion hysteresis fraction. |
| `physics_rope.lod_enabled` | boolean | on/off | `false` | Simulation LOD master switch: bands below Full reduce the internal XPBD substep count instead of the fixed step size. |
| `physics_rope.lod_full_threshold` | number | (0, 1] | `0.75` | Importance at or above which a rope runs the Full band. |
| `physics_rope.lod_interval_animation` | integer | 0 - 65536 | `8` | Fixed steps between updates for Animation-band ropes; `0` freezes them. |
| `physics_rope.lod_interval_cached` | integer | 0 - 65536 | `0` | `0` freezes Cached-band ropes (the default). |
| `physics_rope.lod_interval_procedural` | integer | 0 - 65536 | `4` | Fixed steps between updates for Procedural-band ropes. |
| `physics_rope.lod_interval_simplified` | integer | 0 - 65536 | `2` | Fixed steps between updates for Simplified-band ropes. |
| `physics_rope.lod_procedural_threshold` | number | (0, 1] | `0.25` | Boundary between Simplified and Procedural. |
| `physics_rope.lod_simplified_threshold` | number | (0, 1] | `0.5` | Boundary between Full and Simplified. |
| `physics_rope.maximum_velocity` | number | (0, 1000] | `30.0` | Clamp applied to the per-substep node velocities (meters per second). |
| `physics_rope.substeps_per_step` | integer | 1 - 64 | `8` | XPBD substeps per fixed step. More substeps stiffen the effective solver at proportional cost; the compliance scaling keeps material behavior constant across values. |

## Out-of-range and invalid values

Out-of-range numeric values, missing values, and non-finite values are ignored and the built-in default stays active. Budgets are enforced when ropes are created: `createRope` reports `CapacityExceeded` beyond `maximum_ropes`, when a rope's `segments + 1` exceeds `maximum_nodes_per_rope`, or when the shared `maximum_nodes` budget is exhausted. Lowering a configured limit affects new ropes; it does not evict existing ones. `stop()` releases all ropes and resets the handle and node counters.

## Where the settings show up

- The step cost scales with `nodes x substeps_per_step` (each substep solves the stretch list once and then the bend list once), so the substep key is the direct quality-versus-cost dial; see [overview.md](overview.md) for the stepping path.
- The node budgets bound the per-rope dense arrays: positions, velocities, previous positions, inverse masses, attachment flags, pin positions, attached body handles, and local anchors are per-node vectors sized at creation.
- The accumulator keys (`fixed_rate_hz`, `maximum_steps_per_simulate`) mirror the rigid module's contract so consumers can drive both simulations from one FrameClock loop.
