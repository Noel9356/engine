# physics_character configuration

All keys live in the `[physics_character]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. `module.toml` mirrors every key under `config_keys` for documentation; the kernel validates known keys for engine sections and plugin manifests only, so unknown `[physics_character]` keys are silently accepted.

Every value is read once in `construct()`; the module does not follow configuration reloads. Keys are listed alphabetically.

| Key | Type | Valid range / values | Default | Effect |
| --- | --- | --- | --- | --- |
| `physics_character.jiggle_gravity_magnitude` | number | (0, 100] | `9.81` | Secondary-motion gravity magnitude in meters per square second; each jiggle chain scales it by its `gravityInfluence` factor when it builds the droop force. |
| `physics_character.maximum_characters` | integer | 1 - 4096 | `32` | Maximum concurrently existing characters (active-ragdoll control records over figures). |
| `physics_character.lod_animation_threshold` | number | (0, 1] | `0.1` | Importance score at which the Animation band begins; scores below it sit in Cached. Must stay strictly below `lod_procedural_threshold`. |
| `physics_character.lod_demotion_margin` | number | [0, 1) | `0.8` | Hysteresis fraction: a record keeps its current band until its score falls below that band's entry threshold times this margin. |
| `physics_character.lod_enabled` | boolean | (boolean) | `false` | Turns the simulation-LOD ladder on; with the switch off every character, chain, planter, and lean controller updates every step whatever its score. |
| `physics_character.lod_full_threshold` | number | (0, 1] | `0.75` | Importance score at which the Full band begins. |
| `physics_character.lod_interval_animation` | integer | 0 - 65536 | `8` | Fixed steps between updates for the Animation band; 0 freezes the band's records. |
| `physics_character.lod_interval_cached` | integer | 0 - 65536 | `0` | Fixed steps between updates for the Cached band; 0 freezes the band's records. |
| `physics_character.lod_interval_procedural` | integer | 0 - 65536 | `4` | Fixed steps between updates for the Procedural band. |
| `physics_character.lod_interval_simplified` | integer | 0 - 65536 | `2` | Fixed steps between updates for the Simplified band. |
| `physics_character.lod_procedural_threshold` | number | (0, 1] | `0.25` | Importance score at which the Procedural band begins. |
| `physics_character.lod_simplified_threshold` | number | (0, 1] | `0.5` | Importance score at which the Simplified band begins. |
| `physics_character.maximum_foot_planters` | integer | 1 - 4096 | `64` | Maximum concurrently existing foot planters. |
| `physics_character.maximum_hand_planters` | integer | 1 - 4096 | `64` | Maximum concurrently existing hand planters. |
| `physics_character.maximum_jiggle_chains` | integer | 1 - 4096 | `256` | Maximum concurrently existing jiggle chains. |
| `physics_character.maximum_lean_controllers` | integer | 1 - 4096 | `64` | Maximum concurrently existing lean controllers. |

## Out-of-range and invalid values

Out-of-range numeric values, missing values, and non-finite values are ignored and the built-in default stays active. Budgets are enforced at creation: `createCharacter`, `createJiggleChain`, `createFootPlanter`, `createHandPlanter`, and `createLeanController` report `CapacityExceeded` beyond their matching budget keys, or once a handle counter reaches the `std::uint32_t` limit. Description-level hard limits are independent of configuration: a jiggle chain accepts 1–256 bones regardless of the budget keys. Lowering a configured limit affects new objects; it does not evict existing ones. `stop()` releases all characters, chains, planters, hand planters, and lean controllers and resets the handle counters and statistics.

## Where the settings show up

- The per-update cost scales with the object counts (each `updateCharacter` walks the character's enabled joint targets, each planter update ray-casts twice, each chain update integrates `boneCount` bones, each hand update solves two two-bone IK chains, each lean update integrates one spring), so the five budget keys are the cost dials; see [overview.md](overview.md) for the control path.
- Record memory is sized at creation: a character's per-link joint-target array takes the figure's link count, a chain owns its rest offsets, spring states, and output rotations per bone, a planter owns a copy of its ignored-bodies list, and the hand and lean records are fixed-size.
- The statistics reported by `characterStatistics` show the live counts of all five record kinds against these budgets.
