# physics_character configuration

All keys live in the `[physics_character]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. `module.toml` mirrors every key under `config_keys` for documentation; the kernel validates known keys for engine sections and plugin manifests only, so unknown `[physics_character]` keys are silently accepted.

Every value is read once in `construct()`; the module does not follow configuration reloads. Keys are listed alphabetically.

| Key | Type | Valid range / values | Default | Effect |
| --- | --- | --- | --- | --- |
| `physics_character.maximum_characters` | integer | 1 - 4096 | `32` | Maximum concurrently existing characters (active-ragdoll control records over figures). |
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
