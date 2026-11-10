# world configuration

The world module reads two sections because it bundles two coordinators: `[ecs]` feeds the embedded ecs module (`source/ecs.cpp`, `construct()`) and `[entity_world]` feeds the entity-world layer (`source/entity_world.cpp`, `construct()`). Defaults ship in `config.toml`; the application config (or a profile) may override them. The keys are mirrored in `module.toml` `config_keys` for documentation; the kernel validates known keys for engine sections and game-mod manifests only, so unknown `[ecs]`/`[entity_world]` keys are silently accepted.

Every value is read once in `construct()`; the module does not subscribe to `ConfigurationReloadedEvent`, so restarts are required for changes. Keys are listed alphabetically.

| Key | Type | Valid range (validated in `construct()`) | Default (`config.toml`) | Effect |
| --- | --- | --- | --- | --- |
| `ecs.maximum_worlds` | integer | 1 - 4096 | `64` | Maximum number of concurrently existing ecs worlds; `createWorld` beyond the limit fails with `CapacityExceeded`. |
| `entity_world.initial_entity_capacity` | integer | 1 - 1000000 | `256` | Default slot-vector reservation for a new entity world, used when its `EntityWorldDescription::initialEntityCapacity` is 0 (also clamped to the world's resolved entity maximum). |
| `entity_world.maximum_entities` | integer | 1 - 10000000 | `1000000` | Cap applied to each world's entity maximum: `createWorld` resolves `min(description maximum, this value)` (and uses this value when the description says 0). Also enforced by `createEntity` (`CapacityExceeded`) and by `loadWorld`'s slot-count check. |
| `entity_world.maximum_hierarchy_depth` | integer | 1 - 4096 | `256` | Cap applied to each world's hierarchy depth (same `min(description, this)` rule). Enforced by parent validation, world-transform recomputation, and `loadWorld`. |
| `entity_world.maximum_worlds` | integer | 1 - 65536 | `1024` | Maximum number of concurrently existing entity worlds; `createWorld` beyond the limit fails with `CapacityExceeded`. |

## Out-of-range and invalid values

A value outside its documented range (or a wrong TOML type) is silently ignored and the built-in default listed above stays active - unlike the window module, the world module prints no diagnostic. Because the limits act as caps on per-world description values, a description asking for more than the configured maximum is clamped down rather than failing; only creating beyond a world count limit fails.

## Sizing guidance

- `entity_world.maximum_entities` is the safety net behind every world's own maximum; raise it (and `initial_entity_capacity` with it) for scenes that stream many entities, since the initial reservation directly shapes the first allocation of each world.
- `maximum_hierarchy_depth` bounds transform recomputation recursion; keep it at the depth the scene graph actually uses plus headroom, not an unbounded value.
- The two world-count keys are independent because the ecs and entity-world layers keep separate registries.

## World persistence

`saveWorld` writes format version 5. `loadWorld` accepts versions 2 through 5 and applies fixed input bounds in addition to `entity_world.maximum_entities`: 512 MiB maximum save-file size, 256 MiB maximum slot storage, 128 MiB aggregate entity/world-name data, and 1 MiB per individual name. A save must contain exactly the declared records and no trailing bytes. The live world is replaced only after the complete file has been validated and its replacement storage has been prepared.

See [overview.md](overview.md) for the capability surface and world lifecycle.
