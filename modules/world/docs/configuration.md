# world configuration

The world module reads two sections: `[ecs]` (consumed by the embedded ecs module, `source/ecs.cpp` `construct()`) and `[entity_world]` (consumed by `source/entity_world.cpp` `construct()`). Defaults ship in `config.toml`; the application config (or a profile) may override them. The keys are mirrored in `module.toml` `config_keys` for documentation; the kernel validates known keys for engine sections and game-mod manifests only. Every value is read once in `construct()`; nothing is re-read later (the module does not subscribe to `ConfigurationReloadedEvent`).

| Key | Type | Valid range (validated in `construct()`) | Default (`config.toml`) | Effect |
| --- | --- | --- | --- | --- |
| `ecs.maximum_worlds` | integer | 1 – 4096 | `64` | Maximum number of concurrently existing ecs worlds; `createWorld` beyond the limit fails with `CapacityExceeded`. |
| `entity_world.initial_entity_capacity` | integer | 1 – 1000000 | `256` | Default slot-vector reservation for a new entity world, used when its `EntityWorldDescription::initialEntityCapacity` is 0 (also clamped to the world's resolved entity maximum). |
| `entity_world.maximum_entities` | integer | 1 – 10000000 | `1000000` | Cap applied to each world's entity maximum: `createWorld` resolves `min(description maximum, this value)` (and uses this value when the description says 0). Also enforced by `createEntity` (`CapacityExceeded`) and by `loadWorld`'s slot-count check. |
| `entity_world.maximum_hierarchy_depth` | integer | 1 – 4096 | `256` | Cap applied to each world's hierarchy depth (same `min(description, this)` rule). Enforced by parent validation, world-transform recomputation, and `loadWorld`. |
| `entity_world.maximum_worlds` | integer | 1 – 65536 | `1024` | Maximum number of concurrently existing entity worlds; `createWorld` beyond the limit fails with `CapacityExceeded`. |

Out-of-range and invalid values: a value outside its documented range (or a wrong TOML type) is silently ignored and the built-in default listed above stays active — unlike the window module, the world module prints no diagnostic. Because the limits act as caps on per-world description values, a description asking for more than the configured maximum simply gets clamped down rather than failing.
