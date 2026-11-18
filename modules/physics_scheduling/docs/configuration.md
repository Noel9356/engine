# physics_scheduling configuration

All keys live in the `[physics_scheduling]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. `module.toml` mirrors every key under `config_keys` for documentation; the kernel validates known keys for engine sections and plugin manifests only, so unknown `[physics_scheduling]` keys are silently accepted.

Every value is read once in `construct()`; the module does not follow configuration reloads. Keys are listed alphabetically.

| Key | Type | Valid range / values | Default | Effect |
| --- | --- | --- | --- | --- |
| `maximum_items_per_schedule` | integer | (0, 4000000] | 65536 | Item budget per schedule; `addItem` fails with `CapacityExceeded` once a schedule reaches it. Lowering the value affects only new items. |
| `maximum_schedules` | integer | (0, 4096] | 16 | Schedule budget; `createSchedule` fails with `CapacityExceeded` once reached. |

## Out-of-range and invalid values

Out-of-range, non-positive, or missing integer keys keep the built-in default. The schedule thresholds and the demotion margin are not configuration keys: they belong to `ScheduleDescription` and are validated at `createSchedule` (strictly descending thresholds within [0, 1], margin in [0, 1)), so a schedule always starts with a coherent ladder.

## Where the settings show up

The budgets bound how many schedules and items a single module instance can carry; a consumer that needs more items per schedule than the default raises `maximum_items_per_schedule` in its application config. The per-schedule behavior (intervals, thresholds, hysteresis) is instance data set through `ScheduleDescription` and the runtime retunes, not configuration.
