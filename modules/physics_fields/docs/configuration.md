# physics_fields configuration

All keys live in the `[physics_fields]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. `module.toml` mirrors every key under `config_keys` for documentation; the kernel validates known keys for engine sections and plugin manifests only, so unknown `[physics_fields]` keys are silently accepted.

Every value is read once in `construct()`; the module does not follow configuration reloads. Keys are listed alphabetically.

| Key | Type | Valid range / values | Default | Effect |
| --- | --- | --- | --- | --- |
| `physics_fields.air_density` | number | (0, 100] | `1.225` | Air density in kilograms per cubic meter, the `density` factor of the quadratic wind drag law `0.5 x density x dragCoefficient x area x relativeSpeed x wind_drag_scale` applied by `applyWindDrag`. |
| `physics_fields.maximum_force_volumes` | integer | 1 - 4096 | `256` | Maximum concurrently existing force volumes; `createForceVolume` reports `CapacityExceeded` beyond it. |
| `physics_fields.maximum_wind_fields` | integer | 1 - 4096 | `64` | Maximum concurrently existing wind fields; `createWindField` reports `CapacityExceeded` beyond it. |
| `physics_fields.wind_drag_scale` | number | [0, 100] | `1.0` | Global multiplier on the wind drag force; `0` turns body drag off while `sampleWindVelocity` keeps sampling the field for effects. |

## Out-of-range and invalid values

Out-of-range numeric values, missing values, and non-finite values are ignored and the built-in default stays active. Budgets are enforced when fields are created: `createWindField` reports `CapacityExceeded` beyond `maximum_wind_fields` and `createForceVolume` beyond `maximum_force_volumes`. Lowering a configured limit affects new fields; it does not evict existing ones. `stop()` releases all wind fields and force volumes, resets the handle counters, and zeroes the statistics.

## Where the settings show up

- `air_density` and `wind_drag_scale` enter the per-body drag scale inside `applyWindDrag` directly, so they retune the whole wind-to-force path without touching any wind description; the default `1.225` matches sea-level air density.
- The budget keys bound the record vectors and handle maps, and the live counts surface in `FieldsStatistics` (`windFieldCount`, `forceVolumeCount`).
- The apply entry points are the per-fixed-update cost: each call enumerates the world's bodies through the rigid capability (body count, handle copy, per-body state and description copies), so the budgets double as an upper bound on that enumeration; see [overview.md](overview.md) for the application path.
