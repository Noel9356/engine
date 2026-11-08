# direct_lighting configuration

All keys live in the `[direct_lighting]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. The `config_keys` list in `module.toml` documents the section's keys, but the current kernel does not validate module sections against it: known-key validation (unknown-key warnings) covers the engine's built-in sections and the `config_keys` of game-mod manifests only, so unknown `[direct_lighting]` keys are silently accepted.

Every value is read once in `construct()`; the module does not follow configuration reloads. Keys are listed alphabetically.

| Key | Type | Valid range (validated in `construct()`) | Default (`config.toml`) | Effect |
| --- | --- | --- | --- | --- |
| `direct_lighting.max_lights` | integer | `>= 0` (negative values are ignored); values above 32 are clamped to `maxDirectLights` = 32 | `0` | Legacy explicit cap on both registry creation and `resolveProfile`. `0` uses the separate registry capacity and the automatic device budget. |
| `direct_lighting.maximum_registered_lights` | integer | 1 - 65536 | `256` | Maximum number of light descriptions retained by the module. The registry uses dynamically sized storage and O(1) handle lookup; renderers still clamp each draw to the shader/device budget. |

## Out-of-range and invalid values

A negative `max_lights` is ignored; a value above 32 is clamped to 32. An invalid `maximum_registered_lights` is ignored and leaves the default capacity of 256 active. There is no warning message from the module itself for out-of-range values.

## How the two keys interact

The renderer-facing `maxDirectLights` constant remains 32 because the current shader/uniform layout has 32 slots. `maximum_registered_lights` controls the CPU-side registry and snapshot independently: the registry keeps the full set of declared lights (enabled or not), while `copyLights` hands renderers only the active ones, priority-sorted, truncated to their per-draw budget. An explicit non-zero `max_lights` retains the older behavior by additionally limiting registry creation and the automatic device budget in `resolveProfile`; leave it at `0` to use the larger registry capacity and the per-device budgets (32/24/16/8 by device class, capped by the uniform-buffer range).

The snapshot copy protocol is version-based: renderers poll `copyLights(nullptr, 0, &version)` each frame and only copy when the version moved, so raising the registry capacity has no per-frame cost for consumers. See [overview.md](overview.md) for the registry/snapshot split and the exact budget formula.
