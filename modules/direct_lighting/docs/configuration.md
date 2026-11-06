# direct_lighting configuration

All keys live in the `[direct_lighting]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. The `config_keys` list in `module.toml` documents the section's keys, but the current kernel does not validate module sections against it: known-key validation (unknown-key warnings) covers the engine's built-in sections and the `config_keys` of game-mod manifests only, so unknown `[direct_lighting]` keys are silently accepted.

Every value is read once in `construct()`. A value outside its documented range is ignored and the built-in default stays active.

| Key | Type | Valid range (validated in `construct()`) | Default (`config.toml`) | Effect |
| --- | --- | --- | --- | --- |
| `direct_lighting.max_lights` | integer | `>= 0` (negative values are ignored); values above 32 are clamped to `maxDirectLights` = 32 | `0` | Legacy explicit cap on both registry creation and `resolveProfile`. `0` uses the separate registry capacity and the automatic device budget. |
| `direct_lighting.maximum_registered_lights` | integer | 1..65536 | `256` | Maximum number of light descriptions retained by the module. The registry uses dynamically sized storage and O(1) handle lookup; renderers still clamp each draw to the shader/device budget. |

Out-of-range / invalid values: a negative `max_lights` is ignored; a value above 32 is clamped to 32. An invalid `maximum_registered_lights` is ignored and leaves the default capacity of 256 active. There is no warning message from the module itself for out-of-range values.

`module.toml` `config_keys` mirrors the keys above for documentation; unknown `direct_lighting.*` keys are silently accepted by the kernel.

The renderer-facing `maxDirectLights` constant remains 32 because the current shader/uniform layout has 32 slots. `maximum_registered_lights` controls the CPU-side registry and snapshot independently. An explicit non-zero `max_lights` retains the previous behavior by also limiting creation; leave it at `0` to use the larger registry capacity.
