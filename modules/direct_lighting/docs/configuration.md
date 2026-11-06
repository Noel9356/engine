# direct_lighting configuration

All keys live in the `[direct_lighting]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. The `config_keys` list in `module.toml` documents the section's keys, but the current kernel does not validate module sections against it: known-key validation (unknown-key warnings) covers the engine's built-in sections and the `config_keys` of game-mod manifests only, so unknown `[direct_lighting]` keys are silently accepted.

Every value is read once in `construct()`. A value outside its documented range is ignored and the built-in default stays active.

| Key | Type | Valid range (validated in `construct()`) | Default (`config.toml`) | Effect |
| --- | --- | --- | --- | --- |
| `direct_lighting.max_lights` | integer | `>= 0` (negative values are ignored); values above 32 are clamped to `maxDirectLights` = 32 | `0` | Upper bound on simultaneously existing lights enforced by `createLight` (excess creation fails with `CapacityReached`). `0` means "automatic": the hard maximum of 32 for `createLight`, and the device-dependent budget from `resolveProfile` (8, or 16/24/32 by Vulkan device class, further capped by the uniform-buffer budget). |

Out-of-range / invalid values: a negative `max_lights` is not assigned, so the built-in default `0` (automatic) stays active; a value above 32 is clamped down to 32 rather than rejected. There is no warning message from the module itself for out-of-range values.

`module.toml` `config_keys` mirrors the keys above for documentation; unknown `direct_lighting.*` keys are silently accepted by the kernel.

Note that the configured value caps the *registry* (`createLight` uses `max_lights == 0 ? 32 : max_lights`) and also acts as an upper bound on the `DirectLightingProfile::maxLights` that `resolveProfile` reports — an explicit value can lower the automatic budget, but never exceed the hardware-derived limit.
