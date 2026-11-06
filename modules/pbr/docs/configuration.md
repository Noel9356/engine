# pbr configuration

All keys live in the `[pbr]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. The `config_keys` list in `module.toml` documents the section's keys, but the current kernel does not validate module sections against it: known-key validation (unknown-key warnings) covers the engine's built-in sections and the `config_keys` of game-mod manifests only, so unknown `[pbr]` keys are silently accepted.

Every value is read in `construct()` and re-read on `ConfigurationReloadedEvent` whenever a changed key starts with `pbr.`. A value outside its documented range is ignored and the built-in default stays active (ranges below are exactly what `readConfiguration()` in `source/pbr.cpp` accepts; `FLT_MAX` is `std::numeric_limits<float>::max()`).

| Key | Type | Valid range | Default | Effect |
| --- | --- | --- | --- | --- |
| `sun_direction` | number array | >= 3 finite components; vector length > 1e-8, otherwise the key is ignored | `[0.45, 0.62, 0.35]` | Sun direction; normalized to unit length before storage. |
| `sun_color` | number array | >= 3 components; finite components are clamped to >= 0 (non-finite components keep the default) | `[1.0, 0.96, 0.88]` | Sun color (linear RGB). |
| `sun_intensity` | number | finite, 0 <= v <= FLT_MAX | `3.0` | Sun intensity multiplier. |
| `sun_enabled` | boolean | true / false | `true` | Enables the sun contribution. |
| `ambient_sky_color` | number array | >= 3 components; finite components clamped to >= 0 | `[0.55, 0.65, 0.80]` | Hemisphere sky color. |
| `ambient_ground_color` | number array | >= 3 components; finite components clamped to >= 0 | `[0.35, 0.30, 0.25]` | Hemisphere ground color. |
| `ambient_mode` | string | `"uniform"` or `"hemisphere"` | `"hemisphere"` | `uniform` folds `ambient_color` into both sky and ground; any other value prints a warning on stderr and `"hemisphere"` is used. |
| `ambient_color` | number array | >= 3 components; finite components clamped to >= 0 | `[0.45, 0.48, 0.52]` | Only read when `ambient_mode = "uniform"`; if missing/short, the sky color is copied to the ground color instead. |
| `ambient_intensity` | number | finite, 0 <= v <= FLT_MAX | `0.5` | Ambient intensity multiplier. |
| `exposure` | number | finite, v > 0, <= FLT_MAX | `1.0` | Exposure applied before tone mapping. |
| `tone_mapping` | string | `"none"`, `"reinhard"`, `"aces"` | `"aces"` | Tone mapping operator; any other value prints a warning on stderr and `"aces"` is kept. |
| `model_base_color_scale` | number array | >= 3 components, each finite and >= 0 | `[1.0, 1.0, 1.0]` | Multiplies base color (after optional sRGB decode); components that fail keep the default. |
| `model_emissive_scale` | number | finite, >= 0 | `1.0` | Multiplies emissive color. |
| `model_metallic_scale` | number | finite, >= 0 | `1.0` | Multiplies metallic before the `[0, 1]` clamp. |
| `model_roughness_scale` | number | finite, >= 0 | `1.0` | Multiplies roughness before clamping to `[model_minimum_roughness, 1]`. |
| `model_minimum_roughness` | number | finite, 0 <= v <= 1 | `0.045` | Floor for the converted roughness. |
| `model_use_specular_as_metallic` | boolean | true / false | `false` | Derive metallic from `max(specular)` when the source material has no PBR factors and its metallic is 0. |
| `model_decode_base_color_srgb` | boolean | true / false | `false` | Decode source base color from sRGB to linear before scaling. |
| `model_decode_emissive_srgb` | boolean | true / false | `false` | Decode source emissive from sRGB to linear before scaling. |

## Out-of-range and invalid values

Handling is per key: an absent, mistyped, or out-of-range value is skipped and that key's built-in default stays active - the other keys still apply. For arrays, a vector shorter than 3 (or with a non-finite component, in the `sun_color`/ambient-color cases) keeps the default for the affected components. After all keys are read, the composed `PbrEnvironment` is validated as a whole; if that fails, the entire environment (not just one key) falls back to the built-in `PbrEnvironment{}` defaults. Conversion options are validated again by `validConversionOptions` at conversion time; options assembled from valid keys always pass.

`module.toml` `config_keys` mirrors the keys above for documentation; unknown `[pbr]` keys are silently accepted by the kernel.
