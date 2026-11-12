# pbr module

pbr is the engine's shared physically-based-rendering state holder: it owns the global PBR environment (directional sun, hemisphere ambient, exposure, tone mapping) that the renderer samples every frame, and it converts legacy Phong-style `models::ModelMaterial` data into the PBR `renderer::MeshMaterial` the renderer consumes. The module holds no GPU resources of its own - it is plain state plus pure conversion math, published as the `pbr` capability. The application binds it with `tryAcquireCapability<pbr::PbrCapability>()` (see `demos/character_customisation/source/main.cpp`), tunes it through `[pbr]` configuration with live reload, and can drive it at runtime through the capability entries.

## Capability

The module provides the `pbr` capability.

- ABI constant: `pbrCapabilityAbi = 5` (bump on any struct layout change)
- Header: `modules/pbr/include/engine/pbr/capabilities.hpp`
- Binding: `ParticipantContext::acquireCapability<pbr::PbrCapability>()`; every entry takes `capability.self` as its first argument.

| Entry | Purpose |
| --- | --- |
| `environment(self)` | Pointer to the current normalized environment (owned by the module). |
| `environmentVersion(self)` | Monotonic counter that bumps only when stored values change. |
| `setEnvironment(self, const PbrEnvironment*)` | Validates, normalizes, and stores; returns `false` (old state kept) for invalid input. |
| `resetEnvironment(self)` | Restores the environment built from `[pbr]` configuration. |
| `copyEnvironment(self, PbrEnvironment* out, std::uint64_t* outVersion)` | Snapshot read; either out-pointer may be null (at least one must be set). |
| `blendEnvironments(self, from, to, t, out)` | Stateless interpolation at `t` in `[0, 1]`; `false` for null pointers or out-of-range `t`. |
| `convertModelMaterial(self, source, options, out)` | Converts one legacy material; `options == nullptr` uses the module-configured options. |
| `convertModelMaterials(self, sources, count, options, outMaterials)` | Batch form; empty batch is a no-op success, stops at the first invalid material. |
| `setModelMaterialConversionOptions(self, options)` | Validates and stores module-wide conversion options. |
| `resetModelMaterialConversionOptions(self)` | Restores the configured options snapshot. |
| `copyModelMaterialConversionOptions(self, out, outVersion)` | Snapshot read of the options plus their version. |

## Provided / required / optional capabilities

From `module.toml` and the entry table at the bottom of `source/pbr.cpp`:

- **Provides:** `pbr`
- **Requires:** none
- **Optional:** none
- **Declared permissions:** none

Consumers still need the `capability.pbr` permission granted by the kernel permission system before `acquireCapability` succeeds (the prefix `capability.` is added automatically by `ParticipantContext`).

## Architecture

| File | Role |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capability, declared `config_keys` (documentation; the kernel validates only plugin manifests' keys). |
| `config.toml` | Default values for the `[pbr]` configuration section (see `configuration.md`). |
| `include/engine/pbr/capabilities.hpp` | Public capability types: ABI constant, `ToneMapping`, `SunLight`, `AmbientLight`, `PbrEnvironment`, conversion options/results, and the `PbrCapability` function-pointer struct. |
| `source/pbr.cpp` | The whole implementation: environment normalization/blending, material conversion, configuration reading and live reload, capability binding, module entry table. |

## How it works

**State model.** `PbrModule` keeps two snapshots each for the environment and the conversion options: the *live* values the capability returns, and the *configured* values built from `config.toml` that `resetEnvironment` / `resetModelMaterialConversionOptions` restore. Two monotonic version counters bump only when a comparison against the previous value shows an actual change, so consumers can poll `environmentVersion` cheaply instead of copying every frame.

**Normalization.** Every stored environment passes through `normalizeEnvironment`: the sun direction is renormalized to unit length (non-finite or near-zero input fails), negative color components are clamped to 0 (non-finite components fail), sun/ambient intensity must be non-negative, exposure must be positive, and the tone mapping enum must be in range. `setEnvironment` returns `false` and keeps the old state on failure.

**Blending.** `blendEnvironments` interpolates two normalized environments: colors and intensities lerp linearly, exposure lerps in log space (perceptually even), the blended sun direction is renormalized with a nearest-endpoint fallback if the lerp degenerates to zero, a disabled sun counts as intensity 0 (so enabled falls out of the blended value), and the tone mapping snaps at the midpoint (`t < 0.5` picks `from`).

**Material conversion.** `convertModelMaterial` maps `models::ModelMaterial` to `ConvertedModelMaterial`: optional sRGB decode of base color/emissive, per-channel scales, clamping (base color/emissive to the float range, transmittance/opacity/sheen/clearcoat/clearcoatRoughness/anisotropy to `[0, 1]`, roughness to `[minimumRoughness, 1]`, index of refraction to `[1e-4, 1e4]`), metallic from `specular` as a fallback when the source has no PBR factors and its metallic is 0 (opt-in via `useSpecularAsMetallic`), and `unlit` derived from the source's `unlit` flag or illumination model 0. Non-finite source data fails the conversion and leaves the output untouched.

**Live reload.** The module subscribes to `ConfigurationReloadedEvent` and re-reads the `[pbr]` section whenever at least one changed key starts with `pbr.`; versions bump only for real changes.

## Threading and lifetime

- The module has no internal synchronization: environment and options are plain members. `construct`, `stop`, and the configuration-reload handler run on the kernel's main thread; consumers calling `setEnvironment` from other threads must serialize themselves.
- The pointer returned by `environment()` points into the module and is invalidated by the next successful `setEnvironment`, `resetEnvironment`, or configuration reload (and by module stop). Use `copyEnvironment` for a safe snapshot together with the version.
- The module instance is created by the kernel through the entry table and lives from `construct()` to `stop()`; the `context_` pointer is dropped in `stop()` after unsubscribing from the event bus.

## Configuration

Configurable through the `[pbr]` section; every key is documented in [configuration.md](configuration.md).

## Integration

- **Events published:** none.
- **Events subscribed:** `ConfigurationReloadedEvent` (reacts only to keys starting with `pbr.`).
- **Permissions declared:** none in the entry table; acquisition of the capability is gated by the implicit `capability.pbr` permission.

Consumer example (real signatures; mirrors `demos/character_customisation/source/main.cpp`):

```cpp
#include <engine/pbr/capabilities.hpp>

auto &pbr = appContext->acquireCapability<pbr::PbrCapability>();
if (pbr.abi != pbr::pbrCapabilityAbi)
    return; // ABI mismatch: skip rather than call through a stale struct

pbr::PbrEnvironment environment{};
std::uint64_t       version = 0;
if (pbr.copyEnvironment(pbr.self, &environment, &version))
{
    // Night variant of the configured environment.
    pbr::PbrEnvironment night  = environment;
    night.sun.enabled           = false;
    night.sun.intensity         = 0.0f;
    night.ambient.intensity    *= 0.25f;

    pbr::PbrEnvironment dusk{};
    if (pbr.blendEnvironments(pbr.self, &environment, &night, 0.5f, &dusk))
        pbr.setEnvironment(pbr.self, &dusk); // false + old state kept if invalid
}
```

A renderer that wants to notice external changes keeps the last seen value of `pbr.environmentVersion(pbr.self)` and re-uploads uniforms when it moves.
