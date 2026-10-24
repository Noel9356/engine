# ray_reflections module

The ray-traced glossy reflection technique. The module attaches one compute pass to the renderer that traces a single SDF reflection ray per pixel where the Fresnel weight justifies one, relights the hit with the scene's textured material plus the composed diffuse GI, and writes the INDIRECT SPECULAR radiance layer the composite adds.

## The reflection pass

`shaders/ray_reflections.comp` sphere-traces the same analytic scene SDF every other consumer uses (one geometry truth — the shared parameterization in `scene_common.glsl`):

- **Launch guard.** The ray's origin is inside-test guarded like every other ray consumer: a normal-mapped normal can push launch points inside the sphere and make whole patches reflect the sky. CONVEX SELF-HITS are skipped past instead of discarding the reflection — every scene primitive is convex, so a ray leaving its surface can never legitimately return to it; the hit's owner primitive is compared against the launch point's, and self-hits restart the march just beyond themselves, because discarding a convex self-hit discards a valid reflection.
- **Hit shading.** A hit is relit with the surface's textured material through the shared `surfaceOutgoingRadiance` (`relight_common.glsl` — one material truth shared with the GI technique's relight) plus the composed diffuse GI where the hit point reprojects into the frame. The relight's shadows are budgeted identically to the direct pass.
- **Screen-space GI reuse.** The reuse window compares WORLD positions under a view-depth-scaled tolerance: clip-space depth is non-linear (hyper-strict near the camera, uselessly loose far away) and cannot carry the tolerance.
- **Miss.** A miss sees the shared sky WITH the sun disk (`environment_common.glsl`), matching what the camera itself sees for that direction.
- **Specular partition.** The output is pre-multiplied by a Fresnel factor with a `pow(1 - roughness/max, 1.5)` roughness fade: smooth surfaces hand over fully, mid-roughness surfaces fall off smoothly (a squared fade crushes a polished metal's room reflection to a fifth of its Fresnel weight). The pbr pass dims its analytic specular by the same weight, so the two layers PARTITION the specular channel instead of double-counting it — without the partition the sun draws twice on every glossy pixel.

The pass reads the scene light state through the `LightingCapability` (capability `lighting`, provided by the pbr module) so relit hits see the same sun and point lights as everything else. The acquire is guarded by `ComponentContext::isCapabilityAvailable`: without an active provider the relight sees no lights and reflections carry emissive + sky only.

## Wiring

| Manifest field | Value |
|---|---|
| provided capabilities | `indirect_specular` (this technique's radiance layer) |
| required capabilities | `logging`, `renderer` |
| optional capabilities | `lighting` — used when a provider is active, reflections degrade to emissive + sky without one |
| required permissions | `capability.logging`, `capability.renderer` (`capability.lighting` is derived from the optional declaration) |

The `lighting` acquisition is an OPTIONAL capability (declared in `optionalCapabilities`, like `screen_space_radiance_cascades`'): the relight reads the scene light state (see `modules/pbr/README.md`) when a provider exists, and reflections degrade to emissive + sky without one. The resolver orders the lighting provider before this module when one is active, and the capability never joins the demand closure. A run activates this module exactly when an active mod demands `indirect_specular`; an active module always attaches its pass.

## Configuration

The module ships these values in its own `config.toml` (the lowest configuration layer — `config/engine.toml` and staged overrides win over them, see `docs/configuration.md`):

| Key | Type | Default | Meaning |
|---|---|---|---|
| `max_distance` | float | `40.0` | reflection ray bound in world units |
| `march_steps` | integer | `64` | sphere-tracing steps per reflection ray (the pass's built-in fallback is 96; the shipped default is the interactive-run budget) |

`max_roughness` is not in this section: the hand-off point is the shared `[lighting].max_roughness` key, read by both this pass and the direct pass so the two sides of the specular partition can never drift apart.

## Files

- `ray_reflections.hpp` / `ray_reflections.cpp` — the technique module and its pass attachment
- `shaders/ray_reflections.comp` — the reflection compute pass
- `config.toml` — the module's shipped defaults (the lowest configuration layer)
- `module_plugin.cpp` — the static plugin descriptor (name, factory, provided capabilities)
- `CMakeLists.txt` — target and option wiring (`ENGINE_BUILD_MODULE_RAY_REFLECTIONS`)

The demo scene's `Metal049A` sphere is the reflection showcase: a run demanding `indirect_specular` renders it with the partition and self-hit handling above.
