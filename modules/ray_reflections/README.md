# ray_reflections module

The ray-traced glossy reflection technique. The module attaches one compute pass to the renderer that traces a single SDF reflection ray per pixel where the Fresnel weight justifies one, relights the hit with the scene's textured material plus the composed diffuse GI, and writes the INDIRECT SPECULAR radiance layer the composite adds.

## The reflection pass

`shaders/ray_reflections.comp` sphere-traces the same analytic scene SDF every other consumer uses (one geometry truth — the shared parameterization in `scene_common.glsl`):

- **Launch guard.** The ray's origin is inside-test guarded like every other ray consumer: a normal-mapped normal once pushed launch points inside the sphere and made whole patches reflect the sky. CONVEX SELF-HITS are skipped past instead of discarding the reflection — every scene primitive is convex, so a ray leaving its surface can never legitimately return to it; the hit's owner primitive is compared against the launch point's, and self-hits restart the march just beyond themselves. This removed the dark speckle ring riding the mirror ball's rim (problem 57 in `docs/robustness.md`; the earlier fixed guard that discarded grazing hits outright is problem 49).
- **Hit shading.** A hit is relit with the surface's textured material through the shared `surfaceOutgoingRadiance` (`relight_common.glsl` — one material truth shared with the GI technique's relight) plus the composed diffuse GI where the hit point reprojects into the frame. The relight's shadows are budgeted identically to the direct pass.
- **Screen-space GI reuse.** The reuse window compares WORLD positions under a view-depth-scaled tolerance — the raw clip-depth comparison it replaced is non-linear (hyper-strict near the camera, uselessly loose far away) and silently dropped most reuse windows.
- **Miss.** A miss sees the shared sky WITH the sun disk (`environment_common.glsl`), matching what the camera itself sees for that direction.
- **Specular partition.** The output is pre-multiplied by a Fresnel factor with a `pow(1 - roughness/max, 1.5)` roughness fade: smooth surfaces hand over fully, mid-roughness surfaces fall off smoothly (the earlier squared fade crushed a polished metal's room reflection to a fifth of its Fresnel weight). The pbr pass dims its analytic specular by the same weight, so the two layers PARTITION the specular channel instead of double-counting it — the sun appeared twice on every glossy pixel until the partition (problem 46 in `docs/robustness.md`).

The pass reads the scene light state through the `LightingCapability` (capability `lighting`, provided by the pbr module) so relit hits see the same sun and point lights as everything else; a missing declaration once silently lost its lights to the permission system (problem 48 — capability acquisition without the declared grant throws, and the pass had worked around the throw by dropping lights).

## Wiring

| Manifest field | Value |
|---|---|
| provided capabilities | (none — a technique, not a capability provider) |
| required capabilities | `logging`, `renderer`, `lighting` |
| required permissions | `capability.logging`, `capability.renderer`, `capability.lighting` |

The `lighting` requirement is deliberate: the relight reads the scene light state the pbr module provides. List the technique under `[lighting].techniques`.

## Configuration

| Key | Type | Default | Meaning |
|---|---|---|---|
| `max_roughness` | float | `0.35` | surfaces rougher than this fade to no reflection (a single sharp ray cannot carry a wide glossy lobe). When this technique is listed, the pbr pass reads the same key and hands its analytic specular over by the same Fresnel weight, so both layers partition the specular energy (problem 46) |
| `max_distance` | float | `40.0` | reflection ray bound in world units |
| `march_steps` | integer | `96` | sphere-tracing steps per reflection ray |

## Files

- `ray_reflections.hpp` / `ray_reflections.cpp` — the technique module and its pass attachment
- `shaders/ray_reflections.comp` — the reflection compute pass
- `CMakeLists.txt` — target and option wiring (`ENGINE_BUILD_MODULE_RAY_REFLECTIONS`)

## Tests

Validated by the runtime tiers: the headless smoke renders the full technique stack (the demo scene's `Metal049A` sphere is the reflection showcase), and the QA pixel harness (`scripts/qa/`) pinned the self-hit and partition fixes.
