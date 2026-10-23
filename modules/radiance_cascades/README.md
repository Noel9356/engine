# radiance_cascades module

The global-illumination technique. The module attaches a multi-stage compute pipeline to the renderer (march, gather, compose, denoise) that transports radiance through per-cascade view-depth slabs against the exact analytic scene SDF, composes the cascades front to back, and gathers per-pixel indirect diffuse irradiance into the INDIRECT DIFFUSE radiance layer.

## How the transport is organized

- **Cascade partition.** The interval partition is measured along the VIEW ray and split into slabs that grow by `interval_factor` per cascade. The shipped 3.0 puts room-depth surfaces on probe grids two to four times finer than a doubling schedule at the same march cost — the probe STRIDE doubles per cascade while the slab grows faster.
- **Anchoring.** Cascade 0 anchors its probes on the visible surfaces (the depth buffer) and marches each direction until the ray's view depth leaves cascade 0's slab, so near-contact transport is exact. Coarser cascades anchor probes ON THE CAMERA RAY at their slab start — pulled just in front of the cell's own surface whenever that surface sits before the slab start (a continuous pull from the cell centre's view depth; a quantized anchor would put staircase bands along iso-depth contours) — and march the slab in view depth. Anchoring every coarse cascade on unrelated surfaces mixes incompatible fields along one view ray, and probing from inside geometry blocks light instead of guarding it.
- **March.** The per-direction march range is EXACTLY the ray's segment inside the slab: forward directions march until the ray's view depth reaches the slab end; BACKWARD directions (viewCosine <= 0) march only down to the camera plane. Letting backward rays run a fixed budget past the camera plane fills every probe's field with arbitrary far geometry, and every pixel sampling those directions inherits the per-probe variance as blocky staircases. Debug instrumentation stays out of the committed march stage.
- **Compose.** Front-to-back composition stays geometrically consistent: every slab starts exactly where the finer slab ended along the same view ray. The compose base uses the sky GRADIENT only, never the sun disk — a sub-pixel disk would alias into the coarse angular texels and put sparkle into every GI sample. Emissive panels are visible to coarse cascades through their direct radiance seeding (the panel's outgoing radiance is part of the transported field).
- **Gather.** The final gather is DEPTH SELECTIVE with a SOFT selection: each pixel blends the cascade whose depth interval contains it with the next FINER one over the first 40% of the slab — both fields are valid at the boundary, so the blend is continuous across cascade seams, and leaning on the smoother coarser field suppresses the finer grid's per-probe blockiness. Starved pixels fall back CONTINUOUSLY to the best-covered of the finer cascade and cascade 0 (whose probes anchor on the surfaces right around the pixel — near contact silhouettes its coverage stays high where every coarse field starves), then to the pixel's own direct irradiance over pi; each step is line-of-sight guarded, so the no-leak property is unchanged and the fallback never draws a discontinuity ring around a contact silhouette. Without the fallback chain, probe-starved pixels read false black.
- **The leak guard.** The line-of-sight weighted probe blend is THE leak guard: its result feeds the probe weight directly. Both LOS endpoints are checked against the SDF, and an offset pushed inside geometry by a normal-mapped normal falls back to an unperturbed start (a perturbed normal can collapse a whole neighbourhood's coverage to zero). The guard is also fragile by nature — a shadowed visibility variable silently disables it, and a grazing kernel collapses into black per-probe blobs, so the guard's shape is kept simple and its input data flow direct. Probe contributions renormalize by their ACTUAL surviving bilinear weight sum — a clamped floor amplifies every partially blocked pixel and draws bright rims riding contact silhouettes. Coarse-cell anchoring is CONTINUOUS: the mixed/pure cell classification ramps smoothly between the slab-start anchor and the far-surface anchor instead of branching, because neighbouring cells classified differently carry disagreeing fields and the compose bilinear interpolates rectangular stair steps between them.
- **Denoise.** An edge-aware bilateral filter (radius configurable, shipped at 10; the shader clamps at 16 to stay useful at probe stride 8) removes the probe grid's spatial scale before the composite. Its depth band is measured in LINEARIZED VIEW depth (the near/far planes ride the filter's push constants): a band compared against raw buffer depths is a hyperbola in world space — centimetres near the camera, metres far away — and would average GI across distant parallel surfaces. Its normal weight is power 8 because a power-32 kernel rejected nearly every tap on normal-mapped surfaces and let the blocks survive.
- **Relight.** Ray hits are relit with the surface's textured material through the shared `surfaceOutgoingRadiance` (`relight_common.glsl`) — one material truth shared with the reflection technique — plus shadows budgeted identically to the direct pass (`shadow_steps` in both; an under-budgeted relight shadow ray returns partial light inside sealed rooms and the probe grid amplifies the error into blocky patches). The relight's energy unit is LAMBERTIAN radiance and the gather integrates it into irradiance over pi, with the composite multiplying albedo exactly once.

## Cost model

The march dispatches `(extent/stride)^2 * rays * march_steps` scene-SDF evaluations per frame, and the SDF is linear in the object count. `probe_stride` and `base_rays` dominate the cost: stride 2 at 720p launches 29 million sphere-tracing invocations per frame — multi-second frames that starve the OS message pump and get the window marked "not responding". The shipped stride 8 with the same ray count is a 16x cheaper march; smoothness comes from the LOS-guarded gather, the soft cascade blend and the spatial filter instead of probe density.

## Wiring

| Manifest field | Value |
|---|---|
| provided capabilities | `indirect_diffuse` (this technique's radiance layer) |
| required capabilities | `logging`, `renderer` |
| required permissions | `capability.logging`, `capability.renderer`, `capability.lighting` |

The `lighting` acquisition is deliberate but optional: the technique reads the scene light state (see `modules/pbr/README.md`) when a provider exists, and degrades to emissive + sky transport without one. A run activates this module exactly when an active mod demands `indirect_diffuse`; an active module always attaches its pass.

## Configuration

| Key | Type | Default | Meaning |
|---|---|---|---|
| `cascades` | integer | `5` | cascade count (1..8); the last interval ends at `interval_base * interval_factor^(cascades-1)` world units |
| `interval_base` | float | `0.5` | cascade 0 interval length (near-field detail) |
| `interval_factor` | float | `3.0` | interval growth per cascade; growth above 2 puts room-depth surfaces on finer probe grids (the stride doubles per cascade while the slab grows by this factor) |
| `probe_stride` | integer | `4` | cascade 0 probes every Nth pixel; each coarser cascade doubles the stride. Dominates the cost — see the cost model above (the shipped demo uses 8) |
| `base_rays` | integer | `32` | ray count per probe (azimuth x elevation derived), identical for every cascade; this and `probe_stride` dominate the cost (the shipped demo uses 128) |
| `gather_visibility` | bool | `true` | line-of-sight weighted probe blend (the leak guard); `false` trades leaks for speed |
| `los_steps` | integer | `24` | SDF steps per gather line-of-sight march (the shipped demo uses 16) |
| `march_steps` | integer | `48` | sphere-tracing steps per probe ray |
| `relight_shadows` | bool | `true` | shadows for the first-bounce relight at ray hits |
| `shadow_steps` | integer | `16` | relight shadow march steps; keep in step with the pbr pass (`[lighting.pbr].shadow_steps`, default 24) — the shipped demo uses 24 |
| `spatial_filter` | integer | `5` | edge-aware bilateral filter radius (pixels) on the gathered GI, 0 = off (both the module and the shader clamp at 16). Size it to span at least one probe cell (`probe_stride` pixels) — a radius below the stride cannot average across the grid; the shipped `config/engine.toml` uses 10 |

## Sealing the no-leak property

The no-leak property is a design invariant, not a tuned parameter: every probe weight passes through the line-of-sight guard, every gather fallback is itself LOS-checked, and the relight shadows use the same SDF. An assembly that wants to police it adds a `[leak_test]` section (a sealed region that must stay dark, a lit region that must receive light) and runs `engine --leak-check`, which evaluates a captured frame against those regions and exits 4 on a leak or a dead GI path (`docs/building.md`, "Runtime verification without a display").

## Files

- `radiance_cascades.hpp` / `radiance_cascades.cpp` — the technique module and its pass attachment
- `shaders/rc_march.comp` / `rc_gather.comp` / `rc_compose.comp` / `rc_denoise.comp` — the four pipeline stages
- `module_plugin.cpp` — the static plugin descriptor (name, factory, provided capabilities)
- `CMakeLists.txt` — target and option wiring (`ENGINE_BUILD_MODULE_RADIANCE_CASCADES`)
