# radiance_cascades module

The global-illumination technique. The module attaches a multi-stage compute pipeline to the renderer (march, gather, compose, denoise) that transports radiance through per-cascade view-depth slabs against the exact analytic scene SDF, composes the cascades front to back, and gathers per-pixel indirect diffuse irradiance into the INDIRECT DIFFUSE radiance layer.

## How the transport is organized

- **Cascade partition.** The interval partition is measured along the VIEW ray and split into slabs that grow by `interval_factor` per cascade. The shipped 3.0 puts room-depth surfaces on probe grids two to four times finer than a doubling schedule at the same march cost — the probe STRIDE doubles per cascade while the slab grows faster.
- **Anchoring.** Cascade 0 anchors its probes on the visible surfaces (the depth buffer) and marches each direction until the ray's view depth leaves cascade 0's slab, so near-contact transport is exact. Coarser cascades anchor probes ON THE CAMERA RAY at their slab start — pulled just in front of the cell's own surface whenever that surface sits before the slab start (a continuous pull from the cell centre's view depth; the earlier 4x4 tap minimum quantized the anchor into staircase bands along iso-depth contours) — and march the slab in view depth. Anchoring every coarse cascade on unrelated surfaces once mixed incompatible fields along one view ray (problem 27), and probing from inside geometry blocked light instead of guarding it (problem 28).
- **March.** The per-direction march range is EXACTLY the ray's segment inside the slab: forward directions march until the ray's view depth reaches the slab end; BACKWARD directions (viewCosine <= 0) march only down to the camera plane. Letting backward rays run a fixed budget past the camera plane once filled every probe's field with arbitrary far geometry, and every pixel sampling those directions inherited the per-probe variance as blocky staircases (problem 38 in `docs/robustness.md`). A debug radiance marker once shipped inside the committed march stage — debug instrumentation leaves the committed path (problem 32).
- **Compose.** Front-to-back composition stays geometrically consistent: every slab starts exactly where the finer slab ended along the same view ray. The compose base uses the sky GRADIENT only, never the sun disk — a sub-pixel disk would alias into the coarse angular texels and put sparkle into every GI sample. Emissive panels are visible to coarse cascades through their direct radiance seeding (the panel's outgoing radiance is part of the transported field; problem 24).
- **Gather.** The final gather is DEPTH SELECTIVE with a SOFT selection: each pixel blends the cascade whose depth interval contains it with the next FINER one over the first 40% of the slab — both fields are valid at the boundary, so the blend is continuous across cascade seams, and leaning on the smoother coarser field suppresses the finer grid's per-probe blockiness. Starved pixels fall back CONTINUOUSLY to the best-covered of the finer cascade and cascade 0 (whose probes anchor on the surfaces right around the pixel — near contact silhouettes its coverage stays high where every coarse field starves), then to the pixel's own direct irradiance over pi; each step is line-of-sight guarded, so the no-leak property is unchanged and the fallback never draws a discontinuity ring around a contact silhouette (problems 33 and the marker/fallback family 32-34). Probe-starved pixels once read false black before the fallback chain existed (problem 33).
- **The leak guard.** The line-of-sight weighted probe blend is THE leak guard: its result feeds the probe weight directly. Both LOS endpoints are checked against the SDF, and an offset pushed inside geometry by a normal-mapped normal falls back to an unperturbed start (a sphere pixel's perturbed normal once collapsed its whole neighbourhood's coverage to zero). The guard is also a regression story: a refactor once shadowed the visibility variable and silently disabled the whole guard (problem 52), and a grazing band in the visibility kernel collapsed into black per-probe blobs (problem 50). Probe contributions renormalize by their ACTUAL surviving bilinear weight sum — a clamped floor once amplified every partially blocked pixel by up to 1.4x and drew bright rims riding every contact silhouette (problem 42). Coarse-cell anchoring is CONTINUOUS: the mixed/pure cell classification ramps smoothly between the slab-start anchor and the far-surface anchor instead of branching, because neighbouring cells classified differently carried disagreeing fields and the compose bilinear interpolated rectangular stair steps between them (problem 43).
- **Denoise.** An edge-aware bilateral filter (radius configurable, shipped at 10; the shader clamps at 16 to stay useful at probe stride 8) removes the probe grid's spatial scale before the composite. Its depth band is measured in LINEARIZED VIEW depth (the near/far planes ride the filter's push constants): a band compared against raw buffer depths is a hyperbola in world space — centimetres near the camera, metres far away — and let the filter average GI across distant parallel surfaces (problem 56). Its normal weight is power 8 because a power-32 kernel rejected nearly every tap on normal-mapped surfaces and let the blocks survive.
- **Relight.** Ray hits are relit with the surface's textured material through the shared `surfaceOutgoingRadiance` (`relight_common.glsl`) — one material truth shared with the reflection technique — plus shadows budgeted identically to the direct pass (`shadow_steps` in both; an under-budgeted relight shadow ray returns partial light inside sealed rooms and the probe grid amplifies the error into blocky patches). The relight's energy unit is LAMBERTIAN radiance and the gather integrates it into irradiance over pi, with the composite multiplying albedo exactly once (problem 41).

## Cost model

The march dispatches `(extent/stride)^2 * rays * march_steps` scene-SDF evaluations per frame, and the SDF is linear in the object count. `probe_stride` and `base_rays` dominate the cost: stride 2 at 720p launched 29 million sphere-tracing invocations per frame — multi-second frames that starved the OS message pump and got the window marked "not responding" (problem 47). The shipped stride 8 with the same ray count is a 16x cheaper march; smoothness comes from the LOS-guarded gather, the soft cascade blend and the spatial filter instead of probe density.

## Wiring

| Manifest field | Value |
|---|---|
| provided capabilities | (none — a technique, not a capability provider) |
| required capabilities | `logging`, `renderer`, `lighting` |
| required permissions | `capability.logging`, `capability.renderer`, `capability.lighting` |

The `lighting` requirement is deliberate: the technique reads the scene light state the pbr module provides. Listing the technique without a lighting provider fails resolution precisely (`docs/architecture.md`, "Startup diagnostics"). List it under `[lighting].techniques`.

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

## The no-leak guarantee, automated

`config/leak_test.toml` renders a sealed room with one emissive panel and a window opening using the GI technique ONLY (`techniques = ["radiance_cascades"]`), and `engine --leak-check` evaluates the captured frame against configured regions: the sealed region must stay dark (the transport path cannot leak), the window view must receive light (the GI path is alive). The `engine_leak_check` cTest runs it automatically — this is the automated guard that keeps every fix above fixed (`docs/building.md`, "Testing"; `host/README.md`, exit code 4).

## Files

- `radiance_cascades.hpp` / `radiance_cascades.cpp` — the technique module and its pass attachment
- `shaders/rc_march.comp` / `rc_gather.comp` / `rc_compose.comp` / `rc_denoise.comp` — the four pipeline stages
- `CMakeLists.txt` — target and option wiring (`ENGINE_BUILD_MODULE_RADIANCE_CASCADES`)

## Tests

Validated by the runtime tiers: `engine_leak_check` (no-leak + alive-GI invariants on the sealed room), the headless smoke (full technique stack, validation layers on), and the QA pixel harness (`scripts/qa/`) that drove most of the fixes cited above.
