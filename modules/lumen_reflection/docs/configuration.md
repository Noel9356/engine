# lumen_reflection configuration

Every key of the `[lumen_reflection]` configuration section, in TOML path form. All keys are optional; missing, out-of-range, or non-finite values fall back to the defaults below. Keys reload live through `ConfigurationReloadedEvent`: budget and quality keys apply on the next `evaluate()`, while capacity keys rebuild the affected structures (a resolution change resets the temporal history). Defaults follow Lumen's published reflection settings scaled for CPU evaluation.

## Core

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `lumen_reflection.enabled` | bool | `true` | Master switch: `false` makes `evaluate` a no-op that keeps returning true; the published field keeps its last content and version, so a renderer polling it stops copying. |
| `lumen_reflection.skip_unchanged_frames` | bool | `true` | Static-frame skip: when the camera, the scene, the environment, and the lumen_gi probe-grid version are all unchanged and the temporal history has had the blend's convergence window to settle (see `temporal.strength`), `evaluate` returns early instead of re-tracing the same rays. The exported field version stops changing; renderers polling it skip their upload too. |
| `lumen_reflection.quality` | 0..4 | `3` | Quality preset like the render graph's device resolution: 0 Auto (the host's logical processor count decides - at least 8 processors runs 1x, otherwise 0.5x), 1 Low (0.25x), 2 Medium (0.5x), 3 High (1x), 4 Ultra (2x). The preset scales the linear field resolution (`screen_scale` inversely) and the ray budget; explicit keys below still cap the result. |

## Scene

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `lumen_reflection.scene.max_surfaces` | 1..65536 | `256` | Registry capacity; creating beyond it fails with `CapacityReached`. |
| `lumen_reflection.scene.max_triangles_per_surface` | 3..100000000 | `200000` | Per-surface triangle cap; larger descriptions fail with `InvalidDescription`. |

## Field resolution and rays

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `lumen_reflection.screen_scale` | 1..64 | `2` | Field resolution divisor per axis: the field covers 1/screen_scale of the viewport's width and height. The default (half of the viewport per axis) keeps curved reflectors smooth at typical viewport sizes while staying well below full resolution, matching Lumen's sub-resolution reflection tracing; coarser divisors trade texel detail for CPU cost. |
| `lumen_reflection.rays_per_pixel` | 1..64 | `1` | Stochastic rays per field texel per frame; the temporal accumulation is the effective sample multiplier (a converged frame is hundreds of samples). |
| `lumen_reflection.mirror_band_taps` | 1..16 | `4` | Sub-texel-jittered taps per texel for the near-mirror band (roughness below `roughness.ray_band_max`, transmissive texels included). A mirror surface's traced content is high-frequency at field resolution, so one texel-center sample aliases bright/dark edges into single-texel outliers; the jittered taps integrate the texel's world footprint (estimated from the GBuffer's neighboring surface positions) and the temporal accumulation converges the per-frame rejittered draws into a smooth response. `1` disables the supersampling. |
| `lumen_reflection.screen_ray_distance_m` | 0.05..64 | `2.0` | Screen-trace length: screen traces resolve only what the frame GBuffer covers within this radius; longer rays hand the remainder to the scene trace (Lumen's short screen-ray budget). |

## Screen traces

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `lumen_reflection.screen_trace.enabled` | bool | `true` | Screen-trace tier toggle (Lumen's Screen Traces switch): `false` sends every dedicated ray straight to the scene trace, skipping the GBuffer march. |
| `lumen_reflection.screen_trace.max_steps` | 4..256 | `48` | Marching step budget of one screen trace; longer on-screen extents trade resolution for distance coverage. |
| `lumen_reflection.screen_trace.thickness_m` | 0.01..8 | `0.5` | Thin-occluder thickness: a ray accepts the texel it passes behind when it exceeds the stored camera distance by at most this many meters. |

## Roughness bands

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `lumen_reflection.roughness.ray_band_max` | 0..1 | `0.3` | Dedicated reflection rays are traced for roughness below this value (Lumen's 0.3 dedicated band). |
| `lumen_reflection.roughness.max_trace` | 0..1 | `0.4` | Roughness ceiling for any specular environment response (Lumen's Max Roughness To Trace). Between `ray_band_max` and `max_trace` the response is resampled from the lumen_gi world radiance cache instead of tracing rays; above it the field carries nothing. |

## Trace

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `lumen_reflection.trace.max_distance_m` | 1..2000 | `50.0` | Scene-trace length for reflection rays (the same ceiling as lumen_gi's trace budget). |
| `lumen_reflection.trace.bias_m` | 0.001..0.5 | `0.02` | Surface offset for reflection rays and their shadow rays; one bias behind every trace site, the same acne/leak knob as lumen_gi's `trace.bias_m`. |
| `lumen_reflection.trace.max_transmission_segments` | 0..16 | `3` | How many interfaces a ray chain may cross through transparent or translucent surfaces (screen pass-throughs and refracted hops both count). A chain that exhausts the budget is absorbed; a glass pane costs two segments (entering and exiting the medium). |

## Temporal filter

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `lumen_reflection.temporal.strength` | 0..1 | `0.99` | Cap on the temporal history weight: the blend follows the running-mean ramp N/(N+1) over the texel's accumulated history length and never exceeds this value, so a static stochastic field converges like an accumulated average (1.0 runs the unbounded running mean; the neighborhood clamp and the change detection below keep a long history honest). The history restarts on disocclusion and when a whole-neighborhood move marks the texel changed, so relit content re-converges instead of ghosting. The static-frame skip's convergence threshold derives from this value: the ramp reaches the cap at `ceil(strength / (1 - strength))` clean frames (a 1.0 running mean needs about a hundred, where its per-frame share 1/(N+1) crosses 1%), so a frozen field is always a converged one. |
| `lumen_reflection.temporal.neighborhood_clamp_sigma` | 0..16 | `1.0` | Neighborhood clamp width: the history is clamped to mean +/- sigma * value of the current 3x3 neighborhood before blending, so a changed scene cannot ghost through the accumulation. |

## Spatial reuse and bilateral filter

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `lumen_reflection.filter.spatial_reuse_radius` | 0..9 | `1` | BRDF-weighted spatial reuse radius in field texels (radius 1 = 3x3). Neighbors are gated by depth-plane, normal, and roughness similarity. 0 disables the pass. |
| `lumen_reflection.filter.variance_threshold` | 0..4 | `0.02` | Luminance variance threshold above which the bilateral filter smooths a texel; the spatial reuse pass estimates the true variance of the texel and its accepted neighbors. |
| `lumen_reflection.filter.bilateral` | bool | `true` | Variance-driven bilateral filter (Lumen's last-ditch pass): runs only where the spatial reuse left high variance, at double radius on freshly revealed texels, with tone-mapped weights that bound a firefly's share of the average. |
| `lumen_reflection.filter.bilateral_radius` | 1..9 | `2` | Bilateral filter radius in field texels. |

## Debug

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `lumen_reflection.debug.reference_samples` | 1..4096 | `16` | Brute-force rays per texel for the Reference debug view. All views render at field resolution (the module's working resolution - exactly what the renderer composites, so an A/B comparison against the reference involves no resampling). |
