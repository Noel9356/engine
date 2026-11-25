# radiance_cache configuration

Every key of the `[radiance_cache]` configuration section, in TOML path form. All keys are optional; missing, out-of-range, or non-finite values fall back to the defaults below. Keys reload live through `ConfigurationReloadedEvent`: budget and quality keys apply on the next `evaluate()`, while capacity keys (probe spacing/resolution, debug resolution, surface limits) rebuild the affected structures. Defaults follow Lumen's published budgets scaled for CPU evaluation.

## Core

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `enabled` | bool | `true` | When `false`, `evaluate()` is a no-op and the probe grid stops updating. |
| `skip_unchanged_frames` | bool | `true` | Static-frame skip: when the camera (`setView`), the scene (surface mutators), and the environment (pbr / direct-lighting versions) are all unchanged and every cache has had enough updates to settle (derived from the budgets, the number of cached texels, and the probe refresh cycle), `evaluate()` returns early without touching the caches, and the exported probe version stops changing (renderers polling `copyProbes` skip their per-frame snapshot upload too). Any change re-enables evaluation immediately. |
| `quality` | 0-4 | `3` (High) | Preset scaling of the four amortization budgets below (Auto 0.5x/1x from the host's logical processor count, Low 0.25x, Medium 0.5x, High 1x, Ultra 2x), mirroring the render graph's quality resolution: Auto runs 1x on hosts with at least 8 logical processors and 0.5x below that. Explicit budget keys are scaled by the same factor. |

## Scene

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `scene.max_surfaces` | 1-65536 | `256` | Registry capacity; `createSurface` beyond it reports `CapacityReached`. |
| `scene.max_triangles_per_surface` | 3-1e8 | `200000` | `createSurface`/`updateSurface` reject larger geometries with `InvalidDescription`. |

## Surface cache (cards)

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `cards.per_surface_max` | 1-64 | `12` | Maximum cards per surface (Lumen's "Max Lumen Mesh Cards" default). Cards are local-space face projections of the surface bounds; when a surface would need more cards (large faces split at `max_texels_per_card_axis`), the texel density is lowered until everything fits so every face stays covered. |
| `cards.texels_per_meter` | 1-256 | `24.0` | World-space capture density: a card of extent w x h meters captures ceil(w x density) x ceil(h x density) texels. |
| `cards.max_texels_per_card_axis` | 4-512 | `128` | Faces longer than this along an axis are subdivided (Lumen's 128-texel physical page size). |
| `cards.depth_layers` | 1-4 | `2` | Front-facing hits kept per card texel (nearest first). More than one layer lets a concave mesh, e.g. a room modelled as one mesh, keep faces hidden behind a nearer one; `1` is the old single-layer capture. Memory and lighting cost scale with it. |
| `cards.max_texels_per_surface` | 4096-8388608 | `524288` | Total layered texels one surface's card set may hold (the depth layers multiply every texel). A mesh whose cards would exceed the budget has its texel density lowered until it fits (the same lever the card-count rule uses), so one registration cannot claim an unbounded card-set footprint. |

## Capture / lighting budgets (texels per frame)

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `capture.texels_per_frame` | 4096-16777216 | `262144` | Card capture budget (Lumen's fixed 512x512 per-frame capture budget). Capture is incremental and runs only for cards that are not captured yet (new or changed surfaces), nearest to the camera first; a static scene captures once. |
| `lighting.direct_texels_per_frame` | 1024-16777216 | `65536` | Direct-lighting evaluation budget (Lumen: 1024x1024), in layered card texels. Each card has a rotating cursor, so every texel is refreshed once per cycle even for cards larger than the budget; untouched texels keep their cached radiance. |
| `lighting.indirect_texels_per_frame` | 1024-16777216 | `65536` | Indirect gather budget (Lumen: 512x512), same cursor scheme. Each update propagates one more bounce through the feedback loop. |
| `lighting.indirect_rays_per_texel` | 1-64 | `1` | Stratified cosine rays per refreshed texel (previously tied to `gather.rays_per_probe`). One ray per update keeps the per-pass cost at one trace per refreshed texel; the rotating strata and the history below accumulate the samples across updates, so a texel converges on the same expected radiance with the noise filtered by the temporal blend instead of by ray count. |
| `lighting.sun_shadows` | bool | `true` | Shadow rays for the sun's direct lighting; off makes card lighting cheaper but indirect shadows disappear. |
| `lighting.point_light_shadows` | bool | `false` | Shadow rays for point/spot (and directional) direct lights in card lighting, the final-image view and the reference. Off by default like the renderer's light loop. |
| `lighting.update_speed` | 0.05-16 | `1.0` | Scales how fast the cached lighting refreshes relative to the budgets (Lumen's "Lumen Scene Lighting Update Speed"); multiplies `lighting.direct_texels_per_frame` and `lighting.indirect_texels_per_frame` (on top of `quality`). |
| `lighting.indirect_temporal_blend` | 0-0.99 | `0.85` | Weight of the cached history when a texel is refreshed: 0 replaces it with the fresh gather (noisy), values near the cap almost never change it. The first update of a texel takes the fresh sample whole (history ramp-up), then the weight settles at this value. Raised together with the one-ray-per-refresh gather so the sparser fresh samples are smoothed by the accumulation instead of showing as noise. |

## Caustics

Photon-differential caustics: light that transmissive surfaces focus onto the receivers (the pattern a glass sphere throws onto the floor, or sunlit water onto its bed) is traced as photons from the light sources, refracted with the same Snell / total-internal-reflection rules every other ray of the module uses, and deposited into the receivers' card texels as caustic irradiance. The deposits live in the surface cache, so the indirect gathers, the world radiance cache probes, and the screen-space final gather all carry them; a transmitter's transmitted share is counted exactly once (the pass owns it - the direct, gather, and probe rays treat caustics transmitters as stoppers). Only transmissive surfaces that focus the light they pass become transmitters: a sphere or a cube bends its faces' rays toward different points, so photons aim at it; a plane or a thin slab passes a beam through without converging it, so the standard deterministic refraction path resolves its transmitted share and the photon budget concentrates on the shapes that actually form caustics. Every photon is traced as three rays (the center plus two auxiliary rays offset on the emission cell), so the landing triangle the three endpoints span is the emission cell's image under the actual transport and the focus factor falls out of its area; the deposited irradiance is the photon flux over its footprint, splatted with a normalized density kernel so each photon's flux is conserved whatever the radius.

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `caustics.enabled` | bool | `true` | Master switch; when `false`, no photons are traced and every surface transmits again (the direct, gather, and probe rays pass through transmissive surfaces with their transmitted share, the module's standard behavior without the pass). |
| `caustics.photons_per_frame` | 16-65536 | `1024` | Photon budget per frame (scaled by `quality` like the other amortization budgets). Every photon costs three ray chains, so the budget controls the pass's frame cost directly; the temporal blend below accumulates the estimate across frames, so even small budgets converge. Each frame re-samples the emission with fresh jitter (a frame in which no photon lands on a texel is a legitimate Monte Carlo zero draw, averaged in by the blend). |
| `caustics.temporal_blend` | 0-0.99 | `0.9` | Weight of the previous estimate when a texel receives a fresh deposit; untouched texels decay toward the current (usually zero) estimate by the same weight, so a caustic whose source moved away fades instead of ghosting. The first deposit of a texel lands whole (history ramp). |
| `caustics.footprint_min_m` | 0.001-0.25 | `0.01` | Footprint radius floor: the smallest photon splat radius, bounding the estimator's peak so a near-perfect focus cannot deposit its whole flux into one texel. |
| `caustics.footprint_max_m` | 0.05-8 | `0.75` | Footprint radius ceiling: divergent auxiliary chains (edges, total-internal-reflection disagreement) blur at this radius instead of smearing arbitrarily. |
| `caustics.emissive_sources` | bool | `true` | Emit caustic photons from emissive card texels (a rotating stratum over the emitter area, aimed at the transmitters within the emitting hemisphere). Interior scenes lit only by emissive panels get their caustics this way; the sun and the registry's directional, point, and spot lights cover the rest. |

## Tracing

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `trace.max_distance_m` | 1-2000 | `50.0` | Maximum ray distance for shadow, gather, and probe rays (Lumen's "Max Trace Distance"; too-small values leak light into large enclosed areas). |
| `trace.max_transmission_segments` | 0-16 | `3` | How many interfaces a gather, probe, or shadow ray may cross through transparent or translucent surfaces. Rays past the budget treat the next opaque surface as blocking; a glass pane costs two crossings (entering and exiting the medium). |
| `trace.bias_m` | 0.001-0.5 | `0.02` | Surface offset for shadow, gather, and probe rays. One bias behind every trace site: sun shadow rays scale it by the grazing factor, direct-light and card-gather rays offset along the normal, screen-gather rays offset the GBuffer surface point along its normal before tracing, world-probe rays along each ray's own direction (so a probe sitting exactly on a surface plane does not self-intersect it), and point-light shadow rays are skipped entirely for lights within three biases of the receiver, where the shadow ray would immediately re-hit it. |

## World-space radiance cache

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `probes.spacing_m` | 0.25-32 | `2.0` | Level-0 probe lattice spacing; level 1 uses `spacing x level1_spacing_factor`. Each level is a camera-centered 9x9x9 clip box, so the lattice holds a fixed 729 probes per level regardless of this value: a finer spacing trades coverage extent (and, under a moving camera, probe churn as the box scrolls) for field resolution, at no change in per-frame cost. Probes whose lattice cell lands on or inside geometry relocate their trace origin off the surface (DDGI rules, per-axis clamp at half the spacing); the exported lattice stays regular. |
| `probes.direction_resolution` | 4-64 | `32` | Octahedral radiance map resolution per world probe (Lumen uses 32x32 for world probes). Exported through `GiProbeHeader`. |
| `probes.updates_per_frame` | 1-1458 | `64` | Probes re-traced per frame per level (round-robin; Lumen amortizes probe updates the same way); probes that scrolled into the clip box are updated first. |
| `probes.rays_per_update` | 1-4096 | `256` | Rays traced per probe update. The direction map is refreshed in rotating strata (`direction_resolution^2 / rays_per_update` updates per full refresh); a probe without data traces every direction once. |
| `probes.hysteresis` | 0-0.99 | `0.2` | Fraction of the previous radiance kept when a probe texel is re-traced. Higher values are more stable but adapt slower to light changes. |
| `probes.dynamic_relight_radius_m` | 0-64 | `2.0` | Dynamic relight radius: probes whose lattice position falls inside a moved, deformed, created, or destroyed surface's swept world volume, expanded by this radius, retrace from scratch (fresh probes are served first and `probes.updates_per_frame` bounds the retrace work per frame), so dynamic geometry refreshes the world radiance cache within a few frames instead of waiting out the rotating stratum cycle. 0 disables the marking. |
| `probes.relocation_margin` | 0.005-0.25 | `0.05` | Near-surface clearance a relocated probe keeps off the geometry it was pushed away from, as a fraction of the level spacing. A boundary probe steps off a surface plane by its slab thickness plus this margin, and the clear-space relax rule holds probes at exactly this distance off a face, so they never drift back across it. Lower values hug thin geometry more tightly (for probe counts that must resolve small-scale detail); higher values trade proximity for stability. |
| `probes.level1_spacing_factor` | 2-16 | `4.0` | Coarser level's spacing multiplier (level 1 covers that many times the extent - 64 m across at the 2 m default, which keeps a sunlit field inside the coarse box). Both levels scroll with the camera: crossing a lattice cell recycles only the plane of probes that enters the box. |

## Screen-space final gather (debug view)

The gather only feeds the debug views (the renderer consumes the world probe grid), so it runs on demand when a view is exported instead of on every `evaluate()`; see `debug.continuous_gather`.

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `gather.screen_scale` | 1-64 | `8` | Screen probes sit on a jittered grid at 1/screen_scale of the internal resolution per axis (Lumen's Screen Space Radiance Cache downsampling). |
| `gather.rays_per_probe` | 1-256 | `32` | Rays traced per screen probe per pass, spread over the probe's 8x8 octahedral map (Lumen's screen probe resolution). The first `min(ray budget, strata count)` rays rotate through every above-horizon texel; the map's history weight is one minus the fraction of the strata the budget covers, so the map fills in across passes when the budget cannot refresh it and every texel takes a fresh sample when it can. |
| `gather.importance_sampling` | bool | `true` | Rays beyond full strata coverage are importance-selected by the probe's previous radiance (Lumen's Screen Space Radiance Cache reuse); the coverage rays keep their uniform rotation, so the radiance hint can never starve a stratum of fresh samples. |
| `gather.screen_ray_distance_m` | 0.05-64 | `2.0` | Screen probe ray length; misses resolve from the world cache (level 0, then level 1), then the skylight - Lumen's integration rule. |
| `gather.temporal_blend` | 0-0.99 | `0.85` | Full-resolution temporal accumulation against the reprojection of the previous frame (Lumen's Final Gather Lighting Update Speed analog). The accumulation keeps the raw (unfiltered) gather as its history, so the spatial filter below is applied exactly once to whatever is displayed instead of compounding across accumulation passes. |
| `gather.probe_temporal_blend` | 0-0.99 | `0.9` | Weight of a screen probe direction texel's previous radiance when the pass retraces it: each texel's radiance becomes the running mean of its traces, so the interpolated probe field converges instead of re-showing each pass' single-sample variance. The accumulation resets when a probe's placement moves to another position or surface (Lumen's Screen Space Radiance Cache hysteresis analog). 0 traces every texel fresh each pass. |
| `gather.spatial_filter` | 0-9 | `3` | Bilateral filter radius in pixels (0 = off) guided by depth and normal; applied to the display value at the configured radius verbatim. |
| `gather.plane_depth_tolerance` | 0.01-4 | `0.35` | Depth-plane tolerance for interpolating probes "within the same plane" as the pixel. |

## Debug views

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `debug.internal_width` / `debug.internal_height` | 16-1920 / 16-1080 | `320` x `180` | Resolution of all debug exports and of the screen-space final gather. |
| `debug.reference_samples` | 1-4096 | `64` | Path-traced samples per pixel for the `Reference` ground-truth view. |
| `debug.reference_bounces` | 1-16 | `4` | Path length for the reference view. |
| `debug.continuous_gather` | bool | `false` | Run one screen-gather pass inside every `evaluate()` instead of on demand. Costs CPU every frame. |
| `debug.export_gather_frames` | 1-64 | `24` | Accumulation passes run before a `FinalImage` / `GiOnly` export (unless `continuous_gather`) so the temporal filter has converged. |
| `debug.include_ambient` | bool | `false` | Adds the pbr hemispheric ambient on top of GI in the `FinalImage` view. Off by default so `FinalImage` and `Reference` (which has no ambient) stay comparable; enable it to preview the renderer-hook look where GI replaces the ambient. |

## Device resolution

`resolveProfile` scales the four budgets for a `renderer::DeviceProfile` the same way the render graph does: Vulkan discrete 1.0x, integrated 0.5x, otherwise 0.25x (CPU devices get the 0.25x floor). The configured values are caps; the profile result is what a consumer should hand to a runtime that cannot afford the configured budgets.

## Fallback environment

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `environment.sun_direction` | 3 finite floats | `[0.45, 0.62, 0.35]` | Direction TOWARD the sun of the built-in environment the module shades with when the optional pbr capability is absent (normalized internally; any direction is legal, including below the horizon). With pbr present the pbr environment wins and these keys are unused. |
| `environment.sun_color` | 3 floats, >= 0 | `[1.0, 0.96, 0.88]` | Sun color of the fallback environment. |
| `environment.sun_intensity` | 0-1e6 | `3.0` | Sun intensity of the fallback environment. |
| `environment.sun_enabled` | bool | `true` | Whether the fallback environment carries a sun at all. |
| `environment.ambient_sky` / `environment.ambient_ground` | 3 floats, >= 0 | `[0.55, 0.65, 0.80]` / `[0.35, 0.30, 0.25]` | Hemisphere colors the fallback skylight blends between (miss rays and the no-GI fallback in consumers). |
| `environment.ambient_intensity` | 0-1e6 | `0.5` | Overall intensity of the fallback skylight. |

The defaults match the renderer's built-in environment, so a headless consumer without pbr sees the same scene as the renderer's no-pbr fallback until it overrides these keys.
