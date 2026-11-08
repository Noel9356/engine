# lumen_gi configuration

Every key of the `[lumen_gi]` configuration section, in TOML path form. All
keys are optional; missing, out-of-range, or non-finite values fall back to
the defaults below. Keys reload live through `ConfigurationReloadedEvent`:
budget and quality keys apply on the next `evaluate()`, while capacity keys
(probe spacing/resolution, debug resolution, surface limits) rebuild the
affected structures. Defaults follow Lumen's published budgets scaled for CPU
evaluation.

## Core

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `enabled` | bool | `true` | When `false`, `evaluate()` is a no-op and the probe grid stops updating. |
| `skip_unchanged_frames` | bool | `true` | Static-frame skip: when the camera (`setView`), the scene (surface mutators), and the environment (pbr / direct-lighting versions) are all unchanged and a full probe-update cycle has elapsed since the last change, `evaluate()` returns early without touching the caches, and the exported probe version stops changing (renderers polling `copyProbes` skip their per-frame snapshot upload too). Any change re-enables evaluation immediately. |
| `quality` | 0-4 | `3` (High) | Preset scaling of the four amortization budgets below (Low 0.25x, Medium 0.5x, High 1x, Ultra 2x), mirroring the render graph's quality resolution. Explicit budget keys are scaled by the same factor. |

## Scene

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `scene.max_surfaces` | 1-65536 | `256` | Registry capacity; `createSurface` beyond it reports `CapacityReached`. |
| `scene.max_triangles_per_surface` | 3-1e8 | `200000` | `createSurface`/`updateSurface` reject larger geometries with `InvalidDescription`. |

## Surface cache (cards)

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `cards.per_surface_max` | 1-64 | `12` | Maximum cards per surface (Lumen's "Max Lumen Mesh Cards" default). The six base face cards are placed first, then split pieces by texel count. |
| `cards.texels_per_meter` | 1-256 | `24.0` | World-space capture density: a card of extent w x h meters captures ceil(w x density) x ceil(h x density) texels. |
| `cards.max_texels_per_card_axis` | 4-512 | `128` | Faces longer than this along an axis are subdivided (Lumen's 128-texel physical page size). |

## Capture / lighting budgets (texels per frame)

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `capture.texels_per_frame` | 4096-16777216 | `262144` | Card capture budget (Lumen's fixed 512x512 per-frame capture budget). Priority: stalest first, then camera distance. |
| `lighting.direct_texels_per_frame` | 1024-16777216 | `65536` | Direct-lighting evaluation budget (Lumen: 1024x1024). Untouched texels keep their cached radiance. |
| `lighting.indirect_texels_per_frame` | 1024-16777216 | `65536` | Indirect gather budget (Lumen: 512x512). Each update propagates one more bounce through the feedback loop. |
| `lighting.sun_shadows` | bool | `true` | Shadow rays for the sun's direct lighting; off makes card lighting cheaper but indirect shadows disappear. |
| `lighting.point_light_shadows` | bool | `false` | Shadow rays for point/spot lights (reserved; the current implementation leaves them unshadowed like the renderer's light loop). |
| `lighting.update_speed` | 0.05-16 | `1.0` | Scales how fast the cached lighting refreshes relative to the budgets (Lumen's "Lumen Scene Lighting Update Speed"); not yet wired to the budget path, reserved. |
| `lighting.indirect_temporal_blend` | 0-1 | `0.8` | Blend of the fresh gather into the cached indirect radiance; 0 replaces immediately (noisy), 1 never updates. Light changes therefore propagate over frames. |

## Tracing

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `trace.max_distance_m` | 1-2000 | `50.0` | Maximum ray distance for shadow, gather, and probe rays (Lumen's "Max Trace Distance"; too-small values leak light into large enclosed areas). |

## World-space radiance cache

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `probes.spacing_m` | 0.25-32 | `2.0` | Level-0 probe lattice spacing; level 1 uses `spacing x level1_spacing_factor`. Each level is a camera-centered 9x9x9 clip box. |
| `probes.direction_resolution` | 4-64 | `32` | Octahedral radiance map resolution per world probe (Lumen uses 32x32 for world probes). Exported through `GiProbeHeader`. |
| `probes.updates_per_frame` | 1-1458 | `64` | Probes re-traced per frame per level (round-robin; Lumen amortizes probe updates the same way). Full refresh of one level takes 729 / updates frames. |
| `probes.hysteresis` | 0-0.99 | `0.2` | Fraction of the previous radiance kept when a probe is re-traced. Higher values are more stable but adapt slower to light changes. |
| `probes.level1_spacing_factor` | 2-16 | `4.0` | Coarser level's spacing multiplier. |

## Screen-space final gather (debug view)

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `gather.screen_scale` | 1-64 | `8` | Screen probes sit on a jittered grid at 1/screen_scale of the internal resolution per axis (Lumen's Screen Space Radiance Cache downsampling). |
| `gather.rays_per_probe` | 1-256 | `16` | Rays traced per screen probe per frame, stratified over the probe's 8x8 octahedral map (Lumen's screen probe resolution); the map persists and temporally accumulates. |
| `gather.importance_sampling` | bool | `true` | Half the rays are importance-selected by the probe's previous radiance (Lumen's Screen Space Radiance Cache reuse), the rest stay cosine-stratified. |
| `gather.screen_ray_distance_m` | 0.05-64 | `2.0` | Screen probe ray length; misses resolve from the world cache (level 0, then level 1), then the skylight - Lumen's integration rule. |
| `gather.temporal_blend` | 0-0.99 | `0.85` | Full-resolution temporal accumulation against the reprojection of the previous frame (Lumen's Final Gather Lighting Update Speed analog). |
| `gather.spatial_filter` | 0-9 | `3` | Bilateral filter radius in pixels (0 = off) guided by depth and normal. |
| `gather.plane_depth_tolerance` | 0.01-4 | `0.35` | Depth-plane tolerance for interpolating probes "within the same plane" as the pixel. |

## Debug views

| Key | Range | Default | Meaning |
| --- | --- | --- | --- |
| `debug.internal_width` / `debug.internal_height` | 16-1920 / 16-1080 | `320` x `180` | Resolution of all debug exports and of the screen-space final gather. |
| `debug.reference_samples` | 1-4096 | `64` | Path-traced samples per pixel for the `Reference` ground-truth view. |
| `debug.reference_bounces` | 1-16 | `4` | Path length for the reference view. |
| `debug.include_ambient` | bool | `false` | Adds the pbr hemispheric ambient on top of GI in the `FinalImage` view. Off by default so `FinalImage` and `Reference` (which has no ambient) stay comparable; enable it to preview the renderer-hook look where GI replaces the ambient. |

## Device resolution

`resolveProfile` scales the four budgets for a `renderer::DeviceProfile` the
same way the render graph does: Vulkan discrete 1.0x, integrated 0.5x,
otherwise 0.25x (CPU devices get the 0.25x floor). The configured values are
caps; the profile result is what a consumer should hand to a runtime that
cannot afford the configured budgets.
