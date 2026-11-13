# lumen_gi module

lumen_gi implements the engine's dynamic global illumination following the architecture of Unreal Engine 5's Lumen: a card-based **surface cache** of the scene, amortized direct and indirect lighting evaluated on the cached surfaces, a camera-centered **world-space radiance cache** of octahedral probes, and a **screen-space final gather** with temporal accumulation. The implementation is CPU-side (task-scheduler parallel) and traceable end to end; the renderer-facing result is the world probe grid exposed through `copyProbes`.

## What Lumen actually does (and what this module adapts)

Lumen splits dynamic GI into: (1) a **surface cache** - an automatic parameterization of the nearby scene's surfaces into per-mesh "cards" (axis-aligned ortho projections) that store material properties and are relit every frame; (2) a **hybrid ray tracing pipeline** (screen traces first, then software distance-field or hardware ray tracing, skylight on miss) whose ray hits are shaded by looking up the surface cache; (3) the **final gather** - a Monte Carlo integration of first-bounce diffuse using a screen-space radiance cache backed by a world-space radiance cache for distant lighting; and (4) multi-bounce light transport through the surface cache itself, where each frame's texel gather samples the previous frame's indirect lighting and therefore propagates one more bounce per frame (N+2 bounces through feedback).

This module reproduces those stages on the CPU, with the following documented adaptations:

- **Card placement** uses Lumen's documented fallback projection: six axis-aligned face cards fitted to each surface's LOCAL bounds (so a moving instance carries its cache with it), subdivided when a face would exceed the per-axis texel budget; when the pieces exceed `cards.per_surface_max` (Lumen's default is 12 cards per mesh) the texel density is lowered instead of dropping pieces. Each card texel keeps up to `cards.depth_layers` front-facing hits, which stands in for Lumen's import-time surfel clustering on concave meshes (a room modelled as one mesh keeps its floor behind the ceiling's top face). The clustering itself is not implemented.
- **Ray tracing** runs against the registered triangle scene through a top-level BVH over the surfaces plus per-surface binned-SAH BVHs (rays transform into each surface's local space, so dynamic transforms only rebuild the small top-level tree) instead of building a separate tracing structure over card pages or distance fields. At CPU scene scales this is both faster to build and more accurate; shading at every hit still goes through the card surface cache exactly like Lumen's card lookup, which is the property that makes multi-bounce transport work.
- **Lighting propagation** follows Lumen exactly: direct lighting is evaluated into card texels under a per-frame texel budget (Lumen: 1024x1024 texels per frame), and the indirect gather is run in texture space against a second budget (Lumen: 512x512 texels per frame), sampling current direct and previous indirect radiance. Untouched texels keep their cached value, which is Lumen's "Lumen Scene Lighting Update Speed" behavior.
- **The world-space radiance cache** is a two-level camera-centered clipmap of octahedral probes (Lumen uses higher directional resolution - 32x32 - for world probes than for screen probes - 8x8) with toroidal scrolling: crossing a lattice cell recycles one plane of probes. Probes update on a per-frame budget, re-tracing a rotating stratum of their direction map with hysteresis blending; screen-probe rays are shortened and resolve their remainder from the world cache, which is Lumen's integration rule.
- **The final gather** exists in the module's debug view and runs on demand (when a view is exported) rather than every frame: screen probes on a jittered sub-resolution grid trace a few rays each, importance-sampled from the previous frame's probe radiance (Lumen's Screen Space Radiance Cache reuse), full-resolution pixels interpolate probe SH2 within their depth plane, then temporal accumulation and a bilateral spatial filter run - the same three tiers as Lumen's opaque Final Gather (screen cache, world cache, full-resolution interpolate/integrate/filter).
- **Contact AO, volumetric/fog GI, and translucency GI** are not implemented.
- Lumen itself is not compatible with forward shading; this module is. Where Lumen evaluates GI inside a deferred lighting pass, the engine's forward renderer consumes the exported probe grid instead (see the integration note at the bottom).

## Capability

The module provides the `lumen_gi` capability.

- ABI version: `3`, exposed as the standalone constant `lumen_gi::lumenGiCapabilityAbi` (also the default of `LumenGiCapability::abi`)
- Header: `modules/lumen_gi/include/engine/lumen_gi/capabilities.hpp`
- Binding: `ParticipantContext::acquireCapability<LumenGiCapability>()`; every entry takes `capability.self` as its first argument.

| Entry | Purpose |
| --- | --- |
| `createSurface(self, desc, error)` | Registers one GI scene surface (borrowed geometry copied in; see `GiSurfaceDescription`). Returns a handle that is never reused; `0` means "none". |
| `updateSurface(self, handle, desc, error)` | Replaces geometry/material/transform of a surface; rebuilds its cards. |
| `updateSurfaceTransform(self, handle, transform)` | Fast per-frame transform path for `Dynamic` surfaces. |
| `destroySurface(self, handle)` | Removes a surface and rebuilds the card list. |
| `setView(self, viewProj, cameraPos)` | Camera for probe placement, capture priority, and the final gather. Required before `evaluate`. |
| `evaluate(self, deltaSeconds, error)` | Runs one GI frame on the task scheduler and blocks (the `render_graph::execute` pattern). |
| `copyProbes(self, out, capacity, outHeader, outVersion)` | Snapshot of the world probe grid (position + SH2 radiance + validity); a null/zero-capacity buffer still reports version and header for cheap change detection. |
| `statistics(self, outStats)` | Per-frame and cumulative counters (texels processed, rays traced, timing). |
| `resolveProfile(self, device, outProfile)` | Scales the amortization budgets for a `renderer::DeviceProfile`. |
| `debugExport(self, view, filePath, error)` | Writes one debug view as a binary PPM (`FinalImage`, `GiOnly`, `LumenScene`, `SurfaceCache`, `ProbeIrradiance`, `Reference`). |

`evaluateProbeIrradiance` (and its core `evaluateSh2Irradiance`) are inline in the capability header so a renderer hook and the module agree on the probe representation: SH2 radiance convolved with the clamped-cosine lobe (Ramamoorthi), giving irradiance; a Lambertian BRDF turns it into outgoing radiance as `albedo / pi * irradiance`.

## Provided / required / optional capabilities

- **Provides:** `lumen_gi`
- **Requires:** none
- **Optional:** `pbr` (sun, hemispheric skylight, exposure, tone mapping) and `direct_lighting` (point/spot/directional lights). Without them the module uses the same built-in default environment as the renderer and no direct lights.

## Architecture

| File | Role |
| --- | --- |
| `module.toml` / `config.toml` | Module descriptor (capability + declared config keys) and the default `[lumen_gi]` section. |
| `include/engine/lumen_gi/capabilities.hpp` | Public POD types, function-pointer table, SH2 evaluation helpers. |
| `source/lumen_gi.cpp` | Module class: manifest, capability binding, configuration + live reload, environment snapshot, the `evaluate` entry, static-frame skip, probe snapshot publication, on-demand debug export, entry table. |
| `source/lumen_gi_internal.hpp` | Shared pipeline state: settings, surfaces, cards, probe levels, screen gather buffers, statistics. |
| `source/lumen_gi_scene.cpp` | Geometry ingestion/validation, per-surface and top-level BVHs, closest/any-hit tracing, local-space card placement and layered capture, probe direction tables and toroidal clip levels. |
| `source/lumen_gi_pipeline.cpp` | One amortized frame: capture / direct / indirect + probe planning (rotating cursors, priorities), parallel execution, commit, convergence estimate. |
| `source/lumen_gi_lighting.cpp` | Surface cache sampling (the card lookup), direct lighting units, indirect propagation units (staged, then committed), world probe updates, SH2 projection. |
| `source/lumen_gi_debug.cpp` | GBuffer rasterizer for the debug view, screen-space final gather (probes, plane-aware interpolation, temporal + bilateral). |
| `source/lumen_gi_debug_views.cpp` | Debug view composition, reference path tracer (ground truth), PPM export. |

## How it works

**Frame evaluation (`evaluate`).** One mutex guards the registry; `evaluate` snapshots the environment (pbr version probe + `copyLights`), checks the static-frame skip, then runs one amortized frame (`runGiFrame`): plan and run incremental capture, finalize captured cards, plan the direct / indirect / probe work, then

```
capture -> direct -> ( indirect | probes ) -> commit indirect
```

Each stage fans out over the scheduler with dynamic chunking and `evaluate` blocks until it finishes. The indirect stage reads only the committed cache and writes staging buffers that are committed afterwards, so concurrent traces never see a half-updated cache and texels outside the budget keep their value. The probe grid is published as an immutable snapshot, so `copyProbes` never waits for a running `evaluate`.

**Surface cache.** Each registered surface carries its geometry in local space plus a binned-SAH BVH. Its cards are axis-aligned face projections of the local bounds; capture casts ortho rays against the surface's own triangles (incremental and budgeted, nearest cards first) and stores albedo, local normal, and depth for up to `cards.depth_layers` front-facing hits per texel. A card is captured once and again only when its surface changes; moving a surface only marks its cached lighting stale. Lit points are the real surface points behind the card plane. Texels that find no front-facing surface keep the invalid depth and contribute nothing.

**Direct lighting.** Valid texels accumulate the sun (with shadow rays when `lighting.sun_shadows` is on; a mesh can shadow itself) and the direct-light snapshot using the same attenuation formulas as the renderer's shading (shadow rays with `lighting.point_light_shadows`), plus their emissive term. Unlit surfaces store their albedo as radiance instead.

**Indirect propagation.** Budgeted texels (each card has a rotating cursor, so every texel is visited each cycle) trace `lighting.indirect_rays_per_texel` stratified cosine rays that read direct + committed indirect radiance from the surface cache; hits on geometric backfaces return zero radiance (Lumen's anti-leak rule), misses fall back to the skylight. The result blends with the previous value (`lighting.indirect_temporal_blend` is the history weight, with a ramp-up so a texel's first update takes the fresh sample), so light changes propagate over frames - one more bounce becomes correct each frame, exactly Lumen's feedback loop.

**World radiance cache.** Two camera-centered clip levels (9^3 probes each) store an octahedral radiance map plus its SH2 projection per probe. A rolling budget updates the probes each frame; each direction traces the scene, shades the hit through the surface cache, and falls back to the skylight on miss. Probes whose rays mostly strike back faces (inside geometry) are marked invalid and partial back-face shares lower the validity; back-face rays are filled with the mean of the rest. The SH2 coefficients are stored irradiance-ready (see the header) and de-ringed so the irradiance never goes negative. Probes are exported in logical lattice order regardless of the toroidal storage. `copyProbes` exposes the grid; the header describes both levels.

**Screen-space final gather (debug view).** The module rasterizes its own GBuffer at the configured internal resolution, places jittered screen probes at `1/gather.screen_scale`, and traces `gather.rays_per_probe` stratified directions per probe (importance-selected from the previous frame's radiance when `gather.importance_sampling` is on). Screen probes store radiance in their own persistent 8x8 octahedral maps - Lumen's screen probe directional resolution, coarser than the world probes' 32x32 - which fill in and temporally accumulate across frames as the strata rotate. Short rays resolve near-field hits from the surface cache; misses interpolate the world cache (level 0, then level 1, then skylight). The full-resolution pass evaluates probe SH2 with the pixel normal under plane-aware weights, accumulates temporally through reprojection, and applies a bilateral filter.

**Validation.** The `Reference` debug view path traces the same camera with next-event estimation (shadowed sun and direct lights) and cosine bounces, giving ground truth the other views can be compared against numerically. On a single core, with the demo-style Cornell interior (the room as ONE mesh, lit only by small emissive panels) and an open sunlit field, the converged `FinalImage` reaches 0.97 and 0.99 of the reference's mean brightness (8-bit tone-mapped), against 0.21 and 0.88 before the rework; per-frame CPU cost dropped from about 350 ms to about 40 ms. Remaining known deviations: probe-based irradiance is blurrier than ground truth (world-probe spacing), probes lying exactly on a wall surface are invalid, and small bright emitters lose energy in the SH2 band limit exactly like Lumen's surface-cache lighting.

## Threading and lifetime

- The module mutex serializes registry mutation, `setView`, `evaluate`, `debugExport` and `statistics`. `copyProbes` only takes a tiny snapshot mutex and returns the grid published by the last `evaluate`, so a renderer polling it never waits for a running evaluation.
- `evaluate` blocks the calling thread and runs stage chunks on scheduler workers; stage tasks write disjoint buffers and read only state frozen before the graph executes.
- Surface handles are never reused and are refused once the 32-bit handle space would wrap. Card and probe buffers rebuild when their source data changes; `stop()` drops everything and bumps the probe version.

## Configuration

Every `[lumen_gi]` key, its range, and its default are documented in [configuration.md](configuration.md). Keys reload live via `ConfigurationReloadedEvent`; budget keys apply on the next `evaluate`, capacity keys (probe resolution, spacing) rebuild the affected structures.

## Integration

- **Events subscribed:** `ConfigurationReloadedEvent` (keys starting with `lumen_gi.`).
- **Events published:** none.
- **Permissions declared:** none in the entry table.

Consumer sketch (real signatures):

```cpp
#include <engine/lumen_gi/capabilities.hpp>

auto &gi = context.acquireCapability<lumen_gi::LumenGiCapability>();
if (gi.abi != lumen_gi::lumenGiCapabilityAbi)
    return; // ABI mismatch: skip rather than call through a stale struct

lumen_gi::GiSurfaceDescription box{};
box.positions = positions; box.positionStrideBytes = sizeof(float) * 3;
box.indices   = indices;   box.indexCount = indexCount;
box.flagBits  = lumen_gi::giSurfaceFlagDynamic;
const lumen_gi::GiSurfaceHandle handle = gi.createSurface(gi.self, &box, &error);

gi.setView(gi.self, glm::value_ptr(viewProjection), glm::value_ptr(cameraPosition));
gi.evaluate(gi.self, deltaSeconds, &error);

// Renderer-facing result: the world probe grid.
std::uint64_t            version = 0;
lumen_gi::GiProbeHeader  header{};
const std::uint32_t needed = gi.copyProbes(gi.self, nullptr, 0, &header, &version);
std::vector<lumen_gi::GiProbe> probes(needed);
gi.copyProbes(gi.self, probes.data(), static_cast<std::uint32_t>(probes.size()), &header, &version);
```

### Renderer integration note

The module is intentionally self-contained: it changes no other module. The renderer module (ABI 12) ships the approved consumer of that contract: it declares `lumen_gi` as an optional capability, polls `copyProbes` per `beginFrame` with the same version-gated pattern as the pbr environment and direct lights, and both of its backends sample the grid per pixel (trilinear SH2 interpolation, `evaluateSh2Irradiance` with the pixel normal) and use `albedo / pi * irradiance` as the indirect diffuse term, validity-blended over the hemispheric ambient. The CPU reference of that sampling lives in `modules/renderer/source/core/render_shared.hpp` and its GLSL twin in `modules/renderer/shaders/mesh.frag`; `[renderer] gi_indirect_lighting` switches the hook off. The standalone `demos/lumen_gi/` application demonstrates the full loop (scene registration into both modules, per-frame `evaluate`, hook-composed output, debug-view exports for A/B validation). It pumps window events every frame and, windowed, runs until the window is closed or Escape is pressed; `--frames N` pins an exact deterministic frame count for CI and screenshot exports (headless runs default to 30 frames). Its camera comes from the engine's `camera` module (WASD + mouse look, with Left Alt releasing the cursor) so the GI response to motion can be exercised interactively, window resizes re-render at the new viewport, and `--orbit` sweeps the camera on a path for hands-free dynamic-GI tests. The module's static-frame skip (`skip_unchanged_frames`, on by default) makes a stationary camera and scene converge and then stop consuming CPU entirely.
