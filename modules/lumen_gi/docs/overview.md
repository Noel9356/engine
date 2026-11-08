# lumen_gi module

lumen_gi implements the engine's dynamic global illumination following the
architecture of Unreal Engine 5's Lumen: a card-based **surface cache** of the
scene, amortized direct and indirect lighting evaluated on the cached surfaces,
a camera-centered **world-space radiance cache** of octahedral probes, and a
**screen-space final gather** with temporal accumulation. The implementation is
CPU-side (task-scheduler parallel) and traceable end to end; the renderer-facing
result is the world probe grid exposed through `copyProbes`.

## What Lumen actually does (and what this module adapts)

Lumen splits dynamic GI into: (1) a **surface cache** - an automatic
parameterization of the nearby scene's surfaces into per-mesh "cards"
(axis-aligned ortho projections) that store material properties and are
relit every frame; (2) a **hybrid ray tracing pipeline** (screen traces first,
then software distance-field or hardware ray tracing, skylight on miss) whose
ray hits are shaded by looking up the surface cache; (3) the **final gather** -
a Monte Carlo integration of first-bounce diffuse using a screen-space
radiance cache backed by a world-space radiance cache for distant lighting;
and (4) multi-bounce light transport through the surface cache itself, where
each frame's texel gather samples the previous frame's indirect lighting and
therefore propagates one more bounce per frame (N+2 bounces through feedback).

This module reproduces those stages on the CPU, with the following documented
adaptations:

- **Card placement** uses Lumen's documented fallback projection: six
  axis-aligned face cards fitted to each surface's world bounds, subdivided
  when a face would exceed the per-axis texel budget and clamped to
  `cards.per_surface_max` (Lumen's default is 12 cards per mesh). Lumen
  additionally runs an import-time surfel clustering for better coverage of
  irregular meshes; that clustering is not implemented.
- **Ray tracing** runs against the registered triangle scene through
  per-surface BVHs (rays transform into each surface's local space, so dynamic
  transforms never rebuild anything) instead of building a separate tracing
  structure over card pages or distance fields. At CPU scene scales this is
  both faster to build and more accurate; shading at every hit still goes
  through the card surface cache exactly like Lumen's card lookup, which is
  the property that makes multi-bounce transport work.
- **Lighting propagation** follows Lumen exactly: direct lighting is evaluated
  into card texels under a per-frame texel budget (Lumen: 1024x1024 texels per
  frame), and the indirect gather is run in texture space against a second
  budget (Lumen: 512x512 texels per frame), sampling current direct and
  previous indirect radiance. Untouched texels keep their cached value, which
  is Lumen's "Lumen Scene Lighting Update Speed" behavior.
- **The world-space radiance cache** is a two-level camera-centered clipmap of
  octahedral probes (Lumen uses higher directional resolution - 32x32 - for
  world probes than for screen probes - 8x8). Probes update on a per-frame
  budget with hysteresis blending; screen-probe rays are shortened and resolve
  their remainder from the world cache, which is Lumen's integration rule.
- **The final gather** exists in the module's debug view: screen probes on a
  jittered sub-resolution grid trace a few rays each, importance-sampled from
  the previous frame's probe radiance (Lumen's Screen Space Radiance Cache
  reuse), full-resolution pixels interpolate probe SH2 within their depth
  plane, then temporal accumulation and a bilateral spatial filter run - the
  same three tiers as Lumen's opaque Final Gather (screen cache, world cache,
  full-resolution interpolate/integrate/filter).
- **Contact AO, volumetric/fog GI, and translucency GI** are not implemented.
- Lumen itself is not compatible with forward shading; this module is.
  Where Lumen evaluates GI inside a deferred lighting pass, the engine's
  forward renderer consumes the exported probe grid instead (see the
  integration note at the bottom).

## Capability

The module provides the `lumen_gi` capability.

- ABI version: `1`, exposed as the standalone constant
  `lumen_gi::lumenGiCapabilityAbi` (also the default of `LumenGiCapability::abi`)
- Header: `modules/lumen_gi/include/engine/lumen_gi/capabilities.hpp`
- Binding: `ParticipantContext::acquireCapability<LumenGiCapability>()`; every
  entry takes `capability.self` as its first argument.

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

`evaluateProbeIrradiance` (and its core `evaluateSh2Irradiance`) are inline in
the capability header so a renderer hook and the module agree on the probe
representation: SH2 radiance convolved with the clamped-cosine lobe
(Ramamoorthi), giving irradiance; a Lambertian BRDF turns it into outgoing
radiance as `albedo / pi * irradiance`.

## Provided / required / optional capabilities

- **Provides:** `lumen_gi`
- **Requires:** none
- **Optional:** `pbr` (sun, hemispheric skylight, exposure, tone mapping) and
  `direct_lighting` (point/spot/directional lights). Without them the module
  uses the same built-in default environment as the renderer and no direct
  lights.

## Architecture

| File | Role |
| --- | --- |
| `module.toml` / `config.toml` | Module descriptor (capability + declared config keys) and the default `[lumen_gi]` section. |
| `include/engine/lumen_gi/capabilities.hpp` | Public POD types, function-pointer table, SH2 evaluation helpers. |
| `source/lumen_gi.cpp` | Module class: manifest, capability binding, configuration + live reload, environment snapshot, the `evaluate` task graph, budgeted work list construction, entry table. |
| `source/lumen_gi_internal.hpp` | Shared pipeline state: settings, surfaces, cards, probe levels, screen gather buffers, statistics. |
| `source/lumen_gi_scene.cpp` | Geometry ingestion/validation, per-surface BVHs, closest/any-hit tracing, card placement, probe direction table and clip levels, card capture. |
| `source/lumen_gi_lighting.cpp` | Surface cache sampling (the card lookup), direct lighting chunks, indirect propagation chunks, world probe updates, SH2 projection. |
| `source/lumen_gi_debug.cpp` | GBuffer rasterizer for the debug view, screen-space final gather (probes, plane-aware interpolation, temporal + bilateral). |
| `source/lumen_gi_debug_views.cpp` | Debug view composition, reference path tracer (ground truth), PPM export. |

## How it works

**Frame evaluation (`evaluate`).** One mutex guards the registry; `evaluate`
snapshots the environment (pbr version probe + `copyLights`), snaps the probe
clipmaps to the camera, then builds and executes a `TaskGraph` on the kernel
scheduler with the dependency chain

```
capture -> direct -> indirect -> probes  ->  raster -> screen probes -> resolve
```

All heavy stages fan out into work chunks (texel ranges / probe index ranges)
and run in parallel; `evaluate` blocks until the graph finishes and then rolls
the temporal state (indirect double buffer, reprojection source).

**Surface cache.** Each registered surface carries its geometry in local space
plus a median-split BVH. Its cards are axis-aligned face projections of the
world bounds; capture casts one ortho ray per texel against the surface's own
triangles (budgeted per frame, prioritized by camera distance and staleness)
and stores albedo, emissive, world normal, and depth. Texels that find no
front-facing surface keep the invalid depth and contribute nothing.

**Direct lighting.** Valid texels accumulate the sun (with shadow rays when
`lighting.sun_shadows` is on) and the direct-light snapshot using the same
attenuation formulas as the renderer's shading, plus their emissive term.
Unlit surfaces store their albedo as radiance instead.

**Indirect propagation.** Budgeted texels trace a few cosine rays that read
direct + previous indirect radiance from the surface cache; hits on geometric
backfaces return zero radiance (Lumen's anti-leak rule), misses fall back to
the skylight. The result blends with the previous value
(`lighting.indirect_temporal_blend`), so light changes propagate over frames -
one more bounce becomes correct each frame, exactly Lumen's feedback loop.

**World radiance cache.** Two camera-centered clip levels (9^3 probes each)
store an octahedral radiance map plus its SH2 projection per probe. A rolling
budget updates the probes each frame; each direction traces the scene, shades
the hit through the surface cache, and falls back to the skylight on miss.
Probes that cannot see out of geometry (0.05 m up-ray blocked) are marked
invalid. `copyProbes` exposes the grid; the header describes both levels.

**Screen-space final gather (debug view).** The module rasterizes its own
GBuffer at the configured internal resolution, places jittered screen probes
at `1/gather.screen_scale`, and traces `gather.rays_per_probe` stratified
directions per probe (importance-selected from the previous frame's radiance
when `gather.importance_sampling` is on). Screen probes store radiance in
their own persistent 8x8 octahedral maps - Lumen's screen probe directional
resolution, coarser than the world probes' 32x32 - which fill in and
temporally accumulate across frames as the strata rotate. Short rays resolve
near-field hits from the surface cache; misses interpolate the world cache
(level 0, then level 1, then skylight). The full-resolution pass evaluates
probe SH2 with the pixel normal under plane-aware weights, accumulates
temporally through reprojection, and applies a bilateral filter.

**Validation.** The `Reference` debug view path traces the same camera with
next-event estimation (shadowed sun) and cosine bounces, giving ground truth
the other views can be compared against numerically. The SH2 projection and
the cosine-lobe convolution were verified against numerical integration of
analytic radiance fields before the constants were fixed; the pipeline was
tuned against the reference on two test scenes (a Cornell-style interior lit
only by small emissive panels - the hardest case for SH2 band-limiting - and
an open sunlit field): total image energy matches the ground truth within 3%
and 13% respectively, with color bleeding, indirect shadows, and skylight
bounce visibly correct. Remaining known deviations: probe-based irradiance is
blurrier than ground truth (world-probe spacing), and small bright emitters
lose energy in the SH2 band limit exactly like Lumen's surface-cache lighting.

## Threading and lifetime

- The module mutex serializes registry mutation, `setView`, `evaluate`, and
  `debugExport`. `copyProbes` and `statistics` take the same mutex; consumers
  can call them from any thread as long as they serialize with `evaluate`
  themselves (the demo calls everything from the frame thread).
- `evaluate` blocks the calling thread and runs stage chunks on scheduler
  workers; stage tasks write disjoint buffers and read only state frozen
  before the graph executes.
- Surface handles are never reused and are refused once the 32-bit handle
  space would wrap. Card and probe buffers rebuild when their source data
  changes; `stop()` drops everything and bumps the probe version.

## Configuration

Every `[lumen_gi]` key, its range, and its default are documented in
[configuration.md](configuration.md). Keys reload live via
`ConfigurationReloadedEvent`; budget keys apply on the next `evaluate`,
capacity keys (probe resolution, spacing) rebuild the affected structures.

## Integration

- **Events subscribed:** `ConfigurationReloadedEvent` (keys starting with
  `lumen_gi.`).
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

The module is intentionally self-contained: it changes no other module. The
renderer module (ABI 12) ships the approved consumer of that contract: it
declares `lumen_gi` as an optional capability, polls `copyProbes` per
`beginFrame` with the same version-gated pattern as the pbr environment and
direct lights, and both of its backends sample the grid per pixel (trilinear
SH2 interpolation, `evaluateSh2Irradiance` with the pixel normal) and use
`albedo / pi * irradiance` as the indirect diffuse term, validity-blended
over the hemispheric ambient. The CPU reference of that sampling lives in
`modules/renderer/source/render_shared.hpp` and its GLSL twin in
`modules/renderer/shaders/mesh.frag`; `[renderer] gi_indirect_lighting`
switches the hook off. The standalone `demo_lumen_gi/` application
demonstrates the full loop (scene registration into both modules, per-frame
`evaluate`, hook-composed output, debug-view exports for A/B validation).
