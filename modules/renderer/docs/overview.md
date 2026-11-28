# renderer module

The renderer module turns engine scene state into pixels. It provides a backend-agnostic `renderer` capability - renderers, meshes, textures, camera state, overlay drawing - and ships two backends behind that interface: a full Vulkan backend (device, swapchain, frame scheduling, PBR pipelines, staging uploads) and a headless software rasterizer that draws on the CPU and blits into the window surface. The capability layer in `renderer.cpp` orchestrates the module; its settings model (`renderer_settings.cpp`), version-gated lighting input feeds (`frame_inputs.cpp`), draw-list scheduler (`draw_scheduler.cpp`) and resource normalization (`render_data.cpp`) are the four concerns it routes to; the backends only implement `RenderBackend`.

The module keeps no world-space scene state of its own: draws are per-frame command lists (borrowed pointers, valid for the call), and every lighting input it consumes is a bounded, camera-relative snapshot - the radiance_cache probe grid is a camera-centered clipmap the provider scrolls toroidally, the reflection field is screen-space, and the light set is a flat array. A renderer therefore places no upper bound on the world it draws; scenes orders of magnitude larger than any loading-screen-free budget (planetary scales and beyond) stream their geometry through the same frame command list, and moving or reshaping objects simply submit different matrices and meshes each frame.

## Capability

| Capability | ABI | Header |
| --- | --- | --- |
| `renderer` | `renderCapabilityAbi = 14` | `modules/renderer/include/engine/renderer/capabilities.hpp` |
| `renderer.vulkan` (secondary) | `renderCapabilityAbi = 14` (same constant) | same header |
| `renderer.headless` (secondary) | `renderCapabilityAbi = 14` (same constant) | same header |

The shared data types the capability references live in `shared_types/include/engine/renderer/` (`mesh.hpp`, `material.hpp`, `device_profile.hpp`) and in `modules/renderer/include/engine/renderer/` (`draw_sort.hpp`, `draw_batch.hpp`, `dynamic_resolution.hpp`).

Handles are module-assigned ids starting at 1 (`RendererHandle`, `MeshHandle`, `TextureHandle`); `0` (`invalidRenderer`, `invalidMesh`, `invalidTexture`) always means "none", handles are never reused, and mesh and texture ids are global across all renderers of the module. All capability calls take the renderer handle first; an invalid handle is either ignored or reported through the optional `RenderError*` out-parameter.

## Provided / required / optional capabilities

From `module.toml` (and identical in `makeManifest()` and in the module entry table at the bottom of `renderer.cpp`):

| Kind | Capabilities |
| --- | --- |
| Provided | `renderer`, `renderer.vulkan`, `renderer.headless` |
| Required | `window` |
| Optional | `window.vulkan`, `pbr`, `direct_lighting`, `radiance_cache`, `reflection_field` |

`window` supplies the surface a renderer draws into; `window.vulkan` (`VulkanSurfaceCapability`) supplies the native handles and instance extensions the Vulkan backend needs; `pbr` supplies the sun/ambient/tone-mapping environment (with a version counter so updates are pushed once) and `direct_lighting` supplies point/spot/directional lights; when absent, a built-in default environment is used and no direct lights are set. `radiance_cache` supplies the world-space radiance probe grid for the indirect-lighting hook (see below); when absent, the legacy hemispheric ambient is used unchanged. `reflection_field` supplies the screen-space reflection field for the indirect-specular hook; when absent, the pre-hook specular is kept. The entry table declares no permissions (`declaredPermissions = nullptr`).

Because the renderer now acquires six capabilities in total, the kernel's trust gate classifies it as "broad"; the engine's own application configs trust it by name under `[trust]` (`trustModules = ["renderer"]`).

## radiance_cache probe-grid indirect lighting (ABI 12 hook)

When the optional `radiance_cache` capability is present and `[renderer] gi_indirect_lighting` is true (the default), `beginFrame` polls `copyProbes` for the module's probe-grid version and, on change, copies the snapshot and pushes it to the backend via `RenderBackend::setProbeGrid` - the same version-gated pattern the pbr environment and direct lights use. Both backends turn the snapshot into an indirect term that replaces the legacy hemispheric ambient (both are diffuse integrals, so both scale by the material's `(1 - metallic)` factor, exactly like the direct-lighting diffuse - metal surfaces carry almost no diffuse response, so chrome parts keep their mirror look instead of picking up a chalky ambient):

- **Headless (CPU):** `pbrShade` samples the grid per shaded pixel - trilinear-footprint over the camera-centered probe lattice with smoothstep lattice weights (level 0 inside its half-cell-extended bounds, level 1 elsewhere with boundary clamping; the per-axis smoothstep is the DDGI probe-blend convention that keeps the lattice join C1, so broad flats do not photograph the cell grid), cosine-lobed SH2 convolution per corner probe, irradiance renormalized over the footprint's valid probes (each weighted by its `validity`), and the result blended over the hemispheric ambient through a coverage ramp that reaches full GI at a quarter of valid coverage - probe lattices routinely lose the corners that fall inside nearby geometry, so the ambient remains the fallback only while almost no probe is valid. Unregistered or partially populated scenes therefore degrade gracefully to the legacy look instead of going black.
- **Vulkan (GPU):** `mesh.frag` implements the identical lattice walk, SH2 convolution, and validity-renormalized ambient ramp. The 64-byte grid header rides the frame uniform (`FrameData` grew from 2224 to 2288 bytes; the vertex-stage block is unchanged - it reads only the leading members), and the packed probe records (8 vec4 per probe: position.xyz + validity, then the 27 SH2 floats) live in a read-only storage buffer on set 0 / binding 1, refreshed per frame slot when the snapshot version changes. A zero-filled dummy buffer is bound while the hook is inactive, and the shader branches off the header's active flag, so a machine without the module renders the pre-hook image.

The snapshot can lag the latest `radiance_cache::evaluate()` by one frame: module update order does not guarantee radiance_cache runs before the renderer, and the cache is temporal by design (Lumen's updates are amortized the same way).

The demo `demos/lighting/` exercises this hook end to end: it registers its ground field and object scatter into both the renderer and radiance_cache, and DEMO_SET=renderer.gi_indirect_lighting=false renders the same scene without the hook for A/B validation.

## reflection_field screen-space field hook

When the optional `reflection_field` capability is present and `[renderer] reflection_indirect_specular` is true (the default), `beginFrame` polls `copyField` for the module's field version and, on change, copies the snapshot and pushes it to the backend via `RenderBackend::setReflectionField` - the same version-gated pattern as the probe grid above. The field holds the expected GGX-lobe specular response per screen texel (see the module's overview.md), so the composite is a single add that REPLACES the analytic environment specular wherever it answers (the field's traced response already contains the surface's specular-environment share, so the two terms never double):

- **Headless (CPU):** both rasterization paths (the inline shading loop and the tile path) sample the bilinear field response of the shaded pixel's world position BEFORE the PBR pass and either scale the pass's analytic environment specular out (a traced answer) or add the sample to the linear-space color right after the pass, before emissive and tone mapping; the field is projected through its own camera, so pixels outside it read zero. An invalid/empty field keeps the pre-hook shading bit-identical. Unlit pixels and the overlay/fur paths carry no reflection term.
- **Vulkan (GPU):** the hook stages the snapshot into the frame slot's RGBA32F image (set-0 binding 2) and `shaders/mesh.frag` composites the same value through its `texelFetch` mirror of the module's sampler - same projection, same neutral-roughness and band gates, same env-specular replacement, plus the transmissive commit that switches the output alpha to 1 while the traced response carries the transmitted share. See backends.md ("Behavior parity") for the scope note.

The snapshot can lag the latest `reflection_field::evaluate()` by one frame, exactly like the probe grid above.

The demo `demos/lighting/` exercises this hook alongside radiance_cache: every scene surface is registered into both modules, and DEMO_SET=renderer.reflection_indirect_specular=false renders the same scene without the composite for A/B validation.

Note: the `renderer.vulkan` entries only work when the module was built with `ENGINE_RENDERER_BACKEND_VULKAN=1` (set by CMake when Vulkan headers and a loader were found); otherwise they fail gracefully.

Vulkan swapchains require surface support for color-attachment usage. PPM screenshot readback is enabled when the surface also supports transfer-source usage; otherwise rendering remains available and screenshot requests report a creation failure.

## Transparent draws

A material with `opacity` below 1 renders as a transparent draw: depth testing stays on (opaque geometry in front keeps occluding it) while depth writes turn off, so co-transmitted surfaces do not occlude each other and the composite follows submission order exactly like the shell-fur stack. The composite has two modes per pixel. Where the reflection_field field carries a traced response for the pixel's material (the same gate the field sample applies everywhere), the shaded result commits directly: the field's response already contains the interface's reflected share plus the transmitted share refracted through the medium, so blending the raw background in as well would show the scene behind twice. Where no traced response exists - the surface sits outside the traced roughness band, or the reflection module is absent - the draw alpha blends with the material's opacity over the framebuffer (the classic transparent pass, unbent). Consumers order their transparent draws after the opaque scene, far to near within the transparent group, the same contract the fur stack documents. The surface's own diffuse terms (direct, hemispheric, and probe-grid ambient) carry NO opacity factor - the opacity reaches the output exactly once, through the commit blend on the headless side and through `oColor.a` plus the fixed-function blend in `mesh.frag` (the CPU/GPU twin contract); scaling the diffuse as well would composite it at opacity squared and break backend parity. The pbr module's material conversion maps glTF/MTL opacity and index of refraction into these fields, so imported transmissive materials flow through unchanged.

## Distance fog

`MeshMaterial`'s fog triple (`fogColor`, `fogScaleMeters`, `fogAsymptote`) composites an aerial perspective per sample: every shaded pixel blends toward `fogColor` by `fogAsymptote * (1 - exp(-cameraDistance / fogScaleMeters))`, evaluated at the sample's own interpolated world position after every lighting and emissive term and before the output transform. Because the weight follows the pixel instead of the draw, the fog stays continuous across every mesh and chunk boundary by construction - a per-draw uniform weight steps at each streamed chunk's edge and photographs as a rectangle grid on tiled terrain, which is what the field replaces (the planet demo composes its whole aerial perspective through it). `fogAsymptote` 0 (the default) disables the pass at one branch per sample; unlit draws fog like lit ones, so distant emissive cards fade with the terrain behind them. Both backends composite the identical blend: the CPU rasterizer applies it on all of its shading paths (the sequential core, the per-sample fan core, and the run-shading epilogue, where an active fog defers the run kernel's output transform to the caller so the blend lands in linear space after the reflection-field and emissive adds), and the Vulkan fragment shader reads the triple from the material extension block (`MaterialExtension` in `shaders/mesh.frag`) - a set-2 uniform buffer the renderer binds per distinct material tuple at a dynamic offset, keeping the fields outside the push-constant budget.

Two more per-material appearance fields ride the same paths on both backends. The procedural surface mottle pair (`noiseScaleMeters`, `noiseStrength`) multiplies the shaded albedo by a two-octave world-anchored value noise at the sample's position, with a footprint-adaptive frequency floor (the CPU path estimates the footprint per raster record, the shader from the world-position derivatives) so minified draws widen their patches instead of aliasing the lattice into speckle - tiled ground textures break into natural patchwork instead of visibly repeating. The near-field relief overlay (`detailNormalTexture`, `detailUvScale`, `detailFadeStartMeters`, `detailFadeEndMeters`, `detailStrength`) perturbs the tangent-space normal with a much finer-tiling normal map, faded per pixel by camera distance (full at the fade's start, gone at its end; the slope-add blend keeps the base map's larger relief standing), so a surface gains real grain as the eye descends instead of re-showing the same base tiling at every range - the sixth texture slot (`uDetailNormalTexture`, set-1 binding 5 on Vulkan) pairs with the extension block's detail vec4.

## Cloth shading and dynamic geometry (ABI 14)

Two material fields drive the fabric rendering path, and one appended entry point keeps simulated surfaces cheap to draw:

- **`MeshMaterial.sheenFactor`** (0 = plain metallic-roughness PBR, unchanged default): values above 0 blend the GGX specular toward the Charlie sheen lobe (Estevez & Kulla 2017) with the Neubelt visibility term - the production cloth specular pair - and soften the direct-light terminator with the energy-conserving wrap-diffuse weight ((nDotL + w) / (1 + w)^2, w = 0.5 x sheenFactor), so fabric keeps a grazing-angle rim and light bleeds through the terminator the way thin cloth does. The sheen highlight is tinted toward the material's base color (60% toward white), so colored fabric shows a colored rim instead of a dielectric-gray one. Both backends implement the identical formula (the `shadeDirectLight` helper in `shaders/mesh.frag` and `pbrShade` in `pbr_shading.hpp`), applied to the sun and every direct light.
- **`MeshMaterial.alphaCutoff`** (0 = disabled, unchanged default): alpha-cutout masking for textured opaque draws (glTF's alphaMode MASK). Texels whose sampled base-color alpha lands below the cutoff are culled per pixel before any shading - in the headless rasterizer the sequential and tile-record cores skip the pixel exactly like the fur crop does, the 4-wide run-shading path fetches each accepted lane's texel alpha first and drops masked lanes from the pass (a run shades only its surviving lanes, so foliage canopies pay the SoA discount without shading discarded texels), and the Vulkan fragment shader discards it - while the draw stays an opaque, depth-writing draw, so a texture's alpha channel carves the silhouette (foliage billboards, grass tufts, foam sprites) without transparent-pass sorting. Transparent draws (opacity below 1) ignore the cutoff since their blending already handles partial coverage.
- **`MeshMaterial.doubleSided`**: two-sided rendering for thin sheets. The draw skips back-face culling (a dedicated two-sided pipeline variant in the Vulkan backend - cull mode NONE with the same shaders - and the winding-vote path disabled in the headless rasterizer) and flips the shading normal toward the viewer on back-facing fragments before the tangent frame is built, the glTF `doubleSided` contract (back-face normals are reversed before lighting). Cloth sheets, flags, and curtains therefore shade correctly from both sides without a duplicated shell mesh.
- **`updateMeshVertices(renderer, mesh, vertices, vertexCount)`**: replaces a mesh's whole vertex stream with the canonical interleaved layout (position, normal, color, uv - 11 floats per vertex) at the creation vertex count; the index buffer is untouched. This is the simulated-surface path: a cloth sheet's particle grid IS the mesh's vertex grid, so one update and one draw render the whole deforming sheet each frame - the consumer recomputes smooth per-particle normals from the simulated grid and hands the stream over, and the bounding sphere refreshes for frustum culling. The Vulkan backend streams updates through per-frame-slot staging buffers (a persistent-mapped transfer ring mirroring the instance-matrix ring's slot lifecycle, with the transfer-to-vertex-input barrier recorded in the same command buffer as the consuming draws); updates that arrive between frames park until the next `beginFrame` and record before any draw can bind the mesh. The headless backend rewrites its planar vertex arrays, recomputes bounds, and drops the mesh's cached transforms and winding votes. Callers must null-check the entry point (a module built before ABI 14 leaves it null).

## Sun and sky shadow mapping (headless backend)

The rasterizer's unoccluded light terms gain real occluders through two orthographic depth maps built by `endFrame` from the frame's own draws (`sun_shadows.hpp`):

- **Sun shadows** (`renderer.sun_shadows`): the frame's opaque lit draws rasterize into a sun-aligned map the NEXT frame's sun term shades through a percentage-closer filter - the caster set only exists once every draw has been seen, and the build runs after the last rasterizer band so it never lands on the shaded pixels' critical path. With a running task scheduler the build splits the caster list into per-worker ranges that rasterize concurrently through an atomic per-cell minimum store, so the concurrent build produces exactly the serial build's map while the frame's map builds use both workers. The u/v fit follows the view frustum's world bounds intersected with the casters' own span (texel density tracks what the camera can see; a far plane reaching past the scene does not dilute it), the depth span covers the casters, and the percentage-closer compare carries a slope-scaled bias plus a normal offset with a soft depth-compare ramp, so contact edges shade smoothly without acne. Unlit draws (sky domes, emissive panels - the decorative geometry the lighting environment already accounts for) and transparent draws (their transmitted share is the reflection field's chain, not a hard occluder) do not block the sun, matching the lighting modules' own shadow-ray semantics; a mesh outside the view frustum still casts into it because recording happens before the cull. A hash over the caster set, the sun, and the camera skips the rebuild while nothing changed, so a converged view pays the map rasterization once and the per-sample filter thereafter.
- **Zenith sky-visibility shadows** (`renderer.sky_shadows`): the same machinery aimed straight down shades the HEMISPHERIC ambient by what blocks the open sky above each point - the sky fill a photograph loses under tables, props, and overhangs. The GI/bounce ambient stays untouched (light bounced from every direction does not come from the zenith), so an emissive-lit interior keeps its bounce while an open field grounds its props; the term only bites where the hemispheric ambient is meaningful, which is also why the demo's panel-lit views configure it off.

## Camera response (headless backend)

The presentation passes over the finished, bloomed output rows replace the aliasing a software rasterizer leaves with the response a camera adds, in a fixed order: film antialiasing first (`applyFilmAntialias`, `renderer.film_aa`), then the photographic layer (`applyCameraResponse`). The antialiasing pass runs in the rows' encoded display space: five luminance taps bracket each pixel's local contrast, four corner taps orient the blend axis along the edge normal, and the kernel mixes the center with bilinear samples offset along that axis - silhouettes keep their shape while screen-space diagonals lose the one-texel staircase, and flat neighborhoods take an early-out that leaves them untouched. A second, cross-edge stage (`renderer.film_aa_strength`) catches the one-texel hard step a straight silhouette keeps when it lands between supersample columns: a boundary pixel - one across neighbor on its own side's plateau, the other on the far side - blends toward the two across neighbors' mean, so thin bright features and smooth gradients are left alone while the remaining hard steps soften into ramps. A snapshot of the rows feeds every sample, so the band-parallel output stays deterministic no matter which worker processed which row first.

The photographic layer (`applyCameraResponse`) runs in linear space over the smoothed rows: a contrast curve pivoting at mid-gray (`renderer.contrast`), a radial lens falloff (`renderer.vignette`), and one deterministic sensor-grain sample per pixel (`renderer.grain` - a per-pixel multiplicative gain so the grain reads as luminance-density film grain with the hue preserved, amplitude peaking in the midtones, fixed per frame and pixel so a converged static view stays bit-stable across exports while an animated view shows uncorrelated noise). Rows split across the task scheduler exactly like the resolve bands; every pixel reads and writes only its own cell.

## Architecture
| File | Role |
| --- | --- |
| `module.toml` / `config.toml` | Module descriptor (capabilities + declared config keys) and default values for the `[renderer]` section. |
| `include/engine/renderer/capabilities.hpp` | The public capability structs, handles, and per-draw/per-frame structs. |
| `include/engine/renderer/draw_sort.hpp` | Total-order material comparison and draw-command sorting (sort modes for batching). |
| `include/engine/renderer/draw_batch.hpp` | `RenderInstanceBatchCache`: folds draw commands into instanced batches plus a packed instance-matrix stream. |
| `include/engine/renderer/dynamic_resolution.hpp` | `DynamicResolutionController` (render-scale stepping) and `AdaptiveQualityController` (quality-preset stepping) fed from frame timings. |
| `source/core/renderer.cpp` | Module entry: `RenderModule` (ModuleInterface), capability binding, backend selection, and the entry table. |
| `source/core/renderer_settings.hpp/.cpp` | `RendererSettings` (every `[renderer]` knob in one struct) and `readRendererSettings()`: the layered-configuration overlay with validated ranges plus the hardware-derived `auto_config` path. |
| `source/core/frame_inputs.hpp/.cpp` | `FrameInputs`: the version-gated pushes of the pbr environment, direct lights, radiance_cache probe grid and reflection_field field into a backend. |
| `source/core/draw_scheduler.hpp/.cpp` | `DrawScheduler`: draw-list sorting with the cached permutation, the backend batch submission, and the Vulkan instance fold. |
| `source/core/render_errors.hpp` | The optional-error out-parameter convention shared by the module's entry points and helpers. |
| `source/core/render_shared.hpp` | Umbrella for the focused shading headers plus the backend-neutral pieces: device/presentation enums, default environment, frustum rejection, sRGB/tone-map output encoders, and the PPM/error utilities. |
| `source/core/direct_light_shading.hpp` | `PreparedDirectLight` and `resolveDirectLight()` - the per-point light resolution shared by the PBR and fur passes. |
| `source/core/pbr_shading.hpp` | The CPU Cook-Torrance shading pass `pbrShade` (the reference twin of `shaders/mesh.frag`). |
| `source/core/fur_shading.hpp` | The shell-fur shading helpers and `furShade` (the twin of `shaders/fur.frag`). |
| `source/core/gi_probe_sampling.hpp` | The radiance_cache probe-grid snapshot types, prepared level descriptors, and the smoothstep-weighted trilinear SH2 irradiance walk. |
| `source/core/reflection_field_sampling.hpp` | The reflection_field field snapshot types and the bilinear roughness-band-gated sample. |
| `source/core/shading_inline.hpp` | The hot-path inline marker shared by the per-pixel shading entries. |
| `source/backend/render_backend.hpp` | `RenderBackend` - the abstract interface both backends implement. |
| `source/core/render_data.hpp/.cpp` | Mesh/texture normalization: attribute repacking, index validation, normal generation, pixel format conversion. |
| `source/core/ktx_texture_loader.hpp/.cpp` | KTX loading for `createTextureFromFile`: container detection, libktx-based parsing and Basis transcoding, block-level gathering, bcdec software decode for the mean and the headless backend. |
| `source/backend/headless/headless_types.hpp` | Headless backend data types: `HeadlessMesh` (planar SoA mesh copy with bounding sphere and LOD index chains) and `HeadlessTexture` (RGBA8 upload copy plus float-RGB mip chain) shared between the backend and its rasterizer. |
| `source/backend/headless/headless_backend.hpp/.cpp` | Headless backend: CPU rasterizer with culling, LOD, transform cache, and task-scheduler parallelism. |
| `source/backend/headless/tile_bins.hpp` | The tile-binning policy of the headless rasterizer's parallel paths (workers own disjoint tiles, so rasterization needs no per-pixel locks). |
| `source/backend/vulkan/vulkan_backend.hpp/.cpp` | Vulkan backend: frame lifecycle, command recording, instance streaming, deferred destruction. |
| `source/backend/vulkan/vulkan_device.hpp/.cpp` | `VulkanDeviceContext`: instance/surface/device/VMA bring-up, device profile, async staging-upload queue. |
| `source/backend/vulkan/staging_pool.hpp` | Fence-recycled staging-allocation policy driving the device context's upload queue. |
| `source/backend/vulkan/vulkan_frames.hpp/.cpp` | `VulkanFrameScheduler`: frames-in-flight slots, fences/semaphores, GPU timestamp queries. |
| `source/backend/vulkan/vulkan_pipeline.hpp/.cpp` | `VulkanPipeline`: render pass, descriptor layouts, PBR mesh pipeline, overlay pipeline, disk-backed pipeline cache. |
| `source/backend/vulkan/vulkan_resources.hpp/.cpp` | `VulkanResources`: frame uniform ring, mesh buffers, texture images/samplers, material descriptor-set cache with per-renderer capacity limits. |
| `source/backend/vulkan/vulkan_swapchain.hpp/.cpp` | `VulkanSwapchain`: format/present-mode selection, image views, framebuffers, recreation, screenshot readback (PNG or legacy PPM). |
| `shaders/mesh.vert`, `shaders/mesh.frag` | GLSL sources of the PBR pipeline (the fragment shader is the reference twin of the CPU shading in `source/core/pbr_shading.hpp`). |
| `shaders/ui.vert`, `shaders/ui.frag` | GLSL sources of the overlay pipeline. |
| `shaders/*_gen.hpp` | SPIR-V arrays included by `source/backend/vulkan/vulkan_pipeline.cpp`; regenerated from the GLSL by `engine_module_compile_shaders` at build time. |

## How it works

### Renderer lifecycle

A consumer creates a renderer per window surface and drives it frame by frame:

1. `createRenderer(surface, &error)` - resolves the backend (see below), constructs it, and returns a `RendererHandle`.
2. Optionally `createMesh`/`createTexture`/`createTextureFromFile` (also possible later); `createTextureFromFile` accepts JPG/PNG through stb_image and KTX 2 files (Basis-encoded or raw block-compressed) through the KTX loader - see "Mesh and texture resource model" for the transcode and upload paths; `textureMean` then reports each texture's level-0 mean linear RGB (consumers that multiply a base color by a texture divide by the mean so the multiply preserves the base color's brightness on average and only adds detail).
3. Per frame: `setViewport`, `setCamera`, then `beginFrame(renderer, frame)`.
4. Record draws: `drawMesh`, `drawMeshes` (sorted/batched command list), `drawMeshInstances` (explicit instancing), `drawFur`/`drawFurInstances` (alpha-blended shell-fur layer stacks, ABI 13 - see the physics_fur module's overview), `drawOverlay` (UI triangles); dynamic surfaces refresh their vertices first through `updateMeshVertices` (ABI 14 - the simulated-cloth path).
5. `endFrame(renderer)` submits/presents (Vulkan) or blits (headless).
6. `destroyRenderer`, `destroyMesh`, `destroyTexture` as needed.

`beginFrame` is also where the capability layer pushes state: the pbr environment when its version changed, the direct-light set when its version changed, and frame timing for the kernel profiler.

### Frame graph (Vulkan backend, per frame slot)

```
beginFrame:
    device.beginFrame -> advance VMA frame index, refresh memory budget
        every [renderer] memory_budget_update_interval_frames
    collectCompletedUploads -> retire finished staging copies
    swapchain recreate? -> when dirty or viewport != swapchain extent
    waitForFrameSlot -> wait slot fence (or timeline value), harvest GPU
        timestamps, reset the slot's command pool
    drain deferred destroys -> free resources queued on this slot last frame
    reset instance-stream cursor (one identity matrix reserved at offset 0)
    rebuild frame uniform? -> when camera/environment/lights changed (revision counter)
    begin command buffer, begin GPU timestamps
    acquire swapchain image -> OUT_OF_DATE triggers recreate (frame skipped);
        SUBOPTIMAL marks dirty
    wait image fence (fence path) or rely on timeline ordering
    record: clear color+depth, begin render pass (or vkCmdBeginRendering with
        dynamic rendering), bind PBR pipeline, viewport/scissor,
        bind frame uniform set (set 0)

drawMesh / drawMeshes / drawMeshInstances:
    frustum-cull per mesh/instance (model-transformed bounding sphere)
    dedup-and-record: push constants, material set (set 1), vertex/index buffers
    instanced path: stream matrices into the slot's host-visible ring, flush,
        one vkCmdDrawIndexed with the run's offset and instance count;
        stream overflow falls back to single draws

drawOverlay:
    copy vertices/indices into the slot's scratch buffers
    bind overlay pipeline, push display->NDC transform
    per command: scissor from clip rectangle, bind texture set, draw indexed

endFrame:
    end render pass (+ optional frame->readback copy for PPM export)
    end GPU timestamps, end command buffer
    submit (wait: imageAvailable; signal: renderFinished + slot fence or timeline)
    register deferred export (copy recorded; file written at the recording
        slot's next beginFrame, or on recreation/destruction)
    present (renderFinished) -> OUT_OF_DATE/SUBOPTIMAL mark dirty
    rotate frame slot, framesRendered++
```

The headless backend's frame is the same shape minus the GPU: `beginFrame` (re)allocates and clears the CPU color/depth buffers, draws rasterize directly into them (LOD selection by angular size, frustum culling, a persistent per-mesh vertex transform cache, optional task-scheduler parallelism with per-pixel-group depth locks), and `endFrame` copies the result into the window surface's locked pixel buffer. Shading is the dominant per-sample cost under supersampling, so the tile rasterizer shades run-eligible records in 4-wide structure-of-arrays passes (see `source/backend/headless/quad_shading.hpp`: every 3-component dot product becomes lane-wise multiplies and adds with no horizontal reduction, and each perspective weight, normalize and Cook-Torrance term pays once for four samples; the sun term carries the scalar path's per-sample shadow skip at run granularity - a run whose four lanes all face away from the sun skips the whole Cook-Torrance sun block, since the masked add contributes exact zeros either way, which keeps backlit-heavy scenes (a globe's night hemisphere, self-shaded cliff walls) from paying for lanes that contribute nothing; records whose draws the run kernel does not model - normal maps, cloth sheen, direct lights - plus narrow windows keep the per-sample core), and the 2x2 supersample resolve runs the sample-block sums through a separable bilinear tent kernel over the 3x3 block neighborhood by default (`renderer.headless_resolve_tent` - the box-average rounding path stays for `false` and computes its exact rounding average through a widened SIMD pass); both paths are guarded to the platform SIMD level and fall back to the scalar forms.

Headless parallel rasterization has three execution structures:

- **Default per-draw - band/triangle-range split.** Big draws split their triangle range across workers; few-triangle wide-coverage draws (floor, sky) split their projected row extent into bands. Workers share the framebuffer, so depth/colour writes take a per-pixel-group spin lock. A submission rejected by the scheduler (stopping, or the outstanding bound hit) runs inline on the calling thread instead of silently dropping that band/range's geometry.
- **Per-draw tile-based split** (`[renderer] headless_tile_parallel`, default adaptive: on from four logical processors). One draw's triangles are binned into screen tiles of `[renderer] headless_tile_pixels` edge (default 64) and each worker owns disjoint tiles exclusively, so no per-pixel locks exist. The binning pass (transform/clip/project of every triangle on the calling thread) makes this slower than the band/range split on low-core machines (measured ~+55% frame time on a 2-core host), which is why the default follows the processor count. Tile grids and flattened record lists use 32-bit indices, with allocation counts also bounded by the host's representable vector size; dimensions beyond the signed raster-coordinate range or bins beyond either limit use the sequential path. Triggers mirror the default paths exactly.
- **Batched tile pass for whole draw lists** (`[renderer] headless_batched_tile_rasterization`, default on). `drawMeshes` hands its whole command list to `drawMeshBatch`, which prepares every item, bins all triangles in submission order and rasterizes the tiles across every worker (two or more). Per-tile record lists preserve submission order, so per-pixel triangle order - and therefore the image - equals the sequential per-draw replay; chunks of records keep the scratch bounded on very large frames, and items or situations the batch cannot take (single worker, oversized single draw, allocation failure) fall back to the per-draw paths with identical output. This is the structure that parallelizes many-small-draw scenes: no single draw reaches the per-draw thresholds, but the frame as a whole is large. Draw lists below two items, and the fur/overlay paths, always use the per-draw structures.
- **16-wide AVX-512 shading runs** (`[renderer] headless_wide_runs`, default on; see `wide_shading.hpp`). Run-eligible records shade sixteen horizontally consecutive samples per pass: coverage, depth test, perspective-correct interpolation, the normal-map chain, sun/sky shadow visibility (gather-based) and the Cook-Torrance math all run lane-wise, mirroring the 4-wide run kernel operation for operation so the wide path returns the same colors. The final partial group of a narrow window masks its excess lanes, so narrow records (the small streamed triangles that dominate a planet chunk mesh) join the same amortized path. Compiles only with the AVX-512 foundation macros and gates on the running CPU's support at construction; every record the wide kernel does not model keeps the 4-wide runs or the per-sample core.
- **16-wide AVX-512 presentation passes** (the same gate; see `wide_presentation.hpp`). The endFrame passes the frame pays after the draw run through lane-wise kernels too: the tent resolve's horizontal and vertical folds (sixteen channel lanes per pass, 16-bit lanes carrying the block sums), the bloom blur's interior cells and its full-res add-back (table lookups through gathers, the bilinear weights per lane), and the camera response (decode gather, contrast slope, vignette and grain gains, Bayer dither, encode gather). Each kernel keeps its scalar loop's operand order and rounding, so a wide lane writes the byte the scalar loop would write for that pixel, and the scalar loops stay as the portable fallback for hosts without the foundation or the runtime support.

### Mesh and texture resource model

`createMesh` first runs `prepareMeshData` (render_data.cpp), which validates and normalizes the description into the module's canonical vertex: 11 interleaved floats (position, normal, color, uv; 44-byte stride) and UInt32 indices. Strided attribute-list input is repacked per semantic, missing normals are generated from triangle geometry, unaligned or 16-bit index data is converted, and already-canonical input is borrowed instead of copied. Every index is range-checked; malformed input fails with `RenderError::InvalidMesh` and never reaches a backend.

`createTexture` runs `prepareTextureData`, which converts any supported pixel format/origin into tightly packed top-left RGBA8 (the canonical case is passed through). `createTextureFromFile` additionally recognizes KTX 2 containers: Basis-encoded files (BasisLZ/ETC1S and UASTC, with or without zstd supercompression) are transcoded once at load into the best block format the machine samples natively (BC7, falling back to BC3/BC1, with BC4/BC5 for one- and two-channel data), raw block-compressed BC1/BC3/BC4/BC5/BC7 files are consumed as-is, and uncompressed RGBA8 files flow through the regular RGBA path; the JPG/PNG route through stb_image is unchanged. Block payloads ride a `BlockTextureDescription` through `RenderBackend::createBlockTexture`: the Vulkan backend uploads every mip level's blocks in one command buffer (the chain comes from the file, so no GPU blit pass runs) after `vkGetPhysicalDeviceFormatProperties` verified the device samples that format optimally tiled and linearly filterable, and the headless backend software-decodes each level with bcdec into the same RGBA8 + float-RGB mip layout `createTexture` builds. When no block format is available the loader transcodes to RGBA32 instead, so every conforming file renders on every machine; KTX 1 files, exotic variants and dimensions above 16384 are rejected with a diagnostic. The module's texture pipeline is byte-linear - block payloads are uploaded through UNORM format variants so the sampling matches the RGBA8 path byte for byte - and `textureMean` decodes the level-0 blocks the same way to keep the mean faithful. The Vulkan backend uploads uncompressed textures via the async staging queue, generates the full mip chain by linear blits (when `[renderer] texture_mipmaps` is on and the device supports RGBA8 blitting), and creates a sampler from the requested filter/address/anisotropy; the headless backend keeps a float RGB copy with box-filtered CPU mips for the RGBA path. Screen-mapped fragments on the headless path select their mip from the perspective-correct UV derivatives (the larger screen-axis footprint picks the level, and the floor/ceil level pair blends bilinearly across the band - the CPU twin of the GPU sampler's linear-mipmap-linear filtering), so minified surfaces read as smooth filtering instead of base-level shimmer. A material without a map samples the neutral 1x1 stand-ins (white, or the flat normal (0.5, 0.5, 1.0)). The Vulkan backend defaults to 256 textures and 256 material descriptor sets per renderer; both ceilings are configurable from 1 to 65536. The persistent pipeline-cache disk budget defaults to 64 MiB and accepts 0 to disable disk loading/saving while retaining the in-memory Vulkan cache.

### Material model

`MeshMaterial` (shared_types) carries factors only: base color, metallic/ roughness factors, normal scale, occlusion strength, emissive color, the `unlit` flag, six texture handles (base color, normal, metallic-roughness, occlusion, emissive, and the near-field relief overlay's detail normal), and the appearance extensions - the distance-fog triple, the procedural mottle pair, and the detail overlay's fade and strength (see `Distance fog`). On Vulkan the core factors ride in the 112-byte push constant block, the extension fields ride in the set-2 material extension uniform buffer (interned per distinct tuple, bound at a dynamic offset), and descriptor sets are interned per texture combination, so equal materials share one set and bind-state caching skips redundant binds. The CPU rasterizer runs the same Cook-Torrance shading formulas as `shaders/mesh.frag` (same constants, same clamps - including the analytic environment specular and its reflection-field handoff), so both backends produce the same image.

### Backend selection

At `createRenderer`, the configured backend decides:

- `backend = "auto"` (default): a window surface with `SurfaceApi::Vulkan` takes the Vulkan backend (requires `window.vulkan` and a Vulkan-enabled build); any other surface takes the headless rasterizer.
- `backend = "vulkan"`: always Vulkan; fails with `BackendUnavailable` if the surface API is not Vulkan, `window.vulkan` is missing, or the module was built without the Vulkan backend.
- `backend = "headless"`: always the CPU rasterizer (a Vulkan surface cannot be drawn into headlessly and reports `BackendUnavailable`).

The `renderer.device` key additionally biases Vulkan physical-device selection (discrete/integrated); the headless backend always reports `DeviceClass::Cpu`.

## Threading and lifetime

- **Main thread only.** Every capability entry point, both backends, and the draw-sort/batch helpers are main-thread constructs. There is no internal locking around renderer lookup; the only mutex (the Vulkan device context's queue mutex) serializes Vulkan queue submission, upload bookkeeping, and wait-idle.
- **Frame discipline.** Draw calls must sit between a matching `beginFrame`/`endFrame` pair; the Vulkan backend drops draws issued outside a frame (`frameActive_`). Resource creation/destruction may happen any time.
- **GPU sync model (Vulkan).** N frame slots (`[renderer] max_frames_in_flight`, clamped 1..8) each own a command pool/buffer, an image-available semaphore, and a fence; `waitForFrameSlot` waits the slot's previous submission before re-recording. Swapchain images additionally track an "images in flight" fence so an image acquired early is not rendered into while still presenting (fence path). The timeline semaphore interface exists but is currently left uncreated, so all sync uses fences. Uploads run on one-shot command buffers, each fenced, capped at `[renderer] max_pending_uploads` pending copies; their staging memory comes from a persistently mapped, fence-recycled pool (blocks up to 16 MiB, 64 MiB total; oversized or pool-pressured uploads keep a one-off buffer). Mesh/texture destruction during a frame is deferred to the resource's frame slot and drained by the next `beginFrame` on that slot. PPM screenshots are likewise deferred: the copy rides the frame's command buffer, and the file write waits only for the recording slot's retirement (immediately at the next `beginFrame` of that slot; explicit flushes on a second request, swapchain recreation, and shutdown).
- **Handle lifetime.** Renderer destruction destroys the backend (Vulkan: device idle, then scratch, swapchain, resources, frames, pipeline, device - in that order). The strings returned by `RendererInfoStrings` are owned by the module and die with the renderer; the Vulkan handles from `VulkanDeviceHandles` are borrowed and must not be destroyed by the caller.

## VR support

The renderer currently supports one window surface and one camera view per renderer frame. It does not create OpenXR instances/sessions, acquire or release OpenXR swapchain images, submit stereo projection views, or consume headset/controller tracking poses. The project therefore does not currently support VR rendering. Adding an OpenXR runtime module alone is insufficient: the Vulkan renderer needs an OpenXR graphics binding, per-eye swapchain image targets, stereo view/projection handling, and frame timing/submit integration.

## Configuration

All `[renderer]` configuration keys, their validated ranges, and defaults are documented in `docs/configuration.md`.

## Integration

Events:

- **Subscribed:** `ConfigurationReloadedEvent` (kernel event bus). When any changed key starts with `renderer.`, the module re-reads its configuration; draw-path settings apply on the next frame, backend/device settings apply to renderers created afterwards.
- **Published:** none.
- **Permissions:** none declared.

Minimal consumer sketch (real signatures from `capabilities.hpp`):

```cpp
auto &renderer = context.acquireCapability<renderer::RenderCapability>();
// same pattern for renderer::VulkanRenderCapability / renderer::HeadlessRenderCapability

renderer::RenderError error = renderer::RenderError::None;
renderer::RendererHandle renderer = renderer.createRenderer(renderer.self, windowHandle, &error);

renderer::MeshHandle mesh = renderer.createMesh(renderer.self, renderer, &meshDescription, &error);
renderer::TextureHandle tex = renderer.createTextureFromFile(renderer.self, renderer, "brick.png", &error);

renderer::CameraMatrices camera{}; // column-major view/projection + world-space eye
renderer.setViewport(renderer.self, renderer, width, height);
renderer.setCamera(renderer.self, renderer, &camera);

renderer::FrameParameters frame{};          // clearColor; nullptr = configured clear_color
renderer.beginFrame(renderer.self, renderer, &frame);
std::vector<renderer::RenderDrawCommand> commands = {
    {mesh, modelMatrix0, nullptr},        // nullptr material = default material
    {mesh, modelMatrix1, &material},
};
renderer.drawMeshes(renderer.self, renderer, commands.data(), (std::uint32_t)commands.size());
renderer.drawMeshInstances(renderer.self, renderer, mesh, matrices, count, &material);
renderer.drawOverlay(renderer.self, renderer, &overlayData);   // UI triangles, see OverlayDrawData
renderer.endFrame(renderer.self, renderer);

renderer::FrameStatistics stats{}; // frame counters + last device-local budget sample/pressure
renderer.frameStatistics(renderer.self, renderer, &stats);
renderer.destroyRenderer(renderer.self, renderer);
```

Performance-sensitive consumers can additionally use `renderer::sortRenderDrawCommands` / `renderer::RenderInstanceBatchCache` (draw_sort.hpp / draw_batch.hpp) and the frame-budget controllers in `dynamic_resolution.hpp`, which are plain headers with no capability dependency.
