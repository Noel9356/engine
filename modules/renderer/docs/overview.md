# renderer module

The renderer module turns engine scene state into pixels. It provides a backend-agnostic `renderer` capability - renderers, meshes, textures, camera state, overlay drawing - and ships two backends behind that interface: a full Vulkan backend (device, swapchain, frame scheduling, PBR pipelines, staging uploads) and a headless software rasterizer that draws on the CPU and blits into the window surface. The capability layer in `renderer.cpp` orchestrates the module; its settings model (`renderer_settings.cpp`), version-gated lighting input feeds (`frame_inputs.cpp`), draw-list scheduler (`draw_scheduler.cpp`) and resource normalization (`render_data.cpp`) are the four concerns it routes to; the backends only implement `RenderBackend`.

The module keeps no world-space scene state of its own: draws are per-frame command lists (borrowed pointers, valid for the call), and every lighting input it consumes is a bounded, camera-relative snapshot - the lumen_gi probe grid is a camera-centered clipmap the provider scrolls toroidally, the reflection field is screen-space, and the light set is a flat array. A renderer therefore places no upper bound on the world it draws; scenes orders of magnitude larger than any loading-screen-free budget (planetary scales and beyond) stream their geometry through the same frame command list, and moving or reshaping objects simply submit different matrices and meshes each frame.

## Capability

| Capability | ABI | Header |
| --- | --- | --- |
| `renderer` | `renderCapabilityAbi = 13` | `modules/renderer/include/engine/renderer/capabilities.hpp` |
| `renderer.vulkan` (secondary) | `renderCapabilityAbi = 13` (same constant) | same header |
| `renderer.headless` (secondary) | `renderCapabilityAbi = 13` (same constant) | same header |

The shared data types the capability references live in `shared_types/include/engine/renderer/` (`mesh.hpp`, `material.hpp`, `device_profile.hpp`) and in `modules/renderer/include/engine/renderer/` (`draw_sort.hpp`, `draw_batch.hpp`, `dynamic_resolution.hpp`).

Handles are module-assigned ids starting at 1 (`RendererHandle`, `MeshHandle`, `TextureHandle`); `0` (`invalidRenderer`, `invalidMesh`, `invalidTexture`) always means "none", handles are never reused, and mesh and texture ids are global across all renderers of the module. All capability calls take the renderer handle first; an invalid handle is either ignored or reported through the optional `RenderError*` out-parameter.

## Provided / required / optional capabilities

From `module.toml` (and identical in `makeManifest()` and in the module entry table at the bottom of `renderer.cpp`):

| Kind | Capabilities |
| --- | --- |
| Provided | `renderer`, `renderer.vulkan`, `renderer.headless` |
| Required | `window` |
| Optional | `window.vulkan`, `pbr`, `direct_lighting`, `lumen_gi`, `lumen_reflection` |

`window` supplies the surface a renderer draws into; `window.vulkan` (`VulkanSurfaceCapability`) supplies the native handles and instance extensions the Vulkan backend needs; `pbr` supplies the sun/ambient/tone-mapping environment (with a version counter so updates are pushed once) and `direct_lighting` supplies point/spot/directional lights; when absent, a built-in default environment is used and no direct lights are set. `lumen_gi` supplies the world-space radiance probe grid for the indirect-lighting hook (see below); when absent, the legacy hemispheric ambient is used unchanged. `lumen_reflection` supplies the screen-space reflection field for the indirect-specular hook; when absent, the pre-hook specular is kept. The entry table declares no permissions (`declaredPermissions = nullptr`).

Because the renderer now acquires six capabilities in total, the kernel's trust gate classifies it as "broad"; the engine's own application configs trust it by name under `[trust]` (`trustModules = ["renderer"]`).

## lumen_gi probe-grid indirect lighting (ABI 12 hook)

When the optional `lumen_gi` capability is present and `[renderer] gi_indirect_lighting` is true (the default), `beginFrame` polls `copyProbes` for the module's probe-grid version and, on change, copies the snapshot and pushes it to the backend via `RenderBackend::setProbeGrid` - the same version-gated pattern the pbr environment and direct lights use. Both backends turn the snapshot into an indirect term that replaces the legacy hemispheric ambient (both are diffuse integrals, so both scale by the material's `(1 - metallic)` factor, exactly like the direct-lighting diffuse - metal surfaces carry almost no diffuse response, so chrome parts keep their mirror look instead of picking up a chalky ambient):

- **Headless (CPU):** `pbrShade` samples the grid per shaded pixel - trilinear over the camera-centered probe lattice (level 0 inside its half-cell-extended bounds, level 1 elsewhere with boundary clamping), cosine-lobed SH2 convolution per corner probe, irradiance renormalized over the footprint's valid probes (each weighted by its `validity`), and the result blended over the hemispheric ambient through a coverage ramp that reaches full GI at a quarter of valid coverage - probe lattices routinely lose the corners that fall inside nearby geometry, so the ambient remains the fallback only while almost no probe is valid. Unregistered or partially populated scenes therefore degrade gracefully to the legacy look instead of going black.
- **Vulkan (GPU):** `mesh.frag` implements the identical lattice walk, SH2 convolution, and validity-renormalized ambient ramp. The 64-byte grid header rides the frame uniform (`FrameData` grew from 2224 to 2288 bytes; the vertex-stage block is unchanged - it reads only the leading members), and the packed probe records (8 vec4 per probe: position.xyz + validity, then the 27 SH2 floats) live in a read-only storage buffer on set 0 / binding 1, refreshed per frame slot when the snapshot version changes. A zero-filled dummy buffer is bound while the hook is inactive, and the shader branches off the header's active flag, so a machine without the module renders the pre-hook image.

The snapshot can lag the latest `lumen_gi::evaluate()` by one frame: module update order does not guarantee lumen_gi runs before the renderer, and the cache is temporal by design (Lumen's updates are amortized the same way).

The demo `demos/lumen_gi/` is a standalone harness for this hook: it registers deterministic interior/outdoor scenes into both the renderer and lumen_gi and exports screenshots plus the module's own debug views for A/B validation (`[renderer] gi_indirect_lighting = false` shows the pre-hook image).

## lumen_reflection screen-space field hook

When the optional `lumen_reflection` capability is present and `[renderer] reflection_indirect_specular` is true (the default), `beginFrame` polls `copyField` for the module's field version and, on change, copies the snapshot and pushes it to the backend via `RenderBackend::setReflectionField` - the same version-gated pattern as the probe grid above. The field holds the expected GGX-lobe specular response per screen texel (see the module's overview.md), so the composite is a single add:

- **Headless (CPU):** both rasterization paths (the inline shading loop and the tile path) add the bilinear field sample of the shaded pixel's world position to the linear-space color right after the PBR pass, before emissive and tone mapping; the field is projected through its own camera, so pixels outside it read zero. An invalid/empty field keeps the pre-hook shading bit-identical. Unlit pixels and the overlay/fur paths carry no reflection term.
- **Vulkan (GPU):** the hook accepts the snapshot and the composite runs in the software rasterizer; the field data stays available through the module's `copyField` for consumers that shade on the GPU. See backends.md ("Behavior parity") for the scope note.

The snapshot can lag the latest `lumen_reflection::evaluate()` by one frame, exactly like the probe grid above.

The demo `demos/lumen_gi/` exercises this hook with its reflective surfaces (a polished-metal floor inlay, a chrome sphere, a brushed-metal wall panel indoors; a chrome sphere and a polished water-like slab outdoors): `--no-reflections` renders the same scene without the module for A/B validation. The demo always runs the module alongside lumen_gi in its default configuration and says so at startup.

Note: the `renderer.vulkan` entries only work when the module was built with `ENGINE_RENDERER_BACKEND_VULKAN=1` (set by CMake when Vulkan headers and a loader were found); otherwise they fail gracefully.

Vulkan swapchains require surface support for color-attachment usage. PPM screenshot readback is enabled when the surface also supports transfer-source usage; otherwise rendering remains available and screenshot requests report a creation failure.

## Transparent draws

A material with `opacity` below 1 renders as a transparent draw: depth testing stays on (opaque geometry in front keeps occluding it) while depth writes turn off, so co-transmitted surfaces do not occlude each other and the composite follows submission order exactly like the shell-fur stack. The composite has two modes per pixel. Where the lumen_reflection field carries a traced response for the pixel's material (the same gate the field sample applies everywhere), the shaded result commits directly: the field's response already contains the interface's reflected share plus the transmitted share refracted through the medium, so blending the raw background in as well would show the scene behind twice. Where no traced response exists - the surface sits outside the traced roughness band, or the reflection module is absent - the draw alpha blends with the material's opacity over the framebuffer (the classic transparent pass, unbent). Consumers order their transparent draws after the opaque scene, far to near within the transparent group, the same contract the fur stack documents. The surface's own diffuse terms (direct, hemispheric, and probe-grid ambient) scale by the opacity: a transparent surface scatters only the share it keeps off the transmitted path. The pbr module's material conversion maps glTF/MTL opacity and index of refraction into these fields, so imported transmissive materials flow through unchanged.

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
| `source/core/frame_inputs.hpp/.cpp` | `FrameInputs`: the version-gated pushes of the pbr environment, direct lights, lumen_gi probe grid and lumen_reflection field into a backend. |
| `source/core/draw_scheduler.hpp/.cpp` | `DrawScheduler`: draw-list sorting with the cached permutation, the backend batch submission, and the Vulkan instance fold. |
| `source/core/render_errors.hpp` | The optional-error out-parameter convention shared by the module's entry points and helpers. |
| `source/core/render_shared.hpp` | Umbrella for the focused shading headers plus the backend-neutral pieces: device/presentation enums, default environment, frustum rejection, sRGB/tone-map output encoders, and the PPM/error utilities. |
| `source/core/direct_light_shading.hpp` | `PreparedDirectLight` and `resolveDirectLight()` - the per-point light resolution shared by the PBR and fur passes. |
| `source/core/pbr_shading.hpp` | The CPU Cook-Torrance shading pass `pbrShade` (the reference twin of `shaders/mesh.frag`). |
| `source/core/fur_shading.hpp` | The shell-fur shading helpers and `furShade` (the twin of `shaders/fur.frag`). |
| `source/core/gi_probe_sampling.hpp` | The lumen_gi probe-grid snapshot types, prepared level descriptors, and the trilinear SH2 irradiance walk. |
| `source/core/reflection_field_sampling.hpp` | The lumen_reflection field snapshot types and the bilinear roughness-band-gated sample. |
| `source/core/shading_inline.hpp` | The hot-path inline marker shared by the per-pixel shading entries. |
| `source/backend/render_backend.hpp` | `RenderBackend` - the abstract interface both backends implement. |
| `source/core/render_data.hpp/.cpp` | Mesh/texture normalization: attribute repacking, index validation, normal generation, pixel format conversion. |
| `source/backend/headless/headless_types.hpp` | Headless backend data types: `HeadlessMesh` (planar SoA mesh copy with bounding sphere and LOD index chains) and `HeadlessTexture` (RGBA8 upload copy plus float-RGB mip chain) shared between the backend and its rasterizer. |
| `source/backend/headless/headless_backend.hpp/.cpp` | Headless backend: CPU rasterizer with culling, LOD, transform cache, and task-scheduler parallelism. |
| `source/backend/headless/tile_bins.hpp` | The tile-binning policy of the headless rasterizer's parallel paths (workers own disjoint tiles, so rasterization needs no per-pixel locks). |
| `source/backend/vulkan/vulkan_backend.hpp/.cpp` | Vulkan backend: frame lifecycle, command recording, instance streaming, deferred destruction. |
| `source/backend/vulkan/vulkan_device.hpp/.cpp` | `VulkanDeviceContext`: instance/surface/device/VMA bring-up, device profile, async staging-upload queue. |
| `source/backend/vulkan/staging_pool.hpp` | Fence-recycled staging-allocation policy driving the device context's upload queue. |
| `source/backend/vulkan/vulkan_frames.hpp/.cpp` | `VulkanFrameScheduler`: frames-in-flight slots, fences/semaphores, GPU timestamp queries. |
| `source/backend/vulkan/vulkan_pipeline.hpp/.cpp` | `VulkanPipeline`: render pass, descriptor layouts, PBR mesh pipeline, overlay pipeline, disk-backed pipeline cache. |
| `source/backend/vulkan/vulkan_resources.hpp/.cpp` | `VulkanResources`: frame uniform ring, mesh buffers, texture images/samplers, material descriptor-set cache with per-renderer capacity limits. |
| `source/backend/vulkan/vulkan_swapchain.hpp/.cpp` | `VulkanSwapchain`: format/present-mode selection, image views, framebuffers, recreation, PPM readback. |
| `shaders/mesh.vert`, `shaders/mesh.frag` | GLSL sources of the PBR pipeline (the fragment shader is the reference twin of the CPU shading in `source/core/pbr_shading.hpp`). |
| `shaders/ui.vert`, `shaders/ui.frag` | GLSL sources of the overlay pipeline. |
| `shaders/*_gen.hpp` | SPIR-V arrays included by `source/backend/vulkan/vulkan_pipeline.cpp`; regenerated from the GLSL by `engine_module_compile_shaders` at build time. |

## How it works

### Renderer lifecycle

A consumer creates a renderer per window surface and drives it frame by frame:

1. `createRenderer(surface, &error)` - resolves the backend (see below), constructs it, and returns a `RendererHandle`.
2. Optionally `createMesh`/`createTexture`/`createTextureFromFile` (also possible later).
3. Per frame: `setViewport`, `setCamera`, then `beginFrame(renderer, frame)`.
4. Record draws: `drawMesh`, `drawMeshes` (sorted/batched command list), `drawMeshInstances` (explicit instancing), `drawFur`/`drawFurInstances` (alpha-blended shell-fur layer stacks, ABI 13 - see the physics_fur module's overview), `drawOverlay` (UI triangles).
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

The headless backend's frame is the same shape minus the GPU: `beginFrame` (re)allocates and clears the CPU color/depth buffers, draws rasterize directly into them (LOD selection by angular size, frustum culling, a persistent per-mesh vertex transform cache, optional task-scheduler parallelism with per-pixel-group depth locks), and `endFrame` copies the result into the window surface's locked pixel buffer.

Headless parallel rasterization has three execution structures:

- **Default per-draw - band/triangle-range split.** Big draws split their triangle range across workers; few-triangle wide-coverage draws (floor, sky) split their projected row extent into bands. Workers share the framebuffer, so depth/colour writes take a per-pixel-group spin lock. A submission rejected by the scheduler (stopping, or the outstanding bound hit) runs inline on the calling thread instead of silently dropping that band/range's geometry.
- **Per-draw tile-based split** (`[renderer] headless_tile_parallel`, default adaptive: on from four logical processors). One draw's triangles are binned into screen tiles of `[renderer] headless_tile_pixels` edge (default 64) and each worker owns disjoint tiles exclusively, so no per-pixel locks exist. The binning pass (transform/clip/project of every triangle on the calling thread) makes this slower than the band/range split on low-core machines (measured ~+55% frame time on a 2-core host), which is why the default follows the processor count. Tile grids and flattened record lists use 32-bit indices, with allocation counts also bounded by the host's representable vector size; dimensions beyond the signed raster-coordinate range or bins beyond either limit use the sequential path. Triggers mirror the default paths exactly.
- **Batched tile pass for whole draw lists** (`[renderer] headless_batched_tile_rasterization`, default on). `drawMeshes` hands its whole command list to `drawMeshBatch`, which prepares every item, bins all triangles in submission order and rasterizes the tiles across every worker (two or more). Per-tile record lists preserve submission order, so per-pixel triangle order - and therefore the image - equals the sequential per-draw replay; chunks of records keep the scratch bounded on very large frames, and items or situations the batch cannot take (single worker, oversized single draw, allocation failure) fall back to the per-draw paths with identical output. This is the structure that parallelizes many-small-draw scenes: no single draw reaches the per-draw thresholds, but the frame as a whole is large. Draw lists below two items, and the fur/overlay paths, always use the per-draw structures.

### Mesh and texture resource model

`createMesh` first runs `prepareMeshData` (render_data.cpp), which validates and normalizes the description into the module's canonical vertex: 11 interleaved floats (position, normal, color, uv; 44-byte stride) and UInt32 indices. Strided attribute-list input is repacked per semantic, missing normals are generated from triangle geometry, unaligned or 16-bit index data is converted, and already-canonical input is borrowed instead of copied. Every index is range-checked; malformed input fails with `RenderError::InvalidMesh` and never reaches a backend.

`createTexture` runs `prepareTextureData`, which converts any supported pixel format/origin into tightly packed top-left RGBA8 (the canonical case is passed through). The Vulkan backend uploads via the async staging queue, generates the full mip chain by linear blits (when `[renderer] texture_mipmaps` is on and the device supports RGBA8 blitting), and creates a sampler from the requested filter/address/anisotropy; the headless backend keeps a float RGB copy with box-filtered CPU mips. A material without a map samples the neutral 1x1 stand-ins (white, or the flat normal (0.5, 0.5, 1.0)). The Vulkan backend defaults to 256 textures and 256 material descriptor sets per renderer; both ceilings are configurable from 1 to 65536. The persistent pipeline-cache disk budget defaults to 64 MiB and accepts 0 to disable disk loading/saving while retaining the in-memory Vulkan cache.

### Material model

`MeshMaterial` (shared_types) carries factors only: base color, metallic/ roughness factors, normal scale, occlusion strength, emissive color, the `unlit` flag, and five texture handles. On Vulkan the factors ride in the 112-byte push constant block, and descriptor sets are interned per texture combination, so equal materials share one set and bind-state caching skips redundant binds. The CPU rasterizer runs the same Cook-Torrance shading formulas as `shaders/mesh.frag` (same constants, same clamps), so both backends produce the same image.

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
