# renderer module

The renderer module turns engine scene state into pixels. It provides a backend-agnostic `renderer` capability - renderers, meshes, textures, camera state, overlay drawing - and ships two backends behind that interface: a full Vulkan backend (device, swapchain, frame scheduling, PBR pipelines, staging uploads) and a headless software rasterizer that draws on the CPU and blits into the window surface. The capability layer in `renderer.cpp` owns configuration, resource normalization, draw sorting/batching, and environment integration; the backends only implement `RenderBackend`.

## Capability

| Capability | ABI | Header |
| --- | --- | --- |
| `renderer` | `renderCapabilityAbi = 11` | `modules/renderer/include/engine/renderer/capabilities.hpp` |
| `renderer.vulkan` (secondary) | `renderCapabilityAbi = 11` (same constant) | same header |
| `renderer.headless` (secondary) | `renderCapabilityAbi = 11` (same constant) | same header |

The shared data types the capability references live in `shared_types/include/engine/renderer/` (`mesh.hpp`, `material.hpp`, `device_profile.hpp`) and in `modules/renderer/include/engine/renderer/` (`draw_sort.hpp`, `draw_batch.hpp`, `dynamic_resolution.hpp`).

Handles are module-assigned ids starting at 1 (`RendererHandle`, `MeshHandle`, `TextureHandle`); `0` (`invalidRenderer`, `invalidMesh`, `invalidTexture`) always means "none", handles are never reused, and mesh and texture ids are global across all renderers of the module. All capability calls take the renderer handle first; an invalid handle is either ignored or reported through the optional `RenderError*` out-parameter.

## Provided / required / optional capabilities

From `module.toml` (and identical in `makeManifest()` and in the module entry table at the bottom of `renderer.cpp`):

| Kind | Capabilities |
| --- | --- |
| Provided | `renderer`, `renderer.vulkan`, `renderer.headless` |
| Required | `window` |
| Optional | `window.vulkan`, `pbr`, `direct_lighting` |

`window` supplies the surface a renderer draws into; `window.vulkan` (`VulkanSurfaceCapability`) supplies the native handles and instance extensions the Vulkan backend needs; `pbr` supplies the sun/ambient/tone- mapping environment (with a version counter so updates are pushed once) and `direct_lighting` supplies point/spot/directional lights; when absent, a built-in default environment is used and no direct lights are set. The entry table declares no permissions (`declaredPermissions = nullptr`).

Note: the `renderer.vulkan` entries only work when the module was built with `ENGINE_RENDER_BACKEND_VULKAN=1` (set by CMake when Vulkan headers and a loader were found); otherwise they fail gracefully.

## Architecture

| File | Role |
| --- | --- |
| `module.toml` / `config.toml` | Module descriptor (capabilities + declared config keys) and default values for the `[renderer]` section. |
| `include/engine/renderer/capabilities.hpp` | The public capability structs, handles, and per-draw/per-frame structs. |
| `include/engine/renderer/draw_sort.hpp` | Total-order material comparison and draw-command sorting (sort modes for batching). |
| `include/engine/renderer/draw_batch.hpp` | `RenderInstanceBatchCache`: folds draw commands into instanced batches plus a packed instance-matrix stream. |
| `include/engine/renderer/dynamic_resolution.hpp` | `DynamicResolutionController` (render-scale stepping) and `AdaptiveQualityController` (quality-preset stepping) fed from frame timings. |
| `source/renderer.cpp` | Module entry: `RenderModule` (ModuleInterface), capability binding, configuration, backend selection, draw sorting/batching, entry table. |
| `source/render_backend.hpp` | `RenderBackend` - the abstract interface both backends implement. |
| `source/render_shared.hpp` | Shared PBR math (must match the Vulkan shader), default environment, tone mapping, PPM writer, device preference enum. |
| `source/render_data.hpp/.cpp` | Mesh/texture normalization: attribute repacking, index validation, normal generation, pixel format conversion. |
| `source/headless_backend.hpp/.cpp` | Headless backend: CPU rasterizer with culling, LOD, transform cache, and task-scheduler parallelism. |
| `source/vulkan_backend.hpp/.cpp` | Vulkan backend: frame lifecycle, command recording, instance streaming, deferred destruction. |
| `source/vulkan_device.hpp/.cpp` | `VulkanDeviceContext`: instance/surface/device/VMA bring-up, device profile, async staging-upload queue. |
| `source/vulkan_frames.hpp/.cpp` | `VulkanFrameScheduler`: frames-in-flight slots, fences/semaphores, GPU timestamp queries. |
| `source/vulkan_pipeline.hpp/.cpp` | `VulkanPipeline`: render pass, descriptor layouts, PBR mesh pipeline, overlay pipeline, disk-backed pipeline cache. |
| `source/vulkan_resources.hpp/.cpp` | `VulkanResources`: frame uniform ring, mesh buffers, texture images/samplers, material descriptor-set cache with per-renderer capacity limits. |
| `source/vulkan_swapchain.hpp/.cpp` | `VulkanSwapchain`: format/present-mode selection, image views, framebuffers, recreation, PPM readback. |
| `shaders/mesh.vert`, `shaders/mesh.frag` | GLSL sources of the PBR pipeline (the fragment shader is the reference twin of the CPU shading in `render_shared.hpp`). |
| `shaders/ui.vert`, `shaders/ui.frag` | GLSL sources of the overlay pipeline. |
| `shaders/*_gen.hpp` | SPIR-V arrays included by `vulkan_pipeline.cpp`; regenerated from the GLSL by `engine_module_compile_shaders` at build time. |

## How it works

### Renderer lifecycle

A consumer creates a renderer per window surface and drives it frame by frame:

1. `createRenderer(surface, &error)` - resolves the backend (see below), constructs it, and returns a `RendererHandle`.
2. Optionally `createMesh`/`createTexture`/`createTextureFromFile` (also possible later).
3. Per frame: `setViewport`, `setCamera`, then `beginFrame(renderer, frame)`.
4. Record draws: `drawMesh`, `drawMeshes` (sorted/batched command list), `drawMeshInstances` (explicit instancing), `drawOverlay` (UI triangles).
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

The headless backend's frame is the same shape minus the GPU: `beginFrame` (re)allocates and clears the CPU color/depth buffers, draws rasterize directly into them (LOD selection by angular size, frustum culling, a generation-stamped vertex transform cache, optional task-scheduler parallelism with per-pixel-group depth locks), and `endFrame` copies the result into the window surface's locked pixel buffer.

Headless parallel rasterization has two execution structures, selected per draw:

- **Default - band/triangle-range split.** Big draws split their triangle range across workers; few-triangle wide-coverage draws (floor, sky) split their projected row extent into bands. Workers share the framebuffer, so depth/colour writes take a per-pixel-group spin lock. A submission rejected by the scheduler (stopping, or the outstanding bound hit) runs inline on the calling thread instead of silently dropping that band/range's geometry.
- **Opt-in - tile-based split** (`[renderer] headless_tile_parallel`, default `false`). Triangles are binned into 64x64 screen tiles and each worker owns disjoint tiles exclusively, so no per-pixel locks exist. The binning pass (transform/clip/project of every triangle on the calling thread) makes this slower than the default split on low-core machines (measured ~+55% frame time on a 2-core host); it targets many-core hosts where per-pixel lock contention dominates. Triggers mirror the default paths exactly.

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
