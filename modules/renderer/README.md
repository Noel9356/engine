# renderer module

The graphics backend and the owner of the frame. The renderer is a Vulkan 1.3 implementation (vulkan-hpp `vk::raii`, dynamic rendering, sync2) that rasterizes the shared `[scene]` world description into a G-buffer, dispatches the attached lighting technique modules against it, composites the radiance layers into the swapchain, copies captures, and presents. It exposes the `RendererCapability` (capability name `renderer`, contract in `contracts/include/engine/capabilities/renderer_contract.hpp`) and attaches technique modules through an in-tree interface that is deliberately NOT a capability contract (see below), so the kernel and script mods stay backend-agnostic while C++ technique modules get a typed, zero-overhead integration point.

## Contract

`RendererCapability` is intentionally small: `renderFrame()` renders one frame, `waitUntilIdle()` drains the device (used before teardown and before offscreen reads), `requestFrameCapture()` queues the capture path (the screenshot module's programmatic entry point), and `validationErrorsObserved()` reports whether the debug messenger saw a validation error — the flag the host's smoke mode turns into exit code 3. The contract carries no Vulkan vocabulary; the contract purity guard in `contracts/CMakeLists.txt` fails the configure if it ever does.

## The technique attach point

Lighting technique modules (`pbr`, `radiance_cascades`, `ray_reflections`) attach passes to the renderer at construct time through `modules/renderer/render_interface.hpp`. This header is in-tree and Vulkan-typed on purpose: it is shared by the renderer and the technique modules, both of which are graphics code, while the kernel and the script-facing surface never see it. Each technique declares which radiance layer it writes (direct diffuse, indirect diffuse, indirect specular), so the renderer creates exactly the offscreen targets that are consumed. Which techniques exist in a run is decided by capability demand: each technique provides its layer capability (`direct_diffuse`, `indirect_diffuse`, `indirect_specular`) and is activated exactly when an active mod demands that capability, with the composite order fixed (direct radiance before indirect) as `direct + albedo * (1 - metallic) * diffuseGI + specular + emissive`; no demanded layer (or no technique modules at all) renders unlit, and a demanded capability with no provider fails startup precisely instead of rendering a silent black screen. Per-technique settings live in `[lighting.<technique>]` sections; see each technique module's README.

## Frame pipeline

One command buffer per frame, in order: geometry pass, technique dispatches (direct, then indirect), composite into the swapchain image, capture copy, present.

- **Geometry pass (G-buffer).** Albedo + roughness as RGBA8, normal + metallic as RGBA16F, emissive as RGBA16F, depth as D32. Scene objects come from the shared `[scene]` configuration (`scene_description.cpp`); `plane` is authoring sugar for a thin box so rasterized and traced geometry stay the same primitive.
- **Technique dispatches.** A compute-to-compute memory barrier is emitted between technique records — one technique's storage-image output is the next technique's sampled input, and unordered access here races passes against each other.
- **Composite.** Blends the radiance layers with the material textures, applies `[renderer].exposure` and the `[renderer].tonemap` operator (`aces`, `reinhard`, `none`), and writes the sky for pixels that miss all geometry (one shared sky model in `assets`-independent shader code — `environment_common.glsl`, also consumed by reflection misses).
- **Capture copy and present.** The capture copy is recorded inside the same command buffer and synchronized with the frame fence — the screenshot module never races the compositor.

All offscreen images live in GENERAL layout permanently (written as attachments, read as sampled/storage images), so the frame needs access barriers only, no layout churn. The material texture arrays rest in SHADER_READ_ONLY_OPTIMAL for their lifetime. Semaphore ownership follows the two-scope policy documented in `frame_sync_policy.hpp`: acquire semaphores are owned per frame slot, render-finished (present) semaphores per swapchain image — frame-scoped present ownership raced on NVIDIA/Windows while software rasterizers masked it, so `frame_sync_policy.hpp` encodes the policy in one place. Frame EXECUTION is serialized on the host: transient targets and the host-written uniforms are single-buffered, so each frame waits for both in-flight fences before any uniform write or command recording — barriers order accesses within a submission, only fences order submissions.

## Materials and textures

Scene objects reference PBR texture sets: `[scene.objects].texture` names a folder under `[renderer].texture_directory` as `<name>/` with `Color.png`, `NormalGL.png`, `Roughness.png` and optional `Metalness.png`. The engine ships no content: the directory default comes from the content package (`mods/test_scene/config.toml` points it at the package's own `assets/textures/`, CC0 sets from ambientCG), and a scene that names texture sets with no directory configured fails with a precise error. The renderer uploads the referenced sets as two 2D texture arrays with a full mip chain — albedo in sRGB, data packed as R roughness / G metallic / B,A normal XY — plus a per-primitive material table. The normal map's X and Y are decoded from the RED and GREEN channels of `NormalGL.png` respectively; decoding both from a single-component load returned red twice and collapsed every normal onto the +X+Y diagonal.

UVs and tangent frames come from ONE shared parameterization (`scene_common.glsl`) consumed by the geometry pass and every SDF tracing consumer, so a GI bounce or a reflection ray landing on a point samples exactly the texels the camera sees; `uv_scale` is world meters per texture tile. The unit box mesh spans +-1 and the model matrix scales by the primitive's HALF EXTENTS, so a scaled unit box occupies exactly the box the analytic SDF describes — a half-unit mesh would render a half-size room inside the SDF's closed room: sky through the walls, GI, shadows and reflections all disagreeing. Under ROTATION the agreement is equally exact: the SDF converts world points to primitive-local space through `worldToLocalY`, the inverse of the model matrix's rotation; the forward rotation applied there would trace every rotated box as a mirrored ghost at `-rotationY` — invisible at rotation 0, which is why rotation scenes are the quick manual check.

The upload paths for the texture arrays are deliberately boring — every upload names its explicit array layer, and the texel channels are filled per plane rather than through one interleaved fill — because a bulk `std::fill_n` across interleaved channels silently corrupts texels; a textured scene renders the difference immediately.

## Camera

The renderer integrates the free-fly controller (`camera_controller.hpp`) every frame from the `InputCapability` when present: the configured `[scene.camera]` pose is the STARTING pose, then WASD flies, the mouse looks while the cursor is captured (Esc releases, F or left-click re-captures), Space/E rise, Ctrl/Q sink, Shift boosts, and the wheel scales the fly speed; camera state is re-uploaded per frame. The input capability is acquired OPTIONALLY (declared permission, caught absence), so minimal builds without the input module keep a static camera. `free_fly = false` pins the camera to the configured pose. Camera keys and their meanings live in the `[scene]` section of `docs/configuration.md`.

## Configuration

| Key | Type | Default | Meaning |
|---|---|---|---|
| `enable_validation` | bool | `true` | Vulkan validation layers plus debug messenger. Keep interactive runs at `false`: the layer taxes every dispatch heavily on the CPU, and the host loop polls window events only between frames — multi-second frames starve the message pump and the OS marks the window "not responding". Automated runs keep it `true` so VUID violations fail through exit code 3 |
| `fail_on_validation_error` | bool | `false` | abort the frame loop with a diagnostic at the first observed validation error instead of continuing (development runs; the host's smoke mode surfaces the same condition through its exit code without this key) |
| `vsync` | bool | `true` | present mode selection |
| `prefer_discrete_device` | bool | `true` | device selection: `true` scores discrete GPUs highest, `false` prefers integrated GPUs; CPU/software devices always score lowest |
| `max_frames_in_flight` | integer | `2` | frame slots (command buffers, fences, acquire semaphores). Frames do NOT overlap on the GPU: transient targets and host-written uniforms are single-buffered, so the loop serializes execution by waiting both fences before recording; the second slot only decouples recording from the presentation engine |
| `shader_directory` | string | `"shaders"` | where compiled SPIR-V is loaded from (resolved relative to the executable; module build output lands in `runtime/shaders`) |
| `texture_directory` | string | (empty) | where scene texture sets are loaded from; shipped empty because content belongs to packages — the content mod's defaults provide it, and a scene referencing texture sets without a directory fails precisely |
| `exposure` | float | `1.0` | HDR exposure applied in the composite pass |
| `tonemap` | string | `"aces"` | tonemap operator: `aces`, `reinhard`, `none` |

Vulkan 1.3 features are enabled explicitly, not assumed; the graphics pipeline creation path asserts every stage it configured actually exists, because a silently dropped vertex stage produces an empty pipeline. A failing shader compile is a hard startup error — a silent failure would keep the previous frame's visuals on screen.

## Files

One state struct, several translation units split by concern (the module-private `renderer_state.hpp` is the only place they meet):

- `renderer.hpp` — the module interface (contracts, technique attach point)
- `renderer_state.hpp` — module-private shared state: named constants, GPU-side structs, `RendererState` with every method declaration
- `renderer.cpp` — the module lifecycle: construct/start/stop, capability pointer, technique attach and ordering, frame event publish
- `renderer_device.cpp` — instance, physical-device selection, device, surface, swapchain, frame images and layout transitions
- `renderer_scene.cpp` — scene geometry upload, texture-array loading/mip chain, environment and camera uniform buffers
- `renderer_pipelines.cpp` — scene and composite pipelines with their descriptor sets
- `renderer_frames.cpp` — command/sync objects, capture buffers, per-frame command recording (barriers, geometry pass, techniques, composite) and the frame loop phases
- `render_interface.hpp` — the technique attach point (in-tree, Vulkan-typed)
- `camera_controller.hpp` — the free-fly camera controller (consumer of `InputCapability`)
- `frame_sync_policy.hpp` — the semaphore ownership policy for the render loop
- `vulkan_utilities.hpp` — Vulkan helpers shared with the lighting technique modules (shader bytes, memory type, samplers, mapped buffers)
- `scene_description.hpp` / `scene_description.cpp` — `[scene]` parsing into primitives, materials, lights
- `scene_math.hpp` — shared scene-side math (SDF, transforms)
- `light_uniform.hpp` — light upload structures shared with the lighting modules
- `shaders/` — GLSL sources: `scene.vert`/`scene.frag`, `scene_composite.vert`/`scene_composite.frag`, `common/` (`scene_common.glsl`, `environment_common.glsl`, `lighting_common.glsl`, `materials_common.glsl`, `relight_common.glsl`); compiled via the shared `engine_compile_module_shaders` CMake helper into `runtime/shaders/`
- `module_plugin.cpp` — the static plugin descriptor (name, factory, provided capabilities)
- `CMakeLists.txt` — target and option wiring (`ENGINE_BUILD_MODULE_RENDERER`)
