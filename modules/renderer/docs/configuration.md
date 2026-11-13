# renderer configuration

All keys live in the `[renderer]` section. Default values are in `modules/renderer/config.toml`; `module.toml` `config_keys` documents the section's keys, but the current kernel does not validate module sections against it (only engine sections and plugin manifests are validated). Values are read in `RenderModule::readConfiguration()` (renderer.cpp), re-read on every `ConfigurationReloadedEvent` that touches a `renderer.*` key. Keys are listed alphabetically.

## Keys

| Key | Type | Valid range (as validated in the module) | Default (config.toml) | Effect |
| --- | --- | --- | --- | --- |
| `renderer.anisotropy_level` | float | clamped to 1..16 | `1.0` | Vulkan sampler anisotropy. Clamped again against `limits.maxSamplerAnisotropy` at sampler creation; 1 disables anisotropic filtering. Read at renderer creation. |
| `renderer.auto_config` | bool | - | `false` | Hardware-derived tuning. When `true`, every performance, quality and memory knob in `[renderer]` is derived from the detected hardware (GPU class and device-local memory via the cached Vulkan probe, logical processor count, installed RAM) and the other tuning keys in `[renderer]` have **no effect**; `clear_color` stays content-owned and is still honored. A one-line stderr notice reports the hardware inputs. Re-derived on every `renderer.*` reload, so toggling the key switches between derived and hand-set values on the next renderer/draw call. See "Auto config" below. |
| `renderer.back_face_culling` | bool | - | `true` | Vulkan pipeline culls back faces; headless uses its winding-probe heuristic. |
| `renderer.backend` | string | `auto`, `vulkan`, `headless` | `"auto"` | Backend selection policy for new renderers. `auto` is hardware-aware: Vulkan when the machine exposes a real GPU (discrete, integrated or virtual device with a graphics queue; software Vulkan implementations report a CPU device class and do not count) and the surface can present to Vulkan, headless otherwise. Errors are reported as-is; there is no silent fallback. |
| `renderer.clear_color` | float[4] | >= 4 components, each finite; clamped to 0..1 | `[0.0, 0.0, 0.0, 1.0]` | Clear color used when `beginFrame` is called with `frame == nullptr`. Content-owned: the one key `auto_config` never overrides. |
| `renderer.device` | string | `auto`, `discrete`, `integrated` | `"auto"` | Vulkan physical-device preference. |
| `renderer.draw_sort_min_commands` | integer | clamped to 2..65536 | `2` | Minimum command count before sorting is worth it. |
| `renderer.draw_sort_mode` | string | `mesh_material`, `material_mesh`, `off` | `"mesh_material"` | `drawMeshes` command sorting; any other value warns and disables sorting. |
| `renderer.frustum_culling` | bool | - | `true` | Sphere-vs-frustum rejection of meshes/instances (both backends). |
| `renderer.gi_indirect_lighting` | bool | - | `true` | Consume the optional `lumen_gi` capability's world-space radiance probe grid and replace the legacy hemispheric ambient with its validity-weighted SH2 indirect term (see the "lumen_gi probe-grid indirect lighting" section in overview.md). `false` - or the module being absent - keeps the pre-hook hemispheric ambient bit-identical. Read per frame (live-reloadable); the grid version is polled only while enabled. |
| `renderer.gpu_timestamps_enabled` | bool | - | `false` | Allocate the Vulkan timestamp query pool backing `FrameStatistics::gpuFrameMilliseconds`. |
| `renderer.headless_band_split_triangles` | integer | no range check (used as-is; 0 disables) | `4096` | Headless: draws with at most this many triangles but large screen coverage split the framebuffer by row bands instead of triangle ranges (parallelizes full-screen few-triangle draws such as the floor or sky dome). Read at renderer creation. |
| `renderer.headless_tile_parallel` | boolean | - | `true` | Headless: tile-based parallel rasterization (64x64 screen tiles, exclusive per-worker ownership, no per-pixel locks). Triggers mirror the default paths; the serial binning phase makes it slower than the default split on low-core machines (measured ~+55% frame time on 2 cores) - disable after the benchmark protocol shows per-pixel lock contention dominating on the target. Read at renderer creation. |
| `renderer.headless_transform_cache_mb` | integer | clamped to 0..1024 (MiB) | `32` | Budget for the headless backend's vertex transform cache; 0 disables the cache. |
| `renderer.instance_batch_min_count` | integer | clamped to 2..1024 | `4` | Runs smaller than this fall back to single draws. |
| `renderer.instance_batching` | bool | - | `true` | Fold equal mesh+material runs of `drawMeshes` into instanced draws (Vulkan backend only). |
| `renderer.instance_stream_mb` | integer | clamped to 1..64 (MiB) | `4` | Per-frame instance-matrix ring capacity (also clamped 1..64 at the backend). |
| `renderer.lod_enabled` | bool | - | `true` | Headless mesh LOD chains and angular-size LOD selection. |
| `renderer.lod_min_triangles` | integer | no range check (used as-is; the headless backend's LOD generation uses a fixed 4096*3 index threshold) | `4096` | Headless: meshes below this triangle count get no LOD chain. Read at renderer creation. |
| `renderer.max_frames_in_flight` | integer | clamped to 1..8 | `3` | Number of Vulkan frame slots (command buffers + fences). |
| `renderer.max_material_sets_per_renderer` | integer | clamped to 1..65536 | `256` | Vulkan: maximum number of cached material descriptor sets per renderer. The Vulkan descriptor pool is sized from this limit. |
| `renderer.max_pending_uploads` | integer | clamped to 1..4096 | `64` | Cap on in-flight staging upload submissions; the oldest is waited on when full. |
| `renderer.max_textures_per_renderer` | integer | clamped to 1..65536 | `256` | Vulkan: maximum number of textures held by each renderer. Rejection occurs when the cap is reached; textures are not evicted automatically. |
| `renderer.memory_budget_update_interval_frames` | integer | clamped to 0..3600 | `120` | Frames between VMA device-local budget/usage refreshes (0 disables). |
| `renderer.msaa_samples` | integer | clamped to 1..8 | `1` | Vulkan MSAA sample count. `1` (default) keeps the legacy render-pass path unchanged; values > 1 select the dynamic-rendering path with a multisample color attachment (`DONT_CARE` store + `RESOLVE_MODE_AVERAGE` resolve). Requires dynamic rendering support and color/depth formats that support the sample count; otherwise the renderer falls back to 1 sample with a one-time note. Read at renderer creation. |
| `renderer.parallel_triangle_threshold` | integer | no range check (used as-is) | `4096` | Headless: triangles per draw before splitting across task-scheduler workers. Read at renderer creation. |
| `renderer.pipeline_cache_mb` | integer | clamped to 0..512 MiB | `64` | Vulkan: maximum persistent pipeline-cache data loaded from or saved to disk. `0` disables disk persistence while keeping the in-memory Vulkan cache active. |
| `renderer.pixels_per_lock_group` | integer | no range check (used as-is) | `16` | Headless: pixels per depth lock group during parallel rasterization. Read at renderer creation. |
| `renderer.present_mode` | string | `auto`, `fifo`, `mailbox`, `immediate` | `"auto"` | Vulkan swapchain presentation-queue policy. `auto` keeps the historical vsync-derived choice (FIFO when vsync, else mailbox-fallback-immediate); `fifo` is always tear-free but blocks the present on the compositor; `mailbox` triple-buffers for the lowest-latency tear-free path; `immediate` never waits on the display (lowest latency, may tear). Read at renderer creation. |
| `renderer.texture_mipmaps` | bool | - | `true` | Generate full mip chains for textures (Vulkan: requires RGBA8 blit support, otherwise disabled with a warning). |
| `renderer.validation` | bool | - | `false` | Enable VK_LAYER_KHRONOS_validation + debug utils (only when the layer is installed; warns otherwise). |
| `renderer.vsync` | bool | - | `false` | Vulkan present mode: FIFO when on; mailbox (fallback immediate) when off. Off is the default for the lowest P99 frame time - Mailbox triple-buffers and never blocks the CPU on the compositor/vblank. |

The remaining legacy keys `srgb_output` and `z_prepass` are not read by the current module and have no effect (`srgb_output` has a hard-coded `false` default in the module; the swapchain still prefers an `_SRGB` surface format only when that flag is set, and the shader's `SRGB_OUTPUT` specialization constant follows the actually picked format). The active resource and persistent pipeline-cache limits are configurable through the keys documented above.

## Auto config

`renderer.auto_config = true` replaces the whole `[renderer]` overlay with a hardware-derived set (`applyHardwareDerivedConfiguration` in renderer.cpp). The derivation inputs are the cached GPU probe (`probeHostGpu` in vulkan_gpu_probe.cpp: device class, device-local heap bytes, sampler-anisotropy and timestamp feature bits), the logical processor count and the installed physical RAM (`engine/util/hardware_info.hpp`); unknown inputs fall back to the conservative end of each range. Derived values (all within the validated ranges above):

- `backend`: Vulkan when a real GPU is present, headless otherwise (identical to the `backend = "auto"` policy). `device`: `discrete`/`integrated` when the probe unambiguously found that class, `auto` otherwise.
- `vsync` on, `validation` off, `max_frames_in_flight` 2.
- `max_pending_uploads` 64 (16 when RAM is under 4 GiB or unknown); `headless_transform_cache_mb` 32 on >= 8 GiB machines, 16 below, 8 under 4 GiB.
- `max_textures_per_renderer` / `max_material_sets_per_renderer` 512 only for a discrete GPU with >= 6 GiB of device-local memory, else 256; `pipeline_cache_mb` 128 (discrete), 64 (other GPU), 32 (headless); `instance_stream_mb` 8 for big-VRAM discrete GPUs, else 4.
- `msaa_samples` 4 only for a discrete GPU with >= 6 GiB, else 1; `anisotropy_level` 8 (discrete) / 4 (integrated, virtual) when the device supports sampler anisotropy, else 1.
- `gpu_timestamps_enabled` only when a GPU exposes timestamp-capable queues.
- Sorting (`mesh_material`), instance batching (min 4), mipmaps, frustum/back-face culling and LOD on.
- Headless threading scales with the CPU: tile parallelism from 4 logical processors, parallel triangle thresholds 2048 on >= 8 processors (else 4096).

The manual ranges still apply as the outer bounds; auto_config never sets a value the hand-written path would have clamped.

## Hardware probing notes

The probe runs once per process on first use (a bare `VkInstance` without extensions, `vkEnumeratePhysicalDevices`, static properties only - no logical device, no surface). It is also what makes `backend = "auto"` hardware-aware. On a machine whose only Vulkan implementation is a CPU rasterizer (lavapipe, SwiftShader) the probe reports `gpuPresent == false`, so both `auto` and `auto_config` choose the engine's own headless CPU backend instead of routing frames through a software Vulkan driver. When the build has no Vulkan backend at all, the probe reports no GPU and every auto path deterministically picks headless.

## Out-of-range and malformed values

Behavior as implemented in `readConfiguration()` (renderer.cpp):

- Integer keys with a validated range are clamped into the range and a warning is printed to stderr, e.g. `renderer module: clamping [renderer] max_frames_in_flight 99 to range 1..8`. The clamped value is used.
- `clear_color` requires at least four components; a shorter array is silently ignored (no warning). If any component is non-finite the whole array is ignored with a warning; otherwise each component is clamped to 0..1.
- String enums (`backend`, `device`, `draw_sort_mode`) warn on unknown values and fall back (`auto` for backend/device, sorting disabled for draw_sort_mode). Unknown `draw_sort_mode` values never enable sorting.
- The headless tuning keys (`parallel_triangle_threshold`, `headless_band_split_triangles`, `pixels_per_lock_group`, `headless_tile_parallel`, `lod_min_triangles`) are read in `readConfiguration()` with `value_or(default)` and have no range validation - out-of-range values are passed through to the backend as-is. They still only take effect when a renderer is created.
- Every key in `config_keys` has a reader in the module; settings outside the keys above still do nothing (module sections are not known-key validated). The hard-coded values listed in the removed-keys note are the effective behavior for areas the module does not parameterize yet.

Note that the module's built-in defaults before configuration is applied deliberately differ from config.toml for opt-in features (sorting, batching, mipmaps start disabled in code); with the shipped config.toml the table above is what actually ends up active. `module.toml` `config_keys` documents the section's surface; settings outside these keys are not validated by the kernel (it validates only engine sections and plugin manifests).

Keys that affect a renderer's immutable objects (`backend`, `device`, `msaa_samples`, `anisotropy_level`, and the headless threading keys) are read at renderer creation; a reload changes what the *next* `createRenderer` sees, never a live renderer. See [overview.md](overview.md) for the lifecycle and [backends.md](backends.md) for the per-backend behavior.
