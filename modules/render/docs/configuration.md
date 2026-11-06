# render configuration

All keys live in the `[render]` section. Default values are in `modules/render/config.toml`; `module.toml` `config_keys` documents the section's keys, but the current kernel does not validate module sections against it (only engine sections and game-mod manifests are validated). Values are read in `RenderModule:: readConfiguration()` (render.cpp), re-read on every `ConfigurationReloadedEvent` that touches a `render.*` key.

## Keys

| Key | Type | Valid range (as validated in the module) | Default (config.toml) | Effect |
| --- | --- | --- | --- | --- |
| `render.backend` | string | `auto`, `vulkan`, `headless` | `"auto"` | Backend selection policy for new renderers; `auto` follows the window surface's API. |
| `render.device` | string | `auto`, `discrete`, `integrated` | `"auto"` | Vulkan physical-device preference. |
| `render.vsync` | bool | - | `true` | Vulkan present mode: FIFO when on; mailbox (fallback immediate) when off. |
| `render.validation` | bool | - | `false` | Enable VK_LAYER_KHRONOS_validation + debug utils (only when the layer is installed; warns otherwise). |
| `render.max_frames_in_flight` | integer | clamped to 1..8 | `2` | Number of Vulkan frame slots (command buffers + fences). |
| `render.max_pending_uploads` | integer | clamped to 1..4096 | `64` | Cap on in-flight staging upload submissions; the oldest is waited on when full. |
| `render.memory_budget_update_interval_frames` | integer | clamped to 0..3600 | `120` | Frames between VMA device-local budget/usage refreshes (0 disables). |
| `render.clear_color` | float[4] | >= 4 components, each finite; clamped to 0..1 | `[0.0, 0.0, 0.0, 1.0]` | Clear color used when `beginFrame` is called with `frame == nullptr`. |
| `render.headless_transform_cache_mb` | integer | clamped to 0..1024 (MiB) | `32` | Budget for the headless backend's vertex transform cache; 0 disables the cache. |
| `render.frustum_culling` | bool | - | `true` | Sphere-vs-frustum rejection of meshes/instances (both backends). |
| `render.back_face_culling` | bool | - | `true` | Vulkan pipeline culls back faces; headless uses its winding-probe heuristic. |
| `render.lod_enabled` | bool | - | `true` | Headless mesh LOD chains and angular-size LOD selection. |
| `render.gpu_timestamps_enabled` | bool | - | `false` | Allocate the Vulkan timestamp query pool backing `FrameStatistics::gpuFrameMilliseconds`. |
| `render.draw_sort_mode` | string | `mesh_material`, `material_mesh`, `off` | `"mesh_material"` | `drawMeshes` command sorting; any other value warns and disables sorting. |
| `render.draw_sort_min_commands` | integer | clamped to 2..65536 | `2` | Minimum command count before sorting is worth it. |
| `render.instance_batching` | bool | - | `true` | Fold equal mesh+material runs of `drawMeshes` into instanced draws (Vulkan backend only). |
| `render.instance_batch_min_count` | integer | clamped to 2..1024 | `4` | Runs smaller than this fall back to single draws. |
| `render.instance_stream_mb` | integer | clamped to 1..64 (MiB) | `4` | Per-frame instance-matrix ring capacity (also clamped 1..64 at the backend). |
| `render.texture_mipmaps` | bool | - | `true` | Generate full mip chains for textures (Vulkan: requires RGBA8 blit support, otherwise disabled with a warning). |
| `render.parallel_triangle_threshold` | integer | no range check (used as-is) | `4096` | Headless: triangles per draw before splitting across task-scheduler workers. Read at renderer creation. |
| `render.pixels_per_lock_group` | integer | no range check (used as-is) | `16` | Headless: pixels per depth lock group during parallel rasterization. Read at renderer creation. |
| `render.lod_min_triangles` | integer | no range check (used as-is; the headless backend's LOD generation uses a fixed 4096*3 index threshold) | `4096` | Headless: meshes below this triangle count get no LOD chain. Read at renderer creation. |

Formerly declared, now removed keys: `anisotropy_level`, `msaa_samples`, `srgb_output`, `z_prepass`, `max_textures_per_renderer`, `max_material_sets_per_renderer`, and `pipeline_cache_mb` used to be listed in `config_keys` although no reader existed anywhere in the module; they have been dropped from the declared surface. Setting them in a configuration is harmless (module sections are not known-key validated) and has no effect: the effective behavior stays the module's hard-coded values (anisotropy 1.0, `VK_SAMPLE_COUNT_1_BIT`, UNORM swapchain formats, no depth prepass, and the 256 texture / 256 material-set / 64 MiB pipeline-cache constants in the Vulkan backend).

## Out-of-range and malformed values

Behavior as implemented in `readConfiguration()` (render.cpp):

- Integer keys with a validated range are clamped into the range and a warning is printed to stderr, e.g. `render module: clamping [render] max_frames_in_flight 99 to range 1..8`. The clamped value is used.
- `clear_color` requires at least four components; a shorter array is silently ignored (no warning). If any component is non-finite the whole array is ignored with a warning; otherwise each component is clamped to 0..1.
- String enums (`backend`, `device`, `draw_sort_mode`) warn on unknown values and fall back (`auto` for backend/device, sorting disabled for draw_sort_mode). Unknown `draw_sort_mode` values never enable sorting.
- The three headless tuning keys read at renderer creation (`parallel_triangle_threshold`, `pixels_per_lock_group`, `lod_min_triangles`) are taken with `value_or(default)` and have no range validation - out-of-range values are passed through to the backend as-is.
- Every key in `config_keys` has a reader in the module; settings outside the keys above still do nothing (module sections are not known-key validated). The hard-coded values listed in the removed-keys note are the effective behavior for areas the module does not parameterize yet.

Note that the module's built-in defaults before configuration is applied deliberately differ from config.toml for opt-in features (sorting, batching, mipmaps start disabled in code); with the shipped config.toml the table above is what actually ends up active. `module.toml` `config_keys` documents the section's surface; settings outside these keys are not validated by the kernel (it validates only engine sections and game-mod manifests).
