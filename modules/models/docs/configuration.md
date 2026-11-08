# models configuration

All keys live in the `[models]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. Every value is read once in `construct()` (and again whenever a reload event changes a `models.*` key). Keys are listed alphabetically.

| Key | Type | Valid range (as validated in `readConfiguration()`) | Default | Effect |
| --- | --- | --- | --- | --- |
| `models.flip_texture_v` | bool | presence only | `true` | Rewrites V as `1 - v` before the UV transform (OBJ/PLY/glTF V conventions). |
| `models.format` | string | `auto`, `obj`, `stl`, `ply`, `off`, `gltf`, `glb` | `"auto"` | Forces an importer; `auto` (and unknown values) pick one from the file extension. |
| `models.import_materials` | bool | presence only | `true` | Imports material records and fills per-triangle material indices. |
| `models.import_vertex_colors` | bool | presence only | `false` | Reads OBJ/PLY/COFF/glTF vertex colors into the vertex color triple (white when off). |
| `models.material_search_path` | string | none (used as given) | `""` | Directory the OBJ importer searches for MTL files when the load options do not override it. |
| `models.max_file_size_mb` | integer | `0 <= v <= 1048576` | `512` | Per-file size cap in MiB; larger files fail with `FileTooLarge`. `0` disables the cap. |
| `models.max_loaded_models` | integer | `0 <= v <= 16777216` | `256` | Live model cap; `loadModel`/`loadModelWithOptions` fail with `TooManyModels` beyond it. `0` disables the cap. |
| `models.normal_mode` | string | `preserve_smooth_missing`, `preserve`, `smooth`, `flat` | `"preserve_smooth_missing"` | How vertex normals are produced (see `ModelNormalMode`). |
| `models.optimize_mesh` | bool | presence only | `true` | Runs the meshoptimizer passes below over every part after import. |
| `models.optimize_overdraw` | bool | presence only | `true` | Then reorders triangles to reduce pixel overdraw. |
| `models.optimize_vertex_cache` | bool | presence only | `true` | Reorders triangles for post-transform vertex cache hits. |
| `models.optimize_vertex_fetch` | bool | presence only | `true` | Finally reorders vertices along the index buffer. |
| `models.overdraw_threshold` | float | finite, `1.0 <= v <= 2.0` | `1.05` | Allowed overdraw increase accepted by the overdraw pass. |
| `models.pivot` | float[3] | per component: finite, `abs(v) <= 1e12` | `[0.0, 0.0, 0.0]` | Point that rotation and scale are applied around. |
| `models.rotation` | float[4] | all four components finite and the quaternion length `> 1e-8`; then normalized | `[0.0, 0.0, 0.0, 1.0]` | Unit quaternion (x y z w) the model is rotated by. |
| `models.scale` | float[3] | per component: finite, `abs(v) <= 1e12` | `[1.0, 1.0, 1.0]` | Per-axis model scale, applied with the rotation. |
| `models.search_paths` | string array | none (used as given) | `["assets/models"]` | Directories probed in order when resolving relative model paths. Array order is significant (first hit wins), so entries are not sorted. |
| `models.translation` | float[3] | per component: finite, `abs(v) <= 1e12` | `[0.0, 0.0, 0.0]` | Offset added after rotate/scale. |
| `models.uv_offset` | float[2] | per component: finite, `abs(v) <= 1e12` | `[0.0, 0.0]` | Per-axis UV offset added after the scale. |
| `models.uv_scale` | float[2] | per component: finite, `abs(v) <= 1e12` | `[1.0, 1.0]` | Per-axis UV multiplier. |
| `models.weld_vertices` | bool | presence only | `true` | Merges coincident corners in the OBJ and STL importers. |
| `models.winding_mode` | string | `automatic`, `preserve`, `reverse` | `"automatic"` | Triangle winding handling; `automatic` reverses when the scale mirrors handedness. |

## Out-of-range behavior

A value outside its documented range is never clamped or coerced: it is ignored and the built-in default (the value in the table above) stays active. Only two keys announce this - an unknown `normal_mode` string and an out-of-range `overdraw_threshold` each print one line on stderr; every other failure is silent. Array-valued transform keys (`scale`, `translation`, `pivot`, `uv_scale`, `uv_offset`) are validated per component: a bad component is skipped while valid components of the same array still apply, and arrays shorter than the required length are ignored entirely. `rotation` is all-or-nothing. The same bounds are enforced again, per load, on caller-supplied `ModelLoadOptions` in `validOptions()` - there a violation is not ignored but fails the load with `ModelLoadError::InvalidOptions`.

## How the keys relate

- The transform keys (`pivot`, `rotation`, `scale`, `translation`) and UV keys (`uv_scale`, `uv_offset`) become the precomputed option transform applied to every imported position, normal, and texture coordinate; see [overview.md](overview.md), "How it works", step 2.
- The optimizer keys form a fixed order - vertex cache, then overdraw, then vertex fetch - where each stage consumes the previous stage's result; `optimize_mesh` gates the whole chain and the other three are meaningless when it is false.
- `search_paths` and `material_search_path` are only consulted for relative paths; absolute paths bypass both.

`module.toml` enumerates exactly these 22 keys under `config_keys`; that list is the module's declared configuration surface on disk (the kernel validates known keys only for game-mod manifests, which declare `config_keys` in their own mod.toml, and for the engine's built-in sections).
