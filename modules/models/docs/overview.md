# models module

models gives the engine offline-style mesh import: it loads static model files (OBJ, STL, PLY, OFF, glTF 2.0), turns them into render-ready triangle parts with normals, vertex colors, UVs, and materials, optionally optimizes the geometry with meshoptimizer, and hands the result out as the `models` capability. It is the bridge between asset files on disk and the `renderer::MeshDescription` views the renderer consumes; nothing here talks to the GPU.

## Capability

The module provides the `models` capability, ABI version 7 (`modelsCapabilityAbi`, see `include/engine/models/capabilities.hpp`). Consumers check `capability.abi` against that constant before calling anything. All calls take the capability's own `self` pointer as their first argument; handles are never reused and `0` (`invalidModel`) means "none".

| Entry | Purpose |
| --- | --- |
| `loadModel(self, path, outError)` | Loads a model with the configured module defaults. |
| `loadModelWithOptions(self, path, options, outError)` | Loads with caller-supplied `ModelLoadOptions`; a null `options` selects the module defaults. |
| `destroyModel(self, model)` | Drops the model; every `MeshDescription` previously handed out for it becomes invalid. |
| `partCount(self, model)` | Number of renderable parts (0 for unknown handles). |
| `part(self, model, partIndex, outDescription)` | Fills a `renderer::MeshDescription` that aliases module-owned buffers. |
| `partName(self, model, partIndex)` | Part name (OBJ shape, glTF node/mesh, or file stem); null when out of range. |
| `bounds(self, model, outMin, outMax)` | Post-transform axis-aligned bounds; either out-pointer may be null. |
| `materialCount` / `material` / `materialName` / `materialTexturePath` / `materialTextureOptions` | Material records in source order; texture paths are the raw URIs from the file. |
| `partTriangleMaterials(self, model, partIndex, outIndices, outTriangleCount)` | Per-triangle material index array (`invalidMaterial` = 0xffffffff where unassigned). |
| `statistics(self, model, outStatistics)` | File size, vertex/index/part/material counts of the loaded model. |

**Error convention:** loaders return `invalidModel` and write a `ModelLoadError` through the optional out-parameter (`FileNotFound`, `ParseFailed`, `NoGeometry`, `InvalidOptions`, `FileTooLarge`, `TooManyModels`). Query entries never fail hard: unknown handles behave like empty models (0 parts, 0 materials, null names). Every vertex in every part is interleaved as 11 floats (`renderer::meshVertexFloatCount`): position, normal, color, uv.

## Provided / required / optional capabilities

From `module.toml` and from the exported entry table at the bottom of `source/models.cpp` (`engineQueryModuleEntryTable`), which is what the kernel actually loads the .so against (module ABI `engineModuleAbiVersion` = 8):

| | |
| --- | --- |
| Provides | `models` (the only entry in both the descriptor and the entry table) |
| Requires | none (`requires = []`, `requiredCapabilities = nullptr`) |
| Optional | none (`optionalCapabilities = nullptr`) |
| Declared permissions | none (the module reads configuration and model files only) |
| Module name / version | `models` / `0.1.0` (matches the manifest built in `makeManifest()`) |

## Architecture

| File | Purpose |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provides/requires, and the declared `config_keys`. |
| `config.toml` | Default values for the `[models]` configuration section (see `configuration.md`). |
| `include/engine/models/capabilities.hpp` | Public capability types: ABI, handles, errors, load options, function-pointer table. Material and texture-slot types live in `shared_types/include/engine/models/material.hpp`. |
| `source/models.cpp` | Module entry point: configuration, the model registry, capability binding, and the load pipeline. |
| `source/model_import_internal.hpp` | Shared importer plumbing: `ModelRecord`/`ModelPart` types, the option transform, smooth-normal and index helpers. |
| `source/obj_importer.cpp` | Wavefront OBJ/MTL importer (tinyobjloader v2.0.0rc13, header-only). One part per OBJ shape; MTL materials with PBR inference. |
| `source/stl_importer.cpp` | STL importer (binary and ASCII). Bit-exact vertex welding on the transformed position. |
| `source/ply_importer.cpp` | PLY importer (ascii, binary little/big endian) with a hand-rolled header parser. |
| `source/off_importer.cpp` | OFF/COFF importer; polygon fan triangulation, optional COFF vertex colors. |
| `source/gltf_importer.cpp` | glTF 2.0 importer (cgltf v1.15, header-only): scene walk, node transforms, material extensions. |
| `source/cgltf_implementation.cpp` | The one translation unit that compiles the cgltf parser implementation. |
| `source/model_optimizer.cpp` | meshoptimizer passes (v1.3): vertex cache, overdraw, vertex fetch. |

## How it works

`loadModelWithOptionsImpl` in `source/models.cpp` runs the pipeline; the per-format `import*File` functions fill a `ModelRecord`.

1. **Options.** Caller options replace the configured defaults wholesale and are validated in place by `validOptions`: enum ranges, material search path length (4096 max), finite scale/translation/pivot/uv values bounded by 1e12, and a renormalized rotation quaternion. Invalid options fail with `InvalidOptions` before anything is read.
2. **Transform.** `makeTransform` precomputes the option transform once: position matrix = rotation (from the quaternion) times per-axis scale, normal matrix = rotation divided by the scale, pivot and translation, plus a `reverseWinding` flag (Reverse mode, or Automatic with an odd number of negative scale axes; Preserve never reverses).
3. **File resolution and limits.** Absolute paths must exist as given; relative paths are probed through `models.search_paths` in order, then the working directory. The file size is checked against `models.max_file_size_mb` (0 disables the limit) and the live model count against `models.max_loaded_models`.
4. **Format detection and importer selection.** An explicit `ModelLoadOptions::format` forces the importer; `Auto` maps the lowercased file extension (`.obj`, `.stl`, `.ply`, `.off`, `.gltf`, `.glb`) to one. Unknown extensions fail with `ParseFailed` after a diagnostic on stderr.
5. **Import.** Each importer parses its file, applies the transform to every position and normal, and appends parts (OBJ: one per shape; glTF: one per primitive; STL/PLY/OFF: a single part named after the file stem). Common vertex processing:
    - **Welding** merges coincident corners: OBJ welds by (position, normal, uv) index keys (position-only when the shape has neither normals nor uvs), STL welds bit-exactly on the transformed position plus face normal. PLY, OFF, and glTF keep the source vertex pool as-is.
    - **Normals** follow `normal_mode`: preserved through the transform (OBJ/PLY/ glTF), generated area-weighted smooth over shared vertices, or flat (which expands triangles to unshared vertices). Files without normals fall back to geometry or a +Y placeholder per the mode.
    - **Colors** are read only when `import_vertex_colors` is set (OBJ `v x y z r g b`, PLY `red/green/blue`, COFF colors, glTF `COLOR_0`); otherwise white.
    - **UVs** get the V flip (`flip_texture_v`), then `uv_scale`/`uv_offset`.
    - **Materials**: OBJ MTL files become material records (with PBR factors inferred from shininess/dissolve/ior where OBJ has none); glTF materials are copied factor-by-factor including the supported KHR extensions. Parts record one material index per triangle (`invalidMaterial` when unassigned).
6. **Optimization.** When `optimize_mesh` is set, `optimizeModelRecord` runs the module-configured meshoptimizer passes over every part: vertex cache and overdraw reorder triangles (only for parts whose triangles share a single fully opaque material, so blending order survives), then vertex fetch reorders the vertex buffer along the index order.
7. **Registration.** Statistics are filled from the post-optimization record, a fresh handle is allocated (monotonic counter that skips 0 and live ids), and the record moves into the module's map.

**ModelHandle to GPU-facing data.** `part()` fills a `renderer::MeshDescription` whose `vertices`/`indices` pointers alias the part's own vectors (`vertexCount` = floats / 11, `indexCount` triangles x 3, `MeshIndexFormat::UInt32`); stride and attribute tables stay at the description defaults, the renderer knows the 11-float layout via `renderer::meshVertexFloatCount`. The pointers stay valid until `destroyModel`, module stop, or (for `partTriangleMaterials`/names) the same lifetime.

## Threading and lifetime

All capability entries and the configuration-reload handler run synchronously on the caller's thread; the module keeps no mutexes. Treat the capability as single-threaded: load, query, and destroy models from the thread that owns the engine (the kernel drives construct/start/stop and event dispatch there) and do not share `MeshDescription` pointers across threads without external synchronization.

The module owns everything: `ModelRecord` (vertex/index buffers, material strings, statistics) lives in the registry until `destroyModel` erases it or `stop()` clears the registry. `part()`, `partName()`, `materialName()`, `materialTexturePath()`, and `partTriangleMaterials()` return views into that storage, not copies. Importers allocate their own scratch (cgltf's document is freed via `unique_ptr` before `loadModelWithOptions` returns); nothing imported is retained beyond what the record holds.

## Configuration

Every `[models]` key, its validated range, default, and effect is documented in [configuration.md](configuration.md).

## Integration

**Events.** The module subscribes to `ConfigurationReloadedEvent` in `construct()` (re-reading its defaults when any changed key starts with `models.`) and unsubscribes in `stop()`. It publishes no events.

**Permissions.** The module declares no permissions itself. A consumer acquires the capability with `context.acquireCapability<engine::models::ModelsCapability>()`, which the permission system gates behind the `capability.models` permission; declaring `models` in the consumer's `requires` (module.toml) or required capabilities grants it (the kernel derives `capability.<name>` permissions from declared requirements). `tryAcquireCapability` returns null instead of throwing when the capability is missing.

**Consumer usage** (signatures from `capabilities.hpp`, as used by `demo_character_customisation/source/main.cpp`):

```cpp
auto &models = context.acquireCapability<engine::models::ModelsCapability>();
// optionally: check models.abi == engine::models::modelsCapabilityAbi

engine::models::ModelLoadOptions options{}; // or nullptr for configured defaults
options.normalMode = engine::models::ModelNormalMode::GenerateSmooth;
engine::models::ModelLoadError error = engine::models::ModelLoadError::None;
engine::models::ModelHandle model
    = models.loadModelWithOptions(models.self, "assets/models/dragon.obj", &options, &error);
if (model == engine::models::invalidModel)
{
    // inspect `error`
}
else
{
    engine::models::ModelStatistics statistics;
    models.statistics(models.self, model, &statistics);

    const std::uint32_t parts = models.partCount(models.self, model);
    for (std::uint32_t partIndex = 0; partIndex < parts; ++partIndex)
    {
        engine::renderer::MeshDescription description;
        if (models.part(models.self, model, partIndex, &description))
        {
            // upload description.vertices / description.indices to the renderer
        }
    }
    models.destroyModel(models.self, model); // invalidates the descriptions above
}
```
