# asset_gltf module - content import (glTF 2.0)

## What this module provides

Capability `asset` (implementation-bound member `asset.gltf`, published in `asset_capability.hpp`): content import into the scene registry. A mod ships a glTF 2.0 file, imports it, and every object it describes is immediately live in the same registry scripts spawn into - the renderer rebuilds, physics can step it, animation can tween it. No parallel import path, no special "asset objects".

```lua
local count = engine.import_gltf("mods/my_mod/assets/level.gltf", "mesh")
engine.log_information("imported " .. count .. " objects")
```

## The fidelity option

`import_gltf` takes a fidelity argument, and the import CAPABILITY carries the same option (`asset_fidelity` in `asset_capability.hpp`):

- **"mesh"** (`full_mesh`, the default): each primitive's triangles register as a registry MESH (deterministic name: node name + `#n<node>p<primitive>`, idempotent by name) and spawn one mesh OBJECT per primitive, centered on the primitive's centroid, carrying its material. Positions and indices decode from EMBEDDED base64 buffers; NORMAL accessors are honored, and a primitive without normals gets engine-computed smooth normals (the shipped Stanford dragon exercises exactly that path - see `mods/test_scene/README.md`). The geometry pass rasterizes the actual triangles; the analytic consumers (GI, reflections, shadows) see each mesh through its ORIENTED BOUNDING BOX proxy - a real box primitive with the object's material (docs/robustness.md, invariant 24), so raster and relight paths agree by construction.
- **"bbox"** (`bounding_proxies`, the v1 contract): each primitive maps to one bounding-box object, exactly as documented below - the honest proxy for runs that want level data without the triangle upload.

The registry's mesh caps apply (`SceneCapability::maxMeshes` = 8, `maxMeshVertices` = 1,000,000): a document that exceeds them imports partially, and the returned count is what the caller checks - every refusal logs precisely.

## The glTF mapping contract (bounding-proxies fidelity)

| glTF                        | engine                                            |
| --------------------------- | ------------------------------------------------- |
| node translation/rotation/scale or matrix | composed down the hierarchy into world transforms |
| mesh primitive POSITION accessor `min`/`max` | one box: center + half extents (8 corners through the world transform) |
| material `pbrMetallicRoughness.baseColorFactor` | object albedo (alpha ignored - no transparency model yet) |
| `metallicFactor` / `roughnessFactor` | object metallic / roughness                       |
| `emissiveFactor`            | emissive strength (strongest channel; the engine's emissive is scalar radiance) |
| scene / scenes / root nodes | default scene, or scene 0, or every parentless node |

What is deliberately NOT imported: textures (runtime GPU upload has no path in the incremental rebuild; reference registered texture sets via scripts after import), skins and animation samplers (the animation_tween module tweens transforms, not joint hierarchies), cameras (the renderer owns one camera), and vertex attributes beyond POSITION/NORMAL (the engine's material model is per-object, not per-vertex - UVs on mesh proxies are procedural, over the bounding box in mesh-local meters, so raster and relight paths sample identical texels).

Node transforms bake into the decoded positions (the engine's rotation model is Y-only); the object's `position` carries the node origin plus the primitive's centroid, and its `size` carries the mesh's bounding-box HALF EXTENTS - the same numbers the tracing proxy uses and physics collides against.

## Units and scales

glTF is meters, the engine is meters: 1:1. A 2-meter door imports as a box two meters tall. Files authored in centimeters (common in DCC exports) come in 100x too large - scale in the DCC, or walk the registry after import and fix transforms from a script.

## Files and formats

`.gltf` (JSON) is the supported form. `.glb` containers are refused with a precise message: the JSON form is the interchange form, and the binary container would need a separate demux path for no ecosystem gain. Buffer data comes from EMBEDDED data URIs (`data:application/octet-stream;base64,...`) - the document stays self-contained and the importer reads nothing but the JSON text; an external `.bin` reference is refused with a precise message naming the file. Bounding-proxies imports need no buffers at all: their bounds come from the accessors' JSON `min`/`max` fields.

## Configuration

None. Files resolve relative to the executable directory (the engine's path convention), so mod-internal assets are addressed as `mods/<id>/assets/<file>.gltf` from scripts.

## Files

- `asset_capability.hpp` - the published capability surface
- `asset.hpp` / `asset.cpp` - the module, file reading, precise errors
- `gltf_json.hpp/.cpp` - the minimal strict JSON reader (engine-owned; no fetched JSON dependency, per the engine's dependency policy)
- `gltf_import.hpp/.cpp` - the pure importer (no Vulkan, no backend; feeds the scene registry directly)
- `module_plugin.cpp` - the static plugin descriptor
- `CMakeLists.txt` - target wiring (`ENGINE_BUILD_MODULE_ASSET_GLTF`)
