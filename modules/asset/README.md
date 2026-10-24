# Module "asset" - content import (glTF 2.0)

## What this module provides

Capability `asset` (implementation-bound member `asset.gltf`, published in `asset_capability.hpp`): content import into the scene registry. A mod ships a glTF 2.0 file, imports it, and every object it describes is immediately live in the same registry scripts spawn into - the renderer rebuilds, physics can step it, animation can tween it. No parallel import path, no special "asset objects".

```lua
local count = engine.import_gltf("mods/my_mod/assets/level.gltf")
engine.log_information("imported " .. count .. " objects")
```

## The glTF mapping contract

The renderer rasterizes and traces ANALYTIC primitives (boxes, spheres); a mesh pipeline does not exist yet. The import therefore maps each mesh primitive to its **bounding box** - the honest proxy that keeps traced and rasterized geometry identical (docs/robustness.md, problem 24) while giving mods real level data:

| glTF                        | engine                                            |
| --------------------------- | ------------------------------------------------- |
| node translation/rotation/scale or matrix | composed down the hierarchy into world transforms |
| mesh primitive POSITION accessor `min`/`max` | one box: center + half extents (8 corners through the world transform) |
| material `pbrMetallicRoughness.baseColorFactor` | object albedo (alpha ignored - no transparency model yet) |
| `metallicFactor` / `roughnessFactor` | object metallic / roughness                       |
| `emissiveFactor`            | emissive strength (strongest channel; the engine's emissive is scalar radiance) |
| scene / scenes / root nodes | default scene, or scene 0, or every parentless node |

What is deliberately NOT imported (yet): vertex data (the analytic renderer cannot consume it), textures (runtime GPU upload has no path in the incremental rebuild; reference registered texture sets via scripts after import), skins and animation samplers (the animation module tweens transforms, not joint hierarchies), cameras (the renderer owns one camera).

Bounding-box proxies are a documented v1 contract: when a mesh pipeline lands, the same importer interface grows a fidelity option instead of changing shape.

## Units and scales

glTF is meters, the engine is meters: 1:1. A 2-meter door imports as a box two meters tall. Files authored in centimeters (common in DCC exports) come in 100x too large - scale in the DCC, or walk the registry after import and fix transforms from a script.

## Files and formats

`.gltf` (JSON) is the supported form. `.glb` containers are refused with a precise message: the JSON form is the interchange form, and the binary container would need a separate demux path for no ecosystem gain. External `.bin` buffers are never read - bounds come from the accessors' JSON `min`/`max` fields, so the importer needs only the JSON document.

## Configuration

None. Files resolve relative to the executable directory (the engine's path convention), so mod-internal assets are addressed as `mods/<id>/assets/<file>.gltf` from scripts.

## Files

- `asset_capability.hpp` - the published capability surface
- `asset.hpp` / `asset.cpp` - the module, file reading, precise errors
- `gltf_json.hpp/.cpp` - the minimal strict JSON reader (engine-owned; no fetched JSON dependency, per the engine's dependency policy)
- `gltf_import.hpp/.cpp` - the pure importer (unit tested against a synthetic document and the real scene module)
- `module_plugin.cpp` - the static plugin descriptor
- `CMakeLists.txt` - target wiring (`ENGINE_BUILD_MODULE_ASSET`)
