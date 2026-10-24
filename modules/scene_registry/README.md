# scene_registry module - the runtime object registry

## What this module provides

Capability `scene` (implementation-bound member `scene.registry`, published in `scene_capability.hpp`): the live object registry of the world. The static `[scene]` configuration is parsed once at startup; this module makes the world addressable at runtime on top of it:

- **spawn**: `addObject(SceneObjectDescriptor)` registers an object and returns a stable handle (0 = registry full; the cap mirrors the GPU primitive buffer's allocation size).
- **transform**: `setObjectTransform(handle, position, rotationY)`.
- **material**: `setObjectMaterial(handle, descriptor)` - albedo, roughness, metallic, emissive strength, texture set, uv scale. The object's kind and transform are deliberately untouched by a material edit.
- **remove**: `removeObject(handle)`; handles are never reused.
- **readback**: `object(handle)`, `objectCount()`, and `objectSnapshot()` (every live object in ascending handle order).
- **meshes**: `addMesh(name, SceneMeshDescriptor)` registers a triangle mesh under a name unique within the run (the same name returns the SAME handle - imports are idempotent by name, which is what makes persistence restore mesh references exactly); `mesh(handle)` reads one back; `meshHandle(name)` / `meshName(handle)` translate between the two addresses. Mesh objects are ordinary objects with `type = mesh` and a `meshHandle` - several objects may instance one mesh, each through its own transform and material.

Every mutation publishes an event on the bus: `SceneObjectAddedEvent`, `SceneObjectRemovedEvent`, `SceneObjectChangedEvent`, `SceneMeshAddedEvent` - each carrying the handle. Consumers coalesce events per frame; many edits in one frame cost one GPU refresh, not one per edit.

## How the world gets in

The registry starts EMPTY. During the renderer's start phase the static world - the parsed `[scene]` configuration - is pushed into the registry, so every consumer sees one world regardless of where its objects came from. A headless build without a renderer simply starts with an empty registry and populates it through scripts or modules. Meshes arrive through `addMesh` callers: the asset_gltf module's glTF import (full-mesh fidelity registers under deterministic names like `Dragon#n0p0`), native mods, or the scripting bridge.

## Who consumes it

| consumer            | uses                                                                 |
| ------------------- | -------------------------------------------------------------------- |
| renderer            | subscribes to the events; rebuilds the GPU scene data incrementally; uploads registered meshes once through its staging path and rasterizes mesh objects as real triangles |
| lua_scripting       | exposes spawn/transform/remove to script mods (`engine.spawn_object` and friends) |
| physics             | steps bodies and writes transforms back through `setObjectTransform`; mesh objects collide as their bounding-box proxy |
| animation           | tweens transforms and materials the same way                         |
| asset               | registers imported meshes through `addMesh` (idempotent by name)     |
| persistence         | snapshots the object list to TOML and spawns rows back (mesh objects by NAME) |

## The incremental rebuild contract

The renderer allocates its primitive and material buffers ONCE at the engine-wide GPU maximum (`SceneCapability::gpuMaxObjects` = 256, mirrored by `scene_common.glsl` on the shader side) and only re-uploads their CONTENT when a scene event arrives. No buffer handle, descriptor set or pipeline is ever invalidated by a scene edit, so the lighting techniques need no rebuild path at all: they re-read `primitiveCount` per frame and keep tracing whatever the buffer holds. The full-capacity allocation is also the honest cost statement: the analytic SDF scales linearly with the object count, and the cap is what keeps a runaway script from silently degrading the frame rate.

Registered meshes upload once per handle (staging copy, device idle per docs/robustness.md, invariant 38) and draw in the geometry pass with the SAME interleaved vertex layout (position xyz + normal xyz, `meshFloatsPerVertex` = 6) the unit primitives use - one vertex input state serves both. The analytic consumers never see mesh triangles: each mesh object contributes its ORIENTED BOUNDING BOX as an exact box primitive carrying the object's material (the proxy contract, documented on `scene_object_type::mesh` and in docs/robustness.md invariant 24), so raster and relight paths agree on geometry by construction.

## Configuration

```toml
[scene]
max_objects = 32   # live-object capacity, clamped to 1..256 (the GPU buffer's size)
```

The static initial world comes from the renderer's `[scene]` parse (or an empty registry in builds without a renderer); `max_objects` is the only key the module itself reads. Raising the cap between runs is a configuration edit, never a code change: the GPU buffers are allocated at the 256 maximum regardless, and `objectCapacity()` reports what a run actually enforces.

## Files

- `scene_capability.hpp` - the published capability surface
- `scene.hpp` / `scene.cpp` - the module and the registry implementation
- `module_plugin.cpp` - the static plugin descriptor
- `CMakeLists.txt` - target wiring (`ENGINE_BUILD_MODULE_SCENE_REGISTRY`)
