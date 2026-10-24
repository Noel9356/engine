# Module "scene" - the runtime object registry

## What this module provides

Capability `scene` (implementation-bound member `scene.registry`, published in `scene_capability.hpp`): the live object registry of the world. Before this module, the world was a frozen TOML array - `[scene]` was parsed once at startup and never mutated. This module makes the world addressable at runtime:

- **spawn**: `addObject(SceneObjectDescriptor)` registers an object and returns a stable handle (0 = registry full; the cap mirrors the GPU primitive buffer's allocation size).
- **transform**: `setObjectTransform(handle, position, rotationY)`.
- **material**: `setObjectMaterial(handle, descriptor)` - albedo, roughness, metallic, emissive strength, texture set, uv scale. The object's kind and transform are deliberately untouched by a material edit.
- **remove**: `removeObject(handle)`; handles are never reused.
- **readback**: `object(handle)`, `objectCount()`, and `objectSnapshot()` (every live object in ascending handle order).

Every mutation publishes an event on the bus: `SceneObjectAddedEvent`, `SceneObjectRemovedEvent`, `SceneObjectChangedEvent` - each carrying the handle. Consumers coalesce events per frame; many edits in one frame cost one GPU refresh, not one per edit.

## How the world gets in

The registry starts EMPTY. During the renderer's start phase the static world - the parsed `[scene]` configuration - is pushed into the registry, so every consumer sees one world regardless of where its objects came from. A headless build without a renderer simply starts with an empty registry and populates it through scripts or modules.

## Who consumes it

| consumer            | uses                                                                 |
| ------------------- | -------------------------------------------------------------------- |
| renderer            | subscribes to the events; rebuilds the GPU scene data incrementally  |
| lua_scripting       | exposes spawn/transform/remove to script mods (`engine.spawn_object` and friends) |
| physics             | steps bodies and writes transforms back through `setObjectTransform` |
| animation           | tweens transforms and materials the same way                         |

## The incremental rebuild contract

The renderer allocates its primitive and material buffers ONCE at full capacity (`SceneCapability::maxObjects` = 32, mirrored by `scene_common.glsl` on the shader side) and only re-uploads their CONTENT when a scene event arrives. No buffer handle, descriptor set or pipeline is ever invalidated by a scene edit, so the lighting techniques need no rebuild path at all: they re-read `primitiveCount` per frame and keep tracing whatever the buffer holds. The full-capacity allocation is also the honest cost statement: the analytic SDF scales linearly with the object count, and the cap is what keeps a runaway script from silently degrading the frame rate.

## Configuration

None - the module takes no configuration section. The static initial world comes from the renderer's `[scene]` parse (or an empty registry in builds without a renderer); everything else is runtime behavior.

## Files

- `scene_capability.hpp` - the published capability surface
- `scene.hpp` / `scene.cpp` - the module and the registry implementation
- `module_plugin.cpp` - the static plugin descriptor
- `CMakeLists.txt` - target wiring (`ENGINE_BUILD_MODULE_SCENE`)
