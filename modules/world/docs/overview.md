# world module

world is the engine's entity layer, provided as one module with two complementary capabilities. The `ecs` capability is a raw archetype-based component store (components, chunked queries, deferred command buffers). The `entity_world` capability is a scene-graph-style layer of named, transformed entities with parent/child hierarchies and binary save/load. The module's facade composes both: every entity in a facade world exists in both layers, with generation-checked handle translation so consumers can mix component storage and hierarchy transforms through one handle space.

## Capability

Two capabilities, each with its own header and ABI:

| Capability | ABI constant (value) | Header |
| --- | --- | --- |
| `ecs` | `ecsCapabilityAbi` (2) | `include/engine/ecs/capabilities.hpp` (types in `include/engine/ecs/storage.hpp`) |
| `entity_world` | `entityWorldCapabilityAbi` (2) | `include/engine/entity_world/capabilities.hpp` |

Shared handle vocabulary: `WorldHandle` / `EntityHandle` (from `include/engine/world/types.hpp`). Entity handles pack slot + 1 in the low 32 bits and a generation counter in the high 32 bits, so stale handles are detected after slot reuse; 0 is invalid for worlds, entities, components, and queries.

`ecs` groups: world lifetime (`createWorld`, `destroyWorld`, `worldInfo`), components (`registerComponent`), entities (`createEntity`, `destroyEntity`, `entityExists`), components per entity (`addComponent`, `removeComponent`, `hasComponent`, `getComponent`), queries (`createQuery`, `destroyQuery`, `queryChunks`), and deferred changes (`createCommandBuffer`, `commandAddDefaultComponent`, `commandRemoveComponent`, `commandDestroyEntity`, `applyCommandBuffer`).

`entity_world` groups: world lifetime (`createWorld`, `destroyWorld`, `worldInfo`), entities (`createEntity`, `destroyEntity` with a recursive flag, `entityExists`, `setEntityEnabled`, `isEntityEnabled`, `copyEntities`, `copyEntityName`), transforms (`setLocalTransform`, `getLocalTransform`, `getWorldTransform`), hierarchy (`setParent` with world-transform preservation, `getParent`), and persistence (`saveWorld`, `loadWorld`).

Error convention: fallible calls take an error out-parameter (`StorageError` / `EntityWorldError`). Unknown handles report `InvalidInput` or `InvalidHandle`; structural changes attempted while a query callback is running fail with `InvalidDuringQuery`.

## Provided / required / optional capabilities

From `module.toml` and the entry table at the bottom of `source/world.cpp` (they agree; `source/ecs.cpp` and `source/entity_world.cpp` declare the same capabilities individually when built standalone):

- Provided: `ecs`, `entity_world`
- Required: none
- Optional: none
- Declared permissions: none

Consumers need the kernel permissions `capability.ecs` / `capability.entity_world` to acquire the capabilities through `ParticipantContext::acquireCapability`.

## Architecture

| File | Purpose |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capabilities, declared `config_keys`. |
| `config.toml` | Defaults for the `[ecs]` and `[entity_world]` sections (see `configuration.md`). |
| `include/engine/world/types.hpp` | Shared `WorldHandle` / `EntityHandle` vocabulary. |
| `include/engine/ecs/storage.hpp` | C++ storage layer: `StorageOptions`, component/query/view types, `Storage`, `CommandBuffer`. |
| `include/engine/ecs/capabilities.hpp` | The `ecs` capability struct and its description/info types. |
| `include/engine/entity_world/capabilities.hpp` | The `entity_world` capability struct, `Transform`, descriptions. |
| `source/ecs_storage.cpp` | Archetype storage internals: chunk/column layout, entity slots, transitions, queries, command buffers. |
| `source/ecs.cpp` | `ecs` capability binding: per-world records, function-pointer table, standalone entry table (or embedded hook). |
| `source/entity_world.cpp` | `entity_world` implementation: slots, hierarchy, transforms, save/load, standalone entry table (or embedded hook). |
| `source/world.cpp` | The facade: embeds both inner modules, bridges every capability call with handle translation and mirror consistency. |

## How it works

**The ecs layer (archetype storage).** `Storage` keeps entities as generation-tagged slots in a vector plus an intrusive free list. Component data lives in archetypes — each archetype owns a unique sorted component signature and a list of fixed-capacity chunks; a chunk is one aligned allocation holding one tightly packed column per component (`stride` is the alignment-padded component size). Adding or removing a component moves the entity to the neighboring archetype via a per-archetype transition cache (`appendToArchetype` copies the surviving columns, `eraseAt` swap-removes the old row). Queries are cached: `createQuery` validates and sorts the required / excluded signatures and precomputes, per matching archetype, the column index of each required component; newly created archetypes are matched against all live queries so the cache stays current. `queryChunks` walks the cached matches chunk by chunk, building a `ChunkView` (inline column-pointer array for up to 16 components); the callback may return false to stop early, and a depth counter rejects structural calls made from inside the callback. `CommandBuffer` records structural changes as type-erased closures and applies them in order, stopping at the first failure and clearing the queue either way.

**The entity_world layer.** Entities are slots (`EntitySlot`) with a generation, name, enabled flag, parent, children list, and a local `Transform` (position, x/y/z/w quaternion, scale — validated finite, normalized, and away from zero scale). World transforms are cached per slot and recomputed lazily: `getWorldTransform` walks up to the first clean ancestor, then composes `parent * local` back down the chain; mutations mark the affected subtree dirty via an explicit stack. `destroyEntity` with `recursive = false` re-parents the children while preserving each child's world transform (local = parent^-1 * world); with `recursive = true` the whole subtree is erased children-first. `saveWorld` writes a versioned binary format (magic `WRLD` = 0x444C5257, version 4: world name, per-slot generations, then one record per alive entity); `loadWorld` accepts versions 2-4 and validates everything — slot bounds, duplicate slots, transforms, parent occupancy, depth, cycles — before replacing the world's contents.

**The facade (how the two layers relate).** `WorldModule` embeds the `ecs` and `entity_world` modules and forwards `construct` / `start` / `stop` to them. Each facade world is a `WorldBinding` holding one world per layer plus two handle-map vectors (entity-world handle -> ecs entity and back, both generation-checked). Every capability entry is a facade lambda:

- Creating a world through either facade creates both underlying worlds; the public handle is the entity-world handle (bindings are keyed by it).
- Creating an entity through either facade creates the entity in that layer and its mirror in the other, binding the two handles; failures roll both sides back.
- `ecs` facade component/entity calls translate the public handle to the internal ecs entity before delegating. Command-buffer destroys record the mirror handles so `applyCommandBuffer` can destroy the entity-world side after ecs applied the buffer.
- `entity_world` facade destroys (including recursive ones) first collect the affected handles, let the entity_world layer perform the destroy, then destroy the corresponding ecs entities and unbind them.
- `queryChunks` on the ecs facade installs a bridge callback that rewrites each chunk's entity handles into entity-world handles before the consumer sees it, and raises a query-active counter that blocks structural facade calls (destroy entity/world, apply buffer, load world) until the query ends.
- `loadWorld` on the entity_world facade rebuilds the ecs mirror from scratch: old command buffers and ecs entities are discarded, then one ecs entity is created for every loaded entity.

## Threading and lifetime

- No internal locking anywhere: each world (and each facade binding) must be used from a single thread. Component pointers obtained from `getComponent` or `queryChunks` stay valid only until the next structural change touching that chunk.
- Reentrancy: structural changes are rejected with `InvalidDuringQuery` while a `queryChunks` callback runs (the facade adds the same guard for destroys, applies, and loads, tracked per binding in `activeEcsQueries`).
- Handles are generation-checked in both layers, so using a handle after its entity was destroyed (and its slot reused) fails cleanly instead of corrupting data.
- `stop()` drops every facade binding, which destroys the mirrored worlds in both layers and every command buffer. `WorldInfo::name` / `EntityWorldInfo::name` point into module-owned storage and die with the world.

## Configuration

Two sections, `[ecs]` and `[entity_world]`; see `configuration.md` for every key, its validated range, and its effect.

## Integration

- Events: the module publishes nothing and subscribes to nothing — no `eventBus` use in any of its sources. `construct()` reads configuration only.
- Permissions: none declared in `module.toml` or the entry table.
- Consumer example (host side, inside a `construct(ParticipantContext&)`):

```cpp
// Component storage through the ecs capability.
auto &ecs = context.acquireCapability<engine::ecs::EcsCapability>();

engine::ecs::StorageError error{};
engine::ecs::WorldHandle world = ecs.createWorld(ecs.self, nullptr, &error);

static constexpr engine::ecs::ComponentDescription velocity{
    .name = "velocity", .size = sizeof(float) * 3, .alignment = alignof(float)};
engine::ecs::ComponentId velocityId =
    ecs.registerComponent(ecs.self, world, &velocity, &error);

engine::ecs::Entity entity = ecs.createEntity(ecs.self, world, &error);
const float v[3] = {0.0f, 9.8f, 0.0f};
ecs.addComponent(ecs.self, world, entity, velocityId, v, &error);

// Hierarchy and transforms through the entity_world capability.
auto &entities = context.acquireCapability<engine::entity_world::EntityWorldCapability>();
engine::entity_world::EntityWorldError worldError{};
engine::entity_world::EntityWorldHandle scene =
    entities.createWorld(entities.self, nullptr, &worldError);

engine::entity_world::EntityDescription parentDesc{};
parentDesc.name = "player";
engine::entity_world::EntityHandle player =
    entities.createEntity(entities.self, scene, &parentDesc, &worldError);

engine::entity_world::Transform playerWorld{};
entities.getWorldTransform(entities.self, scene, player, &playerWorld);
```
