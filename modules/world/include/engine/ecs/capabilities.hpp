// Capability surface of the ecs layer: archetype-based component storage
// with chunked columns, cached queries, and deferred command buffers. Types
// live in engine/ecs/storage.hpp; this header binds them into the C ABI.

#ifndef ENGINE_ECS_CAPABILITIES_HPP
#define ENGINE_ECS_CAPABILITIES_HPP

#include <engine/ecs/storage.hpp>

#include <cstdint>

namespace engine::ecs
{

// Component ids start at 1 (invalidComponent = 0); command buffer ids start
// at 1 as well.
using WorldHandle                                         = engine::world::WorldHandle;
using CommandBufferHandle                                 = std::uint64_t;
inline constexpr WorldHandle         invalidWorld         = engine::world::invalidWorld;
inline constexpr CommandBufferHandle invalidCommandBuffer = 0;
inline constexpr std::uint32_t       ecsCapabilityAbi     = 2u;

// Creation parameters; a null name becomes "world". storage tunes the
// capacities (see StorageOptions in storage.hpp).
struct WorldDescription
{
    const char    *name = nullptr;
    StorageOptions storage{};
};

// Snapshot of a world's sizes. name points into module-owned storage and is
// valid until the world is destroyed.
struct WorldInfo
{
    const char   *name               = nullptr;
    std::uint32_t entityCount        = 0;
    std::uint32_t componentTypeCount = 0;
    std::uint32_t queryCount         = 0;
};

// The "ecs" capability: world/component/entity management, chunk queries,
// and deferred command buffers. self is the first argument of every call.
// Unknown world or command-buffer handles report InvalidInput; structural
// calls made from inside a queryChunks callback fail with
// InvalidDuringQuery (component data may be read and mutated in place).
struct EcsCapability
{
    static constexpr const char *capabilityName = "ecs";
    std::uint32_t                abi            = ecsCapabilityAbi;
    void                        *self           = nullptr;

    WorldHandle (*createWorld)(void *, const WorldDescription *, StorageError *);
    void (*destroyWorld)(void *, WorldHandle);
    bool (*worldInfo)(void *, WorldHandle, WorldInfo *);
    ComponentId (*registerComponent)(void *, WorldHandle, const ComponentDescription *, StorageError *);
    Entity (*createEntity)(void *, WorldHandle, StorageError *);
    bool (*destroyEntity)(void *, WorldHandle, Entity, StorageError *);
    bool (*entityExists)(void *, WorldHandle, Entity);
    bool (*addComponent)(void *, WorldHandle, Entity, ComponentId, const void *, StorageError *);
    bool (*removeComponent)(void *, WorldHandle, Entity, ComponentId, StorageError *);
    bool (*hasComponent)(void *, WorldHandle, Entity, ComponentId);
    void *(*getComponent)(void *, WorldHandle, Entity, ComponentId);
    QueryId (*createQuery)(void *, WorldHandle, const QueryDescription *, StorageError *);
    bool (*destroyQuery)(void *, WorldHandle, QueryId);
    bool (*queryChunks)(void *, WorldHandle, QueryId, ChunkCallback, void *, StorageError *);
    CommandBufferHandle (*createCommandBuffer)(void *, WorldHandle, StorageError *);
    bool (*destroyCommandBuffer)(void *, WorldHandle, CommandBufferHandle);
    bool (*commandAddDefaultComponent)(void *, WorldHandle, CommandBufferHandle, Entity, ComponentId, StorageError *);
    bool (*commandRemoveComponent)(void *, WorldHandle, CommandBufferHandle, Entity, ComponentId, StorageError *);
    bool (*commandDestroyEntity)(void *, WorldHandle, CommandBufferHandle, Entity, StorageError *);
    bool (*applyCommandBuffer)(void *, WorldHandle, CommandBufferHandle, StorageError *);
};

} // namespace engine::ecs

#endif
