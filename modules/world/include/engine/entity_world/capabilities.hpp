// Capability surface of the entity_world layer: named entities with a
// transform hierarchy (position/quaternion/scale), enable flags, and a
// versioned binary save/load format. Built on its own slot storage, not on
// the ecs layer.

#ifndef ENGINE_ENTITY_WORLD_CAPABILITIES_HPP
#define ENGINE_ENTITY_WORLD_CAPABILITIES_HPP

#include <engine/world/types.hpp>

#include <cstdint>

namespace engine::entity_world
{

inline constexpr std::uint32_t entityWorldCapabilityAbi = 2u;
using EntityWorldHandle                                 = engine::world::WorldHandle;
using EntityHandle                                      = engine::world::EntityHandle;
inline constexpr EntityWorldHandle invalidEntityWorld   = engine::world::invalidWorld;
inline constexpr EntityHandle      invalidEntity        = engine::world::invalidEntity;

enum class EntityWorldError : std::uint32_t
{
    None = 0,
    InvalidHandle,
    CapacityExceeded,
    InvalidTransform,
    InvalidHierarchy,
    ResourceFailure,
    InvalidDuringQuery
};

// Transform: position, unit quaternion rotation (x, y, z, w; normalized on
// input), scale. Scale components must stay above ~1e-8 in magnitude.
struct Transform
{
    float position[3] = {0.0f, 0.0f, 0.0f};
    float rotation[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    float scale[3]    = {1.0f, 1.0f, 1.0f};
};

// World creation parameters; the module configuration clamps these.
struct EntityWorldDescription
{
    const char   *name                  = nullptr;
    std::uint32_t initialEntityCapacity = 256;
    std::uint32_t maximumEntities       = 1'000'000;
    std::uint32_t maximumHierarchyDepth = 256;
};

// Entity creation parameters. parent = invalidEntity places the entity at
// the root; enabled is only a stored flag for consumers (nothing in the
// layer is gated on it).
struct EntityDescription
{
    const char  *name = nullptr;
    Transform    localTransform{};
    EntityHandle parent  = invalidEntity;
    bool         enabled = true;
};

// Live world sizes. name points into module-owned storage, valid until the
// world is destroyed or a loadWorld replaces its contents.
struct EntityWorldInfo
{
    const char   *name                  = nullptr;
    std::uint32_t entityCount           = 0;
    std::uint32_t maximumEntities       = 0;
    std::uint32_t maximumHierarchyDepth = 0;
};

// The "entity_world" capability: hierarchy of named, transformed entities.
// self is the first argument of every call; handles are generation-checked
// so stale handles fail. destroyEntity(recursive = false) re-parents the
// children while preserving their world transforms; saveWorld/loadWorld use
// a versioned binary format.
struct EntityWorldCapability
{
    static constexpr const char *capabilityName = "entity_world";
    std::uint32_t                abi            = entityWorldCapabilityAbi;
    void                        *self           = nullptr;

    EntityWorldHandle (*createWorld)(void *, const EntityWorldDescription *, EntityWorldError *);
    void (*destroyWorld)(void *, EntityWorldHandle);
    bool (*worldInfo)(void *, EntityWorldHandle, EntityWorldInfo *);
    EntityHandle (*createEntity)(void *, EntityWorldHandle, const EntityDescription *, EntityWorldError *);
    bool (*destroyEntity)(void *, EntityWorldHandle, EntityHandle, bool recursive, EntityWorldError *);
    bool (*entityExists)(void *, EntityWorldHandle, EntityHandle);
    bool (*setEntityEnabled)(void *, EntityWorldHandle, EntityHandle, bool);
    bool (*isEntityEnabled)(void *, EntityWorldHandle, EntityHandle, bool *);
    bool (*setLocalTransform)(void *, EntityWorldHandle, EntityHandle, const Transform *, EntityWorldError *);
    bool (*getLocalTransform)(void *, EntityWorldHandle, EntityHandle, Transform *);
    bool (*getWorldTransform)(void *, EntityWorldHandle, EntityHandle, Transform *);
    bool (*setParent)(
        void *, EntityWorldHandle, EntityHandle, EntityHandle, bool preserveWorldTransform, EntityWorldError *);
    EntityHandle (*getParent)(void *, EntityWorldHandle, EntityHandle);
    // Enumerates alive entity handles (up to capacity; the full count via
    // outTotal so callers can retry with a larger buffer).
    std::uint32_t (*copyEntities)(
        void *, EntityWorldHandle, EntityHandle *, std::uint32_t capacity, std::uint32_t *outTotal);

    // Copies the NUL-terminated name; reports the required size (including
    // the NUL) through outRequired even when the buffer is too small.
    bool (*copyEntityName)(
        void *, EntityWorldHandle, EntityHandle, char *output, std::uint32_t capacity, std::uint32_t *outRequired);
    bool (*saveWorld)(void *, EntityWorldHandle, const char *filePath, EntityWorldError *);
    bool (*loadWorld)(void *, EntityWorldHandle, const char *filePath, EntityWorldError *);
};

} // namespace engine::entity_world

#endif
