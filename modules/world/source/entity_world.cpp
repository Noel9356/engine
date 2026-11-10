// entity_world module implementation: slot-based entity storage with
// generation-checked handles, a transform hierarchy with lazy world-transform
// recomputation and dirty propagation, quaternion math, and a versioned
// binary save/load format. Like ecs.cpp it can be compiled into the world
// facade with ENGINE_WORLD_EMBEDDED.

#include <engine/entity_world/capabilities.hpp>
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/math/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine::entity_world
{
namespace
{

using engine::math::multiplyQuaternion;
using engine::math::normalizeQuaternion;
using engine::math::rotateVector;

void setError(EntityWorldError *output, EntityWorldError value)
{
    if (output != nullptr)
        *output = value;
}

// Rejects NaN/Inf components, near-zero scale, and zero quaternions
// (degenerate transforms that cannot be normalized).
bool finiteTransform(const Transform &value)
{
    for (float component : value.position)
        if (!std::isfinite(component))
            return false;
    for (float component : value.rotation)
        if (!std::isfinite(component))
            return false;
    for (float component : value.scale)
        if (!std::isfinite(component) || std::abs(component) < 1.0e-8f)
            return false;
    const double length = std::sqrt(static_cast<double>(value.rotation[0]) * value.rotation[0]
                                    + static_cast<double>(value.rotation[1]) * value.rotation[1]
                                    + static_cast<double>(value.rotation[2]) * value.rotation[2]
                                    + static_cast<double>(value.rotation[3]) * value.rotation[3]);
    return std::isfinite(length) && length > 1.0e-8;
}

// parent * local: the scaled local offset is rotated into parent space;
// scales multiply componentwise, rotations compose and are re-normalized.
Transform combine(const Transform &parent, const Transform &local)
{
    Transform result{};
    float     scaled[3] = {
        local.position[0] * parent.scale[0], local.position[1] * parent.scale[1], local.position[2] * parent.scale[2]};
    float rotated[3]{};
    rotateVector(parent.rotation, scaled, rotated);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        result.position[axis] = parent.position[axis] + rotated[axis];
        result.scale[axis]    = parent.scale[axis] * local.scale[axis];
    }
    multiplyQuaternion(parent.rotation, local.rotation, result.rotation);
    normalizeQuaternion(result.rotation);
    return result;
}

// Inverse of combine: local = parent^-1 * world. Fails when a parent scale
// axis is too small to divide by.
bool relativeTo(const Transform &parent, const Transform &world, Transform &outLocal)
{
    if (std::abs(parent.scale[0]) < 1.0e-8f || std::abs(parent.scale[1]) < 1.0e-8f
        || std::abs(parent.scale[2]) < 1.0e-8f)
        return false;
    float inverseRotation[4] = {-parent.rotation[0], -parent.rotation[1], -parent.rotation[2], parent.rotation[3]};
    float offset[3]
        = {world.position[0] - parent.position[0],
            world.position[1] - parent.position[1],
            world.position[2] - parent.position[2]};
    float unrotated[3]{};
    rotateVector(inverseRotation, offset, unrotated);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        outLocal.position[axis] = unrotated[axis] / parent.scale[axis];
        outLocal.scale[axis]    = world.scale[axis] / parent.scale[axis];
    }
    multiplyQuaternion(inverseRotation, world.rotation, outLocal.rotation);
    normalizeQuaternion(outLocal.rotation);
    return finiteTransform(outLocal);
}

// Per-slot entity state. generation validates handles after slot reuse;
// parentSlot uses UINT32_MAX for "no parent"; worldTransform is cached until
// invalidated (worldTransformDirty).
struct EntitySlot
{
    std::uint32_t              generation = 1;
    bool                       alive      = false;
    bool                       enabled    = true;
    std::uint32_t              parentSlot = std::numeric_limits<std::uint32_t>::max();
    std::string                name;
    Transform                  local{};
    Transform                  worldTransform{};
    bool                       worldTransformDirty = true;
    std::vector<std::uint32_t> children;
};

// One world: the slot vector plus free-list, and a dirty work stack kept as
// a member to avoid reallocating it per transform invalidation.
struct WorldRecord
{
    std::string                name;
    std::uint32_t              maximumEntities       = 1'000'000;
    std::uint32_t              maximumHierarchyDepth = 256;
    std::uint32_t              aliveCount            = 0;
    std::vector<EntitySlot>    entities;
    std::vector<std::uint32_t> freeSlots;
    std::vector<std::uint32_t> transformDirtyStack;
};

// Handle packing shared with the ecs layer (slot + 1 low, generation high);
// decode rejects zero slot or generation.
EntityHandle makeEntityHandle(std::uint32_t slot, std::uint32_t generation)
{
    return (static_cast<std::uint64_t>(generation) << 32u) | (static_cast<std::uint64_t>(slot) + 1u);
}

bool decodeEntityHandle(EntityHandle handle, std::uint32_t &slot, std::uint32_t &generation)
{
    if (handle == invalidEntity)
        return false;
    const std::uint32_t encodedSlot = static_cast<std::uint32_t>(handle & 0xffffffffu);
    generation                      = static_cast<std::uint32_t>(handle >> 32u);
    if (encodedSlot == 0 || generation == 0)
        return false;
    slot = encodedSlot - 1u;
    return true;
}

class EntityWorldModule final : public ModuleInterface
{
public:
    EntityWorldModule()
        : manifest_(makeManifest())
    {
        assignCapabilityPointers();
    }

    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    // Reads the [entity_world] keys; out-of-range or missing values keep the
    // built-in defaults (capacity/depth/worlds lower bounds of 1).
    void construct(ParticipantContext &context) override
    {
        if (const auto value = context.configurationIntegerInSection("entity_world", "initial_entity_capacity");
            value && *value > 0 && *value <= 1'000'000)
            initialCapacity_ = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("entity_world", "maximum_entities");
            value && *value > 0 && *value <= 10'000'000)
            maximumEntities_ = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("entity_world", "maximum_hierarchy_depth");
            value && *value > 0 && *value <= 4096)
            maximumDepth_ = static_cast<std::uint32_t>(*value);
        if (const auto value = context.configurationIntegerInSection("entity_world", "maximum_worlds");
            value && *value > 0 && *value <= 65536)
            maximumWorlds_ = static_cast<std::size_t>(*value);
    }

    void start() override
    {}
    void stop() override
    {
        worlds_.clear();
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == EntityWorldCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "entity_world",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"entity_world"}};
    }

    void assignCapabilityPointers()
    {
        capability_.self = this;
#define ENGINE_ENTITY_WORLD_BIND(field, method)                                                                        \
    capability_.field = [](void *self, auto... args) -> decltype(auto)                                                 \
    { return static_cast<EntityWorldModule *>(self)->method(args...); }
        ENGINE_ENTITY_WORLD_BIND(createWorld, createWorldImpl);
        ENGINE_ENTITY_WORLD_BIND(destroyWorld, destroyWorldImpl);
        ENGINE_ENTITY_WORLD_BIND(worldInfo, worldInfoImpl);
        ENGINE_ENTITY_WORLD_BIND(createEntity, createEntityImpl);
        ENGINE_ENTITY_WORLD_BIND(destroyEntity, destroyEntityImpl);
        ENGINE_ENTITY_WORLD_BIND(entityExists, entityExistsImpl);
        ENGINE_ENTITY_WORLD_BIND(setEntityEnabled, setEntityEnabledImpl);
        ENGINE_ENTITY_WORLD_BIND(isEntityEnabled, isEntityEnabledImpl);
        ENGINE_ENTITY_WORLD_BIND(setLocalTransform, setLocalTransformImpl);
        ENGINE_ENTITY_WORLD_BIND(getLocalTransform, getLocalTransformImpl);
        ENGINE_ENTITY_WORLD_BIND(getWorldTransform, getWorldTransformImpl);
        ENGINE_ENTITY_WORLD_BIND(setParent, setParentImpl);
        ENGINE_ENTITY_WORLD_BIND(getParent, getParentImpl);
        ENGINE_ENTITY_WORLD_BIND(copyEntities, copyEntitiesImpl);
        ENGINE_ENTITY_WORLD_BIND(copyEntityName, copyEntityNameImpl);
        ENGINE_ENTITY_WORLD_BIND(saveWorld, saveWorldImpl);
        ENGINE_ENTITY_WORLD_BIND(loadWorld, loadWorldImpl);
#undef ENGINE_ENTITY_WORLD_BIND
    }

    WorldRecord *findWorld(EntityWorldHandle handle)
    {
        const auto iterator = worlds_.find(handle);
        return iterator == worlds_.end() ? nullptr : iterator->second.get();
    }

    EntitySlot *findEntity(WorldRecord &world, EntityHandle handle, std::uint32_t *outSlot = nullptr)
    {
        std::uint32_t slot = 0, generation = 0;
        if (!decodeEntityHandle(handle, slot, generation) || slot >= world.entities.size())
            return nullptr;
        EntitySlot &record = world.entities[slot];
        if (!record.alive || record.generation != generation)
            return nullptr;
        if (outSlot != nullptr)
            *outSlot = slot;
        return &record;
    }

    const EntitySlot *findEntity(const WorldRecord &world, EntityHandle handle, std::uint32_t *outSlot = nullptr) const
    {
        std::uint32_t slot = 0, generation = 0;
        if (!decodeEntityHandle(handle, slot, generation) || slot >= world.entities.size())
            return nullptr;
        const EntitySlot &record = world.entities[slot];
        if (!record.alive || record.generation != generation)
            return nullptr;
        if (outSlot != nullptr)
            *outSlot = slot;
        return &record;
    }

    // Lazily recomputes cached world transforms: walks up to the first clean
    // ancestor (or the root), then combines transforms back down the chain,
    // marking slots clean. Fails past maximumHierarchyDepth or on dead slots.
    bool computeWorldTransform(WorldRecord &world, std::uint32_t slot, Transform &output)
    {
        if (slot >= world.entities.size() || !world.entities[slot].alive)
            return false;
        if (!world.entities[slot].worldTransformDirty)
        {
            output = world.entities[slot].worldTransform;
            return true;
        }

        std::array<std::uint32_t, 4096> chain{};
        std::size_t                     count             = 0;
        std::uint32_t                   current           = slot;
        bool                            hasCachedAncestor = false;
        Transform                       accumulated{};
        while (current != std::numeric_limits<std::uint32_t>::max())
        {
            if (current >= world.entities.size() || !world.entities[current].alive
                || count >= world.maximumHierarchyDepth || count >= chain.size())
                return false;
            const EntitySlot &record = world.entities[current];
            if (!record.worldTransformDirty)
            {
                accumulated       = record.worldTransform;
                hasCachedAncestor = true;
                break;
            }
            chain[count++] = current;
            current        = record.parentSlot;
        }
        if (count == 0 && !hasCachedAncestor)
            return false;
        while (count != 0)
        {
            EntitySlot &record         = world.entities[chain[--count]];
            accumulated                = hasCachedAncestor ? combine(accumulated, record.local) : record.local;
            record.worldTransform      = accumulated;
            record.worldTransformDirty = false;
            hasCachedAncestor          = true;
        }
        output = world.entities[slot].worldTransform;
        return true;
    }

    // Marks a subtree's world transforms dirty using an explicit stack (no
    // recursion); early-out when the root is already dirty.
    void invalidateWorldTransforms(WorldRecord &world, std::uint32_t root)
    {
        if (root >= world.entities.size() || !world.entities[root].alive)
            return;

        if (world.entities[root].worldTransformDirty)
            return;
        try
        {
            world.transformDirtyStack.reserve(world.entities.size());
        }
        catch (...)
        {
            for (EntitySlot &record : world.entities)
                if (record.alive)
                    record.worldTransformDirty = true;
            return;
        }
        world.transformDirtyStack.clear();
        world.transformDirtyStack.push_back(root);
        while (!world.transformDirtyStack.empty())
        {
            const std::uint32_t slot = world.transformDirtyStack.back();
            world.transformDirtyStack.pop_back();
            if (slot >= world.entities.size() || !world.entities[slot].alive)
                continue;
            EntitySlot &record         = world.entities[slot];
            record.worldTransformDirty = true;
            world.transformDirtyStack.insert(
                world.transformDirtyStack.end(), record.children.begin(), record.children.end());
        }
    }

    // Depth of the ancestor chain; fails on dead slots or chains deeper
    // than maximumHierarchyDepth.
    bool hierarchyDepth(const WorldRecord &world, std::uint32_t slot, std::uint32_t &outDepth) const
    {
        outDepth              = 0;
        std::uint32_t current = slot;
        while (current != std::numeric_limits<std::uint32_t>::max())
        {
            if (current >= world.entities.size() || !world.entities[current].alive
                || ++outDepth > world.maximumHierarchyDepth)
                return false;
            current = world.entities[current].parentSlot;
        }
        return true;
    }

    // Height of the subtree rooted at slot (inclusive); used to check that
    // re-parenting a subtree stays within the depth limit.
    bool subtreeHeight(const WorldRecord &world, std::uint32_t root, std::uint32_t &outHeight) const
    {
        std::vector<std::pair<std::uint32_t, std::uint32_t>> pending;
        pending.emplace_back(root, 1u);
        outHeight = 0;
        while (!pending.empty())
        {
            const auto [slot, depth] = pending.back();
            pending.pop_back();
            if (slot >= world.entities.size() || !world.entities[slot].alive || depth > world.maximumHierarchyDepth)
                return false;
            outHeight = std::max(outHeight, depth);
            for (const std::uint32_t child : world.entities[slot].children)
                pending.emplace_back(child, depth + 1u);
        }
        return outHeight != 0;
    }

    // Description limits are clamped by the module configuration; entity
    // buffers are reserved up front. The handle doubles as the map key.
    EntityWorldHandle createWorldImpl(const EntityWorldDescription *description, EntityWorldError *error)
    {
        setError(error, EntityWorldError::None);
        if (worlds_.size() >= maximumWorlds_)
        {
            setError(error, EntityWorldError::CapacityExceeded);
            return invalidEntityWorld;
        }
        try
        {
            auto record              = std::make_unique<WorldRecord>();
            record->name             = description != nullptr && description->name != nullptr ? description->name : "world";
            record->maximumEntities  = description != nullptr && description->maximumEntities != 0
                                          ? std::min(description->maximumEntities, maximumEntities_)
                                          : maximumEntities_;
            record->maximumHierarchyDepth = description != nullptr && description->maximumHierarchyDepth != 0
                                                ? std::min(description->maximumHierarchyDepth, maximumDepth_)
                                                : maximumDepth_;
            const std::uint32_t capacity = description != nullptr && description->initialEntityCapacity != 0
                                               ? std::min(description->initialEntityCapacity, record->maximumEntities)
                                               : std::min(initialCapacity_, record->maximumEntities);
            record->entities.reserve(capacity);
            record->freeSlots.reserve(capacity);
            record->transformDirtyStack.reserve(capacity);

            EntityWorldHandle handle = invalidEntityWorld;
            for (std::size_t attempt = 0; attempt < worlds_.size() + 2u; ++attempt)
            {
                const EntityWorldHandle candidate = nextWorld_++;
                if (candidate != invalidEntityWorld && !worlds_.contains(candidate))
                {
                    handle = candidate;
                    break;
                }
            }
            if (handle == invalidEntityWorld)
            {
                setError(error, EntityWorldError::CapacityExceeded);
                return invalidEntityWorld;
            }
            const auto [iterator, inserted] = worlds_.emplace(handle, std::move(record));
            (void)iterator;
            if (!inserted)
            {
                setError(error, EntityWorldError::CapacityExceeded);
                return invalidEntityWorld;
            }
            return handle;
        }
        catch (...)
        {
            setError(error, EntityWorldError::ResourceFailure);
            return invalidEntityWorld;
        }
    }

    void destroyWorldImpl(EntityWorldHandle handle)
    {
        worlds_.erase(handle);
    }

    bool worldInfoImpl(EntityWorldHandle handle, EntityWorldInfo *output)
    {
        const WorldRecord *world = findWorld(handle);
        if (world == nullptr || output == nullptr)
            return false;
        output->name                  = world->name.c_str();
        output->entityCount           = world->aliveCount;
        output->maximumEntities       = world->maximumEntities;
        output->maximumHierarchyDepth = world->maximumHierarchyDepth;
        return true;
    }

    // Validates the transform and parent, reuses a free slot, normalizes the
    // rotation, and links the entity into the parent's child list.
    EntityHandle createEntityImpl(
        EntityWorldHandle handle, const EntityDescription *description, EntityWorldError *error)
    {
        setError(error, EntityWorldError::None);
        WorldRecord *world = findWorld(handle);
        if (world == nullptr)
        {
            setError(error, EntityWorldError::InvalidHandle);
            return invalidEntity;
        }
        const EntityDescription defaults{};
        if (description == nullptr)
            description = &defaults;
        if (!finiteTransform(description->localTransform))
        {
            setError(error, EntityWorldError::InvalidTransform);
            return invalidEntity;
        }
        std::uint32_t parentSlot = std::numeric_limits<std::uint32_t>::max();
        if (description->parent != invalidEntity && findEntity(*world, description->parent, &parentSlot) == nullptr)
        {
            setError(error, EntityWorldError::InvalidHierarchy);
            return invalidEntity;
        }
        if (parentSlot != std::numeric_limits<std::uint32_t>::max())
        {
            std::uint32_t parentDepth = 0;
            if (!hierarchyDepth(*world, parentSlot, parentDepth) || parentDepth >= world->maximumHierarchyDepth)
            {
                setError(error, EntityWorldError::InvalidHierarchy);
                return invalidEntity;
            }
        }
        if (world->aliveCount >= world->maximumEntities)
        {
            setError(error, EntityWorldError::CapacityExceeded);
            return invalidEntity;
        }

        try
        {
            std::string entityName = description->name != nullptr ? description->name : "entity";
            if (parentSlot != std::numeric_limits<std::uint32_t>::max())
            {
                auto &children = world->entities[parentSlot].children;
                children.reserve(children.size() + 1u);
            }

            std::uint32_t slot = 0;
            if (!world->freeSlots.empty())
            {
                slot = world->freeSlots.back();
                world->freeSlots.pop_back();
            }
            else
            {
                if (world->entities.size() >= world->maximumEntities)
                {
                    setError(error, EntityWorldError::CapacityExceeded);
                    return invalidEntity;
                }
                slot = static_cast<std::uint32_t>(world->entities.size());
                world->entities.emplace_back();
            }

            EntitySlot &record       = world->entities[slot];
            record.alive             = true;
            record.enabled           = description->enabled;
            record.parentSlot        = parentSlot;
            record.name              = std::move(entityName);
            record.local             = description->localTransform;
            normalizeQuaternion(record.local.rotation);
            record.worldTransformDirty = true;
            record.children.clear();
            if (parentSlot != std::numeric_limits<std::uint32_t>::max())
                world->entities[parentSlot].children.push_back(slot);
            ++world->aliveCount;
            return makeEntityHandle(slot, record.generation);
        }
        catch (...)
        {
            setError(error, EntityWorldError::ResourceFailure);
            return invalidEntity;
        }
    }

    // Detaches from the parent, clears state, and advances the generation.
    // A slot at generation exhaustion is retired instead of reusing a handle.
    void eraseSlot(WorldRecord &world, std::uint32_t slot)
    {
        EntitySlot &record = world.entities[slot];
        if (record.parentSlot != std::numeric_limits<std::uint32_t>::max() && record.parentSlot < world.entities.size())
        {
            auto &siblings = world.entities[record.parentSlot].children;
            siblings.erase(std::remove(siblings.begin(), siblings.end(), slot), siblings.end());
        }
        record.alive      = false;
        record.enabled    = false;
        record.parentSlot = std::numeric_limits<std::uint32_t>::max();
        record.name.clear();
        record.children.clear();
        const bool retireSlot = record.generation == std::numeric_limits<std::uint32_t>::max();
        record.generation = retireSlot ? 0u : record.generation + 1u;
        if (!retireSlot)
            world.freeSlots.push_back(slot);
        --world.aliveCount;
    }

    // Non-recursive destroy re-parents the children first, preserving each
    // child's world transform by recomputing its local transform relative to
    // the new parent. Recursive destroy collects the subtree and erases
    // children before parents.
    bool destroyEntityImpl(EntityWorldHandle handle, EntityHandle entity, bool recursive, EntityWorldError *error)
    {
        setError(error, EntityWorldError::None);
        WorldRecord  *world = findWorld(handle);
        std::uint32_t root  = 0;
        if (world == nullptr || findEntity(*world, entity, &root) == nullptr)
        {
            setError(error, EntityWorldError::InvalidHandle);
            return false;
        }
        try
        {
        if (!recursive)
        {
            const std::uint32_t newParent = world->entities[root].parentSlot;
            Transform           parentWorld{};
            if (newParent != std::numeric_limits<std::uint32_t>::max()
                && !computeWorldTransform(*world, newParent, parentWorld))
            {
                setError(error, EntityWorldError::InvalidHierarchy);
                return false;
            }

            const auto                                      &children = world->entities[root].children;
            std::vector<std::pair<std::uint32_t, Transform>> replacements;
            replacements.reserve(children.size());
            for (const std::uint32_t child : children)
            {
                Transform childWorld{};
                Transform childLocal{};
                if (!computeWorldTransform(*world, child, childWorld)
                    || (newParent != std::numeric_limits<std::uint32_t>::max()
                        && !relativeTo(parentWorld, childWorld, childLocal)))
                {
                    setError(error, EntityWorldError::InvalidHierarchy);
                    return false;
                }
                if (newParent == std::numeric_limits<std::uint32_t>::max())
                    childLocal = childWorld;
                replacements.emplace_back(child, childLocal);
            }

            if (newParent != std::numeric_limits<std::uint32_t>::max())
                world->entities[newParent].children.reserve(
                    world->entities[newParent].children.size() + children.size());
            world->freeSlots.reserve(world->freeSlots.size() + 1u);
            for (const auto &[child, local] : replacements)
            {
                world->entities[child].parentSlot = newParent;
                world->entities[child].local      = local;
                if (newParent != std::numeric_limits<std::uint32_t>::max())
                    world->entities[newParent].children.push_back(child);
            }
            world->entities[root].children.clear();
            eraseSlot(*world, root);
            return true;
        }
        std::vector<std::uint32_t> pending{root};
        std::vector<std::uint32_t> removal;
        while (!pending.empty())
        {
            const std::uint32_t current = pending.back();
            pending.pop_back();
            if (current >= world->entities.size() || !world->entities[current].alive)
                continue;
            removal.push_back(current);
            pending.insert(
                pending.end(), world->entities[current].children.begin(), world->entities[current].children.end());
        }
        world->freeSlots.reserve(world->freeSlots.size() + removal.size());
        for (auto iterator = removal.rbegin(); iterator != removal.rend(); ++iterator)
            eraseSlot(*world, *iterator);
        return true;
        }
        catch (...)
        {
            setError(error, EntityWorldError::ResourceFailure);
            return false;
        }
    }

    bool entityExistsImpl(EntityWorldHandle handle, EntityHandle entity)
    {
        WorldRecord *world = findWorld(handle);
        return world != nullptr && findEntity(*world, entity) != nullptr;
    }

    bool setEntityEnabledImpl(EntityWorldHandle handle, EntityHandle entity, bool enabled)
    {
        WorldRecord *world  = findWorld(handle);
        EntitySlot  *record = world != nullptr ? findEntity(*world, entity) : nullptr;
        if (record == nullptr)
            return false;
        record->enabled = enabled;
        return true;
    }

    bool isEntityEnabledImpl(EntityWorldHandle handle, EntityHandle entity, bool *output)
    {
        WorldRecord *world  = findWorld(handle);
        EntitySlot  *record = world != nullptr ? findEntity(*world, entity) : nullptr;
        if (record == nullptr || output == nullptr)
            return false;
        *output = record->enabled;
        return true;
    }

    // No-op (success) when the transform is unchanged; otherwise stores the
    // normalized transform and marks the subtree dirty.
    bool setLocalTransformImpl(
        EntityWorldHandle handle, EntityHandle entity, const Transform *transform, EntityWorldError *error)
    {
        setError(error, EntityWorldError::None);
        WorldRecord *world  = findWorld(handle);
        EntitySlot  *record = world != nullptr ? findEntity(*world, entity) : nullptr;
        if (record == nullptr)
        {
            setError(error, EntityWorldError::InvalidHandle);
            return false;
        }
        if (transform == nullptr || !finiteTransform(*transform))
        {
            setError(error, EntityWorldError::InvalidTransform);
            return false;
        }
        Transform normalized = *transform;
        normalizeQuaternion(normalized.rotation);
        const bool unchanged
            = std::equal(
                  std::begin(record->local.position), std::end(record->local.position), std::begin(normalized.position))
           && std::equal(
               std::begin(record->local.rotation), std::end(record->local.rotation), std::begin(normalized.rotation))
           && std::equal(std::begin(record->local.scale), std::end(record->local.scale), std::begin(normalized.scale));
        if (unchanged)
            return true;
        record->local = normalized;
        invalidateWorldTransforms(*world, static_cast<std::uint32_t>(record - world->entities.data()));
        return true;
    }

    bool getLocalTransformImpl(EntityWorldHandle handle, EntityHandle entity, Transform *output)
    {
        WorldRecord      *world  = findWorld(handle);
        const EntitySlot *record = world != nullptr ? findEntity(*world, entity) : nullptr;
        if (record == nullptr || output == nullptr)
            return false;
        *output = record->local;
        return true;
    }

    bool getWorldTransformImpl(EntityWorldHandle handle, EntityHandle entity, Transform *output)
    {
        WorldRecord  *world = findWorld(handle);
        std::uint32_t slot  = 0;
        if (world == nullptr || output == nullptr || findEntity(*world, entity, &slot) == nullptr)
            return false;
        return computeWorldTransform(*world, slot, *output);
    }

    // Rejects self-parenting and cycles (walks the new ancestor chain),
    // checks the resulting depth against maximumHierarchyDepth, and with
    // preserveWorld re-expresses the local transform relative to the new
    // parent.
    bool setParentImpl(
        EntityWorldHandle handle, EntityHandle entity, EntityHandle parent, bool preserveWorld, EntityWorldError *error)
    {
        setError(error, EntityWorldError::None);
        try
        {
        WorldRecord  *world      = findWorld(handle);
        std::uint32_t entitySlot = 0;
        EntitySlot   *record     = world != nullptr ? findEntity(*world, entity, &entitySlot) : nullptr;
        if (record == nullptr)
        {
            setError(error, EntityWorldError::InvalidHandle);
            return false;
        }
        std::uint32_t parentSlot = std::numeric_limits<std::uint32_t>::max();
        if (parent != invalidEntity && findEntity(*world, parent, &parentSlot) == nullptr)
        {
            setError(error, EntityWorldError::InvalidHandle);
            return false;
        }
        if (parentSlot == entitySlot)
        {
            setError(error, EntityWorldError::InvalidHierarchy);
            return false;
        }
        std::uint32_t ancestor = parentSlot;
        while (ancestor != std::numeric_limits<std::uint32_t>::max())
        {
            if (ancestor == entitySlot || ancestor >= world->entities.size())
            {
                setError(error, EntityWorldError::InvalidHierarchy);
                return false;
            }
            ancestor = world->entities[ancestor].parentSlot;
        }
        std::uint32_t parentDepth         = 0;
        std::uint32_t movingSubtreeHeight = 0;
        if ((parentSlot != std::numeric_limits<std::uint32_t>::max()
                && !hierarchyDepth(*world, parentSlot, parentDepth))
            || !subtreeHeight(*world, entitySlot, movingSubtreeHeight)
            || parentDepth > world->maximumHierarchyDepth - movingSubtreeHeight)
        {
            setError(error, EntityWorldError::InvalidHierarchy);
            return false;
        }
        Transform oldWorld{};
        if (preserveWorld && !computeWorldTransform(*world, entitySlot, oldWorld))
        {
            setError(error, EntityWorldError::InvalidHierarchy);
            return false;
        }
        Transform newLocal = record->local;
        if (preserveWorld && parentSlot != std::numeric_limits<std::uint32_t>::max())
        {
            Transform parentWorld{};
            if (!computeWorldTransform(*world, parentSlot, parentWorld) || !relativeTo(parentWorld, oldWorld, newLocal))
            {
                setError(error, EntityWorldError::InvalidTransform);
                return false;
            }
        }
        else if (preserveWorld)
        {
            newLocal = oldWorld;
        }
        if (parentSlot != std::numeric_limits<std::uint32_t>::max())
            world->entities[parentSlot].children.reserve(world->entities[parentSlot].children.size() + 1u);
        if (record->parentSlot != std::numeric_limits<std::uint32_t>::max())
        {
            auto &siblings = world->entities[record->parentSlot].children;
            siblings.erase(std::remove(siblings.begin(), siblings.end(), entitySlot), siblings.end());
        }
        record->parentSlot = parentSlot;
        if (parentSlot != std::numeric_limits<std::uint32_t>::max())
            world->entities[parentSlot].children.push_back(entitySlot);
        record->local = newLocal;
        invalidateWorldTransforms(*world, entitySlot);
        return true;
        }
        catch (...)
        {
            setError(error, EntityWorldError::ResourceFailure);
            return false;
        }
    }

    EntityHandle getParentImpl(EntityWorldHandle handle, EntityHandle entity)
    {
        WorldRecord      *world  = findWorld(handle);
        const EntitySlot *record = world != nullptr ? findEntity(*world, entity) : nullptr;
        if (record == nullptr || record->parentSlot == std::numeric_limits<std::uint32_t>::max())
            return invalidEntity;
        const EntitySlot &parent = world->entities[record->parentSlot];
        return makeEntityHandle(record->parentSlot, parent.generation);
    }

    // Slot-order enumeration; copies up to capacity and reports the total so
    // callers can retry with a larger buffer.
    std::uint32_t copyEntitiesImpl(
        EntityWorldHandle handle, EntityHandle *output, std::uint32_t capacity, std::uint32_t *outTotal)
    {
        WorldRecord *world = findWorld(handle);
        if (world == nullptr)
        {
            if (outTotal)
                *outTotal = 0;
            return 0;
        }
        std::uint32_t total = 0, copied = 0;
        for (std::uint32_t slot = 0; slot < world->entities.size(); ++slot)
        {
            const EntitySlot &record = world->entities[slot];
            if (!record.alive)
                continue;
            if (output != nullptr && copied < capacity)
                output[copied++] = makeEntityHandle(slot, record.generation);
            ++total;
        }
        if (outTotal != nullptr)
            *outTotal = total;
        return copied;
    }

    bool copyEntityNameImpl(
        EntityWorldHandle handle, EntityHandle entity, char *output, std::uint32_t capacity, std::uint32_t *outRequired)
    {
        WorldRecord      *world  = findWorld(handle);
        const EntitySlot *record = world != nullptr ? findEntity(*world, entity) : nullptr;
        if (record == nullptr)
            return false;
        const std::uint32_t required = static_cast<std::uint32_t>(record->name.size() + 1);
        if (outRequired != nullptr)
            *outRequired = required;
        if (output == nullptr || capacity < required)
            return false;
        std::memcpy(output, record->name.c_str(), required);
        return true;
    }

    // Binary format: magic "WRLD" (0x444C5257), version 5, world name, then
    // per-slot generations and one record per alive entity (slot, parent,
    // enabled byte, name, local transform). Truncates any existing file.
    bool saveWorldImpl(EntityWorldHandle handle, const char *filePath, EntityWorldError *error)
    {
        setError(error, EntityWorldError::None);
        WorldRecord *world = findWorld(handle);
        if (world == nullptr || filePath == nullptr || filePath[0] == '\0')
        {
            setError(error, EntityWorldError::InvalidHandle);
            return false;
        }
        constexpr std::size_t maximumStoredNameBytes = 1'048'576u;
        if (world->name.size() > maximumStoredNameBytes)
        {
            setError(error, EntityWorldError::ResourceFailure);
            return false;
        }
        for (const EntitySlot &record : world->entities)
        {
            if (record.alive && record.name.size() > maximumStoredNameBytes)
            {
                setError(error, EntityWorldError::ResourceFailure);
                return false;
            }
        }
        std::ofstream stream(filePath, std::ios::binary | std::ios::trunc);
        if (!stream)
        {
            setError(error, EntityWorldError::ResourceFailure);
            return false;
        }
        constexpr std::uint32_t magic   = 0x444C5257u;
        constexpr std::uint32_t version = 5u;
        stream.write(reinterpret_cast<const char *>(&magic), sizeof(magic));
        stream.write(reinterpret_cast<const char *>(&version), sizeof(version));
        const std::uint32_t nameSize = static_cast<std::uint32_t>(world->name.size());
        stream.write(reinterpret_cast<const char *>(&nameSize), sizeof(nameSize));
        stream.write(world->name.data(), static_cast<std::streamsize>(nameSize));
        const std::uint32_t slotCount = static_cast<std::uint32_t>(world->entities.size());
        stream.write(reinterpret_cast<const char *>(&slotCount), sizeof(slotCount));
        stream.write(reinterpret_cast<const char *>(&world->aliveCount), sizeof(world->aliveCount));
        stream.write(
            reinterpret_cast<const char *>(&world->maximumHierarchyDepth), sizeof(world->maximumHierarchyDepth));
        for (const EntitySlot &record : world->entities)
            stream.write(reinterpret_cast<const char *>(&record.generation), sizeof(record.generation));
        for (std::uint32_t slot = 0; slot < world->entities.size(); ++slot)
        {
            const EntitySlot &record = world->entities[slot];
            if (!record.alive)
                continue;
            const std::uint32_t parent  = record.parentSlot;
            const std::uint32_t length  = static_cast<std::uint32_t>(record.name.size());
            const std::uint8_t  enabled = record.enabled ? 1u : 0u;
            stream.write(reinterpret_cast<const char *>(&slot), sizeof(slot));
            stream.write(reinterpret_cast<const char *>(&parent), sizeof(parent));
            stream.write(reinterpret_cast<const char *>(&enabled), sizeof(enabled));
            stream.write(reinterpret_cast<const char *>(&length), sizeof(length));
            stream.write(record.name.data(), static_cast<std::streamsize>(length));
            stream.write(reinterpret_cast<const char *>(&record.local), sizeof(record.local));
        }
        if (!stream)
        {
            setError(error, EntityWorldError::ResourceFailure);
            return false;
        }
        return true;
    }

    // Reads versions 2-5 (3 adds per-slot generations, 4 the stored depth,
    // 5 reserves generation zero for slots retired after generation exhaustion).
    // Everything is validated before the world is replaced: slot bounds,
    // duplicate slots, transforms, parent occupancy, depth, and cycles.
    bool loadWorldImpl(EntityWorldHandle handle, const char *filePath, EntityWorldError *error)
    {
        setError(error, EntityWorldError::None);
        WorldRecord *world = findWorld(handle);
        if (world == nullptr || filePath == nullptr || filePath[0] == '\0')
        {
            setError(error, EntityWorldError::InvalidHandle);
            return false;
        }
        constexpr std::uintmax_t maximumWorldFileBytes = 512ull * 1024ull * 1024ull;
        constexpr std::uint64_t  maximumSlotStorageBytes = 256ull * 1024ull * 1024ull;
        constexpr std::uint64_t  maximumTotalNameBytes = 128ull * 1024ull * 1024ull;
        std::error_code          fileSizeError;
        const std::uintmax_t     fileSize = std::filesystem::file_size(filePath, fileSizeError);
        if (fileSizeError || fileSize > maximumWorldFileBytes)
        {
            setError(error, EntityWorldError::ResourceFailure);
            return false;
        }
        std::ifstream stream(filePath, std::ios::binary);
        if (!stream)
        {
            setError(error, EntityWorldError::ResourceFailure);
            return false;
        }
        try
        {
        std::uint32_t magic = 0, version = 0, nameSize = 0, slotCount = 0, aliveCount = 0;
        stream.read(reinterpret_cast<char *>(&magic), sizeof(magic));
        stream.read(reinterpret_cast<char *>(&version), sizeof(version));
        stream.read(reinterpret_cast<char *>(&nameSize), sizeof(nameSize));
        if (!stream || magic != 0x444C5257u
            || (version != 2u && version != 3u && version != 4u && version != 5u)
            || nameSize > 1'048'576u)
        {
            setError(error, EntityWorldError::ResourceFailure);
            return false;
        }
        std::string worldName(nameSize, '\0');
        stream.read(worldName.data(), static_cast<std::streamsize>(nameSize));
        stream.read(reinterpret_cast<char *>(&slotCount), sizeof(slotCount));
        stream.read(reinterpret_cast<char *>(&aliveCount), sizeof(aliveCount));
        std::uint32_t loadedMaximumDepth = world->maximumHierarchyDepth;
        if (version >= 4u)
            stream.read(reinterpret_cast<char *>(&loadedMaximumDepth), sizeof(loadedMaximumDepth));
        if (!stream || slotCount > world->maximumEntities
            || static_cast<std::uint64_t>(slotCount) * sizeof(EntitySlot) > maximumSlotStorageBytes
            || aliveCount > slotCount || loadedMaximumDepth == 0
            || loadedMaximumDepth > world->maximumHierarchyDepth)
        {
            setError(error, EntityWorldError::CapacityExceeded);
            return false;
        }
        std::vector<EntitySlot> replacement(slotCount);
        std::vector<bool>       occupied(slotCount, false);
        if (version >= 3u)
        {
            for (EntitySlot &record : replacement)
            {
                stream.read(reinterpret_cast<char *>(&record.generation), sizeof(record.generation));
                if (!stream)
                {
                    setError(error, EntityWorldError::ResourceFailure);
                    return false;
                }
            }
        }
        std::uint64_t totalNameBytes = nameSize;
        for (std::uint32_t index = 0; index < aliveCount; ++index)
        {
            std::uint32_t slot = 0, parent = std::numeric_limits<std::uint32_t>::max(), length = 0;
            std::uint8_t  enabled    = 0;
            std::uint32_t generation = 1;
            stream.read(reinterpret_cast<char *>(&slot), sizeof(slot));
            if (version == 2u)
                stream.read(reinterpret_cast<char *>(&generation), sizeof(generation));
            stream.read(reinterpret_cast<char *>(&parent), sizeof(parent));
            stream.read(reinterpret_cast<char *>(&enabled), sizeof(enabled));
            stream.read(reinterpret_cast<char *>(&length), sizeof(length));
            if (!stream || slot >= slotCount || occupied[slot] || length > 1'048'576u
                || length > maximumTotalNameBytes - std::min(maximumTotalNameBytes, totalNameBytes)
                || enabled > 1u)
            {
                setError(error, EntityWorldError::ResourceFailure);
                return false;
            }
            occupied[slot]     = true;
            EntitySlot &record = replacement[slot];
            if (record.generation == 0)
            {
                setError(error, EntityWorldError::ResourceFailure);
                return false;
            }
            if (version == 2u)
            {
                if (generation == 0)
                {
                    setError(error, EntityWorldError::ResourceFailure);
                    return false;
                }
                record.generation = generation;
            }
            record.alive      = true;
            record.enabled    = enabled != 0;
            record.parentSlot = parent;
            totalNameBytes += length;
            record.name.resize(length);
            stream.read(record.name.data(), static_cast<std::streamsize>(length));
            stream.read(reinterpret_cast<char *>(&record.local), sizeof(record.local));
            if (!stream || !finiteTransform(record.local)
                || (parent != std::numeric_limits<std::uint32_t>::max() && parent >= slotCount))
            {
                setError(error, EntityWorldError::ResourceFailure);
                return false;
            }
            normalizeQuaternion(record.local.rotation);
        }
        for (std::uint32_t slot = 0; slot < slotCount; ++slot)
        {
            if (!occupied[slot])
                continue;
            const std::uint32_t parent = replacement[slot].parentSlot;
            if (parent != std::numeric_limits<std::uint32_t>::max())
            {
                if (!occupied[parent] || parent == slot)
                {
                    setError(error, EntityWorldError::InvalidHierarchy);
                    return false;
                }
                replacement[parent].children.push_back(slot);
            }
        }
        for (std::uint32_t slot = 0; slot < slotCount; ++slot)
        {
            if (!occupied[slot])
                continue;
            std::uint32_t ancestor = replacement[slot].parentSlot, depth = 0;
            while (ancestor != std::numeric_limits<std::uint32_t>::max())
            {
                if (++depth > loadedMaximumDepth || ancestor >= slotCount || !occupied[ancestor] || ancestor == slot)
                {
                    setError(error, EntityWorldError::InvalidHierarchy);
                    return false;
                }
                ancestor = replacement[ancestor].parentSlot;
            }
        }
        if (stream.peek() != std::char_traits<char>::eof())
        {
            setError(error, EntityWorldError::ResourceFailure);
            return false;
        }
        std::vector<std::uint32_t> freeSlots;
        for (std::uint32_t slot = 0; slot < slotCount; ++slot)
            if (!occupied[slot] && replacement[slot].generation != 0)
                freeSlots.push_back(slot);
        world->entities              = std::move(replacement);
        world->freeSlots             = std::move(freeSlots);
        world->aliveCount            = aliveCount;
        world->maximumHierarchyDepth = loadedMaximumDepth;
        world->name                  = std::move(worldName);
        return true;
        }
        catch (...)
        {
            setError(error, EntityWorldError::ResourceFailure);
            return false;
        }
    }

    ParticipantManifest                                                 manifest_;
    EntityWorldCapability                                               capability_{};
    std::unordered_map<EntityWorldHandle, std::unique_ptr<WorldRecord>> worlds_;
    EntityWorldHandle                                                   nextWorld_       = 1;
    std::uint32_t                                                       initialCapacity_ = 256;
    std::uint32_t                                                       maximumEntities_ = 1'000'000;
    std::uint32_t                                                       maximumDepth_    = 256;
    std::size_t                                                         maximumWorlds_   = 1024;
};

} // namespace

#if defined(ENGINE_WORLD_EMBEDDED)
namespace detail
{
// Compiled-in creation hook used by the world module facade (world.cpp).
engine::ModuleInterface *createEmbeddedModule()
{
    return new EntityWorldModule();
}
} // namespace detail
#endif

} // namespace engine::entity_world

// Module entry point for the standalone entity_world .so; omitted in
// embedded builds.
#if !defined(ENGINE_WORLD_EMBEDDED)
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"entity_world"};
    static const engine::EngineModuleEntryTable entry
        = {.abiVersion     = engine::engineModuleAbiVersion,
            .moduleName    = "entity_world",
            .moduleVersion = "0.1.0",
            .createModule = []() -> engine::ModuleInterface * { return new engine::entity_world::EntityWorldModule(); },
            .providedCapabilities    = provided,
            .providedCapabilityCount = 1,
            .requiredCapabilities    = nullptr,
            .requiredCapabilityCount = 0,
            .optionalCapabilities    = nullptr,
            .optionalCapabilityCount = 0,
            .declaredPermissions     = nullptr,
            .declaredPermissionCount = 0};
    return &entry;
}
#endif
