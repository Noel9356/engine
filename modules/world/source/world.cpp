// world module facade: owns embedded ecs and entity_world module instances
// and exposes both capabilities over one set of worlds. Every entity exists
// in both layers (one entity_world entity mirrored by one ecs entity) with
// generation-checked handle translation in both directions.

#include <engine/ecs/capabilities.hpp>
#include <engine/entity_world/capabilities.hpp>
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine::ecs::detail
{
ModuleInterface *createEmbeddedModule();
}

namespace engine::entity_world::detail
{
ModuleInterface *createEmbeddedModule();
}

namespace engine::world
{
namespace
{
using Entity = engine::ecs::Entity;

// One row of a bidirectional handle map: the source handle's generation and
// the mapped handle in the other layer. generation == 0 marks an unused row.
struct EntityMapEntry
{
    std::uint32_t generation   = 0;
    std::uint32_t padding      = 0;
    std::uint64_t mappedHandle = 0;
};

// One facade world: the underlying worlds in each layer, the two handle maps,
// and bookkeeping. commandBufferDestroys remembers which entity-world
// entities an ecs command buffer plans to destroy so the destroy can be
// mirrored when the buffer is applied. activeEcsQueries gates structural
// facade calls while a bridged query runs.
struct WorldBinding
{
    engine::ecs::WorldHandle                ecsWorld    = engine::ecs::invalidWorld;
    engine::entity_world::EntityWorldHandle entityWorld = engine::entity_world::invalidEntityWorld;

    std::vector<EntityMapEntry> entityToEcs;
    std::vector<EntityMapEntry> ecsToEntity;
    std::unordered_map<engine::ecs::CommandBufferHandle, std::vector<engine::entity_world::EntityHandle>>
                  commandBufferDestroys;
    std::uint32_t activeEcsQueries = 0;
};

// Splits a handle into slot and generation; both must be non-zero.
bool decodeHandle(std::uint64_t handle, std::uint32_t &slot, std::uint32_t &generation) noexcept
{
    const std::uint32_t encodedSlot = static_cast<std::uint32_t>(handle);
    generation                      = static_cast<std::uint32_t>(handle >> 32u);
    if (encodedSlot == 0 || generation == 0)
        return false;
    slot = encodedSlot - 1u;
    return true;
}

// Module implementation: embeds the ecs and entity_world modules, forwards
// construct/start/stop to them, and serves both capabilities through facade
// tables that translate handles before delegating to the inner capabilities.
class WorldModule final : public engine::ModuleInterface
{
public:
    WorldModule()
        : ecsModule_(engine::ecs::detail::createEmbeddedModule()),
          entityWorldModule_(engine::entity_world::detail::createEmbeddedModule()),
          manifest_(makeManifest())
    {
        ecsCapability_ = static_cast<engine::ecs::EcsCapability *>(
            ecsModule_->capabilityPointerFor(engine::ecs::EcsCapability::capabilityName));
        entityWorldCapability_ = static_cast<engine::entity_world::EntityWorldCapability *>(
            entityWorldModule_->capabilityPointerFor(engine::entity_world::EntityWorldCapability::capabilityName));
        bindCapabilities();
    }

    const engine::ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    void construct(engine::ParticipantContext &context) override
    {
        ecsModule_->construct(context);
        entityWorldModule_->construct(context);
    }

    void start() override
    {
        // Starts both inner modules; if entity_world fails, ecs is stopped
        // again so the module stays consistent before rethrowing.
        ecsModule_->start();
        try
        {
            entityWorldModule_->start();
        }
        catch (...)
        {
            ecsModule_->stop();
            throw;
        }
    }

    void stop() override
    {
        worlds_.clear();
        entityWorldModule_->stop();
        ecsModule_->stop();
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        if (name == engine::ecs::EcsCapability::capabilityName)
            return &ecsFacade_;
        if (name == engine::entity_world::EntityWorldCapability::capabilityName)
            return &entityWorldFacade_;
        return nullptr;
    }

private:
    static engine::ParticipantManifest makeManifest()
    {
        return engine::ParticipantManifest{
            .name                 = "world",
            .kind                 = engine::ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"ecs", "entity_world"}};
    }

    // Binding lookups. The ecs direction is a linear scan because the number
    // of open worlds is small and ecs handles are only inputs here.
    WorldBinding *world(engine::entity_world::EntityWorldHandle handle)
    {
        const auto found = worlds_.find(handle);
        return found == worlds_.end() ? nullptr : &found->second;
    }

    const WorldBinding *world(engine::entity_world::EntityWorldHandle handle) const
    {
        const auto found = worlds_.find(handle);
        return found == worlds_.end() ? nullptr : &found->second;
    }

    WorldBinding *worldFromEcs(engine::ecs::WorldHandle handle)
    {
        for (auto &[external, binding] : worlds_)
            if (binding.ecsWorld == handle)
                return &binding;
        return nullptr;
    }

    // Translate a handle across the layer boundary; a wrong generation or
    // unknown slot maps to the invalid entity.
    Entity mapEntity(const WorldBinding &binding, engine::entity_world::EntityHandle external) const noexcept
    {
        std::uint32_t slot       = 0;
        std::uint32_t generation = 0;
        if (!decodeHandle(external, slot, generation) || slot >= binding.entityToEcs.size())
            return engine::ecs::invalidEntity;
        const EntityMapEntry &entry = binding.entityToEcs[slot];
        return entry.generation == generation ? entry.mappedHandle : engine::ecs::invalidEntity;
    }

    engine::entity_world::EntityHandle externalEntity(const WorldBinding &binding, Entity entity) const noexcept
    {
        std::uint32_t slot       = 0;
        std::uint32_t generation = 0;
        if (!decodeHandle(entity, slot, generation) || slot >= binding.ecsToEntity.size())
            return engine::entity_world::invalidEntity;
        const EntityMapEntry &entry = binding.ecsToEntity[slot];
        return entry.generation == generation ? entry.mappedHandle : engine::entity_world::invalidEntity;
    }

    // Install/remove one row in both maps. Binding fails when either row is
    // already occupied (generation != 0).
    bool bindEntity(WorldBinding &binding, engine::entity_world::EntityHandle external, Entity internal)
    {
        std::uint32_t externalSlot       = 0;
        std::uint32_t externalGeneration = 0;
        std::uint32_t internalSlot       = 0;
        std::uint32_t internalGeneration = 0;
        if (!decodeHandle(external, externalSlot, externalGeneration)
            || !decodeHandle(internal, internalSlot, internalGeneration))
            return false;
        try
        {
            if (binding.entityToEcs.size() <= externalSlot)
                binding.entityToEcs.resize(static_cast<std::size_t>(externalSlot) + 1u);
            if (binding.ecsToEntity.size() <= internalSlot)
                binding.ecsToEntity.resize(static_cast<std::size_t>(internalSlot) + 1u);
        }
        catch (...)
        {
            return false;
        }
        EntityMapEntry &forward = binding.entityToEcs[externalSlot];
        EntityMapEntry &reverse = binding.ecsToEntity[internalSlot];
        if (forward.generation != 0 || reverse.generation != 0)
            return false;
        forward = EntityMapEntry{externalGeneration, 0, internal};
        reverse = EntityMapEntry{internalGeneration, 0, external};
        return true;
    }

    void unbindEntity(WorldBinding &binding, engine::entity_world::EntityHandle external) noexcept
    {
        std::uint32_t slot       = 0;
        std::uint32_t generation = 0;
        if (!decodeHandle(external, slot, generation) || slot >= binding.entityToEcs.size())
            return;
        EntityMapEntry &forward = binding.entityToEcs[slot];
        if (forward.generation != generation)
            return;
        const Entity  internal           = forward.mappedHandle;
        std::uint32_t internalSlot       = 0;
        std::uint32_t internalGeneration = 0;
        if (decodeHandle(internal, internalSlot, internalGeneration) && internalSlot < binding.ecsToEntity.size())
        {
            EntityMapEntry &reverse = binding.ecsToEntity[internalSlot];
            if (reverse.generation == internalGeneration)
                reverse = {};
        }
        forward = {};
    }

    // After an ecs command buffer applied destroys: destroy and unbind every
    // entity-world mirror whose ecs entity no longer exists.
    void reconcileDestroyedEcsEntities(
        WorldBinding &binding, const std::vector<engine::entity_world::EntityHandle> &candidates)
    {
        for (const auto external : candidates)
        {
            const Entity internal = mapEntity(binding, external);
            if (internal == engine::ecs::invalidEntity
                || ecsCapability_->entityExists(ecsCapability_->self, binding.ecsWorld, internal))
                continue;
            entityWorldCapability_->destroyEntity(
                entityWorldCapability_->self, binding.entityWorld, external, false, nullptr);
            unbindEntity(binding, external);
        }
    }

    // Collects the entity itself and, for recursive destroys, every
    // descendant (found by walking parents of all entities) so the ecs
    // mirrors can be destroyed after the entity_world destroy succeeds.
    bool collectEntityWorldDestroyCandidates(WorldBinding &binding, engine::entity_world::EntityHandle root,
        bool recursive, std::vector<engine::entity_world::EntityHandle> &candidates)
    {
        candidates.clear();
        try
        {
            candidates.push_back(root);
            if (!recursive)
                return true;

            std::uint32_t total = 0;
            entityWorldCapability_->copyEntities(entityWorldCapability_->self, binding.entityWorld, nullptr, 0, &total);
            std::vector<engine::entity_world::EntityHandle> entities(total);
            if (total != 0)
            {
                const std::uint32_t copied = entityWorldCapability_->copyEntities(
                    entityWorldCapability_->self, binding.entityWorld, entities.data(), total, nullptr);
                if (copied != total)
                    return false;
            }

            for (const auto candidate : entities)
            {
                if (candidate == root)
                    continue;
                auto ancestor
                    = entityWorldCapability_->getParent(entityWorldCapability_->self, binding.entityWorld, candidate);
                for (std::uint32_t depth = 0; ancestor != engine::entity_world::invalidEntity && depth < total; ++depth)
                {
                    if (ancestor == root)
                    {
                        candidates.push_back(candidate);
                        break;
                    }
                    ancestor = entityWorldCapability_->getParent(
                        entityWorldCapability_->self, binding.entityWorld, ancestor);
                }
            }
        }
        catch (...)
        {
            candidates.clear();
            return false;
        }
        return true;
    }

    // Mirror of reconcileDestroyedEcsEntities for destroys that started in
    // entity_world: destroy the ecs entity of every vanished mirror.
    void reconcileDestroyedEntityWorldEntities(
        WorldBinding &binding, const std::vector<engine::entity_world::EntityHandle> &candidates)
    {
        for (const auto external : candidates)
        {
            if (entityWorldCapability_->entityExists(entityWorldCapability_->self, binding.entityWorld, external))
                continue;
            const Entity internal = mapEntity(binding, external);
            if (internal != engine::ecs::invalidEntity)
            {
                ecsCapability_->destroyEntity(ecsCapability_->self, binding.ecsWorld, internal, nullptr);
                unbindEntity(binding, external);
            }
        }
    }

    // Translates ecs chunk callbacks: entity handles are mapped to
    // entity-world handles before the consumer's callback runs; a chunk with
    // an unmapped entity aborts the query (false return).
    struct QueryBridge
    {
        WorldModule               *module   = nullptr;
        WorldBinding              *binding  = nullptr;
        engine::ecs::ChunkCallback callback = nullptr;
        void                      *userData = nullptr;
    };

    static bool queryBridge(void *userData, const engine::ecs::ChunkView *chunk) noexcept
    {
        auto *bridge = static_cast<QueryBridge *>(userData);
        if (bridge == nullptr || bridge->module == nullptr || bridge->binding == nullptr || chunk == nullptr
            || bridge->callback == nullptr)
            return false;
        std::vector<engine::ecs::Entity>     entities;
        std::array<engine::ecs::Entity, 128> inlineEntities{};
        engine::ecs::Entity                 *output = inlineEntities.data();
        if (chunk->entityCount > inlineEntities.size())
        {
            try
            {
                entities.resize(chunk->entityCount);
                output = entities.data();
            }
            catch (...)
            {
                return false;
            }
        }
        for (std::uint32_t index = 0; index < chunk->entityCount; ++index)
        {
            output[index] = bridge->module->externalEntity(*bridge->binding, chunk->entities[index]);
            if (output[index] == engine::ecs::invalidEntity)
                return false;
        }
        engine::ecs::ChunkView translated = *chunk;
        translated.entities               = output;
        return bridge->callback(bridge->userData, &translated);
    }

    // Create the partner world so facade worlds always exist in both layers;
    // rolls the created side back when the partner fails.
    engine::entity_world::EntityWorldHandle createEntityWorldForEcs(
        const engine::ecs::WorldDescription *description, engine::ecs::WorldHandle *outEcsWorld)
    {
        if (outEcsWorld == nullptr)
            return engine::entity_world::invalidEntityWorld;
        *outEcsWorld = ecsCapability_->createWorld(ecsCapability_->self, description, nullptr);
        if (*outEcsWorld == engine::ecs::invalidWorld)
            return engine::entity_world::invalidEntityWorld;

        engine::entity_world::EntityWorldDescription entityDescription{};
        if (description != nullptr)
        {
            entityDescription.name                  = description->name;
            entityDescription.initialEntityCapacity = description->storage.initialEntityCapacity;
            entityDescription.maximumEntities       = description->storage.maximumEntities;
        }
        const auto entityWorld
            = entityWorldCapability_->createWorld(entityWorldCapability_->self, &entityDescription, nullptr);
        if (entityWorld == engine::entity_world::invalidEntityWorld)
        {
            ecsCapability_->destroyWorld(ecsCapability_->self, *outEcsWorld);
            *outEcsWorld = engine::ecs::invalidWorld;
        }
        return entityWorld;
    }

    engine::ecs::WorldHandle createEcsWorldForEntityWorld(
        const engine::entity_world::EntityWorldDescription *description,
        engine::entity_world::EntityWorldHandle             entityWorld)
    {
        engine::ecs::WorldDescription ecsDescription{};
        if (description != nullptr)
        {
            ecsDescription.name                          = description->name;
            ecsDescription.storage.initialEntityCapacity = description->initialEntityCapacity;
            ecsDescription.storage.maximumEntities       = description->maximumEntities;
        }
        const auto ecsWorld = ecsCapability_->createWorld(ecsCapability_->self, &ecsDescription, nullptr);
        if (ecsWorld == engine::ecs::invalidWorld)
            entityWorldCapability_->destroyWorld(entityWorldCapability_->self, entityWorld);
        return ecsWorld;
    }

    // Create the mirror entity in the other layer and bind it; on any
    // failure both sides are rolled back.
    engine::entity_world::EntityHandle createEntityForEcs(WorldBinding &binding, engine::ecs::Entity ecsEntity)
    {
        engine::entity_world::EntityDescription description{};
        description.name                                = "entity";
        engine::entity_world::EntityWorldError error    = engine::entity_world::EntityWorldError::None;
        const auto                             external = entityWorldCapability_->createEntity(
            entityWorldCapability_->self, binding.entityWorld, &description, &error);
        if (external == engine::entity_world::invalidEntity || !bindEntity(binding, external, ecsEntity))
        {
            if (external != engine::entity_world::invalidEntity)
                entityWorldCapability_->destroyEntity(
                    entityWorldCapability_->self, binding.entityWorld, external, false, nullptr);
            ecsCapability_->destroyEntity(ecsCapability_->self, binding.ecsWorld, ecsEntity, nullptr);
            return engine::entity_world::invalidEntity;
        }
        return external;
    }

    engine::ecs::Entity createEcsEntityForEntityWorld(
        WorldBinding &binding, engine::entity_world::EntityHandle external)
    {
        const auto internal = ecsCapability_->createEntity(ecsCapability_->self, binding.ecsWorld, nullptr);
        if (internal == engine::ecs::invalidEntity || !bindEntity(binding, external, internal))
        {
            if (internal != engine::ecs::invalidEntity)
                ecsCapability_->destroyEntity(ecsCapability_->self, binding.ecsWorld, internal, nullptr);
            entityWorldCapability_->destroyEntity(
                entityWorldCapability_->self, binding.entityWorld, external, false, nullptr);
            return engine::ecs::invalidEntity;
        }
        return internal;
    }

    // The two facade capability tables. Worlds are keyed by the entity-world
    // handle, so the ecs facade's world handle and the entity_world handle of
    // one binding are the same value.
    void bindCapabilities()
    {
        ecsFacade_.abi  = engine::ecs::ecsCapabilityAbi;
        ecsFacade_.self = this;
        // Returns the entity-world handle as the public world handle; the
        // inner ecs world handle stays internal.
        ecsFacade_.createWorld
            = [](void *self, const engine::ecs::WorldDescription *description, engine::ecs::StorageError *error)
        {
            auto      &module      = *static_cast<WorldModule *>(self);
            auto       ecsWorld    = engine::ecs::invalidWorld;
            const auto entityWorld = module.createEntityWorldForEcs(description, &ecsWorld);
            if (entityWorld == engine::entity_world::invalidEntityWorld)
            {
                if (error)
                    *error = engine::ecs::StorageError::ResourceFailure;
                return engine::ecs::invalidWorld;
            }
            WorldBinding binding{};
            binding.ecsWorld    = ecsWorld;
            binding.entityWorld = entityWorld;
            try
            {
                module.worlds_.emplace(entityWorld, std::move(binding));
            }
            catch (...)
            {
                module.entityWorldCapability_->destroyWorld(module.entityWorldCapability_->self, entityWorld);
                module.ecsCapability_->destroyWorld(module.ecsCapability_->self, ecsWorld);
                if (error)
                    *error = engine::ecs::StorageError::ResourceFailure;
                return engine::ecs::invalidWorld;
            }
            if (error)
                *error = engine::ecs::StorageError::None;
            return entityWorld;
        };
        ecsFacade_.destroyWorld = [](void *self, engine::ecs::WorldHandle handle)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding || binding->activeEcsQueries != 0)
                return;
            module.ecsCapability_->destroyWorld(module.ecsCapability_->self, binding->ecsWorld);
            module.entityWorldCapability_->destroyWorld(module.entityWorldCapability_->self, binding->entityWorld);
            module.worlds_.erase(handle);
        };
        ecsFacade_.worldInfo = [](void *self, engine::ecs::WorldHandle handle, engine::ecs::WorldInfo *out)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            return binding && module.ecsCapability_->worldInfo(module.ecsCapability_->self, binding->ecsWorld, out);
        };
        ecsFacade_.registerComponent = [](void                                     *self,
                                           engine::ecs::WorldHandle                 handle,
                                           const engine::ecs::ComponentDescription *description,
                                           engine::ecs::StorageError               *error)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidInput;
                return engine::ecs::invalidComponent;
            }
            return module.ecsCapability_->registerComponent(
                module.ecsCapability_->self, binding->ecsWorld, description, error);
        };
        // Public entities are entity-world handles; the ecs entity is the
        // internal mirror created for it.
        ecsFacade_.createEntity = [](void *self, engine::ecs::WorldHandle handle, engine::ecs::StorageError *error)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidInput;
                return engine::ecs::invalidEntity;
            }
            const auto internal
                = module.ecsCapability_->createEntity(module.ecsCapability_->self, binding->ecsWorld, error);
            if (internal == engine::ecs::invalidEntity)
                return engine::ecs::invalidEntity;
            const auto external = module.createEntityForEcs(*binding, internal);
            if (external == engine::entity_world::invalidEntity)
            {
                if (error)
                    *error = engine::ecs::StorageError::ResourceFailure;
                return engine::ecs::invalidEntity;
            }
            if (error)
                *error = engine::ecs::StorageError::None;
            return static_cast<engine::ecs::Entity>(external);
        };
        ecsFacade_.destroyEntity = [](void                       *self,
                                       engine::ecs::WorldHandle   handle,
                                       engine::ecs::Entity        entity,
                                       engine::ecs::StorageError *error)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidInput;
                return false;
            }
            if (binding->activeEcsQueries != 0)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidDuringQuery;
                return false;
            }
            const auto external = static_cast<engine::entity_world::EntityHandle>(entity);
            const auto internal = module.mapEntity(*binding, external);
            if (internal == engine::ecs::invalidEntity)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidEntity;
                return false;
            }
            engine::entity_world::EntityWorldError worldError{};
            if (!module.entityWorldCapability_->destroyEntity(
                    module.entityWorldCapability_->self, binding->entityWorld, external, false, &worldError))
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidEntity;
                return false;
            }
            const bool result
                = module.ecsCapability_->destroyEntity(module.ecsCapability_->self, binding->ecsWorld, internal, error);
            if (result)
                module.unbindEntity(*binding, external);
            return result;
        };
        ecsFacade_.entityExists = [](void *self, engine::ecs::WorldHandle handle, engine::ecs::Entity entity)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding)
                return false;
            const auto internal = module.mapEntity(*binding, static_cast<engine::entity_world::EntityHandle>(entity));
            return internal != engine::ecs::invalidEntity
                && module.ecsCapability_->entityExists(module.ecsCapability_->self, binding->ecsWorld, internal);
        };
        ecsFacade_.addComponent = [](void                       *self,
                                      engine::ecs::WorldHandle   handle,
                                      engine::ecs::Entity        entity,
                                      engine::ecs::ComponentId   component,
                                      const void                *value,
                                      engine::ecs::StorageError *error)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidInput;
                return false;
            }
            const auto internal = module.mapEntity(*binding, static_cast<engine::entity_world::EntityHandle>(entity));
            if (internal == engine::ecs::invalidEntity)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidEntity;
                return false;
            }
            return module.ecsCapability_->addComponent(
                module.ecsCapability_->self, binding->ecsWorld, internal, component, value, error);
        };
        ecsFacade_.removeComponent = [](void                       *self,
                                         engine::ecs::WorldHandle   handle,
                                         engine::ecs::Entity        entity,
                                         engine::ecs::ComponentId   component,
                                         engine::ecs::StorageError *error)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidInput;
                return false;
            }
            const auto internal = module.mapEntity(*binding, static_cast<engine::entity_world::EntityHandle>(entity));
            if (internal == engine::ecs::invalidEntity)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidEntity;
                return false;
            }
            return module.ecsCapability_->removeComponent(
                module.ecsCapability_->self, binding->ecsWorld, internal, component, error);
        };
        ecsFacade_.hasComponent = [](void                     *self,
                                      engine::ecs::WorldHandle handle,
                                      engine::ecs::Entity      entity,
                                      engine::ecs::ComponentId component)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding)
                return false;
            const auto internal = module.mapEntity(*binding, static_cast<engine::entity_world::EntityHandle>(entity));
            return internal != engine::ecs::invalidEntity
                && module.ecsCapability_->hasComponent(
                    module.ecsCapability_->self, binding->ecsWorld, internal, component);
        };
        ecsFacade_.getComponent = [](void                     *self,
                                      engine::ecs::WorldHandle handle,
                                      engine::ecs::Entity      entity,
                                      engine::ecs::ComponentId component) -> void *
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding)
                return nullptr;
            const auto internal = module.mapEntity(*binding, static_cast<engine::entity_world::EntityHandle>(entity));
            return internal == engine::ecs::invalidEntity
                     ? nullptr
                     : module.ecsCapability_->getComponent(
                           module.ecsCapability_->self, binding->ecsWorld, internal, component);
        };
        ecsFacade_.createQuery = [](void                                 *self,
                                     engine::ecs::WorldHandle             handle,
                                     const engine::ecs::QueryDescription *description,
                                     engine::ecs::StorageError           *error)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidInput;
                return engine::ecs::invalidQuery;
            }
            return module.ecsCapability_->createQuery(
                module.ecsCapability_->self, binding->ecsWorld, description, error);
        };
        ecsFacade_.destroyQuery = [](void *self, engine::ecs::WorldHandle handle, engine::ecs::QueryId query)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            return binding && binding->activeEcsQueries == 0
                && module.ecsCapability_->destroyQuery(module.ecsCapability_->self, binding->ecsWorld, query);
        };
        ecsFacade_.queryChunks = [](void                       *self,
                                     engine::ecs::WorldHandle   handle,
                                     engine::ecs::QueryId       query,
                                     engine::ecs::ChunkCallback callback,
                                     void                      *userData,
                                     engine::ecs::StorageError *error)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding || !callback)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidInput;
                return false;
            }
            QueryBridge bridge{&module, binding, callback, userData};
            struct QueryScope
            {
                std::uint32_t &depth;
                explicit QueryScope(std::uint32_t &value)
                    : depth(value)
                {
                    ++depth;
                }
                ~QueryScope()
                {
                    --depth;
                }
            } queryScope(binding->activeEcsQueries);
            return module.ecsCapability_->queryChunks(
                module.ecsCapability_->self, binding->ecsWorld, query, &queryBridge, &bridge, error);
        };
        ecsFacade_.createCommandBuffer
            = [](void *self, engine::ecs::WorldHandle handle, engine::ecs::StorageError *error)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidInput;
                return engine::ecs::invalidCommandBuffer;
            }
            const auto commandBuffer
                = module.ecsCapability_->createCommandBuffer(module.ecsCapability_->self, binding->ecsWorld, error);
            if (commandBuffer == engine::ecs::invalidCommandBuffer)
                return commandBuffer;
            try
            {
                binding->commandBufferDestroys.try_emplace(commandBuffer);
            }
            catch (...)
            {
                module.ecsCapability_->destroyCommandBuffer(
                    module.ecsCapability_->self, binding->ecsWorld, commandBuffer);
                if (error)
                    *error = engine::ecs::StorageError::ResourceFailure;
                return engine::ecs::invalidCommandBuffer;
            }
            return commandBuffer;
        };
        ecsFacade_.destroyCommandBuffer
            = [](void *self, engine::ecs::WorldHandle handle, engine::ecs::CommandBufferHandle commandBuffer)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding)
                return false;
            const bool destroyed = module.ecsCapability_->destroyCommandBuffer(
                module.ecsCapability_->self, binding->ecsWorld, commandBuffer);
            if (destroyed)
                binding->commandBufferDestroys.erase(commandBuffer);
            return destroyed;
        };
        ecsFacade_.commandAddDefaultComponent = [](void                             *self,
                                                    engine::ecs::WorldHandle         handle,
                                                    engine::ecs::CommandBufferHandle buffer,
                                                    engine::ecs::Entity              entity,
                                                    engine::ecs::ComponentId         component,
                                                    engine::ecs::StorageError       *error)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidInput;
                return false;
            }
            const auto internal = module.mapEntity(*binding, static_cast<engine::entity_world::EntityHandle>(entity));
            if (internal == engine::ecs::invalidEntity)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidEntity;
                return false;
            }
            return module.ecsCapability_->commandAddDefaultComponent(
                module.ecsCapability_->self, binding->ecsWorld, buffer, internal, component, error);
        };
        ecsFacade_.commandRemoveComponent = [](void                             *self,
                                                engine::ecs::WorldHandle         handle,
                                                engine::ecs::CommandBufferHandle buffer,
                                                engine::ecs::Entity              entity,
                                                engine::ecs::ComponentId         component,
                                                engine::ecs::StorageError       *error)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidInput;
                return false;
            }
            const auto internal = module.mapEntity(*binding, static_cast<engine::entity_world::EntityHandle>(entity));
            if (internal == engine::ecs::invalidEntity)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidEntity;
                return false;
            }
            return module.ecsCapability_->commandRemoveComponent(
                module.ecsCapability_->self, binding->ecsWorld, buffer, internal, component, error);
        };
        // Records the external handle so applyCommandBuffer can mirror the
        // destroy into entity_world once ecs has applied it.
        ecsFacade_.commandDestroyEntity = [](void                             *self,
                                              engine::ecs::WorldHandle         handle,
                                              engine::ecs::CommandBufferHandle buffer,
                                              engine::ecs::Entity              entity,
                                              engine::ecs::StorageError       *error)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidInput;
                return false;
            }
            const auto external = static_cast<engine::entity_world::EntityHandle>(entity);
            const auto internal = module.mapEntity(*binding, external);
            if (internal == engine::ecs::invalidEntity)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidEntity;
                return false;
            }
            const auto pending = binding->commandBufferDestroys.find(buffer);
            if (pending == binding->commandBufferDestroys.end())
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidInput;
                return false;
            }
            try
            {
                pending->second.push_back(external);
            }
            catch (...)
            {
                if (error)
                    *error = engine::ecs::StorageError::ResourceFailure;
                return false;
            }
            const bool queued = module.ecsCapability_->commandDestroyEntity(
                module.ecsCapability_->self, binding->ecsWorld, buffer, internal, error);
            if (!queued)
                pending->second.pop_back();
            return queued;
        };
        // Applies the ecs buffer, then mirrors the queued destroys into
        // entity_world; skipped when the apply failed with InvalidDuringQuery
        // (nothing was changed).
        ecsFacade_.applyCommandBuffer = [](void                             *self,
                                            engine::ecs::WorldHandle         handle,
                                            engine::ecs::CommandBufferHandle buffer,
                                            engine::ecs::StorageError       *error)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidInput;
                return false;
            }
            if (binding->activeEcsQueries != 0)
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidDuringQuery;
                return false;
            }
            const auto pending = binding->commandBufferDestroys.find(buffer);
            if (pending == binding->commandBufferDestroys.end())
            {
                if (error)
                    *error = engine::ecs::StorageError::InvalidInput;
                return false;
            }
            engine::ecs::StorageError applyError = engine::ecs::StorageError::None;
            const bool                result     = module.ecsCapability_->applyCommandBuffer(
                module.ecsCapability_->self, binding->ecsWorld, buffer, &applyError);
            if (applyError != engine::ecs::StorageError::InvalidDuringQuery)
            {
                module.reconcileDestroyedEcsEntities(*binding, pending->second);
                pending->second.clear();
            }
            if (error)
                *error = applyError;
            return result;
        };

        entityWorldFacade_.abi         = engine::entity_world::entityWorldCapabilityAbi;
        entityWorldFacade_.self        = this;
        entityWorldFacade_.createWorld = [](void                                                *self,
                                             const engine::entity_world::EntityWorldDescription *description,
                                             engine::entity_world::EntityWorldError             *error)
        {
            auto      &module = *static_cast<WorldModule *>(self);
            const auto entityWorld
                = module.entityWorldCapability_->createWorld(module.entityWorldCapability_->self, description, error);
            if (entityWorld == engine::entity_world::invalidEntityWorld)
                return engine::entity_world::invalidEntityWorld;
            const auto ecsWorld = module.createEcsWorldForEntityWorld(description, entityWorld);
            if (ecsWorld == engine::ecs::invalidWorld)
            {
                if (error)
                    *error = engine::entity_world::EntityWorldError::ResourceFailure;
                return engine::entity_world::invalidEntityWorld;
            }
            WorldBinding binding{};
            binding.ecsWorld    = ecsWorld;
            binding.entityWorld = entityWorld;
            try
            {
                module.worlds_.emplace(entityWorld, std::move(binding));
            }
            catch (...)
            {
                module.ecsCapability_->destroyWorld(module.ecsCapability_->self, ecsWorld);
                module.entityWorldCapability_->destroyWorld(module.entityWorldCapability_->self, entityWorld);
                if (error)
                    *error = engine::entity_world::EntityWorldError::ResourceFailure;
                return engine::entity_world::invalidEntityWorld;
            }
            if (error)
                *error = engine::entity_world::EntityWorldError::None;
            return entityWorld;
        };
        entityWorldFacade_.destroyWorld = [](void *self, engine::entity_world::EntityWorldHandle handle)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding || binding->activeEcsQueries != 0)
                return;
            module.entityWorldCapability_->destroyWorld(module.entityWorldCapability_->self, binding->entityWorld);
            module.ecsCapability_->destroyWorld(module.ecsCapability_->self, binding->ecsWorld);
            module.worlds_.erase(handle);
        };
        entityWorldFacade_.worldInfo
            = [](void *self, engine::entity_world::EntityWorldHandle handle, engine::entity_world::EntityWorldInfo *out)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            return binding
                && module.entityWorldCapability_->worldInfo(
                    module.entityWorldCapability_->self, binding->entityWorld, out);
        };
        entityWorldFacade_.createEntity = [](void                                           *self,
                                              engine::entity_world::EntityWorldHandle        handle,
                                              const engine::entity_world::EntityDescription *description,
                                              engine::entity_world::EntityWorldError        *error)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding)
            {
                if (error)
                    *error = engine::entity_world::EntityWorldError::InvalidHandle;
                return engine::entity_world::invalidEntity;
            }
            const auto external = module.entityWorldCapability_->createEntity(
                module.entityWorldCapability_->self, binding->entityWorld, description, error);
            if (external == engine::entity_world::invalidEntity)
                return external;
            return module.createEcsEntityForEntityWorld(*binding, external) == engine::ecs::invalidEntity
                     ? engine::entity_world::invalidEntity
                     : external;
        };
        entityWorldFacade_.destroyEntity = [](void                                    *self,
                                               engine::entity_world::EntityWorldHandle handle,
                                               engine::entity_world::EntityHandle      entity,
                                               bool                                    recursive,
                                               engine::entity_world::EntityWorldError *error)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding)
            {
                if (error)
                    *error = engine::entity_world::EntityWorldError::InvalidHandle;
                return false;
            }
            if (binding->activeEcsQueries != 0)
            {
                if (error)
                    *error = engine::entity_world::EntityWorldError::InvalidDuringQuery;
                return false;
            }
            if (module.mapEntity(*binding, entity) == engine::ecs::invalidEntity)
            {
                if (error)
                    *error = engine::entity_world::EntityWorldError::InvalidHandle;
                return false;
            }
            std::vector<engine::entity_world::EntityHandle> candidates;
            if (!module.collectEntityWorldDestroyCandidates(*binding, entity, recursive, candidates))
            {
                if (error)
                    *error = engine::entity_world::EntityWorldError::ResourceFailure;
                return false;
            }
            const bool result = module.entityWorldCapability_->destroyEntity(
                module.entityWorldCapability_->self, binding->entityWorld, entity, recursive, error);
            if (result)
                module.reconcileDestroyedEntityWorldEntities(*binding, candidates);
            return result;
        };
        // True only when the entity exists in both layers (consistency
        // check between the mirrors).
        entityWorldFacade_.entityExists
            = [](void *self, engine::entity_world::EntityWorldHandle handle, engine::entity_world::EntityHandle entity)
        {
            auto      &module   = *static_cast<WorldModule *>(self);
            auto      *binding  = module.world(handle);
            const auto internal = binding ? module.mapEntity(*binding, entity) : engine::ecs::invalidEntity;
            return binding && internal != engine::ecs::invalidEntity
                && module.entityWorldCapability_->entityExists(
                    module.entityWorldCapability_->self, binding->entityWorld, entity)
                && module.ecsCapability_->entityExists(module.ecsCapability_->self, binding->ecsWorld, internal);
        };
        entityWorldFacade_.setEntityEnabled = [](void                                    *self,
                                                  engine::entity_world::EntityWorldHandle handle,
                                                  engine::entity_world::EntityHandle      entity,
                                                  bool                                    enabled)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            return binding && module.mapEntity(*binding, entity) != engine::ecs::invalidEntity
                && module.entityWorldCapability_->setEntityEnabled(
                    module.entityWorldCapability_->self, binding->entityWorld, entity, enabled);
        };
        entityWorldFacade_.isEntityEnabled = [](void                                    *self,
                                                 engine::entity_world::EntityWorldHandle handle,
                                                 engine::entity_world::EntityHandle      entity,
                                                 bool                                   *out)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            return binding && module.mapEntity(*binding, entity) != engine::ecs::invalidEntity
                && module.entityWorldCapability_->isEntityEnabled(
                    module.entityWorldCapability_->self, binding->entityWorld, entity, out);
        };
        entityWorldFacade_.setLocalTransform = [](void                                    *self,
                                                   engine::entity_world::EntityWorldHandle handle,
                                                   engine::entity_world::EntityHandle      entity,
                                                   const engine::entity_world::Transform  *transform,
                                                   engine::entity_world::EntityWorldError *error)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding || module.mapEntity(*binding, entity) == engine::ecs::invalidEntity)
            {
                if (error)
                    *error = engine::entity_world::EntityWorldError::InvalidHandle;
                return false;
            }
            return module.entityWorldCapability_->setLocalTransform(
                module.entityWorldCapability_->self, binding->entityWorld, entity, transform, error);
        };
        entityWorldFacade_.getLocalTransform = [](void                                    *self,
                                                   engine::entity_world::EntityWorldHandle handle,
                                                   engine::entity_world::EntityHandle      entity,
                                                   engine::entity_world::Transform        *out)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            return binding && module.mapEntity(*binding, entity) != engine::ecs::invalidEntity
                && module.entityWorldCapability_->getLocalTransform(
                    module.entityWorldCapability_->self, binding->entityWorld, entity, out);
        };
        entityWorldFacade_.getWorldTransform = [](void                                    *self,
                                                   engine::entity_world::EntityWorldHandle handle,
                                                   engine::entity_world::EntityHandle      entity,
                                                   engine::entity_world::Transform        *out)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            return binding && module.mapEntity(*binding, entity) != engine::ecs::invalidEntity
                && module.entityWorldCapability_->getWorldTransform(
                    module.entityWorldCapability_->self, binding->entityWorld, entity, out);
        };
        entityWorldFacade_.setParent = [](void                                    *self,
                                           engine::entity_world::EntityWorldHandle handle,
                                           engine::entity_world::EntityHandle      entity,
                                           engine::entity_world::EntityHandle      parent,
                                           bool                                    preserve,
                                           engine::entity_world::EntityWorldError *error)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding || module.mapEntity(*binding, entity) == engine::ecs::invalidEntity
                || (parent != engine::entity_world::invalidEntity
                    && module.mapEntity(*binding, parent) == engine::ecs::invalidEntity))
            {
                if (error)
                    *error = engine::entity_world::EntityWorldError::InvalidHandle;
                return false;
            }
            return module.entityWorldCapability_->setParent(
                module.entityWorldCapability_->self, binding->entityWorld, entity, parent, preserve, error);
        };
        entityWorldFacade_.getParent
            = [](void *self, engine::entity_world::EntityWorldHandle handle, engine::entity_world::EntityHandle entity)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            return binding && module.mapEntity(*binding, entity) != engine::ecs::invalidEntity
                     ? module.entityWorldCapability_->getParent(
                           module.entityWorldCapability_->self, binding->entityWorld, entity)
                     : engine::entity_world::invalidEntity;
        };
        entityWorldFacade_.copyEntities = [](void                                    *self,
                                              engine::entity_world::EntityWorldHandle handle,
                                              engine::entity_world::EntityHandle     *out,
                                              std::uint32_t                           capacity,
                                              std::uint32_t                          *total)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            return binding ? module.entityWorldCapability_->copyEntities(
                                 module.entityWorldCapability_->self, binding->entityWorld, out, capacity, total)
                           : (total ? (*total = 0, 0u) : 0u);
        };
        entityWorldFacade_.copyEntityName = [](void                                    *self,
                                                engine::entity_world::EntityWorldHandle handle,
                                                engine::entity_world::EntityHandle      entity,
                                                char                                   *out,
                                                std::uint32_t                           capacity,
                                                std::uint32_t                          *required)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            return binding && module.mapEntity(*binding, entity) != engine::ecs::invalidEntity
                && module.entityWorldCapability_->copyEntityName(
                    module.entityWorldCapability_->self, binding->entityWorld, entity, out, capacity, required);
        };
        entityWorldFacade_.saveWorld = [](void                                    *self,
                                           engine::entity_world::EntityWorldHandle handle,
                                           const char                             *path,
                                           engine::entity_world::EntityWorldError *error)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding)
            {
                if (error)
                    *error = engine::entity_world::EntityWorldError::InvalidHandle;
                return false;
            }
            return module.entityWorldCapability_->saveWorld(
                module.entityWorldCapability_->self, binding->entityWorld, path, error);
        };
        // Loads the entity world, then rebuilds the ecs mirror from scratch:
        // old command buffers and entities are discarded and one ecs entity
        // is created for every loaded entity.
        entityWorldFacade_.loadWorld = [](void                                    *self,
                                           engine::entity_world::EntityWorldHandle handle,
                                           const char                             *path,
                                           engine::entity_world::EntityWorldError *error)
        {
            auto &module  = *static_cast<WorldModule *>(self);
            auto *binding = module.world(handle);
            if (!binding)
            {
                if (error)
                    *error = engine::entity_world::EntityWorldError::InvalidHandle;
                return false;
            }
            if (binding->activeEcsQueries != 0)
            {
                if (error)
                    *error = engine::entity_world::EntityWorldError::InvalidDuringQuery;
                return false;
            }
            if (!module.entityWorldCapability_->loadWorld(
                    module.entityWorldCapability_->self, binding->entityWorld, path, error))
                return false;
            for (const auto &[commandBuffer, pending] : binding->commandBufferDestroys)
            {
                (void)pending;
                module.ecsCapability_->destroyCommandBuffer(
                    module.ecsCapability_->self, binding->ecsWorld, commandBuffer);
            }
            binding->commandBufferDestroys.clear();
            for (std::uint32_t slot = 0; slot < binding->entityToEcs.size(); ++slot)
            {
                const EntityMapEntry entry = binding->entityToEcs[slot];
                if (entry.generation != 0)
                    module.ecsCapability_->destroyEntity(
                        module.ecsCapability_->self, binding->ecsWorld, entry.mappedHandle, nullptr);
            }
            binding->entityToEcs.clear();
            binding->ecsToEntity.clear();
            std::uint32_t total = 0;
            module.entityWorldCapability_->copyEntities(
                module.entityWorldCapability_->self, binding->entityWorld, nullptr, 0, &total);
            std::vector<engine::entity_world::EntityHandle> entities;
            try
            {
                entities.resize(total);
            }
            catch (...)
            {
                if (error)
                    *error = engine::entity_world::EntityWorldError::ResourceFailure;
                return false;
            }
            if (total != 0)
                module.entityWorldCapability_->copyEntities(
                    module.entityWorldCapability_->self, binding->entityWorld, entities.data(), total, nullptr);
            for (const auto external : entities)
                if (module.createEcsEntityForEntityWorld(*binding, external) == engine::ecs::invalidEntity)
                {
                    if (error)
                        *error = engine::entity_world::EntityWorldError::ResourceFailure;
                    return false;
                }
            if (error)
                *error = engine::entity_world::EntityWorldError::None;
            return true;
        };
    }

    // The embedded inner modules with their own capability tables (the
    // facades above delegate into these).
    std::unique_ptr<engine::ModuleInterface>                                  ecsModule_;
    std::unique_ptr<engine::ModuleInterface>                                  entityWorldModule_;
    engine::ecs::EcsCapability                                               *ecsCapability_         = nullptr;
    engine::entity_world::EntityWorldCapability                              *entityWorldCapability_ = nullptr;
    engine::ecs::EcsCapability                                                ecsFacade_{};
    engine::entity_world::EntityWorldCapability                               entityWorldFacade_{};
    engine::ParticipantManifest                                               manifest_;
    std::unordered_map<engine::entity_world::EntityWorldHandle, WorldBinding> worlds_;
};
} // namespace
} // namespace engine::world

// Module entry point queried by the kernel after dlopen: one .so providing
// both ecs and entity_world; no required/optional capabilities, no
// permissions.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"ecs", "entity_world"};
    static const engine::EngineModuleEntryTable entry{
        .abiVersion              = engine::engineModuleAbiVersion,
        .moduleName              = "world",
        .moduleVersion           = "0.1.0",
        .createModule            = []() -> engine::ModuleInterface            *{ return new engine::world::WorldModule(); },
        .providedCapabilities    = provided,
        .providedCapabilityCount = 2,
        .requiredCapabilities    = nullptr,
        .requiredCapabilityCount = 0,
        .optionalCapabilities    = nullptr,
        .optionalCapabilityCount = 0,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0};
    return &entry;
}
