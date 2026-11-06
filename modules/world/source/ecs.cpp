// ecs capability module: binds the Storage/CommandBuffer classes into an
// EcsCapability function-pointer table and owns the per-world records
// (storage plus command buffers). When ENGINE_WORLD_EMBEDDED is defined this
// file is compiled into the world module facade (world.cpp) instead of its
// own .so and exports createEmbeddedModule() instead of the entry table.

#include <engine/ecs/capabilities.hpp>
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

namespace engine::ecs
{
namespace
{
void setError(StorageError *output, StorageError value)
{
    if (output != nullptr)
        *output = value;
}

// One created ecs world: its storage plus the command buffers handed out
// for it.
struct WorldRecord
{
    std::string                                                             name;
    Storage                                                                 storage;
    std::unordered_map<CommandBufferHandle, std::unique_ptr<CommandBuffer>> commandBuffers;
    CommandBufferHandle                                                     nextCommandBuffer = 1;

    WorldRecord(std::string worldName, StorageOptions options)
        : name(std::move(worldName)),
          storage(options)
    {}
};

class EcsModule final : public ModuleInterface
{
public:
    EcsModule()
        : manifest_(makeManifest())
    {
        bindCapability();
    }

    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    // Reads [ecs] maximum_worlds (accepted range 1..4096); out-of-range or
    // missing values keep the default of 64.
    void construct(ParticipantContext &context) override
    {
        maximumWorlds_ = 64;
        if (const auto value = context.configurationIntegerInSection("ecs", "maximum_worlds");
            value && *value >= 1 && *value <= 4096)
            maximumWorlds_ = static_cast<std::size_t>(*value);
    }

    void start() override
    {}

    void stop() override
    {
        // Dropping the worlds destroys every storage and command buffer.
        worlds_.clear();
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == EcsCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name = "ecs", .kind = ParticipantKind::EngineModule, .version = "0.1.0", .providedCapabilities = {"ecs"}};
    }

    // Each lambda validates the world/command-buffer handle and forwards to
    // the Storage/CommandBuffer method; handle errors report InvalidInput.
    void bindCapability()
    {
        capability_.self        = this;
        capability_.createWorld = [](void *self, const WorldDescription *description, StorageError *error)
        { return static_cast<EcsModule *>(self)->createWorldImpl(description, error); };
        capability_.destroyWorld
            = [](void *self, WorldHandle world) { static_cast<EcsModule *>(self)->worlds_.erase(world); };
        capability_.worldInfo = [](void *self, WorldHandle world, WorldInfo *out)
        {
            auto      *module = static_cast<EcsModule *>(self);
            const auto found  = module->worlds_.find(world);
            if (found == module->worlds_.end() || out == nullptr)
                return false;
            out->name               = found->second->name.c_str();
            out->entityCount        = found->second->storage.entityCount();
            out->componentTypeCount = found->second->storage.componentCount();
            out->queryCount         = found->second->storage.queryCount();
            return true;
        };
        capability_.registerComponent
            = [](void *self, WorldHandle world, const ComponentDescription *description, StorageError *error)
        {
            WorldRecord *record = static_cast<EcsModule *>(self)->findWorld(world);
            if (record == nullptr || description == nullptr)
            {
                setError(error, StorageError::InvalidInput);
                return invalidComponent;
            }
            return record->storage.registerComponent(*description, error);
        };
        capability_.createEntity = [](void *self, WorldHandle world, StorageError *error)
        {
            WorldRecord *record = static_cast<EcsModule *>(self)->findWorld(world);
            if (record == nullptr)
            {
                setError(error, StorageError::InvalidInput);
                return invalidEntity;
            }
            return record->storage.createEntity(error);
        };
        capability_.destroyEntity = [](void *self, WorldHandle world, Entity entity, StorageError *error)
        {
            WorldRecord *record = static_cast<EcsModule *>(self)->findWorld(world);
            if (record == nullptr)
            {
                setError(error, StorageError::InvalidInput);
                return false;
            }
            return record->storage.destroyEntity(entity, error);
        };
        capability_.entityExists = [](void *self, WorldHandle world, Entity entity)
        {
            WorldRecord *record = static_cast<EcsModule *>(self)->findWorld(world);
            return record != nullptr && record->storage.entityExists(entity);
        };
        capability_.addComponent = [](void          *self,
                                       WorldHandle   world,
                                       Entity        entity,
                                       ComponentId   component,
                                       const void   *value,
                                       StorageError *error)
        {
            WorldRecord *record = static_cast<EcsModule *>(self)->findWorld(world);
            if (record == nullptr)
            {
                setError(error, StorageError::InvalidInput);
                return false;
            }
            return record->storage.addComponent(entity, component, value, error);
        };
        capability_.removeComponent
            = [](void *self, WorldHandle world, Entity entity, ComponentId component, StorageError *error)
        {
            WorldRecord *record = static_cast<EcsModule *>(self)->findWorld(world);
            if (record == nullptr)
            {
                setError(error, StorageError::InvalidInput);
                return false;
            }
            return record->storage.removeComponent(entity, component, error);
        };
        capability_.hasComponent = [](void *self, WorldHandle world, Entity entity, ComponentId component)
        {
            WorldRecord *record = static_cast<EcsModule *>(self)->findWorld(world);
            return record != nullptr && record->storage.hasComponent(entity, component);
        };
        capability_.getComponent = [](void *self, WorldHandle world, Entity entity, ComponentId component) -> void *
        {
            WorldRecord *record = static_cast<EcsModule *>(self)->findWorld(world);
            return record == nullptr ? nullptr : record->storage.getComponent(entity, component);
        };
        capability_.createQuery
            = [](void *self, WorldHandle world, const QueryDescription *description, StorageError *error)
        {
            WorldRecord *record = static_cast<EcsModule *>(self)->findWorld(world);
            if (record == nullptr || description == nullptr)
            {
                setError(error, StorageError::InvalidInput);
                return invalidQuery;
            }
            return record->storage.createQuery(*description, error);
        };
        capability_.destroyQuery = [](void *self, WorldHandle world, QueryId query)
        {
            WorldRecord *record = static_cast<EcsModule *>(self)->findWorld(world);
            return record != nullptr && record->storage.destroyQuery(query);
        };
        capability_.queryChunks = [](void          *self,
                                      WorldHandle   world,
                                      QueryId       query,
                                      ChunkCallback callback,
                                      void         *userData,
                                      StorageError *error)
        {
            WorldRecord *record = static_cast<EcsModule *>(self)->findWorld(world);
            if (record == nullptr)
            {
                setError(error, StorageError::InvalidInput);
                return false;
            }
            return record->storage.queryChunks(query, callback, userData, error);
        };
        capability_.createCommandBuffer = [](void *self, WorldHandle world, StorageError *error)
        { return static_cast<EcsModule *>(self)->createCommandBufferImpl(world, error); };
        capability_.destroyCommandBuffer = [](void *self, WorldHandle world, CommandBufferHandle commandBuffer)
        {
            WorldRecord *record = static_cast<EcsModule *>(self)->findWorld(world);
            return record != nullptr && record->commandBuffers.erase(commandBuffer) != 0;
        };
        capability_.commandAddDefaultComponent = [](void                *self,
                                                     WorldHandle         world,
                                                     CommandBufferHandle commandBuffer,
                                                     Entity              entity,
                                                     ComponentId         component,
                                                     StorageError       *error)
        {
            CommandBuffer *commands = static_cast<EcsModule *>(self)->findCommandBuffer(world, commandBuffer);
            if (commands == nullptr)
            {
                setError(error, StorageError::InvalidInput);
                return false;
            }
            return commands->addDefaultComponent(entity, component, error);
        };
        capability_.commandRemoveComponent = [](void                *self,
                                                 WorldHandle         world,
                                                 CommandBufferHandle commandBuffer,
                                                 Entity              entity,
                                                 ComponentId         component,
                                                 StorageError       *error)
        {
            CommandBuffer *commands = static_cast<EcsModule *>(self)->findCommandBuffer(world, commandBuffer);
            if (commands == nullptr)
            {
                setError(error, StorageError::InvalidInput);
                return false;
            }
            return commands->removeComponent(entity, component, error);
        };
        capability_.commandDestroyEntity
            = [](void *self, WorldHandle world, CommandBufferHandle commandBuffer, Entity entity, StorageError *error)
        {
            CommandBuffer *commands = static_cast<EcsModule *>(self)->findCommandBuffer(world, commandBuffer);
            if (commands == nullptr)
            {
                setError(error, StorageError::InvalidInput);
                return false;
            }
            return commands->destroyEntity(entity, error);
        };
        capability_.applyCommandBuffer
            = [](void *self, WorldHandle world, CommandBufferHandle commandBuffer, StorageError *error)
        {
            CommandBuffer *commands = static_cast<EcsModule *>(self)->findCommandBuffer(world, commandBuffer);
            if (commands == nullptr)
            {
                setError(error, StorageError::InvalidInput);
                return false;
            }
            return commands->apply(error);
        };
    }

    WorldRecord *findWorld(WorldHandle handle)
    {
        const auto found = worlds_.find(handle);
        return found == worlds_.end() ? nullptr : found->second.get();
    }

    CommandBuffer *findCommandBuffer(WorldHandle world, CommandBufferHandle commandBuffer)
    {
        WorldRecord *record = findWorld(world);
        if (record == nullptr)
            return nullptr;
        const auto found = record->commandBuffers.find(commandBuffer);
        return found == record->commandBuffers.end() ? nullptr : found->second.get();
    }

    // Handles start at 1; skip 0 (invalidCommandBuffer) if the counter wraps.
    CommandBufferHandle createCommandBufferImpl(WorldHandle world, StorageError *error)
    {
        setError(error, StorageError::None);
        WorldRecord *record = findWorld(world);
        if (record == nullptr)
        {
            setError(error, StorageError::InvalidInput);
            return invalidCommandBuffer;
        }
        try
        {
            CommandBufferHandle handle = record->nextCommandBuffer++;
            if (handle == invalidCommandBuffer)
                handle = record->nextCommandBuffer++;
            auto commands = std::make_unique<CommandBuffer>(record->storage);
            record->commandBuffers.emplace(handle, std::move(commands));
            return handle;
        }
        catch (...)
        {
            setError(error, StorageError::ResourceFailure);
            return invalidCommandBuffer;
        }
    }

    // World handles start at 1 (skip 0 on wrap); creation is limited by the
    // configured maximum_worlds.
    WorldHandle createWorldImpl(const WorldDescription *description, StorageError *error)
    {
        setError(error, StorageError::None);
        if (description == nullptr || worlds_.size() >= maximumWorlds_)
        {
            setError(error, description == nullptr ? StorageError::InvalidInput : StorageError::CapacityExceeded);
            return invalidWorld;
        }
        try
        {
            std::string name   = description->name == nullptr ? "world" : description->name;
            WorldHandle handle = nextWorld_++;
            if (handle == invalidWorld)
                handle = nextWorld_++;
            worlds_.emplace(handle, std::make_unique<WorldRecord>(std::move(name), description->storage));
            return handle;
        }
        catch (...)
        {
            setError(error, StorageError::ResourceFailure);
            return invalidWorld;
        }
    }

    ParticipantManifest                                           manifest_;
    EcsCapability                                                 capability_{};
    std::unordered_map<WorldHandle, std::unique_ptr<WorldRecord>> worlds_;
    WorldHandle                                                   nextWorld_     = 1;
    std::size_t                                                   maximumWorlds_ = 64;
};

} // namespace

#if defined(ENGINE_WORLD_EMBEDDED)
namespace detail
{
// Compiled-in creation hook used by the world module facade (world.cpp).
engine::ModuleInterface *createEmbeddedModule()
{
    return new EcsModule();
}
} // namespace detail
#endif

} // namespace engine::ecs

// Module entry point for the standalone ecs .so; omitted in embedded builds.
#if !defined(ENGINE_WORLD_EMBEDDED)
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"ecs"};
    static const engine::EngineModuleEntryTable entry
        = {.abiVersion               = engine::engineModuleAbiVersion,
            .moduleName              = "ecs",
            .moduleVersion           = "0.1.0",
            .createModule            = []() -> engine::ModuleInterface            *{ return new engine::ecs::EcsModule(); },
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
