// C++ layer underneath the ecs capability: the archetype-based Storage and
// the deferred-change CommandBuffer classes that ecs.cpp binds into a
// capability table (and that the world module's facade composes further).

#ifndef ENGINE_ECS_STORAGE_HPP
#define ENGINE_ECS_STORAGE_HPP

#include <engine/world/types.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <new>
#include <vector>

namespace engine::ecs
{

// Component ids start at 1 (invalidComponent = 0); query ids start at 1.
using Entity      = engine::world::EntityHandle;
using ComponentId = std::uint32_t;
using QueryId     = std::uint64_t;

inline constexpr Entity      invalidEntity    = engine::world::invalidEntity;
inline constexpr ComponentId invalidComponent = 0;
inline constexpr QueryId     invalidQuery     = 0;

// Capacity tuning for a Storage. Zero fields are clamped to usable minimums
// by the implementation (chunk capacity 128, at least one component/query,
// maximumEntities raised to initialEntityCapacity).
struct StorageOptions
{
    std::uint32_t initialEntityCapacity = 1024;
    std::uint32_t maximumEntities       = 1'000'000;
    std::uint32_t chunkCapacity         = 128;
    std::uint32_t maximumComponents     = 256;
    std::uint32_t maximumQueries        = 1024;
};

// Error codes for storage calls. InvalidDuringQuery guards structural
// changes (register/create/destroy/add/remove/apply) made from inside a
// running query.
enum class StorageError : std::uint32_t
{
    None = 0,
    InvalidInput,
    InvalidEntity,
    InvalidComponent,
    InvalidQuery,
    CapacityExceeded,
    DuplicateComponentType,
    ComponentAlreadyPresent,
    ComponentNotPresent,
    ResourceFailure,
    InvalidDuringQuery
};

// Type-erased lifecycle hooks. Any may be null: copy falls back to memcpy,
// default construction to zero-fill, destroy to nothing.
using ComponentDefaultConstruct = void (*)(void *destination) noexcept;
using ComponentCopyConstruct    = void (*)(void *destination, const void *source) noexcept;
using ComponentDestroy          = void (*)(void *value) noexcept;

// One component type: fixed size and alignment (alignment must be a power
// of two, at most 4096) plus optional lifecycle callbacks.
struct ComponentDescription
{
    const char               *name             = nullptr;
    std::uint32_t             size             = 0;
    std::uint32_t             alignment        = 0;
    ComponentDefaultConstruct defaultConstruct = nullptr;
    ComponentCopyConstruct    copyConstruct    = nullptr;
    ComponentDestroy          destroy          = nullptr;
};

// Query signature: every required component must be present and every
// excluded one absent. The id arrays are copied into the query.
struct QueryDescription
{
    const ComponentId *required      = nullptr;
    std::uint32_t      requiredCount = 0;
    const ComponentId *excluded      = nullptr;
    std::uint32_t      excludedCount = 0;
};

// View of one chunk (up to chunkCapacity rows) with one column per required
// component: componentColumns[i] is the array for required[i], row r sits at
// offset r * stride. Valid only during the callback.
struct ChunkView
{
    const Entity      *entities         = nullptr;
    std::uint32_t      entityCount      = 0;
    const ComponentId *componentIds     = nullptr;
    void *const       *componentColumns = nullptr;
    std::uint32_t      componentCount   = 0;
};

// Return false to stop iteration early; queryChunks still reports success.
using ChunkCallback = bool (*)(void *userData, const ChunkView *chunk) noexcept;

// Archetype ECS storage. Move-only and not thread-safe: all use of one
// world must come from a single thread, and structural calls are rejected
// while queryChunks iterates. Component pointers stay valid until the next
// structural change (add/remove/destroy) touching their chunk.
class Storage final
{
public:
    explicit Storage(StorageOptions options = {});
    ~Storage();
    Storage(Storage &&) noexcept;
    Storage &operator=(Storage &&) noexcept;
    Storage(const Storage &)            = delete;
    Storage &operator=(const Storage &) = delete;

    ComponentId   registerComponent(const ComponentDescription &description, StorageError *error = nullptr);
    Entity        createEntity(StorageError *error = nullptr);
    bool          destroyEntity(Entity entity, StorageError *error = nullptr);
    bool          entityExists(Entity entity) const noexcept;
    std::uint32_t entityCount() const noexcept;
    std::uint32_t componentCount() const noexcept;
    std::uint32_t queryCount() const noexcept;

    bool addComponent(
        Entity entity, ComponentId component, const void *initialValue = nullptr, StorageError *error = nullptr);
    bool        removeComponent(Entity entity, ComponentId component, StorageError *error = nullptr);
    bool        hasComponent(Entity entity, ComponentId component) const noexcept;
    void       *getComponent(Entity entity, ComponentId component) noexcept;
    const void *getComponent(Entity entity, ComponentId component) const noexcept;

    QueryId createQuery(const QueryDescription &description, StorageError *error = nullptr);
    bool    destroyQuery(QueryId query) noexcept;
    bool    queryChunks(QueryId query, ChunkCallback callback, void *userData, StorageError *error = nullptr) const;

    const StorageOptions &options() const noexcept;

private:
    friend class CommandBuffer;
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Deferred structural changes recorded against one Storage and applied in
// order with apply(). addComponent captures the value by copy, so it must be
// nothrow copy constructible; apply() stops at the first failing command and
// clears the queue either way.
class CommandBuffer final
{
public:
    explicit CommandBuffer(Storage &storage) noexcept
        : storage_(&storage)
    {}

    bool addDefaultComponent(Entity entity, ComponentId component, StorageError *error = nullptr);

    template <typename T>
    bool addComponent(Entity entity, ComponentId component, const T &value, StorageError *error = nullptr)
    {
        static_assert(std::is_nothrow_copy_constructible_v<T>,
            "command-buffer component values must be nothrow copy constructible");
        try
        {
            commands_.emplace_back([entity, component, value](Storage &storage, StorageError *outError)
                { return storage.addComponent(entity, component, &value, outError); });
            if (error != nullptr)
                *error = StorageError::None;
            return true;
        }
        catch (const std::bad_alloc &)
        {
            if (error != nullptr)
                *error = StorageError::ResourceFailure;
            return false;
        }
    }

    bool        removeComponent(Entity entity, ComponentId component, StorageError *error = nullptr);
    bool        destroyEntity(Entity entity, StorageError *error = nullptr);
    bool        apply(StorageError *error = nullptr);
    void        clear() noexcept;
    std::size_t size() const noexcept
    {
        return commands_.size();
    }
    bool empty() const noexcept
    {
        return commands_.empty();
    }

private:
    using Command = std::function<bool(Storage &, StorageError *)>;

    bool append(Command command, StorageError *error);

    Storage             *storage_ = nullptr;
    std::vector<Command> commands_;
};

} // namespace engine::ecs

#endif
