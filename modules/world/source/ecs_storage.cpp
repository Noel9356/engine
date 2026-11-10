// Archetype-based component storage implementing Storage and CommandBuffer.
// Entities are generation-tagged slots; component data lives in per-archetype
// fixed-capacity chunks, each chunk one aligned allocation with a tightly
// packed column per component.

#include <engine/ecs/storage.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <new>
#include <string>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine::ecs
{
namespace
{

// Sentinel row/column index used throughout the storage.
constexpr std::uint32_t invalidIndex = std::numeric_limits<std::uint32_t>::max();

void setError(StorageError *out, StorageError value) noexcept
{
    if (out != nullptr)
        *out = value;
}

// Entity handle packing: low 32 bits = slot + 1 (so 0 is never a valid
// handle), high 32 bits = generation. encodeSlot maps to a zero-based index.
std::uint32_t encodeSlot(Entity entity) noexcept
{
    const std::uint32_t encoded = static_cast<std::uint32_t>(entity & 0xffffffffu);
    return encoded == 0 ? invalidIndex : encoded - 1u;
}

std::uint32_t encodeGeneration(Entity entity) noexcept
{
    return static_cast<std::uint32_t>(entity >> 32u);
}

Entity makeEntity(std::uint32_t slot, std::uint32_t generation) noexcept
{
    return (static_cast<std::uint64_t>(generation) << 32u) | (static_cast<std::uint64_t>(slot) + 1u);
}

bool isPowerOfTwo(std::uint32_t value) noexcept
{
    return value != 0 && (value & (value - 1u)) == 0;
}

std::size_t boundedStringLength(const char *value, std::size_t maximum) noexcept
{
    if (value == nullptr)
        return 0;
    std::size_t length = 0;
    while (length < maximum && value[length] != '\0')
        ++length;
    return length;
}

// Overflow-checked alignUp.
bool tryAlignUp(std::size_t value, std::size_t alignment, std::size_t &out) noexcept
{
    const std::size_t padding = (alignment - (value & (alignment - 1u))) & (alignment - 1u);
    if (value > std::numeric_limits<std::size_t>::max() - padding)
        return false;
    out = value + padding;
    return true;
}

} // namespace

// All storage state; Storage is just a unique_ptr to this (pimpl keeps the
// header light and the move operations noexcept).
struct Storage::Impl
{
    // Registered component type; stride is the alignment-padded size used
    // for column addressing.
    struct ComponentType
    {
        std::string               name;
        std::uint32_t             size             = 0;
        std::uint32_t             alignment        = 0;
        std::size_t               stride           = 0;
        ComponentDefaultConstruct defaultConstruct = nullptr;
        ComponentCopyConstruct    copyConstruct    = nullptr;
        ComponentDestroy          destroy          = nullptr;
    };

    // One component column inside a chunk: pointer into the chunk's storage
    // block, row r addressed as data + stride * r.
    struct Column
    {
        const ComponentType *type     = nullptr;
        void                *data     = nullptr;
        std::uint32_t        capacity = 0;

        Column() = default;
        Column(const ComponentType *componentType, void *columnData, std::uint32_t columnCapacity) noexcept
            : type(componentType),
              data(columnData),
              capacity(columnCapacity)
        {}
        Column(Column &&) noexcept            = default;
        Column &operator=(Column &&) noexcept = default;
        Column(const Column &)                = delete;
        Column &operator=(const Column &)     = delete;

        void *at(std::uint32_t row) noexcept
        {
            return static_cast<std::byte *>(data) + type->stride * row;
        }
        const void *at(std::uint32_t row) const noexcept
        {
            return static_cast<const std::byte *>(data) + type->stride * row;
        }
    };

    // Fixed-capacity block of rows for one archetype. All columns share one
    // aligned allocation; each column starts at its own aligned offset.
    struct Chunk
    {
        std::vector<Entity> entities;
        std::vector<Column> columns;
        std::uint32_t       count            = 0;
        void               *storage          = nullptr;
        std::size_t         storageAlignment = alignof(std::max_align_t);

        Chunk(
            const std::vector<ComponentId> &signature, const std::vector<ComponentType> &types, std::uint32_t capacity)
        {
            entities.reserve(capacity);
            columns.reserve(signature.size());

            std::size_t totalBytes = 0;
            for (const ComponentId id : signature)
            {
                const ComponentType &type = types[id - 1u];
                storageAlignment          = std::max(storageAlignment, static_cast<std::size_t>(type.alignment));
                if (!tryAlignUp(totalBytes, type.alignment, totalBytes)
                    || type.stride > (std::numeric_limits<std::size_t>::max() - totalBytes) / capacity)
                    throw std::bad_alloc();
                totalBytes += type.stride * capacity;
            }

            if (totalBytes != 0)
                storage = ::operator new(totalBytes, std::align_val_t(storageAlignment));

            std::size_t offset = 0;
            for (const ComponentId id : signature)
            {
                const ComponentType &type = types[id - 1u];
                if (!tryAlignUp(offset, type.alignment, offset))
                    throw std::bad_alloc();
                columns.emplace_back(&type, static_cast<std::byte *>(storage) + offset, capacity);
                offset += type.stride * capacity;
            }
        }

        Chunk(Chunk &&other) noexcept
            : entities(std::move(other.entities)),
              columns(std::move(other.columns)),
              count(std::exchange(other.count, 0)),
              storage(std::exchange(other.storage, nullptr)),
              storageAlignment(std::exchange(other.storageAlignment, alignof(std::max_align_t)))
        {}

        Chunk &operator=(Chunk &&other) noexcept
        {
            if (this == &other)
                return *this;
            releaseStorage();
            entities         = std::move(other.entities);
            columns          = std::move(other.columns);
            count            = std::exchange(other.count, 0);
            storage          = std::exchange(other.storage, nullptr);
            storageAlignment = std::exchange(other.storageAlignment, alignof(std::max_align_t));
            return *this;
        }

        Chunk(const Chunk &)            = delete;
        Chunk &operator=(const Chunk &) = delete;

        ~Chunk()
        {
            releaseStorage();
        }

        void releaseStorage() noexcept
        {
            if (storage != nullptr)
                ::operator delete(storage, std::align_val_t(storageAlignment));
            storage          = nullptr;
            storageAlignment = alignof(std::max_align_t);
        }
    };

    // A unique sorted component signature. Chunks hold the rows;
    // add/removeTransitions cache archetype transitions for component
    // add/remove.
    struct Archetype
    {
        std::vector<ComponentId>                       signature;
        std::vector<Chunk>                             chunks;
        std::unordered_map<ComponentId, std::uint32_t> addTransitions;
        std::unordered_map<ComponentId, std::uint32_t> removeTransitions;
    };

    // Per-slot bookkeeping: generation validates handles after slot reuse,
    // archetype/chunk/row locate the entity's data, nextFree threads dead
    // slots into the free list.
    struct EntityLocation
    {
        std::uint32_t generation = 1;
        std::uint32_t archetype  = invalidIndex;
        std::uint32_t chunk      = invalidIndex;
        std::uint32_t row        = invalidIndex;
        bool          alive      = false;
        std::uint32_t nextFree   = invalidIndex;
    };

    // A query caches, per matching archetype, the column index of each
    // required component; matches are kept up to date as archetypes appear.
    struct QueryMatch
    {
        std::uint32_t              archetype = invalidIndex;
        std::vector<std::uint32_t> columns;
    };

    struct QueryRecord
    {
        std::vector<ComponentId> required;
        std::vector<ComponentId> excluded;
        std::vector<QueryMatch>  matches;
    };

    // Clamps the options to usable minimums and creates the empty archetype
    // (index 0) that every entity starts in.
    explicit Impl(StorageOptions value)
        : settings(value)
    {
        if (settings.initialEntityCapacity == 0)
            settings.initialEntityCapacity = 1;
        if (settings.maximumEntities < settings.initialEntityCapacity)
            settings.maximumEntities = settings.initialEntityCapacity;
        if (settings.chunkCapacity == 0)
            settings.chunkCapacity = 128;
        if (settings.maximumComponents == 0)
            settings.maximumComponents = 1;
        if (settings.maximumQueries == 0)
            settings.maximumQueries = 1;
        entities.reserve(settings.initialEntityCapacity);
        componentTypes.reserve(settings.maximumComponents);
        getOrCreateArchetype({});
    }

    // Validates handle layout, slot range, generation, and alive flag.
    bool validEntity(Entity entity, std::uint32_t *outSlot = nullptr) const noexcept
    {
        const std::uint32_t slot = encodeSlot(entity);
        if (slot == invalidIndex || encodeGeneration(entity) == 0 || slot >= entities.size())
            return false;
        const EntityLocation &location = entities[slot];
        if (!location.alive || location.generation != encodeGeneration(entity))
            return false;
        if (outSlot != nullptr)
            *outSlot = slot;
        return true;
    }

    // Binary search in the archetype's sorted signature; invalidIndex when
    // the component is absent.
    std::uint32_t findColumn(const Archetype &archetype, ComponentId id) const noexcept
    {
        const auto found = std::lower_bound(archetype.signature.begin(), archetype.signature.end(), id);
        return found == archetype.signature.end() || *found != id
                 ? invalidIndex
                 : static_cast<std::uint32_t>(found - archetype.signature.begin());
    }

    // A query matches an archetype when every required component is present
    // and no excluded one is; outColumns receives the required columns.
    bool matches(const Archetype &archetype, const QueryRecord &query, std::vector<std::uint32_t> *outColumns) const
    {
        std::vector<std::uint32_t> columns;
        columns.reserve(query.required.size());
        for (const ComponentId id : query.required)
        {
            const std::uint32_t column = findColumn(archetype, id);
            if (column == invalidIndex)
                return false;
            columns.push_back(column);
        }
        for (const ComponentId id : query.excluded)
            if (findColumn(archetype, id) != invalidIndex)
                return false;
        if (outColumns != nullptr)
            *outColumns = std::move(columns);
        return true;
    }

    // Sorts and dedups the signature, reuses or creates the archetype, and
    // matches every existing query against the new archetype so later
    // queryChunks calls see it.
    std::uint32_t getOrCreateArchetype(std::vector<ComponentId> signature)
    {
        std::sort(signature.begin(), signature.end());
        signature.erase(std::unique(signature.begin(), signature.end()), signature.end());
        const auto existing = archetypeMap.find(signature);
        if (existing != archetypeMap.end())
            return existing->second;

        if (archetypes.size() >= invalidIndex)
            throw std::length_error("archetype handle space exhausted");
        auto archetype            = std::make_unique<Archetype>();
        archetype->signature      = signature;
        const std::uint32_t index = static_cast<std::uint32_t>(archetypes.size());

        struct PendingMatch
        {
            QueryRecord *query = nullptr;
            QueryMatch   match;
        };
        std::vector<PendingMatch> pendingMatches;
        pendingMatches.reserve(queries.size());
        for (auto &[queryId, query] : queries)
        {
            (void)queryId;
            QueryMatch match;
            if (matches(*archetype, query, &match.columns))
            {
                match.archetype = index;
                query.matches.reserve(query.matches.size() + 1u);
                pendingMatches.push_back({&query, std::move(match)});
            }
        }

        archetypes.push_back(std::move(archetype));
        try
        {
            archetypeMap.emplace(archetypes.back()->signature, index);
        }
        catch (...)
        {
            archetypes.pop_back();
            throw;
        }
        for (PendingMatch &pending : pendingMatches)
            pending.query->matches.push_back(std::move(pending.match));
        return index;
    }

    // Cached add/remove transition to the neighbor archetype, creating the
    // target on first use (the reverse transition is cached too). A failed
    // cache insert only costs recomputation later.
    std::uint32_t transitionArchetype(std::uint32_t sourceIndex, ComponentId component, bool adding)
    {
        Archetype &source = *archetypes[sourceIndex];
        auto      &cache  = adding ? source.addTransitions : source.removeTransitions;
        const auto cached = cache.find(component);
        if (cached != cache.end())
            return cached->second;

        std::vector<ComponentId> signature = source.signature;
        const auto               position  = std::lower_bound(signature.begin(), signature.end(), component);
        if (adding)
            signature.insert(position, component);
        else
            signature.erase(position);

        const std::uint32_t targetIndex = getOrCreateArchetype(std::move(signature));
        try
        {
            cache.reserve(cache.size() + 1u);
            cache.emplace(component, targetIndex);
        }
        catch (const std::bad_alloc &)
        {}

        Archetype &target = *archetypes[targetIndex];
        try
        {
            auto &reverseCache = adding ? target.removeTransitions : target.addTransitions;
            reverseCache.emplace(component, sourceIndex);
        }
        catch (const std::bad_alloc &)
        {}
        return targetIndex;
    }

    // Lifecycle dispatch: copy from source (memcpy fallback) or default
    // construct (memset fallback); destroy only when a hook exists.
    void constructValue(const ComponentType &type, void *destination, const void *source) const noexcept
    {
        if (source != nullptr)
        {
            if (type.copyConstruct != nullptr)
                type.copyConstruct(destination, source);
            else
                std::memcpy(destination, source, type.size);
        }
        else if (type.defaultConstruct != nullptr)
        {
            type.defaultConstruct(destination);
        }
        else
        {
            std::memset(destination, 0, type.size);
        }
    }

    void destroyValue(const ComponentType &type, void *value) const noexcept
    {
        if (type.destroy != nullptr)
            type.destroy(value);
    }

    // Destroys every live component value through its destroy hook before
    // the chunk allocations are freed.
    ~Impl()
    {
        for (const std::unique_ptr<Archetype> &archetype : archetypes)
        {
            for (const Chunk &chunk : archetype->chunks)
            {
                for (const Column &column : chunk.columns)
                {
                    for (std::uint32_t row = 0; row < chunk.count; ++row)
                        destroyValue(*column.type, const_cast<void *>(column.at(row)));
                }
            }
        }
    }

    // Appends one row: opens a new chunk when the last is full, constructs
    // every column (values moved from the source row, or the added
    // component's initial value), and reports the row.
    bool appendToArchetype(std::uint32_t archetypeIndex, Entity entity, const Archetype *sourceArchetype,
        const Chunk *sourceChunk, std::uint32_t sourceRow, ComponentId addedComponent, const void *addedValue,
        std::uint32_t &outChunk, std::uint32_t &outRow)
    {
        Archetype &target = *archetypes[archetypeIndex];
        if (target.chunks.empty() || target.chunks.back().count == settings.chunkCapacity)
            target.chunks.emplace_back(target.signature, componentTypes, settings.chunkCapacity);
        Chunk              &chunk = target.chunks.back();
        const std::uint32_t row   = chunk.count;
        for (std::size_t targetColumn = 0; targetColumn < target.signature.size(); ++targetColumn)
        {
            const ComponentId    id          = target.signature[targetColumn];
            const ComponentType &type        = componentTypes[id - 1u];
            const void          *sourceValue = nullptr;
            if (id == addedComponent)
                sourceValue = addedValue;
            else if (sourceArchetype != nullptr && sourceChunk != nullptr)
            {
                const std::uint32_t sourceColumn = findColumn(*sourceArchetype, id);
                if (sourceColumn != invalidIndex)
                    sourceValue = sourceChunk->columns[sourceColumn].at(sourceRow);
            }
            constructValue(type, chunk.columns[targetColumn].at(row), sourceValue);
        }
        chunk.entities.push_back(entity);
        ++chunk.count;
        outChunk = static_cast<std::uint32_t>(target.chunks.size() - 1u);
        outRow   = row;
        return true;
    }

    // Swap-removes a row: destroys the values, moves the last row into the
    // hole (copy + destroy), and fixes the moved entity's stored location.
    void eraseAt(std::uint32_t archetypeIndex, std::uint32_t chunkIndex, std::uint32_t row)
    {
        Archetype          &archetype = *archetypes[archetypeIndex];
        Chunk              &chunk     = archetype.chunks[chunkIndex];
        const std::uint32_t last      = chunk.count - 1u;
        for (std::size_t column = 0; column < chunk.columns.size(); ++column)
            destroyValue(*chunk.columns[column].type, chunk.columns[column].at(row));
        if (row != last)
        {
            const Entity movedEntity = chunk.entities[last];
            for (std::size_t column = 0; column < chunk.columns.size(); ++column)
            {
                Column &storage = chunk.columns[column];
                constructValue(*storage.type, storage.at(row), storage.at(last));
                destroyValue(*storage.type, storage.at(last));
            }
            chunk.entities[row]           = movedEntity;
            const std::uint32_t movedSlot = encodeSlot(movedEntity);
            EntityLocation     &location  = entities[movedSlot];
            location.chunk                = chunkIndex;
            location.row                  = row;
        }
        chunk.entities.pop_back();
        --chunk.count;
    }

    StorageOptions                               settings;
    std::vector<ComponentType>                   componentTypes;
    std::unordered_map<std::string, ComponentId> componentNames;
    std::vector<EntityLocation>                  entities;
    // Head of the intrusive free list threaded through dead EntityLocations.
    std::uint32_t                           freeSlotHead = invalidIndex;
    std::vector<std::unique_ptr<Archetype>> archetypes;
    // FNV-1a over the sorted signature so archetype lookup stays cheap.
    struct ArchetypeSigHash
    {
        std::size_t operator()(const std::vector<ComponentId> &sig) const noexcept
        {
            std::uint64_t h = 0xcbf29ce484222325ULL;
            for (auto id : sig)
            {
                h ^= id;
                h *= 0x100000001b3ULL;
            }
            return static_cast<std::size_t>(h);
        }
    };
    std::unordered_map<std::vector<ComponentId>, std::uint32_t, ArchetypeSigHash> archetypeMap;

    // Bumped on every structural change; not read anywhere yet (reserved for
    // cache invalidation).
    std::uint64_t                            structuralVersion = 0;
    std::unordered_map<QueryId, QueryRecord> queries;
    QueryId                                  nextQuery  = 1;
    std::uint32_t                            aliveCount = 0;
    // Nonzero while a queryChunks callback runs; structural calls check it.
    // mutable because queries are const.
    mutable std::uint32_t activeQueryDepth = 0;
};

Storage::Storage(StorageOptions options)
    : impl_(std::make_unique<Impl>(options))
{}
Storage::~Storage()                              = default;
Storage::Storage(Storage &&) noexcept            = default;
Storage &Storage::operator=(Storage &&) noexcept = default;

// Registers a component type: validates name/size/alignment, rejects
// duplicates, and returns ids starting at 1.
ComponentId Storage::registerComponent(const ComponentDescription &description, StorageError *error)
{
    setError(error, StorageError::None);
    const std::size_t nameLength = boundedStringLength(description.name, 4097u);
    if (!impl_ || nameLength == 0 || nameLength > 4096u || description.size == 0
        || !isPowerOfTwo(description.alignment) || description.alignment > 4096)
    {
        setError(error, StorageError::InvalidInput);
        return invalidComponent;
    }
    if (impl_->activeQueryDepth != 0)
    {
        setError(error, StorageError::InvalidDuringQuery);
        return invalidComponent;
    }
    if (impl_->componentTypes.size() >= impl_->settings.maximumComponents)
    {
        setError(error, StorageError::CapacityExceeded);
        return invalidComponent;
    }
    if (impl_->componentNames.contains(description.name))
    {
        setError(error, StorageError::DuplicateComponentType);
        return invalidComponent;
    }
    try
    {
        Impl::ComponentType type;
        type.name      = description.name;
        type.size      = description.size;
        type.alignment = description.alignment;
        if (!tryAlignUp(description.size, description.alignment, type.stride))
        {
            setError(error, StorageError::InvalidInput);
            return invalidComponent;
        }
        type.defaultConstruct = description.defaultConstruct;
        type.copyConstruct    = description.copyConstruct;
        type.destroy          = description.destroy;
        impl_->componentTypes.push_back(std::move(type));
        const ComponentId id = static_cast<ComponentId>(impl_->componentTypes.size());
        try
        {
            impl_->componentNames.emplace(description.name, id);
        }
        catch (...)
        {
            impl_->componentTypes.pop_back();
            throw;
        }
        setError(error, StorageError::None);
        return id;
    }
    catch (const std::bad_alloc &)
    {
        setError(error, StorageError::ResourceFailure);
        return invalidComponent;
    }
    catch (const std::length_error &)
    {
        setError(error, StorageError::ResourceFailure);
        return invalidComponent;
    }
}

// Pops a free slot or grows the slot vector, places the entity in archetype
// 0 (no components), and bumps the alive count. A reserved slot is returned
// to the free list on allocation failure.
Entity Storage::createEntity(StorageError *error)
{
    setError(error, StorageError::None);
    if (!impl_)
    {
        setError(error, StorageError::ResourceFailure);
        return invalidEntity;
    }
    if (impl_->activeQueryDepth != 0)
    {
        setError(error, StorageError::InvalidDuringQuery);
        return invalidEntity;
    }
    if (impl_->aliveCount >= impl_->settings.maximumEntities)
    {
        setError(error, StorageError::CapacityExceeded);
        return invalidEntity;
    }
    std::uint32_t slot = invalidIndex;
    try
    {
        if (impl_->freeSlotHead != invalidIndex)
        {
            slot                           = impl_->freeSlotHead;
            impl_->freeSlotHead            = impl_->entities[slot].nextFree;
            impl_->entities[slot].nextFree = invalidIndex;
        }
        else
        {
            if (impl_->entities.size() >= impl_->settings.maximumEntities)
            {
                setError(error, StorageError::CapacityExceeded);
                return invalidEntity;
            }
            slot = static_cast<std::uint32_t>(impl_->entities.size());
            impl_->entities.emplace_back();
        }
        Impl::EntityLocation &location = impl_->entities[slot];
        const Entity          entity   = makeEntity(slot, location.generation);
        std::uint32_t         chunk    = 0;
        std::uint32_t         row      = 0;
        impl_->appendToArchetype(0, entity, nullptr, nullptr, 0, invalidComponent, nullptr, chunk, row);
        location.alive     = true;
        location.archetype = 0;
        location.chunk     = chunk;
        location.row       = row;
        ++impl_->aliveCount;
        return entity;
    }
    catch (const std::bad_alloc &)
    {
        if (slot != invalidIndex)
        {
            impl_->entities[slot].nextFree = impl_->freeSlotHead;
            impl_->freeSlotHead            = slot;
        }
        setError(error, StorageError::ResourceFailure);
        return invalidEntity;
    }
    catch (const std::length_error &)
    {
        if (slot != invalidIndex)
        {
            impl_->entities[slot].nextFree = impl_->freeSlotHead;
            impl_->freeSlotHead            = slot;
        }
        setError(error, StorageError::ResourceFailure);
        return invalidEntity;
    }
}

// Frees the entity: swap-removes its row, advances the generation, and
// links the slot into the free list. Exhausted generations retire the slot.
bool Storage::destroyEntity(Entity entity, StorageError *error)
{
    setError(error, StorageError::None);
    if (impl_ && impl_->activeQueryDepth != 0)
    {
        setError(error, StorageError::InvalidDuringQuery);
        return false;
    }
    std::uint32_t slot = 0;
    if (!impl_ || !impl_->validEntity(entity, &slot))
    {
        setError(error, StorageError::InvalidEntity);
        return false;
    }
    Impl::EntityLocation &location = impl_->entities[slot];
    impl_->eraseAt(location.archetype, location.chunk, location.row);
    ++impl_->structuralVersion;
    location.alive     = false;
    location.archetype = location.chunk = location.row = invalidIndex;
    if (location.generation == std::numeric_limits<std::uint32_t>::max())
    {
        location.generation = 0;
        location.nextFree   = invalidIndex;
    }
    else
    {
        ++location.generation;
        location.nextFree   = impl_->freeSlotHead;
        impl_->freeSlotHead = slot;
    }
    --impl_->aliveCount;
    return true;
}

bool Storage::entityExists(Entity entity) const noexcept
{
    return impl_ && impl_->validEntity(entity);
}

std::uint32_t Storage::entityCount() const noexcept
{
    return impl_ ? impl_->aliveCount : 0;
}

std::uint32_t Storage::componentCount() const noexcept
{
    return impl_ ? static_cast<std::uint32_t>(impl_->componentTypes.size()) : 0;
}

std::uint32_t Storage::queryCount() const noexcept
{
    return impl_ ? static_cast<std::uint32_t>(impl_->queries.size()) : 0;
}

// Moves the entity to the archetype with one more component, carrying the
// existing values over and constructing the new one (initialValue or
// default). ComponentAlreadyPresent when it already has the component.
bool Storage::addComponent(Entity entity, ComponentId component, const void *initialValue, StorageError *error)
{
    setError(error, StorageError::None);
    if (impl_ && impl_->activeQueryDepth != 0)
    {
        setError(error, StorageError::InvalidDuringQuery);
        return false;
    }
    std::uint32_t slot = 0;
    if (!impl_ || !impl_->validEntity(entity, &slot))
    {
        setError(error, StorageError::InvalidEntity);
        return false;
    }
    if (component == invalidComponent || component > impl_->componentTypes.size())
    {
        setError(error, StorageError::InvalidComponent);
        return false;
    }
    Impl::EntityLocation  &location        = impl_->entities[slot];
    const Impl::Archetype &sourceArchetype = *impl_->archetypes[location.archetype];
    if (impl_->findColumn(sourceArchetype, component) != invalidIndex)
    {
        setError(error, StorageError::ComponentAlreadyPresent);
        return false;
    }
    try
    {
        const std::uint32_t targetIndex = impl_->transitionArchetype(location.archetype, component, true);
        const Impl::Chunk  &sourceChunk = sourceArchetype.chunks[location.chunk];
        std::uint32_t       targetChunk = 0;
        std::uint32_t       targetRow   = 0;
        impl_->appendToArchetype(targetIndex,
            entity,
            &sourceArchetype,
            &sourceChunk,
            location.row,
            component,
            initialValue,
            targetChunk,
            targetRow);
        impl_->eraseAt(location.archetype, location.chunk, location.row);
        ++impl_->structuralVersion;
        location.archetype = targetIndex;
        location.chunk     = targetChunk;
        location.row       = targetRow;
        return true;
    }
    catch (const std::bad_alloc &)
    {
        setError(error, StorageError::ResourceFailure);
        return false;
    }
    catch (const std::length_error &)
    {
        setError(error, StorageError::ResourceFailure);
        return false;
    }
}

// Moves the entity to the archetype with one fewer component;
// ComponentNotPresent when the component is absent.
bool Storage::removeComponent(Entity entity, ComponentId component, StorageError *error)
{
    setError(error, StorageError::None);
    if (impl_ && impl_->activeQueryDepth != 0)
    {
        setError(error, StorageError::InvalidDuringQuery);
        return false;
    }
    std::uint32_t slot = 0;
    if (!impl_ || !impl_->validEntity(entity, &slot))
    {
        setError(error, StorageError::InvalidEntity);
        return false;
    }
    if (component == invalidComponent || component > impl_->componentTypes.size())
    {
        setError(error, StorageError::InvalidComponent);
        return false;
    }
    Impl::EntityLocation  &location        = impl_->entities[slot];
    const Impl::Archetype &sourceArchetype = *impl_->archetypes[location.archetype];
    if (impl_->findColumn(sourceArchetype, component) == invalidIndex)
    {
        setError(error, StorageError::ComponentNotPresent);
        return false;
    }
    try
    {
        const std::uint32_t targetIndex = impl_->transitionArchetype(location.archetype, component, false);
        const Impl::Chunk  &sourceChunk = sourceArchetype.chunks[location.chunk];
        std::uint32_t       targetChunk = 0;
        std::uint32_t       targetRow   = 0;
        impl_->appendToArchetype(targetIndex,
            entity,
            &sourceArchetype,
            &sourceChunk,
            location.row,
            invalidComponent,
            nullptr,
            targetChunk,
            targetRow);
        impl_->eraseAt(location.archetype, location.chunk, location.row);
        ++impl_->structuralVersion;
        location.archetype = targetIndex;
        location.chunk     = targetChunk;
        location.row       = targetRow;
        return true;
    }
    catch (const std::bad_alloc &)
    {
        setError(error, StorageError::ResourceFailure);
        return false;
    }
    catch (const std::length_error &)
    {
        setError(error, StorageError::ResourceFailure);
        return false;
    }
}

bool Storage::hasComponent(Entity entity, ComponentId component) const noexcept
{
    if (!impl_ || !impl_->validEntity(entity) || component == invalidComponent
        || component > impl_->componentTypes.size())
        return false;
    const std::uint32_t slot     = encodeSlot(entity);
    const auto         &location = impl_->entities[slot];
    return impl_->findColumn(*impl_->archetypes[location.archetype], component) != invalidIndex;
}

void *Storage::getComponent(Entity entity, ComponentId component) noexcept
{
    return const_cast<void *>(std::as_const(*this).getComponent(entity, component));
}

// Pointer into chunk storage; invalidated by any structural change.
const void *Storage::getComponent(Entity entity, ComponentId component) const noexcept
{
    if (!impl_ || !impl_->validEntity(entity) || component == invalidComponent
        || component > impl_->componentTypes.size())
        return nullptr;
    const std::uint32_t slot      = encodeSlot(entity);
    const auto         &location  = impl_->entities[slot];
    const auto         &archetype = *impl_->archetypes[location.archetype];
    const std::uint32_t column    = impl_->findColumn(archetype, component);
    if (column == invalidIndex)
        return nullptr;
    return archetype.chunks[location.chunk].columns[column].at(location.row);
}

// Validates and sorts the signature (duplicates and required-intersect-
// excluded are rejected), precomputes archetype matches, and returns ids
// starting at 1.
QueryId Storage::createQuery(const QueryDescription &description, StorageError *error)
{
    setError(error, StorageError::None);
    if (impl_ && impl_->activeQueryDepth != 0)
    {
        setError(error, StorageError::InvalidDuringQuery);
        return invalidQuery;
    }
    if (!impl_ || (description.requiredCount != 0 && description.required == nullptr)
        || (description.excludedCount != 0 && description.excluded == nullptr)
        || description.requiredCount > impl_->componentTypes.size()
        || description.excludedCount > impl_->componentTypes.size())
    {
        setError(error, StorageError::InvalidInput);
        return invalidQuery;
    }
    if (impl_->queries.size() >= impl_->settings.maximumQueries)
    {
        setError(error, StorageError::CapacityExceeded);
        return invalidQuery;
    }
    try
    {
        Impl::QueryRecord query;
        if (description.requiredCount != 0)
            query.required.assign(description.required, description.required + description.requiredCount);
        if (description.excludedCount != 0)
            query.excluded.assign(description.excluded, description.excluded + description.excludedCount);
        std::sort(query.required.begin(), query.required.end());
        std::sort(query.excluded.begin(), query.excluded.end());
        if (std::adjacent_find(query.required.begin(), query.required.end()) != query.required.end()
            || std::adjacent_find(query.excluded.begin(), query.excluded.end()) != query.excluded.end())
        {
            setError(error, StorageError::InvalidInput);
            return invalidQuery;
        }
        for (const ComponentId id : query.required)
            if (id == invalidComponent || id > impl_->componentTypes.size())
            {
                setError(error, StorageError::InvalidComponent);
                return invalidQuery;
            }
        for (const ComponentId id : query.excluded)
            if (id == invalidComponent || id > impl_->componentTypes.size()
                || std::binary_search(query.required.begin(), query.required.end(), id))
            {
                setError(error, StorageError::InvalidComponent);
                return invalidQuery;
            }
        for (std::uint32_t index = 0; index < impl_->archetypes.size(); ++index)
        {
            Impl::QueryMatch match;
            if (impl_->matches(*impl_->archetypes[index], query, &match.columns))
            {
                match.archetype = index;
                query.matches.push_back(std::move(match));
            }
        }
        QueryId id = invalidQuery;
        for (std::size_t attempt = 0; attempt <= impl_->queries.size(); ++attempt)
        {
            const QueryId candidate = impl_->nextQuery++;
            if (candidate != invalidQuery && !impl_->queries.contains(candidate))
            {
                id = candidate;
                break;
            }
        }
        if (id == invalidQuery)
        {
            setError(error, StorageError::CapacityExceeded);
            return invalidQuery;
        }
        const auto inserted = impl_->queries.emplace(id, std::move(query));
        if (!inserted.second)
        {
            setError(error, StorageError::CapacityExceeded);
            return invalidQuery;
        }
        return id;
    }
    catch (const std::bad_alloc &)
    {
        setError(error, StorageError::ResourceFailure);
        return invalidQuery;
    }
    catch (const std::length_error &)
    {
        setError(error, StorageError::ResourceFailure);
        return invalidQuery;
    }
}

bool Storage::destroyQuery(QueryId query) noexcept
{
    return impl_ && impl_->activeQueryDepth == 0 && impl_->queries.erase(query) != 0;
}

// Iterates cached matches chunk by chunk, building the column pointer array
// (inline for at most 16 components). The depth guard rejects structural
// calls made from inside the callback; a false callback return stops early.
bool Storage::queryChunks(QueryId query, ChunkCallback callback, void *userData, StorageError *error) const
{
    setError(error, StorageError::None);
    if (!impl_ || callback == nullptr)
    {
        setError(error, StorageError::InvalidInput);
        return false;
    }
    const auto found = impl_->queries.find(query);
    if (found == impl_->queries.end())
    {
        setError(error, StorageError::InvalidQuery);
        return false;
    }

    struct QueryScope
    {
        std::uint32_t &depth;
        explicit QueryScope(std::uint32_t &value) noexcept
            : depth(value)
        {
            ++depth;
        }
        ~QueryScope()
        {
            --depth;
        }
    } scope(impl_->activeQueryDepth);

    constexpr std::size_t                    inlineColumnCapacity = 16;
    std::array<void *, inlineColumnCapacity> inlineColumns;
    std::vector<void *>                      overflowColumns;
    const std::size_t                        columnCount = found->second.required.size();
    void                                   **columns     = columnCount == 0 ? nullptr : inlineColumns.data();
    if (columnCount > inlineColumns.size())
    {
        try
        {
            overflowColumns.resize(columnCount);
            columns = overflowColumns.data();
        }
        catch (const std::bad_alloc &)
        {
            setError(error, StorageError::ResourceFailure);
            return false;
        }
        catch (const std::length_error &)
        {
            setError(error, StorageError::ResourceFailure);
            return false;
        }
    }
    for (const Impl::QueryMatch &match : found->second.matches)
    {
        const Impl::Archetype &archetype = *impl_->archetypes[match.archetype];
        for (const Impl::Chunk &chunk : archetype.chunks)
        {
            if (chunk.count == 0)
                continue;
            for (std::size_t index = 0; index < match.columns.size(); ++index)
                columns[index] = const_cast<void *>(chunk.columns[match.columns[index]].data);
            const ChunkView view{
                .entities         = chunk.entities.data(),
                .entityCount      = chunk.count,
                .componentIds     = found->second.required.data(),
                .componentColumns = columns,
                .componentCount   = static_cast<std::uint32_t>(columnCount)};
            if (!callback(userData, &view))
                return true;
        }
    }
    return true;
}

const StorageOptions &Storage::options() const noexcept
{
    static constexpr StorageOptions defaults{};
    return impl_ ? impl_->settings : defaults;
}

} // namespace engine::ecs

namespace engine::ecs
{

// Records one command; bad_alloc reports ResourceFailure.
bool CommandBuffer::append(Command command, StorageError *error)
{
    if (storage_ == nullptr)
    {
        setError(error, StorageError::ResourceFailure);
        return false;
    }
    try
    {
        commands_.push_back(std::move(command));
        setError(error, StorageError::None);
        return true;
    }
    catch (const std::bad_alloc &)
    {
        setError(error, StorageError::ResourceFailure);
        return false;
    }
    catch (const std::length_error &)
    {
        setError(error, StorageError::ResourceFailure);
        return false;
    }
}

bool CommandBuffer::addDefaultComponent(Entity entity, ComponentId component, StorageError *error)
{
    return append([entity, component](Storage &storage, StorageError *outError)
        { return storage.addComponent(entity, component, nullptr, outError); },
        error);
}

bool CommandBuffer::removeComponent(Entity entity, ComponentId component, StorageError *error)
{
    return append([entity, component](Storage &storage, StorageError *outError)
        { return storage.removeComponent(entity, component, outError); },
        error);
}

bool CommandBuffer::destroyEntity(Entity entity, StorageError *error)
{
    return append(
        [entity](Storage &storage, StorageError *outError) { return storage.destroyEntity(entity, outError); }, error);
}

// Runs the recorded commands in order; stops at the first failure and clears
// the queue either way. Forbidden while a query is running.
bool CommandBuffer::apply(StorageError *error)
{
    if (storage_ == nullptr || !storage_->impl_)
    {
        setError(error, StorageError::ResourceFailure);
        return false;
    }
    if (storage_->impl_->activeQueryDepth != 0)
    {
        setError(error, StorageError::InvalidDuringQuery);
        return false;
    }
    for (Command &command : commands_)
    {
        if (!command(*storage_, error))
        {
            commands_.clear();
            return false;
        }
    }
    commands_.clear();
    setError(error, StorageError::None);
    return true;
}

void CommandBuffer::clear() noexcept
{
    commands_.clear();
}

} // namespace engine::ecs
