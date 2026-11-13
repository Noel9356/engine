// FlatKeyMap: open-addressing hash map keyed by ChunkKey.
//
// The octree and the chunk cache look nodes up thousands of times per update. A sorted vector pays a binary search
// (and an O(n) shift per insertion); node-based maps pay a pointer chase per lookup. This table keeps entries in one
// contiguous slot array with linear probing, so a lookup normally touches a single cache line and never allocates.
//
// Pointers and references returned by find()/findOrInsert() are invalidated by any later insertion that grows the
// table and by eraseIf(); callers must not hold them across such calls. Capacity is reserved up front so steady-state
// streaming never rehashes.

#ifndef ENGINE_PLANET_FLAT_KEY_MAP_HPP
#define ENGINE_PLANET_FLAT_KEY_MAP_HPP

#include <engine/planet/types.hpp>

#include <bit>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace engine
{
namespace planet
{

template <typename Value>
class FlatKeyMap final
{
public:
    explicit FlatKeyMap(std::size_t expectedEntries = 1024)
    {
        allocate(capacityFor(expectedEntries));
    }

    [[nodiscard]] std::size_t size() const noexcept
    {
        return size_;
    }

    [[nodiscard]] bool empty() const noexcept
    {
        return size_ == 0;
    }

    // Grows the table (once, up front) so `expectedEntries` fit without a later rehash.
    void reserve(std::size_t expectedEntries)
    {
        const std::size_t wanted = capacityFor(expectedEntries);
        if (wanted > slots_.size())
        {
            rehash(wanted);
        }
    }

    void clear()
    {
        for (Slot &slot : slots_)
        {
            slot = Slot{};
        }
        size_ = 0;
    }

    [[nodiscard]] Value *find(ChunkKey key) noexcept
    {
        const std::size_t index = locate(key);
        return slots_[index].key == key ? &slots_[index].value : nullptr;
    }

    [[nodiscard]] const Value *find(ChunkKey key) const noexcept
    {
        const std::size_t index = locate(key);
        return slots_[index].key == key ? &slots_[index].value : nullptr;
    }

    [[nodiscard]] bool contains(ChunkKey key) const noexcept
    {
        return find(key) != nullptr;
    }

    // Returns the entry for `key`, default-constructing it when absent.
    Value &findOrInsert(ChunkKey key)
    {
        std::size_t index = locate(key);
        if (slots_[index].key == key)
        {
            return slots_[index].value;
        }
        if ((size_ + 1) * 2 > slots_.size())
        {
            rehash(slots_.size() * 2);
            index = locate(key);
        }
        slots_[index].key = key;
        ++size_;
        return slots_[index].value;
    }

    // Removes every entry for which predicate(key, value) is true. Rebuilds the table in one pass, so tombstones
    // never accumulate and probe sequences stay short.
    template <typename Predicate>
    void eraseIf(Predicate &&predicate)
    {
        std::vector<Slot> previous(slots_.size());
        previous.swap(slots_);
        size_ = 0;
        for (Slot &slot : previous)
        {
            if (slot.key == emptyKey || predicate(slot.key, slot.value))
            {
                continue;
            }
            const std::size_t index = locate(slot.key);
            slots_[index].key       = slot.key;
            slots_[index].value     = std::move(slot.value);
            ++size_;
        }
    }

    // Visits every entry as visitor(key, value).
    template <typename Visitor>
    void forEach(Visitor &&visitor)
    {
        for (Slot &slot : slots_)
        {
            if (slot.key != emptyKey)
            {
                visitor(slot.key, slot.value);
            }
        }
    }

    template <typename Visitor>
    void forEach(Visitor &&visitor) const
    {
        for (const Slot &slot : slots_)
        {
            if (slot.key != emptyKey)
            {
                visitor(slot.key, slot.value);
            }
        }
    }

private:
    // Real keys pack the depth into the top nibble and 20-bit coordinates below it, so the all-ones pattern (depth 15
    // with every coordinate at its maximum) never addresses a node of a valid tree.
    static constexpr ChunkKey emptyKey = ~ChunkKey{0};

    struct Slot
    {
        ChunkKey key = emptyKey;
        Value    value{};
    };

    static std::size_t hashKey(ChunkKey key) noexcept
    {
        // 64-bit finalizer (MurmurHash3 fmix64): packed coordinates differ in few bits, so they must be scrambled
        // before masking to the table size.
        key ^= key >> 33;
        key *= 0xFF51AFD7ED558CCDull;
        key ^= key >> 33;
        key *= 0xC4CEB9FE1A85EC53ull;
        key ^= key >> 33;
        return static_cast<std::size_t>(key);
    }

    // Smallest power-of-two capacity keeping the load factor at or below one half.
    static std::size_t capacityFor(std::size_t expectedEntries) noexcept
    {
        const std::size_t wanted = expectedEntries < 8 ? 16 : expectedEntries * 2;
        return std::bit_ceil(wanted);
    }

    void allocate(std::size_t capacity)
    {
        slots_.assign(capacity, Slot{});
        size_ = 0;
    }

    // Index of the slot holding `key`, or of the empty slot where it would be inserted.
    [[nodiscard]] std::size_t locate(ChunkKey key) const noexcept
    {
        const std::size_t mask  = slots_.size() - 1;
        std::size_t       index = hashKey(key) & mask;
        while (slots_[index].key != emptyKey && slots_[index].key != key)
        {
            index = (index + 1) & mask;
        }
        return index;
    }

    void rehash(std::size_t capacity)
    {
        std::vector<Slot> previous;
        previous.swap(slots_);
        allocate(capacity);
        for (Slot &slot : previous)
        {
            if (slot.key == emptyKey)
            {
                continue;
            }
            const std::size_t index = locate(slot.key);
            slots_[index].key       = slot.key;
            slots_[index].value     = std::move(slot.value);
            ++size_;
        }
    }

    std::vector<Slot> slots_;
    std::size_t       size_ = 0;
};

} // namespace planet
} // namespace engine

#endif // ENGINE_PLANET_FLAT_KEY_MAP_HPP
