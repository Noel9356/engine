// Bump-arena and fixed-size pool allocators for frame/hot-path
// allocations.

#ifndef ENGINE_KERNEL_MEMORY_HPP
#define ENGINE_KERNEL_MEMORY_HPP

#include <algorithm>
#include <cstddef>
#include <new>
#include <stdexcept>
#include <utility>
#include <vector>

namespace engine
{

// Rounds up to a non-zero power-of-two alignment; throws bad_alloc on
// overflow, invalid_argument on bad alignment.
[[nodiscard]] inline constexpr std::size_t alignUp(std::size_t value, std::size_t alignment)
{
    if (alignment == 0 || (alignment & (alignment - 1u)) != 0)
        throw std::invalid_argument("alignment must be a non-zero power of two");
    if (value > static_cast<std::size_t>(-1) - (alignment - 1u))
        throw std::bad_alloc();
    return (value + alignment - 1u) & ~(alignment - 1u);
}

// Grows by blocks; individual allocations are never freed. reset()
// rewinds all blocks (cheap), release() frees them. Pointers stay valid
// until release/destruction (not across reset!).
class LinearArena final
{
public:
    explicit LinearArena(std::size_t initialBlockSize = 64u * 1024u, std::size_t maximumReservedBytes = 0)
        : initialBlockSize_(std::max<std::size_t>(initialBlockSize, 256u)),
          maximumReservedBytes_(maximumReservedBytes)
    {}

    LinearArena(const LinearArena &)            = delete;
    LinearArena &operator=(const LinearArena &) = delete;

    ~LinearArena()
    {
        release();
    }

    // Scans blocks from currentBlock_ for space (larger alignments may skip
    // ahead), then appends a new block. size 0 consumes 1 byte so pointers
    // stay distinct.
    void *allocate(std::size_t size, std::size_t alignment = alignof(std::max_align_t))
    {
        if (size == 0)
            size = 1;
        if (!isValidAlignment(alignment))
            throw std::invalid_argument("arena alignment must be a non-zero power of two");

        for (std::size_t index = currentBlock_; index < blocks_.size(); ++index)
        {
            Block &block = blocks_[index];
            if (block.alignment < alignment)
                continue;
            const std::size_t previousUsed = block.used;
            const std::size_t offset       = alignUp(previousUsed, alignment);
            if (offset <= block.capacity && size <= block.capacity - offset)
            {
                currentBlock_ = index;
                block.used    = offset + size;
                usedBytes_ += block.used - previousUsed;
                return block.data + offset;
            }
        }

        // New blocks are at least initialBlockSize_, or exactly the oversized
        // request.
        const std::size_t capacity = std::max(initialBlockSize_, alignUp(size, alignment));
        // maximumReservedBytes_ is a hard cap on total reserved bytes; 0 disables
        // it.
        if (maximumReservedBytes_ != 0
            && capacity > maximumReservedBytes_ - std::min(maximumReservedBytes_, reservedBytes_))
            throw std::bad_alloc();
        Block block;
        block.alignment = std::max(alignment, alignof(std::max_align_t));
        block.data      = static_cast<std::byte *>(::operator new(capacity, std::align_val_t(block.alignment)));
        block.capacity  = capacity;
        block.used      = size;
        void *result    = block.data;
        try
        {
            blocks_.push_back(block);
        }
        catch (...)
        {
            ::operator delete(block.data, std::align_val_t(block.alignment));
            throw;
        }
        reservedBytes_ += capacity;
        usedBytes_ += size;
        currentBlock_ = blocks_.size() - 1u;
        return result;
    }

    // Placement-news T into the arena; no automatic destroy - call the
    // destructor manually when needed.
    template <typename T, typename... Args>
    T *create(Args &&...args)
    {
        return ::new (allocate(sizeof(T), alignof(T))) T(std::forward<Args>(args)...);
    }

    // Keeps the memory, rewinds the bump pointers.
    void reset() noexcept
    {
        for (Block &block : blocks_)
            block.used = 0;
        currentBlock_ = 0;
        usedBytes_    = 0;
    }

    // Frees every block back to the OS.
    void release() noexcept
    {
        for (Block &block : blocks_)
            ::operator delete(block.data, std::align_val_t(block.alignment));
        blocks_.clear();
        currentBlock_  = 0;
        usedBytes_     = 0;
        reservedBytes_ = 0;
    }

    [[nodiscard]] std::size_t usedBytes() const noexcept
    {
        return usedBytes_;
    }
    [[nodiscard]] std::size_t reservedBytes() const noexcept
    {
        return reservedBytes_;
    }
    [[nodiscard]] std::size_t blockCount() const noexcept
    {
        return blocks_.size();
    }

private:
    // alignment: over-aligned allocation (max_align_t minimum) so any type
    // fits in any block.
    struct Block
    {
        std::byte  *data      = nullptr;
        std::size_t capacity  = 0;
        std::size_t used      = 0;
        std::size_t alignment = alignof(std::max_align_t);
    };

    static constexpr bool isValidAlignment(std::size_t alignment) noexcept
    {
        return alignment != 0 && (alignment & (alignment - 1u)) == 0;
    }

    // currentBlock_: index where allocation scans start.
    std::vector<Block> blocks_;
    std::size_t        initialBlockSize_     = 0;
    std::size_t        maximumReservedBytes_ = 0;
    std::size_t        usedBytes_            = 0;
    std::size_t        reservedBytes_        = 0;
    std::size_t        currentBlock_         = 0;
};

// Fixed-stride free-list pool: allocate/deallocate are O(1); blocks grow
// by objectsPerBlock_. Objects are not constructed automatically - use
// create/destroy or placement-new manually.
class PoolAllocator final
{
public:
    explicit PoolAllocator(std::size_t objectSize, std::size_t objectAlignment = alignof(std::max_align_t),
        std::size_t objectsPerBlock = 256)
        : objectSize_(std::max(objectSize, sizeof(void *))),
          objectAlignment_(normaliseAlignment(objectAlignment)),
          stride_(alignUp(objectSize_, objectAlignment_)),
          objectsPerBlock_(std::max<std::size_t>(objectsPerBlock, 1))
    {}

    PoolAllocator(const PoolAllocator &)            = delete;
    PoolAllocator &operator=(const PoolAllocator &) = delete;

    ~PoolAllocator()
    {
        for (std::byte *block : blocks_)
            ::operator delete(block, std::align_val_t(objectAlignment_));
    }

    // Pops the free list, growing a new block when empty.
    void *allocate()
    {
        if (freeList_ == nullptr)
            grow();
        FreeNode *node = freeList_;
        freeList_      = node->next;
        ++liveObjects_;
        return node;
    }

    // LIFO free-list push; nullptr ignored.
    void deallocate(void *pointer) noexcept
    {
        if (pointer == nullptr)
            return;
        auto *node = static_cast<FreeNode *>(pointer);
        node->next = freeList_;
        freeList_  = node;
        if (liveObjects_ != 0)
            --liveObjects_;
    }

    // Validates that T fits the configured stride, then constructs in place;
    // construction failure returns the memory to the pool.
    template <typename T, typename... Args>
    T *create(Args &&...args)
    {
        if (sizeof(T) > objectSize_ || alignof(T) > objectAlignment_)
            throw std::invalid_argument("pooled type exceeds the configured object size or alignment");
        void *memory = allocate();
        try
        {
            return ::new (memory) T(std::forward<Args>(args)...);
        }
        catch (...)
        {
            deallocate(memory);
            throw;
        }
    }

    // Destructor + pool return.
    template <typename T>
    void destroy(T *object) noexcept(std::is_nothrow_destructible_v<T>)
    {
        if (object != nullptr)
        {
            object->~T();
            deallocate(object);
        }
    }

    [[nodiscard]] std::size_t liveObjects() const noexcept
    {
        return liveObjects_;
    }
    [[nodiscard]] std::size_t capacity() const noexcept
    {
        return blocks_.size() * objectsPerBlock_;
    }
    [[nodiscard]] std::size_t blockCount() const noexcept
    {
        return blocks_.size();
    }

private:
    // The free list is stored inside the free memory itself.
    struct FreeNode
    {
        FreeNode *next = nullptr;
    };

    static std::size_t normaliseAlignment(std::size_t alignment)
    {
        if (alignment == 0 || (alignment & (alignment - 1u)) != 0)
            throw std::invalid_argument("pool alignment must be a non-zero power of two");
        return std::max(alignment, alignof(void *));
    }

    // Links every slot of the new block into the free list (stride-aligned).
    void grow()
    {
        if (objectsPerBlock_ > static_cast<std::size_t>(-1) / stride_)
            throw std::bad_alloc();
        const std::size_t bytes = stride_ * objectsPerBlock_;
        auto             *block = static_cast<std::byte *>(::operator new(bytes, std::align_val_t(objectAlignment_)));
        try
        {
            blocks_.push_back(block);
        }
        catch (...)
        {
            ::operator delete(block, std::align_val_t(objectAlignment_));
            throw;
        }
        for (std::size_t index = 0; index < objectsPerBlock_; ++index)
        {
            auto *node = reinterpret_cast<FreeNode *>(block + index * stride_);
            node->next = freeList_;
            freeList_  = node;
        }
    }

    std::vector<std::byte *> blocks_;
    FreeNode                *freeList_        = nullptr;
    std::size_t              objectSize_      = 0;
    std::size_t              objectAlignment_ = 0;
    std::size_t              stride_          = 0;
    std::size_t              objectsPerBlock_ = 0;
    std::size_t              liveObjects_     = 0;
};

} // namespace engine

#endif
