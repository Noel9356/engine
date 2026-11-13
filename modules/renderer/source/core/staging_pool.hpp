// Fence-recycled staging allocator policy (engine::renderer, module-internal).
//
// Pure resource policy over (capacity, cursor, outstanding-count) block
// records: no Vulkan types, no GPU memory calls, no I/O - the device context
// drives the actual Vulkan/VMA calls from the decisions made here. Kept
// dependency-free so it is unit-test covered independently of a GPU (same
// pattern as memory_budget_tracker.hpp and draw_sort.hpp).
//
// P99 relevance (potential_optimisations_hybrid.md, candidates 1 and 11):
// every host-to-device upload previously allocated a fresh staging buffer
// (vmaCreateBuffer -> map -> flush -> unmap -> destroy on fence completion).
// An asset burst - the frames where P99 lives - turns that into one
// allocation cycle per upload, competing with rendering for the same VMA
// allocator and the same host-visible memory. The pool keeps a small set of
// persistently mapped staging blocks and recycles a block only after every
// copy from it has signaled its fence, so steady-state uploads pay no
// allocation or map/unmap cost while total staging memory stays bounded by
// maxPoolBytes.
//
// Invariants the unit tests pin down:
//   * spans are bump-allocated inside a block and never overlap;
//   * span offsets and flush sizes honour the caller's alignment
//     (nonCoherentAtomSize) so vmaFlushAllocation ranges stay valid;
//   * a retired block becomes reusable only after every span allocated from
//     it has been released (its fence signaled) - a span is never
//     overwritten while its copy may still be executing;
//   * pooled memory is capped: when a completed block would push the total
//     over maxPoolBytes it is dropped (the caller destroys its storage)
//     instead of recycled; open blocks are never dropped;
//   * requests larger than maxBlockBytes never enter the pool - the caller
//     keeps its one-off staging path for them;
//   * acquire() always terminates: a request that no free block can fit
//     falls through to new-block creation / the one-off path instead of
//     being re-offered the same undersized block forever.
//
// Caller protocol (see VulkanDeviceContext):
//   span = pool.acquire(size, &newCapacity);
//   if (result == NeedNewBlock) { create storage of newCapacity bytes;
//                                 pool.beginBlock(actualCapacity);
//                                 span = pool.acquire(size, nullptr); }
//   ... memcpy into blockBase + span.offset ... flush(span.flushOffset,
//   span.flushSize) ... submit; on fence completion: pool.release(span).

#ifndef ENGINE_RENDERER_MODULE_STAGING_POOL_HPP
#define ENGINE_RENDERER_MODULE_STAGING_POOL_HPP

#include <cstdint>
#include <limits>
#include <vector>

namespace engine
{
namespace renderer
{

class StagingPool
{
public:
    using BlockToken = std::uint32_t;

    static constexpr BlockToken invalidBlock = 0;

    enum class AcquireResult : std::uint8_t
    {
        Served,       // *outSpan valid: memcpy into it, flush, submit
        NeedNewBlock, // *outNewCapacity set: create storage, beginBlock(), re-acquire
        UseFallback   // caller's one-off staging path (oversize / zero / pool full)
    };

    // One allocated range. `size` is the caller's requested byte count;
    // `flushOffset`/`flushSize` is the alignment-rounded range the caller
    // must flush (host-visible non-coherent memory) before submitting.
    struct Span
    {
        BlockToken    block       = invalidBlock;
        std::uint64_t offset      = 0; // write offset inside the block
        std::uint64_t size        = 0; // requested bytes (unaligned)
        std::uint64_t flushOffset = 0; // aligned range covering [offset, offset + size)
        std::uint64_t flushSize   = 0;
    };

    // alignment: byte alignment for every span offset and flush size (pass
    // the device's nonCoherentAtomSize; power of two per spec, 1 is valid).
    // maxBlockBytes: requests above this never enter the pool.
    // maxPoolBytes: ceiling on total caller-owned pooled block bytes.
    // blockStepBytes: new-block capacity granularity (default 1 MiB); the
    // engine keeps the default, tests use small steps.
    static constexpr std::uint64_t kDefaultBlockStep = 1024ull * 1024ull;
    StagingPool(std::uint64_t alignment,
        std::uint64_t        maxBlockBytes,
        std::uint64_t        maxPoolBytes,
        std::uint64_t        blockStepBytes = kDefaultBlockStep) noexcept
        : alignment_(normalizeAlignment(alignment)),
          maxBlockBytes_(maxBlockBytes),
          maxPoolBytes_(maxPoolBytes),
          blockStep_(blockStepBytes == 0 ? kDefaultBlockStep : blockStepBytes)
    {}

    // See the caller protocol above. `outNewCapacity`/`outSpan` may be null
    // when the caller does not need that output.
    AcquireResult acquire(std::uint64_t size, Span *outSpan, std::uint64_t *outNewCapacity) noexcept
    {
        if (outSpan != nullptr)
            *outSpan = {};
        if (outNewCapacity != nullptr)
            *outNewCapacity = 0;
        if (size == 0 || size > maxBlockBytes_)
            return AcquireResult::UseFallback;

        // Serve from the open block when possible; a failed serve retires
        // the block (it is full after alignment), so the loop moves on to
        // free blocks and then to a new block. openFreeBlock() only opens a
        // block that can fit this request, so every iteration either serves,
        // consumes one free block, or falls through to the returns below -
        // the loop cannot re-offer the same undersized block forever.
        for (;;)
        {
            if (open_ != 0 && serve(size, outSpan))
                return AcquireResult::Served;
            if (openFreeBlock(size))
                continue;
            if (outNewCapacity == nullptr || !canGrow(size))
                return AcquireResult::UseFallback;
            *outNewCapacity = blockCapacityFor(size);
            return AcquireResult::NeedNewBlock;
        }
    }

    // Registers freshly created storage as the pool's open block and
    // returns its token. `actualCapacity` may exceed the requested one; the
    // pool records the real value. The only throwing operation in this
    // class (the block record append); callers that must not throw wrap it
    // and treat a throw as "create no block" (the device context falls back
    // to its one-off staging path).
    BlockToken beginBlock(std::uint64_t actualCapacity)
    {
        if (actualCapacity == 0 || nextToken_ == std::numeric_limits<BlockToken>::max()
            || actualCapacity > maxPoolBytes_ - std::min(pooledBytes_, maxPoolBytes_))
            return invalidBlock;
        const BlockToken token = nextToken_++;
        blocks_.push_back(Block{token, actualCapacity, 0, 0, true});
        pooledBytes_ += actualCapacity;
        open_ = token;
        ++blocksCreated_;
        return token;
    }

    // Marks the span's copy complete. Call exactly once per served span,
    // after its fence signaled. Returns DropBlock (with *dropBlockCapacity
    // set) when the pool dropped the block (byte cap) and the caller must
    // destroy its storage; KeepBlock when the block returned to the free
    // list; otherwise Deferred (block still open or spans outstanding).
    enum class ReleaseResult : std::uint8_t
    {
        Deferred,
        KeepBlock,
        DropBlock
    };
    ReleaseResult release(const Span &span, std::uint64_t *dropBlockCapacity = nullptr) noexcept
    {
        if (dropBlockCapacity != nullptr)
            *dropBlockCapacity = 0;
        Block *block = findBlock(span.block);
        if (block == nullptr || block->pending == 0)
            return ReleaseResult::Deferred;
        --block->pending;
        --outstandingSpans_;
        pendingBytes_ -= span.size;
        if (block->pending != 0 || block->open)
            return ReleaseResult::Deferred;
        // Complete, retired block: recycle under the byte cap, else drop.
        if (pooledBytes_ <= maxPoolBytes_)
        {
            block->used = 0;
            return ReleaseResult::KeepBlock;
        }
        if (dropBlockCapacity != nullptr)
            *dropBlockCapacity = block->capacity;
        pooledBytes_ -= block->capacity;
        removeBlock(block);
        ++blocksDroppedByCap_;
        return ReleaseResult::DropBlock;
    }

    // Bytes the caller still owns as pool blocks (open + free).
    [[nodiscard]] std::uint64_t pooledBytes() const noexcept
    {
        return pooledBytes_;
    }
    [[nodiscard]] std::uint32_t liveBlocks() const noexcept
    {
        return static_cast<std::uint32_t>(blocks_.size());
    }
    [[nodiscard]] std::uint32_t outstandingSpans() const noexcept
    {
        return outstandingSpans_;
    }

    // Teardown/bring-up diagnostics (never read on the frame path; the
    // device context logs them once at destroy).
    [[nodiscard]] std::uint32_t blocksCreated() const noexcept
    {
        return blocksCreated_;
    }
    [[nodiscard]] std::uint32_t blocksReused() const noexcept
    {
        return blocksReused_;
    }
    [[nodiscard]] std::uint32_t blocksDroppedByCap() const noexcept
    {
        return blocksDroppedByCap_;
    }
    [[nodiscard]] std::uint64_t bytesStaged() const noexcept
    {
        return bytesStaged_;
    }
    [[nodiscard]] std::uint64_t peakPendingBytes() const noexcept
    {
        return peakPendingBytes_;
    }
    // Bytes currently reserved by unreleased spans (in-flight copies).
    [[nodiscard]] std::uint64_t pendingBytes() const noexcept
    {
        return pendingBytes_;
    }

    // Forgets all bookkeeping; the caller must have destroyed every block's
    // storage already (device teardown path).
    void reset() noexcept
    {
        blocks_.clear();
        open_             = 0;
        pooledBytes_      = 0;
        outstandingSpans_ = 0;
        pendingBytes_     = 0;
        nextToken_        = 1;
    }

private:
    struct Block
    {
        BlockToken    id       = 0;
        std::uint64_t capacity = 0;
        std::uint64_t used     = 0;
        std::uint32_t pending  = 0;
        bool          open     = false; // true while bump-allocating
    };

    static std::uint64_t normalizeAlignment(std::uint64_t alignment) noexcept
    {
        // Spec-valid nonCoherentAtomSize values are powers of two in
        // [1, 256]; treat anything else (0, non-pow2) as byte alignment.
        if (alignment == 0 || (alignment & (alignment - 1)) != 0)
            return 1;
        return alignment;
    }

    // Rounded-up block capacity with alignment slack so the first span of a
    // fresh block always fits even when the offset rounds up.
    [[nodiscard]] std::uint64_t blockCapacityFor(std::uint64_t size) const noexcept
    {
        const std::uint64_t blockCount = size / blockStep_ + (size % blockStep_ != 0 ? 1u : 0u);
        if (blockCount > (std::numeric_limits<std::uint64_t>::max() - alignment_) / blockStep_)
            return std::numeric_limits<std::uint64_t>::max();
        return blockCount * blockStep_ + alignment_;
    }

    [[nodiscard]] bool canGrow(std::uint64_t size) const noexcept
    {
        const std::uint64_t capacity = blockCapacityFor(size);
        return capacity <= maxPoolBytes_ && pooledBytes_ <= maxPoolBytes_ - capacity;
    }

    // Opens the smallest free block that can fit `size` bytes (best fit);
    // false when no free block fits. The fit check is what keeps acquire()
    // terminating: without it, a request larger than every free block would
    // open the same undersized block, fail to serve, retire it, and re-open
    // it forever (an infinite CPU spin with no error and no log output).
    [[nodiscard]] bool openFreeBlock(std::uint64_t size) noexcept
    {
        Block *best = nullptr;
        for (Block &block : blocks_)
        {
            if (block.pending != 0 || block.open)
                continue;
            if (block.used > std::numeric_limits<std::uint64_t>::max() - (alignment_ - 1u))
                continue;
            const std::uint64_t offset = roundUp(block.used, alignment_);
            const std::uint64_t flushSize = roundUp(size, alignment_);
            if (offset > std::numeric_limits<std::uint64_t>::max() - flushSize
                || offset + flushSize > block.capacity)
                continue; // cannot fit the write and its aligned flush range
            if (best == nullptr || block.capacity < best->capacity)
                best = &block;
        }
        if (best == nullptr)
            return false;
        best->open = true;
        open_      = best->id;
        ++blocksReused_;
        return true;
    }

    // Bump-allocates from the open block. Returns false when the block
    // cannot fit the (aligned) request, in which case the block is retired
    // (a block with no outstanding spans becomes free-list eligible at once,
    // otherwise release() recycles it later) and the caller retries with the
    // next block.
    [[nodiscard]] bool serve(std::uint64_t size, Span *outSpan) noexcept
    {
        Block *block = findBlock(open_);
        if (block == nullptr)
        {
            open_ = 0; // unreachable; guards the open_ invariant
            return false;
        }
        if (block->used > std::numeric_limits<std::uint64_t>::max() - (alignment_ - 1u))
        {
            block->open = false;
            open_       = 0;
            if (block->pending == 0)
                block->used = 0;
            return false;
        }
        const std::uint64_t offset = roundUp(block->used, alignment_);
        if (offset > std::numeric_limits<std::uint64_t>::max() - size)
        {
            block->open = false;
            open_       = 0;
            if (block->pending == 0)
                block->used = 0;
            return false;
        }
        const std::uint64_t flushSize = roundUp(size, alignment_);
        if (offset > std::numeric_limits<std::uint64_t>::max() - flushSize
            || offset + flushSize > block->capacity)
        {
            block->open = false;
            open_       = 0;
            if (block->pending == 0)
                block->used = 0;
            return false;
        }
        const std::uint64_t end = offset + size;
        block->used = end;
        ++block->pending;
        ++outstandingSpans_;
        bytesStaged_ += size;
        pendingBytes_ += size;
        if (pendingBytes_ > peakPendingBytes_)
            peakPendingBytes_ = pendingBytes_;
        if (outSpan != nullptr)
            *outSpan = Span{open_, offset, size, offset, flushSize};
        return true;
    }

    [[nodiscard]] Block *findBlock(BlockToken token) noexcept
    {
        if (token == invalidBlock)
            return nullptr;
        for (Block &block : blocks_)
        {
            if (block.id == token)
                return &block;
        }
        return nullptr;
    }

    void removeBlock(const Block *block) noexcept
    {
        if (open_ == block->id)
            open_ = 0;
        for (std::size_t index = 0; index < blocks_.size(); ++index)
        {
            if (blocks_[index].id == block->id)
            {
                blocks_.erase(blocks_.begin() + static_cast<std::ptrdiff_t>(index));
                return;
            }
        }
    }

    static std::uint64_t roundUp(std::uint64_t value, std::uint64_t alignment) noexcept
    {
        if (alignment <= 1)
            return value;
        const std::uint64_t remainder = value % alignment;
        if (remainder == 0)
            return value;
        const std::uint64_t padding = alignment - remainder;
        return value > std::numeric_limits<std::uint64_t>::max() - padding
            ? std::numeric_limits<std::uint64_t>::max()
            : value + padding;
    }

    std::uint64_t      alignment_;
    std::uint64_t      maxBlockBytes_;
    std::uint64_t      maxPoolBytes_;
    std::uint64_t      blockStep_;
    std::vector<Block> blocks_;
    BlockToken         open_             = 0; // token of the block taking bump allocations
    std::uint64_t      pooledBytes_      = 0; // caller-owned block storage, bytes
    std::uint32_t      outstandingSpans_ = 0;
    std::uint64_t      pendingBytes_     = 0;
    BlockToken         nextToken_        = 1;
    std::uint32_t      blocksCreated_      = 0;
    std::uint32_t      blocksReused_       = 0;
    std::uint32_t      blocksDroppedByCap_ = 0;
    std::uint64_t      bytesStaged_        = 0;
    std::uint64_t      peakPendingBytes_   = 0;
};

} // namespace renderer
} // namespace engine

#endif
