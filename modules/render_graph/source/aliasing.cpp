// Transient resource aliasing allocator implementation.

#include "internal/aliasing.hpp"

#include <algorithm>
#include <cstdint>
#include <map>
#include <unordered_map>
#include <vector>

namespace engine::render_graph::detail
{
namespace
{

// One physical allocation that transient resources alias. size and alignment
// grow to fit every resource assigned to the slot.
struct TransientSlot
{
    std::uint32_t compatibility = 0;
    std::uint32_t lastUse       = 0;
    std::uint64_t size          = 0;
    std::uint64_t alignment     = 1;
};

// A live slot together with the schedule position where its resource is
// last used. Kept in a heap ordered so the earliest-expiring slot is on top.
struct PendingSlot
{
    std::uint32_t lastUse = 0;
    std::uint32_t slot    = 0;
};

// Comparator making std::push_heap/pop_heap treat the smallest lastUse as
// the "greatest" (front of the heap), so the earliest-expiring slot is
// retired first.
struct PendingSlotEarlier
{
    bool operator()(const PendingSlot &left, const PendingSlot &right) const noexcept
    {
        return left.lastUse != right.lastUse ? left.lastUse > right.lastUse : left.slot > right.slot;
    }
};

// Remaining budget given what is already allocated; never underflows.
inline std::uint64_t remainingBudget(std::uint64_t budget, std::uint64_t allocated) noexcept
{
    return allocated >= budget ? 0 : budget - allocated;
}

// Selects a slot from a non-empty bucket according to the strategy. Returns
// an iterator into bucket; the caller erases it.
std::multimap<std::uint64_t, std::uint32_t>::iterator selectSlot(
    std::multimap<std::uint64_t, std::uint32_t> &bucket, std::uint64_t sizeBytes, AliasingStrategy strategy)
{
    switch (strategy)
    {
    case AliasingStrategy::FirstFit:
        // First slot >= size; if none, take the largest (will grow).
        {
            auto it = bucket.lower_bound(sizeBytes);
            return it != bucket.end() ? it : std::prev(bucket.end());
        }
    case AliasingStrategy::WorstFit:
        // Largest slot; spreads usage, leaves small slots for small resources.
        return bucket.empty() ? bucket.end() : std::prev(bucket.end());
    case AliasingStrategy::BestFit:
    default:
        // Smallest slot >= size; if none, take the largest (minimal growth).
        {
            auto it = bucket.lower_bound(sizeBytes);
            return it != bucket.end() ? it : std::prev(bucket.end());
        }
    }
}

} // namespace

bool allocateTransientSlots(const RenderGraphPlan &plan, const RenderGraphResourceDescription *resources,
    std::uint32_t resourceCount, std::uint32_t *outSlots, const AliasingConfig &config, AliasingResult &outResult,
    RenderGraphError &outError)
{
    std::vector<TransientSlot> slots;
    std::vector<PendingSlot>   pendingSlots;
    // Per-compatibility pool of free slots keyed by size for best-fit lookup.
    std::unordered_map<std::uint32_t, std::multimap<std::uint64_t, std::uint32_t>> availableSlots;
    slots.reserve(plan.resourceOrder.size());
    pendingSlots.reserve(plan.resourceOrder.size());
    if (resourceCount != 0)
        std::fill(outSlots, outSlots + resourceCount, invalidResourceSlot);

    std::uint64_t allocatedBytes = 0;
    for (const std::uint32_t resource : plan.resourceOrder)
    {
        const auto   &description = resources[resource];
        std::uint32_t selected    = invalidResourceSlot;
        if (config.aliasingEnabled)
        {
            // Retire slots whose last use precedes this resource's first use.
            while (!pendingSlots.empty() && pendingSlots.front().lastUse < plan.firstUse[resource])
            {
                std::pop_heap(pendingSlots.begin(), pendingSlots.end(), PendingSlotEarlier{});
                const PendingSlot released = pendingSlots.back();
                pendingSlots.pop_back();
                const auto &slot = slots[released.slot];
                availableSlots[slot.compatibility].emplace(slot.size, released.slot);
            }
            auto found = availableSlots.find(description.compatibilityClass);
            if (found != availableSlots.end() && !found->second.empty())
            {
                auto candidate = selectSlot(found->second, description.sizeBytes, config.strategy);
                selected = candidate->second;
                found->second.erase(candidate);
            }
        }

        if (selected == invalidResourceSlot)
        {
            selected = static_cast<std::uint32_t>(slots.size());
            slots.push_back(TransientSlot{description.compatibilityClass, plan.lastUse[resource], description.sizeBytes,
                description.alignmentBytes});
            if (description.sizeBytes > remainingBudget(config.transientMemoryBudgetBytes, allocatedBytes))
            {
                outError = RenderGraphError::BudgetExceeded;
                return false;
            }
            allocatedBytes += description.sizeBytes;
        }
        else
        {
            TransientSlot &slot = slots[selected];
            if (description.sizeBytes > slot.size)
            {
                const std::uint64_t growth = description.sizeBytes - slot.size;
                if (growth > remainingBudget(config.transientMemoryBudgetBytes, allocatedBytes))
                {
                    outError = RenderGraphError::BudgetExceeded;
                    return false;
                }
                allocatedBytes += growth;
                slot.size = description.sizeBytes;
            }
            slot.lastUse   = plan.lastUse[resource];
            slot.alignment = std::max(slot.alignment, description.alignmentBytes);
        }
        outSlots[resource] = selected;
        if (config.aliasingEnabled)
        {
            pendingSlots.push_back(PendingSlot{plan.lastUse[resource], selected});
            std::push_heap(pendingSlots.begin(), pendingSlots.end(), PendingSlotEarlier{});
        }
    }

    outResult.slotCount      = static_cast<std::uint32_t>(slots.size());
    outResult.allocatedBytes = allocatedBytes;
    outError                 = RenderGraphError::None;
    return true;
}

} // namespace engine::render_graph::detail
