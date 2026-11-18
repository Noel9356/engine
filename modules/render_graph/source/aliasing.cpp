// Transient resource aliasing allocator implementation.

#include <engine/render_graph/capabilities.hpp>
#include <engine/render_graph/compiler.hpp>
#include "internal/aliasing.hpp"

#include <algorithm>
#include <cstdint>
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

// One retired slot awaiting reuse, in the order it became available. The
// vector keeps availability order so first-fit can honour it; best/worst-fit
// scan by size instead.
struct AvailableSlot
{
    std::uint64_t size = 0;
    std::uint32_t slot = 0;
    std::uint32_t availabilityOrder = 0;
};

// Selects a slot from a non-empty bucket according to the strategy. Returns
// an index into bucket; the caller erases it. Ties prefer the slot that
// became available earliest, keeping the choice deterministic.
std::size_t selectSlot(const std::vector<AvailableSlot> &bucket, std::uint64_t sizeBytes, AliasingStrategy strategy)
{
    std::size_t selected = 0;
    const auto  prefer = [&bucket](std::size_t candidate, std::size_t incumbent)
    {
        return bucket[candidate].availabilityOrder < bucket[incumbent].availabilityOrder;
    };
    switch (strategy)
    {
    case AliasingStrategy::FirstFit:
    {
        // Earliest-available slot the resource fits into; first-fit is a
        // placement rule, so a non-fitting pool falls through to growth.
        bool fits = false;
        for (std::size_t index = 0; index < bucket.size(); ++index)
        {
            if (bucket[index].size >= sizeBytes && (!fits || prefer(index, selected)))
            {
                selected = index;
                fits     = true;
            }
        }
        if (fits)
        {
            return selected;
        }
        break;
    }
    case AliasingStrategy::BestFit:
    {
        // Smallest slot >= size (minimal internal fragmentation).
        bool fits = false;
        for (std::size_t index = 0; index < bucket.size(); ++index)
        {
            const bool indexFits = bucket[index].size >= sizeBytes;
            if (indexFits
                && (!fits || bucket[index].size < bucket[selected].size
                    || (bucket[index].size == bucket[selected].size && prefer(index, selected))))
            {
                selected = index;
                fits     = true;
            }
        }
        if (fits)
        {
            return selected;
        }
        break;
    }
    case AliasingStrategy::WorstFit:
    default:
        break;
    }
    // No slot fits (or worst-fit): take the largest slot, which grows to fit.
    for (std::size_t index = 1; index < bucket.size(); ++index)
    {
        if (bucket[index].size > bucket[selected].size
            || (bucket[index].size == bucket[selected].size && prefer(index, selected)))
        {
            selected = index;
        }
    }
    return selected;
}

} // namespace

bool allocateTransientSlots(const RenderGraphPlan &plan, const RenderGraphResourceDescription *resources,
    std::uint32_t resourceCount, std::uint32_t *outSlots, const AliasingConfig &config, AliasingResult &outResult,
    RenderGraphError &outError)
{
    std::vector<TransientSlot> slots;
    std::vector<PendingSlot>   pendingSlots;
    // Per-compatibility pool of retired slots, in availability order (see
    // AvailableSlot: the strategies scan it by size or by order).
    std::unordered_map<std::uint32_t, std::vector<AvailableSlot>> availableSlots;
    std::uint32_t nextAvailabilityOrder = 0;
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
                availableSlots[slot.compatibility].push_back(
                    AvailableSlot{slot.size, released.slot, nextAvailabilityOrder++});
            }
            auto found = availableSlots.find(description.compatibilityClass);
            if (found != availableSlots.end() && !found->second.empty())
            {
                const std::size_t candidate = selectSlot(found->second, description.sizeBytes, config.strategy);
                selected = found->second[candidate].slot;
                found->second.erase(found->second.begin() + static_cast<std::ptrdiff_t>(candidate));
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
