// Render graph plan builder: validation, read/write hazard analysis,
// topological pass ordering, and transient resource lifetimes.
//
// Engine-independent implementation of compiler.hpp (no kernel includes);
// the analysis is pure computation over the caller's arrays.

#include <engine/render_graph/compiler.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <queue>
#include <unordered_set>
#include <vector>

namespace engine::render_graph
{
namespace
{
// Hard cap on total resource uses across all passes.
inline constexpr std::uint64_t maxGraphUses = 1'000'000;
} // namespace

bool buildRenderGraphPlan(const RenderGraphPassDescription *passes, std::uint32_t passCount,
    const RenderGraphResourceDescription *resources, std::uint32_t resourceCount, RenderGraphPlan &outPlan,
    RenderGraphError &outError, bool includeResourceLifetimes)
{
    const auto fail = [&outError](RenderGraphError error)
    {
        outError = error;
        return false;
    };
    if ((passCount != 0 && passes == nullptr) || (resourceCount != 0 && resources == nullptr))
        return fail(RenderGraphError::InvalidInput);

    std::uint64_t totalUseCount = 0;
    // Resources need a non-zero size and a power-of-two alignment; transient
    // and initiallyDefined are mutually exclusive.
    for (std::uint32_t resource = 0; resource < resourceCount; ++resource)
    {
        const auto &description = resources[resource];
        if (description.sizeBytes == 0 || description.alignmentBytes == 0
            || (description.alignmentBytes & (description.alignmentBytes - 1u)) != 0)
            return fail(RenderGraphError::InvalidInput);
        if (description.transient && description.initiallyDefined)
            return fail(RenderGraphError::InvalidInput);
    }

    // Pass arrays must be consistent with their counts; total uses are
    // capped so analysis cost stays bounded.
    for (std::uint32_t pass = 0; pass < passCount; ++pass)
    {
        if ((passes[pass].useCount != 0 && passes[pass].uses == nullptr)
            || (passes[pass].dependencyCount != 0 && passes[pass].dependencies == nullptr))
            return fail(RenderGraphError::InvalidInput);
        totalUseCount += passes[pass].useCount;
        if (totalUseCount > maxGraphUses)
            return fail(RenderGraphError::CapacityExceeded);
    }

    std::vector<std::vector<std::uint32_t>> successors(passCount);
    std::vector<std::uint32_t>              indegree(passCount, 0);
    std::unordered_set<std::uint64_t>       edgeSet;
    edgeSet.reserve(static_cast<std::size_t>(passCount) * 2u + static_cast<std::size_t>(totalUseCount));
    outPlan.edges.clear();
    outPlan.edges.reserve(static_cast<std::size_t>(passCount) + static_cast<std::size_t>(totalUseCount));

    // Deduplicating edge insert (64-bit packed key); returns false only for
    // out-of-range indices or a self-dependency.
    const auto addEdge = [&](std::uint32_t prerequisite, std::uint32_t dependent) -> bool
    {
        if (prerequisite >= passCount || dependent >= passCount || prerequisite == dependent)
            return false;
        const std::uint64_t key = (static_cast<std::uint64_t>(prerequisite) << 32u) | dependent;
        if (!edgeSet.insert(key).second)
            return true;
        successors[prerequisite].push_back(dependent);
        ++indegree[dependent];
        outPlan.edges.emplace_back(prerequisite, dependent);
        return true;
    };

    for (std::uint32_t pass = 0; pass < passCount; ++pass)
    {
        for (std::uint32_t index = 0; index < passes[pass].dependencyCount; ++index)
        {
            if (!addEdge(passes[pass].dependencies[index], pass))
                return fail(RenderGraphError::InvalidInput);
        }
    }

    // Kahn's algorithm with a min-heap so ties resolve to the lowest pass
    // index, keeping the schedule deterministic.
    const auto topologicalOrder = [&]()
    {
        std::vector<std::uint32_t>                                                     order;
        std::vector<std::uint32_t>                                                     remaining = indegree;
        std::priority_queue<std::uint32_t, std::vector<std::uint32_t>, std::greater<>> ready;
        for (std::uint32_t pass = 0; pass < passCount; ++pass)
            if (remaining[pass] == 0)
                ready.push(pass);
        order.reserve(passCount);
        while (!ready.empty())
        {
            const std::uint32_t pass = ready.top();
            ready.pop();
            order.push_back(pass);
            for (const std::uint32_t dependent : successors[pass])
                if (--remaining[dependent] == 0)
                    ready.push(dependent);
        }
        return order;
    };

    std::vector<std::uint32_t> explicitOrder = topologicalOrder();
    if (explicitOrder.size() != passCount)
        return fail(RenderGraphError::DependencyCycle);

    // Hazard analysis in topological order: reads depend on the resource's
    // previous write, writes on the previous write and all pending reads.
    // Reading a transient/undefined resource before any write, or using one
    // resource twice in a single pass, is invalid.
    std::vector<std::uint32_t>              lastWriter(resourceCount, invalidResourceSlot);
    std::vector<std::vector<std::uint32_t>> readers(resourceCount);
    std::vector<std::uint32_t>              stamp(resourceCount, invalidResourceSlot);

    for (const std::uint32_t pass : explicitOrder)
    {
        for (std::uint32_t index = 0; index < passes[pass].useCount; ++index)
        {
            const RenderGraphResourceUse &use = passes[pass].uses[index];
            if (use.resourceIndex >= resourceCount || use.access > ResourceAccess::ReadWrite)
                return fail(RenderGraphError::InvalidInput);
            const std::uint32_t resource = use.resourceIndex;
            if (stamp[resource] == pass)
                return fail(RenderGraphError::InvalidInput);
            stamp[resource] = pass;

            const bool reads  = use.access != ResourceAccess::Write;
            const bool writes = use.access != ResourceAccess::Read;
            if (reads)
            {
                if (lastWriter[resource] == invalidResourceSlot
                    && (resources[resource].transient || !resources[resource].initiallyDefined))
                    return fail(RenderGraphError::InvalidInput);
                if (lastWriter[resource] != invalidResourceSlot && !addEdge(lastWriter[resource], pass))
                    return fail(RenderGraphError::InvalidInput);
            }

            if (writes)
            {
                if (lastWriter[resource] != invalidResourceSlot && !addEdge(lastWriter[resource], pass))
                    return fail(RenderGraphError::InvalidInput);
                for (const std::uint32_t reader : readers[resource])
                    if (!addEdge(reader, pass))
                        return fail(RenderGraphError::InvalidInput);
                readers[resource].clear();
                lastWriter[resource] = pass;
            }
            else
            {
                readers[resource].push_back(pass);
            }
        }
    }

    // Re-sort now that hazard edges exist; a cycle here means hazards
    // contradicted the explicit dependencies.
    outPlan.order = topologicalOrder();
    if (outPlan.order.size() != passCount)
        return fail(RenderGraphError::DependencyCycle);

    if (!includeResourceLifetimes)
    {
        outPlan.firstUse.clear();
        outPlan.lastUse.clear();
        outPlan.resourceOrder.clear();
        outError = RenderGraphError::None;
        return true;
    }

    // Lifetimes in final schedule order: firstUse/lastUse bracket the
    // schedule slots where a resource must exist; transient resources enter
    // resourceOrder (the aliasing walk), sorted by first use.
    outPlan.firstUse.assign(resourceCount, invalidResourceSlot);
    outPlan.lastUse.assign(resourceCount, invalidResourceSlot);
    outPlan.resourceOrder.clear();
    for (std::uint32_t scheduleIndex = 0; scheduleIndex < passCount; ++scheduleIndex)
    {
        const std::uint32_t pass = outPlan.order[scheduleIndex];
        for (std::uint32_t index = 0; index < passes[pass].useCount; ++index)
        {
            const std::uint32_t resource = passes[pass].uses[index].resourceIndex;
            if (outPlan.firstUse[resource] == invalidResourceSlot)
            {
                outPlan.firstUse[resource] = scheduleIndex;
                if (resources[resource].transient)
                    outPlan.resourceOrder.push_back(resource);
            }
            outPlan.lastUse[resource] = scheduleIndex;
        }
    }
    std::sort(outPlan.resourceOrder.begin(),
        outPlan.resourceOrder.end(),
        [&](std::uint32_t left, std::uint32_t right)
        {
            return outPlan.firstUse[left] != outPlan.firstUse[right] ? outPlan.firstUse[left] < outPlan.firstUse[right]
                                                                     : left < right;
        });
    outError = RenderGraphError::None;
    return true;
}

} // namespace engine::render_graph
