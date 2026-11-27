// Cached TaskGraph execution implementation.

#include <engine/kernel/task_scheduler.hpp>
#include <engine/render_graph/capabilities.hpp>
#include <engine/render_graph/compiler.hpp>
#include "internal/execution.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace engine::render_graph::detail
{

void ExecutionCache::CachedExecution::release() noexcept
{
    plan.reset();
    bindings.reset();
    graph.reset();
    nodeForPass.clear();
}

void ExecutionCache::setEnabled(bool enabled) noexcept
{
    enabled_ = enabled;
}

bool ExecutionCache::enabled() const noexcept
{
    return enabled_;
}

bool ExecutionCache::execute(const std::shared_ptr<const RenderGraphPlan> &planOwner, const RenderGraphPlan &plan,
    std::uint32_t passCount, RenderGraphExecuteCallback callback, void *userData, TaskScheduler &scheduler,
    RenderGraphError &outError, const std::vector<std::pair<std::uint32_t, std::uint32_t>> *extraEdges) const
{
    // Only cache-owned plans can keep the cached TaskGraph alive safely.
    if (!enabled_.load(std::memory_order_relaxed) || !planOwner)
        return runPerCall(plan, passCount, callback, userData, scheduler, outError, extraEdges);

    bool cacheBusy = false;
    {
        std::lock_guard lock(mutex_);
        cacheBusy = cache_.executing.exchange(true, std::memory_order_acq_rel);
    }
    if (cacheBusy)
    {
        // Cache busy (concurrent caller): fall back to a per-call graph.
        return runPerCall(plan, passCount, callback, userData, scheduler, outError, extraEdges);
    }
    // The owner proceeds with the mutex released: the cached TaskGraph's
    // execution is synchronous (TaskGraph::execute waits for every pass on
    // the calling thread, and its workers may re-enter this path), so
    // holding the mutex across it would serialize concurrent callers the
    // flag promises a fallback for and deadlock a re-entrant one. Ownership
    // of the cache is serialized by the executing flag alone - only the
    // owner ever mutates the cached graph or its bindings, and a caller
    // that observes the flag busy never touches them.
    return runCached(cache_, planOwner, plan, passCount, callback, userData, scheduler, outError, extraEdges);
}

bool ExecutionCache::runCached(CachedExecution &cache, const std::shared_ptr<const RenderGraphPlan> &planOwner,
    const RenderGraphPlan &plan, std::uint32_t passCount, RenderGraphExecuteCallback callback, void *userData,
    TaskScheduler &scheduler, RenderGraphError &outError,
    const std::vector<std::pair<std::uint32_t, std::uint32_t>> *extraEdges) const
{
    if (cache.plan.get() != &plan)
    {
        try
        {
            cache.graph.reset();
            cache.nodeForPass.assign(passCount, invalidResourceSlot);
            cache.bindings          = std::make_shared<ExecutionBindings>();
            std::vector<std::uint32_t> &nodes = cache.nodeForPass;
            for (std::uint32_t pass = 0; pass < passCount; ++pass)
                nodes[pass] = cache.graph.addTask(
                    [bindings = cache.bindings, pass]
                    {
                        const RenderGraphExecuteCallback currentCallback = bindings->callback;
                        void                            *currentUserData = bindings->userData;
                        if (currentCallback != nullptr)
                            currentCallback(currentUserData, pass);
                    });
            for (const auto &[prerequisite, dependent] : plan.edges)
                if (!cache.graph.addDependency(nodes[prerequisite], nodes[dependent]))
                {
                    cache.release();
                    cache.executing.store(false, std::memory_order_release);
                    outError = RenderGraphError::InvalidInput;
                    return false;
                }
            // The aliasing handoff edges join the plan's own hazard edges: a
            // slot's previous owner must finish before the resource reusing the
            // slot starts, whatever topological order the scheduler picks.
            if (extraEdges != nullptr)
            {
                for (const auto &[prerequisite, dependent] : *extraEdges)
                    if (!cache.graph.addDependency(nodes[prerequisite], nodes[dependent]))
                    {
                        cache.release();
                        cache.executing.store(false, std::memory_order_release);
                        outError = RenderGraphError::InvalidInput;
                        return false;
                    }
            }
            cache.plan = planOwner;
        }
        catch (...)
        {
            // A rebuild that cannot finish (node, closure or adjacency
            // allocation failure) leaves the cache empty but usable: the flag
            // must clear so later calls can retry instead of silently taking
            // the per-call path forever.
            cache.release();
            cache.executing.store(false, std::memory_order_release);
            outError = RenderGraphError::ExecutionFailed;
            return false;
        }
    }
    cache.bindings->callback = callback;
    cache.bindings->userData = userData;
    try
    {
        cache.graph.execute(scheduler);
    }
    catch (...)
    {
        cache.bindings->callback = nullptr;
        cache.bindings->userData = nullptr;
        cache.executing.store(false, std::memory_order_release);
        outError = RenderGraphError::ExecutionFailed;
        return false;
    }
    // Clear the bindings so a stale callback can never run if the cached
    // graph were re-executed outside this call.
    cache.bindings->callback = nullptr;
    cache.bindings->userData = nullptr;
    cache.executing.store(false, std::memory_order_release);
    outError = RenderGraphError::None;
    return true;
}

bool ExecutionCache::runPerCall(const RenderGraphPlan &plan, std::uint32_t passCount,
    RenderGraphExecuteCallback callback, void *userData, TaskScheduler &scheduler, RenderGraphError &outError,
    const std::vector<std::pair<std::uint32_t, std::uint32_t>> *extraEdges) const
{
    try
    {
        TaskGraph                 graph;
        std::vector<std::uint32_t> nodes(passCount, invalidResourceSlot);
        for (std::uint32_t pass = 0; pass < passCount; ++pass)
            nodes[pass] = graph.addTask([callback, userData, pass] { callback(userData, pass); });
        for (const auto &[prerequisite, dependent] : plan.edges)
            if (!graph.addDependency(nodes[prerequisite], nodes[dependent]))
            {
                outError = RenderGraphError::InvalidInput;
                return false;
            }
        if (extraEdges != nullptr)
        {
            for (const auto &[prerequisite, dependent] : *extraEdges)
                if (!graph.addDependency(nodes[prerequisite], nodes[dependent]))
                {
                    outError = RenderGraphError::InvalidInput;
                    return false;
                }
        }
        graph.execute(scheduler);
    }
    catch (...)
    {
        outError = RenderGraphError::ExecutionFailed;
        return false;
    }
    outError = RenderGraphError::None;
    return true;
}

} // namespace engine::render_graph::detail
