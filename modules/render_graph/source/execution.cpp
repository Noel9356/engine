// Cached TaskGraph execution implementation.

#include "internal/execution.hpp"

#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>
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
    RenderGraphError &outError) const
{
    // Only cache-owned plans can keep the cached TaskGraph alive safely.
    if (!enabled_ || !planOwner)
        return runPerCall(plan, passCount, callback, userData, scheduler, outError);

    std::unique_lock lock(mutex_, std::defer_lock);
    lock.lock();
    if (!cache_.executing.exchange(true, std::memory_order_acq_rel))
    {
        const bool success = runCached(cache_, planOwner, plan, passCount, callback, userData, scheduler, outError);
        // runCached always clears the executing flag before returning.
        return success;
    }
    lock.unlock();
    // Cache busy (concurrent caller): fall back to a per-call graph.
    return runPerCall(plan, passCount, callback, userData, scheduler, outError);
}

bool ExecutionCache::runCached(CachedExecution &cache, const std::shared_ptr<const RenderGraphPlan> &planOwner,
    const RenderGraphPlan &plan, std::uint32_t passCount, RenderGraphExecuteCallback callback, void *userData,
    TaskScheduler &scheduler, RenderGraphError &outError) const
{
    if (cache.plan.get() != &plan)
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
        cache.plan = planOwner;
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
    RenderGraphExecuteCallback callback, void *userData, TaskScheduler &scheduler, RenderGraphError &outError) const
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
