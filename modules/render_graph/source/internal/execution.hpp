// Cached TaskGraph execution for render graph passes (internal).
//
// A cached plan can be executed every frame without per-frame node,
// std::function or adjacency allocations: one TaskGraph is built per plan
// shape and rebound to the current callback through ExecutionBindings.
// Concurrent or alternating use falls back to a per-call graph.

#ifndef ENGINE_RENDER_GRAPH_INTERNAL_EXECUTION_HPP
#define ENGINE_RENDER_GRAPH_INTERNAL_EXECUTION_HPP

#include <engine/render_graph/capabilities.hpp>
#include <engine/render_graph/compiler.hpp>
#include <engine/kernel/task_scheduler.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace engine::render_graph::detail
{

// Rebindable callback slot shared between the caller and the cached graph's
// task closures. Writes happen under the execution mutex; reads happen on
// scheduler worker threads after the TaskGraph::execute release barrier.
struct ExecutionBindings
{
    RenderGraphExecuteCallback callback = nullptr;
    void                      *userData = nullptr;
};

// One prebuilt TaskGraph per cached plan shape. executing rejects overlapping
// use (callers fall back to the per-call graph path, matching the historical
// behaviour for concurrent execution). All state is mutable so the public
// execute() entry can be called from the module's const executeImpl.
class ExecutionCache final
{
public:
    ExecutionCache() = default;
    ~ExecutionCache() = default;
    ExecutionCache(const ExecutionCache &)            = delete;
    ExecutionCache &operator=(const ExecutionCache &) = delete;

    void setEnabled(bool enabled) noexcept;
    [[nodiscard]] bool enabled() const noexcept;

    // Builds (or rebinds) the cached TaskGraph for the given plan and runs it
    // on the scheduler. Returns true on success; false (with outError set to
    // InvalidInput or ExecutionFailed) when the graph cannot be built or the
    // scheduler rejects it. When the cache is disabled or busy, falls back to
    // a per-call graph. planOwner keeps the plan alive while the cached graph
    // exists; pass a null shared_ptr when the plan is not cache-owned.
    bool execute(const std::shared_ptr<const RenderGraphPlan> &planOwner, const RenderGraphPlan &plan,
        std::uint32_t passCount, RenderGraphExecuteCallback callback, void *userData, TaskScheduler &scheduler,
        RenderGraphError &outError) const;

private:
    struct CachedExecution
    {
        void release() noexcept;

        std::shared_ptr<const RenderGraphPlan> plan;
        std::shared_ptr<ExecutionBindings>     bindings;
        TaskGraph                              graph;
        std::vector<std::uint32_t>             nodeForPass;
        std::atomic<bool>                      executing{false};
    };

    bool runCached(CachedExecution &cache, const std::shared_ptr<const RenderGraphPlan> &planOwner,
        const RenderGraphPlan &plan, std::uint32_t passCount, RenderGraphExecuteCallback callback, void *userData,
        TaskScheduler &scheduler, RenderGraphError &outError) const;
    bool runPerCall(const RenderGraphPlan &plan, std::uint32_t passCount, RenderGraphExecuteCallback callback,
        void *userData, TaskScheduler &scheduler, RenderGraphError &outError) const;

    bool                   enabled_ = true;
    mutable std::mutex     mutex_;
    mutable CachedExecution cache_;
};

} // namespace engine::render_graph::detail

#endif
