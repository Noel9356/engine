// Cached TaskGraph execution for render graph passes (internal).
//
// A cached plan can be executed every frame without per-frame node,
// std::function or adjacency allocations: one TaskGraph is built per plan
// shape and rebound to the current callback through ExecutionBindings.
// Concurrent or alternating use falls back to a per-call graph.

#pragma once

#include <engine/kernel/task_scheduler.hpp>
#include <engine/render_graph/capabilities.hpp>
#include <engine/render_graph/compiler.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace engine::render_graph::detail
{

// Rebindable callback slot shared between the caller and the cached graph's
// task closures. Writes happen while the executing flag is held (the owning
// execute() call); reads happen on scheduler worker threads after the
// TaskGraph::execute release barrier.
struct ExecutionBindings
{
    RenderGraphExecuteCallback callback = nullptr;
    void                      *userData = nullptr;
};

// One prebuilt TaskGraph per cached plan shape. executing rejects overlapping
// use (callers fall back to the per-call graph path, matching the historical
// behaviour for concurrent execution); the flag alone serializes ownership
// of the cached graph and its bindings. All state is mutable so the public
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
    // on the scheduler. extraEdges (prerequisite, dependent) pass pairs are
    // wired in addition to the plan's own edges - the transient aliasing
    // handoffs that keep slot-reuse pairs ordered; pass nullptr when the
    // caller has none. Returns true on success; false (with outError set to
    // InvalidInput or ExecutionFailed) when the graph cannot be built or the
    // scheduler rejects it. When the cache is disabled or busy, falls back to
    // a per-call graph. planOwner keeps the plan alive while the cached graph
    // exists; pass a null shared_ptr when the plan is not cache-owned. The
    // extra edges must be a deterministic function of the plan (the cached
    // graph bakes them in per plan object; a changed edge set arrives with a
    // rebuilt plan).
    bool execute(const std::shared_ptr<const RenderGraphPlan> &planOwner, const RenderGraphPlan &plan,
        std::uint32_t passCount, RenderGraphExecuteCallback callback, void *userData, TaskScheduler &scheduler,
        RenderGraphError &outError, const std::vector<std::pair<std::uint32_t, std::uint32_t>> *extraEdges) const;

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
        TaskScheduler &scheduler, RenderGraphError &outError,
        const std::vector<std::pair<std::uint32_t, std::uint32_t>> *extraEdges) const;
    bool runPerCall(const RenderGraphPlan &plan, std::uint32_t passCount, RenderGraphExecuteCallback callback,
        void *userData, TaskScheduler &scheduler, RenderGraphError &outError,
        const std::vector<std::pair<std::uint32_t, std::uint32_t>> *extraEdges) const;

    // setEnabled runs on the main thread while execute() runs on render
    // threads, so the switch is atomic.
    std::atomic<bool>      enabled_{true};
    mutable std::mutex     mutex_;
    mutable CachedExecution cache_;
};

} // namespace engine::render_graph::detail

