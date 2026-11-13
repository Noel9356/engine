// Compiled-plan LRU cache (internal).
//
// Plans are keyed by structural equality of the graph (values, not pointers)
// so a stable graph shape is analysed once and reused across frames. The
// cache is mutex-protected and degrades to "no caching" on allocation
// failure instead of throwing.

#ifndef ENGINE_RENDER_GRAPH_INTERNAL_PLAN_CACHE_HPP
#define ENGINE_RENDER_GRAPH_INTERNAL_PLAN_CACHE_HPP

#include <engine/render_graph/capabilities.hpp>
#include <engine/render_graph/compiler.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace engine::render_graph::detail
{

class RenderGraphPlanCache final
{
public:
    // Entry capacity is hard-capped at 16; zero disables caching.
    void configure(std::size_t entryCapacity, std::size_t maximumGraphElements) noexcept;
    void clear() noexcept;

    [[nodiscard]] bool enabled() const noexcept;

    std::shared_ptr<const RenderGraphPlan> find(const RenderGraphPassDescription *passes, std::uint32_t passCount,
        const RenderGraphResourceDescription *resources, std::uint32_t resourceCount,
        bool includeResourceLifetimes) const noexcept;

    // Copies the graph shape into the entry so later lookups can compare
    // without the caller's arrays. Returns the stored (or matching) plan.
    std::shared_ptr<const RenderGraphPlan> store(const RenderGraphPassDescription *passes, std::uint32_t passCount,
        const RenderGraphResourceDescription *resources, std::uint32_t resourceCount, bool includeResourceLifetimes,
        std::size_t graphElementCount, RenderGraphPlan &plan) const noexcept;

private:
    struct PassShape
    {
        std::size_t   useOffset        = 0;
        std::size_t   dependencyOffset = 0;
        std::uint32_t useCount         = 0;
        std::uint32_t dependencyCount  = 0;
    };

    struct Entry
    {
        bool                                        includeResourceLifetimes = true;
        std::uint64_t                               lastUse                  = 0;
        std::vector<PassShape>                      passShapes;
        std::vector<RenderGraphResourceDescription> resources;
        std::vector<RenderGraphResourceUse>         uses;
        std::vector<std::uint32_t>                  dependencies;
        std::shared_ptr<const RenderGraphPlan>      plan;
    };

    static bool sameResource(
        const RenderGraphResourceDescription &left, const RenderGraphResourceDescription &right) noexcept;

    static bool sameUse(const RenderGraphResourceUse &left, const RenderGraphResourceUse &right) noexcept;

    // Full structural comparison: a hit must reproduce the same plan.
    static bool matches(const Entry &entry, const RenderGraphPassDescription *passes, std::uint32_t passCount,
        const RenderGraphResourceDescription *resources, std::uint32_t resourceCount,
        bool includeResourceLifetimes) noexcept;

    mutable std::mutex         mutex_;
    mutable std::vector<Entry> entries_;
    mutable std::uint64_t      clock_                = 0;
    std::size_t                capacity_             = 0;
    std::size_t                maximumGraphElements_ = 0;
};

} // namespace engine::render_graph::detail

#endif
