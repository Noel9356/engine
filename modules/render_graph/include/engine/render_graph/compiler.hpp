// Render graph analysis shared by the module's compile and execute paths.
//
// buildRenderGraphPlan validates a graph, orders passes topologically, adds
// read/write hazard edges, and optionally computes transient resource
// lifetimes for the aliasing allocator. Pure computation over the input
// arrays; no engine state involved.

#ifndef ENGINE_RENDER_GRAPH_COMPILER_HPP
#define ENGINE_RENDER_GRAPH_COMPILER_HPP

#include <engine/render_graph/capabilities.hpp>

#include <cstdint>
#include <utility>
#include <vector>

namespace engine::render_graph
{

// Analysis result for one graph shape (see buildRenderGraphPlan).
struct RenderGraphPlan
{
    std::vector<std::uint32_t>                           order;    // pass indices in execution order
    std::vector<std::pair<std::uint32_t, std::uint32_t>> edges;    // (prerequisite, dependent), hazards included
    std::vector<std::uint32_t>                           firstUse; // per-resource first schedule slot (or sentinel)
    std::vector<std::uint32_t>                           lastUse;  // per-resource last schedule slot (or sentinel)
    std::vector<std::uint32_t>                           resourceOrder; // transient resources by first use
};

// Validates and analyses one graph. Returns false with outError set
// (InvalidInput, DependencyCycle, CapacityExceeded) and outPlan partially
// filled; includeResourceLifetimes=false skips the aliasing lifetimes.
bool buildRenderGraphPlan(const RenderGraphPassDescription *passes, std::uint32_t passCount,
    const RenderGraphResourceDescription *resources, std::uint32_t resourceCount, RenderGraphPlan &outPlan,
    RenderGraphError &outError, bool includeResourceLifetimes = true);

} // namespace engine::render_graph

#endif
