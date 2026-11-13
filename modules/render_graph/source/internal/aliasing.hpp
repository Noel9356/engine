// Transient resource aliasing allocator (internal).
//
// Walks the plan's resourceOrder (transient resources by first use), retires
// physical slots whose last use is past, and reuses a compatible free slot
// or allocates a new one within the configured budget. Extracted from the
// module class so the allocator is independently testable and the module
// stays a thin capability binding.

#ifndef ENGINE_RENDER_GRAPH_INTERNAL_ALIASING_HPP
#define ENGINE_RENDER_GRAPH_INTERNAL_ALIASING_HPP

#include <engine/render_graph/capabilities.hpp>
#include <engine/render_graph/compiler.hpp>

#include <cstdint>

namespace engine::render_graph::detail
{

// Strategy for selecting a free slot when multiple compatible slots are
// available. BestFit minimises wasted space (default); FirstFit is cheapest
// to compute; WorstFit leaves large slots open for future large resources.
enum class AliasingStrategy : std::uint8_t
{
    BestFit = 0,
    FirstFit,
    WorstFit
};

// Inputs to the allocator. budgetBytes == 0 disables the budget check.
struct AliasingConfig
{
    bool            aliasingEnabled             = true;
    AliasingStrategy strategy                    = AliasingStrategy::BestFit;
    std::uint64_t   transientMemoryBudgetBytes   = 0;
};

// Outputs reported back to the caller for RenderGraphCompileResult.
struct AliasingResult
{
    std::uint32_t slotCount      = 0;
    std::uint64_t allocatedBytes = 0;
};

// Assigns transient resources to physical slots. outSlots must have room for
// resourceCount entries and is pre-filled with invalidResourceSlot for
// resources that never alias. On failure outError is set (BudgetExceeded)
// and the outputs are not meaningful.
bool allocateTransientSlots(const RenderGraphPlan &plan, const RenderGraphResourceDescription *resources,
    std::uint32_t resourceCount, std::uint32_t *outSlots, const AliasingConfig &config, AliasingResult &outResult,
    RenderGraphError &outError);

} // namespace engine::render_graph::detail

#endif
