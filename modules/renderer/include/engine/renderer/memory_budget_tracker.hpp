// Device-local memory budget pressure tracking (engine::renderer).
//
// Pure policy over sampled (usage, budget) observations: peak usage, minimum
// headroom, and a hysteresis pressure state machine that reports every state
// transition exactly once. No Vulkan dependency, so the policy is unit-test
// covered independently of a GPU.
//
// P99 relevance: residency pressure (budget exhaustion, driver eviction,
// paging) shows up as rare, severe frame-time spikes. Tracking peak/minimum
// headroom beside the P99 histograms is what makes such a tail attributable
// (see potential_optimisations_gpu.md, "VRAM pressure and residency" and
// "Residency budgets and eviction avoidance").
//
// Sampling cadence is the caller's concern; the Vulkan device context feeds
// this from its periodic vmaGetHeapBudgets() refresh.

#ifndef ENGINE_RENDERER_MEMORY_BUDGET_TRACKER_HPP
#define ENGINE_RENDERER_MEMORY_BUDGET_TRACKER_HPP

#include <cstdint>

namespace engine
{
namespace renderer
{

enum class MemoryBudgetPressure : std::uint8_t
{
    None = 0,
    High
};

// State after a transition into or out of pressure.
struct MemoryBudgetEvent
{
    MemoryBudgetPressure pressure = MemoryBudgetPressure::None;
    std::uint64_t        usageBytes  = 0;
    std::uint64_t        budgetBytes = 0;
};

class MemoryBudgetTracker
{
public:
    // Pressure is entered at >= enterFraction of the budget in use and left
    // at <= leaveFraction. The gap between the two is the hysteresis band;
    // keep enter > leave so a sample hovering at the boundary cannot
    // oscillate the state. Defaults: enter at 7/8 used, leave at 3/4 used.
    explicit MemoryBudgetTracker(float enterFraction = 0.875f, float leaveFraction = 0.75f) noexcept
        : enterFraction_(enterFraction),
          leaveFraction_(leaveFraction)
    {}

    // Records one sample. Returns true when the sample caused a pressure
    // transition (details in lastEvent()); repeated identical samples are
    // silent. Samples with budgetBytes == 0 (budget unknown - the
    // VK_EXT_memory_budget path is unavailable) are ignored entirely so an
    // absent budget never fabricates pressure.
    bool record(std::uint64_t usageBytes, std::uint64_t budgetBytes) noexcept
    {
        if (budgetBytes == 0u)
        {
            return false;
        }

        ++sampleCount_;
        lastUsageBytes_  = usageBytes;
        lastBudgetBytes_ = budgetBytes;
        if (usageBytes > peakUsageBytes_)
        {
            peakUsageBytes_ = usageBytes;
        }

        // Over-commit (usage >= budget) means zero headroom: clamp instead of
        // wrapping, and treat it as pressure regardless of the fractions.
        const std::uint64_t headroom = usageBytes >= budgetBytes ? 0u : budgetBytes - usageBytes;
        if (headroom < minimumHeadroomBytes_)
        {
            minimumHeadroomBytes_ = headroom;
        }

        const bool overCommitted = usageBytes >= budgetBytes;
        bool       transition    = false;
        if (pressure_ == MemoryBudgetPressure::None)
        {
            if (overCommitted || static_cast<float>(usageBytes) >= enterFraction_ * static_cast<float>(budgetBytes))
            {
                pressure_   = MemoryBudgetPressure::High;
                transition  = true;
            }
        }
        else if (static_cast<float>(usageBytes) <= leaveFraction_ * static_cast<float>(budgetBytes))
        {
            pressure_  = MemoryBudgetPressure::None;
            transition = true;
        }

        if (transition)
        {
            lastEvent_.pressure    = pressure_;
            lastEvent_.usageBytes  = usageBytes;
            lastEvent_.budgetBytes = budgetBytes;
        }
        return transition;
    }

    bool inPressure() const noexcept
    {
        return pressure_ == MemoryBudgetPressure::High;
    }

    // Last recorded sample (0/0 before the first budget observation); the
    // renderer capability publishes these beside the pressure state so a
    // CPU-side asset policy can react to headroom, not just transitions.
    std::uint64_t lastUsageBytes() const noexcept
    {
        return lastUsageBytes_;
    }
    std::uint64_t lastBudgetBytes() const noexcept
    {
        return lastBudgetBytes_;
    }
    bool hasBudgetSample() const noexcept
    {
        return lastBudgetBytes_ != 0u;
    }

    MemoryBudgetPressure pressure() const noexcept
    {
        return pressure_;
    }

    // Highest usage observed since creation; 0 before the first sample.
    std::uint64_t peakUsageBytes() const noexcept
    {
        return peakUsageBytes_;
    }

    // Smallest observed (budget - usage). Starts at UINT64_MAX so the first
    // sample always defines it; treat UINT64_MAX as "no budget observation
    // yet".
    std::uint64_t minimumHeadroomBytes() const noexcept
    {
        return minimumHeadroomBytes_;
    }

    std::uint64_t sampleCount() const noexcept
    {
        return sampleCount_;
    }

    const MemoryBudgetEvent &lastEvent() const noexcept
    {
        return lastEvent_;
    }

    float enterFraction() const noexcept
    {
        return enterFraction_;
    }
    float leaveFraction() const noexcept
    {
        return leaveFraction_;
    }

    // Forgets history (peaks, headroom, samples) but keeps the current
    // pressure state, so a reset mid-pressure still leaves on the way back
    // down. Not used by the device context today; provided for long-running
    // diagnostics that want per-scene windows.
    void resetStatistics() noexcept
    {
        peakUsageBytes_       = 0;
        minimumHeadroomBytes_ = UINT64_MAX;
        sampleCount_          = 0;
    }

private:
    float                  enterFraction_;
    float                  leaveFraction_;
    MemoryBudgetPressure   pressure_             = MemoryBudgetPressure::None;
    MemoryBudgetEvent      lastEvent_{};
    std::uint64_t          lastUsageBytes_       = 0;
    std::uint64_t          lastBudgetBytes_      = 0;
    std::uint64_t          peakUsageBytes_       = 0;
    std::uint64_t          minimumHeadroomBytes_ = UINT64_MAX;
    std::uint64_t          sampleCount_          = 0;
};

} // namespace renderer
} // namespace engine

#endif
