// Shared fixed-step accumulator for the engine's simulation modules.
//
// The engine's stepping contract is constant-delta: `step` advances exactly
// one fixed step, and `simulate` accumulates wall time and drains it in whole
// fixed steps. Every simulation module used to re-implement that drain loop;
// this header keeps the single canonical behavior:
//
//  - elapsed seconds are added to the budget and whole fixed deltas are
//    executed through the caller's step function until the budget no longer
//    covers one step or the per-call step cap is reached;
//  - when the cap is hit the residual backlog is dropped, so a persistent
//    deficit cannot grow without bound and starve the budget;
//  - a failed step (a simulator reporting divergence) stops the batch and
//    also drops the residual backlog, so a diverging simulator is not retried
//    in a tight loop from stale debt.

#pragma once

#include <cmath>
#include <cstdint>

namespace engine::util
{

// Fixed-step time budget for one simulated object (a world, an emitter, a
// cloth, ...). The owning record keeps one instance and passes its step
// function to `drain`.
struct StepAccumulator
{
    double budgetSeconds = 0.0;

    // Adds `elapsedSeconds` to the budget and executes whole fixed steps of
    // `fixedDeltaSeconds` through `step` (a callable returning true on
    // success, false on a failed step that ends the batch). `maximumSteps`
    // bounds the executed count per call. Returns the number of executed
    // steps.
    template <typename StepFn>
    std::uint32_t drain(double fixedDeltaSeconds, std::uint32_t maximumSteps, double elapsedSeconds, StepFn &&step)
    {
        if (!std::isfinite(elapsedSeconds) || elapsedSeconds <= 0.0 || !(fixedDeltaSeconds > 0.0)
            || maximumSteps == 0u)
        {
            return 0u;
        }
        budgetSeconds += elapsedSeconds;
        std::uint32_t executed = 0;
        while (budgetSeconds >= fixedDeltaSeconds && executed < maximumSteps)
        {
            budgetSeconds -= fixedDeltaSeconds;
            if (!step())
            {
                budgetSeconds = 0.0;
                break;
            }
            ++executed;
        }
        if (executed == maximumSteps)
        {
            budgetSeconds = 0.0;
        }
        return executed;
    }
};

} // namespace engine::util
