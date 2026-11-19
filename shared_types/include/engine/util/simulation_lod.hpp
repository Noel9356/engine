// Shared simulation level-of-detail vocabulary for the engine's simulation
// modules.
//
// Header-only and dependency-free, so any module (and any application built
// on the engine) can adopt simulation LOD without pulling in a scheduler or
// another module's capability. Two cooperating pieces live here:
//
//  1. The fidelity ladder: an importance score in [0, 1] maps onto five
//     fidelity bands (Cached, Animation, Procedural, Simplified, Full) with
//     demotion hysteresis. Promotions apply immediately; demotions wait for
//     the score to fall below the current band's entry threshold times the
//     demotion margin, which keeps boundary scores (distance jitter, screen
//     coverage oscillation) from flickering an object between bands every
//     step. The bands carry the same meaning as the physics_scheduling
//     module's FidelityLevel, whose values match band for band.
//
//  2. The staggered cadence: a per-object update interval expressed in fixed
//     steps plus a phase derived from the object's registration order, so
//     objects sharing an interval spread their work evenly across steps
//     instead of bunching into one heavy pass.
//
// The caller owns the policy: it derives importance however the application
// measures relevance (camera distance, screen coverage, gameplay tag), picks
// the per-band intervals, and decides what each band means for its own
// integrator. Constrained systems (cloth, ropes, fluids) keep their fixed
// step size and lower their fidelity through cheaper solver settings;
// unconstrained systems (particles, ambient springs) may cover one full
// interval of wall time per update by integrating with interval x delta.

#pragma once

#include <cmath>
#include <cstdint>

namespace engine::util
{

// Fidelity bands, from the cheapest to the most detailed. The values match
// physics_scheduling's FidelityLevel one for one, so consumers of both can
// cast between them.
enum class SimulationFidelity : std::uint32_t
{
    Cached = 0,     // no continuously running simulation; state holds still
    Animation,      // predefined motion with physical adjustments
    Procedural,     // mathematical models and springs
    Simplified,     // reduced solver complexity
    Full            // the detailed physical model
};

inline constexpr std::uint32_t kSimulationFidelityBandCount = 5u;

// Importance thresholds that map a score in [0, 1] onto the ladder. They must
// be finite and strictly descending from Full down to Animation; scores below
// animationThreshold sit in Cached.
struct SimulationFidelityThresholds
{
    double fullThreshold       = 0.75;
    double simplifiedThreshold = 0.5;
    double proceduralThreshold = 0.25;
    double animationThreshold  = 0.1;

    // Demotion hysteresis fraction in [0, 1): an item keeps its current band
    // until its score falls below the band's entry threshold times this
    // margin. Zero switches immediately in both directions.
    double demotionMargin = 0.8;
};

// Index of a band for tables (per-band intervals, per-band statistics).
inline std::uint32_t simulationFidelityIndex(SimulationFidelity band)
{
    return static_cast<std::uint32_t>(band);
}

// Band from a table index; out-of-range indices clamp to Full.
inline SimulationFidelity simulationFidelityFromIndex(std::uint32_t index)
{
    return index >= kSimulationFidelityBandCount ? SimulationFidelity::Full : static_cast<SimulationFidelity>(index);
}

// True when the four thresholds are finite and strictly descending with the
// same bounds the physics_scheduling module enforces, and the demotion margin
// lies in [0, 1).
inline bool simulationFidelityThresholdsValid(const SimulationFidelityThresholds &thresholds)
{
    return std::isfinite(thresholds.fullThreshold) && std::isfinite(thresholds.simplifiedThreshold)
        && std::isfinite(thresholds.proceduralThreshold) && std::isfinite(thresholds.animationThreshold)
        && thresholds.fullThreshold > thresholds.simplifiedThreshold
        && thresholds.simplifiedThreshold > thresholds.proceduralThreshold
        && thresholds.proceduralThreshold > thresholds.animationThreshold && thresholds.animationThreshold > 0.0
        && thresholds.animationThreshold <= 1.0 && thresholds.fullThreshold <= 1.0
        && std::isfinite(thresholds.demotionMargin) && thresholds.demotionMargin >= 0.0
        && thresholds.demotionMargin < 1.0;
}

// The entry threshold that granted `band`: scores below this threshold times
// the demotion margin release the object back down the ladder.
inline double simulationFidelityEntryThreshold(const SimulationFidelityThresholds &thresholds, SimulationFidelity band)
{
    switch (band)
    {
    case SimulationFidelity::Full:
        return thresholds.fullThreshold;
    case SimulationFidelity::Simplified:
        return thresholds.simplifiedThreshold;
    case SimulationFidelity::Procedural:
        return thresholds.proceduralThreshold;
    case SimulationFidelity::Animation:
        return thresholds.animationThreshold;
    case SimulationFidelity::Cached:
    default:
        return 0.0;
    }
}

// The band a score maps onto by the thresholds alone (no hysteresis).
inline SimulationFidelity simulationFidelityForScore(const SimulationFidelityThresholds &thresholds, double score)
{
    if (score >= thresholds.fullThreshold)
    {
        return SimulationFidelity::Full;
    }
    if (score >= thresholds.simplifiedThreshold)
    {
        return SimulationFidelity::Simplified;
    }
    if (score >= thresholds.proceduralThreshold)
    {
        return SimulationFidelity::Procedural;
    }
    if (score >= thresholds.animationThreshold)
    {
        return SimulationFidelity::Animation;
    }
    return SimulationFidelity::Cached;
}

// Resolves the fidelity band for a clamped importance score, keeping the
// current band when demotion hysteresis applies: promotions apply
// immediately, demotions wait until the score falls below the current band's
// entry threshold times the margin. The importance value is clamped to
// [0, 1]; non-finite scores resolve to Cached.
inline SimulationFidelity simulationFidelityForImportance(
    const SimulationFidelityThresholds &thresholds, double importance, SimulationFidelity currentBand)
{
    if (!std::isfinite(importance))
    {
        return SimulationFidelity::Cached;
    }
    const double clamped = importance <= 0.0 ? 0.0 : (importance >= 1.0 ? 1.0 : importance);
    const SimulationFidelity target = simulationFidelityForScore(thresholds, clamped);
    if (target >= currentBand)
    {
        return target;
    }
    const double releaseAt = simulationFidelityEntryThreshold(thresholds, currentBand) * thresholds.demotionMargin;
    return clamped < releaseAt ? target : currentBand;
}

// Staggered update cadence for one simulated object. interval counts fixed
// steps between updates (1 = every step); phase spreads objects sharing an
// interval across steps; stepCounter advances once per fixed step for every
// object, whether it updates or not.
struct SimulationCadence
{
    std::uint32_t interval    = 1;
    std::uint32_t phase       = 0;
    std::uint64_t stepCounter = 0;
};

// True when the object updates on the current step.
inline bool simulationCadenceDue(const SimulationCadence &cadence)
{
    if (cadence.interval <= 1u)
    {
        return true;
    }
    return (cadence.stepCounter + cadence.phase) % cadence.interval == 0u;
}

// Advances the cadence to the next fixed step and reports whether the object
// is due on it. Call once per fixed step for every simulated object, due or
// not, so the stagger phases stay aligned.
inline bool simulationCadenceAdvance(SimulationCadence &cadence)
{
    ++cadence.stepCounter;
    return simulationCadenceDue(cadence);
}

// Clamp helper for importance scores written through module setters.
inline double simulationImportanceClamped(double importance)
{
    if (!std::isfinite(importance) || importance <= 0.0)
    {
        return 0.0;
    }
    return importance >= 1.0 ? 1.0 : importance;
}

} // namespace engine::util
