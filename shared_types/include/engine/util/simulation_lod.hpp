// Shared simulation level-of-detail vocabulary for the engine's simulation
// modules.
//
// Header-only and dependency-free, so any module (and any application built
// on the engine) can adopt simulation LOD without pulling in a scheduler or
// another module's capability. The cooperating pieces live here:
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
//  3. The shared configuration reader and pass resolver: every module reads
//     the same lod_* keys into one SimulationLodSettings and resolves each
//     update pass through simulationLodResolvePass, so the ladder, the
//     cadence and the skip accounting behave identically everywhere a
//     module adopts them.
//
// The caller owns the policy: it derives importance however the application
// measures relevance (camera distance, screen coverage, gameplay tag), picks
// the per-band intervals, and decides what each band means for its own
// integrator. Constrained systems (cloth, ropes, fluids, rigid contact
// islands) keep their fixed step size and lower their fidelity through
// cheaper solver settings (simulationLodSubstepsForBand, per-constraint
// iteration budgets); unconstrained systems (particles, ambient springs) may
// cover one full interval of wall time per update by integrating with
// interval x delta; force appliers scale their forces by the interval so the
// impulse per wall time stays exact.

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

// Per-module simulation-LOD configuration: the shared fidelity ladder plus
// the per-band update intervals, all read from one configuration section.
// intervals is indexed by simulationFidelityIndex (Cached, Animation,
// Procedural, Simplified, Full); interval 0 freezes that band's objects.
struct SimulationLodSettings
{
    bool                          enabled     = false;
    SimulationFidelityThresholds  thresholds{};
    std::uint32_t                 intervals[kSimulationFidelityBandCount] = {0u, 8u, 4u, 2u, 1u};
};

// Reads the standard simulation-LOD keys from one configuration section:
// lod_enabled, lod_full_threshold, lod_simplified_threshold,
// lod_procedural_threshold, lod_animation_threshold, lod_demotion_margin,
// lod_interval_simplified, lod_interval_procedural, lod_interval_animation
// and lod_interval_cached. `Context` is the ParticipantContext the module
// receives in construct (duck-typed so this header stays dependency-free).
// Out-of-range or non-finite values keep their defaults; a threshold
// combination that is not strictly descending resets the whole ladder to
// the defaults, matching the per-module readers this replaces.
template <typename Context>
SimulationLodSettings readSimulationLodSettings(const Context &context, const char *section)
{
    SimulationLodSettings settings;
    settings.enabled = context.configurationBooleanInSection(section, "lod_enabled").value_or(false);
    const auto readThreshold = [&context, section](const char *key, double &target)
    {
        if (const auto value = context.configurationFloatingInSection(section, key);
            value && *value >= 0.0 && *value <= 1.0 && std::isfinite(*value))
        {
            target = *value;
        }
    };
    readThreshold("lod_full_threshold", settings.thresholds.fullThreshold);
    readThreshold("lod_simplified_threshold", settings.thresholds.simplifiedThreshold);
    readThreshold("lod_procedural_threshold", settings.thresholds.proceduralThreshold);
    readThreshold("lod_animation_threshold", settings.thresholds.animationThreshold);
    readThreshold("lod_demotion_margin", settings.thresholds.demotionMargin);
    if (!simulationFidelityThresholdsValid(settings.thresholds))
    {
        settings.thresholds = SimulationFidelityThresholds{};
    }
    const auto readInterval = [&context, section](const char *key, std::uint32_t &target)
    {
        if (const auto value = context.configurationIntegerInSection(section, key);
            value && *value >= 0 && *value <= 65536)
        {
            target = static_cast<std::uint32_t>(*value);
        }
    };
    readInterval(
        "lod_interval_simplified", settings.intervals[simulationFidelityIndex(SimulationFidelity::Simplified)]);
    readInterval(
        "lod_interval_procedural", settings.intervals[simulationFidelityIndex(SimulationFidelity::Procedural)]);
    readInterval("lod_interval_animation", settings.intervals[simulationFidelityIndex(SimulationFidelity::Animation)]);
    readInterval("lod_interval_cached", settings.intervals[simulationFidelityIndex(SimulationFidelity::Cached)]);
    return settings;
}

// Resolves one LOD-gated update pass: the fidelity band follows the clamped
// importance through the ladder (with demotion hysteresis), the cadence
// staggers passes across fixed steps, and the return value is the
// wall-time multiple a due pass must cover - interval fixed steps - so an
// unconstrained integrator steps with interval x delta and a force applier
// scales its forces by the same factor. Returns 0 when the pass should
// skip this step (a frozen band, or the staggered cadence is not due);
// every skip bumps *skippedSteps when the pointer is non-null. With
// `enabled` false every pass runs Full and is due every step.
inline std::uint32_t simulationLodResolvePass(SimulationFidelity &band, SimulationCadence &cadence,
    const SimulationFidelityThresholds &thresholds, const std::uint32_t *intervals, bool enabled, double importance,
    std::uint64_t *skippedSteps)
{
    if (!enabled)
    {
        return 1u;
    }
    band = simulationFidelityForImportance(thresholds, importance, band);
    const std::uint32_t interval = intervals[simulationFidelityIndex(band)];
    if (interval == 0u)
    {
        cadence.interval = 1u;
        if (skippedSteps != nullptr)
        {
            ++*skippedSteps;
        }
        return 0u;
    }
    cadence.interval = interval;
    if (!simulationCadenceAdvance(cadence))
    {
        if (skippedSteps != nullptr)
        {
            ++*skippedSteps;
        }
        return 0u;
    }
    return interval;
}

// Substep budget for a constrained solver running in `band`: Full keeps the
// configured substeps, Simplified halves them, Procedural quarters them,
// and Animation/Cached keep one - always at least one substep, because the
// fixed step size never changes for constrained systems.
inline std::uint32_t simulationLodSubstepsForBand(SimulationFidelity band, std::uint32_t fullSubsteps)
{
    const std::uint32_t clamped = fullSubsteps < 1u ? 1u : fullSubsteps;
    switch (band)
    {
    case SimulationFidelity::Full:
        return clamped;
    case SimulationFidelity::Simplified:
        return clamped / 2u < 1u ? 1u : clamped / 2u;
    case SimulationFidelity::Procedural:
        return clamped / 4u < 1u ? 1u : clamped / 4u;
    case SimulationFidelity::Animation:
    case SimulationFidelity::Cached:
    default:
        return 1u;
    }
}

} // namespace engine::util
