// lumen_reflection denoising chain, following Lumen's published order:
// spatial reuse -> temporal accumulation -> bilateral filter.
//
//   Spatial reuse: BRDF-weighted averaging over screen-space neighbors
//   (Lumen: "looks at screen space neighbors and reweights them based on
//   their BRDF"). Plane, normal, and roughness similarity gate the neighbors;
//   the pass also leaves a per-texel variance estimate (the true luminance
//   variance of the texel and its accepted neighbors, which the bilateral
//   stage then gates on).
//
//   Temporal accumulation: the previous frame's field is reprojected through
//   the stored matrices, validated against the current depth (disocclusion
//   detection), clamped to the current neighborhood (mean +/-
//   neighborhood_clamp_sigma * sigma so a changed scene cannot ghost
//   through), and blended with a history-length-driven weight: a texel whose
//   history has survived N clean frames blends as the running mean of N
//   fresh samples (weight N/(N+1), capped at temporal.strength), so a static
//   stochastic signal converges instead of re-showing each frame's variance
//   (an exponentially weighted average forgets at strength; a fresh sample
//   whose value disagrees with the clamped history by more than the
//   neighborhood spread marks the texel changed and restarts its history, so
//   relit content re-converges). Freshly revealed texels take the fresh frame
//   whole and are marked for the bilateral filter's double strength.
//
//   Bilateral filter: Lumen's "last ditch effort" - only texels whose
//   post-reuse variance is high are smoothed, with tone-mapped weights so a
//   firefly crushes only this pass's neighborhood, never the physically
//   based reuse above it.

#include "lumen_reflection_internal.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace engine
{
namespace lumen_reflection
{

namespace
{

// Tone-mapped weight (1/(1+L)): fireflies (huge L deltas) contribute a
// bounded share of the bilateral average, exactly the "tonemapped weighting
// to remove fireflies" the Lumen presentation describes for this pass.
inline float tonemappedWeight(float centerLuminance, float neighborLuminance)
{
    const float difference = std::fabs(centerLuminance - neighborLuminance);
    return 1.0f / (1.0f + difference);
}

// History-length depth cap: the running-mean weight N/(N+1) saturates long
// before this, so the counter only needs room to say "converged".
inline constexpr std::uint32_t historyLengthCap = 1024u;

// A neighborhood mean this far outside the clamped history (in standard
// errors of the mean, with an absolute floor for near-uniform neighborhoods)
// marks the texel changed and restarts its accumulation. Testing the mean of
// the 3x3 (not the single fresh sample) is what keeps a static stochastic
// texel converged: one firefly draw - the stochastic Fresnel pick that
// occasionally returns a bright lamp at full weight - moves a single sample
// far outside any bound, but it does not move the neighborhood mean, so the
// history survives it and the running mean converges instead of restarting
// every time a rare bright draw lands. A real relight moves the whole
// neighborhood together: the mean travels while its spread stays small, so
// the standard-error bound catches it within a few frames.
inline constexpr float historyChangeSigma   = 2.5f;
inline constexpr float historyChangeFloor    = 0.02f;

} // namespace

// ---- spatial reuse -------------------------------------------------------------------

void rfSpatialReuse(RfPipelineState &state, TaskScheduler *scheduler)
{
    const std::size_t pixelCount = static_cast<std::size_t>(state.fieldWidth) * state.fieldHeight;
    state.scratchSpatial.assign(pixelCount, glm::vec3(0.0f));
    state.scratchVariance.assign(pixelCount, 0.0f);
    if (state.fieldLuminance.size() != pixelCount)
    {
        state.fieldLuminance.resize(pixelCount);
    }

    const std::uint32_t radius    = state.settings.spatialReuseRadius;
    const float         planeTolerance = 0.1f; // same-surface plane distance gate (m)
    const float         roughnessTolerance = 0.15f;

    // Luminance of this frame's raw field, once per texel (the spatial pass
    // and the bilateral pass both weight through it).
    parallelRange(scheduler, state.fieldHeight, 1,
        [&](std::size_t rowBegin, std::size_t rowEnd)
        {
            for (std::size_t y = rowBegin; y < rowEnd; ++y)
            {
                for (std::uint32_t x = 0; x < state.fieldWidth; ++x)
                {
                    const std::size_t index = y * state.fieldWidth + x;
                    state.fieldLuminance[index] = state.fieldValid[index] != 0u
                        ? luminance(state.fieldRaw[index])
                        : 0.0f;
                }
            }
        });

    parallelRange(scheduler, state.fieldHeight, 1,
        [&](std::size_t rowBegin, std::size_t rowEnd)
        {
            for (std::size_t y = rowBegin; y < rowEnd; ++y)
            {
                for (std::uint32_t x = 0; x < state.fieldWidth; ++x)
                {
                    const std::size_t index = y * state.fieldWidth + x;
                    if (state.fieldValid[index] == 0u)
                    {
                        continue;
                    }
                    const glm::vec3 center = state.fieldRaw[index];
                    const glm::vec3 position = state.gbufferPosition[index];
                    const glm::vec3 normal   = state.gbufferNormal[index];
                    const float     roughness = state.gbufferRoughness[index];

                    glm::vec3 sum(0.0f);
                    float     weightSum = 0.0f;
                    // The texel itself is one sample of the variance estimate.
                    float     sampleCount = 1.0f;
                    const float centerLuminance = state.fieldLuminance[index];
                    float luminanceSum        = centerLuminance;
                    float luminanceSquaredSum = centerLuminance * centerLuminance;
                    for (std::int32_t offsetY = -static_cast<std::int32_t>(radius);
                         offsetY <= static_cast<std::int32_t>(radius); ++offsetY)
                    {
                        const std::int32_t sampledY = static_cast<std::int32_t>(y) + offsetY;
                        if (sampledY < 0 || sampledY >= static_cast<std::int32_t>(state.fieldHeight))
                        {
                            continue;
                        }
                        for (std::int32_t offsetX = -static_cast<std::int32_t>(radius);
                             offsetX <= static_cast<std::int32_t>(radius); ++offsetX)
                        {
                            const std::int32_t sampledX = static_cast<std::int32_t>(x) + offsetX;
                            if (sampledX < 0 || sampledX >= static_cast<std::int32_t>(state.fieldWidth))
                            {
                                continue;
                            }
                            const std::size_t sampledIndex
                                = static_cast<std::size_t>(sampledY) * state.fieldWidth + static_cast<std::size_t>(sampledX);
                            if (state.fieldValid[sampledIndex] == 0u)
                            {
                                continue;
                            }
                            const glm::vec3 sampledPosition = state.gbufferPosition[sampledIndex];
                            const glm::vec3 sampledNormal   = state.gbufferNormal[sampledIndex];
                            const float     sampledRoughness = state.gbufferRoughness[sampledIndex];

                            // BRDF similarity: same depth plane, compatible
                            // normal, compatible roughness. The plane gate
                            // compares |n.d| against the tolerance times the
                            // neighbor distance, so no per-neighbor
                            // normalization is needed.
                            const glm::vec3 delta = sampledPosition - position;
                            const float     deltaLength = glm::length(delta);
                            if (deltaLength > 0.0f
                                && std::fabs(glm::dot(normal, delta)) > planeTolerance * deltaLength)
                            {
                                continue;
                            }
                            if (glm::dot(normal, sampledNormal) < 0.75f)
                            {
                                continue;
                            }
                            if (std::fabs(roughness - sampledRoughness) > roughnessTolerance)
                            {
                                continue;
                            }
                            const glm::vec3 sampled = state.fieldRaw[sampledIndex];
                            const float sampledLuminance = state.fieldLuminance[sampledIndex];
                            // Value-aware neighbor weight: the reuse averages
                            // NOISY estimates of the same signal, but the
                            // deterministic estimator this pipeline traces
                            // also produces real high-dynamic-range edges
                            // (an emissive panel's mirror image towers over its
                            // surround by an order of magnitude), and an
                            // unweighted mean would smear those into the dark
                            // rim the eye reads first. The tone-mapped weight
                            // lets a neighbor with a similar luminance share
                            // fully while a far-brighter or far-dimmer one
                            // contributes a bounded share, so noise averages
                            // out and structure stays put.
                            const float value = tonemappedWeight(centerLuminance, sampledLuminance);
                            sum += sampled * value;
                            weightSum += value;
                            luminanceSum += sampledLuminance;
                            luminanceSquaredSum += sampledLuminance * sampledLuminance;
                            sampleCount += 1.0f;
                        }
                    }
                    if (weightSum <= 0.0f)
                    {
                        state.scratchSpatial[index] = center;
                        continue;
                    }
                    const glm::vec3 mean = sum / weightSum;
                    state.scratchSpatial[index] = mean;
                    // Luminance variance of the texel and its accepted
                    // neighbors (E[x^2] - E[x]^2 over the sample set).
                    const float meanLuminance = luminanceSum / sampleCount;
                    state.scratchVariance[index] = std::max(0.0f,
                        luminanceSquaredSum / sampleCount - meanLuminance * meanLuminance);
                }
            }
        });
}

// ---- temporal accumulation -----------------------------------------------------------

void rfTemporalAccumulate(RfPipelineState &state, TaskScheduler *scheduler)
{
    const std::size_t pixelCount = static_cast<std::size_t>(state.fieldWidth) * state.fieldHeight;
    // The published field of the previous frame becomes this pass's history
    // by swapping the buffers rather than copying them: this pass reassigns
    // fieldFiltered wholesale below, so after the swap the old history buffer
    // is scratch that the fill overwrites, and the reprojection reads the
    // last published field through historyField at no copy cost.
    state.historyField.swap(state.fieldFiltered);
    state.fieldFiltered.assign(pixelCount, glm::vec3(0.0f));
    state.scratchDisoccluded.assign(pixelCount, 0u);
    // The per-texel accumulation depth travels with the field: a resolution
    // change invalidates every texel's history, so every length restarts.
    if (state.historyLength.size() != pixelCount)
    {
        state.historyLength.assign(pixelCount, 0u);
    }

    const bool hasHistory = state.historyValid && state.historyWidth == state.fieldWidth
        && state.historyHeight == state.fieldHeight && !state.historyField.empty();
    const float strength = std::clamp(state.settings.temporalStrength, 0.0f, 1.0f);
    const float sigmaScale = std::max(state.settings.neighborhoodClampSigma, 0.0f);

    parallelRange(scheduler, state.fieldHeight, 1,
        [&](std::size_t rowBegin, std::size_t rowEnd)
        {
            for (std::size_t y = rowBegin; y < rowEnd; ++y)
            {
                for (std::uint32_t x = 0; x < state.fieldWidth; ++x)
                {
                    const std::size_t index = y * state.fieldWidth + x;
                    const glm::vec3 current = state.scratchSpatial[index];
                    if (state.fieldValid[index] == 0u)
                    {
                        state.fieldFiltered[index] = glm::vec3(0.0f);
                        state.scratchDisoccluded[index] = 1u;
                        state.historyLength[index]   = 0u;
                        continue;
                    }

                    bool historyUsable = false;
                    glm::vec3 history(0.0f);
                    if (hasHistory)
                    {
                        // Reproject this texel's world position into the
                        // previous frame's field and gather its history
                        // BILINEARLY: the reprojected point rarely lands on a
                        // texel center, so nearest-texel sampling re-read a
                        // neighbor up to half a texel away and stepped the
                        // accumulation across motion. Each footprint corner
                        // carries its bilinear weight only when it passes the
                        // same depth validation the nearest sample used, so
                        // disocclusions and silhouette borders reject the
                        // stale corners instead of blending across them.
                        const glm::vec4 clip
                            = state.historyViewProjection * glm::vec4(state.gbufferPosition[index], 1.0f);
                        if (clip.w > 0.05f)
                        {
                            const float u = (clip.x / clip.w * 0.5f + 0.5f) * static_cast<float>(state.fieldWidth) - 0.5f;
                            const float v = (0.5f - clip.y / clip.w * 0.5f) * static_cast<float>(state.fieldHeight) - 0.5f;
                            if (u >= 0.0f && v >= 0.0f && u <= static_cast<float>(state.fieldWidth) - 1.0f
                                && v <= static_cast<float>(state.fieldHeight) - 1.0f)
                            {
                                const float       floorU = std::floor(u);
                                const float       floorV = std::floor(v);
                                const float       fractionU = u - floorU;
                                const float       fractionV = v - floorV;
                                const std::uint32_t x0 = static_cast<std::uint32_t>(floorU);
                                const std::uint32_t y0 = static_cast<std::uint32_t>(floorV);
                                const float currentDepth = state.gbufferDepth[index];
                                float       weightSum = 0.0f;
                                glm::vec3   gathered(0.0f);
                                for (std::uint32_t corner = 0; corner < 4; ++corner)
                                {
                                    const std::uint32_t hx = x0 + (corner & 1u);
                                    const std::uint32_t hy = y0 + ((corner >> 1) & 1u);
                                    const float bilinear = ((corner & 1u) != 0u ? fractionU : 1.0f - fractionU)
                                        * (((corner >> 1) & 1u) != 0u ? fractionV : 1.0f - fractionV);
                                    if (bilinear <= 0.0f)
                                    {
                                        continue;
                                    }
                                    const std::size_t hIndex
                                        = static_cast<std::size_t>(hy) * state.fieldWidth + hx;
                                    // Depth validation per corner: the texel
                                    // that occupied this screen point last
                                    // frame must be the one (or the same
                                    // plane) that occupies it now, else the
                                    // surface was revealed and the corner is
                                    // rejected.
                                    const float historyDepth = state.historyDepth[hIndex];
                                    if (rfIsBackground(historyDepth)
                                        || std::fabs(historyDepth - currentDepth)
                                            >= 0.1f * std::max(currentDepth, 1.0f))
                                    {
                                        continue;
                                    }
                                    gathered += state.historyField[hIndex] * bilinear;
                                    weightSum += bilinear;
                                }
                                if (weightSum > 1.0e-6f)
                                {
                                    history = gathered / weightSum;
                                    historyUsable = true;
                                }
                            }
                        }
                    }

                    if (!historyUsable)
                    {
                        // Freshly revealed: the fresh frame whole, and the
                        // bilateral filter runs here at double strength.
                        state.fieldFiltered[index] = current;
                        state.scratchDisoccluded[index] = 1u;
                        state.historyLength[index]   = 0u;
                        continue;
                    }

                    // Neighborhood clamp bounds: the mean +/- sigma band of
                    // the current 3x3, widened to the neighborhood's own
                    // min/max so a changed scene cannot ghost through the
                    // accumulated history while deterministic
                    // high-dynamic-range structure survives. A mean +/- sigma
                    // band alone reads an emissive edge - a lamp's mirror
                    // image, radiance an order of magnitude over its
                    // surround - as an outlier and crushes its border texels
                    // into a dark rim; the range extension keeps any value
                    // the neighborhood already holds legal (a firefly still
                    // lands outside a uniform neighborhood's band).
                    glm::vec3 mean(0.0f);
                    glm::vec3 squaredMean(0.0f);
                    glm::vec3 neighborhoodMin(std::numeric_limits<float>::max());
                    glm::vec3 neighborhoodMax(-std::numeric_limits<float>::max());
                    float     count = 0.0f;
                    for (std::int32_t offsetY = -1; offsetY <= 1; ++offsetY)
                    {
                        const std::int32_t sampledY = static_cast<std::int32_t>(y) + offsetY;
                        if (sampledY < 0 || sampledY >= static_cast<std::int32_t>(state.fieldHeight))
                        {
                            continue;
                        }
                        for (std::int32_t offsetX = -1; offsetX <= 1; ++offsetX)
                        {
                            const std::int32_t sampledX = static_cast<std::int32_t>(x) + offsetX;
                            if (sampledX < 0 || sampledX >= static_cast<std::int32_t>(state.fieldWidth))
                            {
                                continue;
                            }
                            const std::size_t sampledIndex
                                = static_cast<std::size_t>(sampledY) * state.fieldWidth + static_cast<std::size_t>(sampledX);
                            if (state.fieldValid[sampledIndex] == 0u)
                            {
                                continue;
                            }
                            const glm::vec3 sampled = state.scratchSpatial[sampledIndex];
                            mean += sampled;
                            squaredMean += sampled * sampled;
                            neighborhoodMin = glm::min(neighborhoodMin, sampled);
                            neighborhoodMax = glm::max(neighborhoodMax, sampled);
                            count += 1.0f;
                        }
                    }
                    glm::vec3 clamped = history;
                    glm::vec3 boundLow(0.0f);
                    glm::vec3 boundHigh(0.0f);
                    // History-length blend: N clean frames hold the running
                    // mean of N fresh samples (weight N/(N+1), capped at the
                    // configured strength), so a static stochastic field
                    // converges like an accumulated average instead of an
                    // exponentially forgetting one. The change test reads the
                    // NEIGHBORHOOD MEAN against the raw history (see the
                    // historyChangeSigma note above): a whole-neighborhood
                    // move is a changed scene and restarts the history so the
                    // new content re-converges instead of ghosting out over
                    // the length ramp, while a single outlier draw does not
                    // restart anything.
                    const std::uint32_t historyAge = state.historyLength[index];
                    bool changed = false;
                    glm::vec3 sigma(0.0f);
                    if (count > 0.0f)
                    {
                        mean /= count;
                        squaredMean /= count;
                        sigma = glm::sqrt(glm::max(squaredMean - mean * mean, glm::vec3(0.0f)));
                        boundLow  = glm::min(mean - sigma * sigmaScale, neighborhoodMin);
                        boundHigh = glm::max(mean + sigma * sigmaScale, neighborhoodMax);
                        clamped = glm::clamp(history, boundLow, boundHigh);
                        const glm::vec3 standardError = sigma / glm::vec3(glm::sqrt(count));
                        const glm::vec3 changeBound = standardError * historyChangeSigma
                            + glm::vec3(historyChangeFloor);
                        changed = glm::any(glm::greaterThan(glm::abs(mean - history), changeBound));
                    }
                    if (changed)
                    {
                        state.fieldFiltered[index]     = current;
                        state.scratchDisoccluded[index] = 1u;
                        state.historyLength[index]      = 0u;
                        continue;
                    }
                    // The fresh sample joins the running mean clamped into
                    // the neighborhood bound (the same bound the history was
                    // clamped into): a firefly draw contributes its clamped
                    // value instead of dragging the mean, and the variance-
                    // driven bilateral filter below still sees the texel as
                    // high-variance through the spatial pass' estimate.
                    const glm::vec3 blendedSample = count > 0.0f
                        ? glm::clamp(current, boundLow, boundHigh)
                        : current;
                    const float historyWeight = std::min(strength,
                        static_cast<float>(historyAge) / (static_cast<float>(historyAge) + 1.0f));
                    state.fieldFiltered[index] = glm::mix(blendedSample, clamped, historyWeight);
                    state.historyLength[index] = std::min(historyAge + 1u, historyLengthCap);
                }
            }
        });

    // This frame's depth becomes what the next reprojection validates
    // against (a copy: the raster pass reassigns gbufferDepth wholesale next
    // frame, so the history must own its own buffer).
    state.historyDepth = state.gbufferDepth;
    state.historyViewProjection = state.viewProjection;
    state.historyWidth  = state.fieldWidth;
    state.historyHeight = state.fieldHeight;
    state.historyValid = true;
}

// ---- bilateral filter ----------------------------------------------------------------

void rfBilateralFilter(RfPipelineState &state, TaskScheduler *scheduler)
{
    if (!state.settings.bilateralEnabled)
    {
        return;
    }
    const std::size_t pixelCount = static_cast<std::size_t>(state.fieldWidth) * state.fieldHeight;
    if (state.fieldLuminance.size() != pixelCount)
    {
        state.fieldLuminance.resize(pixelCount);
    }

    const std::uint32_t radius = state.settings.bilateralRadius;
    // Variance threshold above which the pass smooths (luminance variance
    // of the spatial-reused field).
    const float varianceThreshold = std::max(state.settings.varianceThreshold, 0.0f);

    // Luminance of the filtered field the bilateral weights read (computed
    // once per texel instead of per neighbor pair).
    parallelRange(scheduler, state.fieldHeight, 1,
        [&](std::size_t rowBegin, std::size_t rowEnd)
        {
            for (std::size_t y = rowBegin; y < rowEnd; ++y)
            {
                for (std::uint32_t x = 0; x < state.fieldWidth; ++x)
                {
                    const std::size_t index = y * state.fieldWidth + x;
                    state.fieldLuminance[index] = state.fieldValid[index] != 0u
                        ? luminance(state.fieldFiltered[index])
                        : 0.0f;
                }
            }
        });

    parallelRange(scheduler, state.fieldHeight, 1,
        [&](std::size_t rowBegin, std::size_t rowEnd)
        {
            for (std::size_t y = rowBegin; y < rowEnd; ++y)
            {
                for (std::uint32_t x = 0; x < state.fieldWidth; ++x)
                {
                    const std::size_t index = y * state.fieldWidth + x;
                    const glm::vec3   center = state.fieldFiltered[index];
                    if (state.fieldValid[index] == 0u)
                    {
                        state.scratchSpatial[index] = center;
                        continue;
                    }
                    const float centerLuminance = state.fieldLuminance[index];
                    const bool disoccluded = state.scratchDisoccluded[index] != 0u;
                    if (!disoccluded && state.scratchVariance[index] < varianceThreshold)
                    {
                        // Low variance: physically based reuse carried it; the
                        // filter only pays where it is needed.
                        state.scratchSpatial[index] = center;
                        continue;
                    }
                    // Disoccluded texels run at double strength: the filter
                    // radius doubles so a freshly revealed area converges
                    // immediately instead of sparkling until the temporal
                    // history rebuilds.
                    const std::uint32_t effectiveRadius = disoccluded ? radius * 2u : radius;
                    glm::vec3 sum(0.0f);
                    float     weightSum = 0.0f;
                    for (std::int32_t offsetY = -static_cast<std::int32_t>(effectiveRadius);
                         offsetY <= static_cast<std::int32_t>(effectiveRadius); ++offsetY)
                    {
                        const std::int32_t sampledY = static_cast<std::int32_t>(y) + offsetY;
                        if (sampledY < 0 || sampledY >= static_cast<std::int32_t>(state.fieldHeight))
                        {
                            continue;
                        }
                        for (std::int32_t offsetX = -static_cast<std::int32_t>(effectiveRadius);
                             offsetX <= static_cast<std::int32_t>(effectiveRadius); ++offsetX)
                        {
                            const std::int32_t sampledX = static_cast<std::int32_t>(x) + offsetX;
                            if (sampledX < 0 || sampledX >= static_cast<std::int32_t>(state.fieldWidth))
                            {
                                continue;
                            }
                            const std::size_t sampledIndex
                                = static_cast<std::size_t>(sampledY) * state.fieldWidth + static_cast<std::size_t>(sampledX);
                            if (state.fieldValid[sampledIndex] == 0u)
                            {
                                continue;
                            }
                            const glm::vec3 sampled = state.fieldFiltered[sampledIndex];
                            const float     weight  = tonemappedWeight(centerLuminance, state.fieldLuminance[sampledIndex]);
                            sum += sampled * weight;
                            weightSum += weight;
                        }
                    }
                    state.scratchSpatial[index] = weightSum > 0.0f ? sum / weightSum : center;
                }
            }
        });
    state.fieldFiltered.swap(state.scratchSpatial);
}

} // namespace lumen_reflection
} // namespace engine
