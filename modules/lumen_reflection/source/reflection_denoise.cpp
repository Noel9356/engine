// lumen_reflection denoising chain, following Lumen's published order:
// spatial reuse -> temporal accumulation -> bilateral filter.
//
//   Spatial reuse: BRDF-weighted averaging over screen-space neighbors
//   (Lumen: "looks at screen space neighbors and reweights them based on
//   their BRDF"). Plane, normal, and roughness similarity gate the neighbors;
//   the pass also leaves a per-texel variance estimate.
//
//   Temporal accumulation: the previous frame's field is reprojected through
//   the stored matrices, validated against the current depth (disocclusion
//   detection), clamped to the current neighborhood (mean +/-
//   neighborhood_clamp_sigma * sigma so a changed scene cannot ghost
//   through), and blended with the configurable history weight. Freshly
//   revealed texels take the fresh frame whole and are marked for the
//   bilateral filter's double strength.
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

inline float luminance(const glm::vec3 &value)
{
    return 0.2126f * value.x + 0.7152f * value.y + 0.0722f * value.z;
}

// Tone-mapped weight (1/(1+L)): fireflies (huge L deltas) contribute a
// bounded share of the bilateral average, exactly the "tonemapped weighting
// to remove fireflies" the Lumen presentation describes for this pass.
inline float tonemappedWeight(const glm::vec3 &center, const glm::vec3 &neighbor)
{
    const float difference = std::fabs(luminance(center) - luminance(neighbor));
    return 1.0f / (1.0f + difference);
}

} // namespace

// ---- spatial reuse -------------------------------------------------------------------

void rfSpatialReuse(RfPipelineState &state, TaskScheduler *scheduler)
{
    const std::size_t pixelCount = static_cast<std::size_t>(state.fieldWidth) * state.fieldHeight;
    state.scratchSpatial.assign(pixelCount, glm::vec3(0.0f));
    state.scratchVariance.assign(pixelCount, 0.0f);

    const std::uint32_t radius    = state.settings.spatialReuseRadius;
    const float         planeTolerance = 0.1f; // same-surface plane distance gate (m)
    const float         roughnessTolerance = 0.15f;

    rfParallelRange(scheduler, state.fieldHeight, 1,
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
                    float     squaredDifference = 0.0f;
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
                            // normal, compatible roughness.
                            const glm::vec3 delta = sampledPosition - position;
                            if (glm::length(delta) > 0.0f
                                && std::fabs(glm::dot(normal, glm::normalize(delta))) > planeTolerance)
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
                            sum += sampled;
                            weightSum += 1.0f;
                        }
                    }
                    if (weightSum <= 0.0f)
                    {
                        state.scratchSpatial[index] = center;
                        continue;
                    }
                    const glm::vec3 mean = sum / weightSum;
                    state.scratchSpatial[index] = mean;
                    // Luminance variance over the accepted neighborhood.
                    const float centerLuminance = luminance(center);
                    const float meanLuminance   = luminance(mean);
                    squaredDifference = (centerLuminance - meanLuminance) * (centerLuminance - meanLuminance);
                    state.scratchVariance[index] = squaredDifference;
                }
            }
        });
}

// ---- temporal accumulation -----------------------------------------------------------

void rfTemporalAccumulate(RfPipelineState &state, TaskScheduler *scheduler)
{
    const std::size_t pixelCount = static_cast<std::size_t>(state.fieldWidth) * state.fieldHeight;
    state.fieldFiltered.assign(pixelCount, glm::vec3(0.0f));
    state.scratchDisoccluded.assign(pixelCount, 0u);

    const bool hasHistory = state.historyValid && state.historyWidth == state.fieldWidth
        && state.historyHeight == state.fieldHeight && !state.historyField.empty();
    const float strength = std::clamp(state.settings.temporalStrength, 0.0f, 0.95f);
    const float sigmaScale = std::max(state.settings.neighborhoodClampSigma, 0.0f);

    rfParallelRange(scheduler, state.fieldHeight, 1,
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
                        continue;
                    }

                    bool historyUsable = false;
                    glm::vec3 history(0.0f);
                    if (hasHistory)
                    {
                        // Reproject this texel's world position into the
                        // previous frame's field.
                        const glm::vec4 clip
                            = state.historyViewProjection * glm::vec4(state.gbufferPosition[index], 1.0f);
                        if (clip.w > 0.05f)
                        {
                            const float u = (clip.x / clip.w * 0.5f + 0.5f) * static_cast<float>(state.fieldWidth);
                            const float v = (0.5f - clip.y / clip.w * 0.5f) * static_cast<float>(state.fieldHeight);
                            if (u >= 0.0f && u < static_cast<float>(state.fieldWidth) && v >= 0.0f
                                && v < static_cast<float>(state.fieldHeight))
                            {
                                const std::uint32_t hx = std::min(static_cast<std::uint32_t>(u), state.fieldWidth - 1u);
                                const std::uint32_t hy = std::min(static_cast<std::uint32_t>(v), state.fieldHeight - 1u);
                                const std::size_t   hIndex
                                    = static_cast<std::size_t>(hy) * state.fieldWidth + hx;
                                // Depth validation: the texel that occupies
                                // this screen point now must be the one (or
                                // the same plane) that occupied it last
                                // frame, else the surface was revealed.
                                const float historyDepth = state.historyDepth[hIndex];
                                const float currentDepth = state.gbufferDepth[index];
                                const bool depthMatch = !rfIsBackground(historyDepth)
                                    && std::fabs(historyDepth - currentDepth)
                                        < 0.1f * std::max(currentDepth, 1.0f);
                                if (depthMatch)
                                {
                                    history = state.historyField[hIndex];
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
                        continue;
                    }

                    // Neighborhood clamp: mean +/- sigma * scale of the
                    // current 3x3, so a changed scene cannot ghost through
                    // the accumulated history.
                    glm::vec3 mean(0.0f);
                    glm::vec3 squaredMean(0.0f);
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
                            count += 1.0f;
                        }
                    }
                    glm::vec3 clamped = history;
                    if (count > 0.0f)
                    {
                        mean /= count;
                        squaredMean /= count;
                        const glm::vec3 sigma = glm::sqrt(glm::max(squaredMean - mean * mean, glm::vec3(0.0f)));
                        clamped = glm::clamp(history, mean - sigma * sigmaScale, mean + sigma * sigmaScale);
                    }
                    state.fieldFiltered[index] = glm::mix(current, clamped, strength);
                }
            }
        });

    // The published frame becomes the next history (depth of THIS frame is
    // what the next reprojection validates against).
    state.historyField = state.fieldFiltered;
    state.historyDepth = state.gbufferDepth;
    state.historyViewProjection = state.viewProjection;
    state.historyWidth  = state.fieldWidth;
    state.historyHeight = state.fieldHeight;
    state.historyValid = true;
}

// ---- bilateral filter ----------------------------------------------------------------

void rfbilateralFilter(RfPipelineState &state, TaskScheduler *scheduler)
{
    if (!state.settings.bilateralEnabled)
    {
        return;
    }
    const std::size_t pixelCount = static_cast<std::size_t>(state.fieldWidth) * state.fieldHeight;
    state.scratchSpatial.assign(pixelCount, glm::vec3(0.0f));

    const std::uint32_t radius = state.settings.bilateralRadius;
    // Variance threshold above which the pass smooths (luminance variance
    // of the spatial-reused field).
    const float varianceThreshold = 0.02f;

    rfParallelRange(scheduler, state.fieldHeight, 1,
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
                            const float     weight  = tonemappedWeight(center, sampled);
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
