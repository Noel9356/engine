// Screen-size LOD evaluation for the fur module.
//
// Shell fur's cost is fill-rate: every shell layer blends over every covered
// pixel, so the layer count is the lever that matters (a 20-layer shell
// stack measured ~1.1 ms at Full HD on a mid-range GPU - see
// modules/fur/docs/overview.md for the sources). The evaluator maps a
// surface's angular size (world bounding radius / camera distance, the
// headless backend's mesh-LOD metric) onto a layer budget across five
// bands, and fades the OUTERMOST layer's alpha across the lower part of
// each band so the reduced layer count of the next band dissolves the
// strand tips smoothly instead of clipping them with a visible pop.

#include "fur_internal.hpp"

#include <algorithm>
#include <cmath>

namespace engine
{
namespace fur
{

std::size_t lodBandFor(const LodBands &bands, float angularSize)
{
    for (std::size_t band = 0; band < LodBands::bandCount - 1u; ++band)
    {
        if (angularSize >= bands.thresholds[band])
        {
            return band;
        }
    }
    return LodBands::bandCount - 1u;
}

FurLodDecision evaluateFurLodDecision(const LodBands &bands, float angularSize)
{
    FurLodDecision decision;
    if (!std::isfinite(angularSize) || angularSize <= 0.0f)
    {
        // Degenerate input (culled/invalid distance): the smallest coat.
        decision.layerCount     = std::min(bands.layerCounts[LodBands::bandCount - 1u], bands.maximumLayers);
        decision.outerLayerFade = 1.0f;
        decision.furHeightScale = bands.heightScales[LodBands::bandCount - 1u];
        decision.lodLevel       = static_cast<std::uint32_t>(LodBands::bandCount - 1u);
        return decision;
    }

    const std::size_t band = lodBandFor(bands, angularSize);
    // Band geometry: band i spans [thresholds[i], thresholds[i-1]) upward
    // (its upper edge is only used for the fade-span width), and the last
    // band spans [0, thresholds[3]).
    const float bandLowerEdge = band + 1u < LodBands::bandCount ? bands.thresholds[band] : 0.0f;
    const float bandUpperEdge = band > 0 ? bands.thresholds[band - 1u] : bandLowerEdge + 1.0f;

    // Outer-layer fade: within the last fadeFraction of the band (measured
    // from its lower edge upward), the outermost layer dissolves so the
    // next band's smaller stack takes over without a visible pop.
    float fade = 1.0f;
    if (band + 1u < LodBands::bandCount)
    {
        const float fadeSpan = std::max(bandUpperEdge - bandLowerEdge, 1e-6f) * bands.fadeFraction;
        fade = std::clamp((angularSize - bandLowerEdge) / std::max(fadeSpan, 1e-6f), 0.0f, 1.0f);
    }
    else
    {
        // The last band keeps a minimum presence: the base surface's fur
        // vertex tint carries the look from here, but a single nearly
        // invisible layer avoids the flat "shorn" read at extreme distance.
        const float fadeSpan = std::max(bands.thresholds[LodBands::bandCount - 2u], 1e-3f) * bands.fadeFraction;
        fade = std::clamp(angularSize / std::max(fadeSpan, 1e-6f), 0.25f, 1.0f);
    }

    decision.layerCount = std::min(bands.layerCounts[band], bands.maximumLayers);
    // An outer layer faded to zero would still pay for its draw; fold it
    // away once it can no longer contribute.
    if (fade <= 0.01f && decision.layerCount > 1u)
    {
        decision.layerCount -= 1u;
        decision.outerLayerFade = 1.0f;
    }
    else
    {
        decision.outerLayerFade = fade;
    }
    decision.furHeightScale = bands.heightScales[band];
    decision.lodLevel       = static_cast<std::uint32_t>(band);
    return decision;
}

} // namespace fur
} // namespace engine
