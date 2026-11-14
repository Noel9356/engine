// Procedural fur strand texture synthesis for the fur module.
//
// Generates the tiled RGBA8 strand texture the shell-fur pipelines sample:
//   rgb   - strand albedo variation (per-strand tint + per-texel noise)
//   alpha - the per-texel STRAND HEIGHT FIELD: 1 at a strand's root, 0 at
//           its tip. The shell layers crop this field against the layer
//           ratio (fur.frag: alpha = clamp((field - t) * densityBoost)),
//           which is the classic shell-texturing fur of Lengyel et al. 2001.
//
// The pattern is continuous across tile edges (hashes work on wrapped cell
// coordinates), so the Repeat-addressed material texture never shows a seam.
// Everything is integer-hash based and deterministic for a given parameter
// set + seed - the demo's capture-matrix regression relies on byte-identical
// output across runs.

#include "fur_internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace engine
{
namespace fur
{
namespace
{

// 32-bit integer hash (mulberry-style mixing); wraps safely for any input.
std::uint32_t hashUint32(std::uint32_t value)
{
    value += 0x9e3779b9u;
    value ^= value >> 16;
    value *= 0x21f0aaadu;
    value ^= value >> 15;
    value *= 0x735a2d97u;
    value ^= value >> 13;
    return value;
}

std::uint32_t hashCombine(std::uint32_t a, std::uint32_t b, std::uint32_t c)
{
    return hashUint32(hashUint32(hashUint32(a) ^ b) ^ c);
}

// Hash to float in [0, 1).
float hashFloat(std::uint32_t a, std::uint32_t b = 0u, std::uint32_t c = 0u)
{
    return static_cast<float>(hashCombine(a, b, c) >> 8) * (1.0f / 16777216.0f);
}

float smoothStep(float edge0, float edge1, float x)
{
    const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// Tileable 2D value noise: bilinear interpolation of per-grid-node hashes on
// a period-wrapped integer lattice (period matches the texture's tile so the
// noise repeats with the pattern).
float valueNoise(float x, float y, float periodX, float periodY, std::uint32_t seed)
{
    const float xf = std::floor(x);
    const float yf = std::floor(y);
    const float fx = x - xf;
    const float fy = y - yf;
    const auto wrap = [](float v, float period) -> std::uint32_t
    {
        const float wrapped = v - std::floor(v / period) * period;
        return static_cast<std::uint32_t>(wrapped + (wrapped < 0.0f ? period : 0.0f));
    };
    const std::uint32_t x0 = wrap(xf, periodX);
    const std::uint32_t y0 = wrap(yf, periodY);
    const std::uint32_t x1 = wrap(xf + 1.0f, periodX);
    const std::uint32_t y1 = wrap(yf + 1.0f, periodY);
    const float n00 = hashFloat(x0, y0, seed);
    const float n10 = hashFloat(x1, y0, seed);
    const float n01 = hashFloat(x0, y1, seed);
    const float n11 = hashFloat(x1, y1, seed);
    const float n0  = n00 * (1.0f - fx) + n10 * fx;
    const float n1  = n01 * (1.0f - fx) + n11 * fx;
    return n0 * (1.0f - fy) + n1 * fy;
}

// One strand cell's cross-section: a soft-edged vertical strip inside a cell
// of width 1, with the strand's center jittered per cell. Returns 0 outside
// the strand, 1 at its core.
float strandCrossProfile(float fractionX, float centerJitter)
{
    const float center  = 0.5f + (centerJitter - 0.5f) * 0.5f; // 0.25..0.75
    const float half    = 0.34f;
    const float leftEdge  = center - half;
    const float rightEdge = center + half;
    return smoothStep(leftEdge - 0.18f, leftEdge + 0.10f, fractionX)
         * (1.0f - smoothStep(rightEdge - 0.10f, rightEdge + 0.18f, fractionX));
}

} // namespace

void generateFurTexturePixels(const FurParameters &parameters, std::uint32_t width, std::uint32_t height,
    std::uint8_t *outPixelsRgba)
{
    // Strand cells: patternScale vertical strand columns per UV unit across
    // the tile; strands run along +V (tapering toward v = 1). The density
    // scales the column count so denser coats get finer strands.
    const float columnCount = std::clamp(parameters.patternScale, 0.25f, 64.0f) * 8.0f;
    const float noisePeriodX = std::max(columnCount * 0.5f, 2.0f);
    const float noisePeriodY = std::max(columnCount * 0.5f, 2.0f);

    const float density        = std::clamp(parameters.density, 0.0f, 1.0f);
    const float lengthVariance = std::clamp(parameters.lengthVariance, 0.0f, 1.0f);
    const float colorVariation = std::clamp(parameters.colorVariation, 0.0f, 1.0f);
    const std::uint32_t seed   = parameters.seed;

    for (std::uint32_t y = 0; y < height; ++y)
    {
        const float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(height);
        for (std::uint32_t x = 0; x < width; ++x)
        {
            const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(width);

            // Primary guard-hair layer: coarse column cells.
            const float columnU   = u * columnCount;
            const float columnF   = std::floor(columnU);
            const float fractionX = columnU - columnF;
            const std::uint32_t columnId = static_cast<std::uint32_t>(
                columnF - std::floor(columnF / columnCount) * columnCount);

            const float columnHash  = hashFloat(columnId, 17u, seed);
            const float columnHash2 = hashFloat(columnId, 89u, seed);
            const float columnHash3 = hashFloat(columnId, 131u, seed);

            // A column carries a guard hair only when its hash clears the
            // density cut; otherwise the undercoat (below) still contributes.
            const bool hasGuard = columnHash < density;

            // Guard hair length: full-length columns taper to the tile edge;
            // variance shortens individual strands (uniform-length coats set
            // lengthVariance to 0).
            const float guardLength = std::clamp(1.0f - lengthVariance * columnHash2 * 0.85f, 0.2f, 1.0f);

            float field = 0.0f;
            if (hasGuard)
            {
                const float profile = strandCrossProfile(fractionX, columnHash3);
                // Height field: 1 at the root (v = 0), 0 at the strand tip
                // (v = guardLength). The slight overshoot before the tip
                // gives strands their soft, wispy ends after the shell crop.
                const float taper = std::clamp((guardLength - v) / std::max(guardLength, 1e-4f), 0.0f, 1.0f);
                field = std::max(field, profile * taper);
            }

            // Undercoat: half-offset finer columns, shorter and thinner, so
            // the root-side shell layers stay dense even where guard hairs
            // are missing (this is also what fills the silhouette gaps that
            // Lengyel's fins addressed).
            const float underColumnU  = u * columnCount * 2.0f + columnCount;
            const float underColumnF  = std::floor(underColumnU);
            const float underFraction = underColumnU - underColumnF;
            const std::uint32_t underId = static_cast<std::uint32_t>(
                underColumnF - std::floor(underColumnF / (columnCount * 2.0f)) * columnCount * 2.0f);
            const float underHash  = hashFloat(underId, 271u, seed);
            const float underHash2 = hashFloat(underId, 389u, seed);
            if (underHash < density * 0.9f)
            {
                const float underLength = std::clamp(0.45f + 0.3f * underHash2, 0.3f, 0.75f);
                const float profile     = strandCrossProfile(underFraction, underHash2);
                const float taper       = std::clamp((underLength - v) / std::max(underLength, 1e-4f), 0.0f, 1.0f);
                field = std::max(field, profile * taper * 0.65f);
            }

            // Wispiness: fine stretched noise breaks the perfectly straight
            // strand edges (stretched along V - the strand direction).
            const float wisp = valueNoise(u * noisePeriodX, v * noisePeriodY * 0.35f, noisePeriodX, noisePeriodY, seed ^ 0x5bd1e995u);
            field = std::clamp(field + (wisp - 0.5f) * 0.14f, 0.0f, 1.0f);

            // Albedo variation: per-strand tint plus per-texel noise, kept
            // DARKER than 1.0 on average so the stacked shell layers darken
            // and roughen the silhouette instead of washing it out (the
            // shell stack re-blends the strand color over the base surface
            // once per layer; a mean of ~0.85 keeps the authored vertex
            // colors dominant).
            const float strandTint = 0.68f + 0.38f * columnHash2;
            const float texelNoise = valueNoise(u * noisePeriodX * 2.0f, v * noisePeriodY * 2.0f, noisePeriodX * 2.0f, noisePeriodY * 2.0f, seed ^ 0x85ebca6bu);
            const float variation  = 1.0f + (texelNoise - 0.5f) * 2.0f * colorVariation;
            // Tips catch slightly more light than strand roots.
            const float tipLift = 1.0f + 0.05f * v;
            float r = strandTint * variation * tipLift;
            float g = strandTint * (0.96f + 0.08f * columnHash3) * variation * tipLift;
            float b = strandTint * (1.04f - 0.08f * columnHash3) * variation * tipLift;

            const std::size_t pixelIndex = (static_cast<std::size_t>(y) * width + x) * 4;
            outPixelsRgba[pixelIndex + 0] = static_cast<std::uint8_t>(std::clamp(r, 0.0f, 1.0f) * 255.0f + 0.5f);
            outPixelsRgba[pixelIndex + 1] = static_cast<std::uint8_t>(std::clamp(g, 0.0f, 1.0f) * 255.0f + 0.5f);
            outPixelsRgba[pixelIndex + 2] = static_cast<std::uint8_t>(std::clamp(b, 0.0f, 1.0f) * 255.0f + 0.5f);
            outPixelsRgba[pixelIndex + 3] = static_cast<std::uint8_t>(field * 255.0f + 0.5f);
        }
    }
}

} // namespace fur
} // namespace engine
