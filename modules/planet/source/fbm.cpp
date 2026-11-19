// Shared fBm machinery implementation (see fbm.hpp).

#include <cstdint>
#include <engine/planet/fbm.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>

#include <engine/planet/noise.hpp>

#include <algorithm>
#include <cmath>

namespace engine
{
namespace planet
{

FbmStreamSeeds makeFbmSeeds(std::int64_t baseSeed, std::uint32_t octaves)
{
    octaves = std::min(octaves, maxFbmOctaves);
    FbmStreamSeeds seeds;
    seeds.count = octaves;
    for (std::uint32_t octave = 0; octave < octaves; ++octave)
    {
        seeds.perOctave[octave] = static_cast<std::int64_t>(
            hashCombine64(static_cast<std::uint64_t>(baseSeed), octave));
    }
    return seeds;
}

double lowPassFade(double cutoffMeters, double termWavelengthMeters)
{
    if (cutoffMeters <= 0.0)
    {
        return 1.0;
    }
    return smoothstep(cutoffMeters, 2.0 * cutoffMeters, termWavelengthMeters);
}

double fbm(const FbmStreamSeeds &seeds, const Vec3 &point, double wavelengthMeters, const FbmParams &fbmParams,
    double lowPassCutoffMeters)
{
    const double frequency = 1.0 / std::max(1.0, wavelengthMeters);
    double       x         = point.x * frequency;
    double       y         = point.y * frequency;
    double       z         = point.z * frequency;
    double       amplitude = 1.0;
    double       sum       = 0.0;
    double       norm      = 0.0;
    double       octaveWavelength = wavelengthMeters;
    for (std::uint32_t octave = 0; octave < seeds.count; ++octave)
    {
        const double fade = lowPassFade(lowPassCutoffMeters, octaveWavelength);
        if (fade <= 0.0)
        {
            break; // octaves shorten monotonically: every later fade is also zero
        }
        // The fade is APPLIED, not just tested: octaves in the transition band
        // (cutoff..2*cutoff wavelengths) must contribute proportionally to
        // their fade, and the normalizer must carry the same weight, or the
        // surviving octaves get renormalized UP whenever the cutoff crosses
        // one of their wavelengths - a discontinuous amplitude jump between
        // the LOD depths either side of the crossing (and the gradient bound
        // the octree probe margins derive from assumes exactly this
        // proportional fade).
        sum += amplitude * fade * noise::simplex3ImproveXY(seeds.perOctave[octave], x, y, z);
        norm += amplitude * fade;
        x *= fbmParams.lacunarity;
        y *= fbmParams.lacunarity;
        z *= fbmParams.lacunarity;
        amplitude *= fbmParams.gain;
        octaveWavelength /= fbmParams.lacunarity;
    }
    return sum / std::max(norm, 1.0e-9);
}

double ridgedFbm(const FbmStreamSeeds &seeds, const Vec3 &point, double wavelengthMeters, const FbmParams &fbmParams,
    double lowPassCutoffMeters)
{
    const double frequency = 1.0 / std::max(1.0, wavelengthMeters);
    double       x         = point.x * frequency;
    double       y         = point.y * frequency;
    double       z         = point.z * frequency;
    double       amplitude = 1.0;
    double       sum       = 0.0;
    double       norm      = 0.0;
    double       octaveWavelength = wavelengthMeters;
    for (std::uint32_t octave = 0; octave < seeds.count; ++octave)
    {
        const double fade = lowPassFade(lowPassCutoffMeters, octaveWavelength);
        if (fade <= 0.0)
        {
            break; // octaves shorten monotonically: every later fade is also zero
        }
        const double n = noise::simplex3ImproveXY(seeds.perOctave[octave], x, y, z);
        sum += amplitude * fade * (1.0 - std::abs(n));
        norm += amplitude * fade;
        x *= fbmParams.lacunarity;
        y *= fbmParams.lacunarity;
        z *= fbmParams.lacunarity;
        amplitude *= fbmParams.gain;
        octaveWavelength /= fbmParams.lacunarity;
    }
    return clamp01(sum / std::max(norm, 1.0e-9));
}

double bandLimitedReliefGradient(const PlanetGenerationParams &params, double lowPassCutoffMeters)
{
    // Mirrors the relief composition of DensityField::reliefImpl: every
    // radial term's metric gradient is bounded by amplitude/wavelength, and
    // each term is weighted by ITS OWN fade at the cutoff (faded terms
    // contribute nothing to the band-limited field, unfaded terms contribute
    // their full gradient - so the bound is the per-term sum, never an
    // average: averaging would both overstate the margin while any term
    // survives and understate it once others fade). The factor covers the
    // simplex gradient constant and the domain warp's position shear (the
    // warp bends sample positions by roughly warp_amplitude * radius per unit
    // of its own field, amplifying the composed gradient).
    const auto term = [&](double amplitudeMeters, double wavelengthMeters)
    { return amplitudeMeters / std::max(1.0, wavelengthMeters)
         * lowPassFade(lowPassCutoffMeters, wavelengthMeters); };
    // The abyssal deepening rides a smoothstep over the abyssal mask window;
    // its metric gradient bound carries the smoothstep's worst slope
    // (1.5 / window width), mirroring the field itself.
    const double abyssalSlopeFactor = 1.5 / std::max(1.0e-3, params.abyssalMaskHigh - params.abyssalMaskLow);
    // The classic mass-and-crest profile carries (1 - erosion weight) of the
    // mountain amplitude; the eroded profile below carries the rest.
    const double fbmMountainShare = 1.0 - params.mountainErosionWeight;
    const double gradient = term(params.continentAmplitudeMeters, params.continentWavelengthMeters)
        + term(params.abyssalDepthMeters * abyssalSlopeFactor, params.continentWavelengthMeters)
        + fbmMountainShare * term(params.mountainAmplitudeMeters, params.mountainWavelengthMeters)
        + fbmMountainShare * term(params.mountainAmplitudeMeters, params.derivedMountainMassWavelengthMeters)
        + term(params.mountainAmplitudeMeters, params.rangeClusterWavelengthMeters)
        + term(params.regionalAmplitudeMeters, params.regionalWavelengthMeters)
        + term(params.hillAmplitudeMeters, params.hillWavelengthMeters)
        + term(params.detailAmplitudeMeters, params.detailWavelengthMeters);

    // Eroded-mountain slope bound, separate from the composed bound above:
    // each gully octave contributes its stripe strength over its own stripe
    // wavelength (the stripe gradient bound is exact: the normalized sine
    // carries the full circle rate) and the massif term carries the terrain
    // amplitude over the massif wavelength. The per-octave sum assumes every
    // octave peaks in alignment AND every mask stage saturates; the filter's
    // faded gullies cannot actually reach that (the combi mask opens with
    // the stripe slope and therefore peaks exactly where the stripe height
    // crosses zero - the anti-correlation bounds a single octave's realized
    // slope well under the raw stripe rate - and the stacked mask product
    // that gates octave i must have EVERY prior stage open with it, which
    // slope scans of the recommended shaping put at a vanishing fraction of
    // the terrain). The factor takes the provable single-octave
    // anti-correlation and leaves the residual stacked-mask tail to the
    // octree's own undecided-refine path (any probe disagreement refines
    // regardless of the margin). The range-cluster term above stays
    // unweighted: the mask gates the eroded profile exactly as it gates the
    // classic one.
    double erosionGradient = 0.0;
    if (params.mountainErosionWeight > 0.0)
    {
        const double metersPerUnit = std::max(1.0, params.mountainWavelengthMeters);
        const double span = std::max(0.01, params.mountainErosionProfileHigh - params.mountainErosionProfileLow);
        double stripeSlope = 0.0;
        double strength = params.mountainErosionStrength * params.mountainErosionScale;
        double stripeWavelength = params.mountainErosionScale * params.mountainErosionCellScale * metersPerUnit;
        for (std::uint32_t octave = 0; octave < params.mountainErosionOctaves; ++octave)
        {
            stripeSlope += strength / std::max(1.0, stripeWavelength)
                * lowPassFade(lowPassCutoffMeters, stripeWavelength);
            strength *= params.mountainErosionGain;
            stripeWavelength /= params.mountainErosionLacunarity;
        }
        const double massifWavelength =
            metersPerUnit / std::max(0.25, params.mountainErosionTerrainFrequency);
        const double massifSlope = params.mountainErosionTerrainAmplitude / massifWavelength
            * lowPassFade(lowPassCutoffMeters, massifWavelength);
        erosionGradient = params.mountainErosionWeight * params.mountainAmplitudeMeters / span
            * (6.283185307179586 * stripeSlope + 2.0 * massifSlope) * 1.25;
    }

    return 3.0 * gradient + erosionGradient;
}

double bandLimitedBoundedTerms(const PlanetGenerationParams &params, double lowPassCutoffMeters)
{
    // Overhang and strata displace the iso-surface by at most twice their
    // amplitude; caves carve at most their depth. Each is faded by the band
    // limit like every other term.
    return 2.0 * params.overhangAmplitudeMeters * lowPassFade(lowPassCutoffMeters, params.overhangWavelengthMeters)
         + 2.0 * params.strataAmplitudeMeters * lowPassFade(lowPassCutoffMeters, params.strataBandMeters)
         + params.caveCarveMeters * lowPassFade(lowPassCutoffMeters, params.caveWormWavelengthMeters);
}

} // namespace planet
} // namespace engine
