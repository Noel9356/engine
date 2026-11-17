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
        sum += amplitude * noise::simplex3ImproveXY(seeds.perOctave[octave], x, y, z);
        norm += amplitude;
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
        sum += amplitude * (1.0 - std::abs(n));
        norm += amplitude;
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
    const double gradient = term(params.continentAmplitudeMeters, params.continentWavelengthMeters)
        + term(params.abyssalDepthMeters * (1.0 / 0.58), params.continentWavelengthMeters)
        + term(params.mountainAmplitudeMeters, params.mountainWavelengthMeters)
        + term(params.mountainAmplitudeMeters, params.derivedMountainMassWavelengthMeters)
        + term(params.mountainAmplitudeMeters, params.rangeClusterWavelengthMeters)
        + term(params.regionalAmplitudeMeters, params.regionalWavelengthMeters)
        + term(params.hillAmplitudeMeters, params.hillWavelengthMeters)
        + term(params.detailAmplitudeMeters, params.detailWavelengthMeters);
    return 3.0 * gradient;
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
