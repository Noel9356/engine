// Shared fBm machinery for the planet module's noise fields.
//
// Every large-scale field of the generator (relief layers, climate noise,
// albedo jitter) evaluates layered OpenSimplex2 octaves over planet-local
// positions with a metric wavelength, so feature sizes stay physical at any
// planet radius. The helpers here are the ONE implementation of that pattern:
// per-octave seed schedules (FbmStreamSeeds, derived once per field from the
// planet seed), the band-limit fade used by the LOD system, and the plain and
// ridged fBm evaluators. All functions are pure and thread safe.

#ifndef ENGINE_PLANET_FBM_HPP
#define ENGINE_PLANET_FBM_HPP

#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>

#include <array>
#include <cstdint>

namespace engine
{
namespace planet
{

// Per-octave noise seeds for one fBm stream. hashCombine64(baseSeed, octave)
// does not depend on the sample position, so the tables are derived once per
// field instead of once per sample. Octave counts are clamped to
// maxFbmOctaves by parameter validation.
struct FbmStreamSeeds
{
    std::uint32_t                count = 0;
    std::array<std::int64_t, maxFbmOctaves> perOctave{};
};

// Derives the per-octave seed schedule of one fBm stream (count clamped to
// maxFbmOctaves).
FbmStreamSeeds makeFbmSeeds(std::int64_t baseSeed, std::uint32_t octaves);

// Per-octave fade of a band-limited field: a term of the given wavelength
// contributes fully once its wavelength reaches twice the cutoff, and nothing
// at or below the cutoff. cutoff <= 0 disables the fade (the raw field).
double lowPassFade(double cutoffMeters, double termWavelengthMeters);

// fBm over planet-local positions with a metric wavelength; returns [-1, 1].
// `point` carries PHYSICAL positions (meters); octaves shorten by the
// configured lacunarity and their contributions fade out below the optional
// low-pass cutoff (terms monotonically shorten, so evaluation stops at the
// first fully-faded octave).
double fbm(const FbmStreamSeeds &seeds, const Vec3 &point, double wavelengthMeters, const FbmParams &fbmParams,
    double lowPassCutoffMeters = 0.0);

// Ridged fBm in [0, 1]: mountain crests from |noise| inverted per octave.
double ridgedFbm(const FbmStreamSeeds &seeds, const Vec3 &point, double wavelengthMeters, const FbmParams &fbmParams,
    double lowPassCutoffMeters = 0.0);

// Upper bound of the band-limited field's metric gradient used by the octree's
// solid/air corner probes: the amplitude-per-wavelength sum of every relief
// term, each weighted by its fade at `lowPassCutoffMeters` (faded terms
// contribute nothing), times a safety factor covering the domain warp's
// position shear and the simplex gradient constant. The radial -|p| term of
// the density is NOT part of this bound (callers add it separately).
double bandLimitedReliefGradient(const PlanetGenerationParams &params, double lowPassCutoffMeters);

// Sum of the bounded 3D density terms (overhang, strata, cave carve), each
// weighted by its fade at `lowPassCutoffMeters`. Unlike the relief gradient
// these fields are bounded (not gradient-limited), so their worst-case
// contribution over any node is twice the amplitude (or the carve depth).
double bandLimitedBoundedTerms(const PlanetGenerationParams &params, double lowPassCutoffMeters);

} // namespace planet
} // namespace engine

#endif // ENGINE_PLANET_FBM_HPP
