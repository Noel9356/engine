// Climate and biome classification for the planet module.
//
// The ClimateSampler turns a surface point (direction, radius, slope) into a
// climate state - temperature and moisture in [0, 1] plus a Biome id and an
// albedo color. Temperature follows latitude with an altitude lapse rate and
// a perturbation noise field; moisture blends a large-scale humidity noise
// with the classic latitude bands (a wet ITCZ belt at the equator, subtropical
// dry belts, a mild wet belt in temperate latitudes). Biomes are classified
// Whittaker-style from the (temperature, moisture) pair, with hard overrides
// for water depth, beaches, bare rock slopes and the snow line so coastlines,
// cliffs and polar caps read correctly regardless of the noise fields.
//
// All knobs live in PlanetGenerationParams; the sampler derives its own noise
// streams from the planet seed, so sampling is a pure function of position and
// safe to call from any thread in any order.

#pragma once

#include <cstdint>
#include <engine/planet/fbm.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>

#include <array>

namespace engine
{
namespace planet
{

class ClimateSampler final
{
public:
    explicit ClimateSampler(const PlanetGenerationParams &params);

    // Classifies one surface point. direction must be unit length (the
    // planet-local radial direction), radiusMeters is the point's distance
    // from the planet center and slope01 is 0 for ground flush with the
    // gravity gradient and 1 for vertical cliff faces (1 - dot(normal, down)).
    // lowPassCutoffMeters optionally band-limits the ALBEDO jitter fields
    // (like every other noise term of a rendered chunk): below the cutoff the
    // metre-scale mottle fades out, so coarse LOD chunks stop aliasing
    // sub-voxel color speckle. The climate fields themselves (temperature,
    // moisture, biome) are always evaluated in full - they are smooth at
    // landscape scale and consumers (capability sampling, the water pass)
    // need their true values.
    ClimateSample sample(const Vec3 &direction, double radiusMeters, double slope01, double lowPassCutoffMeters = 0.0) const;

    // Sea-level temperature/moisture without terrain effects; exposed for
    // debugging and the preview tooling.
    ClimateSample sampleSeaLevel(const Vec3 &direction) const;

    // Sea-level temperature01 only (latitude gradient plus the perturbation
    // noise, no altitude lapse). The water surface's sea-ice edge is keyed to
    // this field, and sampling it per water vertex costs a fraction of the
    // full sea-level sample.
    double seaLevelTemperature(const Vec3 &direction) const;

private:
    const PlanetGenerationParams &params_;

    std::int64_t seedTemperature_;
    std::int64_t seedMoisture_;
    std::int64_t seedMoistureWarp_;
    // Rock-face sediment stripes: shares its tag with the density field's
    // strata stream, so the color stripes and the geometric strata bands
    // correlate (the stripe's phase warp uses the same seed and scale).
    std::int64_t seedRockStripes_;

    // Per-octave noise seeds, derived once per sampler from the base seeds
    // (hashCombine64(baseSeed, octave) is sample-independent). The moisture
    // warp table holds the three single-octave warp stream seeds; the albedo
    // jitter carries one stream at the configured wavelength, one at an
    // eighth of it and one at the ground wavelength (see ClimateSampler::sample).
    FbmStreamSeeds seedTemperatureOctaves_{};
    FbmStreamSeeds seedMoistureOctaves_{};
    FbmStreamSeeds seedJitterOctaves_{};
    FbmStreamSeeds seedDetailJitterOctaves_{};
    FbmStreamSeeds seedGroundJitterOctaves_{};
    std::array<std::int64_t, 3> seedMoistureWarpOctaves_{};
};

} // namespace planet
} // namespace engine

