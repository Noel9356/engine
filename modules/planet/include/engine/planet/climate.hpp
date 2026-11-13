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

#ifndef ENGINE_PLANET_CLIMATE_HPP
#define ENGINE_PLANET_CLIMATE_HPP

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
    ClimateSample sample(const Vec3 &direction, double radiusMeters, double slope01) const;

    // Sea-level temperature/moisture without terrain effects; exposed for
    // debugging and the preview tooling.
    ClimateSample sampleSeaLevel(const Vec3 &direction) const;

private:
    const PlanetGenerationParams &params_;

    std::int64_t seedTemperature_;
    std::int64_t seedMoisture_;
    std::int64_t seedMoistureWarp_;

    // Per-octave noise seeds, derived once per sampler from the base seeds
    // (hashCombine64(baseSeed, octave) is sample-independent). The moisture
    // warp table holds the three single-octave warp stream seeds.
    std::array<std::int64_t, 12> seedTemperatureOctaves_{};
    std::array<std::int64_t, 12> seedMoistureOctaves_{};
    std::array<std::int64_t, 12> seedJitterOctaves_{};
    std::array<std::int64_t, 12> seedMoistureWarpOctaves_{};
};

} // namespace planet
} // namespace engine

#endif // ENGINE_PLANET_CLIMATE_HPP
