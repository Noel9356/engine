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

// Low-frequency climate fields shared by many neighboring vertices, evaluated
// on the mesher's macro grid (see buildChunkMesh): every field here varies on
// landscape scales (thousands of meters to tens of thousands), so the mesher
// samples it once per grid corner and interpolates per vertex instead of
// re-evaluating the full noise stacks per vertex. The grid anchors to exact
// planet-local positions, so the interpolation is a pure function of position:
// vertices shared between neighbor chunks (and across LOD depths) evaluate
// bit-identical values. Consumers that sample single points (the capability
// path) evaluate the same fields exactly for that point - the interpolated
// values land between the same bounds.
struct ClimateMacroSample
{
    double temperature01   = 0.0; // sea-level temperature (no altitude lapse)
    double moisture01      = 0.0; // sea-level humidity incl. latitude belts, continentality and rain shadow
    double cloudShadow     = 0.0; // cloud shadow factor in [0, 1]
    double provinceChemistry = 0.0; // rock character: 0 = shield/volcanic, 1 = layered sedimentary
    double provinceCarbonate = 0.0; // carbonate (limestone) patch weight in [0, 1]
};

class ClimateSampler final
{
public:
    explicit ClimateSampler(const PlanetGenerationParams &params);

    // Classifies one surface point from a macro sample (see
    // ClimateMacroSample): the macro fields carry temperature, moisture and
    // the low-frequency albedo modulators; this call applies the point's own
    // altitude lapse, slope/beach/snow overrides and the ground-scale albedo
    // fields. direction must be unit length (the planet-local radial
    // direction), radiusMeters is the point's distance from the planet center
    // and slope01 is 0 for ground flush with the gravity gradient and 1 for
    // vertical cliff faces (1 - dot(normal, down)). lowPassCutoffMeters
    // optionally band-limits the ALBEDO jitter fields (like every other noise
    // term of a rendered chunk): below the cutoff the metre-scale mottle fades
    // out, so coarse LOD chunks stop aliasing sub-voxel color speckle.
    // Underwater samples classify and color by depth alone; the macro fields
    // are ignored there (the sea floor has no weather). The macro sample
    // itself comes from DensityField::macroSampleAt, which owns the fields the
    // macro layers read (continent, mountain mass, provinces, clouds).
    ClimateSample sample(const ClimateMacroSample &macro, const Vec3 &direction, double radiusMeters, double slope01,
        double lowPassCutoffMeters = 0.0) const;

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
    // Erosion patches: the sparse ground-scale field that pulls vegetated
    // ground toward the bare-soil tone (see the soil_patch_* keys).
    std::int64_t seedSoilPatches_;
    // Dryness mosaic: the broad ground-scale field that senesces vegetated
    // ground toward its dry, yellow-tan counterpart (see the dry_patch_* keys).
    std::int64_t seedDryPatches_;

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
    FbmStreamSeeds seedSoilPatchOctaves_{};
    FbmStreamSeeds seedDryPatchOctaves_{};
    std::array<std::int64_t, 3> seedMoistureWarpOctaves_{};
};

} // namespace planet
} // namespace engine

