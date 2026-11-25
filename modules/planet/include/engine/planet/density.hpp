// Terrain density field for the planet module.
//
// DensityField is the geometric heart of the generator: a pure, stateless
// (per-instance) signed field over planet-local space. Positive density is
// solid rock, negative is air/water; the units are meters, so the field value
// at a point is approximately the signed distance to the surface along the
// radial direction.
//
// The field is built from radially-shaped relief (continents, ridged
// mountains, rolling hills, terraces) - which alone would be a 2D heightmap
// wrapped on a sphere - PLUS true 3D terms (overhang displacement, rock
// strata, optional cave worms) that displace the iso-surface in every
// direction. The 3D terms are what produce overhanging cliffs, ledges and
// mouths that no heightfield can represent.
//
// Everything is expressed through OpenSimplex2 streams seeded from the planet
// seed via SplitMix64: sampling a point gives the same bits no matter which
// thread evaluates it or in what chunk order, which is what makes chunk
// generation order-independent.

#pragma once

#include <cstddef>
#include <cstdint>
#include <engine/planet/atmosphere.hpp>
#include <engine/planet/climate.hpp>
#include <engine/planet/erosion.hpp>
#include <engine/planet/fbm.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>

#include <array>

namespace engine
{
namespace planet
{

// Relief sampled along one radial direction, before 3D displacement.
struct ReliefSample
{
    double heightMeters   = 0.0; // terrain height above the datum sphere
    double mountainFactor = 0.0; // mountain mask in [0, 1]
    double cliffFactor    = 0.0; // terrace/cliff mask in [0, 1]
    double detailFade     = 1.0; // submarine fade of the detail fields in [0, 1]
    double coastalTaper   = 1.0; // detail damping near the datum in [0, 1] (coastal flattening)
    double provinceChemistry = 0.0; // rock character in [0, 1]: 0 = shield/volcanic, 1 = sedimentary
};

class DensityField final
{
public:
    explicit DensityField(const PlanetGenerationParams &params);

    // Radial relief (heightfield part) for one unit direction.
    ReliefSample reliefAt(const Vec3 &unitDirection) const;

    // Signed density in meters at a planet-local point: > 0 inside the body.
    // lowPassCutoffMeters > 0 returns a BAND-LIMITED variant of the field:
    // every noise term's contribution is faded by smoothstep over its
    // wavelength (terms at or below the cutoff vanish, terms at twice the
    // cutoff survive untouched). This is the shared field both sides of an
    // LOD transition evaluate: a marching-cubes surface can only follow
    // curvature its own lattice resolves, so the fine isoline and the coarse
    // chords meet without slivers only when the field near their shared
    // boundary carries no content the coarse lattice cannot represent.
    //
    // denseSampling declares that the caller walks a dense sample grid
    // around this point (the mesher's chunk lattices): the eroded-mountain
    // relief then reads from its shared corner lattice instead of the exact
    // filter per point. Both are pure functions of (point, cutoff), so any
    // caller mix reads consistent values; the flag only chooses which cost
    // model serves the query (see MountainErosionField::sample).
    double densityAt(const Vec3 &point, double lowPassCutoffMeters = 0.0, bool denseSampling = false) const;

    // Convenience: FIRST radius where the density crosses zero along the
    // given unit direction (surface radius). The shell band is scanned
    // outward for the first solid-to-air bracket, which the bisection then
    // tightens - overhangs and (optional) caves can create several crossings
    // along one ray, and the first one is what a camera or probe hits.
    // Falls back to the datum when the ray never crosses (should not happen
    // for validated parameters).
    double surfaceRadius(const Vec3 &unitDirection) const;

    const ClimateSampler &climate() const
    {
        return climate_;
    }

    // Cloud shadow factor in [0, 1] for one surface point: how much of the
    // sunlight the cloud layer blocks there. The coverage field is sampled
    // along the SUN RAY through the point (the bank that actually blocks
    // this point's sunlight sits up-sun at the shell altitude), so shadows
    // fall away from their banks the way real cloud shadows do - from orbit
    // the banks and their displaced shadows read as a pair, from the ground
    // the deck up-sun matches the shade on the terrain. Night points (the
    // sun at or below the local horizon) return 0 - no direct sun to block.
    // lowPassCutoffMeters band-limits the coverage like every other field
    // (coarse LOD chunks get smooth shadows instead of speckle). Returns 0
    // when cloud shadows are disabled.
    double cloudShadow(const Vec3 &unitDirection, double lowPassCutoffMeters) const;

    // Terrain self-shadow factor in [0, 1] for one surface point: how much of
    // the sunlight the RELIEF up-sun of the point blocks (an escarpment's
    // lee, the far wall of a ravine - terrain occluding terrain, the one
    // thing slope shading alone cannot express). The march samples the
    // density field along the configured sun ray over
    // terrainSelfShadowReachMeters of travel, band-limited at a fixed
    // fraction of that reach so the march itself is LOD-independent: every
    // chunk depth evaluating the same point agrees, and shared boundary
    // vertices never disagree across chunk depths. Night points (the sun at
    // or below the local horizon) return 0 - no direct sun for the relief to
    // block. The CONSUMER's vertex spacing fades the whole term like the
    // albedo jitter: the march resolves relief at a fixed metric scale
    // (fractions of the reach), and a vertex lattice coarser than that
    // scale samples the shade field uncorrelatedly - per-vertex speckle
    // instead of readable lee shading (a globe-view chunk's vertices sit
    // hundreds of kilometers apart; real orbital photography shows relief
    // shading only at the horizon, never as speckle). Returns 0 when terrain
    // self-shading is disabled. Pure function of (point, parameters);
    // thread safe like every sampler of the field.
    double terrainSelfShadow(const Vec3 &surfacePoint, double vertexSpacingMeters) const;

    // Full single-point climate classification (macro fields evaluated
    // exactly for this point, then classified). The single-point path for
    // capability consumers and tooling; the mesher routes through
    // macroSampleAt + ClimateSampler::sample to share the low-frequency
    // fields between vertices.
    ClimateSample sampleClimate(const Vec3 &unitDirection, double radiusMeters, double slope01,
        double lowPassCutoffMeters = 0.0) const;

    // Low-frequency climate fields at one planet-local surface point
    // (see ClimateMacroSample): sea-level temperature and moisture
    // (with continentality and rain shadow), cloud shadow, geologic-province
    // character. Everything here varies over landscape scales - the mesher
    // samples this on a coarse position-anchored grid and interpolates per
    // vertex, so shared vertices evaluate identical values from any chunk.
    void macroSampleAt(const Vec3 &surfacePoint, double lowPassCutoffMeters, ClimateMacroSample &outMacro) const;

    const CloudField &clouds() const
    {
        return clouds_;
    }

private:
    // Radial relief with an optional low-pass cutoff (the raw path is
    // reliefImpl(direction, 0)); denseSampling forwards to the eroded-
    // mountain field's lattice path (see densityAt).
    ReliefSample reliefImpl(const Vec3 &unitDirection, double lowPassCutoffMeters, bool denseSampling) const;

    // Sample position for the continent-scale fields: the radial position,
    // bent by the domain warp when the warp amplitude is configured. Shared
    // by the relief composition and the macro climate path so both read the
    // same warped geography.
    Vec3 warpedSamplePosition(const Vec3 &unitDirection, double lowPassCutoffMeters) const;

    // Continent base field (signed, positive = land) at an already-warped
    // sample position; shared by the relief composition and the macro
    // climate path (continentality).
    double continentFieldAt(const Vec3 &warpedPosition, double lowPassCutoffMeters) const;

    // Broad mountain-mass envelope in [0, 1] at an already-warped sample
    // position; shared by the relief composition (the mountain profile) and
    // the macro climate path (rain shadow).
    double mountainMassAt(const Vec3 &warpedPosition, double lowPassCutoffMeters) const;

    // Unit tangent pointing DOWNWIND at one direction (prevailing-wind frame
    // for the rain shadow; see the climate_wind_zones_enabled key).
    Vec3 prevailingWindDownwind(const Vec3 &unitDirection) const;

    // The fBm streams (see fbm()/ridgedFbm() in fbm.hpp), indexed for the
    // fbmSeeds_ table below.
    enum FbmStream : std::size_t
    {
        streamWarpX = 0,
        streamWarpY,
        streamWarpZ,
        streamContinents,
        streamRangeCluster,
        streamMountainMass,
        streamMountains,
        streamRegional,
        streamHills,
        streamCliffMask,
        streamDetail,
        streamCaveA,
        streamCaveB,
        streamProvince,
        streamProvinceCarbonate,
        streamCount
    };

    const PlanetGenerationParams &params_;
    ClimateSampler                climate_;
    CloudField                    clouds_;

    // Per-field noise streams (derived once from the planet seed).
    std::int64_t seedContinents_;
    std::int64_t seedWarp_;
    std::int64_t seedMountains_;
    std::int64_t seedHills_;
    std::int64_t seedDetail_;
    std::int64_t seedCliffMask_;
    std::int64_t seedOverhang_;
    std::int64_t seedStrata_;
    std::int64_t seedCaveA_;
    std::int64_t seedCaveB_;
    std::int64_t seedMountainMass_;
    std::int64_t seedRangeCluster_;
    std::int64_t seedRegional_;
    std::int64_t seedErosion_;

    // Eroded-mountain field (see erosion.hpp), seeded above.
    MountainErosionField erosion_;

    // Per-octave seed tables for the fBm streams (derived in the constructor).
    std::array<FbmStreamSeeds, streamCount> fbmSeeds_;

    double terraceStepMeters_ = 1.0;
};

} // namespace planet
} // namespace engine

