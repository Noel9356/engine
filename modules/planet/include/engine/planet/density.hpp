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

#ifndef ENGINE_PLANET_DENSITY_HPP
#define ENGINE_PLANET_DENSITY_HPP

#include <engine/planet/climate.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>

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
    double densityAt(const Vec3 &point, double lowPassCutoffMeters = 0.0) const;

    // Convenience: first radius where the density crosses zero along the
    // given unit direction (surface radius). Bisection over the shell band;
    // falls back to the datum when the ray never crosses (should not happen
    // for validated parameters).
    double surfaceRadius(const Vec3 &unitDirection) const;

    const ClimateSampler &climate() const
    {
        return climate_;
    }

private:
    // Radial relief with an optional low-pass cutoff (the raw path is
    // reliefImpl(direction, 0)).
    ReliefSample reliefImpl(const Vec3 &unitDirection, double lowPassCutoffMeters) const;

    const PlanetGenerationParams &params_;
    ClimateSampler                climate_;

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

    double terraceStepMeters_ = 1.0;
};

} // namespace planet
} // namespace engine

#endif // ENGINE_PLANET_DENSITY_HPP
