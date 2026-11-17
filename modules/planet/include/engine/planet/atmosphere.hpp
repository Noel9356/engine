// Sky backdrop model for the planet module.
//
// The planet renders against space: from orbit the background is black, while
// a camera inside the atmosphere sees a sky. This header exposes the pure
// pieces both cases need - a zenith-to-horizon gradient keyed to the view
// direction's elevation above the local horizon, an altitude weight that
// fades the whole sky out toward space, and a sun-elevation tint that warms
// the sky toward sunset and darkens it at night. A CloudField completes the
// sky: a band-limited coverage field on a spherical shell above the surface
// (the [planet] cloud_* keys), from which a consumer paints a cloud layer -
// the demo meshes the shell for its globe and orbit views.
// Everything is a pure function of its inputs (no hidden state, safe from any
// thread), so a consumer can paint a sky dome, a clear color, or both per
// frame without any synchronization with the generation pipeline.
//
// All colors are LINEAR RGB in [0, 1]; consumers feed them straight into the
// renderer's vertex colors, which apply the shared tone mapping and sRGB
// encode. The knobs live in PlanetGenerationParams (the [planet] section,
// atmosphere_* and cloud_* keys), so tuning needs no code changes.

#pragma once

#include <engine/planet/fbm.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>

namespace engine
{
namespace planet
{

// How deep the eye sits in the atmosphere: 1 at and below the fade floor
// (a fraction of atmosphereHeightMeters), fading smoothly to 0 at the top of
// the atmosphere. Multiply sky colors by this to hand orbit cameras space.
double atmosphereWeight(const PlanetGenerationParams &params, double eyeAltitudeMeters);

// Sky gradient for one view direction: upDot is the direction's dot product
// with the local up (1 at the zenith, 0 at the horizon, negative below it).
// Above the horizon the color blends from the horizon color to the zenith
// color; below it the horizon color darkens gently so ground-level gaps read
// as haze rather than voids. Linear RGB, unweighted by altitude - combine
// with atmosphereWeight() at the consumer.
Vec3 skyGradientColor(const PlanetGenerationParams &params, double upDot);

// Sky tint keyed to the sun's elevation above the local horizon (sunDotUp,
// the sun direction's dot product with the local up): neutral through the
// day, warm gold through the low-sun band, deep blue at night. Linear RGB
// multiplier for skyGradientColor(). A disabled sun (see PbrEnvironment)
// counts as permanently high noon.
Vec3 sunSkyTint(const PlanetGenerationParams &params, double sunDotUp, bool sunEnabled);

// Star field for the space backdrop: a seed-derived, direction-space
// distribution of point lights (black + stars) a consumer paints behind the
// planet when the atmosphere weight has faded (the demo meshes a star dome
// for its globe and orbit views). The field is a pure function of the
// direction: the unit sphere is quantized onto a 3D lattice whose cell size
// follows stars_density, each cell hashes (seed + cell) into at most one
// star with a jittered position, a power-law brightness (most stars dim, a
// few bright) and a color temperature (red through white through blue), and
// the sample reads a smooth radial falloff around the star. Small stars are
// band-limited away below the consumer's sample spacing like every other
// field of the module (sampleSpacingRadians <= 0 keeps everything), with the
// surviving stars brightened so the field's energy survives a coarse
// sampling. Returns LINEAR RGB in [0, 1].
Vec3 starFieldColor(const PlanetGenerationParams &params, const Vec3 &unitDirection, double sampleSpacingRadians);

// Visitor for one star of the field: direction (unit), linear RGB color in
// [0, 1] (already carrying brightness) and the raw brightness in [0, 1].
// Returning false stops the sweep.
using StarVisitor = bool (*)(const Vec3 &direction, const Vec3 &color, double brightness, void *context);

// Sweeps every star of the field once (the same distribution
// starFieldColor samples): the natural enumeration for consumers that build
// star GEOMETRY - one small disc per star - instead of sampling a painted
// backdrop. The sweep covers the full direction sphere; visitors that only
// need a hemisphere can early-out by returning false (the sweep stops).
void forEachStar(const PlanetGenerationParams &params, StarVisitor visitor, void *context);

// Cloud coverage field: a band-limited noise field sampled on a spherical
// shell at cloudAltitudeMeters above the datum, thresholded into [0, 1]
// coverage (0 = clear sky, 1 = dense overcast). The threshold sits so the
// mean coverage approximates cloudCoverageAmount, cloudDetailStrength mixes
// in a fine breakup octave so banks read as ragged cumulus fields instead of
// smooth blotches, a seed-derived wind frame stretches the banks along a
// deterministic flow direction (real weather systems organize into elongated
// streaks, not round blobs) with a low-frequency swirl bending their shapes,
// and a very-low-frequency weather field shifts the threshold regionally so
// banks organize into large cloudy and clear systems. cloudEdgeSoftness sets
// how wide the threshold band is (crisp clumps vs soft decks). Seeds are
// derived once from the planet seed, like every other stream of the module;
// sampling is a pure function and thread safe. lowPassCutoffMeters
// band-limits the field (0 = raw) so a consumer with a coarse sample spacing
// (a globe-shell vertex grid, a low-resolution sky texture) fades the
// sub-spacing octaves out instead of aliasing them into speckle.
class CloudField final
{
public:
    explicit CloudField(const PlanetGenerationParams &params);

    // Coverage in [0, 1] for one unit direction (planet-local). direction
    // need not be exactly unit length; the sample position is its
    // normalization scaled onto the cloud shell.
    double coverage(const Vec3 &unitDirection, double lowPassCutoffMeters = 0.0) const;

private:
    const PlanetGenerationParams &params_;
    FbmStreamSeeds seedsBase_{};
    FbmStreamSeeds seedsDetail_{};
    FbmStreamSeeds seedsWeather_{};
    FbmStreamSeeds seedsSwirl_{};
    Vec3 flowAxis_{};
    Vec3 flowSide_{};
    Vec3 spinAxis_{};
};

} // namespace planet
} // namespace engine

