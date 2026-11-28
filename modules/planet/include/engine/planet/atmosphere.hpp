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

#include <array>

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

// Sun-aware twin: sunDot is the view direction's dot product with the sun
// direction (forward scattering keys the sun-side sky structure) and sunUp
// is the sun's own dot product with the local up (the low-sun band switches
// the warm horizon on). The plain overload is the sunDot = 0 case: every
// directional term carries a positive sun-proximity factor, so a caller
// without a view-relative sun direction gets the historical ramp exactly.
// The whitening toward the sun (the Mie glare wedge, the horizon band
// brightening on the sun's side) rides skySunHaloStrength; the gold horizon
// toward the sun through the low-sun band rides skyWarmHorizonStrength.
Vec3 skyGradientColor(const PlanetGenerationParams &params, double upDot, double sunDot, double sunUp);

// Sky tint keyed to the sun's elevation above the local horizon (sunDotUp,
// the sun direction's dot product with the local up): neutral through the
// day, warm gold through the low-sun band, deep blue at night. Linear RGB
// multiplier for skyGradientColor(). A disabled sun (see PbrEnvironment)
// counts as permanently high noon. This is the SCATTERED-light tint - the
// sky gradient and the aerial haze are the air's own glow, so they carry
// it. Direct sun light (terrain/water/specular lighting, the sun disc)
// takes sunDirectTint() below instead: its color follows transmission, not
// scattering, and the two models deliberately disagree toward the horizon
// (a red sun under a gold sky is the real sunset read).
Vec3 sunSkyTint(const PlanetGenerationParams &params, double sunDotUp, bool sunEnabled);

// Transmittance tint of the DIRECT sun light for one sun elevation
// (sunDotUp, the sun direction's dot product with the local up): a
// Rayleigh-style transmission curve whose airmass grows as the sun
// approaches the horizon, so the sun's own light runs white -> warm ->
// deep red the way real low sun does (blue is scattered out of the ray
// first, red survives the longest path). Feed the result into the sun
// color consumers light terrain and water with and into the sun disc's
// color, scaled by the eye's atmosphereWeight() so a camera above the
// atmosphere sees the unattenuated disc and a ground camera sees the
// full curve (a ground-level surface and the eye above it share
// approximately one airmass in a plane-parallel column). The amplitude
// rides the sun_direct_tint_strength key: 0 returns white at every
// elevation (the configured sun color stands unchanged), 1 gives the
// full physical curve. Linear RGB in [0, 1]; a disabled sun is white.
Vec3 sunDirectTint(const PlanetGenerationParams &params, double sunDotUp, bool sunEnabled);

// Aerial perspective for one sight line: the sun-tinted haze the air adds
// between the eye and a distant point, so terrain fades toward the horizon
// color instead of staying uniformly saturated to the far distance. The
// weight grows with the sight line's length through the air (an exponential
// absorption profile whose scale rides the configured atmosphere height) and
// fades to zero as the eye climbs out of the atmosphere (space views keep
// the limb shell as their depth cue); the color is the horizon band of the
// sky gradient tinted by the same sun tint the consumer applies to the sky,
// so distant ground melts into exactly the color the sky shows behind it.
// Consumers without per-pixel fog can apply it as a multiplicative tint:
// mix(1, color / meanShadedTerrainRadiance, weight) - dark surfaces lift
// toward the haze, bright ones (snow) keep poking through it.
struct AerialPerspective
{
    Vec3    color;
    double  weight;
};

AerialPerspective aerialPerspective(const PlanetGenerationParams &params, const Vec3 &eyePlanetLocal,
    const Vec3 &targetPlanetLocal, const Vec3 &sunTint);

// Star field for the space backdrop: a seed-derived, direction-space
// distribution of point lights (black + stars) a consumer paints behind the
// planet when the atmosphere weight has faded (the demo meshes a star dome
// for its globe and orbit views). The field is a pure function of the
// direction: the unit sphere is quantized onto a 3D lattice whose cell size
// follows stars_density, each cell hashes (seed + cell) into at most one
// star with a jittered position, a power-law brightness (most stars dim, a
// few bright) and a color temperature (red through white through blue), and
// the sample reads a smooth radial falloff around the star. When
// galaxy_band_strength is positive the distribution additionally
// concentrates along a seed-derived great circle - the galactic plane:
// cells near the plane light up several times more often and brighter,
// streaked by a seed-derived dark rift of dust lanes, so the backdrop
// reads as a Milky Way band over the uniform field instead of an
// undifferentiated scatter. Small stars are
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

// Visual weight (0..1) of the galactic band for one sky direction: the
// seed-derived great circle's proximity faded by the seed-derived dust
// rift along its midline - the quantity a consumer paints as the band's
// unresolved glow between the band's own stars (the demo's star dome emits
// dim glow quads keyed to it, so the band reads as a continuous Milky Way
// strip instead of a scatter of points). Pure function of its inputs.
double galaxyBandWeight(const PlanetGenerationParams &params, const Vec3 &unitDirection);

// High cirrus streak field (see the params' cirrus section): the visual
// weight (0..1) of the thin ice-crystal deck at cirrusAltitudeMeters for one
// unit sky direction (planet-local). The streaks run along the latitude
// circles (the zonal wind), stretched by a fixed anisotropy, broken up by a
// fine filament octave and thresholded by the cirrus_coverage_amount
// quantile - a translucent veil of filaments instead of the cumulus
// shell's banks. lowPassCutoffMeters band-limits the field (0 = raw) the
// way CloudField::coverage's does, so a consumer with a coarse sample grid
// keeps the streaks its grid can resolve instead of aliasing the fine
// breakup into speckle. Pure function of its inputs.
double cirrusStreakWeight(const PlanetGenerationParams &params, const Vec3 &unitDirection,
    double lowPassCutoffMeters = 0.0);

// Direction (unit, planet-local) of the moon companion at a given elapsed
// seconds count: a circular orbit around the planet whose plane is tilted
// off the equator by moon_orbit_inclination_radians, swept at
// 2*pi/moon_orbital_period_seconds radians per second. The orbital phase at
// t = 0 is seed-derived and anchored to the configured sunlight direction
// (the same cloud_shadow_sun_direction convention), so a consumer whose sun
// is up sees the moon in that sky too instead of a random placement. Pure
// function of its inputs - no state, safe from any thread. Returns a zero
// vector when the moon is disabled.
Vec3 moonDirection(const PlanetGenerationParams &params, double seconds);

// Aurora ring geometry facts for consumers that mesh the curtain: the
// latitude band the polar oval occupies (radians off the equator, keyed to
// the spin axis) and the altitude band the curtains climb above the datum.
// Real curtains stand vertically from a narrow oval, so the latitude span
// is tight while the altitude span is broad - from space the ring reads as
// a crown of vertical rays around each pole. The glow itself is the
// consumer's bake - the model owns only the oval's shape.
constexpr double auroraOvalLatitudeLowRadians  = 1.127; // ~64.6 deg off the equator
constexpr double auroraOvalLatitudeHighRadians = 1.222; // ~70.0 deg off the equator
constexpr double auroraBaseAltitudeMeters      = 95000.0;  // curtain foot above the datum (real
                                                          // curtains stand 95-250 km up, above the
                                                          // air shell's blue limb)
constexpr double auroraTopAltitudeMeters       = 250000.0; // curtain top above the datum

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
    // normalization scaled onto the cloud shell. softnessScale widens the
    // threshold band (1 = the configured cloud_edge_softness; 2 doubles it):
    // cloud SHADOWS sample with a wider band than the painted banks so the
    // cast shade reads as penumbra instead of a hard copy of the outline.
    // includeBillow gates the cellular billow octave (the cumulus lobes):
    // rendered banks sample it (the lobes are the point), while cloud
    // shadows pass false - a penumbra-widened shadow field cannot resolve
    // lobe-scale detail anyway, and the shadow's field evaluation runs
    // per streaming chunk, where the octave's 27-cell walk would tax every
    // build.
    double coverage(const Vec3 &unitDirection, double lowPassCutoffMeters = 0.0, double softnessScale = 1.0,
        bool includeBillow = true) const;

    // Full-resolution coverage for texture consumers: the same field
    // coverage() paints, evaluated with no band limit plus the detail a
    // texture resolves that no mesh grid can - a pair of wisp octaves whose
    // feature size sits below every shell's vertex spacing, and a set of
    // seed-derived cyclone vortices that swirl the nearby banks into spiral
    // storm systems (the comma heads and spiral arms satellite photography
    // shows over the mid-latitudes; mesh shells band-limit the vortex shear
    // out at their grid spacing, so the two reads stay consistent at the
    // scale each resolves). Pure and thread safe like every sampler here.
    // The default wispDetail 1 keeps the octaves at the configured strength;
    // 0 collapses coverageTexel onto coverage's raw field exactly. The
    // optional lowPassCutoffMeters band-limits the metric texture octaves
    // (wisp pair, convective pair) to the caller's own texel footprint: a
    // bake whose texels sample the shell coarser than the octaves' feature
    // size fades them out instead of aliasing them into salt-and-pepper
    // flecks; 0 (the default) keeps them raw for consumers with fine
    // footprints (the ground deck).
    double coverageTexel(const Vec3 &unitDirection, double wispDetail = 1.0,
        double lowPassCutoffMeters = 0.0) const;

    // The ground-deck twin of coverageTexel: the same texture-resolution
    // detail (wisps, convective octaves) but WITHOUT the cyclone vortex
    // shear. A ground-level eye photographs the cloud shell through true
    // perspective: the horizon band it actually sees spans a few dozen
    // kilometers of shell, and a storm's Rankine shear displaces banks
    // across several of those kilometers - the displacement combs the
    // horizon band into converging fan arcs that read as a mapping
    // artifact from the ground even at the strengths the orbital view
    // wants. Space consumers keep coverageTexel (the comma heads belong
    // to that scale); the sky-deck bake the ground views draw uses this
    // twin, so the two reads each show the sky their own scale of eye
    // actually photographs. Pure and thread safe like every sampler here.
    double coverageTexelGround(const Vec3 &unitDirection) const;

private:
    const PlanetGenerationParams &params_;
    FbmStreamSeeds seedsBase_{};
    FbmStreamSeeds seedsDetail_{};
    FbmStreamSeeds seedsWeather_{};
    FbmStreamSeeds seedsSwirl_{};
    FbmStreamSeeds seedsWisp_[2]{};
    std::int64_t seedBillow_ = 0;
    std::int64_t seedConvective_ = 0;
    std::int64_t seedConvectiveFine_ = 0;
    Vec3 flowAxis_{};
    Vec3 flowSide_{};
    Vec3 spinAxis_{};
    // Cyclone vortices (see coverageTexel): each entry is center (unit),
    // core radius and strength in meters/radians of the shell the shear acts
    // over. Derived once from the seed like every other stream.
    struct Cyclone
    {
        Vec3   center{};
        double coreMeters = 0.0;
        double strength    = 0.0;
    };
    std::array<Cyclone, 8> cyclones_{};

    // Shared body of coverage() and coverageTexel(): assembles the full
    // field value (wind stretch, swirl, cyclone shear, base/billow/detail/
    // wisp octaves) and thresholds it into [0, 1]. texelDetail gates the
    // sub-grid octaves and the vortex shear - coverage() passes false and
    // stays bit-identical to its historical output; includeCycloneShear
    // (the ground twin's gate) drops the whole storm system - the vortex
    // shear and the shield pattern alike - while the texture detail stays.
    double sampleCoverage(const Vec3 &unitDirection, double lowPassCutoffMeters, double softnessScale,
        bool includeBillow, bool texelDetail, double wispDetail,
        bool includeCycloneShear = true) const;
};

} // namespace planet
} // namespace engine

