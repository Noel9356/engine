// Sky backdrop + cloud coverage model implementation; see atmosphere.hpp.

#include <cstddef>
#include <cstdint>
#include <engine/planet/atmosphere.hpp>
#include <engine/planet/noise.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>

#include <algorithm>

namespace engine
{
namespace planet
{
namespace
{

// Fixed shape of the cloud field: three smooth octaves carry the banks, a
// two-octave detail stream breaks them up (amplitude set by
// cloudDetailStrength), an inverted-Worley billow octave packs the banks'
// rounded cumulus lobes (smooth fBm alone reads as soft cotton; the
// cellular octave contributes the cauliflower clumping real cumulus shows -
// the Perlin-Worley pairing volumetric-cloud pipelines use) and a
// very-low-frequency weather field shifts the coverage threshold
// regionally, so banks organize into large cloudy and clear systems instead
// of uniform popcorn. The base banks are sampled in a wind-stretched frame:
// a deterministic flow direction (derived from the seed) compresses the base
// field along the flow, elongating the banks into the organized streaks real
// weather systems follow - unwarped, the banks read as round popcorn blobs.
// A two-octave swirl bends the bank shapes so their edges stay organic. The
// profiles stay constants of the model; coverage, scale, edge softness, the
// billow octave's amplitude (cloud_billow_strength) and the weather strength
// are configured.
constexpr FbmParams kCloudBaseFbm{3, 2.0, 0.55};
constexpr FbmParams kCloudDetailFbm{2, 2.0, 0.5};
constexpr FbmParams kCloudWeatherFbm{2, 2.0, 0.5};
constexpr FbmParams kCloudSwirlFbm{2, 2.0, 0.5};

// Wisp octaves for texture consumers (see coverageTexel): the wavelengths
// ride at a 24th and a 40th of the bank scale, under every practical mesh
// grid's vertex spacing but ABOVE the bake's own texel footprint - a
// 2048-wide equirect over the 60 km showcase puts ~187 m per texel at the
// equator, so both octaves resolve (a sub-texel octave would alias into
// uncorrelated salt-and-pepper instead of ragged fringes). The pair's
// amplitude halves for the finer octave, carrying the hierarchy real
// cloud decks photograph (large torn sheets, finer ragged fringes on
// their edges).
constexpr FbmParams kCloudWispFbm{2, 2.0, 0.5};
constexpr double kCloudWispWavelengthFractionA = 1.0 / 24.0;
constexpr double kCloudWispWavelengthFractionB = 1.0 / 40.0;

// Amplitude of the bank-elongation (1.0 = features stretch to about twice
// their width along the flow) and of the swirl bend, in units of the cloud
// wavelength. The billow octave's wavelength rides at 0.7x the bank scale -
// the lobes sit one size class under the banks while keeping the rendered
// shells' own grids at six-plus samples per lobe wavelength (the contour
// walk needs that headroom to keep lobe outlines off the grid stairs) - with
// the amplitude below as its share of the field.
constexpr double kCloudStretchFactor = 0.50;
constexpr double kCloudSwirlAmplitude = 0.20;
constexpr double kCloudBillowWavelengthFraction = 0.70;

// Fixed shape of the cirrus deck: three smooth octaves carry the streak
// field at half the cumulus wavelength, a two-octave filament stream tears
// the thresholded streaks into bundles, and the zonal stretch compresses the
// field across latitude by the factor below (1.7 stretches the streaks to
// roughly 2.7 times their width along the latitude circles - cirrus decks
// photograph with directional coherence, but an over-stretched field reads
// as motion-blurred scratches instead of wind-sheared ice). The threshold
// band stays wider than the cumulus deck's so each filament keeps a soft
// feathered fringe; hard-edged white scratches are the single most
// synthetic-looking failure a cirrus pass can show.
constexpr FbmParams kCirrusBaseFbm{3, 2.0, 0.55};
constexpr FbmParams kCirrusFilamentFbm{2, 2.0, 0.5};
constexpr double kCirrusStretchFactor = 1.7;
constexpr double kCirrusWavelengthFraction = 0.6;
constexpr double kCirrusFilamentWavelengthFraction = 0.16;
constexpr double kCirrusFilamentStrength = 0.32;
constexpr double kCirrusEdgeHalfWidth = 0.15;
constexpr std::uint64_t kCirrusSeedTag = 0x5177ULL;
constexpr std::uint64_t kCirrusFilamentSeedTag = 0x5178ULL;

constexpr std::uint64_t kCloudSeedTag    = 0xC10D1ULL;
constexpr std::uint64_t kCloudDetailSeedTag  = 0xC10D2ULL;
constexpr std::uint64_t kCloudWeatherSeedTag = 0xC10D3ULL;
constexpr std::uint64_t kCloudSwirlSeedTag   = 0xC10D4ULL;
constexpr std::uint64_t kCloudBillowSeedTag  = 0xC10D5ULL;
constexpr std::uint64_t kCloudWispSeedTagA   = 0xC10D6ULL;
constexpr std::uint64_t kCloudWispSeedTagB   = 0xC10D7ULL;

// Star field constants: the lattice spans the unit-cube projection of the
// direction sphere (cellsPerAxis across [-1, 1]), the presence probability
// tunes how many cells light up, and the falloff radius is a fraction of the
// cell's angular size (a star never reaches past its own cell, so one
// lookup answers a sample - no neighbor walks).
constexpr std::uint64_t kStarSeedTag = 0x57A25ULL;
constexpr double kStarCellScaleLow  = 14.0;  // cells per axis at stars_density = 0
constexpr double kStarCellScaleHigh = 44.0;  // cells per axis at stars_density = 1
constexpr double kStarPresence = 0.16;       // fraction of cells holding a star
constexpr double kStarFalloffCells = 0.34;   // star radius in cell units

// Galactic band shape constants (see the galaxy_band_* keys): how much more
// often a band cell lights up (multiplier on the presence probability), how
// much brighter the band stars run, and the dust rift's depth along the dark
// lane that snakes through the plane's midline. The rift noise wavelength
// (~0.28 rad of sky) keeps the lane a long, continuous feature the way the
// real Great Rift reads; a shorter wavelength would shatter it into speckle.
constexpr std::uint64_t kGalaxySeedTag = 0x6A1AB5ULL;
constexpr std::uint64_t kGalaxyDustSeedTag = 0xD057ULL;
// The band needs to READ over the uniform field at capture resolutions,
// where the power-law tail leaves most stars below the display floor: the
// presence boost saturates the band cells (nearly every in-plane cell
// holds a star - the unresolved-star glow of the real band), and the
// brightness boost lifts the band's stars over that floor by a factor of
// nearly three (with the disc radius growing as sqrt(brightness), the
// band's stars also draw visibly larger).
constexpr double kGalaxyPresenceBoost = 6.5;   // band cells hold stars this much more often
constexpr double kGalaxyBrightnessBoost = 1.8; // band stars run this much brighter
constexpr double kGalaxyDustDepth = 0.80;       // rift presence suppression at the lane
constexpr double kGalaxyDustWavelengthRadians = 0.28;

// Moon model constants (see moonDirection): the orbital phase offset from
// the sub-sun meridian at t = 0. The lead sits near quadrature: an anchor
// that rides the sun's own meridian (or within a few tenths of a radian of
// it) photographs a near-NEW companion from the planet - the lit-sphere
// shading then draws its unlit face as a hard black disc against the
// daylight sky, which reads as a floating artifact where a real new moon
// is simply invisible. At quadrature the visible face is roughly half lit,
// the terminator runs down the disc's middle, and the companion reads as
// the moon every daytime photograph shows.
constexpr std::uint64_t kMoonSeedTag = 0x300BULL;
constexpr double kMoonLeadRadians = 1.57;

// Unit normal of the galactic plane (a seed-derived great circle): hashed
// from the planet seed exactly the way the cloud field's wind frame is, so
// every consumer of the star field agrees on one plane. Pure function.
Vec3 galaxyPlaneNormal(const PlanetGenerationParams &params)
{
    const std::uint64_t hash = hashCombine64(params.seed, kGalaxySeedTag);
    const double longitude = static_cast<double>(hash & 0xFFFFull) / 65536.0 * 6.283185307179586;
    const double latitude  = (static_cast<double>((hash >> 16) & 0x7FFFull) / 32768.0 - 0.5) * 1.5707963267948966;
    return normalized(Vec3{std::cos(latitude) * std::cos(longitude), std::sin(latitude),
        std::cos(latitude) * std::sin(longitude)});
}

} // namespace

CloudField::CloudField(const PlanetGenerationParams &params)
    : params_(params)
{
    const std::int64_t baseSeed = static_cast<std::int64_t>(hashCombine64(params.seed, kCloudSeedTag));
    seedsBase_    = makeFbmSeeds(baseSeed, kCloudBaseFbm.octaves);
    seedsDetail_  = makeFbmSeeds(static_cast<std::int64_t>(hashCombine64(static_cast<std::uint64_t>(baseSeed), kCloudDetailSeedTag)),
        kCloudDetailFbm.octaves);
    seedsWeather_ = makeFbmSeeds(static_cast<std::int64_t>(hashCombine64(static_cast<std::uint64_t>(baseSeed), kCloudWeatherSeedTag)),
        kCloudWeatherFbm.octaves);
    seedsSwirl_   = makeFbmSeeds(static_cast<std::int64_t>(hashCombine64(static_cast<std::uint64_t>(baseSeed), kCloudSwirlSeedTag)),
        kCloudSwirlFbm.octaves);
    seedsWisp_[0] = makeFbmSeeds(static_cast<std::int64_t>(hashCombine64(static_cast<std::uint64_t>(baseSeed), kCloudWispSeedTagA)),
        kCloudWispFbm.octaves);
    seedsWisp_[1] = makeFbmSeeds(static_cast<std::int64_t>(hashCombine64(static_cast<std::uint64_t>(baseSeed), kCloudWispSeedTagB)),
        kCloudWispFbm.octaves);
    seedBillow_  = static_cast<std::int64_t>(hashCombine64(static_cast<std::uint64_t>(baseSeed), kCloudBillowSeedTag));

    // Deterministic wind frame: a unit flow direction derived from the cloud
    // seed. The base field is sampled in a frame whose axis is compressed
    // along the flow, elongating banks along it; the two transverse axes stay
    // unscaled. A pure seed function - no state, order independent.
    const std::uint64_t flowHash = hashCombine64(static_cast<std::uint64_t>(baseSeed), 0xF10Full);
    const double longitude = static_cast<double>(flowHash & 0xFFFFull) / 65536.0 * 6.283185307179586;
    const double latitude  = (static_cast<double>((flowHash >> 16) & 0x7FFFull) / 32768.0 - 0.5) * 1.5707963267948966;
    flowAxis_ = normalized(Vec3{std::cos(latitude) * std::cos(longitude), std::sin(latitude),
        std::cos(latitude) * std::sin(longitude)});
    spinAxis_ = normalized(Vec3{params.axis[0], params.axis[1], params.axis[2]});
    // Transverse frame for the compressed axis.
    Vec3 tangent = cross(flowAxis_, Vec3{0.0, 1.0, 0.0});
    if (lengthSquared(tangent) < 1.0e-6)
    {
        tangent = cross(flowAxis_, Vec3{1.0, 0.0, 0.0});
    }
    flowSide_ = normalized(tangent);

    // Cyclone vortices (see coverageTexel): eight seed-derived storms over
    // the mid-latitudes, where the general circulation actually spawns them
    // (a cyclone riding the equatorial belt or the polar cap reads as a
    // swirl placed by a random brush). The core radius spans a 25th to a
    // 18th of the circumference - several bank wavelengths across, the
    // comma-head scale orbital photography resolves - and the shear
    // strength several core radii of displacement: the shear must dominate
    // the bank wavelength so the arms wrap fully (a displacement under a
    // wavelength only dents the banks), while the Rankine envelope keeps
    // the far field untouched.
    const double shellCircumference = 6.283185307179586
        * (params.radiusMeters + params.cloudAltitudeMeters);
    for (std::size_t storm = 0; storm < cyclones_.size(); ++storm)
    {
        const std::uint64_t stormHash = hashCombine64(static_cast<std::uint64_t>(baseSeed),
            0xC7C10E5ull + static_cast<std::uint64_t>(storm) * 0x9E3779B97F4A7C15ull);
        const double stormLongitude = static_cast<double>(stormHash & 0xFFFFull) / 65536.0 * 6.283185307179586;
        const double stormLatitude  = (static_cast<double>((stormHash >> 16) & 0x7FFFull) / 32768.0 - 0.5) * 1.2;
        cyclones_[storm].center = normalized(Vec3{
            std::cos(stormLatitude) * std::cos(stormLongitude), std::sin(stormLatitude),
            std::cos(stormLatitude) * std::sin(stormLongitude)});
        cyclones_[storm].coreMeters = shellCircumference
            * (0.040 + 0.014 * static_cast<double>((stormHash >> 28) & 0xFull) / 15.0);
        cyclones_[storm].strength = params_.cloudCycloneStrength
            * (3.4 + 2.6 * static_cast<double>((stormHash >> 32) & 0xFull) / 15.0);
    }
}

double CloudField::coverage(const Vec3 &unitDirection, double lowPassCutoffMeters, double softnessScale,
    bool includeBillow) const
{
    return sampleCoverage(unitDirection, lowPassCutoffMeters, softnessScale, includeBillow, false, 0.0);
}

double CloudField::coverageTexel(const Vec3 &unitDirection, double wispDetail) const
{
    return sampleCoverage(unitDirection, 0.0, 1.0, true, true, wispDetail);
}

double CloudField::sampleCoverage(const Vec3 &unitDirection, double lowPassCutoffMeters, double softnessScale,
    bool includeBillow, bool texelDetail, double wispDetail) const
{
    if (!params_.cloudsEnabled || params_.cloudCoverageAmount <= 0.0)
    {
        return 0.0;
    }
    const Vec3 direction = normalized(unitDirection);
    if (lengthSquared(direction) < 0.5)
    {
        return 0.0; // a (near-)zero direction has no sky position to sample
    }
    Vec3 shellPosition = direction * (params_.radiusMeters + params_.cloudAltitudeMeters);

    // Wind stretch: sample the base banks in a frame compressed along the
    // flow direction, elongating the banks into organized streaks. The
    // sample position decomposes against the flow frame and recombines with
    // the flow axis scaled up (a compressed sample axis = stretched
    // features).
    {
        const double alongFlow   = dot(shellPosition, flowAxis_);
        const double sideFlow    = dot(shellPosition, flowSide_);
        const Vec3   normalAxis   = cross(flowAxis_, flowSide_);
        const double normalFlow   = dot(shellPosition, normalAxis);
        shellPosition = flowAxis_ * (alongFlow / (1.0 - kCloudStretchFactor))
            + flowSide_ * sideFlow + normalAxis * normalFlow;
    }

    // Swirl: a low-frequency two-stream bend of the sample position, so the
    // stretched banks curve instead of running as straight ribbons.
    const double swirl = kCloudSwirlAmplitude * params_.cloudWavelengthMeters;
    shellPosition = shellPosition
        + Vec3{fbm(seedsSwirl_, direction * params_.radiusMeters, params_.cloudWavelengthMeters * 2.0,
                kCloudSwirlFbm, lowPassCutoffMeters),
            fbm(seedsSwirl_, direction * (params_.radiusMeters + 4096.0), params_.cloudWavelengthMeters * 2.0,
                kCloudSwirlFbm, lowPassCutoffMeters),
            fbm(seedsSwirl_, direction * (params_.radiusMeters + 8192.0), params_.cloudWavelengthMeters * 2.0,
                kCloudSwirlFbm, lowPassCutoffMeters)}
            * swirl;

    // Cyclone shear (texture consumers only): each storm's Rankine-like
    // vortex displaces the sample position azimuthally around the storm's
    // axis - speed rising linearly through the core, falling like 1/r
    // outside it - which shears whatever banks the field already owns into
    // trailing spiral arms, exactly the way a real cyclone rearranges the
    // cloud deck it forms in. A slight inward radial drift through the
    // influence tightens the wrap toward the eyewall without dragging the
    // far-field banks around. The displacement stays a pure function of the
    // position, so the texture bake remains deterministic.
    if (texelDetail)
    {
        for (const Cyclone &storm : cyclones_)
        {
            const Vec3 offset = shellPosition - storm.center
                * (params_.radiusMeters + params_.cloudAltitudeMeters);
            const double radial = length(offset);
            if (radial < 1.0e-6 || radial > 4.0 * storm.coreMeters)
            {
                continue;
            }
            // Tangential (azimuthal) direction around the storm axis.
            Vec3 tangential = cross(storm.center, offset);
            const double tangentialLength = length(tangential);
            if (tangentialLength < 1.0e-9)
            {
                continue;
            }
            tangential = tangential * (1.0 / tangentialLength);
            // Rankine envelope: speed ~ r/core inside the eyewall, ~ core/r
            // outside it, tapered to zero at the influence's rim.
            const double speed = (radial < storm.coreMeters
                    ? radial / storm.coreMeters
                    : storm.coreMeters / radial)
                * (1.0 - smoothstep(3.0 * storm.coreMeters, 4.0 * storm.coreMeters, radial));
            const Vec3 inward = offset * (1.0 / radial);
            shellPosition = shellPosition
                + tangential * (storm.strength * storm.coreMeters * speed)
                - inward * (0.35 * storm.strength * storm.coreMeters * speed);
        }
    }

    // Base banks plus the breakup octave; both fade under the consumer's
    // band limit like every other noise term of the module. The billow
    // octave rides at 0.7x the bank scale: inverted cellular distance (one
    // at the lobe centers, zero at the cell walls) packs the rounded
    // cumulus lobes into the bank bodies. Cloud shadows pass
    // includeBillow = false - the penumbra widening erases lobe-scale
    // shadow detail, and the shadow field's evaluation runs per streaming
    // chunk, where the octave's 27-cell neighbor walk would tax every build.
    double field = fbm(seedsBase_, shellPosition, params_.cloudWavelengthMeters, kCloudBaseFbm, lowPassCutoffMeters);
    const double billowFade = includeBillow
        ? params_.cloudBillowStrength
            * lowPassFade(lowPassCutoffMeters, params_.cloudWavelengthMeters * kCloudBillowWavelengthFraction)
        : 0.0;
    if (billowFade > 0.0)
    {
        const double billowWavelength = params_.cloudWavelengthMeters * kCloudBillowWavelengthFraction;
        field += billowFade * (1.0 - 2.0 * noise::worley3(seedBillow_,
            shellPosition.x / billowWavelength, shellPosition.y / billowWavelength, shellPosition.z / billowWavelength));
    }
    const double detailFade = params_.cloudDetailStrength * lowPassFade(lowPassCutoffMeters, params_.cloudWavelengthMeters * 0.25);
    if (detailFade > 0.0)
    {
        field += detailFade
            * fbm(seedsDetail_, shellPosition, params_.cloudWavelengthMeters * 0.25, kCloudDetailFbm, lowPassCutoffMeters);
    }

    // Wisp octaves (texture consumers only): two octaves whose feature size
    // sits under every mesh grid's vertex spacing, so only a texture's texel
    // lattice resolves them. They carry the torn fringes and ragged holes
    // real decks photograph at close range - the structure a contoured mesh
    // cannot hold at any practical resolution. The amplitude follows the
    // configured detail strength so cloud_detail_strength keeps tuning the
    // whole breakup family together.
    if (texelDetail && wispDetail > 0.0)
    {
        const double wispStrength = wispDetail * params_.cloudWispStrength
            * std::max(params_.cloudDetailStrength, 0.12);
        field += wispStrength
            * fbm(seedsWisp_[0], shellPosition, params_.cloudWavelengthMeters * kCloudWispWavelengthFractionA,
                kCloudWispFbm, 0.0);
        field += 0.5 * wispStrength
            * fbm(seedsWisp_[1], shellPosition, params_.cloudWavelengthMeters * kCloudWispWavelengthFractionB,
                kCloudWispFbm, 0.0);
    }

    // Threshold into coverage: the crossing band centers on the quantile the
    // configured coverage implies (fbm output is roughly symmetric around 0),
    // so raising cloud_coverage_amount grows the banks instead of moving the
    // whole field. A very-low-frequency weather field shifts the threshold
    // regionally (positive weather = cloudier), organizing the banks into
    // large cloudy and clear systems. The band half-width is the configured
    // edge softness - narrow values scatter crisp clumps, wide ones grow
    // soft decks.
    double crossing = 1.0 - 2.0 * params_.cloudCoverageAmount;
    // Latitude bands: a wet belt at the equator, dry subtropics, stormy
    // mid-latitudes - the circulation-cell pattern every rotating planet
    // with an atmosphere shows. The band weight follows the spin axis, so
    // the belts wrap the planet the way the climate's latitude bands do; the
    // band frequency is the configured cloud_band_frequency key.
    const double latitude = std::asin(std::clamp(dot(direction, spinAxis_), -1.0, 1.0));
    crossing -= params_.cloudLatitudeBandStrength
        * std::cos(latitude * params_.cloudBandFrequency) * lowPassFade(lowPassCutoffMeters,
            params_.cloudWavelengthMeters * 2.0);
    const double weatherFade = params_.cloudWeatherStrength * lowPassFade(lowPassCutoffMeters,
        params_.cloudWavelengthMeters * params_.cloudWeatherWavelengthMultiple);
    if (weatherFade > 0.0)
    {
        crossing -= weatherFade * fbm(seedsWeather_, shellPosition,
            params_.cloudWavelengthMeters * params_.cloudWeatherWavelengthMultiple, kCloudWeatherFbm, lowPassCutoffMeters);
    }
    const double edgeHalfWidth = std::max(0.01, params_.cloudEdgeSoftness * std::max(1.0, softnessScale));
    return smoothstep(crossing - edgeHalfWidth, crossing + edgeHalfWidth, field);
}

double atmosphereWeight(const PlanetGenerationParams &params, double eyeAltitudeMeters)
{
    // The sky owns the air below the fade floor and hands over to space at
    // the configured height; the floor keeps ground-level cameras fully
    // inside the atmosphere regardless of how tall the atmosphere is set.
    const double height = std::max(1.0, params.atmosphereHeightMeters);
    const double floor = height * 0.15;
    return 1.0 - smoothstep(floor, height, eyeAltitudeMeters);
}

AerialPerspective aerialPerspective(const PlanetGenerationParams &params, const Vec3 &eyePlanetLocal,
    const Vec3 &targetPlanetLocal, const Vec3 &sunTint)
{
    AerialPerspective result{Vec3{0.0, 0.0, 0.0}, 0.0};
    const double eyeAltitude = length(eyePlanetLocal) - params.radiusMeters;
    const double airWeight = atmosphereWeight(params, eyeAltitude);
    if (airWeight <= 0.0)
    {
        return result; // the eye is in space: the limb shell owns the depth cue
    }
    // Absorption profile: the haze saturates exponentially along the sight
    // line, with a configured scale length in meters (aerial_perspective_
    // scale_meters) and an asymptote that keeps even the farthest terrain
    // faintly readable as terrain instead of sky. A larger scale keeps the
    // midground clear; a lower asymptote hands less of the far distance to
    // the haze color.
    const double asymptote = std::clamp(params.aerialPerspectiveAsymptote, 0.0, 1.0);
    const double pathMeters = length(targetPlanetLocal - eyePlanetLocal);
    const double scale = std::max(1.0, params.aerialPerspectiveScaleMeters);
    result.weight = asymptote * (1.0 - std::exp(-pathMeters / scale)) * airWeight;
    // The haze color is the horizon band of the sky gradient under the same
    // sun tint the consumer paints the sky with, so distant ground fades
    // into the exact color the sky shows behind it.
    const Vec3 horizon = skyGradientColor(params, 0.04);
    result.color = Vec3{horizon.x * sunTint.x, horizon.y * sunTint.y, horizon.z * sunTint.z};
    return result;
}

Vec3 skyGradientColor(const PlanetGenerationParams &params, double upDot)
{
    // Pale band along the horizon, deepening toward the zenith: real skies
    // keep their horizon haze over roughly the first quarter of the climb,
    // which the smoothstep edges encode (the band saturates by ~20 degrees
    // of elevation, where a standing eye reads the transition). Below the
    // horizon the haze darkens gently instead of cutting off, so any
    // ground-level gap (a chunk still streaming in, the sliver between a
    // sky dome's rim and the visual horizon) photographs as atmosphere, not
    // as a hole to space.
    const double climb = smoothstep(0.01, 0.36, upDot);
    const double channels[3] = {
        static_cast<double>(params.atmosphereHorizonColor[0]),
        static_cast<double>(params.atmosphereHorizonColor[1]),
        static_cast<double>(params.atmosphereHorizonColor[2])};
    const double zeniths[3] = {
        static_cast<double>(params.atmosphereZenithColor[0]),
        static_cast<double>(params.atmosphereZenithColor[1]),
        static_cast<double>(params.atmosphereZenithColor[2])};
    Vec3 color{};
    double *out[3] = {&color.x, &color.y, &color.z};
    for (std::size_t channel = 0; channel < 3; ++channel)
    {
        double value = channels[channel] + (zeniths[channel] - channels[channel]) * climb;
        if (upDot < 0.0)
        {
            value *= 1.0 - 0.3 * std::min(1.0, -upDot);
        }
        *out[channel] = std::clamp(value, 0.0, 1.0);
    }
    return color;
}

namespace
{

// Star of one lattice cell (see starFieldColor): presence, jittered unit
// direction, power-law brightness (raw, pre-band-limit) and temperature
// color. Shared by the field sampler and the star enumerator so the two
// always describe the same sky.
struct StarCell
{
    bool        present = false;
    Vec3        direction{};
    double      brightness = 0.0; // raw, pre-band-limit; can exceed 1 for the brightest stars
    Vec3        color{};          // temperature color (unit-scale)
};

StarCell starOfCell(const PlanetGenerationParams &params, double cellsPerAxis, std::int64_t cellX,
    std::int64_t cellY, std::int64_t cellZ)
{
    StarCell star;
    const std::int64_t cell[3] = {cellX, cellY, cellZ};
    std::uint64_t mixed = static_cast<std::uint64_t>(cell[0]) * 0x9E3779B97F4A7C15ull
                        ^ static_cast<std::uint64_t>(cell[1]) * 0xC2B2AE3D27D4EB4Full
                        ^ static_cast<std::uint64_t>(cell[2]) * 0x165667B19E3779F9ull
                        ^ params.seed;
    mixed = splitMix64(mixed);
    std::uint64_t starA = splitMix64(mixed);
    std::uint64_t starB = splitMix64(starA);
    std::uint64_t starC = splitMix64(starB);
    const auto unit01 = [](std::uint64_t value)
    { return static_cast<double>(value >> 11) / 9007199254740992.0; };

    // Star position: jittered toward the cell interior so the falloff never
    // crosses the cell boundary (the margin is the falloff radius in cell
    // units), then normalized back onto the sphere.
    const double margin = kStarFalloffCells + 0.08;
    const double starLattice[3] = {
        static_cast<double>(cell[0]) + margin + (1.0 - 2.0 * margin) * unit01(starA),
        static_cast<double>(cell[1]) + margin + (1.0 - 2.0 * margin) * unit01(starB),
        static_cast<double>(cell[2]) + margin + (1.0 - 2.0 * margin) * unit01(starC)};
    star.direction = normalized(Vec3{starLattice[0] / (0.5 * cellsPerAxis), starLattice[1] / (0.5 * cellsPerAxis),
        starLattice[2] / (0.5 * cellsPerAxis)});

    // Galactic band (galaxy_band_strength / galaxy_band_width_radians):
    // cells near the seed-derived galactic plane light up more often and
    // brighter, streaked by a dust rift along the plane's midline, so the
    // backdrop reads as a Milky Way band over the uniform scatter. Both the
    // presence and the brightness ride the STAR's own direction (not the
    // cell's center), so the sampler and the geometry sweep describe one
    // sky. Strength 0 keeps the historical uniform field exactly.
    double bandWeight = 0.0;
    double dustWeight = 0.0;
    if (params.galaxyBandStrength > 0.0)
    {
        const Vec3 galaxyNormal = galaxyPlaneNormal(params);
        const double sineLatitude = dot(star.direction, galaxyNormal);
        const double scaled = sineLatitude / params.galaxyBandWidthRadians;
        bandWeight = params.galaxyBandStrength * std::exp(-scaled * scaled);
        if (bandWeight > 0.0)
        {
            // Dust rift: a long-wavelength simplex lane keyed to the star's
            // direction; the smoothstep keeps it a continuous dark streak
            // through the band's brightest stretch instead of isolated
            // pinpricks of missing sky.
            const double inverseWavelength = 1.0 / kGalaxyDustWavelengthRadians;
            const double dust = noise::simplex3ImproveXY(
                static_cast<std::int64_t>(hashCombine64(params.seed, kGalaxyDustSeedTag)),
                star.direction.x * inverseWavelength, star.direction.y * inverseWavelength,
                star.direction.z * inverseWavelength);
            dustWeight = smoothstep(0.1, 0.62, 0.5 + 0.5 * dust) * bandWeight;
        }
    }

    const double presence = std::min(0.9, kStarPresence
        * (1.0 + kGalaxyPresenceBoost * bandWeight * (1.0 - kGalaxyDustDepth * dustWeight)));
    if (unit01(mixed) >= presence)
    {
        return star; // no star in this cell
    }

    // Power-law brightness: most stars are faint, a few are bright (the real
    // magnitude distribution is roughly a power law). The floor keeps the
    // power law finite; the brightest stars saturate to white through the
    // per-channel clamp, exactly what a bright star reads as after tone
    // mapping. Band stars run brighter still (the unresolved-glow read).
    const double magnitude01 = std::max(1.0e-4, unit01(splitMix64(starC)));
    star.brightness = params.starsBrightness * (0.04 + 0.96 * std::pow(magnitude01, -0.55) * 0.18)
        * (1.0 + kGalaxyBrightnessBoost * bandWeight * (1.0 - 0.5 * dustWeight));

    // Color temperature: hash-driven mix between warm (red giant), neutral
    // white and hot blue. The weights stay subtle - stars read as points of
    // slightly different character, not as colored confetti. Band stars lean
    // toward the warm galactic-core tone away from the dust lane.
    std::uint64_t temperatureHash = starB ^ 0xB5297A4D11D5ull;
    const double temperature = unit01(splitMix64(temperatureHash));
    if (temperature < 0.28)
    {
        star.color = Vec3{1.0, 0.78, 0.58}; // warm
    }
    else if (temperature < 0.72)
    {
        star.color = Vec3{1.0, 0.97, 0.92}; // white
    }
    else
    {
        star.color = Vec3{0.76, 0.85, 1.0}; // blue
    }
    const double coreLean = 0.55 * bandWeight * (1.0 - dustWeight);
    star.color = Vec3{mix(star.color.x, 1.00, coreLean), mix(star.color.y, 0.93, coreLean),
        mix(star.color.z, 0.84, coreLean)};
    star.present = true;
    return star;
}

} // namespace

Vec3 starFieldColor(const PlanetGenerationParams &params, const Vec3 &unitDirection, double sampleSpacingRadians)
{
    if (!params.starsEnabled || params.starsBrightness <= 0.0)
    {
        return Vec3{0.0, 0.0, 0.0};
    }
    const Vec3 direction = normalized(unitDirection);
    if (lengthSquared(direction) < 0.5)
    {
        return Vec3{0.0, 0.0, 0.0}; // a (near-)zero direction has no sky position
    }

    // Lattice cell of the sample direction: the unit cube [-1, 1]^3 holds
    // the sphere's projection, and one cell spans an angle of roughly
    // 2 / cellsPerAxis radians on the sphere's surface.
    const double cellsPerAxis = kStarCellScaleLow
        + (kStarCellScaleHigh - kStarCellScaleLow) * params.starsDensity;
    const double cellAngleRadians = 2.0 / cellsPerAxis;

    // Band limit: a star smaller than the consumer's sample spacing would
    // alias into isolated single-sample speckle (or vanish between samples),
    // so the falloff radius grows to meet the spacing while the brightness
    // falls to keep the field's energy steady - a coarse backdrop keeps the
    // bright stars as soft glows instead of popping noise.
    const double spacing = std::max(0.0, sampleSpacingRadians);
    const double radiusRadians = std::max(cellAngleRadians * kStarFalloffCells, spacing * 0.75);
    const double energyScale = cellAngleRadians * kStarFalloffCells / radiusRadians;

    const double lattice[3] = {direction.x * 0.5 * cellsPerAxis, direction.y * 0.5 * cellsPerAxis,
        direction.z * 0.5 * cellsPerAxis};
    const StarCell star = starOfCell(params, cellsPerAxis, static_cast<std::int64_t>(std::floor(lattice[0])),
        static_cast<std::int64_t>(std::floor(lattice[1])), static_cast<std::int64_t>(std::floor(lattice[2])));
    if (!star.present)
    {
        return Vec3{0.0, 0.0, 0.0};
    }

    // Angular distance sample -> star (both unit vectors).
    const double cosine = std::clamp(dot(direction, star.direction), -1.0, 1.0);
    const double angle  = std::acos(cosine);
    if (angle >= radiusRadians)
    {
        return Vec3{0.0, 0.0, 0.0};
    }
    // Smooth radial falloff: bright core, soft rim (a plain linear disk would
    // read as a polygon at coarse dome resolutions).
    const double falloff = 1.0 - (angle / radiusRadians) * (angle / radiusRadians);
    const double shape   = falloff * falloff;

    const double brightness = star.brightness * energyScale * shape;
    return Vec3{std::clamp(star.color.x * brightness, 0.0, 1.0), std::clamp(star.color.y * brightness, 0.0, 1.0),
        std::clamp(star.color.z * brightness, 0.0, 1.0)};
}

void forEachStar(const PlanetGenerationParams &params, StarVisitor visitor, void *context)
{
    if (!params.starsEnabled || params.starsBrightness <= 0.0 || visitor == nullptr)
    {
        return;
    }
    // Every cell of the projection cube may hold one star; the sweep covers
    // the whole direction sphere exactly once (a jittered cell position
    // normalizes onto the sphere; cube-corner cells simply project to a
    // slightly compressed solid angle).
    const double cellsPerAxis = kStarCellScaleLow
        + (kStarCellScaleHigh - kStarCellScaleLow) * params.starsDensity;
    const std::int64_t halfCells = static_cast<std::int64_t>(cellsPerAxis * 0.5);
    for (std::int64_t z = -halfCells; z < halfCells; ++z)
    {
        for (std::int64_t y = -halfCells; y < halfCells; ++y)
        {
            for (std::int64_t x = -halfCells; x < halfCells; ++x)
            {
                const StarCell star = starOfCell(params, cellsPerAxis, x, y, z);
                if (!star.present)
                {
                    continue;
                }
                const Vec3 color{std::clamp(star.color.x * star.brightness, 0.0, 1.0),
                    std::clamp(star.color.y * star.brightness, 0.0, 1.0),
                    std::clamp(star.color.z * star.brightness, 0.0, 1.0)};
                if (!visitor(star.direction, color, star.brightness, context))
                {
                    return;
                }
            }
        }
    }
}

double galaxyBandWeight(const PlanetGenerationParams &params, const Vec3 &unitDirection)
{
    if (params.galaxyBandStrength <= 0.0)
    {
        return 0.0;
    }
    const Vec3 direction = normalized(unitDirection);
    if (lengthSquared(direction) < 0.5)
    {
        return 0.0; // a (near-)zero direction has no sky position
    }
    const Vec3 galaxyNormal = galaxyPlaneNormal(params);
    const double sineLatitude = dot(direction, galaxyNormal);
    const double scaled = sineLatitude / params.galaxyBandWidthRadians;
    const double band = params.galaxyBandStrength * std::exp(-scaled * scaled);
    if (band <= 0.0)
    {
        return 0.0;
    }
    // The dust rift carves the Great Rift lane out of the glow the same
    // way it suppresses the band's own stars, so the painted band and the
    // enumerated stars describe one sky.
    const double inverseWavelength = 1.0 / kGalaxyDustWavelengthRadians;
    const double dust = noise::simplex3ImproveXY(
        static_cast<std::int64_t>(hashCombine64(params.seed, kGalaxyDustSeedTag)),
        direction.x * inverseWavelength, direction.y * inverseWavelength,
        direction.z * inverseWavelength);
    const double dustWeight = smoothstep(0.1, 0.62, 0.5 + 0.5 * dust) * band;
    return std::clamp(band * (1.0 - kGalaxyDustDepth * dustWeight), 0.0, 1.0);
}

double cirrusStreakWeight(const PlanetGenerationParams &params, const Vec3 &unitDirection,
    double lowPassCutoffMeters)
{
    if (!params.cirrusEnabled || params.cirrusStrength <= 0.0 || params.cirrusCoverageAmount <= 0.0)
    {
        return 0.0;
    }
    const Vec3 direction = normalized(unitDirection);
    if (lengthSquared(direction) < 0.5)
    {
        return 0.0; // a (near-)zero direction has no sky position
    }
    Vec3 shellPosition = direction * (params.radiusMeters + params.cirrusAltitudeMeters);

    // Zonal stretch: the ice-crystal veil rides the upper-level wind, whose
    // prevailing direction runs along the latitude circles. The sample
    // position's component along the spin axis scales UP by the stretch
    // factor, compressing the field across latitude - the streaks then run
    // several times longer than wide around each latitude circle, the
    // anisotropy cirrus photographs show (cumulus banks organize along a
    // seed-derived flow instead; cirrus follows the zonal circulation).
    const Vec3 spinAxis = normalized(Vec3{params.axis[0], params.axis[1], params.axis[2]});
    const double spinComponent = dot(shellPosition, spinAxis);
    shellPosition = shellPosition + spinAxis * (spinComponent * kCirrusStretchFactor);

    const double wavelength = std::max(1.0, params.cloudWavelengthMeters * kCirrusWavelengthFraction);
    const FbmStreamSeeds baseSeeds
        = makeFbmSeeds(static_cast<std::int64_t>(hashCombine64(params.seed, kCirrusSeedTag)), kCirrusBaseFbm.octaves);
    double field = fbm(baseSeeds, shellPosition, wavelength, kCirrusBaseFbm, lowPassCutoffMeters);

    // Filament breakup: a fine octave in the SAME stretched frame tears the
    // thresholded streaks into the filament bundles real cirrus decks show -
    // without it the threshold returns smooth ribbons that read as painted
    // arcs. The octave rides at a fixed fraction of the streak scale and
    // band-limits like every other noise term of the module.
    const double filamentWavelength = std::max(1.0, wavelength * kCirrusFilamentWavelengthFraction);
    const FbmStreamSeeds filamentSeeds = makeFbmSeeds(
        static_cast<std::int64_t>(hashCombine64(params.seed, kCirrusFilamentSeedTag)), kCirrusFilamentFbm.octaves);
    const double filamentFade = kCirrusFilamentStrength
        * lowPassFade(lowPassCutoffMeters, filamentWavelength);
    if (filamentFade > 0.0)
    {
        field += filamentFade * fbm(filamentSeeds, shellPosition, filamentWavelength, kCirrusFilamentFbm,
            lowPassCutoffMeters);
    }

    // Threshold into the streak weight: the crossing centers on the quantile
    // the configured coverage implies (the fbm output is roughly symmetric
    // around zero, exactly the cloud field's convention), and the band stays
    // narrow - cirrus edges ARE crisp filaments, unlike the cumulus deck's
    // soft fringes.
    const double crossing = 1.0 - 2.0 * params.cirrusCoverageAmount;
    return smoothstep(crossing - kCirrusEdgeHalfWidth, crossing + kCirrusEdgeHalfWidth, field);
}

Vec3 sunSkyTint(const PlanetGenerationParams &params, double sunDotUp, bool sunEnabled)
{
    (void)params;
    // Three bands: neutral through the day, warm gold as the sun grazes the
    // horizon, deep blue night past twilight. The bands overlap so walking
    // across a terminator never pops. This is the SCATTERED-light tint: the
    // sky gradient and the aerial haze carry it (they are the air's own
    // glow). Direct sun light uses sunDirectTint() below, whose reddening
    // follows transmission instead of scattering.
    const Vec3 day   = {1.0, 1.0, 1.0};
    const Vec3 gold  = {1.0, 0.58, 0.34};
    const Vec3 night = {0.045, 0.07, 0.14};
    if (!sunEnabled)
    {
        return day;
    }
    const double dayW   = smoothstep(0.10, 0.38, sunDotUp);
    const double nightW = 1.0 - smoothstep(-0.16, 0.02, sunDotUp);
    const double days[3]     = {day.x, day.y, day.z};
    const double golds[3]    = {gold.x, gold.y, gold.z};
    const double nights[3]   = {night.x, night.y, night.z};
    Vec3 tint{};
    double *out[3] = {&tint.x, &tint.y, &tint.z};
    for (std::size_t channel = 0; channel < 3; ++channel)
    {
        const double lowSun = golds[channel] + (days[channel] - golds[channel]) * dayW;
        *out[channel] = std::clamp(nights[channel] + (lowSun - nights[channel]) * (1.0 - nightW), 0.0, 1.0);
    }
    return tint;
}

namespace
{

// Rayleigh vertical optical depths of the whole air column at three
// representative wavelengths (red 680 nm, green 550 nm, blue 440 nm), from
// the lambda^-4 scattering profile. These are constants of the atmosphere
// model the way the sky gradient's band edges are; the amplitude of the
// whole curve is the configured sun_direct_tint_strength.
constexpr double kSunDirectOpticalDepth[3] = {0.035, 0.090, 0.225};

} // namespace

Vec3 sunDirectTint(const PlanetGenerationParams &params, double sunDotUp, bool sunEnabled)
{
    if (!sunEnabled)
    {
        return Vec3{1.0, 1.0, 1.0};
    }
    // Plane-parallel airmass along the sun ray, clamped at the horizon: the
    // transmission curve runs white -> warm -> deep red as the path through
    // the air lengthens - the wavelength selectivity real low sun shows
    // (blue scatters away first, red survives the longest sight line). The
    // max() with the elevation floor caps the airmass at the horizon value
    // (1/0.035 ~ 28.6), the deepest transmission a visible disc or terrain
    // slope can carry.
    const double airmass = 1.0 / std::max(sunDotUp, 0.035);
    const double strength = std::clamp(params.sunDirectTintStrength, 0.0, 1.0);
    Vec3 tint{};
    double *out[3] = {&tint.x, &tint.y, &tint.z};
    for (std::size_t channel = 0; channel < 3; ++channel)
    {
        const double transmitted = std::exp(-kSunDirectOpticalDepth[channel] * airmass);
        // The strength blends the physical curve against white, so 0 keeps
        // the consumer's configured sun color at every elevation.
        *out[channel] = std::clamp(1.0 + (transmitted - 1.0) * strength, 0.0, 1.0);
    }
    return tint;
}

Vec3 moonDirection(const PlanetGenerationParams &params, double seconds)
{
    if (!params.moonEnabled)
    {
        return Vec3{0.0, 0.0, 0.0};
    }

    // Orbit plane: the spin axis tilted by the configured inclination toward
    // a seed-derived equatorial azimuth (an inclined circular orbit whose
    // node longitude varies planet to planet, like real satellite families).
    const Vec3 spinAxis = normalized(Vec3{params.axis[0], params.axis[1], params.axis[2]});
    Vec3 node = cross(spinAxis, Vec3{0.0, 1.0, 0.0});
    if (lengthSquared(node) < 1.0e-6)
    {
        node = cross(spinAxis, Vec3{1.0, 0.0, 0.0});
    }
    const std::uint64_t nodeHash = hashCombine64(params.seed, kMoonSeedTag);
    const double nodeLongitude = static_cast<double>(nodeHash & 0xFFFFull) / 65536.0 * 6.283185307179586;
    // Rotate the node around the spin axis by the hashed longitude, then
    // tilt the plane normal toward it by the inclination.
    const double cosNode = std::cos(nodeLongitude);
    const double sinNode = std::sin(nodeLongitude);
    const Vec3 tiltedNode = normalized(node * cosNode + cross(spinAxis, node) * sinNode
        + spinAxis * (dot(spinAxis, node) * (1.0 - cosNode)));
    const Vec3 orbitNormal = normalized(spinAxis * std::cos(params.moonOrbitInclinationRadians)
        + tiltedNode * std::sin(params.moonOrbitInclinationRadians));

    // Orbit basis: two orthonormal directions in the plane. The reference
    // meridian is the sun's projection onto the plane (the sub-sun crossing),
    // so the phase-zero anchor is the configured sunlight direction - the
    // same convention cloud_shadow_sun_direction documents.
    const Vec3 sunDirection = normalized(
        Vec3{params.cloudShadowSunDirection[0], params.cloudShadowSunDirection[1], params.cloudShadowSunDirection[2]});
    Vec3 reference = sunDirection - orbitNormal * dot(sunDirection, orbitNormal);
    if (lengthSquared(reference) < 1.0e-9)
    {
        // Sun sits on the orbit pole: any in-plane meridian works.
        reference = cross(orbitNormal, Vec3{0.0, 1.0, 0.0});
        if (lengthSquared(reference) < 1.0e-9)
        {
            reference = cross(orbitNormal, Vec3{1.0, 0.0, 0.0});
        }
    }
    reference = normalized(reference);
    const Vec3 companion = cross(orbitNormal, reference);

    // Phase: kMoonLeadRadians ahead of the sub-sun meridian (see the constant
    // above), plus a seed-derived offset so sibling planets do not share one
    // calendar, swept at the configured period.
    const double seedPhase = static_cast<double>((nodeHash >> 24) & 0xFFFFull) / 65536.0 * 0.5;
    const double period = std::max(60.0, params.moonOrbitalPeriodSeconds);
    const double angle = kMoonLeadRadians + seedPhase + 6.283185307179586 * seconds / period;
    return normalized(reference * std::cos(angle) + companion * std::sin(angle));
}

} // namespace planet
} // namespace engine
