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
// cloudDetailStrength) and a very-low-frequency weather field shifts the
// coverage threshold regionally, so banks organize into large cloudy and
// clear systems instead of uniform popcorn. The base banks are sampled in a
// wind-stretched frame: a deterministic flow direction (derived from the
// seed) compresses the base field along the flow, elongating the banks into
// the organized streaks real weather systems follow - unwarped, the banks
// read as round popcorn blobs. A two-octave swirl bends the bank shapes so
// their edges stay organic. The profiles stay constants of the model;
// coverage, scale, edge softness and the weather strength are configured.
constexpr FbmParams kCloudBaseFbm{3, 2.0, 0.55};
constexpr FbmParams kCloudDetailFbm{2, 2.0, 0.5};
constexpr FbmParams kCloudWeatherFbm{2, 2.0, 0.5};
constexpr FbmParams kCloudSwirlFbm{2, 2.0, 0.5};

// Amplitude of the bank-elongation (1.0 = features stretch to about twice
// their width along the flow) and of the swirl bend, in units of the cloud
// wavelength.
constexpr double kCloudStretchFactor = 0.62;
constexpr double kCloudSwirlAmplitude = 0.35;

constexpr std::uint64_t kCloudSeedTag    = 0xC10D1ULL;
constexpr std::uint64_t kCloudDetailSeedTag  = 0xC10D2ULL;
constexpr std::uint64_t kCloudWeatherSeedTag = 0xC10D3ULL;
constexpr std::uint64_t kCloudSwirlSeedTag   = 0xC10D4ULL;

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
}

double CloudField::coverage(const Vec3 &unitDirection, double lowPassCutoffMeters, double softnessScale) const
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

    // Base banks plus the breakup octave; both fade under the consumer's
    // band limit like every other noise term of the module.
    double field = fbm(seedsBase_, shellPosition, params_.cloudWavelengthMeters, kCloudBaseFbm, lowPassCutoffMeters);
    const double detailFade = params_.cloudDetailStrength * lowPassFade(lowPassCutoffMeters, params_.cloudWavelengthMeters * 0.25);
    if (detailFade > 0.0)
    {
        field += detailFade
            * fbm(seedsDetail_, shellPosition, params_.cloudWavelengthMeters * 0.25, kCloudDetailFbm, lowPassCutoffMeters);
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
    // line, with a scale riding the atmosphere height (a tall air column
    // hazes over longer sight lines) and an asymptote that keeps even the
    // farthest terrain faintly readable as terrain instead of sky. Both are
    // configured (aerial_perspective_scale_fraction / _asymptote): a larger
    // scale keeps the midground clear, a lower asymptote hands less of the
    // far distance to the haze color.
    const double scaleFraction = std::max(0.05, params.aerialPerspectiveScaleFraction);
    const double asymptote = std::clamp(params.aerialPerspectiveAsymptote, 0.0, 1.0);
    const double pathMeters = length(targetPlanetLocal - eyePlanetLocal);
    const double scale = std::max(1.0, scaleFraction * params.atmosphereHeightMeters);
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
    // which the smoothstep edges encode. Below the horizon the haze darkens
    // gently instead of cutting off, so any ground-level gap (a chunk still
    // streaming in, the sliver between a sky dome's rim and the visual
    // horizon) photographs as atmosphere, not as a hole to space.
    const double climb = smoothstep(0.02, 0.5, upDot);
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
    if ((static_cast<double>(mixed >> 11) / 9007199254740992.0) >= kStarPresence)
    {
        return star; // no star in this cell
    }
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

    // Power-law brightness: most stars are faint, a few are bright (the real
    // magnitude distribution is roughly a power law). The floor keeps the
    // power law finite; the brightest stars saturate to white through the
    // per-channel clamp, exactly what a bright star reads as after tone
    // mapping.
    const double magnitude01 = std::max(1.0e-4, unit01(splitMix64(starC)));
    star.brightness = params.starsBrightness * (0.04 + 0.96 * std::pow(magnitude01, -0.55) * 0.18);

    // Color temperature: hash-driven mix between warm (red giant), neutral
    // white and hot blue. The weights stay subtle - stars read as points of
    // slightly different character, not as colored confetti.
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

Vec3 sunSkyTint(const PlanetGenerationParams &params, double sunDotUp, bool sunEnabled)
{
    (void)params;
    // Three bands: neutral through the day, warm gold as the sun grazes the
    // horizon, deep blue night past twilight. The bands overlap so walking
    // across a terminator never pops.
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

} // namespace planet
} // namespace engine
