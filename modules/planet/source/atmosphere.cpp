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

// Amplitude of the regional weather shift's wavelength, as a multiple of the
// cloud field's own wavelength.
constexpr double kCloudWeatherWavelengthMultiple = 3.2;
// Latitude band frequency: ~0.9 wet/dry cycles per hemisphere (equatorial
// belt, subtropical dry band, mid-latitude storm belt).
constexpr double kCloudBandFrequency = 5.5;

constexpr std::uint64_t kCloudSeedTag    = 0xC10D1ULL;
constexpr std::uint64_t kCloudDetailSeedTag  = 0xC10D2ULL;
constexpr std::uint64_t kCloudWeatherSeedTag = 0xC10D3ULL;
constexpr std::uint64_t kCloudSwirlSeedTag   = 0xC10D4ULL;

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

double CloudField::coverage(const Vec3 &unitDirection, double lowPassCutoffMeters) const
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
    // the belts wrap the planet the way the climate's latitude bands do.
    const double latitude = std::asin(std::clamp(dot(direction, spinAxis_), -1.0, 1.0));
    crossing -= params_.cloudLatitudeBandStrength
        * std::cos(latitude * kCloudBandFrequency) * lowPassFade(lowPassCutoffMeters,
            params_.cloudWavelengthMeters * 2.0);
    const double weatherFade = params_.cloudWeatherStrength * lowPassFade(lowPassCutoffMeters,
        params_.cloudWavelengthMeters * kCloudWeatherWavelengthMultiple);
    if (weatherFade > 0.0)
    {
        crossing -= weatherFade * fbm(seedsWeather_, shellPosition,
            params_.cloudWavelengthMeters * kCloudWeatherWavelengthMultiple, kCloudWeatherFbm, lowPassCutoffMeters);
    }
    const double edgeHalfWidth = std::max(0.01, params_.cloudEdgeSoftness);
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
