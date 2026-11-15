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
// clear systems instead of uniform popcorn. The profiles stay constants of
// the model; coverage, scale and height are the configured knobs.
constexpr FbmParams kCloudBaseFbm{3, 2.0, 0.55};
constexpr FbmParams kCloudDetailFbm{2, 2.0, 0.5};
constexpr FbmParams kCloudWeatherFbm{2, 2.0, 0.5};

// Coverage threshold half-width: how sharply the field crosses into cloud.
// A narrow band gives crisp, scattered banks; a wide one a soft deck.
constexpr double kCloudEdgeWidth = 0.08;
// Amplitude of the regional weather shift on the coverage threshold, and the
// wavelength multiple of the cloud field it rides at.
constexpr double kCloudWeatherStrength = 0.45;
constexpr double kCloudWeatherWavelengthMultiple = 3.2;

constexpr std::uint64_t kCloudSeedTag    = 0xC10D1ULL;
constexpr std::uint64_t kCloudDetailSeedTag  = 0xC10D2ULL;
constexpr std::uint64_t kCloudWeatherSeedTag = 0xC10D3ULL;

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
    const Vec3 shellPosition = direction * (params_.radiusMeters + params_.cloudAltitudeMeters);

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
    // large cloudy and clear systems.
    double crossing = 1.0 - 2.0 * params_.cloudCoverageAmount;
    const double weatherFade = kCloudWeatherStrength * lowPassFade(lowPassCutoffMeters,
        params_.cloudWavelengthMeters * kCloudWeatherWavelengthMultiple);
    if (weatherFade > 0.0)
    {
        crossing -= weatherFade * fbm(seedsWeather_, shellPosition,
            params_.cloudWavelengthMeters * kCloudWeatherWavelengthMultiple, kCloudWeatherFbm, lowPassCutoffMeters);
    }
    return smoothstep(crossing - kCloudEdgeWidth, crossing + kCloudEdgeWidth, field);
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
