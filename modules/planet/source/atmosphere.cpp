// Sky backdrop model implementation; see atmosphere.hpp for the model.

#include <cstddef>
#include <engine/planet/atmosphere.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>

#include <algorithm>

namespace engine
{
namespace planet
{

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
