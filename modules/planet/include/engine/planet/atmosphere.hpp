// Sky backdrop model for the planet module.
//
// The planet renders against space: from orbit the background is black, while
// a camera inside the atmosphere sees a sky. This header exposes the pure
// pieces both cases need - a zenith-to-horizon gradient keyed to the view
// direction's elevation above the local horizon, an altitude weight that
// fades the whole sky out toward space, and a sun-elevation tint that warms
// the sky toward sunset and darkens it at night. Everything is a pure
// function of its inputs (no hidden state, safe from any thread), so a
// consumer can paint a sky dome, a clear color, or both per frame without
// any synchronization with the generation pipeline.
//
// All colors are LINEAR RGB in [0, 1]; consumers feed them straight into the
// renderer's vertex colors, which apply the shared tone mapping and sRGB
// encode. The knobs live in PlanetGenerationParams (the [planet] section,
// atmosphere_* keys), so tuning needs no code changes.

#pragma once

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

} // namespace planet
} // namespace engine

