// Internals shared between the physics_fur module's translation units.

#pragma once

#include <cstddef>
#include <cstdint>
#include <engine/physics_fur/capabilities.hpp>

namespace engine
{
namespace physics_fur
{

// Texture synthesis entry point (strand_texture.cpp): writes width*height*4
// RGBA8 bytes. Parameters are pre-validated by the caller.
void generateFurTexturePixels(
    const FurParameters &parameters, std::uint32_t width, std::uint32_t height, std::uint8_t *outPixelsRgba);

// LOD band tables (lod_bands.cpp / physics_fur.cpp): five screen-size bands, most
// detailed first. thresholds[i] is the angular size where band i begins;
// bands[4] catches everything smaller than thresholds[3]. layerCounts is
// the shell budget per band, heightScales shrink the strand length at
// distance (long stray strands at 3 layers read as sparse wire spikes),
// and fadeFraction is the fraction of each band over which the outermost
// layer's alpha dissolves before the next band's layer-count drop.
struct LodBands
{
    static constexpr std::size_t bandCount = 5;

    float        thresholds[4]   = {0.32f, 0.16f, 0.08f, 0.03f};
    std::uint32_t layerCounts[bandCount] = {14u, 10u, 6u, 3u, 1u};
    float        heightScales[bandCount] = {1.0f, 1.0f, 0.9f, 0.8f, 0.7f};
    float        fadeFraction    = 0.35f;
    std::uint32_t maximumLayers  = 16u;
};

// Pure band evaluation (no configuration): band index, fade and height
// scale for an angular size. Exposed for the module's self-checks.
std::size_t lodBandFor(const LodBands &bands, float angularSize);

// The full LOD decision using the given band tables.
FurLodDecision evaluateFurLodDecision(const LodBands &bands, float angularSize);

} // namespace physics_fur
} // namespace engine

