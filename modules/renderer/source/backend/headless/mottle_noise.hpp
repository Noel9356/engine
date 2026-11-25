// Procedural surface mottle (MeshMaterial::noiseScaleMeters /
// noiseStrengthMeters): a footprint-adaptive two-octave 3D value noise that
// perturbs the shaded albedo at the sample's world position.
//
// The wavelength rides the pixel's ground footprint - never below the
// material's base scale, never under what one pixel covers - so the mottle
// photographs as soil/sward variation at every distance: grain at a metre,
// patchiness at a hundred, gentle tonal drift from orbit. Unlike a tiled
// scan there is no lattice to read, no gate where the detail stops, and no
// minification moire (the noise band-limits itself).
//
// The scalar form serves the per-sample core and the 4-wide run lanes; the
// AVX-512 form (wide_shading.hpp's gate) serves the 16-lane run kernel.
// Both evaluate the SAME lattice with the SAME hashes and the SAME blend
// weights, so a sample's mottle factor is a pure function of its world
// position, scale and footprint whichever path shades it.

#pragma once

#include "shading_inline.hpp"

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include <algorithm>
#include <cstdint>

namespace engine
{
namespace renderer
{
namespace mottle
{

// One lattice corner's value in [0, 1): an integer avalanche hash of the
// corner's coordinates (deterministic, seedless - the material's scale and
// the geometry's position already identify the surface, and a per-material
// seed would make the mottle jump when the draw's scale key changes).
inline float latticeValue(std::int32_t x, std::int32_t y, std::int32_t z)
{
    std::uint32_t state = static_cast<std::uint32_t>(x) * 0x8DA6B343u ^ static_cast<std::uint32_t>(y) * 0xD816394Cu
        ^ static_cast<std::uint32_t>(z) * 0xCB1AB31Fu;
    state ^= state >> 15;
    state *= 0x2C1B3C6Du;
    state ^= state >> 12;
    state *= 0x97A2D39Bu; // low word of the wide constant; uint32 wraps to it
    state ^= state >> 15;
    return static_cast<float>(state & 0x00FFFFFFu) * (1.0f / 16777216.0f);
}

// Trilinear value noise over the unit lattice: p is the sample position
// already divided by the octave's wavelength. The smoothstep fade keeps the
// interpolation C1-continuous so the mottle shows no lattice creases.
inline float valueNoise3d(const glm::vec3 &p)
{
    const std::int32_t ix = static_cast<std::int32_t>(std::floor(p.x));
    const std::int32_t iy = static_cast<std::int32_t>(std::floor(p.y));
    const std::int32_t iz = static_cast<std::int32_t>(std::floor(p.z));
    const float fx = p.x - static_cast<float>(ix);
    const float fy = p.y - static_cast<float>(iy);
    const float fz = p.z - static_cast<float>(iz);
    const float fadeX = fx * fx * (3.0f - 2.0f * fx);
    const float fadeY = fy * fy * (3.0f - 2.0f * fy);
    const float fadeZ = fz * fz * (3.0f - 2.0f * fz);
    const float c000 = latticeValue(ix, iy, iz);
    const float c100 = latticeValue(ix + 1, iy, iz);
    const float c010 = latticeValue(ix, iy + 1, iz);
    const float c110 = latticeValue(ix + 1, iy + 1, iz);
    const float c001 = latticeValue(ix, iy, iz + 1);
    const float c101 = latticeValue(ix + 1, iy, iz + 1);
    const float c011 = latticeValue(ix, iy + 1, iz + 1);
    const float c111 = latticeValue(ix + 1, iy + 1, iz + 1);
    const float x00 = c000 + (c100 - c000) * fadeX;
    const float x10 = c010 + (c110 - c010) * fadeX;
    const float x01 = c001 + (c101 - c001) * fadeX;
    const float x11 = c011 + (c111 - c011) * fadeX;
    const float y0 = x00 + (x10 - x00) * fadeY;
    const float y1 = x01 + (x11 - x01) * fadeY;
    return y0 + (y1 - y0) * fadeZ;
}

// The mottle value for one sample: the two-octave blend in [0, 1], with
// each octave's wavelength floored at the sample's ground footprint (the
// pixel's world size - the caller's per-record estimate) so minified draws
// fade the sub-pixel octaves out instead of aliasing them into speckle.
inline float mottleValue(const glm::vec3 &worldPosition, float baseScaleMeters, float footprintMeters)
{
    const float scaleLow = std::max(baseScaleMeters, footprintMeters);
    const float scaleHigh = std::max(baseScaleMeters * 0.5f, footprintMeters);
    return 0.6f * valueNoise3d(worldPosition * (1.0f / scaleLow))
        + 0.4f * valueNoise3d(worldPosition * (1.0f / scaleHigh));
}

// The albedo modulation the shading paths apply: albedo *= 1 + strength *
// (2 * n - 1) - symmetric around unity, so the material's mean albedo (and
// every tuned exposure) survives the pass.
inline float albedoModulation(const glm::vec3 &worldPosition, float baseScaleMeters, float footprintMeters,
    float strength)
{
    if (baseScaleMeters <= 0.0f || strength <= 0.0f)
    {
        return 1.0f;
    }
    const float n = mottleValue(worldPosition, baseScaleMeters, footprintMeters);
    return 1.0f + strength * (2.0f * n - 1.0f);
}

} // namespace mottle
} // namespace renderer
} // namespace engine
