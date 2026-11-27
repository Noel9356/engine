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

#include "finite_math.hpp"

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

#include <algorithm>
#include <cstdint>

namespace engine
{
namespace renderer
{

// Entry gate shared by both mottle paths (the scalar albedoModulation and
// the AVX-512 twin in wide_shading.hpp must accept and reject exactly the
// same inputs): the isFinite bit tests come FIRST so the whole gate stays
// NaN-proof under every math-flag combination. A bare negated comparison
// (`!(scale > 0)`) is enough without -ffinite-math-only, but that flag lets
// the compiler fold it to `scale <= 0` - which NaN passes - so the bit test
// is what actually rejects NaN in such units. Non-finite footprints disable
// the pass too: an Inf footprint previously produced a constant-noise
// surface (scale collapsed to 0), and NaN flowed into std::max where the
// fast-math and standard-semantics builds disagree about which operand
// survives; 1.0 (no modulation) is the deterministic choice every build
// agrees on.
inline bool mottleInputsUsable(float baseScaleMeters, float footprintMeters, float strength) noexcept
{
    return isFinite(baseScaleMeters) && isFinite(footprintMeters) && isFinite(strength) && baseScaleMeters > 0.0f
        && strength > 0.0f;
}
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
// The lattice cast is domain-guarded: a p component outside the int32 domain
// (NaN, or an overflow of position * inverse-scale) would make
// static_cast<int32_t>(floor(p)) undefined behaviour. Every such component
// fails the range test and the noise returns the neutral 0.5 - which makes
// albedoModulation return exactly 1.0, i.e. the mottle pass silently disables
// itself for that sample instead of invoking UB. In-range inputs are
// bit-identical to the unguarded form (and to the AVX-512 twin, whose
// _mm512_cvtps_epi32 returns the same INT_MIN for the same garbage but whose
// belt mirrors this 0.5 neutral so both paths agree everywhere).
inline float valueNoise3d(const glm::vec3 &p)
{
    // Largest float that is <= INT32_MAX after conversion: INT32_MAX itself
    // is not exactly representable (rounds up to 2^31), so the exact bound is
    // the next lower representable value, 2^31 - 128.
    constexpr float kLatticeLimit = 2147483520.0f;
    const float     flooredX      = std::floor(p.x);
    const float     flooredY      = std::floor(p.y);
    const float     flooredZ      = std::floor(p.z);
    if (!isFinite(flooredX) || !isFinite(flooredY) || !isFinite(flooredZ) || flooredX < -kLatticeLimit
        || flooredX > kLatticeLimit || flooredY < -kLatticeLimit || flooredY > kLatticeLimit || flooredZ < -kLatticeLimit
        || flooredZ > kLatticeLimit)
    {
        return 0.5f;
    }
    const std::int32_t ix = static_cast<std::int32_t>(flooredX);
    const std::int32_t iy = static_cast<std::int32_t>(flooredY);
    const std::int32_t iz = static_cast<std::int32_t>(flooredZ);
    const float fx = p.x - flooredX;
    const float fy = p.y - flooredY;
    const float fz = p.z - flooredZ;
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
    if (!mottleInputsUsable(baseScaleMeters, footprintMeters, strength))
    {
        return 1.0f;
    }
    const float n = mottleValue(worldPosition, baseScaleMeters, footprintMeters);
    return 1.0f + strength * (2.0f * n - 1.0f);
}

} // namespace mottle
} // namespace renderer
} // namespace engine
