// 16-wide AVX-512 kernels for the presentation passes (endFrame's resolve,
// bloom, camera response and their shared table lookups).
//
// The presentation passes run per output pixel AFTER the draw: the tent
// resolve folds 16-bit block sums, the bloom add-back decodes, adds glare
// and re-encodes every pixel, and the camera response layers contrast,
// vignette, grain and dither on top. All of it is straight-line
// lane-independent math over the locked surface's bytes - the ideal
// wide-kernel shape (no branches, no cross-lane dependencies, table lookups
// through gathers). The kernels below mirror the scalar passes operation
// for operation with the same operand order and the same rounding (a
// final + 0.5 then truncation, or the shared byte tables' exact grid), so
// a wide pass returns the bytes the scalar loop would write.
//
// Availability follows wide_shading.hpp: compile-time feature macros and a
// runtime avx512Available() check at the call site; without either, the
// callers keep their existing scalar loops (see resolveRowsTent,
// applyPresentationBloom, applyCameraResponse).
//
// The gather form of the decode tables keeps one table shared: each lane's
// byte indexes a 256-float table through _mm512_i32gather_ps, which the
// hardware resolves in one pass. The byte stores write full 16-lane blocks,
// so the vectorized loops carry their own scalar tails.

#pragma once

#include "wide_shading.hpp"

#if ENGINE_WIDE_SHADING_AVX512

#include <cstdint>

namespace engine
{
namespace renderer
{
namespace widepresentation
{

// ---------------------------------------------------------------------------
// Tent resolve: the horizontal [1 2 1] fold of one row of 16-bit block
// sums. The fold stays in 16-bit lanes: a block sum holds at most
// sampleCount * 255 (8 samples worst case = 2040), and the fold adds three
// of those (6120) - inside uint16. The vector loop covers the interior
// columns 16 at a time (their west/east neighbors exist in memory - the
// loads are unaligned by one texel), and the scalar tail covers the head
// and trailing columns with the caller's edge-clamped rule.
// ---------------------------------------------------------------------------

ENGINE_RASTERIZER_INLINE void tentFoldRowWide(const std::uint16_t *sums, std::uint16_t *folded,
    std::uint32_t columns)
{
    if (columns < 2u)
    {
        return; // the scalar tail (or the caller) owns degenerate rows
    }
    // Channel lanes: one __m512i holds 32 u16 channel values = eight texels
    // (four channels each). The west/east neighbors of a channel shift by
    // one TEXEL - four channels - so the loads offset by +/- 4 u16 elements.
    // A block starting at texel `t` (channel offset 4t) needs texel t-1's
    // channels (the west load) and texel t+8's channels (the east load);
    // the first block starts at texel 1, so the west load reads texel 0, and
    // the last covered texel keeps its east neighbor inside the row.
    const __m512i two = _mm512_set1_epi16(2);
    // Blocks of 8 texels from texel 1; the last covered texel t+7 needs its
    // east neighbor t+8 to exist inside the row: t+8 <= columns-1 (exactly
    // the bound the scalar coverage walk below applies).
    for (std::uint32_t t = 1u; t + 8u <= columns - 1u; t += 8u)
    {
        const std::size_t channelBase = static_cast<std::size_t>(t) * 4u;
        const __m512i center = _mm512_loadu_si512(
            reinterpret_cast<const __m512i *>(sums + channelBase));
        const __m512i west = _mm512_loadu_si512(
            reinterpret_cast<const __m512i *>(sums + channelBase - 4u));
        const __m512i east = _mm512_loadu_si512(
            reinterpret_cast<const __m512i *>(sums + channelBase + 4u));
        const __m512i foldedBlock = _mm512_add_epi16(
            _mm512_add_epi16(west, _mm512_mullo_epi16(center, two)), east);
        _mm512_storeu_si512(reinterpret_cast<__m512i *>(folded + channelBase), foldedBlock);
    }
    // Scalar tail: texel 0, and every texel the vector blocks did not cover
    // (including the last texel, whose east neighbor clamps to itself).
    const auto scalarFold = [sums, folded, columns](std::uint32_t x)
    {
        const std::size_t west = static_cast<std::size_t>(x > 0u ? x - 1u : 0u);
        const std::size_t here = static_cast<std::size_t>(x);
        const std::size_t east = static_cast<std::size_t>(x + 1u < columns ? x + 1u : x);
        for (std::uint32_t channel = 0; channel < 4; ++channel)
        {
            folded[here * 4u + channel] = static_cast<std::uint16_t>(sums[west * 4u + channel]
                + 2u * sums[here * 4u + channel] + sums[east * 4u + channel]);
        }
    };
    // Cover: texel 0, then texels from the end of the last full block. The
    // vector loop covers texels [1, 1+8k) for every block whose east
    // neighbor t+8 <= columns-1 exists - the same bound this walk applies -
    // so the scalar tail picks up exactly the texels no block covered
    // (including the last texel, whose east neighbor clamps to itself).
    scalarFold(0u);
    std::uint32_t covered = 1u; // texel 0 done; blocks cover [1, lastCovered)
    std::uint32_t t = 1u;
    while (t + 8u <= columns - 1u)
    {
        covered = t + 8u;
        t += 8u;
    }
    for (std::uint32_t x = covered; x < columns; ++x)
    {
        scalarFold(x);
    }
}

// ---------------------------------------------------------------------------
// Shared decode/encode helpers.
// ---------------------------------------------------------------------------

// Encode one linear float through the shared encode table the way the
// scalar passes do: index = clamp(trunc(linear * 255 + 0.5)), value =
// encodeTable[index]. The lanes hold ENCODED BYTE VALUES as floats (the
// callers narrow through _mm512_cvtps_epi16 / cvttps back to bytes).
ENGINE_RASTERIZER_INLINE __m512 encodeLinearWide(const float *encodeTable, __m512 linear)
{
    const __m512 clamped = _mm512_min_ps(_mm512_max_ps(linear, _mm512_setzero_ps()),
        _mm512_set1_ps(1.0f));
    const __m512 scaled = _mm512_mul_ps(clamped, _mm512_set1_ps(255.0f));
    const __m512 halfAdded = _mm512_add_ps(scaled, _mm512_set1_ps(0.5f));
    const __m512i index = _mm512_cvttps_epi32(halfAdded);
    const __m512i clampedIndex = _mm512_min_epi32(_mm512_max_epi32(index, _mm512_setzero_epi32()),
        _mm512_set1_epi32(255));
    return _mm512_i32gather_ps(clampedIndex, encodeTable, 4);
}

// Decode a lane of BYTES (as floats) through the inverse table.
ENGINE_RASTERIZER_INLINE __m512 decodeBytesWide(const float *decodeTable, __m512 bytes)
{
    const __m512i index = _mm512_cvtps_epi32(bytes);
    return _mm512_i32gather_ps(index, decodeTable, 4);
}

// ---------------------------------------------------------------------------
// Camera response core: decode through the inverse table, apply the pivot
// contrast (reduced to slope/intercept per lane by the caller), multiply
// the vignette+grain gain, add the dither, clamp, encode back. The grain
// hash and the Bayer dither are precomputed per lane by the caller (they
// need the x/y positions, which the caller owns), so this kernel is pure
// math with the scalar pass's operand order.
// ---------------------------------------------------------------------------

ENGINE_RASTERIZER_INLINE __m512 cameraResponseWide(const float *decodeTable, const float *encodeTable,
    __m512 bytes, __m512 slope, __m512 intercept, __m512 gain, __m512 dither)
{
    const __m512 decoded = decodeBytesWide(decodeTable, bytes);
    const __m512 contrasted = _mm512_add_ps(_mm512_mul_ps(decoded, slope), intercept);
    // The scalar path clamps between the contrast pivot and the
    // vignette/grain gain (processScalarCameraPixel's glm::clamp): without
    // it a contrasted value above 1 keeps scaling past the clamp the scalar
    // sequence applies first, and the wide lanes disagree with the scalar
    // tail of the same row. With contrast == 1 (slope 1, intercept 0) the
    // clamp is a no-op, exactly like the scalar path's conditional clamp.
    const __m512 contrastClamped = _mm512_min_ps(_mm512_max_ps(contrasted, _mm512_setzero_ps()),
        _mm512_set1_ps(1.0f));
    const __m512 gained = _mm512_mul_ps(contrastClamped, gain);
    const __m512 clamped = _mm512_min_ps(_mm512_max_ps(gained, _mm512_setzero_ps()),
        _mm512_set1_ps(1.0f));
    return encodeLinearWide(encodeTable, _mm512_add_ps(clamped, dither));
}

} // namespace widepresentation
} // namespace renderer
} // namespace engine

#endif // ENGINE_WIDE_SHADING_AVX512
