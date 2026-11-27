// 16-wide AVX-512 shading runs for the tile rasterizer's scan loops.
//
// The 4-wide runs (quad_shading.hpp) already amortize the shading math over
// four horizontally consecutive samples, but the machine this backend most
// often ships on carries AVX-512: sixteen lanes per vector. This kernel
// shades SIXTEEN consecutive samples of one fan record in one pass over the
// same SoA forms quad_shading.hpp uses - every 3-component dot product is
// three lane-wise multiplies and two adds, every normalize pays once for
// sixteen samples - and mirrors shadeRun4 operation for operation (same
// operand order, no fused multiply-add contraction anywhere the 4-wide
// kernel does not use one), so a wide run returns bit-identical colors to
// the 4-wide pass the scalar tails already produce. The sun/sky shadow
// visibility twin below ports sunshadows::sunShadowVisibility the same
// way: the map taps become gathers over the map's single depth array with
// the scalar path's per-tap bounds rule, and the project / ramp math keeps
// the scalar operand order, so the per-lane visibility equals the value the
// scalar sampler would return.
//
// Availability is decided at RUNTIME as well as compile time: the file
// compiles to nothing without the AVX-512 feature macros (a -march older
// than AVX-512, or a toolchain without the intrinsics), and even where it
// compiles, avx512Available() must confirm the running CPU before any
// caller routes a record through the wide path. A record the wide path
// does not model (double-sided shading needs the view vector's flip before
// the kernel's normalize; the 4-wide path already handles it) keeps the
// existing 4-wide run, and everything the wide path does not carry (texture
// fetches, the probe-grid tile cache walk, the reflection field, emissive,
// the framebuffer commits) stays per sample with the caller, exactly like
// the 4-wide split of responsibilities.
//
// All lane arrays are SoA: position/normal/color channels and per-lane
// scalars live in separate 16-element arrays so the kernel loads them
// straight into __m512 registers.

#pragma once

#include "mottle_noise.hpp"
#include "quad_shading.hpp"
#include "sun_shadows.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>

#ifndef ENGINE_RASTERIZER_INLINE
#if defined(__GNUC__) || defined(__clang__)
#define ENGINE_RASTERIZER_INLINE inline __attribute__((always_inline))
#else
#define ENGINE_RASTERIZER_INLINE inline
#endif
#endif

#if defined(__AVX512F__) && defined(__AVX512DQ__) && defined(__AVX512BW__) && defined(__AVX512VL__)
#define ENGINE_WIDE_SHADING_AVX512 1
#else
#define ENGINE_WIDE_SHADING_AVX512 0
#endif

#if ENGINE_WIDE_SHADING_AVX512
#include <immintrin.h>
#endif

namespace engine
{
namespace renderer
{
namespace wideshading
{

// Number of samples one wide run shades (the kernel's lane count).
inline constexpr std::uint32_t kWideLanes = 16;

#if ENGINE_WIDE_SHADING_AVX512

// Whether the running CPU executes the AVX-512 foundation the kernel needs
// (F + DQ + BW + VL). Cached on first call: the check is a cpuid walk and
// the rasterizer consults it per record.
inline bool avx512Available()
{
    static const bool available = __builtin_cpu_supports("avx512f") && __builtin_cpu_supports("avx512dq")
        && __builtin_cpu_supports("avx512bw") && __builtin_cpu_supports("avx512vl");
    return available;
}

// ---- shadow visibility -------------------------------------------------------
//
// Ports sunshadows::sunShadowVisibility over sixteen (position, normal)
// pairs at once. Lane i returns the same value the scalar sampler computes
// for its pair: the early-outs (invalid map, back-facing normal, projection
// outside the map) leave the lane at 1.0 through masked selects instead of
// branches, and the tap loop's four box corners become four gathers over
// map.depth with the scalar path's integer truncation, bounds rule and ramp
// clamps.
ENGINE_RASTERIZER_INLINE __m512 sunShadowVisibilityWide(const sunshadows::SunShadowMap &map, __m512 posX, __m512 posY,
    __m512 posZ, __m512 nrmX, __m512 nrmY, __m512 nrmZ, float filterRadiusTexels)
{
    const __m512 one = _mm512_set1_ps(1.0f);
    if (!map.valid || map.resolution < 8u)
    {
        return one;
    }
    // Lane-wise dot(normal, sunDirection): lanes facing away keep 1.0.
    const __m512 sunDirX = _mm512_set1_ps(map.sunDirection.x);
    const __m512 sunDirY = _mm512_set1_ps(map.sunDirection.y);
    const __m512 sunDirZ = _mm512_set1_ps(map.sunDirection.z);
    const __m512 nDotLfull = _mm512_add_ps(_mm512_add_ps(_mm512_mul_ps(nrmX, sunDirX), _mm512_mul_ps(nrmY, sunDirY)),
        _mm512_mul_ps(nrmZ, sunDirZ));
    const __mmask16 facing = _mm512_cmp_ps_mask(nDotLfull, _mm512_setzero_ps(), _CMP_GT_OQ);

    // offsetScale = (1.5 * texelWorldSize) / max(nDotL, 0.35): the scalar
    // denominator clamp keeps grazing normals from inflating the offset.
    const __m512 clampedNDotL = _mm512_max_ps(nDotLfull, _mm512_set1_ps(0.35f));
    const __m512 offsetScale = _mm512_div_ps(_mm512_set1_ps(1.5f * map.texelWorldSize), clampedNDotL);
    const __m512 offX = _mm512_mul_ps(nrmX, offsetScale);
    const __m512 offY = _mm512_mul_ps(nrmY, offsetScale);
    const __m512 offZ = _mm512_mul_ps(nrmZ, offsetScale);
    const __m512 deltaX = _mm512_sub_ps(_mm512_add_ps(posX, offX), _mm512_set1_ps(map.origin.x));
    const __m512 deltaY = _mm512_sub_ps(_mm512_add_ps(posY, offY), _mm512_set1_ps(map.origin.y));
    const __m512 deltaZ = _mm512_sub_ps(_mm512_add_ps(posZ, offZ), _mm512_set1_ps(map.origin.z));
    // u = (dot(delta, right) / halfWidth + 1) * 0.5 (same operand order as
    // the scalar sampler, so the rounding matches lane-for-lane).
    const __m512 dotRight = _mm512_add_ps(
        _mm512_add_ps(_mm512_mul_ps(deltaX, _mm512_set1_ps(map.right.x)),
            _mm512_mul_ps(deltaY, _mm512_set1_ps(map.right.y))),
        _mm512_mul_ps(deltaZ, _mm512_set1_ps(map.right.z)));
    const __m512 uFull = _mm512_mul_ps(
        _mm512_add_ps(_mm512_div_ps(dotRight, _mm512_set1_ps(map.halfWidth)), one), _mm512_set1_ps(0.5f));
    const __m512 dotUp = _mm512_add_ps(_mm512_add_ps(_mm512_mul_ps(deltaX, _mm512_set1_ps(map.up.x)),
                                             _mm512_mul_ps(deltaY, _mm512_set1_ps(map.up.y))),
        _mm512_mul_ps(deltaZ, _mm512_set1_ps(map.up.z)));
    const __m512 vFull = _mm512_mul_ps(
        _mm512_add_ps(_mm512_div_ps(dotUp, _mm512_set1_ps(map.halfHeight)), one), _mm512_set1_ps(0.5f));
    const __m512 dotForward = _mm512_add_ps(
        _mm512_add_ps(_mm512_mul_ps(deltaX, _mm512_set1_ps(map.forward.x)),
            _mm512_mul_ps(deltaY, _mm512_set1_ps(map.forward.y))),
        _mm512_mul_ps(deltaZ, _mm512_set1_ps(map.forward.z)));
    // depth = (dot(delta, forward) - minDepth) / depthExtent.
    const __m512 depthFull = _mm512_div_ps(
        _mm512_sub_ps(dotForward, _mm512_set1_ps(map.minDepth)), _mm512_set1_ps(map.depthExtent));

    const __mmask16 insideMap = facing & _mm512_cmp_ps_mask(uFull, _mm512_setzero_ps(), _CMP_GT_OQ)
        & _mm512_cmp_ps_mask(vFull, _mm512_setzero_ps(), _CMP_GT_OQ) & _mm512_cmp_ps_mask(uFull, one, _CMP_LT_OQ)
        & _mm512_cmp_ps_mask(vFull, one, _CMP_LT_OQ) & _mm512_cmp_ps_mask(depthFull, _mm512_setzero_ps(), _CMP_GT_OQ)
        & _mm512_cmp_ps_mask(depthFull, one, _CMP_LT_OQ);
    if (_mm512_kortestz(insideMap, insideMap))
    {
        return one;
    }

    const __m512 u = _mm512_mask_mov_ps(one, insideMap, uFull);
    const __m512 v = _mm512_mask_mov_ps(one, insideMap, vFull);
    const __m512 depth = _mm512_mask_mov_ps(one, insideMap, depthFull);
    const __m512 nDotL = _mm512_mask_mov_ps(one, insideMap, nDotLfull);

    // slope = sqrt(max(0, 1 - nDotL^2)); bias = ((2 + 3*slope) * texel) /
    // extent (the scalar division order, kept so rounding matches); refDepth
    // per lane.
    const __m512 nDotLSquared = _mm512_mul_ps(nDotL, nDotL);
    const __m512 slope = _mm512_sqrt_ps(_mm512_max_ps(_mm512_sub_ps(one, nDotLSquared), _mm512_setzero_ps()));
    const __m512 bias = _mm512_div_ps(
        _mm512_mul_ps(_mm512_add_ps(_mm512_set1_ps(2.0f), _mm512_mul_ps(_mm512_set1_ps(3.0f), slope)),
            _mm512_set1_ps(map.texelWorldSize)),
        _mm512_set1_ps(map.depthExtent));
    const __m512 refDepth = _mm512_sub_ps(depth, bias);

    const __m512 mapSize = _mm512_set1_ps(static_cast<float>(map.resolution));
    const __m512 radius = _mm512_max_ps(_mm512_set1_ps(filterRadiusTexels), _mm512_setzero_ps());
    // The ramp constants stay scalar (the map is one shared projection), so
    // the scalar path's compare-and-invert reads the same values.
    const float compareRampScalar = 1.5f * map.texelWorldSize / map.depthExtent;
    const __m512 invRamp = compareRampScalar > 1.0e-9f ? _mm512_set1_ps(1.0f / compareRampScalar)
                                                       : _mm512_setzero_ps();
    const __m512 sampleU = _mm512_sub_ps(_mm512_mul_ps(u, mapSize), _mm512_set1_ps(0.5f));
    const __m512 sampleV = _mm512_sub_ps(_mm512_mul_ps(v, mapSize), _mm512_set1_ps(0.5f));

    // tapLit(tapX, tapY): integer truncation, per-tap bounds, one gather,
    // the ramp clamp. Out-of-bounds taps read as lit (1.0).
    const auto tapLitWide = [&](__m512 tapX, __m512 tapY) -> __m512
    {
        const __m512 zero = _mm512_setzero_ps();
        // Column/row via the scalar path's float->int32 truncation.
        const __m512i columnI = _mm512_cvttps_epi32(tapX);
        const __m512i rowI = _mm512_cvttps_epi32(tapY);
        const __mmask16 tapInside = _mm512_cmp_ps_mask(_mm512_cvtepi32_ps(columnI), zero, _CMP_GE_OQ)
            & _mm512_cmp_ps_mask(_mm512_cvtepi32_ps(rowI), zero, _CMP_GE_OQ)
            & _mm512_mask_cmp_epi32_mask(insideMap, columnI, _mm512_set1_epi32(static_cast<std::int32_t>(map.resolution)),
                _CMP_LT_OS)
            & _mm512_mask_cmp_epi32_mask(insideMap, rowI, _mm512_set1_epi32(static_cast<std::int32_t>(map.resolution)),
                _CMP_LT_OS);
        // Index = row * resolution + column (int32 math; the map fits int32).
        const __m512i index = _mm512_add_epi32(
            _mm512_mullo_epi32(rowI, _mm512_set1_epi32(static_cast<std::int32_t>(map.resolution))), columnI);
        // Gather with the lane index clamped into the array (out-of-bounds
        // lanes read texel 0 and are masked out below; the value is never
        // observed). The upper bound is folded in 64-bit first: a resolution
        // whose square exceeds INT32_MAX would make the int32 cast wrap
        // negative and clamp every lane to a negative index (unreachable
        // while the [renderer] settings cap the map at 8192 texels, but the
        // clamp must stay safe for any map the fit accepts).
        const std::uint64_t texelCount = static_cast<std::uint64_t>(map.resolution) * map.resolution;
        const std::int32_t  maximumTexelIndex = static_cast<std::int32_t>(
            std::min<std::uint64_t>(texelCount, static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()) + 1u)
            - 1u);
        const __m512i safeIndex = _mm512_min_epi32(_mm512_max_epi32(index, _mm512_set1_epi32(0)),
            _mm512_set1_epi32(maximumTexelIndex));
        const __m512 stored = _mm512_i32gather_ps(safeIndex, map.depth.data(), 4);
        // clamp((stored - refDepth) * invRamp + 0.5, 0, 1)
        const __m512 ramp = _mm512_add_ps(_mm512_mul_ps(_mm512_sub_ps(stored, refDepth), invRamp),
            _mm512_set1_ps(0.5f));
        const __m512 lit = _mm512_min_ps(_mm512_max_ps(ramp, zero), one);
        return _mm512_mask_mov_ps(one, tapInside, lit);
    };

    __m512 sum;
    if (filterRadiusTexels < 0.25f) // the scalar path's own scalar branch
    {
        sum = tapLitWide(sampleU, sampleV);
    }
    else
    {
        const __m512 sumA = _mm512_add_ps(tapLitWide(_mm512_sub_ps(sampleU, radius), _mm512_sub_ps(sampleV, radius)),
            tapLitWide(_mm512_add_ps(sampleU, radius), _mm512_sub_ps(sampleV, radius)));
        const __m512 sumB = _mm512_add_ps(tapLitWide(_mm512_sub_ps(sampleU, radius), _mm512_add_ps(sampleV, radius)),
            tapLitWide(_mm512_add_ps(sampleU, radius), _mm512_add_ps(sampleV, radius)));
        sum = _mm512_mul_ps(_mm512_add_ps(sumA, sumB), _mm512_set1_ps(0.25f));
    }
    return _mm512_mask_mov_ps(one, insideMap, sum);
}

// ---- 16-lane shading inputs ---------------------------------------------------

// Per-sample inputs for one wide run of sixteen, in the SoA forms the
// kernel loads directly (the 4-wide RunInputs' fields, widened). Only
// active lanes carry meaningful values. Every float member is filled or
// read by a 64-byte-aligned 16-lane store/load, so each carries the same
// alignment the stack arrays beside the struct already declare (the scalar
// flag fields need none, and member-level alignment keeps every address
// valid whatever order the fields sit in).
struct WideRunInputs
{
    alignas(64) float correctedA[16] = {};
    alignas(64) float correctedB[16] = {};
    alignas(64) float correctedC[16] = {};
    alignas(64) float albedo[3][16] = {};
    alignas(64) float metallic[16] = {};
    alignas(64) float roughness[16] = {};
    alignas(64) float occlusion[16] = {};
    alignas(64) float giIrradiance[3][16] = {};
    alignas(64) float giCoverage[16] = {};
    alignas(64) float giOcclusion[16] = {};
    bool giValid[16] = {};
    alignas(64) float shadeNormal[3][16] = {};
    alignas(64) float sunVisibility[16] = {};
    alignas(64) float skyVisibility[16] = {};
    std::uint16_t active = 0;
};

// The tone mappers' 16-lane twin of quad_shading.hpp's toneMapSoA (the same
// per-channel math; the exposure multiply is the caller's).
ENGINE_RASTERIZER_INLINE void toneMapSoA16(__m512 &x, __m512 &y, __m512 &z, pbr::ToneMapping mapper)
{
    switch (mapper)
    {
    case pbr::ToneMapping::None:
        break;
    case pbr::ToneMapping::Reinhard:
    {
        const __m512 one = _mm512_set1_ps(1.0f);
        x = _mm512_div_ps(x, _mm512_add_ps(one, x));
        y = _mm512_div_ps(y, _mm512_add_ps(one, y));
        z = _mm512_div_ps(z, _mm512_add_ps(one, z));
        break;
    }
    case pbr::ToneMapping::AcesFilmic:
    {
        const __m512 zero = _mm512_setzero_ps();
        const __m512 one = _mm512_set1_ps(1.0f);
        const auto aces = [zero, one](__m512 v)
        {
            const __m512 scaled = _mm512_div_ps(
                _mm512_mul_ps(v, _mm512_add_ps(_mm512_mul_ps(v, _mm512_set1_ps(2.51f)), _mm512_set1_ps(0.03f))),
                _mm512_add_ps(_mm512_mul_ps(v, _mm512_add_ps(_mm512_mul_ps(v, _mm512_set1_ps(2.43f)), _mm512_set1_ps(0.59f))),
                    _mm512_set1_ps(0.14f)));
            return _mm512_min_ps(_mm512_max_ps(scaled, zero), one);
        };
        x = aces(x);
        y = aces(y);
        z = aces(z);
        break;
    }
    }
}

// Shades the wide run's active lanes and returns the TONE-MAPPED colors (the
// 4-wide kernel's contract, widened: exposure + tone map fold in unless the
// caller deferred them behind linear-space adds). outShaded is a 3x16 SoA
// block; inactive lanes carry the computed value too (branch-free) and the
// caller ignores them.
ENGINE_RASTERIZER_INLINE void shadeRun16(const shadingrun::RunContext &context, const glm::vec3 position[3],
    const glm::vec3 normal[3], const glm::vec3 color[3], const WideRunInputs &inputs, float (*outShaded)[16])
{
    const __m512 corrA = _mm512_loadu_ps(inputs.correctedA);
    const __m512 corrB = _mm512_loadu_ps(inputs.correctedB);
    const __m512 corrC = _mm512_loadu_ps(inputs.correctedC);

    // Perspective-correct interpolation of the corner varyings, per lane.
    const auto interpolate = [corrA, corrB, corrC](float ax, float ay, float az, float bx, float by, float bz,
                               float cx, float cy, float cz, __m512 &outX, __m512 &outY, __m512 &outZ)
    {
        const __m512 aX = _mm512_set1_ps(ax), aY = _mm512_set1_ps(ay), aZ = _mm512_set1_ps(az);
        const __m512 bX = _mm512_set1_ps(bx), bY = _mm512_set1_ps(by), bZ = _mm512_set1_ps(bz);
        const __m512 cX = _mm512_set1_ps(cx), cY = _mm512_set1_ps(cy), cZ = _mm512_set1_ps(cz);
        outX = _mm512_add_ps(_mm512_add_ps(_mm512_mul_ps(corrA, aX), _mm512_mul_ps(corrB, bX)),
            _mm512_mul_ps(corrC, cX));
        outY = _mm512_add_ps(_mm512_add_ps(_mm512_mul_ps(corrA, aY), _mm512_mul_ps(corrB, bY)),
            _mm512_mul_ps(corrC, cY));
        outZ = _mm512_add_ps(_mm512_add_ps(_mm512_mul_ps(corrA, aZ), _mm512_mul_ps(corrB, bZ)),
            _mm512_mul_ps(corrC, cZ));
    };

    __m512 albX, albY, albZ;
    if (context.albedoFilled)
    {
        albX = _mm512_loadu_ps(inputs.albedo[0]);
        albY = _mm512_loadu_ps(inputs.albedo[1]);
        albZ = _mm512_loadu_ps(inputs.albedo[2]);
    }
    else
    {
        interpolate(color[0].x, color[0].y, color[0].z, color[1].x, color[1].y, color[1].z, color[2].x, color[2].y,
            color[2].z, albX, albY, albZ);
        const __m512 factorX = _mm512_set1_ps(context.baseColorFactor.x);
        const __m512 factorY = _mm512_set1_ps(context.baseColorFactor.y);
        const __m512 factorZ = _mm512_set1_ps(context.baseColorFactor.z);
        albX = _mm512_mul_ps(albX, factorX);
        albY = _mm512_mul_ps(albY, factorY);
        albZ = _mm512_mul_ps(albZ, factorZ);
    }

    if (context.unlit)
    {
        if (context.toneMapInside)
        {
            albX = _mm512_mul_ps(albX, _mm512_set1_ps(context.exposure));
            albY = _mm512_mul_ps(albY, _mm512_set1_ps(context.exposure));
            albZ = _mm512_mul_ps(albZ, _mm512_set1_ps(context.exposure));
            toneMapSoA16(albX, albY, albZ, context.toneMapping);
        }
        _mm512_storeu_ps(outShaded[0], albX);
        _mm512_storeu_ps(outShaded[1], albY);
        _mm512_storeu_ps(outShaded[2], albZ);
        return;
    }

    __m512 posX, posY, posZ;
    interpolate(position[0].x, position[0].y, position[0].z, position[1].x, position[1].y, position[1].z,
        position[2].x, position[2].y, position[2].z, posX, posY, posZ);

    // Lane-wise 3-component dot and normalize (identical forms to the
    // 4-wide kernel's, so the value sequence matches lane-for-lane).
    const auto dot3 = [](__m512 x0, __m512 y0, __m512 z0, __m512 x1, __m512 y1, __m512 z1)
    {
        return _mm512_add_ps(_mm512_add_ps(_mm512_mul_ps(x0, x1), _mm512_mul_ps(y0, y1)), _mm512_mul_ps(z0, z1));
    };
    const auto normalize3 = [&dot3](__m512 &x, __m512 &y, __m512 &z)
    {
        const __m512 dot = dot3(x, y, z, x, y, z);
        const __m512 inv = _mm512_div_ps(_mm512_set1_ps(1.0f), _mm512_sqrt_ps(dot));
        x = _mm512_mul_ps(x, inv);
        y = _mm512_mul_ps(y, inv);
        z = _mm512_mul_ps(z, inv);
    };

    // view = normalize(cameraPosition - worldPosition); the double-sided
    // flip's viewForFlip is the same expression, so the run computes it
    // before the normal block and reuses it there and in pbrShade (the
    // 4-wide kernel's structure, widened).
    __m512 viewX = _mm512_sub_ps(_mm512_set1_ps(context.cameraPosition.x), posX);
    __m512 viewY = _mm512_sub_ps(_mm512_set1_ps(context.cameraPosition.y), posY);
    __m512 viewZ = _mm512_sub_ps(_mm512_set1_ps(context.cameraPosition.z), posZ);
    normalize3(viewX, viewY, viewZ);

    __m512 nrmX, nrmY, nrmZ;
    if (context.normalMapped)
    {
        nrmX = _mm512_loadu_ps(inputs.shadeNormal[0]);
        nrmY = _mm512_loadu_ps(inputs.shadeNormal[1]);
        nrmZ = _mm512_loadu_ps(inputs.shadeNormal[2]);
    }
    else
    {
        __m512 rawX, rawY, rawZ;
        interpolate(normal[0].x, normal[0].y, normal[0].z, normal[1].x, normal[1].y, normal[1].z, normal[2].x,
            normal[2].y, normal[2].z, rawX, rawY, rawZ);
        normalize3(rawX, rawY, rawZ);
        // Double-sided draws flip the interpolated normal toward the viewer
        // on back-facing lanes, exactly where the per-sample path and the
        // 4-wide kernel flip (mesh.frag's glTF doubleSided rule).
        if (context.doubleSided)
        {
            const __m512 facing = dot3(rawX, rawY, rawZ, viewX, viewY, viewZ);
            const __mmask16 flip = _mm512_cmp_ps_mask(facing, _mm512_setzero_ps(), _CMP_LT_OQ);
            rawX = _mm512_mask_mov_ps(rawX, flip, _mm512_sub_ps(_mm512_setzero_ps(), rawX));
            rawY = _mm512_mask_mov_ps(rawY, flip, _mm512_sub_ps(_mm512_setzero_ps(), rawY));
            rawZ = _mm512_mask_mov_ps(rawZ, flip, _mm512_sub_ps(_mm512_setzero_ps(), rawZ));
        }
        nrmX = rawX;
        nrmY = rawY;
        nrmZ = rawZ;
    }

    // ---- ambient block -----------------------------------------------------
    __m512 metallic = _mm512_loadu_ps(inputs.metallic);
    __m512 roughness = _mm512_loadu_ps(inputs.roughness);
    const __m512 occlusion = _mm512_loadu_ps(inputs.occlusion);
    const __m512 zero = _mm512_setzero_ps();
    const __m512 one = _mm512_set1_ps(1.0f);
    metallic = _mm512_min_ps(_mm512_max_ps(metallic, zero), one);
    roughness = _mm512_min_ps(_mm512_max_ps(roughness, _mm512_set1_ps(minimumRoughness)), one);

    // diffuseAlbedo = albedo * (1 - metallic); no opacity factor - mesh.frag's
    // twin carries none, and the commit paths apply the blend exactly once
    // (see pbrShade's header note and shadeRun4's ambient block).
    const __m512 oneMinusMetallic = _mm512_sub_ps(one, metallic);
    __m512 diffuseX = _mm512_mul_ps(albX, oneMinusMetallic);
    __m512 diffuseY = _mm512_mul_ps(albY, oneMinusMetallic);
    __m512 diffuseZ = _mm512_mul_ps(albZ, oneMinusMetallic);

    const __m512 skyWeight = _mm512_add_ps(_mm512_mul_ps(nrmY, _mm512_set1_ps(0.5f)), _mm512_set1_ps(0.5f));
    const __m512 invSkyWeight = _mm512_sub_ps(one, skyWeight);
    __m512 ambientX = _mm512_mul_ps(
        _mm512_add_ps(_mm512_mul_ps(_mm512_set1_ps(context.ambientGround.x), invSkyWeight),
            _mm512_mul_ps(_mm512_set1_ps(context.ambientSky.x), skyWeight)),
        diffuseX);
    __m512 ambientY = _mm512_mul_ps(
        _mm512_add_ps(_mm512_mul_ps(_mm512_set1_ps(context.ambientGround.y), invSkyWeight),
            _mm512_mul_ps(_mm512_set1_ps(context.ambientSky.y), skyWeight)),
        diffuseY);
    __m512 ambientZ = _mm512_mul_ps(
        _mm512_add_ps(_mm512_mul_ps(_mm512_set1_ps(context.ambientGround.z), invSkyWeight),
            _mm512_mul_ps(_mm512_set1_ps(context.ambientSky.z), skyWeight)),
        diffuseZ);
    const __m512 skyVisible = _mm512_loadu_ps(inputs.skyVisibility);
    ambientX = _mm512_mul_ps(_mm512_mul_ps(ambientX, occlusion), skyVisible);
    ambientY = _mm512_mul_ps(_mm512_mul_ps(ambientY, occlusion), skyVisible);
    ambientZ = _mm512_mul_ps(_mm512_mul_ps(ambientZ, occlusion), skyVisible);

    // Probe-grid blend: giTrust = smoothstep(0, 0.25, coverage), the GI
    // ambient's diffuse albedo form, and the valid-lane select through the
    // kernel's mask register.
    {
        const __m512 coverage = _mm512_loadu_ps(inputs.giCoverage);
        const __m512 t = _mm512_min_ps(_mm512_max_ps(_mm512_mul_ps(coverage, _mm512_set1_ps(4.0f)), zero), one);
        const __m512 trust = _mm512_mul_ps(_mm512_mul_ps(t, t), _mm512_sub_ps(_mm512_set1_ps(3.0f),
            _mm512_mul_ps(_mm512_set1_ps(2.0f), t)));
        const __m512 invPi = _mm512_set1_ps(1.0f / glm::pi<float>());
        const __m512 irradX = _mm512_loadu_ps(inputs.giIrradiance[0]);
        const __m512 irradY = _mm512_loadu_ps(inputs.giIrradiance[1]);
        const __m512 irradZ = _mm512_loadu_ps(inputs.giIrradiance[2]);
        const __m512 giOcclusion = _mm512_loadu_ps(inputs.giOcclusion);
        const __m512 giX = _mm512_mul_ps(_mm512_mul_ps(_mm512_mul_ps(diffuseX, _mm512_mul_ps(irradX, invPi)),
                                             occlusion),
            giOcclusion);
        const __m512 giY = _mm512_mul_ps(_mm512_mul_ps(_mm512_mul_ps(diffuseY, _mm512_mul_ps(irradY, invPi)),
                                             occlusion),
            giOcclusion);
        const __m512 giZ = _mm512_mul_ps(_mm512_mul_ps(_mm512_mul_ps(diffuseZ, _mm512_mul_ps(irradZ, invPi)),
                                             occlusion),
            giOcclusion);
        const __m512 oneMinusTrust = _mm512_sub_ps(one, trust);
        const __m512 blendX = _mm512_add_ps(_mm512_mul_ps(ambientX, oneMinusTrust), _mm512_mul_ps(giX, trust));
        const __m512 blendY = _mm512_add_ps(_mm512_mul_ps(ambientY, oneMinusTrust), _mm512_mul_ps(giY, trust));
        const __m512 blendZ = _mm512_add_ps(_mm512_mul_ps(ambientZ, oneMinusTrust), _mm512_mul_ps(giZ, trust));
        std::uint16_t validBits = 0;
        for (std::uint32_t lane = 0; lane < 16; ++lane)
        {
            validBits = static_cast<std::uint16_t>(validBits | (inputs.giValid[lane] ? (1u << lane) : 0u));
        }
        ambientX = _mm512_mask_mov_ps(ambientX, validBits, blendX);
        ambientY = _mm512_mask_mov_ps(ambientY, validBits, blendY);
        ambientZ = _mm512_mask_mov_ps(ambientZ, validBits, blendZ);
    }

    __m512 resultX = ambientX;
    __m512 resultY = ambientY;
    __m512 resultZ = ambientZ;

    // ---- sun block ---------------------------------------------------------
    const __m512 nDotV = _mm512_max_ps(dot3(nrmX, nrmY, nrmZ, viewX, viewY, viewZ), _mm512_set1_ps(1.0e-4f));
    const __m512 f0X = _mm512_add_ps(_mm512_mul_ps(_mm512_set1_ps(dielectricF0.x), oneMinusMetallic),
        _mm512_mul_ps(albX, metallic));
    const __m512 f0Y = _mm512_add_ps(_mm512_mul_ps(_mm512_set1_ps(dielectricF0.y), oneMinusMetallic),
        _mm512_mul_ps(albY, metallic));
    const __m512 f0Z = _mm512_add_ps(_mm512_mul_ps(_mm512_set1_ps(dielectricF0.z), oneMinusMetallic),
        _mm512_mul_ps(albZ, metallic));
    const __m512 invPi = _mm512_set1_ps(1.0f / glm::pi<float>());
    // diffuse = albedo * ((1 - metallic) * invPi); no opacity factor (see the
    // ambient block's note).
    const __m512 diffuseTermX = _mm512_mul_ps(albX, _mm512_mul_ps(oneMinusMetallic, invPi));
    const __m512 diffuseTermY = _mm512_mul_ps(albY, _mm512_mul_ps(oneMinusMetallic, invPi));
    const __m512 diffuseTermZ = _mm512_mul_ps(albZ, _mm512_mul_ps(oneMinusMetallic, invPi));

    const __m512 sunNDotL = _mm512_max_ps(
        dot3(nrmX, nrmY, nrmZ, _mm512_set1_ps(context.lightDirection.x), _mm512_set1_ps(context.lightDirection.y),
            _mm512_set1_ps(context.lightDirection.z)),
        zero);
    const bool sunActive = context.sunRadiance.x > 0.0f || context.sunRadiance.y > 0.0f || context.sunRadiance.z > 0.0f;
    const __mmask16 sunMask = _mm512_cmp_ps_mask(sunNDotL, zero, _CMP_GT_OQ);
    if (sunActive && sunMask != 0)
    {
        __m512 halfX = _mm512_add_ps(viewX, _mm512_set1_ps(context.lightDirection.x));
        __m512 halfY = _mm512_add_ps(viewY, _mm512_set1_ps(context.lightDirection.y));
        __m512 halfZ = _mm512_add_ps(viewZ, _mm512_set1_ps(context.lightDirection.z));
        normalize3(halfX, halfY, halfZ);

        const __m512 nDotH = _mm512_max_ps(dot3(nrmX, nrmY, nrmZ, halfX, halfY, halfZ), zero);
        const __m512 vDotH = _mm512_max_ps(dot3(viewX, viewY, viewZ, halfX, halfY, halfZ), zero);

        const __m512 a = _mm512_mul_ps(roughness, roughness);
        const __m512 a2 = _mm512_mul_ps(a, a);
        const __m512 d = _mm512_add_ps(_mm512_mul_ps(_mm512_mul_ps(nDotH, nDotH),
                                        _mm512_sub_ps(a2, one)),
            one);
        const __m512 invD2 = _mm512_div_ps(one, _mm512_mul_ps(d, d));
        const __m512 dTerm = _mm512_mul_ps(_mm512_mul_ps(a2, invPi), invD2);

        const __m512 rPlus1 = _mm512_add_ps(roughness, one);
        const __m512 k = _mm512_mul_ps(_mm512_mul_ps(rPlus1, rPlus1), _mm512_set1_ps(0.125f));
        const __m512 gView = _mm512_div_ps(nDotV, _mm512_add_ps(_mm512_mul_ps(nDotV, _mm512_sub_ps(one, k)), k));
        const __m512 gLight = _mm512_div_ps(sunNDotL, _mm512_add_ps(_mm512_mul_ps(sunNDotL, _mm512_sub_ps(one, k)), k));
        const __m512 gTerm = _mm512_mul_ps(gView, gLight);

        const __m512 oneMinusCos = _mm512_sub_ps(one, vDotH);
        const __m512 squared = _mm512_mul_ps(oneMinusCos, oneMinusCos);
        const __m512 fifth = _mm512_mul_ps(_mm512_mul_ps(squared, squared), oneMinusCos);
        const __m512 fX = _mm512_add_ps(f0X, _mm512_mul_ps(_mm512_sub_ps(one, f0X), fifth));
        const __m512 fY = _mm512_add_ps(f0Y, _mm512_mul_ps(_mm512_sub_ps(one, f0Y), fifth));
        const __m512 fZ = _mm512_add_ps(f0Z, _mm512_mul_ps(_mm512_sub_ps(one, f0Z), fifth));

        const __m512 invSpec = _mm512_div_ps(one,
            _mm512_add_ps(_mm512_mul_ps(_mm512_mul_ps(_mm512_set1_ps(4.0f), nDotV), sunNDotL),
                _mm512_set1_ps(1.0e-5f)));
        const __m512 specScale = _mm512_mul_ps(_mm512_mul_ps(dTerm, gTerm), invSpec);

        const __m512 sunWeight = _mm512_mul_ps(sunNDotL, _mm512_loadu_ps(inputs.sunVisibility));
        const __m512 termX = _mm512_mul_ps(
            _mm512_mul_ps(_mm512_add_ps(diffuseTermX, _mm512_mul_ps(specScale, fX)),
                _mm512_set1_ps(context.sunRadiance.x)),
            sunWeight);
        const __m512 termY = _mm512_mul_ps(
            _mm512_mul_ps(_mm512_add_ps(diffuseTermY, _mm512_mul_ps(specScale, fY)),
                _mm512_set1_ps(context.sunRadiance.y)),
            sunWeight);
        const __m512 termZ = _mm512_mul_ps(
            _mm512_mul_ps(_mm512_add_ps(diffuseTermZ, _mm512_mul_ps(specScale, fZ)),
                _mm512_set1_ps(context.sunRadiance.z)),
            sunWeight);
        resultX = _mm512_add_ps(resultX, _mm512_maskz_mov_ps(sunMask, termX));
        resultY = _mm512_add_ps(resultY, _mm512_maskz_mov_ps(sunMask, termY));
        resultZ = _mm512_add_ps(resultZ, _mm512_maskz_mov_ps(sunMask, termZ));
    }

    if (context.toneMapInside)
    {
        resultX = _mm512_mul_ps(resultX, _mm512_set1_ps(context.exposure));
        resultY = _mm512_mul_ps(resultY, _mm512_set1_ps(context.exposure));
        resultZ = _mm512_mul_ps(resultZ, _mm512_set1_ps(context.exposure));
        toneMapSoA16(resultX, resultY, resultZ, context.toneMapping);
    }
    _mm512_storeu_ps(outShaded[0], resultX);
    _mm512_storeu_ps(outShaded[1], resultY);
    _mm512_storeu_ps(outShaded[2], resultZ);
}

// ---- 16-lane surface mottle ---------------------------------------------------
//
// The AVX-512 twin of mottle::albedoModulation: the same two-octave value
// noise over the same integer lattice (the hash's uint32 avalanche and the
// smoothstep trilinear blend, lane-wise), with the octave scales floored at
// the sample footprint exactly like the scalar form. Returns the per-lane
// multiplier the shading paths apply to the albedo: 1 + strength * (2n - 1).
ENGINE_RASTERIZER_INLINE __m512 mottleModulationWide(__m512 posX, __m512 posY, __m512 posZ, float baseScaleMeters,
    float footprintMeters, float strength)
{
    const __m512 one = _mm512_set1_ps(1.0f);
    // The shared mottle entry gate (mottle_noise.hpp mottleInputsUsable):
    // bit-level finiteness tests first, so NaN/Inf scale, footprint or
    // strength disable the pass identically here and in the scalar twin,
    // under every math-flag combination (a bare comparison gate folds to a
    // NaN-passing form under -ffinite-math-only).
    if (!mottleInputsUsable(baseScaleMeters, footprintMeters, strength))
    {
        return one;
    }
    const float scaleLow  = baseScaleMeters > footprintMeters ? baseScaleMeters : footprintMeters;
    const float scaleHigh = baseScaleMeters * 0.5f > footprintMeters ? baseScaleMeters * 0.5f : footprintMeters;

    // One octave of trilinear value noise over the integer lattice, lane
    // wise: floor the scaled position, fade with the smoothstep of the
    // fractional parts, and hash the eight corners with mottle::latticeValue's
    // exact avalanche (uint32 wrapping arithmetic, so the wide hash equals the
    // scalar hash bit for bit).
    const auto octaveNoise = [](__m512 x, __m512 y, __m512 z, float inverseScale) -> __m512
    {
        const __m512 px = _mm512_mul_ps(x, _mm512_set1_ps(inverseScale));
        const __m512 py = _mm512_mul_ps(y, _mm512_set1_ps(inverseScale));
        const __m512 pz = _mm512_mul_ps(z, _mm512_set1_ps(inverseScale));
        const __m512 fx = _mm512_floor_ps(px);
        const __m512 fy = _mm512_floor_ps(py);
        const __m512 fz = _mm512_floor_ps(pz);
        // The scalar twin's int32 domain belt (mottle_noise.hpp valueNoise3d):
        // a lane whose floored lattice coordinate is non-finite or outside the
        // int32 domain takes the neutral 0.5 instead of hashing
        // _mm512_cvtps_epi32's INT_MIN indefinite, so both paths return the
        // same bytes for the same garbage. Largest float <= INT32_MAX is
        // 2^31 - 128 (INT32_MAX itself rounds up to 2^31).
        const __m512 latticeLimit = _mm512_set1_ps(2147483520.0f);
        const __m512 negatedLatticeLimit = _mm512_xor_ps(latticeLimit, _mm512_set1_ps(-0.0f));
        const __mmask16 latticeInRange
            = _mm512_cmp_ps_mask(fx, latticeLimit, _CMP_LE_OQ) & _mm512_cmp_ps_mask(fx, negatedLatticeLimit, _CMP_GE_OQ)
            & _mm512_cmp_ps_mask(fy, latticeLimit, _CMP_LE_OQ) & _mm512_cmp_ps_mask(fy, negatedLatticeLimit, _CMP_GE_OQ)
            & _mm512_cmp_ps_mask(fz, latticeLimit, _CMP_LE_OQ) & _mm512_cmp_ps_mask(fz, negatedLatticeLimit, _CMP_GE_OQ);
        const __m512i ix = _mm512_cvtps_epi32(fx);
        const __m512i iy = _mm512_cvtps_epi32(fy);
        const __m512i iz = _mm512_cvtps_epi32(fz);
        const __m512 fracX = _mm512_sub_ps(px, fx);
        const __m512 fracY = _mm512_sub_ps(py, fy);
        const __m512 fracZ = _mm512_sub_ps(pz, fz);
        const auto fade = [](__m512 f)
        {
            return _mm512_mul_ps(_mm512_mul_ps(f, f),
                _mm512_sub_ps(_mm512_set1_ps(3.0f), _mm512_mul_ps(_mm512_set1_ps(2.0f), f)));
        };
        const __m512 fadeX = fade(fracX);
        const __m512 fadeY = fade(fracY);
        const __m512 fadeZ = fade(fracZ);
        const __m512i one = _mm512_set1_epi32(1);
        const auto hashCorner = [&](__m512i cx, __m512i cy, __m512i cz) -> __m512
        {
            const __m512i productX = _mm512_mullo_epi32(cx, _mm512_set1_epi32(static_cast<std::int32_t>(0x8DA6B343u)));
            const __m512i productY = _mm512_mullo_epi32(cy, _mm512_set1_epi32(static_cast<std::int32_t>(0xD816394Cu)));
            const __m512i productZ = _mm512_mullo_epi32(cz, _mm512_set1_epi32(static_cast<std::int32_t>(0xCB1AB31Fu)));
            __m512i state = _mm512_xor_si512(
                _mm512_xor_si512(productX, productY), productZ);
            state = _mm512_xor_si512(state, _mm512_srli_epi32(state, 15));
            state = _mm512_mullo_epi32(state, _mm512_set1_epi32(static_cast<std::int32_t>(0x2C1B3C6Du)));
            state = _mm512_xor_si512(state, _mm512_srli_epi32(state, 12));
            state = _mm512_mullo_epi32(state, _mm512_set1_epi32(static_cast<std::int32_t>(0x97A2D39Bu)));
            state = _mm512_xor_si512(state, _mm512_srli_epi32(state, 15));
            return _mm512_mul_ps(_mm512_cvtepi32_ps(_mm512_and_si512(state, _mm512_set1_epi32(0x00FFFFFFu))),
                _mm512_set1_ps(1.0f / 16777216.0f));
        };
        const __m512i ix1 = _mm512_add_epi32(ix, one);
        const __m512i iy1 = _mm512_add_epi32(iy, one);
        const __m512i iz1 = _mm512_add_epi32(iz, one);
        const __m512 c000 = hashCorner(ix, iy, iz);
        const __m512 c100 = hashCorner(ix1, iy, iz);
        const __m512 c010 = hashCorner(ix, iy1, iz);
        const __m512 c110 = hashCorner(ix1, iy1, iz);
        const __m512 c001 = hashCorner(ix, iy, iz1);
        const __m512 c101 = hashCorner(ix1, iy, iz1);
        const __m512 c011 = hashCorner(ix, iy1, iz1);
        const __m512 c111 = hashCorner(ix1, iy1, iz1);
        const __m512 x00 = _mm512_add_ps(c000, _mm512_mul_ps(_mm512_sub_ps(c100, c000), fadeX));
        const __m512 x10 = _mm512_add_ps(c010, _mm512_mul_ps(_mm512_sub_ps(c110, c010), fadeX));
        const __m512 x01 = _mm512_add_ps(c001, _mm512_mul_ps(_mm512_sub_ps(c101, c001), fadeX));
        const __m512 x11 = _mm512_add_ps(c011, _mm512_mul_ps(_mm512_sub_ps(c111, c011), fadeX));
        const __m512 y0 = _mm512_add_ps(x00, _mm512_mul_ps(_mm512_sub_ps(x10, x00), fadeY));
        const __m512 y1 = _mm512_add_ps(x01, _mm512_mul_ps(_mm512_sub_ps(x11, x01), fadeY));
        const __m512 noise = _mm512_add_ps(y0, _mm512_mul_ps(_mm512_sub_ps(y1, y0), fadeZ));
        return _mm512_mask_blend_ps(latticeInRange, _mm512_set1_ps(0.5f), noise);
    };

    const __m512 noiseLow = octaveNoise(posX, posY, posZ, 1.0f / scaleLow);
    const __m512 noiseHigh = octaveNoise(posX, posY, posZ, 1.0f / scaleHigh);
    const __m512 blend = _mm512_add_ps(_mm512_mul_ps(_mm512_set1_ps(0.6f), noiseLow),
        _mm512_mul_ps(_mm512_set1_ps(0.4f), noiseHigh));
    return _mm512_add_ps(one,
        _mm512_mul_ps(_mm512_set1_ps(strength),
            _mm512_sub_ps(_mm512_mul_ps(_mm512_set1_ps(2.0f), blend), one)));
}

#else // !ENGINE_WIDE_SHADING_AVX512

// Without the AVX-512 feature macros the wide kernel does not exist and no
// caller may route a record through it; the flag reads false everywhere.
inline bool avx512Available()
{
    return false;
}

#endif // ENGINE_WIDE_SHADING_AVX512

} // namespace wideshading
} // namespace renderer
} // namespace engine
