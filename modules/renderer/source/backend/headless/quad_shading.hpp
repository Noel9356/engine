// 4-wide shading runs for the tile rasterizer's scan loops.
//
// The headless rasterizer shades every supersample through the full
// Cook-Torrance pass (see shadeTrianglePixel), and that per-sample lighting
// math dominates the frame cost at 1080p with 2x2 supersampling. This kernel
// shades four horizontally consecutive samples of one fan record together in
// a structure-of-arrays layout: every 3-component dot product becomes three
// lane-wise multiplies and two adds (no horizontal reductions), and every
// normalize/division pays once for four samples. The math mirrors
// shadeTrianglePixel's !unlit path and pbrShade's sun/ambient block operation
// for operation (including glm's mix/normalize/smoothstep forms), so the
// run path returns the same values the per-sample path computes. The sun
// block additionally carries the scalar path's per-sample `nDotL > 0` skip
// at RUN granularity: a run whose four lanes all sit in shadow skips the
// whole Cook-Torrance sun term (the masked add contributes exact zeros
// either way) - the 4-wide twin of pbrShade's own branch, without which
// backlit-heavy scenes (a globe's night hemisphere, the self-shaded walls
// of a cliff view) would pay the full sun math for lanes that all
// contribute nothing.
//
// Runs only cover draws the kernel fully models: no direct lights (the
// per-light resolve loop stays scalar in pbrShade), no cloth sheen (the
// Charlie lobe needs a per-lane pow), and no probe-grid walk outside the quad
// cache (the caller's GiTileCache already amortizes that walk over its
// blocks and quads). Normal-mapped draws route through the kernel with the caller
// supplying the finished per-lane shading normal: the TBN build needs a
// per-sample texture fetch and its own normalize chain, so the caller runs
// shadeTrianglePixel's exact normal block per lane (geometry normal,
// two-sided flip, map fetch, TBN multiply) and the kernel lights the
// perturbed normals in SoA. Everything else - texture fetches, the
// reflection-field sample, emissive, tone mapping and the framebuffer
// commits - stays with the caller, per sample.
//
// Fallback: platforms without SSE2 (and any run the caller elects not to
// vectorize) keep the scalar pbrShade path; the scalar fallback here calls
// it directly so the two paths cannot drift apart.

#pragma once

#include "render_shared.hpp"

#include <engine/pbr/capabilities.hpp>

#include <glm/common.hpp>
#include <glm/geometric.hpp>

#include <algorithm>
#include <bit>
#include <cstdint>

#if defined(__SSE2__) || defined(__x86_64__) || defined(_M_X64) || defined(_M_AMD64)
#define ENGINE_SHADING_RUN_SIMD 1
#include <immintrin.h>
#else
#define ENGINE_SHADING_RUN_SIMD 0
#endif

namespace engine
{
namespace renderer
{
namespace shadingrun
{

// Per-draw and per-frame shading constants plus the record's corner
// varyings: everything the run needs that does not vary per sample.
struct RunContext
{
    glm::vec3 cameraPosition{0.0f};
    glm::vec3 lightDirection{0.0f, 0.0f, 1.0f};
    glm::vec3 sunRadiance{0.0f};
    glm::vec3 ambientSky{0.0f};
    glm::vec3 ambientGround{0.0f};
    glm::vec3 baseColorFactor{1.0f};
    float     opacity    = 1.0f;
    float     sheenFactor = 0.0f;
    // Output transform (the per-sample path applies it in the caller): the
    // run folds exposure + tone mapping in, so the returned colors are ready
    // for the encode/commit. Deferred (false) when the caller still has
    // linear-space work to add first - the reflection field's response and
    // emissive both add in linear space, before the transform, exactly like
    // the per-sample path applies them.
    float     exposure = 1.0f;
    pbr::ToneMapping toneMapping = pbr::ToneMapping::None;
    bool      toneMapInside = true;
    bool      doubleSided = false;
    bool      albedoFilled = false; // caller multiplied the base color map in
    bool      unlit = false;        // skip the lighting block; result = albedo
    // Normal-mapped draws supply the finished per-lane shading normal (see
    // RunInputs::shadeNormal): the caller ran the TBN chain per lane, so the
    // kernel consumes the normal as-is instead of interpolating and
    // normalizing the corner normals (and the two-sided flip already
    // happened, before the TBN, exactly where the per-sample path flips).
    bool      normalMapped = false;
};

// Per-sample inputs for one run of four. Only the lanes the caller marked
// active are guaranteed to carry meaningful values; the others are ignored.
struct RunInputs
{
    // Perspective-correct barycentric weights (already normalized to sum to
    // 1) per lane.
    float correctedA[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float correctedB[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float correctedC[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // Albedo per lane (interpolated color * baseColorFactor, texture
    // multiplied by the caller when a base color map exists; derived from the
    // corner colors inside when albedoFilled is false).
    float albedo[3][4]{};
    // Raw material inputs per lane (texture multiplied when maps exist);
    // clamped inside exactly like the scalar path clamps them.
    float metallic[4]{};
    float roughness[4]{};
    float occlusion[4]{};
    // Probe-grid tile-cache sample per lane (see GiTileCache): the hemispheric
    // ambient covers lanes without a valid sample.
    float giIrradiance[3][4]{};
    float giCoverage[4]{};
    float giOcclusion[4]{};
    bool giValid[4]{};
    // Finished per-lane shading normal, read only when
    // RunContext::normalMapped (the caller's TBN chain output).
    float shadeNormal[3][4]{};
    // Sun visibility per lane (the shadow map's percentage-closer factor;
    // 1 = lit, 0 = fully blocked). The caller resolves it with the lane's
    // interpolated position and normal exactly where the per-sample path
    // resolves its own; runs without a shadow map leave the default 1s.
    float sunVisibility[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    // Zenith sky visibility per lane (the hemispheric ambient's grounding
    // factor; 1 = open sky). Scales ONLY the hemispheric term, exactly where
    // the per-sample path scales its ambientSky/ambientGround copies.
    float skyVisibility[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    // Run coverage: lanes the caller wants shaded.
    bool active[4]{};
};

// The tone mappers' SoA twin (mirrors toneMapColor's per-channel math; the
// exposure multiply is the caller's). Operates on the run's X/Y/Z registers.
inline void toneMapSoA(__m128 &x, __m128 &y, __m128 &z, pbr::ToneMapping mapper)
{
    switch (mapper)
    {
    case pbr::ToneMapping::None:
        break;
    case pbr::ToneMapping::Reinhard:
    {
        const __m128 one = _mm_set1_ps(1.0f);
        x = _mm_div_ps(x, _mm_add_ps(one, x));
        y = _mm_div_ps(y, _mm_add_ps(one, y));
        z = _mm_div_ps(z, _mm_add_ps(one, z));
        break;
    }
    case pbr::ToneMapping::AcesFilmic:
    {
        const __m128 zero = _mm_setzero_ps();
        const __m128 one  = _mm_set1_ps(1.0f);
        const auto aces = [zero, one](__m128 v)
        {
            const __m128 scaled = _mm_div_ps(_mm_mul_ps(v, _mm_add_ps(_mm_mul_ps(v, _mm_set1_ps(2.51f)), _mm_set1_ps(0.03f))),
                _mm_add_ps(_mm_mul_ps(v, _mm_add_ps(_mm_mul_ps(v, _mm_set1_ps(2.43f)), _mm_set1_ps(0.59f))), _mm_set1_ps(0.14f)));
            return _mm_min_ps(_mm_max_ps(scaled, zero), one);
        };
        x = aces(x);
        y = aces(y);
        z = aces(z);
        break;
    }
    }
}

// Shades the run's active lanes and returns the TONE-MAPPED colors (the
// per-sample path's shadeTrianglePixel also tone-maps before returning; the
// run folds the same exposure + mapper in, so the caller only encodes and
// commits). outShaded must have room for four vec3s; inactive lanes are
// written with the computed value too (cheap, branch-free) and must be
// ignored by the caller.
//
// Force-inlined like the per-pixel kernels: an outlined call would turn the
// RunInputs marshalling into a real stack round-trip per run.
ENGINE_SHADING_INLINE void shadeRun4(const RunContext &context, const glm::vec3 position[3], const glm::vec3 normal[3],
    const glm::vec3 color[3], const RunInputs &inputs, glm::vec3 *outShaded)
{
#if ENGINE_SHADING_RUN_SIMD
    // ---- lane state ------------------------------------------------------------
    __m128 corrA = _mm_loadu_ps(inputs.correctedA);
    __m128 corrB = _mm_loadu_ps(inputs.correctedB);
    __m128 corrC = _mm_loadu_ps(inputs.correctedC);

    // Perspective-correct interpolation of the corner varyings, per lane:
    // lane i gets corrA_i * A + corrB_i * B + corrC_i * C.
    const auto interpolate = [corrA, corrB, corrC](float ax, float ay, float az, float bx, float by, float bz,
                               float cx, float cy, float cz, __m128 &outX, __m128 &outY, __m128 &outZ)
    {
        const __m128 aX = _mm_set1_ps(ax), aY = _mm_set1_ps(ay), aZ = _mm_set1_ps(az);
        const __m128 bX = _mm_set1_ps(bx), bY = _mm_set1_ps(by), bZ = _mm_set1_ps(bz);
        const __m128 cX = _mm_set1_ps(cx), cY = _mm_set1_ps(cy), cZ = _mm_set1_ps(cz);
        outX = _mm_add_ps(_mm_add_ps(_mm_mul_ps(corrA, aX), _mm_mul_ps(corrB, bX)), _mm_mul_ps(corrC, cX));
        outY = _mm_add_ps(_mm_add_ps(_mm_mul_ps(corrA, aY), _mm_mul_ps(corrB, bY)), _mm_mul_ps(corrC, cY));
        outZ = _mm_add_ps(_mm_add_ps(_mm_mul_ps(corrA, aZ), _mm_mul_ps(corrB, bZ)), _mm_mul_ps(corrC, cZ));
    };

    __m128 albX, albY, albZ;
    if (context.albedoFilled)
    {
        albX = _mm_loadu_ps(inputs.albedo[0]);
        albY = _mm_loadu_ps(inputs.albedo[1]);
        albZ = _mm_loadu_ps(inputs.albedo[2]);
    }
    else
    {
        interpolate(color[0].x, color[0].y, color[0].z, color[1].x, color[1].y, color[1].z, color[2].x, color[2].y,
            color[2].z, albX, albY, albZ);
        const __m128 factorX = _mm_set1_ps(context.baseColorFactor.x);
        const __m128 factorY = _mm_set1_ps(context.baseColorFactor.y);
        const __m128 factorZ = _mm_set1_ps(context.baseColorFactor.z);
        albX = _mm_mul_ps(albX, factorX);
        albY = _mm_mul_ps(albY, factorY);
        albZ = _mm_mul_ps(albZ, factorZ);
    }

    // Unlit draws stop at the albedo (the per-sample path's `if (!unlit)`): the
    // shells that cover whole frames (star domes, atmosphere limbs) spend
    // their whole budget here.
    if (context.unlit)
    {
        // The output transform runs for unlit lanes too (the per-sample path
        // tone-maps unlit colors before returning) - unless deferred behind
        // linear-space adds.
        if (context.toneMapInside)
        {
            albX = _mm_mul_ps(albX, _mm_set1_ps(context.exposure));
            albY = _mm_mul_ps(albY, _mm_set1_ps(context.exposure));
            albZ = _mm_mul_ps(albZ, _mm_set1_ps(context.exposure));
            toneMapSoA(albX, albY, albZ, context.toneMapping);
        }
        alignas(16) float outX[4];
        alignas(16) float outY[4];
        alignas(16) float outZ[4];
        _mm_store_ps(outX, albX);
        _mm_store_ps(outY, albY);
        _mm_store_ps(outZ, albZ);
        for (int lane = 0; lane < 4; ++lane)
        {
            outShaded[lane] = glm::vec3(outX[lane], outY[lane], outZ[lane]);
        }
        return;
    }

    __m128 posX, posY, posZ;
    interpolate(position[0].x, position[0].y, position[0].z, position[1].x, position[1].y, position[1].z,
        position[2].x, position[2].y, position[2].z, posX, posY, posZ);

    // Lane-wise helper: the 3-component dot of two SoA vectors (no horizontal
    // reduction - each lane is one sample's dot).
    const auto dot3 = [](__m128 x0, __m128 y0, __m128 z0, __m128 x1, __m128 y1, __m128 z1)
    {
        return _mm_add_ps(_mm_add_ps(_mm_mul_ps(x0, x1), _mm_mul_ps(y0, y1)), _mm_mul_ps(z0, z1));
    };
    // glm::normalize = v * inversesqrt(dot(v, v)), inversesqrt = 1 / sqrt.
    const auto normalize3 = [&dot3](__m128 &x, __m128 &y, __m128 &z)
    {
        const __m128 dot = dot3(x, y, z, x, y, z);
        const __m128 inv = _mm_div_ps(_mm_set1_ps(1.0f), _mm_sqrt_ps(dot));
        x = _mm_mul_ps(x, inv);
        y = _mm_mul_ps(y, inv);
        z = _mm_mul_ps(z, inv);
    };
    const auto clamp01 = [](__m128 v) { return _mm_min_ps(_mm_max_ps(v, _mm_setzero_ps()), _mm_set1_ps(1.0f)); };

    // ---- geometry normal + view ----------------------------------------------
    // Normal-mapped runs carry the finished per-lane shading normal (the
    // caller ran the TBN chain: fetch, two-sided flip - before the TBN,
    // exactly like the per-sample path - and the final normalize), so the
    // kernel consumes it directly and skips the corner interpolation, the
    // normalize and the flip.
    __m128 nrmX, nrmY, nrmZ;
    if (context.normalMapped)
    {
        nrmX = _mm_loadu_ps(inputs.shadeNormal[0]);
        nrmY = _mm_loadu_ps(inputs.shadeNormal[1]);
        nrmZ = _mm_loadu_ps(inputs.shadeNormal[2]);
    }
    else
    {
        __m128 rawX, rawY, rawZ;
        interpolate(normal[0].x, normal[0].y, normal[0].z, normal[1].x, normal[1].y, normal[1].z, normal[2].x,
            normal[2].y, normal[2].z, rawX, rawY, rawZ);
        normalize3(rawX, rawY, rawZ);
        nrmX = rawX;
        nrmY = rawY;
        nrmZ = rawZ;
    }

    // view = normalize(cameraPosition - worldPosition); the double-sided
    // flip's viewForFlip is the same expression, so the run computes it once
    // and reuses it for pbrShade's view (identical value, half the work).
    __m128 viewX = _mm_sub_ps(_mm_set1_ps(context.cameraPosition.x), posX);
    __m128 viewY = _mm_sub_ps(_mm_set1_ps(context.cameraPosition.y), posY);
    __m128 viewZ = _mm_sub_ps(_mm_set1_ps(context.cameraPosition.z), posZ);
    normalize3(viewX, viewY, viewZ);

    if (!context.normalMapped && context.doubleSided)
    {
        const __m128 facing = dot3(nrmX, nrmY, nrmZ, viewX, viewY, viewZ);
        const __m128 flip = _mm_cmplt_ps(facing, _mm_setzero_ps());
        nrmX = _mm_or_ps(_mm_and_ps(flip, _mm_sub_ps(_mm_setzero_ps(), nrmX)), _mm_andnot_ps(flip, nrmX));
        nrmY = _mm_or_ps(_mm_and_ps(flip, _mm_sub_ps(_mm_setzero_ps(), nrmY)), _mm_andnot_ps(flip, nrmY));
        nrmZ = _mm_or_ps(_mm_and_ps(flip, _mm_sub_ps(_mm_setzero_ps(), nrmZ)), _mm_andnot_ps(flip, nrmZ));
    }

    // ---- pbrShade: ambient block -----------------------------------------------
    __m128 metallic = _mm_loadu_ps(inputs.metallic);
    __m128 roughness = _mm_loadu_ps(inputs.roughness);
    const __m128 occlusion = _mm_loadu_ps(inputs.occlusion);
    const __m128 opacity = clamp01(_mm_set1_ps(context.opacity));
    metallic = clamp01(metallic);
    roughness = _mm_min_ps(_mm_max_ps(roughness, _mm_set1_ps(minimumRoughness)), _mm_set1_ps(1.0f));

    // diffuseAlbedo = albedo * (1 - metallic) * opacity.
    const __m128 oneMinusMetallic = _mm_sub_ps(_mm_set1_ps(1.0f), metallic);
    __m128 diffuseX = _mm_mul_ps(_mm_mul_ps(albX, oneMinusMetallic), opacity);
    __m128 diffuseY = _mm_mul_ps(_mm_mul_ps(albY, oneMinusMetallic), opacity);
    __m128 diffuseZ = _mm_mul_ps(_mm_mul_ps(albZ, oneMinusMetallic), opacity);

    // hemispheric = mix(ambientGround, ambientSky, normal.y * 0.5 + 0.5) *
    // diffuseAlbedo * occlusion; glm::mix(x, y, a) = x * (1 - a) + y * a.
    const __m128 skyWeight = _mm_add_ps(_mm_mul_ps(nrmY, _mm_set1_ps(0.5f)), _mm_set1_ps(0.5f));
    const __m128 invSkyWeight = _mm_sub_ps(_mm_set1_ps(1.0f), skyWeight);
    __m128 ambientX = _mm_mul_ps(
        _mm_add_ps(_mm_mul_ps(_mm_set1_ps(context.ambientGround.x), invSkyWeight),
            _mm_mul_ps(_mm_set1_ps(context.ambientSky.x), skyWeight)),
        diffuseX);
    __m128 ambientY = _mm_mul_ps(
        _mm_add_ps(_mm_mul_ps(_mm_set1_ps(context.ambientGround.y), invSkyWeight),
            _mm_mul_ps(_mm_set1_ps(context.ambientSky.y), skyWeight)),
        diffuseY);
    __m128 ambientZ = _mm_mul_ps(
        _mm_add_ps(_mm_mul_ps(_mm_set1_ps(context.ambientGround.z), invSkyWeight),
            _mm_mul_ps(_mm_set1_ps(context.ambientSky.z), skyWeight)),
        diffuseZ);
    // Zenith sky visibility (per lane): the hemispheric ambient is the sky's
    // diffuse approximation, so what blocks the open sky scales it - the GI
    // ambient blend below stays untouched, matching the per-sample path.
    const __m128 skyVisible = _mm_loadu_ps(inputs.skyVisibility);
    ambientX = _mm_mul_ps(ambientX, occlusion);
    ambientX = _mm_mul_ps(ambientX, skyVisible);
    ambientY = _mm_mul_ps(ambientY, occlusion);
    ambientY = _mm_mul_ps(ambientY, skyVisible);
    ambientZ = _mm_mul_ps(ambientZ, occlusion);
    ambientZ = _mm_mul_ps(ambientZ, skyVisible);

    // Probe-grid quad samples: giTrust = smoothstep(0, 0.25, coverage),
    // giAmbient = diffuseAlbedo * (irradiance / pi) * occlusion, ambient =
    // mix(hemispheric, giAmbient, giTrust) on valid lanes.
    {
        // smoothstep(0, 0.25, c) = t * t * (3 - 2t) with t = clamp(c * 4, 0, 1).
        const __m128 coverage = _mm_loadu_ps(inputs.giCoverage);
        const __m128 t = clamp01(_mm_mul_ps(coverage, _mm_set1_ps(4.0f)));
        const __m128 trust = _mm_mul_ps(_mm_mul_ps(t, t), _mm_sub_ps(_mm_set1_ps(3.0f), _mm_mul_ps(_mm_set1_ps(2.0f), t)));
        const __m128 invPi = _mm_set1_ps(1.0f / glm::pi<float>());
        const __m128 irradX = _mm_loadu_ps(inputs.giIrradiance[0]);
        const __m128 irradY = _mm_loadu_ps(inputs.giIrradiance[1]);
        const __m128 irradZ = _mm_loadu_ps(inputs.giIrradiance[2]);
        // The gather field's contact occlusion rides the sample: the
        // indirect term darkens by it so the tight bands at surface meet
        // lines survive the field's resolution (scalar twin in pbrShade).
        const __m128 giOcclusion = _mm_loadu_ps(inputs.giOcclusion);
        const __m128 giX = _mm_mul_ps(_mm_mul_ps(_mm_mul_ps(diffuseX, _mm_mul_ps(irradX, invPi)), occlusion), giOcclusion);
        const __m128 giY = _mm_mul_ps(_mm_mul_ps(_mm_mul_ps(diffuseY, _mm_mul_ps(irradY, invPi)), occlusion), giOcclusion);
        const __m128 giZ = _mm_mul_ps(_mm_mul_ps(_mm_mul_ps(diffuseZ, _mm_mul_ps(irradZ, invPi)), occlusion), giOcclusion);
        // glm::mix(x, y, a) = x * (1 - a) + y * a; oneMinusTrust = 1 - a.
        const __m128 oneMinusTrust = _mm_sub_ps(_mm_set1_ps(1.0f), trust);
        const __m128 blendX = _mm_add_ps(_mm_mul_ps(ambientX, oneMinusTrust), _mm_mul_ps(giX, trust));
        const __m128 blendY = _mm_add_ps(_mm_mul_ps(ambientY, oneMinusTrust), _mm_mul_ps(giY, trust));
        const __m128 blendZ = _mm_add_ps(_mm_mul_ps(ambientZ, oneMinusTrust), _mm_mul_ps(giZ, trust));
        // Valid-lane select (bitwise; lanes without a valid tile-cache sample
        // keep the hemispheric value). The mask lanes must carry the
        // all-ones / all-zeros BIT patterns: the float VALUE -1.0f encodes
        // as 0xBF800000, and AND-ing that with the blend would splice the
        // blend's exponent onto the hemispheric's mantissa instead of
        // selecting, so the bits are produced with std::bit_cast.
        float validBits[4];
        for (int lane = 0; lane < 4; ++lane)
        {
            validBits[lane] = std::bit_cast<float>(inputs.giValid[lane] ? 0xFFFFFFFFu : 0u);
        }
        const __m128 validMask = _mm_loadu_ps(validBits);
        ambientX = _mm_or_ps(_mm_and_ps(validMask, blendX), _mm_andnot_ps(validMask, ambientX));
        ambientY = _mm_or_ps(_mm_and_ps(validMask, blendY), _mm_andnot_ps(validMask, ambientY));
        ambientZ = _mm_or_ps(_mm_and_ps(validMask, blendZ), _mm_andnot_ps(validMask, ambientZ));
    }

    __m128 resultX = ambientX;
    __m128 resultY = ambientY;
    __m128 resultZ = ambientZ;

    // ---- pbrShade: sun block ----------------------------------------------------
    const __m128 nDotV = _mm_max_ps(dot3(nrmX, nrmY, nrmZ, viewX, viewY, viewZ), _mm_set1_ps(1.0e-4f));
    // f0 = mix(dielectricF0, albedo, metallic).
    const __m128 f0X = _mm_add_ps(_mm_mul_ps(_mm_set1_ps(dielectricF0.x), oneMinusMetallic),
        _mm_mul_ps(albX, metallic));
    const __m128 f0Y = _mm_add_ps(_mm_mul_ps(_mm_set1_ps(dielectricF0.y), oneMinusMetallic),
        _mm_mul_ps(albY, metallic));
    const __m128 f0Z = _mm_add_ps(_mm_mul_ps(_mm_set1_ps(dielectricF0.z), oneMinusMetallic),
        _mm_mul_ps(albZ, metallic));
    // diffuse = albedo * ((1 - metallic) * invPi) * opacity.
    const __m128 invPi = _mm_set1_ps(1.0f / glm::pi<float>());
    const __m128 diffuseTermX = _mm_mul_ps(_mm_mul_ps(albX, _mm_mul_ps(oneMinusMetallic, invPi)), opacity);
    const __m128 diffuseTermY = _mm_mul_ps(_mm_mul_ps(albY, _mm_mul_ps(oneMinusMetallic, invPi)), opacity);
    const __m128 diffuseTermZ = _mm_mul_ps(_mm_mul_ps(albZ, _mm_mul_ps(oneMinusMetallic, invPi)), opacity);

    const __m128 sunNDotL = _mm_max_ps(
        dot3(nrmX, nrmY, nrmZ, _mm_set1_ps(context.lightDirection.x), _mm_set1_ps(context.lightDirection.y),
            _mm_set1_ps(context.lightDirection.z)),
        _mm_setzero_ps());
    const bool sunActive = context.sunRadiance.x > 0.0f || context.sunRadiance.y > 0.0f || context.sunRadiance.z > 0.0f;
    // Whole-run shadow skip (see the header comment): the scalar path's
    // `sunNDotL > 0` test per sample, hoisted to one movemask per run. Lanes
    // with sunNDotL == 0 add exact zeros through the mask below, so skipping
    // the block when NO lane is lit changes no output value.
    const __m128 sunMask = _mm_cmpgt_ps(sunNDotL, _mm_setzero_ps());
    if (sunActive && _mm_movemask_ps(sunMask) != 0)
    {
        // halfVector = normalize(view + lightDirection).
        __m128 halfX = _mm_add_ps(viewX, _mm_set1_ps(context.lightDirection.x));
        __m128 halfY = _mm_add_ps(viewY, _mm_set1_ps(context.lightDirection.y));
        __m128 halfZ = _mm_add_ps(viewZ, _mm_set1_ps(context.lightDirection.z));
        normalize3(halfX, halfY, halfZ);

        const __m128 nDotH = _mm_max_ps(dot3(nrmX, nrmY, nrmZ, halfX, halfY, halfZ), _mm_setzero_ps());
        const __m128 vDotH = _mm_max_ps(dot3(viewX, viewY, viewZ, halfX, halfY, halfZ), _mm_setzero_ps());

        // pbrDistributionGGX: a2 * (1/pi) * (1 / (d * d)), d = nDotH^2 * (a2 - 1) + 1.
        const __m128 a = _mm_mul_ps(roughness, roughness);
        const __m128 a2 = _mm_mul_ps(a, a);
        const __m128 d = _mm_add_ps(_mm_mul_ps(_mm_mul_ps(nDotH, nDotH), _mm_sub_ps(a2, _mm_set1_ps(1.0f))), _mm_set1_ps(1.0f));
        const __m128 invD2 = _mm_div_ps(_mm_set1_ps(1.0f), _mm_mul_ps(d, d));
        const __m128 dTerm = _mm_mul_ps(_mm_mul_ps(a2, invPi), invD2);

        // pbrGeometrySmith with k = (roughness + 1)^2 / 8.
        const __m128 rPlus1 = _mm_add_ps(roughness, _mm_set1_ps(1.0f));
        const __m128 k = _mm_mul_ps(_mm_mul_ps(rPlus1, rPlus1), _mm_set1_ps(0.125f));
        const __m128 gView = _mm_div_ps(nDotV, _mm_add_ps(_mm_mul_ps(nDotV, _mm_sub_ps(_mm_set1_ps(1.0f), k)), k));
        const __m128 gLight = _mm_div_ps(sunNDotL, _mm_add_ps(_mm_mul_ps(sunNDotL, _mm_sub_ps(_mm_set1_ps(1.0f), k)), k));
        const __m128 gTerm = _mm_mul_ps(gView, gLight);

        // pbrFresnelSchlick: f0 + (1 - f0) * (1 - cos)^5.
        const __m128 oneMinusCos = _mm_sub_ps(_mm_set1_ps(1.0f), vDotH);
        const __m128 squared = _mm_mul_ps(oneMinusCos, oneMinusCos);
        const __m128 fifth = _mm_mul_ps(_mm_mul_ps(squared, squared), oneMinusCos);
        const __m128 fX = _mm_add_ps(f0X, _mm_mul_ps(_mm_sub_ps(_mm_set1_ps(1.0f), f0X), fifth));
        const __m128 fY = _mm_add_ps(f0Y, _mm_mul_ps(_mm_sub_ps(_mm_set1_ps(1.0f), f0Y), fifth));
        const __m128 fZ = _mm_add_ps(f0Z, _mm_mul_ps(_mm_sub_ps(_mm_set1_ps(1.0f), f0Z), fifth));

        const __m128 invSpec = _mm_div_ps(_mm_set1_ps(1.0f),
            _mm_add_ps(_mm_mul_ps(_mm_mul_ps(_mm_set1_ps(4.0f), nDotV), sunNDotL), _mm_set1_ps(1.0e-5f)));
        const __m128 specScale = _mm_mul_ps(_mm_mul_ps(dTerm, gTerm), invSpec);

        // result += (diffuse + specular) * sunRadiance * sunNDotL, masked to
        // the lanes whose sunNDotL > 0 (the scalar path's if). The sun
        // shadow map's per-lane visibility multiplies the weight only - the
        // BRDF terms keep the true sunNDotL, exactly where the scalar path
        // scales its sunRadiance copy.
        const __m128 sunWeight = _mm_mul_ps(sunNDotL, _mm_loadu_ps(inputs.sunVisibility));
        const __m128 zero = _mm_setzero_ps();
        const __m128 termX = _mm_mul_ps(_mm_mul_ps(_mm_add_ps(diffuseTermX, _mm_mul_ps(specScale, fX)),
                                             _mm_set1_ps(context.sunRadiance.x)),
            sunWeight);
        const __m128 termY = _mm_mul_ps(_mm_mul_ps(_mm_add_ps(diffuseTermY, _mm_mul_ps(specScale, fY)),
                                             _mm_set1_ps(context.sunRadiance.y)),
            sunWeight);
        const __m128 termZ = _mm_mul_ps(_mm_mul_ps(_mm_add_ps(diffuseTermZ, _mm_mul_ps(specScale, fZ)),
                                             _mm_set1_ps(context.sunRadiance.z)),
            sunWeight);
        resultX = _mm_add_ps(resultX, _mm_or_ps(_mm_and_ps(sunMask, termX), _mm_andnot_ps(sunMask, zero)));
        resultY = _mm_add_ps(resultY, _mm_or_ps(_mm_and_ps(sunMask, termY), _mm_andnot_ps(sunMask, zero)));
        resultZ = _mm_add_ps(resultZ, _mm_or_ps(_mm_and_ps(sunMask, termZ), _mm_andnot_ps(sunMask, zero)));
    }

    // ---- tone map + store (the per-sample path's tail) -------------------------
    if (context.toneMapInside)
    {
        resultX = _mm_mul_ps(resultX, _mm_set1_ps(context.exposure));
        resultY = _mm_mul_ps(resultY, _mm_set1_ps(context.exposure));
        resultZ = _mm_mul_ps(resultZ, _mm_set1_ps(context.exposure));
        toneMapSoA(resultX, resultY, resultZ, context.toneMapping);
    }
    alignas(16) float outX[4];
    alignas(16) float outY[4];
    alignas(16) float outZ[4];
    _mm_store_ps(outX, resultX);
    _mm_store_ps(outY, resultY);
    _mm_store_ps(outZ, resultZ);
    for (int lane = 0; lane < 4; ++lane)
    {
        outShaded[lane] = glm::vec3(outX[lane], outY[lane], outZ[lane]);
    }
#else
    // Scalar fallback: replicate the per-sample path exactly by calling the
    // scalar kernel per active lane (pbrShade handles the ambient + sun
    // blocks; the caller already multiplied the texture maps in).
    (void)context.sheenFactor;
    for (int lane = 0; lane < 4; ++lane)
    {
        if (!inputs.active[lane])
        {
            outShaded[lane] = glm::vec3(0.0f);
            continue;
        }
        const glm::vec3 albedo(context.albedo[0][lane], context.albedo[1][lane], context.albedo[2][lane]);
        if (context.unlit)
        {
            outShaded[lane] = albedo;
            continue;
        }
        glm::vec3 interpolatedPosition = inputs.correctedA[lane] * position[0] + inputs.correctedB[lane] * position[1]
            + inputs.correctedC[lane] * position[2];
        // Normal-mapped lanes arrive with the caller's finished shading
        // normal (TBN chain output; the flip already applied) - consume it
        // instead of interpolating and flipping the corner normals.
        glm::vec3 shadingNormal;
        if (context.normalMapped)
        {
            shadingNormal = glm::vec3(inputs.shadeNormal[0][lane], inputs.shadeNormal[1][lane],
                inputs.shadeNormal[2][lane]);
        }
        else
        {
            const glm::vec3 interpolatedNormal = inputs.correctedA[lane] * normal[0]
                + inputs.correctedB[lane] * normal[1] + inputs.correctedC[lane] * normal[2];
            shadingNormal = glm::normalize(interpolatedNormal);
        }
        const glm::vec3 view = glm::normalize(context.cameraPosition - interpolatedPosition);
        if (!context.normalMapped && context.doubleSided)
        {
            if (glm::dot(shadingNormal, view) < 0.0f)
            {
                shadingNormal = -shadingNormal;
            }
        }
        GiAmbientSample giSample;
        giSample.valid = inputs.giValid[lane];
        giSample.coverage = inputs.giCoverage[lane];
        giSample.occlusion = inputs.giOcclusion[lane];
        giSample.irradiance = glm::vec3(
            inputs.giIrradiance[0][lane], inputs.giIrradiance[1][lane], inputs.giIrradiance[2][lane]);
        outShaded[lane] = toneMapColor(pbrShade(shadingNormal,
            interpolatedPosition,
            context.cameraPosition,
            context.lightDirection,
            context.sunRadiance * inputs.sunVisibility[lane],
            context.ambientSky,
            context.ambientGround,
            albedo,
            inputs.metallic[lane],
            inputs.roughness[lane],
            inputs.occlusion[lane],
            nullptr,
            0u,
            giSample.valid ? &giSample : nullptr,
            context.opacity,
            context.sheenFactor,
            &giSample),
            context.toneMapInside ? context.exposure : 1.0f,
            context.toneMapInside ? context.toneMapping : pbr::ToneMapping::None);
    }
#endif
}

} // namespace shadingrun
} // namespace renderer
} // namespace engine
