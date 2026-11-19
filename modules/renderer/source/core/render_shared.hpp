// Shared math/data helpers used by both backends: the default PBR
// environment, the CPU shading model (kept numerically identical to the
// Vulkan fragment shader), tone mapping, and small error/PPM utilities.

#pragma once

#include <engine/direct_lighting/capabilities.hpp>
#include <engine/lumen_gi/capabilities.hpp>
#include <engine/lumen_reflection/capabilities.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/util/ppm.hpp>

#include <glm/common.hpp>
#include <glm/ext/scalar_constants.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace engine
{
namespace renderer
{

// The CPU shading kernel runs once per covered pixel; an outlined call per
// pixel measurably regresses the frame time (and blocks hoisting loop-
// invariant material terms out of the rasterizer's scan loops), so the hot
// shading entry points force inlining on compilers that support it.
#if defined(__GNUC__) || defined(__clang__)
#define ENGINE_SHADING_INLINE inline __attribute__((always_inline))
// Outlined but ODR-safe: declared inline for the header definition while the
// attribute keeps calls to it real calls.
#define ENGINE_SHADING_NOINLINE inline __attribute__((noinline))
#else
#define ENGINE_SHADING_INLINE inline
#define ENGINE_SHADING_NOINLINE inline
#endif

// Device selection bias used when a renderer is created.
enum class DevicePreference : std::uint32_t
{
    Auto, // any device, discrete preferred
    Discrete,
    Integrated
};

// Presentation-queue policy for the swapchain. Auto keeps the historical
// behaviour; the explicit modes exist so frame-latency behaviour is a
// configuration decision instead of a driver default. The enum lives here
// (backend-neutral) because [renderer] present_mode is parsed by the module
// core regardless of which backends are compiled in; only the Vulkan
// backend's swapchain consumes it.
enum class SwapchainPresentMode
{
    Auto,      // FIFO when vsync, else mailbox, else immediate
    Fifo,      // always tear-free, one image on screen, blocks on the compositor
    Mailbox,   // triple buffered, latency-bounded, never blocks the CPU for long
    Immediate, // no presentation queue at all: lowest latency, may tear
};

// BRDF constants shared with shaders/mesh.frag so both backends shade alike.
inline float     minimumRoughness = 0.045f;
inline glm::vec3 dielectricF0     = glm::vec3(0.04f);

// Probes a GiProbeGrid holds at most (two 9^3 clip levels); the Vulkan probe
// storage buffer is sized for this bound.
inline constexpr std::uint32_t maxGiProbes = 2u * 9u * 9u * 9u;
// Floats per probe in the packed storage-buffer/std430 record: 1x vec4
// (position.xyz + validity) + 7x vec4 (the 27 SH2 floats plus one pad).
inline constexpr std::uint32_t giProbeRecordVec4s = 8u;

// Gribb-Hartmann frustum rejection for a world-space bounding sphere: true
// when the sphere lies fully behind any one of the four side planes, so the
// draw cannot touch the viewport. Both backends cull with this (the planes
// are extracted once per camera), so the rejection rule is defined once.
inline bool sphereOutsideFrustum(const glm::vec4 frustumPlanes[4], const glm::vec4 &worldCenter, float worldRadius)
{
    for (std::uint32_t plane = 0; plane < 4; ++plane)
    {
        const float signedDistance = glm::dot(frustumPlanes[plane], worldCenter);
        if (std::isfinite(signedDistance) && signedDistance < -worldRadius)
        {
            return true;
        }
    }
    return false;
}

// Sun + hemisphere ambient + ACES tone mapping, used whenever the pbr
// capability is absent.
inline pbr::PbrEnvironment defaultEnvironment()
{
    const glm::vec3 direction = glm::normalize(glm::vec3(0.45f, 0.62f, 0.35f));
    pbr::PbrEnvironment environment{
        .sun = {
            .direction = {direction.x, direction.y, direction.z},
            .color     = {1.0f, 0.96f, 0.88f},
            .intensity = 3.0f,
            .enabled   = true
        },
        .ambient = {
            .skyColor    = {0.55f, 0.65f, 0.80f},
            .groundColor = {0.35f, 0.30f, 0.25f},
            .intensity   = 0.5f,
        },
        .exposure            = 1.0f,
        .toneMapping         = pbr::ToneMapping::AcesFilmic};
    return environment;
}

// A direct light translated into shading inputs: direction faces away from
// the surface, radiance is color * intensity, and the inverse-squared range
// falloff is precomputed (range is clamped to >= 1e-3 to avoid division by 0).
struct PreparedDirectLight
{
    direct_lighting::DirectLightType type = direct_lighting::DirectLightType::Point;
    glm::vec3                        position{0.0f};
    glm::vec3                        direction{0.0f, -1.0f, 0.0f};
    glm::vec3                        radiance{0.0f};
    float                            inverseRangeSquared = 1.0f;
    float                            innerConeCosine     = 1.0f;
    float                            outerConeCosine     = 0.0f;
};

// Cook-Torrance BRDF terms; formulas must stay in sync with mesh.frag.
inline float pbrDistributionGGX(float nDotH, float roughness)
{
    const float a     = roughness * roughness;
    const float a2    = a * a;
    const float d     = nDotH * nDotH * (a2 - 1.0f) + 1.0f;
    const float invD2 = 1.0f / (d * d);
    return a2 * (1.0f / glm::pi<float>()) * invD2;
}

inline float pbrGeometrySmith(float nDotV, float nDotL, float k)
{
    const float gView  = nDotV / (nDotV * (1.0f - k) + k);
    const float gLight = nDotL / (nDotL * (1.0f - k) + k);
    return gView * gLight;
}

inline glm::vec3 pbrFresnelSchlick(float cosTheta, const glm::vec3 &f0)
{
    const float oneMinusCosine = 1.0f - cosTheta;
    const float squared        = oneMinusCosine * oneMinusCosine;
    const float fifthPower     = squared * squared * oneMinusCosine;
    return f0 + (1.0f - f0) * fifthPower;
}

// ---- lumen_gi probe-grid hook -----------------------------------------------
// One level of the exported probe cache: a regular dims^3 lattice anchored at
// anchor with uniform spacing, index = x + dims.x * (y + dims.y * z), the
// ordering copyProbes documents (z-major, level 0 first then level 1).
struct GiProbeLevel
{
    glm::vec3      anchor{0.0f};
    glm::ivec3     dims{0};
    float          spacing = 1.0f;
    std::uint32_t  offset  = 0; // first probe index inside GiProbeGrid::probes
};

// Snapshot of the lumen_gi world-space radiance cache borrowed by the
// renderer: a GiProbeHeader plus the probe array behind it. Backends copy the
// data synchronously inside setProbeGrid(), so the borrowed view only needs
// to stay valid for that call. A null/empty snapshot means "no GI": the
// shading model keeps its legacy hemispheric ambient (bit-identical to the
// pre-hook behavior).
struct GiProbeGrid
{
    lumen_gi::GiProbeHeader  header{};
    const lumen_gi::GiProbe *probes     = nullptr; // borrowed; header.probeCount entries
    std::uint32_t            probeCount = 0;

    // Precomputed level descriptors, filled once per snapshot by
    // buildProbeLevels(); giProbeLevelFor reads them instead of rebuilding
    // both structs for every shaded pixel.
    GiProbeLevel level0{};
    GiProbeLevel level1{};
    bool         levelsBuilt = false;

    bool valid() const
    {
        return probes != nullptr && probeCount != 0;
    }
};

// ---- lumen_reflection field hook ---------------------------------------------
// Snapshot of the lumen_reflection screen-space field borrowed by the
// renderer: the indirect specular radiance buffer produced for one camera.
// Backends copy the data synchronously inside setReflectionField(), so the
// borrowed view only needs to stay valid for that call. A null/empty field
// means "no reflections": the shading model keeps its pre-hook behavior
// bit-identical.
struct ReflectionField
{
    lumen_reflection::ReflectionFieldHeader header{};
    // Borrowed; width*height*reflectionFieldTexelFloats linear HDR texels
    // (radiance RGB interleaved with the texel roughness, per the
    // lumen_reflection ABI).
    const float *radiance = nullptr;

    bool valid() const
    {
        return radiance != nullptr && header.width != 0 && header.height != 0;
    }
};

// The reflection field flattened for the pixel path: the field's own
// view-projection built once per snapshot instead of per shaded sample, plus
// the dimensions and radiance array the bilinear walk reads.
struct PreparedReflectionField
{
    glm::mat4     viewProjection{0.0f};
    std::uint32_t width  = 0;
    std::uint32_t height = 0;
    // The module's roughness band edge (roughness.max_trace): pixels
    // rougher than this receive no specular environment response, so the
    // field sample early-outs for them.
    float         maxRoughness = 0.4f;
    // Borrowed from the owning backend copy (valid for the frame; see
    // ReflectionField's borrowed-view contract).
    const float *radiance = nullptr;
    bool         valid     = false;
};

inline PreparedReflectionField prepareReflectionField(const ReflectionField &field)
{
    PreparedReflectionField prepared;
    if (field.valid())
    {
        prepared.viewProjection = glm::make_mat4(field.header.viewProjection);
        prepared.width          = field.header.width;
        prepared.height         = field.header.height;
        prepared.maxRoughness   = field.header.maxRoughness;
        prepared.radiance       = field.radiance;
        prepared.valid          = true;
    }
    return prepared;
}

// Bilinear sample of the reflection field at a shaded point: the point is
// projected through the field's own camera, and the four surrounding texels
// are blended (the field is lower-resolution than the framebuffer by
// design; the screen traces inside the module already accounted for the
// geometry the pixel sees). `pixelRoughness` is the shaded pixel's clamped
// roughness and gates the sample twice: a pixel rougher than the field's
// maxRoughness band receives no specular environment response at all (its
// surface never traced one, exactly like the module's own band rule), and
// the four surrounding texels are refused when their surface roughness
// leaves the pixel's band (lumen_reflection::reflectionFieldRoughnessBand)
// or carries the neutral value (no traced response: background, unlit,
// above the traced roughness band), so the bilinear weights renormalize
// over the surviving texels and a fully-refused neighborhood reads black -
// the documented sample-gate contract of the field's texel layout. The
// band gate keeps one material's traced response from bleeding across a
// silhouette onto a rougher neighbor's pixels (the field is
// lower-resolution, so a reflective surface's texels unavoidably touch the
// border of whatever sits behind it). Returns black when the point falls
// outside the field (behind the camera or off its frustum).
// outTraced reports whether any texel of the footprint survived the gates
// (the pixel's material actually has a traced response there). Transmissive
// draws use it to composite the traced response instead of alpha blending:
// the field's transmitted share already carries what the surface shows of
// the scene behind it, so blending the raw background in as well would
// double it.
inline glm::vec3 sampleReflectionField(const PreparedReflectionField &field, const glm::vec3 &worldPosition,
    float pixelRoughness, bool *outTraced = nullptr)
{
    if (!field.valid)
    {
        return glm::vec3(0.0f);
    }
    if (pixelRoughness > field.maxRoughness)
    {
        return glm::vec3(0.0f);
    }
    const glm::mat4 &viewProjection = field.viewProjection;
    const glm::vec4 clip = viewProjection * glm::vec4(worldPosition, 1.0f);
    if (clip.w <= 0.05f)
    {
        return glm::vec3(0.0f);
    }
    const float u = (clip.x / clip.w * 0.5f + 0.5f) * static_cast<float>(field.width) - 0.5f;
    const float v = (0.5f - clip.y / clip.w * 0.5f) * static_cast<float>(field.height) - 0.5f;
    if (u < 0.0f || v < 0.0f || u > static_cast<float>(field.width) - 1.0f
        || v > static_cast<float>(field.height) - 1.0f)
    {
        return glm::vec3(0.0f);
    }
    const std::uint32_t x0 = static_cast<std::uint32_t>(std::floor(u));
    const std::uint32_t y0 = static_cast<std::uint32_t>(std::floor(v));
    const std::uint32_t x1 = std::min(x0 + 1u, field.width - 1u);
    const std::uint32_t y1 = std::min(y0 + 1u, field.height - 1u);
    const float       fx = u - static_cast<float>(x0);
    const float       fy = v - static_cast<float>(y0);
    float roughness00 = 1.0f, roughness10 = 1.0f, roughness01 = 1.0f, roughness11 = 1.0f;
    const auto texelAt = [&field](std::uint32_t x, std::uint32_t y, bool &traced, float &texelRoughness)
    {
        const float *texel
            = field.radiance
              + (static_cast<std::size_t>(y) * field.width + x) * lumen_reflection::reflectionFieldTexelFloats;
        // The trailing roughness is the sample gate: the module reports the
        // neutral value for every texel it never traced a response into, and
        // blending those in would smear background radiance across the
        // neighborhood (or resurrect a stale frame's field when the module
        // skipped an evaluate).
        texelRoughness = texel[3];
        traced = texel[3] < lumen_reflection::reflectionFieldNeutralRoughness;
        return glm::vec3(texel[0], texel[1], texel[2]);
    };
    bool traced00 = false, traced10 = false, traced01 = false, traced11 = false;
    const glm::vec3 t00 = texelAt(x0, y0, traced00, roughness00);
    const glm::vec3 t10 = texelAt(x1, y0, traced10, roughness10);
    const glm::vec3 t01 = texelAt(x0, y1, traced01, roughness01);
    const glm::vec3 t11 = texelAt(x1, y1, traced11, roughness11);
    // Roughness-band gate on top of the traced/neutral gate: a texel traced
    // for a different lobe than the pixel's belongs to another material
    // (the module traced every texel in its own band, so the difference can
    // only come from a silhouette neighbor).
    const float band      = lumen_reflection::reflectionFieldRoughnessBand;
    traced00 = traced00 && std::fabs(roughness00 - pixelRoughness) <= band;
    traced10 = traced10 && std::fabs(roughness10 - pixelRoughness) <= band;
    traced01 = traced01 && std::fabs(roughness01 - pixelRoughness) <= band;
    traced11 = traced11 && std::fabs(roughness11 - pixelRoughness) <= band;
    // Refused texels drop out of the blend; the surviving weights
    // renormalize, and an empty neighborhood (all four refused) reads black.
    const float w00 = traced00 ? (1.0f - fx) * (1.0f - fy) : 0.0f;
    const float w10 = traced10 ? fx * (1.0f - fy) : 0.0f;
    const float w01 = traced01 ? (1.0f - fx) * fy : 0.0f;
    const float w11 = traced11 ? fx * fy : 0.0f;
    const float weightSum = w00 + w10 + w01 + w11;
    if (weightSum <= 0.0f)
    {
        if (outTraced != nullptr)
        {
            *outTraced = false;
        }
        return glm::vec3(0.0f);
    }
    if (outTraced != nullptr)
    {
        *outTraced = true;
    }
    return (t00 * w00 + t10 * w10 + t01 * w01 + t11 * w11) / weightSum;
}

// Builds both level descriptors from the snapshot's header and probe array;
// backends call this once per setProbeGrid so per-pixel shading can pick a
// level without re-reading probe positions. levelsBuilt reports whether the
// snapshot carries a usable pair of levels.
inline bool buildProbeLevels(GiProbeGrid &grid)
{
    const lumen_gi::GiProbeHeader &header = grid.header;
    grid.levelsBuilt                      = false;
    if (!grid.valid() || header.level0Dimensions[0] < 2u || header.level1Dimensions[0] < 2u)
    {
        return false;
    }
    const std::uint32_t level0Count = header.level1ProbeOffset - header.level0ProbeOffset;
    if (header.level0ProbeOffset >= grid.probeCount || level0Count == 0
        || header.level1ProbeOffset >= grid.probeCount)
    {
        return false;
    }

    grid.level0.anchor  = glm::make_vec3(grid.probes[header.level0ProbeOffset].position);
    grid.level0.dims    = glm::ivec3(static_cast<int>(header.level0Dimensions[0]),
        static_cast<int>(header.level0Dimensions[1]), static_cast<int>(header.level0Dimensions[2]));
    grid.level0.spacing = header.level0Spacing;
    grid.level0.offset  = header.level0ProbeOffset;

    grid.level1.anchor  = glm::make_vec3(grid.probes[header.level1ProbeOffset].position);
    grid.level1.dims    = glm::ivec3(static_cast<int>(header.level1Dimensions[0]),
        static_cast<int>(header.level1Dimensions[1]), static_cast<int>(header.level1Dimensions[2]));
    grid.level1.spacing = header.level1Spacing;
    grid.level1.offset  = header.level1ProbeOffset;

    grid.levelsBuilt = true;
    return true;
}

// ---- prepared per-snapshot shading inputs ------------------------------------
//
// Per-pixel shading used to rebuild the probe-grid level descriptors (and the
// reflection field's view-projection) for every shaded sample. These prepared
// views copy the per-snapshot constants once (setProbeGrid / setReflectionField
// time) so the pixel path spends its math on the actual probe walk and field
// sample. The values and expression order are identical to the previous
// per-pixel derivations; only where they are computed moved.

// The probe grid flattened for the pixel path: both level descriptors plus
// the level-0 bounds-test constants (the level pick's only per-snapshot
// data) and the probe array the trilinear walk reads.
struct PreparedGiProbeGrid
{
    GiProbeLevel level0{};
    GiProbeLevel level1{};
    glm::vec3    anchor0{0.0f};
    glm::vec3    extent0{0.0f};
    float        halfCell0 = 0.0f;
    // Borrowed from the owning backend copy (valid for the frame; see
    // GiProbeGrid's borrowed-view contract).
    const lumen_gi::GiProbe *probes     = nullptr;
    std::uint32_t            probeCount = 0;
    bool                     valid      = false;
};

inline PreparedGiProbeGrid prepareGiProbeGrid(GiProbeGrid &grid)
{
    PreparedGiProbeGrid prepared;
    prepared.probes     = grid.probes;
    prepared.probeCount = grid.probeCount;
    if (buildProbeLevels(grid))
    {
        prepared.level0    = grid.level0;
        prepared.level1    = grid.level1;
        prepared.anchor0   = grid.level0.anchor;
        prepared.extent0   = glm::vec3(grid.level0.dims - 1) * grid.level0.spacing;
        prepared.halfCell0 = grid.level0.spacing * 0.5f;
        prepared.valid     = true;
    }
    return prepared;
}

// Picks the lattice level for a world position: level 0 inside its
// half-cell-extended bounds, level 1 everywhere else (its trilinear lookup
// clamps to the boundary probes, so distant geometry still receives the
// coarse ambient). Returns false when the grid holds no usable levels.
inline bool giProbeLevelFor(const PreparedGiProbeGrid &grid, const glm::vec3 &position, GiProbeLevel &outLevel)
{
    if (!grid.valid)
    {
        return false;
    }
    const glm::vec3 local        = position - grid.anchor0;
    const bool      insideLevel0 = glm::all(glm::bvec4(local.x >= -grid.halfCell0, local.y >= -grid.halfCell0,
        local.z >= -grid.halfCell0, true))
                                && glm::all(glm::bvec4(local.x <= grid.extent0.x + grid.halfCell0,
                                     local.y <= grid.extent0.y + grid.halfCell0,
                                     local.z <= grid.extent0.z + grid.halfCell0, true));
    outLevel = insideLevel0 ? grid.level0 : grid.level1;
    return true;
}

// The SH2 basis evaluated at one direction. The band-1 terms are the
// direction's components and the band-2 terms their products; a probe
// footprint evaluation (giProbeGridIrradiance) shades several probes against
// the SAME normal, so the products are computed once per lookup and shared by
// every probe instead of being re-derived per probe.
struct Sh2Basis
{
    float x  = 0.0f;
    float y  = 0.0f;
    float z  = 0.0f;
    float xy = 0.0f;
    float yz = 0.0f;
    float zz3m1 = 0.0f; // 3zz - 1
    float xz = 0.0f;
    float xxmyy = 0.0f; // xx - yy
};

inline Sh2Basis sh2BasisFor(const glm::vec3 &normal)
{
    const float x  = normal.x;
    const float y  = normal.y;
    const float z  = normal.z;
    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;

    Sh2Basis basis;
    basis.x     = x;
    basis.y     = y;
    basis.z     = z;
    basis.xy    = x * y;
    basis.yz    = y * z;
    basis.zz3m1 = 3.0f * zz - 1.0f;
    basis.xz    = x * z;
    basis.xxmyy = xx - yy;
    return basis;
}

// Cosine-lobed SH2 irradiance at one probe (the same convolution as
// lumen_gi::evaluateProbeIrradiance; kept textually in sync because the GLSL
// mirror in shaders/mesh.frag must match this CPU path like the rest of the
// shading model). sh2 is 27 floats in the GiProbe [basis][channel] order; the
// basis comes from sh2BasisFor so a shared normal evaluates its products once.
//
// The three color channels run as packed lanes of one vector each (the SH
// layout is [basis][channel], so each basis term's three channel coefficients
// are contiguous): every lane performs the exact scalar operation sequence the
// per-channel form performed - same products, same addition grouping, hence
// the same rounding - while issuing a third of the instructions.
ENGINE_SHADING_INLINE glm::vec3 giEvaluateSh2Irradiance(const float sh2[27], const Sh2Basis &basis)
{
    constexpr float k0 = 3.141592654f; // pi      (band 0)
    constexpr float k1 = 2.094395102f; // 2pi/3   (band 1)
    constexpr float k2 = 0.785398163f; // pi/4    (band 2)

    const glm::vec3 band0(sh2[0], sh2[1], sh2[2]);
    const glm::vec3 band1Y(sh2[3], sh2[4], sh2[5]);
    const glm::vec3 band1Z(sh2[6], sh2[7], sh2[8]);
    const glm::vec3 band1X(sh2[9], sh2[10], sh2[11]);
    const glm::vec3 band2XY(sh2[12], sh2[13], sh2[14]);
    const glm::vec3 band2YZ(sh2[15], sh2[16], sh2[17]);
    const glm::vec3 band2ZZ(sh2[18], sh2[19], sh2[20]);
    const glm::vec3 band2XZ(sh2[21], sh2[22], sh2[23]);
    const glm::vec3 band2XXYY(sh2[24], sh2[25], sh2[26]);

    // Grouped exactly as the scalar per-channel form grouped its terms:
    // (k0*band0 + k1*((Y*y + Z*z) + X*x)) + k2*(((XY*xy + YZ*yz) + ZZ*zz3m1) + XZ*xz) + XXYY*xxmyy).
    return (k0 * band0 + k1 * ((band1Y * basis.y + band1Z * basis.z) + band1X * basis.x))
           + (k2
                 * ((((band2XY * basis.xy + band2YZ * basis.yz) + band2ZZ * basis.zz3m1) + band2XZ * basis.xz)
                     + band2XXYY * basis.xxmyy));
}

// Trilinear probe-grid irradiance for one shaded point, validity-weighted.
// Returns false when the grid holds no valid probes along this footprint (the
// caller keeps its legacy ambient); outWeight is the validity coverage of the
// footprint in 0..1, used to blend GI against the hemispheric ambient.
ENGINE_SHADING_INLINE bool giProbeGridIrradiance(const PreparedGiProbeGrid &grid, const glm::vec3 &position,
    const glm::vec3 &normal, glm::vec3 &outIrradiance, float &outWeight)
{
    outIrradiance = glm::vec3(0.0f);
    outWeight     = 0.0f;
    GiProbeLevel  level;
    if (!giProbeLevelFor(grid, position, level))
    {
        return false;
    }

    // The SH2 basis is shared by every probe of the footprint (same normal),
    // so its products are computed once here instead of once per probe.
    const Sh2Basis basis = sh2BasisFor(normal);

    const glm::vec3 cell     = (position - level.anchor) / level.spacing;
    const glm::ivec3 base    = glm::clamp(glm::ivec3(glm::floor(cell)), glm::ivec3(0), level.dims - glm::ivec3(2));
    const glm::vec3 fraction = glm::clamp(cell - glm::vec3(base), 0.0f, 1.0f);

    // The irradiance sum evaluates each valid probe corner against the shared
    // basis and accumulates its weighted result; the corner loop keeps its
    // early-outs (probe index range, zero-weight corners) so invalid probes
    // never touch the arithmetic at all.
    float    weightSum     = 0.0f;
    glm::vec3 irradianceSum(0.0f);
    for (int cornerZ = 0; cornerZ < 2; ++cornerZ)
    {
        for (int cornerY = 0; cornerY < 2; ++cornerY)
        {
            for (int cornerX = 0; cornerX < 2; ++cornerX)
            {
                const glm::ivec3 corner = base + glm::ivec3(cornerX, cornerY, cornerZ);
                const std::size_t probeIndex = static_cast<std::size_t>(level.offset)
                    + static_cast<std::size_t>(corner.x + level.dims.x * (corner.y + level.dims.y * corner.z));
                if (probeIndex >= grid.probeCount)
                {
                    continue;
                }
                const lumen_gi::GiProbe &probe = grid.probes[probeIndex];
                const float latticeWeight = (cornerX != 0 ? fraction.x : 1.0f - fraction.x)
                                          * (cornerY != 0 ? fraction.y : 1.0f - fraction.y)
                                          * (cornerZ != 0 ? fraction.z : 1.0f - fraction.z);
                const float weight = latticeWeight * probe.validity;
                if (weight <= 0.0f)
                {
                    continue;
                }
                irradianceSum += weight * giEvaluateSh2Irradiance(&probe.sh2[0][0], basis);
                weightSum += weight;
            }
        }
    }
    if (weightSum <= 0.0f)
    {
        return false;
    }
    outIrradiance = irradianceSum / weightSum;
    outWeight     = glm::clamp(weightSum, 0.0f, 1.0f);
    return true;
}

// ---- shell-fur shading (CPU mirror of shaders/fur.frag) ---------------------
//
// The fur pipeline's fragment stage is deliberately cheap (fur cost is
// fill-rate-bound: every shell layer blends over every covered pixel), so the
// CPU mirror is small. furAlpha is the per-layer strand crop, furShade the
// half-Lambert + Kajiya-Kay lighting. Both formulas must stay in sync with
// fur.frag exactly like pbrShade stays in sync with mesh.frag - changing one
// side requires updating the other (headless_backend.cpp consumes these).

// Per-layer strand alpha (the shell texturing crop). field is the fur
// texture's per-texel strand height (1 at a strand root, 0 at its tip);
// shellRatio the layer ratio in 0..1; densityBoost sharpens the strand edges;
// outerFade is the outermost layer's LOD fade multiplier. Mirrors the alpha
// computation at the top of fur.frag's main().
inline float furAlpha(float field, float shellRatio, float densityBoost, float outerFade)
{
    float alpha = glm::clamp((field - shellRatio) * glm::max(densityBoost, 0.0f), 0.0f, 1.0f);
    if (shellRatio >= 0.999f)
    {
        alpha *= glm::clamp(outerFade, 0.0f, 1.0f);
    }
    return alpha;
}

// Kajiya-Kay anisotropic specular lobe (Kajiya & Kay 1989) with the strand
// tangent: the highlight is a cone around the tangent direction, so
// sin(T,H) = sqrt(1 - (T.H)^2) replaces the N-dot-H of a surface BRDF.
// Mirrors kajiyaKaySpecular() in fur.frag.
inline float furKajiyaKaySpecular(const glm::vec3 &tangent, const glm::vec3 &view, const glm::vec3 &lightDirection)
{
    const glm::vec3 halfVector = glm::normalize(view + lightDirection);
    const float     tDotH      = glm::clamp(glm::dot(tangent, halfVector), -1.0f, 1.0f);
    const float     sinTH      = std::sqrt(glm::max(1.0f - tDotH * tDotH, 0.0f));
    return std::pow(sinTH, 24.0f);
}

// Per-layer strand-field jitter (mirror of furFieldJitter in fur.frag): a
// small uv-hash offset unique to each shell layer, breaking up the classic
// shell-band artifact (horizontal streaks when viewing parallel to the
// shell stack) into noise. Amplitude +-0.06 on the 0..1 field.
inline float furFieldJitter(const glm::vec2 &uv, float layerIndex)
{
    const glm::vec2 hashed = uv * 43.758f + glm::vec2(layerIndex * 7.31f, layerIndex * 3.17f);
    return glm::fract(glm::sin(glm::dot(hashed, glm::vec2(12.9898f, 78.233f))) * 43758.5453f);
}

// The jittered strand field for one shell-layer pixel (the input to
// furAlpha). Shared by the shader and the rasterizer so both crop the same
// field.
inline float furStrandField(float textureAlpha, const glm::vec2 &uv, float layerIndex)
{
    return glm::clamp(textureAlpha + (furFieldJitter(uv, layerIndex) - 0.5f) * 0.12f, 0.0f, 1.0f);
}

// Full fur shading pass for one shell-layer pixel: half-Lambert diffuse
// (fur must not go black when back-lit) with the root-to-tip shade ramp
// applied by the caller via strandShade, Kajiya-Kay sun + direct lights
// against the strand tangent (= the shell normal; strands grow along it),
// and the hemispheric ambient. Mirrors fur.frag's main() lighting section.
// The specular constants (color 0.26, power 24) are the same fixed values
// fur.frag uses.
inline glm::vec3 furShade(const glm::vec3 &normal, const glm::vec3 &worldPosition, const glm::vec3 &cameraPosition,
    const glm::vec3 &lightDirection, const glm::vec3 &sunRadiance, const glm::vec3 &ambientSky,
    const glm::vec3 &ambientGround, const glm::vec3 &strandAlbedo, float strandShade,
    const PreparedDirectLight *directLights, std::uint32_t directLightCount)
{
    const glm::vec3 view       = glm::normalize(cameraPosition - worldPosition);
    const float     halfLambert = glm::dot(normal, lightDirection) * 0.5f + 0.5f;
    const glm::vec3 diffuse    = strandAlbedo * (halfLambert * halfLambert) * strandShade;
    const float     sunSpecular = furKajiyaKaySpecular(normal, view, lightDirection) * strandShade;

    const glm::vec3 upColor   = ambientSky;
    const glm::vec3 downColor = ambientGround;
    const glm::vec3 ambient   = glm::mix(downColor, upColor, normal.y * 0.5f + 0.5f) * strandAlbedo * strandShade;

    glm::vec3 result = ambient + (diffuse + glm::vec3(0.26f) * sunSpecular) * sunRadiance;

    const std::uint32_t lightCount = std::min(directLightCount, direct_lighting::maxDirectLights);
    for (std::uint32_t index = 0; index < lightCount; ++index)
    {
        const PreparedDirectLight &light = directLights[index];
        glm::vec3                  localLightDirection;
        float                      attenuation = 1.0f;
        if (light.type == direct_lighting::DirectLightType::Directional)
        {
            localLightDirection = -light.direction;
        }
        else
        {
            const glm::vec3 toLight           = light.position - worldPosition;
            const float     distanceSquared   = std::max(glm::dot(toLight, toLight), 1.0e-4f);
            const float     rangeRatioSquared = distanceSquared * light.inverseRangeSquared;
            if (rangeRatioSquared >= 1.0f)
            {
                continue;
            }
            const float inverseDistance = 1.0f / std::sqrt(distanceSquared);
            localLightDirection         = toLight * inverseDistance;
            const float rangeFade       = 1.0f - rangeRatioSquared * rangeRatioSquared;
            attenuation                 = rangeFade * rangeFade / distanceSquared;
            if (light.type == direct_lighting::DirectLightType::Spot)
            {
                const float coneCosine = glm::dot(-localLightDirection, light.direction);
                const float coneT = std::clamp((coneCosine - light.outerConeCosine)
                                                   / std::max(light.innerConeCosine - light.outerConeCosine, 1.0e-5f),
                    0.0f,
                    1.0f);
                attenuation *= coneT * coneT * (3.0f - 2.0f * coneT);
            }
        }
        if (attenuation <= 0.0f)
        {
            continue;
        }
        const float lightHalfLambert = glm::dot(normal, localLightDirection) * 0.5f + 0.5f;
        const float lightSpecular    = furKajiyaKaySpecular(normal, view, localLightDirection) * strandShade;
        result += (strandAlbedo * (lightHalfLambert * lightHalfLambert) * strandShade
                   + glm::vec3(0.26f) * lightSpecular)
                * light.radiance * attenuation;
    }
    return result;
}

// Full CPU shading pass: hemispheric ambient (or the lumen_gi probe-grid
// indirect term blended over it by probe validity), one directional sun, then
// the prepared direct lights (range-faded point/spot attenuation with
// smoothstep cones). Roughness/metallic are clamped here exactly like the
// shader. giGrid is the borrowed probe-grid snapshot; nullptr keeps the
// legacy ambient path bit-identical.
ENGINE_SHADING_INLINE glm::vec3 pbrShade(const glm::vec3 &normal, const glm::vec3 &worldPosition, const glm::vec3 &cameraPosition,
    const glm::vec3 &lightDirection, const glm::vec3 &sunRadiance, const glm::vec3 &ambientSky,
    const glm::vec3 &ambientGround, const glm::vec3 &albedo, float metallic, float roughness, float ambientOcclusion,
    const PreparedDirectLight *directLights, std::uint32_t directLightCount,
    const PreparedGiProbeGrid *giGrid = nullptr, float opacity = 1.0f)
{
    // Metallic surfaces carry almost no diffuse response. The hemispheric
    // ambient and the probe-grid GI are diffuse integrals, so they scale by
    // (1 - metallic) exactly like the direct-lighting diffuse below
    // (mesh.frag keeps the twin of this factor). Transparent surfaces
    // scatter only the share they keep off the transmitted path, so their
    // diffuse terms also scale by the opacity: the transmitted share lives
    // in the reflection field (and in the alpha blend for surfaces outside
    // the traced band), never in the surface's own diffuse response.
    metallic         = std::clamp(metallic, 0.0f, 1.0f);
    const glm::vec3 diffuseAlbedo = albedo * (1.0f - metallic) * glm::clamp(opacity, 0.0f, 1.0f);
    const glm::vec3 hemispheric = glm::mix(ambientGround, ambientSky, normal.y * 0.5f + 0.5f) * diffuseAlbedo
                                  * ambientOcclusion;
    glm::vec3       ambient     = hemispheric;
    if (giGrid != nullptr && giGrid->valid)
    {
        glm::vec3 giIrradiance(0.0f);
        float     giWeight         = 0.0f;
        if (giProbeGridIrradiance(*giGrid, worldPosition, normal, giIrradiance, giWeight))
        {
            // The SH2 convolution returns incident irradiance; a Lambertian
            // BRDF turns it into outgoing radiance as albedo / pi * E. The
            // occlusion map darkens the indirect term like the ambient it
            // replaces. The irradiance is already renormalized over the valid
            // probes of the footprint, so a quarter of valid coverage is
            // enough to trust it (probe lattices routinely lose the corners
            // that fall inside nearby geometry); the ambient remains the
            // fallback only while almost no probe is valid.
            const float     giTrust   = glm::smoothstep(0.0f, 0.25f, giWeight);
            const glm::vec3 giAmbient = diffuseAlbedo * (giIrradiance * (1.0f / glm::pi<float>())) * ambientOcclusion;
            ambient                   = glm::mix(hemispheric, giAmbient, giTrust);
        }
    }
    glm::vec3 result = ambient;

    roughness = std::clamp(roughness, minimumRoughness, 1.0f);

    const glm::vec3 view    = glm::normalize(cameraPosition - worldPosition);
    const float     nDotV   = std::max(glm::dot(normal, view), 1.0e-4f);
    const glm::vec3 f0      = glm::mix(dielectricF0, albedo, metallic);
    const float     invPi   = 1.0f / glm::pi<float>();
    const glm::vec3 diffuse = albedo * ((1.0f - metallic) * invPi) * glm::clamp(opacity, 0.0f, 1.0f);

    const float geomK = (roughness + 1.0f) * (roughness + 1.0f) * 0.125f;

    const float sunNDotL = std::max(glm::dot(normal, lightDirection), 0.0f);
    if (sunNDotL > 0.0f && (sunRadiance.x > 0.0f || sunRadiance.y > 0.0f || sunRadiance.z > 0.0f))
    {
        const glm::vec3 halfVector = glm::normalize(view + lightDirection);
        const float     nDotH      = std::max(glm::dot(normal, halfVector), 0.0f);
        const float     vDotH      = std::max(glm::dot(view, halfVector), 0.0f);
        const float     dTerm      = pbrDistributionGGX(nDotH, roughness);
        const float     gTerm      = pbrGeometrySmith(nDotV, sunNDotL, geomK);
        const glm::vec3 fTerm      = pbrFresnelSchlick(vDotH, f0);

        const float     invSpec  = 1.0f / (4.0f * nDotV * sunNDotL + 1.0e-5f);
        const glm::vec3 specular = (dTerm * gTerm * invSpec) * fTerm;
        result += (diffuse + specular) * sunRadiance * sunNDotL;
    }

    const std::uint32_t lightCount = std::min(directLightCount, direct_lighting::maxDirectLights);
    for (std::uint32_t index = 0; index < lightCount; ++index)
    {
        const PreparedDirectLight &light = directLights[index];
        glm::vec3                  localLightDirection;
        float                      attenuation = 1.0f;
        if (light.type == direct_lighting::DirectLightType::Directional)
        {
            localLightDirection = -light.direction;
        }
        else
        {
            const glm::vec3 toLight           = light.position - worldPosition;
            const float     distanceSquared   = std::max(glm::dot(toLight, toLight), 1.0e-4f);
            const float     rangeRatioSquared = distanceSquared * light.inverseRangeSquared;
            if (rangeRatioSquared >= 1.0f)
            {
                continue;
            }
            const float inverseDistance = 1.0f / std::sqrt(distanceSquared);
            localLightDirection         = toLight * inverseDistance;
            const float rangeFade       = 1.0f - rangeRatioSquared * rangeRatioSquared;
            attenuation                 = rangeFade * rangeFade / distanceSquared;
            if (light.type == direct_lighting::DirectLightType::Spot)
            {
                const float coneCosine = glm::dot(-localLightDirection, light.direction);
                const float coneT = std::clamp((coneCosine - light.outerConeCosine)
                                                   / std::max(light.innerConeCosine - light.outerConeCosine, 1.0e-5f),
                    0.0f,
                    1.0f);
                attenuation *= coneT * coneT * (3.0f - 2.0f * coneT);
            }
        }
        const float localNDotL = std::max(glm::dot(normal, localLightDirection), 0.0f);
        if (localNDotL <= 0.0f || attenuation <= 0.0f)
        {
            continue;
        }
        const glm::vec3 halfVector = glm::normalize(view + localLightDirection);
        const float     nDotH      = std::max(glm::dot(normal, halfVector), 0.0f);
        const float     vDotH      = std::max(glm::dot(view, halfVector), 0.0f);

        const float     dTerm    = pbrDistributionGGX(nDotH, roughness);
        const float     gTerm    = pbrGeometrySmith(nDotV, localNDotL, geomK);
        const glm::vec3 fTerm    = pbrFresnelSchlick(vDotH, f0);
        const float     invSpec  = 1.0f / (4.0f * nDotV * localNDotL + 1.0e-5f);
        const glm::vec3 specular = (dTerm * gTerm * invSpec) * fTerm;
        result += (diffuse + specular) * light.radiance * attenuation * localNDotL;
    }
    return result;
}

// Unsigned-byte quantization with round-to-nearest; NaN-safe through the
// clamps (toByte(NaN) would otherwise be undefined through the float->int
// conversion).
inline std::uint8_t toByte(float value)
{
    return static_cast<std::uint8_t>(std::min(1.0f, std::max(0.0f, value)) * 255.0f + 0.5f);
}

// Fast sRGB encode for the CPU rasterizer. glm::pow(c, 1/2.2) costs three
// powf calls per pixel - by far the most expensive op in the shading loop -
// while the output lands in an 8-bit target anyway. A 256-entry nearest
// lookup keeps the error below half an output quantization step.
//
// The tables are C++17 inline variables (dynamically initialized before the
// first use of anything defined alongside them), so the hot pixel path reads
// them without a thread-safe-initialization guard check.
namespace srgb_tables
{
struct FloatEncodeTable
{
    float values[256];
    FloatEncodeTable()
    {
        for (std::uint32_t index = 0; index < 256; ++index)
        {
            values[index] = std::pow(static_cast<float>(index) / 255.0f, 1.0f / 2.2f);
        }
    }
};

inline const FloatEncodeTable floatEncode;
} // namespace srgb_tables

inline float fastSrgbEncode(float channel)
{
    const float clamped = channel < 0.0f ? 0.0f : (channel > 1.0f ? 1.0f : channel);
    return srgb_tables::floatEncode.values[static_cast<std::uint32_t>(clamped * 255.0f + 0.5f)];
}

inline glm::vec3 fastSrgbEncode(const glm::vec3 &color)
{
    return glm::vec3(fastSrgbEncode(color.r), fastSrgbEncode(color.g), fastSrgbEncode(color.b));
}

// The composed output transform for opaque pixels: fastSrgbEncode followed
// by toByte. Both stages quantize through the same 256-step grid, so the
// composition is itself a function of the grid index alone; folding the two
// tables into one byte-per-index table reproduces the exact sequence of
// operations the two-stage form performs (the intermediate float is the float
// table's entry in both cases), bit for bit, while replacing three clamps,
// three multiplies and three conversions per pixel with one lookup per
// channel.
namespace srgb_tables
{
struct ByteEncodeTable
{
    std::uint8_t values[256];
    ByteEncodeTable()
    {
        // Built from the same float encode the two-stage form uses, so every
        // entry equals toByte(fastSrgbEncode(index / 255)).
        for (std::uint32_t index = 0; index < 256; ++index)
        {
            values[index] = toByte(std::pow(static_cast<float>(index) / 255.0f, 1.0f / 2.2f));
        }
    }
};

inline const ByteEncodeTable byteEncode;
} // namespace srgb_tables

// Encodes a tone-mapped linear color as the packed little-endian RGBA word
// the rasterizer commits (R at the lowest address, alpha 255). Channels at or
// outside [0, 1] clamp exactly like the two-stage encode does.
ENGINE_SHADING_INLINE std::uint32_t encodeColorRgba(const glm::vec3 &color)
{
    const std::uint8_t *values = srgb_tables::byteEncode.values;
    const auto channelByte = [values](float channel)
    {
        const float clamped = channel < 0.0f ? 0.0f : (channel > 1.0f ? 1.0f : channel);
        return values[static_cast<std::uint32_t>(clamped * 255.0f + 0.5f)];
    };
    const std::uint32_t red   = channelByte(color.r);
    const std::uint32_t green = channelByte(color.g);
    const std::uint32_t blue  = channelByte(color.b);
    return red | (green << 8) | (blue << 16) | (0xFFu << 24);
}

// Tone mappers mirroring mesh.frag's toneMap(): none / Reinhard / ACES filmic.
ENGINE_SHADING_INLINE glm::vec3 toneMapColor(glm::vec3 color, float exposure, pbr::ToneMapping mapper)
{
    color *= exposure;
    switch (mapper)
    {
    case pbr::ToneMapping::None:
        break;
    case pbr::ToneMapping::Reinhard:
        color = color / (1.0f + color);
        break;
    case pbr::ToneMapping::AcesFilmic:
        color = glm::clamp((color * (2.51f * color + 0.03f)) / (color * (2.43f * color + 0.59f) + 0.14f), 0.0f, 1.0f);
        break;
    }
    return color;
}

// Stores an error code through an optional out-parameter (nullptr allowed).
inline void reportError(RenderError *error, RenderError value)
{
    if (error != nullptr)
    {
        *error = value;
    }
}

// Writes a binary P6 PPM from an RGBA8 raster. flipVertically and
// swapRedBlue adapt the source row and channel order.
inline bool writePpm(const char *filePath, std::uint32_t width, std::uint32_t height, const std::uint8_t *rgba,
    std::size_t sourceStrideBytes, bool flipVertically, bool swapRedBlue, RenderError *error)
{
    if (!util::writePpmRgba(filePath, width, height, rgba, sourceStrideBytes, flipVertically, swapRedBlue))
    {
        reportError(error, RenderError::CreationFailed);
        return false;
    }
    reportError(error, RenderError::None);
    return true;
}

} // namespace renderer
} // namespace engine

