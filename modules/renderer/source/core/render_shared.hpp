// Shared math/data helpers used by both backends: the default PBR
// environment, the CPU shading model (kept numerically identical to the
// Vulkan fragment shader), tone mapping, and small error/PPM utilities.

#ifndef ENGINE_RENDERER_SHARED_HPP
#define ENGINE_RENDERER_SHARED_HPP

#include <engine/direct_lighting/capabilities.hpp>
#include <engine/lumen_gi/capabilities.hpp>
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
#include <vector>

namespace engine
{
namespace renderer
{

// Device selection bias used when a renderer is created.
enum class DevicePreference : std::uint32_t
{
    Auto, // any device, discrete preferred
    Discrete,
    Integrated
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

// Picks the lattice level for a world position: level 0 inside its
// half-cell-extended bounds, level 1 everywhere else (its trilinear lookup
// clamps to the boundary probes, so distant geometry still receives the
// coarse ambient). Returns false only when the snapshot cannot serve a
// lookup at all. Uses the descriptors built by buildProbeLevels() when
// present and falls back to building them locally otherwise.
inline bool giProbeLevelFor(const GiProbeGrid &grid, const glm::vec3 &position, GiProbeLevel &outLevel)
{
    GiProbeLevel level0;
    GiProbeLevel level1;
    if (grid.levelsBuilt)
    {
        level0 = grid.level0;
        level1 = grid.level1;
    }
    else
    {
        const lumen_gi::GiProbeHeader &header = grid.header;
        if (header.level0Dimensions[0] < 2u || header.level1Dimensions[0] < 2u)
        {
            return false;
        }
        const std::uint32_t level0Count = header.level1ProbeOffset - header.level0ProbeOffset;
        if (header.level0ProbeOffset >= grid.probeCount || level0Count == 0
            || header.level1ProbeOffset >= grid.probeCount)
        {
            return false;
        }
        level0.anchor  = glm::make_vec3(grid.probes[header.level0ProbeOffset].position);
        level0.dims    = glm::ivec3(static_cast<int>(header.level0Dimensions[0]),
            static_cast<int>(header.level0Dimensions[1]), static_cast<int>(header.level0Dimensions[2]));
        level0.spacing = header.level0Spacing;
        level0.offset  = header.level0ProbeOffset;
        level1.anchor  = glm::make_vec3(grid.probes[header.level1ProbeOffset].position);
        level1.dims    = glm::ivec3(static_cast<int>(header.level1Dimensions[0]),
            static_cast<int>(header.level1Dimensions[1]), static_cast<int>(header.level1Dimensions[2]));
        level1.spacing = header.level1Spacing;
        level1.offset  = header.level1ProbeOffset;
    }

    const glm::vec3 local        = position - level0.anchor;
    const glm::vec3 extent0      = glm::vec3(level0.dims - 1) * level0.spacing;
    const float     halfCell0    = level0.spacing * 0.5f;
    const bool      insideLevel0 = glm::all(glm::bvec4(local.x >= -halfCell0, local.y >= -halfCell0,
        local.z >= -halfCell0, true))
                                && glm::all(glm::bvec4(local.x <= extent0.x + halfCell0, local.y <= extent0.y + halfCell0,
                                    local.z <= extent0.z + halfCell0, true));
    outLevel = insideLevel0 ? level0 : level1;
    return true;
}

// Cosine-lobed SH2 irradiance at one probe (the same convolution as
// lumen_gi::evaluateProbeIrradiance; kept textually in sync because the GLSL
// mirror in shaders/mesh.frag must match this CPU path like the rest of the
// shading model). sh2 is 27 floats in the GiProbe [basis][channel] order.
inline glm::vec3 giEvaluateSh2Irradiance(const float sh2[27], const glm::vec3 &normal)
{
    constexpr float k0 = 3.141592654f;  // pi      (band 0)
    constexpr float k1 = 2.094395102f;  // 2pi/3   (band 1)
    constexpr float k2 = 0.785398163f;  // pi/4    (band 2)

    const float x  = normal.x;
    const float y  = normal.y;
    const float z  = normal.z;
    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;

    glm::vec3 irradiance(0.0f);
    for (std::uint32_t channel = 0; channel < 3; ++channel)
    {
        irradiance[channel] = k0 * sh2[channel]
                            + k1 * (sh2[3 + channel] * y + sh2[6 + channel] * z + sh2[9 + channel] * x)
                            + k2 * (sh2[12 + channel] * x * y + sh2[15 + channel] * y * z
                                    + sh2[18 + channel] * (3.0f * zz - 1.0f) + sh2[21 + channel] * x * z
                                    + sh2[24 + channel] * (xx - yy));
    }
    return irradiance;
}

// Trilinear probe-grid irradiance for one shaded point, validity-weighted.
// Returns false when the grid holds no valid probes along this footprint (the
// caller keeps its legacy ambient); outWeight is the validity coverage of the
// footprint in 0..1, used to blend GI against the hemispheric ambient.
inline bool giProbeGridIrradiance(const GiProbeGrid &grid, const glm::vec3 &position, const glm::vec3 &normal,
    glm::vec3 &outIrradiance, float &outWeight)
{
    outIrradiance = glm::vec3(0.0f);
    outWeight     = 0.0f;
    GiProbeLevel  level;
    if (!giProbeLevelFor(grid, position, level))
    {
        return false;
    }

    const glm::vec3 cell     = (position - level.anchor) / level.spacing;
    const glm::ivec3 base    = glm::clamp(glm::ivec3(glm::floor(cell)), glm::ivec3(0), level.dims - glm::ivec3(2));
    const glm::vec3 fraction = glm::clamp(cell - glm::vec3(base), 0.0f, 1.0f);

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
                irradianceSum += weight * giEvaluateSh2Irradiance(&probe.sh2[0][0], normal);
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

// Full CPU shading pass: hemispheric ambient (or the lumen_gi probe-grid
// indirect term blended over it by probe validity), one directional sun, then
// the prepared direct lights (range-faded point/spot attenuation with
// smoothstep cones). Roughness/metallic are clamped here exactly like the
// shader. giGrid is the borrowed probe-grid snapshot; nullptr keeps the
// legacy ambient path bit-identical.
inline glm::vec3 pbrShade(const glm::vec3 &normal, const glm::vec3 &worldPosition, const glm::vec3 &cameraPosition,
    const glm::vec3 &lightDirection, const glm::vec3 &sunRadiance, const glm::vec3 &ambientSky,
    const glm::vec3 &ambientGround, const glm::vec3 &albedo, float metallic, float roughness, float ambientOcclusion,
    const PreparedDirectLight *directLights, std::uint32_t directLightCount, const GiProbeGrid *giGrid = nullptr)
{
    const glm::vec3 hemispheric = glm::mix(ambientGround, ambientSky, normal.y * 0.5f + 0.5f) * albedo * ambientOcclusion;
    glm::vec3       ambient     = hemispheric;
    if (giGrid != nullptr && giGrid->valid())
    {
        glm::vec3 giIrradiance(0.0f);
        float     giWeight         = 0.0f;
        if (giProbeGridIrradiance(*giGrid, worldPosition, normal, giIrradiance, giWeight))
        {
            // The SH2 convolution returns incident irradiance; a Lambertian
            // BRDF turns it into outgoing radiance as albedo / pi * E. The
            // occlusion map darkens the indirect term like the ambient it
            // replaces. Partial probe coverage blends back to hemispheric.
            const glm::vec3 giAmbient = albedo * (giIrradiance * (1.0f / glm::pi<float>())) * ambientOcclusion;
            ambient                   = glm::mix(hemispheric, giAmbient, giWeight);
        }
    }
    glm::vec3 result = ambient;

    metallic  = std::clamp(metallic, 0.0f, 1.0f);
    roughness = std::clamp(roughness, minimumRoughness, 1.0f);

    const glm::vec3 view    = glm::normalize(cameraPosition - worldPosition);
    const float     nDotV   = std::max(glm::dot(normal, view), 1.0e-4f);
    const glm::vec3 f0      = glm::mix(dielectricF0, albedo, metallic);
    const float     invPi   = 1.0f / glm::pi<float>();
    const glm::vec3 diffuse = albedo * ((1.0f - metallic) * invPi);

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

// Fast sRGB encode for the CPU rasterizer. glm::pow(c, 1/2.2) costs three
// powf calls per pixel - by far the most expensive op in the shading loop -
// while the output lands in an 8-bit target anyway. A 256-entry nearest
// lookup keeps the error below half an output quantization step.
inline float fastSrgbEncode(float channel)
{
    struct SrgbTable
    {
        float values[256];
        SrgbTable()
        {
            for (std::uint32_t index = 0; index < 256; ++index)
            {
                values[index] = std::pow(static_cast<float>(index) / 255.0f, 1.0f / 2.2f);
            }
        }
    };
    static const SrgbTable table;
    const float            clamped = channel < 0.0f ? 0.0f : (channel > 1.0f ? 1.0f : channel);
    return table.values[static_cast<std::uint32_t>(clamped * 255.0f + 0.5f)];
}

inline glm::vec3 fastSrgbEncode(const glm::vec3 &color)
{
    return glm::vec3(fastSrgbEncode(color.r), fastSrgbEncode(color.g), fastSrgbEncode(color.b));
}

// Tone mappers mirroring mesh.frag's toneMap(): none / Reinhard / ACES filmic.
inline glm::vec3 toneMapColor(glm::vec3 color, float exposure, pbr::ToneMapping mapper)
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

inline std::uint8_t toByte(float value)
{
    return static_cast<std::uint8_t>(std::min(1.0f, std::max(0.0f, value)) * 255.0f + 0.5f);
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

#endif
