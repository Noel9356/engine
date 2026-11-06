// Shared math/data helpers used by both backends: the default PBR
// environment, the CPU shading model (kept numerically identical to the
// Vulkan fragment shader), tone mapping, and small error/PPM utilities.

#ifndef ENGINE_RENDER_SHARED_HPP
#define ENGINE_RENDER_SHARED_HPP

#include <engine/direct_lighting/capabilities.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/render/capabilities.hpp>

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
#include <cstdio>
#include <vector>

namespace engine
{
namespace render
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

// Full CPU shading pass: hemispheric ambient, one directional sun, then the
// prepared direct lights (range-faded point/spot attenuation with smoothstep
// cones). Roughness/metallic are clamped here exactly like the shader.
inline glm::vec3 pbrShade(const glm::vec3 &normal, const glm::vec3 &worldPosition, const glm::vec3 &cameraPosition,
    const glm::vec3 &lightDirection, const glm::vec3 &sunRadiance, const glm::vec3 &ambientSky,
    const glm::vec3 &ambientGround, const glm::vec3 &albedo, float metallic, float roughness, float ambientOcclusion,
    const PreparedDirectLight *directLights, std::uint32_t directLightCount)
{
    const glm::vec3 ambient = glm::mix(ambientGround, ambientSky, normal.y * 0.5f + 0.5f) * albedo * ambientOcclusion;
    glm::vec3       result  = ambient;

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
// swapRedBlue adapt to the source's row order and channel order (BGRA
// swapchain readbacks). Always writes *error; CreationFailed on failure.
inline bool writePpm(const char *filePath, std::uint32_t width, std::uint32_t height, const std::uint8_t *rgba,
    std::size_t sourceStrideBytes, bool flipVertically, bool swapRedBlue, RenderError *error)
{
    if (filePath == nullptr)
    {
        reportError(error, RenderError::CreationFailed);
        return false;
    }
    std::FILE *file = std::fopen(filePath, "wb");
    if (file == nullptr)
    {
        reportError(error, RenderError::CreationFailed);
        return false;
    }
    std::fprintf(file, "P6\n%u %u\n255\n", width, height);
    std::vector<std::uint8_t> row(static_cast<std::size_t>(width) * 3);
    for (std::uint32_t line = 0; line < height; ++line)
    {
        const std::uint32_t sourceLine = flipVertically ? height - 1 - line : line;
        const std::uint8_t *source     = rgba + static_cast<std::size_t>(sourceLine) * sourceStrideBytes;
        for (std::uint32_t x = 0; x < width; ++x)
        {
            row[x * 3 + 0] = swapRedBlue ? source[x * 4 + 2] : source[x * 4 + 0];
            row[x * 3 + 1] = source[x * 4 + 1];
            row[x * 3 + 2] = swapRedBlue ? source[x * 4 + 0] : source[x * 4 + 2];
        }
        std::fwrite(row.data(), 1, row.size(), file);
    }
    std::fclose(file);
    reportError(error, RenderError::None);
    return true;
}

} // namespace render
} // namespace engine

#endif
