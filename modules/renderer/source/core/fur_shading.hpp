// Shell-fur shading (the CPU mirror of shaders/fur.frag): the per-layer
// strand crop, the Kajiya-Kay strand specular, the strand-field jitter, and
// the full per-layer shading pass. The fur pipeline's fragment stage is
// deliberately cheap (fur cost is fill-rate-bound: every shell layer blends
// over every covered pixel), so the mirror is small. Formulas must stay in
// sync with fur.frag exactly like pbrShade stays in sync with mesh.frag -
// changing one side requires updating the other (headless_backend.cpp
// consumes these).

#pragma once

#include "direct_light_shading.hpp"

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace engine
{
namespace renderer
{

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
    const glm::vec3 view        = glm::normalize(cameraPosition - worldPosition);
    const float     halfLambert = glm::dot(normal, lightDirection) * 0.5f + 0.5f;
    const glm::vec3 diffuse     = strandAlbedo * (halfLambert * halfLambert) * strandShade;
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
        if (!resolveDirectLight(light, worldPosition, localLightDirection, attenuation))
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

} // namespace renderer
} // namespace engine
