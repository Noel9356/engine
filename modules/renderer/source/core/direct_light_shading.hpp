// Direct-light shading inputs shared by both backends: the prepared-light
// record the module copies from the direct_lighting capability and the
// per-point light resolution (direction + attenuation) shared by the PBR and
// shell-fur shading passes. The attenuation formulas must stay in sync with
// the direct-light loops of shaders/mesh.frag and shaders/fur.frag.

#pragma once

#include <engine/direct_lighting/capabilities.hpp>

#include <glm/geometric.hpp>
#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace engine
{
namespace renderer
{

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

// Resolves one prepared direct light at a shaded point: the normalized
// direction toward the light and its multiplicative attenuation (inverse
// square with the (1 - r^2)^2 range fade, times the smoothstep spot cone).
// Returns false when the light contributes nothing here - beyond its range
// or inside the cone's outer cutoff - so both shading passes skip it with
// the same early-out. The operation sequence is identical in both callers,
// which is what keeps the PBR and fur paths bit-compatible with their shader
// twins.
inline bool resolveDirectLight(const PreparedDirectLight &light, const glm::vec3 &worldPosition,
    glm::vec3 &outDirection, float &outAttenuation)
{
    if (light.type == direct_lighting::DirectLightType::Directional)
    {
        outDirection  = -light.direction;
        outAttenuation = 1.0f;
        return outAttenuation > 0.0f;
    }
    const glm::vec3 toLight           = light.position - worldPosition;
    const float     distanceSquared   = std::max(glm::dot(toLight, toLight), 1.0e-4f);
    const float     rangeRatioSquared = distanceSquared * light.inverseRangeSquared;
    if (rangeRatioSquared >= 1.0f)
    {
        return false;
    }
    const float inverseDistance = 1.0f / std::sqrt(distanceSquared);
    outDirection                = toLight * inverseDistance;
    const float rangeFade       = 1.0f - rangeRatioSquared * rangeRatioSquared;
    float       attenuation     = rangeFade * rangeFade / distanceSquared;
    if (light.type == direct_lighting::DirectLightType::Spot)
    {
        const float coneCosine = glm::dot(-outDirection, light.direction);
        const float coneT      = std::clamp((coneCosine - light.outerConeCosine)
                                                / std::max(light.innerConeCosine - light.outerConeCosine, 1.0e-5f),
            0.0f,
            1.0f);
        attenuation *= coneT * coneT * (3.0f - 2.0f * coneT);
    }
    outAttenuation = attenuation;
    return outAttenuation > 0.0f;
}

} // namespace renderer
} // namespace engine
