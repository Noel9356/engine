#ifndef ENGINE_MODULES_RENDERER_LIGHT_UNIFORM_HPP
#define ENGINE_MODULES_RENDERER_LIGHT_UNIFORM_HPP

// std140 mirror of the LightUniform block in lighting_common.glsl. Both
// lighting modules upload their own copy of this block per frame: the direct
// module from its own state (it implements LightingCapability), the GI module
// from the acquired capability (so relight stays consistent with whatever
// lights exist at runtime).

#include <array>
#include <cmath>
#include <cstdint>

#include "engine/capabilities/lighting_contract.hpp"
#include "scene_description.hpp" // maxPointLights

namespace engine
{

struct GpuLightUniform
{
    // xyz direction (towards the scene), w intensity
    std::array<float, 4> sunDirectionIntensity{0.0f, -1.0f, 0.0f, 0.0f};
    // rgb color, w enabled
    std::array<float, 4> sunColorEnabled{1.0f, 1.0f, 1.0f, 0.0f};
    // x point light count, y shadows enabled, z shadow steps, w unused
    std::array<std::int32_t, 4>                      lightCountsAndFlags{0, 0, 24, 0};
    std::array<std::array<float, 4>, maxPointLights> pointPositionRadius{};
    std::array<std::array<float, 4>, maxPointLights> pointColorIntensity{};
};

static_assert(sizeof(GpuLightUniform) == 3 * 16 + 2 * maxPointLights * 16, "std140 layout drift");

// Fills the sun and point-light fields of one uniform block, normalizing the
// sun direction (a zero-length direction keeps intensity 0 instead of NaN).
// One definition: the pbr module fills from its own LightState, the GI and
// reflection relights fill from the LightingCapability - all three previously
// carried byte-identical copies of this math. The flags field is left to the
// caller: shadows on/off and the step count are per-technique policy.
inline void fillSunAndPointLights(
    GpuLightUniform                   &uniform,
    bool                               sunEnabled,
    const DirectionalLightDescription &sun,
    const PointLightDescription       *pointLights,
    std::uint32_t                      pointLightCount
)
{
    if (sunEnabled)
    {
        float directionLength = std::sqrt(
            sun.direction[0] * sun.direction[0] + sun.direction[1] * sun.direction[1]
            + sun.direction[2] * sun.direction[2]
        );
        if (directionLength < 1e-6f)
        {
            directionLength = 1.0f;
        }
        uniform.sunDirectionIntensity = {
            sun.direction[0] / directionLength,
            sun.direction[1] / directionLength,
            sun.direction[2] / directionLength,
            sun.intensity
        };
        uniform.sunColorEnabled = {sun.color[0], sun.color[1], sun.color[2], 1.0f};
    }
    pointLightCount = std::min(pointLightCount, maxPointLights);
    for (std::uint32_t index = 0; index < pointLightCount; ++index)
    {
        const PointLightDescription &light = pointLights[index];
        uniform.pointPositionRadius[index] = {light.position[0], light.position[1], light.position[2], light.radius};
        uniform.pointColorIntensity[index] = {light.color[0], light.color[1], light.color[2], light.intensity};
    }
}

} // namespace engine

#endif // ENGINE_MODULES_RENDERER_LIGHT_UNIFORM_HPP
