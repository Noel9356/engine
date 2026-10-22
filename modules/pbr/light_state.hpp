#ifndef ENGINE_MODULES_PBR_LIGHT_STATE_HPP
#define ENGINE_MODULES_PBR_LIGHT_STATE_HPP

// Shared light state between the module (capability implementation) and the
// PBR pass (per-frame GPU upload). Owned by the module through shared_ptr so
// the pass survives a module stop until the renderer detaches it.

#include "engine/capabilities/lighting_contract.hpp"
#include "light_uniform.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace engine
{

struct LightState
{
    bool sunEnabled = false;
    DirectionalLightDescription sun;
    std::vector<PointLightDescription> pointLights;
    bool shadowsEnabled = true;
    float penumbra = 12.0f;
    std::int32_t shadowSteps = 24;

    void fillUniform(GpuLightUniform& uniform) const
    {
        uniform = GpuLightUniform{};
        fillSunAndPointLights(uniform, sunEnabled, sun, pointLights.data(),
                              static_cast<std::uint32_t>(pointLights.size()));
        uniform.lightCountsAndFlags = {static_cast<std::int32_t>(std::min<std::size_t>(pointLights.size(), maxPointLights)),
                                       shadowsEnabled ? 1 : 0, shadowSteps, 0};
    }
};

} // namespace engine

#endif // ENGINE_MODULES_PBR_LIGHT_STATE_HPP
