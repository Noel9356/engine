#ifndef ENGINE_MODULES_PBR_PBR_HPP
#define ENGINE_MODULES_PBR_PBR_HPP

#include "lighting_capability.hpp"
#include "logging_capability.hpp"
#include "engine/kernel/component_interface.hpp"

#include "light_state.hpp"

#include <memory>
#include <string>

namespace engine
{

// Internal Vulkan state of the pass (defined in pbr.cpp).
class PbrLightingPass;

// Module "pbr": a lighting TECHNIQUE module named after its algorithm, not
// after a rendering category. It owns the scene light state (the
// LightingCapability contract: sun + point lights, mutable by C++ mods at
// runtime) and provides the direct_diffuse layer capability: when the run
// demands it, the module attaches the analytic PBR pass, which evaluates the
// lights with a Cook-Torrance GGX BRDF and analytic SDF soft shadows and
// writes the direct radiance target. A module that is activated only for its
// light state stays an idle provider: mods can still place lights, and
// another lighting technique (or none) consumes them.
class PbrLightingModule final : public ModuleInterface, public LightingCapability
{
public:
    PbrLightingModule();
    ~PbrLightingModule() override;

    const ComponentManifest &manifest() const override
    {
        return componentManifest;
    }

    void construct(ComponentContext &context) override;
    void start() override;
    void stop() override;

    void *capabilityPointerFor(const std::string &capabilityName) override;

    // LightingCapability (single threaded by the engine contract).
    void                        setSunLight(const DirectionalLightDescription &light) override;
    void                        clearSunLight() override;
    bool                        hasSunLight() const override;
    DirectionalLightDescription sunLight() const override;
    PointLightHandle            addPointLight(const PointLightDescription &light) override;
    void                        removePointLight(PointLightHandle handle) override;
    void                        clearPointLights() override;
    std::uint32_t               pointLightCount() const override;
    PointLightDescription       pointLight(std::uint32_t index) const override;

private:
    ComponentManifest                componentManifest;
    LoggerCapability                *logger       = nullptr;
    bool                             passAttached = false;
    std::shared_ptr<PbrLightingPass> pass;
    std::shared_ptr<LightState>      state;
};

} // namespace engine

#endif // ENGINE_MODULES_PBR_PBR_HPP
