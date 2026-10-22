#ifndef ENGINE_MODULES_RAY_REFLECTIONS_HPP
#define ENGINE_MODULES_RAY_REFLECTIONS_HPP

#include "engine/kernel/component_interface.hpp"

#include <memory>

namespace engine
{

class LightingTechniquePass;
class LoggerCapability;

// Module "ray_reflections": ray traced glossy reflections against the exact
// analytic scene SDF, attached to the renderer as the "ray_reflections"
// lighting technique (algorithm naming rule, like "pbr" and
// "radiance_cascades"). Writes the renderer's indirect specular radiance
// layer; the composite sums it after direct light and diffuse GI.
class RayReflectionsModule final : public ModuleInterface
{
public:
    RayReflectionsModule();
    ~RayReflectionsModule() override;

    const ComponentManifest& manifest() const override
    {
        return componentManifest;
    }

    void construct(ComponentContext& context) override;
    void start() override;
    void stop() override;

    void* capabilityPointerFor(const std::string& capabilityName) override;

private:
    ComponentManifest componentManifest;
    LoggerCapability* logger = nullptr;
    std::shared_ptr<LightingTechniquePass> pass;
    bool passAttached = false;
};

} // namespace engine

#endif // ENGINE_MODULES_RAY_REFLECTIONS_HPP
