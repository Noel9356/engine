#ifndef ENGINE_MODULES_RADIANCE_CASCADES_HPP
#define ENGINE_MODULES_RADIANCE_CASCADES_HPP

#include "logging_capability.hpp"
#include "engine/kernel/component_interface.hpp"

#include <memory>
#include <string>

namespace engine
{

class LightingTechniquePass;

// Module "radiance_cascades": a lighting TECHNIQUE module named after its
// algorithm. When [lighting] techniques lists "radiance_cascades", it attaches
// the radiance cascades pass, which transports light through the scene
// (surface-anchored probes, interval-partitioned sphere tracing against the
// exact analytic scene SDF) and writes the indirect radiance target. Without
// a light provider the pass degrades to emissive + sky transport; the
// composite falls back to direct + emissive + sky only.
class RadianceCascadesModule final : public ModuleInterface
{
public:
    RadianceCascadesModule();
    ~RadianceCascadesModule() override;

    const ComponentManifest &manifest() const override
    {
        return componentManifest;
    }

    void construct(ComponentContext &context) override;
    void start() override;
    void stop() override;

    void *capabilityPointerFor(const std::string &capabilityName) override;

private:
    ComponentManifest                      componentManifest;
    LoggerCapability                      *logger            = nullptr;
    bool                                   passAttached      = false;
    bool                                   lightingAvailable = false;
    std::shared_ptr<LightingTechniquePass> pass;
};

} // namespace engine

#endif // ENGINE_MODULES_RADIANCE_CASCADES_HPP
