#ifndef ENGINE_MODULES_SCREEN_SPACE_RADIANCE_CASCADES_HPP
#define ENGINE_MODULES_SCREEN_SPACE_RADIANCE_CASCADES_HPP

#include "logging_capability.hpp"
#include "engine/kernel/component_interface.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace engine
{

class LightingTechniquePass;

// Module "screen_space_radiance_cascades": the SCREEN-SPACE radiance cascades
// lighting TECHNIQUE module, named after its algorithm. A run activates it
// exactly when an active mod demands the indirect_diffuse radiance layer it
// provides; attaching the pass then transports light through the scene
// (surface-anchored probes, interval-partitioned sphere tracing against the
// exact analytic scene SDF) and writes the indirect radiance target. Without
// a light provider the pass degrades to emissive + sky transport; the
// composite falls back to direct + emissive + sky only.
class ScreenSpaceRadianceCascadesModule final : public ModuleInterface
{
public:
    ScreenSpaceRadianceCascadesModule();
    ~ScreenSpaceRadianceCascadesModule() override;

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
    LoggerCapability                      *logger                = nullptr;
    bool                                   passAttached          = false;
    bool                                   lightingAvailable     = false;
    // Start-log reporting copies of the configured [lighting.screen_space_radiance_cascades]
    // mapping and bounces (the pass keeps the authoritative settings).
    std::string                            configuredMappingName = "octahedron";
    std::uint32_t                          configuredBounces     = 1;
    std::shared_ptr<LightingTechniquePass> pass;
};

} // namespace engine

#endif // ENGINE_MODULES_SCREEN_SPACE_RADIANCE_CASCADES_HPP
