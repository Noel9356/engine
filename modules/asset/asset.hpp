#ifndef ENGINE_MODULES_ASSET_HPP
#define ENGINE_MODULES_ASSET_HPP

#include "asset_capability.hpp"
#include "logging_capability.hpp"
#include "scene_capability.hpp"
#include "engine/kernel/component_interface.hpp"
#include "engine/kernel/event_bus.hpp"

#include <string>

namespace engine
{

// Module "asset": content import into the scene registry. The first source is
// glTF 2.0 (see the module README for the mapping contract): node hierarchy,
// world transforms, per-primitive bounds and material factors become scene
// registry objects, and every consumer of the registry - renderer, physics,
// animation, scripts - sees the imported level exactly like any other
// runtime-spawned content.
class AssetModule final : public ModuleInterface, public AssetCapability
{
public:
    AssetModule();
    ~AssetModule() override;

    const ComponentManifest &manifest() const override
    {
        return componentManifest;
    }

    void construct(ComponentContext &context) override;
    void start() override;
    void stop() override;

    void *capabilityPointerFor(const std::string &capabilityName) override;

    // AssetCapability.
    std::uint32_t importGltfIntoScene(const std::string &path, std::string &errorReport) override;

private:
    ComponentManifest componentManifest;
    ComponentContext *boundContext   = nullptr;
    LoggerCapability *logger         = nullptr;
    SceneCapability  *sceneCapability = nullptr;
};

} // namespace engine

#endif // ENGINE_MODULES_ASSET_HPP
