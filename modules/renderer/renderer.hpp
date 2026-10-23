#ifndef ENGINE_MODULES_RENDERER_RENDERER_HPP
#define ENGINE_MODULES_RENDERER_RENDERER_HPP

#include "logging_capability.hpp"
#include "renderer_capability.hpp"
#include "window_capability.hpp"
#include "engine/kernel/component_interface.hpp"
#include "engine/kernel/event_bus.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace engine
{

class LoggerCapability;
class LightingTechniquePass;

// Internal Vulkan state; defined in the implementation file so the heavy
// vulkan-hpp headers never leak into modules that merely reference the
// renderer capability.
struct RendererState;

// Module "renderer": provides the RendererCapability on top of Vulkan 1.3
// through the vk::raii interfaces. The module owns the instance, device,
// swapchain, the scene rasterization (from the shared [scene] configuration)
// and the frame pipeline: geometry pass, attached lighting techniques,
// composite. Lighting techniques are separate modules that attach through the
// method attachLightingTechnique and declare which radiance layers they write;
// which techniques exist in a run is decided by capability demand, not by any
// list here. When none is attached the renderer renders unlit (albedo +
// emissive + sky), so every lighting layer can be left out independently.
class RendererModule final : public ModuleInterface, public RendererCapability
{
public:
    RendererModule();
    ~RendererModule() override;

    const ComponentManifest &manifest() const override
    {
        return componentManifest;
    }

    void construct(ComponentContext &context) override;
    void start() override;
    void stop() override;

    void *capabilityPointerFor(const std::string &capabilityName) override;

    void renderFrame() override;
    void waitUntilIdle() override;
    void requestFrameCapture() override;
    bool validationErrorsObserved() const override;

    // In-tree technique registration for lighting modules
    // (render_interface.hpp). Safe to call during the construct phase, before
    // start; the renderer records the techniques into every frame after its
    // geometry pass, ordered by the physical layer they write (direct radiance
    // before indirect). Re-attaching a technique with the same name replaces
    // the previous one.
    void attachLightingTechnique(const std::shared_ptr<LightingTechniquePass> &pass);

private:
    void handleWindowResized(const WindowResizedEvent &event);

    ComponentManifest componentManifest;
    LoggerCapability *logger           = nullptr;
    WindowCapability *windowCapability = nullptr;
    // Frame events are published through channel references cached at
    // construct: the bus anchors channels on the heap, and the frame loop
    // publishes with zero lookups (see event_bus.hpp, hot path note).
    EventChannel<FrameBeginEvent>    *frameBeginChannel    = nullptr;
    EventChannel<FrameRenderedEvent> *frameRenderedChannel = nullptr;
    EventSubscription                 resizeSubscription;
    std::unique_ptr<RendererState>    state;
};

} // namespace engine

#endif // ENGINE_MODULES_RENDERER_RENDERER_HPP
