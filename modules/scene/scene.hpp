#ifndef ENGINE_MODULES_SCENE_HPP
#define ENGINE_MODULES_SCENE_HPP

#include "logging_capability.hpp"
#include "scene_capability.hpp"
#include "engine/kernel/component_interface.hpp"
#include "engine/kernel/event_bus.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace engine
{

// Module "scene": provides the SceneCapability - the runtime object registry.
//
// The registry starts empty. During the renderer's start phase the static
// world (the parsed [scene] configuration) is pushed into it, so every
// consumer sees one world regardless of where its objects came from; script
// mods and modules then mutate through the capability, and every mutation
// publishes an event on the bus. The renderer subscribes and rebuilds its
// GPU scene data incrementally (content-only re-upload of the fixed-capacity
// primitive and material buffers); the physics module writes stepped body
// transforms back through the same interface.
//
// The module is pure logic: no window, no Vulkan, no scripting runtime. That
// keeps it usable in minimal builds (a headless simulation with physics and
// animation but no renderer never loads a frame loop).
class SceneModule final : public ModuleInterface, public SceneCapability
{
public:
    SceneModule();
    ~SceneModule() override;

    const ComponentManifest &manifest() const override
    {
        return componentManifest;
    }

    void construct(ComponentContext &context) override;
    void start() override;
    void stop() override;

    void *capabilityPointerFor(const std::string &capabilityName) override;

    // SceneCapability (the module IS the implementation behind the family
    // member "scene.registry").
    ObjectHandle addObject(const SceneObjectDescriptor &object) override;
    bool         removeObject(ObjectHandle handle) override;
    bool         setObjectTransform(ObjectHandle handle, const std::array<float, 3> &position, float rotationYRadians)
        override;
    bool         setObjectMaterial(ObjectHandle handle, const SceneObjectDescriptor &material) override;
    bool         object(ObjectHandle handle, SceneObjectDescriptor &outObject) const override;
    std::uint32_t objectCount() const override;
    std::vector<SceneObjectDescriptor> objectSnapshot() const override;

private:
    struct RegistryEntry
    {
        ObjectHandle         handle = invalidObject;
        SceneObjectDescriptor object;
    };

    // Publishes the change event for one object through the cached channel.
    template <typename EventType> void publishEvent(
        const EventType &event
    );

    ComponentManifest   componentManifest;
    ComponentContext   *boundContext = nullptr;
    LoggerCapability   *logger       = nullptr;

    // Channel references cached at construct (the bus anchors channels on the
    // heap; the registry's hot path publishes with zero lookups).
    EventChannel<SceneObjectAddedEvent>    *addedChannel    = nullptr;
    EventChannel<SceneObjectRemovedEvent>  *removedChannel  = nullptr;
    EventChannel<SceneObjectChangedEvent>  *changedChannel  = nullptr;

    // The registry: live objects keyed by handle. An unordered_map keeps
    // add/remove/update constant-cost; the snapshot sorts by handle so
    // consumers see a deterministic order.
    std::unordered_map<ObjectHandle, RegistryEntry> registry;
    ObjectHandle                                    nextHandle = invalidObject + 1;
};

} // namespace engine

#endif // ENGINE_MODULES_SCENE_HPP
