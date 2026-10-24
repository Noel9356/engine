#include "scene.hpp"

#include <algorithm>

namespace engine
{

SceneModule::SceneModule()
{
    componentManifest.name = "scene";
    componentManifest.kind = ComponentKind::Module;
    // The implementation-bound member name: a future scene provider (a
    // streamed-world service, an editor bridge) declares its own member of
    // the "scene" family and the [providers] configuration picks between us.
    componentManifest.providedCapabilities = {"scene.registry"};
    componentManifest.requiredCapabilities = {"logging"};
    componentManifest.requiredPermissions  = {"capability.logging"};
}

SceneModule::~SceneModule()
{
    stop();
}

void SceneModule::construct(
    ComponentContext &context
)
{
    boundContext = &context;
    logger       = &context.acquireCapability<LoggerCapability>();

    // Channel references cached once: the registry publishes through these
    // for the rest of the run with zero lookups (event_bus.hpp hot path
    // note). Consumers - the renderer, physics, animation - subscribe to the
    // same channels and coalesce the events into per-frame GPU or simulation
    // work.
    EventBus                           &bus    = context.eventBus();
    addedChannel    = &bus.channel<SceneObjectAddedEvent>();
    removedChannel  = &bus.channel<SceneObjectRemovedEvent>();
    changedChannel  = &bus.channel<SceneObjectChangedEvent>();
}

void SceneModule::start()
{
    logger->logMessage(
        LogLevel::Information,
        "scene module started (" + std::to_string(registry.size()) + " object(s) registered)"
    );
}

void SceneModule::stop()
{
    registry.clear();
    boundContext = nullptr;
    logger       = nullptr;
    addedChannel   = nullptr;
    removedChannel = nullptr;
    changedChannel = nullptr;
}

void *SceneModule::capabilityPointerFor(
    const std::string &capabilityName
)
{
    // The kernel validates and resolves through the DECLARED member name
    // (the construct phase asks for every name in providedCapabilities, and
    // the acquisition walk passes the resolved member, not the family name
    // the consumer requested).
    if (capabilityName == "scene.registry")
    {
        return static_cast<SceneCapability *>(this);
    }
    return nullptr;
}

SceneCapability::ObjectHandle SceneModule::addObject(
    const SceneObjectDescriptor &object
)
{
    if (registry.size() >= maxObjects)
    {
        // The precise failure: the GPU primitive buffer is allocated once at
        // this capacity (see scene_capability.hpp), so silently wrapping or
        // replacing would corrupt a live object. Callers see invalidObject
        // and decide what a full registry means for them.
        if (logger != nullptr)
        {
            logger->logMessage(
                LogLevel::Error,
                "scene module: refused to register object " + std::to_string(registry.size() + 1)
                    + ": the registry is at capacity (" + std::to_string(maxObjects) + " objects)"
            );
        }
        return invalidObject;
    }
    const ObjectHandle handle = nextHandle++;
    RegistryEntry      entry;
    entry.handle = handle;
    entry.object = object;
    registry.emplace(handle, entry);
    if (logger != nullptr)
    {
        logger->logMessage(
            LogLevel::Debug,
            "scene module: object " + std::to_string(handle) + " registered (type "
                + std::to_string(object.type) + ", " + std::to_string(registry.size()) + " live)"
        );
    }
    if (addedChannel != nullptr)
    {
        addedChannel->publish(SceneObjectAddedEvent{.objectHandle = handle});
    }
    return handle;
}

bool SceneModule::removeObject(
    ObjectHandle handle
)
{
    const auto found = registry.find(handle);
    if (found == registry.end())
    {
        return false;
    }
    registry.erase(found);
    if (logger != nullptr)
    {
        logger->logMessage(
            LogLevel::Debug,
            "scene module: object " + std::to_string(handle) + " removed (" + std::to_string(registry.size())
                + " live)"
        );
    }
    if (removedChannel != nullptr)
    {
        removedChannel->publish(SceneObjectRemovedEvent{.objectHandle = handle});
    }
    return true;
}

bool SceneModule::setObjectTransform(
    ObjectHandle handle, const std::array<float, 3> &position, float rotationYRadians
)
{
    const auto found = registry.find(handle);
    if (found == registry.end())
    {
        return false;
    }
    found->second.object.position         = position;
    found->second.object.rotationYRadians = rotationYRadians;
    if (changedChannel != nullptr)
    {
        changedChannel->publish(SceneObjectChangedEvent{.objectHandle = handle});
    }
    return true;
}

bool SceneModule::setObjectMaterial(
    ObjectHandle handle, const SceneObjectDescriptor &material
)
{
    const auto found = registry.find(handle);
    if (found == registry.end())
    {
        return false;
    }
    // Only the material side of the descriptor: the object's identity (type)
    // and transform stay untouched - a material edit must never teleport an
    // object or turn a sphere into a box.
    found->second.object.albedo           = material.albedo;
    found->second.object.roughness        = material.roughness;
    found->second.object.metallic         = material.metallic;
    found->second.object.emissiveStrength = material.emissiveStrength;
    found->second.object.textureSet       = material.textureSet;
    found->second.object.uvScale          = material.uvScale;
    if (changedChannel != nullptr)
    {
        changedChannel->publish(SceneObjectChangedEvent{.objectHandle = handle});
    }
    return true;
}

bool SceneModule::object(
    ObjectHandle handle, SceneObjectDescriptor &outObject
) const
{
    const auto found = registry.find(handle);
    if (found == registry.end())
    {
        return false;
    }
    outObject = found->second.object;
    return true;
}

std::uint32_t SceneModule::objectCount() const
{
    return static_cast<std::uint32_t>(registry.size());
}

std::vector<SceneObjectDescriptor> SceneModule::objectSnapshot() const
{
    // Ascending handle order: the renderer's per-primitive material table and
    // the geometry pass both index by snapshot position, so the order must be
    // stable across frames regardless of unordered_map iteration order.
    std::vector<const RegistryEntry *> ordered;
    ordered.reserve(registry.size());
    for (const auto &entry : registry)
    {
        ordered.push_back(&entry.second);
    }
    std::sort(
        ordered.begin(),
        ordered.end(),
        [](const RegistryEntry *left, const RegistryEntry *right) { return left->handle < right->handle; }
    );
    std::vector<SceneObjectDescriptor> snapshot;
    snapshot.reserve(ordered.size());
    for (const RegistryEntry *entry : ordered)
    {
        snapshot.push_back(entry->object);
    }
    return snapshot;
}

} // namespace engine
