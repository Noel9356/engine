#include "scene.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

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
    componentManifest.knownConfigKeys      = {"scene.max_objects"};
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

    // The configurable object capacity. Clamped once, here: a run whose
    // configuration asks beyond the GPU allocation (gpuMaxObjects - the
    // renderer's primitive/material buffers) would otherwise corrupt the
    // registry's contract halfway through the run.
    const std::int64_t configured = context.configurationIntegerInSection("scene", "max_objects")
                                        .value_or(static_cast<std::int64_t>(defaultMaxObjects));
    if (configured < 1 || configured > static_cast<std::int64_t>(gpuMaxObjects))
    {
        throw std::runtime_error(
            "scene module: [scene] max_objects must be between 1 and " + std::to_string(gpuMaxObjects)
                + " (the GPU primitive buffer's capacity), got " + std::to_string(configured)
        );
    }
    configuredObjectCapacity = static_cast<std::uint32_t>(configured);

    // Channel references cached once: the registry publishes through these
    // for the rest of the run with zero lookups (event_bus.hpp hot path
    // note). Consumers - the renderer, physics, animation - subscribe to the
    // same channels and coalesce the events into per-frame GPU or simulation
    // work.
    EventBus                           &bus    = context.eventBus();
    addedChannel    = &bus.channel<SceneObjectAddedEvent>();
    removedChannel  = &bus.channel<SceneObjectRemovedEvent>();
    changedChannel  = &bus.channel<SceneObjectChangedEvent>();
    meshAddedChannel = &bus.channel<SceneMeshAddedEvent>();
}

void SceneModule::start()
{
    logger->logMessage(
        LogLevel::Information,
        "scene module started (" + std::to_string(registry.size()) + " object(s) registered, capacity "
            + std::to_string(configuredObjectCapacity) + ", " + std::to_string(meshRegistry.size())
            + " mesh(es) registered)"
    );
}

void SceneModule::stop()
{
    registry.clear();
    meshRegistry.clear();
    meshHandlesByName.clear();
    nextMeshHandle = invalidMesh + 1;
    boundContext = nullptr;
    logger       = nullptr;
    addedChannel   = nullptr;
    removedChannel = nullptr;
    changedChannel = nullptr;
    meshAddedChannel = nullptr;
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
    if (registry.size() >= configuredObjectCapacity)
    {
        // The precise failure: the GPU primitive buffer is allocated at the
        // engine-wide maximum (see scene_capability.hpp), so the configured
        // capacity is what keeps the SDF's per-frame cost - linear in the
        // object count - honest. Callers see invalidObject and decide what a
        // full registry means for them.
        if (logger != nullptr)
        {
            logger->logMessage(
                LogLevel::Error,
                "scene module: refused to register object " + std::to_string(registry.size() + 1)
                    + ": the registry is at capacity (" + std::to_string(configuredObjectCapacity)
                    + " objects; raise [scene] max_objects, at most " + std::to_string(gpuMaxObjects) + ")"
            );
        }
        return invalidObject;
    }
    // Mesh instances must name a registered mesh: a mesh object with no mesh
    // would rasterize nothing and trace a zero-size proxy - a silent visual
    // bug. The registry knows every mesh handle, so the check is exact.
    if (object.type == scene_object_type::mesh
        && (object.meshHandle == invalidMesh || meshRegistry.find(object.meshHandle) == meshRegistry.end()))
    {
        if (logger != nullptr)
        {
            logger->logMessage(
                LogLevel::Error,
                "scene module: refused to register mesh object: meshHandle " + std::to_string(object.meshHandle)
                    + " names no registered mesh (register it through addMesh first)"
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

std::uint32_t SceneModule::objectCapacity() const
{
    return configuredObjectCapacity;
}

SceneCapability::MeshHandle SceneModule::addMesh(
    const std::string &name, const SceneMeshDescriptor &mesh
)
{
    // Idempotent registration: the same name is the same asset. An importer
    // that runs twice (or two objects sharing one mesh) must not duplicate
    // the GPU buffer.
    const auto known = meshHandlesByName.find(name);
    if (known != meshHandlesByName.end())
    {
        return known->second;
    }
    const std::size_t vertexFloats = mesh.vertices.size();
    const bool malformed = vertexFloats == 0
        || vertexFloats % meshFloatsPerVertex != 0
        || mesh.indices.size() < 3
        || mesh.indices.size() % 3 != 0;
    if (malformed)
    {
        if (logger != nullptr)
        {
            logger->logMessage(
                LogLevel::Error,
                "scene module: refused to register mesh '" + name + "': the vertex array must be non-empty and a "
                    + "multiple of " + std::to_string(meshFloatsPerVertex) + " floats, the index array a multiple of 3"
            );
        }
        return invalidMesh;
    }
    if (meshRegistry.size() >= maxMeshes)
    {
        if (logger != nullptr)
        {
            logger->logMessage(
                LogLevel::Error,
                "scene module: refused to register mesh '" + name + "': the mesh registry is at capacity ("
                    + std::to_string(maxMeshes) + " meshes)"
            );
        }
        return invalidMesh;
    }
    const std::size_t vertexCount = vertexFloats / meshFloatsPerVertex;
    if (vertexCount > maxMeshVertices)
    {
        if (logger != nullptr)
        {
            logger->logMessage(
                LogLevel::Error,
                "scene module: refused to register mesh '" + name + "': " + std::to_string(vertexCount)
                    + " vertices exceed the per-mesh limit (" + std::to_string(maxMeshVertices) + ")"
            );
        }
        return invalidMesh;
    }
    for (const std::uint32_t index : mesh.indices)
    {
        if (index >= vertexCount)
        {
            if (logger != nullptr)
            {
                logger->logMessage(
                    LogLevel::Error,
                    "scene module: refused to register mesh '" + name + "': index " + std::to_string(index)
                        + " is out of range (" + std::to_string(vertexCount) + " vertices)"
                );
            }
            return invalidMesh;
        }
    }
    // Bounds from the vertices, once: every proxy box and every spawn
    // descriptor reads the same numbers, so they cannot drift apart.
    SceneMeshDescriptor stored;
    stored.vertices = mesh.vertices;
    stored.indices  = mesh.indices;
    std::array<float, 3> minimum{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                                 std::numeric_limits<float>::max()};
    std::array<float, 3> maximum{-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
                                 -std::numeric_limits<float>::max()};
    for (std::size_t vertex = 0; vertex < vertexCount; ++vertex)
    {
        const float *position = mesh.vertices.data() + vertex * meshFloatsPerVertex;
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            minimum[axis] = std::min(minimum[axis], position[axis]);
            maximum[axis] = std::max(maximum[axis], position[axis]);
        }
    }
    stored.boundsMin = minimum;
    stored.boundsMax = maximum;
    const MeshHandle handle = nextMeshHandle++;
    MeshEntry        entry;
    entry.handle = handle;
    entry.name   = name;
    entry.mesh   = std::move(stored);
    meshRegistry.emplace(handle, std::move(entry));
    meshHandlesByName.emplace(name, handle);
    if (logger != nullptr)
    {
        logger->logMessage(
            LogLevel::Debug,
            "scene module: mesh '" + name + "' registered as handle " + std::to_string(handle) + " ("
                + std::to_string(vertexCount) + " vertices, " + std::to_string(mesh.indices.size() / 3)
                + " triangles)"
        );
    }
    if (meshAddedChannel != nullptr)
    {
        meshAddedChannel->publish(SceneMeshAddedEvent{.meshHandle = handle});
    }
    return handle;
}

bool SceneModule::mesh(
    MeshHandle handle, SceneMeshDescriptor &outMesh
) const
{
    const auto found = meshRegistry.find(handle);
    if (found == meshRegistry.end())
    {
        return false;
    }
    outMesh = found->second.mesh;
    return true;
}

SceneCapability::MeshHandle SceneModule::meshHandle(
    const std::string &name
) const
{
    const auto found = meshHandlesByName.find(name);
    return found != meshHandlesByName.end() ? found->second : invalidMesh;
}

std::string SceneModule::meshName(
    MeshHandle handle
) const
{
    const auto found = meshRegistry.find(handle);
    return found != meshRegistry.end() ? found->second.name : std::string();
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

std::vector<SceneCapability::ObjectHandle> SceneModule::objectHandles() const
{
    std::vector<ObjectHandle> handles;
    handles.reserve(registry.size());
    for (const auto &entry : registry)
    {
        handles.push_back(entry.first);
    }
    std::sort(handles.begin(), handles.end());
    return handles;
}

} // namespace engine
