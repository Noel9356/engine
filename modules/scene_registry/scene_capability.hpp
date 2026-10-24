#ifndef ENGINE_MODULES_SCENE_SCENE_CAPABILITY_HPP
#define ENGINE_MODULES_SCENE_SCENE_CAPABILITY_HPP

// The published capability surface of the scene module. Every consumer -
// other modules, native mods, the host - includes THIS header, from the
// module that provides the capability: the provider owns and publishes its
// own capability vocabulary, so there is no central contract directory and
// no mirrored copy that could drift. Like every published capability header
// this stays implementation-, platform- and vendor-agnostic.
//
// The capability is the RUNTIME object registry: it starts from the static
// world (the renderer pushes the parsed [scene] objects into it during its
// start phase) and then accepts mutations from any granted consumer - script
// mods through the scripting bridge, native mods and sibling modules through
// this interface. The renderer and the lighting techniques rebuild their GPU
// data from the registry incrementally: the primitive and material buffers
// are allocated once at full capacity and only their CONTENT is re-uploaded,
// so no descriptor set, pipeline or buffer handle is ever invalidated by a
// scene edit (docs/robustness.md, invariant 25's lifetime rules stay intact).

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace engine
{

// Object kinds, mirroring the renderer's SceneObjectType values (the GPU
// primitive type lane carries these exact numbers; scene_common.glsl
// documents the shader side). Declared as constants instead of an enum class
// so descriptors survive any ABI boundary as plain data.
namespace scene_object_type
{
inline constexpr std::uint32_t box    = 0;
inline constexpr std::uint32_t sphere = 1;
// A triangle mesh registered through addMesh. The renderer rasterizes the
// mesh's actual triangles; the analytic tracing consumers (GI, reflections,
// shadows) see the mesh's oriented bounding-box proxy - a real box primitive
// carrying the same material (docs/robustness.md, invariant 24: the two paths
// agree on the proxy exactly).
inline constexpr std::uint32_t mesh   = 2;
} // namespace scene_object_type

// Floats per vertex in SceneMeshDescriptor::vertices: position xyz, normal
// xyz, interleaved. This is the same layout the renderer's unit meshes use
// (the geometry pass consumes registered meshes through the same vertex input
// state), so a registered mesh uploads exactly the bytes the descriptor
// carries.
inline constexpr std::size_t meshFloatsPerVertex = 6;

// One scene object as the registry stores and hands it out. Plain values
// only: position and size in world units, Y rotation in radians, the PBR
// material quad (albedo / roughness / metallic / emissive strength) and the
// optional texture set name (resolved against the renderer's texture array
// layers; an unknown name renders flat - the startup diagnostics call this
// out per object, exactly like the static scene path).
struct SceneObjectDescriptor
{
    std::uint32_t       type     = scene_object_type::box;
    std::array<float, 3> position = {0.0f, 0.0f, 0.0f};
    // Box: half extents; sphere: radius in x (y/z ignored - spheres keep
    // exact normals under uniform radius); mesh: the registry fills this with
    // the registered mesh's bounding-box HALF EXTENTS (the proxy box the
    // tracing consumers see and the collider shape physics uses).
    std::array<float, 3> size              = {1.0f, 1.0f, 1.0f};
    float                rotationYRadians  = 0.0f;
    std::array<float, 3> albedo            = {0.8f, 0.8f, 0.8f};
    float                roughness         = 0.8f;
    float                metallic          = 0.0f;
    float                emissiveStrength  = 0.0f;
    std::string          textureSet;
    float                uvScale           = 1.0f;
    // scene_object_type::mesh only: the registry mesh this object instances
    // (a handle addMesh returned). Zero for analytic primitives. Meshes are
    // shared: several objects may instance one mesh, each through its own
    // transform and material.
    std::uint32_t        meshHandle        = 0;
};

// One registered triangle mesh, in the object's local space (meters). The
// registry computes the bounds from the vertices at registration; the bounds
// are what spawn descriptors receive as their size and what the renderer
// uploads as the tracing proxy box. Vertices are interleaved
// meshFloatsPerVertex floats: position xyz, normal xyz - the same layout the
// renderer's unit meshes use, so the geometry pass draws a registered mesh
// with its existing vertex input state. Normals are required (the asset
// importer computes smooth normals when the source file has none).
struct SceneMeshDescriptor
{
    std::vector<float>         vertices; // meshFloatsPerVertex floats per vertex
    std::vector<std::uint32_t> indices;  // triangle list
    std::array<float, 3>       boundsMin = {0.0f, 0.0f, 0.0f}; // registry-computed
    std::array<float, 3>       boundsMax = {0.0f, 0.0f, 0.0f}; // registry-computed
};

// Published after the registry accepted one mesh. The handle is the
// registry's stable address of the mesh for every later instance.
struct SceneMeshAddedEvent
{
    std::uint32_t meshHandle = 0;
};

// Published after the registry accepted one object. The handle is the
// registry's stable address of the object for every later call.
struct SceneObjectAddedEvent
{
    std::uint32_t objectHandle = 0;
};

// Published after an object left the registry. The handle is NOT reused:
// consumers that still hold it are expected to drop it.
struct SceneObjectRemovedEvent
{
    std::uint32_t objectHandle = 0;
};

// Published after an object's transform or material changed. The renderer
// coalesces these into one GPU data refresh per frame: many edits in one
// frame cost one re-upload, not one per edit.
struct SceneObjectChangedEvent
{
    std::uint32_t objectHandle = 0;
};

// Capability "scene": the runtime object registry. Handles are stable and
// never reused; the object cap exists because the GPU primitive buffer is
// allocated once at the engine-wide maximum (the analytic SDF's cost model
// scales linearly with the object count - see the renderer README) and the
// cap is what keeps a runaway script from silently degrading the frame rate.
class SceneCapability
{
public:
    static constexpr const char *capabilityName = "scene";

    using ObjectHandle = std::uint32_t;
    using MeshHandle   = std::uint32_t;

    static constexpr ObjectHandle invalidObject = 0;
    static constexpr MeshHandle   invalidMesh   = 0;

    // The default live-object capacity ([scene] max_objects; configurable up
    // to gpuMaxObjects through objectCapacity's configuration). The GPU
    // primitive and material buffers are allocated once at gpuMaxObjects, so
    // raising the configured cap never reallocates anything.
    static constexpr std::uint32_t defaultMaxObjects = 32;
    // The engine-wide GPU capacity (renderer's primitive/material buffer
    // allocation size; also the hard ceiling for the configured cap).
    static constexpr std::uint32_t gpuMaxObjects = 256;

    // Registry mesh limits: meshes are assets shared by instances, so a
    // modest count is enough; the vertex cap bounds one mesh's GPU buffer.
    static constexpr std::uint32_t maxMeshes       = 8;
    static constexpr std::uint32_t maxMeshVertices = 1000000;

    virtual ~SceneCapability() = default;

    // Registers one object and returns its handle, or invalidObject when the
    // registry is full. The failure is precise (the provider logs it); the
    // caller decides whether that is an error or a tolerable cap.
    virtual ObjectHandle addObject(
        const SceneObjectDescriptor &object
    ) = 0;

    // Removes one object. Returns false for unknown handles (including one
    // that was already removed - handles are never reused).
    virtual bool removeObject(
        ObjectHandle handle
    ) = 0;

    // Replaces one object's transform. Returns false for unknown handles.
    virtual bool setObjectTransform(
        ObjectHandle handle, const std::array<float, 3> &position, float rotationYRadians
    ) = 0;

    // Replaces one object's material (albedo, roughness, metallic, emissive,
    // texture set, uv scale). Returns false for unknown handles.
    virtual bool setObjectMaterial(
        ObjectHandle handle, const SceneObjectDescriptor &material
    ) = 0;

    // Readback of one object. Returns false for unknown handles.
    virtual bool object(
        ObjectHandle handle, SceneObjectDescriptor &outObject
    ) const = 0;

    // Current live object count (registry size, never counting removed
    // handles).
    virtual std::uint32_t objectCount() const = 0;

    // The configured live-object capacity ([scene] max_objects, clamped to
    // [1, gpuMaxObjects]). addObject refuses beyond this; the GPU buffers are
    // allocated at gpuMaxObjects regardless, so raising the cap between runs
    // is a configuration edit, never a code change.
    virtual std::uint32_t objectCapacity() const = 0;

    // Registers one triangle mesh under a name unique within this run (the
    // same name returns the SAME handle: imports are idempotent) and returns
    // its handle. invalidMesh when the mesh cap is reached, the vertex/index
    // counts exceed the limits, or the arrays are malformed - each failure
    // logged precisely. The registry copies the data; the caller's vectors
    // are free afterwards. Bounds are computed here from the vertices.
    virtual MeshHandle addMesh(const std::string &name, const SceneMeshDescriptor &mesh) = 0;

    // Readback of one registered mesh (the renderer uploads new handles from
    // this). Returns false for unknown handles.
    virtual bool mesh(MeshHandle handle, SceneMeshDescriptor &outMesh) const = 0;

    // Lookup by the registration name. invalidMesh for unknown names
    // (persistence and scripts address meshes by name; handles are runtime
    // values that a saved file cannot legally carry).
    virtual MeshHandle meshHandle(const std::string &name) const = 0;

    // The registration name of one registered mesh, or an empty string for
    // unknown handles (diagnostics and the persistence module).
    virtual std::string meshName(MeshHandle handle) const = 0;

    // Snapshot of every live object in ascending handle order. The renderer
    // consumes this once per dirty frame to rebuild its GPU data; the stable
    // order is what keeps primitive indices (and with them the per-primitive
    // material table) deterministic across frames.
    virtual std::vector<SceneObjectDescriptor> objectSnapshot() const = 0;

    // Every live handle in ascending order (the same order objectSnapshot
    // uses). Consumers that must remove everything or address each object
    // individually read this instead of probing handles.
    virtual std::vector<ObjectHandle> objectHandles() const = 0;
};

} // namespace engine

#endif // ENGINE_MODULES_SCENE_SCENE_CAPABILITY_HPP
