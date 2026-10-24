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
// scene edit (docs/robustness.md, problem 25's lifetime rules stay intact).

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
} // namespace scene_object_type

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
    // exact normals under uniform radius), matching SceneObject's convention.
    std::array<float, 3> size              = {1.0f, 1.0f, 1.0f};
    float                rotationYRadians  = 0.0f;
    std::array<float, 3> albedo            = {0.8f, 0.8f, 0.8f};
    float                roughness         = 0.8f;
    float                metallic          = 0.0f;
    float                emissiveStrength  = 0.0f;
    std::string          textureSet;
    float                uvScale           = 1.0f;
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
// allocated once at full capacity (the analytic SDF's cost model scales
// linearly with the object count - see the renderer README) and the cap is
// what keeps a runaway script from silently degrading the frame rate.
class SceneCapability
{
public:
    static constexpr const char *capabilityName = "scene";

    using ObjectHandle = std::uint32_t;

    static constexpr ObjectHandle invalidObject = 0;

    // The registry capacity. Must stay identical to the renderer's
    // maxSceneObjects (both are the GPU primitive buffer's allocation size;
    // scene_common.glsl mirrors the value on the shader side).
    static constexpr std::uint32_t maxObjects = 32;

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

    // Snapshot of every live object in ascending handle order. The renderer
    // consumes this once per dirty frame to rebuild its GPU data; the stable
    // order is what keeps primitive indices (and with them the per-primitive
    // material table) deterministic across frames.
    virtual std::vector<SceneObjectDescriptor> objectSnapshot() const = 0;
};

} // namespace engine

#endif // ENGINE_MODULES_SCENE_SCENE_CAPABILITY_HPP
