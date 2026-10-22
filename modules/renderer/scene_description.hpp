#ifndef ENGINE_MODULES_RENDERER_SCENE_DESCRIPTION_HPP
#define ENGINE_MODULES_RENDERER_SCENE_DESCRIPTION_HPP

// Shared world description for the renderer and the lighting modules. Both
// rasterize and trace THE SAME scene: the renderer turns objects into meshes,
// the GI module turns them into an analytic signed distance field. Sharing one
// parser and one GPU layout is what makes the analytic SDF exact for the
// configured geometry - the foundation of the no-light-leak guarantee
// (docs/robustness.md, problem 24).

#include "engine/kernel/component_interface.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace engine
{

inline constexpr std::uint32_t maxSceneObjects = 32;
inline constexpr std::uint32_t maxPointLights = 16;

enum class SceneObjectType : std::uint32_t
{
    Box = 0,
    Sphere = 1,
};

struct SceneObject
{
    SceneObjectType type = SceneObjectType::Box;
    std::array<float, 3> position = {0.0f, 0.0f, 0.0f};
    // Box: half extents; sphere: radius in x (y/z ignored - spheres keep exact
    // normals under uniform radius).
    std::array<float, 3> size = {1.0f, 1.0f, 1.0f};
    float rotationYRadians = 0.0f;
    std::array<float, 3> albedo = {0.8f, 0.8f, 0.8f};
    float roughness = 0.8f;
    float metallic = 0.0f;
    // Emissive radiance is albedo * emissiveStrength (unbounded HDR).
    float emissiveStrength = 0.0f;
    // PBR texture set ([scene.objects].texture = a folder named <name> under
    // [renderer].texture_directory with Color/NormalGL/Roughness/Metalness
    // PNGs; the directory comes from the content mod's shipped defaults).
    // Empty = flat configured albedo/roughness/metallic. The renderer resolves
    // the name into a texture array layer index.
    std::string textureSet;
    // Texture tiling scale: one texture tile covers uvScale world meters
    // ([scene.objects].uv_scale, default 1).
    float uvScale = 1.0f;
};

// Camera-feel defaults shared by [scene.camera] parsing (SceneCamera below)
// and the fly controller's own fallbacks (camera_controller.hpp); one
// definition so the two sites cannot drift apart.
inline constexpr float defaultFlySpeed = 4.0f;
inline constexpr float defaultFlyBoost = 4.0f;
inline constexpr float defaultLookSensitivity = 0.0022f;
// Fly speed scaling per scroll-wheel notch.
inline constexpr float flyWheelSpeedFactor = 1.15f;

struct SceneCamera
{
    std::array<float, 3> position = {0.0f, 1.6f, 3.5f};
    std::array<float, 3> lookAt = {0.0f, 1.0f, 0.0f};
    float verticalFieldOfViewDegrees = 60.0f;
    float nearPlane = 0.05f;
    float farPlane = 500.0f;
    // Free-fly controls ([scene.camera] free_fly). When enabled the camera
    // starts at position/look_at and is then driven by the keyboard and mouse
    // every frame (see camera_controller.hpp for the binding).
    bool freeFly = true;
    // Camera-feel defaults shared with the fly controller's own fallbacks
    // (one definition so the two cannot drift).
    float flySpeed = defaultFlySpeed;            // base speed in world units per second
    float flyBoost = defaultFlyBoost;            // speed multiplier while the boost key is held
    float lookSensitivity = defaultLookSensitivity; // yaw/pitch radians per cursor pixel
};

// GPU layout shared with the shaders (scene_common.glsl documents the
// std430 struct; keep both sides in sync).
struct GpuScenePrimitive
{
    std::array<float, 4> positionType;  // xyz position, w SceneObjectType
    std::array<float, 4> sizeRotation;  // xyz size, w rotationY (radians)
    std::array<float, 4> albedoRoughness;          // rgb albedo, a roughness
    // x metallic, y emissive strength, z sin(rotationY), w cos(rotationY).
    // The rotation trig is baked here at upload: the SDF shaders apply this
    // transform for every primitive on every sphere-tracing step, and the two
    // transcendentals must not be re-evaluated inside that loop (the march
    // alone evaluates the field up to (extent/stride)^2 * rays * steps times
    // per frame). The z/w lanes were previously unused.
    std::array<float, 4> metallicEmissiveStrength;
};

// Sun description ([scene.sun]); the direction points TOWARDS the scene (the
// lit side faces -direction). Both the pbr module and the environment's sun
// disk read the same section.
struct SceneSun
{
    std::array<float, 3> direction = {0.0f, -1.0f, 0.0f};
    std::array<float, 3> color = {1.0f, 1.0f, 1.0f};
    float intensity = 0.0f;
};

struct SceneSky
{
    // Vertical gradient radiance ([scene.sky]; every component defaults to
    // the flat sky_radiance value, which keeps old configurations exactly
    // flat). The sun disk is direct-view only (composite + reflections); the
    // GI transport field deliberately stays disk-free.
    std::array<float, 3> zenith = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> horizon = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> ground = {0.0f, 0.0f, 0.0f};
    bool sunDisk = true;
    float sunDiskSizeDegrees = 0.6f;
};

struct SceneDescription
{
    SceneCamera camera;
    SceneSun sun;
    std::array<float, 3> skyRadiance = {0.0f, 0.0f, 0.0f};
    SceneSky sky;
    std::vector<SceneObject> objects;
    // Texture set names in first-use order; a scene object's textureSet name
    // maps to its texture array layer through this list (the renderer loads
    // one array layer pair per set).
    std::vector<std::string> textureSets;

    // Parses [scene] from the configuration. Returns false with a precise
    // message for structurally invalid content (unknown type, wrong array
    // lengths, too many objects). The convenience type "plane" is normalized
    // into a very thin box: rasterized geometry and the analytic SDF then use
    // the same primitive for every object, which is what keeps traced and
    // rasterized geometry consistent. An absent [scene] yields an empty scene
    // and returns true - callers decide what an empty scene means for them.
    static bool parseFromConfiguration(ComponentContext& context, SceneDescription& scene, std::string& error);
};

} // namespace engine

#endif // ENGINE_MODULES_RENDERER_SCENE_DESCRIPTION_HPP
