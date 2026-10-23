#ifndef ENGINE_MODULES_RENDERER_RENDER_INTERFACE_HPP
#define ENGINE_MODULES_RENDERER_RENDER_INTERFACE_HPP

// In-tree pass interface between the renderer module and the lighting
// technique modules. This is deliberately NOT a capability contract: it is
// typed against the current graphics backend and lives inside the module tree,
// so the kernel and script mods stay implementation-agnostic (mods/README.md,
// "Portability"). The renderer owns every resource; lighting modules attach
// pass objects that record their work into the renderer's frame command
// buffer.
//
// A technique module is named after its ALGORITHM ("pbr", "radiance_cascades"),
// never after a rendering category: categories grow implementations while the
// algorithm stays what it is. What a technique contributes is declared as a
// physical output (which radiance target it writes), and the renderer composes
// the frame from that declaration.

#include <vulkan/vulkan.hpp>
#include <vulkan/vulkan_raii.hpp>

#include <array>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>

namespace engine
{

// Camera uniforms as the shaders see them. Kept in one place because the
// renderer uploads it and both lighting modules consume it. mat4s are
// column-major float[16] (GLSL layout).
struct CameraData
{
    std::array<float, 16> viewProjection{};
    std::array<float, 16> inverseViewProjection{};
    std::array<float, 4>  position{}; // xyz world position, w unused
};

// Everything a lighting pass may touch for one frame. References stay valid
// for the duration of the record call; image views are in GENERAL layout
// (they persist across frames and are shared between graphics and compute).
struct FrameRenderContext
{
    vk::raii::Device  &device;
    vk::PhysicalDevice physicalDevice; // for memory type queries in passes
    vk::Queue          graphicsQueue;  // for pass-owned setup submissions
    std::uint32_t      graphicsQueueFamily;
    vk::Extent2D       extent;

    // Where the passes load their SPIR-V from (the renderer's configured
    // shader directory).
    const std::filesystem::path &shaderDirectory;

    // G-buffer attachments written by the geometry pass.
    vk::ImageView albedoView;   // RGBA8_UNORM: rgb albedo, a roughness
    vk::ImageView normalView;   // RGBA16F: rgb world normal, a metallic
    vk::ImageView emissiveView; // RGBA16F: rgb emissive radiance
    vk::ImageView depthView;    // D32_SFLOAT

    // Lighting targets. A target is a real image only when at least one
    // attached technique declares it as output; otherwise the composite path
    // receives a cleared 1x1 stand-in and skips reading it. Techniques add
    // radiance (the composite sums the layers); a technique that needs to
    // modulate instead of add belongs in the composite pass, not here.
    vk::ImageView directRadianceView;   // RGBA16F: radiance from directly evaluated emitters
    vk::ImageView indirectRadianceView; // RGBA16F: diffuse GI irradiance / pi (transport)
    vk::ImageView indirectSpecularView; // RGBA16F: Fresnel-weighted reflection radiance

    // Shared scene data (uploaded by the renderer from [scene]).
    vk::Buffer    primitivesBuffer; // std430 array of GpuScenePrimitive
    std::uint32_t primitiveCount;
    vk::Buffer    materialsBuffer;   // std430 array of GpuSceneMaterial (one per primitive)
    vk::ImageView albedoArrayView;   // RGBA8_SRGB 2D array, one layer per texture set (1x1 when none)
    vk::ImageView dataArrayView;     // RGBA8_UNORM 2D array: R rough, G metallic, BA normal xy
    vk::Buffer    environmentBuffer; // std140 EnvironmentUniform (sky gradient + sun disk)
    vk::Buffer    cameraBuffer;      // uniform CameraData
    // Perspective near/far planes (meters). The GI spatial filter linearizes
    // buffer depth with them - NDC depth differences cannot express one
    // edge-awareness metric across the depth range.
    float cameraNearPlane = 0.0f;
    float cameraFarPlane  = 0.0f;
};

// Which radiance layers a technique writes. Declared per technique so the
// renderer creates exactly the targets that are consumed and the composite
// pass knows which layers hold valid data.
enum class LightingTarget : std::uint32_t
{
    Direct           = 1u << 0, // directRadianceView: directly evaluated emitters
    IndirectDiffuse  = 1u << 1, // indirectRadianceView: transported diffuse (global illumination)
    IndirectSpecular = 1u << 2, // indirectSpecularView: Fresnel-weighted reflection radiance
};

constexpr bool hasLightingTarget(
    LightingTarget mask, LightingTarget flag
)
{
    return (static_cast<std::uint32_t>(mask) & static_cast<std::uint32_t>(flag)) != 0;
}

// Specular hand-off contract between the analytic direct pass (pbr) and the
// ray traced reflection pass: pbr fades its analytic specular to zero as
// roughness approaches the hand-off point and the reflection pass takes over
// above it, so the sun is not drawn twice on glossy surfaces. Both modules
// read [lighting.ray_reflections].max_roughness - with these defaults, from
// one place, because a mismatch would double-light or drop the sun exactly at
// the fade band.
inline constexpr float         defaultReflectionMaxRoughness = 0.35f;
inline constexpr float         defaultReflectionMaxDistance  = 40.0f;
inline constexpr std::uint32_t defaultReflectionMarchSteps   = 96;

// One lighting technique implementation (for example the PBR analytic pass or
// the radiance cascades transport pass). record appends the technique's
// dispatches for one frame; it may read the G-buffer and scene data and must
// leave its declared targets in a shader-readable state.
class LightingTechniquePass
{
public:
    virtual ~LightingTechniquePass() = default;

    // Algorithm name, matched against [lighting] techniques entries.
    virtual const char *techniqueName() const = 0;

    virtual LightingTarget targets() const = 0;

    virtual void record(vk::raii::CommandBuffer &commandBuffer, const FrameRenderContext &context) = 0;

    // Called by the renderer after a device-side waitIdle whenever the frame
    // resources (G-buffer views, extent) were recreated. Passes rebuild their
    // lazily created pipelines and descriptors in their next record call; the
    // waitIdle guarantees no in-flight frame still references the old objects.
    virtual void notifyFrameResourcesRecreated() = 0;
};

} // namespace engine

#endif // ENGINE_MODULES_RENDERER_RENDER_INTERFACE_HPP
