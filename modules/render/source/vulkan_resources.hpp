// GPU resource store for the Vulkan backend: per-frame frame-uniform
// buffers + descriptor sets, mesh vertex/index buffers, texture images with
// samplers, and a cache of material descriptor sets keyed by the five bound
// texture handles. Compiled only under ENGINE_RENDER_BACKEND_VULKAN.

#ifndef ENGINE_RENDER_MODULE_VULKAN_RESOURCES_HPP
#define ENGINE_RENDER_MODULE_VULKAN_RESOURCES_HPP

#if defined(ENGINE_RENDER_BACKEND_VULKAN)

#include "engine/render/capabilities.hpp"
#include "engine/render/material.hpp"
#include "engine/render/mesh.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>
#include <vk_mem_alloc.h>
#include <vulkan/vulkan.h>
#include <vulkan/vulkan_core.h>

namespace engine
{
namespace render
{

class VulkanDeviceContext;

// One direct light in the std140 layout the fragment shader expects; spot
// parameters carry cos(inner)/cos(outer) cones (1/-1 disables the cone).
struct DirectLightUniformData
{
    float positionType[4]   = {};
    float directionRange[4] = {};
    float colorIntensity[4] = {};
    float spotParameters[4] = {};
};
static_assert(sizeof(DirectLightUniformData) == 64, "a direct light occupies four std140 vec4 values");

// Per-frame uniform block (std140), one buffer per frame in flight. Mirrors
// the FrameData block in shaders/mesh.vert and shaders/mesh.frag.
struct FrameUniformData
{
    float                  viewProj[16]       = {};
    float                  cameraPos[4]       = {};
    float                  sunDirection[4]    = {};
    float                  sunColor[4]        = {};
    float                  ambientSky[4]      = {};
    float                  ambientGround[4]   = {};
    float                  toneMapping[4]     = {};
    float                  directLightMeta[4] = {};
    DirectLightUniformData directLights[32]   = {};
};
static_assert(sizeof(FrameUniformData) == 2224, "FrameData must match the shader's std140 layout");

class VulkanResources
{
public:
    VulkanResources() = default;

    // Allocates the per-frame uniform buffers, the descriptor pool sized for
    // framesInFlight + material sets, the frame-uniform descriptor sets, and
    // the neutral white/flat-normal stand-in textures.
    bool create(const VulkanDeviceContext &device, std::uint32_t framesInFlight, VkDescriptorSetLayout frameSetLayout,
        VkDescriptorSetLayout materialSetLayout, bool mipmapsEnabled, float anisotropyLevel,
        std::uint32_t maximumTextures, std::uint32_t maximumMaterialSets);

    void destroy();

    // Skips the memcpy when the frame already holds this revision; returns
    // true in that case too (idempotent per frame slot).
    bool updateFrameUniform(std::uint32_t frameIndex, const FrameUniformData &data, std::uint64_t revision);

    VkDescriptorSet frameUniformSet(std::uint32_t frameIndex) const
    {
        return frameUniformSets_[frameIndex];
    }

    const VkDescriptorSet *frameUniformSetPtr(std::uint32_t frameIndex) const
    {
        return &frameUniformSets_[frameIndex];
    }

    bool createMesh(MeshHandle id, const MeshDescription &description, RenderError *error);
    void destroyMesh(MeshHandle mesh);

    bool createTexture(TextureHandle id, const TextureDescription &description, RenderError *error);
    void destroyTexture(TextureHandle texture);

    // Material descriptor sets are interned per texture-combination; null
    // handles map to the neutral stand-ins. materialSet() allocates on first
    // use and returns the fully neutral set when allocation fails or the
    // cache is full.
    VkDescriptorSet materialSet(const MeshMaterial &material);

private:
    // GPU mesh: device-local vertex/index buffers filled asynchronously via
    // the device's upload queue, plus bounding-sphere data for culling.
    struct VulkanMesh
    {
        VkBuffer      vertexBuffer     = VK_NULL_HANDLE;
        VmaAllocation vertexAllocation = nullptr;
        VkBuffer      indexBuffer      = VK_NULL_HANDLE;
        VmaAllocation indexAllocation  = nullptr;
        std::uint32_t indexCount       = 0;
        float         boundsCenter[3]  = {}; // mesh-local bounding sphere
        float         boundsRadius     = 0.0f;
        bool          boundsValid      = false;
    };

    struct VulkanTexture
    {
        VkImage       image           = VK_NULL_HANDLE;
        VmaAllocation imageAllocation = nullptr;
        VkImageView   view            = VK_NULL_HANDLE;
        VkSampler     sampler         = VK_NULL_HANDLE; // filter/address/anisotropy baked in
        std::uint32_t mipLevels       = 1;
    };

    // A material is identified purely by its five texture handles; factors
    // ride in push constants and never invalidate the set.
    using MaterialKey = std::array<TextureHandle, 5>;

    struct MaterialKeyHash
    {
        std::size_t operator()(const MaterialKey &key) const noexcept
        {
            std::size_t hash = 0xcbf29ce484222325ull;
            for (TextureHandle texture : key)
            {
                hash ^= std::hash<TextureHandle>{}(texture) + 0x9e3779b97f4a7c15ull + (hash << 6) + (hash >> 2);
            }
            return hash;
        }
    };

public:
    const VulkanMesh *meshFor(MeshHandle mesh) const;

private:
    // Uploads RGBA8 pixels through a staging buffer, generating the full mip
    // chain by successive linear blits when mipLevels > 1, and leaves the
    // image in SHADER_READ_ONLY_OPTIMAL. Asynchronous: queued via the device.
    bool uploadImagePixels(
        VkImage image, std::uint32_t width, std::uint32_t height, std::uint32_t mipLevels, const std::uint8_t *pixels);

    bool createNeutralTexture(VulkanTexture &destination, const std::uint8_t pixel[4]);

    bool createTextureObjects(VulkanTexture &texture, std::uint32_t width, std::uint32_t height,
        const std::uint8_t *pixels, TextureFilter filter, TextureAddress address, bool allowMipmaps);

    void destroyMeshObject(VulkanMesh &mesh);
    void destroyTextureObject(VulkanTexture &texture);

    bool writeMaterialSet(VkDescriptorSet set, const MaterialKey &key);

    const VulkanDeviceContext *device_ = nullptr;

    std::vector<VkBuffer>        frameUniformBuffers_;
    std::vector<VmaAllocation>   frameUniformAllocations_;
    std::vector<void *>          frameUniformMappings_;
    std::vector<std::uint64_t>   frameUniformVersions_;
    std::vector<VkDescriptorSet> frameUniformSets_;
    VkDescriptorPool             descriptorPool_ = VK_NULL_HANDLE;

    std::unordered_map<MeshHandle, VulkanMesh> meshes_;

    std::unordered_map<TextureHandle, VulkanTexture> textures_;

    // Neutral stand-ins bound wherever a material omits a map.
    VulkanTexture   whiteTexture_{};
    VulkanTexture   flatNormalTexture_{};
    VkDescriptorSet fullyNeutralSet_ = VK_NULL_HANDLE; // all five slots neutral; error fallback

    std::unordered_map<MaterialKey, VkDescriptorSet, MaterialKeyHash> materialSets_;

    // One-entry lookup cache: consecutive draws with the same material skip
    // the hash lookup.
    MaterialKey     lastMaterialKey_{};
    VkDescriptorSet lastMaterialSet_    = VK_NULL_HANDLE;
    bool            hasLastMaterialSet_ = false;

    VkDescriptorSetLayout materialSetLayout_   = VK_NULL_HANDLE;
    bool                  mipmapsEnabled_          = true;  // [render] texture_mipmaps
    bool                  mipmapBlitSupported_     = false; // device supports linear blit on RGBA8
    float                 anisotropyLevel_         = 1.0f;
    std::uint32_t         maximumTextures_         = 256;
    std::uint32_t         maximumMaterialSets_     = 256;
};

} // namespace render
} // namespace engine

#endif

#endif
