// GPU resource store for the Vulkan backend: per-frame frame-uniform
// buffers + descriptor sets, mesh vertex/index buffers, texture images with
// samplers, and a cache of material descriptor sets keyed by the five bound
// texture handles. Compiled only under ENGINE_RENDERER_BACKEND_VULKAN.

#pragma once

#if defined(ENGINE_RENDERER_BACKEND_VULKAN)

#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/material.hpp>
#include <engine/renderer/mesh.hpp>
#include "render_shared.hpp"

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
namespace renderer
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

// std140 header of the radiance_cache probe grid, consumed by the fragment stage's
// indirect-lighting hook (the probe SH2 records themselves live in the set-0
// binding-1 storage buffer, not here). Keep in sync with the uProbeGrid
// comment block in shaders/mesh.frag.
struct ProbeGridUniformData
{
    float centerSpacing[4]    = {}; // xyz = grid center, w = level0 spacing
    float level0Meta[4]       = {}; // xyz = level0 dimensions, w = level1 spacing
    float level1Meta[4]       = {}; // xyz = level1 dimensions, w = grid active (1 = probes bound)
    float countMeta[4]        = {}; // x = probe count, y = direction resolution, z = level0 offset, w = level1 offset
};
static_assert(sizeof(ProbeGridUniformData) == 64, "the probe grid header occupies four std140 vec4 values");

// Per-frame uniform block (std140), one buffer per frame in flight. Mirrors
// the FrameData block in shaders/mesh.frag (mesh.vert reads only the leading
// members, so its block stays untouched by the probe-grid and field tails).
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
    ProbeGridUniformData   probeGrid          = {};
    // reflection_field screen-space field header: the field's own camera
    // (GL convention, the same matrix sampleReflectionField projects through
    // on the CPU) plus (width, height, max roughness, active) - the shader's
    // uFieldViewProj / uFieldMeta twins.
    float                  fieldViewProj[16]  = {};
    float                  fieldMeta[4]       = {};
};
static_assert(sizeof(FrameUniformData) == 2368, "FrameData must match the shader's std140 layout");

class VulkanResources
{
public:
    VulkanResources() = default;

    // Allocates the per-frame uniform buffers, the descriptor pool sized for
    // framesInFlight + material sets, the frame-uniform descriptor sets, and
    // the neutral white/flat-normal stand-in textures.
    bool create(const VulkanDeviceContext &device, std::uint32_t framesInFlight, VkDescriptorSetLayout frameSetLayout,
        VkDescriptorSetLayout materialSetLayout, VkDescriptorSetLayout materialExtensionSetLayout,
        bool mipmapsEnabled, float anisotropyLevel,
        std::uint32_t maximumTextures, std::uint32_t maximumMaterialSets);

    void destroy();

    // Skips the memcpy when the frame already holds this revision; returns
    // true in that case too (idempotent per frame slot).
    bool updateFrameUniform(std::uint32_t frameIndex, const FrameUniformData &data, std::uint64_t revision);

    // Replaces the packed probe records of the set-0 binding-1 storage buffer
    // (host-visible, written directly). The revision guard mirrors
    // updateFrameUniform; recordBytes may be 0 (dummy bound, hook off).
    bool updateProbeStorage(
        std::uint32_t frameIndex, const void *packedRecords, VkDeviceSize recordBytes, std::uint64_t revision);

    bool probeStorageCurrent(std::uint32_t frameIndex, std::uint64_t revision) const
    {
        return frameIndex < probeStorageVersions_.size() && probeStorageVersions_[frameIndex] == revision;
    }

    // ---- material extension blocks (set 2, dynamic-offset UBO) -------------
    // One host-visible ring per frame in flight; each mesh draw acquires a
    // 48-byte block carrying the material fields that do not fit the
    // 128-byte push-constant budget (the distance-fog triple, the procedural
    // mottle pair and the near-field relief overlay's triple - mesh.frag's
    // MaterialExtension). Identical tuples within a frame share a block, so
    // a streamed scene's hundreds of chunks spend a handful of blocks.

    // Clears the frame's block cursor and tuple dedupe (call at beginFrame,
    // after the frame's fence wait).
    void resetMaterialExtensions(std::uint32_t frameIndex);

    // Writes (or reuses) this material's extension block in the frame's ring
    // and returns the block's byte offset for the dynamic-offset bind; 0
    // (the all-off block) when the ring is exhausted.
    std::uint32_t acquireMaterialExtension(std::uint32_t frameIndex, const MeshMaterial &material);

    // The frame's extension descriptor set (bind at set 2 with the acquired
    // offset); VK_NULL_HANDLE before create().
    VkDescriptorSet materialExtensionSet(std::uint32_t frameIndex) const
    {
        return frameIndex < materialExtensionSets_.size() ? materialExtensionSets_[frameIndex] : VK_NULL_HANDLE;
    }

    // ---- reflection_field screen-space field (set 0, binding 2) ------------
    // One RGBA32F image per frame in flight, refreshed from the module's
    // latest snapshot through a per-slot staging buffer. The images are
    // (re)created when the field's dimensions change; a resize retires the
    // old images through the backend's deferred destroy queue (they may
    // still be sampled by in-flight frames of other slots).

    // True when the slot's image already holds this field revision (the
    // upload can be skipped - static scenes keep their field for free).
    bool reflectionFieldCurrent(std::uint32_t frameIndex, std::uint64_t revision) const
    {
        return frameIndex < reflectionFieldVersions_.size() && reflectionFieldVersions_[frameIndex] == revision
            && reflectionFieldWidth_ != 0;
    }

    // (Re)creates the per-slot field images when the dimensions change and
    // rewrites the binding-2 descriptors. Returns false when the objects
    // could not be created (the hook then stays on the neutral stand-in).
    bool ensureReflectionFieldImages(std::uint32_t width, std::uint32_t height);

    // Copies a width*height*4 float field into the slot's staging buffer
    // (persistently mapped) and marks the slot for this revision. Must run
    // after the slot's previous command buffer retired (the staging buffer
    // may still be read by it).
    bool stageReflectionField(std::uint32_t frameIndex, const float *radiance, std::uint32_t width,
        std::uint32_t height, std::uint64_t revision);

    // Records the staging -> image copy plus the layout barriers on the
    // frame's command buffer (before the render pass; the copy must precede
    // every draw that samples the image).
    void recordReflectionFieldUpload(std::uint32_t frameIndex, VkCommandBuffer command);

    // Advances the resize-retirement countdown; call once per beginFrame.
    void tickFrame();

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

    // Dynamic-geometry refresh: validates the new vertex stream against the
    // mesh's creation layout (same vertex count, canonical 11-float stride),
    // recomputes the bounding sphere from the new positions, and hands the
    // destination buffer plus byte size to the caller, which records the
    // copy (the frame-path copy the backend owns; see updateMeshVertices).
    // Returns false with RenderError::InvalidMesh when the handle is unknown
    // or the count does not match creation.
    bool prepareMeshVertexUpdate(MeshHandle mesh, const void *vertices, std::uint32_t vertexCount,
        VkBuffer *outVertexBuffer, VkDeviceSize *outVertexBytes, RenderError *error);

    bool createTexture(TextureHandle id, const TextureDescription &description, RenderError *error);
    bool createBlockTexture(TextureHandle id, const BlockTextureDescription &description, RenderError *error);
    void destroyTexture(TextureHandle texture);

    // Which block formats this device samples natively (queried once at
    // create() through vkGetPhysicalDeviceFormatProperties).
    [[nodiscard]] BlockTextureFormatSupport blockFormatSupport() const
    {
        return blockFormatSupport_;
    }

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
        VkDeviceSize  vertexBytes      = 0; // the creation vertex stream's size; updates must match
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
    using MaterialKey = std::array<TextureHandle, 6>;

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

    // Uploads every mip level's block payload in one command buffer (one
    // vkCmdCopyBufferToImage per level; the chain comes from the file, so
    // no blit pass runs) and leaves the image in SHADER_READ_ONLY_OPTIMAL.
    bool uploadImageBlocks(VkImage image, const BlockTextureDescription &description, const std::uint8_t *blocks);

    bool createNeutralTexture(VulkanTexture &destination, const std::uint8_t pixel[4]);

    bool createTextureObjects(VulkanTexture &texture, std::uint32_t width, std::uint32_t height,
        const std::uint8_t *pixels, TextureFilter filter, TextureAddress address, bool allowMipmaps);

    void destroyMeshObject(VulkanMesh &mesh);
    void destroyTextureObject(VulkanTexture &texture);

    bool writeMaterialSet(VkDescriptorSet set, const MaterialKey &key);

    const VulkanDeviceContext *device_ = nullptr;

    // Material extension ring (set 2): one host-visible buffer and one
    // dynamic-offset descriptor set per frame in flight. Block layout is
    // mesh.frag's MaterialExtension (3 x vec4 = 48 bytes), padded to the
    // device's minUniformBufferOffsetAlignment so every block offset is a
    // legal dynamic offset.
    struct MaterialExtensionBlock
    {
        float fogColor[4];
        float fogScaleNoiseScaleNoiseStrength[4];
        float detailUvScaleStrengthFadeStartFadeEnd[4];
    };
    static constexpr std::uint32_t kMaterialExtensionBlockFloats = 12;
    static constexpr std::uint32_t kMaterialExtensionCapacity    = 256;

    std::vector<VkBuffer>        materialExtensionBuffers_;
    std::vector<VmaAllocation>   materialExtensionAllocations_;
    std::vector<void *>          materialExtensionMappings_;
    std::vector<VkDescriptorSet> materialExtensionSets_;
    // Per-frame allocator state: the next free block and the tuples already
    // written this frame (dedupe; identical materials share one block).
    struct ExtensionTuple
    {
        MaterialExtensionBlock block;
        std::uint32_t          offset;
        bool                   used = false;
    };
    std::vector<std::vector<ExtensionTuple>> materialExtensionTuples_;
    std::vector<std::uint32_t>               materialExtensionCursors_;
    std::uint32_t                            materialExtensionStride_ = 256;

    std::vector<VkBuffer>        frameUniformBuffers_;
    std::vector<VmaAllocation>   frameUniformAllocations_;
    std::vector<void *>          frameUniformMappings_;
    std::vector<std::uint64_t>   frameUniformVersions_;
    std::vector<VkDescriptorSet> frameUniformSets_;

    // Probe storage for the radiance_cache indirect-lighting hook: one host-visible
    // std430 buffer per frame in flight, always bound on set 0 binding 1
    // (a zeroed dummy when no grid is active keeps the pipeline valid).
    static constexpr VkDeviceSize probeStorageBytes_ = maxGiProbes * giProbeRecordFloats * sizeof(float);
    std::vector<VkBuffer>        probeStorageBuffers_;
    std::vector<VmaAllocation>   probeStorageAllocations_;
    std::vector<void *>          probeStorageMappings_;
    std::vector<std::uint64_t>   probeStorageVersions_;

    // reflection_field images: one RGBA32F image + view + staging buffer per
    // frame in flight, all sharing one nearest-filter clamp sampler (the
    // shader reads through texelFetch, so no filtering support is needed).
    struct ReflectionFieldSlot
    {
        VkImage       image           = VK_NULL_HANDLE;
        VmaAllocation imageAllocation = nullptr;
        VkImageView   view            = VK_NULL_HANDLE;
        VkBuffer      stagingBuffer   = VK_NULL_HANDLE;
        VmaAllocation stagingAllocation = nullptr;
        void         *stagingMapping  = nullptr;
        bool          stagedThisFrame = false;
        std::uint64_t stagedRevision  = 0;
    };
    std::vector<ReflectionFieldSlot> reflectionFieldSlots_;
    std::vector<std::uint64_t>       reflectionFieldVersions_;
    std::uint32_t                    reflectionFieldWidth_  = 0;
    std::uint32_t                    reflectionFieldHeight_ = 0;
    VkSampler                        reflectionFieldSampler_ = VK_NULL_HANDLE;

    // Slots replaced by a resize, kept alive until every in-flight frame
    // that may still sample them has retired (fieldRetireCountdown_ counts
    // beginFrame calls; tickFrame destroys the set when it reaches zero).
    std::vector<ReflectionFieldSlot> retiredFieldSlots_;
    std::uint32_t                    fieldRetireCountdown_ = 0;

    // Destroys one slot's objects (image, view, staging).
    void destroyReflectionFieldSlot(ReflectionFieldSlot &slot);

    // Rewrites binding 2 onto the neutral white texture (see destroyReflectionFieldObjects).
    void bindNeutralFieldSampler();

    // Destroys the per-slot field objects (active, retired, sampler).
    void destroyReflectionFieldObjects();

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
    VkDescriptorSetLayout materialExtensionSetLayout_ = VK_NULL_HANDLE; // set 2 (mesh pipelines)
    bool                  mipmapsEnabled_          = true;  // [renderer] texture_mipmaps
    bool                  mipmapBlitSupported_     = false; // device supports linear blit on RGBA8
    BlockTextureFormatSupport blockFormatSupport_ {};      // device-native block formats (KTX path)
    float                 anisotropyLevel_         = 1.0f;
    std::uint32_t         maximumTextures_         = 256;
    std::uint32_t         maximumMaterialSets_     = 256;
};

} // namespace renderer
} // namespace engine

#endif

