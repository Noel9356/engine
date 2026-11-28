// Pipelines and descriptor layouts for the Vulkan backend: the mesh PBR
// pipeline (plus its depth-write, two-sided and transparent twins), the
// alpha-blended shell-fur pipeline, the alpha-blended overlay pipeline, a
// disk-backed pipeline cache, and the shader modules from the
// generated SPIR-V headers (mesh_*_gen.hpp, fur_*_gen.hpp, ui_*_gen.hpp).
// Compiled only under ENGINE_RENDERER_BACKEND_VULKAN.

#pragma once

#if defined(ENGINE_RENDERER_BACKEND_VULKAN)
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vulkan/vulkan.h>
#include <vulkan/vulkan_core.h>

namespace engine
{
namespace renderer
{

class VulkanDeviceContext;

// Per-draw state pushed to both shader stages (128 bytes, matching the
// push_constant block declared in shaders/mesh.vert / mesh.frag).
// metallicRoughness packs (metallic, roughness, normalScale, occlusionStrength);
// emissive packs (emissiveColor.rgb, unlit flag); cloth packs
// (sheenFactor, doubleSided flag, unused, unused).
struct PushConstants
{
    float model[16]          = {};
    float baseColorFactor[4] = {1.0f, 1.0f, 1.0f, 1.0f};

    float metallicRoughness[4] = {0.0f, 1.0f, 1.0f, 1.0f};

    float emissive[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    float cloth[4] = {0.0f, 0.0f, 0.0f, 0.0f};
};
static_assert(sizeof(PushConstants) == 128, "the shader's push_constant block is 128 bytes");

// Per-draw state pushed to the fur pipeline's stages (128 bytes, matching
// shaders/fur.vert / fur.frag: the mesh PushData block minus the cloth vec4
// plus one fur vec4 - both fifth slots share the same 16 bytes at offset
// 112, so the fur block stays at the 128-byte push-constant minimum).
// On the fur pipeline metallicRoughness is reinterpreted as
// (metallic, roughness, densityBoost, outerLayerFade) - fur shading never
// reads the normal-map/occlusion factors (see fur.frag). fur packs
// (layerIndex, layerCount, furHeight, gravityStrength).
struct FurPushConstants
{
    float model[16]          = {};
    float baseColorFactor[4] = {1.0f, 1.0f, 1.0f, 1.0f};

    float metallicRoughness[4] = {0.0f, 1.0f, 6.0f, 1.0f};

    float emissive[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    float fur[4] = {1.0f, 8.0f, 0.035f, 0.0f};
};
static_assert(sizeof(FurPushConstants) == 128, "the fur shader's push_constant block is 128 bytes");

class VulkanPipeline
{
public:
    VulkanPipeline() = default;

    // Back-face culling is baked into the pipeline rasterization state, so it
    // must be chosen before build().
    void setCullBackFaces(bool enabled) noexcept
    {
        cullBackFaces_ = enabled;
    }
    void setPipelineCacheMaxBytes(std::size_t maximumBytes) noexcept
    {
        pipelineCacheMaxBytes_ = maximumBytes;
    }

    // Creates render pass, descriptor layouts (set 0: frame uniform buffer +
    // probe storage buffer + reflection-field sampler; set 1: five combined
    // image samplers), pipeline layout, the mesh pipeline family, fur
    // pipeline, and overlay pipeline. srgbOutput_ and sampleCount_ must be
    // set first.
    bool build(const VulkanDeviceContext &device, VkFormat colorFormat, VkFormat depthFormat);

    // Serializes the pipeline cache to its disk path immediately. The normal
    // save happens only inside destroy() (clean shutdown); without an earlier
    // flush, a process killed at any other moment loses every compiled
    // pipeline and the next launch pays the full driver compile again.
    // Returns the byte count written (0 when there was nothing to save).
    std::size_t flushPipelineCacheToDisk(VkDevice device);

    void destroy(VkDevice device);

    VkRenderPass renderPass() const
    {
        return renderPass_;
    }
    // Resume twin of the render pass for the legacy path: identical
    // attachments (compatible, so the same framebuffers bind) but LOAD
    // operations and PRESENT_SRC initial layout. The mid-frame
    // updateMeshVertices path ends the render pass to record a transfer,
    // then re-begins drawing through this one so the already-drawn contents
    // survive.
    VkRenderPass renderPassResume() const
    {
        return renderPassResume_;
    }
    VkDescriptorSetLayout frameSetLayout() const
    {
        return frameSetLayout_;
    }
    VkDescriptorSetLayout materialSetLayout() const
    {
        return materialSetLayout_;
    }
    VkDescriptorSetLayout materialExtensionSetLayout() const
    {
        return materialExtensionSetLayout_;
    }
    VkPipelineLayout pipelineLayout() const
    {
        return pipelineLayout_;
    }
    VkPipeline pipeline() const
    {
        return pipeline_;
    }
    // The mesh pipeline family has three independent axes:
    //   depthWrite - off for backdrop shells and (always) transparent draws,
    //     so a blended surface never poisons the depth buffer;
    //   twoSided - cull mode NONE for double-sided materials (cloth sheets,
    //     flags, thin shells), whose fragments may face either way and flip
    //     their shading normal in the fragment stage;
    //   cwWinding - VK_FRONT_FACE_CLOCKWISE for materials declaring the
    //     mirrored winding convention (MeshMaterial::frontFaceWinding < 0);
    //   transparent - alpha blending (src-alpha / one-minus-src-alpha) for
    //     materials with opacity below 1, the mesh.frag oColor.a twin of the
    //     software rasterizer's transparent commit.
    // The opaque family crosses depthWrite x twoSided x cwWinding; the
    // transparent family fixes depthWrite off, leaving twoSided x cwWinding.
    VkPipeline meshPipeline(bool depthWrite, bool twoSided, bool cwWinding, bool transparent) const;
    VkPipelineLayout overlayPipelineLayout() const
    {
        return overlayPipelineLayout_;
    }
    VkPipeline overlayPipeline() const
    {
        return overlayPipeline_;
    }
    // Shell-fur pipeline: alpha-blended (src-alpha / one-minus-src-alpha),
    // depth-test on, depth-write OFF, back-face culling like the mesh
    // pipeline. Its layout is its own (128-byte push constants) but reuses
    // the SAME set-0/set-1 descriptor set layout objects, so material
    // descriptor sets interned for mesh draws bind unchanged.
    VkPipelineLayout furPipelineLayout() const
    {
        return furPipelineLayout_;
    }
    VkPipeline furPipeline() const
    {
        return furPipeline_;
    }

    void setSampleCount(VkSampleCountFlagBits s) noexcept
    {
        sampleCount_ = s;
    }
    VkSampleCountFlagBits sampleCount() const noexcept
    {
        return sampleCount_;
    }
    void setSrgbOutput(bool s) noexcept
    {
        srgbOutput_ = s; // flips the SRGB_OUTPUT specialization constant fed to the fragment shader
    }
    void setDynamicRenderingEnabled(bool d) noexcept
    {
        dynamicRenderingEnabled_ = d; // true = vkCmdBeginRendering path, no render pass objects
    }
    bool dynamicRenderingEnabled() const noexcept
    {
        return dynamicRenderingEnabled_;
    }
private:
    VkSampleCountFlagBits sampleCount_             = VK_SAMPLE_COUNT_1_BIT;
    bool                  srgbOutput_              = false;
    bool                  dynamicRenderingEnabled_ = false;
    std::size_t           pipelineCacheMaxBytes_   = 64u * 1024u * 1024u; // refuse to load/save bigger caches
    bool buildFurPipeline(VkDevice device, char *resultScratch, std::size_t resultScratchCapacity);
    bool buildOverlayPipeline(VkDevice device, char *resultScratch, std::size_t resultScratchCapacity);

    VkRenderPass          renderPass_            = VK_NULL_HANDLE;
    VkRenderPass          renderPassResume_      = VK_NULL_HANDLE; // LOAD-op twin (mid-frame transfer resume)
    VkDescriptorSetLayout frameSetLayout_        = VK_NULL_HANDLE; // set 0: uniform buffer + probe storage + field sampler
    VkDescriptorSetLayout materialSetLayout_     = VK_NULL_HANDLE; // set 1: six combined image samplers
    VkDescriptorSetLayout materialExtensionSetLayout_ = VK_NULL_HANDLE; // set 2 (mesh): dynamic-offset material extension UBO
    VkPipelineLayout      pipelineLayout_        = VK_NULL_HANDLE;
    VkPipeline            pipeline_              = VK_NULL_HANDLE;
    VkPipeline            pipelineNoDepthWrite_  = VK_NULL_HANDLE;
    VkPipeline            pipelineTwoSided_      = VK_NULL_HANDLE;
    VkPipeline            pipelineTwoSidedNoDepthWrite_ = VK_NULL_HANDLE;
    // Declared-winding (CW front face) and transparent (alpha-blended)
    // mesh variants; see meshPipeline().
    VkPipeline            pipelineCwWinding_     = VK_NULL_HANDLE;
    VkPipeline            pipelineCwWindingNoDepthWrite_ = VK_NULL_HANDLE;
    VkPipeline            pipelineTwoSidedCwWinding_     = VK_NULL_HANDLE;
    VkPipeline            pipelineTwoSidedCwWindingNoDepthWrite_ = VK_NULL_HANDLE;
    VkPipeline            pipelineTransparent_   = VK_NULL_HANDLE;
    VkPipeline            pipelineTwoSidedTransparent_   = VK_NULL_HANDLE;
    VkPipeline            pipelineTransparentCwWinding_  = VK_NULL_HANDLE;
    VkPipeline            pipelineTwoSidedTransparentCwWinding_ = VK_NULL_HANDLE;
    VkPipelineLayout      furPipelineLayout_     = VK_NULL_HANDLE; // set layouts + 128-byte push constants
    VkPipeline            furPipeline_           = VK_NULL_HANDLE;
    VkPipelineLayout      overlayPipelineLayout_ = VK_NULL_HANDLE; // set 1 layout + 16-byte vertex push constants
    VkPipeline            overlayPipeline_       = VK_NULL_HANDLE;
    VkPipelineCache       pipelineCache_         = VK_NULL_HANDLE; // seeded from / saved to pipelineCachePath_
    std::string           pipelineCachePath_;
    bool                  cullBackFaces_ = true;

    // Target formats captured by build() so the fur pipeline (created by its
    // own build step) can fill its VkPipelineRenderingCreateInfo.
    VkFormat renderTargetColorFormat_ = VK_FORMAT_UNDEFINED;
    VkFormat renderTargetDepthFormat_ = VK_FORMAT_UNDEFINED;

    // Bring-up diagnostics (logged once by build()): bytes seeded from the
    // disk cache and the wall time spent creating both graphics pipelines.
    std::size_t    diskCacheBytesLoaded_ = 0;
    std::chrono::steady_clock::time_point pipelineBuildStart_{};
};

} // namespace renderer
} // namespace engine

#endif

