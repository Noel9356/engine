// Pipelines and descriptor layouts for the Vulkan backend: the mesh PBR
// pipeline, the alpha-blended overlay pipeline, a disk-backed pipeline cache,
// and the shader modules from the generated SPIR-V headers (mesh_*_gen.hpp,
// ui_*_gen.hpp). Compiled only under ENGINE_RENDER_BACKEND_VULKAN.

#ifndef ENGINE_RENDER_MODULE_VULKAN_PIPELINE_HPP
#define ENGINE_RENDER_MODULE_VULKAN_PIPELINE_HPP

#if defined(ENGINE_RENDER_BACKEND_VULKAN)
#include <cstddef>
#include <cstdint>
#include <string>
#include <vulkan/vulkan.h>
#include <vulkan/vulkan_core.h>

namespace engine
{
namespace render
{

class VulkanDeviceContext;

// Per-draw state pushed to both shader stages (112 bytes, matching the
// push_constant block declared in shaders/mesh.vert / mesh.frag).
// metallicRoughness packs (metallic, roughness, normalScale, occlusionStrength);
// emissive packs (emissiveColor.rgb, unlit flag).
struct PushConstants
{
    float model[16]          = {};
    float baseColorFactor[4] = {1.0f, 1.0f, 1.0f, 1.0f};

    float metallicRoughness[4] = {0.0f, 1.0f, 1.0f, 1.0f};

    float emissive[4] = {0.0f, 0.0f, 0.0f, 0.0f};
};
static_assert(sizeof(PushConstants) == 112, "the shader's push_constant block is 112 bytes");

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

    // Creates render pass, descriptor layouts (set 0: frame uniform buffer;
    // set 1: five combined image samplers), pipeline layout, mesh pipeline,
    // and overlay pipeline. srgbOutput_ and sampleCount_ must be set first.
    bool build(const VulkanDeviceContext &device, VkFormat colorFormat, VkFormat depthFormat);

    void destroy(VkDevice device);

    VkRenderPass renderPass() const
    {
        return renderPass_;
    }
    VkDescriptorSetLayout frameSetLayout() const
    {
        return frameSetLayout_;
    }
    VkDescriptorSetLayout materialSetLayout() const
    {
        return materialSetLayout_;
    }
    VkPipelineLayout pipelineLayout() const
    {
        return pipelineLayout_;
    }
    VkPipeline pipeline() const
    {
        return pipeline_;
    }
    VkPipelineLayout overlayPipelineLayout() const
    {
        return overlayPipelineLayout_;
    }
    VkPipeline overlayPipeline() const
    {
        return overlayPipeline_;
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
    VkPipeline pipelineForMaterialMask(std::uint32_t) // placeholder: one pipeline for all materials
    {
        return pipeline_;
    }

private:
    VkSampleCountFlagBits sampleCount_             = VK_SAMPLE_COUNT_1_BIT;
    bool                  srgbOutput_              = false;
    bool                  dynamicRenderingEnabled_ = false;
    std::size_t           pipelineCacheMaxBytes_   = 64u * 1024u * 1024u; // refuse to load/save bigger caches
    bool                  buildOverlayPipeline(VkDevice device, char *resultScratch, std::size_t resultScratchCapacity);

    VkRenderPass          renderPass_            = VK_NULL_HANDLE;
    VkDescriptorSetLayout frameSetLayout_        = VK_NULL_HANDLE; // set 0: one uniform buffer (FrameData)
    VkDescriptorSetLayout materialSetLayout_     = VK_NULL_HANDLE; // set 1: five combined image samplers
    VkPipelineLayout      pipelineLayout_        = VK_NULL_HANDLE;
    VkPipeline            pipeline_              = VK_NULL_HANDLE;
    VkPipelineLayout      overlayPipelineLayout_ = VK_NULL_HANDLE; // set 1 layout + 16-byte vertex push constants
    VkPipeline            overlayPipeline_       = VK_NULL_HANDLE;
    VkPipelineCache       pipelineCache_         = VK_NULL_HANDLE; // seeded from / saved to pipelineCachePath_
    std::string           pipelineCachePath_;
    bool                  cullBackFaces_ = true;
};

} // namespace render
} // namespace engine

#endif

#endif
