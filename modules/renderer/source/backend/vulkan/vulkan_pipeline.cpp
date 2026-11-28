

// Pipeline creation for the Vulkan backend: render pass, descriptor layouts,
// mesh + overlay graphics pipelines, and the disk-backed pipeline cache.
//
// Error reporting goes through resultName(), which pretty-prints VkResult
// values into a caller-provided scratch buffer.

#if defined(ENGINE_RENDERER_BACKEND_VULKAN)

#include "vulkan_device.hpp"
#include "vulkan_pipeline.hpp"

#include "fur_frag_gen.hpp"
#include "fur_vert_gen.hpp"
#include "mesh_frag_gen.hpp"
#include "mesh_vert_gen.hpp"
#include "ui_frag_gen.hpp"
#include "ui_vert_gen.hpp"

#include <engine/direct_lighting/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/mesh.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <ios>
#include <string>
#include <system_error>
#include <vector>
#include <vulkan/vulkan_core.h>

namespace engine
{
namespace renderer
{

static const char *resultName(VkResult result, char *scratch, std::size_t scratchCapacity)
{
    switch (result)
    {
    case VK_SUCCESS:
        return "VK_SUCCESS";
    case VK_NOT_READY:
        return "VK_NOT_READY";
    case VK_TIMEOUT:
        return "VK_TIMEOUT";
    case VK_EVENT_SET:
        return "VK_EVENT_SET";
    case VK_EVENT_RESET:
        return "VK_EVENT_RESET";
    case VK_INCOMPLETE:
        return "VK_INCOMPLETE";
    case VK_ERROR_OUT_OF_HOST_MEMORY:
        return "VK_ERROR_OUT_OF_HOST_MEMORY";
    case VK_ERROR_OUT_OF_DEVICE_MEMORY:
        return "VK_ERROR_OUT_OF_DEVICE_MEMORY";
    case VK_ERROR_INITIALIZATION_FAILED:
        return "VK_ERROR_INITIALIZATION_FAILED";
    case VK_ERROR_DEVICE_LOST:
        return "VK_ERROR_DEVICE_LOST";
    case VK_ERROR_MEMORY_MAP_FAILED:
        return "VK_ERROR_MEMORY_MAP_FAILED";
    case VK_ERROR_LAYER_NOT_PRESENT:
        return "VK_ERROR_LAYER_NOT_PRESENT";
    case VK_ERROR_EXTENSION_NOT_PRESENT:
        return "VK_ERROR_EXTENSION_NOT_PRESENT";
    case VK_ERROR_FEATURE_NOT_PRESENT:
        return "VK_ERROR_FEATURE_NOT_PRESENT";
    case VK_ERROR_INCOMPATIBLE_DRIVER:
        return "VK_ERROR_INCOMPATIBLE_DRIVER";
    case VK_ERROR_TOO_MANY_OBJECTS:
        return "VK_ERROR_TOO_MANY_OBJECTS";
    case VK_ERROR_FORMAT_NOT_SUPPORTED:
        return "VK_ERROR_FORMAT_NOT_SUPPORTED";
    case VK_ERROR_FRAGMENTED_POOL:
        return "VK_ERROR_FRAGMENTED_POOL";
    case VK_ERROR_UNKNOWN:
        return "VK_ERROR_UNKNOWN";
    case VK_ERROR_OUT_OF_POOL_MEMORY:
        return "VK_ERROR_OUT_OF_POOL_MEMORY";
    case VK_ERROR_INVALID_EXTERNAL_HANDLE:
        return "VK_ERROR_INVALID_EXTERNAL_HANDLE";
    case VK_ERROR_FRAGMENTATION:
        return "VK_ERROR_FRAGMENTATION";
    case VK_ERROR_INVALID_OPAQUE_CAPTURE_ADDRESS:
        return "VK_ERROR_INVALID_OPAQUE_CAPTURE_ADDRESS";
    case VK_PIPELINE_COMPILE_REQUIRED:
        return "VK_PIPELINE_COMPILE_REQUIRED";
    case VK_ERROR_INVALID_SHADER_NV:
        return "VK_ERROR_INVALID_SHADER_NV";
    case VK_ERROR_VALIDATION_FAILED_EXT:
        return "VK_ERROR_VALIDATION_FAILED_EXT";
    case VK_ERROR_FULL_SCREEN_EXCLUSIVE_MODE_LOST_EXT:
        return "VK_ERROR_FULL_SCREEN_EXCLUSIVE_MODE_LOST_EXT";
    case VK_ERROR_COMPRESSION_EXHAUSTED_EXT:
        return "VK_ERROR_COMPRESSION_EXHAUSTED_EXT";
#ifdef VK_ERROR_NOT_PERMITTED_KHR
    case VK_ERROR_NOT_PERMITTED_KHR:
        return "VK_ERROR_NOT_PERMITTED_KHR";
#endif
    default:
        break;
    }
    std::snprintf(scratch,
        scratchCapacity,
        "unrecognized VkResult %d (hex 0x%08X)",
        static_cast<int>(result),
        static_cast<unsigned>(result));
    return scratch;
}

namespace
{

// Where the pipeline cache file lives: $ENGINE_CACHE_DIR, then the platform
// cache directory, then the temp directory. Per vendor/device/driver file
// name in savePipelineCache's caller.
std::filesystem::path pipelineCacheDirectory()
{
    if (const char *overrideDirectory = std::getenv("ENGINE_CACHE_DIR");
        overrideDirectory != nullptr && overrideDirectory[0] != '\0')
        return std::filesystem::path(overrideDirectory) / "vulkan";

#if defined(_WIN32)
    if (const char *localAppData = std::getenv("LOCALAPPDATA"); localAppData != nullptr && localAppData[0] != '\0')
        return std::filesystem::path(localAppData) / "engine" / "cache" / "vulkan";
#elif defined(__APPLE__)
    if (const char *home = std::getenv("HOME"); home != nullptr && home[0] != '\0')
        return std::filesystem::path(home) / "Library" / "Caches" / "engine" / "vulkan";
#else
    if (const char *xdgCache = std::getenv("XDG_CACHE_HOME"); xdgCache != nullptr && xdgCache[0] != '\0')
        return std::filesystem::path(xdgCache) / "engine" / "vulkan";
    if (const char *home = std::getenv("HOME"); home != nullptr && home[0] != '\0')
        return std::filesystem::path(home) / ".cache" / "engine" / "vulkan";
#endif

    std::error_code error;
    const auto      temporaryDirectory = std::filesystem::temp_directory_path(error);
    return (error ? std::filesystem::path(".") : temporaryDirectory) / "engine" / "vulkan";
}

// Atomic-ish cache save: write to <path>.tmp then rename (with a
// remove-and-retry fallback on Windows-style rename failures).
std::size_t savePipelineCache(
    VkDevice device, VkPipelineCache cache, const std::string &path, std::size_t maxBytes = 64u * 1024u * 1024u)
{
    if (device == VK_NULL_HANDLE || cache == VK_NULL_HANDLE || path.empty())
        return 0;
    std::size_t dataSize = 0;
    if (vkGetPipelineCacheData(device, cache, &dataSize, nullptr) != VK_SUCCESS || dataSize == 0 || dataSize > maxBytes)
        return 0;
    std::vector<std::uint8_t> data(dataSize);
    if (vkGetPipelineCacheData(device, cache, &dataSize, data.data()) != VK_SUCCESS)
        return 0;

    const std::filesystem::path destination(path);
    std::error_code             error;
    std::filesystem::create_directories(destination.parent_path(), error);
    if (error)
        return 0;
    const std::filesystem::path temporary = destination.string() + ".tmp";
    {
        std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
        if (!stream)
            return 0;
        stream.write(reinterpret_cast<const char *>(data.data()), static_cast<std::streamsize>(dataSize));
        stream.flush();
        if (!stream)
        {
            stream.close();
            std::filesystem::remove(temporary, error);
            return 0;
        }
    }
    std::filesystem::rename(temporary, destination, error);
    if (error)
    {
        error.clear();
        std::filesystem::remove(destination, error);
        error.clear();
        std::filesystem::rename(temporary, destination, error);
        if (error)
        {
            std::filesystem::remove(temporary, error);
            return 0;
        }
    }
    return dataSize;
}

} // namespace

// Build order: render pass (one subpass, color+depth, external dependency
// for load/store overlap) -> descriptor layouts -> pipeline layout (2 sets +
// 112-byte push constants) -> shader modules from the generated SPIR-V ->
// graphics pipeline (the SRGB_OUTPUT specialization constant is wired here:
// constant_id 6 gets srgbOutput_) -> fur pipeline -> overlay pipeline. The
// pipeline cache is seeded from disk when a previous run saved one.
bool VulkanPipeline::build(const VulkanDeviceContext &device, VkFormat colorFormat, VkFormat depthFormat)
{
    const VkDevice vulkanDevice = device.device();

    char resultScratch[64];

    renderTargetColorFormat_ = colorFormat;
    renderTargetDepthFormat_ = depthFormat;

    {
        const VkAttachmentDescription colorAttachment{
            .flags          = 0,
            .format         = colorFormat,
            .samples        = sampleCount_,
            .loadOp         = VK_ATTACHMENT_LOAD_OP_CLEAR,
            .storeOp        = VK_ATTACHMENT_STORE_OP_STORE,
            .stencilLoadOp  = VK_ATTACHMENT_LOAD_OP_DONT_CARE,
            .stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE,
            .initialLayout  = VK_IMAGE_LAYOUT_UNDEFINED,
            .finalLayout    = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
        };
        const VkAttachmentDescription depthAttachment{
            .flags          = 0,
            .format         = depthFormat,
            .samples        = sampleCount_,
            .loadOp         = VK_ATTACHMENT_LOAD_OP_CLEAR,
            .storeOp        = VK_ATTACHMENT_STORE_OP_DONT_CARE,
            .stencilLoadOp  = VK_ATTACHMENT_LOAD_OP_DONT_CARE,
            .stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE,
            .initialLayout  = VK_IMAGE_LAYOUT_UNDEFINED,
            .finalLayout    = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
        };

        const VkAttachmentReference colorReference{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        const VkAttachmentReference depthReference{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
        const VkSubpassDescription  subpass{
            .flags                   = 0,
            .pipelineBindPoint       = VK_PIPELINE_BIND_POINT_GRAPHICS,
            .inputAttachmentCount    = 0,
            .pInputAttachments       = nullptr,
            .colorAttachmentCount    = 1,
            .pColorAttachments       = &colorReference,
            .pResolveAttachments     = nullptr,
            .pDepthStencilAttachment = &depthReference,
            .preserveAttachmentCount = 0,
            .pPreserveAttachments    = nullptr,
        };
        const VkSubpassDependency dependency{
            .srcSubpass    = VK_SUBPASS_EXTERNAL,
            .dstSubpass    = 0,
            .srcStageMask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT,
            .dstStageMask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT,
            .srcAccessMask = 0,
            .dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
            .dependencyFlags = 0,
        };
        const VkAttachmentDescription attachments[2] = {colorAttachment, depthAttachment};
        const VkRenderPassCreateInfo  renderPassInfo{
            .sType           = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO,
            .pNext           = nullptr,
            .flags           = 0,
            .attachmentCount = 2,
            .pAttachments    = attachments,
            .subpassCount    = 1,
            .pSubpasses      = &subpass,
            .dependencyCount = 1,
            .pDependencies   = &dependency,
        };
        const VkResult renderPassResult = vkCreateRenderPass(vulkanDevice, &renderPassInfo, nullptr, &renderPass_);
        if (renderPassResult != VK_SUCCESS)
        {
            std::fprintf(stderr,
                "renderer module: vkCreateRenderPass failed (%s)\n",
                resultName(renderPassResult, resultScratch, sizeof(resultScratch)));
            return false;
        }

        // Resume twin of the render pass for the mid-frame mesh-vertex update
        // path (see VulkanBackend::recordDynamicMeshUpdate): transfer commands
        // are illegal inside a render pass instance, so that path ends the
        // pass to record its copy and re-begins drawing here. The attachments
        // match the original exactly (formats and sample counts, so the
        // framebuffers stay compatible and rebind unchanged), but every load
        // operation is LOAD - the frame's already-drawn contents survive - and
        // the color attachment's initial layout is PRESENT_SRC_KHR, the
        // layout the interrupted first instance left the swapchain image in
        // (its finalLayout). The resume instance leaves it there again, so
        // endFrame presents exactly as the uninterrupted frame would.
        const VkAttachmentDescription resumeColorAttachment{
            .flags          = 0,
            .format         = colorFormat,
            .samples        = sampleCount_,
            .loadOp         = VK_ATTACHMENT_LOAD_OP_LOAD,
            .storeOp        = VK_ATTACHMENT_STORE_OP_STORE,
            .stencilLoadOp  = VK_ATTACHMENT_LOAD_OP_DONT_CARE,
            .stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE,
            .initialLayout  = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
            .finalLayout    = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
        };
        const VkAttachmentDescription resumeDepthAttachment{
            .flags          = 0,
            .format         = depthFormat,
            .samples        = sampleCount_,
            .loadOp         = VK_ATTACHMENT_LOAD_OP_LOAD,
            .storeOp        = VK_ATTACHMENT_STORE_OP_DONT_CARE,
            .stencilLoadOp  = VK_ATTACHMENT_LOAD_OP_DONT_CARE,
            .stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE,
            .initialLayout  = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
            .finalLayout    = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
        };
        const VkAttachmentDescription resumeAttachments[2] = {resumeColorAttachment, resumeDepthAttachment};
        const VkRenderPassCreateInfo  resumePassInfo{
            .sType           = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO,
            .pNext           = nullptr,
            .flags           = 0,
            .attachmentCount = 2,
            .pAttachments    = resumeAttachments,
            .subpassCount    = 1,
            .pSubpasses      = &subpass,
            .dependencyCount = 1,
            .pDependencies   = &dependency,
        };
        const VkResult resumePassResult
            = vkCreateRenderPass(vulkanDevice, &resumePassInfo, nullptr, &renderPassResume_);
        if (resumePassResult != VK_SUCCESS)
        {
            std::fprintf(stderr,
                "renderer module: vkCreateRenderPass (resume twin) failed (%s)\n",
                resultName(resumePassResult, resultScratch, sizeof(resultScratch)));
            return false;
        }
    }

    {
        const VkDescriptorSetLayoutBinding frameBindings[3]
            = {{.binding            = 0,
                   .descriptorType     = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                   .descriptorCount    = 1,
                   .stageFlags         = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                   .pImmutableSamplers = nullptr},
                // binding 1: the radiance_cache probe-grid storage buffer (read-only
                // from the fragment stage; zero-filled while the hook is
                // inactive). The overlay pipeline does not use this layout.
                {.binding            = 1,
                    .descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                    .descriptorCount    = 1,
                    .stageFlags         = VK_SHADER_STAGE_FRAGMENT_BIT,
                    .pImmutableSamplers = nullptr},
                // binding 2: the reflection_field screen-space field (RGBA32F,
                // sampled per fragment through texelFetch; a 1x1 neutral
                // stand-in stays bound while the hook is inactive).
                {.binding            = 2,
                    .descriptorType     = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    .descriptorCount    = 1,
                    .stageFlags         = VK_SHADER_STAGE_FRAGMENT_BIT,
                    .pImmutableSamplers = nullptr}};
        const VkDescriptorSetLayoutCreateInfo frameLayoutInfo{
            .sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
            .pNext        = nullptr,
            .flags        = 0,
            .bindingCount = 3,
            .pBindings    = frameBindings,
        };
        const VkResult frameLayoutResult
            = vkCreateDescriptorSetLayout(vulkanDevice, &frameLayoutInfo, nullptr, &frameSetLayout_);
        if (frameLayoutResult != VK_SUCCESS)
        {
            std::fprintf(stderr,
                "renderer module: vkCreateDescriptorSetLayout failed (%s)\n",
                resultName(frameLayoutResult, resultScratch, sizeof(resultScratch)));
            return false;
        }

        const VkDescriptorSetLayoutBinding materialBindings[6]
            = {{.binding               = 0,
                   .descriptorType     = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                   .descriptorCount    = 1,
                   .stageFlags         = VK_SHADER_STAGE_FRAGMENT_BIT,
                   .pImmutableSamplers = nullptr},
                {.binding               = 1,
                    .descriptorType     = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    .descriptorCount    = 1,
                    .stageFlags         = VK_SHADER_STAGE_FRAGMENT_BIT,
                    .pImmutableSamplers = nullptr},
                {.binding               = 2,
                    .descriptorType     = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    .descriptorCount    = 1,
                    .stageFlags         = VK_SHADER_STAGE_FRAGMENT_BIT,
                    .pImmutableSamplers = nullptr},
                {.binding               = 3,
                    .descriptorType     = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    .descriptorCount    = 1,
                    .stageFlags         = VK_SHADER_STAGE_FRAGMENT_BIT,
                    .pImmutableSamplers = nullptr},
                {.binding               = 4,
                    .descriptorType     = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    .descriptorCount    = 1,
                    .stageFlags         = VK_SHADER_STAGE_FRAGMENT_BIT,
                    .pImmutableSamplers = nullptr},
                {.binding               = 5,
                    .descriptorType     = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    .descriptorCount    = 1,
                    .stageFlags         = VK_SHADER_STAGE_FRAGMENT_BIT,
                    .pImmutableSamplers = nullptr}};
        const VkDescriptorSetLayoutCreateInfo materialLayoutInfo{
            .sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
            .pNext        = nullptr,
            .flags        = 0,
            .bindingCount = 6,
            .pBindings    = materialBindings,
        };
        const VkResult materialLayoutResult
            = vkCreateDescriptorSetLayout(vulkanDevice, &materialLayoutInfo, nullptr, &materialSetLayout_);
        if (materialLayoutResult != VK_SUCCESS)
        {
            std::fprintf(stderr,
                "renderer module: vkCreateDescriptorSetLayout failed (%s)\n",
                resultName(materialLayoutResult, resultScratch, sizeof(resultScratch)));
            return false;
        }
        // Set 2 (mesh pipelines only): the material extension block - a
        // dynamic-offset UBO whose 48-byte blocks carry the per-material
        // fields that do not fit the 128-byte push-constant budget (the
        // distance-fog triple, the procedural mottle pair and the near-field
        // relief overlay's triple; see mesh.frag's MaterialExtension and
        // VulkanResources' extension ring).
        const VkDescriptorSetLayoutBinding extensionBinding{
            .binding            = 0,
            .descriptorType     = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC,
            .descriptorCount    = 1,
            .stageFlags         = VK_SHADER_STAGE_FRAGMENT_BIT,
            .pImmutableSamplers = nullptr,
        };
        const VkDescriptorSetLayoutCreateInfo extensionLayoutInfo{
            .sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
            .pNext        = nullptr,
            .flags        = 0,
            .bindingCount = 1,
            .pBindings    = &extensionBinding,
        };
        const VkResult extensionLayoutResult
            = vkCreateDescriptorSetLayout(vulkanDevice, &extensionLayoutInfo, nullptr, &materialExtensionSetLayout_);
        if (extensionLayoutResult != VK_SUCCESS)
        {
            std::fprintf(stderr,
                "renderer module: vkCreateDescriptorSetLayout failed (%s)\n",
                resultName(extensionLayoutResult, resultScratch, sizeof(resultScratch)));
            return false;
        }
    }

    {
        const VkPushConstantRange pushRange{
            .stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            .offset     = 0,
            .size       = sizeof(PushConstants),
        };
        const VkDescriptorSetLayout      setLayouts[3] = {frameSetLayout_, materialSetLayout_,
            materialExtensionSetLayout_};
        const VkPipelineLayoutCreateInfo layoutInfo{
            .sType                  = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,
            .pNext                  = nullptr,
            .flags                  = 0,
            .setLayoutCount         = 3,
            .pSetLayouts            = setLayouts,
            .pushConstantRangeCount = 1,
            .pPushConstantRanges    = &pushRange,
        };
        const VkResult pipelineLayoutResult
            = vkCreatePipelineLayout(vulkanDevice, &layoutInfo, nullptr, &pipelineLayout_);
        if (pipelineLayoutResult != VK_SUCCESS)
        {
            std::fprintf(stderr,
                "renderer module: vkCreatePipelineLayout failed (%s)\n",
                resultName(pipelineLayoutResult, resultScratch, sizeof(resultScratch)));
            return false;
        }
    }

    VkShaderModule vertexModule   = VK_NULL_HANDLE;
    VkShaderModule fragmentModule = VK_NULL_HANDLE;
    {
        const VkShaderModuleCreateInfo vertexInfo{
            .sType    = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,
            .pNext    = nullptr,
            .flags    = 0,
            .codeSize = sizeof(meshVertexSpirv),
            .pCode    = meshVertexSpirv,
        };
        const VkShaderModuleCreateInfo fragmentInfo{
            .sType    = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,
            .pNext    = nullptr,
            .flags    = 0,
            .codeSize = sizeof(meshFragmentSpirv),
            .pCode    = meshFragmentSpirv,
        };
        VkResult shaderModuleResult = vkCreateShaderModule(vulkanDevice, &vertexInfo, nullptr, &vertexModule);
        if (shaderModuleResult == VK_SUCCESS)
        {
            shaderModuleResult = vkCreateShaderModule(vulkanDevice, &fragmentInfo, nullptr, &fragmentModule);
        }
        if (shaderModuleResult != VK_SUCCESS)
        {
            std::fprintf(stderr,
                "renderer module: vkCreateShaderModule failed (%s)\n",
                resultName(shaderModuleResult, resultScratch, sizeof(resultScratch)));
            if (vertexModule != VK_NULL_HANDLE)
            {
                vkDestroyShaderModule(vulkanDevice, vertexModule, nullptr);
            }
            return false;
        }
    }
    // The mesh pipeline family has three independent axes (see meshPipeline()
    // in the header): depth-write (backdrop shells and every transparent
    // draw leave the depth buffer alone), two-sidedness (cull mode NONE for
    // double-sided materials), winding convention (VK_FRONT_FACE_CLOCKWISE
    // for materials that declare the mirrored convention), and transparency
    // (src-alpha / one-minus-src-alpha blending, the software rasterizer's
    // transparent-commit twin). The transparent axis fixes depth-write off,
    // so the family holds eight opaque plus four blended variants. Everything
    // else - layout, shaders - is identical, so one builder lambda stamps
    // them all.
    const auto createMeshPipeline = [&](bool depthWriteEnable, bool twoSided, bool cwWinding, bool alphaBlend,
                                   VkPipeline &outPipeline) -> bool
    {
        // Plain locals (not static): the values derive from this build's own
        // srgbOutput_, so a second renderer in the process with a different
        // surface format must specialize from ITS own value - a function-local
        // static would keep the first renderer's constant forever.
        const std::uint32_t                  specData[2]    = {static_cast<std::uint32_t>(direct_lighting::maxDirectLights), srgbOutput_ ? 1u : 0u}; // MAX_LIGHTS, SRGB_OUT
        const VkSpecializationMapEntry specEntries[2] = {
            {0, 0, sizeof(std::uint32_t)},
            {6, sizeof(std::uint32_t), sizeof(std::uint32_t)},
        };
        const VkSpecializationInfo specInfo{
            .mapEntryCount = 2,
            .pMapEntries   = specEntries,
            .dataSize      = sizeof(specData),
            .pData         = specData,
        };

        const VkPipelineShaderStageCreateInfo stages[2]
            = {{.sType                  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
                   .pNext               = nullptr,
                   .flags               = 0,
                   .stage               = VK_SHADER_STAGE_VERTEX_BIT,
                   .module              = vertexModule,
                   .pName               = "main",
                   .pSpecializationInfo = nullptr},
                {.sType                  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
                    .pNext               = nullptr,
                    .flags               = 0,
                    .stage               = VK_SHADER_STAGE_FRAGMENT_BIT,
                    .module              = fragmentModule,
                    .pName               = "main",
                    .pSpecializationInfo = &specInfo}};

        const VkVertexInputBindingDescription vertexBindings[2]
            // binding 0: per-vertex mesh data (44-byte stride); binding 1: per-instance mat4
            = {{.binding = 0, .stride = meshVertexFloatCount * sizeof(float), .inputRate = VK_VERTEX_INPUT_RATE_VERTEX},
                {.binding = 1, .stride = sizeof(float) * 16u, .inputRate = VK_VERTEX_INPUT_RATE_INSTANCE}};

        const VkVertexInputAttributeDescription vertexAttributes[8]
            = {{.location = 0, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = 0},
                {.location = 1, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = 12},
                {.location = 2, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = 24},
                {.location = 3, .binding = 0, .format = VK_FORMAT_R32G32_SFLOAT, .offset = 36},
                {.location = 4, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = 0},
                {.location = 5, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = 16},
                {.location = 6, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = 32},
                {.location = 7, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = 48}};
        const VkPipelineVertexInputStateCreateInfo vertexInput{
            .sType                           = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO,
            .pNext                           = nullptr,
            .flags                           = 0,
            .vertexBindingDescriptionCount   = 2,
            .pVertexBindingDescriptions      = vertexBindings,
            .vertexAttributeDescriptionCount = 8,
            .pVertexAttributeDescriptions    = vertexAttributes,
        };
        const VkPipelineInputAssemblyStateCreateInfo inputAssembly{
            .sType                  = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,
            .pNext                  = nullptr,
            .flags                  = 0,
            .topology               = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST,
            .primitiveRestartEnable = VK_FALSE,
        };
        const VkPipelineViewportStateCreateInfo viewportState{
            .sType         = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,
            .pNext         = nullptr,
            .flags         = 0,
            .viewportCount = 1,
            .pViewports    = nullptr,
            .scissorCount  = 1,
            .pScissors     = nullptr,
        };

        const VkCullModeFlags cullMode = (cullBackFaces_ && !twoSided) ? VK_CULL_MODE_BACK_BIT : VK_CULL_MODE_NONE;
        // Consumers supply OpenGL-convention camera matrices (mesh.vert's
        // comment; the engine's front faces are CCW in that convention), and
        // the frame uniform's clip fixup flips Y into Vulkan clip space. The
        // flip mirrors every triangle's winding in the Y-down framebuffer
        // space, so a GL front face rasterizes as CLOCKWISE there: keeping
        // frontFace COUNTER_CLOCKWISE would cull every front face as a back
        // face under cullBackFaces_. CLOCKWISE keeps GL front faces as the
        // pipeline's front faces after the flip.
        const VkPipelineRasterizationStateCreateInfo rasterization{
            .sType                   = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,
            .pNext                   = nullptr,
            .flags                   = 0,
            .depthClampEnable        = VK_FALSE,
            .rasterizerDiscardEnable = VK_FALSE,
            .polygonMode             = VK_POLYGON_MODE_FILL,
            .cullMode                = cullMode,
            .frontFace               = cwWinding ? VK_FRONT_FACE_CLOCKWISE : VK_FRONT_FACE_COUNTER_CLOCKWISE,
            .depthBiasEnable         = VK_FALSE,
            .depthBiasConstantFactor = 0.0f,
            .depthBiasClamp          = 0.0f,
            .depthBiasSlopeFactor    = 0.0f,
            .lineWidth               = 1.0f,
        };
        const VkPipelineMultisampleStateCreateInfo multisample{
            .sType                 = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,
            .pNext                 = nullptr,
            .flags                 = 0,
            .rasterizationSamples  = sampleCount_,
            .sampleShadingEnable   = VK_FALSE,
            .minSampleShading      = 0.0f,
            .pSampleMask           = nullptr,
            .alphaToCoverageEnable = VK_FALSE,
            .alphaToOneEnable      = VK_FALSE,
        };
        const VkPipelineDepthStencilStateCreateInfo depthStencil{
            .sType                 = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO,
            .pNext                 = nullptr,
            .flags                 = 0,
            .depthTestEnable       = VK_TRUE,
            .depthWriteEnable      = (depthWriteEnable && !alphaBlend) ? VK_TRUE : VK_FALSE,
            .depthCompareOp        = VK_COMPARE_OP_LESS,
            .depthBoundsTestEnable = VK_FALSE,
            .stencilTestEnable     = VK_FALSE,
            .front                 = {},
            .back                  = {},
            .minDepthBounds        = 0.0f,
            .maxDepthBounds        = 1.0f,
        };
        const VkPipelineColorBlendAttachmentState colorAttachment{
            .blendEnable         = alphaBlend ? VK_TRUE : VK_FALSE,
            .srcColorBlendFactor = alphaBlend ? VK_BLEND_FACTOR_SRC_ALPHA : VK_BLEND_FACTOR_ONE,
            .dstColorBlendFactor = alphaBlend ? VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA : VK_BLEND_FACTOR_ZERO,
            .colorBlendOp        = VK_BLEND_OP_ADD,
            .srcAlphaBlendFactor = alphaBlend ? VK_BLEND_FACTOR_ONE : VK_BLEND_FACTOR_ONE,
            .dstAlphaBlendFactor = alphaBlend ? VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA : VK_BLEND_FACTOR_ZERO,
            .alphaBlendOp        = VK_BLEND_OP_ADD,
            .colorWriteMask
            = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT,
        };
        const VkPipelineColorBlendStateCreateInfo colorBlend{
            .sType           = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,
            .pNext           = nullptr,
            .flags           = 0,
            .logicOpEnable   = VK_FALSE,
            .logicOp         = VK_LOGIC_OP_COPY,
            .attachmentCount = 1,
            .pAttachments    = &colorAttachment,
            .blendConstants  = {0.0f, 0.0f, 0.0f, 0.0f},
        };
        // Viewport/scissor are dynamic states, so one pipeline serves every
        // window size (and the overlay scissor changes per command).
        const VkDynamicState                   dynamicStates[2] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
        const VkPipelineDynamicStateCreateInfo dynamicState{
            .sType             = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,
            .pNext             = nullptr,
            .flags             = 0,
            .dynamicStateCount = 2,
            .pDynamicStates    = dynamicStates,
        };

        if (pipelineCache_ == VK_NULL_HANDLE)
        {
            pipelineBuildStart_ = std::chrono::steady_clock::now();
            VkPhysicalDeviceProperties properties{};
            vkGetPhysicalDeviceProperties(device.physicalDevice(), &properties);
            const std::filesystem::path cacheFile
                = pipelineCacheDirectory()
                / ("pipeline_" + std::to_string(properties.vendorID) + "_" + std::to_string(properties.deviceID) + "_"
                    + std::to_string(properties.driverVersion) + ".bin");
            pipelineCachePath_ = cacheFile.string();

            std::vector<std::uint8_t> initialData;
            {
                std::ifstream stream(cacheFile, std::ios::binary | std::ios::ate);
                if (stream)
                {
                    const std::streamoff size = stream.tellg();
                    if (size > 0 && static_cast<std::size_t>(size) <= pipelineCacheMaxBytes_)
                    {
                        initialData.resize(static_cast<std::size_t>(size));
                        stream.seekg(0, std::ios::beg);
                        stream.read(reinterpret_cast<char *>(initialData.data()), size);
                        if (!stream)
                            initialData.clear();
                    }
                }
            }
            diskCacheBytesLoaded_ = initialData.size();
            const VkPipelineCacheCreateInfo cacheInfo{
                .sType           = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO,
                .pNext           = nullptr,
                .flags           = 0,
                .initialDataSize = initialData.size(),
                .pInitialData    = initialData.empty() ? nullptr : initialData.data(),
            };
            if (vkCreatePipelineCache(vulkanDevice, &cacheInfo, nullptr, &pipelineCache_) != VK_SUCCESS)
            {
                // A failed creation leaves the handle's contents unspecified,
                // and the graphics-pipeline creations below accept a null
                // cache (they simply run uncached): only a provably null
                // handle may reach them. A non-empty seeding payload gets one
                // retry with an empty cache - stale or foreign cache data is
                // dropped, which the disk-seed failure already implies.
                pipelineCache_ = VK_NULL_HANDLE;
                if (!initialData.empty())
                {
                    const VkPipelineCacheCreateInfo emptyCacheInfo{
                        .sType           = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO,
                        .pNext           = nullptr,
                        .flags           = 0,
                        .initialDataSize = 0,
                        .pInitialData    = nullptr,
                    };
                    if (vkCreatePipelineCache(vulkanDevice, &emptyCacheInfo, nullptr, &pipelineCache_) != VK_SUCCESS)
                    {
                        pipelineCache_ = VK_NULL_HANDLE;
                    }
                }
            }
        }

        const VkPipelineRenderingCreateInfo renderingInfo{
            .sType                   = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO,
            .pNext                   = nullptr,
            .viewMask                = 0,
            .colorAttachmentCount    = 1,
            .pColorAttachmentFormats = &colorFormat,
            .depthAttachmentFormat   = depthFormat,
            .stencilAttachmentFormat = VK_FORMAT_UNDEFINED,
        };
        const VkGraphicsPipelineCreateInfo pipelineInfo{
            .sType               = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO,
            .pNext               = dynamicRenderingEnabled_ ? &renderingInfo : nullptr,
            .flags               = 0,
            .stageCount          = 2,
            .pStages             = stages,
            .pVertexInputState   = &vertexInput,
            .pInputAssemblyState = &inputAssembly,
            .pTessellationState  = nullptr,
            .pViewportState      = &viewportState,
            .pRasterizationState = &rasterization,
            .pMultisampleState   = &multisample,
            .pDepthStencilState  = &depthStencil,
            .pColorBlendState    = &colorBlend,
            .pDynamicState       = &dynamicState,
            .layout              = pipelineLayout_,
            .renderPass          = dynamicRenderingEnabled_ ? VK_NULL_HANDLE : renderPass_,
            .subpass             = 0,
            .basePipelineHandle  = VK_NULL_HANDLE,
            .basePipelineIndex   = 0,
        };
        const VkResult pipelineResult
            = vkCreateGraphicsPipelines(vulkanDevice, pipelineCache_, 1, &pipelineInfo, nullptr, &outPipeline);
        if (pipelineResult != VK_SUCCESS)
        {
            std::fprintf(stderr,
                "renderer module: vkCreateGraphicsPipelines failed (%s)\n",
                resultName(pipelineResult, resultScratch, sizeof(resultScratch)));
            return false;
        }
        return true;
    };
    const auto buildMeshFamily = [this, &createMeshPipeline]() -> bool
    {
        const struct VariantSpec
        {
            bool     depthWrite;
            bool     twoSided;
            bool     cwWinding;
            bool     alphaBlend;
            VkPipeline *out;
        } variants[12] = {
            {true,  false, false, false, &pipeline_},
            {false, false, false, false, &pipelineNoDepthWrite_},
            {true,  true,  false, false, &pipelineTwoSided_},
            {false, true,  false, false, &pipelineTwoSidedNoDepthWrite_},
            {true,  false, true,  false, &pipelineCwWinding_},
            {false, false, true,  false, &pipelineCwWindingNoDepthWrite_},
            {true,  true,  true,  false, &pipelineTwoSidedCwWinding_},
            {false, true,  true,  false, &pipelineTwoSidedCwWindingNoDepthWrite_},
            {false, false, false, true,  &pipelineTransparent_},
            {false, true,  false, true,  &pipelineTwoSidedTransparent_},
            {false, false, true,  true,  &pipelineTransparentCwWinding_},
            {false, true,  true,  true,  &pipelineTwoSidedTransparentCwWinding_},
        };
        for (const VariantSpec &variant : variants)
        {
            if (!createMeshPipeline(variant.depthWrite, variant.twoSided, variant.cwWinding, variant.alphaBlend,
                    *variant.out))
            {
                return false;
            }
        }
        return true;
    };
    if (!buildMeshFamily())
    {
        vkDestroyShaderModule(vulkanDevice, vertexModule, nullptr);
        vkDestroyShaderModule(vulkanDevice, fragmentModule, nullptr);
        return false;
    }
    vkDestroyShaderModule(vulkanDevice, vertexModule, nullptr);
    vkDestroyShaderModule(vulkanDevice, fragmentModule, nullptr);
    if (!buildFurPipeline(vulkanDevice, resultScratch, sizeof(resultScratch)))
    {
        return false;
    }
    if (!buildOverlayPipeline(vulkanDevice, resultScratch, sizeof(resultScratch)))
    {
        return false;
    }

    // One-time bring-up diagnostic (never on the frame path): whether the
    // disk pipeline cache contributed anything and how long the three
    // graphics pipelines took to create. Cold builds and slow creates here
    // are exactly the first-use stall the P99 protocol looks for; a warm
    // cache should show near-zero times.
    if (pipelineBuildStart_.time_since_epoch().count() != 0)
    {
        const double buildMilliseconds
            = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - pipelineBuildStart_)
                  .count();
        std::fprintf(stderr,
            "renderer module: pipeline cache %s; mesh/fur/overlay pipelines built in %.1f ms\n",
            diskCacheBytesLoaded_ != 0 ? "seeded from disk" : "cold (no usable disk cache)",
            buildMilliseconds);
    }
    return true;
}

// Shell-fur pipeline: same render pass / descriptor layouts / vertex input as
// the mesh pipeline, but the fur_*_gen.hpp shaders, a 128-byte push constant
// range, alpha blending (src-alpha / one-minus-src-alpha), depth TEST on with
// depth WRITE off, and back-face culling like the mesh pipeline. The shell
// layers are nested outward from the base surface: drawing them in layer
// order (innermost first) composites back-to-front along every view ray that
// touches an outward-facing shell, and the base surface's depth rejects the
// far-side shells.
bool VulkanPipeline::buildFurPipeline(VkDevice device, char *resultScratch, std::size_t resultScratchCapacity)
{
    const VkPushConstantRange pushRange{
        .stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
        .offset     = 0,
        .size       = sizeof(FurPushConstants),
    };
    const VkDescriptorSetLayout      setLayouts[2] = {frameSetLayout_, materialSetLayout_};
    const VkPipelineLayoutCreateInfo layoutInfo{
        .sType                  = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,
        .pNext                  = nullptr,
        .flags                  = 0,
        .setLayoutCount         = 2,
        .pSetLayouts            = setLayouts,
        .pushConstantRangeCount = 1,
        .pPushConstantRanges    = &pushRange,
    };
    VkResult result = vkCreatePipelineLayout(device, &layoutInfo, nullptr, &furPipelineLayout_);
    if (result != VK_SUCCESS)
    {
        std::fprintf(stderr,
            "renderer module: fur pipeline layout creation failed (%s)\n",
            resultName(result, resultScratch, resultScratchCapacity));
        return false;
    }

    VkShaderModule                 vertexModule   = VK_NULL_HANDLE;
    VkShaderModule                 fragmentModule = VK_NULL_HANDLE;
    const VkShaderModuleCreateInfo vertexInfo{
        .sType    = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,
        .pNext    = nullptr,
        .flags    = 0,
        .codeSize = sizeof(furVertexSpirv),
        .pCode    = furVertexSpirv,
    };
    const VkShaderModuleCreateInfo fragmentInfo{
        .sType    = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,
        .pNext    = nullptr,
        .flags    = 0,
        .codeSize = sizeof(furFragmentSpirv),
        .pCode    = furFragmentSpirv,
    };
    result = vkCreateShaderModule(device, &vertexInfo, nullptr, &vertexModule);
    if (result == VK_SUCCESS)
        result = vkCreateShaderModule(device, &fragmentInfo, nullptr, &fragmentModule);
    if (result != VK_SUCCESS)
    {
        if (vertexModule != VK_NULL_HANDLE)
            vkDestroyShaderModule(device, vertexModule, nullptr);
        std::fprintf(stderr,
            "renderer module: fur shader module creation failed (%s)\n",
            resultName(result, resultScratch, resultScratchCapacity));
        return false;
    }

    // Same specialization contract as the mesh pipeline: MAX_LIGHTS
    // (constant_id 0) and SRGB_OUTPUT (constant_id 6) on the fragment stage.
    // Plain locals, same reason as the mesh pipeline's: per-build values.
    const std::uint32_t                  specData[2]    = {static_cast<std::uint32_t>(direct_lighting::maxDirectLights), srgbOutput_ ? 1u : 0u};
    const VkSpecializationMapEntry specEntries[2] = {
        {0, 0, sizeof(std::uint32_t)},
        {6, sizeof(std::uint32_t), sizeof(std::uint32_t)},
    };
    const VkSpecializationInfo specInfo{
        .mapEntryCount = 2,
        .pMapEntries   = specEntries,
        .dataSize      = sizeof(specData),
        .pData         = specData,
    };

    const VkPipelineShaderStageCreateInfo stages[2]
        = {{.sType                  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
               .pNext               = nullptr,
               .flags               = 0,
               .stage               = VK_SHADER_STAGE_VERTEX_BIT,
               .module              = vertexModule,
               .pName               = "main",
               .pSpecializationInfo = nullptr},
            {.sType                  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
                .pNext               = nullptr,
                .flags               = 0,
                .stage               = VK_SHADER_STAGE_FRAGMENT_BIT,
                .module              = fragmentModule,
                .pName               = "main",
                .pSpecializationInfo = &specInfo}};

    // Identical vertex input to the mesh pipeline (same vertex ABI, same
    // per-instance mat4 binding): fur shells draw the same meshes.
    const VkVertexInputBindingDescription vertexBindings[2]
        = {{.binding = 0, .stride = meshVertexFloatCount * sizeof(float), .inputRate = VK_VERTEX_INPUT_RATE_VERTEX},
            {.binding = 1, .stride = sizeof(float) * 16u, .inputRate = VK_VERTEX_INPUT_RATE_INSTANCE}};

    const VkVertexInputAttributeDescription vertexAttributes[8]
        = {{.location = 0, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = 0},
            {.location = 1, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = 12},
            {.location = 2, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = 24},
            {.location = 3, .binding = 0, .format = VK_FORMAT_R32G32_SFLOAT, .offset = 36},
            {.location = 4, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = 0},
            {.location = 5, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = 16},
            {.location = 6, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = 32},
            {.location = 7, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = 48}};
    const VkPipelineVertexInputStateCreateInfo vertexInput{
        .sType                           = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO,
        .pNext                           = nullptr,
        .flags                           = 0,
        .vertexBindingDescriptionCount   = 2,
        .pVertexBindingDescriptions      = vertexBindings,
        .vertexAttributeDescriptionCount = 8,
        .pVertexAttributeDescriptions    = vertexAttributes,
    };
    const VkPipelineInputAssemblyStateCreateInfo inputAssembly{
        .sType                  = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,
        .pNext                  = nullptr,
        .flags                  = 0,
        .topology               = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST,
        .primitiveRestartEnable = VK_FALSE,
    };
    const VkPipelineViewportStateCreateInfo viewportState{
        .sType         = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,
        .pNext         = nullptr,
        .flags         = 0,
        .viewportCount = 1,
        .pViewports    = nullptr,
        .scissorCount  = 1,
        .pScissors     = nullptr,
    };
    const VkPipelineRasterizationStateCreateInfo rasterization{
        .sType                   = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,
        .pNext                   = nullptr,
        .flags                   = 0,
        .depthClampEnable        = VK_FALSE,
        .rasterizerDiscardEnable = VK_FALSE,
        .polygonMode             = VK_POLYGON_MODE_FILL,
        .cullMode                = cullBackFaces_ ? VK_CULL_MODE_BACK_BIT : VK_CULL_MODE_NONE,
        // CLOCKWISE for the same reason as the mesh pipeline: the clip fixup
        // mirrors winding, so GL front faces must read as this pipeline's
        // front faces after the flip.
        .frontFace               = VK_FRONT_FACE_CLOCKWISE,
        .depthBiasEnable         = VK_FALSE,
        .depthBiasConstantFactor = 0.0f,
        .depthBiasClamp          = 0.0f,
        .depthBiasSlopeFactor    = 0.0f,
        .lineWidth               = 1.0f,
    };
    const VkPipelineMultisampleStateCreateInfo multisample{
        .sType                 = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,
        .pNext                 = nullptr,
        .flags                 = 0,
        .rasterizationSamples  = sampleCount_,
        .sampleShadingEnable   = VK_FALSE,
        .minSampleShading      = 0.0f,
        .pSampleMask           = nullptr,
        .alphaToCoverageEnable = VK_FALSE,
        .alphaToOneEnable      = VK_FALSE,
    };
    // Depth test on (shell layers behind the opaque base surface must be
    // rejected), depth write OFF (transparent layers must not occlude each
    // other or the geometry drawn after them).
    const VkPipelineDepthStencilStateCreateInfo depthStencil{
        .sType                 = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO,
        .pNext                 = nullptr,
        .flags                 = 0,
        .depthTestEnable       = VK_TRUE,
        .depthWriteEnable      = VK_FALSE,
        .depthCompareOp        = VK_COMPARE_OP_LESS,
        .depthBoundsTestEnable = VK_FALSE,
        .stencilTestEnable     = VK_FALSE,
        .front                 = {},
        .back                  = {},
        .minDepthBounds        = 0.0f,
        .maxDepthBounds        = 1.0f,
    };
    // Standard non-premultiplied alpha blending, matching the headless
    // rasterizer's fur write path (blend AFTER tone map + gamma encode, the
    // same space fixed-function blending operates in on the UNORM target).
    const VkPipelineColorBlendAttachmentState blendAttachment{
        .blendEnable         = VK_TRUE,
        .srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA,
        .dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA,
        .colorBlendOp        = VK_BLEND_OP_ADD,
        .srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE,
        .dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA,
        .alphaBlendOp        = VK_BLEND_OP_ADD,
        .colorWriteMask
        = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT,
    };
    const VkPipelineColorBlendStateCreateInfo colorBlend{
        .sType           = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,
        .pNext           = nullptr,
        .flags           = 0,
        .logicOpEnable   = VK_FALSE,
        .logicOp         = VK_LOGIC_OP_COPY,
        .attachmentCount = 1,
        .pAttachments    = &blendAttachment,
        .blendConstants  = {0.0f, 0.0f, 0.0f, 0.0f},
    };
    const VkDynamicState                   dynamicStates[2] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    const VkPipelineDynamicStateCreateInfo dynamicState{
        .sType             = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,
        .pNext             = nullptr,
        .flags             = 0,
        .dynamicStateCount = 2,
        .pDynamicStates    = dynamicStates,
    };
    const VkPipelineRenderingCreateInfo renderingInfo{
        .sType                   = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO,
        .pNext                   = nullptr,
        .viewMask                = 0,
        .colorAttachmentCount    = 1,
        .pColorAttachmentFormats = &renderTargetColorFormat_,
        .depthAttachmentFormat   = renderTargetDepthFormat_,
        .stencilAttachmentFormat = VK_FORMAT_UNDEFINED,
    };
    const VkGraphicsPipelineCreateInfo pipelineInfo{
        .sType               = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO,
        .pNext               = dynamicRenderingEnabled_ ? &renderingInfo : nullptr,
        .flags               = 0,
        .stageCount          = 2,
        .pStages             = stages,
        .pVertexInputState   = &vertexInput,
        .pInputAssemblyState = &inputAssembly,
        .pTessellationState  = nullptr,
        .pViewportState      = &viewportState,
        .pRasterizationState = &rasterization,
        .pMultisampleState   = &multisample,
        .pDepthStencilState  = &depthStencil,
        .pColorBlendState    = &colorBlend,
        .pDynamicState       = &dynamicState,
        .layout              = furPipelineLayout_,
        .renderPass          = dynamicRenderingEnabled_ ? VK_NULL_HANDLE : renderPass_,
        .subpass             = 0,
        .basePipelineHandle  = VK_NULL_HANDLE,
        .basePipelineIndex   = 0,
    };
    result = vkCreateGraphicsPipelines(device, pipelineCache_, 1, &pipelineInfo, nullptr, &furPipeline_);
    vkDestroyShaderModule(device, vertexModule, nullptr);
    vkDestroyShaderModule(device, fragmentModule, nullptr);
    if (result != VK_SUCCESS)
    {
        std::fprintf(stderr,
            "renderer module: fur graphics pipeline creation failed (%s)\n",
            resultName(result, resultScratch, resultScratchCapacity));
        return false;
    }
    return true;
}

// Overlay pipeline: same target layouts as the mesh pipeline but no depth
// test, src-alpha/one-minus-src-alpha blending, no culling, a 16-byte vertex
// push constant (scale + translate), and the ui_*_gen.hpp shaders.
bool VulkanPipeline::buildOverlayPipeline(VkDevice device, char *resultScratch, std::size_t resultScratchCapacity)
{
    const VkPushConstantRange pushRange{
        .stageFlags = VK_SHADER_STAGE_VERTEX_BIT,
        .offset     = 0,
        .size       = 16,
    };
    const VkPipelineLayoutCreateInfo layoutInfo{
        .sType                  = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,
        .pNext                  = nullptr,
        .flags                  = 0,
        .setLayoutCount         = 1,
        .pSetLayouts            = &materialSetLayout_,
        .pushConstantRangeCount = 1,
        .pPushConstantRanges    = &pushRange,
    };
    VkResult result = vkCreatePipelineLayout(device, &layoutInfo, nullptr, &overlayPipelineLayout_);
    if (result != VK_SUCCESS)
    {
        std::fprintf(stderr,
            "renderer module: overlay pipeline layout creation failed (%s)\n",
            resultName(result, resultScratch, resultScratchCapacity));
        return false;
    }

    VkShaderModule                 vertexModule   = VK_NULL_HANDLE;
    VkShaderModule                 fragmentModule = VK_NULL_HANDLE;
    const VkShaderModuleCreateInfo vertexInfo{
        .sType    = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,
        .pNext    = nullptr,
        .flags    = 0,
        .codeSize = sizeof(uiVertexSpirv),
        .pCode    = uiVertexSpirv,
    };
    const VkShaderModuleCreateInfo fragmentInfo{
        .sType    = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,
        .pNext    = nullptr,
        .flags    = 0,
        .codeSize = sizeof(uiFragmentSpirv),
        .pCode    = uiFragmentSpirv,
    };
    result = vkCreateShaderModule(device, &vertexInfo, nullptr, &vertexModule);
    if (result == VK_SUCCESS)
        result = vkCreateShaderModule(device, &fragmentInfo, nullptr, &fragmentModule);
    if (result != VK_SUCCESS)
    {
        if (vertexModule != VK_NULL_HANDLE)
            vkDestroyShaderModule(device, vertexModule, nullptr);
        std::fprintf(stderr,
            "renderer module: overlay shader module creation failed (%s)\n",
            resultName(result, resultScratch, resultScratchCapacity));
        return false;
    }

    const VkPipelineShaderStageCreateInfo stages[2]{
        {.sType                  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
            .pNext               = nullptr,
            .flags               = 0,
            .stage               = VK_SHADER_STAGE_VERTEX_BIT,
            .module              = vertexModule,
            .pName               = "main",
            .pSpecializationInfo = nullptr},
        {.sType                  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
            .pNext               = nullptr,
            .flags               = 0,
            .stage               = VK_SHADER_STAGE_FRAGMENT_BIT,
            .module              = fragmentModule,
            .pName               = "main",
            .pSpecializationInfo = nullptr}};
    const VkVertexInputBindingDescription binding{
        .binding   = 0,
        .stride    = sizeof(OverlayVertex),
        .inputRate = VK_VERTEX_INPUT_RATE_VERTEX,
    };
    const VkVertexInputAttributeDescription attributes[3]{
        {.location   = 0,
            .binding = 0,
            .format  = VK_FORMAT_R32G32_SFLOAT,
            .offset  = static_cast<std::uint32_t>(offsetof(OverlayVertex, position))},
        {.location   = 1,
            .binding = 0,
            .format  = VK_FORMAT_R32G32_SFLOAT,
            .offset  = static_cast<std::uint32_t>(offsetof(OverlayVertex, uv))},
        {.location   = 2,
            .binding = 0,
            .format  = VK_FORMAT_R8G8B8A8_UNORM,
            .offset  = static_cast<std::uint32_t>(offsetof(OverlayVertex, color))}};
    const VkPipelineVertexInputStateCreateInfo vertexInput{
        .sType                           = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO,
        .pNext                           = nullptr,
        .flags                           = 0,
        .vertexBindingDescriptionCount   = 1,
        .pVertexBindingDescriptions      = &binding,
        .vertexAttributeDescriptionCount = 3,
        .pVertexAttributeDescriptions    = attributes,
    };
    const VkPipelineInputAssemblyStateCreateInfo inputAssembly{
        .sType                  = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,
        .pNext                  = nullptr,
        .flags                  = 0,
        .topology               = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST,
        .primitiveRestartEnable = VK_FALSE,
    };
    const VkPipelineViewportStateCreateInfo viewportState{
        .sType         = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,
        .pNext         = nullptr,
        .flags         = 0,
        .viewportCount = 1,
        .pViewports    = nullptr,
        .scissorCount  = 1,
        .pScissors     = nullptr,
    };
    const VkPipelineRasterizationStateCreateInfo rasterization{
        .sType                   = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,
        .pNext                   = nullptr,
        .flags                   = 0,
        .depthClampEnable        = VK_FALSE,
        .rasterizerDiscardEnable = VK_FALSE,
        .polygonMode             = VK_POLYGON_MODE_FILL,
        .cullMode                = VK_CULL_MODE_NONE,
        .frontFace               = VK_FRONT_FACE_COUNTER_CLOCKWISE,
        .depthBiasEnable         = VK_FALSE,
        .depthBiasConstantFactor = 0.0f,
        .depthBiasClamp          = 0.0f,
        .depthBiasSlopeFactor    = 0.0f,
        .lineWidth               = 1.0f,
    };
    const VkPipelineMultisampleStateCreateInfo multisample{
        .sType                 = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,
        .pNext                 = nullptr,
        .flags                 = 0,
        .rasterizationSamples  = sampleCount_,
        .sampleShadingEnable   = VK_FALSE,
        .minSampleShading      = 0.0f,
        .pSampleMask           = nullptr,
        .alphaToCoverageEnable = VK_FALSE,
        .alphaToOneEnable      = VK_FALSE,
    };
    const VkPipelineDepthStencilStateCreateInfo depthStencil{
        .sType                 = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO,
        .pNext                 = nullptr,
        .flags                 = 0,
        .depthTestEnable       = VK_FALSE,
        .depthWriteEnable      = VK_FALSE,
        .depthCompareOp        = VK_COMPARE_OP_ALWAYS,
        .depthBoundsTestEnable = VK_FALSE,
        .stencilTestEnable     = VK_FALSE,
        .front                 = {},
        .back                  = {},
        .minDepthBounds        = 0.0f,
        .maxDepthBounds        = 1.0f,
    };
    const VkPipelineColorBlendAttachmentState blendAttachment{
        .blendEnable         = VK_TRUE,
        .srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA,
        .dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA,
        .colorBlendOp        = VK_BLEND_OP_ADD,
        .srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE,
        .dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA,
        .alphaBlendOp        = VK_BLEND_OP_ADD,
        .colorWriteMask
        = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT,
    };
    const VkPipelineColorBlendStateCreateInfo colorBlend{
        .sType           = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,
        .pNext           = nullptr,
        .flags           = 0,
        .logicOpEnable   = VK_FALSE,
        .logicOp         = VK_LOGIC_OP_COPY,
        .attachmentCount = 1,
        .pAttachments    = &blendAttachment,
        .blendConstants  = {0.0f, 0.0f, 0.0f, 0.0f},
    };
    const VkDynamicState                   dynamicStates[2] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    const VkPipelineDynamicStateCreateInfo dynamicState{
        .sType             = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,
        .pNext             = nullptr,
        .flags             = 0,
        .dynamicStateCount = 2,
        .pDynamicStates    = dynamicStates,
    };
    // Same dynamic-rendering chain the mesh/fur pipelines attach: the
    // pipeline declares its attachment formats inline instead of relying on
    // a null render pass alone.
    const VkFormat overlayColorFormat = renderTargetColorFormat_;
    const VkFormat overlayDepthFormat = renderTargetDepthFormat_;
    const VkPipelineRenderingCreateInfo renderingInfo{
        .sType                   = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO,
        .pNext                   = nullptr,
        .viewMask                = 0,
        .colorAttachmentCount    = 1,
        .pColorAttachmentFormats = &overlayColorFormat,
        .depthAttachmentFormat   = overlayDepthFormat,
        .stencilAttachmentFormat = VK_FORMAT_UNDEFINED,
    };
    const VkGraphicsPipelineCreateInfo pipelineInfo{
        .sType               = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO,
        .pNext               = dynamicRenderingEnabled_ ? &renderingInfo : nullptr,
        .flags               = 0,
        .stageCount          = 2,
        .pStages             = stages,
        .pVertexInputState   = &vertexInput,
        .pInputAssemblyState = &inputAssembly,
        .pTessellationState  = nullptr,
        .pViewportState      = &viewportState,
        .pRasterizationState = &rasterization,
        .pMultisampleState   = &multisample,
        .pDepthStencilState  = &depthStencil,
        .pColorBlendState    = &colorBlend,
        .pDynamicState       = &dynamicState,
        .layout              = overlayPipelineLayout_,
        .renderPass          = dynamicRenderingEnabled_ ? VK_NULL_HANDLE : renderPass_,
        .subpass             = 0,
        .basePipelineHandle  = VK_NULL_HANDLE,
        .basePipelineIndex   = 0,
    };
    result = vkCreateGraphicsPipelines(device, pipelineCache_, 1, &pipelineInfo, nullptr, &overlayPipeline_);
    vkDestroyShaderModule(device, vertexModule, nullptr);
    vkDestroyShaderModule(device, fragmentModule, nullptr);
    if (result != VK_SUCCESS)
    {
        std::fprintf(stderr,
            "renderer module: overlay graphics pipeline creation failed (%s)\n",
            resultName(result, resultScratch, resultScratchCapacity));
        return false;
    }
    return true;
}

// Destruction also saves the pipeline cache to disk for the next run.
void VulkanPipeline::destroy(VkDevice device)
{
    if (device == VK_NULL_HANDLE)
    {
        return;
    }
    if (overlayPipeline_ != VK_NULL_HANDLE)
    {
        vkDestroyPipeline(device, overlayPipeline_, nullptr);
        overlayPipeline_ = VK_NULL_HANDLE;
    }
    if (overlayPipelineLayout_ != VK_NULL_HANDLE)
    {
        vkDestroyPipelineLayout(device, overlayPipelineLayout_, nullptr);
        overlayPipelineLayout_ = VK_NULL_HANDLE;
    }
    if (furPipeline_ != VK_NULL_HANDLE)
    {
        vkDestroyPipeline(device, furPipeline_, nullptr);
        furPipeline_ = VK_NULL_HANDLE;
    }
    if (furPipelineLayout_ != VK_NULL_HANDLE)
    {
        vkDestroyPipelineLayout(device, furPipelineLayout_, nullptr);
        furPipelineLayout_ = VK_NULL_HANDLE;
    }
    if (pipeline_ != VK_NULL_HANDLE)
    {
        vkDestroyPipeline(device, pipeline_, nullptr);
        pipeline_ = VK_NULL_HANDLE;
    }
    if (pipelineNoDepthWrite_ != VK_NULL_HANDLE)
    {
        vkDestroyPipeline(device, pipelineNoDepthWrite_, nullptr);
        pipelineNoDepthWrite_ = VK_NULL_HANDLE;
    }
    if (pipelineTwoSided_ != VK_NULL_HANDLE)
    {
        vkDestroyPipeline(device, pipelineTwoSided_, nullptr);
        pipelineTwoSided_ = VK_NULL_HANDLE;
    }
    if (pipelineTwoSidedNoDepthWrite_ != VK_NULL_HANDLE)
    {
        vkDestroyPipeline(device, pipelineTwoSidedNoDepthWrite_, nullptr);
        pipelineTwoSidedNoDepthWrite_ = VK_NULL_HANDLE;
    }
    VkPipeline *const meshFamily[8] = {&pipelineCwWinding_, &pipelineCwWindingNoDepthWrite_, &pipelineTwoSidedCwWinding_,
        &pipelineTwoSidedCwWindingNoDepthWrite_, &pipelineTransparent_, &pipelineTwoSidedTransparent_,
        &pipelineTransparentCwWinding_, &pipelineTwoSidedTransparentCwWinding_};
    for (VkPipeline *pipeline : meshFamily)
    {
        if (*pipeline != VK_NULL_HANDLE)
        {
            vkDestroyPipeline(device, *pipeline, nullptr);
            *pipeline = VK_NULL_HANDLE;
        }
    }
    if (pipelineLayout_ != VK_NULL_HANDLE)
    {
        vkDestroyPipelineLayout(device, pipelineLayout_, nullptr);
        pipelineLayout_ = VK_NULL_HANDLE;
    }
    if (materialSetLayout_ != VK_NULL_HANDLE)
    {
        vkDestroyDescriptorSetLayout(device, materialSetLayout_, nullptr);
        materialSetLayout_ = VK_NULL_HANDLE;
    }
    if (materialExtensionSetLayout_ != VK_NULL_HANDLE)
    {
        vkDestroyDescriptorSetLayout(device, materialExtensionSetLayout_, nullptr);
        materialExtensionSetLayout_ = VK_NULL_HANDLE;
    }
    if (frameSetLayout_ != VK_NULL_HANDLE)
    {
        vkDestroyDescriptorSetLayout(device, frameSetLayout_, nullptr);
        frameSetLayout_ = VK_NULL_HANDLE;
    }
    if (renderPass_ != VK_NULL_HANDLE)
    {
        vkDestroyRenderPass(device, renderPass_, nullptr);
        renderPass_ = VK_NULL_HANDLE;
    }
    if (renderPassResume_ != VK_NULL_HANDLE)
    {
        vkDestroyRenderPass(device, renderPassResume_, nullptr);
        renderPassResume_ = VK_NULL_HANDLE;
    }
    if (pipelineCache_ != VK_NULL_HANDLE)
    {
        (void)flushPipelineCacheToDisk(device);
        vkDestroyPipelineCache(device, pipelineCache_, nullptr);
        pipelineCache_ = VK_NULL_HANDLE;
    }
    pipelineCachePath_.clear();
}

// See the declaration in vulkan_pipeline.hpp: the whole point of this method
// is to move the cache save from "clean shutdown only" to "as soon as the
// compile work is done". A first launch that ends in a window close, Ctrl+C,
// or a crash must still leave the compiled pipelines on disk, or every later
// launch repeats the expensive driver compile.
std::size_t VulkanPipeline::flushPipelineCacheToDisk(VkDevice device)
{
    if (device == VK_NULL_HANDLE || pipelineCache_ == VK_NULL_HANDLE)
        return 0;
    const std::size_t savedBytes
        = savePipelineCache(device, pipelineCache_, pipelineCachePath_, pipelineCacheMaxBytes_);
    if (savedBytes != 0)
    {
        std::fprintf(stderr,
            "renderer module: pipeline cache flushed to disk (%zu bytes) - later launches reuse the "
            "compiled pipelines and start fast\n",
            savedBytes);
    }
    return savedBytes;
}

// Mesh-family variant resolution (see the axis table in the header): the
// transparent axis fixes depth-write off, so a transparent request ignores
// the depthWrite flag.
VkPipeline VulkanPipeline::meshPipeline(bool depthWrite, bool twoSided, bool cwWinding, bool transparent) const
{
    if (transparent)
    {
        if (twoSided)
        {
            return cwWinding ? pipelineTwoSidedTransparentCwWinding_ : pipelineTwoSidedTransparent_;
        }
        return cwWinding ? pipelineTransparentCwWinding_ : pipelineTransparent_;
    }
    if (twoSided)
    {
        if (cwWinding)
        {
            return depthWrite ? pipelineTwoSidedCwWinding_ : pipelineTwoSidedCwWindingNoDepthWrite_;
        }
        return depthWrite ? pipelineTwoSided_ : pipelineTwoSidedNoDepthWrite_;
    }
    if (cwWinding)
    {
        return depthWrite ? pipelineCwWinding_ : pipelineCwWindingNoDepthWrite_;
    }
    return depthWrite ? pipeline_ : pipelineNoDepthWrite_;
}

} // namespace renderer
} // namespace engine

#endif
