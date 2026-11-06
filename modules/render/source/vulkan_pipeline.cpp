

// Pipeline creation for the Vulkan backend: render pass, descriptor layouts,
// mesh + overlay graphics pipelines, and the disk-backed pipeline cache.
//
// Error reporting goes through resultName(), which pretty-prints VkResult
// values into a caller-provided scratch buffer.

#if defined(ENGINE_RENDER_BACKEND_VULKAN)

#include "vulkan_pipeline.hpp"
#include "vulkan_device.hpp"

#include "mesh_frag_gen.hpp"
#include "mesh_vert_gen.hpp"
#include "ui_frag_gen.hpp"
#include "ui_vert_gen.hpp"

#include "engine/render/capabilities.hpp"
#include "engine/render/mesh.hpp"

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
namespace render
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
void savePipelineCache(
    VkDevice device, VkPipelineCache cache, const std::string &path, std::size_t maxBytes = 64u * 1024u * 1024u)
{
    if (device == VK_NULL_HANDLE || cache == VK_NULL_HANDLE || path.empty())
        return;
    std::size_t dataSize = 0;
    if (vkGetPipelineCacheData(device, cache, &dataSize, nullptr) != VK_SUCCESS || dataSize == 0 || dataSize > maxBytes)
        return;
    std::vector<std::uint8_t> data(dataSize);
    if (vkGetPipelineCacheData(device, cache, &dataSize, data.data()) != VK_SUCCESS)
        return;

    const std::filesystem::path destination(path);
    std::error_code             error;
    std::filesystem::create_directories(destination.parent_path(), error);
    if (error)
        return;
    const std::filesystem::path temporary = destination.string() + ".tmp";
    {
        std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
        if (!stream)
            return;
        stream.write(reinterpret_cast<const char *>(data.data()), static_cast<std::streamsize>(dataSize));
        stream.flush();
        if (!stream)
        {
            stream.close();
            std::filesystem::remove(temporary, error);
            return;
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
            std::filesystem::remove(temporary, error);
    }
}

} // namespace

// Build order: render pass (one subpass, color+depth, external dependency
// for load/store overlap) -> descriptor layouts -> pipeline layout (2 sets +
// 112-byte push constants) -> shader modules from the generated SPIR-V ->
// graphics pipeline (the SRGB_OUTPUT specialization constant is wired here:
// constant_id 6 gets srgbOutput_) -> overlay pipeline. The pipeline cache is
// seeded from disk when a previous run saved one.
bool VulkanPipeline::build(const VulkanDeviceContext &device, VkFormat colorFormat, VkFormat depthFormat)
{
    const VkDevice vulkanDevice = device.device();

    char resultScratch[64];

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
                "render module: vkCreateRenderPass failed (%s)\n",
                resultName(renderPassResult, resultScratch, sizeof(resultScratch)));
            return false;
        }
    }

    {
        const VkDescriptorSetLayoutBinding frameBinding{
            .binding            = 0,
            .descriptorType     = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
            .descriptorCount    = 1,
            .stageFlags         = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            .pImmutableSamplers = nullptr,
        };
        const VkDescriptorSetLayoutCreateInfo frameLayoutInfo{
            .sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
            .pNext        = nullptr,
            .flags        = 0,
            .bindingCount = 1,
            .pBindings    = &frameBinding,
        };
        const VkResult frameLayoutResult
            = vkCreateDescriptorSetLayout(vulkanDevice, &frameLayoutInfo, nullptr, &frameSetLayout_);
        if (frameLayoutResult != VK_SUCCESS)
        {
            std::fprintf(stderr,
                "render module: vkCreateDescriptorSetLayout failed (%s)\n",
                resultName(frameLayoutResult, resultScratch, sizeof(resultScratch)));
            return false;
        }

        const VkDescriptorSetLayoutBinding materialBindings[5]
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
                    .pImmutableSamplers = nullptr}};
        const VkDescriptorSetLayoutCreateInfo materialLayoutInfo{
            .sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
            .pNext        = nullptr,
            .flags        = 0,
            .bindingCount = 5,
            .pBindings    = materialBindings,
        };
        const VkResult materialLayoutResult
            = vkCreateDescriptorSetLayout(vulkanDevice, &materialLayoutInfo, nullptr, &materialSetLayout_);
        if (materialLayoutResult != VK_SUCCESS)
        {
            std::fprintf(stderr,
                "render module: vkCreateDescriptorSetLayout failed (%s)\n",
                resultName(materialLayoutResult, resultScratch, sizeof(resultScratch)));
            return false;
        }
    }

    {
        const VkPushConstantRange pushRange{
            .stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            .offset     = 0,
            .size       = sizeof(PushConstants),
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
        const VkResult pipelineLayoutResult
            = vkCreatePipelineLayout(vulkanDevice, &layoutInfo, nullptr, &pipelineLayout_);
        if (pipelineLayoutResult != VK_SUCCESS)
        {
            std::fprintf(stderr,
                "render module: vkCreatePipelineLayout failed (%s)\n",
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
                "render module: vkCreateShaderModule failed (%s)\n",
                resultName(shaderModuleResult, resultScratch, sizeof(resultScratch)));
            if (vertexModule != VK_NULL_HANDLE)
            {
                vkDestroyShaderModule(vulkanDevice, vertexModule, nullptr);
            }
            return false;
        }
    }
    {
        static const std::uint32_t            specData[2]    = {32u, srgbOutput_ ? 1u : 0u}; // MAX_LIGHTS, SRGB_OUT
        static const VkSpecializationMapEntry specEntries[2] = {
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

        const VkCullModeFlags cullMode = cullBackFaces_ ? VK_CULL_MODE_BACK_BIT : VK_CULL_MODE_NONE;
        const VkPipelineRasterizationStateCreateInfo rasterization{
            .sType                   = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,
            .pNext                   = nullptr,
            .flags                   = 0,
            .depthClampEnable        = VK_FALSE,
            .rasterizerDiscardEnable = VK_FALSE,
            .polygonMode             = VK_POLYGON_MODE_FILL,
            .cullMode                = cullMode,
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
            .depthTestEnable       = VK_TRUE,
            .depthWriteEnable      = VK_TRUE,
            .depthCompareOp        = VK_COMPARE_OP_LESS,
            .depthBoundsTestEnable = VK_FALSE,
            .stencilTestEnable     = VK_FALSE,
            .front                 = {},
            .back                  = {},
            .minDepthBounds        = 0.0f,
            .maxDepthBounds        = 1.0f,
        };
        const VkPipelineColorBlendAttachmentState colorAttachment{
            .blendEnable         = VK_FALSE,
            .srcColorBlendFactor = VK_BLEND_FACTOR_ONE,
            .dstColorBlendFactor = VK_BLEND_FACTOR_ZERO,
            .colorBlendOp        = VK_BLEND_OP_ADD,
            .srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE,
            .dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO,
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
            const VkPipelineCacheCreateInfo cacheInfo{
                .sType           = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO,
                .pNext           = nullptr,
                .flags           = 0,
                .initialDataSize = initialData.size(),
                .pInitialData    = initialData.empty() ? nullptr : initialData.data(),
            };
            if (vkCreatePipelineCache(vulkanDevice, &cacheInfo, nullptr, &pipelineCache_) != VK_SUCCESS
                && !initialData.empty())
            {
                const VkPipelineCacheCreateInfo emptyCacheInfo{
                    .sType           = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO,
                    .pNext           = nullptr,
                    .flags           = 0,
                    .initialDataSize = 0,
                    .pInitialData    = nullptr,
                };
                (void)vkCreatePipelineCache(vulkanDevice, &emptyCacheInfo, nullptr, &pipelineCache_);
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
            = vkCreateGraphicsPipelines(vulkanDevice, pipelineCache_, 1, &pipelineInfo, nullptr, &pipeline_);
        vkDestroyShaderModule(vulkanDevice, vertexModule, nullptr);
        vkDestroyShaderModule(vulkanDevice, fragmentModule, nullptr);
        if (pipelineResult != VK_SUCCESS)
        {
            std::fprintf(stderr,
                "render module: vkCreateGraphicsPipelines failed (%s)\n",
                resultName(pipelineResult, resultScratch, sizeof(resultScratch)));
            return false;
        }
    }
    if (!buildOverlayPipeline(vulkanDevice, resultScratch, sizeof(resultScratch)))
    {
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
            "render module: overlay pipeline layout creation failed (%s)\n",
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
            "render module: overlay shader module creation failed (%s)\n",
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
    const VkGraphicsPipelineCreateInfo pipelineInfo{
        .sType               = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO,
        .pNext               = nullptr,
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
            "render module: overlay graphics pipeline creation failed (%s)\n",
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
    if (pipeline_ != VK_NULL_HANDLE)
    {
        vkDestroyPipeline(device, pipeline_, nullptr);
        pipeline_ = VK_NULL_HANDLE;
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
    if (pipelineCache_ != VK_NULL_HANDLE)
    {
        savePipelineCache(device, pipelineCache_, pipelineCachePath_, pipelineCacheMaxBytes_);
        vkDestroyPipelineCache(device, pipelineCache_, nullptr);
        pipelineCache_ = VK_NULL_HANDLE;
    }
    pipelineCachePath_.clear();
}

} // namespace render
} // namespace engine

#endif
