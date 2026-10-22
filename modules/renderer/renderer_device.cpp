#include "renderer_state.hpp"

#include "vulkan_utilities.hpp"
#include "engine/kernel/filesystem_utilities.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace engine
{

bool validationLayersAvailable()
{
    const std::vector<vk::LayerProperties> availableLayers = vk::enumerateInstanceLayerProperties();
    for (const vk::LayerProperties& layer : availableLayers)
    {
        if (std::string_view(layer.layerName.data()) == validationLayerName)
        {
            return true;
        }
    }
    return false;
}

namespace
{

// Debug messenger callback in the vk-namespace signature: passing it to
// setPfnUserCallback selects the typed setter instead of the one deprecated
// for C function pointer types. Errors are also remembered in the renderer
// state so hosts and automated runs can observe them through the capability
// contract instead of having to scrape the log.
VKAPI_ATTR vk::Bool32 VKAPI_CALL debugMessageCallback(vk::DebugUtilsMessageSeverityFlagBitsEXT severity,
                                                      vk::DebugUtilsMessageTypeFlagsEXT /*messageTypes*/,
                                                      const vk::DebugUtilsMessengerCallbackDataEXT* callbackData,
                                                      void* userData)
{
    if (callbackData == nullptr)
    {
        return VK_FALSE;
    }
    auto* state = static_cast<RendererState*>(userData);
    if (state == nullptr)
    {
        return VK_FALSE;
    }
    const bool isError = severity >= vk::DebugUtilsMessageSeverityFlagBitsEXT::eError;
    if (isError)
    {
        state->validationErrorSeen.store(true);
    }
    if (state->validationLogger != nullptr)
    {
        const LogLevel level = isError ? LogLevel::Error : LogLevel::Warning;
        state->validationLogger->logMessage(level, std::string("vulkan validation: ") + callbackData->pMessage);
    }
    return VK_FALSE;
}

} // namespace

QueueFamilyIndices RendererState::findQueueFamilies(const vk::raii::PhysicalDevice& candidate,
                                                    const vk::raii::SurfaceKHR& targetSurface) const
{
    QueueFamilyIndices indices;
    const std::vector<vk::QueueFamilyProperties> families = candidate.getQueueFamilyProperties();
    for (std::uint32_t familyIndex = 0; familyIndex < families.size(); ++familyIndex)
    {
        if (!indices.graphicsFamily.has_value() && (families[familyIndex].queueFlags & vk::QueueFlagBits::eGraphics))
        {
            indices.graphicsFamily = familyIndex;
        }
        if (!indices.presentFamily.has_value() && candidate.getSurfaceSupportKHR(familyIndex, targetSurface))
        {
            indices.presentFamily = familyIndex;
        }
        if (indices.complete())
        {
            break;
        }
    }
    return indices;
}

bool RendererState::supportsSwapchainExtension(const vk::raii::PhysicalDevice& candidate) const
{
    const std::vector<vk::ExtensionProperties> extensions = candidate.enumerateDeviceExtensionProperties();
    for (const vk::ExtensionProperties& extension : extensions)
    {
        if (std::string_view(extension.extensionName.data()) == swapchainExtensionName)
        {
            return true;
        }
    }
    return false;
}

std::int32_t RendererState::deviceScore(const vk::raii::PhysicalDevice& candidate) const
{
    const vk::PhysicalDeviceProperties properties = candidate.getProperties();
    std::int32_t score = 0;
    if (properties.deviceType == vk::PhysicalDeviceType::eDiscreteGpu)
    {
        score += preferDiscreteDevice ? preferredDeviceScoreBonus : 0;
    }
    else if (properties.deviceType == vk::PhysicalDeviceType::eIntegratedGpu)
    {
        score += preferDiscreteDevice ? 0 : preferredDeviceScoreBonus;
    }
    else
    {
        score += nonQueueDeviceScorePenalty;
    }
    // Small limits bonus (one point per 1000 supported 2D image dimensions,
    // capped): a deterministic tie-breaker that prefers larger GPUs.
    score += static_cast<std::int32_t>(std::min<std::uint32_t>(properties.limits.maxImageDimension2D,
                                                               maxImageDimensionScoreCap) /
                                       maxImageDimensionScoreDivisor);
    return score;
}

void RendererState::createInstance(WindowCapability& window, LoggerCapability& logger)
{
    validationEnabled = enableValidation && validationLayersAvailable();
    if (enableValidation && !validationEnabled)
    {
        logger.logMessage(LogLevel::Warning,
                          logPrefix + "validation layers requested but 'VK_LAYER_KHRONOS_validation' is not "
                          "installed; continuing without validation");
    }
    validationLogger = &logger;

    vk::ApplicationInfo applicationInfo;
    applicationInfo.setPApplicationName(vulkanApplicationName);
    applicationInfo.setApplicationVersion(vulkanInstanceVersion);
    applicationInfo.setPEngineName(vulkanApplicationName);
    applicationInfo.setEngineVersion(vulkanInstanceVersion);
    applicationInfo.setApiVersion(VK_API_VERSION_1_3);

    std::vector<std::string> extensionNames = window.requiredInstanceExtensions(vulkanBackendName);
    std::vector<const char*> extensionPointers;
    extensionPointers.reserve(extensionNames.size() + 1);
    for (const std::string& extension : extensionNames)
    {
        extensionPointers.push_back(extension.c_str());
    }
    if (validationEnabled)
    {
        extensionPointers.push_back(debugUtilsExtensionName);
    }

    std::vector<const char*> layerPointers;
    if (validationEnabled)
    {
        layerPointers.push_back(validationLayerName);
    }

    vk::InstanceCreateInfo createInfo;
    createInfo.setPApplicationInfo(&applicationInfo);
    createInfo.setPEnabledLayerNames(layerPointers);
    createInfo.setPEnabledExtensionNames(extensionPointers);

    instance = std::make_unique<vk::raii::Instance>(context, createInfo);

    if (validationEnabled)
    {
        vk::DebugUtilsMessengerCreateInfoEXT messengerInfo;
        messengerInfo.setMessageSeverity(vk::DebugUtilsMessageSeverityFlagBitsEXT::eWarning |
                                         vk::DebugUtilsMessageSeverityFlagBitsEXT::eError);
        messengerInfo.setMessageType(vk::DebugUtilsMessageTypeFlagBitsEXT::eGeneral |
                                     vk::DebugUtilsMessageTypeFlagBitsEXT::eValidation |
                                     vk::DebugUtilsMessageTypeFlagBitsEXT::ePerformance);
        messengerInfo.setPfnUserCallback(&debugMessageCallback);
        messengerInfo.setPUserData(this);
        debugMessenger = std::make_unique<vk::raii::DebugUtilsMessengerEXT>(*instance, messengerInfo);
    }
}

void RendererState::createDeviceAndSurface(WindowCapability& window, LoggerCapability& logger)
{
    // Opaque presentation surface from the window contract, interpreted for the
    // "vulkan" family: the handle is a VkSurfaceKHR created against this
    // instance, adopted by vk::raii::SurfaceKHR.
    const VkSurfaceKHR rawSurface = static_cast<VkSurfaceKHR>(
        window.createPresentationSurface(vulkanBackendName, static_cast<VkInstance>(**instance), targetWindow));
    surface = std::make_unique<vk::raii::SurfaceKHR>(*instance, rawSurface);

    vk::raii::PhysicalDevices physicalDevices(*instance);
    // A CPU or software device scores below zero (base -100 plus a small
    // limits bonus), so the initial best score must not be a small positive
    // sentinel: it would silently reject the only device on software Vulkan.
    std::int32_t bestScore = std::numeric_limits<std::int32_t>::min();
    for (vk::raii::PhysicalDevice& candidate : physicalDevices)
    {
        const vk::PhysicalDeviceProperties candidateProperties = candidate.getProperties();
        const QueueFamilyIndices indices = findQueueFamilies(candidate, *surface);
        if (!indices.complete())
        {
            logger.logMessage(LogLevel::Warning,
                              logPrefix + "skipping physical device '" +
                                  std::string(candidateProperties.deviceName.data()) +
                                  "': no queue family with both graphics and present support");
            continue;
        }
        if (!supportsSwapchainExtension(candidate))
        {
            logger.logMessage(LogLevel::Warning,
                              logPrefix + "skipping physical device '" +
                                  std::string(candidateProperties.deviceName.data()) +
                                  "': no VK_KHR_swapchain device extension");
            continue;
        }
        if (candidateProperties.apiVersion < VK_API_VERSION_1_3)
        {
            logger.logMessage(LogLevel::Warning,
                              logPrefix + "skipping physical device '" +
                                  std::string(candidateProperties.deviceName.data()) +
                                  "': supports Vulkan " +
                                  std::to_string(VK_API_VERSION_MAJOR(candidateProperties.apiVersion)) + "." +
                                  std::to_string(VK_API_VERSION_MINOR(candidateProperties.apiVersion)) +
                                  " but the renderer requires 1.3 (dynamic rendering, sync2)");
            continue;
        }
        const std::int32_t score = deviceScore(candidate);
        if (score > bestScore)
        {
            bestScore = score;
            physicalDevice = std::make_unique<vk::raii::PhysicalDevice>(std::move(candidate));
            graphicsQueueFamily = *indices.graphicsFamily;
            presentQueueFamily = *indices.presentFamily;
        }
    }
    if (physicalDevice == nullptr)
    {
        throw std::runtime_error(
            logPrefix + "no physical device with graphics and present queues, VK_KHR_swapchain and Vulkan "
            "1.3 support");
    }

    const vk::PhysicalDeviceProperties properties = physicalDevice->getProperties();
    logger.logMessage(LogLevel::Information,
                      std::string(logPrefix + "using physical device '") + properties.deviceName.data() + "'");

    // Software rasterizers render every frame on the CPU: the shipped demo
    // configuration takes seconds per frame there, and the host loop polls
    // window events only between frames - the OS marks the window "not
    // responding" within its hang-detection window (~5 s). Name the situation
    // and the exact knobs instead of leaving the user with a frozen-looking
    // window (docs/robustness.md, problem 47).
    const std::string deviceName = properties.deviceName.data();
    if (deviceName.find("llvmpipe") != std::string::npos || deviceName.find("lavapipe") != std::string::npos ||
        deviceName.find("SwiftShader") != std::string::npos)
    {
        logger.logMessage(
            LogLevel::Warning,
            logPrefix + "software rasterizer detected (" + deviceName +
                ") - expect seconds per frame at full quality, and the window may be reported as 'not "
                "responding' while a frame renders. For interactive exploration lower [window] width/height "
                "(e.g. 960x540), [lighting.radiance_cascades] probe_stride (e.g. 12) and spatial_filter "
                "(e.g. 6), and drop ray_reflections from [lighting].techniques.");
    }

    std::vector<vk::DeviceQueueCreateInfo> queueCreateInfos;
    const float queuePriority = 1.0f;
    {
        std::vector<std::uint32_t> families{graphicsQueueFamily};
        if (presentQueueFamily != graphicsQueueFamily)
        {
            families.push_back(presentQueueFamily);
        }
        for (std::uint32_t family : families)
        {
            vk::DeviceQueueCreateInfo queueInfo;
            queueInfo.setQueueFamilyIndex(family);
            queueInfo.setQueuePriorities(queuePriority);
            queueCreateInfos.push_back(queueInfo);
        }
    }

    std::vector<const char*> deviceExtensions{swapchainExtensionName};
    vk::DeviceCreateInfo deviceInfo;
    deviceInfo.setQueueCreateInfos(queueCreateInfos);
    deviceInfo.setPEnabledExtensionNames(deviceExtensions);

    // Dynamic rendering and synchronization2 are core in Vulkan 1.3 but
    // remain device features that must be enabled explicitly: without
    // dynamic rendering a pipeline may not set renderPass to null, and
    // without synchronization2 every pipelineBarrier2 call is invalid. The
    // physical device was already filtered to 1.3.
    vk::PhysicalDeviceVulkan13Features vulkan13Features;
    vulkan13Features.setDynamicRendering(true);
    vulkan13Features.setSynchronization2(true);
    vk::PhysicalDeviceFeatures2 enabledFeatures;
    enabledFeatures.setPNext(&vulkan13Features);
    deviceInfo.setPNext(&enabledFeatures);

    device = std::make_unique<vk::raii::Device>(*physicalDevice, deviceInfo);

    graphicsQueue = device->getQueue(graphicsQueueFamily, 0);
    presentQueue = device->getQueue(presentQueueFamily, 0);
}

void RendererState::createSwapchain(WindowCapability& window, LoggerCapability& logger)
{
    const vk::SurfaceCapabilitiesKHR capabilities = physicalDevice->getSurfaceCapabilitiesKHR(**surface);
    const std::vector<vk::SurfaceFormatKHR> formats = physicalDevice->getSurfaceFormatsKHR(**surface);
    const std::vector<vk::PresentModeKHR> presentModes = physicalDevice->getSurfacePresentModesKHR(**surface);
    if (formats.empty())
    {
        throw std::runtime_error(logPrefix + "the surface supports no formats");
    }

    vk::SurfaceFormatKHR chosenFormat = formats.front();
    for (const vk::SurfaceFormatKHR& candidate : formats)
    {
        if (candidate.format == vk::Format::eB8G8R8A8Srgb &&
            candidate.colorSpace == vk::ColorSpaceKHR::eSrgbNonlinear)
        {
            chosenFormat = candidate;
            break;
        }
    }
    swapchainFormat = chosenFormat.format;

    vk::PresentModeKHR chosenPresentMode = vk::PresentModeKHR::eFifo;
    if (!vsync)
    {
        for (const vk::PresentModeKHR candidate : presentModes)
        {
            if (candidate == vk::PresentModeKHR::eMailbox)
            {
                chosenPresentMode = candidate;
                break;
            }
            if (candidate == vk::PresentModeKHR::eImmediate)
            {
                chosenPresentMode = candidate;
            }
        }
    }

    vk::Extent2D extent = capabilities.currentExtent;
    if (extent.width == std::numeric_limits<std::uint32_t>::max())
    {
        const std::uint32_t framebufferWidth = window.framebufferWidth(targetWindow);
        const std::uint32_t framebufferHeight = window.framebufferHeight(targetWindow);
        extent.width = std::clamp(framebufferWidth, capabilities.minImageExtent.width,
                                  capabilities.maxImageExtent.width);
        extent.height = std::clamp(framebufferHeight, capabilities.minImageExtent.height,
                                   capabilities.maxImageExtent.height);
    }

    std::uint32_t imageCount = capabilities.minImageCount + 1;
    if (capabilities.maxImageCount > 0 && imageCount > capabilities.maxImageCount)
    {
        imageCount = capabilities.maxImageCount;
    }

    vk::SwapchainCreateInfoKHR swapchainInfo;
    swapchainInfo.setSurface(**surface);
    swapchainInfo.setMinImageCount(imageCount);
    swapchainInfo.setImageFormat(swapchainFormat);
    swapchainInfo.setImageColorSpace(chosenFormat.colorSpace);
    swapchainInfo.setImageExtent(extent);
    swapchainInfo.setImageArrayLayers(1);
    swapchainInfo.setImageUsage(vk::ImageUsageFlagBits::eColorAttachment | vk::ImageUsageFlagBits::eTransferSrc);
    swapchainInfo.setPreTransform(capabilities.currentTransform);
    swapchainInfo.setCompositeAlpha(vk::CompositeAlphaFlagBitsKHR::eOpaque);
    swapchainInfo.setPresentMode(chosenPresentMode);
    swapchainInfo.setClipped(true);

    std::uint32_t families[2] = {graphicsQueueFamily, presentQueueFamily};
    if (graphicsQueueFamily != presentQueueFamily)
    {
        swapchainInfo.setImageSharingMode(vk::SharingMode::eConcurrent);
        swapchainInfo.setQueueFamilyIndices(families);
    }
    else
    {
        swapchainInfo.setImageSharingMode(vk::SharingMode::eExclusive);
    }

    swapchain = std::make_unique<vk::raii::SwapchainKHR>(*device, swapchainInfo);
    swapchainImages = swapchain->getImages();
    swapchainExtent = extent;
    createSwapchainImageViews();

    logger.logMessage(LogLevel::Information,
                      logPrefix + "swapchain " + std::to_string(extent.width) + "x" +
                          std::to_string(extent.height) + " with " + std::to_string(swapchainImages.size()) +
                          " images");
}

void RendererState::createSwapchainImageViews()
{
    swapchainImageViews.clear();
    swapchainImageViews.reserve(swapchainImages.size());
    for (const vk::Image image : swapchainImages)
    {
        vk::ImageViewCreateInfo viewInfo;
        viewInfo.setImage(image);
        viewInfo.setViewType(vk::ImageViewType::e2D);
        viewInfo.setFormat(swapchainFormat);
        viewInfo.setSubresourceRange(vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1));
        swapchainImageViews.emplace_back(*device, viewInfo);
    }
}

void RendererState::createFrameImage(FrameImage& target, vk::Extent2D extent, vk::Format format,
                                     vk::ImageUsageFlags usages, vk::ImageAspectFlags aspect,
                                     vk::ImageLayout finalLayout)
{
    vk::ImageCreateInfo imageInfo;
    imageInfo.setImageType(vk::ImageType::e2D);
    imageInfo.setFormat(format);
    imageInfo.setExtent(vk::Extent3D(extent.width, extent.height, 1));
    imageInfo.setMipLevels(1);
    imageInfo.setArrayLayers(1);
    imageInfo.setSamples(vk::SampleCountFlagBits::e1);
    imageInfo.setTiling(vk::ImageTiling::eOptimal);
    imageInfo.setUsage(usages);
    imageInfo.setSharingMode(vk::SharingMode::eExclusive);
    imageInfo.setInitialLayout(vk::ImageLayout::eUndefined);
    target.image = std::make_unique<vk::raii::Image>(*device, imageInfo);

    const vk::MemoryRequirements requirements = target.image->getMemoryRequirements();
    vk::MemoryAllocateInfo allocateInfo;
    allocateInfo.setAllocationSize(requirements.size);
    allocateInfo.setMemoryTypeIndex(findMemoryType(**physicalDevice, requirements.memoryTypeBits,
                                                   vk::MemoryPropertyFlagBits::eDeviceLocal));
    target.memory = std::make_unique<vk::raii::DeviceMemory>(*device, allocateInfo);
    target.image->bindMemory(*target.memory, 0);

    vk::ImageViewCreateInfo viewInfo;
    viewInfo.setImage(**target.image);
    viewInfo.setViewType(vk::ImageViewType::e2D);
    viewInfo.setFormat(format);
    viewInfo.setSubresourceRange(vk::ImageSubresourceRange(aspect, 0, 1, 0, 1));
    target.view = std::make_unique<vk::raii::ImageView>(*device, viewInfo);

    transitionImageLayout(**target.image, vk::ImageLayout::eUndefined, finalLayout, aspect);
}

void RendererState::transitionImageLayout(const vk::Image& image, vk::ImageLayout from, vk::ImageLayout to,
                                          vk::ImageAspectFlags aspect)
{
    // One-shot submission; used during setup only (frame layout discipline
    // keeps frame resources in their layouts permanently).
    vk::CommandBufferAllocateInfo allocateInfo;
    allocateInfo.setCommandPool(**commandPool);
    allocateInfo.setLevel(vk::CommandBufferLevel::ePrimary);
    allocateInfo.setCommandBufferCount(1);
    vk::raii::CommandBuffer setupBuffer = std::move(vk::raii::CommandBuffers(*device, allocateInfo).front());
    setupBuffer.begin(vk::CommandBufferBeginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit));

    vk::ImageMemoryBarrier2 barrier;
    barrier.setSrcStageMask(vk::PipelineStageFlagBits2::eAllCommands);
    barrier.setSrcAccessMask(vk::AccessFlags2());
    barrier.setDstStageMask(vk::PipelineStageFlagBits2::eAllCommands);
    barrier.setDstAccessMask(vk::AccessFlagBits2::eMemoryRead | vk::AccessFlagBits2::eMemoryWrite);
    barrier.setOldLayout(from);
    barrier.setNewLayout(to);
    barrier.setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
    barrier.setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
    barrier.setImage(image);
    barrier.setSubresourceRange(vk::ImageSubresourceRange(aspect, 0, 1, 0, 1));

    std::array<vk::ImageMemoryBarrier2, 1> barriers{barrier};
    vk::DependencyInfo dependency;
    dependency.setImageMemoryBarriers(barriers);
    setupBuffer.pipelineBarrier2(dependency);
    setupBuffer.end();

    vk::SubmitInfo submitInfo;
    submitInfo.setCommandBuffers(*setupBuffer);
    graphicsQueue.submit(submitInfo);
    graphicsQueue.waitIdle();
}

void RendererState::createFrameResources()
{
    const vk::Extent2D extent = swapchainExtent;
    constexpr auto colorUsage = vk::ImageUsageFlagBits::eColorAttachment | vk::ImageUsageFlagBits::eSampled |
                                vk::ImageUsageFlagBits::eStorage;
    constexpr auto sampledUsage = vk::ImageUsageFlagBits::eSampled | vk::ImageUsageFlagBits::eStorage;
    constexpr auto aspectColor = vk::ImageAspectFlagBits::eColor;

    createFrameImage(albedoImage, extent, gbufferAlbedoFormat, colorUsage, aspectColor,
                     vk::ImageLayout::eGeneral);
    createFrameImage(normalImage, extent, hdrColorFormat, colorUsage, aspectColor,
                     vk::ImageLayout::eGeneral);
    createFrameImage(emissiveImage, extent, hdrColorFormat, colorUsage, aspectColor,
                     vk::ImageLayout::eGeneral);
    if (hasDirectRadianceTarget)
    {
        createFrameImage(directRadianceImage, extent, hdrColorFormat, sampledUsage, aspectColor,
                         vk::ImageLayout::eGeneral);
    }
    if (hasIndirectDiffuseTarget)
    {
        createFrameImage(indirectRadianceImage, extent, hdrColorFormat, sampledUsage, aspectColor,
                         vk::ImageLayout::eGeneral);
    }
    if (hasIndirectSpecularTarget)
    {
        createFrameImage(indirectSpecularImage, extent, hdrColorFormat, sampledUsage, aspectColor,
                         vk::ImageLayout::eGeneral);
    }
    createFrameImage(depthImage, extent, gbufferDepthFormat,
                     vk::ImageUsageFlagBits::eDepthStencilAttachment | vk::ImageUsageFlagBits::eSampled,
                     vk::ImageAspectFlagBits::eDepth, vk::ImageLayout::eGeneral);

    // The dummy stands in for disabled lighting targets: the composite
    // descriptor layout always declares the bindings, the flags in the push
    // constants decide whether they are read.
    createFrameImage(dummyImage, vk::Extent2D(1, 1), hdrColorFormat,
                     sampledUsage | vk::ImageUsageFlagBits::eTransferDst, aspectColor, vk::ImageLayout::eGeneral);
    clearFrameImage(dummyImage, {0.0f, 0.0f, 0.0f, 1.0f});

    vk::SamplerCreateInfo samplerInfo;
    samplerInfo.setMagFilter(vk::Filter::eLinear);
    samplerInfo.setMinFilter(vk::Filter::eLinear);
    samplerInfo.setMipmapMode(vk::SamplerMipmapMode::eNearest);
    samplerInfo.setAddressModeU(vk::SamplerAddressMode::eClampToEdge);
    samplerInfo.setAddressModeV(vk::SamplerAddressMode::eClampToEdge);
    samplerInfo.setAddressModeW(vk::SamplerAddressMode::eClampToEdge);
    linearSampler = std::make_unique<vk::raii::Sampler>(*device, samplerInfo);
}

void RendererState::releaseFrameResources()
{
    albedoImage = FrameImage{};
    normalImage = FrameImage{};
    emissiveImage = FrameImage{};
    directRadianceImage = FrameImage{};
    indirectRadianceImage = FrameImage{};
    indirectSpecularImage = FrameImage{};
    depthImage = FrameImage{};
    dummyImage = FrameImage{};
    linearSampler.reset();
}

void RendererState::clearFrameImage(const FrameImage& target, const std::array<float, 4>& color)
{
    vk::CommandBufferAllocateInfo allocateInfo;
    allocateInfo.setCommandPool(**commandPool);
    allocateInfo.setLevel(vk::CommandBufferLevel::ePrimary);
    allocateInfo.setCommandBufferCount(1);
    vk::raii::CommandBuffer setupBuffer = std::move(vk::raii::CommandBuffers(*device, allocateInfo).front());
    setupBuffer.begin(vk::CommandBufferBeginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit));
    vk::ClearColorValue clearValue(color);
    vk::ImageSubresourceRange range(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1);
    setupBuffer.clearColorImage(**target.image, vk::ImageLayout::eGeneral, clearValue, range);
    setupBuffer.end();
    vk::SubmitInfo submitInfo;
    submitInfo.setCommandBuffers(*setupBuffer);
    graphicsQueue.submit(submitInfo);
    graphicsQueue.waitIdle();
}
} // namespace engine
