

// Vulkan device context implementation: instance + surface bring-up,
// physical-device selection and scoring, logical device with the extension
// set the backend needs, the VMA allocator, the device profile snapshot, and
// the fenced staging-upload queue used for all host-to-device copies.

#if defined(ENGINE_RENDER_BACKEND_VULKAN)

#include "engine/render/capabilities.hpp"
#include "engine/render/device_profile.hpp"
#include "engine/window/capabilities.hpp"
#include "render_shared.hpp"

#include <vulkan/vk_platform.h>
#include <vulkan/vulkan_core.h>

#if defined(_WIN32)
#include <minwindef.h>
#include <windef.h>
#define VK_USE_PLATFORM_WIN32_KHR
#elif defined(__linux__)
#define VK_USE_PLATFORM_XLIB_KHR
#endif

#define VMA_IMPLEMENTATION
#include "vulkan_device.hpp"

#undef None

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <vector>

namespace engine
{
namespace render
{

namespace
{

// Validation-layer callback: forwards every warning/error to stderr.
VKAPI_ATTR VkBool32 VKAPI_CALL vulkanDebugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT,
    VkDebugUtilsMessageTypeFlagsEXT, const VkDebugUtilsMessengerCallbackDataEXT *callbackData, void *)
{
    std::fprintf(stderr, "render module (validation): %s\n", callbackData->pMessage);
    return VK_FALSE;
}

// Best-effort vendor name for the info strings; unknown ids fall back to a
// formatted "vendor 0x..." string at the call site.
const char *vendorName(std::uint32_t vendorId)
{
    switch (vendorId)
    {
    case 0x1002:
        return "AMD";
    case 0x10DE:
        return "NVIDIA";
    case 0x8086:
        return "Intel";
    case 0x13B5:
        return "ARM";
    case 0x5143:
        return "Qualcomm";
    case 0x1010:
        return "Imagination";
    default:
        return nullptr;
    }
}

DeviceClass deviceClassFromType(VkPhysicalDeviceType type)
{
    switch (type)
    {
    case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU:
        return DeviceClass::DiscreteGpu;
    case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU:
        return DeviceClass::IntegratedGpu;
    case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU:
        return DeviceClass::VirtualGpu;
    case VK_PHYSICAL_DEVICE_TYPE_CPU:
        return DeviceClass::Cpu;

    default:
        return DeviceClass::Unknown;
    }
}

// Whether a candidate device type satisfies the configured preference.
bool preferenceAccepts(DevicePreference preference, VkPhysicalDeviceType type)
{
    switch (preference)
    {
    case DevicePreference::Auto:
        return true;
    case DevicePreference::Discrete:
        return type == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU;
    case DevicePreference::Integrated:
        return type == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU;
    }
    return true;
}

// Selection score: device class dominates, then a queue with timestamp
// support wins (score = class*2 + timestamps). Applied per queue family.
std::uint32_t deviceScore(VkPhysicalDeviceType type)
{
    switch (type)
    {
    case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU:
        return 3u;
    case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU:
        return 2u;
    default:
        return 1u;
    }
}

} // namespace

VulkanDeviceContext::~VulkanDeviceContext()
{
    destroy();
}

void VulkanDeviceContext::destroy()
{
    if (device_ != VK_NULL_HANDLE)
    {
        std::lock_guard queueLock(queueMutex_);
        vkDeviceWaitIdle(device_);
        collectCompletedUploadsLocked(true);
        if (uploadPool_ != VK_NULL_HANDLE)
        {
            vkDestroyCommandPool(device_, uploadPool_, nullptr);
            uploadPool_ = VK_NULL_HANDLE;
        }
        if (timelineSemaphore_ != VK_NULL_HANDLE)
        {
            vkDestroySemaphore(device_, timelineSemaphore_, nullptr);
            timelineSemaphore_ = VK_NULL_HANDLE;
        }
        if (allocator_ != nullptr)
        {
            vmaDestroyAllocator(allocator_);
            allocator_ = nullptr;
        }
        vkDestroyDevice(device_, nullptr);
        device_ = VK_NULL_HANDLE;
    }
    if (surface_ != VK_NULL_HANDLE && instance_ != VK_NULL_HANDLE)
    {
        vkDestroySurfaceKHR(instance_, surface_, nullptr);
        surface_ = VK_NULL_HANDLE;
    }
    if (messenger_ != VK_NULL_HANDLE && instance_ != VK_NULL_HANDLE)
    {
        const auto destroyMessenger = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
            vkGetInstanceProcAddr(instance_, "vkDestroyDebugUtilsMessengerEXT"));
        if (destroyMessenger != nullptr)
        {
            destroyMessenger(instance_, messenger_, nullptr);
        }
        messenger_ = VK_NULL_HANDLE;
    }
    if (instance_ != VK_NULL_HANDLE)
    {
        vkDestroyInstance(instance_, nullptr);
        instance_ = VK_NULL_HANDLE;
    }
}

// Full bring-up. Extension set: what window.vulkan requires, plus
// KHR_get_physical_device_properties2 when available (needed for features2
// queries). Validation adds VK_LAYER_KHRONOS_validation + debug utils when
// installed; a missing layer only warns. Device extensions: swapchain,
// memory budget (when present, enables VMA budget tracking), conservative
// rasterization (when present). Returns null with *error on any failure.
std::unique_ptr<VulkanDeviceContext> VulkanDeviceContext::create(window::WindowHandle surface, bool validation,
    DevicePreference preference, std::uint32_t maxPendingUploads, window::VulkanSurfaceCapability *vulkanSurface,
    RenderError *error)
{
    auto context                    = std::unique_ptr<VulkanDeviceContext>(new VulkanDeviceContext());
    context->maximumPendingUploads_ = std::clamp(maxPendingUploads, 1u, 4096u);

    window::NativeWindowHandles nativeHandles;
    if (vulkanSurface == nullptr || !vulkanSurface->nativeSurfaceHandles(vulkanSurface->self, surface, &nativeHandles))
    {
        reportError(error, RenderError::BackendUnavailable);
        return nullptr;
    }
    std::uint32_t      extensionCount = 0;
    const char *const *instanceExtensions
        = vulkanSurface->requiredInstanceExtensions(vulkanSurface->self, &extensionCount);
    if (instanceExtensions == nullptr || extensionCount == 0)
    {
        reportError(error, RenderError::BackendUnavailable);
        return nullptr;
    }

    const VkApplicationInfo applicationInfo{
        .sType              = VK_STRUCTURE_TYPE_APPLICATION_INFO,
        .pNext              = nullptr,
        .pApplicationName   = "engine",
        .applicationVersion = VK_MAKE_API_VERSION(0, 0, 1, 0),
        .pEngineName        = "engine",
        .engineVersion      = VK_MAKE_API_VERSION(0, 0, 1, 0),
        .apiVersion         = VK_API_VERSION_1_0,
    };

    std::vector<const char *> enabledExtensions(instanceExtensions, instanceExtensions + extensionCount);
    std::vector<const char *> enabledLayers;

    std::uint32_t availableInstanceExtensionCount = 0;
    vkEnumerateInstanceExtensionProperties(nullptr, &availableInstanceExtensionCount, nullptr);
    std::vector<VkExtensionProperties> availableInstanceExtensions(availableInstanceExtensionCount);
    vkEnumerateInstanceExtensionProperties(
        nullptr, &availableInstanceExtensionCount, availableInstanceExtensions.data());
    const bool hasPhysicalDeviceProperties2 = std::any_of(availableInstanceExtensions.begin(),
        availableInstanceExtensions.end(),
        [](const VkExtensionProperties &extension)
        { return std::strcmp(extension.extensionName, VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME) == 0; });
    if (hasPhysicalDeviceProperties2
        && std::none_of(enabledExtensions.begin(),
            enabledExtensions.end(),
            [](const char *extension)
            { return std::strcmp(extension, VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME) == 0; }))
    {
        enabledExtensions.push_back(VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME);
    }

    bool validationEnabled = validation;
    if (validationEnabled)
    {
        std::uint32_t layerCount = 0;
        vkEnumerateInstanceLayerProperties(&layerCount, nullptr);
        std::vector<VkLayerProperties> layers(layerCount);
        vkEnumerateInstanceLayerProperties(&layerCount, layers.data());
        const bool validationLayerAvailable = std::any_of(layers.begin(),
            layers.end(),
            [](const VkLayerProperties &layer)
            { return std::strcmp(layer.layerName, "VK_LAYER_KHRONOS_validation") == 0; });
        if (validationLayerAvailable)
        {
            enabledLayers.push_back("VK_LAYER_KHRONOS_validation");
            enabledExtensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
        }
        else
        {
            std::fprintf(
                stderr, "render module: [render] validation is on but VK_LAYER_KHRONOS_validation is not installed\n");
            validationEnabled = false;
        }
    }

    const VkInstanceCreateInfo instanceInfo{
        .sType                   = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
        .pNext                   = nullptr,
        .flags                   = 0,
        .pApplicationInfo        = &applicationInfo,
        .enabledLayerCount       = static_cast<std::uint32_t>(enabledLayers.size()),
        .ppEnabledLayerNames     = enabledLayers.data(),
        .enabledExtensionCount   = static_cast<std::uint32_t>(enabledExtensions.size()),
        .ppEnabledExtensionNames = enabledExtensions.data(),
    };
    if (vkCreateInstance(&instanceInfo, nullptr, &context->instance_) != VK_SUCCESS)
    {
        std::fprintf(stderr, "render module: vkCreateInstance failed\n");
        reportError(error, RenderError::CreationFailed);
        return nullptr;
    }

    if (validationEnabled)
    {
        const auto createMessenger = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
            vkGetInstanceProcAddr(context->instance_, "vkCreateDebugUtilsMessengerEXT"));
        if (createMessenger != nullptr)
        {
            const VkDebugUtilsMessengerCreateInfoEXT messengerInfo{
                .sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT,
                .pNext = nullptr,
                .flags = 0,
                .messageSeverity
                = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT,
                .messageType     = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT
                                 | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT
                                 | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT,
                .pfnUserCallback = vulkanDebugCallback,
                .pUserData       = nullptr,
            };
            createMessenger(context->instance_, &messengerInfo, nullptr, &context->messenger_);
        }
    }

    const bool wantHeadlessSurface = (nativeHandles.win32Hwnd == nullptr && nativeHandles.x11Window == nullptr
                                      && nativeHandles.x11Display == nullptr); // window.vulkan found no window
    if (wantHeadlessSurface)
    {
        PFN_vkCreateHeadlessSurfaceEXT createHeadlessSurface = reinterpret_cast<PFN_vkCreateHeadlessSurfaceEXT>(
            vkGetInstanceProcAddr(context->instance_, "vkCreateHeadlessSurfaceEXT"));
        if (createHeadlessSurface == nullptr)
        {
            std::fprintf(stderr,
                "render module: VK_EXT_headless_surface is not available; cannot create a surface without native "
                "handles\n");
            reportError(error, RenderError::BackendUnavailable);
            return nullptr;
        }
        const VkHeadlessSurfaceCreateInfoEXT headlessInfo{
            .sType = VK_STRUCTURE_TYPE_HEADLESS_SURFACE_CREATE_INFO_EXT,
            .pNext = nullptr,
            .flags = 0,
        };
        if (createHeadlessSurface(context->instance_, &headlessInfo, nullptr, &context->surface_) != VK_SUCCESS)
        {
            std::fprintf(stderr, "render module: vkCreateHeadlessSurfaceEXT failed\n");
            reportError(error, RenderError::CreationFailed);
            return nullptr;
        }
    }
    else
    {
#if defined(_WIN32)
        {
            const VkWin32SurfaceCreateInfoKHR surfaceInfo{
                .sType     = VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR,
                .pNext     = nullptr,
                .flags     = 0,
                .hinstance = static_cast<HINSTANCE>(nativeHandles.win32Hinstance),
                .hwnd      = static_cast<HWND>(nativeHandles.win32Hwnd),
            };
            if (vkCreateWin32SurfaceKHR(context->instance_, &surfaceInfo, nullptr, &context->surface_) != VK_SUCCESS)
            {
                std::fprintf(stderr, "render module: vkCreateWin32SurfaceKHR failed\n");
                reportError(error, RenderError::CreationFailed);
                return nullptr;
            }
        }
#elif defined(__linux__)
        {
            const VkXlibSurfaceCreateInfoKHR surfaceInfo{
                .sType  = VK_STRUCTURE_TYPE_XLIB_SURFACE_CREATE_INFO_KHR,
                .pNext  = nullptr,
                .flags  = 0,
                .dpy    = static_cast<Display *>(nativeHandles.x11Display),
                .window = static_cast<Window>(reinterpret_cast<std::uintptr_t>(nativeHandles.x11Window)),
            };
            if (vkCreateXlibSurfaceKHR(context->instance_, &surfaceInfo, nullptr, &context->surface_) != VK_SUCCESS)
            {
                std::fprintf(stderr, "render module: vkCreateXlibSurfaceKHR failed\n");
                reportError(error, RenderError::CreationFailed);
                return nullptr;
            }
        }
#else
        std::fprintf(stderr, "render module: no Vulkan surface path for this platform\n");
        reportError(error, RenderError::BackendUnavailable);
        return nullptr;
#endif
    }

    std::uint32_t physicalDeviceCount = 0;
    vkEnumeratePhysicalDevices(context->instance_, &physicalDeviceCount, nullptr);
    if (physicalDeviceCount == 0)
    {
        std::fprintf(stderr, "render module: no Vulkan physical devices\n");
        reportError(error, RenderError::CreationFailed);
        return nullptr;
    }
    std::vector<VkPhysicalDevice> physicalDevices(physicalDeviceCount);
    vkEnumeratePhysicalDevices(context->instance_, &physicalDeviceCount, physicalDevices.data());

    std::uint32_t    bestScore                   = 0;
    VkPhysicalDevice chosenDevice                = VK_NULL_HANDLE;
    std::uint32_t    chosenFamily                = 0;
    bool             chosenQueueSupportsCompute  = false;
    bool             chosenDeviceSupportsCompute = false;
    bool             chosenMemoryBudgetAvailable = false;
    std::uint32_t    chosenTimestampValidBits    = 0;
    for (VkPhysicalDevice candidate : physicalDevices)
    {
        // Requirements: swapchain extension, preferred device type, and a
        // queue family that does graphics AND presents to the surface.
        std::uint32_t familyCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(candidate, &familyCount, nullptr);
        std::vector<VkQueueFamilyProperties> families(familyCount);
        vkGetPhysicalDeviceQueueFamilyProperties(candidate, &familyCount, families.data());
        const bool deviceSupportsCompute = std::any_of(families.begin(),
            families.end(),
            [](const auto &family) { return (family.queueFlags & VK_QUEUE_COMPUTE_BIT) != 0u; });

        std::uint32_t deviceExtensionCount = 0;
        vkEnumerateDeviceExtensionProperties(candidate, nullptr, &deviceExtensionCount, nullptr);
        std::vector<VkExtensionProperties> deviceExtensions(deviceExtensionCount);
        vkEnumerateDeviceExtensionProperties(candidate, nullptr, &deviceExtensionCount, deviceExtensions.data());
        const bool hasSwapchain = std::any_of(deviceExtensions.begin(),
            deviceExtensions.end(),
            [](const VkExtensionProperties &extension)
            { return std::strcmp(extension.extensionName, VK_KHR_SWAPCHAIN_EXTENSION_NAME) == 0; });
        if (!hasSwapchain)
        {
            continue;
        }

        VkPhysicalDeviceProperties properties;
        vkGetPhysicalDeviceProperties(candidate, &properties);
        if (!preferenceAccepts(preference, properties.deviceType))
        {
            continue;
        }

        for (std::uint32_t family = 0; family < familyCount; ++family)
        {
            if ((families[family].queueFlags & VK_QUEUE_GRAPHICS_BIT) == 0u)
            {
                continue;
            }
            VkBool32 presentSupported = VK_FALSE;
            if (vkGetPhysicalDeviceSurfaceSupportKHR(candidate, family, context->surface_, &presentSupported)
                    != VK_SUCCESS
                || presentSupported != VK_TRUE)
            {
                continue;
            }

            const std::uint32_t score
                = deviceScore(properties.deviceType) * 2u + (families[family].timestampValidBits != 0u ? 1u : 0u);
            if (score > bestScore)
            {
                bestScore                   = score;
                chosenDevice                = candidate;
                chosenFamily                = family;
                chosenQueueSupportsCompute  = (families[family].queueFlags & VK_QUEUE_COMPUTE_BIT) != 0u;
                chosenDeviceSupportsCompute = deviceSupportsCompute;
                chosenTimestampValidBits    = families[family].timestampValidBits;
                chosenMemoryBudgetAvailable
                    = hasPhysicalDeviceProperties2 && hasSwapchain
                   && std::any_of(deviceExtensions.begin(),
                       deviceExtensions.end(),
                       [](const VkExtensionProperties &extension)
                       { return std::strcmp(extension.extensionName, VK_EXT_MEMORY_BUDGET_EXTENSION_NAME) == 0; });
            }
        }
    }
    if (chosenDevice == VK_NULL_HANDLE)
    {
        if (preference == DevicePreference::Discrete)
        {
            std::fprintf(stderr, "render module: no discrete Vulkan device with a graphics-and-present queue\n");
        }
        else if (preference == DevicePreference::Integrated)
        {
            std::fprintf(stderr, "render module: no integrated Vulkan device with a graphics-and-present queue\n");
        }
        else
        {
            std::fprintf(stderr, "render module: no Vulkan device with a graphics-and-present queue\n");
        }
        reportError(error, RenderError::CreationFailed);
        return nullptr;
    }
    context->physicalDevice_ = chosenDevice;
    context->graphicsFamily_ = chosenFamily;

    const float                   queuePriority = 1.0f;
    const VkDeviceQueueCreateInfo queueInfo{
        .sType            = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO,
        .pNext            = nullptr,
        .flags            = 0,
        .queueFamilyIndex = context->graphicsFamily_,
        .queueCount       = 1,
        .pQueuePriorities = &queuePriority,
    };
    VkPhysicalDeviceFeatures supportedDeviceFeatures{};
    vkGetPhysicalDeviceFeatures(context->physicalDevice_, &supportedDeviceFeatures);

    std::vector<const char *> deviceExtensionsVec;
    deviceExtensionsVec.push_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
    if (chosenMemoryBudgetAvailable)
        deviceExtensionsVec.push_back(VK_EXT_MEMORY_BUDGET_EXTENSION_NAME);

    // Logical device: enable anisotropy when supported; the 1.2/1.3 feature
    // chains are queried so the DeviceProfile can report availability, but
    // nothing beyond anisotropy is switched on.
    VkPhysicalDeviceFeatures2        supportedFeatures2{};
    VkPhysicalDeviceVulkan12Features supported12{};
    VkPhysicalDeviceVulkan13Features supported13{};
    supportedFeatures2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
    supportedFeatures2.pNext = &supported12;
    supported12.sType        = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
    supported12.pNext        = &supported13;
    supported13.sType        = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
    supported13.pNext        = nullptr;
    vkGetPhysicalDeviceFeatures2(context->physicalDevice_, &supportedFeatures2);

    VkPhysicalDeviceFeatures2        enabledFeatures2{};
    VkPhysicalDeviceVulkan12Features enabled12{};
    VkPhysicalDeviceVulkan13Features enabled13{};
    enabledFeatures2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
    enabledFeatures2.pNext = &enabled12;
    enabled12.sType        = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
    enabled12.pNext        = &enabled13;
    enabled13.sType        = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
    enabled13.pNext        = nullptr;

    if (supportedFeatures2.features.samplerAnisotropy == VK_TRUE)
        enabledFeatures2.features.samplerAnisotropy = VK_TRUE;

    bool hasConservativeRaster = false;
    {
        std::uint32_t extCount = 0;
        vkEnumerateDeviceExtensionProperties(context->physicalDevice_, nullptr, &extCount, nullptr);
        std::vector<VkExtensionProperties> exts(extCount);
        vkEnumerateDeviceExtensionProperties(context->physicalDevice_, nullptr, &extCount, exts.data());
        hasConservativeRaster = std::any_of(exts.begin(),
            exts.end(),
            [](const VkExtensionProperties &e)
            { return std::strcmp(e.extensionName, "VK_EXT_conservative_rasterization") == 0; });
    }
    if (hasConservativeRaster)
        deviceExtensionsVec.push_back("VK_EXT_conservative_rasterization");

    const VkDeviceCreateInfo deviceInfo{
        .sType                   = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO,
        .pNext                   = &enabledFeatures2,
        .flags                   = 0,
        .queueCreateInfoCount    = 1,
        .pQueueCreateInfos       = &queueInfo,
        .enabledLayerCount       = 0,
        .ppEnabledLayerNames     = nullptr,
        .enabledExtensionCount   = static_cast<std::uint32_t>(deviceExtensionsVec.size()),
        .ppEnabledExtensionNames = deviceExtensionsVec.data(),
        .pEnabledFeatures        = nullptr,
    };
    if (vkCreateDevice(context->physicalDevice_, &deviceInfo, nullptr, &context->device_) != VK_SUCCESS)
    {
        std::fprintf(stderr, "render module: vkCreateDevice failed\n");
        reportError(error, RenderError::CreationFailed);
        return nullptr;
    }
    VmaAllocatorCreateInfo allocatorInfo{};
    allocatorInfo.physicalDevice   = context->physicalDevice_;
    allocatorInfo.device           = context->device_;
    allocatorInfo.instance         = context->instance_;
    allocatorInfo.vulkanApiVersion = VK_API_VERSION_1_0;

    if (chosenMemoryBudgetAvailable)
        allocatorInfo.flags |= VMA_ALLOCATOR_CREATE_EXT_MEMORY_BUDGET_BIT;
    if (vmaCreateAllocator(&allocatorInfo, &context->allocator_) != VK_SUCCESS)
    {
        std::fprintf(stderr, "render module: vmaCreateAllocator failed\n");
        reportError(error, RenderError::CreationFailed);
        return nullptr;
    }
    vkGetDeviceQueue(context->device_, context->graphicsFamily_, 0, &context->queue_);

    VkPhysicalDeviceProperties deviceProperties;
    vkGetPhysicalDeviceProperties(context->physicalDevice_, &deviceProperties);
    {
        const char *vendor = vendorName(deviceProperties.vendorID);
        if (vendor != nullptr)
        {
            context->vendor_ = vendor;
        }
        else
        {
            char buffer[32];
            std::snprintf(buffer, sizeof(buffer), "vendor 0x%04X", deviceProperties.vendorID);
            context->vendor_ = buffer;
        }
        context->renderer_ = deviceProperties.deviceName;
        char versionBuffer[64];
        std::snprintf(versionBuffer,
            sizeof(versionBuffer),
            "Vulkan %u.%u.%u",
            VK_API_VERSION_MAJOR(deviceProperties.apiVersion),
            VK_API_VERSION_MINOR(deviceProperties.apiVersion),
            VK_API_VERSION_PATCH(deviceProperties.apiVersion));
        context->version_     = versionBuffer;
        context->deviceClass_ = deviceClassFromType(deviceProperties.deviceType);

        // DeviceProfile snapshot for capability consumers (limits, features,
        // extension availability, device-local heap total).
        DeviceProfile &profile                 = context->profile_;
        profile.deviceClass                    = context->deviceClass_;
        profile.apiVersion                     = deviceProperties.apiVersion;
        profile.maxImageDimension2D            = deviceProperties.limits.maxImageDimension2D;
        profile.maxStorageBufferRange          = deviceProperties.limits.maxStorageBufferRange;
        profile.maxComputeWorkGroupInvocations = deviceProperties.limits.maxComputeWorkGroupInvocations;
        profile.maxComputeSharedMemoryBytes    = deviceProperties.limits.maxComputeSharedMemorySize;
        profile.maxUniformBufferRange          = deviceProperties.limits.maxUniformBufferRange;
        profile.vendorId                       = deviceProperties.vendorID;
        profile.deviceId                       = deviceProperties.deviceID;
        profile.driverVersion                  = deviceProperties.driverVersion;
        profile.maxPushConstantsSize           = deviceProperties.limits.maxPushConstantsSize;
        profile.maxBoundDescriptorSets         = deviceProperties.limits.maxBoundDescriptorSets;
        profile.maxDrawIndirectCount           = deviceProperties.limits.maxDrawIndirectCount;
        profile.timestampValidBits             = chosenTimestampValidBits;
        profile.timestampPeriodNanoseconds     = deviceProperties.limits.timestampPeriod;
        profile.timestampsSupported = chosenTimestampValidBits != 0u && deviceProperties.limits.timestampPeriod > 0.0f;
        profile.multiDrawIndirectAvailable         = supportedFeatures2.features.multiDrawIndirect == VK_TRUE;
        profile.drawIndirectFirstInstanceAvailable = supportedFeatures2.features.drawIndirectFirstInstance == VK_TRUE;
        profile.samplerAnisotropyAvailable         = supportedFeatures2.features.samplerAnisotropy == VK_TRUE;
        profile.isVulkan                           = true;
        profile.graphicsQueueSupportsCompute       = chosenQueueSupportsCompute;

        profile.computeShader = chosenDeviceSupportsCompute;

        VkPhysicalDeviceMemoryProperties memoryProperties{};
        vkGetPhysicalDeviceMemoryProperties(context->physicalDevice_, &memoryProperties);
        context->memoryProperties_       = memoryProperties;
        context->memoryPropertiesCached_ = true;
        for (std::uint32_t index = 0; index < memoryProperties.memoryHeapCount; ++index)
        {
            const VkMemoryHeap &heap = memoryProperties.memoryHeaps[index];
            if ((heap.flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) != 0u)
                profile.deviceLocalMemoryBytes += heap.size;
        }

        std::uint32_t supportedExtensionCount = 0;
        vkEnumerateDeviceExtensionProperties(context->physicalDevice_, nullptr, &supportedExtensionCount, nullptr);
        std::vector<VkExtensionProperties> supportedExtensions(supportedExtensionCount);
        vkEnumerateDeviceExtensionProperties(
            context->physicalDevice_, nullptr, &supportedExtensionCount, supportedExtensions.data());
        const auto hasExtension = [&supportedExtensions](const char *name)
        {
            return std::any_of(supportedExtensions.begin(),
                supportedExtensions.end(),
                [name](const VkExtensionProperties &extension)
                { return std::strcmp(extension.extensionName, name) == 0; });
        };
        const std::uint32_t apiMajor            = VK_API_VERSION_MAJOR(deviceProperties.apiVersion);
        const std::uint32_t apiMinor            = VK_API_VERSION_MINOR(deviceProperties.apiVersion);
        const bool          core12              = apiMajor > 1u || (apiMajor == 1u && apiMinor >= 2u);
        const bool          core13              = apiMajor > 1u || (apiMajor == 1u && apiMinor >= 3u);
        profile.timelineSemaphoreAvailable      = core12 || hasExtension("VK_KHR_timeline_semaphore");
        profile.descriptorIndexingAvailable     = core12 || hasExtension("VK_EXT_descriptor_indexing");
        profile.dynamicRenderingAvailable       = core13 || hasExtension("VK_KHR_dynamic_rendering");
        profile.memoryBudgetExtensionAvailable  = chosenMemoryBudgetAvailable;
        profile.rayTracingExtensionSetAvailable = hasExtension("VK_KHR_acceleration_structure")
                                               && hasExtension("VK_KHR_ray_tracing_pipeline")
                                               && hasExtension("VK_KHR_deferred_host_operations");

        profile.conservativeRasterizationAvailable = hasExtension("VK_EXT_conservative_rasterization");
        profile.imageCompressionControlAvailable   = hasExtension("VK_EXT_image_compression_control");
        profile.fragmentDensityMapAvailable        = hasExtension("VK_EXT_fragment_density_map");
        profile.fragmentShadingRateAvailable       = core13 || hasExtension("VK_KHR_fragment_shading_rate");
        profile.rasterizationOrderAttachmentAccess = hasExtension("VK_ARM_rasterization_order_attachment_access");
        profile.image2dViewOf3dAvailable           = hasExtension("VK_EXT_image_2d_view_of_3d");
    }

    context->refreshMemoryBudget();

    // The timeline semaphore stays null (see vulkan_device.hpp): frame sync
    // always uses the per-slot fences in VulkanFrameScheduler.
    const VkCommandPoolCreateInfo uploadPoolInfo{
        .sType            = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
        .pNext            = nullptr,
        .flags            = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT,
        .queueFamilyIndex = context->graphicsFamily_,
    };
    if (vkCreateCommandPool(context->device_, &uploadPoolInfo, nullptr, &context->uploadPool_) != VK_SUCCESS)
    {
        std::fprintf(stderr, "render module: could not create the upload command pool\n");
        reportError(error, RenderError::CreationFailed);
        return nullptr;
    }
    try
    {
        context->pendingUploads_.reserve(context->maximumPendingUploads_);
    }
    catch (const std::bad_alloc &)
    {
        std::fprintf(stderr, "render module: could not reserve upload tracking capacity\n");
        reportError(error, RenderError::CreationFailed);
        return nullptr;
    }

    context->timelineSemaphore_ = VK_NULL_HANDLE;
    reportError(error, RenderError::None);
    return context;
}

std::uint64_t VulkanDeviceContext::completedTimelineValue() const
{
    if (device_ == VK_NULL_HANDLE || timelineSemaphore_ == VK_NULL_HANDLE)
        return 0;
    std::uint64_t v = 0;
    vkGetSemaphoreCounterValue(device_, timelineSemaphore_, &v);
    return v;
}

// Blocks until the given timeline value is reached (used by the screenshot
// export path when the device semaphore exists).
bool VulkanDeviceContext::waitTimeline(std::uint64_t value) const
{
    if (device_ == VK_NULL_HANDLE || timelineSemaphore_ == VK_NULL_HANDLE)
        return false;
    const VkSemaphoreWaitInfo wi{
        .sType          = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO,
        .pNext          = nullptr,
        .flags          = 0,
        .semaphoreCount = 1,
        .pSemaphores    = &timelineSemaphore_,
        .pValues        = &value,
    };
    return vkWaitSemaphores(device_, &wi, UINT64_MAX) == VK_SUCCESS;
}

void VulkanDeviceContext::beginFrame(std::uint32_t memoryBudgetUpdateIntervalFrames) noexcept
{
    if (allocator_ == nullptr)
        return;

    vmaSetCurrentFrameIndex(allocator_, ++memoryBudgetFrameIndex_);

    if (memoryBudgetUpdateIntervalFrames == 0u || (memoryBudgetFrameIndex_ % memoryBudgetUpdateIntervalFrames) != 0u)
        return;
    refreshMemoryBudget();
}

void VulkanDeviceContext::refreshMemoryBudget() noexcept
{
    if (allocator_ == nullptr || !profile_.memoryBudgetExtensionAvailable)
        return;

    if (!memoryPropertiesCached_)
    {
        vkGetPhysicalDeviceMemoryProperties(physicalDevice_, &memoryProperties_);
        memoryPropertiesCached_ = true;
    }

    VmaBudget heapBudgets[VK_MAX_MEMORY_HEAPS]{};
    vmaGetHeapBudgets(allocator_, heapBudgets);

    std::uint64_t budget = 0;
    std::uint64_t usage  = 0;
    for (std::uint32_t index = 0; index < memoryProperties_.memoryHeapCount; ++index)
    {
        if ((memoryProperties_.memoryHeaps[index].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) == 0u)
            continue;
        budget += heapBudgets[index].budget;
        usage += heapBudgets[index].usage;
    }
    profile_.deviceLocalMemoryBudgetBytes = budget;
    profile_.deviceLocalMemoryUsageBytes  = usage;
}

// Memory placement per intent (see BufferMemoryIntent). All allocations use
// VMA's auto selection with the intent's required/preferred property flags.
bool VulkanDeviceContext::createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, BufferMemoryIntent memoryIntent,
    VkBuffer *outBuffer, VmaAllocation *outAllocation) const
{
    if (outBuffer == nullptr || outAllocation == nullptr || size == 0 || allocator_ == nullptr)
        return false;
    *outBuffer     = VK_NULL_HANDLE;
    *outAllocation = nullptr;

    const VkBufferCreateInfo bufferInfo{
        .sType                 = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,
        .pNext                 = nullptr,
        .flags                 = 0,
        .size                  = size,
        .usage                 = usage,
        .sharingMode           = VK_SHARING_MODE_EXCLUSIVE,
        .queueFamilyIndexCount = 0,
        .pQueueFamilyIndices   = nullptr,
    };
    VmaAllocationCreateInfo allocationInfo{};
    switch (memoryIntent)
    {
    case BufferMemoryIntent::DeviceLocal:
        allocationInfo.usage         = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
        allocationInfo.requiredFlags = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
        break;
    case BufferMemoryIntent::HostSequentialWrite:
        allocationInfo.usage          = VMA_MEMORY_USAGE_AUTO_PREFER_HOST;
        allocationInfo.flags          = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
        allocationInfo.requiredFlags  = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;
        allocationInfo.preferredFlags = VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
        break;
    case BufferMemoryIntent::HostRandomAccess:
        allocationInfo.usage          = VMA_MEMORY_USAGE_AUTO_PREFER_HOST;
        allocationInfo.flags          = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
        allocationInfo.requiredFlags  = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;
        allocationInfo.preferredFlags = VK_MEMORY_PROPERTY_HOST_CACHED_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
        break;
    default:
        return false;
    }
    return vmaCreateBuffer(allocator_, &bufferInfo, &allocationInfo, outBuffer, outAllocation, nullptr) == VK_SUCCESS;
}

void VulkanDeviceContext::destroyBuffer(VkBuffer buffer, VmaAllocation allocation) const noexcept
{
    if (allocator_ != nullptr && buffer != VK_NULL_HANDLE)
        vmaDestroyBuffer(allocator_, buffer, allocation);
}

bool VulkanDeviceContext::mapAllocation(VmaAllocation allocation, void **outMapped) const
{
    return allocator_ != nullptr && allocation != nullptr && outMapped != nullptr
        && vmaMapMemory(allocator_, allocation, outMapped) == VK_SUCCESS;
}

bool VulkanDeviceContext::flushAllocation(VmaAllocation allocation) const
{
    return flushAllocation(allocation, 0, VK_WHOLE_SIZE);
}

bool VulkanDeviceContext::flushAllocation(VmaAllocation allocation, VkDeviceSize offset, VkDeviceSize size) const
{
    return allocator_ != nullptr && allocation != nullptr
        && vmaFlushAllocation(allocator_, allocation, offset, size) == VK_SUCCESS;
}

bool VulkanDeviceContext::invalidateAllocation(VmaAllocation allocation) const
{
    return allocator_ != nullptr && allocation != nullptr
        && vmaInvalidateAllocation(allocator_, allocation, 0, VK_WHOLE_SIZE) == VK_SUCCESS;
}

void VulkanDeviceContext::unmapAllocation(VmaAllocation allocation) const noexcept
{
    if (allocator_ != nullptr && allocation != nullptr)
        vmaUnmapMemory(allocator_, allocation);
}

bool VulkanDeviceContext::createImage(std::uint32_t width, std::uint32_t height, std::uint32_t mipLevels,
    VkFormat format, VkImageUsageFlags usage, VkImage *outImage, VmaAllocation *outAllocation) const
{
    if (outImage == nullptr || outAllocation == nullptr || width == 0 || height == 0 || mipLevels == 0
        || allocator_ == nullptr)
        return false;
    *outImage      = VK_NULL_HANDLE;
    *outAllocation = nullptr;
    const VkImageCreateInfo imageInfo{
        .sType                 = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
        .pNext                 = nullptr,
        .flags                 = 0,
        .imageType             = VK_IMAGE_TYPE_2D,
        .format                = format,
        .extent                = {width, height, 1},
        .mipLevels             = mipLevels,
        .arrayLayers           = 1,
        .samples               = VK_SAMPLE_COUNT_1_BIT,
        .tiling                = VK_IMAGE_TILING_OPTIMAL,
        .usage                 = usage,
        .sharingMode           = VK_SHARING_MODE_EXCLUSIVE,
        .queueFamilyIndexCount = 0,
        .pQueueFamilyIndices   = nullptr,
        .initialLayout         = VK_IMAGE_LAYOUT_UNDEFINED,
    };
    VmaAllocationCreateInfo allocationInfo{};
    allocationInfo.usage         = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
    allocationInfo.requiredFlags = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
    return vmaCreateImage(allocator_, &imageInfo, &allocationInfo, outImage, outAllocation, nullptr) == VK_SUCCESS;
}

void VulkanDeviceContext::destroyImage(VkImage image, VmaAllocation allocation) const noexcept
{
    if (allocator_ != nullptr && image != VK_NULL_HANDLE)
        vmaDestroyImage(allocator_, image, allocation);
}

// Destroys every finished upload (staging buffer, command buffer, fence);
// with force it also destroys in-flight ones (used before shutdown).
void VulkanDeviceContext::collectCompletedUploadsLocked(bool force) const
{
    for (auto iterator = pendingUploads_.begin(); iterator != pendingUploads_.end();)
    {
        const VkResult status = force ? VK_SUCCESS : vkGetFenceStatus(device_, iterator->fence);
        if (status != VK_SUCCESS)
        {
            ++iterator;
            continue;
        }
        if (iterator->stagingBuffer != VK_NULL_HANDLE)
            destroyBuffer(iterator->stagingBuffer, iterator->stagingAllocation);
        if (iterator->command != VK_NULL_HANDLE && uploadPool_ != VK_NULL_HANDLE)
            vkFreeCommandBuffers(device_, uploadPool_, 1, &iterator->command);
        if (iterator->fence != VK_NULL_HANDLE)
            vkDestroyFence(device_, iterator->fence, nullptr);
        iterator = pendingUploads_.erase(iterator);
    }
    hasPendingUploads_.store(!pendingUploads_.empty(), std::memory_order_release);
}

void VulkanDeviceContext::collectCompletedUploads() const
{
    if (device_ == VK_NULL_HANDLE || !hasPendingUploads_.load(std::memory_order_acquire))
        return;
    std::lock_guard queueLock(queueMutex_);
    collectCompletedUploadsLocked(false);
}

void VulkanDeviceContext::waitForUploads() const
{
    if (device_ == VK_NULL_HANDLE)
        return;
    std::lock_guard queueLock(queueMutex_);
    for (const PendingUpload &upload : pendingUploads_)
        if (upload.fence != VK_NULL_HANDLE)
            vkWaitForFences(device_, 1, &upload.fence, VK_TRUE, UINT64_MAX);
    collectCompletedUploadsLocked(true);
}

// Submits the prepared copy command buffer with a fresh fence and registers
// it. When the queue is at maximumPendingUploads_, waits for the oldest
// upload first; still full means failure (caller cleans up).
bool VulkanDeviceContext::enqueueUploadLocked(
    VkCommandBuffer command, VkBuffer stagingBuffer, VmaAllocation stagingAllocation) const
{
    if (device_ == VK_NULL_HANDLE || command == VK_NULL_HANDLE || stagingBuffer == VK_NULL_HANDLE
        || stagingAllocation == nullptr || uploadPool_ == VK_NULL_HANDLE)
        return false;

    collectCompletedUploadsLocked(false);
    if (pendingUploads_.size() >= maximumPendingUploads_)
    {
        const VkFence oldestFence = pendingUploads_.front().fence;
        if (oldestFence == VK_NULL_HANDLE
            || vkWaitForFences(device_, 1, &oldestFence, VK_TRUE, UINT64_MAX) != VK_SUCCESS)
            return false;
        collectCompletedUploadsLocked(false);
        if (pendingUploads_.size() >= maximumPendingUploads_)
            return false;
    }
    const VkFenceCreateInfo fenceInfo{
        .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO,
        .pNext = nullptr,
        .flags = 0,
    };
    VkFence fence = VK_NULL_HANDLE;
    if (vkCreateFence(device_, &fenceInfo, nullptr, &fence) != VK_SUCCESS)
        return false;

    const VkSubmitInfo submitInfo{
        .sType                = VK_STRUCTURE_TYPE_SUBMIT_INFO,
        .pNext                = nullptr,
        .waitSemaphoreCount   = 0,
        .pWaitSemaphores      = nullptr,
        .pWaitDstStageMask    = nullptr,
        .commandBufferCount   = 1,
        .pCommandBuffers      = &command,
        .signalSemaphoreCount = 0,
        .pSignalSemaphores    = nullptr,
    };
    if (vkQueueSubmit(queue_, 1, &submitInfo, fence) != VK_SUCCESS)
    {
        vkDestroyFence(device_, fence, nullptr);
        return false;
    }

    try
    {
        pendingUploads_.push_back({command, fence, stagingBuffer, stagingAllocation});
        hasPendingUploads_.store(true, std::memory_order_release);
        return true;
    }
    catch (...)
    {
        vkWaitForFences(device_, 1, &fence, VK_TRUE, UINT64_MAX);
        vkDestroyFence(device_, fence, nullptr);
        return false;
    }
}

// Staging-buffer upload path: pack all uploads into one host-visible buffer,
// record a single copy command buffer with a transfer->vertex-input barrier
// per destination, and hand it to the async queue. Sizes must be 4-byte
// multiples (Vulkan buffer copy alignment). Fully serialized on the queue
// mutex; failures destroy the staging copy before returning.
bool VulkanDeviceContext::copyHostToDevice(const BufferUpload *uploads, std::uint32_t uploadCount) const
{
    if (uploads == nullptr || uploadCount == 0 || device_ == VK_NULL_HANDLE)
        return false;
    std::lock_guard queueLock(queueMutex_);
    collectCompletedUploadsLocked(false);

    VkDeviceSize stagingSize = 0;
    for (std::uint32_t index = 0; index < uploadCount; ++index)
    {
        const BufferUpload &upload = uploads[index];
        if (upload.destination == VK_NULL_HANDLE || upload.data == nullptr || upload.size == 0
            || upload.destinationAccess == 0 || upload.size % 4 != 0
            || stagingSize > std::numeric_limits<VkDeviceSize>::max() - upload.size)
            return false;
        stagingSize += upload.size;
    }

    VkBuffer      stagingBuffer     = VK_NULL_HANDLE;
    VmaAllocation stagingAllocation = nullptr;
    if (!createBuffer(stagingSize,
            VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
            BufferMemoryIntent::HostSequentialWrite,
            &stagingBuffer,
            &stagingAllocation))
        return false;

    void *mapped = nullptr;
    if (!mapAllocation(stagingAllocation, &mapped))
    {
        destroyBuffer(stagingBuffer, stagingAllocation);
        return false;
    }
    VkDeviceSize sourceOffset = 0;
    for (std::uint32_t index = 0; index < uploadCount; ++index)
    {
        std::memcpy(static_cast<std::uint8_t *>(mapped) + sourceOffset,
            uploads[index].data,
            static_cast<std::size_t>(uploads[index].size));
        sourceOffset += uploads[index].size;
    }
    const bool flushed = flushAllocation(stagingAllocation);
    unmapAllocation(stagingAllocation);
    if (!flushed)
    {
        destroyBuffer(stagingBuffer, stagingAllocation);
        return false;
    }

    const VkCommandBufferAllocateInfo allocateInfo{
        .sType              = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
        .pNext              = nullptr,
        .commandPool        = uploadPool_,
        .level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
        .commandBufferCount = 1,
    };
    VkCommandBuffer command = VK_NULL_HANDLE;
    if (vkAllocateCommandBuffers(device_, &allocateInfo, &command) != VK_SUCCESS)
    {
        destroyBuffer(stagingBuffer, stagingAllocation);
        return false;
    }
    const VkCommandBufferBeginInfo beginInfo{
        .sType            = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
        .pNext            = nullptr,
        .flags            = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT,
        .pInheritanceInfo = nullptr,
    };
    if (vkBeginCommandBuffer(command, &beginInfo) != VK_SUCCESS)
    {
        vkFreeCommandBuffers(device_, uploadPool_, 1, &command);
        destroyBuffer(stagingBuffer, stagingAllocation);
        return false;
    }

    sourceOffset = 0;
    for (std::uint32_t index = 0; index < uploadCount; ++index)
    {
        const VkBufferCopy copy{sourceOffset, 0, uploads[index].size};
        vkCmdCopyBuffer(command, stagingBuffer, uploads[index].destination, 1, &copy);
        sourceOffset += uploads[index].size;
    }

    std::array<VkBufferMemoryBarrier, 2> smallBarriers{};
    std::vector<VkBufferMemoryBarrier>   largeBarriers;
    VkBufferMemoryBarrier               *barriers = smallBarriers.data();
    if (uploadCount > smallBarriers.size())
    {
        try
        {
            largeBarriers.resize(uploadCount);
        }
        catch (...)
        {
            vkFreeCommandBuffers(device_, uploadPool_, 1, &command);
            destroyBuffer(stagingBuffer, stagingAllocation);
            return false;
        }
        barriers = largeBarriers.data();
    }
    for (std::uint32_t index = 0; index < uploadCount; ++index)
    {
        barriers[index] = VkBufferMemoryBarrier{
            .sType               = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER,
            .pNext               = nullptr,
            .srcAccessMask       = VK_ACCESS_TRANSFER_WRITE_BIT,
            .dstAccessMask       = uploads[index].destinationAccess,
            .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .buffer              = uploads[index].destination,
            .offset              = 0,
            .size                = uploads[index].size,
        };
    }
    vkCmdPipelineBarrier(command,
        VK_PIPELINE_STAGE_TRANSFER_BIT,
        VK_PIPELINE_STAGE_VERTEX_INPUT_BIT,
        0,
        0,
        nullptr,
        uploadCount,
        barriers,
        0,
        nullptr);

    if (vkEndCommandBuffer(command) != VK_SUCCESS || !enqueueUploadLocked(command, stagingBuffer, stagingAllocation))
    {
        vkFreeCommandBuffers(device_, uploadPool_, 1, &command);
        destroyBuffer(stagingBuffer, stagingAllocation);
        return false;
    }
    return true;
}

} // namespace render
} // namespace engine

#endif
