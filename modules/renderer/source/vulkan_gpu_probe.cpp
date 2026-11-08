// Implementation of the one-shot host GPU probe (see vulkan_gpu_probe.hpp).
//
// Compiled into the renderer module both with and without
// ENGINE_RENDERER_BACKEND_VULKAN: without the Vulkan backend the probe simply
// reports "no Vulkan runtime, no GPU" so the auto policy deterministically
// picks the headless rasterizer.

#include "vulkan_gpu_probe.hpp"

#if defined(ENGINE_RENDERER_BACKEND_VULKAN)

#include <vulkan/vulkan.h>

#include <algorithm>
#include <vector>

namespace engine::renderer
{
namespace
{

std::uint32_t deviceClassScore(DeviceClass deviceClass) noexcept
{
    switch (deviceClass)
    {
    case DeviceClass::DiscreteGpu:
        return 3u;
    case DeviceClass::IntegratedGpu:
        return 2u;
    case DeviceClass::VirtualGpu:
        return 1u;
    case DeviceClass::Cpu:
    case DeviceClass::Unknown:
        break;
    }
    return 0u;
}

DeviceClass deviceClassFromVulkanType(VkPhysicalDeviceType type) noexcept
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
    case VK_PHYSICAL_DEVICE_TYPE_OTHER:
    case VK_PHYSICAL_DEVICE_TYPE_MAX_ENUM:
        break;
    }
    return DeviceClass::Unknown;
}

GpuProbeResult probeOnce()
{
    GpuProbeResult result;

    // Bare instance: no extensions, no layers, API 1.0. Every implementation
    // supports this shape; the probe must never be the reason a machine
    // cannot start.
    const VkApplicationInfo applicationInfo{
        .sType              = VK_STRUCTURE_TYPE_APPLICATION_INFO,
        .pNext              = nullptr,
        .pApplicationName   = "engine",
        .applicationVersion = VK_MAKE_API_VERSION(0, 0, 1, 0),
        .pEngineName        = "engine",
        .engineVersion      = VK_MAKE_API_VERSION(0, 0, 1, 0),
        .apiVersion         = VK_API_VERSION_1_0,
    };
    const VkInstanceCreateInfo instanceInfo{
        .sType                   = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
        .pNext                   = nullptr,
        .flags                   = 0,
        .pApplicationInfo        = &applicationInfo,
        .enabledLayerCount       = 0,
        .ppEnabledLayerNames     = nullptr,
        .enabledExtensionCount   = 0,
        .ppEnabledExtensionNames = nullptr,
    };

    VkInstance instance = VK_NULL_HANDLE;
    if (vkCreateInstance(&instanceInfo, nullptr, &instance) != VK_SUCCESS || instance == VK_NULL_HANDLE)
    {
        return result; // no loader / no ICD: runtime unavailable, no GPU
    }

    std::uint32_t physicalDeviceCount = 0;
    vkEnumeratePhysicalDevices(instance, &physicalDeviceCount, nullptr);
    if (physicalDeviceCount == 0)
    {
        vkDestroyInstance(instance, nullptr);
        return result; // loader present but no implementation installed
    }
    std::vector<VkPhysicalDevice> physicalDevices(physicalDeviceCount);
    vkEnumeratePhysicalDevices(instance, &physicalDeviceCount, physicalDevices.data());
    if (physicalDevices.empty())
    {
        vkDestroyInstance(instance, nullptr);
        return result;
    }
    result.vulkanRuntimeAvailable = true;

    std::uint32_t bestScore = 0;
    for (const VkPhysicalDevice candidate : physicalDevices)
    {
        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(candidate, &properties);
        const DeviceClass deviceClass = deviceClassFromVulkanType(properties.deviceType);
        const std::uint32_t score = deviceClassScore(deviceClass);
        if (score == 0u)
        {
            continue; // CPU / other-class implementations never count as a GPU
        }

        // A GPU without a graphics queue cannot render; keep looking.
        std::uint32_t familyCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(candidate, &familyCount, nullptr);
        std::vector<VkQueueFamilyProperties> families(familyCount);
        vkGetPhysicalDeviceQueueFamilyProperties(candidate, &familyCount, families.data());
        const bool hasGraphicsQueue = std::any_of(families.begin(),
            families.end(),
            [](const VkQueueFamilyProperties &family)
            { return (family.queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0u; });
        if (!hasGraphicsQueue)
        {
            continue;
        }

        if (score <= bestScore)
        {
            continue;
        }

        VkPhysicalDeviceMemoryProperties memoryProperties{};
        vkGetPhysicalDeviceMemoryProperties(candidate, &memoryProperties);
        std::uint64_t deviceLocalBytes = 0;
        for (std::uint32_t heap = 0; heap < memoryProperties.memoryHeapCount; ++heap)
        {
            if ((memoryProperties.memoryHeaps[heap].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) != 0u)
            {
                deviceLocalBytes += memoryProperties.memoryHeaps[heap].size;
            }
        }

        VkPhysicalDeviceFeatures features{};
        vkGetPhysicalDeviceFeatures(candidate, &features);
        const bool timestamps = std::any_of(families.begin(),
            families.end(),
            [](const VkQueueFamilyProperties &family) { return family.timestampValidBits != 0u; });

        bestScore                     = score;
        result.gpuPresent             = true;
        result.bestDeviceClass        = deviceClass;
        result.deviceLocalMemoryBytes = deviceLocalBytes;
        result.samplerAnisotropyAvailable = features.samplerAnisotropy == VK_TRUE;
        result.timestampsSupported        = timestamps;
    }

    vkDestroyInstance(instance, nullptr);
    return result;
}

} // namespace

const GpuProbeResult &probeHostGpu()
{
    static const GpuProbeResult cached = probeOnce();
    return cached;
}

} // namespace engine::renderer

#else // !ENGINE_RENDERER_BACKEND_VULKAN

namespace engine::renderer
{

// Build without the Vulkan backend: the host cannot run a Vulkan renderer,
// so the probe reports no runtime and no GPU without touching anything.
const GpuProbeResult &probeHostGpu()
{
    static const GpuProbeResult cached{};
    return cached;
}

} // namespace engine::renderer

#endif
