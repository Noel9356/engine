// Shared GPU device capability profile (engine::render).
//
// Populated by the renderer after device creation; limits and feature
// flags let modules pick quality tiers without touching the graphics API.

#ifndef ENGINE_SHARED_RENDER_DEVICE_PROFILE_HPP
#define ENGINE_SHARED_RENDER_DEVICE_PROFILE_HPP

#include <cstdint>

namespace engine::render
{

enum class DeviceClass : std::uint32_t
{
    Unknown = 0,
    DiscreteGpu,
    IntegratedGpu,
    VirtualGpu,
    Cpu
};

struct DeviceProfile
{
    DeviceClass   deviceClass                        = DeviceClass::Unknown;
    std::uint32_t apiVersion                         = 0;
    std::uint64_t deviceLocalMemoryBytes             = 0;
    std::uint64_t maxStorageBufferRange              = 0;
    std::uint32_t maxImageDimension2D                = 0;
    std::uint32_t maxComputeWorkGroupInvocations     = 0;
    std::uint32_t maxComputeSharedMemoryBytes        = 0;
    std::uint32_t maxUniformBufferRange              = 0;
    std::uint32_t vendorId                           = 0;
    std::uint32_t deviceId                           = 0;
    std::uint32_t driverVersion                      = 0;
    std::uint32_t maxPushConstantsSize               = 0;
    std::uint32_t maxBoundDescriptorSets             = 0;
    std::uint32_t maxDrawIndirectCount               = 0;
    std::uint32_t timestampValidBits                 = 0;
    float         timestampPeriodNanoseconds         = 0.0f;
    bool          multiDrawIndirectAvailable         = false;
    bool          drawIndirectFirstInstanceAvailable = false;
    bool          samplerAnisotropyAvailable         = false;
    bool          memoryBudgetExtensionAvailable     = false;
    bool          timestampsSupported                = false;
    std::uint64_t deviceLocalMemoryBudgetBytes       = 0;
    std::uint64_t deviceLocalMemoryUsageBytes        = 0;
    bool          isVulkan                           = false;
    bool          computeShader                      = false;
    bool          graphicsQueueSupportsCompute       = false;
    bool          timelineSemaphoreAvailable         = false;
    bool          descriptorIndexingAvailable        = false;
    bool          dynamicRenderingAvailable          = false;

    bool rayTracingExtensionSetAvailable = false;
    bool rayTracingPipelineEnabled       = false;

    bool conservativeRasterizationAvailable = false;
    bool imageCompressionControlAvailable   = false;
    bool fragmentDensityMapAvailable        = false;
    bool fragmentShadingRateAvailable       = false;
    bool rasterizationOrderAttachmentAccess = false;
    bool image2dViewOf3dAvailable           = false;
};

enum class QualityPreset : std::uint32_t
{
    Auto = 0,
    Low,
    Medium,
    High,
    Ultra
};

// Memory-based heuristics: CPU/virtual GPUs get Low; iGPU and dGPU tiers scale with
// device-local memory.
inline QualityPreset recommendedQualityPreset(const DeviceProfile &device) noexcept
{
    if (device.deviceClass == DeviceClass::Cpu || device.deviceClass == DeviceClass::VirtualGpu)
        return QualityPreset::Low;
    const std::uint64_t memory = device.deviceLocalMemoryBudgetBytes != 0 ? device.deviceLocalMemoryBudgetBytes
                                                                          : device.deviceLocalMemoryBytes;
    if (device.deviceClass == DeviceClass::IntegratedGpu)
    {
        if (device.deviceLocalMemoryBudgetBytes == 0)
            return QualityPreset::Medium;
        return memory >= 6ull * 1024ull * 1024ull * 1024ull ? QualityPreset::High
             : memory >= 3ull * 1024ull * 1024ull * 1024ull ? QualityPreset::Medium
                                                            : QualityPreset::Low;
    }
    if (device.deviceClass == DeviceClass::DiscreteGpu)
        return memory >= 10ull * 1024ull * 1024ull * 1024ull ? QualityPreset::Ultra
             : memory >= 5ull * 1024ull * 1024ull * 1024ull  ? QualityPreset::High
             : memory >= 2ull * 1024ull * 1024ull * 1024ull  ? QualityPreset::Medium
                                                             : QualityPreset::Low;
    return QualityPreset::Medium;
}

inline QualityPreset resolveQualityPreset(QualityPreset requested, const DeviceProfile &device) noexcept
{
    return requested == QualityPreset::Auto ? recommendedQualityPreset(device) : requested;
}

} // namespace engine::render

#endif
