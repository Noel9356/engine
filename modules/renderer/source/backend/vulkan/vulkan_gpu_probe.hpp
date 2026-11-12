// One-shot host GPU probe used by the renderer module's hardware-aware
// configuration (`renderer.backend = "auto"` and `renderer.auto_config`).
//
// The probe answers one question cheaply: does this machine have a real
// graphics accelerator, and what shape is it? It never creates a logical
// device, never touches a surface, and never loads shader machinery - it
// only creates a bare VkInstance, enumerates physical devices and reads
// static properties. The result is computed once and cached for the
// process lifetime (the hardware cannot change under a running engine).
//
// "GPU present" deliberately follows hardware classes: a device reporting
// VK_PHYSICAL_DEVICE_TYPE_CPU (software rasterizers such as lavapipe or
// SwiftShader) is a Vulkan implementation but NOT a GPU, so a machine with
// only such a device reports gpuPresent == false and the auto policy picks
// the headless rasterizer.

#ifndef ENGINE_RENDERER_VULKAN_GPU_PROBE_HPP
#define ENGINE_RENDERER_VULKAN_GPU_PROBE_HPP

#include <engine/renderer/device_profile.hpp>

#include <cstdint>

namespace engine::renderer
{

// Static snapshot of the host's graphics hardware. Fields are 0/false when
// the information does not apply (no Vulkan runtime, no GPU, or a build
// without the Vulkan backend).
struct GpuProbeResult
{
    // A Vulkan loader + implementation could be brought up and at least one
    // physical device was enumerated (includes CPU-type implementations).
    bool vulkanRuntimeAvailable = false;

    // A hardware-class accelerator (discrete, integrated or virtual) with a
    // graphics-capable queue family exists. This is the "GPU present" bit
    // the auto-backend policy keys on.
    bool gpuPresent = false;

    // Class of the best GPU-class device found (highest of
    // DiscreteGpu > IntegratedGpu > VirtualGpu), Unknown when none.
    DeviceClass   bestDeviceClass = DeviceClass::Unknown;

    // Device-local heap bytes of the best device (heuristic sizing input;
    // integrated chips report their carve-out here).
    std::uint64_t deviceLocalMemoryBytes = 0;

    // Feature bits of the best device, relevant to the auto-derived quality
    // knobs (anisotropy for sampler quality, timestamps for GPU profiling).
    bool samplerAnisotropyAvailable = false;
    bool timestampsSupported        = false;
};

// Cached probe; the first call runs the enumeration, every later call
// returns the same snapshot. Thread-safe (function-local static init).
const GpuProbeResult &probeHostGpu();

} // namespace engine::renderer

#endif // ENGINE_RENDERER_VULKAN_GPU_PROBE_HPP
