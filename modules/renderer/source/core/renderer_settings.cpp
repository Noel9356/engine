// Implementation of the [renderer] settings model (renderer_settings.hpp):
// the built-in defaults, the layered-configuration overlay with validated
// ranges, and the hardware-derived auto_config path.

#include "renderer_settings.hpp"

#include "render_errors.hpp"

#include <engine/renderer/device_profile.hpp>
#include <engine/util/hardware_info.hpp>
#include "vulkan_gpu_probe.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

namespace engine
{
namespace renderer
{
namespace
{

// Reads [renderer] clear_color into the settings' clearColor. Requires at
// least four finite components; non-finite arrays are ignored with a
// warning, and each component is clamped to 0..1. Shared between the manual
// configuration path and the hardware-derived path because clear_color is
// content-owned (auto_config never overrides it).
void readClearColor(ParticipantContext &context, std::array<float, 4> &clearColor)
{
    const std::vector<double> components = context.configurationFloatingArrayInSection("renderer", "clear_color");
    if (components.size() < 4)
    {
        return;
    }
    bool valid = true;
    for (std::size_t index = 0; index < clearColor.size(); ++index)
    {
        valid = valid && std::isfinite(components[index]);
    }
    if (valid)
    {
        for (std::size_t index = 0; index < clearColor.size(); ++index)
        {
            clearColor[index] = static_cast<float>(std::clamp(components[index], 0.0, 1.0));
        }
    }
    else
    {
        std::fprintf(stderr, "renderer module: ignoring [renderer] clear_color; components must be finite\n");
    }
}

// Reads an integer key and clamps it into its validated range, warning
// when the value needed adjusting; an absent key keeps the fallback (the
// caller passes the current default).
std::int64_t readClampedInteger(
    ParticipantContext &context, const char *key, std::int64_t fallback, std::int64_t lowest, std::int64_t highest)
{
    const std::optional<std::int64_t> value = context.configurationIntegerInSection("renderer", key);
    if (!value.has_value())
    {
        return fallback;
    }
    if (*value < lowest || *value > highest)
    {
        std::fprintf(stderr,
            "renderer module: clamping [renderer] %s %lld to range %lld..%lld\n",
            key,
            static_cast<long long>(*value),
            static_cast<long long>(lowest),
            static_cast<long long>(highest));
    }
    return std::clamp<std::int64_t>(*value, lowest, highest);
}

// The nearest power of two inside 16..256 (the tile grid must stay aligned
// with the framebuffer); exact powers of two and range ends stay as they
// are, everything else rounds to the closer neighbour (ties round up).
std::uint32_t nearestTileEdge(std::int64_t requested)
{
    const std::int64_t bounded = std::clamp<std::int64_t>(requested, 16, 256);
    std::int64_t       edge    = 16;
    while (edge * 2 <= bounded)
    {
        edge *= 2;
    }
    if (edge != bounded && bounded - edge >= edge * 2 - bounded)
    {
        edge *= 2;
    }
    return static_cast<std::uint32_t>(edge);
}

// Overlays the [renderer] keys present in the configuration onto the
// built-in defaults. Out-of-range numeric values are clamped to their
// validated range with a stderr warning; unknown enum strings warn and fall
// back (backend/device to "auto", draw_sort_mode to disabled).
void overlayConfiguredSettings(ParticipantContext &context, RendererSettings &settings)
{
    if (const std::optional<std::string> backend = context.configurationStringInSection("renderer", "backend");
        backend.has_value())
    {
        if (*backend == "vulkan")
        {
            settings.backend = ConfiguredBackend::Vulkan;
        }
        else if (*backend == "headless")
        {
            settings.backend = ConfiguredBackend::Headless;
        }
        else if (*backend != "auto")
        {
            std::fprintf(
                stderr, "renderer module: unknown [renderer] backend '%s'; using \"auto\"\n", backend->c_str());
        }
    }

    if (const std::optional<std::string> device = context.configurationStringInSection("renderer", "device");
        device.has_value())
    {
        if (*device == "discrete")
        {
            settings.devicePreference = DevicePreference::Discrete;
        }
        else if (*device == "integrated")
        {
            settings.devicePreference = DevicePreference::Integrated;
        }
        else if (*device != "auto")
        {
            std::fprintf(stderr, "renderer module: unknown [renderer] device '%s'; using \"auto\"\n", device->c_str());
        }
    }

    settings.vsync      = context.configurationBooleanInSection("renderer", "vsync").value_or(false);
    settings.validation = context.configurationBooleanInSection("renderer", "validation").value_or(false);

    // present_mode pins the presentation-queue policy. Frame latency on
    // this backend is dominated by vkQueuePresentKHR blocking on the
    // compositor, so making the mode explicit is the difference between
    // a ~2 ms and a ~7 ms tail; "auto" keeps the old vsync-derived choice.
    if (const std::optional<std::string> presentModeText
        = context.configurationStringInSection("renderer", "present_mode");
        presentModeText.has_value())
    {
        if (*presentModeText == "auto")
            settings.presentMode = SwapchainPresentMode::Auto;
        else if (*presentModeText == "fifo")
            settings.presentMode = SwapchainPresentMode::Fifo;
        else if (*presentModeText == "mailbox")
            settings.presentMode = SwapchainPresentMode::Mailbox;
        else if (*presentModeText == "immediate")
            settings.presentMode = SwapchainPresentMode::Immediate;
        else
            std::fprintf(stderr,
                "renderer module: unknown [renderer] present_mode '%s'; using \"auto\"\n",
                presentModeText->c_str());
    }

    settings.maxFramesInFlight = static_cast<std::uint32_t>(
        readClampedInteger(context, "max_frames_in_flight", settings.maxFramesInFlight, 1, 8));
    settings.maxPendingUploads = static_cast<std::uint32_t>(
        readClampedInteger(context, "max_pending_uploads", settings.maxPendingUploads, 1, 4096));

    const auto readResourceLimit = [&context, &settings](const char *key, std::uint32_t &destination)
    {
        destination = static_cast<std::uint32_t>(
            readClampedInteger(context, key, destination, 1, 65536));
    };
    readResourceLimit("max_textures_per_renderer", settings.maxTexturesPerRenderer);
    readResourceLimit("max_material_sets_per_renderer", settings.maxMaterialSetsPerRenderer);

    settings.maxPipelineCacheBytes = static_cast<std::size_t>(
        readClampedInteger(context,
            "pipeline_cache_mb",
            static_cast<std::int64_t>(settings.maxPipelineCacheBytes / (1024u * 1024u)),
            0,
            512))
        * 1024u * 1024u;

    settings.memoryBudgetUpdateIntervalFrames = static_cast<std::uint32_t>(readClampedInteger(
        context, "memory_budget_update_interval_frames", settings.memoryBudgetUpdateIntervalFrames, 0, 3600));

    readClearColor(context, settings.clearColor);

    settings.headlessTransformCacheBudgetBytes
        = static_cast<std::size_t>(readClampedInteger(context,
              "headless_transform_cache_mb",
              static_cast<std::int64_t>(settings.headlessTransformCacheBudgetBytes / (1024u * 1024u)),
              0,
              1024))
        * 1024u * 1024u;
    settings.frustumCulling  = context.configurationBooleanInSection("renderer", "frustum_culling").value_or(true);
    settings.backFaceCulling = context.configurationBooleanInSection("renderer", "back_face_culling").value_or(true);
    settings.lodEnabled      = context.configurationBooleanInSection("renderer", "lod_enabled").value_or(true);
    settings.giIndirectLighting
        = context.configurationBooleanInSection("renderer", "gi_indirect_lighting").value_or(true);
    settings.reflectionIndirectSpecular
        = context.configurationBooleanInSection("renderer", "reflection_indirect_specular").value_or(true);
    settings.furEnabled = context.configurationBooleanInSection("renderer", "fur_enabled").value_or(true);
    settings.furMaximumLayers
        = static_cast<std::uint32_t>(readClampedInteger(context, "fur_max_layers", settings.furMaximumLayers, 0, 64));
    settings.gpuTimestampsEnabled
        = context.configurationBooleanInSection("renderer", "gpu_timestamps_enabled").value_or(false);
    settings.instanceBatchingEnabled
        = context.configurationBooleanInSection("renderer", "instance_batching").value_or(true);
    settings.textureMipmapsEnabled
        = context.configurationBooleanInSection("renderer", "texture_mipmaps").value_or(true);
    settings.instanceBatchMinimumCount = static_cast<std::uint32_t>(
        readClampedInteger(context, "instance_batch_min_count", settings.instanceBatchMinimumCount, 2, 1024));

    // Headless rasterizer tuning. Read here (not per renderer creation) so
    // live reloads and auto_config treat it like every other knob. The
    // per-draw tile path default adapts: the serial binning phase loses to
    // the band/triangle-range split on two-core hosts (measured ~+55%
    // frame time) but wins once per-pixel lock contention dominates on
    // four cores and up. auto_config applies the same rule explicitly.
    settings.headlessTileParallel = context.configurationBooleanInSection("renderer", "headless_tile_parallel")
                                        .value_or(util::logicalProcessorCount() >= 4u);
    settings.headlessBatchedTileRasterization
        = context.configurationBooleanInSection("renderer", "headless_batched_tile_rasterization").value_or(true);
    // MSAA-style interior shading (see RendererSettings::headlessMsaaInteriorShading):
    // a quality-preserving frame-cost cut for the supersampled target - the
    // per-sample coverage/depth work stays, the per-sample shading collapses
    // to one evaluation per fully-covered output pixel.
    settings.headlessMsaaInteriorShading
        = context.configurationBooleanInSection("renderer", "headless_msaa_interior_shading").value_or(true);
    settings.headlessParallelTriangleThreshold = static_cast<std::uint32_t>(readClampedInteger(
        context, "headless_parallel_triangle_threshold", settings.headlessParallelTriangleThreshold, 1, 1048576));
    settings.headlessPixelsPerLockGroup        = static_cast<std::size_t>(readClampedInteger(
        context, "headless_pixels_per_lock_group", settings.headlessPixelsPerLockGroup, 1, 4096));
    settings.lodMinTriangles
        = static_cast<std::size_t>(readClampedInteger(context, "lod_min_triangles", settings.lodMinTriangles, 0, 1048576));
    settings.headlessBandSplitTriangleThreshold = static_cast<std::uint32_t>(readClampedInteger(
        context, "headless_band_split_triangles", settings.headlessBandSplitTriangleThreshold, 0, 1048576));
    // Vertex-stage parallelism: meshes at or above this many vertices
    // transform across workers when a scheduler is available.
    settings.headlessParallelTransformVertexThreshold = static_cast<std::uint32_t>(readClampedInteger(
        context, "headless_parallel_transform_vertices", settings.headlessParallelTransformVertexThreshold, 64, 1048576));

    // Screen tile edge in pixels for the tile-parallel paths; powers of
    // two keep the tile grid aligned with the framebuffer, so the value
    // rounds to the nearest power of two in 16..256.
    if (const std::optional<std::int64_t> tileEdge
        = context.configurationIntegerInSection("renderer", "headless_tile_pixels");
        tileEdge.has_value())
    {
        const bool isPowerOfTwo = (*tileEdge & (*tileEdge - 1)) == 0;
        if (!isPowerOfTwo || *tileEdge < 16 || *tileEdge > 256)
        {
            std::fprintf(stderr,
                "renderer module: rounding [renderer] headless_tile_pixels %lld to the nearest power of two in "
                "16..256\n",
                static_cast<long long>(*tileEdge));
        }
        settings.headlessTilePixels = nearestTileEdge(*tileEdge);
    }

    // Record cap of the batched tile path (one record per rasterisable
    // fan triangle): memory-tight targets can lower it, and oversized
    // batches simply flush in more chunks with identical output.
    settings.headlessMaxBatchTileRecords = static_cast<std::size_t>(readClampedInteger(
        context, "headless_batch_tile_records", settings.headlessMaxBatchTileRecords, 1024, 4194304));

    // Angular-size LOD cutoffs (boundsRadius / distance): four strictly
    // ascending values in (0, 1]; coarser chains are picked as the
    // apparent size drops below each cutoff.
    if (const std::vector<double> thresholds
        = context.configurationFloatingArrayInSection("renderer", "lod_thresholds");
        !thresholds.empty())
    {
        bool usable = thresholds.size() == 4;
        for (double cutoff : thresholds)
        {
            if (!std::isfinite(cutoff) || cutoff <= 0.0 || cutoff > 1.0)
            {
                usable = false;
            }
        }
        for (std::size_t index = 1; index < thresholds.size() && usable; ++index)
        {
            if (!(thresholds[index - 1] < thresholds[index]))
            {
                usable = false;
            }
        }
        if (!usable)
        {
            std::fprintf(stderr,
                "renderer module: ignoring [renderer] lod_thresholds (need four finite strictly ascending "
                "values in (0, 1]); keeping the current cutoffs\n");
        }
        else
        {
            for (std::size_t index = 0; index < 4; ++index)
            {
                settings.lodThresholds[index] = static_cast<float>(thresholds[index]);
            }
        }
    }

    // Triangle-stride LOD decimation: three strictly ascending strides in
    // 2..1024; level 0 is always the full index list.
    if (const std::vector<double> strides
        = context.configurationFloatingArrayInSection("renderer", "lod_strides");
        !strides.empty())
    {
        bool usable = strides.size() == 3;
        for (double stride : strides)
        {
            if (!std::isfinite(stride) || stride < 2.0 || stride > 1024.0)
            {
                usable = false;
            }
        }
        for (std::size_t index = 1; index < strides.size() && usable; ++index)
        {
            if (!(strides[index - 1] < strides[index]))
            {
                usable = false;
            }
        }
        if (!usable)
        {
            std::fprintf(stderr,
                "renderer module: ignoring [renderer] lod_strides (need three strictly ascending values in "
                "2..1024); keeping the current strides\n");
        }
        else
        {
            for (std::size_t index = 0; index < 3; ++index)
            {
                settings.lodStrides[index] = static_cast<std::uint32_t>(strides[index]);
            }
        }
    }

    // sRGB output target: the Vulkan surface pick and the shader's
    // SRGB_OUTPUT specialization follow it; the headless target always
    // writes UNORM with its fixed encode.
    settings.srgbOutput = context.configurationBooleanInSection("renderer", "srgb_output").value_or(settings.srgbOutput);

    if (const auto fraction
        = context.configurationFloatingInSection("renderer", "headless_band_split_min_screen_fraction");
        fraction.has_value())
    {
        if (!std::isfinite(*fraction) || *fraction < 0.0 || *fraction > 1.0)
        {
            std::fprintf(stderr,
                "renderer module: clamping [renderer] headless_band_split_min_screen_fraction %g to range 0..1\n",
                *fraction);
        }
        settings.headlessBandSplitMinimumScreenFraction
            = static_cast<float>(std::clamp(*fraction, 0.0, 1.0));
    }

    settings.drawSortMinimumCommands = static_cast<std::uint32_t>(
        readClampedInteger(context, "draw_sort_min_commands", settings.drawSortMinimumCommands, 2, 65536));

    settings.instanceStreamMegabytes = static_cast<std::uint32_t>(
        readClampedInteger(context, "instance_stream_mb", settings.instanceStreamMegabytes, 1, 64));

    // MSAA sample count: > 1 selects the capability-checked
    // dynamic-rendering resolve path on Vulkan (falling back to 1 with a
    // logged note when the device, or the picked formats, do not support
    // it) and the matching integer supersampling grid on the headless
    // rasterizer. Anti-aliasing ships enabled by default.
    if (const std::optional<std::int64_t> msaa = context.configurationIntegerInSection("renderer", "msaa_samples");
        msaa.has_value())
    {
        settings.msaaSamples
            = static_cast<std::uint32_t>(readClampedInteger(context, "msaa_samples", settings.msaaSamples, 1, 8));
    }

    // Anisotropic filtering level for texture samplers; the backend clamps
    // this against limits.maxSamplerAnisotropy before creating a sampler.
    if (const auto anisotropy = context.configurationFloatingInSection("renderer", "anisotropy_level");
        anisotropy.has_value())
    {
        if (*anisotropy < 1.0 || *anisotropy > 16.0)
        {
            std::fprintf(stderr,
                "renderer module: clamping [renderer] anisotropy_level %g to range 1..16\n",
                *anisotropy);
        }
        settings.anisotropyLevel = static_cast<float>(std::clamp(*anisotropy, 1.0, 16.0));
    }
    if (const auto sortMode = context.configurationStringInSection("renderer", "draw_sort_mode"))
    {
        if (*sortMode == "mesh_material")
        {
            settings.drawSortMode    = RenderDrawSortMode::MeshThenMaterial;
            settings.drawSortEnabled = true;
        }
        else if (*sortMode == "material_mesh")
        {
            settings.drawSortMode    = RenderDrawSortMode::MaterialThenMesh;
            settings.drawSortEnabled = true;
        }
        else if (*sortMode == "off")
        {
            settings.drawSortEnabled = false;
        }
        else
        {
            std::fprintf(stderr,
                "renderer module: unknown [renderer] draw_sort_mode '%s'; sorting disabled\n",
                sortMode->c_str());
            settings.drawSortEnabled = false;
        }
    }
}

// auto_config = true: derive every performance, quality and memory knob
// from the host hardware and skip the [renderer] overlay entirely (the
// tuning keys have no effect until auto_config goes back to false).
// clear_color is content, not tuning, and is still read.
//
// Hardware inputs: the cached GPU probe (class, device-local memory,
// feature bits), the logical processor count, and installed RAM. Every
// derived value stays inside the validated ranges the manual config path
// clamps to, so a hardware-derived renderer is indistinguishable from a
// hand-tuned one apart from the stderr notice.
void deriveHardwareSettings(ParticipantContext &context, RendererSettings &settings)
{
    const GpuProbeResult &gpu       = probeHostGpu();
    const std::uint32_t  cores      = util::logicalProcessorCount();
    const std::uint64_t  ram        = util::physicalMemoryBytes();
    const bool           ramKnown   = ram != 0u;
    const bool           lowRam     = !ramKnown || ram < 4ull * 1024ull * 1024ull * 1024ull;
    const bool           bigRam     = ramKnown && ram >= 8ull * 1024ull * 1024ull * 1024ull;
    const bool           discrete   = gpu.bestDeviceClass == DeviceClass::DiscreteGpu;
    const bool           bigVram    = discrete && gpu.deviceLocalMemoryBytes >= 6ull * 1024ull * 1024ull * 1024ull;

    // Backend: Vulkan when a real GPU is present (the same policy the
    // backend = "auto" default follows), headless CPU rasterizer when it
    // is not. Never a software Vulkan implementation: those report a CPU
    // device class and the probe refuses to count them as a GPU.
    settings.backend = gpu.gpuPresent ? ConfiguredBackend::Vulkan : ConfiguredBackend::Headless;

    // Device pick follows the detected class; Auto already prefers
    // discrete, so only the unambiguous single-class cases pin it.
    settings.devicePreference = DevicePreference::Auto;
    if (gpu.gpuPresent)
    {
        if (discrete)
        {
            settings.devicePreference = DevicePreference::Discrete;
        }
        else if (gpu.bestDeviceClass == DeviceClass::IntegratedGpu)
        {
            settings.devicePreference = DevicePreference::Integrated;
        }
    }

    settings.vsync                                   = false;
    settings.validation                              = false;
    settings.presentMode                             = SwapchainPresentMode::Auto;
    settings.maxFramesInFlight                       = 3;
    settings.maxPendingUploads                       = lowRam ? 16u : 64u;
    settings.maxTexturesPerRenderer                  = bigVram ? 512u : 256u;
    settings.maxMaterialSetsPerRenderer              = bigVram ? 512u : 256u;
    settings.maxPipelineCacheBytes
        = discrete ? 128u * 1024u * 1024u : (gpu.gpuPresent ? 64u * 1024u * 1024u : 32u * 1024u * 1024u);
    settings.memoryBudgetUpdateIntervalFrames        = 120;
    settings.headlessTransformCacheBudgetBytes
        = bigRam ? 256u * 1024u * 1024u : (lowRam ? 64u * 1024u * 1024u : 128u * 1024u * 1024u);
    settings.frustumCulling                          = true;
    settings.backFaceCulling                         = true;
    settings.lodEnabled                              = true;
    settings.gpuTimestampsEnabled                    = gpu.gpuPresent && gpu.timestampsSupported;
    settings.drawSortMode                            = RenderDrawSortMode::MeshThenMaterial;
    settings.drawSortEnabled                         = true;
    settings.drawSortMinimumCommands                 = 2;
    settings.instanceBatchingEnabled                 = true;
    settings.instanceBatchMinimumCount               = 4;
    settings.instanceStreamMegabytes                 = bigVram ? 8u : 4u;
    settings.textureMipmapsEnabled                   = true;

    // Quality knobs scale with the accelerator: MSAA everywhere (the
    // 4-sample tier where the memory bandwidth is there, 2 otherwise),
    // anisotropy capped by the device limit at sampler creation.
    settings.msaaSamples     = bigVram ? 4u : 2u;
    settings.anisotropyLevel = gpu.samplerAnisotropyAvailable ? (discrete ? 8.0f : 4.0f) : 1.0f;

    // Headless rasterizer threading scales with the CPU shape: the
    // per-draw tile path turns on from 4 logical processors (its serial
    // binning loses on two cores), the batched tile path from 2 (one
    // binning pass per frame amortizes the fork/join), and the parallel
    // triangle thresholds drop once there are enough workers to win.
    settings.headlessTileParallel                = cores >= 4u;
    settings.headlessBatchedTileRasterization    = cores >= 2u;
    settings.headlessParallelTriangleThreshold   = cores >= 8u ? 2048u : 4096u;
    settings.headlessBandSplitTriangleThreshold  = settings.headlessParallelTriangleThreshold;
    settings.headlessBandSplitMinimumScreenFraction = 0.125f;
    settings.headlessPixelsPerLockGroup          = 16;
    settings.lodMinTriangles                     = 4096;
    settings.headlessTilePixels                  = 64u;
    settings.headlessMaxBatchTileRecords         = 512u * 1024u;

    readClearColor(context, settings.clearColor);

    const char *gpuLabel = "none";
    switch (gpu.bestDeviceClass)
    {
    case DeviceClass::DiscreteGpu:
        gpuLabel = "discrete";
        break;
    case DeviceClass::IntegratedGpu:
        gpuLabel = "integrated";
        break;
    case DeviceClass::VirtualGpu:
        gpuLabel = "virtual";
        break;
    case DeviceClass::Cpu:
        gpuLabel = "software-only";
        break;
    case DeviceClass::Unknown:
        break;
    }
    char ramLabel[32];
    if (ramKnown)
    {
        std::snprintf(ramLabel, sizeof(ramLabel), "%.1f GiB", static_cast<double>(ram) / (1024.0 * 1024.0 * 1024.0));
    }
    else
    {
        std::snprintf(ramLabel, sizeof(ramLabel), "unknown");
    }
    std::fprintf(stderr,
        "renderer module: auto_config is on; [renderer] tuning derived from hardware "
        "(gpu=%s, %u logical processor(s), ram=%s); other tuning keys in [renderer] are ignored\n",
        gpuLabel,
        cores,
        ramLabel);
}

} // namespace

RendererSettings readRendererSettings(ParticipantContext &context)
{
    RendererSettings settings;
    settings.autoConfig = context.configurationBooleanInSection("renderer", "auto_config").value_or(false);
    if (settings.autoConfig)
    {
        deriveHardwareSettings(context, settings);
        return settings;
    }
    overlayConfiguredSettings(context, settings);
    return settings;
}

} // namespace renderer
} // namespace engine
