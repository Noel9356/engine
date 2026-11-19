// renderer module entry point: the ModuleInterface implementation that binds
// the "renderer" capability family to a backend instance.
//
// Responsibilities: hold the current [renderer] settings (read and re-read
// by renderer_settings.cpp), acquire the window/pbr/direct_lighting/
// lumen_gi/lumen_reflection capabilities, create renderers by resolving the
// configured backend (Vulkan or headless), and route capability calls into
// the per-renderer frame input feeds (frame_inputs.cpp), the draw-list
// scheduler (draw_scheduler.cpp) and the backend. The module entry table
// for the kernel's loader sits at the bottom.

#include <cstdint>
#include <engine/direct_lighting/capabilities.hpp>
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/lumen_gi/capabilities.hpp>
#include <engine/lumen_reflection/capabilities.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/window/capabilities.hpp>

#include <engine/kernel/event_bus.hpp>
#include <engine/kernel/profiling.hpp>
#include <engine/renderer/device_profile.hpp>
#include <engine/renderer/draw_sort.hpp>
#include <engine/renderer/material.hpp>
#include <engine/renderer/mesh.hpp>

#include "draw_scheduler.hpp"
#include "frame_inputs.hpp"
#include "headless_backend.hpp"
#include "render_backend.hpp"
#include "render_data.hpp"
#include "render_errors.hpp"
#include "render_shared.hpp"
#include "renderer_settings.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <new>
#include <span>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include "vulkan_backend.hpp"
#include "vulkan_gpu_probe.hpp"

// Pulls in stb_image once for createTextureFromFile; no other file in the
// module decodes images.
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

namespace engine
{
namespace renderer
{
namespace
{

// Backend actually powering a renderer (vs. what the configuration asked
// for, RendererSettings::backend).
enum class Backend
{
    None,
    Vulkan,
    Headless
};

// One live renderer: its backend object plus the per-renderer state the
// module manages on its behalf - the version-gated frame input feeds, the
// draw-list scheduler, the negotiated light budget and frame timing.
struct RendererRecord
{
    RendererHandle       id      = invalidRenderer;
    Backend              backend = Backend::None;
    window::WindowHandle surface = 0;

    // How many direct lights beginFrame may copy for this renderer; bound
    // once at creation through the direct_lighting capability's device
    // profile (see registerRenderer).
    std::uint32_t directLightLimit = direct_lighting::maxDirectLights;

    FrameInputs    frameInputs{};
    DrawScheduler  drawScheduler{};

    std::chrono::steady_clock::time_point frameStart{};
    bool                                  frameTimingActive = false;

    std::unique_ptr<RenderBackend> backendInstance = nullptr;
};

// Main-thread module object. All capability entry points funnel through the
// *Impl methods below; assignCapabilityPointers() wires the lambdas once.
class RenderModule final : public ModuleInterface
{
public:
    RenderModule()
        : manifest_(makeManifest())
    {
        renderers_.reserve(16);
        assignCapabilityPointers();
    }

    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    // Requires the window capability; the Vulkan surface and the lighting
    // providers are optional and probed. Reads the configuration and
    // subscribes to ConfigurationReloadedEvent for live updates.
    void construct(ParticipantContext &context) override
    {
        window_ = &context.acquireCapability<window::WindowCapability>();
#if defined(ENGINE_RENDERER_BACKEND_VULKAN)
        vulkanSurface_ = context.tryAcquireCapability<window::VulkanSurfaceCapability>();
#endif

        pbr_             = context.tryAcquireCapability<pbr::PbrCapability>();
        directLighting_  = context.tryAcquireCapability<direct_lighting::DirectLightingCapability>();
        lumenGi_         = context.tryAcquireCapability<lumen_gi::LumenGiCapability>();
        lumenReflection_ = context.tryAcquireCapability<lumen_reflection::LumenReflectionCapability>();
        context_         = &context;
        profiler_        = &context.profiler();
        readConfiguration();
        reloadSubscription_ = context.eventBus()
                                  .channel<ConfigurationReloadedEvent>()
                                  .subscribe<RenderModule, &RenderModule::onConfigurationReloaded>(this);
    }

    void start() override
    {}

    void stop() override
    {
        renderers_.clear();
        if (reloadSubscription_ != 0 && context_ != nullptr)
        {
            context_->eventBus().channel<ConfigurationReloadedEvent>().unsubscribe(reloadSubscription_);
            reloadSubscription_ = 0;
        }
        context_        = nullptr;
        profiler_       = nullptr;
        window_         = nullptr;
        pbr_            = nullptr;
        directLighting_ = nullptr;
        lumenGi_        = nullptr;
        lumenReflection_ = nullptr;
#if defined(ENGINE_RENDERER_BACKEND_VULKAN)
        vulkanSurface_ = nullptr;
#endif
    }

    void *capabilityPointerFor(const std::string &capabilityName) override
    {
        if (capabilityName == RenderCapability::capabilityName)
        {
            return &renderCapability_;
        }
        if (capabilityName == VulkanRenderCapability::capabilityName)
        {
            return &vulkanCapability_;
        }
        if (capabilityName == HeadlessRenderCapability::capabilityName)
        {
            return &headlessCapability_;
        }
        return nullptr;
    }

private:
    // Re-reads the whole [renderer] settings set. The values take effect on
    // the next renderer or draw call (some, like backend, only for renderers
    // created later). Toggling auto_config re-derives (or un-derives)
    // everything on the next renderer/draw call, exactly like editing any
    // other key.
    void readConfiguration()
    {
        if (context_ == nullptr)
        {
            return;
        }
        settings_ = readRendererSettings(*context_);
    }

    void onConfigurationReloaded(const ConfigurationReloadedEvent &event)
    {
        const bool changed = std::any_of(event.changedKeys.begin(),
            event.changedKeys.end(),
            [](const std::string &key) { return key.rfind("renderer.", 0) == 0; });
        if (changed)
        {
            readConfiguration();
        }
    }

    // Must mirror module.toml: provided/required/optional capability lists.
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "renderer",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"renderer", "renderer.vulkan", "renderer.headless"},
            .requiredCapabilities = {"window"},
            .optionalCapabilities = {"window.vulkan", "pbr", "direct_lighting", "lumen_gi", "lumen_reflection"}};
    }

    RendererRecord *recordFor(RendererHandle renderer)
    {
        const auto found = renderers_.find(renderer);
        return found != renderers_.end() ? &found->second : nullptr;
    }

    const RendererRecord *recordFor(RendererHandle renderer) const
    {
        const auto found = renderers_.find(renderer);
        return found != renderers_.end() ? &found->second : nullptr;
    }

    const char *activeBackendNameImpl(RendererHandle renderer) const
    {
        const RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr)
        {
            return "none";
        }
        return record->backendInstance->name();
    }

    // Picks the backend for a new renderer. "auto" is hardware-aware:
    // Vulkan when the machine exposes a real GPU (probeHostGpu: discrete,
    // integrated or virtual device with a graphics queue - software Vulkan
    // implementations report a CPU device class and do not count) AND the
    // surface can present to Vulkan; headless otherwise (no GPU, CPU-only
    // Vulkan implementations, non-Vulkan surfaces, or builds without the
    // Vulkan backend). An explicit backend = "vulkan" that cannot start
    // reports the error as-is; the headless rasterizer cannot drive a
    // Vulkan-API surface, so there is no silent fallback.
    RendererHandle createRendererImpl(window::WindowHandle surface, RenderError *error)
    {
        if (window_ == nullptr)
        {
            reportError(error, RenderError::BackendUnavailable);
            return invalidRenderer;
        }

        const window::SurfaceApi surfaceApi = window_->surfaceApi(window_->self, surface);
        bool                     wantVulkan = false;
        switch (settings_.backend)
        {
        case ConfiguredBackend::Auto:
#if defined(ENGINE_RENDERER_BACKEND_VULKAN)
            wantVulkan = surfaceApi == window::SurfaceApi::Vulkan && vulkanSurface_ != nullptr
                      && probeHostGpu().gpuPresent;
#else
            wantVulkan = false; // no Vulkan backend in this build: auto is headless
#endif
            break;
        case ConfiguredBackend::Vulkan:
            wantVulkan = true;
            break;
        case ConfiguredBackend::Headless:
            wantVulkan = false;
            break;
        }

        if (wantVulkan)
        {
#if defined(ENGINE_RENDERER_BACKEND_VULKAN)
            if (surfaceApi != window::SurfaceApi::Vulkan || vulkanSurface_ == nullptr)
            {
                reportError(error, RenderError::BackendUnavailable);
                return invalidRenderer;
            }
            return registerRenderer(Backend::Vulkan,
                surface,
                VulkanBackend::create(surface,
                    settings_.vsync,
                    settings_.validation,
                    settings_.maxFramesInFlight,
                    settings_.maxPendingUploads,
                    settings_.memoryBudgetUpdateIntervalFrames,
                    settings_.devicePreference,
                    settings_.frustumCulling,
                    settings_.backFaceCulling,
                    settings_.gpuTimestampsEnabled,
                    settings_.instanceStreamMegabytes,
                    settings_.textureMipmapsEnabled,
                    settings_.anisotropyLevel,
                    settings_.msaaSamples,
                    settings_.srgbOutput,
                    settings_.maxTexturesPerRenderer,
                    settings_.maxMaterialSetsPerRenderer,
                    settings_.maxPipelineCacheBytes,
                    vulkanSurface_,
                    error,
                    settings_.presentMode),
                error);
#else
            reportError(error, RenderError::BackendUnavailable);
            return invalidRenderer;
#endif
        }

        if (surfaceApi == window::SurfaceApi::Vulkan)
        {
            reportError(error, RenderError::BackendUnavailable);
            return invalidRenderer;
        }

        try
        {
            HeadlessBackendOptions headlessOptions;
            headlessOptions.transformCacheBudgetBytes        = settings_.headlessTransformCacheBudgetBytes;
            headlessOptions.frustumCullingEnabled            = settings_.frustumCulling;
            headlessOptions.backFaceCullingEnabled           = settings_.backFaceCulling;
            headlessOptions.lodEnabled                       = settings_.lodEnabled;
            headlessOptions.tileParallelEnabled              = settings_.headlessTileParallel;
            headlessOptions.batchedTileRasterizationEnabled  = settings_.headlessBatchedTileRasterization;
            headlessOptions.taskScheduler
                = context_ != nullptr ? &context_->taskScheduler() : nullptr;
            headlessOptions.msaaSamples                       = settings_.msaaSamples;
            headlessOptions.parallelTriangleThreshold         = settings_.headlessParallelTriangleThreshold;
            headlessOptions.pixelsPerLockGroup                = settings_.headlessPixelsPerLockGroup;
            headlessOptions.lodMinTriangles                   = settings_.lodMinTriangles;
            headlessOptions.bandSplitTriangleThreshold        = settings_.headlessBandSplitTriangleThreshold;
            headlessOptions.bandSplitMinimumScreenFraction    = settings_.headlessBandSplitMinimumScreenFraction;
            headlessOptions.parallelTransformVertexThreshold  = settings_.headlessParallelTransformVertexThreshold;
            headlessOptions.tilePixels                        = settings_.headlessTilePixels;
            headlessOptions.maxBatchTileRecords               = settings_.headlessMaxBatchTileRecords;
            headlessOptions.lodThresholds                     = settings_.lodThresholds;
            headlessOptions.lodStrides                        = settings_.lodStrides;
            return registerRenderer(Backend::Headless,
                surface,
                std::make_unique<HeadlessBackend>(surface, window_, headlessOptions),
                error);
        }
        catch (const std::bad_alloc &)
        {
            reportError(error, RenderError::CreationFailed);
            return invalidRenderer;
        }
    }

    // Assigns the next renderer id and consults the direct_lighting
    // capability's device profile to bound how many lights each frame copies.
    RendererHandle registerRenderer(Backend backend, window::WindowHandle surface,
        std::unique_ptr<RenderBackend> backendInstance, RenderError *error)
    {
        if (backendInstance == nullptr || nextRendererId_ == invalidRenderer)
        {
            reportError(error, RenderError::CreationFailed);
            return invalidRenderer;
        }
        const RendererHandle id = nextRendererId_++;
        RendererRecord       record{.id = id, .backend = backend, .surface = surface};
        record.backendInstance = std::move(backendInstance);
        // Attach the shared profiler so backends record their explicit wait
        // points (frame-slot wait, acquire/present, upload fences) into the
        // same percentile histograms every other module reports to.
        record.backendInstance->setProfiler(profiler_);
        if (directLighting_ != nullptr && directLighting_->resolveProfile != nullptr)
        {
            const DeviceProfile                    deviceProfile = record.backendInstance->deviceProfile();
            direct_lighting::DirectLightingProfile lightProfile{};
            if (directLighting_->resolveProfile(directLighting_->self, &deviceProfile, &lightProfile))
            {
                record.directLightLimit = std::min(lightProfile.maxLights, direct_lighting::maxDirectLights);
            }
        }
        try
        {
            renderers_.emplace(id, std::move(record));
        }
        catch (const std::bad_alloc &)
        {
            reportError(error, RenderError::CreationFailed);
            return invalidRenderer;
        }
        reportError(error, RenderError::None);
        return id;
    }

    void destroyRendererImpl(RendererHandle renderer)
    {
        renderers_.erase(renderer);
    }

    void setViewportImpl(RendererHandle renderer, std::uint32_t width, std::uint32_t height)
    {
        RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr)
        {
            return;
        }
        record->backendInstance->setViewport(width, height);
    }

    void setCameraImpl(RendererHandle renderer, const CameraMatrices *camera)
    {
        RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr || camera == nullptr)
        {
            return;
        }
        record->backendInstance->setCamera(*camera);
    }

    // Pushes the version-gated lighting inputs (pbr environment, direct
    // lights, lumen_gi probe grid, lumen_reflection field), starts frame
    // timing for the profiler, then hands off to the backend.
    // frame == nullptr selects the configured clear_color.
    void beginFrameImpl(RendererHandle renderer, const FrameParameters *frame)
    {
        RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr)
        {
            return;
        }
        FrameParameters configuredFrame{};
        for (std::size_t index = 0; index < settings_.clearColor.size(); ++index)
        {
            configuredFrame.clearColor[index] = settings_.clearColor[index];
        }
        const FrameParameters &frameParameters = frame != nullptr ? *frame : configuredFrame;

        RenderBackend &backend = *record->backendInstance;
        record->frameInputs.pushEnvironment(backend, pbr_);
        record->frameInputs.pushDirectLights(backend, directLighting_, record->directLightLimit);
        record->frameInputs.pushProbeGrid(backend, lumenGi_, settings_.giIndirectLighting);
        record->frameInputs.pushReflectionField(backend, lumenReflection_, settings_.reflectionIndirectSpecular);

        record->frameStart        = std::chrono::steady_clock::now();
        record->frameTimingActive = true;
        backend.beginFrame(frameParameters);
    }

    // Ends the backend frame and records the wall-clock frame time under
    // PerformanceMetric::RenderFrame.
    void endFrameImpl(RendererHandle renderer)
    {
        RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr)
        {
            return;
        }
        record->backendInstance->endFrame();
        if (record->frameTimingActive && profiler_ != nullptr)
        {
            const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - record->frameStart)
                                     .count();
            profiler_->record(
                PerformanceMetric::RenderFrame, static_cast<std::uint64_t>(std::max<std::int64_t>(elapsed, 0)));
        }
        record->frameTimingActive = false;
    }

    // Validates + normalizes through render_data.cpp first so both backends
    // see the same canonical data; the module assigns mesh ids.
    MeshHandle createMeshImpl(RendererHandle renderer, const MeshDescription *description, RenderError *error)
    {
        RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr || description == nullptr)
        {
            reportError(error, RenderError::InvalidHandle);
            return invalidMesh;
        }

        try
        {
            PreparedMeshData prepared;
            if (!prepareMeshData(*description, prepared, error))
            {
                return invalidMesh;
            }
            const MeshDescription normalized = prepared.description();

            if (nextMeshId_ == invalidMesh)
            {
                reportError(error, RenderError::CreationFailed);
                return invalidMesh;
            }
            const MeshHandle meshId = nextMeshId_++;
            if (!record->backendInstance->createMesh(meshId, normalized, error))
            {
                return invalidMesh;
            }
            reportError(error, RenderError::None);
            return meshId;
        }
        catch (const std::bad_alloc &)
        {
            reportError(error, RenderError::CreationFailed);
            return invalidMesh;
        }
    }

    void destroyMeshImpl(RendererHandle renderer, MeshHandle mesh)
    {
        RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr)
        {
            return;
        }
        record->backendInstance->destroyMesh(mesh);
    }

    void drawMeshImpl(RendererHandle renderer, MeshHandle mesh, const float *modelMatrix, const MeshMaterial *material)
    {
        RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr || modelMatrix == nullptr)
        {
            return;
        }
        record->backendInstance->drawMesh(mesh, modelMatrix, effectiveRenderMaterial(material));
    }

    void drawMeshInstancesImpl(RendererHandle renderer, MeshHandle mesh, const float *modelMatrices,
        std::uint32_t instanceCount, const MeshMaterial *material)
    {
        RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr || modelMatrices == nullptr || instanceCount == 0
            || mesh == invalidMesh)
        {
            return;
        }
        record->backendInstance->drawMeshInstances(
            mesh, modelMatrices, instanceCount, effectiveRenderMaterial(material));
    }

    // Shell-fur draws: the [renderer] fur_enabled switch gates the whole
    // path (a disabled fur draws nothing, so applications can A/B the cost
    // with one config change), and layerCount is clamped to fur_max_layers -
    // the hard worst-case bound that keeps a buggy LOD policy from turning
    // one fur surface into an unbounded overdraw spike. Everything else
    // passes through to the backend unchanged: the layer loop IS the
    // compositing order, so no sorting or batching may reorder it.
    void drawFurImpl(RendererHandle renderer, MeshHandle mesh, const float *modelMatrix, const MeshMaterial *material,
        const FurLayerParams *params)
    {
        RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr || modelMatrix == nullptr || mesh == invalidMesh
            || !settings_.furEnabled)
        {
            return;
        }
        FurLayerParams clamped = params != nullptr ? *params : FurLayerParams{};
        clamped.layerCount     = std::clamp(clamped.layerCount, 0u, settings_.furMaximumLayers);
        record->backendInstance->drawFur(
            mesh, modelMatrix, effectiveRenderMaterial(material), clamped);
    }

    void drawFurInstancesImpl(RendererHandle renderer, MeshHandle mesh, const float *modelMatrices,
        std::uint32_t instanceCount, const MeshMaterial *material, const FurLayerParams *params)
    {
        RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr || modelMatrices == nullptr || instanceCount == 0
            || mesh == invalidMesh || !settings_.furEnabled)
        {
            return;
        }
        FurLayerParams clamped = params != nullptr ? *params : FurLayerParams{};
        clamped.layerCount     = std::clamp(clamped.layerCount, 0u, settings_.furMaximumLayers);
        record->backendInstance->drawFurInstances(
            mesh, modelMatrices, instanceCount, effectiveRenderMaterial(material), clamped);
    }

    void drawOverlayImpl(RendererHandle renderer, const OverlayDrawData *data)
    {
        RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr || data == nullptr || data->vertices == nullptr
            || data->indices == nullptr || data->commands == nullptr)
        {
            return;
        }
        record->backendInstance->drawOverlay(*data);
    }

    // The batched draw path: sorted and batched by the per-renderer
    // scheduler (draw_scheduler.cpp).
    void drawMeshesImpl(RendererHandle renderer, const RenderDrawCommand *commands, std::uint32_t commandCount)
    {
        RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr || commands == nullptr || commandCount == 0)
        {
            return;
        }
        record->drawScheduler.submit(*record->backendInstance,
            record->backend == Backend::Vulkan,
            commands,
            commandCount,
            settings_,
            profiler_);
    }

    TextureHandle createTextureImpl(RendererHandle renderer, const TextureDescription *description, RenderError *error)
    {
        RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr || description == nullptr)
        {
            reportError(error, RenderError::InvalidHandle);
            return invalidTexture;
        }

        try
        {
            PreparedTextureData prepared;
            if (!prepareTextureData(*description, prepared, error))
            {
                return invalidTexture;
            }
            const TextureDescription normalized = prepared.description();

            if (nextTextureId_ == invalidTexture)
            {
                reportError(error, RenderError::CreationFailed);
                return invalidTexture;
            }
            const TextureHandle textureId = nextTextureId_++;
            if (!record->backendInstance->createTexture(textureId, normalized, error))
            {
                return invalidTexture;
            }
            reportError(error, RenderError::None);
            return textureId;
        }
        catch (const std::bad_alloc &)
        {
            reportError(error, RenderError::CreationFailed);
            return invalidTexture;
        }
    }

    // Decodes with stb_image (forced to RGBA8), uploads, and always frees
    // the decoded pixels; the texture keeps its own copy.
    TextureHandle createTextureFromFileImpl(RendererHandle renderer, const char *filePath, RenderError *error)
    {
        RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr)
        {
            reportError(error, RenderError::InvalidHandle);
            return invalidTexture;
        }
        if (filePath == nullptr)
        {
            reportError(error, RenderError::InvalidTexture);
            return invalidTexture;
        }
        int width          = 0;
        int height         = 0;
        int channelsInFile = 0;

        stbi_uc *pixels = stbi_load(filePath, &width, &height, &channelsInFile, 4);
        if (pixels == nullptr)
        {
            std::fprintf(stderr, "renderer module: could not decode texture '%s': %s\n", filePath, stbi_failure_reason());
            reportError(error, RenderError::InvalidTexture);
            return invalidTexture;
        }
        const TextureDescription description{
            .width   = static_cast<std::uint32_t>(width),
            .height  = static_cast<std::uint32_t>(height),
            .pixels  = pixels,
            .filter  = TextureFilter::Linear,
            .address = TextureAddress::Repeat,
        };
        const TextureHandle textureId = createTextureImpl(renderer, &description, error);
        stbi_image_free(pixels);
        return textureId;
    }

    void destroyTextureImpl(RendererHandle renderer, TextureHandle texture)
    {
        RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr)
        {
            return;
        }
        record->backendInstance->destroyTexture(texture);
    }

    DeviceClass deviceClassImpl(RendererHandle renderer) const
    {
        const RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr)
        {
            return DeviceClass::Unknown;
        }
        return record->backendInstance->deviceClass();
    }

    const char *deviceNameImpl(RendererHandle renderer) const
    {
        const RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr)
        {
            return nullptr;
        }
        return record->backendInstance->deviceName();
    }

    bool deviceProfileImpl(RendererHandle renderer, DeviceProfile *outProfile) const
    {
        const RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr || outProfile == nullptr)
        {
            return false;
        }
        *outProfile = record->backendInstance->deviceProfile();
        return true;
    }

    void frameStatisticsImpl(RendererHandle renderer, FrameStatistics *outStatistics) const
    {
        const RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr || outStatistics == nullptr)
        {
            return;
        }
        *outStatistics = record->backendInstance->statistics();
    }

    // Whether endFrame already blocks on the display's vblank. Unknown
    // handles report false (nothing to assume about pacing).
    [[nodiscard]] bool waitsForPresentationImpl(RendererHandle renderer) const
    {
        const RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr)
        {
            return false;
        }
        return record->backendInstance->waitsForPresentation();
    }

    bool vulkanInfoStringsImpl(RendererHandle renderer, RendererInfoStrings *outInfo) const
    {
#if defined(ENGINE_RENDERER_BACKEND_VULKAN)
        const RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backend != Backend::Vulkan)
        {
            return false;
        }
        return static_cast<VulkanBackend *>(record->backendInstance.get())->infoStrings(outInfo);
#else
        (void)renderer;
        (void)outInfo;
        return false;
#endif
    }

    bool vulkanNativeHandlesImpl(RendererHandle renderer, VulkanDeviceHandles *outHandles) const
    {
#if defined(ENGINE_RENDERER_BACKEND_VULKAN)
        const RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backend != Backend::Vulkan)
        {
            return false;
        }
        return static_cast<VulkanBackend *>(record->backendInstance.get())->nativeHandles(outHandles);
#else
        (void)renderer;
        (void)outHandles;
        return false;
#endif
    }

    bool vulkanDeviceProfileImpl(RendererHandle renderer, DeviceProfile *outProfile) const
    {
#if defined(ENGINE_RENDERER_BACKEND_VULKAN)
        if (outProfile == nullptr)
            return false;
        const RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backend != Backend::Vulkan)
            return false;
        *outProfile = static_cast<VulkanBackend *>(record->backendInstance.get())->deviceProfile();
        return true;
#else
        (void)renderer;
        (void)outProfile;
        return false;
#endif
    }

    bool vulkanExportFramePpmImpl(RendererHandle renderer, const char *filePath, RenderError *error)
    {
#if defined(ENGINE_RENDERER_BACKEND_VULKAN)
        const RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backend != Backend::Vulkan)
        {
            reportError(error, RenderError::Unsupported);
            return false;
        }
        return static_cast<VulkanBackend *>(record->backendInstance.get())->exportFramePpm(filePath, error);
#else
        (void)renderer;
        (void)filePath;
        reportError(error, RenderError::Unsupported);
        return false;
#endif
    }

    void headlessStatisticsImpl(RendererHandle renderer, HeadlessRenderStatistics *outStatistics) const
    {
        const RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backend != Backend::Headless || outStatistics == nullptr)
        {
            return;
        }
        *outStatistics = static_cast<HeadlessBackend *>(record->backendInstance.get())->headlessStatistics();
    }

    // Fills each capability struct's function pointers exactly once, at
    // construction. The lambdas just cast self back to RenderModule.
    void assignCapabilityPointers()
    {
        renderCapability_.self   = this;
        vulkanCapability_.self   = this;
        headlessCapability_.self = this;

        renderCapability_.activeBackendName = [](void *self, RendererHandle renderer)
        { return static_cast<RenderModule *>(self)->activeBackendNameImpl(renderer); };
        renderCapability_.deviceClass = [](void *self, RendererHandle renderer)
        { return static_cast<RenderModule *>(self)->deviceClassImpl(renderer); };
        renderCapability_.deviceName = [](void *self, RendererHandle renderer)
        { return static_cast<RenderModule *>(self)->deviceNameImpl(renderer); };
        renderCapability_.deviceProfile = [](void *self, RendererHandle renderer, DeviceProfile *outProfile)
        { return static_cast<RenderModule *>(self)->deviceProfileImpl(renderer, outProfile); };
        renderCapability_.createRenderer = [](void *self, window::WindowHandle surface, RenderError *error)
        { return static_cast<RenderModule *>(self)->createRendererImpl(surface, error); };
        renderCapability_.destroyRenderer = [](void *self, RendererHandle renderer)
        { static_cast<RenderModule *>(self)->destroyRendererImpl(renderer); };
        renderCapability_.setViewport
            = [](void *self, RendererHandle renderer, std::uint32_t width, std::uint32_t height)
        { static_cast<RenderModule *>(self)->setViewportImpl(renderer, width, height); };
        renderCapability_.setCamera = [](void *self, RendererHandle renderer, const CameraMatrices *camera)
        { static_cast<RenderModule *>(self)->setCameraImpl(renderer, camera); };
        renderCapability_.createMesh
            = [](void *self, RendererHandle renderer, const MeshDescription *description, RenderError *error)
        { return static_cast<RenderModule *>(self)->createMeshImpl(renderer, description, error); };
        renderCapability_.destroyMesh = [](void *self, RendererHandle renderer, MeshHandle mesh)
        { static_cast<RenderModule *>(self)->destroyMeshImpl(renderer, mesh); };
        renderCapability_.drawMesh = [](void                *self,
                                         RendererHandle      renderer,
                                         MeshHandle          mesh,
                                         const float        *modelMatrix,
                                         const MeshMaterial *material)
        { static_cast<RenderModule *>(self)->drawMeshImpl(renderer, mesh, modelMatrix, material); };
        renderCapability_.drawOverlay = [](void *self, RendererHandle renderer, const OverlayDrawData *data)
        { static_cast<RenderModule *>(self)->drawOverlayImpl(renderer, data); };
        renderCapability_.drawMeshInstances = [](void                *self,
                                                  RendererHandle      renderer,
                                                  MeshHandle          mesh,
                                                  const float        *modelMatrices,
                                                  std::uint32_t       instanceCount,
                                                  const MeshMaterial *material)
        {
            static_cast<RenderModule *>(self)->drawMeshInstancesImpl(
                renderer, mesh, modelMatrices, instanceCount, material);
        };
        renderCapability_.drawMeshes
            = [](void *self, RendererHandle renderer, const RenderDrawCommand *commands, std::uint32_t count)
        { static_cast<RenderModule *>(self)->drawMeshesImpl(renderer, commands, count); };
        renderCapability_.createTexture
            = [](void *self, RendererHandle renderer, const TextureDescription *description, RenderError *error)
        { return static_cast<RenderModule *>(self)->createTextureImpl(renderer, description, error); };
        renderCapability_.createTextureFromFile
            = [](void *self, RendererHandle renderer, const char *filePath, RenderError *error)
        { return static_cast<RenderModule *>(self)->createTextureFromFileImpl(renderer, filePath, error); };
        renderCapability_.destroyTexture = [](void *self, RendererHandle renderer, TextureHandle texture)
        { static_cast<RenderModule *>(self)->destroyTextureImpl(renderer, texture); };
        renderCapability_.beginFrame = [](void *self, RendererHandle renderer, const FrameParameters *frame)
        { static_cast<RenderModule *>(self)->beginFrameImpl(renderer, frame); };
        renderCapability_.endFrame
            = [](void *self, RendererHandle renderer) { static_cast<RenderModule *>(self)->endFrameImpl(renderer); };
        renderCapability_.frameStatistics = [](void *self, RendererHandle renderer, FrameStatistics *outStatistics)
        { return static_cast<const RenderModule *>(self)->frameStatisticsImpl(renderer, outStatistics); };
        renderCapability_.waitsForPresentation = [](void *self, RendererHandle renderer)
        { return static_cast<const RenderModule *>(self)->waitsForPresentationImpl(renderer); };
        renderCapability_.drawFur = [](void *self,
                                        RendererHandle renderer,
                                        MeshHandle mesh,
                                        const float *modelMatrix,
                                        const MeshMaterial *material,
                                        const FurLayerParams *params)
        { static_cast<RenderModule *>(self)->drawFurImpl(renderer, mesh, modelMatrix, material, params); };
        renderCapability_.drawFurInstances = [](void *self,
                                                RendererHandle renderer,
                                                MeshHandle mesh,
                                                const float *modelMatrices,
                                                std::uint32_t instanceCount,
                                                const MeshMaterial *material,
                                                const FurLayerParams *params)
        {
            static_cast<RenderModule *>(self)->drawFurInstancesImpl(
                renderer, mesh, modelMatrices, instanceCount, material, params);
        };

        vulkanCapability_.infoStrings = [](void *self, RendererHandle renderer, RendererInfoStrings *outInfo)
        { return static_cast<RenderModule *>(self)->vulkanInfoStringsImpl(renderer, outInfo); };
        vulkanCapability_.nativeHandles = [](void *self, RendererHandle renderer, VulkanDeviceHandles *outHandles)
        { return static_cast<RenderModule *>(self)->vulkanNativeHandlesImpl(renderer, outHandles); };
        vulkanCapability_.deviceProfile = [](void *self, RendererHandle renderer, DeviceProfile *outProfile)
        { return static_cast<RenderModule *>(self)->vulkanDeviceProfileImpl(renderer, outProfile); };
        vulkanCapability_.exportFramePpm
            = [](void *self, RendererHandle renderer, const char *filePath, RenderError *error)
        { return static_cast<RenderModule *>(self)->vulkanExportFramePpmImpl(renderer, filePath, error); };

        headlessCapability_.statistics
            = [](void *self, RendererHandle renderer, HeadlessRenderStatistics *outStatistics)
        { static_cast<RenderModule *>(self)->headlessStatisticsImpl(renderer, outStatistics); };
    }

    RenderCapability         renderCapability_{};
    VulkanRenderCapability   vulkanCapability_{};
    HeadlessRenderCapability headlessCapability_{};

    ParticipantContext  *context_            = nullptr;
    PerformanceProfiler *profiler_           = nullptr;
    SubscriptionToken    reloadSubscription_ = 0;

    window::WindowCapability *window_ = nullptr;

    // Borrowed lighting providers (all optional; the frame inputs fall back
    // to the built-in default environment and empty light/probe/field sets).
    pbr::PbrCapability                        *pbr_             = nullptr;
    direct_lighting::DirectLightingCapability *directLighting_  = nullptr;
    lumen_gi::LumenGiCapability               *lumenGi_         = nullptr;
    lumen_reflection::LumenReflectionCapability *lumenReflection_ = nullptr;

#if defined(ENGINE_RENDERER_BACKEND_VULKAN)
    window::VulkanSurfaceCapability *vulkanSurface_ = nullptr;
#endif

    // Current settings; read at construct and on every renderer.* reload
    // (see renderer_settings.cpp). Draw-path values apply on the next frame;
    // backend/device values apply to renderers created afterwards.
    RendererSettings settings_{};

    // Mesh/texture ids are global across renderers and never reused.
    std::unordered_map<RendererHandle, RendererRecord> renderers_;
    RendererHandle                                     nextRendererId_ = 1;
    MeshHandle                                         nextMeshId_     = 1;
    TextureHandle                                      nextTextureId_  = 1;

    ParticipantManifest manifest_;
};

} // namespace

} // namespace renderer

using renderer::RenderModule;

} // namespace engine

// The kernel's only required export: module metadata + factory. The lists
// here must match makeManifest() and module.toml.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const providedCapabilities[] = {"renderer", "renderer.vulkan", "renderer.headless"};
    static const char *const requiredCapabilities[] = {"window"};
    static const char *const optionalCapabilities[]
        = {"window.vulkan", "pbr", "direct_lighting", "lumen_gi", "lumen_reflection"};

    static const engine::EngineModuleEntryTable entry = {
        .abiVersion              = engine::engineModuleAbiVersion,
        .moduleName              = "renderer",
        .moduleVersion           = "0.1.0",
        .createModule            = []() -> engine::ModuleInterface            *{ return new engine::renderer::RenderModule(); },
        .providedCapabilities    = providedCapabilities,
        .providedCapabilityCount = 3,
        .requiredCapabilities    = requiredCapabilities,
        .requiredCapabilityCount = 1,
        .optionalCapabilities    = optionalCapabilities,
        .optionalCapabilityCount = 5,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0,
    };
    return &entry;
}
