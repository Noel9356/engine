// render module entry point: the ModuleInterface implementation that binds
// the "render" capability family to a backend instance.
//
// Responsibilities: read/refresh [render] configuration, acquire the window/
// pbr/direct_lighting capabilities, create and route renderers (Vulkan or
// headless), normalize mesh/texture uploads, sort and batch drawMeshes
// command lists, and expose the module entry table for the kernel's loader.

#include <cstdint>
#include <engine/direct_lighting/capabilities.hpp>
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/render/capabilities.hpp>
#include <engine/window/capabilities.hpp>

#include "engine/kernel/event_bus.hpp"
#include "engine/kernel/profiling.hpp"
#include "engine/render/device_profile.hpp"
#include "engine/render/material.hpp"
#include "engine/render/mesh.hpp"
#include <engine/render/draw_sort.hpp>

#include "headless_backend.hpp"
#include "render_backend.hpp"
#include "render_data.hpp"
#include "render_shared.hpp"
#include "vulkan_backend.hpp"
#include <utility>

// Pulls in stb_image once for createTextureFromFile; no other file in the
// module decodes images.
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <new>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine
{
namespace render
{
namespace
{

// Backend actually powering a renderer vs. what the configuration asked for.
enum class Backend
{
    None,
    Vulkan,
    Headless
};

enum class ConfiguredBackend
{
    Auto, // follow the window surface's API (Vulkan surface -> Vulkan backend)
    Vulkan,
    Headless
};

// One live renderer: its backend object plus the state render.cpp manages on
// its behalf (environment/light versioning, frame timing, draw scratch).
struct RendererRecord
{
    RendererHandle       id      = invalidRenderer;
    Backend              backend = Backend::None;
    window::WindowHandle surface = 0;

    // Environment and light copies are pushed into the backend only when the
    // provider's version counter changes (see beginFrameImpl).
    std::uint64_t                         environmentVersion      = 0;
    bool                                  environmentInitialized  = false;
    std::uint64_t                         directLightVersion      = 0;
    std::uint32_t                         directLightLimit        = direct_lighting::maxDirectLights;
    bool                                  directLightsInitialized = false;
    std::chrono::steady_clock::time_point frameStart{};
    bool                                  frameTimingActive = false;

    std::unique_ptr<RenderBackend> backendInstance = nullptr;

    // Scratch reused by drawMeshesImpl: the sorted copy of the command list
    // and the packed instance matrices for the current batch. Static geometry
    // redraws the same key set every frame, so the sorted ORDER is cached as
    // a permutation (validated by a hash over mesh/material keys) and only
    // the current frame's matrix pointers are gathered - the per-frame copy +
    // std::sort with the field-wise material comparator disappears.
    std::vector<RenderDrawCommand> sortedCommandsScratch{};
    std::vector<float>             instanceMatricesScratch{};
    std::vector<std::uint32_t>     sortPermutation{};
    std::uint64_t                  sortSignature      = 0;
    bool                           sortSignatureValid = false;
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

    // Requires the window capability; Vulkan surface / pbr / direct_lighting
    // are optional and probed. Re-reads configuration and subscribes to
    // ConfigurationReloadedEvent for live updates.
    void construct(ParticipantContext &context) override
    {
        window_ = &context.acquireCapability<window::WindowCapability>();
#if defined(ENGINE_RENDER_BACKEND_VULKAN)
        vulkanSurface_ = context.tryAcquireCapability<window::VulkanSurfaceCapability>();
#endif

        pbr_            = context.tryAcquireCapability<pbr::PbrCapability>();
        directLighting_ = context.tryAcquireCapability<direct_lighting::DirectLightingCapability>();
        context_        = &context;
        profiler_       = &context.profiler();
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
#if defined(ENGINE_RENDER_BACKEND_VULKAN)
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
    // Resets every setting to its built-in default, then overlays whatever
    // [render] keys are present. Out-of-range numeric values are clamped to
    // their validated range with a stderr warning; unknown enum strings warn
    // and fall back (backend/device to "auto", draw_sort_mode to disabled).
    // Note the built-in defaults here deliberately differ from config.toml
    // for the opt-in features (sorting, batching, mipmaps): they stay off
    // until the config file enables them.
    void readConfiguration()
    {
        if (context_ == nullptr)
        {
            return;
        }
        ParticipantContext &context              = *context_;
        configuredBackend_                       = ConfiguredBackend::Auto;
        devicePreference_                        = DevicePreference::Auto;
        vsync_                                   = true;
        validation_                              = false;
        maxFramesInFlight_                       = 2;
        maxPendingUploads_                = 64;
        maximumTexturesPerRenderer_       = 256;
        maximumMaterialSetsPerRenderer_   = 256;
        pipelineCacheMaximumBytes_        = 64u * 1024u * 1024u;
        memoryBudgetUpdateIntervalFrames_ = 120;
        configuredClearColor_                    = {0.0f, 0.0f, 0.0f, 1.0f};
        headlessTransformCacheBudgetBytes_       = 32u * 1024u * 1024u;
        frustumCulling_                          = true;
        backFaceCulling_                         = true;
        lodEnabled_                              = true;
        gpuTimestampsEnabled_                    = true;
        drawSortMode_                            = RenderDrawSortMode::MeshThenMaterial;
        drawSortEnabled_                         = false;
        drawSortMinimumCommands_                 = 2;
        instanceBatchingEnabled_                 = false;
        instanceBatchMinimumCount_               = 4;
        instanceStreamMegabytes_                 = 4;
        textureMipmapsEnabled_                   = false;
        const std::optional<std::string> backend = context.configurationStringInSection("render", "backend");
        if (backend.has_value())
        {
            if (*backend == "vulkan")
            {
                configuredBackend_ = ConfiguredBackend::Vulkan;
            }
            else if (*backend == "headless")
            {
                configuredBackend_ = ConfiguredBackend::Headless;
            }
            else if (*backend != "auto")
            {
                std::fprintf(
                    stderr, "render module: unknown [render] backend '%s'; using \"auto\"\n", backend->c_str());
            }
        }

        const std::optional<std::string> device = context.configurationStringInSection("render", "device");
        if (device.has_value())
        {
            if (*device == "discrete")
            {
                devicePreference_ = DevicePreference::Discrete;
            }
            else if (*device == "integrated")
            {
                devicePreference_ = DevicePreference::Integrated;
            }
            else if (*device != "auto")
            {
                std::fprintf(stderr, "render module: unknown [render] device '%s'; using \"auto\"\n", device->c_str());
            }
        }

        vsync_      = context.configurationBooleanInSection("render", "vsync").value_or(true);
        validation_ = context.configurationBooleanInSection("render", "validation").value_or(false);

        const std::optional<std::int64_t> framesInFlight
            = context.configurationIntegerInSection("render", "max_frames_in_flight");
        if (framesInFlight.has_value())
        {
            if (*framesInFlight < 1 || *framesInFlight > 8)
            {
                std::fprintf(stderr,
                    "render module: clamping [render] max_frames_in_flight %lld to range 1..8\n",
                    (long long)*framesInFlight);
            }
            maxFramesInFlight_ = static_cast<std::uint32_t>(std::clamp<std::int64_t>(*framesInFlight, 1, 8));
        }

        const std::optional<std::int64_t> pendingUploads
            = context.configurationIntegerInSection("render", "max_pending_uploads");
        if (pendingUploads.has_value())
        {
            if (*pendingUploads < 1 || *pendingUploads > 4096)
            {
                std::fprintf(stderr,
                    "render module: clamping [render] max_pending_uploads %lld to range 1..4096\n",
                    static_cast<long long>(*pendingUploads));
            }
            maxPendingUploads_ = static_cast<std::uint32_t>(std::clamp<std::int64_t>(*pendingUploads, 1, 4096));
        }

        const auto readResourceLimit = [&context](const char *key, std::uint32_t &destination)
        {
            const auto value = context.configurationIntegerInSection("render", key);
            if (!value.has_value())
            {
                return;
            }
            if (*value < 1 || *value > 65536)
            {
                std::fprintf(stderr,
                    "render module: clamping [render] %s %lld to range 1..65536\n",
                    key,
                    static_cast<long long>(*value));
            }
            destination = static_cast<std::uint32_t>(std::clamp<std::int64_t>(*value, 1, 65536));
        };
        readResourceLimit("max_textures_per_renderer", maximumTexturesPerRenderer_);
        readResourceLimit("max_material_sets_per_renderer", maximumMaterialSetsPerRenderer_);

        if (const auto cacheMegabytes = context.configurationIntegerInSection("render", "pipeline_cache_mb");
            cacheMegabytes.has_value())
        {
            if (*cacheMegabytes < 0 || *cacheMegabytes > 512)
            {
                std::fprintf(stderr,
                    "render module: clamping [render] pipeline_cache_mb %lld to range 0..512\n",
                    static_cast<long long>(*cacheMegabytes));
            }
            pipelineCacheMaximumBytes_
                = static_cast<std::size_t>(std::clamp<std::int64_t>(*cacheMegabytes, 0, 512)) * 1024u * 1024u;
        }

        if (const auto interval
            = context.configurationIntegerInSection("render", "memory_budget_update_interval_frames");
            interval.has_value())
        {
            if (*interval < 0 || *interval > 3600)
            {
                std::fprintf(stderr,
                    "render module: clamping [render] memory_budget_update_interval_frames %lld to range 0..3600\n",
                    static_cast<long long>(*interval));
            }
            memoryBudgetUpdateIntervalFrames_
                = static_cast<std::uint32_t>(std::clamp<std::int64_t>(*interval, 0, 3600));
        }

        const std::vector<double> clearColor = context.configurationFloatingArrayInSection("render", "clear_color");
        if (clearColor.size() >= 4)
        {
            bool valid = true;
            for (std::size_t index = 0; index < configuredClearColor_.size(); ++index)
            {
                valid = valid && std::isfinite(clearColor[index]);
            }
            if (valid)
            {
                for (std::size_t index = 0; index < configuredClearColor_.size(); ++index)
                {
                    configuredClearColor_[index] = static_cast<float>(std::clamp(clearColor[index], 0.0, 1.0));
                }
            }
            else
            {
                std::fprintf(stderr, "render module: ignoring [render] clear_color; components must be finite\n");
            }
        }

        const std::optional<std::int64_t> transformCacheMb
            = context.configurationIntegerInSection("render", "headless_transform_cache_mb");
        if (transformCacheMb.has_value())
        {
            if (*transformCacheMb < 0 || *transformCacheMb > 1024)
            {
                std::fprintf(stderr,
                    "render module: clamping [render] headless_transform_cache_mb %lld to range 0..1024\n",
                    static_cast<long long>(*transformCacheMb));
            }
            const std::int64_t bounded         = std::clamp<std::int64_t>(*transformCacheMb, 0, 1024);
            headlessTransformCacheBudgetBytes_ = static_cast<std::size_t>(bounded) * 1024u * 1024u;
        }
        frustumCulling_  = context.configurationBooleanInSection("render", "frustum_culling").value_or(true);
        backFaceCulling_ = context.configurationBooleanInSection("render", "back_face_culling").value_or(true);
        lodEnabled_      = context.configurationBooleanInSection("render", "lod_enabled").value_or(true);
        gpuTimestampsEnabled_
            = context.configurationBooleanInSection("render", "gpu_timestamps_enabled").value_or(false);
        instanceBatchingEnabled_ = context.configurationBooleanInSection("render", "instance_batching").value_or(true);
        textureMipmapsEnabled_   = context.configurationBooleanInSection("render", "texture_mipmaps").value_or(true);
        if (const auto batchMinimum = context.configurationIntegerInSection("render", "instance_batch_min_count");
            batchMinimum.has_value())
        {
            if (*batchMinimum < 2 || *batchMinimum > 1024)
            {
                std::fprintf(stderr,
                    "render module: clamping [render] instance_batch_min_count %lld to range 2..1024\n",
                    static_cast<long long>(*batchMinimum));
            }
            instanceBatchMinimumCount_ = static_cast<std::uint32_t>(std::clamp<std::int64_t>(*batchMinimum, 2, 1024));
        }

        const std::optional<std::int64_t> sortMinimum
            = context.configurationIntegerInSection("render", "draw_sort_min_commands");
        if (sortMinimum.has_value())
        {
            if (*sortMinimum < 2 || *sortMinimum > 65536)
            {
                std::fprintf(stderr,
                    "render module: clamping [render] draw_sort_min_commands %lld to range 2..65536\n",
                    static_cast<long long>(*sortMinimum));
            }
            drawSortMinimumCommands_ = static_cast<std::uint32_t>(std::clamp<std::int64_t>(*sortMinimum, 2, 65536));
        }

        const std::optional<std::int64_t> instanceStreamMb
            = context.configurationIntegerInSection("render", "instance_stream_mb");
        if (instanceStreamMb.has_value())
        {
            if (*instanceStreamMb < 1 || *instanceStreamMb > 64)
            {
                std::fprintf(stderr,
                    "render module: clamping [render] instance_stream_mb %lld to range 1..64\n",
                    static_cast<long long>(*instanceStreamMb));
            }
            instanceStreamMegabytes_ = static_cast<std::uint32_t>(std::clamp<std::int64_t>(*instanceStreamMb, 1, 64));
        }
        if (const auto sortMode = context.configurationStringInSection("render", "draw_sort_mode"))
        {
            if (*sortMode == "mesh_material")
            {
                drawSortMode_    = RenderDrawSortMode::MeshThenMaterial;
                drawSortEnabled_ = true;
            }
            else if (*sortMode == "material_mesh")
            {
                drawSortMode_    = RenderDrawSortMode::MaterialThenMesh;
                drawSortEnabled_ = true;
            }
            else if (*sortMode == "off")
            {
                drawSortEnabled_ = false;
            }
            else
            {
                std::fprintf(stderr,
                    "render module: unknown [render] draw_sort_mode '%s'; sorting disabled\n",
                    sortMode->c_str());
                drawSortEnabled_ = false;
            }
        }
    }

    // Live reload: re-read configuration when any changed key starts with
    // "render.". Read only - the values take effect on the next renderer or
    // drawMeshes call (some, like backend, only for renderers created later).
    void onConfigurationReloaded(const ConfigurationReloadedEvent &event)
    {
        const bool changed = std::any_of(event.changedKeys.begin(),
            event.changedKeys.end(),
            [](const std::string &key) { return key.rfind("render.", 0) == 0; });
        if (changed)
        {
            readConfiguration();
        }
    }

    // Must mirror module.toml: provided/required/optional capability lists.
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "render",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"render", "render.vulkan", "render.headless"},
            .requiredCapabilities = {"window"},
            .optionalCapabilities = {"window.vulkan", "pbr", "direct_lighting"}};
    }

    RendererRecord *recordFor(RendererHandle renderer)
    {
        const auto found = renderers_.find(renderer);
        return found != renderers_.end() ? &found->second : nullptr;
    }

    pbr::PbrEnvironment currentEnvironment() const
    {
        if (pbr_ != nullptr)
        {
            const pbr::PbrEnvironment *environment = pbr_->environment(pbr_->self);
            if (environment != nullptr)
            {
                return *environment;
            }
        }
        return defaultEnvironment();
    }

    const char *activeBackendNameImpl(RendererHandle renderer) const
    {
        const auto found = renderers_.find(renderer);
        if (found == renderers_.end() || found->second.backendInstance == nullptr)
        {
            return "none";
        }
        return found->second.backendInstance->name();
    }

    // Picks the backend for a new renderer. "auto" follows the surface API:
    // a Vulkan surface needs the Vulkan backend (and window.vulkan), any
    // other surface takes the headless rasterizer. Errors are reported
    // through *error; a Vulkan request on a non-Vulkan build reports
    // BackendUnavailable.
    RendererHandle createRendererImpl(window::WindowHandle surface, RenderError *error)
    {
        if (window_ == nullptr)
        {
            reportError(error, RenderError::BackendUnavailable);
            return invalidRenderer;
        }

        const window::SurfaceApi surfaceApi = window_->surfaceApi(window_->self, surface);
        bool                     wantVulkan = false;
        switch (configuredBackend_)
        {
        case ConfiguredBackend::Auto:
            wantVulkan = surfaceApi == window::SurfaceApi::Vulkan;
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
#if defined(ENGINE_RENDER_BACKEND_VULKAN)
            if (surfaceApi != window::SurfaceApi::Vulkan || vulkanSurface_ == nullptr)
            {
                reportError(error, RenderError::BackendUnavailable);
                return invalidRenderer;
            }
            return registerRenderer(Backend::Vulkan,
                surface,
                VulkanBackend::create(surface,
                    vsync_,
                    validation_,
                    maxFramesInFlight_,
                    maxPendingUploads_,
                    memoryBudgetUpdateIntervalFrames_,
                    devicePreference_,
                    frustumCulling_,
                    backFaceCulling_,
                    gpuTimestampsEnabled_,
                    instanceStreamMegabytes_,
                    textureMipmapsEnabled_,
                    anisotropyLevel_,
                    msaaSamples_,
                    srgbOutput_,
                    maximumTexturesPerRenderer_,
                    maximumMaterialSetsPerRenderer_,
                    pipelineCacheMaximumBytes_,
                    vulkanSurface_,
                    error),
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
            headlessOptions.transformCacheBudgetBytes = headlessTransformCacheBudgetBytes_;
            headlessOptions.frustumCullingEnabled     = frustumCulling_;
            headlessOptions.backFaceCullingEnabled    = backFaceCulling_;
            headlessOptions.lodEnabled                = lodEnabled_;
            headlessOptions.taskScheduler             = context_ != nullptr ? &context_->taskScheduler() : nullptr;

            headlessOptions.parallelTriangleThreshold = static_cast<std::uint32_t>(
                context_->configurationIntegerInSection("render", "parallel_triangle_threshold").value_or(4096));
            headlessOptions.pixelsPerLockGroup = static_cast<std::size_t>(
                context_->configurationIntegerInSection("render", "pixels_per_lock_group").value_or(16));
            headlessOptions.lodMinTriangles = static_cast<std::size_t>(
                context_->configurationIntegerInSection("render", "lod_min_triangles").value_or(4096));
            headlessOptions.bandSplitTriangleThreshold = static_cast<std::uint32_t>(
                context_->configurationIntegerInSection("render", "headless_band_split_triangles").value_or(4096));
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

    // Pushes the pbr environment and direct lights into the backend when
    // their versions changed, starts frame timing for the profiler, then
    // hands off to the backend. frame == nullptr selects the configured
    // clear_color.
    void beginFrameImpl(RendererHandle renderer, const FrameParameters *frame)
    {
        RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr)
        {
            return;
        }
        FrameParameters configuredFrame{};
        for (std::size_t index = 0; index < configuredClearColor_.size(); ++index)
        {
            configuredFrame.clearColor[index] = configuredClearColor_[index];
        }
        const FrameParameters &frameParameters = frame != nullptr ? *frame : configuredFrame;

        if (pbr_ == nullptr)
        {
            if (!record->environmentInitialized)
            {
                record->backendInstance->setEnvironment(defaultEnvironment());
                record->environmentInitialized = true;
            }
        }
        else if (pbr_->environmentVersion != nullptr)
        {
            const std::uint64_t version = pbr_->environmentVersion(pbr_->self);
            if (!record->environmentInitialized || record->environmentVersion != version)
            {
                record->backendInstance->setEnvironment(currentEnvironment());
                record->environmentVersion     = version;
                record->environmentInitialized = true;
            }
        }
        else
        {
            record->backendInstance->setEnvironment(currentEnvironment());
        }
        if (directLighting_ != nullptr && directLighting_->copyLights != nullptr)
        {
            std::uint64_t version = 0;
            directLighting_->copyLights(directLighting_->self, nullptr, 0, &version);
            if (!record->directLightsInitialized || record->directLightVersion != version)
            {
                std::array<direct_lighting::DirectLight, direct_lighting::maxDirectLights> lights{};
                const std::uint32_t count = directLighting_->copyLights(
                    directLighting_->self, lights.data(), record->directLightLimit, &version);
                record->backendInstance->setDirectLights(lights.data(), count);
                record->directLightVersion      = version;
                record->directLightsInitialized = true;
            }
        }
        else if (!record->directLightsInitialized)
        {
            record->backendInstance->setDirectLights(nullptr, 0);
            record->directLightsInitialized = true;
        }
        record->frameStart        = std::chrono::steady_clock::now();
        record->frameTimingActive = true;
        record->backendInstance->beginFrame(frameParameters);
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
        const MeshMaterial defaultMaterial;
        record->backendInstance->drawMesh(mesh, modelMatrix, material != nullptr ? *material : defaultMaterial);
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
        const MeshMaterial defaultMaterial;
        record->backendInstance->drawMeshInstances(
            mesh, modelMatrices, instanceCount, material != nullptr ? *material : defaultMaterial);
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

    // The batched draw path: optionally sorts the commands (draw_sort_mode,
    // only once commandCount >= draw_sort_min_commands), then - on the Vulkan
    // backend only, when instance_batching is on - folds runs of equal
    // mesh+material into drawMeshInstances. Runs smaller than
    // instance_batch_min_count fall back to single draws.
    void drawMeshesImpl(RendererHandle renderer, const RenderDrawCommand *commands, std::uint32_t commandCount)
    {
        RendererRecord *record = recordFor(renderer);
        if (record == nullptr || record->backendInstance == nullptr || commands == nullptr || commandCount == 0)
        {
            return;
        }
        const MeshMaterial       defaultMaterial;
        RenderBackend           *backend         = record->backendInstance.get();
        const RenderDrawCommand *orderedCommands = commands;

        if (drawSortEnabled_ && commandCount >= drawSortMinimumCommands_
            && static_cast<std::size_t>(commandCount) <= record->sortedCommandsScratch.max_size())
        {
            try
            {
                const std::uint64_t signature = renderDrawCommandSortSignature(
                    std::span<const RenderDrawCommand>(commands, commandCount), drawSortMode_);
                if (record->sortSignatureValid && record->sortSignature == signature
                    && record->sortPermutation.size() == static_cast<std::size_t>(commandCount))
                {
                    // Same key multiset in the same input order as the cached
                    // sort: gather through the cached permutation. Matrix data
                    // is dereferenced at draw time, so only the pointers must
                    // be current - which the gather guarantees.
                    record->sortedCommandsScratch.resize(commandCount);
                    const std::vector<std::uint32_t> &permutation = record->sortPermutation;
                    for (std::uint32_t index = 0; index < commandCount; ++index)
                        record->sortedCommandsScratch[index] = commands[permutation[index]];
                }
                else
                {
                    record->sortedCommandsScratch.assign(commands, commands + commandCount);
                    sortRenderDrawCommands(record->sortedCommandsScratch, drawSortMode_);
                    // Record the permutation this sorted order came from so
                    // later frames with the same keys can skip the sort.
                    record->sortPermutation.resize(commandCount);
                    std::vector<bool> consumed(commandCount, false);
                    for (std::uint32_t sortedIndex = 0; sortedIndex < commandCount; ++sortedIndex)
                    {
                        const RenderDrawCommand &sortedCommand = record->sortedCommandsScratch[sortedIndex];
                        for (std::uint32_t originalIndex = 0; originalIndex < commandCount; ++originalIndex)
                        {
                            if (consumed[originalIndex])
                                continue;
                            const RenderDrawCommand &candidate = commands[originalIndex];
                            const bool materialsEqual
                                = candidate.material == sortedCommand.material
                                  || (candidate.material != nullptr && sortedCommand.material != nullptr
                                      && compareRenderMaterialValues(*candidate.material, *sortedCommand.material)
                                          == 0)
                                  || (candidate.material == nullptr && sortedCommand.material == nullptr);
                            if (candidate.mesh == sortedCommand.mesh && candidate.modelMatrix == sortedCommand.modelMatrix
                                && materialsEqual)
                            {
                                record->sortPermutation[sortedIndex] = originalIndex;
                                consumed[originalIndex]              = true;
                                break;
                            }
                        }
                    }
                    record->sortSignature      = signature;
                    record->sortSignatureValid = true;
                }
                orderedCommands = record->sortedCommandsScratch.data();
            }
            catch (const std::bad_alloc &)
            {
                record->sortSignatureValid = false;
            }
        }
        else
        {
            record->sortSignatureValid = false;
        }

        const bool useInstancing = instanceBatchingEnabled_ && record->backend == Backend::Vulkan
                                && commandCount >= instanceBatchMinimumCount_;
        if (!useInstancing)
        {
            for (std::uint32_t index = 0; index < commandCount; ++index)
            {
                const RenderDrawCommand &command = orderedCommands[index];
                if (command.modelMatrix == nullptr || command.mesh == invalidMesh)
                {
                    continue;
                }
                backend->drawMesh(command.mesh,
                    command.modelMatrix,
                    command.material != nullptr ? *command.material : defaultMaterial);
            }
            return;
        }

        std::uint32_t index = 0;
        while (index < commandCount)
        {
            const RenderDrawCommand &first = orderedCommands[index];
            if (first.modelMatrix == nullptr || first.mesh == invalidMesh)
            {
                ++index;
                continue;
            }

            // Same mesh + same material folds into one instanced draw; a run
            // that is too small, overflows the scratch, or fails to allocate
            // falls back to per-command drawMesh calls.
            const MeshMaterial *const material = first.material != nullptr ? first.material : &defaultMaterial;
            std::uint32_t             end      = index + 1u;
            while (end < commandCount)
            {
                const RenderDrawCommand  &next         = orderedCommands[end];
                const MeshMaterial *const nextMaterial = next.material != nullptr ? next.material : &defaultMaterial;
                if (next.modelMatrix == nullptr || next.mesh != first.mesh
                    || (next.material != first.material && compareRenderMaterialValues(*nextMaterial, *material) != 0))
                {
                    break;
                }
                ++end;
            }

            const std::size_t instanceCount = static_cast<std::size_t>(end - index);
            if (instanceCount < instanceBatchMinimumCount_)
            {
                for (std::uint32_t fallback = index; fallback < end; ++fallback)
                {
                    const RenderDrawCommand &command = orderedCommands[fallback];
                    backend->drawMesh(command.mesh, command.modelMatrix, effectiveRenderMaterial(command.material));
                }
                index = end;
                continue;
            }

            if (instanceCount > record->instanceMatricesScratch.max_size() / 16u)
            {
                for (std::uint32_t fallback = index; fallback < end; ++fallback)
                {
                    const RenderDrawCommand &command = orderedCommands[fallback];
                    backend->drawMesh(command.mesh, command.modelMatrix, *material);
                }
                index = end;
                continue;
            }

            try
            {
                record->instanceMatricesScratch.resize(instanceCount * 16u);
            }
            catch (const std::bad_alloc &)
            {
                for (std::uint32_t fallback = index; fallback < end; ++fallback)
                {
                    const RenderDrawCommand &command = orderedCommands[fallback];
                    backend->drawMesh(command.mesh, command.modelMatrix, *material);
                }
                index = end;
                continue;
            }

            float *destination = record->instanceMatricesScratch.data();
            for (std::uint32_t source = index; source < end; ++source)
            {
                std::copy_n(orderedCommands[source].modelMatrix, 16u, destination);
                destination += 16u;
            }
            backend->drawMeshInstances(first.mesh,
                record->instanceMatricesScratch.data(),
                static_cast<std::uint32_t>(instanceCount),
                *material);
            index = end;
        }
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
            std::fprintf(stderr, "render module: could not decode texture '%s': %s\n", filePath, stbi_failure_reason());
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
        const auto found = renderers_.find(renderer);
        if (found == renderers_.end() || found->second.backendInstance == nullptr)
        {
            return DeviceClass::Unknown;
        }
        return found->second.backendInstance->deviceClass();
    }

    const char *deviceNameImpl(RendererHandle renderer) const
    {
        const auto found = renderers_.find(renderer);
        if (found == renderers_.end() || found->second.backendInstance == nullptr)
        {
            return nullptr;
        }
        return found->second.backendInstance->deviceName();
    }

    bool deviceProfileImpl(RendererHandle renderer, DeviceProfile *outProfile) const
    {
        const auto found = renderers_.find(renderer);
        if (found == renderers_.end() || found->second.backendInstance == nullptr || outProfile == nullptr)
        {
            return false;
        }
        *outProfile = found->second.backendInstance->deviceProfile();
        return true;
    }

    void frameStatisticsImpl(RendererHandle renderer, FrameStatistics *outStatistics) const
    {
        const auto found = renderers_.find(renderer);
        if (found == renderers_.end() || found->second.backendInstance == nullptr || outStatistics == nullptr)
        {
            return;
        }
        *outStatistics = found->second.backendInstance->statistics();
    }

    bool vulkanInfoStringsImpl(RendererHandle renderer, RendererInfoStrings *outInfo) const
    {
#if defined(ENGINE_RENDER_BACKEND_VULKAN)
        const auto found = renderers_.find(renderer);
        if (found == renderers_.end() || found->second.backend != Backend::Vulkan)
        {
            return false;
        }
        return static_cast<VulkanBackend *>(found->second.backendInstance.get())->infoStrings(outInfo);
#else
        (void)renderer;
        (void)outInfo;
        return false;
#endif
    }

    bool vulkanNativeHandlesImpl(RendererHandle renderer, VulkanDeviceHandles *outHandles) const
    {
#if defined(ENGINE_RENDER_BACKEND_VULKAN)
        const auto found = renderers_.find(renderer);
        if (found == renderers_.end() || found->second.backend != Backend::Vulkan)
        {
            return false;
        }
        return static_cast<VulkanBackend *>(found->second.backendInstance.get())->nativeHandles(outHandles);
#else
        (void)renderer;
        (void)outHandles;
        return false;
#endif
    }

    bool vulkanDeviceProfileImpl(RendererHandle renderer, DeviceProfile *outProfile) const
    {
#if defined(ENGINE_RENDER_BACKEND_VULKAN)
        if (outProfile == nullptr)
            return false;
        const auto found = renderers_.find(renderer);
        if (found == renderers_.end() || found->second.backend != Backend::Vulkan)
            return false;
        *outProfile = static_cast<VulkanBackend *>(found->second.backendInstance.get())->deviceProfile();
        return true;
#else
        (void)renderer;
        (void)outProfile;
        return false;
#endif
    }

    bool vulkanExportFramePpmImpl(RendererHandle renderer, const char *filePath, RenderError *error)
    {
#if defined(ENGINE_RENDER_BACKEND_VULKAN)
        const auto found = renderers_.find(renderer);
        if (found == renderers_.end() || found->second.backend != Backend::Vulkan)
        {
            reportError(error, RenderError::Unsupported);
            return false;
        }
        return static_cast<VulkanBackend *>(found->second.backendInstance.get())->exportFramePpm(filePath, error);
#else
        (void)renderer;
        (void)filePath;
        reportError(error, RenderError::Unsupported);
        return false;
#endif
    }

    void headlessStatisticsImpl(RendererHandle renderer, HeadlessRenderStatistics *outStatistics) const
    {
        const auto found = renderers_.find(renderer);
        if (found == renderers_.end() || found->second.backend != Backend::Headless || outStatistics == nullptr)
        {
            return;
        }
        *outStatistics = static_cast<HeadlessBackend *>(found->second.backendInstance.get())->headlessStatistics();
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
        { static_cast<const RenderModule *>(self)->frameStatisticsImpl(renderer, outStatistics); };

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

    pbr::PbrCapability                        *pbr_            = nullptr;
    direct_lighting::DirectLightingCapability *directLighting_ = nullptr;

#if defined(ENGINE_RENDER_BACKEND_VULKAN)
    window::VulkanSurfaceCapability *vulkanSurface_ = nullptr;
#endif

    // Current settings. Keep in mind these are the built-in defaults; the
    // config file may override them at construct/reload time. anisotropy,
    // msaa and srgb have no config readers yet - see configuration.md.
    ConfiguredBackend configuredBackend_ = ConfiguredBackend::Auto;
    DevicePreference  devicePreference_  = DevicePreference::Auto;
    bool              vsync_             = true;
    bool              validation_        = false;

    std::uint32_t        maxFramesInFlight_                 = 2;
    std::uint32_t        maxPendingUploads_                = 64;
    std::uint32_t        maximumTexturesPerRenderer_       = 256;
    std::uint32_t        maximumMaterialSetsPerRenderer_   = 256;
    std::size_t          pipelineCacheMaximumBytes_        = 64u * 1024u * 1024u;
    std::uint32_t        memoryBudgetUpdateIntervalFrames_ = 120;
    std::array<float, 4> configuredClearColor_              = {0.0f, 0.0f, 0.0f, 1.0f};
    std::size_t          headlessTransformCacheBudgetBytes_ = 32u * 1024u * 1024u;
    bool                 frustumCulling_                    = true;
    bool                 backFaceCulling_                   = true;
    bool                 lodEnabled_                        = true;
    bool                 gpuTimestampsEnabled_              = true;
    bool                 drawSortEnabled_                   = false;
    std::uint32_t        drawSortMinimumCommands_           = 2;
    bool                 instanceBatchingEnabled_           = false;
    std::uint32_t        instanceBatchMinimumCount_         = 4;
    std::uint32_t        instanceStreamMegabytes_           = 4;
    bool                 textureMipmapsEnabled_             = false;
    float                anisotropyLevel_                   = 1.0f;
    std::uint32_t        msaaSamples_                       = 1u;
    bool                 srgbOutput_                        = false;

    RenderDrawSortMode drawSortMode_ = RenderDrawSortMode::MeshThenMaterial;

    // Mesh/texture ids are global across renderers and never reused.
    std::unordered_map<RendererHandle, RendererRecord> renderers_;
    RendererHandle                                     nextRendererId_ = 1;
    MeshHandle                                         nextMeshId_     = 1;
    TextureHandle                                      nextTextureId_  = 1;

    ParticipantManifest manifest_;
};

} // namespace

} // namespace render

using render::RenderModule;

} // namespace engine

// The kernel's only required export: module metadata + factory. The lists
// here must match makeManifest() and module.toml.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const providedCapabilities[] = {"render", "render.vulkan", "render.headless"};
    static const char *const requiredCapabilities[] = {"window"};
    static const char *const optionalCapabilities[] = {"window.vulkan", "pbr", "direct_lighting"};

    static const engine::EngineModuleEntryTable entry = {
        .abiVersion              = engine::engineModuleAbiVersion,
        .moduleName              = "render",
        .moduleVersion           = "0.1.0",
        .createModule            = []() -> engine::ModuleInterface            *{ return new engine::render::RenderModule(); },
        .providedCapabilities    = providedCapabilities,
        .providedCapabilityCount = 3,
        .requiredCapabilities    = requiredCapabilities,
        .requiredCapabilityCount = 1,
        .optionalCapabilities    = optionalCapabilities,
        .optionalCapabilityCount = 3,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0,
    };
    return &entry;
}
