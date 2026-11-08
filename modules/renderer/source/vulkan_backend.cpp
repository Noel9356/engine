

// Vulkan backend implementation: frame lifecycle and command recording.
//
// beginFrame waits for a frame slot, recreates the swapchain when needed,
// flushes the cached frame uniform, and opens the render pass; the draw
// functions record with bind-state caching (push constants, material set,
// vertex buffers); endFrame ends the pass, optionally copies the frame into
// the readback buffer, submits (with per-slot fence or timeline semaphore),
// presents, and rotates the slot. Mesh/texture destruction is deferred one
// frame slot so GPU work referencing the objects retires first.

#if defined(ENGINE_RENDERER_BACKEND_VULKAN)

#include "engine/direct_lighting/capabilities.hpp"
#include "engine/pbr/capabilities.hpp"
#include "engine/renderer/capabilities.hpp"
#include "engine/renderer/material.hpp"
#include "engine/renderer/mesh.hpp"
#include "engine/window/capabilities.hpp"
#include "render_backend.hpp"
#include "render_shared.hpp"

#include "vulkan_device.hpp"
#include "vulkan_frames.hpp"
#include "vulkan_pipeline.hpp"
#include "vulkan_resources.hpp"
#include "vulkan_swapchain.hpp"

#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/ext/vector_float4.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vulkan/vulkan_core.h>

#if defined(_WIN32)
#define VK_USE_PLATFORM_WIN32_KHR
#elif defined(__linux__)
#define VK_USE_PLATFORM_XLIB_KHR
#endif

#include "vulkan_backend.hpp"

#undef None

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

namespace engine
{
namespace renderer
{

namespace
{

// Post-projection matrix mapping OpenGL-convention camera matrices onto
// Vulkan clip space: flip Y and remap Z from [-1,1] to [0,1].
constexpr glm::mat4 vulkanClipFixup(glm::vec4(1.0f, 0.0f, 0.0f, 0.0f), glm::vec4(0.0f, -1.0f, 0.0f, 0.0f),
    glm::vec4(0.0f, 0.0f, 0.5f, 0.0f), glm::vec4(0.0f, 0.0f, 0.5f, 1.0f));

bool sameEnvironment(const pbr::PbrEnvironment &left, const pbr::PbrEnvironment &right)
{
    return std::memcmp(left.sun.direction, right.sun.direction, sizeof(left.sun.direction)) == 0
        && std::memcmp(left.sun.color, right.sun.color, sizeof(left.sun.color)) == 0
        && left.sun.intensity == right.sun.intensity && left.sun.enabled == right.sun.enabled
        && std::memcmp(left.ambient.skyColor, right.ambient.skyColor, sizeof(left.ambient.skyColor)) == 0
        && std::memcmp(left.ambient.groundColor, right.ambient.groundColor, sizeof(left.ambient.groundColor)) == 0
        && left.ambient.intensity == right.ambient.intensity && left.exposure == right.exposure
        && left.toneMapping == right.toneMapping;
}

// Builds the std140 frame uniform: view-projection (with the Vulkan fixup),
// camera position, sun/ambient/exposure/tone-mapper, the packed light
// array with its active count, and the lumen_gi probe-grid header.
FrameUniformData makeFrameUniform(const CameraMatrices &camera, const pbr::PbrEnvironment &environment,
    const DirectLightUniformData *packedLights, std::uint32_t lightCount, const ProbeGridUniformData &probeGrid)
{
    FrameUniformData data;
    const glm::mat4  viewProj = vulkanClipFixup * glm::make_mat4(camera.projection) * glm::make_mat4(camera.view);
    std::memcpy(data.viewProj, glm::value_ptr(viewProj), sizeof(data.viewProj));

    data.cameraPos[0] = camera.position[0];
    data.cameraPos[1] = camera.position[1];
    data.cameraPos[2] = camera.position[2];
    data.cameraPos[3] = 1.0f;

    data.sunDirection[0] = environment.sun.direction[0];
    data.sunDirection[1] = environment.sun.direction[1];
    data.sunDirection[2] = environment.sun.direction[2];
    data.sunDirection[3] = environment.sun.enabled ? 1.0f : 0.0f;

    data.sunColor[0] = environment.sun.color[0];
    data.sunColor[1] = environment.sun.color[1];
    data.sunColor[2] = environment.sun.color[2];
    data.sunColor[3] = environment.sun.intensity;

    data.ambientSky[0] = environment.ambient.skyColor[0];
    data.ambientSky[1] = environment.ambient.skyColor[1];
    data.ambientSky[2] = environment.ambient.skyColor[2];
    data.ambientSky[3] = environment.ambient.intensity;

    data.ambientGround[0] = environment.ambient.groundColor[0];
    data.ambientGround[1] = environment.ambient.groundColor[1];
    data.ambientGround[2] = environment.ambient.groundColor[2];
    data.ambientGround[3] = 0.0f;

    data.toneMapping[0] = environment.exposure;
    data.toneMapping[1] = static_cast<float>(environment.toneMapping);
    data.toneMapping[2] = 0.0f;
    data.toneMapping[3] = 0.0f;

    const std::uint32_t packedLightCount = std::min(lightCount, direct_lighting::maxDirectLights);
    data.directLightMeta[0]              = static_cast<float>(packedLightCount);
    if (packedLightCount != 0)
    {
        std::copy_n(packedLights, packedLightCount, data.directLights);
    }
    data.probeGrid = probeGrid;
    return data;
}

} // namespace

// Factory: device -> MSAA sample count (requested value clamped to what the
// framebuffer supports, at most 8x) -> swapchain formats -> pipelines ->
// frame scheduler -> instance streams -> resources -> swapchain objects.
// Returns null with *error set on any failure.
std::unique_ptr<RenderBackend> VulkanBackend::create(window::WindowHandle surface, bool vsync, bool validation,
    std::uint32_t framesInFlight, std::uint32_t maxPendingUploads, std::uint32_t memoryBudgetUpdateIntervalFrames,
    DevicePreference devicePreference, bool frustumCullingEnabled, bool backFaceCullingEnabled,
    bool gpuTimestampsEnabled, std::uint32_t instanceStreamMegabytes, bool textureMipmapsEnabled, float anisotropyLevel,
    std::uint32_t msaaSamples, bool srgbOutput, std::uint32_t maximumTextures, std::uint32_t maximumMaterialSets,
    std::size_t pipelineCacheMaxBytes, window::VulkanSurfaceCapability *vulkanSurface, RenderError *error)
{
    auto backend                               = std::unique_ptr<VulkanBackend>(new VulkanBackend());
    backend->surfaceHandle_                    = surface;
    backend->vsync_                            = vsync;
    backend->framesInFlight_                   = framesInFlight != 0 ? framesInFlight : defaultMaxFramesInFlight;
    backend->memoryBudgetUpdateIntervalFrames_ = memoryBudgetUpdateIntervalFrames;
    backend->frustumCullingEnabled_            = frustumCullingEnabled;
    backend->srgbOutput_                       = srgbOutput;
    backend->anisotropyLevel_                  = anisotropyLevel;

    backend->msaaSamples_                    = VK_SAMPLE_COUNT_1_BIT;
    const std::uint32_t requestedMsaaSamples = msaaSamples;

    backend->dynamicRenderingEnabled_ = false;
    // Instance stream capacity: clamped 1..64 MiB, halved on allocation
    // failure inside createInstanceBuffers.
    const std::uint32_t boundedInstanceStreamMb = std::clamp(instanceStreamMegabytes, 1u, 64u);
    backend->instanceBufferCapacity_ = static_cast<VkDeviceSize>(boundedInstanceStreamMb) * 1024ull * 1024ull;

    backend->device_
        = VulkanDeviceContext::create(surface, validation, devicePreference, maxPendingUploads, vulkanSurface, error);
    if (backend->device_ == nullptr)
    {
        return nullptr;
    }

    // Surface-format selection needs the sRGB preference up front: it decides
    // whether an _SRGB format is picked, and the pipeline's SRGB_OUTPUT
    // specialization constant must match the *actually picked* format (see
    // formatIsSrgb below), not merely the requested one.
    backend->swapchain_ = std::make_unique<VulkanSwapchain>(*backend->device_, backend->device_->surface(), vsync);
    backend->swapchain_->setSrgbOutput(backend->srgbOutput_);
    if (!backend->swapchain_->pickFormats())
    {
        std::fprintf(stderr, "renderer module: no supported surface/depth format\n");
        reportError(error, RenderError::CreationFailed);
        return nullptr;
    }

    // Effective MSAA decision. The resolve path (MSAA color attachment with
    // DONT_CARE store + RESOLVE_MODE_AVERAGE) is implemented on the
    // dynamic-rendering path only, so MSAA > 1 requires that capability, and
    // the picked color/depth formats must support the sample count. Anything
    // else falls back to 1 sample with a one-time note - the legacy
    // render-pass path is unchanged when msaa_samples is at its default.
    {
        const std::uint32_t wanted = std::clamp(requestedMsaaSamples, 1u, 8u);
        if (wanted > 1u)
        {
            if (!backend->device_->profile().dynamicRenderingAvailable)
            {
                std::fprintf(stderr,
                    "renderer module: msaa %u requested but dynamic rendering is unavailable; running at 1 sample\n",
                    wanted);
            }
            else
            {
                VkSampleCountFlags supported = 0;
                VkImageFormatProperties    colorProperties{};
                VkImageFormatProperties    depthProperties{};
                if (vkGetPhysicalDeviceImageFormatProperties(backend->device_->physicalDevice(),
                        backend->swapchain_->format(),
                        VK_IMAGE_TYPE_2D,
                        VK_IMAGE_TILING_OPTIMAL,
                        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
                        0,
                        &colorProperties)
                        == VK_SUCCESS
                    && vkGetPhysicalDeviceImageFormatProperties(backend->device_->physicalDevice(),
                           backend->swapchain_->depthFormat(),
                           VK_IMAGE_TYPE_2D,
                           VK_IMAGE_TILING_OPTIMAL,
                           VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
                           0,
                           &depthProperties)
                           == VK_SUCCESS)
                {
                    supported = colorProperties.sampleCounts & depthProperties.sampleCounts;
                }
                VkPhysicalDeviceProperties deviceProperties{};
                vkGetPhysicalDeviceProperties(backend->device_->physicalDevice(), &deviceProperties);
                supported &= deviceProperties.limits.framebufferColorSampleCounts
                           & deviceProperties.limits.framebufferDepthSampleCounts;

                if (wanted >= 8u && (supported & VK_SAMPLE_COUNT_8_BIT) != 0u)
                    backend->msaaSamples_ = VK_SAMPLE_COUNT_8_BIT;
                else if (wanted >= 4u && (supported & VK_SAMPLE_COUNT_4_BIT) != 0u)
                    backend->msaaSamples_ = VK_SAMPLE_COUNT_4_BIT;
                else if (wanted >= 2u && (supported & VK_SAMPLE_COUNT_2_BIT) != 0u)
                    backend->msaaSamples_ = VK_SAMPLE_COUNT_2_BIT;

                if (backend->msaaSamples_ == VK_SAMPLE_COUNT_1_BIT)
                {
                    std::fprintf(stderr,
                        "renderer module: msaa %u requested but the picked color/depth formats do not support it; "
                        "running at 1 sample\n",
                        wanted);
                }
                else
                {
                    std::fprintf(stderr,
                        "renderer module: MSAA %ux enabled on the dynamic-rendering path\n",
                        static_cast<unsigned>(backend->msaaSamples_));
                }
            }
        }
    }

    backend->pipeline_ = std::make_unique<VulkanPipeline>();
    backend->pipeline_->setCullBackFaces(backFaceCullingEnabled);
    backend->pipeline_->setPipelineCacheMaxBytes(pipelineCacheMaxBytes);
    // The pipeline must be built with the exact state beginFrame renders
    // with: sample count for rasterization/resolve, dynamic rendering when
    // MSAA needs the inline resolve, and the SRGB_OUTPUT specialization
    // constant matching the actually picked surface format.
    backend->pipeline_->setSampleCount(backend->msaaSamples_);
    backend->pipeline_->setDynamicRenderingEnabled(backend->msaaSamples_ != VK_SAMPLE_COUNT_1_BIT);
    backend->pipeline_->setSrgbOutput(backend->swapchain_->formatIsSrgb());
    if (!backend->pipeline_->build(
            *backend->device_, backend->swapchain_->format(), backend->swapchain_->depthFormat()))
    {
        reportError(error, RenderError::CreationFailed);
        return nullptr;
    }

    backend->frames_ = std::make_unique<VulkanFrameScheduler>();
    if (!backend->frames_->create(*backend->device_, backend->framesInFlight_, gpuTimestampsEnabled))
    {
        reportError(error, RenderError::CreationFailed);
        return nullptr;
    }
    backend->overlayBuffers_.resize(backend->framesInFlight_);

    backend->deferredDestroys_.resize(backend->framesInFlight_);
    if (!backend->createInstanceBuffers())
    {
        reportError(error, RenderError::CreationFailed);
        return nullptr;
    }
    backend->resources_ = std::make_unique<VulkanResources>();
    if (!backend->resources_->create(*backend->device_,
            backend->framesInFlight_,
            backend->pipeline_->frameSetLayout(),
            backend->pipeline_->materialSetLayout(),
            textureMipmapsEnabled,
            backend->anisotropyLevel_,
            maximumTextures,
            maximumMaterialSets))
    {
        reportError(error, RenderError::CreationFailed);
        return nullptr;
    }

    // srgbOutput_ was already handed to the swapchain before pickFormats();
    // only the sample count remains to be applied before creating objects.
    backend->swapchain_->setSampleCount(backend->msaaSamples_);
    if (!backend->swapchain_->createObjects(
            backend->pipeline_->renderPass(), backend->viewportWidth_, backend->viewportHeight_, backend->framesInFlight_))
    {
        std::fprintf(stderr, "renderer module: could not create the swapchain\n");
        reportError(error, RenderError::CreationFailed);
        return nullptr;
    }

    // Everything above is object setup; nothing has been submitted yet. One
    // warm-up frame here absorbs the driver's deferred pipeline compilation
    // (see warmUpPipelines) while the application still pumps the window.
    backend->warmUpPipelines();

    reportError(error, RenderError::None);
    return backend;
}

// Submits one minimal real frame (a 3-vertex triangle through the mesh
// pipeline) at the end of create(), then waits for it to retire. Why this
// exists: drivers may defer graphics-pipeline shader compilation to
// background threads and block the first submission that uses a pipeline
// (NVIDIA does this). Without the warm-up, that block lands in the demo's
// first frame on the MAIN thread: the message pump starves, the OS flags the
// window "not responding", and the driver's compile threads keep every core
// busy (loud fans) - users close a window that is actually still working, and
// because the driver's shader disk cache is only written after compilation
// completes, every fresh start pays the full cost again. Here the application
// is still pumping the window around createRenderer, so the identical cost is
// invisible and the cache persists before the frame loop takes over. Best
// effort: any failure logs and continues; the frame loop then behaves exactly
// as it would have without the warm-up.
void VulkanBackend::warmUpPipelines()
{
    if (device_ == nullptr || device_->device() == VK_NULL_HANDLE || resources_ == nullptr || frames_ == nullptr
        || pipeline_ == nullptr || swapchain_ == nullptr)
    {
        return;
    }

    std::fprintf(stderr,
        "renderer module: submitting one warm-up frame so the driver finishes pipeline compilation "
        "before the frame loop starts...\n");

    // Mesh ids belong to the module wrapper, which is not involved here; pick
    // any id resources_ has not interned and use it only inside this call.
    MeshHandle warmUpMesh = 1;
    while (warmUpMesh != invalidMesh && resources_->meshFor(warmUpMesh) != nullptr)
    {
        ++warmUpMesh;
    }
    if (warmUpMesh == invalidMesh)
    {
        std::fprintf(stderr, "renderer module: warm-up frame skipped (no free mesh handle)\n");
        return;
    }

    // A 3-vertex triangle in the fixed 11-float layout (position, normal,
    // color, uv). With the default (zero) camera the view-projection matrix is
    // all zeros, so the triangle collapses to a degenerate clip position and
    // nothing is visible; the draw still forces real driver compile + submit
    // work for the pipeline, which is the entire point.
    static constexpr float warmUpVertices[3 * 11] = {
        -0.05f, -0.05f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f,
         0.05f, -0.05f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f,
         0.00f,  0.05f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.5f, 1.0f,
    };
    static constexpr std::uint32_t warmUpIndices[3] = {0u, 1u, 2u};
    const MeshDescription warmUpDescription{
        .vertices    = warmUpVertices,
        .vertexCount = 3u,
        .indices     = warmUpIndices,
        .indexCount  = 3u};
    RenderError warmUpError = RenderError::None;
    if (!createMesh(warmUpMesh, warmUpDescription, &warmUpError))
    {
        std::fprintf(stderr,
            "renderer module: warm-up frame skipped (mesh creation failed, error %u)\n",
            static_cast<std::uint32_t>(warmUpError));
        return;
    }
    // The draw below must not read the vertex buffer before its copy retires;
    // the fence wait is the host-side dependency between the two submissions.
    device_->waitForUploads();

    const std::uint64_t framesBeforeWarmUp = statistics_.framesRendered;
    const auto          warmUpStart        = std::chrono::steady_clock::now();
    // acquire can time out (minimized/occluded window) or the swapchain can
    // need a recreate on the first frames; retry until one frame actually
    // submits instead of giving up after a single attempt.
    constexpr std::uint32_t maximumWarmUpAttempts = 8u;
    // A tiny UI quad through the overlay pipeline, drawn in the same frame:
    // the demo's first real frame renders the menu through that pipeline, and
    // each pipeline's driver-side compilation only happens at the first
    // submission that uses it, so both pipelines must submit here or the
    // unwarmed one stalls the first real frame on the main thread.
    static constexpr OverlayVertex warmUpOverlayVertices[4] = {
        {{8.0f, 8.0f}, {0.0f, 0.0f}, 0xFFFFFFFFu},
        {{28.0f, 8.0f}, {1.0f, 0.0f}, 0xFFFFFFFFu},
        {{28.0f, 28.0f}, {1.0f, 1.0f}, 0xFFFFFFFFu},
        {{8.0f, 28.0f}, {0.0f, 1.0f}, 0xFFFFFFFFu}};
    static constexpr std::uint32_t warmUpOverlayIndices[6] = {0u, 1u, 2u, 0u, 2u, 3u};
    const OverlayDrawCommand       warmUpOverlayCommand{.indexOffset     = 0u,
        .vertexOffset                                    = 0u,
        .elementCount                                    = 6u,
        .clipRectangle                                   = {0.0f, 0.0f, 320.0f, 240.0f},
        .texture                                         = invalidTexture,
        .resetRenderState                                = false};
    const OverlayDrawData          warmUpOverlayData{.vertices          = warmUpOverlayVertices,
        .vertexCount                              = 4u,
        .indices                                  = warmUpOverlayIndices,
        .indexCount                               = 6u,
        .commands                                 = &warmUpOverlayCommand,
        .commandCount                             = 1u,
        .displayPosition                          = {0.0f, 0.0f},
        .displaySize                              = {320.0f, 240.0f},
        .framebufferScale                         = {1.0f, 1.0f}};
    for (std::uint32_t attempt = 0; attempt < maximumWarmUpAttempts && statistics_.framesRendered == framesBeforeWarmUp;
        ++attempt)
    {
        const FrameParameters warmUpFrame{};
        beginFrame(warmUpFrame);
        if (frameActive_)
        {
            static constexpr float identityModel[16]
                = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
            const MeshMaterial warmUpMaterial{}; // every texture invalid -> neutral stand-ins
            drawMesh(warmUpMesh, identityModel, warmUpMaterial);
            drawOverlay(warmUpOverlayData);
            endFrame();
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    if (statistics_.framesRendered == framesBeforeWarmUp)
    {
        std::fprintf(stderr,
            "renderer module: warm-up frame could not acquire a swapchain image; the first rendered frame "
            "will pay the driver's pipeline compilation instead\n");
    }
    else
    {
        // Wait (bounded) until the warm-up submission retires: this is where
        // a deferred driver compilation completes. waitForFrameSlot caps each
        // wait at 1s and logs its own stall notices; the deadline bounds the
        // total so a wedged driver cannot hang renderer creation forever.
        // The explicit progress line below exists because a cold driver shader
        // cache turns this wait into a 30-120s, all-core, loud-fan compile:
        // without a named message users read that as a hang and kill the
        // process mid-compile, which throws the work away and guarantees the
        // next launch pays the full cost again.
        const auto warmUpDeadline       = std::chrono::steady_clock::now() + std::chrono::seconds(120);
        auto       lastCompileNotice    = warmUpStart;
        constexpr auto compileNoticeGap = std::chrono::seconds(10);
        while (!frames_->waitForFrameSlot())
        {
            const auto now = std::chrono::steady_clock::now();
            if (now >= warmUpDeadline)
            {
                std::fprintf(stderr,
                    "renderer module: warm-up frame did not retire within 120s; continuing (the frame loop "
                    "will skip frames until the GPU catches up)\n");
                break;
            }
            if (now - lastCompileNotice >= compileNoticeGap)
            {
                const double waited
                    = std::chrono::duration_cast<std::chrono::duration<double>>(now - warmUpStart).count();
                std::fprintf(stderr,
                    "renderer module: first submission still retiring after %.0f s - the driver is compiling "
                    "shader pipelines. This happens only once per driver update; the compiled pipelines are "
                    "saved to disk the moment it finishes, so later launches start fast even if you close "
                    "the demo right after. Loud fans during the compile are expected - please let it finish\n",
                    waited);
                std::fflush(stderr);
                lastCompileNotice = now;
            }
        }
    }

    // Persist the compiled pipelines the moment the warm-up settles: this is
    // the first point at which the driver's compile work is definitely done.
    // Without this flush the cache only reached disk at clean shutdown, so a
    // user closing a slow first launch discarded the entire compile and every
    // later launch paid it again - the exact "slow and loud every time" loop.
    if (device_ != nullptr && device_->device() != VK_NULL_HANDLE && pipeline_ != nullptr)
    {
        (void)pipeline_->flushPipelineCacheToDisk(device_->device());
    }

    const auto warmUpMilliseconds
        = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - warmUpStart)
              .count();
    std::fprintf(stderr,
        "renderer module: warm-up frame done in %.1f s\n",
        static_cast<double>(warmUpMilliseconds) / 1000.0);

    // The warm-up submission has retired (or we gave up waiting with a log),
    // so nothing references the 3-vertex buffers anymore; resources_ destroys
    // them immediately and the wait above is what makes that safe.
    resources_->destroyMesh(warmUpMesh);
}

// Tear-down order matters: wait idle, drop per-frame scratch, then
// swapchain -> resources -> frames -> pipeline -> device.
VulkanBackend::~VulkanBackend()
{
    waitForDeviceQuiet();
    // Complete a deferred export before teardown destroys the readback
    // buffer: waitForDeviceQuiet above guarantees the copy finished, so no
    // wait (and no screenshot file loss when the app quits right after the
    // request) - see flushPendingExport.
    flushPendingExport(false);
    destroyOverlayBuffers();
    destroyInstanceBuffers();
    if (swapchain_ != nullptr)
    {
        swapchain_->destroyObjects();
        swapchain_.reset();
    }
    if (resources_ != nullptr)
    {
        resources_->destroy();
        resources_.reset();
    }
    if (frames_ != nullptr)
    {
        frames_->destroy();
        frames_.reset();
    }
    if (pipeline_ != nullptr)
    {
        pipeline_->destroy(device_ != nullptr ? device_->device() : VK_NULL_HANDLE);
        pipeline_.reset();
    }
    if (device_ != nullptr)
    {
        device_->destroy();
        device_.reset();
    }
}

void VulkanBackend::waitForDeviceQuiet()
{
    if (device_ == nullptr || device_->device() == VK_NULL_HANDLE)
    {
        return;
    }
    {
        std::lock_guard queueLock(device_->queueMutex());
        vkDeviceWaitIdle(device_->device());
    }
    if (frames_ != nullptr)
    {
        frames_->waitForAll();
    }
    device_->collectCompletedUploads();
}

void VulkanBackend::setViewport(std::uint32_t width, std::uint32_t height)
{
    viewportWidth_  = width;
    viewportHeight_ = height;
}

void VulkanBackend::setCamera(const CameraMatrices &newCamera)
{
    if (cameraInitialized_ && std::memcmp(&camera_, &newCamera, sizeof(CameraMatrices)) == 0)
    {
        return;
    }
    camera_            = newCamera;
    cameraInitialized_ = true;
    frameUniformDirty_ = true;

    const glm::mat4 viewProjection = glm::make_mat4(newCamera.projection) * glm::make_mat4(newCamera.view);
    const glm::vec4 row0(viewProjection[0][0], viewProjection[1][0], viewProjection[2][0], viewProjection[3][0]);
    const glm::vec4 row1(viewProjection[0][1], viewProjection[1][1], viewProjection[2][1], viewProjection[3][1]);
    const glm::vec4 row3(viewProjection[0][3], viewProjection[1][3], viewProjection[2][3], viewProjection[3][3]);
    frustumPlanes_ = {row3 + row0, row3 - row0, row3 + row1, row3 - row1};
    for (glm::vec4 &plane : frustumPlanes_)
    {
        const float length = glm::length(glm::vec3(plane));
        if (std::isfinite(length) && length > 1.0e-8f)
        {
            plane /= length;
        }
        else
        {
            plane = glm::vec4(0.0f);
        }
    }
}

void VulkanBackend::setEnvironment(const pbr::PbrEnvironment &environment)
{
    if (sameEnvironment(environment_, environment))
    {
        return;
    }
    environment_       = environment;
    frameUniformDirty_ = true;
}

void VulkanBackend::setDirectLights(const direct_lighting::DirectLight *lights, std::uint32_t count)
{
    count = std::min(count, direct_lighting::maxDirectLights);
    if (count != 0 && lights == nullptr)
    {
        return;
    }
    for (std::uint32_t index = 0; index < count; ++index)
    {
        const direct_lighting::DirectLightDescription &source = lights[index].description;
        DirectLightUniformData                        &target = packedDirectLights_[index];
        target.positionType[0]                                = source.position[0];
        target.positionType[1]                                = source.position[1];
        target.positionType[2]                                = source.position[2];
        target.positionType[3]                                = static_cast<float>(source.type);
        target.directionRange[0]                              = source.direction[0];
        target.directionRange[1]                              = source.direction[1];
        target.directionRange[2]                              = source.direction[2];
        target.directionRange[3]                              = source.range;
        target.colorIntensity[0]                              = source.color[0];
        target.colorIntensity[1]                              = source.color[1];
        target.colorIntensity[2]                              = source.color[2];
        target.colorIntensity[3]                              = source.intensity;
        if (source.type == direct_lighting::DirectLightType::Spot)
        {
            target.spotParameters[0] = std::cos(source.innerConeRadians);
            target.spotParameters[1] = std::cos(source.outerConeRadians);
        }
        else
        {
            target.spotParameters[0] = 1.0f;
            target.spotParameters[1] = -1.0f;
        }
        target.spotParameters[2] = 0.0f;
        target.spotParameters[3] = 0.0f;
    }
    directLightCount_  = count;
    frameUniformDirty_ = true;
}

// Packs the borrowed lumen_gi snapshot into the std140 frame-uniform header
// and the std430 probe-record scratch (8 vec4 per probe: position.xyz +
// validity, then the 27 SH2 floats plus one pad - the exact layout the
// ProbeData block in shaders/mesh.frag walks). The per-slot probe storage
// buffers are refreshed lazily at their next beginFrame.
void VulkanBackend::setProbeGrid(const GiProbeGrid &grid)
{
    packedProbeGrid_       = ProbeGridUniformData{};
    packedProbeRecords_.clear();
    if (grid.valid())
    {
        const lumen_gi::GiProbeHeader &header = grid.header;
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            packedProbeGrid_.centerSpacing[axis] = header.gridCenter[axis];
            packedProbeGrid_.level0Meta[axis]    = static_cast<float>(header.level0Dimensions[axis]);
            packedProbeGrid_.level1Meta[axis]    = static_cast<float>(header.level1Dimensions[axis]);
        }
        packedProbeGrid_.centerSpacing[3] = header.level0Spacing;
        packedProbeGrid_.level0Meta[3]    = header.level1Spacing;
        packedProbeGrid_.level1Meta[3]    = 1.0f; // grid active: probes are bound below
        packedProbeGrid_.countMeta[0]     = static_cast<float>(grid.probeCount);
        packedProbeGrid_.countMeta[1]     = static_cast<float>(header.directionResolution);
        packedProbeGrid_.countMeta[2]     = static_cast<float>(header.level0ProbeOffset);
        packedProbeGrid_.countMeta[3]     = static_cast<float>(header.level1ProbeOffset);

        packedProbeRecords_.reserve(static_cast<std::size_t>(grid.probeCount) * giProbeRecordVec4s);
        for (std::uint32_t index = 0; index < grid.probeCount; ++index)
        {
            const lumen_gi::GiProbe &probe = grid.probes[index];
            packedProbeRecords_.push_back(probe.position[0]);
            packedProbeRecords_.push_back(probe.position[1]);
            packedProbeRecords_.push_back(probe.position[2]);
            packedProbeRecords_.push_back(probe.validity);
            for (std::uint32_t coefficient = 0; coefficient < 27; ++coefficient)
            {
                packedProbeRecords_.push_back(probe.sh2[coefficient / 3][coefficient % 3]);
            }
            packedProbeRecords_.push_back(0.0f); // pad to 8 vec4
        }
    }
    ++probeGridRevision_;
    probeGridDirty_    = true;
    frameUniformDirty_ = true; // the header rides the frame uniform
}

// Rebuilds the swapchain when it is dirty or the requested viewport no
// longer matches. A failed recreate leaves frameActive_ false, so the
// draw/endFrame calls become no-ops for this frame.
void VulkanBackend::beginFrame(const FrameParameters &frame)
{
    (void)frame;
    device_->beginFrame(memoryBudgetUpdateIntervalFrames_);
    device_->collectCompletedUploads();
    if (swapchain_->dirty()
        || (viewportWidth_ != 0 && viewportHeight_ != 0
            && (viewportWidth_ != swapchain_->extent().width || viewportHeight_ != swapchain_->extent().height)))
    {
        // Recreation destroys the readback buffer; complete any pending
        // export first. The flush waits the recording slot's fence, which
        // is precisely "the copy is done" (recreate's device idle would
        // also cover it, but the flush must run before the buffer dies).
        flushPendingExport(true);
        if (!swapchain_->recreate(pipeline_->renderPass(), viewportWidth_, viewportHeight_, framesInFlight_))
        {
            return;
        }
    }

    // FrameSlotWait covers the slot-retirement call (timeline/fence wait +
    // timestamp harvest + command-pool reset): when the CPU runs ahead of
    // the GPU by more than frames-in-flight, the stall shows up here, and
    // the histogram separates that from slow rendering.
    {
        const ScopedProfileTimer slotWaitTimer(profiler_, PerformanceMetric::FrameSlotWait);
        const bool               slotReady = frames_->waitForFrameSlot();
        if (!slotReady)
            return;
    }

    drainDeferredDestroys(frames_->currentFrame());

    // Deferred export completion (H2): the copy recorded on this slot is
    // provably complete after the slot-retirement wait above, so the file
    // write happens here without any additional GPU wait - the frame path
    // never pays for the readback.
    completePendingExportIfReady();
    if (frames_->currentFrame() < instanceBuffers_.size())
    {
        instanceBuffers_[frames_->currentFrame()].cursor = sizeof(float) * 16u;
    }

    if (frameUniformDirty_)
    {
        cachedFrameUniform_ = makeFrameUniform(
            camera_, environment_, packedDirectLights_.data(), directLightCount_, packedProbeGrid_);
        frameUniformDirty_  = false;
        ++frameUniformRevision_;
    }
    if (!resources_->updateFrameUniform(frames_->currentFrame(), cachedFrameUniform_, frameUniformRevision_))
    {
        frameUniformDirty_ = true;
        std::fprintf(stderr, "renderer module: could not flush frame uniform memory\n");
        return;
    }
    if (probeGridDirty_)
    {
        // The recording slot is refreshed eagerly; the other slots catch up
        // on their next beginFrame (their stored revision is older).
        const VkDeviceSize recordBytes
            = static_cast<VkDeviceSize>(packedProbeRecords_.size()) * sizeof(float);
        if (!resources_->updateProbeStorage(
                frames_->currentFrame(), packedProbeRecords_.data(), recordBytes, probeGridRevision_))
        {
            std::fprintf(stderr, "renderer module: could not flush the probe storage buffer\n");
            return;
        }
        probeGridDirty_ = false;
    }

    // Acquire FIRST, before any command-buffer recording: every skipped-frame
    // path below (out-of-date swapchain, bounded-acquire timeout, acquire
    // error) returns without touching recording state, so a skipped frame can
    // never abandon a command buffer left in the recording state.
    std::uint32_t imageIndex    = 0;
    VkResult      acquireResult = VK_SUCCESS;
    {
        // Acquire is a blocking point of its own: the presentation engine may
        // have no free image yet (queue depth / pacing), which is not GPU
        // execution time.
        const ScopedProfileTimer acquireTimer(profiler_, PerformanceMetric::SwapchainAcquire);
        acquireResult = swapchain_->acquire(frames_->imageAvailableSemaphore(), &imageIndex);
    }
    if (acquireResult == VK_ERROR_OUT_OF_DATE_KHR)
    {
        if (!swapchain_->recreate(pipeline_->renderPass(), viewportWidth_, viewportHeight_, framesInFlight_))
        {
            return;
        }
        return;
    }
    if (acquireResult == VK_TIMEOUT)
    {
        // No free swapchain image within the bounded acquire wait (typically
        // a minimized or fully occluded window). Skip the frame: frameActive_
        // stays false so endFrame() no-ops, and the main loop keeps polling
        // window events instead of blocking here, so the OS never sees a hung
        // window. The acquire semaphore is left unsignaled on timeout, so
        // reusing it on the next acquire is safe.
        return;
    }
    if (acquireResult != VK_SUCCESS && acquireResult != VK_SUBOPTIMAL_KHR)
    {
        std::fprintf(stderr, "renderer module: vkAcquireNextImageKHR failed (%d)\n", (int)acquireResult);
        return;
    }
    if (acquireResult == VK_SUBOPTIMAL_KHR)
    {
        swapchain_->markDirty();
    }

    // Fallback path (devices without timeline semaphores): the swapchain
    // image may still be rendering from an older frame; wait for its fence
    // before reusing it. The wait is bounded and runs BEFORE the command
    // buffer is begun, so a timeout can skip the frame without abandoning a
    // command buffer left in the recording state. Recorded so an
    // image-still-in-flight stall is attributable (no-timeline devices only).
    const bool useTimeline = device_->timelineSemaphore() != VK_NULL_HANDLE;
    if (!useTimeline && swapchain_->imagesInFlight()[imageIndex] != VK_NULL_HANDLE)
    {
        constexpr std::uint64_t imageFenceTimeoutNanoseconds = 1'000'000'000ULL; // 1s
        const ScopedProfileTimer imageFenceTimer(profiler_, PerformanceMetric::ImageFenceWait);
        if (vkWaitForFences(device_->device(),
                1,
                &swapchain_->imagesInFlight()[imageIndex],
                VK_TRUE,
                imageFenceTimeoutNanoseconds)
            != VK_SUCCESS)
        {
            static std::chrono::steady_clock::time_point lastImageStallLog{};
            const auto                                   now = std::chrono::steady_clock::now();
            const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastImageStallLog).count();
            if (lastImageStallLog.time_since_epoch().count() == 0 || elapsed >= 5000)
            {
                std::fprintf(stderr,
                    "renderer module: swapchain image %u is still in flight after 1.0s (the GPU or driver is "
                    "slow or overloaded); skipping frames until it completes\n",
                    imageIndex);
                lastImageStallLog = now;
            }
            return; // frameActive_ stays false: endFrame() no-ops this frame
        }
    }
    swapchain_->setImagesInFlight(imageIndex, useTimeline ? VK_NULL_HANDLE : frames_->inFlightFence());
    frames_->resetFrameSlotFence();

    VkCommandBuffer                command = frames_->commandBuffer();
    const VkCommandBufferBeginInfo beginInfo{
        .sType            = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
        .pNext            = nullptr,
        .flags            = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT,
        .pInheritanceInfo = nullptr,
    };
    if (vkBeginCommandBuffer(command, &beginInfo) != VK_SUCCESS)
    {
        std::fprintf(
            stderr, "renderer module: could not begin command buffer for frame slot %u\n", frames_->currentFrame());
        return;
    }
    frames_->beginGpuTiming();

    currentImageIndex_ = imageIndex;
    frameActive_       = true;

    hasLastPushConstants_       = false;
    lastBoundMesh_              = invalidMesh;
    lastBoundMaterialSet_       = VK_NULL_HANDLE;
    lastBoundInstanceOffset_    = 0;
    hasLastBoundInstanceBuffer_ = false;

    std::uint32_t width  = viewportWidth_ != 0 ? viewportWidth_ : swapchain_->extent().width;
    std::uint32_t height = viewportHeight_ != 0 ? viewportHeight_ : swapchain_->extent().height;
    width                = std::min(width, swapchain_->extent().width);
    height               = std::min(height, swapchain_->extent().height);

    VkClearValue clearValues[2];
    std::memcpy(clearValues[0].color.float32, frame.clearColor, sizeof(frame.clearColor));
    clearValues[1].depthStencil = {1.0f, 0};
    const VkRenderPassBeginInfo renderPassBegin{
        .sType           = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO,
        .pNext           = nullptr,
        .renderPass      = pipeline_->renderPass(),
        .framebuffer     = swapchain_->framebuffer(frames_->currentFrame(), imageIndex),
        .renderArea      = {{0, 0}, {swapchain_->extent().width, swapchain_->extent().height}},
        .clearValueCount = 2,
        .pClearValues    = clearValues,
    };

    if (pipeline_->dynamicRenderingEnabled())
    {
        // Dynamic-rendering path: the render pass's initialLayout/external
        // dependency does not exist here, so the acquired image (PRESENT_SRC
        // after a present, or its initial UNDEFINED state) must be
        // transitioned explicitly. oldLayout=UNDEFINED is always valid and
        // discards the previous contents, which the CLEAR load op discards
        // anyway. The barrier sits at COLOR_ATTACHMENT_OUTPUT on both sides
        // so it is ordered after the acquire semaphore's wait stage in the
        // submit (the presentation engine may still be reading until then).
        const VkImageMemoryBarrier fromPresent{
            .sType               = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
            .pNext               = nullptr,
            .srcAccessMask       = 0,
            .dstAccessMask       = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
            .oldLayout           = VK_IMAGE_LAYOUT_UNDEFINED,
            .newLayout           = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .image               = swapchain_->images()[currentImageIndex_],
            .subresourceRange    = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
        };
        vkCmdPipelineBarrier(command,
            VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
            0,
            0,
            nullptr,
            0,
            nullptr,
            1,
            &fromPresent);

        // vkCmdBeginRendering path (Vulkan 1.3 / dynamic rendering extension):
        // attachments declared inline; MSAA resolves in the color attachment.
        VkRenderingAttachmentInfo colorAtt{
            .sType              = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO,
            .pNext              = nullptr,
            .imageView          = swapchain_->colorView(frames_->currentFrame(), imageIndex),
            .imageLayout        = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            .resolveMode        = VK_RESOLVE_MODE_NONE,
            .resolveImageView   = VK_NULL_HANDLE,
            .resolveImageLayout = VK_IMAGE_LAYOUT_UNDEFINED,
            .loadOp             = VK_ATTACHMENT_LOAD_OP_CLEAR,
            .storeOp            = VK_ATTACHMENT_STORE_OP_STORE,
            .clearValue         = {},
        };
        std::memcpy(colorAtt.clearValue.color.float32, frame.clearColor, sizeof(frame.clearColor));
        if (pipeline_->sampleCount() != VK_SAMPLE_COUNT_1_BIT)
        {
            colorAtt.resolveMode        = VK_RESOLVE_MODE_AVERAGE_BIT;
            colorAtt.resolveImageView   = swapchain_->resolveView(imageIndex);
            colorAtt.resolveImageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            colorAtt.storeOp            = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        }
        VkRenderingAttachmentInfo depthAtt{
            .sType              = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO,
            .pNext              = nullptr,
            .imageView          = swapchain_->depthView(frames_->currentFrame()),
            .imageLayout        = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
            .resolveMode        = VK_RESOLVE_MODE_NONE,
            .resolveImageView   = VK_NULL_HANDLE,
            .resolveImageLayout = VK_IMAGE_LAYOUT_UNDEFINED,
            .loadOp             = VK_ATTACHMENT_LOAD_OP_CLEAR,
            .storeOp            = VK_ATTACHMENT_STORE_OP_DONT_CARE,
            .clearValue         = {.depthStencil = {1.0f, 0}},
        };
        const VkRenderingInfo ri{
            .sType                = VK_STRUCTURE_TYPE_RENDERING_INFO,
            .pNext                = nullptr,
            .flags                = 0,
            .renderArea           = {{0, 0}, {swapchain_->extent().width, swapchain_->extent().height}},
            .layerCount           = 1,
            .viewMask             = 0,
            .colorAttachmentCount = 1,
            .pColorAttachments    = &colorAtt,
            .pDepthAttachment     = &depthAtt,
            .pStencilAttachment   = nullptr,
        };
        vkCmdBeginRendering(command, &ri);
    }
    else
    {
        vkCmdBeginRenderPass(command, &renderPassBegin, VK_SUBPASS_CONTENTS_INLINE);
    }
    vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_->pipeline());

    const VkViewport viewport{0.0f, 0.0f, static_cast<float>(width), static_cast<float>(height), 0.0f, 1.0f};
    const VkRect2D   scissor{{0, 0}, {width, height}};
    vkCmdSetViewport(command, 0, 1, &viewport);
    vkCmdSetScissor(command, 0, 1, &scissor);

    vkCmdBindDescriptorSets(command,
        VK_PIPELINE_BIND_POINT_GRAPHICS,
        pipeline_->pipelineLayout(),
        0,
        1,
        resources_->frameUniformSetPtr(frames_->currentFrame()),
        0,
        nullptr);
}

void VulkanBackend::drawMesh(MeshHandle mesh, const float *modelMatrix, const MeshMaterial &material)
{
    if (!frameActive_)
    {
        return;
    }

    const auto *vulkanMesh = resources_->meshFor(mesh);
    if (vulkanMesh == nullptr)
    {
        return;
    }

    if (frustumCullingEnabled_ && cameraInitialized_ && vulkanMesh->boundsValid && vulkanMesh->indexCount >= 12)
    {
        // Sphere-vs-plane cull: reject when the model-transformed bounding
        // sphere is fully behind any frustum plane (scaled by the model
        // matrix's basis length to handle non-uniform sizing).
        const glm::mat4 model = glm::make_mat4(modelMatrix);
        const glm::vec3 localCenter(
            vulkanMesh->boundsCenter[0], vulkanMesh->boundsCenter[1], vulkanMesh->boundsCenter[2]);
        const glm::vec4 worldCenter = model * glm::vec4(localCenter, 1.0f);
        const glm::vec3 basisX      = glm::vec3(model[0]);
        const glm::vec3 basisY      = glm::vec3(model[1]);
        const glm::vec3 basisZ      = glm::vec3(model[2]);
        const float     worldScale
            = std::sqrt(glm::dot(basisX, basisX) + glm::dot(basisY, basisY) + glm::dot(basisZ, basisZ));
        const float worldRadius = vulkanMesh->boundsRadius * worldScale;
        for (const glm::vec4 &plane : frustumPlanes_)
        {
            const float signedDistance = glm::dot(plane, worldCenter);
            if (std::isfinite(signedDistance) && signedDistance < -worldRadius)
            {
                return;
            }
        }
    }

    VkCommandBuffer command = frames_->commandBuffer();

    PushConstants push; // .model is the object matrix; material factors fill the rest
    std::memcpy(push.model, modelMatrix, sizeof(push.model));
    push.baseColorFactor[0]   = material.baseColor[0];
    push.baseColorFactor[1]   = material.baseColor[1];
    push.baseColorFactor[2]   = material.baseColor[2];
    push.baseColorFactor[3]   = 1.0f;
    push.metallicRoughness[0] = material.metallicFactor;
    push.metallicRoughness[1] = material.roughnessFactor;
    push.metallicRoughness[2] = material.normalScale;
    push.metallicRoughness[3] = material.occlusionStrength;
    push.emissive[0]          = material.emissiveColor[0];
    push.emissive[1]          = material.emissiveColor[1];
    push.emissive[2]          = material.emissiveColor[2];
    push.emissive[3]          = material.unlit ? 1.0f : 0.0f;
    if (!hasLastPushConstants_ || std::memcmp(&push, &lastPushConstants_, sizeof(PushConstants)) != 0)
    {
        vkCmdPushConstants(command,
            pipeline_->pipelineLayout(),
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0,
            sizeof(PushConstants),
            &push);
        lastPushConstants_    = push;
        hasLastPushConstants_ = true;
    }

    const VkDescriptorSet materialSet = resources_->materialSet(material);
    if (materialSet != lastBoundMaterialSet_)
    {
        vkCmdBindDescriptorSets(
            command, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_->pipelineLayout(), 1, 1, &materialSet, 0, nullptr);
        lastBoundMaterialSet_ = materialSet;
    }

    if (mesh != lastBoundMesh_)
    {
        const VkDeviceSize offset = 0;
        vkCmdBindVertexBuffers(command, 0, 1, &vulkanMesh->vertexBuffer, &offset);
        vkCmdBindIndexBuffer(command, vulkanMesh->indexBuffer, 0, VK_INDEX_TYPE_UINT32);
        lastBoundMesh_ = mesh;
    }
    // Instance stream slot 0 always holds an identity matrix, so non-instanced
    // draws can leave the instance binding at offset 0 from a previous draw.
    if (!hasLastBoundInstanceBuffer_ || lastBoundInstanceOffset_ != 0)
    {
        const InstanceBuffer &instanceBuffer = instanceBuffers_[frames_->currentFrame()];
        const VkDeviceSize    offset         = 0;
        vkCmdBindVertexBuffers(command, 1, 1, &instanceBuffer.buffer, &offset);
        lastBoundInstanceOffset_    = 0;
        hasLastBoundInstanceBuffer_ = true;
    }
    vkCmdDrawIndexed(command, vulkanMesh->indexCount, 1, 0, 0, 0);

    statistics_.trianglesSubmitted += vulkanMesh->indexCount / 3;
    statistics_.trianglesRasterized += vulkanMesh->indexCount / 3;
}

// Streams the instance matrices into this frame slot's host-visible ring
// buffer (frustum-culled and finite-checked first), then issues one indexed
// draw for the whole run with the instance buffer bound at the run's offset.
// Matrices that overflow the stream capacity fall back to per-instance
// drawMesh calls (streamOverflow path re-walks the list, skipping the ones
// already drawn).
void VulkanBackend::drawMeshInstances(
    MeshHandle mesh, const float *modelMatrices, std::uint32_t instanceCount, const MeshMaterial &material)
{
    if (!frameActive_ || modelMatrices == nullptr || instanceCount == 0)
    {
        return;
    }
    const auto *vulkanMesh = resources_->meshFor(mesh);
    if (vulkanMesh == nullptr || frames_->currentFrame() >= instanceBuffers_.size())
    {
        return;
    }

    InstanceBuffer        &instanceBuffer      = instanceBuffers_[frames_->currentFrame()];
    constexpr VkDeviceSize matrixSize          = sizeof(float) * 16u;
    const VkDeviceSize     firstInstanceOffset = instanceBuffer.cursor;
    std::uint32_t          visibleCount        = 0;
    bool                   streamOverflow      = false;

    const auto isFiniteMatrix = [](const float *matrix)
    {
        for (std::size_t component = 0; component < 16; ++component)
        {
            if (!std::isfinite(matrix[component]))
            {
                return false;
            }
        }
        return true;
    };
    const auto isVisible = [this, vulkanMesh](const float *matrixData)
    {
        if (!frustumCullingEnabled_ || !cameraInitialized_ || !vulkanMesh->boundsValid || vulkanMesh->indexCount < 12)
        {
            return true;
        }
        const glm::mat4 model = glm::make_mat4(matrixData);
        const glm::vec3 localCenter(
            vulkanMesh->boundsCenter[0], vulkanMesh->boundsCenter[1], vulkanMesh->boundsCenter[2]);
        const glm::vec4 worldCenter = model * glm::vec4(localCenter, 1.0f);
        const glm::vec3 basisX      = glm::vec3(model[0]);
        const glm::vec3 basisY      = glm::vec3(model[1]);
        const glm::vec3 basisZ      = glm::vec3(model[2]);
        const float     worldScale
            = std::sqrt(glm::dot(basisX, basisX) + glm::dot(basisY, basisY) + glm::dot(basisZ, basisZ));
        const float worldRadius = vulkanMesh->boundsRadius * worldScale;
        for (const glm::vec4 &plane : frustumPlanes_)
        {
            const float signedDistance = glm::dot(plane, worldCenter);
            if (std::isfinite(signedDistance) && signedDistance < -worldRadius)
            {
                return false;
            }
        }
        return true;
    };

    for (std::uint32_t index = 0; index < instanceCount; ++index)
    {
        const float *matrixData = modelMatrices + static_cast<std::size_t>(index) * 16u;
        if (!isFiniteMatrix(matrixData) || !isVisible(matrixData))
        {
            continue;
        }
        if (instanceBuffer.cursor > instanceBuffer.capacity
            || matrixSize > instanceBuffer.capacity - instanceBuffer.cursor)
        {
            streamOverflow = true;
            continue;
        }
        std::memcpy(static_cast<std::byte *>(instanceBuffer.mapping) + instanceBuffer.cursor,
            matrixData,
            static_cast<std::size_t>(matrixSize));
        instanceBuffer.cursor += matrixSize;
        ++visibleCount;
    }

    if (visibleCount != 0)
    {
        const VkDeviceSize writtenBytes = instanceBuffer.cursor - firstInstanceOffset;
        if (!device_->flushAllocation(instanceBuffer.allocation, firstInstanceOffset, writtenBytes))
        {
            std::fprintf(stderr, "renderer module: could not flush the instance stream; falling back to single draws\n");
            instanceBuffer.cursor = firstInstanceOffset;
            visibleCount          = 0;
            streamOverflow        = true;
        }
        else
        {
            VkCommandBuffer command = frames_->commandBuffer();
            PushConstants   push;
            push.model[0]             = 1.0f;
            push.model[5]             = 1.0f;
            push.model[10]            = 1.0f;
            push.model[15]            = 1.0f; // instance model is identity; real matrices come from the stream
            push.baseColorFactor[0]   = material.baseColor[0];
            push.baseColorFactor[1]   = material.baseColor[1];
            push.baseColorFactor[2]   = material.baseColor[2];
            push.baseColorFactor[3]   = 1.0f;
            push.metallicRoughness[0] = material.metallicFactor;
            push.metallicRoughness[1] = material.roughnessFactor;
            push.metallicRoughness[2] = material.normalScale;
            push.metallicRoughness[3] = material.occlusionStrength;
            push.emissive[0]          = material.emissiveColor[0];
            push.emissive[1]          = material.emissiveColor[1];
            push.emissive[2]          = material.emissiveColor[2];
            push.emissive[3]          = material.unlit ? 1.0f : 0.0f;
            if (!hasLastPushConstants_ || std::memcmp(&push, &lastPushConstants_, sizeof(PushConstants)) != 0)
            {
                vkCmdPushConstants(command,
                    pipeline_->pipelineLayout(),
                    VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                    0,
                    sizeof(PushConstants),
                    &push);
                lastPushConstants_    = push;
                hasLastPushConstants_ = true;
            }

            const VkDescriptorSet materialSet = resources_->materialSet(material);
            if (materialSet != lastBoundMaterialSet_)
            {
                vkCmdBindDescriptorSets(command,
                    VK_PIPELINE_BIND_POINT_GRAPHICS,
                    pipeline_->pipelineLayout(),
                    1,
                    1,
                    &materialSet,
                    0,
                    nullptr);
                lastBoundMaterialSet_ = materialSet;
            }
            if (mesh != lastBoundMesh_)
            {
                const VkDeviceSize offset = 0;
                vkCmdBindVertexBuffers(command, 0, 1, &vulkanMesh->vertexBuffer, &offset);
                vkCmdBindIndexBuffer(command, vulkanMesh->indexBuffer, 0, VK_INDEX_TYPE_UINT32);
                lastBoundMesh_ = mesh;
            }
            if (!hasLastBoundInstanceBuffer_ || lastBoundInstanceOffset_ != firstInstanceOffset)
            {
                vkCmdBindVertexBuffers(command, 1, 1, &instanceBuffer.buffer, &firstInstanceOffset);
                lastBoundInstanceOffset_    = firstInstanceOffset;
                hasLastBoundInstanceBuffer_ = true;
            }
            vkCmdDrawIndexed(command, vulkanMesh->indexCount, visibleCount, 0, 0, 0);
            const std::uint64_t triangles = static_cast<std::uint64_t>(vulkanMesh->indexCount / 3) * visibleCount;
            statistics_.trianglesSubmitted += triangles;
            statistics_.trianglesRasterized += triangles;
        }
    }

    if (streamOverflow)
    {
        std::uint32_t skipped = 0;
        for (std::uint32_t index = 0; index < instanceCount; ++index)
        {
            const float *matrixData = modelMatrices + static_cast<std::size_t>(index) * 16u;
            if (!isFiniteMatrix(matrixData) || !isVisible(matrixData))
            {
                continue;
            }
            if (skipped < visibleCount)
            {
                ++skipped;
                continue;
            }
            drawMesh(mesh, matrixData, material);
        }
    }
}

// Allocates the per-slot instance streams, pre-writing an identity matrix at
// offset 0 for non-instanced draws. Halves the requested capacity (down to a
// 1 MiB floor) until allocation succeeds.
bool VulkanBackend::createInstanceBuffers()
{
    if (device_ == nullptr)
    {
        return false;
    }
    instanceBuffers_.resize(framesInFlight_);
    constexpr VkDeviceSize minimumCapacity = 1ull * 1024ull * 1024ull;
    constexpr float        identity[16]    = {
        1.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        1.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        1.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        1.0f,
    };

    while (true)
    {
        bool allocationSucceeded = true;
        for (InstanceBuffer &buffer : instanceBuffers_)
        {
            if (!device_->createBuffer(instanceBufferCapacity_,
                    VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                    BufferMemoryIntent::HostSequentialWrite,
                    &buffer.buffer,
                    &buffer.allocation)
                || !device_->mapAllocation(buffer.allocation, &buffer.mapping))
            {
                allocationSucceeded = false;
                break;
            }
            buffer.capacity = instanceBufferCapacity_;
            buffer.cursor   = sizeof(identity);
            std::memcpy(buffer.mapping, identity, sizeof(identity));
            if (!device_->flushAllocation(buffer.allocation))
            {
                allocationSucceeded = false;
                break;
            }
        }
        if (allocationSucceeded)
        {
            return true;
        }

        destroyInstanceBuffers();
        if (instanceBufferCapacity_ <= minimumCapacity)
        {
            std::fprintf(stderr, "renderer module: could not allocate a 1 MiB per-frame instance stream\n");
            return false;
        }
        instanceBufferCapacity_ = std::max(minimumCapacity, instanceBufferCapacity_ / 2u);
        std::fprintf(stderr,
            "renderer module: instance stream allocation failed; retrying at %.1f MiB per frame\n",
            static_cast<double>(instanceBufferCapacity_) / (1024.0 * 1024.0));
    }
}

void VulkanBackend::destroyInstanceBuffers() noexcept
{
    if (device_ == nullptr)
    {
        return;
    }
    for (InstanceBuffer &buffer : instanceBuffers_)
    {
        if (buffer.mapping != nullptr)
        {
            device_->unmapAllocation(buffer.allocation);
        }
        if (buffer.buffer != VK_NULL_HANDLE)
        {
            device_->destroyBuffer(buffer.buffer, buffer.allocation);
        }
        buffer = {};
    }
    instanceBuffers_.clear();
}

// Grows (doubling from 4 KiB) a persistent-mapped vertex/index pair when the
// incoming overlay data no longer fits.
bool VulkanBackend::ensureOverlayBuffers(std::uint32_t frameIndex, VkDeviceSize vertexBytes, VkDeviceSize indexBytes)
{
    if (device_ == nullptr || frameIndex >= overlayBuffers_.size())
        return false;
    OverlayBuffer &buffers = overlayBuffers_[frameIndex];

    const auto grow = [this](VkDeviceSize    required,
                          VkBufferUsageFlags usage,
                          VkBuffer          &buffer,
                          VmaAllocation     &allocation,
                          void             *&mapping,
                          VkDeviceSize      &capacity)
    {
        if (required <= capacity && buffer != VK_NULL_HANDLE && mapping != nullptr)
            return true;
        if (required == 0)
            return true;
        VkDeviceSize newCapacity = std::max<VkDeviceSize>(4096, capacity);
        while (newCapacity < required)
        {
            if (newCapacity > std::numeric_limits<VkDeviceSize>::max() / 2)
            {
                newCapacity = required;
                break;
            }
            newCapacity *= 2;
        }
        VkBuffer      newBuffer     = VK_NULL_HANDLE;
        VmaAllocation newAllocation = nullptr;
        void         *newMapping    = nullptr;
        if (!device_->createBuffer(
                newCapacity, usage, BufferMemoryIntent::HostSequentialWrite, &newBuffer, &newAllocation)
            || !device_->mapAllocation(newAllocation, &newMapping))
        {
            if (newBuffer != VK_NULL_HANDLE)
                device_->destroyBuffer(newBuffer, newAllocation);
            return false;
        }
        if (mapping != nullptr)
            device_->unmapAllocation(allocation);
        if (buffer != VK_NULL_HANDLE)
            device_->destroyBuffer(buffer, allocation);
        buffer     = newBuffer;
        allocation = newAllocation;
        mapping    = newMapping;
        capacity   = newCapacity;
        return true;
    };

    return grow(vertexBytes,
               VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
               buffers.vertexBuffer,
               buffers.vertexAllocation,
               buffers.vertexMapping,
               buffers.vertexCapacity)
        && grow(indexBytes,
            VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
            buffers.indexBuffer,
            buffers.indexAllocation,
            buffers.indexMapping,
            buffers.indexCapacity);
}

void VulkanBackend::destroyOverlayBuffers() noexcept
{
    if (device_ == nullptr)
        return;
    for (OverlayBuffer &buffers : overlayBuffers_)
    {
        if (buffers.vertexMapping != nullptr)
            device_->unmapAllocation(buffers.vertexAllocation);
        if (buffers.vertexBuffer != VK_NULL_HANDLE)
            device_->destroyBuffer(buffers.vertexBuffer, buffers.vertexAllocation);
        if (buffers.indexMapping != nullptr)
            device_->unmapAllocation(buffers.indexAllocation);
        if (buffers.indexBuffer != VK_NULL_HANDLE)
            device_->destroyBuffer(buffers.indexBuffer, buffers.indexAllocation);
        buffers = {};
    }
    overlayBuffers_.clear();
}

// Overlay pass: copies the vertices/indices into this frame slot's scratch
// buffers, pushes the display->NDC transform (scale + translate as 4 floats),
// then per command sets the scissor from the clip rectangle and binds the
// command's texture as a one-entry material set. resetRenderState rebinds
// everything first (for UI recorders that interleave state changes).
void VulkanBackend::drawOverlay(const OverlayDrawData &data)
{
    if (!frameActive_ || data.vertices == nullptr || data.indices == nullptr || data.commands == nullptr
        || data.vertexCount == 0 || data.indexCount == 0 || data.commandCount == 0
        || !std::isfinite(data.displaySize[0]) || !std::isfinite(data.displaySize[1]) || data.displaySize[0] <= 0.0f
        || data.displaySize[1] <= 0.0f || !std::isfinite(data.framebufferScale[0])
        || !std::isfinite(data.framebufferScale[1]) || data.framebufferScale[0] <= 0.0f
        || data.framebufferScale[1] <= 0.0f)
    {
        return;
    }

    const std::uint32_t frameIndex  = frames_->currentFrame();
    const VkDeviceSize  vertexBytes = static_cast<VkDeviceSize>(data.vertexCount) * sizeof(OverlayVertex);
    const VkDeviceSize  indexBytes  = static_cast<VkDeviceSize>(data.indexCount) * sizeof(std::uint32_t);
    if (!ensureOverlayBuffers(frameIndex, vertexBytes, indexBytes))
        return;
    OverlayBuffer &buffers = overlayBuffers_[frameIndex];
    std::memcpy(buffers.vertexMapping, data.vertices, static_cast<std::size_t>(vertexBytes));
    std::memcpy(buffers.indexMapping, data.indices, static_cast<std::size_t>(indexBytes));
    if (!device_->flushAllocation(buffers.vertexAllocation) || !device_->flushAllocation(buffers.indexAllocation))
        return;

    VkCommandBuffer     command = frames_->commandBuffer();
    const VkExtent2D    extent  = swapchain_->extent();
    const std::uint32_t width   = viewportWidth_ != 0 ? std::min(viewportWidth_, extent.width) : extent.width;
    const std::uint32_t height  = viewportHeight_ != 0 ? std::min(viewportHeight_, extent.height) : extent.height;
    if (width == 0 || height == 0)
        return;

    vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_->overlayPipeline());
    const float scale[2]     = {2.0f / data.displaySize[0], 2.0f / data.displaySize[1]};
    const float translate[2] = {-1.0f - data.displayPosition[0] * scale[0], -1.0f - data.displayPosition[1] * scale[1]};
    const float pushConstants[4] = {scale[0], scale[1], translate[0], translate[1]};
    vkCmdPushConstants(command,
        pipeline_->overlayPipelineLayout(),
        VK_SHADER_STAGE_VERTEX_BIT,
        0,
        sizeof(pushConstants),
        pushConstants);

    const VkDeviceSize bufferOffset = 0;
    vkCmdBindVertexBuffers(command, 0, 1, &buffers.vertexBuffer, &bufferOffset);
    vkCmdBindIndexBuffer(command, buffers.indexBuffer, 0, VK_INDEX_TYPE_UINT32);
    const VkViewport viewport{0.0f, 0.0f, static_cast<float>(width), static_cast<float>(height), 0.0f, 1.0f};
    vkCmdSetViewport(command, 0, 1, &viewport);

    MeshMaterial textureMaterial{};

    VkDescriptorSet lastOverlayMaterialSet = VK_NULL_HANDLE;
    for (std::uint32_t commandIndex = 0; commandIndex < data.commandCount; ++commandIndex)
    {
        const OverlayDrawCommand &draw = data.commands[commandIndex];
        if (draw.resetRenderState)
        {
            vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_->overlayPipeline());
            vkCmdPushConstants(command,
                pipeline_->overlayPipelineLayout(),
                VK_SHADER_STAGE_VERTEX_BIT,
                0,
                sizeof(pushConstants),
                pushConstants);
            vkCmdBindVertexBuffers(command, 0, 1, &buffers.vertexBuffer, &bufferOffset);
            vkCmdBindIndexBuffer(command, buffers.indexBuffer, 0, VK_INDEX_TYPE_UINT32);
            vkCmdSetViewport(command, 0, 1, &viewport);
        }
        if (draw.elementCount == 0 || draw.indexOffset > data.indexCount
            || draw.elementCount > data.indexCount - draw.indexOffset
            || draw.vertexOffset > static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max()))
        {
            continue;
        }

        const float clipLeft   = (draw.clipRectangle[0] - data.displayPosition[0]) * data.framebufferScale[0];
        const float clipTop    = (draw.clipRectangle[1] - data.displayPosition[1]) * data.framebufferScale[1];
        const float clipRight  = (draw.clipRectangle[2] - data.displayPosition[0]) * data.framebufferScale[0];
        const float clipBottom = (draw.clipRectangle[3] - data.displayPosition[1]) * data.framebufferScale[1];
        if (!std::isfinite(clipLeft) || !std::isfinite(clipTop) || !std::isfinite(clipRight)
            || !std::isfinite(clipBottom))
        {
            continue;
        }
        const float        boundedLeft   = std::clamp(clipLeft, 0.0f, static_cast<float>(width));
        const float        boundedTop    = std::clamp(clipTop, 0.0f, static_cast<float>(height));
        const float        boundedRight  = std::clamp(clipRight, 0.0f, static_cast<float>(width));
        const float        boundedBottom = std::clamp(clipBottom, 0.0f, static_cast<float>(height));
        const std::int32_t x0            = static_cast<std::int32_t>(std::floor(boundedLeft));
        const std::int32_t y0            = static_cast<std::int32_t>(std::floor(boundedTop));
        const std::int32_t x1            = static_cast<std::int32_t>(std::ceil(boundedRight));
        const std::int32_t y1            = static_cast<std::int32_t>(std::ceil(boundedBottom));
        if (x0 >= x1 || y0 >= y1)
            continue;

        const VkRect2D scissor{{x0, y0}, {static_cast<std::uint32_t>(x1 - x0), static_cast<std::uint32_t>(y1 - y0)}};
        vkCmdSetScissor(command, 0, 1, &scissor);

        textureMaterial.baseColorTexture = draw.texture;
        const VkDescriptorSet textureSet = resources_->materialSet(textureMaterial);
        if (textureSet == VK_NULL_HANDLE)
            continue;
        if (textureSet != lastOverlayMaterialSet)
        {
            vkCmdBindDescriptorSets(command,
                VK_PIPELINE_BIND_POINT_GRAPHICS,
                pipeline_->overlayPipelineLayout(),
                0,
                1,
                &textureSet,
                0,
                nullptr);
            lastOverlayMaterialSet = textureSet;
        }
        vkCmdDrawIndexed(
            command, draw.elementCount, 1, draw.indexOffset, static_cast<std::int32_t>(draw.vertexOffset), 0);
    }
}

// Finishes the render pass, records the optional PPM readback copy, ends
// GPU timing, submits (retirement via slot fence or timeline semaphore),
// presents, and rotates to the next frame slot. A requested screenshot does
// NOT block here: its file write is deferred until the copy provably
// completed (deferred-export lifecycle, see the PendingExport comment).
void VulkanBackend::endFrame()
{
    if (!frameActive_)
    {
        return;
    }
    VkCommandBuffer command = frames_->commandBuffer();

    if (pipeline_->dynamicRenderingEnabled())
    {
        vkCmdEndRendering(command);
        const VkImageMemoryBarrier toPresent{
            .sType               = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
            .pNext               = nullptr,
            .srcAccessMask       = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
            .dstAccessMask       = 0,
            .oldLayout           = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            .newLayout           = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
            .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .image               = swapchain_->images()[currentImageIndex_],
            .subresourceRange    = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
        };
        vkCmdPipelineBarrier(command,
            VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
            0,
            0,
            nullptr,
            0,
            nullptr,
            1,
            &toPresent);
    }
    else
    {
        vkCmdEndRenderPass(command);
    }

    bool exportCopyRecorded = false;
    std::uint32_t exportWidth = 0;
    std::uint32_t exportHeight = 0;
    bool          exportBgra   = false;
    if (exportRequested_)
    {
        // Export path: transition to TRANSFER_SRC, copy into the readback
        // buffer, then back to PRESENT. The file write happens later (next
        // slot rotation, or an explicit flush) - not between submit and
        // present, which is where the synchronous path stalled the frame.
        if (swapchain_->ensureReadbackBuffer())
        {
            const VkImageMemoryBarrier toTransfer{
                .sType               = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
                .pNext               = nullptr,
                .srcAccessMask       = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
                .dstAccessMask       = VK_ACCESS_TRANSFER_READ_BIT,
                .oldLayout           = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                .newLayout           = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
                .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
                .image               = swapchain_->images()[currentImageIndex_],
                .subresourceRange    = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
            };
            vkCmdPipelineBarrier(command,
                VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                VK_PIPELINE_STAGE_TRANSFER_BIT,
                0,
                0,
                nullptr,
                0,
                nullptr,
                1,
                &toTransfer);
            exportWidth  = swapchain_->extent().width;
            exportHeight = swapchain_->extent().height;
            exportBgra   = swapchain_->formatIsBgra();
            const VkBufferImageCopy region{
                .bufferOffset      = 0,
                .bufferRowLength   = 0,
                .bufferImageHeight = 0,
                .imageSubresource  = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1},
                .imageOffset       = {0, 0, 0},
                .imageExtent       = {exportWidth, exportHeight, 1},
            };
            vkCmdCopyImageToBuffer(command,
                swapchain_->images()[currentImageIndex_],
                VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                swapchain_->readbackBuffer(),
                1,
                &region);
            const VkImageMemoryBarrier toPresent{
                .sType               = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
                .pNext               = nullptr,
                .srcAccessMask       = VK_ACCESS_TRANSFER_READ_BIT,
                .dstAccessMask       = 0,
                .oldLayout           = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                .newLayout           = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
                .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
                .image               = swapchain_->images()[currentImageIndex_],
                .subresourceRange    = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
            };
            vkCmdPipelineBarrier(command,
                VK_PIPELINE_STAGE_TRANSFER_BIT,
                VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
                0,
                0,
                nullptr,
                0,
                nullptr,
                1,
                &toPresent);
            exportCopyRecorded = true;
        }
        else
        {
            std::fprintf(stderr, "renderer module: screenshot export failed\n");
            exportRequested_ = false;
        }
    }

    frames_->endGpuTiming();
    vkEndCommandBuffer(command);

    const VkPipelineStageFlags waitStage       = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    const VkSemaphore          waitSemaphore   = frames_->imageAvailableSemaphore();
    const VkSemaphore          signalSemaphore = swapchain_->renderFinishedSemaphore(currentImageIndex_);
    if (signalSemaphore == VK_NULL_HANDLE)
    {
        std::fprintf(
            stderr, "renderer module: missing presentation semaphore for swapchain image %u\n", currentImageIndex_);
        frameActive_ = false;
        swapchain_->markDirty();
        return;
    }

    const bool    useTimeline      = device_->timelineSemaphore() != VK_NULL_HANDLE;
    const VkFence submitFence      = useTimeline ? VK_NULL_HANDLE : frames_->inFlightFence(); // signals the slot
    std::uint64_t frameSignalValue = 0;
    if (useTimeline)
    {
        frameSignalValue                                        = device_->acquireTimelineValue();
        frames_->frameTimelineValues()[frames_->currentFrame()] = frameSignalValue;
    }
    const VkTimelineSemaphoreSubmitInfo timelineSubmit{
        .sType                     = VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO,
        .pNext                     = nullptr,
        .waitSemaphoreValueCount   = 0,
        .pWaitSemaphoreValues      = nullptr,
        .signalSemaphoreValueCount = useTimeline ? 1u : 0u,
        .pSignalSemaphoreValues    = useTimeline ? &frameSignalValue : nullptr,
    };
    const VkSemaphore  signalSems[2] = {signalSemaphore, device_->timelineSemaphore()};
    const uint32_t     signalCount   = useTimeline ? 2u : 1u;
    const VkSubmitInfo submitInfo{
        .sType                = VK_STRUCTURE_TYPE_SUBMIT_INFO,
        .pNext                = useTimeline ? &timelineSubmit : nullptr,
        .waitSemaphoreCount   = 1,
        .pWaitSemaphores      = &waitSemaphore,
        .pWaitDstStageMask    = &waitStage,
        .commandBufferCount   = 1,
        .pCommandBuffers      = &command,
        .signalSemaphoreCount = signalCount,
        .pSignalSemaphores    = signalSems,
    };
    VkResult submitResult = VK_SUCCESS;
    {
        // QueueSubmit timing includes the queue-mutex wait when another
        // thread holds the submission lock (uploads share the queue mutex):
        // contention here appears as a tail on this metric.
        const ScopedProfileTimer submitTimer(profiler_, PerformanceMetric::QueueSubmit);
        std::lock_guard          queueLock(device_->queueMutex());
        submitResult = vkQueueSubmit(device_->queue(), 1, &submitInfo, submitFence);
    }
    if (submitResult != VK_SUCCESS)
    {
        std::fprintf(stderr, "renderer module: vkQueueSubmit failed\n");
        frameActive_ = false;
        return;
    }

    if (exportCopyRecorded)
    {
        // Register only after a successful submit: the slot's retirement is
        // now the copy's completion signal. A failed submit keeps the
        // request pending so the next endFrame re-records it.
        pendingExportResult_.active        = true;
        pendingExportResult_.path          = exportPath_;
        pendingExportResult_.frameSlot     = frames_->currentFrame();
        pendingExportResult_.width         = exportWidth;
        pendingExportResult_.height        = exportHeight;
        pendingExportResult_.bgra          = exportBgra;
        pendingExportResult_.timelineValue = frameSignalValue;
        exportRequested_                   = false;
    }

    VkResult presentResult = VK_SUCCESS;
    {
        // Present-call duration is compositor/queue behaviour, not GPU
        // execution time: kept separate so a slow vkQueuePresentKHR is not
        // misattributed to rendering.
        const ScopedProfileTimer presentTimer(profiler_, PerformanceMetric::SwapchainPresent);
        presentResult = swapchain_->present(signalSemaphore, currentImageIndex_);
    }
    if (presentResult != VK_SUCCESS && presentResult != VK_ERROR_OUT_OF_DATE_KHR && presentResult != VK_SUBOPTIMAL_KHR)
    {
        std::fprintf(stderr, "renderer module: vkQueuePresentKHR failed (%d)\n", (int)presentResult);
    }

    frameActive_ = false;
    frames_->rotate();

    ++statistics_.framesRendered;
}

bool VulkanBackend::createMesh(MeshHandle id, const MeshDescription &description, RenderError *error)
{
    return resources_->createMesh(id, description, error);
}

void VulkanBackend::destroyMesh(MeshHandle mesh)
{
    // Defer while a frame is recorded so the GPU stops using the buffers
    // first; the else branch waits for the device before immediate teardown.
    if (mesh != invalidMesh && !deferredDestroys_.empty())
        deferredDestroys_[frames_->currentFrame()].push_back({mesh, invalidTexture});
    else
    {
        waitForDeviceQuiet();
        resources_->destroyMesh(mesh);
    }
}

bool VulkanBackend::createTexture(TextureHandle id, const TextureDescription &description, RenderError *error)
{
    return resources_->createTexture(id, description, error);
}

void VulkanBackend::destroyTexture(TextureHandle texture)
{
    if (texture != invalidTexture && !deferredDestroys_.empty())
        deferredDestroys_[frames_->currentFrame()].push_back({invalidMesh, texture});
    else
    {
        waitForDeviceQuiet();
        resources_->destroyTexture(texture);
    }
}

// Frees the resources queued by destroyMesh/destroyTexture for this frame
// slot; called from beginFrame once the slot's previous work has retired.
void VulkanBackend::drainDeferredDestroys(std::uint32_t frameIndex)
{
    if (frameIndex >= deferredDestroys_.size())
        return;
    auto &queue = deferredDestroys_[frameIndex];
    for (const auto &entry : queue)
    {
        if (entry.mesh != invalidMesh)
            resources_->destroyMesh(entry.mesh);
        if (entry.texture != invalidTexture)
            resources_->destroyTexture(entry.texture);
    }
    queue.clear();
}

bool VulkanBackend::infoStrings(RendererInfoStrings *outInfo) const
{
    if (outInfo == nullptr)
    {
        return false;
    }
    outInfo->vendor          = device_->vendor().c_str();
    outInfo->renderer        = device_->renderer().c_str();
    outInfo->version         = device_->version().c_str();
    outInfo->shadingLanguage = "SPIR-V";
    return true;
}

bool VulkanBackend::nativeHandles(VulkanDeviceHandles *outHandles) const
{
    if (outHandles == nullptr)
    {
        return false;
    }
    outHandles->vkInstance          = device_->instance();
    outHandles->vkPhysicalDevice    = device_->physicalDevice();
    outHandles->vkDevice            = device_->device();
    outHandles->vkGraphicsQueue     = device_->queue();
    outHandles->graphicsQueueFamily = device_->graphicsFamily();
    return true;
}

// exportFramePpm is asynchronous by design: it records the request, the
// copy runs at the next endFrame, and the file is written once the copy's
// submission provably completed (next rotation of its frame slot, an
// explicit flush, or renderer destruction). A second request while one is
// still outstanding flushes the first precisely - order is preserved and
// the shared readback buffer is never overwritten by two pending copies.
bool VulkanBackend::exportFramePpm(const char *filePath, RenderError *error)
{
    if (pendingExportResult_.active)
    {
        // Rare (two screenshots back to back): complete the outstanding
        // copy first, before any validation returns early. This is a
        // diagnostic path off the frame loop, so the precise wait here
        // costs nothing on the frame-time tail.
        flushPendingExport(true);
    }
    if (swapchain_->extent().width == 0 || swapchain_->extent().height == 0)
    {
        reportError(error, RenderError::CreationFailed);
        return false;
    }
    if (filePath == nullptr)
    {
        reportError(error, RenderError::CreationFailed);
        return false;
    }

    exportRequested_ = true;
    exportPath_      = filePath;
    reportError(error, RenderError::None);
    return true;
}

// Completes the pending export when its copy provably finished - the
// recording slot retired (its fence/timeline wait already ran in
// waitForFrameSlot). Non-blocking by construction.
void VulkanBackend::completePendingExportIfReady()
{
    if (!pendingExportResult_.active || swapchain_ == nullptr)
        return;
    if (pendingExportResult_.frameSlot != frames_->currentFrame())
        return;
    RenderError exportError = RenderError::None;
    swapchain_->finishReadback(pendingExportResult_.path.c_str(),
        pendingExportResult_.width,
        pendingExportResult_.height,
        pendingExportResult_.bgra,
        &exportError);
    if (exportError != RenderError::None)
    {
        std::fprintf(stderr, "renderer module: screenshot export failed\n");
    }
    pendingExportResult_ = PendingExport{};
}

// Writes a pending export out of band. With allowBlocking it waits the
// recording slot's fence first (precise: that fence signals exactly when
// the copy's submission completed); without, the caller guarantees the GPU
// is idle (shutdown path). The only blocking readback wait left in the
// engine - recorded under ReadbackWait, which must stay empty during
// benchmark runs.
void VulkanBackend::flushPendingExport(bool allowBlocking)
{
    if (!pendingExportResult_.active)
        return;
    if (allowBlocking && device_ != nullptr && device_->device() != VK_NULL_HANDLE)
    {
        const ScopedProfileTimer readbackWaitTimer(profiler_, PerformanceMetric::ReadbackWait);
        if (device_->timelineSemaphore() != VK_NULL_HANDLE && pendingExportResult_.timelineValue != 0)
        {
            device_->waitTimeline(pendingExportResult_.timelineValue);
        }
        else if (frames_ != nullptr)
        {
            const VkFence slotFence = frames_->inFlightFenceFor(pendingExportResult_.frameSlot);
            if (slotFence != VK_NULL_HANDLE)
                vkWaitForFences(device_->device(), 1, &slotFence, VK_TRUE, UINT64_MAX);
        }
    }
    if (swapchain_ != nullptr)
    {
        RenderError exportError = RenderError::None;
        swapchain_->finishReadback(pendingExportResult_.path.c_str(),
            pendingExportResult_.width,
            pendingExportResult_.height,
            pendingExportResult_.bgra,
            &exportError);
        if (exportError != RenderError::None)
        {
            std::fprintf(stderr, "renderer module: screenshot export failed\n");
        }
    }
    pendingExportResult_ = PendingExport{};
}

} // namespace renderer
} // namespace engine

#endif
