// Vulkan renderer backend: owns the device context, pipeline, frame scheduler,
// resources, and swapchain, and implements RenderBackend on top of them. The
// class body only exists when ENGINE_RENDERER_BACKEND_VULKAN is defined (set by
// CMakeLists.txt when Vulkan headers/loader were found); without it the
// backend is compiled out and renderer.cpp never constructs one.

#ifndef ENGINE_RENDERER_MODULE_VULKAN_BACKEND_HPP
#define ENGINE_RENDERER_MODULE_VULKAN_BACKEND_HPP

#if defined(ENGINE_RENDERER_BACKEND_VULKAN)

#include "engine/direct_lighting/capabilities.hpp"
#include "engine/pbr/capabilities.hpp"
#include "engine/renderer/capabilities.hpp"
#include "engine/renderer/device_profile.hpp"
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

#include <array>
#include <cstddef>
#include <cstdint>
#include <glm/ext/vector_float4.hpp>
#include <memory>
#include <string>
#include <vector>
#include <vulkan/vulkan.h>
#include <vulkan/vulkan_core.h>

namespace engine
{
namespace renderer
{

// Frames in flight used when the caller passes 0.
inline constexpr std::uint32_t defaultMaxFramesInFlight = 2;

// One Vulkan renderer == one window surface. create() is the factory used by
// renderer.cpp; it runs the full device/swapchain/pipeline bring-up and returns
// null (with *error set) on any failure. All entry points are main-thread
// only; GPU sync is handled internally (fences per frame slot or the device
// timeline semaphore when available).
class VulkanBackend final : public RenderBackend
{
public:
    static std::unique_ptr<RenderBackend> create(window::WindowHandle surface, bool vsync, bool validation,
        std::uint32_t framesInFlight, std::uint32_t maxPendingUploads, std::uint32_t memoryBudgetUpdateIntervalFrames,
        DevicePreference devicePreference, bool frustumCullingEnabled, bool backFaceCullingEnabled,
        bool gpuTimestampsEnabled, std::uint32_t instanceStreamMegabytes, bool textureMipmapsEnabled,
        float anisotropyLevel, std::uint32_t msaaSamples, bool srgbOutput,
        std::uint32_t maximumTextures, std::uint32_t maximumMaterialSets, std::size_t pipelineCacheMaxBytes,
        window::VulkanSurfaceCapability *vulkanSurface, RenderError *error);

    ~VulkanBackend() override;

    const char *name() const override
    {
        return "vulkan";
    }

    window::WindowHandle surface() const override
    {
        return surfaceHandle_;
    }

    DeviceClass deviceClass() const override
    {
        return device_->deviceClass();
    }

    const char *deviceName() const override
    {
        return device_->renderer().c_str();
    }
    DeviceProfile deviceProfile() const override
    {
        return device_->profile();
    }

    void setViewport(std::uint32_t width, std::uint32_t height) override;
    void setCamera(const CameraMatrices &newCamera) override;
    void setEnvironment(const pbr::PbrEnvironment &environment) override;
    void setDirectLights(const direct_lighting::DirectLight *lights, std::uint32_t count) override;
    void setProbeGrid(const GiProbeGrid &grid) override;

    void beginFrame(const FrameParameters &frame) override;
    void drawMesh(MeshHandle mesh, const float *modelMatrix, const MeshMaterial &material) override;
    void drawMeshInstances(MeshHandle mesh, const float *modelMatrices, std::uint32_t instanceCount,
        const MeshMaterial &material) override;
    void drawOverlay(const OverlayDrawData &data) override;
    void endFrame() override;

    bool createMesh(MeshHandle id, const MeshDescription &description, RenderError *error) override;
    void destroyMesh(MeshHandle mesh) override;

    bool createTexture(TextureHandle id, const TextureDescription &description, RenderError *error) override;
    void destroyTexture(TextureHandle texture) override;

    FrameStatistics statistics() const override
    {
        FrameStatistics result = statistics_;
        if (frames_ != nullptr)
        {
            result.gpuTimingAvailable   = frames_->gpuTimingAvailable();
            result.gpuFrameMilliseconds = frames_->gpuMilliseconds();
        }
        // GPU-memory-budget feedback (candidate 12): publish the tracker's
        // last sample and pressure state; zeros when the budget extension
        // is unavailable. The tracker is updated by the periodic refresh,
        // never on this path.
        const MemoryBudgetTracker &tracker = device_->memoryBudgetTracker();
        result.deviceLocalBudgetBytes     = tracker.lastBudgetBytes();
        result.deviceLocalUsageBytes      = tracker.lastUsageBytes();
        result.deviceLocalMemoryPressure  = tracker.hasBudgetSample() && tracker.inPressure();
        return result;
    }

    bool infoStrings(RendererInfoStrings *outInfo) const;
    bool nativeHandles(VulkanDeviceHandles *outHandles) const;
    bool exportFramePpm(const char *filePath, RenderError *error);

    // Attaches the shared profiler here and forwards it to the device
    // context, whose upload queue records its oldest-fence and drain waits.
    void setProfiler(engine::PerformanceProfiler *profiler) override
    {
        RenderBackend::setProfiler(profiler);
        if (device_ != nullptr)
            device_->setProfiler(profiler);
    }

private:
    VulkanBackend() = default;

    void waitForDeviceQuiet(); // vkDeviceWaitIdle + drain frame fences + finish uploads

    // One minimal real frame submitted at the end of create(), on the creator
    // thread, so driver-side deferred pipeline compilation (and every other
    // first-submit cost) completes while the application still pumps the OS
    // window around createRenderer. Best effort: failures log and continue.
    void warmUpPipelines();

    // Per-frame overlay scratch: host-visible vertex/index buffers that grow
    // geometrically; one pair per frame in flight.
    bool ensureOverlayBuffers(std::uint32_t frameIndex, VkDeviceSize vertexBytes, VkDeviceSize indexBytes);
    void destroyOverlayBuffers() noexcept;

    // Per-frame instance-matrix streams (see drawMeshInstances); halves the
    // requested capacity and retries when allocation fails.
    bool createInstanceBuffers();
    void destroyInstanceBuffers() noexcept;

    window::WindowHandle surfaceHandle_ = 0;

    bool vsync_ = true;

    std::uint32_t framesInFlight_                   = defaultMaxFramesInFlight;
    std::uint32_t memoryBudgetUpdateIntervalFrames_ = 120;

    std::unique_ptr<VulkanDeviceContext>  device_;
    std::unique_ptr<VulkanPipeline>       pipeline_;
    std::unique_ptr<VulkanFrameScheduler> frames_;
    std::unique_ptr<VulkanResources>      resources_;
    std::unique_ptr<VulkanSwapchain>      swapchain_;

    struct OverlayBuffer
    {
        VkBuffer      vertexBuffer     = VK_NULL_HANDLE;
        VmaAllocation vertexAllocation = nullptr;
        void         *vertexMapping    = nullptr; // persistently mapped host memory
        VkDeviceSize  vertexCapacity   = 0;       // bytes
        VkBuffer      indexBuffer      = VK_NULL_HANDLE;
        VmaAllocation indexAllocation  = nullptr;
        void         *indexMapping     = nullptr;
        VkDeviceSize  indexCapacity    = 0; // bytes
    };
    std::vector<OverlayBuffer> overlayBuffers_; // one entry per frame in flight

    // Ring of host-visible buffers instance matrices are streamed through;
    // cursor resets to one identity matrix at the start of each frame (slot
    // 0 is reserved so non-instanced draws can bind offset 0).
    struct InstanceBuffer
    {
        VkBuffer      buffer     = VK_NULL_HANDLE;
        VmaAllocation allocation = nullptr;
        void         *mapping    = nullptr;
        VkDeviceSize  capacity   = 0; // bytes
        VkDeviceSize  cursor     = 0; // write offset for the next batch, bytes
    };
    VkDeviceSize                instanceBufferCapacity_ = 4ull * 1024ull * 1024ull;
    std::vector<InstanceBuffer> instanceBuffers_; // one entry per frame in flight

    bool frameActive_ = false; // true between a successful beginFrame and endFrame; draw calls check it

    // Bind-state caches so repeated draws skip redundant vkCmd* calls.
    bool            hasLastPushConstants_ = false;
    PushConstants   lastPushConstants_{};
    MeshHandle      lastBoundMesh_              = invalidMesh;
    VkDescriptorSet lastBoundMaterialSet_       = VK_NULL_HANDLE;
    VkDeviceSize    lastBoundInstanceOffset_    = 0;
    bool            hasLastBoundInstanceBuffer_ = false;

    std::uint32_t currentImageIndex_ = 0; // swapchain image acquired this frame

    CameraMatrices           camera_{};
    bool                     cameraInitialized_     = false;
    bool                     frustumCullingEnabled_ = true;
    std::array<glm::vec4, 4> frustumPlanes_{}; // left/right/top/bottom, world space, unit normal

    pbr::PbrEnvironment environment_{};

    // Lights packed into the std140 layout the fragment shader expects.
    std::array<DirectLightUniformData, direct_lighting::maxDirectLights> packedDirectLights_{};
    std::uint32_t                                                        directLightCount_ = 0;

    // lumen_gi probe-grid hook: the std140 header rides the frame uniform,
    // the SH2 records are packed std430 (8 vec4 per probe) into a scratch
    // buffer and streamed to the per-slot probe storage by setProbeGrid.
    ProbeGridUniformData packedProbeGrid_{};
    std::uint64_t        probeGridRevision_    = 0;
    bool                 probeGridDirty_       = false;
    std::vector<float>   packedProbeRecords_{}; // giProbeRecordVec4s floats per probe

    FrameUniformData cachedFrameUniform_{}; // rebuilt only when inputs change (frameUniformDirty_)
    std::uint64_t    frameUniformRevision_ = 0;
    bool             frameUniformDirty_    = true;

    std::uint32_t viewportWidth_  = 0;
    std::uint32_t viewportHeight_ = 0;

    FrameStatistics statistics_{};

    bool        exportRequested_ = false; // a PPM export was requested; the copy runs at the next endFrame
    std::string exportPath_;

    // Deferred PPM export (P99 hybrid round, H2 - asynchronous readback):
    // the frame's copy is recorded into the frame command buffer and lands
    // in the shared readback buffer at submit; the file write is deferred
    // until the copy provably completed, so submit -> present is never
    // stalled by a readback wait. The copy provably completed once its frame
    // slot retires (the slot's fence/timeline wait in waitForFrameSlot), so
    // the common path completes at the next beginFrame of the same slot
    // with no wait at all; explicit flushes (a new export request, swapchain
    // recreation, shutdown) wait the recording slot's fence precisely. At
    // most one export is outstanding because the readback buffer is shared.
    struct PendingExport
    {
        bool          active        = false;
        std::string   path;
        std::uint32_t frameSlot     = 0; // slot whose fence retires the copy
        std::uint32_t width         = 0; // pixel geometry captured at copy time
        std::uint32_t height        = 0;
        bool          bgra          = false;
        std::uint64_t timelineValue = 0; // device timeline value, when used
    };
    PendingExport pendingExportResult_{};
    void          completePendingExportIfReady();
    void          flushPendingExport(bool allowBlocking);

    // Resource destruction is deferred to a later frame slot so GPU work
    // referencing the object has finished (queued per frame in flight).
    struct DeferredDestroy
    {
        MeshHandle    mesh    = invalidMesh;
        TextureHandle texture = invalidTexture;
    };
    std::vector<std::vector<DeferredDestroy>> deferredDestroys_;
    void                                      drainDeferredDestroys(std::uint32_t frameIndex);

    VkSampleCountFlagBits msaaSamples_ = VK_SAMPLE_COUNT_1_BIT; // clamped to device-supported counts

    bool  srgbOutput_      = false;
    float anisotropyLevel_ = 1.0f;

    // Set from the device profile; when true the backend uses vkCmdBeginRendering
    // instead of render passes/framebuffers.
    bool dynamicRenderingEnabled_ = false;
};

} // namespace renderer
} // namespace engine

#endif

#endif
