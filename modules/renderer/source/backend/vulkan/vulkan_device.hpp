// Vulkan device bring-up and shared GPU plumbing: instance/surface creation,
// physical-device selection, VMA allocator, a timeline semaphore handle, and
// the asynchronous staging-buffer upload queue shared by all Vulkan
// sub-units. Compiled only under ENGINE_RENDERER_BACKEND_VULKAN.

#ifndef ENGINE_RENDERER_MODULE_VULKAN_DEVICE_HPP
#define ENGINE_RENDERER_MODULE_VULKAN_DEVICE_HPP

#if defined(ENGINE_RENDERER_BACKEND_VULKAN)
#include "engine/renderer/capabilities.hpp"
#include "engine/renderer/device_profile.hpp"
#include "engine/renderer/memory_budget_tracker.hpp"
#include "engine/kernel/profiling.hpp"
#include "render_shared.hpp"
#include "staging_pool.hpp"

#include <vk_mem_alloc.h>
#include <vulkan/vulkan.h>
#include <vulkan/vulkan_core.h>

#if !defined(VK_USE_PLATFORM_WIN32_KHR) && !defined(VK_USE_PLATFORM_XLIB_KHR)
#if defined(_WIN32)
#define VK_USE_PLATFORM_WIN32_KHR
#elif defined(__linux__)
#define VK_USE_PLATFORM_XLIB_KHR
#endif
#endif

#include <engine/window/capabilities.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace engine
{
namespace renderer
{

// Memory placement hint for createBuffer(); maps onto VMA usage flags.
enum class BufferMemoryIntent : std::uint8_t
{
    DeviceLocal,         // GPU-only memory, filled via staging copies
    HostSequentialWrite, // host-visible, CPU writes linearly (uniform/instance/overlay buffers)
    HostRandomAccess     // host-visible, CPU also reads back (screenshot readback)
};

// One host-to-device copy request consumed by copyHostToDevice(); data is
// borrowed for the duration of the call.
struct BufferUpload
{
    VkBuffer      destination       = VK_NULL_HANDLE;
    const void   *data              = nullptr;
    VkDeviceSize  size              = 0; // bytes, must be a multiple of 4
    VkAccessFlags destinationAccess = 0; // barrier target after the copy (e.g. vertex-attribute read)
};

// Staging memory backing one upload. Exactly one of the two members is
// active: `pooled` (span.block != invalidBlock) points into a persistently
// mapped, fence-recycled pool block, or the one-off buffer pair is set for
// uploads the pool declined (oversized requests, pool pressure). The device
// context frees both paths when the copy fence signals.
struct UploadStaging
{
    StagingPool::Span span{}; // active when span.block != invalidBlock

    VkBuffer      oneOffBuffer     = VK_NULL_HANDLE;
    VmaAllocation oneOffAllocation = nullptr;

    [[nodiscard]] VkBuffer stagingBuffer() const noexcept
    {
        return span.block != StagingPool::invalidBlock ? pooledBuffer : oneOffBuffer;
    }

    // Pooled path extras (valid only when span.block is set).
    VkBuffer      pooledBuffer = VK_NULL_HANDLE;
    VmaAllocation pooledAllocation = nullptr;
    void         *pooledHostBase = nullptr; // block base; write at hostBase + span.offset
};

// Owns everything device-scope. create() builds instance -> surface
// (win32/xlib, or VK_EXT_headless_surface when the window provides no native
// handles) -> physical device -> logical device -> VMA allocator. The queue
// mutex serializes every submit/present on the single graphics queue; the
// upload system runs its own one-shot command buffers guarded by fences.
class VulkanDeviceContext
{
public:
    static std::unique_ptr<VulkanDeviceContext> create(window::WindowHandle surface, bool validation,
        DevicePreference preference, std::uint32_t maxPendingUploads, window::VulkanSurfaceCapability *vulkanSurface,
        RenderError *error);

    ~VulkanDeviceContext();

    VulkanDeviceContext(const VulkanDeviceContext &)            = delete;
    VulkanDeviceContext &operator=(const VulkanDeviceContext &) = delete;

    VkInstance instance() const
    {
        return instance_;
    }
    VkPhysicalDevice physicalDevice() const
    {
        return physicalDevice_;
    }
    VkDevice device() const
    {
        return device_;
    }
    VkQueue queue() const
    {
        return queue_;
    }
    std::uint32_t graphicsFamily() const
    {
        return graphicsFamily_;
    }
    VkSurfaceKHR surface() const
    {
        return surface_;
    }
    VkCommandPool uploadPool() const
    {
        return uploadPool_;
    }
    VmaAllocator allocator() const
    {
        return allocator_;
    }
    std::mutex &queueMutex() const
    {
        return queueMutex_; // guards queue submission, upload bookkeeping, and wait-idle
    }

    // Device timeline semaphore used to retire frame submissions. Reserved
    // by the interface, but create() currently leaves it null, so all frame
    // sync falls back to per-slot fences (see VulkanFrameScheduler).
    VkSemaphore timelineSemaphore() const noexcept
    {
        return timelineSemaphore_;
    }
    std::uint64_t acquireTimelineValue() const noexcept
    {
        return ++nextTimelineValue_; // atomic, monotonically increasing
    }
    std::uint64_t completedTimelineValue() const;
    bool          waitTimeline(std::uint64_t value) const;
    // Bounded variant used by the frame path: a wedged or glacially slow
    // driver must never pin the presenting thread forever; the caller skips
    // the frame instead and the window loop keeps polling events.
    bool waitTimelineFor(std::uint64_t value, std::uint64_t timeoutNanoseconds) const;

    const std::string &vendor() const
    {
        return vendor_;
    }
    const std::string &renderer() const
    {
        return renderer_;
    }
    const std::string &version() const
    {
        return version_;
    }
    DeviceClass deviceClass() const
    {
        return deviceClass_;
    }
    const DeviceProfile &profile() const
    {
        return profile_;
    }

    // Advances VMA's frame index and periodically refreshes the cached
    // device-local memory budget (every memoryBudgetUpdateIntervalFrames).
    // Each refresh also feeds the budget pressure tracker; entering or
    // leaving pressure logs exactly one diagnostic line (no per-frame spam).
    void beginFrame(std::uint32_t memoryBudgetUpdateIntervalFrames) noexcept;
    void refreshMemoryBudget() noexcept;

    // Peak/minimum-headroom/pressure history over the sampled budgets.
    const MemoryBudgetTracker &memoryBudgetTracker() const noexcept
    {
        return memoryBudgetTracker_;
    }

    // Device limit for sampler anisotropy (limits.maxSamplerAnisotropy);
    // sampler creation must clamp the configured level against it.
    float maxSamplerAnisotropy() const noexcept
    {
        return maxSamplerAnisotropy_;
    }

    void destroy();

    // VMA wrappers; all return false on allocation failure and never throw.
    bool createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, BufferMemoryIntent memoryIntent, VkBuffer *outBuffer,
        VmaAllocation *outAllocation) const;
    void destroyBuffer(VkBuffer buffer, VmaAllocation allocation) const noexcept;
    bool mapAllocation(VmaAllocation allocation, void **outMapped) const;
    bool flushAllocation(VmaAllocation allocation) const;
    bool flushAllocation(VmaAllocation allocation, VkDeviceSize offset, VkDeviceSize size) const;
    bool invalidateAllocation(VmaAllocation allocation) const;
    void unmapAllocation(VmaAllocation allocation) const noexcept;

    // samples selects the image sample count (1 for everything except MSAA
    // attachments; the caller is responsible for capability checking).
    bool createImage(std::uint32_t width, std::uint32_t height, std::uint32_t mipLevels, VkFormat format,
        VkImageUsageFlags usage, VkImage *outImage, VmaAllocation *outAllocation,
        VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT) const;
    void destroyImage(VkImage image, VmaAllocation allocation) const noexcept;

    // Persistently-mapped staging pool for host-to-device copies (policy in
    // staging_pool.hpp). acquireStagingSpan returns a mapped range served
    // from a pool block, or false when the caller must use its one-off path
    // (oversized request, pool at its byte cap, or block creation failure);
    // it may allocate a new pool block, which counts as pool growth and is
    // bounded by the pool's byte cap. commitStagingSpan flushes the span's
    // aligned range (a no-op internally on host-coherent memory). Callers
    // must hold the queue mutex for both.
    bool acquireStagingSpan(VkDeviceSize size, UploadStaging *outStaging) const;
    void commitStagingSpan(const UploadStaging &staging) const;
    // Releases staging memory that will never be submitted (command-buffer
    // allocation or enqueue failure): pooled spans go back to the policy,
    // one-off buffers are destroyed. Queue mutex held.
    void discardUploadStaging(const UploadStaging &staging) const;

    // Asynchronous upload queue. enqueueUploadLocked submits a one-shot copy
    // command buffer with its own fence and the given staging memory
    // (pooled range or one-off buffer); collect* reaps finished ones and
    // recycles/frees their staging memory, and waitForUploads blocks until
    // the queue drains. The pending count is capped at
    // maximumPendingUploads_ (oldest is waited on when full). Callers must
    // hold the queue mutex for the *Locked variants.
    bool enqueueUploadLocked(VkCommandBuffer command, const UploadStaging &staging) const;
    void collectCompletedUploads() const;
    void waitForUploads() const;

    // Optional diagnostics hook: the upload queue records its oldest-fence
    // wait (queue at capacity) and whole-queue drain wait into the shared
    // profiler's percentile histograms. Null simply disables recording.
    void setProfiler(engine::PerformanceProfiler *profiler) noexcept
    {
        profiler_ = profiler;
    }

    // Packs every upload into one staging buffer, copies them to their
    // destinations, and barriers to the requested access flags; the copy is
    // asynchronous - safe to forget, cleaned up by collectCompletedUploads.
    bool copyHostToDevice(const BufferUpload *uploads, std::uint32_t uploadCount) const;

private:
    VulkanDeviceContext() = default;

    VkInstance                         instance_       = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT           messenger_      = VK_NULL_HANDLE;
    VkSurfaceKHR                       surface_        = VK_NULL_HANDLE;
    VkPhysicalDevice                   physicalDevice_ = VK_NULL_HANDLE;
    std::uint32_t                      graphicsFamily_ = 0;
    VkDevice                           device_         = VK_NULL_HANDLE;
    VkQueue                            queue_          = VK_NULL_HANDLE;
    VmaAllocator                       allocator_      = nullptr;
    mutable std::mutex                 queueMutex_;
    VkSemaphore                        timelineSemaphore_ = VK_NULL_HANDLE;
    mutable std::atomic<std::uint64_t> nextTimelineValue_{0};

    struct PendingUpload
    {
        VkCommandBuffer command           = VK_NULL_HANDLE; // freed back to uploadPool_ when done
        VkFence         fence             = VK_NULL_HANDLE; // signals when the copy is safe to release
        VkBuffer        stagingBuffer     = VK_NULL_HANDLE; // one-off path: kept alive until the copy lands
        VmaAllocation   stagingAllocation = nullptr;
        StagingPool::Span pooledSpan{};    // pooled path: recycled through stagingPool_ on completion
    };
    VkCommandPool                      uploadPool_            = VK_NULL_HANDLE;
    std::uint32_t                      maximumPendingUploads_ = 64;
    mutable std::vector<PendingUpload> pendingUploads_;
    mutable std::atomic_bool           hasPendingUploads_{false};
    engine::PerformanceProfiler       *profiler_ = nullptr; // set via setProfiler; never owned
    void                               collectCompletedUploadsLocked(bool force) const;

    // Waits until every fence in the set signals (vkWaitForFences with an
    // unbounded timeout). Non-timeout results (device lost and friends)
    // return immediately; the caller's existing error handling takes over.
    // Queue mutex held.
    void waitForUploadFences(VkFence const *fences, std::uint32_t fenceCount) const;

    // Staging pool storage: the policy lives in stagingPool_, the Vulkan
    // resources it hands out live here, keyed by block token (index
    // token - 1 would be wrong once blocks are dropped, so tokens are
    // looked up linearly - live blocks are few). The pool is re-created at
    // device bring-up with the device's nonCoherentAtomSize as the span
    // alignment; the declaration initializer only provides safe defaults.
    static constexpr std::uint64_t kMaxPooledStagingBlockBytes = 16ull * 1024ull * 1024ull;
    static constexpr std::uint64_t kMaxPooledStagingBytes      = 64ull * 1024ull * 1024ull;
    struct StagingBlockStorage
    {
        StagingPool::BlockToken token      = StagingPool::invalidBlock;
        VkBuffer                buffer     = VK_NULL_HANDLE;
        VmaAllocation           allocation = nullptr;
        void                   *mapped     = nullptr; // persistent mapping, whole block
        VkDeviceSize            capacity   = 0;
    };
    mutable StagingPool                      stagingPool_{1, kMaxPooledStagingBlockBytes, kMaxPooledStagingBytes};
    mutable std::vector<StagingBlockStorage> stagingStorage_;
    VkDeviceSize                             nonCoherentAtomSize_ = 1;
    bool createMappedStagingBlock(VkDeviceSize capacity, StagingBlockStorage *outBlock) const;
    void destroyStagingBlock(const StagingBlockStorage &block) const noexcept;
    void releaseUploadStaging(const PendingUpload &upload) const;
    [[nodiscard]] const StagingBlockStorage *stagingBlockFor(StagingPool::BlockToken token) const noexcept;

    std::string                      vendor_;
    std::string                      renderer_;
    std::string                      version_;
    DeviceClass                      deviceClass_ = DeviceClass::Unknown;
    DeviceProfile                    profile_{};
    VkPhysicalDeviceMemoryProperties memoryProperties_{};
    bool                             memoryPropertiesCached_ = false;
    std::uint32_t                    memoryBudgetFrameIndex_ = 0;
    MemoryBudgetTracker              memoryBudgetTracker_{};
    float                            maxSamplerAnisotropy_   = 1.0f;
};

} // namespace renderer
} // namespace engine

#endif

#endif
