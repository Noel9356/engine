// Vulkan device bring-up and shared GPU plumbing: instance/surface creation,
// physical-device selection, VMA allocator, a timeline semaphore handle, and
// the asynchronous staging-buffer upload queue shared by all Vulkan
// sub-units. Compiled only under ENGINE_RENDER_BACKEND_VULKAN.

#ifndef ENGINE_RENDER_MODULE_VULKAN_DEVICE_HPP
#define ENGINE_RENDER_MODULE_VULKAN_DEVICE_HPP

#if defined(ENGINE_RENDER_BACKEND_VULKAN)
#include "engine/render/capabilities.hpp"
#include "engine/render/device_profile.hpp"
#include "render_shared.hpp"

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
namespace render
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
    void beginFrame(std::uint32_t memoryBudgetUpdateIntervalFrames) noexcept;
    void refreshMemoryBudget() noexcept;

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

    bool createImage(std::uint32_t width, std::uint32_t height, std::uint32_t mipLevels, VkFormat format,
        VkImageUsageFlags usage, VkImage *outImage, VmaAllocation *outAllocation) const;
    void destroyImage(VkImage image, VmaAllocation allocation) const noexcept;

    // Asynchronous upload queue. enqueueUploadLocked submits a one-shot copy
    // command buffer with its own fence and staging buffer; collect* reaps
    // finished ones, and waitForUploads blocks until the queue drains. The
    // pending count is capped at maximumPendingUploads_ (oldest is waited on
    // when full). Callers must hold the queue mutex for the *Locked variants.
    bool enqueueUploadLocked(VkCommandBuffer command, VkBuffer stagingBuffer, VmaAllocation stagingAllocation) const;
    void collectCompletedUploads() const;
    void waitForUploads() const;

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
        VkBuffer        stagingBuffer     = VK_NULL_HANDLE; // kept alive until the copy lands
        VmaAllocation   stagingAllocation = nullptr;
    };
    VkCommandPool                      uploadPool_            = VK_NULL_HANDLE;
    std::uint32_t                      maximumPendingUploads_ = 64;
    mutable std::vector<PendingUpload> pendingUploads_;
    mutable std::atomic_bool           hasPendingUploads_{false};
    void                               collectCompletedUploadsLocked(bool force) const;

    std::string                      vendor_;
    std::string                      renderer_;
    std::string                      version_;
    DeviceClass                      deviceClass_ = DeviceClass::Unknown;
    DeviceProfile                    profile_{};
    VkPhysicalDeviceMemoryProperties memoryProperties_{};
    bool                             memoryPropertiesCached_ = false;
    std::uint32_t                    memoryBudgetFrameIndex_ = 0;
};

} // namespace render
} // namespace engine

#endif

#endif
