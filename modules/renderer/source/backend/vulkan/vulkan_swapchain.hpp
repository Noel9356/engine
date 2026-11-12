// Swapchain management for the Vulkan backend: surface format/depth format
// selection, present-mode choice from [renderer] vsync, image views,
// framebuffers, depth attachment, per-image presentation semaphores,
// out-of-date detection/recreation, and the readback buffer used for PPM
// exports. Compiled only under ENGINE_RENDERER_BACKEND_VULKAN.

#ifndef ENGINE_RENDERER_MODULE_VULKAN_SWAPCHAIN_HPP
#define ENGINE_RENDERER_MODULE_VULKAN_SWAPCHAIN_HPP

#if defined(ENGINE_RENDERER_BACKEND_VULKAN)
#include "engine/renderer/capabilities.hpp"
#include "vulkan_device.hpp"

#include <cstdint>
#include <vector>
#include <vulkan/vulkan.h>
#include <vulkan/vulkan_core.h>

namespace engine
{
namespace renderer
{

// Presentation-queue policy for the swapchain. Auto keeps the historical
// behaviour; the explicit modes exist so frame-latency behaviour is a
// configuration decision instead of a driver default.
enum class SwapchainPresentMode
{
    Auto,      // FIFO when vsync, else mailbox, else immediate
    Fifo,      // always tear-free, one image on screen, blocks on the compositor
    Mailbox,   // triple buffered, latency-bounded, never blocks the CPU for long
    Immediate, // no presentation queue at all: lowest latency, may tear
};

// One swapchain per renderer. pickFormats() must be called before
// createObjects(); recreate() tears everything down and rebuilds it (called
// on VK_ERROR_OUT_OF_DATE_KHR or a dirty flag from present/suboptimal).
class VulkanSwapchain
{
public:
    VulkanSwapchain(const VulkanDeviceContext &device, VkSurfaceKHR surface, bool vsync)
        : device_(device),
          surface_(surface),
          vsync_(vsync)
    {}

    // See presentMode_ below. Called before createObjects().
    void setPresentMode(SwapchainPresentMode mode) noexcept
    {
        presentMode_ = mode;
    }

    [[nodiscard]] VkPresentModeKHR chosenPresentMode() const noexcept
    {
        return chosenPresentMode_;
    }

    // Chooses the color format (sRGB preferred when srgbOutput_ is set, else
    // 8-bit UNORM with SRGB_NONLINEAR color space) and a depth format from
    // D32/D24S8/D16 support. Returns false if none exist.
    bool pickFormats();

    // Builds the swapchain, per-image views and presentation semaphores, the
    // per-frame-slot depth (and, with MSAA, multisample color) attachments,
    // and (when not using dynamic rendering) framebuffers. One attachment set
    // per frame in flight: a slot's submissions are serialized by its own
    // fence/timeline value, so two slots can execute concurrently and must
    // not share depth/MSAA attachments.
    bool createObjects(VkRenderPass renderPass, std::uint32_t viewportWidth, std::uint32_t viewportHeight,
        std::uint32_t framesInFlight);

    void destroyObjects();

    bool recreate(VkRenderPass renderPass, std::uint32_t viewportWidth, std::uint32_t viewportHeight,
        std::uint32_t framesInFlight);

    // vkAcquireNextImageKHR wrapper with a bounded wait; the caller reacts to
    // OUT_OF_DATE/SUBOPTIMAL by recreating or marking dirty and to VK_TIMEOUT
    // by skipping the frame (no free image yet; semaphore left unsignaled).
    VkResult acquire(VkSemaphore imageAvailable, std::uint32_t *outImageIndex);

    VkSemaphore renderFinishedSemaphore(std::uint32_t imageIndex) const
    {
        return imageIndex < renderFinished_.size() ? renderFinished_[imageIndex] : VK_NULL_HANDLE;
    }

    VkResult present(VkSemaphore renderFinished, std::uint32_t imageIndex);

    bool dirty() const
    {
        return dirty_; // set by OUT_OF_DATE/SUBOPTIMAL present results; checked each beginFrame
    }

    void markDirty()
    {
        dirty_ = true;
    }

    VkFormat format() const
    {
        return colorFormat_;
    }

    bool formatIsBgra() const
    {
        return formatIsBgra_;
    }

    VkFormat depthFormat() const
    {
        return depthFormat_;
    }

    void setSrgbOutput(bool enabled) noexcept
    {
        srgbOutput_ = enabled;
    }

    bool formatIsSrgb() const noexcept
    {
        return formatIsSrgb_;
    }

    void setSampleCount(VkSampleCountFlagBits samples) noexcept
    {
        sampleCount_ = samples;
    }
    VkSampleCountFlagBits sampleCount() const noexcept
    {
        return sampleCount_;
    }

    // With MSAA, colorView() is the slot's multisample attachment and
    // resolveView() the per-image resolve target; without MSAA the color
    // target is views_[imageIndex] for any slot.
    VkImageView colorView(std::uint32_t frameSlot, std::uint32_t imageIndex) const
    {
        if (sampleCount_ != VK_SAMPLE_COUNT_1_BIT)
            return frameSlot < msaaColorViews_.size() ? msaaColorViews_[frameSlot] : VK_NULL_HANDLE;
        return imageIndex < views_.size() ? views_[imageIndex] : VK_NULL_HANDLE;
    }

    // Depth attachment for the given frame slot (one per frame in flight).
    VkImageView depthView(std::uint32_t frameSlot) const
    {
        return frameSlot < depthViews_.size() ? depthViews_[frameSlot] : VK_NULL_HANDLE;
    }
    VkImageView resolveView(std::uint32_t imageIndex) const
    {
        return imageIndex < views_.size() ? views_[imageIndex] : VK_NULL_HANDLE;
    }

    // Legacy render-pass path: framebuffer for (slot, image); the depth
    // attachment baked into it is the slot's own.
    VkFramebuffer framebuffer(std::uint32_t frameSlot, std::uint32_t imageIndex) const
    {
        const std::size_t imageCount = views_.size();
        if (imageCount == 0 || frameSlot * imageCount + imageIndex >= framebuffers_.size())
            return VK_NULL_HANDLE;
        return framebuffers_[frameSlot * imageCount + imageIndex];
    }

    VkExtent2D extent() const
    {
        return extent_;
    }

    const std::vector<VkImage> &images() const
    {
        return images_;
    }

    VkSwapchainKHR swapchain() const
    {
        return swapchain_;
    }

    const std::vector<VkFramebuffer> &framebuffers() const
    {
        return framebuffers_;
    }

    // Fences (or null with the timeline path) recorded per image so acquire
    // can wait for a still-in-flight image without a frame slot tie.
    const std::vector<VkFence> &imagesInFlight() const
    {
        return imagesInFlight_;
    }

    void setImagesInFlight(std::uint32_t imageIndex, VkFence fence)
    {
        if (imageIndex < imagesInFlight_.size())
        {
            imagesInFlight_[imageIndex] = fence;
        }
    }

    // PPM export support: lazily (re)creates a host-visible buffer of
    // extent * 4 bytes; finishReadback maps it after the copy's submission
    // completed on the GPU (the copy itself is recorded by the backend into
    // the frame's command buffer) and writes the file (R/B swapped for BGRA
    // surfaces). The pixel geometry is passed in by the caller because the
    // swapchain may have resized between the copy and the flush - the
    // captured values belong to the recorded copy.
    bool ensureReadbackBuffer();

    VkBuffer readbackBuffer() const
    {
        return readbackBuffer_;
    }

    void finishReadback(
        const char *filePath, std::uint32_t width, std::uint32_t height, bool bgra, RenderError *error);

private:
    bool pickDepthFormat(VkFormat *outFormat) const;

    const VulkanDeviceContext &device_;
    VkSurfaceKHR               surface_ = VK_NULL_HANDLE;
    bool                       vsync_   = false; // false = mailbox if available, else immediate, else FIFO

    // Explicit presentation-queue policy. Auto preserves the historical
    // behaviour (FIFO under vsync, mailbox-else-immediate otherwise); the
    // other values force a mode so latency behaviour can be pinned by
    // configuration rather than left to driver defaults.
    SwapchainPresentMode       presentMode_       = SwapchainPresentMode::Auto;
    VkPresentModeKHR           chosenPresentMode_ = VK_PRESENT_MODE_FIFO_KHR;

    VkFormat                   colorFormat_  = VK_FORMAT_UNDEFINED;
    VkColorSpaceKHR            colorSpace_   = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
    bool                       formatIsBgra_ = false; // readback must swap R/B
    bool                       transferSourceSupported_ = false;
    VkFormat                   depthFormat_  = VK_FORMAT_UNDEFINED;
    VkExtent2D                extent_       = {0, 0};
    VkSwapchainKHR            swapchain_    = VK_NULL_HANDLE;
    std::vector<VkImage>      images_;
    std::vector<VkImageView>  views_;
    std::vector<VkFramebuffer> framebuffers_;       // [slot * imageCount + image]; empty on dynamic rendering
    std::vector<VkFence>      imagesInFlight_;
    std::vector<VkSemaphore>  renderFinished_; // one per image: signals the submit done before present

    // Depth and (with MSAA) multisample color attachments, one set per frame
    // in flight: slot N's submissions are serialized by slot N's fence, so
    // two concurrently executing slots never write the same attachment.
    std::vector<VkImage>       depthImages_;
    std::vector<VmaAllocation> depthAllocations_;
    std::vector<VkImageView>   depthViews_;
    std::vector<VkImage>       msaaColorImages_;
    std::vector<VmaAllocation> msaaColorAllocations_;
    std::vector<VkImageView>   msaaColorViews_;

    bool dirty_ = false;

    bool srgbOutput_ = false; // prefer _SRGB surface formats when picking

    bool formatIsSrgb_ = false; // whether the picked format actually is sRGB

    VkSampleCountFlagBits sampleCount_ = VK_SAMPLE_COUNT_1_BIT;

    VkBuffer      readbackBuffer_     = VK_NULL_HANDLE;
    VmaAllocation readbackAllocation_ = nullptr;
    VkDeviceSize  readbackSize_       = 0; // bytes; 0 until the first export
};

} // namespace renderer
} // namespace engine

#endif

#endif
