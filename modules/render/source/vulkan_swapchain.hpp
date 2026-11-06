// Swapchain management for the Vulkan backend: surface format/depth format
// selection, present-mode choice from [render] vsync, image views,
// framebuffers, depth attachment, per-image presentation semaphores,
// out-of-date detection/recreation, and the readback buffer used for PPM
// exports. Compiled only under ENGINE_RENDER_BACKEND_VULKAN.

#ifndef ENGINE_RENDER_MODULE_VULKAN_SWAPCHAIN_HPP
#define ENGINE_RENDER_MODULE_VULKAN_SWAPCHAIN_HPP

#if defined(ENGINE_RENDER_BACKEND_VULKAN)
#include "engine/render/capabilities.hpp"
#include "vulkan_device.hpp"

#include <cstdint>
#include <vector>
#include <vulkan/vulkan.h>
#include <vulkan/vulkan_core.h>

namespace engine
{
namespace render
{

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

    // Chooses the color format (sRGB preferred when srgbOutput_ is set, else
    // 8-bit UNORM with SRGB_NONLINEAR color space) and a depth format from
    // D32/D24S8/D16 support. Returns false if none exist.
    bool pickFormats();

    // Builds the swapchain, per-image views and presentation semaphores, the
    // depth attachment, and (when not using dynamic rendering) framebuffers.
    bool createObjects(VkRenderPass renderPass, std::uint32_t viewportWidth, std::uint32_t viewportHeight);

    void destroyObjects();

    bool recreate(VkRenderPass renderPass, std::uint32_t viewportWidth, std::uint32_t viewportHeight);

    // vkAcquireNextImageKHR wrapper; the caller reacts to OUT_OF_DATE/
    // SUBOPTIMAL by recreating or marking dirty.
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

    // With MSAA, colorView() is the multisample attachment and resolveView()
    // the per-image resolve target; otherwise both paths use views_[i].
    VkImageView colorView(std::uint32_t i) const
    {
        return sampleCount_ != VK_SAMPLE_COUNT_1_BIT ? msaaColorView_ : views_[i];
    }
    VkImageView depthView() const
    {
        return depthView_;
    }
    VkImageView resolveView(std::uint32_t i) const
    {
        return views_[i];
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
    // extent * 4 bytes; finishReadback maps it after the GPU copy completes
    // and writes the file (R/B swapped for BGRA surfaces).
    bool ensureReadbackBuffer();

    VkBuffer readbackBuffer() const
    {
        return readbackBuffer_;
    }

    void finishReadback(const char *filePath, RenderError *error);

private:
    bool pickDepthFormat(VkFormat *outFormat) const;

    const VulkanDeviceContext &device_;
    VkSurfaceKHR               surface_ = VK_NULL_HANDLE;
    bool                       vsync_   = true; // false = mailbox if available, else immediate, else FIFO

    VkFormat                   colorFormat_  = VK_FORMAT_UNDEFINED;
    VkColorSpaceKHR            colorSpace_   = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
    bool                       formatIsBgra_ = false; // readback must swap R/B
    VkFormat                   depthFormat_  = VK_FORMAT_UNDEFINED;
    VkExtent2D                 extent_       = {0, 0};
    VkSwapchainKHR             swapchain_    = VK_NULL_HANDLE;
    std::vector<VkImage>       images_;
    std::vector<VkImageView>   views_;
    std::vector<VkFramebuffer> framebuffers_;
    std::vector<VkFence>       imagesInFlight_;
    std::vector<VkSemaphore>   renderFinished_; // one per image: signals the submit done before present
    VkImage                    depthImage_      = VK_NULL_HANDLE;
    VmaAllocation              depthAllocation_ = nullptr;
    VkImageView                depthView_       = VK_NULL_HANDLE;

    bool dirty_ = false;

    bool srgbOutput_ = false; // prefer _SRGB surface formats when picking

    bool formatIsSrgb_ = false; // whether the picked format actually is sRGB

    VkSampleCountFlagBits sampleCount_         = VK_SAMPLE_COUNT_1_BIT;
    VkImage               msaaColorImage_      = VK_NULL_HANDLE; // created only when sampleCount_ > 1
    VmaAllocation         msaaColorAllocation_ = nullptr;
    VkImageView           msaaColorView_       = VK_NULL_HANDLE;

    VkBuffer      readbackBuffer_     = VK_NULL_HANDLE;
    VmaAllocation readbackAllocation_ = nullptr;
    VkDeviceSize  readbackSize_       = 0; // bytes; 0 until the first export
};

} // namespace render
} // namespace engine

#endif

#endif
