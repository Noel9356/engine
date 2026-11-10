

// Swapchain implementation: format picking, creation, per-image views and
// semaphores, depth attachment, recreation, present, and PPM readback.

#if defined(ENGINE_RENDERER_BACKEND_VULKAN)

#include "vulkan_swapchain.hpp"
#include "engine/renderer/capabilities.hpp"
#include "render_shared.hpp"
#include "vulkan_device.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <mutex>
#include <vector>
#include <vulkan/vulkan_core.h>

namespace engine
{
namespace renderer
{

// First supported depth format wins; D32 preferred for precision.
bool VulkanSwapchain::pickDepthFormat(VkFormat *outFormat) const
{
    const VkFormat candidates[] = {VK_FORMAT_D32_SFLOAT, VK_FORMAT_D24_UNORM_S8_UINT, VK_FORMAT_D16_UNORM};
    for (VkFormat candidate : candidates)
    {
        VkFormatProperties properties;
        vkGetPhysicalDeviceFormatProperties(device_.physicalDevice(), candidate, &properties);
        if ((properties.optimalTilingFeatures & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT) != 0u)
        {
            *outFormat = candidate;
            return true;
        }
    }
    return false;
}

bool VulkanSwapchain::pickFormats()
{
    std::uint32_t formatCount = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(device_.physicalDevice(), surface_, &formatCount, nullptr);
    std::vector<VkSurfaceFormatKHR> formats(formatCount);
    if (formatCount == 0
        || vkGetPhysicalDeviceSurfaceFormatsKHR(device_.physicalDevice(), surface_, &formatCount, formats.data())
               != VK_SUCCESS)
    {
        return false;
    }
    VkSurfaceFormatKHR chosenFormat = formats[0];

    bool gotSrgb = false;
    if (srgbOutput_)
    {
        for (const VkSurfaceFormatKHR &candidate : formats)
        {
            if ((candidate.format == VK_FORMAT_B8G8R8A8_SRGB || candidate.format == VK_FORMAT_R8G8B8A8_SRGB)
                && candidate.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
            {
                chosenFormat = candidate;
                gotSrgb      = true;
                break;
            }
        }
    }
    if (!gotSrgb)
    {
        for (const VkSurfaceFormatKHR &candidate : formats)
        {
            if ((candidate.format == VK_FORMAT_B8G8R8A8_UNORM || candidate.format == VK_FORMAT_R8G8B8A8_UNORM)
                && candidate.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
            {
                chosenFormat = candidate;
                break;
            }
        }
    }
    colorFormat_  = chosenFormat.format;
    colorSpace_   = chosenFormat.colorSpace;
    formatIsBgra_ = (chosenFormat.format == VK_FORMAT_B8G8R8A8_UNORM || chosenFormat.format == VK_FORMAT_B8G8R8A8_SRGB);

    formatIsSrgb_ = gotSrgb;
    return pickDepthFormat(&depthFormat_);
}

// Present mode: FIFO always exists and is the vsync choice; without vsync,
// mailbox (or immediate as fallback) avoids the FIFO latency at full speed.
bool VulkanSwapchain::createObjects(VkRenderPass renderPass, std::uint32_t viewportWidth, std::uint32_t viewportHeight,
    std::uint32_t framesInFlight)
{
    // One attachment set per frame in flight (at least one): two slots can
    // execute concurrently, so a single shared depth/MSAA attachment would be
    // a write/write race whenever frames-in-flight > 1.
    const std::uint32_t slotCount = std::max(framesInFlight, 1u);
    if (renderPass == VK_NULL_HANDLE && !device_.profile().dynamicRenderingAvailable)
    {
        std::fprintf(stderr, "renderer module: dynamic rendering required but unavailable\n");
        return false;
    }
    VkSurfaceCapabilitiesKHR capabilities;
    if (vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device_.physicalDevice(), surface_, &capabilities) != VK_SUCCESS)
    {
        return false;
    }

    VkPresentModeKHR presentMode = VK_PRESENT_MODE_FIFO_KHR;
    if (!vsync_)
    {
        std::uint32_t presentModeCount = 0;
        vkGetPhysicalDeviceSurfacePresentModesKHR(device_.physicalDevice(), surface_, &presentModeCount, nullptr);
        std::vector<VkPresentModeKHR> presentModes(presentModeCount);
        if (presentModeCount > 0
            && vkGetPhysicalDeviceSurfacePresentModesKHR(
                   device_.physicalDevice(), surface_, &presentModeCount, presentModes.data())
                   == VK_SUCCESS)
        {
            const bool hasMailbox   = std::any_of(presentModes.begin(),
                presentModes.end(),
                [](VkPresentModeKHR mode) { return mode == VK_PRESENT_MODE_MAILBOX_KHR; });
            const bool hasImmediate = std::any_of(presentModes.begin(),
                presentModes.end(),
                [](VkPresentModeKHR mode) { return mode == VK_PRESENT_MODE_IMMEDIATE_KHR; });
            if (hasMailbox)
            {
                presentMode = VK_PRESENT_MODE_MAILBOX_KHR;
            }
            else if (hasImmediate)
            {
                presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR;
            }
        }
    }

    // Extent: currentExtent is authoritative except when the compositor
    // reports UINT32_MAX (then the requested viewport, clamped to limits).
    VkExtent2D resolvedExtent = capabilities.currentExtent;
    if (resolvedExtent.width == UINT32_MAX || resolvedExtent.height == UINT32_MAX)
    {
        resolvedExtent.width  = viewportWidth != 0 ? viewportWidth : capabilities.minImageExtent.width;
        resolvedExtent.height = viewportHeight != 0 ? viewportHeight : capabilities.minImageExtent.height;
    }
    resolvedExtent.width
        = std::clamp(resolvedExtent.width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
    resolvedExtent.height
        = std::clamp(resolvedExtent.height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height);
    if (resolvedExtent.width == 0 || resolvedExtent.height == 0)
    {
        return false;
    }

    if ((capabilities.supportedUsageFlags & VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT) == 0u)
    {
        return false;
    }
    transferSourceSupported_ = (capabilities.supportedUsageFlags & VK_IMAGE_USAGE_TRANSFER_SRC_BIT) != 0u;
    extent_ = resolvedExtent;

    std::uint32_t imageCount = capabilities.minImageCount + 1; // one extra image for low-latency presentation
    if (capabilities.maxImageCount > 0 && imageCount > capabilities.maxImageCount)
    {
        imageCount = capabilities.maxImageCount;
    }

    // PPM exports use TRANSFER_SRC when the surface supports it; normal
    // rendering only requires COLOR_ATTACHMENT.
    VkImageUsageFlags imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    if (transferSourceSupported_)
    {
        imageUsage |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    }
    const VkSwapchainCreateInfoKHR swapchainInfo{
        .sType                 = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR,
        .pNext                 = nullptr,
        .flags                 = 0,
        .surface               = surface_,
        .minImageCount         = imageCount,
        .imageFormat           = colorFormat_,
        .imageColorSpace       = colorSpace_,
        .imageExtent           = resolvedExtent,
        .imageArrayLayers      = 1,
        .imageUsage            = imageUsage,
        .imageSharingMode      = VK_SHARING_MODE_EXCLUSIVE,
        .queueFamilyIndexCount = 0,
        .pQueueFamilyIndices   = nullptr,
        .preTransform          = capabilities.currentTransform,
        .compositeAlpha        = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR,
        .presentMode           = presentMode,
        .clipped               = VK_TRUE,
        .oldSwapchain          = VK_NULL_HANDLE,
    };
    if (vkCreateSwapchainKHR(device_.device(), &swapchainInfo, nullptr, &swapchain_) != VK_SUCCESS)
    {
        return false;
    }

    std::uint32_t imageCountActual = 0;
    vkGetSwapchainImagesKHR(device_.device(), swapchain_, &imageCountActual, nullptr);
    images_.resize(imageCountActual);
    if (vkGetSwapchainImagesKHR(device_.device(), swapchain_, &imageCountActual, images_.data()) != VK_SUCCESS)
    {
        return false;
    }

    imagesInFlight_.assign(imageCountActual, VK_NULL_HANDLE);

    renderFinished_.assign(imageCountActual, VK_NULL_HANDLE);
    const VkSemaphoreCreateInfo presentSemaphoreInfo{
        .sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO,
        .pNext = nullptr,
        .flags = 0,
    };
    for (std::uint32_t index = 0; index < imageCountActual; ++index)
    {
        if (vkCreateSemaphore(device_.device(), &presentSemaphoreInfo, nullptr, &renderFinished_[index]) != VK_SUCCESS)
        {
            std::fprintf(stderr, "renderer module: could not create a swapchain presentation semaphore\n");
            return false;
        }
    }

    views_.resize(imageCountActual);
    for (std::uint32_t index = 0; index < imageCountActual; ++index)
    {
        const VkImageViewCreateInfo viewInfo{
            .sType    = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
            .pNext    = nullptr,
            .flags    = 0,
            .image    = images_[index],
            .viewType = VK_IMAGE_VIEW_TYPE_2D,
            .format   = colorFormat_,
            .components
            = {VK_COMPONENT_SWIZZLE_IDENTITY,
                VK_COMPONENT_SWIZZLE_IDENTITY,
                VK_COMPONENT_SWIZZLE_IDENTITY,
                VK_COMPONENT_SWIZZLE_IDENTITY},
            .subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
        };
        if (vkCreateImageView(device_.device(), &viewInfo, nullptr, &views_[index]) != VK_SUCCESS)
        {
            return false;
        }
    }

    depthImages_.assign(slotCount, VK_NULL_HANDLE);
    depthAllocations_.assign(slotCount, nullptr);
    depthViews_.assign(slotCount, VK_NULL_HANDLE);
    msaaColorImages_.assign(slotCount, VK_NULL_HANDLE);
    msaaColorAllocations_.assign(slotCount, nullptr);
    msaaColorViews_.assign(slotCount, VK_NULL_HANDLE);
    for (std::uint32_t slot = 0; slot < slotCount; ++slot)
    {
        if (!device_.createImage(extent_.width,
                extent_.height,
                1,
                depthFormat_,
                VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
                &depthImages_[slot],
                &depthAllocations_[slot],
                sampleCount_))
        {
            std::fprintf(stderr, "renderer module: could not allocate the depth attachment for frame slot %u\n", slot);
            return false;
        }
        const VkImageViewCreateInfo depthViewInfo{
            .sType    = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
            .pNext    = nullptr,
            .flags    = 0,
            .image    = depthImages_[slot],
            .viewType = VK_IMAGE_VIEW_TYPE_2D,
            .format   = depthFormat_,
            .components
            = {VK_COMPONENT_SWIZZLE_IDENTITY,
                VK_COMPONENT_SWIZZLE_IDENTITY,
                VK_COMPONENT_SWIZZLE_IDENTITY,
                VK_COMPONENT_SWIZZLE_IDENTITY},
            .subresourceRange = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1},
        };
        if (vkCreateImageView(device_.device(), &depthViewInfo, nullptr, &depthViews_[slot]) != VK_SUCCESS)
        {
            std::fprintf(stderr, "renderer module: could not create the depth attachment view for frame slot %u\n", slot);
            return false;
        }

        // MSAA color attachment for the dynamic-rendering path: a multisample
        // image resolved into the swapchain images by vkCmdEndRendering's
        // resolve mode. TRANSIENT_ATTACHMENT lets tiler GPUs keep it entirely
        // in tile memory (its contents are never read by anything but the
        // resolve), and lazily-allocated memory is a pure saving here.
        if (sampleCount_ != VK_SAMPLE_COUNT_1_BIT)
        {
            if (!device_.createImage(extent_.width,
                    extent_.height,
                    1,
                    colorFormat_,
                    VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT,
                    &msaaColorImages_[slot],
                    &msaaColorAllocations_[slot],
                    sampleCount_))
            {
                std::fprintf(stderr, "renderer module: could not allocate the MSAA color attachment for frame slot %u\n", slot);
                return false;
            }
            const VkImageViewCreateInfo msaaViewInfo{
                .sType    = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
                .pNext    = nullptr,
                .flags    = 0,
                .image    = msaaColorImages_[slot],
                .viewType = VK_IMAGE_VIEW_TYPE_2D,
                .format   = colorFormat_,
                .components
                = {VK_COMPONENT_SWIZZLE_IDENTITY,
                    VK_COMPONENT_SWIZZLE_IDENTITY,
                    VK_COMPONENT_SWIZZLE_IDENTITY,
                    VK_COMPONENT_SWIZZLE_IDENTITY},
                .subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
            };
            if (vkCreateImageView(device_.device(), &msaaViewInfo, nullptr, &msaaColorViews_[slot]) != VK_SUCCESS)
            {
                std::fprintf(stderr, "renderer module: could not create the MSAA color attachment view for frame slot %u\n", slot);
                return false;
            }
        }
    }

    // Depth image starts UNDEFINED; a one-shot command buffer transitions it
    // to DEPTH_ATTACHMENT_OPTIMAL and waits on a fence before continuing.
    VkCommandBufferAllocateInfo allocateInfo{
        .sType              = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
        .pNext              = nullptr,
        .commandPool        = device_.uploadPool(),
        .level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
        .commandBufferCount = 1,
    };
    VkCommandBuffer command = VK_NULL_HANDLE;
    VkResult        result  = VK_ERROR_UNKNOWN;
    {
        std::lock_guard queueLock(device_.queueMutex());
        result = vkAllocateCommandBuffers(device_.device(), &allocateInfo, &command);
    }
    if (result != VK_SUCCESS)
    {
        std::fprintf(stderr,
            "renderer module: could not allocate the depth-transition command buffer (%d)\n",
            static_cast<int>(result));
        return false;
    }

    const VkCommandBufferBeginInfo beginInfo{
        .sType            = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
        .pNext            = nullptr,
        .flags            = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT,
        .pInheritanceInfo = nullptr,
    };
    result = vkBeginCommandBuffer(command, &beginInfo);
    if (result == VK_SUCCESS)
    {
        std::vector<VkImageMemoryBarrier> depthBarriers;
        depthBarriers.reserve(slotCount);
        for (std::uint32_t slot = 0; slot < slotCount; ++slot)
        {
            depthBarriers.push_back(VkImageMemoryBarrier{
                .sType               = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
                .pNext               = nullptr,
                .srcAccessMask       = 0,
                .dstAccessMask       = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
                .oldLayout           = VK_IMAGE_LAYOUT_UNDEFINED,
                .newLayout           = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
                .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
                .image               = depthImages_[slot],
                .subresourceRange    = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1},
            });
        }
        vkCmdPipelineBarrier(command,
            VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
            VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT,
            0,
            0,
            nullptr,
            0,
            nullptr,
            static_cast<std::uint32_t>(depthBarriers.size()),
            depthBarriers.data());
        result = vkEndCommandBuffer(command);
    }
    if (result != VK_SUCCESS)
    {
        {
            std::lock_guard queueLock(device_.queueMutex());
            vkFreeCommandBuffers(device_.device(), device_.uploadPool(), 1, &command);
        }
        std::fprintf(stderr,
            "renderer module: could not record the depth-transition command buffer (%d)\n",
            static_cast<int>(result));
        return false;
    }

    const VkSubmitInfo submitInfo{
        .sType                = VK_STRUCTURE_TYPE_SUBMIT_INFO,
        .pNext                = nullptr,
        .waitSemaphoreCount   = 0,
        .pWaitSemaphores      = nullptr,
        .pWaitDstStageMask    = nullptr,
        .commandBufferCount   = 1,
        .pCommandBuffers      = &command,
        .signalSemaphoreCount = 0,
        .pSignalSemaphores    = nullptr,
    };
    const VkFenceCreateInfo fenceInfo{
        .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO,
        .pNext = nullptr,
        .flags = 0,
    };
    VkFence fence = VK_NULL_HANDLE;
    result        = vkCreateFence(device_.device(), &fenceInfo, nullptr, &fence);
    if (result != VK_SUCCESS)
    {
        {
            std::lock_guard queueLock(device_.queueMutex());
            vkFreeCommandBuffers(device_.device(), device_.uploadPool(), 1, &command);
        }
        std::fprintf(
            stderr, "renderer module: could not create the depth-transition fence (%d)\n", static_cast<int>(result));
        return false;
    }

    {
        std::lock_guard queueLock(device_.queueMutex());
        result = vkQueueSubmit(device_.queue(), 1, &submitInfo, fence);
    }
    if (result == VK_SUCCESS)
    {
        result = vkWaitForFences(device_.device(), 1, &fence, VK_TRUE, UINT64_MAX);
    }
    vkDestroyFence(device_.device(), fence, nullptr);
    {
        std::lock_guard queueLock(device_.queueMutex());
        vkFreeCommandBuffers(device_.device(), device_.uploadPool(), 1, &command);
    }
    if (result != VK_SUCCESS)
    {
        std::fprintf(stderr, "renderer module: depth-transition submission failed (%d)\n", static_cast<int>(result));
        return false;
    }

    // Framebuffers only exist for the legacy render-pass path; the dynamic
    // rendering path (renderPass == VK_NULL_HANDLE) declares attachments
    // inline via VkRenderingInfo and must not try to build framebuffers
    // against a null render pass. One framebuffer per (slot, image) pair -
    // each bakes in that slot's own depth attachment.
    if (renderPass != VK_NULL_HANDLE)
    {
        framebuffers_.assign(static_cast<std::size_t>(slotCount) * imageCountActual, VK_NULL_HANDLE);
        for (std::uint32_t slot = 0; slot < slotCount; ++slot)
        {
            for (std::uint32_t index = 0; index < imageCountActual; ++index)
            {
                const VkImageView             attachments[2] = {views_[index], depthViews_[slot]};
                const VkFramebufferCreateInfo framebufferInfo{
                    .sType           = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO,
                    .pNext           = nullptr,
                    .flags           = 0,
                    .renderPass      = renderPass,
                    .attachmentCount = 2,
                    .pAttachments    = attachments,
                    .width           = resolvedExtent.width,
                    .height          = resolvedExtent.height,
                    .layers          = 1,
                };
                if (vkCreateFramebuffer(device_.device(),
                        &framebufferInfo,
                        nullptr,
                        &framebuffers_[static_cast<std::size_t>(slot) * imageCountActual + index])
                    != VK_SUCCESS)
                {
                    return false;
                }
            }
        }
    }
    return true;
}

void VulkanSwapchain::destroyObjects()
{
    for (VkFramebuffer framebuffer : framebuffers_)
    {
        vkDestroyFramebuffer(device_.device(), framebuffer, nullptr);
    }
    framebuffers_.clear();
    for (VkImageView view : views_)
    {
        vkDestroyImageView(device_.device(), view, nullptr);
    }
    views_.clear();
    for (VkImageView view : depthViews_)
    {
        if (view != VK_NULL_HANDLE)
        {
            vkDestroyImageView(device_.device(), view, nullptr);
        }
    }
    depthViews_.clear();
    for (std::size_t slot = 0; slot < depthImages_.size(); ++slot)
    {
        if (depthImages_[slot] != VK_NULL_HANDLE)
        {
            device_.destroyImage(depthImages_[slot], depthAllocations_[slot]);
        }
    }
    depthImages_.clear();
    depthAllocations_.clear();
    for (VkImageView view : msaaColorViews_)
    {
        if (view != VK_NULL_HANDLE)
        {
            vkDestroyImageView(device_.device(), view, nullptr);
        }
    }
    msaaColorViews_.clear();
    for (std::size_t slot = 0; slot < msaaColorImages_.size(); ++slot)
    {
        if (msaaColorImages_[slot] != VK_NULL_HANDLE)
        {
            device_.destroyImage(msaaColorImages_[slot], msaaColorAllocations_[slot]);
        }
    }
    msaaColorImages_.clear();
    msaaColorAllocations_.clear();
    for (VkSemaphore semaphore : renderFinished_)
    {
        if (semaphore != VK_NULL_HANDLE)
        {
            vkDestroySemaphore(device_.device(), semaphore, nullptr);
        }
    }
    renderFinished_.clear();
    if (swapchain_ != VK_NULL_HANDLE)
    {
        vkDestroySwapchainKHR(device_.device(), swapchain_, nullptr);
        swapchain_ = VK_NULL_HANDLE;
    }
    images_.clear();
    imagesInFlight_.clear();

    if (readbackBuffer_ != VK_NULL_HANDLE)
    {
        device_.destroyBuffer(readbackBuffer_, readbackAllocation_);
        readbackBuffer_     = VK_NULL_HANDLE;
        readbackAllocation_ = nullptr;
    }
    readbackSize_ = 0;
}

// Device-idle teardown + rebuild; used on OUT_OF_DATE/SUBOPTIMAL and before
// the first frame of a resized viewport.
bool VulkanSwapchain::recreate(VkRenderPass renderPass, std::uint32_t viewportWidth, std::uint32_t viewportHeight,
    std::uint32_t framesInFlight)
{
    {
        std::lock_guard queueLock(device_.queueMutex());
        vkDeviceWaitIdle(device_.device());
    }
    destroyObjects();
    if (!createObjects(renderPass, viewportWidth, viewportHeight, framesInFlight))
    {
        std::fprintf(stderr, "renderer module: could not rebuild the swapchain\n");
        return false;
    }
    dirty_ = false;
    return true;
}

namespace
{
// Upper bound on how long vkAcquireNextImageKHR may block waiting for a free
// swapchain image. A finite timeout keeps a stalled presentation engine (a
// minimized or fully occluded window on some compositors) from freezing the
// frame loop: the caller treats VK_TIMEOUT as "skip this frame", so window
// events keep being pumped and the OS never flags the window as unresponsive.
constexpr std::uint64_t acquireTimeoutNanoseconds = 250ULL * 1000ULL * 1000ULL;
} // namespace

VkResult VulkanSwapchain::acquire(VkSemaphore imageAvailable, std::uint32_t *outImageIndex)
{
    return vkAcquireNextImageKHR(
        device_.device(), swapchain_, acquireTimeoutNanoseconds, imageAvailable, VK_NULL_HANDLE, outImageIndex);
}

// Present is serialized on the queue mutex like every other queue op;
// OUT_OF_DATE/SUBOPTIMAL set the dirty flag for the next beginFrame.
VkResult VulkanSwapchain::present(VkSemaphore renderFinished, std::uint32_t imageIndex)
{
    const VkPresentInfoKHR presentInfo{
        .sType              = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR,
        .pNext              = nullptr,
        .waitSemaphoreCount = 1,
        .pWaitSemaphores    = &renderFinished,
        .swapchainCount     = 1,
        .pSwapchains        = &swapchain_,
        .pImageIndices      = &imageIndex,
        .pResults           = nullptr,
    };
    std::lock_guard queueLock(device_.queueMutex());
    const VkResult  presentResult = vkQueuePresentKHR(device_.queue(), &presentInfo);
    if (presentResult == VK_ERROR_OUT_OF_DATE_KHR || presentResult == VK_SUBOPTIMAL_KHR)
    {
        dirty_ = true;
    }
    return presentResult;
}

bool VulkanSwapchain::ensureReadbackBuffer()
{
    if (!transferSourceSupported_ || extent_.width == 0 || extent_.height == 0)
    {
        return false;
    }
    constexpr VkDeviceSize maximumSize = std::numeric_limits<VkDeviceSize>::max();
    if (static_cast<VkDeviceSize>(extent_.width) > maximumSize / 4u / extent_.height)
    {
        return false;
    }
    const VkDeviceSize needed = static_cast<VkDeviceSize>(extent_.width) * extent_.height * 4u;
    if (readbackBuffer_ != VK_NULL_HANDLE && readbackSize_ == needed)
    {
        return true;
    }
    if (readbackBuffer_ != VK_NULL_HANDLE)
    {
        device_.destroyBuffer(readbackBuffer_, readbackAllocation_);
        readbackBuffer_     = VK_NULL_HANDLE;
        readbackAllocation_ = nullptr;
    }
    if (!device_.createBuffer(needed,
            VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            BufferMemoryIntent::HostRandomAccess,
            &readbackBuffer_,
            &readbackAllocation_))
    {
        std::fprintf(stderr, "renderer module: could not allocate the readback buffer\n");
        readbackSize_ = 0;
        return false;
    }
    readbackSize_ = needed;
    return true;
}

// Maps the readback buffer and writes the PPM (invalidate first so the CPU
// sees the GPU's writes on non-coherent memory). Pixel geometry comes from
// the caller: the recorded copy may predate a swapchain resize.
void VulkanSwapchain::finishReadback(
    const char *filePath, std::uint32_t width, std::uint32_t height, bool bgra, RenderError *error)
{
    if (readbackBuffer_ == VK_NULL_HANDLE || readbackSize_ == 0 || filePath == nullptr || width == 0 || height == 0
        || static_cast<VkDeviceSize>(width) > std::numeric_limits<VkDeviceSize>::max() / 4u / height)
    {
        reportError(error, RenderError::CreationFailed);
        return;
    }
    const VkDeviceSize requiredBytes = static_cast<VkDeviceSize>(width) * height * 4u;
    if (requiredBytes > readbackSize_ || requiredBytes > std::numeric_limits<std::size_t>::max())
    {
        reportError(error, RenderError::CreationFailed);
        return;
    }
    void *mapped = nullptr;
    if (!device_.mapAllocation(readbackAllocation_, &mapped))
    {
        reportError(error, RenderError::CreationFailed);
        return;
    }
    if (!device_.invalidateAllocation(readbackAllocation_))
    {
        device_.unmapAllocation(readbackAllocation_);
        reportError(error, RenderError::CreationFailed);
        return;
    }
    RenderError exportError = RenderError::None;
    writePpm(filePath,
        width,
        height,
        static_cast<const std::uint8_t *>(mapped),
        static_cast<std::size_t>(width) * 4,
        false,
        bgra,
        &exportError);
    device_.unmapAllocation(readbackAllocation_);
    if (error != nullptr)
    {
        *error = exportError;
    }
}

} // namespace renderer
} // namespace engine

#endif
