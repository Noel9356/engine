#include "renderer_state.hpp"

#include "vulkan_utilities.hpp"
#include "engine/kernel/filesystem_utilities.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace engine
{

void RendererState::createCommandAndSyncObjects()
{
    vk::CommandPoolCreateInfo poolInfo;
    poolInfo.setFlags(vk::CommandPoolCreateFlagBits::eResetCommandBuffer);
    poolInfo.setQueueFamilyIndex(graphicsQueueFamily);
    commandPool = std::make_unique<vk::raii::CommandPool>(*device, poolInfo);

    vk::CommandBufferAllocateInfo allocateInfo;
    allocateInfo.setCommandPool(**commandPool);
    allocateInfo.setLevel(vk::CommandBufferLevel::ePrimary);
    allocateInfo.setCommandBufferCount(maxFramesInFlight);
    commandBuffers = vk::raii::CommandBuffers(*device, allocateInfo);

    frameSyncPolicy.framesInFlight = maxFramesInFlight;
    frameSyncPolicy.setSwapchainImageCount(static_cast<std::uint32_t>(swapchainImages.size()));

    imageAvailableSemaphores.reserve(frameSyncPolicy.acquireSemaphoreCount());
    inFlightFences.reserve(frameSyncPolicy.acquireSemaphoreCount());
    for (std::uint32_t index = 0; index < frameSyncPolicy.acquireSemaphoreCount(); ++index)
    {
        imageAvailableSemaphores.emplace_back(*device, vk::SemaphoreCreateInfo());
        vk::FenceCreateInfo fenceInfo;
        fenceInfo.setFlags(vk::FenceCreateFlagBits::eSignaled);
        inFlightFences.emplace_back(*device, fenceInfo);
    }
    createRenderFinishedSemaphores();
}

void RendererState::createRenderFinishedSemaphores()
{
    // One render-finished semaphore per swapchain image (FrameSyncPolicy):
    // the presentation engine consumes the signal when it presents that image,
    // and the image can only come back through acquire, so the semaphore can
    // only be re-signaled after its previous use has been consumed. A pool
    // sized by frame slot instead re-signals while an older presentation may
    // still hold the semaphore - the VUID-vkQueueSubmit-pSignalSemaphores-00067
    // race reported on NVIDIA/Windows (docs/robustness.md, problem 22).
    renderingFinishedSemaphores.clear();
    renderingFinishedSemaphores.reserve(frameSyncPolicy.renderFinishedSemaphoreCount());
    for (std::uint32_t index = 0; index < frameSyncPolicy.renderFinishedSemaphoreCount(); ++index)
    {
        renderingFinishedSemaphores.emplace_back(*device, vk::SemaphoreCreateInfo());
    }
}

void RendererState::recreateSwapchain(
    WindowCapability &window, LoggerCapability &logger
)
{
    const vk::Format previousFormat = swapchainFormat;
    device->waitIdle();
    releaseCaptureBuffer();
    releaseFrameResources();
    swapchainImages.clear();
    swapchain.reset();
    createSwapchain(window, logger);

    // Rebuild the image-scoped semaphore pool: the image count may have
    // changed, and a present that failed with OUT_OF_DATE may have left one
    // of the previous semaphores signaled. waitIdle above guarantees no
    // present still waits on any of the old ones. The frame-slot scoped
    // acquire semaphores and fences are deliberately kept.
    frameSyncPolicy.setSwapchainImageCount(static_cast<std::uint32_t>(swapchainImages.size()));
    createRenderFinishedSemaphores();

    // Frame resources and the camera (aspect ratio) follow the new extent;
    // the pipelines only when the surface format changed. createPipelines
    // tears down and rebuilds both pipelines in reverse dependency order.
    createFrameResources();
    updateCameraBuffer();
    writeCompositeDescriptors();
    if (bloomEnabled)
    {
        // The bloom sets bind the recreated frame images; the pipelines are
        // format-independent (only the final pass's rendering info carries
        // the swapchain format, rebuilt only when the format changed below).
        writeBloomDescriptors();
    }
    if (swapchainFormat != previousFormat)
    {
        createPipelines();
    }

    // Lighting passes own descriptor sets that reference the recreated frame
    // resources; the waitIdle above makes rebuilding them safe.
    for (const auto &technique : lightingTechniques)
    {
        technique->notifyFrameResourcesRecreated();
    }
    framebufferResized = false;
}

void RendererState::releaseCaptureBuffer()
{
    if (captureMapped != nullptr && captureMemory != nullptr)
    {
        captureMemory->unmapMemory();
        captureMapped = nullptr;
    }
    captureMemory.reset();
    captureBuffer.reset();
    captureBufferSize = 0;
}

void RendererState::ensureCaptureBuffer()
{
    const vk::DeviceSize requiredSize = static_cast<vk::DeviceSize>(swapchainExtent.width) * swapchainExtent.height
                                      * bytesPerPixel;
    if (captureBuffer != nullptr && captureBufferSize >= requiredSize)
    {
        return;
    }
    releaseCaptureBuffer();

    vk::BufferCreateInfo bufferInfo;
    bufferInfo.setSize(requiredSize);
    bufferInfo.setUsage(vk::BufferUsageFlagBits::eTransferDst);
    bufferInfo.setSharingMode(vk::SharingMode::eExclusive);
    captureBuffer = std::make_unique<vk::raii::Buffer>(*device, bufferInfo);

    const vk::MemoryRequirements requirements = captureBuffer->getMemoryRequirements();
    vk::MemoryAllocateInfo       allocateInfo;
    allocateInfo.setAllocationSize(requirements.size);
    allocateInfo.setMemoryTypeIndex(findMemoryType(
        **physicalDevice,
        requirements.memoryTypeBits,
        vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent
    ));
    captureMemory = std::make_unique<vk::raii::DeviceMemory>(*device, allocateInfo);
    captureBuffer->bindMemory(*captureMemory, 0);
    captureBufferSize = requirements.size;
    captureMapped     = captureMemory->mapMemory(0, requirements.size);
}

void RendererState::recordGeometryPass(
    vk::raii::CommandBuffer &commandBuffer, const FrameRenderContext &frameContext
)
{
    std::array<vk::ClearValue, 3> clearValues;
    clearValues[0].setColor(vk::ClearColorValue(std::array<float, 4>{0.0f, 0.0f, 0.0f, 1.0f}));
    clearValues[1].setColor(vk::ClearColorValue(std::array<float, 4>{0.0f, 0.0f, 0.0f, 1.0f}));
    clearValues[2].setColor(vk::ClearColorValue(std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f}));

    std::array<vk::RenderingAttachmentInfo, 3> colorAttachments;
    const vk::ImageView                        attachmentViews[3] = {
        frameContext.emissiveView, frameContext.albedoView, frameContext.normalView
    };
    for (std::size_t index = 0; index < 3; ++index)
    {
        colorAttachments[index].setImageView(attachmentViews[index]);
        colorAttachments[index].setImageLayout(vk::ImageLayout::eGeneral);
        colorAttachments[index].setLoadOp(vk::AttachmentLoadOp::eClear);
        colorAttachments[index].setStoreOp(vk::AttachmentStoreOp::eStore);
        colorAttachments[index].setClearValue(clearValues[index]);
    }

    vk::RenderingAttachmentInfo depthAttachment;
    depthAttachment.setImageView(frameContext.depthView);
    depthAttachment.setImageLayout(vk::ImageLayout::eGeneral);
    depthAttachment.setLoadOp(vk::AttachmentLoadOp::eClear);
    // The depth buffer is consumed inside the frame; not storing it saves the
    // write-back bandwidth entirely (memory rule). The GENERAL layout covers
    // both the attachment use here and the sampled reads by the lighting
    // passes without per-frame layout churn.
    depthAttachment.setStoreOp(vk::AttachmentStoreOp::eNone);
    depthAttachment.setClearValue(vk::ClearDepthStencilValue(1.0f, 0));

    vk::RenderingInfo renderingInfo;
    renderingInfo.setRenderArea(vk::Rect2D(vk::Offset2D(0, 0), frameContext.extent));
    renderingInfo.setLayerCount(1);
    renderingInfo.setColorAttachmentCount(static_cast<std::uint32_t>(colorAttachments.size()));
    renderingInfo.setPColorAttachments(colorAttachments.data());
    renderingInfo.setPDepthAttachment(&depthAttachment);

    commandBuffer.beginRendering(renderingInfo);
    commandBuffer.bindPipeline(vk::PipelineBindPoint::eGraphics, **scenePipeline);
    const vk::Viewport viewport(
        0.0f,
        0.0f,
        static_cast<float>(frameContext.extent.width),
        static_cast<float>(frameContext.extent.height),
        0.0f,
        1.0f
    );
    commandBuffer.setViewport(0, viewport);
    const vk::Rect2D scissor(vk::Offset2D(0, 0), frameContext.extent);
    commandBuffer.setScissor(0, scissor);
    commandBuffer.bindDescriptorSets(
        vk::PipelineBindPoint::eGraphics, **scenePipelineLayout, 0, **sceneDescriptorSet, {}
    );

    const GeometryBuffer *currentMesh = nullptr;
    for (const SceneObject &object : activeObjects)
    {
        Mat4 model = modelMatrix(object.position.data(), object.rotationYRadians, object.size.data());
        // Material layers resolve the object's texture set the same way the
        // material table does (createSceneGeometry); -1 = no texture.
        std::int32_t albedoLayer = -1;
        if (!object.textureSet.empty())
        {
            const auto registered = std::find(scene.textureSets.begin(), scene.textureSets.end(), object.textureSet);
            if (registered != scene.textureSets.end())
            {
                albedoLayer = static_cast<std::int32_t>(registered - scene.textureSets.begin());
            }
        }
        struct ObjectPush
        {
            Mat4                 model;
            std::array<float, 4> albedoRoughness;
            std::array<float, 4> metallicEmissiveStrength;
            std::array<float, 4> materialLayers;
        } pushValue{
            model,
            {object.albedo[0], object.albedo[1], object.albedo[2], object.roughness},
            {object.metallic, object.emissiveStrength, 0.0f, 0.0f},
            {static_cast<float>(albedoLayer),
                static_cast<float>(albedoLayer),
                object.uvScale,
                static_cast<float>(static_cast<std::uint32_t>(object.type))}
        };
        commandBuffer.pushConstants(
            **scenePipelineLayout,
            vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment,
            0,
            sizeof(pushValue),
            &pushValue
        );

        const GeometryBuffer &mesh = object.type == SceneObjectType::Sphere ? sphereMesh : boxMesh;
        if (currentMesh != &mesh)
        {
            commandBuffer.bindVertexBuffers(0, **mesh.buffer, {0});
            commandBuffer.bindIndexBuffer(
                **mesh.buffer, static_cast<vk::DeviceSize>(mesh.indexOffsetBytes), vk::IndexType::eUint32
            );
            currentMesh = &mesh;
        }
        commandBuffer.drawIndexed(mesh.indexCount, 1, 0, 0, 0);
    }
    commandBuffer.endRendering();
}

void RendererState::recordFrameStartBarrier(
    vk::raii::CommandBuffer &commandBuffer
)
{
    // Reused scratch (renderer_state.hpp): recording runs every frame and the
    // fresh-vector-per-frame allocations showed up as pure heap churn on the
    // record path.
    std::vector<vk::ImageMemoryBarrier2> &barriers = frameStartBarriers;
    barriers.clear();
    const vk::Image frameImages[6] = {
        **emissiveImage.image,
        **albedoImage.image,
        **normalImage.image,
        hasDirectRadianceTarget ? **directRadianceImage.image : vk::Image{},
        hasIndirectDiffuseTarget ? **indirectRadianceImage.image : vk::Image{},
        hasIndirectSpecularTarget ? **indirectSpecularImage.image : vk::Image{}
    };
    for (const vk::Image image : frameImages)
    {
        if (!image)
        {
            continue;
        }
        vk::ImageMemoryBarrier2 barrier;
        barrier.setSrcStageMask(vk::PipelineStageFlagBits2::eAllCommands);
        barrier.setSrcAccessMask(vk::AccessFlagBits2::eMemoryRead | vk::AccessFlagBits2::eMemoryWrite);
        barrier.setDstStageMask(vk::PipelineStageFlagBits2::eAllCommands);
        barrier.setDstAccessMask(vk::AccessFlagBits2::eMemoryWrite);
        barrier.setOldLayout(vk::ImageLayout::eGeneral);
        barrier.setNewLayout(vk::ImageLayout::eGeneral);
        barrier.setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
        barrier.setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
        barrier.setImage(image);
        barrier.setSubresourceRange(vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1));
        barriers.push_back(barrier);
    }
    vk::DependencyInfo dependency;
    dependency.setImageMemoryBarriers(barriers);
    commandBuffer.pipelineBarrier2(dependency);
}

void RendererState::recordGBufferHandoffBarrier(
    vk::raii::CommandBuffer &commandBuffer
)
{
    std::array<vk::ImageMemoryBarrier2, 4> barriers;
    const vk::Image images[4] = {**emissiveImage.image, **albedoImage.image, **normalImage.image, **depthImage.image};
    for (std::size_t index = 0; index < 4; ++index)
    {
        barriers[index].setSrcStageMask(
            vk::PipelineStageFlagBits2::eColorAttachmentOutput | vk::PipelineStageFlagBits2::eLateFragmentTests
        );
        barriers[index].setSrcAccessMask(
            vk::AccessFlagBits2::eColorAttachmentWrite | vk::AccessFlagBits2::eDepthStencilAttachmentWrite
        );
        barriers[index].setDstStageMask(
            vk::PipelineStageFlagBits2::eComputeShader | vk::PipelineStageFlagBits2::eFragmentShader
        );
        barriers[index].setDstAccessMask(vk::AccessFlagBits2::eShaderRead | vk::AccessFlagBits2::eShaderStorageRead);
        barriers[index].setOldLayout(vk::ImageLayout::eGeneral);
        barriers[index].setNewLayout(vk::ImageLayout::eGeneral);
        barriers[index].setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
        barriers[index].setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
        barriers[index].setImage(images[index]);
        barriers[index].setSubresourceRange(
            vk::ImageSubresourceRange(
                index == 3 ? vk::ImageAspectFlagBits::eDepth : vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1
            )
        );
    }
    vk::DependencyInfo dependency;
    dependency.setImageMemoryBarriers(barriers);
    commandBuffer.pipelineBarrier2(dependency);
}

void RendererState::recordRadianceToCompositeBarrier(
    vk::raii::CommandBuffer &commandBuffer
)
{
    // Compute writes (direct, diffuse, specular) -> composite fragment reads.
    // Scratch reused across frames (renderer_state.hpp) - see
    // recordFrameStartBarrier.
    if (!hasDirectRadianceTarget && !hasIndirectDiffuseTarget && !hasIndirectSpecularTarget)
    {
        return;
    }
    std::vector<vk::ImageMemoryBarrier2> &barriers = radianceBarriers;
    barriers.clear();
    const auto radianceBarrier = [&barriers](const vk::Image image)
    {
        vk::ImageMemoryBarrier2 barrier;
        barrier.setSrcStageMask(vk::PipelineStageFlagBits2::eComputeShader);
        barrier.setSrcAccessMask(vk::AccessFlagBits2::eShaderStorageWrite);
        barrier.setDstStageMask(vk::PipelineStageFlagBits2::eFragmentShader);
        barrier.setDstAccessMask(vk::AccessFlagBits2::eShaderRead);
        barrier.setOldLayout(vk::ImageLayout::eGeneral);
        barrier.setNewLayout(vk::ImageLayout::eGeneral);
        barrier.setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
        barrier.setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
        barrier.setImage(image);
        barrier.setSubresourceRange(vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1));
        barriers.push_back(barrier);
    };
    if (hasDirectRadianceTarget)
    {
        radianceBarrier(**directRadianceImage.image);
    }
    if (hasIndirectDiffuseTarget)
    {
        radianceBarrier(**indirectRadianceImage.image);
    }
    if (hasIndirectSpecularTarget)
    {
        radianceBarrier(**indirectSpecularImage.image);
    }
    vk::DependencyInfo dependency;
    dependency.setImageMemoryBarriers(barriers);
    commandBuffer.pipelineBarrier2(dependency);
}

void RendererState::recordSwapchainEnterTransition(
    vk::raii::CommandBuffer &commandBuffer, std::uint32_t imageIndex
)
{
    // Composite into the swapchain image: UNDEFINED -> COLOR_ATTACHMENT.
    vk::ImageMemoryBarrier2 enterBarrier;
    enterBarrier.setSrcStageMask(vk::PipelineStageFlagBits2::eTopOfPipe);
    enterBarrier.setSrcAccessMask(vk::AccessFlags2());
    enterBarrier.setDstStageMask(vk::PipelineStageFlagBits2::eColorAttachmentOutput);
    enterBarrier.setDstAccessMask(vk::AccessFlagBits2::eColorAttachmentWrite);
    enterBarrier.setOldLayout(vk::ImageLayout::eUndefined);
    enterBarrier.setNewLayout(vk::ImageLayout::eColorAttachmentOptimal);
    enterBarrier.setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
    enterBarrier.setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
    enterBarrier.setImage(swapchainImages[imageIndex]);
    enterBarrier.setSubresourceRange(vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1));
    std::array<vk::ImageMemoryBarrier2, 1> enter{enterBarrier};
    vk::DependencyInfo                     dependency;
    dependency.setImageMemoryBarriers(enter);
    commandBuffer.pipelineBarrier2(dependency);
}

void RendererState::recordSwapchainCaptureAndPresentTransitions(
    vk::raii::CommandBuffer &commandBuffer, std::uint32_t imageIndex
)
{
    if (captureRequested)
    {
        // COLOR_ATTACHMENT -> TRANSFER_SRC, copy, then PRESENT. The copy
        // happens while the frame still owns the image, before presentation
        // rewrites it.
        vk::ImageMemoryBarrier2 captureEnterBarrier;
        captureEnterBarrier.setSrcStageMask(vk::PipelineStageFlagBits2::eColorAttachmentOutput);
        captureEnterBarrier.setSrcAccessMask(vk::AccessFlagBits2::eColorAttachmentWrite);
        captureEnterBarrier.setDstStageMask(vk::PipelineStageFlagBits2::eTransfer);
        captureEnterBarrier.setDstAccessMask(vk::AccessFlagBits2::eTransferRead);
        captureEnterBarrier.setOldLayout(vk::ImageLayout::eColorAttachmentOptimal);
        captureEnterBarrier.setNewLayout(vk::ImageLayout::eTransferSrcOptimal);
        captureEnterBarrier.setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
        captureEnterBarrier.setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
        captureEnterBarrier.setImage(swapchainImages[imageIndex]);
        captureEnterBarrier.setSubresourceRange(vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1));
        std::array<vk::ImageMemoryBarrier2, 1> captureEnter{captureEnterBarrier};
        vk::DependencyInfo                     captureEnterDependency;
        captureEnterDependency.setImageMemoryBarriers(captureEnter);
        commandBuffer.pipelineBarrier2(captureEnterDependency);

        vk::BufferImageCopy copyRegion;
        copyRegion.setBufferRowLength(0);
        copyRegion.setBufferImageHeight(0);
        copyRegion.setImageSubresource(vk::ImageSubresourceLayers(vk::ImageAspectFlagBits::eColor, 0, 0, 1));
        copyRegion.setImageOffset(vk::Offset3D(0, 0, 0));
        copyRegion.setImageExtent(vk::Extent3D(swapchainExtent.width, swapchainExtent.height, 1));
        std::array<vk::BufferImageCopy, 1> copyRegions{copyRegion};
        commandBuffer.copyImageToBuffer(
            swapchainImages[imageIndex], vk::ImageLayout::eTransferSrcOptimal, **captureBuffer, copyRegions
        );
    }

    // -> PRESENT_SRC_KHR (from attachment or transfer layout).
    {
        vk::ImageMemoryBarrier2 presentBarrier;
        presentBarrier.setSrcStageMask(
            vk::PipelineStageFlagBits2::eColorAttachmentOutput | vk::PipelineStageFlagBits2::eTransfer
        );
        presentBarrier.setSrcAccessMask(
            vk::AccessFlagBits2::eColorAttachmentWrite | vk::AccessFlagBits2::eTransferRead
        );
        presentBarrier.setDstStageMask(vk::PipelineStageFlagBits2::eNone);
        presentBarrier.setDstAccessMask(vk::AccessFlags2());
        presentBarrier.setOldLayout(
            captureRequested ? vk::ImageLayout::eTransferSrcOptimal : vk::ImageLayout::eColorAttachmentOptimal
        );
        presentBarrier.setNewLayout(vk::ImageLayout::ePresentSrcKHR);
        presentBarrier.setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
        presentBarrier.setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
        presentBarrier.setImage(swapchainImages[imageIndex]);
        presentBarrier.setSubresourceRange(vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1));
        std::array<vk::ImageMemoryBarrier2, 1> present{presentBarrier};
        vk::DependencyInfo                     presentDependency;
        presentDependency.setImageMemoryBarriers(present);
        commandBuffer.pipelineBarrier2(presentDependency);
    }
}

void RendererState::recordCompositeIntoTarget(
    vk::raii::CommandBuffer &commandBuffer, const vk::ImageView &targetView, vk::Extent2D extent,
    std::int32_t tonemapModeValue
)
{
    vk::RenderingAttachmentInfo colorAttachment;
    colorAttachment.setImageView(targetView);
    colorAttachment.setImageLayout(vk::ImageLayout::eColorAttachmentOptimal);
    colorAttachment.setLoadOp(vk::AttachmentLoadOp::eDontCare);
    colorAttachment.setStoreOp(vk::AttachmentStoreOp::eStore);

    vk::RenderingInfo renderingInfo;
    renderingInfo.setRenderArea(vk::Rect2D(vk::Offset2D(0, 0), extent));
    renderingInfo.setLayerCount(1);
    renderingInfo.setColorAttachmentCount(1);
    renderingInfo.setPColorAttachments(&colorAttachment);

    struct CompositePush
    {
        std::array<float, 4>        flagsExposure; // x unused, w exposure
        std::array<std::int32_t, 4> modeFlags;
    } pushValue{
        {0.0f, 0.0f, 0.0f, exposure},
        {tonemapModeValue,
            hasDirectRadianceTarget ? 1 : 0,
            hasIndirectDiffuseTarget ? 1 : 0,
            hasIndirectSpecularTarget ? 1 : 0}
    };

    commandBuffer.beginRendering(renderingInfo);
    commandBuffer.bindPipeline(vk::PipelineBindPoint::eGraphics, **compositePipeline);
    const vk::Viewport viewport(
        0.0f, 0.0f, static_cast<float>(extent.width), static_cast<float>(extent.height), 0.0f, 1.0f
    );
    commandBuffer.setViewport(0, viewport);
    commandBuffer.setScissor(0, vk::Rect2D(vk::Offset2D(0, 0), extent));
    commandBuffer.bindDescriptorSets(
        vk::PipelineBindPoint::eGraphics, **compositePipelineLayout, 0, **compositeDescriptorSet, {}
    );
    commandBuffer.pushConstants(
        **compositePipelineLayout, vk::ShaderStageFlagBits::eFragment, 0, sizeof(pushValue), &pushValue
    );
    commandBuffer.draw(3, 1, 0, 0);
    commandBuffer.endRendering();
}

void RendererState::recordCompositePass(
    vk::raii::CommandBuffer &commandBuffer, std::uint32_t imageIndex
)
{
    const std::int32_t mode =
        tonemapMode == "aces" ? tonemapAces : (tonemapMode == "reinhard" ? tonemapReinhard : tonemapNone);
    recordCompositeIntoTarget(commandBuffer, swapchainImageViews[imageIndex], swapchainExtent, mode);
}

void RendererState::recordBloomPass(
    vk::raii::CommandBuffer &commandBuffer, std::uint32_t imageIndex
)
{
    // The bloom path: composite (UNTONEMAPPED) -> hdrSceneImage, bright
    // extract -> half-res A, blur H (A -> B), blur V (B -> A), final
    // (tonemap + add) -> swapchain. All frame images persist in GENERAL;
    // every hand-off is an access barrier, never a layout transition.
    const std::int32_t mode =
        tonemapMode == "aces" ? tonemapAces : (tonemapMode == "reinhard" ? tonemapReinhard : tonemapNone);

    // 1. Composite the lit scene into the HDR intermediate, no tonemap.
    recordCompositeIntoTarget(commandBuffer, *hdrSceneImage.view, swapchainExtent, tonemapNone);

    // HDR composite writes -> extract reads.
    vk::MemoryBarrier2 hdrBarrier;
    hdrBarrier.setSrcStageMask(vk::PipelineStageFlagBits2::eColorAttachmentOutput);
    hdrBarrier.setSrcAccessMask(vk::AccessFlagBits2::eColorAttachmentWrite);
    hdrBarrier.setDstStageMask(vk::PipelineStageFlagBits2::eComputeShader);
    hdrBarrier.setDstAccessMask(vk::AccessFlagBits2::eShaderRead);
    vk::DependencyInfo hdrDependency;
    hdrDependency.setMemoryBarriers(hdrBarrier);
    commandBuffer.pipelineBarrier2(hdrDependency);

    const vk::Extent2D halfExtent{
        std::max(swapchainExtent.width / 2, 1u), std::max(swapchainExtent.height / 2, 1u)};

    // 2. Bright extract (compute, 8x8 groups over the half-res target).
    struct ExtractPush
    {
        std::array<float, 4> thresholdKnee;
    } extractPush{{bloomThreshold, 0.5f, 0.0f, 0.0f}};
    commandBuffer.bindPipeline(vk::PipelineBindPoint::eCompute, **bloomExtractPipeline);
    commandBuffer.bindDescriptorSets(
        vk::PipelineBindPoint::eCompute, **bloomExtractPipelineLayout, 0, **bloomExtractDescriptorSet, {}
    );
    commandBuffer.pushConstants(
        **bloomExtractPipelineLayout, vk::ShaderStageFlagBits::eCompute, 0, sizeof(extractPush), &extractPush
    );
    commandBuffer.dispatch((halfExtent.width + 7) / 8, (halfExtent.height + 7) / 8, 1);

    // Extract writes -> blur H reads.
    vk::MemoryBarrier2 extractBarrier;
    extractBarrier.setSrcStageMask(vk::PipelineStageFlagBits2::eComputeShader);
    extractBarrier.setSrcAccessMask(vk::AccessFlagBits2::eShaderStorageWrite);
    extractBarrier.setDstStageMask(vk::PipelineStageFlagBits2::eComputeShader);
    extractBarrier.setDstAccessMask(vk::AccessFlagBits2::eShaderRead);
    vk::DependencyInfo extractDependency;
    extractDependency.setMemoryBarriers(extractBarrier);
    commandBuffer.pipelineBarrier2(extractDependency);

    // 3. Separable blur: horizontal (A -> B), then vertical (B -> A).
    struct BlurPush
    {
        std::array<float, 4> directionTexel;
    };
    commandBuffer.bindPipeline(vk::PipelineBindPoint::eCompute, **bloomBlurPipeline);
    BlurPush horizontal{{1.0f, 0.0f, 0.0f, 0.0f}};
    commandBuffer.bindDescriptorSets(
        vk::PipelineBindPoint::eCompute, **bloomBlurPipelineLayout, 0, **bloomBlurSetA, {}
    );
    commandBuffer.pushConstants(
        **bloomBlurPipelineLayout, vk::ShaderStageFlagBits::eCompute, 0, sizeof(horizontal), &horizontal
    );
    commandBuffer.dispatch((halfExtent.width + 7) / 8, (halfExtent.height + 7) / 8, 1);

    vk::MemoryBarrier2 blurHBarrier;
    blurHBarrier.setSrcStageMask(vk::PipelineStageFlagBits2::eComputeShader);
    blurHBarrier.setSrcAccessMask(vk::AccessFlagBits2::eShaderStorageWrite);
    blurHBarrier.setDstStageMask(vk::PipelineStageFlagBits2::eComputeShader);
    blurHBarrier.setDstAccessMask(vk::AccessFlagBits2::eShaderRead);
    vk::DependencyInfo blurHDependency;
    blurHDependency.setMemoryBarriers(blurHBarrier);
    commandBuffer.pipelineBarrier2(blurHDependency);

    BlurPush vertical{{0.0f, 1.0f, 0.0f, 0.0f}};
    commandBuffer.bindDescriptorSets(
        vk::PipelineBindPoint::eCompute, **bloomBlurPipelineLayout, 0, **bloomBlurSetB, {}
    );
    commandBuffer.pushConstants(
        **bloomBlurPipelineLayout, vk::ShaderStageFlagBits::eCompute, 0, sizeof(vertical), &vertical
    );
    commandBuffer.dispatch((halfExtent.width + 7) / 8, (halfExtent.height + 7) / 8, 1);

    // Blur writes -> final fragment reads.
    vk::MemoryBarrier2 blurVBarrier;
    blurVBarrier.setSrcStageMask(vk::PipelineStageFlagBits2::eComputeShader);
    blurVBarrier.setSrcAccessMask(vk::AccessFlagBits2::eShaderStorageWrite);
    blurVBarrier.setDstStageMask(vk::PipelineStageFlagBits2::eFragmentShader);
    blurVBarrier.setDstAccessMask(vk::AccessFlagBits2::eShaderRead);
    vk::DependencyInfo blurVDependency;
    blurVDependency.setMemoryBarriers(blurVBarrier);
    commandBuffer.pipelineBarrier2(blurVDependency);

    // 4. Final composite: tonemap(hdr + bloom * strength) into the swapchain.
    vk::RenderingAttachmentInfo colorAttachment;
    colorAttachment.setImageView(swapchainImageViews[imageIndex]);
    colorAttachment.setImageLayout(vk::ImageLayout::eColorAttachmentOptimal);
    colorAttachment.setLoadOp(vk::AttachmentLoadOp::eDontCare);
    colorAttachment.setStoreOp(vk::AttachmentStoreOp::eStore);
    vk::RenderingInfo renderingInfo;
    renderingInfo.setRenderArea(vk::Rect2D(vk::Offset2D(0, 0), swapchainExtent));
    renderingInfo.setLayerCount(1);
    renderingInfo.setColorAttachmentCount(1);
    renderingInfo.setPColorAttachments(&colorAttachment);

    struct FinalPush
    {
        std::array<float, 4>        flagsExposure;
        std::array<std::int32_t, 4> modeFlags;
    } finalPush{{bloomStrength, 0.0f, 0.0f, exposure}, {mode, 0, 0, 0}};

    commandBuffer.beginRendering(renderingInfo);
    commandBuffer.bindPipeline(vk::PipelineBindPoint::eGraphics, **bloomFinalPipeline);
    const vk::Viewport viewport(
        0.0f, 0.0f, static_cast<float>(swapchainExtent.width), static_cast<float>(swapchainExtent.height), 0.0f, 1.0f
    );
    commandBuffer.setViewport(0, viewport);
    commandBuffer.setScissor(0, vk::Rect2D(vk::Offset2D(0, 0), swapchainExtent));
    commandBuffer.bindDescriptorSets(
        vk::PipelineBindPoint::eGraphics, **bloomFinalPipelineLayout, 0, **bloomFinalDescriptorSet, {}
    );
    commandBuffer.pushConstants(
        **bloomFinalPipelineLayout, vk::ShaderStageFlagBits::eFragment, 0, sizeof(finalPush), &finalPush
    );
    commandBuffer.draw(3, 1, 0, 0);
    commandBuffer.endRendering();
}

void RendererState::recordFrameCommands(
    vk::raii::CommandBuffer &commandBuffer, std::uint32_t imageIndex
)
{
    commandBuffer.reset();
    commandBuffer.begin(vk::CommandBufferBeginInfo());

    // FrameRenderContext: everything the lighting passes may touch this frame.
    FrameRenderContext frameContext{
        .device               = *device,
        .physicalDevice       = **physicalDevice,
        .graphicsQueue        = graphicsQueue,
        .graphicsQueueFamily  = graphicsQueueFamily,
        .extent               = swapchainExtent,
        .shaderDirectory      = resolvedShaderDirectory,
        .albedoView           = *albedoImage.view,
        .normalView           = *normalImage.view,
        .emissiveView         = *emissiveImage.view,
        .depthView            = *depthImage.view,
        .directRadianceView   = hasDirectRadianceTarget ? *directRadianceImage.view : *dummyImage.view,
        .indirectRadianceView = hasIndirectDiffuseTarget ? *indirectRadianceImage.view : *dummyImage.view,
        .indirectSpecularView = hasIndirectSpecularTarget ? *indirectSpecularImage.view : *dummyImage.view,
        .primitivesBuffer     = **primitivesBuffer.buffer,
        .primitiveCount       = static_cast<std::uint32_t>(activeObjects.size()),
        .materialsBuffer      = **materialsBuffer.buffer,
        .albedoArrayView      = *albedoArrayImage.view,
        .dataArrayView        = *dataArrayImage.view,
        .environmentBuffer    = **environmentBufferStorage.buffer,
        .cameraBuffer         = **cameraBufferStorage.buffer,
        .cameraNearPlane      = scene.camera.nearPlane,
        .cameraFarPlane       = scene.camera.farPlane
    };

    recordGeometryPass(commandBuffer, frameContext);

    // Frame-start hazard: the previous frame's compute passes READ these
    // images (lighting dispatches, composite fragments); this frame writes
    // them - the G-buffer as color/depth attachments, the radiance targets
    // through COMPUTE STORAGE WRITES. The dependency must therefore cover
    // both write styles on the destination side: the earlier destination
    // scope (color attachment output only) left the targets' compute writes
    // unordered against the previous frame's reads. Cross-FRAME ordering
    // itself does not come from barriers - a barrier's scope never crosses
    // a queue submission - but from the host-side frame serialization in
    // drawFrame (both in-flight fences waited before recording). With that
    // ordering in place, this barrier extends the dependency into this
    // submission and covers the write-after-read hazard between the frames'
    // access styles.
    recordFrameStartBarrier(commandBuffer);

    // Geometry writes -> compute/fragment reads: one barrier covers the
    // G-buffer hand-off (all frame images stay in GENERAL, so this is an
    // access dependency, not a layout transition).
    recordGBufferHandoffBarrier(commandBuffer);

    // Techniques record in attach order; direct-radiance techniques were
    // attached before indirect ones by attachLightingTechnique, so emitters
    // evaluate before transport consumes their output.
    //
    // Inter-technique dependency: a technique's storage-image writes are read
    // by the NEXT technique's dispatches inside the same command buffer
    // (ray_reflections samples the cascade denoiser's indirect target). The
    // renderer inserts no ordering between technique records itself, and a
    // compute-write -> compute-read pair needs a memory dependency to be
    // ordered and visible - without it the reflection pass sampled stale or
    // garbage GI (nondeterministic reflections that shifted every frame).
    bool firstTechnique = true;
    for (const auto &technique : lightingTechniques)
    {
        if (!firstTechnique)
        {
            vk::MemoryBarrier2 interTechniqueBarrier;
            interTechniqueBarrier.setSrcStageMask(vk::PipelineStageFlagBits2::eComputeShader);
            interTechniqueBarrier.setSrcAccessMask(vk::AccessFlagBits2::eShaderStorageWrite);
            interTechniqueBarrier.setDstStageMask(vk::PipelineStageFlagBits2::eComputeShader);
            interTechniqueBarrier.setDstAccessMask(
                vk::AccessFlagBits2::eShaderRead | vk::AccessFlagBits2::eShaderStorageRead
            );
            vk::DependencyInfo interTechniqueDependency;
            interTechniqueDependency.setMemoryBarriers(interTechniqueBarrier);
            commandBuffer.pipelineBarrier2(interTechniqueDependency);
        }
        firstTechnique = false;
        technique->record(commandBuffer, frameContext);
    }

    recordRadianceToCompositeBarrier(commandBuffer);

    if (bloomEnabled && bloomFinalPipeline != nullptr)
    {
        // The bloom path: the swapchain enter transition happens INSIDE
        // recordBloomPass's final stage ordering (the compute chain runs
        // first, then the final pass targets the swapchain image).
        recordSwapchainEnterTransition(commandBuffer, imageIndex);
        recordBloomPass(commandBuffer, imageIndex);
    }
    else
    {
        recordSwapchainEnterTransition(commandBuffer, imageIndex);
        recordCompositePass(commandBuffer, imageIndex);
    }

    recordSwapchainCaptureAndPresentTransitions(commandBuffer, imageIndex);

    commandBuffer.end();
}

void RendererState::updateCameraIntegration(
    LoggerCapability &logger
)
{
    // Free-fly camera integration runs EVERY frame (even a skipped one), so
    // the delta time stays truthful and input never accumulates unnoticed.
    // The first frame after start seeds the controller from the configured
    // pose; later frames integrate movement into the same [scene.camera]
    // state the camera uniform is built from.
    const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    if (!flyCameraSeeded)
    {
        flyCamera.syncFromCamera(scene.camera);
        lastFrameTime   = now;
        flyCameraSeeded = true;
        if (inputCapability != nullptr && scene.camera.freeFly)
        {
            inputCapability->setCursorCaptured(true);
            logger.logMessage(
                LogLevel::Information,
                "camera: free-fly active - WASD move, mouse look, Space/E up, Ctrl/Q down, "
                "Shift boost, wheel speed, F toggle mouse capture, Esc release"
            );
        }
        else if (inputCapability == nullptr)
        {
            logger.logMessage(LogLevel::Information, "camera: static (input capability not available in this build)");
        }
    }
    else
    {
        double deltaTime = std::chrono::duration<double>(now - lastFrameTime).count();
        // Clamp against pathological stalls (debugger pauses, window drags):
        // a five-second gap must not teleport the camera.
        deltaTime = std::min(deltaTime, maxCameraDeltaTimeSeconds);
        if (scene.camera.freeFly)
        {
            flyCamera.update(inputCapability, deltaTime, logger);
            flyCamera.applyToCamera(scene.camera);
        }
        lastFrameTime = now;
    }
}

bool RendererState::waitForFrameSerializationGuarded(
    LoggerCapability &logger
)
{
    // Frame execution serialization. All transient GPU resources - the
    // G-buffer, the radiance targets, the cascade fields - are single-buffered
    // and shared by every frame slot, and so are the host-written uniform
    // buffers (camera, environment, technique light states). Nothing in a
    // frame can therefore overlap the NEXT frame without racing it:
    //  - GPU vs GPU: frame N+1's writes against frame N's reads. An image
    //    barrier cannot fix this - a barrier's synchronization scope never
    //    crosses a queue submission boundary; only semaphores or fences do.
    //  - Host vs GPU: this function's camera memcpy against frame N's
    //    shaders, which read torn matrices mid-motion (the reported
    //    interactive flicker; invisible on software renderers, which execute
    //    submissions synchronously on the calling thread).
    // The fix is host-side full serialization: every drawFrame waits for BOTH
    // in-flight fences before touching any host-mapped uniform or recording
    // commands, so at most one frame is ever executing on the device. With
    // every pass reading the shared targets, overlapped execution would carry
    // no throughput benefit anyway; what this costs is CPU recording overlap,
    // which is microseconds against multi-millisecond GPU frames. Per-slot
    // transient targets are the path back to real pipelining if it is ever
    // needed (docs/robustness.md, problem 38).
    //
    // The slot fence is waited first: it stays signaled until its submission
    // succeeded, so an early return (swapchain out of date) must not leave it
    // unsignaled, or the next frame would wait forever. Errors throw; only
    // eSuccess and eTimeout return. On timeout the GPU did not retire that
    // frame, so this frame is skipped instead of re-recording a command
    // buffer that is still in flight.
    const vk::Result slotFenceDone = device->waitForFences(
        {*inFlightFences[currentFrame]}, true, frameTimeoutNanoseconds
    );
    if (slotFenceDone == vk::Result::eTimeout)
    {
        logger.logMessage(LogLevel::Warning, logPrefix + "frame fence timed out; skipping a frame");
        return false;
    }
    // The OTHER slot's fence covers the immediately preceding frame: when it
    // is retired, no submission of this renderer is executing any more (the
    // slot fence already retired the one before it). Fences start signaled,
    // so the first two frames pass straight through.
    if (maxFramesInFlight > 1)
    {
        const std::uint32_t previousSlot      = (currentFrame + 1) % maxFramesInFlight;
        const vk::Result    previousFrameDone = device->waitForFences(
            {*inFlightFences[previousSlot]}, true, frameTimeoutNanoseconds
        );
        if (previousFrameDone == vk::Result::eTimeout)
        {
            logger.logMessage(LogLevel::Warning, logPrefix + "previous frame fence timed out; skipping a frame");
            return false;
        }
    }
    return true;
}

bool RendererState::readbackCapturePixels(
    LoggerCapability &logger
)
{
    // Errors throw; only eSuccess and eTimeout return. The readback needs
    // the fence: reading mapped memory while the GPU may still write it
    // would race, so a timeout discards the capture instead. The window
    // is the generous capture budget, not the per-frame present timeout -
    // a 2 s CPU-rendered frame must still capture.
    const vk::Result captureDone = device->waitForFences(
        {*inFlightFences[currentFrame]}, true, captureTimeoutNanoseconds
    );
    if (captureDone == vk::Result::eTimeout)
    {
        logger.logMessage(LogLevel::Warning, logPrefix + "frame capture fence timed out; capture discarded");
        return false;
    }

    capturePixels.resize(static_cast<std::size_t>(swapchainExtent.width) * swapchainExtent.height * bytesPerPixel);
    const std::uint8_t *source      = static_cast<const std::uint8_t *>(captureMapped);
    const bool          isBgraOrder = swapchainFormat == vk::Format::eB8G8R8A8Srgb
                                   || swapchainFormat == vk::Format::eB8G8R8A8Unorm;
    for (std::size_t pixel = 0; pixel < capturePixels.size() / bytesPerPixel; ++pixel)
    {
        const std::size_t offset = pixel * bytesPerPixel;
        if (isBgraOrder)
        {
            capturePixels[offset + 0] = source[offset + 2];
            capturePixels[offset + 1] = source[offset + 1];
            capturePixels[offset + 2] = source[offset + 0];
            capturePixels[offset + 3] = source[offset + 3];
        }
        else
        {
            capturePixels[offset + 0] = source[offset + 0];
            capturePixels[offset + 1] = source[offset + 1];
            capturePixels[offset + 2] = source[offset + 2];
            capturePixels[offset + 3] = source[offset + 3];
        }
    }

    FrameCapturedEvent captured;
    captured.pixelData = capturePixels.data();
    captured.width     = swapchainExtent.width;
    captured.height    = swapchainExtent.height;
    if (frameCapturedChannel != nullptr)
    {
        frameCapturedChannel->publish(captured);
    }
    else
    {
        logger.logMessage(
            LogLevel::Warning, logPrefix + "frame captured but no capture channel is bound; capture dropped"
        );
    }
    logger.logMessage(LogLevel::Debug, logPrefix + "frame captured");
    return true;
}

void RendererState::renderFrame(
    LoggerCapability &logger
)
{
    updateCameraIntegration(logger);
    if (!waitForFrameSerializationGuarded(logger))
    {
        return;
    }

    // Scene edits (scripts, physics, animation) only flagged the registry
    // dirty; the one re-sync per dirty frame happens here, on the serialized
    // frame path with the device idle - the staging copies race nothing
    // (docs/robustness.md, problem 38).
    if (sceneRegistryDirty)
    {
        sceneRegistryDirty = false;
        refreshSceneRegistry();
    }

    // Only now is the device idle with respect to this renderer: writing the
    // camera uniform cannot tear against a still-executing frame.
    updateCameraBuffer();

    std::uint32_t imageIndex = 0;
    try
    {
        imageIndex = swapchain
                         ->acquireNextImage(
                             frameTimeoutNanoseconds,
                             *imageAvailableSemaphores[FrameSyncPolicy::acquireSemaphoreIndex(currentFrame)],
                             nullptr
                         )
                         .value;
    }
    catch (const vk::OutOfDateKHRError &)
    {
        framebufferResized = true;
        return;
    }

    device->resetFences({*inFlightFences[currentFrame]});

    vk::raii::CommandBuffer &commandBuffer = commandBuffers[currentFrame];
    if (captureRequested)
    {
        ensureCaptureBuffer();
    }
    recordFrameCommands(commandBuffer, imageIndex);

    vk::PipelineStageFlags waitStage = vk::PipelineStageFlagBits::eColorAttachmentOutput;
    vk::SubmitInfo         submitInfo;
    submitInfo.setWaitSemaphores(*imageAvailableSemaphores[FrameSyncPolicy::acquireSemaphoreIndex(currentFrame)]);
    submitInfo.setWaitDstStageMask(waitStage);
    submitInfo.setCommandBuffers(*commandBuffer);
    submitInfo.setSignalSemaphores(
        *renderingFinishedSemaphores[FrameSyncPolicy::renderFinishedSemaphoreIndex(imageIndex)]
    );
    graphicsQueue.submit(submitInfo, *inFlightFences[currentFrame]);

    if (captureRequested)
    {
        readbackCapturePixels(logger);
        captureRequested = false;
    }

    vk::PresentInfoKHR presentInfo;
    presentInfo.setWaitSemaphores(
        *renderingFinishedSemaphores[FrameSyncPolicy::renderFinishedSemaphoreIndex(imageIndex)]
    );
    presentInfo.setSwapchains(**swapchain);
    presentInfo.setImageIndices(imageIndex);
    try
    {
        // eSuboptimalKHR is a success code here: presentation worked, but the
        // surface no longer matches, so the swapchain is recreated next frame.
        if (presentQueue.presentKHR(presentInfo) == vk::Result::eSuboptimalKHR)
        {
            framebufferResized = true;
        }
    }
    catch (const vk::OutOfDateKHRError &)
    {
        framebufferResized = true;
    }

    ++frameCounter;
    currentFrame = (currentFrame + 1) % maxFramesInFlight;
}

void RendererState::initialize(
    WindowCapability &window, LoggerCapability &logger
)
{
    runtimeLogger                                         = &logger;
    const std::vector<WindowIdentifier> windowIdentifiers = window.windowIdentifiers();
    if (windowIdentifiers.empty())
    {
        throw std::runtime_error(logPrefix + "no window exists before renderer start");
    }
    targetWindow = windowIdentifiers.front();

    createInstance(window, logger);
    createDeviceAndSurface(window, logger);
    createSwapchain(window, logger);
    // The command pool backs the setup submissions (uploads, layout
    // transitions), so it must exist before the frame resources are built.
    createCommandAndSyncObjects();
    createFrameResources();
    createSceneGeometry();
    createPipelines();

    logger.logMessage(LogLevel::Information, "renderer module started");
}
} // namespace engine
