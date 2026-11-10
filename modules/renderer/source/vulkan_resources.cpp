

// GPU resources for the Vulkan backend: frame uniform ring, mesh/texture
// storage, and the material descriptor-set cache.
//
// The upload path is asynchronous: mesh/texture creation enqueues staging
// copies on the device context and the objects are usable once the queue
// retires them; destroy* paths wait (or were deferred by the backend) so no
// in-flight copy references freed memory.

#if defined(ENGINE_RENDERER_BACKEND_VULKAN)

#include "vulkan_resources.hpp"
#include "engine/renderer/capabilities.hpp"
#include "engine/renderer/material.hpp"
#include "engine/renderer/mesh.hpp"
#include "render_shared.hpp"
#include "vulkan_device.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <mutex>
#include <new>
#include <vector>
#include <vulkan/vulkan_core.h>

namespace engine
{
namespace renderer
{

// Builds the per-frame uniform buffers + descriptor sets, the descriptor
// pool (sized for framesInFlight frame sets + maxMaterialSetsPerRenderer + 1
// neutral set), and the neutral white / flat-normal 1x1 textures. Mipmaps
// require RGBA8 linear blit support and are disabled with a warning when the
// device cannot do them.
bool VulkanResources::create(const VulkanDeviceContext &device, std::uint32_t framesInFlight,
    VkDescriptorSetLayout frameSetLayout, VkDescriptorSetLayout materialSetLayout, bool mipmapsEnabled,
    float anisotropyLevel, std::uint32_t maximumTextures, std::uint32_t maximumMaterialSets)
{
    constexpr std::uint32_t maximumDescriptorMaterialSets
        = std::numeric_limits<std::uint32_t>::max() / 5u - 1u;
    const std::uint32_t effectiveMaterialSets = std::max(1u, maximumMaterialSets);
    if (framesInFlight == 0 || effectiveMaterialSets > maximumDescriptorMaterialSets
        || framesInFlight > std::numeric_limits<std::uint32_t>::max() - effectiveMaterialSets - 1u
        || device.device() == VK_NULL_HANDLE || frameSetLayout == VK_NULL_HANDLE
        || materialSetLayout == VK_NULL_HANDLE)
    {
        return false;
    }

    device_                = &device;
    materialSetLayout_     = materialSetLayout;
    mipmapsEnabled_        = mipmapsEnabled;
    anisotropyLevel_       = std::isfinite(anisotropyLevel) ? std::max(1.0f, anisotropyLevel) : 1.0f;
    maximumTextures_       = std::max(1u, maximumTextures);
    maximumMaterialSets_   = std::max(1u, maximumMaterialSets);
    VkFormatProperties rgba8Properties{};
    vkGetPhysicalDeviceFormatProperties(device.physicalDevice(), VK_FORMAT_R8G8B8A8_UNORM, &rgba8Properties);
    constexpr VkFormatFeatureFlags mipmapFeatures = VK_FORMAT_FEATURE_BLIT_SRC_BIT | VK_FORMAT_FEATURE_BLIT_DST_BIT
                                                  | VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT;
    mipmapBlitSupported_ = (rgba8Properties.optimalTilingFeatures & mipmapFeatures) == mipmapFeatures;
    if (mipmapsEnabled_ && !mipmapBlitSupported_)
    {
        std::fprintf(stderr,
            "renderer module: RGBA8 linear blitting is unsupported; texture mipmaps are disabled for this device\n");
    }
    const VkDevice vulkanDevice = device.device();

    meshes_.reserve(64);
    textures_.reserve(maximumTextures_);
    materialSets_.reserve(maximumMaterialSets_);

    frameUniformBuffers_.resize(framesInFlight, VK_NULL_HANDLE);
    frameUniformAllocations_.resize(framesInFlight, nullptr);
    frameUniformMappings_.resize(framesInFlight, nullptr);
    frameUniformVersions_.resize(framesInFlight, 0);
    for (std::uint32_t index = 0; index < framesInFlight; ++index)
    {
        if (!device.createBuffer(sizeof(FrameUniformData),
                VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                BufferMemoryIntent::HostSequentialWrite,
                &frameUniformBuffers_[index],
                &frameUniformAllocations_[index]))
        {
            std::fprintf(stderr, "renderer module: could not allocate the frame uniform buffer\n");
            return false;
        }
        if (!device.mapAllocation(frameUniformAllocations_[index], &frameUniformMappings_[index]))
        {
            std::fprintf(stderr, "renderer module: could not persistently map the frame uniform buffer\n");
            return false;
        }
    }

    // Probe storage (set 0 binding 1): sized for the full lumen_gi probe
    // bound and zero-filled so the inactive hook reads nothing.
    probeStorageBuffers_.resize(framesInFlight, VK_NULL_HANDLE);
    probeStorageAllocations_.resize(framesInFlight, nullptr);
    probeStorageMappings_.resize(framesInFlight, nullptr);
    probeStorageVersions_.resize(framesInFlight, 0);
    for (std::uint32_t index = 0; index < framesInFlight; ++index)
    {
        if (!device.createBuffer(probeStorageBytes_,
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                BufferMemoryIntent::HostSequentialWrite,
                &probeStorageBuffers_[index],
                &probeStorageAllocations_[index]))
        {
            std::fprintf(stderr, "renderer module: could not allocate the probe storage buffer\n");
            return false;
        }
        if (!device.mapAllocation(probeStorageAllocations_[index], &probeStorageMappings_[index]))
        {
            std::fprintf(stderr, "renderer module: could not persistently map the probe storage buffer\n");
            return false;
        }
        std::memset(probeStorageMappings_[index], 0, static_cast<std::size_t>(probeStorageBytes_));
    }

    {
        const VkDescriptorPoolSize poolSizes[3]
            = {{.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, .descriptorCount = framesInFlight},
                {.type            = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                    .descriptorCount = framesInFlight},
                {.type               = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    .descriptorCount = 5 * (maximumMaterialSets_ + 1)}};
        const VkDescriptorPoolCreateInfo poolInfo{
            .sType         = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO,
            .pNext         = nullptr,
            .flags         = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT,
            .maxSets       = framesInFlight + maximumMaterialSets_ + 1,
            .poolSizeCount = 3,
            .pPoolSizes    = poolSizes,
        };
        if (vkCreateDescriptorPool(vulkanDevice, &poolInfo, nullptr, &descriptorPool_) != VK_SUCCESS)
        {
            std::fprintf(stderr, "renderer module: vkCreateDescriptorPool failed\n");
            return false;
        }
    }
    {
        frameUniformSets_.resize(framesInFlight);
        std::vector<VkDescriptorSetLayout> layouts(framesInFlight, frameSetLayout);
        const VkDescriptorSetAllocateInfo  setAllocate{
            .sType              = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO,
            .pNext              = nullptr,
            .descriptorPool     = descriptorPool_,
            .descriptorSetCount = framesInFlight,
            .pSetLayouts        = layouts.data(),
        };
        if (vkAllocateDescriptorSets(vulkanDevice, &setAllocate, frameUniformSets_.data()) != VK_SUCCESS)
        {
            std::fprintf(stderr, "renderer module: vkAllocateDescriptorSets failed\n");
            return false;
        }
        for (std::uint32_t index = 0; index < framesInFlight; ++index)
        {
            const VkDescriptorBufferInfo bufferInfo{frameUniformBuffers_[index], 0, sizeof(FrameUniformData)};
            const VkWriteDescriptorSet   write{
                .sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                .pNext            = nullptr,
                .dstSet           = frameUniformSets_[index],
                .dstBinding       = 0,
                .dstArrayElement  = 0,
                .descriptorCount  = 1,
                .descriptorType   = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                .pImageInfo       = nullptr,
                .pBufferInfo      = &bufferInfo,
                .pTexelBufferView = nullptr,
            };
            const VkDescriptorBufferInfo probeBufferInfo{probeStorageBuffers_[index], 0, probeStorageBytes_};
            const VkWriteDescriptorSet   probeWrite{
                .sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                .pNext            = nullptr,
                .dstSet           = frameUniformSets_[index],
                .dstBinding       = 1,
                .dstArrayElement  = 0,
                .descriptorCount  = 1,
                .descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                .pImageInfo       = nullptr,
                .pBufferInfo      = &probeBufferInfo,
                .pTexelBufferView = nullptr,
            };
            const VkWriteDescriptorSet writes[2] = {write, probeWrite};
            vkUpdateDescriptorSets(vulkanDevice, 2, writes, 0, nullptr);
        }
    }

    {
        const std::uint8_t whitePixel[4] = {255, 255, 255, 255};
        if (!createNeutralTexture(whiteTexture_, whitePixel))
        {
            std::fprintf(stderr, "renderer module: could not create the neutral white texture\n");
            return false;
        }
        const std::uint8_t flatNormalPixel[4] = {128, 128, 255, 255};
        if (!createNeutralTexture(flatNormalTexture_, flatNormalPixel))
        {
            std::fprintf(stderr, "renderer module: could not create the flat normal texture\n");
            return false;
        }
    }

    {
        const VkDescriptorSetAllocateInfo setAllocate{
            .sType              = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO,
            .pNext              = nullptr,
            .descriptorPool     = descriptorPool_,
            .descriptorSetCount = 1,
            .pSetLayouts        = &materialSetLayout_,
        };
        if (vkAllocateDescriptorSets(vulkanDevice, &setAllocate, &fullyNeutralSet_) != VK_SUCCESS
            || !writeMaterialSet(fullyNeutralSet_, MaterialKey{}))
        {
            std::fprintf(stderr, "renderer module: could not allocate the neutral material set\n");
            return false;
        }
    }

    return true;
}

void VulkanResources::destroy()
{
    if (device_ == nullptr)
    {
        return;
    }
    const VkDevice vulkanDevice = device_->device();

    for (auto &entry : meshes_)
    {
        destroyMeshObject(entry.second);
    }
    meshes_.clear();

    destroyTextureObject(whiteTexture_);
    destroyTextureObject(flatNormalTexture_);
    for (auto &entry : textures_)
    {
        destroyTextureObject(entry.second);
    }
    textures_.clear();
    materialSets_.clear();
    lastMaterialKey_    = {};
    lastMaterialSet_    = VK_NULL_HANDLE;
    hasLastMaterialSet_ = false;
    fullyNeutralSet_    = VK_NULL_HANDLE;

    for (std::size_t index = 0; index < frameUniformBuffers_.size(); ++index)
    {
        if (index < frameUniformMappings_.size() && frameUniformMappings_[index] != nullptr
            && frameUniformAllocations_[index] != nullptr)
        {
            device_->unmapAllocation(frameUniformAllocations_[index]);
            frameUniformMappings_[index] = nullptr;
        }
        if (frameUniformBuffers_[index] != VK_NULL_HANDLE)
        {
            device_->destroyBuffer(frameUniformBuffers_[index], frameUniformAllocations_[index]);
        }
    }
    frameUniformBuffers_.clear();
    frameUniformAllocations_.clear();
    frameUniformMappings_.clear();
    frameUniformVersions_.clear();
    frameUniformSets_.clear();

    for (std::size_t index = 0; index < probeStorageBuffers_.size(); ++index)
    {
        if (probeStorageMappings_[index] != nullptr && probeStorageAllocations_[index] != nullptr)
        {
            device_->unmapAllocation(probeStorageAllocations_[index]);
            probeStorageMappings_[index] = nullptr;
        }
        if (probeStorageBuffers_[index] != VK_NULL_HANDLE)
        {
            device_->destroyBuffer(probeStorageBuffers_[index], probeStorageAllocations_[index]);
        }
    }
    probeStorageBuffers_.clear();
    probeStorageAllocations_.clear();
    probeStorageMappings_.clear();
    probeStorageVersions_.clear();

    if (descriptorPool_ != VK_NULL_HANDLE)
    {
        vkDestroyDescriptorPool(vulkanDevice, descriptorPool_, nullptr);
        descriptorPool_ = VK_NULL_HANDLE;
    }
    device_ = nullptr;
}

bool VulkanResources::updateFrameUniform(std::uint32_t frameIndex, const FrameUniformData &data, std::uint64_t revision)
{
    if (frameIndex >= frameUniformMappings_.size() || frameUniformMappings_[frameIndex] == nullptr
        || frameUniformVersions_[frameIndex] == revision)
    {
        return frameIndex < frameUniformMappings_.size() && frameUniformMappings_[frameIndex] != nullptr
            && frameUniformVersions_[frameIndex] == revision;
    }
    std::memcpy(frameUniformMappings_[frameIndex], &data, sizeof(FrameUniformData));
    if (!device_->flushAllocation(frameUniformAllocations_[frameIndex]))
        return false;
    frameUniformVersions_[frameIndex] = revision;
    return true;
}

bool VulkanResources::updateProbeStorage(
    std::uint32_t frameIndex, const void *packedRecords, VkDeviceSize recordBytes, std::uint64_t revision)
{
    if (frameIndex >= probeStorageMappings_.size() || probeStorageMappings_[frameIndex] == nullptr
        || recordBytes > probeStorageBytes_ || (recordBytes != 0 && packedRecords == nullptr))
    {
        return false;
    }
    if (probeStorageVersions_[frameIndex] == revision)
    {
        return true; // this slot already holds this snapshot
    }
    std::memset(probeStorageMappings_[frameIndex], 0, static_cast<std::size_t>(probeStorageBytes_));
    if (recordBytes != 0 && packedRecords != nullptr)
    {
        std::memcpy(probeStorageMappings_[frameIndex], packedRecords, static_cast<std::size_t>(recordBytes));
    }
    if (!device_->flushAllocation(probeStorageAllocations_[frameIndex]))
        return false;
    probeStorageVersions_[frameIndex] = revision;
    return true;
}

void VulkanResources::destroyMeshObject(VulkanMesh &mesh)
{
    if (mesh.vertexBuffer != VK_NULL_HANDLE)
    {
        device_->destroyBuffer(mesh.vertexBuffer, mesh.vertexAllocation);
        mesh.vertexBuffer     = VK_NULL_HANDLE;
        mesh.vertexAllocation = nullptr;
    }
    if (mesh.indexBuffer != VK_NULL_HANDLE)
    {
        device_->destroyBuffer(mesh.indexBuffer, mesh.indexAllocation);
        mesh.indexBuffer     = VK_NULL_HANDLE;
        mesh.indexAllocation = nullptr;
    }
}

// Allocates and interns a mesh: device-local vertex/index buffers uploaded
// through the staging queue. Bounding sphere is computed on the CPU here so
// drawMesh can cull without touching GPU data. Fails (and cleans up) when a
// handle is reused or an upload/allocation fails.
bool VulkanResources::createMesh(MeshHandle id, const MeshDescription &description, RenderError *error)
{
    VulkanMesh mesh;
    mesh.indexCount = description.indexCount;

    const auto *vertices = static_cast<const float *>(description.vertices);
    float       minimum[3]
        = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    float maximum[3]
        = {std::numeric_limits<float>::lowest(),
            std::numeric_limits<float>::lowest(),
            std::numeric_limits<float>::lowest()};
    for (std::uint32_t index = 0; index < description.vertexCount; ++index)
    {
        const float *position = vertices + static_cast<std::size_t>(index) * meshVertexFloatCount;
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            minimum[axis] = std::min(minimum[axis], position[axis]);
            maximum[axis] = std::max(maximum[axis], position[axis]);
        }
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        mesh.boundsCenter[axis] = (minimum[axis] + maximum[axis]) * 0.5f;
    }
    float radiusSquared = 0.0f;
    for (std::uint32_t index = 0; index < description.vertexCount; ++index)
    {
        const float *position = vertices + static_cast<std::size_t>(index) * meshVertexFloatCount;
        const float  x        = position[0] - mesh.boundsCenter[0];
        const float  y        = position[1] - mesh.boundsCenter[1];
        const float  z        = position[2] - mesh.boundsCenter[2];
        radiusSquared         = std::max(radiusSquared, x * x + y * y + z * z);
    }
    mesh.boundsRadius = std::sqrt(radiusSquared);
    mesh.boundsValid  = std::isfinite(mesh.boundsRadius);

    const VkDeviceSize vertexBytes
        = static_cast<VkDeviceSize>(description.vertexCount) * meshVertexFloatCount * sizeof(float);
    const VkDeviceSize indexBytes = static_cast<VkDeviceSize>(description.indexCount) * sizeof(std::uint32_t);
    // Device-local buffers filled by the copy below (hence TRANSFER_DST usage);
    // VMA handles the sub-allocation.
    const bool vertexCreated = device_->createBuffer(vertexBytes,
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        BufferMemoryIntent::DeviceLocal,
        &mesh.vertexBuffer,
        &mesh.vertexAllocation);
    const bool indexCreated  = vertexCreated
                            && device_->createBuffer(indexBytes,
                               VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                               BufferMemoryIntent::DeviceLocal,
                               &mesh.indexBuffer,
                               &mesh.indexAllocation);
    if (!vertexCreated || !indexCreated)
    {
        destroyMeshObject(mesh);
        std::fprintf(stderr, "renderer module: could not allocate mesh buffers\n");
        reportError(error, RenderError::CreationFailed);
        return false;
    }
    const BufferUpload uploads[2] = {
        {mesh.vertexBuffer, description.vertices, vertexBytes, VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT},
        {mesh.indexBuffer, description.indices, indexBytes, VK_ACCESS_INDEX_READ_BIT},
    };
    if (!device_->copyHostToDevice(uploads, 2))
    {
        std::fprintf(stderr, "renderer module: could not upload mesh data\n");
        destroyMeshObject(mesh);
        reportError(error, RenderError::CreationFailed);
        return false;
    }
    try
    {
        const auto [entry, inserted] = meshes_.emplace(id, mesh);
        (void)entry;
        if (!inserted)
        {
            device_->waitForUploads();
            destroyMeshObject(mesh);
            reportError(error, RenderError::CreationFailed);
            return false;
        }
    }
    catch (const std::bad_alloc &)
    {
        device_->waitForUploads();
        destroyMeshObject(mesh);
        reportError(error, RenderError::CreationFailed);
        return false;
    }
    reportError(error, RenderError::None);
    return true;
}

void VulkanResources::destroyMesh(MeshHandle mesh)
{
    const auto found = meshes_.find(mesh);
    if (found == meshes_.end())
    {
        return;
    }

    destroyMeshObject(found->second);
    meshes_.erase(found);
}

void VulkanResources::destroyTextureObject(VulkanTexture &texture)
{
    const VkDevice vulkanDevice = device_->device();
    if (texture.sampler != VK_NULL_HANDLE)
    {
        vkDestroySampler(vulkanDevice, texture.sampler, nullptr);
        texture.sampler = VK_NULL_HANDLE;
    }
    if (texture.view != VK_NULL_HANDLE)
    {
        vkDestroyImageView(vulkanDevice, texture.view, nullptr);
        texture.view = VK_NULL_HANDLE;
    }
    if (texture.image != VK_NULL_HANDLE)
    {
        device_->destroyImage(texture.image, texture.imageAllocation);
        texture.image           = VK_NULL_HANDLE;
        texture.imageAllocation = nullptr;
    }
}

bool VulkanResources::uploadImagePixels(
    VkImage image, std::uint32_t width, std::uint32_t height, std::uint32_t mipLevels, const std::uint8_t *pixels)
{
    const VkDeviceSize byteCount = static_cast<VkDeviceSize>(width) * height * 4;
    // The queue mutex is taken up front: staging spans are handed out under
    // it (fence-recycled pool blocks; see staging_pool.hpp), and the command
    // buffer below is enqueued with the same lock held.
    std::lock_guard queueLock(device_->queueMutex());

    UploadStaging staging;
    void         *mapped      = nullptr;
    VkDeviceSize  stagingBase = 0;
    if (device_->acquireStagingSpan(byteCount, &staging))
    {
        // Pooled path: persistently mapped block, no map/unmap, no VMA
        // allocation for this upload.
        mapped      = static_cast<std::uint8_t *>(staging.pooledHostBase) + staging.span.offset;
        stagingBase = static_cast<VkDeviceSize>(staging.span.offset);
    }
    else
    {
        // One-off path (unchanged semantics): oversized requests, pool at
        // its byte cap, or pool block creation failure.
        if (!device_->createBuffer(byteCount,
                VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                BufferMemoryIntent::HostSequentialWrite,
                &staging.oneOffBuffer,
                &staging.oneOffAllocation))
            return false;
        if (!device_->mapAllocation(staging.oneOffAllocation, &mapped))
        {
            device_->destroyBuffer(staging.oneOffBuffer, staging.oneOffAllocation);
            staging.oneOffBuffer     = VK_NULL_HANDLE;
            staging.oneOffAllocation = nullptr;
            return false;
        }
        std::memcpy(mapped, pixels, static_cast<std::size_t>(byteCount));
        const bool flushed = device_->flushAllocation(staging.oneOffAllocation);
        device_->unmapAllocation(staging.oneOffAllocation);
        if (!flushed)
        {
            device_->destroyBuffer(staging.oneOffBuffer, staging.oneOffAllocation);
            staging.oneOffBuffer     = VK_NULL_HANDLE;
            staging.oneOffAllocation = nullptr;
            return false;
        }
    }
    if (staging.span.block != StagingPool::invalidBlock)
    {
        std::memcpy(static_cast<std::uint8_t *>(staging.pooledHostBase) + staging.span.offset,
            pixels,
            static_cast<std::size_t>(byteCount));
        device_->commitStagingSpan(staging);
    }

    const VkCommandBufferAllocateInfo allocateInfo{
        .sType              = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
        .pNext              = nullptr,
        .commandPool        = device_->uploadPool(),
        .level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
        .commandBufferCount = 1,
    };
    VkCommandBuffer command = VK_NULL_HANDLE;
    if (vkAllocateCommandBuffers(device_->device(), &allocateInfo, &command) != VK_SUCCESS)
    {
        device_->discardUploadStaging(staging);
        return false;
    }

    const VkCommandBufferBeginInfo beginInfo{
        .sType            = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
        .pNext            = nullptr,
        .flags            = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT,
        .pInheritanceInfo = nullptr,
    };
    if (vkBeginCommandBuffer(command, &beginInfo) != VK_SUCCESS)
    {
        vkFreeCommandBuffers(device_->device(), device_->uploadPool(), 1, &command);
        device_->discardUploadStaging(staging);
        return false;
    }

    const VkImageMemoryBarrier toTransfer{
        .sType               = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
        .pNext               = nullptr,
        .srcAccessMask       = 0,
        .dstAccessMask       = VK_ACCESS_TRANSFER_WRITE_BIT,
        .oldLayout           = VK_IMAGE_LAYOUT_UNDEFINED,
        .newLayout           = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
        .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
        .image               = image,
        .subresourceRange    = {VK_IMAGE_ASPECT_COLOR_BIT, 0, mipLevels, 0, 1},
    };
    vkCmdPipelineBarrier(command,
        VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
        VK_PIPELINE_STAGE_TRANSFER_BIT,
        0,
        0,
        nullptr,
        0,
        nullptr,
        1,
        &toTransfer);

    const VkBufferImageCopy region{
        .bufferOffset      = stagingBase,
        .bufferRowLength   = 0,
        .bufferImageHeight = 0,
        .imageSubresource  = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1},
        .imageOffset       = {0, 0, 0},
        .imageExtent       = {width, height, 1},
    };
    vkCmdCopyBufferToImage(command, staging.stagingBuffer(), image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

    if (mipLevels > 1)
    {
        // Mip generation: blit level N-1 into level N, re-transitioning each
        // source level before reading it; both final barriers land the whole
        // chain in SHADER_READ_ONLY_OPTIMAL for the fragment shader.
        std::int32_t mipWidth  = static_cast<std::int32_t>(width);
        std::int32_t mipHeight = static_cast<std::int32_t>(height);
        for (std::uint32_t level = 1; level < mipLevels; ++level)
        {
            const VkImageMemoryBarrier toSource{
                .sType               = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
                .pNext               = nullptr,
                .srcAccessMask       = VK_ACCESS_TRANSFER_WRITE_BIT,
                .dstAccessMask       = VK_ACCESS_TRANSFER_READ_BIT,
                .oldLayout           = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                .newLayout           = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
                .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
                .image               = image,
                .subresourceRange    = {VK_IMAGE_ASPECT_COLOR_BIT, level - 1u, 1, 0, 1},
            };
            vkCmdPipelineBarrier(command,
                VK_PIPELINE_STAGE_TRANSFER_BIT,
                VK_PIPELINE_STAGE_TRANSFER_BIT,
                0,
                0,
                nullptr,
                0,
                nullptr,
                1,
                &toSource);

            const std::int32_t nextWidth  = std::max(mipWidth / 2, 1);
            const std::int32_t nextHeight = std::max(mipHeight / 2, 1);
            const VkImageBlit  blit{
                .srcSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, level - 1u, 0, 1},
                .srcOffsets     = {{0, 0, 0}, {mipWidth, mipHeight, 1}},
                .dstSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, level, 0, 1},
                .dstOffsets     = {{0, 0, 0}, {nextWidth, nextHeight, 1}},
            };
            vkCmdBlitImage(command,
                image,
                VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                image,
                VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                1,
                &blit,
                VK_FILTER_LINEAR);
            mipWidth  = nextWidth;
            mipHeight = nextHeight;
        }

        const VkImageMemoryBarrier finalBarriers[2]{
            {
                .sType               = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
                .pNext               = nullptr,
                .srcAccessMask       = VK_ACCESS_TRANSFER_READ_BIT,
                .dstAccessMask       = VK_ACCESS_SHADER_READ_BIT,
                .oldLayout           = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                .newLayout           = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
                .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
                .image               = image,
                .subresourceRange    = {VK_IMAGE_ASPECT_COLOR_BIT, 0, mipLevels - 1u, 0, 1},
            },
            {
                .sType               = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
                .pNext               = nullptr,
                .srcAccessMask       = VK_ACCESS_TRANSFER_WRITE_BIT,
                .dstAccessMask       = VK_ACCESS_SHADER_READ_BIT,
                .oldLayout           = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                .newLayout           = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
                .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
                .image               = image,
                .subresourceRange    = {VK_IMAGE_ASPECT_COLOR_BIT, mipLevels - 1u, 1, 0, 1},
            },
        };
        vkCmdPipelineBarrier(command,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            0,
            0,
            nullptr,
            0,
            nullptr,
            2,
            finalBarriers);
    }
    else
    {
        const VkImageMemoryBarrier toShaderRead{
            .sType               = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
            .pNext               = nullptr,
            .srcAccessMask       = VK_ACCESS_TRANSFER_WRITE_BIT,
            .dstAccessMask       = VK_ACCESS_SHADER_READ_BIT,
            .oldLayout           = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            .newLayout           = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .image               = image,
            .subresourceRange    = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
        };
        vkCmdPipelineBarrier(command,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
            0,
            0,
            nullptr,
            0,
            nullptr,
            1,
            &toShaderRead);
    }

    if (vkEndCommandBuffer(command) != VK_SUCCESS || !device_->enqueueUploadLocked(command, staging))
    {
        vkFreeCommandBuffers(device_->device(), device_->uploadPool(), 1, &command);
        device_->discardUploadStaging(staging);
        return false;
    }
    return true;
}

// Creates the image (mips = full chain when allowed/enabled/supported), the
// view, and a sampler with the requested filter/address mode/anisotropy,
// then queues the pixel upload.
bool VulkanResources::createTextureObjects(VulkanTexture &texture, std::uint32_t width, std::uint32_t height,
    const std::uint8_t *pixels, TextureFilter filter, TextureAddress address, bool allowMipmaps)
{
    // Mip level count: full chain (bit_width of the larger dimension) when allowed.
    const std::uint32_t mipLevels
        = allowMipmaps && mipmapsEnabled_ && mipmapBlitSupported_ ? std::bit_width(std::max(width, height)) : 1u;
    texture.mipLevels            = mipLevels;
    VkImageUsageFlags imageUsage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    if (mipLevels > 1u)
        imageUsage |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    if (!device_->createImage(width,
            height,
            mipLevels,

            VK_FORMAT_R8G8B8A8_UNORM,
            imageUsage,
            &texture.image,
            &texture.imageAllocation))
    {
        std::fprintf(stderr, "renderer module: could not create the texture image\n");
        return false;
    }
    const VkImageViewCreateInfo viewInfo{
        .sType    = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
        .pNext    = nullptr,
        .flags    = 0,
        .image    = texture.image,
        .viewType = VK_IMAGE_VIEW_TYPE_2D,
        .format   = VK_FORMAT_R8G8B8A8_UNORM,
        .components
        = {VK_COMPONENT_SWIZZLE_IDENTITY,
            VK_COMPONENT_SWIZZLE_IDENTITY,
            VK_COMPONENT_SWIZZLE_IDENTITY,
            VK_COMPONENT_SWIZZLE_IDENTITY},
        .subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, mipLevels, 0, 1},
    };
    if (vkCreateImageView(device_->device(), &viewInfo, nullptr, &texture.view) != VK_SUCCESS)
    {
        destroyTextureObject(texture);
        return false;
    }

    const VkFilter             magnification = filter == TextureFilter::Nearest ? VK_FILTER_NEAREST : VK_FILTER_LINEAR;
    const VkSamplerAddressMode addressMode
        = address == TextureAddress::ClampEdge ? VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE : VK_SAMPLER_ADDRESS_MODE_REPEAT;
    // anisotropyEnable must pair with a maxAnisotropy clamped to the device
    // limit: a level above limits.maxSamplerAnisotropy is a valid-usage
    // violation, and a failed vkCreateSampler would turn into a texture-load
    // failure (falling back to neutral stand-ins) for every texture that
    // shares the filter mode.
    float effectiveAnisotropy = 1.0f;
    if (device_->profile().samplerAnisotropyAvailable && anisotropyLevel_ > 1.0f)
    {
        const float deviceLimit = device_->maxSamplerAnisotropy();
        effectiveAnisotropy     = deviceLimit > 1.0f ? std::min(anisotropyLevel_, deviceLimit) : 1.0f;
    }
    const VkSamplerCreateInfo samplerInfo{
        .sType      = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO,
        .pNext      = nullptr,
        .flags      = 0,
        .magFilter  = magnification,
        .minFilter  = magnification,
        .mipmapMode = filter == TextureFilter::Nearest ? VK_SAMPLER_MIPMAP_MODE_NEAREST : VK_SAMPLER_MIPMAP_MODE_LINEAR,
        .addressModeU = addressMode,
        .addressModeV = addressMode,
        .addressModeW = addressMode,
        .mipLodBias   = 0.0f,

        .anisotropyEnable = effectiveAnisotropy > 1.0f ? VK_TRUE : VK_FALSE,
        .maxAnisotropy    = effectiveAnisotropy,
        .compareEnable           = VK_FALSE,
        .compareOp               = VK_COMPARE_OP_NEVER,
        .minLod                  = 0.0f,
        .maxLod                  = static_cast<float>(mipLevels - 1u),
        .borderColor             = VK_BORDER_COLOR_INT_OPAQUE_BLACK,
        .unnormalizedCoordinates = VK_FALSE,
    };
    if (vkCreateSampler(device_->device(), &samplerInfo, nullptr, &texture.sampler) != VK_SUCCESS)
    {
        destroyTextureObject(texture);
        return false;
    }
    if (!uploadImagePixels(texture.image, width, height, mipLevels, pixels))
    {
        std::fprintf(stderr, "renderer module: could not enqueue texture upload\n");
        destroyTextureObject(texture);
        return false;
    }
    return true;
}

bool VulkanResources::createTexture(TextureHandle id, const TextureDescription &description, RenderError *error)
{
    if (description.pixels == nullptr || description.width == 0 || description.height == 0)
    {
        reportError(error, RenderError::InvalidTexture);
        return false;
    }
    if (textures_.size() >= maximumTextures_)
    {
        // Configured per-renderer ceiling; no eviction - creation fails.
        std::fprintf(stderr,
            "renderer module: renderer already holds %u textures; texture creation rejected\n",
            (unsigned)maximumTextures_);
        reportError(error, RenderError::CreationFailed);
        return false;
    }

    VulkanTexture texture;
    if (!createTextureObjects(texture,
            description.width,
            description.height,
            description.pixels,
            description.filter,
            description.address,
            true))
    {
        reportError(error, RenderError::CreationFailed);
        return false;
    }

    try
    {
        const auto [entry, inserted] = textures_.emplace(id, texture);
        (void)entry;
        if (!inserted)
        {
            device_->waitForUploads();
            destroyTextureObject(texture);
            reportError(error, RenderError::CreationFailed);
            return false;
        }
    }
    catch (const std::bad_alloc &)
    {
        device_->waitForUploads();
        destroyTextureObject(texture);
        reportError(error, RenderError::CreationFailed);
        return false;
    }
    reportError(error, RenderError::None);
    return true;
}

bool VulkanResources::createNeutralTexture(VulkanTexture &destination, const std::uint8_t pixel[4])
{
    return createTextureObjects(destination, 1, 1, pixel, TextureFilter::Nearest, TextureAddress::Repeat, false);
}

// Frees the texture and every material set (plus the lookup cache) that
// referenced it; sets are returned to the descriptor pool.
void VulkanResources::destroyTexture(TextureHandle textureHandle)
{
    const auto found = textures_.find(textureHandle);
    if (found == textures_.end())
    {
        return;
    }

    if (hasLastMaterialSet_
        && std::find(lastMaterialKey_.begin(), lastMaterialKey_.end(), textureHandle) != lastMaterialKey_.end())
    {
        lastMaterialKey_    = {};
        lastMaterialSet_    = VK_NULL_HANDLE;
        hasLastMaterialSet_ = false;
    }
    for (auto cacheEntry = materialSets_.begin(); cacheEntry != materialSets_.end();)
    {
        if (std::find(cacheEntry->first.begin(), cacheEntry->first.end(), textureHandle) != cacheEntry->first.end())
        {
            const VkDescriptorSet set = cacheEntry->second;
            vkFreeDescriptorSets(device_->device(), descriptorPool_, 1, &set);
            cacheEntry = materialSets_.erase(cacheEntry);
        }
        else
        {
            ++cacheEntry;
        }
    }
    destroyTextureObject(found->second);
    textures_.erase(found);
}

// Writes the five combined image-sampler slots; missing handles fall back to
// the neutral textures (white for color-ish slots, flat normal for slot 1).
bool VulkanResources::writeMaterialSet(VkDescriptorSet set, const MaterialKey &key)
{
    const auto textureOr = [this](TextureHandle handle, const VulkanTexture &fallback)
    {
        const auto found = textures_.find(handle);
        return found != textures_.end() ? &found->second : &fallback;
    };
    const VulkanTexture *slotTextures[5] = {
        textureOr(key[0], whiteTexture_),
        textureOr(key[1], flatNormalTexture_),
        textureOr(key[2], whiteTexture_),
        textureOr(key[3], whiteTexture_),
        textureOr(key[4], whiteTexture_),
    };

    VkDescriptorImageInfo imageInfos[5];
    VkWriteDescriptorSet  writes[5];
    for (std::uint32_t slot = 0; slot < 5; ++slot)
    {
        imageInfos[slot] = VkDescriptorImageInfo{
            .sampler     = slotTextures[slot]->sampler,
            .imageView   = slotTextures[slot]->view,
            .imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
        };
        writes[slot] = VkWriteDescriptorSet{
            .sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
            .pNext            = nullptr,
            .dstSet           = set,
            .dstBinding       = slot,
            .dstArrayElement  = 0,
            .descriptorCount  = 1,
            .descriptorType   = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            .pImageInfo       = &imageInfos[slot],
            .pBufferInfo      = nullptr,
            .pTexelBufferView = nullptr,
        };
    }
    vkUpdateDescriptorSets(device_->device(), 5, writes, 0, nullptr);
    return true;
}

// Interns a material descriptor set for the material's five texture handles.
// Cache exhausted or allocation failed -> the fully neutral set is returned
// (a visible-but-benign degradation, not an error).
VkDescriptorSet VulkanResources::materialSet(const MeshMaterial &material)
{
    const MaterialKey key{
        material.baseColorTexture,
        material.normalTexture,
        material.metallicRoughnessTexture,
        material.occlusionTexture,
        material.emissiveTexture};
    if (hasLastMaterialSet_ && key == lastMaterialKey_)
    {
        return lastMaterialSet_;
    }
    const auto found = materialSets_.find(key);
    if (found != materialSets_.end())
    {
        lastMaterialKey_    = key;
        lastMaterialSet_    = found->second;
        hasLastMaterialSet_ = true;
        return found->second;
    }
    if (materialSets_.size() >= maximumMaterialSets_)
    {
        std::fprintf(stderr,
            "renderer module: renderer already holds %u material sets; binding the neutral set\n",
            (unsigned)maximumMaterialSets_);
        return fullyNeutralSet_;
    }

    VkDescriptorSet                   set = VK_NULL_HANDLE;
    const VkDescriptorSetAllocateInfo setAllocate{
        .sType              = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO,
        .pNext              = nullptr,
        .descriptorPool     = descriptorPool_,
        .descriptorSetCount = 1,
        .pSetLayouts        = &materialSetLayout_,
    };
    if (vkAllocateDescriptorSets(device_->device(), &setAllocate, &set) != VK_SUCCESS || !writeMaterialSet(set, key))
    {
        if (set != VK_NULL_HANDLE)
        {
            vkFreeDescriptorSets(device_->device(), descriptorPool_, 1, &set);
        }
        std::fprintf(stderr, "renderer module: could not allocate a material descriptor set\n");
        return fullyNeutralSet_;
    }
    try
    {
        const auto [entry, inserted] = materialSets_.emplace(key, set);
        (void)entry;
        if (!inserted)
        {
            vkFreeDescriptorSets(device_->device(), descriptorPool_, 1, &set);
            return fullyNeutralSet_;
        }
    }
    catch (const std::bad_alloc &)
    {
        vkFreeDescriptorSets(device_->device(), descriptorPool_, 1, &set);
        return fullyNeutralSet_;
    }
    lastMaterialKey_    = key;
    lastMaterialSet_    = set;
    hasLastMaterialSet_ = true;
    return set;
}

const VulkanResources::VulkanMesh *VulkanResources::meshFor(MeshHandle mesh) const
{
    const auto found = meshes_.find(mesh);
    return found != meshes_.end() ? &found->second : nullptr;
}

} // namespace renderer
} // namespace engine

#endif
