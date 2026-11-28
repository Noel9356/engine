

// GPU resources for the Vulkan backend: frame uniform ring, mesh/texture
// storage, and the material descriptor-set cache.
//
// The upload path is asynchronous: mesh/texture creation enqueues staging
// copies on the device context and the objects are usable once the queue
// retires them; destroy* paths wait (or were deferred by the backend) so no
// in-flight copy references freed memory.

#if defined(ENGINE_RENDERER_BACKEND_VULKAN)

#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/material.hpp>
#include <engine/renderer/mesh.hpp>
#include "render_shared.hpp"
#include "vulkan_device.hpp"
#include "vulkan_resources.hpp"

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
    VkDescriptorSetLayout frameSetLayout, VkDescriptorSetLayout materialSetLayout,
    VkDescriptorSetLayout materialExtensionSetLayout, bool mipmapsEnabled,
    float anisotropyLevel, std::uint32_t maximumTextures, std::uint32_t maximumMaterialSets)
{
    constexpr std::uint32_t maximumDescriptorMaterialSets
        = std::numeric_limits<std::uint32_t>::max() / 6u - 1u;
    const std::uint32_t effectiveMaterialSets = std::max(1u, maximumMaterialSets);
    if (framesInFlight == 0 || effectiveMaterialSets > maximumDescriptorMaterialSets
        || framesInFlight > std::numeric_limits<std::uint32_t>::max() - effectiveMaterialSets - 1u
        || device.device() == VK_NULL_HANDLE || frameSetLayout == VK_NULL_HANDLE
        || materialSetLayout == VK_NULL_HANDLE || materialExtensionSetLayout == VK_NULL_HANDLE)
    {
        return false;
    }

    device_                = &device;
    materialSetLayout_     = materialSetLayout;
    materialExtensionSetLayout_ = materialExtensionSetLayout;
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
    // Block-format support for the KTX path: a format is offered only when
    // the device samples it optimally tiled and can linearly filter it, so a
    // transcoded BC payload never reaches a device that would sample it
    // nearest-only (or reject the image outright).
    const auto blockSupport = [physicalDevice = device.physicalDevice()](VkFormat format)
    {
        VkFormatProperties properties{};
        vkGetPhysicalDeviceFormatProperties(physicalDevice, format, &properties);
        constexpr VkFormatFeatureFlags blockFeatures
            = VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT | VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT;
        return (properties.optimalTilingFeatures & blockFeatures) == blockFeatures;
    };
    // UNORM variants only: the module's texture pipeline is byte-linear
    // (createTexture feeds UNORM RGBA8 and the shaders treat the samples as
    // linear), so block payloads take the UNORM variant to reproduce the
    // RGBA path's samples byte for byte.
    blockFormatSupport_.bc1 = blockSupport(VK_FORMAT_BC1_RGB_UNORM_BLOCK);
    blockFormatSupport_.bc3 = blockSupport(VK_FORMAT_BC3_UNORM_BLOCK);
    blockFormatSupport_.bc4 = blockSupport(VK_FORMAT_BC4_UNORM_BLOCK);
    blockFormatSupport_.bc5 = blockSupport(VK_FORMAT_BC5_UNORM_BLOCK);
    blockFormatSupport_.bc7 = blockSupport(VK_FORMAT_BC7_UNORM_BLOCK);
    const VkDevice vulkanDevice = device.device();

    meshes_.reserve(64);
    textures_.reserve(maximumTextures_);
    materialSets_.reserve(maximumMaterialSets_);

    // Material extension ring (set 2): the block stride is the 48-byte
    // payload padded to the device's uniform buffer offset alignment, so
    // every block's offset is a legal dynamic offset.
    {
        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(device.physicalDevice(), &properties);
        const VkDeviceSize minAlignment = properties.limits.minUniformBufferOffsetAlignment;
        VkDeviceSize stride = sizeof(MaterialExtensionBlock);
        stride              = (stride + minAlignment - 1) / minAlignment * minAlignment;
        materialExtensionStride_ = static_cast<std::uint32_t>(std::max<std::uint64_t>(stride, 16));
    }
    materialExtensionBuffers_.resize(framesInFlight, VK_NULL_HANDLE);
    materialExtensionAllocations_.resize(framesInFlight, nullptr);
    materialExtensionMappings_.resize(framesInFlight, nullptr);
    materialExtensionSets_.resize(framesInFlight, VK_NULL_HANDLE);
    materialExtensionTuples_.resize(framesInFlight);
    materialExtensionCursors_.resize(framesInFlight, 0);
    for (std::uint32_t index = 0; index < framesInFlight; ++index)
    {
        materialExtensionTuples_[index].resize(kMaterialExtensionCapacity);
        if (!device.createBuffer(static_cast<VkDeviceSize>(materialExtensionStride_) * kMaterialExtensionCapacity,
                VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                BufferMemoryIntent::HostSequentialWrite,
                &materialExtensionBuffers_[index],
                &materialExtensionAllocations_[index]))
        {
            std::fprintf(stderr, "renderer module: could not allocate the material extension buffer\n");
            return false;
        }
        if (!device.mapAllocation(materialExtensionAllocations_[index], &materialExtensionMappings_[index]))
        {
            std::fprintf(stderr, "renderer module: could not persistently map the material extension buffer\n");
            return false;
        }
        // Block zero is the all-off default every extension-less material
        // binds (fog off, mottle off, detail off).
        MaterialExtensionBlock neutral{};
        std::memcpy(materialExtensionMappings_[index], &neutral, sizeof(neutral));
    }

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

    // Probe storage (set 0 binding 1): sized for the full radiance_cache probe
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
            = {{.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, .descriptorCount = 2 * framesInFlight},
                {.type            = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                    .descriptorCount = framesInFlight},
                {.type               = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    .descriptorCount = 6 * (maximumMaterialSets_ + 1) + framesInFlight}};
        const VkDescriptorPoolCreateInfo poolInfo{
            .sType         = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO,
            .pNext         = nullptr,
            .flags         = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT,
            .maxSets       = 2 * framesInFlight + maximumMaterialSets_ + 1,
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
        // The extension sets (one per frame in flight) bind the whole ring
        // with the block size as the descriptor's range; the dynamic offset
        // selects the draw's block.
        for (std::uint32_t index = 0; index < framesInFlight; ++index)
        {
            const VkDescriptorSetAllocateInfo setAllocate{
                .sType              = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO,
                .pNext              = nullptr,
                .descriptorPool     = descriptorPool_,
                .descriptorSetCount = 1,
                .pSetLayouts        = &materialExtensionSetLayout_,
            };
            if (vkAllocateDescriptorSets(vulkanDevice, &setAllocate, &materialExtensionSets_[index]) != VK_SUCCESS)
            {
                std::fprintf(stderr, "renderer module: could not allocate the material extension set\n");
                return false;
            }
            const VkDescriptorBufferInfo bufferInfo{materialExtensionBuffers_[index], 0,
                sizeof(MaterialExtensionBlock)};
            const VkWriteDescriptorSet write{
                .sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                .pNext            = nullptr,
                .dstSet           = materialExtensionSets_[index],
                .dstBinding       = 0,
                .dstArrayElement  = 0,
                .descriptorCount  = 1,
                .descriptorType   = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC,
                .pImageInfo       = nullptr,
                .pBufferInfo      = &bufferInfo,
                .pTexelBufferView = nullptr,
            };
            vkUpdateDescriptorSets(vulkanDevice, 1, &write, 0, nullptr);
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

    // Set 0 binding 2 (the reflection-field sampler) starts on the neutral
    // white texture: its alpha (1.0) is the module's neutral "never traced"
    // roughness, and the frame uniform keeps the hook inactive until a real
    // field arrives, so the stand-in is never read with meaning.
    bindNeutralFieldSampler();

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

    destroyReflectionFieldObjects();

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

    for (std::size_t index = 0; index < materialExtensionBuffers_.size(); ++index)
    {
        if (index < materialExtensionMappings_.size() && materialExtensionMappings_[index] != nullptr
            && materialExtensionAllocations_[index] != nullptr)
        {
            device_->unmapAllocation(materialExtensionAllocations_[index]);
            materialExtensionMappings_[index] = nullptr;
        }
        if (materialExtensionBuffers_[index] != VK_NULL_HANDLE)
        {
            device_->destroyBuffer(materialExtensionBuffers_[index], materialExtensionAllocations_[index]);
        }
    }
    materialExtensionBuffers_.clear();
    materialExtensionAllocations_.clear();
    materialExtensionMappings_.clear();
    materialExtensionSets_.clear();
    materialExtensionTuples_.clear();
    materialExtensionCursors_.clear();

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
    mesh.indexCount  = description.indexCount;
    mesh.vertexBytes = static_cast<VkDeviceSize>(description.vertexCount) * meshVertexFloatCount * sizeof(float);

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

    const VkDeviceSize vertexBytes = mesh.vertexBytes;
    const VkDeviceSize indexBytes  = static_cast<VkDeviceSize>(description.indexCount) * sizeof(std::uint32_t);
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

bool VulkanResources::prepareMeshVertexUpdate(MeshHandle mesh, const void *vertices, std::uint32_t vertexCount,
    VkBuffer *outVertexBuffer, VkDeviceSize *outVertexBytes, RenderError *error)
{
    const auto found = meshes_.find(mesh);
    if (found == meshes_.end())
    {
        reportError(error, RenderError::InvalidMesh);
        return false;
    }
    VulkanMesh &record = found->second;
    const VkDeviceSize updateBytes
        = static_cast<VkDeviceSize>(vertexCount) * meshVertexFloatCount * sizeof(float);
    if (vertices == nullptr || vertexCount == 0 || updateBytes != record.vertexBytes)
    {
        // The update must cover the whole creation stream: a partial or
        // oversized write would desynchronize the vertex layout the pipeline
        // binds (or leave stale vertices behind the new tail).
        reportError(error, RenderError::InvalidMesh);
        return false;
    }

    // Refresh the bounding sphere from the new positions so draw-time
    // frustum culling sees the simulated surface, not its rest pose.
    const auto *positions = static_cast<const float *>(vertices);
    float minimum[3]
        = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    float maximum[3]
        = {std::numeric_limits<float>::lowest(),
            std::numeric_limits<float>::lowest(),
            std::numeric_limits<float>::lowest()};
    for (std::uint32_t index = 0; index < vertexCount; ++index)
    {
        const float *position = positions + static_cast<std::size_t>(index) * meshVertexFloatCount;
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            minimum[axis] = std::min(minimum[axis], position[axis]);
            maximum[axis] = std::max(maximum[axis], position[axis]);
        }
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        record.boundsCenter[axis] = (minimum[axis] + maximum[axis]) * 0.5f;
    }
    float radiusSquared = 0.0f;
    for (std::uint32_t index = 0; index < vertexCount; ++index)
    {
        const float *position = positions + static_cast<std::size_t>(index) * meshVertexFloatCount;
        const float  x        = position[0] - record.boundsCenter[0];
        const float  y        = position[1] - record.boundsCenter[1];
        const float  z        = position[2] - record.boundsCenter[2];
        radiusSquared         = std::max(radiusSquared, x * x + y * y + z * z);
    }
    record.boundsRadius = std::sqrt(radiusSquared);
    record.boundsValid  = std::isfinite(record.boundsRadius);

    *outVertexBuffer = record.vertexBuffer;
    *outVertexBytes  = record.vertexBytes;
    reportError(error, RenderError::None);
    return true;
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

// VkFormat for each supported block format (UNORM variants; see the
// blockFormatSupport_ comment in create()).
VkFormat blockVkFormat(BlockTextureFormat format)
{
    switch (format)
    {
    case BlockTextureFormat::Bc1Rgb:
        return VK_FORMAT_BC1_RGB_UNORM_BLOCK;
    case BlockTextureFormat::Bc3Rgba:
        return VK_FORMAT_BC3_UNORM_BLOCK;
    case BlockTextureFormat::Bc4R:
        return VK_FORMAT_BC4_UNORM_BLOCK;
    case BlockTextureFormat::Bc5Rg:
        return VK_FORMAT_BC5_UNORM_BLOCK;
    case BlockTextureFormat::Bc7Rgba:
        return VK_FORMAT_BC7_UNORM_BLOCK;
    }
    return VK_FORMAT_BC7_UNORM_BLOCK;
}

// Extent of one mip level of a block texture.
std::uint32_t blockLevelExtent(std::uint32_t base, std::uint32_t level)
{
    return std::max(1u, base >> level);
}

bool VulkanResources::createBlockTexture(TextureHandle id, const BlockTextureDescription &description,
    RenderError *error)
{
    if (description.pixels == nullptr || description.levelBytes == nullptr || description.width == 0
        || description.height == 0 || description.levelCount == 0
        || !blockFormatSupport().bc1 && !blockFormatSupport().bc3 && !blockFormatSupport().bc4
        && !blockFormatSupport().bc5 && !blockFormatSupport().bc7)
    {
        reportError(error, RenderError::InvalidTexture);
        return false;
    }
    // The requested format must be one this device actually samples.
    const bool formatSupported = [this, format = description.format]()
    {
        switch (format)
        {
        case BlockTextureFormat::Bc1Rgb:
            return blockFormatSupport_.bc1;
        case BlockTextureFormat::Bc3Rgba:
            return blockFormatSupport_.bc3;
        case BlockTextureFormat::Bc4R:
            return blockFormatSupport_.bc4;
        case BlockTextureFormat::Bc5Rg:
            return blockFormatSupport_.bc5;
        case BlockTextureFormat::Bc7Rgba:
            return blockFormatSupport_.bc7;
        }
        return false;
    }();
    if (!formatSupported)
    {
        reportError(error, RenderError::InvalidTexture);
        return false;
    }
    if (textures_.size() >= maximumTextures_)
    {
        std::fprintf(stderr,
            "renderer module: renderer already holds %u textures; texture creation rejected\n",
            (unsigned)maximumTextures_);
        reportError(error, RenderError::CreationFailed);
        return false;
    }

    VulkanTexture texture;
    const VkFormat format = blockVkFormat(description.format);
    texture.mipLevels    = description.levelCount;
    if (!device_->createImage(description.width,
            description.height,
            description.levelCount,
            format,
            VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            &texture.image,
            &texture.imageAllocation))
    {
        std::fprintf(stderr, "renderer module: could not create the block texture image\n");
        return false;
    }
    const VkImageViewCreateInfo viewInfo{
        .sType           = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
        .pNext           = nullptr,
        .flags           = 0,
        .image           = texture.image,
        .viewType        = VK_IMAGE_VIEW_TYPE_2D,
        .format          = format,
        .components
        = {VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY,
            VK_COMPONENT_SWIZZLE_IDENTITY},
        .subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, description.levelCount, 0, 1},
    };
    if (vkCreateImageView(device_->device(), &viewInfo, nullptr, &texture.view) != VK_SUCCESS)
    {
        destroyTextureObject(texture);
        return false;
    }

    // Sampler: identical policy to createTextureObjects (mip chain from the
    // file, so maxLod covers every level the file provides).
    const VkFilter             magnification = description.filter == TextureFilter::Nearest ? VK_FILTER_NEAREST : VK_FILTER_LINEAR;
    const VkSamplerAddressMode addressMode
        = description.address == TextureAddress::ClampEdge ? VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE
                                                            : VK_SAMPLER_ADDRESS_MODE_REPEAT;
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
        .mipmapMode = description.filter == TextureFilter::Nearest ? VK_SAMPLER_MIPMAP_MODE_NEAREST
                                                                   : VK_SAMPLER_MIPMAP_MODE_LINEAR,
        .addressModeU = addressMode,
        .addressModeV = addressMode,
        .addressModeW = addressMode,
        .mipLodBias   = 0.0f,
        .anisotropyEnable = effectiveAnisotropy > 1.0f ? VK_TRUE : VK_FALSE,
        .maxAnisotropy    = effectiveAnisotropy,
        .compareEnable           = VK_FALSE,
        .compareOp               = VK_COMPARE_OP_NEVER,
        .minLod                  = 0.0f,
        .maxLod                  = static_cast<float>(description.levelCount - 1u),
        .borderColor             = VK_BORDER_COLOR_INT_OPAQUE_BLACK,
        .unnormalizedCoordinates = VK_FALSE,
    };
    if (vkCreateSampler(device_->device(), &samplerInfo, nullptr, &texture.sampler) != VK_SUCCESS)
    {
        destroyTextureObject(texture);
        return false;
    }
    if (!uploadImageBlocks(texture.image, description, description.pixels))
    {
        std::fprintf(stderr, "renderer module: could not enqueue block texture upload\n");
        destroyTextureObject(texture);
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

// One fenced command buffer: transition every level to TRANSFER_DST, copy
// each level's blocks, then land the whole image in SHADER_READ_ONLY.
bool VulkanResources::uploadImageBlocks(VkImage image, const BlockTextureDescription &description,
    const std::uint8_t *blocks)
{
    std::uint64_t totalBytes = 0;
    for (std::uint32_t level = 0; level < description.levelCount; ++level)
    {
        if (description.levelBytes[level] > std::numeric_limits<std::uint64_t>::max() - totalBytes)
        {
            return false;
        }
        totalBytes += description.levelBytes[level];
    }
    const VkDeviceSize byteCount = static_cast<VkDeviceSize>(totalBytes);
    std::lock_guard queueLock(device_->queueMutex());

    UploadStaging staging;
    void         *mapped      = nullptr;
    VkDeviceSize  stagingBase = 0;
    if (device_->acquireStagingSpan(byteCount, &staging))
    {
        mapped      = static_cast<std::uint8_t *>(staging.pooledHostBase) + staging.span.offset;
        stagingBase = static_cast<VkDeviceSize>(staging.span.offset);
    }
    else
    {
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
        std::memcpy(mapped, blocks, static_cast<std::size_t>(byteCount));
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
            blocks,
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
        .subresourceRange    = {VK_IMAGE_ASPECT_COLOR_BIT, 0, description.levelCount, 0, 1},
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

    VkDeviceSize levelOffset = 0;
    for (std::uint32_t level = 0; level < description.levelCount; ++level)
    {
        const std::uint32_t levelWidth  = blockLevelExtent(description.width, level);
        const std::uint32_t levelHeight = blockLevelExtent(description.height, level);
        const VkBufferImageCopy region{
            .bufferOffset      = stagingBase + levelOffset,
            .bufferRowLength   = 0,
            .bufferImageHeight = 0,
            .imageSubresource  = {VK_IMAGE_ASPECT_COLOR_BIT, level, 0, 1},
            .imageOffset       = {0, 0, 0},
            .imageExtent       = {levelWidth, levelHeight, 1},
        };
        vkCmdCopyBufferToImage(
            command, staging.stagingBuffer(), image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
        levelOffset += static_cast<VkDeviceSize>(description.levelBytes[level]);
    }

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
        .subresourceRange    = {VK_IMAGE_ASPECT_COLOR_BIT, 0, description.levelCount, 0, 1},
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

    if (vkEndCommandBuffer(command) != VK_SUCCESS || !device_->enqueueUploadLocked(command, staging))
    {
        vkFreeCommandBuffers(device_->device(), device_->uploadPool(), 1, &command);
        device_->discardUploadStaging(staging);
        return false;
    }
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

// Writes the six combined image-sampler slots; missing handles fall back to
// the neutral textures (white for color-ish slots, flat normal for slots 1
// and 5).
bool VulkanResources::writeMaterialSet(VkDescriptorSet set, const MaterialKey &key)
{
    const auto textureOr = [this](TextureHandle handle, const VulkanTexture &fallback)
    {
        const auto found = textures_.find(handle);
        return found != textures_.end() ? &found->second : &fallback;
    };
    const VulkanTexture *slotTextures[6] = {
        textureOr(key[0], whiteTexture_),
        textureOr(key[1], flatNormalTexture_),
        textureOr(key[2], whiteTexture_),
        textureOr(key[3], whiteTexture_),
        textureOr(key[4], whiteTexture_),
        textureOr(key[5], flatNormalTexture_),
    };

    VkDescriptorImageInfo imageInfos[6];
    VkWriteDescriptorSet  writes[6];
    for (std::uint32_t slot = 0; slot < 6; ++slot)
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
    vkUpdateDescriptorSets(device_->device(), 6, writes, 0, nullptr);
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
        material.emissiveTexture,
        material.detailNormalTexture};
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

// ---- material extension blocks (set 2) ------------------------------------

void VulkanResources::resetMaterialExtensions(std::uint32_t frameIndex)
{
    if (frameIndex >= materialExtensionTuples_.size())
    {
        return;
    }
    materialExtensionCursors_[frameIndex] = 0;
    for (ExtensionTuple &tuple : materialExtensionTuples_[frameIndex])
    {
        tuple.used = false;
    }
}

std::uint32_t VulkanResources::acquireMaterialExtension(std::uint32_t frameIndex, const MeshMaterial &material)
{
    if (frameIndex >= materialExtensionTuples_.size() || materialExtensionMappings_.empty())
    {
        return 0;
    }
    // The material's extension payload (mesh.frag's MaterialExtension, the
    // RasterSetup carry's twin: fog color/asymptote, fog scale + mottle pair,
    // detail overlay triple).
    MaterialExtensionBlock block{};
    block.fogColor[0] = material.fogColor[0];
    block.fogColor[1] = material.fogColor[1];
    block.fogColor[2] = material.fogColor[2];
    block.fogColor[3] = std::clamp(material.fogAsymptote, 0.0f, 1.0f);
    block.fogScaleNoiseScaleNoiseStrength[0] = std::max(material.fogScaleMeters, 1.0f);
    block.fogScaleNoiseScaleNoiseStrength[1] = std::max(material.noiseScaleMeters, 0.0f);
    block.fogScaleNoiseScaleNoiseStrength[2] = std::max(material.noiseStrength, 0.0f);
    block.fogScaleNoiseScaleNoiseStrength[3] = 0.0f;
    const float detailScale = material.detailNormalTexture != invalidTexture ? std::max(material.detailUvScale, 0.0f)
                                                                          : 0.0f;
    block.detailUvScaleStrengthFadeStartFadeEnd[0] = detailScale;
    block.detailUvScaleStrengthFadeStartFadeEnd[1] = std::clamp(material.detailStrength, 0.0f, 4.0f);
    block.detailUvScaleStrengthFadeStartFadeEnd[2] = std::max(material.detailFadeStartMeters, 0.0f);
    block.detailUvScaleStrengthFadeStartFadeEnd[3] = std::max(material.detailFadeEndMeters,
        std::max(material.detailFadeStartMeters, 0.0f) + 0.1f);

    // Extension-less materials (the common case outside the planet demo)
    // share the all-off block zero.
    const bool carriesExtension = block.fogColor[3] > 0.0f
        || block.fogScaleNoiseScaleNoiseStrength[1] > 0.0f || detailScale > 0.0f;
    if (!carriesExtension)
    {
        return 0;
    }

    // Dedupe within the frame: identical tuples share one block (a streamed
    // scene's hundreds of chunks carry a handful of distinct fog/mottle
    // values).
    std::vector<ExtensionTuple> &tuples = materialExtensionTuples_[frameIndex];
    for (std::uint32_t index = 0; index < materialExtensionCursors_[frameIndex]; ++index)
    {
        if (tuples[index].used && std::memcmp(&tuples[index].block, &block, sizeof(block)) == 0)
        {
            return tuples[index].offset;
        }
    }
    const std::uint32_t cursor = materialExtensionCursors_[frameIndex];
    if (cursor >= kMaterialExtensionCapacity)
    {
        // Pathologically many distinct tuples in one frame: the extras fall
        // back to the all-off block (visual degradation, not a crash).
        return 0;
    }
    const std::uint32_t offset = cursor * materialExtensionStride_;
    std::memcpy(static_cast<std::uint8_t *>(materialExtensionMappings_[frameIndex]) + offset, &block, sizeof(block));
    tuples[cursor].block  = block;
    tuples[cursor].offset = offset;
    tuples[cursor].used   = true;
    materialExtensionCursors_[frameIndex] = cursor + 1;
    return offset;
}


// ---- reflection_field screen-space field ----------------------------------
// One RGBA32F image per frame in flight, refreshed through a per-slot staging
// buffer. The shader reads the field with texelFetch (manual bilinear with
// the module's roughness-band gates), so the shared sampler is nearest/clamp
// and no float-filtering feature is required.

void VulkanResources::destroyReflectionFieldSlot(ReflectionFieldSlot &slot)
{
    if (device_ == nullptr)
    {
        return;
    }
    const VkDevice vulkanDevice = device_->device();
    if (slot.view != VK_NULL_HANDLE)
    {
        vkDestroyImageView(vulkanDevice, slot.view, nullptr);
        slot.view = VK_NULL_HANDLE;
    }
    if (slot.image != VK_NULL_HANDLE)
    {
        device_->destroyImage(slot.image, slot.imageAllocation);
        slot.image           = VK_NULL_HANDLE;
        slot.imageAllocation = nullptr;
    }
    if (slot.stagingMapping != nullptr && slot.stagingAllocation != nullptr)
    {
        device_->unmapAllocation(slot.stagingAllocation);
        slot.stagingMapping = nullptr;
    }
    if (slot.stagingBuffer != VK_NULL_HANDLE)
    {
        device_->destroyBuffer(slot.stagingBuffer, slot.stagingAllocation);
        slot.stagingBuffer       = VK_NULL_HANDLE;
        slot.stagingAllocation   = nullptr;
    }
    slot.stagedThisFrame = false;
    slot.stagedRevision  = 0;
}

// Rewrites every frame set's binding 2 onto the neutral white texture, so
// the descriptor never references a destroyed view after the field objects
// go away (the shader's static texelFetch makes an invalid view a hazard
// even while the uniform keeps the hook inactive).
void VulkanResources::bindNeutralFieldSampler()
{
    if (device_ == nullptr || whiteTexture_.view == VK_NULL_HANDLE)
    {
        return;
    }
    for (std::uint32_t index = 0; index < frameUniformSets_.size(); ++index)
    {
        const VkDescriptorImageInfo imageInfo{
            whiteTexture_.sampler,
            whiteTexture_.view,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
        };
        const VkWriteDescriptorSet fieldWrite{
            .sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
            .pNext            = nullptr,
            .dstSet           = frameUniformSets_[index],
            .dstBinding       = 2,
            .dstArrayElement  = 0,
            .descriptorCount  = 1,
            .descriptorType   = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            .pImageInfo       = &imageInfo,
            .pBufferInfo      = nullptr,
            .pTexelBufferView = nullptr,
        };
        vkUpdateDescriptorSets(device_->device(), 1, &fieldWrite, 0, nullptr);
    }
}

void VulkanResources::destroyReflectionFieldObjects()
{
    if (device_ == nullptr)
    {
        return;
    }
    for (ReflectionFieldSlot &slot : reflectionFieldSlots_)
    {
        destroyReflectionFieldSlot(slot);
    }
    reflectionFieldSlots_.clear();
    for (ReflectionFieldSlot &slot : retiredFieldSlots_)
    {
        destroyReflectionFieldSlot(slot);
    }
    retiredFieldSlots_.clear();
    fieldRetireCountdown_ = 0;
    if (reflectionFieldSampler_ != VK_NULL_HANDLE)
    {
        vkDestroySampler(device_->device(), reflectionFieldSampler_, nullptr);
        reflectionFieldSampler_ = VK_NULL_HANDLE;
    }
    reflectionFieldVersions_.clear();
    reflectionFieldWidth_  = 0;
    reflectionFieldHeight_ = 0;
    bindNeutralFieldSampler();
}

void VulkanResources::tickFrame()
{
    if (fieldRetireCountdown_ == 0 || --fieldRetireCountdown_ != 0)
    {
        return;
    }
    for (ReflectionFieldSlot &slot : retiredFieldSlots_)
    {
        destroyReflectionFieldSlot(slot);
    }
    retiredFieldSlots_.clear();
}

bool VulkanResources::ensureReflectionFieldImages(std::uint32_t width, std::uint32_t height)
{
    if (device_ == nullptr || width == 0 || height == 0)
    {
        return false;
    }
    if (width == reflectionFieldWidth_ && height == reflectionFieldHeight_ && !reflectionFieldSlots_.empty())
    {
        return true;
    }
    const std::uint32_t slotCount = static_cast<std::uint32_t>(frameUniformSets_.size());
    if (slotCount == 0)
    {
        return false;
    }

    // The previous images may still be sampled by in-flight frames of the
    // other slots: retire them for framesInFlight more beginFrames instead
    // of destroying them now.
    if (!reflectionFieldSlots_.empty())
    {
        for (ReflectionFieldSlot &slot : reflectionFieldSlots_)
        {
            retiredFieldSlots_.push_back(slot);
        }
        reflectionFieldSlots_.clear();
        fieldRetireCountdown_ = slotCount + 1;
    }

    if (reflectionFieldSampler_ == VK_NULL_HANDLE)
    {
        const VkSamplerCreateInfo samplerInfo{
            .sType                   = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO,
            .pNext                   = nullptr,
            .flags                   = 0,
            .magFilter               = VK_FILTER_NEAREST,
            .minFilter               = VK_FILTER_NEAREST,
            .mipmapMode              = VK_SAMPLER_MIPMAP_MODE_NEAREST,
            .addressModeU            = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
            .addressModeV            = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
            .addressModeW            = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
            .mipLodBias              = 0.0f,
            .anisotropyEnable        = VK_FALSE,
            .maxAnisotropy           = 1.0f,
            .compareEnable           = VK_FALSE,
            .compareOp               = VK_COMPARE_OP_ALWAYS,
            .minLod                  = 0.0f,
            .maxLod                  = 0.0f,
            .borderColor             = VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK,
            .unnormalizedCoordinates = VK_FALSE,
        };
        if (vkCreateSampler(device_->device(), &samplerInfo, nullptr, &reflectionFieldSampler_) != VK_SUCCESS)
        {
            std::fprintf(stderr, "renderer module: could not create the reflection field sampler\n");
            return false;
        }
    }

    const VkDeviceSize fieldBytes
        = static_cast<VkDeviceSize>(width) * static_cast<VkDeviceSize>(height) * 4ull * sizeof(float);
    reflectionFieldSlots_.resize(slotCount);
    for (std::uint32_t index = 0; index < slotCount; ++index)
    {
        ReflectionFieldSlot &slot = reflectionFieldSlots_[index];
        if (!device_->createImage(width,
                height,
                1u,
                VK_FORMAT_R32G32B32A32_SFLOAT,
                VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                &slot.image,
                &slot.imageAllocation))
        {
            std::fprintf(stderr, "renderer module: could not create the reflection field image\n");
            destroyReflectionFieldObjects();
            return false;
        }
        const VkImageViewCreateInfo viewInfo{
            .sType           = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
            .pNext           = nullptr,
            .flags           = 0,
            .image           = slot.image,
            .viewType        = VK_IMAGE_VIEW_TYPE_2D,
            .format          = VK_FORMAT_R32G32B32A32_SFLOAT,
            .components      = {VK_COMPONENT_SWIZZLE_IDENTITY,
                VK_COMPONENT_SWIZZLE_IDENTITY,
                VK_COMPONENT_SWIZZLE_IDENTITY,
                VK_COMPONENT_SWIZZLE_IDENTITY},
            .subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
        };
        if (vkCreateImageView(device_->device(), &viewInfo, nullptr, &slot.view) != VK_SUCCESS)
        {
            std::fprintf(stderr, "renderer module: could not create the reflection field image view\n");
            destroyReflectionFieldObjects();
            return false;
        }
        if (!device_->createBuffer(fieldBytes,
                VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                BufferMemoryIntent::HostSequentialWrite,
                &slot.stagingBuffer,
                &slot.stagingAllocation))
        {
            std::fprintf(stderr, "renderer module: could not create the reflection field staging buffer\n");
            destroyReflectionFieldObjects();
            return false;
        }
        if (!device_->mapAllocation(slot.stagingAllocation, &slot.stagingMapping))
        {
            std::fprintf(stderr, "renderer module: could not map the reflection field staging buffer\n");
            destroyReflectionFieldObjects();
            return false;
        }
    }
    reflectionFieldWidth_   = width;
    reflectionFieldHeight_ = height;
    reflectionFieldVersions_.assign(slotCount, 0);

    // Rewrite the binding-2 descriptors onto the new views.
    for (std::uint32_t index = 0; index < slotCount; ++index)
    {
        const VkDescriptorImageInfo imageInfo{
            reflectionFieldSampler_,
            reflectionFieldSlots_[index].view,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
        };
        const VkWriteDescriptorSet fieldWrite{
            .sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
            .pNext            = nullptr,
            .dstSet           = frameUniformSets_[index],
            .dstBinding       = 2,
            .dstArrayElement  = 0,
            .descriptorCount  = 1,
            .descriptorType   = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            .pImageInfo       = &imageInfo,
            .pBufferInfo      = nullptr,
            .pTexelBufferView = nullptr,
        };
        vkUpdateDescriptorSets(device_->device(), 1, &fieldWrite, 0, nullptr);
    }
    return true;
}

bool VulkanResources::stageReflectionField(
    std::uint32_t frameIndex, const float *radiance, std::uint32_t width, std::uint32_t height, std::uint64_t revision)
{
    if (frameIndex >= reflectionFieldSlots_.size() || radiance == nullptr || width != reflectionFieldWidth_
        || height != reflectionFieldHeight_)
    {
        return false;
    }
    ReflectionFieldSlot &slot = reflectionFieldSlots_[frameIndex];
    if (slot.stagingMapping == nullptr)
    {
        return false;
    }
    const std::size_t fieldBytes
        = static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4u * sizeof(float);
    std::memcpy(slot.stagingMapping, radiance, fieldBytes);
    if (!device_->flushAllocation(slot.stagingAllocation, 0, static_cast<VkDeviceSize>(fieldBytes)))
    {
        return false;
    }
    slot.stagedThisFrame = true;
    slot.stagedRevision  = revision;
    return true;
}

void VulkanResources::recordReflectionFieldUpload(std::uint32_t frameIndex, VkCommandBuffer command)
{
    if (frameIndex >= reflectionFieldSlots_.size() || command == VK_NULL_HANDLE)
    {
        return;
    }
    ReflectionFieldSlot &slot = reflectionFieldSlots_[frameIndex];
    if (!slot.stagedThisFrame || slot.image == VK_NULL_HANDLE || slot.stagingBuffer == VK_NULL_HANDLE)
    {
        return;
    }
    // UNDEFINED -> TRANSFER_DST (discard the previous contents), copy the
    // whole field, then TRANSFER_DST -> SHADER_READ with a srcScope wide
    // enough for the fragment reads of this same command buffer.
    const VkImageMemoryBarrier toTransfer{
        .sType               = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
        .pNext               = nullptr,
        .srcAccessMask       = 0,
        .dstAccessMask       = VK_ACCESS_TRANSFER_WRITE_BIT,
        .oldLayout           = VK_IMAGE_LAYOUT_UNDEFINED,
        .newLayout           = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
        .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
        .image               = slot.image,
        .subresourceRange    = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1},
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
        .bufferOffset      = 0,
        .bufferRowLength   = 0,
        .bufferImageHeight = 0,
        .imageSubresource  = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1},
        .imageOffset       = {0, 0, 0},
        .imageExtent       = {reflectionFieldWidth_, reflectionFieldHeight_, 1},
    };
    vkCmdCopyBufferToImage(
        command, slot.stagingBuffer, slot.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
    const VkImageMemoryBarrier toShaderRead{
        .sType               = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
        .pNext               = nullptr,
        .srcAccessMask       = VK_ACCESS_TRANSFER_WRITE_BIT,
        .dstAccessMask       = VK_ACCESS_SHADER_READ_BIT,
        .oldLayout           = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        .newLayout           = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
        .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
        .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
        .image               = slot.image,
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
    slot.stagedThisFrame = false;
    if (frameIndex < reflectionFieldVersions_.size())
    {
        reflectionFieldVersions_[frameIndex] = slot.stagedRevision;
    }
}

} // namespace renderer
} // namespace engine

#endif
