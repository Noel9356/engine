#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include "renderer_state.hpp"

#include "mesh_generation.hpp"
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

void RendererState::uploadToDeviceLocal(
    GeometryBuffer &target, const void *data, vk::DeviceSize byteSize, vk::BufferUsageFlags usages
)
{
    vk::BufferCreateInfo stagingInfo;
    stagingInfo.setSize(byteSize);
    stagingInfo.setUsage(vk::BufferUsageFlagBits::eTransferSrc);
    stagingInfo.setSharingMode(vk::SharingMode::eExclusive);
    vk::raii::Buffer             stagingBuffer(*device, stagingInfo);
    const vk::MemoryRequirements stagingRequirements = stagingBuffer.getMemoryRequirements();
    vk::MemoryAllocateInfo       stagingAllocate;
    stagingAllocate.setAllocationSize(stagingRequirements.size);
    stagingAllocate.setMemoryTypeIndex(findMemoryType(
        **physicalDevice,
        stagingRequirements.memoryTypeBits,
        vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent
    ));
    vk::raii::DeviceMemory stagingMemory(*device, stagingAllocate);
    stagingBuffer.bindMemory(*stagingMemory, 0);
    void *mapped = stagingMemory.mapMemory(0, stagingRequirements.size);
    std::memcpy(mapped, data, static_cast<std::size_t>(byteSize));
    stagingMemory.unmapMemory();

    vk::BufferCreateInfo bufferInfo;
    bufferInfo.setSize(byteSize);
    bufferInfo.setUsage(usages | vk::BufferUsageFlagBits::eTransferDst);
    bufferInfo.setSharingMode(vk::SharingMode::eExclusive);
    target.buffer                             = std::make_unique<vk::raii::Buffer>(*device, bufferInfo);
    const vk::MemoryRequirements requirements = target.buffer->getMemoryRequirements();
    vk::MemoryAllocateInfo       allocateInfo;
    allocateInfo.setAllocationSize(requirements.size);
    allocateInfo.setMemoryTypeIndex(
        findMemoryType(**physicalDevice, requirements.memoryTypeBits, vk::MemoryPropertyFlagBits::eDeviceLocal)
    );
    target.memory = std::make_unique<vk::raii::DeviceMemory>(*device, allocateInfo);
    target.buffer->bindMemory(*target.memory, 0);

    vk::CommandBufferAllocateInfo commandAllocateInfo;
    commandAllocateInfo.setCommandPool(**commandPool);
    commandAllocateInfo.setLevel(vk::CommandBufferLevel::ePrimary);
    commandAllocateInfo.setCommandBufferCount(1);
    vk::raii::CommandBuffer setupBuffer = std::move(vk::raii::CommandBuffers(*device, commandAllocateInfo).front());
    setupBuffer.begin(vk::CommandBufferBeginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit));
    vk::BufferCopy copyRegion(0, 0, byteSize);
    setupBuffer.copyBuffer(*stagingBuffer, *target.buffer, copyRegion);
    setupBuffer.end();
    vk::SubmitInfo submitInfo;
    submitInfo.setCommandBuffers(*setupBuffer);
    graphicsQueue.submit(submitInfo);
    graphicsQueue.waitIdle();
}

void RendererState::copyIntoDeviceLocal(
    GeometryBuffer &target, const void *data, vk::DeviceSize byteSize
)
{
    // Content-only re-upload into an ALREADY allocated device-local buffer:
    // the incremental scene rebuild path. The target's handle stays valid, so
    // every descriptor set that binds it stays valid - no technique ever
    // rebuilds because of a scene edit. Requires TransferDst on the target
    // and a capacity of at least byteSize (both guaranteed by
    // allocateSceneDataBuffers for the primitive/material buffers).
    if (target.buffer == nullptr || target.memory == nullptr)
    {
        throw std::runtime_error(logPrefix + "copyIntoDeviceLocal called before the target buffer was allocated");
    }
    vk::BufferCreateInfo stagingInfo;
    stagingInfo.setSize(byteSize);
    stagingInfo.setUsage(vk::BufferUsageFlagBits::eTransferSrc);
    stagingInfo.setSharingMode(vk::SharingMode::eExclusive);
    vk::raii::Buffer             stagingBuffer(*device, stagingInfo);
    const vk::MemoryRequirements stagingRequirements = stagingBuffer.getMemoryRequirements();
    vk::MemoryAllocateInfo       stagingAllocate;
    stagingAllocate.setAllocationSize(stagingRequirements.size);
    stagingAllocate.setMemoryTypeIndex(findMemoryType(
        **physicalDevice,
        stagingRequirements.memoryTypeBits,
        vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent
    ));
    vk::raii::DeviceMemory stagingMemory(*device, stagingAllocate);
    stagingBuffer.bindMemory(*stagingMemory, 0);
    void *mapped = stagingMemory.mapMemory(0, stagingRequirements.size);
    std::memcpy(mapped, data, static_cast<std::size_t>(byteSize));
    stagingMemory.unmapMemory();

    vk::CommandBufferAllocateInfo commandAllocateInfo;
    commandAllocateInfo.setCommandPool(**commandPool);
    commandAllocateInfo.setLevel(vk::CommandBufferLevel::ePrimary);
    commandAllocateInfo.setCommandBufferCount(1);
    vk::raii::CommandBuffer setupBuffer = std::move(vk::raii::CommandBuffers(*device, commandAllocateInfo).front());
    setupBuffer.begin(vk::CommandBufferBeginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit));
    vk::BufferCopy copyRegion(0, 0, byteSize);
    setupBuffer.copyBuffer(*stagingBuffer, *target.buffer, copyRegion);
    setupBuffer.end();
    vk::SubmitInfo submitInfo;
    submitInfo.setCommandBuffers(*setupBuffer);
    graphicsQueue.submit(submitInfo);
    graphicsQueue.waitIdle();
}

void RendererState::allocateSceneDataBuffers()
{
    // One allocation per buffer, at FULL registry capacity, for the process
    // lifetime: the incremental scene rebuild swaps content, never buffers.
    // The capacity constant is shared with the scene module (its registry cap
    // is the same number) and with the SDF shaders (scene_common.glsl).
    static_assert(
        SceneCapability::maxObjects == maxSceneObjects,
        "the scene registry cap and the renderer's GPU primitive capacity must agree"
    );
    constexpr vk::DeviceSize primitivesCapacity =
        static_cast<vk::DeviceSize>(SceneCapability::maxObjects) * sizeof(GpuScenePrimitive);
    constexpr vk::DeviceSize materialsCapacity =
        static_cast<vk::DeviceSize>(SceneCapability::maxObjects) * sizeof(GpuSceneMaterial);

    // Allocation without content: a degenerate stand-in fills the buffer in
    // uploadSceneData right after, so only this function may leave them
    // uncopyable-empty.
    {
        vk::BufferCreateInfo bufferInfo;
        bufferInfo.setSize(primitivesCapacity);
        bufferInfo.setUsage(vk::BufferUsageFlagBits::eStorageBuffer | vk::BufferUsageFlagBits::eTransferDst);
        bufferInfo.setSharingMode(vk::SharingMode::eExclusive);
        primitivesBuffer.buffer                    = std::make_unique<vk::raii::Buffer>(*device, bufferInfo);
        const vk::MemoryRequirements requirements   = primitivesBuffer.buffer->getMemoryRequirements();
        vk::MemoryAllocateInfo       allocateInfo;
        allocateInfo.setAllocationSize(requirements.size);
        allocateInfo.setMemoryTypeIndex(
            findMemoryType(**physicalDevice, requirements.memoryTypeBits, vk::MemoryPropertyFlagBits::eDeviceLocal)
        );
        primitivesBuffer.memory = std::make_unique<vk::raii::DeviceMemory>(*device, allocateInfo);
        primitivesBuffer.buffer->bindMemory(*primitivesBuffer.memory, 0);
    }
    {
        vk::BufferCreateInfo bufferInfo;
        bufferInfo.setSize(materialsCapacity);
        bufferInfo.setUsage(vk::BufferUsageFlagBits::eStorageBuffer | vk::BufferUsageFlagBits::eTransferDst);
        bufferInfo.setSharingMode(vk::SharingMode::eExclusive);
        materialsBuffer.buffer                     = std::make_unique<vk::raii::Buffer>(*device, bufferInfo);
        const vk::MemoryRequirements requirements   = materialsBuffer.buffer->getMemoryRequirements();
        vk::MemoryAllocateInfo       allocateInfo;
        allocateInfo.setAllocationSize(requirements.size);
        allocateInfo.setMemoryTypeIndex(
            findMemoryType(**physicalDevice, requirements.memoryTypeBits, vk::MemoryPropertyFlagBits::eDeviceLocal)
        );
        materialsBuffer.memory = std::make_unique<vk::raii::DeviceMemory>(*device, allocateInfo);
        materialsBuffer.buffer->bindMemory(*materialsBuffer.memory, 0);
    }
}

void RendererState::uploadSceneData()
{
    // Analytic primitives for the lighting modules' SDF: the exact same
    // objects the geometry pass renders (scene_description.hpp documents the
    // std430 layout the shaders mirror). Always at least one entry: the
    // buffer is handed to the lighting passes even for an empty scene.
    std::vector<GpuScenePrimitive> primitives;
    primitives.reserve(std::max<std::size_t>(activeObjects.size(), 1));
    for (const SceneObject &object : activeObjects)
    {
        GpuScenePrimitive primitive;
        primitive.positionType = {
            object.position[0],
            object.position[1],
            object.position[2],
            static_cast<float>(static_cast<std::uint32_t>(object.type))
        };
        primitive.sizeRotation    = {object.size[0], object.size[1], object.size[2], object.rotationYRadians};
        primitive.albedoRoughness = {object.albedo[0], object.albedo[1], object.albedo[2], object.roughness};
        // Rotation trig baked for the SDF shaders (scene_description.hpp): the
        // tracing loops must not re-evaluate sin/cos per primitive per step.
        primitive.metallicEmissiveStrength = {
            object.metallic,
            object.emissiveStrength,
            std::sin(object.rotationYRadians),
            std::cos(object.rotationYRadians)
        };
        primitives.push_back(primitive);
    }
    if (primitives.empty())
    {
        // Degenerate stand-in with rotation 0: sin 0, cos 1.
        primitives.push_back(
            GpuScenePrimitive{
                .positionType             = {0.0f, 0.0f, 0.0f, 0.0f},
                .sizeRotation             = {0.001f, 0.001f, 0.001f, 0.0f},
                .albedoRoughness          = {0.0f, 0.0f, 0.0f, 1.0f},
                .metallicEmissiveStrength = {0.0f, 0.0f, 0.0f, 1.0f}
            }
        );
    }
    if (primitives.size() > SceneCapability::maxObjects)
    {
        throw std::runtime_error(
            logPrefix + "scene data exceeds the GPU primitive capacity ("
                + std::to_string(SceneCapability::maxObjects) + "): the scene registry must enforce the cap"
        );
    }
    copyIntoDeviceLocal(
        primitivesBuffer, primitives.data(), static_cast<vk::DeviceSize>(primitives.size() * sizeof(GpuScenePrimitive))
    );

    // Material table: one entry per primitive, resolving each object's
    // texture set name into array layer indices (materials_common.glsl
    // consumes this on the tracing side; the geometry pass receives the same
    // numbers through push constants). Texture sets are the static [scene]
    // world's list: runtime-spawned objects reference the registered sets by
    // name or render flat - new GPU texture layers require a real asset
    // upload path, which the incremental rebuild deliberately does not have.
    std::vector<GpuSceneMaterial> materialTable;
    materialTable.reserve(std::max<std::size_t>(activeObjects.size(), 1));
    for (const SceneObject &object : activeObjects)
    {
        GpuSceneMaterial material;
        material.layers        = {-1, -1, 0, 0};
        material.scaleAndFlags = {object.uvScale, 0.0f, 0.0f, 0.0f};
        if (!object.textureSet.empty())
        {
            const auto registered = std::find(scene.textureSets.begin(), scene.textureSets.end(), object.textureSet);
            if (registered != scene.textureSets.end())
            {
                const std::int32_t layer = static_cast<std::int32_t>(registered - scene.textureSets.begin());
                material.layers          = {layer, layer, 0, 0};
            }
        }
        materialTable.push_back(material);
    }
    if (materialTable.empty())
    {
        materialTable.push_back(GpuSceneMaterial{.layers = {-1, -1, 0, 0}, .scaleAndFlags = {1.0f, 0.0f, 0.0f, 0.0f}});
    }
    copyIntoDeviceLocal(
        materialsBuffer, materialTable.data(), static_cast<vk::DeviceSize>(materialTable.size() * sizeof(GpuSceneMaterial))
    );

    // Material diagnostics: which texture set resolved to which array layer,
    // and the loaded data statistics of each set (roughness/metallic means).
    // Without this, a silently failed set registration renders the object
    // with flat defaults and the only symptom is a wrong-looking material.
    // Logged once per upload: the log is the audit trail of every runtime
    // material change, capped by the registry capacity.
    if (runtimeLogger != nullptr)
    {
        for (std::size_t index = 0; index < scene.textureSets.size(); ++index)
        {
            const float roughMean = index < textureSetDataMeans.size() ? textureSetDataMeans[index][0] : -1.0f;
            const float metalMean = index < textureSetDataMeans.size() ? textureSetDataMeans[index][1] : -1.0f;
            runtimeLogger->logMessage(
                LogLevel::Debug,
                logPrefix + "texture set '" + scene.textureSets[index] + "' -> layer " + std::to_string(index)
                    + " (mean roughness " + std::to_string(roughMean) + ", mean metallic " + std::to_string(metalMean)
                    + ")"
            );
        }
        for (std::size_t index = 0; index < activeObjects.size(); ++index)
        {
            const SceneObject &object = activeObjects[index];
            if (!object.textureSet.empty())
            {
                const auto registered = std::find(
                    scene.textureSets.begin(), scene.textureSets.end(), object.textureSet
                );
                runtimeLogger->logMessage(
                    LogLevel::Debug,
                    logPrefix + "object " + std::to_string(index) + " texture '" + object.textureSet + "' -> layer "
                        + (registered != scene.textureSets.end()
                                ? std::to_string(registered - scene.textureSets.begin())
                                : std::string("UNRESOLVED (flat fallback)"))
                );
            }
        }
    }
}

void RendererState::refreshSceneRegistry()
{
    // One frame-at-most re-sync: scene events only set the dirty flag; the
    // work happens here, once per dirty frame, on the serialized frame path
    // (device idle per docs/robustness.md, problem 38 - the staging copies
    // race nothing).
    if (sceneCapability != nullptr)
    {
        const std::vector<SceneObjectDescriptor> snapshot = sceneCapability->objectSnapshot();
        activeObjects.clear();
        activeObjects.reserve(snapshot.size());
        for (const SceneObjectDescriptor &descriptor : snapshot)
        {
            activeObjects.push_back(sceneObjectFromDescriptor(descriptor));
        }
    }
    else
    {
        activeObjects = scene.objects;
    }
    uploadSceneData();
}

SceneObject RendererState::sceneObjectFromDescriptor(
    const SceneObjectDescriptor &object
)
{
    SceneObject converted;
    converted.type             = object.type == scene_object_type::sphere ? SceneObjectType::Sphere
                                                                                          : SceneObjectType::Box;
    converted.position         = object.position;
    converted.size             = object.size;
    converted.rotationYRadians = object.rotationYRadians;
    converted.albedo           = object.albedo;
    converted.roughness        = object.roughness;
    converted.metallic         = object.metallic;
    converted.emissiveStrength = object.emissiveStrength;
    converted.textureSet       = object.textureSet;
    converted.uvScale          = object.uvScale;
    return converted;
}

SceneObjectDescriptor RendererState::sceneObjectToDescriptor(
    const SceneObject &object
)
{
    SceneObjectDescriptor converted;
    converted.type             = static_cast<std::uint32_t>(object.type);
    converted.position         = object.position;
    converted.size             = object.size;
    converted.rotationYRadians = object.rotationYRadians;
    converted.albedo           = object.albedo;
    converted.roughness        = object.roughness;
    converted.metallic         = object.metallic;
    converted.emissiveStrength = object.emissiveStrength;
    converted.textureSet       = object.textureSet;
    converted.uvScale          = object.uvScale;
    return converted;
}

void RendererState::createSceneGeometry()
{
    const MeshData box    = buildBoxMesh();
    const MeshData sphere = buildSphereMesh(sphereMeshSlices, sphereMeshStacks);

    auto uploadMesh = [this](const MeshData &mesh, GeometryBuffer &target)
    {
        const vk::DeviceSize      vertexBytes = mesh.vertices.size() * sizeof(float);
        const vk::DeviceSize      indexBytes  = mesh.indices.size() * sizeof(std::uint32_t);
        std::vector<std::uint8_t> combined(static_cast<std::size_t>(vertexBytes + indexBytes));
        std::memcpy(combined.data(), mesh.vertices.data(), static_cast<std::size_t>(vertexBytes));
        std::memcpy(combined.data() + vertexBytes, mesh.indices.data(), static_cast<std::size_t>(indexBytes));
        uploadToDeviceLocal(
            target,
            combined.data(),
            static_cast<vk::DeviceSize>(combined.size()),
            vk::BufferUsageFlagBits::eVertexBuffer | vk::BufferUsageFlagBits::eIndexBuffer
        );
        target.indexCount       = static_cast<std::uint32_t>(mesh.indices.size());
        target.indexOffsetBytes = static_cast<std::size_t>(vertexBytes);
    };
    uploadMesh(box, boxMesh);
    uploadMesh(sphere, sphereMesh);

    // The static [scene] world is the initial active object list; when the
    // scene capability connects (start phase), the registry takes over and
    // this vector is re-synced from it (refreshSceneRegistry).
    if (activeObjects.empty())
    {
        activeObjects = scene.objects;
    }

    // Primitive and material buffers: allocated ONCE at full registry
    // capacity, content re-uploaded on every scene change (the incremental
    // rebuild contract in scene_capability.hpp).
    allocateSceneDataBuffers();

    // Analytic primitives + material table: uploaded from activeObjects (the
    // static world now, the registry snapshot once the scene capability
    // connected). uploadSceneData is called again on every scene change; only
    // the buffer CONTENT changes, never the buffer handles.
    createTextureArrays();
    uploadSceneData();

    buildEnvironmentBuffer();

    // Camera uniform: host visible and persistently mapped; rebuilt on resize
    // (the aspect ratio feeds the projection).
    vk::BufferCreateInfo cameraInfo;
    cameraInfo.setSize(sizeof(CameraData));
    cameraInfo.setUsage(vk::BufferUsageFlagBits::eUniformBuffer);
    cameraInfo.setSharingMode(vk::SharingMode::eExclusive);
    cameraBufferStorage.buffer                      = std::make_unique<vk::raii::Buffer>(*device, cameraInfo);
    const vk::MemoryRequirements cameraRequirements = cameraBufferStorage.buffer->getMemoryRequirements();
    vk::MemoryAllocateInfo       cameraAllocate;
    cameraAllocate.setAllocationSize(cameraRequirements.size);
    cameraAllocate.setMemoryTypeIndex(findMemoryType(
        **physicalDevice,
        cameraRequirements.memoryTypeBits,
        vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent
    ));
    cameraBufferStorage.memory = std::make_unique<vk::raii::DeviceMemory>(*device, cameraAllocate);
    cameraBufferStorage.buffer->bindMemory(*cameraBufferStorage.memory, 0);
    cameraMapped = cameraBufferStorage.memory->mapMemory(0, sizeof(CameraData));
    updateCameraBuffer();
}

void RendererState::createTextureArrays()
{
    // One albedo+data layer pair per registered texture set. Data packing
    // (materials_common.glsl documents the channels): R roughness, G
    // metallic, B/A tangent-space normal XY. Missing maps fall back to
    // neutral values: roughness 0.5, metallic 0, identity normal (0.5, 0.5).
    loadTextureSets();
    createArrayImages();
    uploadArrayLayersAndGenerateMips();

    vk::SamplerCreateInfo samplerInfo;
    samplerInfo.setMagFilter(vk::Filter::eLinear);
    samplerInfo.setMinFilter(vk::Filter::eLinear);
    samplerInfo.setMipmapMode(vk::SamplerMipmapMode::eLinear);
    samplerInfo.setAddressModeU(vk::SamplerAddressMode::eRepeat);
    samplerInfo.setAddressModeV(vk::SamplerAddressMode::eRepeat);
    samplerInfo.setAddressModeW(vk::SamplerAddressMode::eRepeat);
    samplerInfo.setMinLod(0.0f);
    samplerInfo.setMaxLod(static_cast<float>(textureArrayMipLevels));
    textureSampler = std::make_unique<vk::raii::Sampler>(*device, samplerInfo);
}

void RendererState::loadTextureSets()
{
    const std::size_t setCount = scene.textureSets.size();
    if (setCount > 0 && textureDirectory.empty())
    {
        // The engine ships no content: texture sets come from a mod package,
        // which provides [renderer].texture_directory through its shipped
        // defaults. A scene naming texture sets without a directory is a
        // precise configuration problem, not a silent flat-material scene.
        std::string setList;
        for (const std::string &textureSet : scene.textureSets)
        {
            setList += (setList.empty() ? "" : ", ") + textureSet;
        }
        throw std::runtime_error(
            logPrefix + "the scene references texture set(s) [" + setList
            + "] but [renderer].texture_directory is not configured; the content mod "
              "that ships the textures normally provides it"
        );
    }
    textureSetDataMeans.clear();
    loadedTextureSets.clear();
    loadedTextureSets.resize(setCount);
    loadedArrayExtent   = vk::Extent2D{1, 1};
    anyTextureSetLoaded = false;

    for (std::size_t index = 0; index < setCount; ++index)
    {
        const std::filesystem::path setDirectory = resolvedTextureDirectory / scene.textureSets[index];
        LoadedTextureSet           &loaded       = loadedTextureSets[index];
        loaded.albedo.assign(4, 255); // opaque white until Color.png loads
        loaded.data.assign(neutralDataTexel.begin(), neutralDataTexel.end());

        int                         albedoWidth    = 0;
        int                         albedoHeight   = 0;
        int                         albedoChannels = 0;
        const std::filesystem::path colorPath      = setDirectory / colorMapName;
        if (stbi_uc *color = stbi_load(colorPath.string().c_str(), &albedoWidth, &albedoHeight, &albedoChannels, 4))
        {
            const std::size_t pixelCount = static_cast<std::size_t>(albedoWidth)
                                         * static_cast<std::size_t>(albedoHeight);
            loaded.albedo.resize(pixelCount * 4);
            std::memcpy(loaded.albedo.data(), color, pixelCount * 4);
            stbi_image_free(color);
            if (anyTextureSetLoaded
                && (loadedArrayExtent.width != static_cast<std::uint32_t>(albedoWidth)
                    || loadedArrayExtent.height != static_cast<std::uint32_t>(albedoHeight)))
            {
                throw std::runtime_error(
                    logPrefix + "texture set '" + scene.textureSets[index] + "' is " + std::to_string(albedoWidth) + "x"
                    + std::to_string(albedoHeight) + " but earlier sets are " + std::to_string(loadedArrayExtent.width)
                    + "x" + std::to_string(loadedArrayExtent.height)
                    + "; every texture set in a scene must share one resolution"
                );
            }
            loadedArrayExtent = vk::Extent2D(
                static_cast<std::uint32_t>(albedoWidth), static_cast<std::uint32_t>(albedoHeight)
            );
            anyTextureSetLoaded = true;

            // Loads one single source channel of `fileName` into the packed
            // data texel's `channel` slot. `requestedComponents` selects how
            // many components stb_image decodes (1 = red only, 2 = RG) and
            // `sourceComponent` which of those lands in the slot: the normal
            // map's X and Y live in the RED and GREEN channels of NormalGL.png
            // respectively - decoding with 1 component twice would return the
            // RED channel both times, collapsing every normal to the +X+Y
            // diagonal and skewing the shading of every textured surface.
            const auto loadDataChannel = [&](const char *fileName,
                                             int         channel,
                                             int         requestedComponents,
                                             int         sourceComponent) // NOLINT(readability-identifier-length)
            {
                const std::filesystem::path path     = setDirectory / fileName;
                int                         width    = 0;
                int                         height   = 0;
                int                         channels = 0;
                if (stbi_uc *map = stbi_load(path.string().c_str(), &width, &height, &channels, requestedComponents))
                {
                    if (static_cast<std::uint32_t>(width) != loadedArrayExtent.width
                        || static_cast<std::uint32_t>(height) != loadedArrayExtent.height)
                    {
                        throw std::runtime_error(
                            logPrefix + "texture set '" + scene.textureSets[index] + "' map '" + fileName
                            + "' does not match the set's color map size"
                        );
                    }
                    const std::size_t pixelCount = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
                    if (loaded.data.size() != pixelCount * 4)
                    {
                        loaded.data.resize(pixelCount * 4);
                        // Neutral data texel: roughness 0.5, metallic 0,
                        // identity normal XY (0.5, 0.5). Per-channel strided
                        // fill - a flat std::fill_n would misalign the
                        // interleaved channels.
                        for (std::size_t pixel = 0; pixel < pixelCount; ++pixel)
                        {
                            std::uint8_t *texel = loaded.data.data() + pixel * 4;
                            std::copy(neutralDataTexel.begin(), neutralDataTexel.end(), texel);
                        }
                    }
                    for (std::size_t pixel = 0; pixel < pixelCount; ++pixel)
                    {
                        loaded.data[pixel * 4 + channel] = map[pixel * requestedComponents + sourceComponent];
                    }
                    stbi_image_free(map);
                }
            };
            loadDataChannel(roughnessMapName, dataChannelRoughness, 1, 0);
            loadDataChannel(metalnessMapName, dataChannelMetallic, 1, 0);
            loadDataChannel(normalMapName, dataChannelNormalX, 2, 0); // normal X = red channel
            loadDataChannel(normalMapName, dataChannelNormalY, 2, 1); // normal Y = green channel

            // Startup diagnostics: mean roughness/metallic of the packed
            // data layer (logged with the material table below).
            const std::size_t dataPixels = loaded.data.size() / 4;
            double            roughSum   = 0.0;
            double            metalSum   = 0.0;
            for (std::size_t pixel = 0; pixel < dataPixels; ++pixel)
            {
                roughSum += loaded.data[pixel * 4 + 0];
                metalSum += loaded.data[pixel * 4 + 1];
            }
            textureSetDataMeans.push_back(
                {static_cast<float>(dataPixels > 0 ? roughSum / (dataPixels * 255.0) : 0.0),
                    static_cast<float>(dataPixels > 0 ? metalSum / (dataPixels * 255.0) : 0.0)}
            );
        }
        else
        {
            if (runtimeLogger != nullptr)
            {
                runtimeLogger->logMessage(
                    LogLevel::Warning,
                    logPrefix + "texture set '" + scene.textureSets[index] + "' has no Color.png under '"
                        + setDirectory.string() + "'; objects using it fall back to flat materials"
                );
            }
        }
    }

    textureSetDataMeans.resize(scene.textureSets.size());
}

void RendererState::createArrayImages()
{
    const std::size_t   setCount = scene.textureSets.size();
    const std::uint32_t layers   = static_cast<std::uint32_t>(std::max<std::size_t>(setCount, 1));
    textureArrayMipLevels = anyTextureSetLoaded
                              ? static_cast<std::uint32_t>(
                                    std::floor(std::log2(std::max(loadedArrayExtent.width, loadedArrayExtent.height)))
                                ) + 1
                              : 1;
    arrayLayerCount       = layers;

    // Layout plan: UNDEFINED -> transfer dst (upload) -> transfer src/dst
    // (mip chain) -> SHADER_READ_ONLY_OPTIMAL. The arrays are only ever
    // sampled afterwards, so they rest in the read-only layout for life.
    const auto createArray = [&](FrameImage &target, vk::Format format)
    {
        vk::ImageCreateInfo imageInfo;
        imageInfo.setImageType(vk::ImageType::e2D);
        imageInfo.setFormat(format);
        imageInfo.setExtent(vk::Extent3D(loadedArrayExtent.width, loadedArrayExtent.height, 1));
        imageInfo.setMipLevels(textureArrayMipLevels);
        imageInfo.setArrayLayers(layers);
        imageInfo.setSamples(vk::SampleCountFlagBits::e1);
        imageInfo.setTiling(vk::ImageTiling::eOptimal);
        imageInfo.setUsage(
            vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eTransferSrc
            | vk::ImageUsageFlagBits::eSampled
        );
        imageInfo.setSharingMode(vk::SharingMode::eExclusive);
        imageInfo.setInitialLayout(vk::ImageLayout::eUndefined);
        target.image                              = std::make_unique<vk::raii::Image>(*device, imageInfo);
        const vk::MemoryRequirements requirements = target.image->getMemoryRequirements();
        vk::MemoryAllocateInfo       allocate;
        allocate.setAllocationSize(requirements.size);
        allocate.setMemoryTypeIndex(
            findMemoryType(**physicalDevice, requirements.memoryTypeBits, vk::MemoryPropertyFlagBits::eDeviceLocal)
        );
        target.memory = std::make_unique<vk::raii::DeviceMemory>(*device, allocate);
        target.image->bindMemory(*target.memory, 0);
        vk::ImageViewCreateInfo viewInfo;
        viewInfo.setImage(**target.image);
        viewInfo.setViewType(vk::ImageViewType::e2DArray);
        viewInfo.setFormat(format);
        viewInfo.setSubresourceRange(
            vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, textureArrayMipLevels, 0, layers)
        );
        target.view = std::make_unique<vk::raii::ImageView>(*device, viewInfo);
    };

    createArray(albedoArrayImage, textureAlbedoFormat);
    createArray(dataArrayImage, textureDataFormat);

    if (!anyTextureSetLoaded)
    {
        // No texture sets: the 1x1x1 stand-ins still need a defined layout -
        // every material-consuming shader interface references the arrays,
        // and the validation layer checks the declared layout even when a
        // branch never reaches the sampling instruction.
        vk::CommandBufferAllocateInfo allocateInfo;
        allocateInfo.setCommandPool(**commandPool);
        allocateInfo.setLevel(vk::CommandBufferLevel::ePrimary);
        allocateInfo.setCommandBufferCount(1);
        vk::raii::CommandBuffer setupBuffer = std::move(vk::raii::CommandBuffers(*device, allocateInfo).front());
        setupBuffer.begin(vk::CommandBufferBeginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit));
        std::array<vk::ImageMemoryBarrier2, 2> barriers;
        const vk::Image                        arrayImages[2] = {*albedoArrayImage.image, *dataArrayImage.image};
        for (std::size_t index = 0; index < 2; ++index)
        {
            barriers[index].setSrcStageMask(vk::PipelineStageFlagBits2::eTopOfPipe);
            barriers[index].setSrcAccessMask(vk::AccessFlags2());
            barriers[index].setDstStageMask(vk::PipelineStageFlagBits2::eAllCommands);
            barriers[index].setDstAccessMask(vk::AccessFlagBits2::eShaderRead);
            barriers[index].setOldLayout(vk::ImageLayout::eUndefined);
            barriers[index].setNewLayout(vk::ImageLayout::eShaderReadOnlyOptimal);
            barriers[index].setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
            barriers[index].setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
            barriers[index].setImage(arrayImages[index]);
            barriers[index].setSubresourceRange(vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1));
        }
        vk::DependencyInfo dependency;
        dependency.setImageMemoryBarriers(barriers);
        setupBuffer.pipelineBarrier2(dependency);
        setupBuffer.end();
        vk::SubmitInfo submitInfo;
        submitInfo.setCommandBuffers(*setupBuffer);
        graphicsQueue.submit(submitInfo);
        graphicsQueue.waitIdle();
    }
}

void RendererState::uploadArrayLayersAndGenerateMips()
{
    const std::uint32_t layers = arrayLayerCount;
    if (anyTextureSetLoaded)
    {
        // One staging buffer for both arrays; layer i's bytes start at
        // (2*layerIndex + array) * layerSize (4 bytes per pixel, no padding
        // because the extent is a multiple of 4 bytes per row).
        const std::size_t layerBytes = static_cast<std::size_t>(loadedArrayExtent.width) * loadedArrayExtent.height * 4;
        std::vector<std::uint8_t> staging(loadedTextureSets.size() * 2 * layerBytes);
        for (std::size_t index = 0; index < loadedTextureSets.size(); ++index)
        {
            std::memcpy(staging.data() + (index * 2) * layerBytes, loadedTextureSets[index].albedo.data(), layerBytes);
            std::memcpy(
                staging.data() + (index * 2 + 1) * layerBytes, loadedTextureSets[index].data.data(), layerBytes
            );
        }

        vk::BufferCreateInfo bufferInfo;
        bufferInfo.setSize(static_cast<vk::DeviceSize>(staging.size()));
        bufferInfo.setUsage(vk::BufferUsageFlagBits::eTransferSrc);
        bufferInfo.setSharingMode(vk::SharingMode::eExclusive);
        vk::raii::Buffer             stagingBuffer(*device, bufferInfo);
        const vk::MemoryRequirements stagingRequirements = stagingBuffer.getMemoryRequirements();
        vk::MemoryAllocateInfo       stagingAllocate;
        stagingAllocate.setAllocationSize(stagingRequirements.size);
        stagingAllocate.setMemoryTypeIndex(findMemoryType(
            **physicalDevice,
            stagingRequirements.memoryTypeBits,
            vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent
        ));
        vk::raii::DeviceMemory stagingMemory(*device, stagingAllocate);
        stagingBuffer.bindMemory(*stagingMemory, 0);
        void *mapped = stagingMemory.mapMemory(0, stagingRequirements.size);
        std::memcpy(mapped, staging.data(), staging.size());
        stagingMemory.unmapMemory();

        vk::CommandBufferAllocateInfo allocateInfo;
        allocateInfo.setCommandPool(**commandPool);
        allocateInfo.setLevel(vk::CommandBufferLevel::ePrimary);
        allocateInfo.setCommandBufferCount(1);
        vk::raii::CommandBuffer setupBuffer = std::move(vk::raii::CommandBuffers(*device, allocateInfo).front());
        setupBuffer.begin(vk::CommandBufferBeginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit));

        const auto uploadArray = [&](const FrameImage &target, bool dataArray)
        {
            std::vector<vk::BufferImageCopy> copies(loadedTextureSets.size());
            for (std::size_t index = 0; index < loadedTextureSets.size(); ++index)
            {
                // Staging layout: albedo layer i at (2i)*layerBytes, data
                // layer i at (2i+1)*layerBytes. The copy lands in ARRAY
                // LAYER index - baseArrayLayer is the third subresource
                // member and must be set explicitly (a zero here silently
                // writes every set into layer 0, last one wins).
                copies[index].setBufferOffset(
                    static_cast<vk::DeviceSize>((index * 2 + (dataArray ? 1 : 0)) * layerBytes)
                );
                copies[index].setBufferRowLength(0);
                copies[index].setBufferImageHeight(0);
                copies[index].setImageSubresource(
                    vk::ImageSubresourceLayers(vk::ImageAspectFlagBits::eColor, 0, static_cast<std::uint32_t>(index), 1)
                );
                copies[index].setImageOffset(vk::Offset3D(0, 0, 0));
                copies[index].setImageExtent(vk::Extent3D(loadedArrayExtent.width, loadedArrayExtent.height, 1));
            }
            setupBuffer.copyBufferToImage(stagingBuffer, *target.image, vk::ImageLayout::eTransferDstOptimal, copies);
        };

        // UNDEFINED -> TRANSFER_DST for the copy, then TRANSFER_SRC for the
        // blit chain.
        std::array<vk::ImageMemoryBarrier2, 2> enterBarriers;
        const vk::Image                        arrayImages[2] = {*albedoArrayImage.image, *dataArrayImage.image};
        for (std::size_t index = 0; index < 2; ++index)
        {
            enterBarriers[index].setSrcStageMask(vk::PipelineStageFlagBits2::eTopOfPipe);
            enterBarriers[index].setSrcAccessMask(vk::AccessFlags2());
            enterBarriers[index].setDstStageMask(vk::PipelineStageFlagBits2::eTransfer);
            enterBarriers[index].setDstAccessMask(vk::AccessFlagBits2::eTransferWrite);
            enterBarriers[index].setOldLayout(vk::ImageLayout::eUndefined);
            enterBarriers[index].setNewLayout(vk::ImageLayout::eTransferDstOptimal);
            enterBarriers[index].setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
            enterBarriers[index].setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
            enterBarriers[index].setImage(arrayImages[index]);
            enterBarriers[index].setSubresourceRange(
                vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, layers)
            );
        }
        {
            vk::DependencyInfo dependency;
            dependency.setImageMemoryBarriers(enterBarriers);
            setupBuffer.pipelineBarrier2(dependency);
        }
        uploadArray(albedoArrayImage, false);
        uploadArray(dataArrayImage, true);

        // Mipmap chain per layer: blit level l-1 into level l for every
        // layer, halving the extent (the last level clamps to 1x1). Each
        // source level moves TRANSFER_DST -> TRANSFER_SRC right before it is
        // read; each destination level enters UNDEFINED -> TRANSFER_DST
        // before the first write (explicit per-level transitions: mip levels
        // are independent subresources with independent layouts).
        for (std::uint32_t level = 1; level < textureArrayMipLevels; ++level)
        {
            std::array<vk::ImageMemoryBarrier2, 4> levelBarriers;
            const vk::Image                        arrayImages2[2] = {*albedoArrayImage.image, *dataArrayImage.image};
            for (std::size_t index = 0; index < 2; ++index)
            {
                // Source level: written last iteration (or by the upload for
                // level 0), now read by the blit.
                levelBarriers[index * 2].setSrcStageMask(vk::PipelineStageFlagBits2::eTransfer);
                levelBarriers[index * 2].setSrcAccessMask(vk::AccessFlagBits2::eTransferWrite);
                levelBarriers[index * 2].setDstStageMask(vk::PipelineStageFlagBits2::eTransfer);
                levelBarriers[index * 2].setDstAccessMask(vk::AccessFlagBits2::eTransferRead);
                levelBarriers[index * 2].setOldLayout(vk::ImageLayout::eTransferDstOptimal);
                levelBarriers[index * 2].setNewLayout(vk::ImageLayout::eTransferSrcOptimal);
                levelBarriers[index * 2].setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
                levelBarriers[index * 2].setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
                levelBarriers[index * 2].setImage(arrayImages2[index]);
                levelBarriers[index * 2].setSubresourceRange(
                    vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, level - 1, 1, 0, layers)
                );
                // Destination level: first use.
                levelBarriers[index * 2 + 1].setSrcStageMask(vk::PipelineStageFlagBits2::eTopOfPipe);
                levelBarriers[index * 2 + 1].setSrcAccessMask(vk::AccessFlags2());
                levelBarriers[index * 2 + 1].setDstStageMask(vk::PipelineStageFlagBits2::eTransfer);
                levelBarriers[index * 2 + 1].setDstAccessMask(vk::AccessFlagBits2::eTransferWrite);
                levelBarriers[index * 2 + 1].setOldLayout(vk::ImageLayout::eUndefined);
                levelBarriers[index * 2 + 1].setNewLayout(vk::ImageLayout::eTransferDstOptimal);
                levelBarriers[index * 2 + 1].setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
                levelBarriers[index * 2 + 1].setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
                levelBarriers[index * 2 + 1].setImage(arrayImages2[index]);
                levelBarriers[index * 2 + 1].setSubresourceRange(
                    vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, level, 1, 0, layers)
                );
            }
            {
                vk::DependencyInfo dependency;
                dependency.setImageMemoryBarriers(levelBarriers);
                setupBuffer.pipelineBarrier2(dependency);
            }

            const std::uint32_t         sourceWidth  = std::max(1u, loadedArrayExtent.width >> (level - 1));
            const std::uint32_t         sourceHeight = std::max(1u, loadedArrayExtent.height >> (level - 1));
            const std::uint32_t         targetWidth  = std::max(1u, loadedArrayExtent.width >> level);
            const std::uint32_t         targetHeight = std::max(1u, loadedArrayExtent.height >> level);
            std::vector<vk::ImageBlit2> blits(layers);
            for (std::uint32_t layer = 0; layer < layers; ++layer)
            {
                blits[layer].setSrcSubresource(
                    vk::ImageSubresourceLayers(vk::ImageAspectFlagBits::eColor, level - 1, layer, 1)
                );
                blits[layer].setSrcOffsets(
                    {vk::Offset3D(0, 0, 0),
                        vk::Offset3D(
                            static_cast<std::int32_t>(sourceWidth), static_cast<std::int32_t>(sourceHeight), 1
                        )}
                );
                blits[layer].setDstSubresource(
                    vk::ImageSubresourceLayers(vk::ImageAspectFlagBits::eColor, level, layer, 1)
                );
                blits[layer].setDstOffsets(
                    {vk::Offset3D(0, 0, 0),
                        vk::Offset3D(
                            static_cast<std::int32_t>(targetWidth), static_cast<std::int32_t>(targetHeight), 1
                        )}
                );
            }
            vk::BlitImageInfo2 blitInfo;
            blitInfo.setSrcImage(*albedoArrayImage.image);
            blitInfo.setSrcImageLayout(vk::ImageLayout::eTransferSrcOptimal);
            blitInfo.setDstImage(*albedoArrayImage.image);
            blitInfo.setDstImageLayout(vk::ImageLayout::eTransferDstOptimal);
            blitInfo.setRegionCount(static_cast<std::uint32_t>(blits.size()));
            blitInfo.setPRegions(blits.data());
            blitInfo.setFilter(vk::Filter::eLinear);
            setupBuffer.blitImage2(blitInfo);
            blitInfo.setSrcImage(*dataArrayImage.image);
            blitInfo.setDstImage(*dataArrayImage.image);
            setupBuffer.blitImage2(blitInfo);
        }

        // Everything -> SHADER_READ_ONLY_OPTIMAL for the engine's lifetime.
        // Sources: level 0..N-2 rest in TRANSFER_SRC, level N-1 in
        // TRANSFER_DST (both hold after the last blit); a single-level chain
        // (no mips) only has the TRANSFER_DST copy.
        {
            std::vector<vk::ImageMemoryBarrier2> finalBarriers;
            for (std::size_t index = 0; index < 2; ++index)
            {
                const vk::Image image = arrayImages[index];
                if (textureArrayMipLevels > 1)
                {
                    vk::ImageMemoryBarrier2 sourceLevels;
                    sourceLevels.setSrcStageMask(vk::PipelineStageFlagBits2::eTransfer);
                    sourceLevels.setSrcAccessMask(
                        vk::AccessFlagBits2::eTransferWrite | vk::AccessFlagBits2::eTransferRead
                    );
                    sourceLevels.setDstStageMask(vk::PipelineStageFlagBits2::eAllCommands);
                    sourceLevels.setDstAccessMask(vk::AccessFlagBits2::eShaderRead);
                    sourceLevels.setOldLayout(vk::ImageLayout::eTransferSrcOptimal);
                    sourceLevels.setNewLayout(vk::ImageLayout::eShaderReadOnlyOptimal);
                    sourceLevels.setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
                    sourceLevels.setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
                    sourceLevels.setImage(image);
                    sourceLevels.setSubresourceRange(
                        vk::ImageSubresourceRange(
                            vk::ImageAspectFlagBits::eColor, 0, textureArrayMipLevels - 1, 0, layers
                        )
                    );
                    finalBarriers.push_back(sourceLevels);
                }
                vk::ImageMemoryBarrier2 topLevel;
                topLevel.setSrcStageMask(vk::PipelineStageFlagBits2::eTransfer);
                topLevel.setSrcAccessMask(vk::AccessFlagBits2::eTransferWrite);
                topLevel.setDstStageMask(vk::PipelineStageFlagBits2::eAllCommands);
                topLevel.setDstAccessMask(vk::AccessFlagBits2::eShaderRead);
                topLevel.setOldLayout(vk::ImageLayout::eTransferDstOptimal);
                topLevel.setNewLayout(vk::ImageLayout::eShaderReadOnlyOptimal);
                topLevel.setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
                topLevel.setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
                topLevel.setImage(image);
                topLevel.setSubresourceRange(
                    vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, textureArrayMipLevels - 1, 1, 0, layers)
                );
                finalBarriers.push_back(topLevel);
            }
            vk::DependencyInfo dependency;
            dependency.setImageMemoryBarriers(finalBarriers);
            setupBuffer.pipelineBarrier2(dependency);
        }
        setupBuffer.end();
        vk::SubmitInfo submitInfo;
        submitInfo.setCommandBuffers(*setupBuffer);
        graphicsQueue.submit(submitInfo);
        graphicsQueue.waitIdle();
    }
}

void RendererState::buildEnvironmentBuffer()
{
    // Sky gradient from [scene.sky] (defaults to the flat sky_radiance) plus
    // the sun disk. The disk radiance is the sun's irradiance divided by its
    // solid angle, so the disk integrates back to exactly the configured sun
    // intensity: bright enough for ACES to clamp, without adding energy.
    EnvironmentData environment{};
    environment.skyZenith  = {scene.sky.zenith[0], scene.sky.zenith[1], scene.sky.zenith[2], skyZenithBlendExponent};
    environment.skyHorizon = {scene.sky.horizon[0], scene.sky.horizon[1], scene.sky.horizon[2], skyGroundBlendExponent};
    environment.skyGround  = {scene.sky.ground[0], scene.sky.ground[1], scene.sky.ground[2], 0.0f};

    float                sunIntensity = 0.0f;
    std::array<float, 3> sunColor     = scene.sun.color;
    // The sun description rides on the scene (parsed from [scene.sun]); the
    // environment buffer is rebuilt whenever the scene changes.
    sunIntensity                         = scene.sun.intensity;
    std::array<float, 3> sunDirection    = scene.sun.direction;
    float                directionLength = std::sqrt(
        sunDirection[0] * sunDirection[0] + sunDirection[1] * sunDirection[1] + sunDirection[2] * sunDirection[2]
    );
    if (directionLength < 1e-6f)
    {
        directionLength = 1.0f;
    }
    for (float &component : sunDirection)
    {
        component /= directionLength;
    }

    const bool  diskEnabled  = scene.sky.sunDisk && sunIntensity > 0.0f;
    const float innerDegrees = std::max(scene.sky.sunDiskSizeDegrees * 0.5f, sunDiskMinSizeDegrees);
    const float outerDegrees = innerDegrees + sunDiskSoftLimbDegrees;
    const float cosInner     = std::cos(degreesToRadians(innerDegrees));
    const float cosOuter     = std::cos(degreesToRadians(outerDegrees));
    const float solidAngle   = twoPi * (1.0f - cosInner);
    const float diskRadiance = solidAngle > 1e-8f ? sunIntensity / solidAngle : 0.0f;
    environment.skyGround[3] = diskEnabled ? 1.0f : 0.0f;
    // [scene.sun].direction points TOWARDS the scene (the light's travel
    // direction); the disk sits in the sky at the OPPOSITE direction - the
    // one an observer looks towards to see the sun.
    environment.sunDiskDirection = {-sunDirection[0], -sunDirection[1], -sunDirection[2], cosInner};
    environment.sunDiskRadiance  = {
        sunColor[0] * diskRadiance, sunColor[1] * diskRadiance, sunColor[2] * diskRadiance, cosOuter
    };

    if (environmentBufferStorage.buffer == nullptr)
    {
        vk::BufferCreateInfo environmentInfo;
        environmentInfo.setSize(sizeof(EnvironmentData));
        environmentInfo.setUsage(vk::BufferUsageFlagBits::eUniformBuffer);
        environmentInfo.setSharingMode(vk::SharingMode::eExclusive);
        environmentBufferStorage.buffer = std::make_unique<vk::raii::Buffer>(*device, environmentInfo);
        const vk::MemoryRequirements environmentRequirements = environmentBufferStorage.buffer->getMemoryRequirements();
        vk::MemoryAllocateInfo       environmentAllocate;
        environmentAllocate.setAllocationSize(environmentRequirements.size);
        environmentAllocate.setMemoryTypeIndex(findMemoryType(
            **physicalDevice,
            environmentRequirements.memoryTypeBits,
            vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent
        ));
        environmentBufferStorage.memory = std::make_unique<vk::raii::DeviceMemory>(*device, environmentAllocate);
        environmentBufferStorage.buffer->bindMemory(*environmentBufferStorage.memory, 0);
        environmentMapped = environmentBufferStorage.memory->mapMemory(0, sizeof(EnvironmentData));
    }
    if (environmentMapped != nullptr)
    {
        std::memcpy(environmentMapped, &environment, sizeof(EnvironmentData));
    }
}

void RendererState::updateCameraBuffer()
{
    const float aspect = swapchainExtent.height > 0
                           ? static_cast<float>(swapchainExtent.width) / static_cast<float>(swapchainExtent.height)
                           : 1.0f;
    const Mat4  view   = lookAtMatrix(
        scene.camera.position.data(), scene.camera.lookAt.data(), std::array<float, 3>{0.0f, 1.0f, 0.0f}.data()
    );
    const Mat4 projection = perspectiveMatrix(
        degreesToRadians(scene.camera.verticalFieldOfViewDegrees), aspect, scene.camera.nearPlane, scene.camera.farPlane
    );
    const Mat4 viewProjection        = multiplyMatrix(projection, view);
    cameraData.viewProjection        = viewProjection;
    cameraData.inverseViewProjection = inverseMatrix(viewProjection);
    cameraData.position = {scene.camera.position[0], scene.camera.position[1], scene.camera.position[2], 0.0f};
    if (cameraMapped != nullptr)
    {
        std::memcpy(cameraMapped, &cameraData, sizeof(CameraData));
    }
}

} // namespace engine
