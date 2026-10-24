#ifndef ENGINE_MODULES_RENDERER_VULKAN_UTILITIES_HPP
#define ENGINE_MODULES_RENDERER_VULKAN_UTILITIES_HPP

// Vulkan helpers shared by the renderer module and the lighting technique
// modules. Header-only and in-tree on purpose: this is backend plumbing, not
// a mod-facing contract, and keeping it in one place is what retires the
// per-module copies (readShaderBytes and findMemoryType previously existed
// verbatim in every technique module).

#include <vulkan/vulkan.hpp>
#include <vulkan/vulkan_raii.hpp>

#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace engine
{

// Reads one compiled SPIR-V file. errorContext names the emitting module in
// the diagnostic ("pbr module: shader file '...' could not be opened"). The
// bytes must be a whole number of SPIR-V words: vkCreateShaderModule requires
// codeSize to be a multiple of 4, and a truncated file would otherwise fail
// far from its cause.
inline std::vector<char> readShaderBytes(
    const std::filesystem::path &path, std::string_view errorContext
)
{
    std::ifstream file(path, std::ios::ate | std::ios::binary);
    if (!file.is_open())
    {
        throw std::runtime_error(
            std::string(errorContext) + ": shader file '" + path.string() + "' could not be opened"
        );
    }
    const std::streamsize size = file.tellg();
    file.seekg(0);
    std::vector<char> bytes(static_cast<std::size_t>(size));
    file.read(bytes.data(), size);
    if (bytes.empty() || bytes.size() % sizeof(std::uint32_t) != 0)
    {
        throw std::runtime_error(
            std::string(errorContext) + ": shader file '" + path.string()
            + "' does not contain a whole number of SPIR-V words"
        );
    }
    return bytes;
}

// First memory type matching both the requirement bits and the allowed type
// bitmask. Throws when the physical device offers none (never expected for
// the property combinations the engine asks for).
inline std::uint32_t findMemoryType(
    const vk::PhysicalDevice &physicalDevice, std::uint32_t typeFilter, vk::MemoryPropertyFlags properties
)
{
    const vk::PhysicalDeviceMemoryProperties memoryProperties = physicalDevice.getMemoryProperties();
    for (std::uint32_t typeIndex = 0; typeIndex < memoryProperties.memoryTypeCount; ++typeIndex)
    {
        const bool typeMatches     = (typeFilter & (1u << typeIndex)) != 0;
        const bool propertiesMatch = (memoryProperties.memoryTypes[typeIndex].propertyFlags & properties) == properties;
        if (typeMatches && propertiesMatch)
        {
            return typeIndex;
        }
    }
    throw std::runtime_error("no suitable memory type found");
}

// The G-buffer sampling sampler shared by the technique passes: nearest
// filtering, clamp to edge (texel-exact G-buffer reads, no wraparound at the
// screen border).
inline std::unique_ptr<vk::raii::Sampler> makeNearestClampSampler(
    vk::raii::Device &device
)
{
    vk::SamplerCreateInfo samplerInfo;
    samplerInfo.setMagFilter(vk::Filter::eNearest);
    samplerInfo.setMinFilter(vk::Filter::eNearest);
    samplerInfo.setMipmapMode(vk::SamplerMipmapMode::eNearest);
    samplerInfo.setAddressModeU(vk::SamplerAddressMode::eClampToEdge);
    samplerInfo.setAddressModeV(vk::SamplerAddressMode::eClampToEdge);
    samplerInfo.setAddressModeW(vk::SamplerAddressMode::eClampToEdge);
    return std::make_unique<vk::raii::Sampler>(device, samplerInfo);
}

// A device buffer in host visible memory, persistently mapped: the shape of
// every per-frame uniform the techniques and the renderer upload (one memcpy
// per frame, no staging round trip). Destroy through reset(): the mapping
// dies with the memory object.
struct MappedHostVisibleBuffer
{
    std::unique_ptr<vk::raii::Buffer>       buffer;
    std::unique_ptr<vk::raii::DeviceMemory> memory;
    void                                   *mapped = nullptr;

    void reset()
    {
        buffer.reset();
        memory.reset();
        mapped = nullptr;
    }
};

inline MappedHostVisibleBuffer createMappedHostVisibleBuffer(
    vk::raii::Device         &device,
    const vk::PhysicalDevice &physicalDevice,
    vk::DeviceSize            byteSize,
    vk::BufferUsageFlags      usage
)
{
    MappedHostVisibleBuffer result;
    vk::BufferCreateInfo    bufferInfo;
    bufferInfo.setSize(byteSize);
    bufferInfo.setUsage(usage);
    bufferInfo.setSharingMode(vk::SharingMode::eExclusive);
    result.buffer                             = std::make_unique<vk::raii::Buffer>(device, bufferInfo);
    const vk::MemoryRequirements requirements = result.buffer->getMemoryRequirements();
    vk::MemoryAllocateInfo       allocateInfo;
    allocateInfo.setAllocationSize(requirements.size);
    allocateInfo.setMemoryTypeIndex(findMemoryType(
        physicalDevice,
        requirements.memoryTypeBits,
        vk::MemoryPropertyFlagBits::eHostVisible | vk::MemoryPropertyFlagBits::eHostCoherent
    ));
    result.memory = std::make_unique<vk::raii::DeviceMemory>(device, allocateInfo);
    result.buffer->bindMemory(*result.memory, 0);
    result.mapped = result.memory->mapMemory(0, byteSize);
    return result;
}

} // namespace engine

#endif // ENGINE_MODULES_RENDERER_VULKAN_UTILITIES_HPP
