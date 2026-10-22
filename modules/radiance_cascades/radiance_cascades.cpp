#include "radiance_cascades.hpp"

#include "engine/capabilities/lighting_contract.hpp"
#include "light_uniform.hpp"
#include "render_interface.hpp"
#include "renderer.hpp"
#include "scene_description.hpp"
#include "vulkan_utilities.hpp"

#include <vulkan/vulkan.hpp>
#include <vulkan/vulkan_raii.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <vector>

namespace engine
{

namespace
{

constexpr std::string_view moduleContext = "radiance_cascades module";

// Technique settings defaults. The Settings struct's member initializers and
// the [lighting.radiance_cascades] fallbacks state the same numbers through
// these constants: the struct defaults previously drifted from the config
// fallbacks (probeStride 8 vs 4, baseRays 64 vs 32, losSteps 16 vs 24) -
// unreachable but a silent trap for the first direct construction. The
// values here are the ones the configuration layer has always applied.
constexpr std::uint32_t defaultCascades = 5;
constexpr float defaultIntervalBase = 0.5f;
constexpr float defaultIntervalFactor = 3.0f;
constexpr std::uint32_t defaultProbeStride = 4;
constexpr std::uint32_t defaultBaseRays = 32;
constexpr std::uint32_t defaultLosSteps = 24;
constexpr std::uint32_t defaultMarchSteps = 48;
constexpr std::uint32_t defaultShadowSteps = 16;
constexpr std::uint32_t defaultSpatialFilterRadius = 5;

// Push-constant blocks as the .comp shaders see them (file scope so the
// pipeline layouts size their ranges with sizeof; the static_asserts pin the
// shader-side byte counts).
struct MarchPush
{
    std::array<float, 4> interval;
    std::array<std::uint32_t, 4> flags;
    std::array<std::uint32_t, 4> grid;
    std::array<std::uint32_t, 4> extentStride;
};

struct ComposePush
{
    std::array<float, 4> coarsestFlag; // w 1 when this is the coarsest cascade
    std::array<std::uint32_t, 4> fineGrid;
    std::array<std::uint32_t, 4> coarseGrid;
};

struct GatherPush
{
    std::array<std::uint32_t, 4> counts;
    std::array<float, 4> schedule;
    std::array<std::uint32_t, 4> dirs;
};

struct FilterPush
{
    std::array<std::int32_t, 4> params;
    std::array<float, 4> cameraPlanes;
};

static_assert(sizeof(MarchPush) == 64, "rc_march.comp expects a 64-byte push block");
static_assert(sizeof(ComposePush) == 48, "rc_compose.comp expects a 48-byte push block");
static_assert(sizeof(GatherPush) == 48, "rc_gather.comp expects a 48-byte push block");
static_assert(sizeof(FilterPush) == 32, "rc_denoise.comp expects a 32-byte push block");

// The spatial filter's fixed edge-awareness weights (rc_denoise.comp reads
// params.y/z): the normal power and the depth-band percentage of the
// camera depth range.
constexpr std::int32_t filterNormalPower = 8;
constexpr std::int32_t filterDepthBandPercent = 4;

struct CascadeImage
{
    std::unique_ptr<vk::raii::Image> image;
    std::unique_ptr<vk::raii::DeviceMemory> memory;
    std::unique_ptr<vk::raii::ImageView> view;
};

} // namespace

// The radiance cascades pass: march (per cascade interval), compose
// (front-to-back interval chaining, coarsest first), gather (leak-proof
// visibility weighted probe blend). See the .comp files for the technique.
class RadianceCascadesPass final : public LightingTechniquePass
{
public:
    struct Settings
    {
        std::uint32_t cascades = defaultCascades;
        float intervalBase = defaultIntervalBase;
        float intervalFactor = defaultIntervalFactor;
        // Cost note: the march dispatches (extent/stride)^2 * rays * steps
        // scene-SDF evaluations per frame. Stride 2 at 720p meant 29 million
        // march invocations - multi-second frames and a starved message pump
        // ("not responding"); the configured stride (demo: 8+) trades march
        // cost against smoothness, which the LOS-guarded gather and the
        // spatial filter carry.
        std::uint32_t probeStride = defaultProbeStride;
        std::uint32_t baseRays = defaultBaseRays;
        bool gatherVisibility = true;
        std::uint32_t losSteps = defaultLosSteps;
        std::uint32_t marchSteps = defaultMarchSteps;
        bool relightShadows = true;
        std::uint32_t shadowSteps = defaultShadowSteps;
        // Edge-aware spatial filter radius on the gather output (pixels, 0 =
        // pass-through copy). Removes the probe grid's spatial scale; the
        // depth/normal weights keep silhouettes hard.
        std::uint32_t spatialFilterRadius = defaultSpatialFilterRadius;
    };

    RadianceCascadesPass(Settings settings, LightingCapability* lightingCapability)
        : settings(settings), lighting(lightingCapability)
    {
    }

    const char* techniqueName() const override
    {
        return "radiance_cascades";
    }

    LightingTarget targets() const override
    {
        return LightingTarget::IndirectDiffuse;
    }

    void record(vk::raii::CommandBuffer& commandBuffer, const FrameRenderContext& context) override
    {
        if (cascadeCount() == 0 || context.extent != cachedExtent || needsRecreate)
        {
            rebuild(context);
        }

        updateLightUniform();

        const std::uint32_t cascades = cascadeCount();
        // Angular schedule: CONSTANT across cascades. Every cascade carries
        // the same direction count; the spatial grid refines per cascade
        // (probe stride doubles) while the interval partition grows. The
        // former per-cascade halving with a 2x3 floor handed the entire far
        // field six directions, which is where the composed field's angular
        // structure (giant soft blobs) came from.
        const auto azimuthCount = [this](std::uint32_t)
        {
            return azimuthBase;
        };
        const auto elevationCount = [this](std::uint32_t)
        {
            return elevationBase;
        };
        const auto gridWidth = [this, &context](std::uint32_t cascade)
        {
            const std::uint32_t stride = settings.probeStride << cascade;
            return (context.extent.width + stride - 1) / stride;
        };
        const auto gridHeight = [this, &context](std::uint32_t cascade)
        {
            const std::uint32_t stride = settings.probeStride << cascade;
            return (context.extent.height + stride - 1) / stride;
        };
        const auto intervalStart = [this](std::uint32_t cascade)
        {
            return cascade == 0 ? 0.0f : settings.intervalBase * std::pow(settings.intervalFactor,
                                                                          static_cast<float>(cascade - 1));
        };
        const auto intervalEnd = [this](std::uint32_t cascade)
        {
            return settings.intervalBase * std::pow(settings.intervalFactor, static_cast<float>(cascade));
        };

        // ---- March: every cascade traces its own interval. ----------------
        commandBuffer.bindPipeline(vk::PipelineBindPoint::eCompute, **marchPipeline);
        commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, **marchPipelineLayout, 0, **sharedSet, {});
        for (std::uint32_t cascade = 0; cascade < cascades; ++cascade)
        {
            commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, **marchPipelineLayout, 1,
                                             **marchSets[cascade], {});
            const MarchPush pushValue{
                {intervalStart(cascade), intervalEnd(cascade), 0.0f, 0.0f},
                {context.primitiveCount, settings.marchSteps,
                 cascade > 0 ? 1u : 0u, 0u},
                {gridWidth(cascade), gridHeight(cascade), azimuthCount(cascade), elevationCount(cascade)},
                {context.extent.width, context.extent.height, settings.probeStride << cascade,
                 settings.probeStride << cascade}};
            commandBuffer.pushConstants(**marchPipelineLayout, vk::ShaderStageFlagBits::eCompute, 0,
                                        sizeof(pushValue), &pushValue);
            commandBuffer.dispatch((gridWidth(cascade) + 3) / 4, (gridHeight(cascade) + 3) / 4,
                                   azimuthCount(cascade) * elevationCount(cascade));
        }

        // march[c] writes -> compose reads.
        barrierCascades(commandBuffer, marchBarriers(), vk::PipelineStageFlagBits2::eComputeShader,
                        vk::PipelineStageFlagBits2::eComputeShader);

        // ---- Compose: front-to-back interval chaining. --------------------
        commandBuffer.bindPipeline(vk::PipelineBindPoint::eCompute, **composePipeline);
        commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, **composePipelineLayout, 0, **sharedSet, {});
        for (std::uint32_t cascade = cascades; cascade-- > 0;)
        {
            commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, **composePipelineLayout, 1,
                                             **composeSets[cascade], {});
            const ComposePush pushValue{{0.0f, 0.0f, 0.0f, cascade == cascades - 1 ? 1.0f : 0.0f},
                        {gridWidth(cascade), gridHeight(cascade), azimuthCount(cascade), elevationCount(cascade)},
                        {gridWidth(cascade + 1), gridHeight(cascade + 1), azimuthCount(cascade + 1),
                         elevationCount(cascade + 1)}};
            commandBuffer.pushConstants(**composePipelineLayout, vk::ShaderStageFlagBits::eCompute, 0,
                                        sizeof(pushValue), &pushValue);
            commandBuffer.dispatch((gridWidth(cascade) + 3) / 4, (gridHeight(cascade) + 3) / 4,
                                   azimuthCount(cascade) * elevationCount(cascade));

            // full[c] write -> full[c-1] read in the next iteration.
            std::array<vk::ImageMemoryBarrier2, 1> barrier;
            barrier[0].setSrcStageMask(vk::PipelineStageFlagBits2::eComputeShader);
            barrier[0].setSrcAccessMask(vk::AccessFlagBits2::eShaderStorageWrite);
            barrier[0].setDstStageMask(vk::PipelineStageFlagBits2::eComputeShader);
            barrier[0].setDstAccessMask(vk::AccessFlagBits2::eShaderRead);
            barrier[0].setOldLayout(vk::ImageLayout::eGeneral);
            barrier[0].setNewLayout(vk::ImageLayout::eGeneral);
            barrier[0].setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
            barrier[0].setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
            barrier[0].setImage(**fullImages[cascade].image);
            barrier[0].setSubresourceRange(vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1));
            vk::DependencyInfo dependency;
            dependency.setImageMemoryBarriers(barrier);
            commandBuffer.pipelineBarrier2(dependency);
        }

        // ---- Gather: per pixel diffuse irradiance from cascade 0. ---------
        // The gather writes a module-owned temp image; the spatial filter
        // below removes the probe grid's spatial scale and writes the final
        // indirect target.
        commandBuffer.bindPipeline(vk::PipelineBindPoint::eCompute, **gatherPipeline);
        commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, **gatherPipelineLayout, 0, **sharedSet, {});
        commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, **gatherPipelineLayout, 1, **gatherSet, {});
        const GatherPush gatherPush{{context.primitiveCount, settings.losSteps, settings.gatherVisibility ? 1u : 0u, cascades},
                     {settings.intervalBase, settings.intervalFactor, static_cast<float>(settings.probeStride), 0.0f},
                     {azimuthCount(0), elevationCount(0), 0u, 0u}};
        commandBuffer.pushConstants(**gatherPipelineLayout, vk::ShaderStageFlagBits::eCompute, 0,
                                    sizeof(gatherPush), &gatherPush);
        commandBuffer.dispatch((context.extent.width + 7) / 8, (context.extent.height + 7) / 8, 1);

        // ---- Spatial filter: edge-aware cleanup of the gather output. -----
        {
            std::array<vk::ImageMemoryBarrier2, 1> barrier;
            barrier[0].setSrcStageMask(vk::PipelineStageFlagBits2::eComputeShader);
            barrier[0].setSrcAccessMask(vk::AccessFlagBits2::eShaderStorageWrite);
            barrier[0].setDstStageMask(vk::PipelineStageFlagBits2::eComputeShader);
            barrier[0].setDstAccessMask(vk::AccessFlagBits2::eShaderRead);
            barrier[0].setOldLayout(vk::ImageLayout::eGeneral);
            barrier[0].setNewLayout(vk::ImageLayout::eGeneral);
            barrier[0].setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
            barrier[0].setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
            barrier[0].setImage(**gatherTemp.image);
            barrier[0].setSubresourceRange(vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1));
            vk::DependencyInfo dependency;
            dependency.setImageMemoryBarriers(barrier);
            commandBuffer.pipelineBarrier2(dependency);
        }
        commandBuffer.bindPipeline(vk::PipelineBindPoint::eCompute, **denoisePipeline);
        commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, **denoisePipelineLayout, 0, **sharedSet, {});
        commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, **denoisePipelineLayout, 1, **denoiseSet, {});
        const FilterPush filterPush{{static_cast<std::int32_t>(settings.spatialFilterRadius), filterNormalPower,
                                     filterDepthBandPercent, 0},
                                    {context.cameraNearPlane, context.cameraFarPlane, 0.0f, 0.0f}};
        commandBuffer.pushConstants(**denoisePipelineLayout, vk::ShaderStageFlagBits::eCompute, 0,
                                    sizeof(filterPush), &filterPush);
        commandBuffer.dispatch((context.extent.width + 7) / 8, (context.extent.height + 7) / 8, 1);
    }

    void notifyFrameResourcesRecreated() override
    {
        needsRecreate = true;
    }

private:
    std::uint32_t cascadeCount() const
    {
        return settings.cascades;
    }

    void barrierCascades(vk::raii::CommandBuffer& commandBuffer, const std::vector<vk::Image>& images,
                         vk::PipelineStageFlagBits2 sourceStage, vk::PipelineStageFlagBits2 destinationStage)
    {
        std::vector<vk::ImageMemoryBarrier2> barriers;
        barriers.reserve(images.size());
        for (const vk::Image& image : images)
        {
            vk::ImageMemoryBarrier2 barrier;
            barrier.setSrcStageMask(sourceStage);
            barrier.setSrcAccessMask(vk::AccessFlagBits2::eShaderStorageWrite);
            barrier.setDstStageMask(destinationStage);
            barrier.setDstAccessMask(vk::AccessFlagBits2::eShaderRead);
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

    std::vector<vk::Image> marchBarriers() const
    {
        std::vector<vk::Image> images;
        images.reserve(marchImages.size());
        for (const CascadeImage& cascade : marchImages)
        {
            images.push_back(**cascade.image);
        }
        return images;
    }

    void createCascadeImage(CascadeImage& target, const FrameRenderContext& context, vk::Extent2D grid,
                            std::uint32_t layers, vk::ImageViewType viewType = vk::ImageViewType::e2DArray)
    {
        vk::ImageCreateInfo imageInfo;
        imageInfo.setImageType(vk::ImageType::e2D);
        imageInfo.setFormat(vk::Format::eR16G16B16A16Sfloat);
        imageInfo.setExtent(vk::Extent3D(grid.width, grid.height, 1));
        imageInfo.setMipLevels(1);
        imageInfo.setArrayLayers(layers);
        imageInfo.setSamples(vk::SampleCountFlagBits::e1);
        imageInfo.setTiling(vk::ImageTiling::eOptimal);
        imageInfo.setUsage(vk::ImageUsageFlagBits::eSampled | vk::ImageUsageFlagBits::eStorage);
        imageInfo.setSharingMode(vk::SharingMode::eExclusive);
        imageInfo.setInitialLayout(vk::ImageLayout::eUndefined);
        target.image = std::make_unique<vk::raii::Image>(context.device, imageInfo);
        const vk::MemoryRequirements requirements = target.image->getMemoryRequirements();
        vk::MemoryAllocateInfo allocate;
        allocate.setAllocationSize(requirements.size);
        allocate.setMemoryTypeIndex(findMemoryType(context.physicalDevice, requirements.memoryTypeBits,
                                                   vk::MemoryPropertyFlagBits::eDeviceLocal));
        target.memory = std::make_unique<vk::raii::DeviceMemory>(context.device, allocate);
        target.image->bindMemory(*target.memory, 0);
        vk::ImageViewCreateInfo viewInfo;
        viewInfo.setImage(**target.image);
        viewInfo.setViewType(viewType);
        viewInfo.setFormat(vk::Format::eR16G16B16A16Sfloat);
        viewInfo.setSubresourceRange(vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, layers));
        target.view = std::make_unique<vk::raii::ImageView>(context.device, viewInfo);
        // UNDEFINED -> GENERAL through a one-shot submission on the renderer's
        // graphics queue: storage image writes and sampled reads both require
        // a defined layout before the first dispatch that touches the image.
        vk::CommandPoolCreateInfo poolInfo;
        poolInfo.setFlags(vk::CommandPoolCreateFlagBits::eTransient);
        poolInfo.setQueueFamilyIndex(context.graphicsQueueFamily);
        vk::raii::CommandPool setupPool(context.device, poolInfo);
        vk::CommandBufferAllocateInfo allocateInfo;
        allocateInfo.setCommandPool(*setupPool);
        allocateInfo.setLevel(vk::CommandBufferLevel::ePrimary);
        allocateInfo.setCommandBufferCount(1);
        vk::raii::CommandBuffer setupBuffer = std::move(vk::raii::CommandBuffers(context.device, allocateInfo).front());
        setupBuffer.begin(vk::CommandBufferBeginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit));
        vk::ImageMemoryBarrier2 barrier;
        barrier.setSrcStageMask(vk::PipelineStageFlagBits2::eAllCommands);
        barrier.setSrcAccessMask(vk::AccessFlags2());
        barrier.setDstStageMask(vk::PipelineStageFlagBits2::eAllCommands);
        barrier.setDstAccessMask(vk::AccessFlagBits2::eMemoryRead | vk::AccessFlagBits2::eMemoryWrite);
        barrier.setOldLayout(vk::ImageLayout::eUndefined);
        barrier.setNewLayout(vk::ImageLayout::eGeneral);
        barrier.setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
        barrier.setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
        barrier.setImage(**target.image);
        barrier.setSubresourceRange(vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, layers));
        std::array<vk::ImageMemoryBarrier2, 1> barriers{barrier};
        vk::DependencyInfo dependency;
        dependency.setImageMemoryBarriers(barriers);
        setupBuffer.pipelineBarrier2(dependency);
        setupBuffer.end();
        vk::SubmitInfo submitInfo;
        submitInfo.setCommandBuffers(*setupBuffer);
        context.graphicsQueue.submit(submitInfo);
        context.graphicsQueue.waitIdle();
    }

    void updateLightUniform()
    {
        GpuLightUniform uniform;
        const bool hasLighting = lighting != nullptr;
        DirectionalLightDescription sun;
        const bool sunEnabled = hasLighting && lighting->hasSunLight();
        if (sunEnabled)
        {
            sun = lighting->sunLight();
        }
        const std::uint32_t count = hasLighting ? std::min(lighting->pointLightCount(), maxPointLights) : 0;
        std::array<PointLightDescription, maxPointLights> pointLights{};
        for (std::uint32_t index = 0; index < count; ++index)
        {
            pointLights[index] = lighting->pointLight(index);
        }
        fillSunAndPointLights(uniform, sunEnabled, sun, pointLights.data(), count);
        uniform.lightCountsAndFlags = {static_cast<std::int32_t>(count), settings.relightShadows ? 1 : 0,
                                       static_cast<std::int32_t>(settings.shadowSteps), 0};
        std::memcpy(lightStorage.mapped, &uniform, sizeof(uniform));
    }

    void rebuild(const FrameRenderContext& context)
    {
        // Teardown in strict reverse dependency order: descriptor sets, pools,
        // pipelines, pipeline layouts, descriptor set layouts, then images,
        // buffers and samplers. Pipelines embed their layouts and layouts
        // embed the descriptor set layouts, so a layout must never die while
        // a pipeline that references it is alive (docs/robustness.md,
        // problem 25).
        gatherSet.reset();
        gatherPool.reset();
        denoiseSet.reset();
        denoisePool.reset();
        for (auto& set : composeSets)
        {
            set.reset();
        }
        composePool.reset();
        for (auto& set : marchSets)
        {
            set.reset();
        }
        marchPool.reset();
        sharedSet.reset();
        sharedPool.reset();
        gatherPipeline.reset();
        gatherPipelineLayout.reset();
        denoisePipeline.reset();
        denoisePipelineLayout.reset();
        composePipeline.reset();
        composePipelineLayout.reset();
        marchPipeline.reset();
        marchPipelineLayout.reset();
        sharedSetLayout.reset();
        marchSetLayout.reset();
        composeSetLayout.reset();
        gatherSetLayout.reset();
        filterSetLayout.reset();
        for (auto& image : fullImages)
        {
            image = CascadeImage{};
        }
        for (auto& image : marchImages)
        {
            image = CascadeImage{};
        }
        gatherTemp = CascadeImage{};
        skyImage = CascadeImage{};
        lightStorage.reset();
        composeSampler.reset();
        nearestSampler.reset();

        const std::uint32_t cascades = cascadeCount();
        // Direction table floors: 2 azimuth x 3 elevation is the minimum that
        // still covers overhead/underhead transport (an elevation floor of 2
        // missed a ceiling panel at ~70 degrees during bring-up).
        azimuthBase = std::max(2u, static_cast<std::uint32_t>(std::lround(std::sqrt(
                                        static_cast<double>(settings.baseRays) * 2.0))));
        elevationBase = std::max(3u, settings.baseRays / azimuthBase);

        const auto gridWidth = [this, &context](std::uint32_t cascade)
        {
            const std::uint32_t stride = settings.probeStride << cascade;
            return std::max(1u, (context.extent.width + stride - 1) / stride);
        };
        const auto gridHeight = [this, &context](std::uint32_t cascade)
        {
            const std::uint32_t stride = settings.probeStride << cascade;
            return std::max(1u, (context.extent.height + stride - 1) / stride);
        };
        const auto layers = [this](std::uint32_t)
        {
            return azimuthBase * elevationBase;
        };

        // Cascade storage: march (radiance + transmittance) and full (composed
        // radiance) per cascade. Sized to the cascade grid, so memory decays
        // by 4x per cascade (memory rule).
        marchImages.resize(cascades);
        fullImages.resize(cascades);
        for (std::uint32_t cascade = 0; cascade < cascades; ++cascade)
        {
            const vk::Extent2D grid{gridWidth(cascade), gridHeight(cascade)};
            createCascadeImage(marchImages[cascade], context, grid, layers(cascade));
            createCascadeImage(fullImages[cascade], context, grid, layers(cascade));
        }
        createCascadeImage(skyImage, context, vk::Extent2D{1, 1}, 1);

        nearestSampler = makeNearestClampSampler(context.device);

        vk::SamplerCreateInfo linearInfo;
        linearInfo.setMagFilter(vk::Filter::eLinear);
        linearInfo.setMinFilter(vk::Filter::eLinear);
        linearInfo.setMipmapMode(vk::SamplerMipmapMode::eNearest);
        linearInfo.setAddressModeU(vk::SamplerAddressMode::eClampToEdge);
        linearInfo.setAddressModeV(vk::SamplerAddressMode::eClampToEdge);
        linearInfo.setAddressModeW(vk::SamplerAddressMode::eClampToEdge);
        composeSampler = std::make_unique<vk::raii::Sampler>(context.device, linearInfo);

        // Light uniform buffer (own copy: the GI relight configures shadow
        // behaviour from [lighting.radiance_cascades], independent of the
        // direct module).
        lightStorage = createMappedHostVisibleBuffer(context.device, context.physicalDevice, sizeof(GpuLightUniform),
                                                     vk::BufferUsageFlagBits::eUniformBuffer);

        // Layouts -----------------------------------------------------------
        {
            // Shared set 0: camera UBO, primitives SSBO, light UBO (the two
            // relighting passes), material table SSBO, albedo/data texture
            // arrays (textured relight), environment UBO (the compose sky
            // base).
            std::array<vk::DescriptorSetLayoutBinding, 7> bindings;
            bindings[0].setBinding(0).setDescriptorType(vk::DescriptorType::eUniformBuffer).setDescriptorCount(1);
            bindings[0].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[1].setBinding(1).setDescriptorType(vk::DescriptorType::eStorageBuffer).setDescriptorCount(1);
            bindings[1].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[2].setBinding(2).setDescriptorType(vk::DescriptorType::eUniformBuffer).setDescriptorCount(1);
            bindings[2].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[3].setBinding(3).setDescriptorType(vk::DescriptorType::eStorageBuffer).setDescriptorCount(1);
            bindings[3].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[4].setBinding(4).setDescriptorType(vk::DescriptorType::eCombinedImageSampler).setDescriptorCount(1);
            bindings[4].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[5].setBinding(5).setDescriptorType(vk::DescriptorType::eCombinedImageSampler).setDescriptorCount(1);
            bindings[5].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[6].setBinding(6).setDescriptorType(vk::DescriptorType::eUniformBuffer).setDescriptorCount(1);
            bindings[6].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            vk::DescriptorSetLayoutCreateInfo setInfo;
            setInfo.setBindings(bindings);
            sharedSetLayout = std::make_unique<vk::raii::DescriptorSetLayout>(context.device, setInfo);
        }
        {
            std::array<vk::DescriptorSetLayoutBinding, 3> bindings;
            for (std::uint32_t binding = 0; binding < 2; ++binding)
            {
                bindings[binding].setBinding(binding)
                    .setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
                    .setDescriptorCount(1);
                bindings[binding].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            }
            bindings[2].setBinding(2).setDescriptorType(vk::DescriptorType::eStorageImage).setDescriptorCount(1);
            bindings[2].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            vk::DescriptorSetLayoutCreateInfo setInfo;
            setInfo.setBindings(bindings);
            marchSetLayout = std::make_unique<vk::raii::DescriptorSetLayout>(context.device, setInfo);
        }
        {
            std::array<vk::DescriptorSetLayoutBinding, 3> bindings;
            bindings[0].setBinding(0).setDescriptorType(vk::DescriptorType::eCombinedImageSampler).setDescriptorCount(1);
            bindings[0].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[1].setBinding(1).setDescriptorType(vk::DescriptorType::eStorageImage).setDescriptorCount(1);
            bindings[1].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[2].setBinding(2).setDescriptorType(vk::DescriptorType::eStorageImage).setDescriptorCount(1);
            bindings[2].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            vk::DescriptorSetLayoutCreateInfo setInfo;
            setInfo.setBindings(bindings);
            composeSetLayout = std::make_unique<vk::raii::DescriptorSetLayout>(context.device, setInfo);
        }
        {
            // Spatial filter: gathered GI, depth, normal + the output storage
            // image.
            std::array<vk::DescriptorSetLayoutBinding, 4> bindings;
            for (std::uint32_t binding = 0; binding < 3; ++binding)
            {
                bindings[binding].setBinding(binding)
                    .setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
                    .setDescriptorCount(1);
                bindings[binding].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            }
            bindings[3].setBinding(3).setDescriptorType(vk::DescriptorType::eStorageImage)
                .setDescriptorCount(1);
            bindings[3].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            vk::DescriptorSetLayoutCreateInfo setInfo;
            setInfo.setBindings(bindings);
            filterSetLayout = std::make_unique<vk::raii::DescriptorSetLayout>(context.device, setInfo);
        }
        {
            // Gather: one composed-cascade sampler per cascade slot (the
            // shader switches over the per-pixel cascade index), depth,
            // normal, and the gather output storage image.
            std::array<vk::DescriptorSetLayoutBinding, 11> bindings;
            for (std::uint32_t binding = 0; binding < 10; ++binding)
            {
                bindings[binding].setBinding(binding)
                    .setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
                    .setDescriptorCount(1);
                bindings[binding].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            }
            bindings[10].setBinding(10).setDescriptorType(vk::DescriptorType::eStorageImage)
                .setDescriptorCount(1);
            bindings[10].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            vk::DescriptorSetLayoutCreateInfo setInfo;
            setInfo.setBindings(bindings);
            gatherSetLayout = std::make_unique<vk::raii::DescriptorSetLayout>(context.device, setInfo);
        }

        // Pipelines ---------------------------------------------------------
        auto makeComputePipeline = [&context](const std::vector<char>& code,
                                              const vk::raii::DescriptorSetLayout& sharedLayout,
                                              const vk::raii::DescriptorSetLayout& roleLayout, std::uint32_t pushSize)
        {
            vk::PushConstantRange pushRange;
            pushRange.setStageFlags(vk::ShaderStageFlagBits::eCompute);
            pushRange.setOffset(0);
            pushRange.setSize(pushSize);
            vk::PipelineLayoutCreateInfo layoutInfo;
            const std::array<vk::DescriptorSetLayout, 2> layouts{*sharedLayout, *roleLayout};
            layoutInfo.setSetLayouts(layouts);
            layoutInfo.setPushConstantRanges(pushRange);
            auto layout = std::make_unique<vk::raii::PipelineLayout>(context.device, layoutInfo);

            vk::ShaderModuleCreateInfo moduleInfo;
            moduleInfo.setCodeSize(code.size()).setPCode(reinterpret_cast<const std::uint32_t*>(code.data()));
            vk::raii::ShaderModule module(context.device, moduleInfo);
            vk::PipelineShaderStageCreateInfo stage;
            stage.setStage(vk::ShaderStageFlagBits::eCompute).setModule(*module).setPName("main");
            vk::ComputePipelineCreateInfo pipelineInfo;
            pipelineInfo.setStage(stage);
            pipelineInfo.setLayout(**layout);
            auto pipeline = std::make_unique<vk::raii::Pipeline>(context.device, nullptr, pipelineInfo);
            struct Built
            {
                std::unique_ptr<vk::raii::PipelineLayout> layout;
                std::unique_ptr<vk::raii::Pipeline> pipeline;
            };
            return Built{std::move(layout), std::move(pipeline)};
        };

        auto marchBuilt = makeComputePipeline(readShaderBytes(context.shaderDirectory / "rc_march.comp.spv",
                                                              moduleContext),
                                              *sharedSetLayout, *marchSetLayout, sizeof(MarchPush));
        marchPipelineLayout = std::move(marchBuilt.layout);
        marchPipeline = std::move(marchBuilt.pipeline);
        auto composeBuilt = makeComputePipeline(readShaderBytes(context.shaderDirectory / "rc_compose.comp.spv",
                                                                moduleContext),
                                                *sharedSetLayout, *composeSetLayout, sizeof(ComposePush));
        composePipelineLayout = std::move(composeBuilt.layout);
        composePipeline = std::move(composeBuilt.pipeline);
        auto gatherBuilt = makeComputePipeline(readShaderBytes(context.shaderDirectory / "rc_gather.comp.spv",
                                                               moduleContext),
                                               *sharedSetLayout, *gatherSetLayout, sizeof(GatherPush));
        gatherPipelineLayout = std::move(gatherBuilt.layout);
        gatherPipeline = std::move(gatherBuilt.pipeline);
        // The filter pass has its own small layout (3 sampled images + 1
        // storage image) - the gather layout grew per-cascade sampler
        // bindings the filter does not use. Push block: params + camera
        // planes (32 bytes) - the planes linearize buffer depth for the
        // edge-aware depth band.
        auto denoiseBuilt = makeComputePipeline(readShaderBytes(context.shaderDirectory / "rc_denoise.comp.spv",
                                                                moduleContext),
                                                *sharedSetLayout, *filterSetLayout, sizeof(FilterPush));
        denoisePipelineLayout = std::move(denoiseBuilt.layout);
        denoisePipeline = std::move(denoiseBuilt.pipeline);

        // Gather output temp: full screen extent, single layer, same format
        // as the indirect target. The gather and filter shaders declare plain
        // image2D/sampler2D, so the view is 2D (not the cascade 2D array).
        createCascadeImage(gatherTemp, context, vk::Extent2D{context.extent.width, context.extent.height}, 1,
                           vk::ImageViewType::e2D);

        // Descriptor sets ---------------------------------------------------
        {
            std::array<vk::DescriptorPoolSize, 3> poolSizes{
                vk::DescriptorPoolSize(vk::DescriptorType::eUniformBuffer, 3),
                vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 2),
                vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler, 2)};
            vk::DescriptorPoolCreateInfo poolInfo;
            poolInfo.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
            poolInfo.setMaxSets(1);
            poolInfo.setPoolSizes(poolSizes);
            sharedPool = std::make_unique<vk::raii::DescriptorPool>(context.device, poolInfo);
            vk::DescriptorSetAllocateInfo allocate;
            allocate.setDescriptorPool(**sharedPool);
            allocate.setSetLayouts(**sharedSetLayout);
            sharedSet = std::make_unique<vk::raii::DescriptorSet>(
                std::move(vk::raii::DescriptorSets(context.device, allocate).front()));

            vk::DescriptorBufferInfo cameraInfo(context.cameraBuffer, 0, sizeof(CameraData));
            vk::DescriptorBufferInfo primitivesInfo(context.primitivesBuffer, 0, vk::WholeSize);
            vk::DescriptorBufferInfo lightInfo(**lightStorage.buffer, 0, sizeof(GpuLightUniform));
            vk::DescriptorBufferInfo materialsInfo(context.materialsBuffer, 0, vk::WholeSize);
            vk::DescriptorImageInfo albedoArrayInfo(**nearestSampler, context.albedoArrayView,
                                                    vk::ImageLayout::eShaderReadOnlyOptimal);
            vk::DescriptorImageInfo dataArrayInfo(**nearestSampler, context.dataArrayView,
                                                  vk::ImageLayout::eShaderReadOnlyOptimal);
            vk::DescriptorBufferInfo environmentInfo(context.environmentBuffer, 0, vk::WholeSize);
            std::array<vk::WriteDescriptorSet, 7> writes;
            writes[0].setDstSet(**sharedSet).setDstBinding(0).setDescriptorType(vk::DescriptorType::eUniformBuffer);
            writes[0].setBufferInfo(cameraInfo);
            writes[1].setDstSet(**sharedSet).setDstBinding(1).setDescriptorType(vk::DescriptorType::eStorageBuffer);
            writes[1].setBufferInfo(primitivesInfo);
            writes[2].setDstSet(**sharedSet).setDstBinding(2).setDescriptorType(vk::DescriptorType::eUniformBuffer);
            writes[2].setBufferInfo(lightInfo);
            writes[3].setDstSet(**sharedSet).setDstBinding(3).setDescriptorType(vk::DescriptorType::eStorageBuffer);
            writes[3].setBufferInfo(materialsInfo);
            writes[4].setDstSet(**sharedSet).setDstBinding(4).setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
            writes[4].setImageInfo(albedoArrayInfo);
            writes[5].setDstSet(**sharedSet).setDstBinding(5).setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
            writes[5].setImageInfo(dataArrayInfo);
            writes[6].setDstSet(**sharedSet).setDstBinding(6).setDescriptorType(vk::DescriptorType::eUniformBuffer);
            writes[6].setBufferInfo(environmentInfo);
            context.device.updateDescriptorSets(writes, {});
        }

        const vk::ImageView dummyView = *skyImage.view;
        marchSets.resize(cascades);
        {
            std::array<vk::DescriptorPoolSize, 2> poolSizes{
                vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler, cascades * 2),
                vk::DescriptorPoolSize(vk::DescriptorType::eStorageImage, cascades)};
            vk::DescriptorPoolCreateInfo poolInfo;
            poolInfo.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
            poolInfo.setMaxSets(cascades);
            poolInfo.setPoolSizes(poolSizes);
            marchPool = std::make_unique<vk::raii::DescriptorPool>(context.device, poolInfo);
            for (std::uint32_t cascade = 0; cascade < cascades; ++cascade)
            {
                vk::DescriptorSetAllocateInfo allocate;
                allocate.setDescriptorPool(**marchPool);
                allocate.setSetLayouts(**marchSetLayout);
                marchSets[cascade] = std::make_unique<vk::raii::DescriptorSet>(
                    std::move(vk::raii::DescriptorSets(context.device, allocate).front()));
                vk::DescriptorImageInfo depthInfo(**nearestSampler, context.depthView, vk::ImageLayout::eGeneral);
                vk::DescriptorImageInfo normalInfo(**nearestSampler, context.normalView, vk::ImageLayout::eGeneral);
                vk::DescriptorImageInfo outInfo(**nearestSampler, *marchImages[cascade].view,
                                                vk::ImageLayout::eGeneral);
                std::array<vk::WriteDescriptorSet, 3> writes;
                writes[0].setDstSet(**marchSets[cascade]).setDstBinding(0)
                    .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
                writes[0].setImageInfo(depthInfo);
                writes[1].setDstSet(**marchSets[cascade]).setDstBinding(1)
                    .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
                writes[1].setImageInfo(normalInfo);
                writes[2].setDstSet(**marchSets[cascade]).setDstBinding(2)
                    .setDescriptorType(vk::DescriptorType::eStorageImage);
                writes[2].setImageInfo(outInfo);
                context.device.updateDescriptorSets(writes, {});
            }
        }

        composeSets.resize(cascades);
        {
            std::array<vk::DescriptorPoolSize, 2> poolSizes{
                vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler, cascades),
                vk::DescriptorPoolSize(vk::DescriptorType::eStorageImage, cascades * 2)};
            vk::DescriptorPoolCreateInfo poolInfo;
            poolInfo.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
            poolInfo.setMaxSets(cascades);
            poolInfo.setPoolSizes(poolSizes);
            composePool = std::make_unique<vk::raii::DescriptorPool>(context.device, poolInfo);
            for (std::uint32_t cascade = 0; cascade < cascades; ++cascade)
            {
                vk::DescriptorSetAllocateInfo allocate;
                allocate.setDescriptorPool(**composePool);
                allocate.setSetLayouts(**composeSetLayout);
                composeSets[cascade] = std::make_unique<vk::raii::DescriptorSet>(
                    std::move(vk::raii::DescriptorSets(context.device, allocate).front()));
                // The coarsest cascade has no coarser neighbor: bind the sky
                // dummy (the shader branches to the sky constant before it
                // could ever read it).
                const vk::ImageView coarseView =
                    cascade + 1 < cascades ? *fullImages[cascade + 1].view : dummyView;
                vk::DescriptorImageInfo coarseInfo(**composeSampler, coarseView, vk::ImageLayout::eGeneral);
                vk::DescriptorImageInfo outInfo(**nearestSampler, *fullImages[cascade].view, vk::ImageLayout::eGeneral);
                vk::DescriptorImageInfo marchInfo(**nearestSampler, *marchImages[cascade].view,
                                                  vk::ImageLayout::eGeneral);
                std::array<vk::WriteDescriptorSet, 3> writes;
                writes[0].setDstSet(**composeSets[cascade]).setDstBinding(0)
                    .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
                writes[0].setImageInfo(coarseInfo);
                writes[1].setDstSet(**composeSets[cascade]).setDstBinding(1)
                    .setDescriptorType(vk::DescriptorType::eStorageImage);
                writes[1].setImageInfo(outInfo);
                writes[2].setDstSet(**composeSets[cascade]).setDstBinding(2)
                    .setDescriptorType(vk::DescriptorType::eStorageImage);
                writes[2].setImageInfo(marchInfo);
                context.device.updateDescriptorSets(writes, {});
            }
        }

        {
            std::array<vk::DescriptorPoolSize, 2> poolSizes{
                vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler, 10),
                vk::DescriptorPoolSize(vk::DescriptorType::eStorageImage, 1)};
            vk::DescriptorPoolCreateInfo poolInfo;
            poolInfo.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
            poolInfo.setMaxSets(1);
            poolInfo.setPoolSizes(poolSizes);
            gatherPool = std::make_unique<vk::raii::DescriptorPool>(context.device, poolInfo);
            vk::DescriptorSetAllocateInfo allocate;
            allocate.setDescriptorPool(**gatherPool);
            allocate.setSetLayouts(**gatherSetLayout);
            gatherSet = std::make_unique<vk::raii::DescriptorSet>(
                std::move(vk::raii::DescriptorSets(context.device, allocate).front()));
            // Bindings 0..7: the composed cascade of that index; slots beyond
            // the configured cascade count bind the sky stand-in (the gather
            // selection clamps to the configured count and never reads them).
            // The image info array must outlive updateDescriptorSets: the
            // write structs only reference it.
            std::array<vk::WriteDescriptorSet, 11> writes;
            std::array<vk::DescriptorImageInfo, 8> cascadeInfos;
            for (std::uint32_t cascade = 0; cascade < 8; ++cascade)
            {
                const vk::ImageView cascadeView =
                    cascade < cascades ? *fullImages[cascade].view : *skyImage.view;
                cascadeInfos[cascade] = vk::DescriptorImageInfo(**nearestSampler, cascadeView,
                                                                 vk::ImageLayout::eGeneral);
                writes[cascade].setDstSet(**gatherSet).setDstBinding(cascade)
                    .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
                writes[cascade].setImageInfo(cascadeInfos[cascade]);
            }
            vk::DescriptorImageInfo depthInfo(**nearestSampler, context.depthView, vk::ImageLayout::eGeneral);
            vk::DescriptorImageInfo normalInfo(**nearestSampler, context.normalView, vk::ImageLayout::eGeneral);
            // The gather writes the MODULE temp; the filter pass turns it
            // into the renderer-owned indirect target.
            vk::DescriptorImageInfo outInfo(**nearestSampler, *gatherTemp.view, vk::ImageLayout::eGeneral);
            writes[8].setDstSet(**gatherSet).setDstBinding(8)
                .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
            writes[8].setImageInfo(depthInfo);
            writes[9].setDstSet(**gatherSet).setDstBinding(9)
                .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
            writes[9].setImageInfo(normalInfo);
            writes[10].setDstSet(**gatherSet).setDstBinding(10).setDescriptorType(vk::DescriptorType::eStorageImage);
            writes[10].setImageInfo(outInfo);
            context.device.updateDescriptorSets(writes, {});
        }

        {
            std::array<vk::DescriptorPoolSize, 2> poolSizes{
                vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler, 3),
                vk::DescriptorPoolSize(vk::DescriptorType::eStorageImage, 1)};
            vk::DescriptorPoolCreateInfo poolInfo;
            poolInfo.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
            poolInfo.setMaxSets(1);
            poolInfo.setPoolSizes(poolSizes);
            denoisePool = std::make_unique<vk::raii::DescriptorPool>(context.device, poolInfo);
            vk::DescriptorSetAllocateInfo allocate;
            allocate.setDescriptorPool(**denoisePool);
            allocate.setSetLayouts(**filterSetLayout);
            denoiseSet = std::make_unique<vk::raii::DescriptorSet>(
                std::move(vk::raii::DescriptorSets(context.device, allocate).front()));
            vk::DescriptorImageInfo giInfo(**nearestSampler, *gatherTemp.view, vk::ImageLayout::eGeneral);
            vk::DescriptorImageInfo depthInfo(**nearestSampler, context.depthView, vk::ImageLayout::eGeneral);
            vk::DescriptorImageInfo normalInfo(**nearestSampler, context.normalView, vk::ImageLayout::eGeneral);
            vk::DescriptorImageInfo outInfo(**nearestSampler, context.indirectRadianceView,
                                            vk::ImageLayout::eGeneral);
            std::array<vk::WriteDescriptorSet, 4> writes;
            writes[0].setDstSet(**denoiseSet).setDstBinding(0)
                .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
            writes[0].setImageInfo(giInfo);
            writes[1].setDstSet(**denoiseSet).setDstBinding(1)
                .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
            writes[1].setImageInfo(depthInfo);
            writes[2].setDstSet(**denoiseSet).setDstBinding(2)
                .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
            writes[2].setImageInfo(normalInfo);
            writes[3].setDstSet(**denoiseSet).setDstBinding(3).setDescriptorType(vk::DescriptorType::eStorageImage);
            writes[3].setImageInfo(outInfo);
            context.device.updateDescriptorSets(writes, {});
        }

        cachedExtent = context.extent;
        needsRecreate = false;
    }

    Settings settings;
    LightingCapability* lighting = nullptr;

    std::uint32_t azimuthBase = 0;
    std::uint32_t elevationBase = 0;
    vk::Extent2D cachedExtent{};
    bool needsRecreate = false;

    std::vector<CascadeImage> marchImages;
    std::vector<CascadeImage> fullImages;
    CascadeImage gatherTemp; // full extent gather output, filtered into the target
    CascadeImage skyImage; // 1x1x1 stand-in for the coarsest compose binding

    std::unique_ptr<vk::raii::Sampler> nearestSampler;
    std::unique_ptr<vk::raii::Sampler> composeSampler;
    MappedHostVisibleBuffer lightStorage;

    std::unique_ptr<vk::raii::DescriptorSetLayout> sharedSetLayout;
    std::unique_ptr<vk::raii::DescriptorSetLayout> marchSetLayout;
    std::unique_ptr<vk::raii::DescriptorSetLayout> composeSetLayout;
    std::unique_ptr<vk::raii::DescriptorSetLayout> gatherSetLayout;
    std::unique_ptr<vk::raii::DescriptorSetLayout> filterSetLayout;
    std::unique_ptr<vk::raii::DescriptorPool> sharedPool;
    std::unique_ptr<vk::raii::DescriptorSet> sharedSet;
    std::unique_ptr<vk::raii::DescriptorPool> marchPool;
    std::vector<std::unique_ptr<vk::raii::DescriptorSet>> marchSets;
    std::unique_ptr<vk::raii::DescriptorPool> composePool;
    std::vector<std::unique_ptr<vk::raii::DescriptorSet>> composeSets;
    std::unique_ptr<vk::raii::DescriptorPool> gatherPool;
    std::unique_ptr<vk::raii::DescriptorSet> gatherSet;

    std::unique_ptr<vk::raii::PipelineLayout> marchPipelineLayout;
    std::unique_ptr<vk::raii::Pipeline> marchPipeline;
    std::unique_ptr<vk::raii::PipelineLayout> composePipelineLayout;
    std::unique_ptr<vk::raii::Pipeline> composePipeline;
    std::unique_ptr<vk::raii::PipelineLayout> gatherPipelineLayout;
    std::unique_ptr<vk::raii::Pipeline> gatherPipeline;
    std::unique_ptr<vk::raii::PipelineLayout> denoisePipelineLayout;
    std::unique_ptr<vk::raii::Pipeline> denoisePipeline;
    std::unique_ptr<vk::raii::DescriptorPool> denoisePool;
    std::unique_ptr<vk::raii::DescriptorSet> denoiseSet;
};

// ---- Module lifecycle ---------------------------------------------------------------

RadianceCascadesModule::RadianceCascadesModule()
{
    componentManifest.name = "radiance_cascades";
    componentManifest.kind = ComponentKind::Module;
    componentManifest.providedCapabilities = {};
    componentManifest.requiredCapabilities = {"logging", "renderer"};
    // capability.lighting is optional here: without a light provider there
    // are no lights, and GI degrades to emissive + sky transport.
    componentManifest.requiredPermissions = {"capability.logging", "capability.renderer", "capability.lighting"};
    // The pass settings the module reads from [lighting.radiance_cascades],
    // plus the technique composition list it shares with the renderer.
    componentManifest.knownConfigKeys = {
        "lighting.radiance_cascades.cascades",         "lighting.radiance_cascades.interval_base",
        "lighting.radiance_cascades.interval_factor",  "lighting.radiance_cascades.probe_stride",
        "lighting.radiance_cascades.base_rays",        "lighting.radiance_cascades.gather_visibility",
        "lighting.radiance_cascades.los_steps",        "lighting.radiance_cascades.march_steps",
        "lighting.radiance_cascades.relight_shadows",  "lighting.radiance_cascades.shadow_steps",
        "lighting.radiance_cascades.spatial_filter",   "lighting.techniques",
    };
}

RadianceCascadesModule::~RadianceCascadesModule()
{
    stop();
}

void RadianceCascadesModule::construct(ComponentContext& context)
{
    logger = &context.acquireCapability<LoggerCapability>();

    RadianceCascadesPass::Settings settings;
    settings.cascades = static_cast<std::uint32_t>(std::clamp<std::int64_t>(
        context.configurationIntegerInSection("lighting", "radiance_cascades.cascades").value_or(defaultCascades),
        1, 8));
    settings.intervalBase = static_cast<float>(
        context.configurationFloatingInSection("lighting", "radiance_cascades.interval_base")
            .value_or(defaultIntervalBase));
    settings.intervalFactor = static_cast<float>(
        context.configurationFloatingInSection("lighting", "radiance_cascades.interval_factor")
            .value_or(defaultIntervalFactor));
    settings.probeStride = static_cast<std::uint32_t>(std::clamp<std::int64_t>(
        context.configurationIntegerInSection("lighting", "radiance_cascades.probe_stride")
            .value_or(defaultProbeStride),
        1, 64));
    settings.baseRays = static_cast<std::uint32_t>(std::clamp<std::int64_t>(
        context.configurationIntegerInSection("lighting", "radiance_cascades.base_rays").value_or(defaultBaseRays),
        4, 256));
    settings.gatherVisibility =
        context.configurationBooleanInSection("lighting", "radiance_cascades.gather_visibility").value_or(true);
    settings.losSteps = static_cast<std::uint32_t>(std::clamp<std::int64_t>(
        context.configurationIntegerInSection("lighting", "radiance_cascades.los_steps").value_or(defaultLosSteps),
        4, 256));
    settings.marchSteps = static_cast<std::uint32_t>(std::clamp<std::int64_t>(
        context.configurationIntegerInSection("lighting", "radiance_cascades.march_steps").value_or(defaultMarchSteps),
        4, 512));
    settings.relightShadows =
        context.configurationBooleanInSection("lighting", "radiance_cascades.relight_shadows").value_or(true);
    settings.shadowSteps = static_cast<std::uint32_t>(std::clamp<std::int64_t>(
        context.configurationIntegerInSection("lighting", "radiance_cascades.shadow_steps")
            .value_or(defaultShadowSteps),
        4, 256));
    // Filter radius clamp follows the shader's (rc_denoise.comp clamps 0..16):
    // the shipped demo asks for 10 (one probe cell spans 8 pixels at stride 8,
    // so the window must average ACROSS cells), and an 8-cap here silently
    // applied 8 for 10 - the same stale dense-probe-era limit the shader side
    // already shed.
    settings.spatialFilterRadius = static_cast<std::uint32_t>(std::clamp<std::int64_t>(
        context.configurationIntegerInSection("lighting", "radiance_cascades.spatial_filter")
            .value_or(defaultSpatialFilterRadius),
        0, 16));
    if (settings.intervalBase <= 0.0f || settings.intervalFactor < 1.0f)
    {
        throw std::runtime_error("radiance_cascades module: interval_base must be > 0 and interval_factor >= 1");
    }

    // The lighting capability is optional (see the manifest comment): a
    // denied or missing provider only disables the first-bounce relight.
    LightingCapability* lighting = nullptr;
    try
    {
        lighting = &context.acquireCapability<LightingCapability>();
    }
    catch (const ComponentAccessError&)
    {
        lighting = nullptr;
    }

    // Selection: [lighting] techniques = ["radiance_cascades", ...]. A
    // technique module that is loaded but not listed stays idle; the renderer
    // composes only the listed ones.
    const std::vector<std::string> techniques = context.configurationStringArrayInSection("lighting", "techniques");
    passAttached = std::find(techniques.begin(), techniques.end(), "radiance_cascades") != techniques.end();
    lightingAvailable = lighting != nullptr;

    pass = std::make_shared<RadianceCascadesPass>(settings, lighting);
    if (passAttached)
    {
        RendererCapability& rendererCapability = context.acquireCapability<RendererCapability>();
        auto* rendererModule = static_cast<RendererModule*>(&rendererCapability);
        rendererModule->attachLightingTechnique(pass);
    }
}

void RadianceCascadesModule::start()
{
    logger->logMessage(LogLevel::Information,
                       std::string("radiance_cascades module started; technique ") +
                           (passAttached ? "attached (indirect radiance)"
                                         : "not listed in [lighting].techniques; idle") +
                           ", light capability " + (lightingAvailable ? "present" : "absent (no first-bounce relight)"));
}

void RadianceCascadesModule::stop()
{
    passAttached = false;
    pass.reset();
    logger = nullptr;
}

void* RadianceCascadesModule::capabilityPointerFor(const std::string& /*capabilityName*/)
{
    return nullptr;
}

} // namespace engine
