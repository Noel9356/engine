#include "ray_reflections.hpp"

#include "engine/capabilities/lighting_contract.hpp"
#include "light_uniform.hpp"
#include "render_interface.hpp"
#include "renderer.hpp"
#include "vulkan_utilities.hpp"

#include <vulkan/vulkan.hpp>
#include <vulkan/vulkan_raii.hpp>

#include <algorithm>
#include <array>
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

constexpr std::string_view moduleContext = "ray_reflections module";

// Push-constant block mirrored by ray_reflections.comp (settings + counts).
struct ReflectionPush
{
    std::array<float, 4>         settings;
    std::array<std::uint32_t, 4> counts;
};

static_assert(sizeof(ReflectionPush) == 32, "ray_reflections.comp expects a 32-byte push block");

// The relight's fixed shadow policy for reflection hits (flags in
// GpuLightUniform.lightCountsAndFlags): shadows on, the same step budget the
// direct pass defaults to.
constexpr std::int32_t relightShadowsEnabled = 1;
constexpr std::int32_t relightShadowSteps    = 16;

// The pass object handed to the renderer. Owns its Vulkan objects; creates
// them lazily on the first record and rebuilds them whenever the frame
// resources were recreated. Same lifetime discipline as the other technique
// passes: layouts are declared members, teardown runs in strict reverse
// dependency order (docs/robustness.md, problem 25).
class RayReflectionsPass final : public LightingTechniquePass
{
public:
    struct Settings
    {
        // Surfaces rougher than this fade out to zero reflection: one ray per
        // pixel cannot represent a wide glossy lobe without accumulation.
        float         maxRoughness = defaultReflectionMaxRoughness;
        float         maxDistance  = defaultReflectionMaxDistance;
        std::uint32_t marchSteps   = defaultReflectionMarchSteps;
    };

    explicit RayReflectionsPass(
        Settings settings, LightingCapability *lightingCapability
    )
        : settings(settings), lighting(lightingCapability)
    {
    }

    const char *techniqueName() const override
    {
        return "ray_reflections";
    }

    LightingTarget targets() const override
    {
        return LightingTarget::IndirectSpecular;
    }

    void record(
        vk::raii::CommandBuffer &commandBuffer, const FrameRenderContext &context
    ) override
    {
        if (pipeline == nullptr || context.extent != cachedExtent || needsRecreate)
        {
            rebuild(context);
        }

        updateLightUniform();

        commandBuffer.bindPipeline(vk::PipelineBindPoint::eCompute, **pipeline);
        commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, **pipelineLayout, 0, **sharedSet, {});
        commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, **pipelineLayout, 1, **frameSet, {});

        const ReflectionPush pushValue{
            {settings.maxRoughness, settings.maxDistance, 0.0f, 0.0f},
            {context.primitiveCount, settings.marchSteps, 0u, 0u}
        };
        commandBuffer.pushConstants(
            **pipelineLayout, vk::ShaderStageFlagBits::eCompute, 0, sizeof(pushValue), &pushValue
        );

        const std::uint32_t groupsX = (context.extent.width + 7) / 8;
        const std::uint32_t groupsY = (context.extent.height + 7) / 8;
        commandBuffer.dispatch(groupsX, groupsY, 1);
    }

    void notifyFrameResourcesRecreated() override
    {
        needsRecreate = true;
    }

private:
    // Light uniform for the SDF relight (own copy: the module fills it from
    // the optional lighting capability every frame, exactly like the
    // radiance cascades relight does).
    void updateLightUniform()
    {
        GpuLightUniform             uniform;
        const bool                  hasLighting = lighting != nullptr;
        DirectionalLightDescription sun;
        const bool                  sunEnabled = hasLighting && lighting->hasSunLight();
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
        uniform.lightCountsAndFlags = {static_cast<std::int32_t>(count), relightShadowsEnabled, relightShadowSteps, 0};
        std::memcpy(lightStorage.mapped, &uniform, sizeof(uniform));
    }

    void rebuild(
        const FrameRenderContext &context
    )
    {
        // Teardown in strict reverse dependency order.
        frameSet.reset();
        framePool.reset();
        sharedSet.reset();
        sharedPool.reset();
        pipeline.reset();
        pipelineLayout.reset();
        frameSetLayout.reset();
        sharedSetLayout.reset();
        lightStorage.reset();
        sampler.reset();

        sampler = makeNearestClampSampler(context.device);

        // Light uniform buffer: host visible, persistently mapped, updated
        // every record call.
        lightStorage = createMappedHostVisibleBuffer(
            context.device, context.physicalDevice, sizeof(GpuLightUniform), vk::BufferUsageFlagBits::eUniformBuffer
        );
        // Set 0: camera, primitives, light uniforms (the relight), material
        // table, texture arrays, environment (sky fallback).
        {
            std::array<vk::DescriptorSetLayoutBinding, 7> bindings;
            bindings[0].setBinding(0).setDescriptorType(vk::DescriptorType::eUniformBuffer).setDescriptorCount(1);
            bindings[0].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[1].setBinding(1).setDescriptorType(vk::DescriptorType::eStorageBuffer).setDescriptorCount(1);
            bindings[1].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[2].setBinding(2).setDescriptorType(vk::DescriptorType::eUniformBuffer).setDescriptorCount(1);
            bindings[2].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[3].setBinding(3).setDescriptorType(vk::DescriptorType::eStorageBuffer).setDescriptorCount(1);
            bindings[3].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[4]
                .setBinding(4)
                .setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
                .setDescriptorCount(1);
            bindings[4].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[5]
                .setBinding(5)
                .setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
                .setDescriptorCount(1);
            bindings[5].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[6].setBinding(6).setDescriptorType(vk::DescriptorType::eUniformBuffer).setDescriptorCount(1);
            bindings[6].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            vk::DescriptorSetLayoutCreateInfo setInfo;
            setInfo.setBindings(bindings);
            sharedSetLayout = std::make_unique<vk::raii::DescriptorSetLayout>(context.device, setInfo);
        }

        // Set 1: G-buffer inputs, composed diffuse GI, specular output.
        {
            std::array<vk::DescriptorSetLayoutBinding, 5> bindings;
            for (std::uint32_t binding = 0; binding < 4; ++binding)
            {
                bindings[binding]
                    .setBinding(binding)
                    .setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
                    .setDescriptorCount(1);
                bindings[binding].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            }
            bindings[4].setBinding(4).setDescriptorType(vk::DescriptorType::eStorageImage).setDescriptorCount(1);
            bindings[4].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            vk::DescriptorSetLayoutCreateInfo setInfo;
            setInfo.setBindings(bindings);
            frameSetLayout = std::make_unique<vk::raii::DescriptorSetLayout>(context.device, setInfo);
        }

        vk::PushConstantRange pushRange;
        pushRange.setStageFlags(vk::ShaderStageFlagBits::eCompute);
        pushRange.setOffset(0);
        pushRange.setSize(sizeof(ReflectionPush));
        vk::PipelineLayoutCreateInfo                 layoutInfo;
        const std::array<vk::DescriptorSetLayout, 2> setLayouts{**sharedSetLayout, **frameSetLayout};
        layoutInfo.setSetLayouts(setLayouts);
        layoutInfo.setPushConstantRanges(pushRange);
        pipelineLayout = std::make_unique<vk::raii::PipelineLayout>(context.device, layoutInfo);

        const std::vector<char> code = readShaderBytes(
            context.shaderDirectory / "ray_reflections.comp.spv", moduleContext
        );
        vk::ShaderModuleCreateInfo moduleInfo;
        moduleInfo.setCodeSize(code.size()).setPCode(reinterpret_cast<const std::uint32_t *>(code.data()));
        vk::raii::ShaderModule            module(context.device, moduleInfo);
        vk::PipelineShaderStageCreateInfo stage;
        stage.setStage(vk::ShaderStageFlagBits::eCompute).setModule(*module).setPName("main");
        vk::ComputePipelineCreateInfo pipelineInfo;
        pipelineInfo.setStage(stage);
        pipelineInfo.setLayout(**pipelineLayout);
        pipeline = std::make_unique<vk::raii::Pipeline>(context.device, nullptr, pipelineInfo);

        // Shared set 0.
        {
            std::array<vk::DescriptorPoolSize, 3> poolSizes{
                vk::DescriptorPoolSize(vk::DescriptorType::eUniformBuffer, 3),
                vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 2),
                vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler, 2)
            };
            vk::DescriptorPoolCreateInfo poolInfo;
            poolInfo.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
            poolInfo.setMaxSets(1);
            poolInfo.setPoolSizes(poolSizes);
            sharedPool = std::make_unique<vk::raii::DescriptorPool>(context.device, poolInfo);
            vk::DescriptorSetAllocateInfo allocate;
            allocate.setDescriptorPool(**sharedPool);
            allocate.setSetLayouts(**sharedSetLayout);
            sharedSet = std::make_unique<vk::raii::DescriptorSet>(
                std::move(vk::raii::DescriptorSets(context.device, allocate).front())
            );

            // The relight needs the light uniform block: the module owns a
            // zeroed copy (lights arrive through the G-buffer's direct pass,
            // but SDF relighting reads the same uniform layout).
            vk::DescriptorBufferInfo cameraInfo(context.cameraBuffer, 0, sizeof(CameraData));
            vk::DescriptorBufferInfo primitivesInfo(context.primitivesBuffer, 0, vk::WholeSize);
            vk::DescriptorBufferInfo lightInfo(**lightStorage.buffer, 0, sizeof(GpuLightUniform));
            vk::DescriptorBufferInfo materialsInfo(context.materialsBuffer, 0, vk::WholeSize);
            vk::DescriptorImageInfo  albedoArrayInfo(
                **sampler, context.albedoArrayView, vk::ImageLayout::eShaderReadOnlyOptimal
            );
            vk::DescriptorImageInfo dataArrayInfo(
                **sampler, context.dataArrayView, vk::ImageLayout::eShaderReadOnlyOptimal
            );
            vk::DescriptorBufferInfo              environmentInfo(context.environmentBuffer, 0, vk::WholeSize);
            std::array<vk::WriteDescriptorSet, 7> writes;
            writes[0].setDstSet(**sharedSet).setDstBinding(0).setDescriptorType(vk::DescriptorType::eUniformBuffer);
            writes[0].setBufferInfo(cameraInfo);
            writes[1].setDstSet(**sharedSet).setDstBinding(1).setDescriptorType(vk::DescriptorType::eStorageBuffer);
            writes[1].setBufferInfo(primitivesInfo);
            writes[2].setDstSet(**sharedSet).setDstBinding(2).setDescriptorType(vk::DescriptorType::eUniformBuffer);
            writes[2].setBufferInfo(lightInfo);
            writes[3].setDstSet(**sharedSet).setDstBinding(3).setDescriptorType(vk::DescriptorType::eStorageBuffer);
            writes[3].setBufferInfo(materialsInfo);
            writes[4]
                .setDstSet(**sharedSet)
                .setDstBinding(4)
                .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
            writes[4].setImageInfo(albedoArrayInfo);
            writes[5]
                .setDstSet(**sharedSet)
                .setDstBinding(5)
                .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
            writes[5].setImageInfo(dataArrayInfo);
            writes[6].setDstSet(**sharedSet).setDstBinding(6).setDescriptorType(vk::DescriptorType::eUniformBuffer);
            writes[6].setBufferInfo(environmentInfo);
            context.device.updateDescriptorSets(writes, {});
        }

        // Frame set 1.
        {
            std::array<vk::DescriptorPoolSize, 2> poolSizes{
                vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler, 4),
                vk::DescriptorPoolSize(vk::DescriptorType::eStorageImage, 1)
            };
            vk::DescriptorPoolCreateInfo poolInfo;
            poolInfo.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
            poolInfo.setMaxSets(1);
            poolInfo.setPoolSizes(poolSizes);
            framePool = std::make_unique<vk::raii::DescriptorPool>(context.device, poolInfo);
            vk::DescriptorSetAllocateInfo allocate;
            allocate.setDescriptorPool(**framePool);
            allocate.setSetLayouts(**frameSetLayout);
            frameSet = std::make_unique<vk::raii::DescriptorSet>(
                std::move(vk::raii::DescriptorSets(context.device, allocate).front())
            );

            std::array<vk::DescriptorImageInfo, 4> inputs;
            inputs[0] = vk::DescriptorImageInfo(**sampler, context.albedoView, vk::ImageLayout::eGeneral);
            inputs[1] = vk::DescriptorImageInfo(**sampler, context.normalView, vk::ImageLayout::eGeneral);
            inputs[2] = vk::DescriptorImageInfo(**sampler, context.depthView, vk::ImageLayout::eGeneral);
            inputs[3] = vk::DescriptorImageInfo(**sampler, context.indirectRadianceView, vk::ImageLayout::eGeneral);
            vk::DescriptorImageInfo target(**sampler, context.indirectSpecularView, vk::ImageLayout::eGeneral);
            std::array<vk::WriteDescriptorSet, 5> writes;
            for (std::uint32_t binding = 0; binding < 4; ++binding)
            {
                writes[binding]
                    .setDstSet(**frameSet)
                    .setDstBinding(binding)
                    .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
                writes[binding].setImageInfo(inputs[binding]);
            }
            writes[4].setDstSet(**frameSet).setDstBinding(4).setDescriptorType(vk::DescriptorType::eStorageImage);
            writes[4].setImageInfo(target);
            context.device.updateDescriptorSets(writes, {});
        }

        cachedExtent  = context.extent;
        needsRecreate = false;
    }

    Settings                                       settings;
    LightingCapability                            *lighting = nullptr;
    vk::Extent2D                                   cachedExtent{};
    bool                                           needsRecreate = false;
    MappedHostVisibleBuffer                        lightStorage;
    std::unique_ptr<vk::raii::Sampler>             sampler;
    std::unique_ptr<vk::raii::DescriptorSetLayout> sharedSetLayout;
    std::unique_ptr<vk::raii::DescriptorSetLayout> frameSetLayout;
    std::unique_ptr<vk::raii::DescriptorPool>      sharedPool;
    std::unique_ptr<vk::raii::DescriptorSet>       sharedSet;
    std::unique_ptr<vk::raii::DescriptorPool>      framePool;
    std::unique_ptr<vk::raii::DescriptorSet>       frameSet;
    std::unique_ptr<vk::raii::PipelineLayout>      pipelineLayout;
    std::unique_ptr<vk::raii::Pipeline>            pipeline;
};

} // namespace

// ---- Module lifecycle ---------------------------------------------------------------

RayReflectionsModule::RayReflectionsModule()
{
    componentManifest.name                 = "ray_reflections";
    componentManifest.kind                 = ComponentKind::Module;
    componentManifest.providedCapabilities = {};
    componentManifest.requiredCapabilities = {"logging", "renderer"};
    // capability.lighting is optional at acquire time (a missing or denied
    // provider only strips the direct relight from reflection hits - they
    // keep emissive + sky), but the PERMISSION must still be declared:
    // acquireCapability throws on an undeclared permission, the construct
    // catch turned that into a PERMANENT null lighting pointer, and every
    // reflection then rendered lightless (black mirror ball) - exactly the
    // silent failure mode the permission system exists to make precise.
    componentManifest.requiredPermissions = {"capability.logging", "capability.renderer", "capability.lighting"};
    // The pass settings the module reads from [lighting.ray_reflections],
    // plus the technique composition list it shares with the renderer.
    componentManifest.knownConfigKeys = {
        "lighting.ray_reflections.max_roughness",
        "lighting.ray_reflections.max_distance",
        "lighting.ray_reflections.march_steps",
        "lighting.techniques",
    };
}

RayReflectionsModule::~RayReflectionsModule()
{
    stop();
}

void RayReflectionsModule::construct(
    ComponentContext &context
)
{
    logger = &context.acquireCapability<LoggerCapability>();

    RayReflectionsPass::Settings settings;
    settings.maxRoughness = static_cast<float>(context
            .configurationFloatingInSection("lighting", "ray_reflections.max_roughness")
            .value_or(defaultReflectionMaxRoughness));
    settings.maxDistance  = static_cast<float>(context
            .configurationFloatingInSection("lighting", "ray_reflections.max_distance")
            .value_or(defaultReflectionMaxDistance));
    settings.marchSteps   = static_cast<std::uint32_t>(std::clamp<std::int64_t>(
        context.configurationIntegerInSection("lighting", "ray_reflections.march_steps")
            .value_or(defaultReflectionMarchSteps),
        8,
        512
    ));
    if (settings.maxRoughness <= 0.0f || settings.maxDistance <= 0.0f)
    {
        throw std::runtime_error("ray_reflections module: max_roughness and max_distance must be > 0");
    }

    // The lighting capability is optional: without a provider the relight
    // sees no lights and reflections carry emissive + sky only.
    LightingCapability *lighting = nullptr;
    try
    {
        lighting = &context.acquireCapability<LightingCapability>();
    }
    catch (const ComponentAccessError &)
    {
        lighting = nullptr;
    }

    // Selection: [lighting] techniques = ["ray_reflections", ...].
    const std::vector<std::string> techniques = context.configurationStringArrayInSection("lighting", "techniques");
    passAttached = std::find(techniques.begin(), techniques.end(), "ray_reflections") != techniques.end();

    pass = std::make_shared<RayReflectionsPass>(settings, lighting);
    if (passAttached)
    {
        RendererCapability &rendererCapability = context.acquireCapability<RendererCapability>();
        auto               *rendererModule     = static_cast<RendererModule *>(&rendererCapability);
        rendererModule->attachLightingTechnique(pass);
    }
}

void RayReflectionsModule::start()
{
    logger->logMessage(
        LogLevel::Information,
        std::string("ray_reflections module started; technique ")
            + (passAttached ? "attached (indirect specular)" : "not listed in [lighting].techniques; idle")
    );
}

void RayReflectionsModule::stop()
{
    passAttached = false;
    pass.reset();
    logger = nullptr;
}

void *RayReflectionsModule::capabilityPointerFor(
    const std::string & /*capabilityName*/
)
{
    return nullptr;
}

} // namespace engine
