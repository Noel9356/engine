#include "pbr.hpp"

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

} // namespace

constexpr std::string_view moduleContext = "pbr module";

// Push-constant block mirrored by pbr_lighting.comp (shadow/flag scalars,
// dispatch counts, reflection-fold hand-off). File scope so the pipeline
// layout can size its range with sizeof instead of a raw byte count.
struct PbrPassPush
{
    std::array<float, 4>         shadowAndFlags;
    std::array<std::uint32_t, 4> counts;
    std::array<float, 4>         reflectionFoldPush;
};

static_assert(sizeof(PbrPassPush) == 48, "pbr_lighting.comp expects a 48-byte push block");

// The pass object handed to the renderer. Owns its Vulkan objects; creates
// them lazily on the first record (the renderer guarantees a live device by
// then) and rebuilds them whenever the frame resources were recreated.
//
// Vulkan object lifetime: the pipeline layout embeds the descriptor set
// layouts it was created from, and pipelines embed the layout, and descriptor
// sets embed both. Every rebuild therefore destroys in strict reverse
// dependency order - descriptor sets, pools, pipelines, pipeline layouts,
// descriptor set layouts, then buffers and samplers - and every object that
// outlives one rebuild is a declared member, never a local. A local layout
// here once died at the end of rebuild() while the pipeline kept referencing
// it, which NVIDIA's validation layer rejects on every frame bind
// (docs/robustness.md, problem 25).
class PbrLightingPass final : public LightingTechniquePass
{
public:
    struct ReflectionFoldSettings
    {
        // True when the reflection layer (indirect_specular) is active in
        // this run alongside this pass: the analytic specular then hands over
        // to the ray traced layer per pixel (see pbr_lighting.comp
        // reflectionWeight), so the sun is not drawn twice on glossy surfaces.
        bool  enabled      = false;
        float maxRoughness = defaultReflectionMaxRoughness;
    };

    PbrLightingPass(
        std::shared_ptr<LightState> sharedState, const ReflectionFoldSettings &reflectionFoldSettings
    )
        : lights(std::move(sharedState)), reflectionFold(reflectionFoldSettings)
    {
    }

    const char *techniqueName() const override
    {
        return "pbr";
    }

    LightingTarget targets() const override
    {
        return LightingTarget::Direct;
    }

    void record(
        vk::raii::CommandBuffer &commandBuffer, const FrameRenderContext &context
    ) override
    {
        if (pipeline == nullptr || context.extent != cachedExtent || needsRecreate)
        {
            rebuild(context);
        }

        // Light uniforms are read by this frame's dispatch; write them now.
        GpuLightUniform uniform;
        lights->fillUniform(uniform);
        std::memcpy(lightStorage.mapped, &uniform, sizeof(uniform));

        commandBuffer.bindPipeline(vk::PipelineBindPoint::eCompute, **pipeline);
        commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, **pipelineLayout, 0, **sharedSet, {});
        commandBuffer.bindDescriptorSets(vk::PipelineBindPoint::eCompute, **pipelineLayout, 1, **frameSet, {});

        const PbrPassPush pushValue{
            .shadowAndFlags     = {lights->shadowsEnabled ? 1.0f : 0.0f, lights->penumbra, 0.0f, 0.0f},
            .counts             = {context.primitiveCount, 0u, 0u, 0u},
            .reflectionFoldPush = {reflectionFold.enabled ? 1.0f : 0.0f, reflectionFold.maxRoughness, 0.0f, 0.0f}
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
    void rebuild(
        const FrameRenderContext &context
    )
    {
        // Teardown in strict reverse dependency order (see the class comment).
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

        // Light uniform buffer: host visible, persistently mapped, updated
        // every frame (computation rule: one memcpy, no staging round trip).
        lightStorage = createMappedHostVisibleBuffer(
            context.device, context.physicalDevice, sizeof(GpuLightUniform), vk::BufferUsageFlagBits::eUniformBuffer
        );

        sampler = makeNearestClampSampler(context.device);

        // Set 0 layout: camera UBO, primitives SSBO, lights UBO.
        {
            std::array<vk::DescriptorSetLayoutBinding, 3> bindings;
            bindings[0].setBinding(0).setDescriptorType(vk::DescriptorType::eUniformBuffer).setDescriptorCount(1);
            bindings[0].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[1].setBinding(1).setDescriptorType(vk::DescriptorType::eStorageBuffer).setDescriptorCount(1);
            bindings[1].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            bindings[2].setBinding(2).setDescriptorType(vk::DescriptorType::eUniformBuffer).setDescriptorCount(1);
            bindings[2].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            vk::DescriptorSetLayoutCreateInfo setInfo;
            setInfo.setBindings(bindings);
            sharedSetLayout = std::make_unique<vk::raii::DescriptorSetLayout>(context.device, setInfo);
        }

        // Set 1 layout: G-buffer inputs + direct radiance target. A member:
        // the pipeline layout and the frame descriptor set both embed it, so
        // it must outlive this rebuild call (robustness.md problem 25).
        {
            std::array<vk::DescriptorSetLayoutBinding, 4> bindings;
            for (std::uint32_t binding = 0; binding < 3; ++binding)
            {
                bindings[binding]
                    .setBinding(binding)
                    .setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
                    .setDescriptorCount(1);
                bindings[binding].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            }
            bindings[3].setBinding(3).setDescriptorType(vk::DescriptorType::eStorageImage).setDescriptorCount(1);
            bindings[3].setStageFlags(vk::ShaderStageFlagBits::eCompute);
            vk::DescriptorSetLayoutCreateInfo setInfo;
            setInfo.setBindings(bindings);
            frameSetLayout = std::make_unique<vk::raii::DescriptorSetLayout>(context.device, setInfo);
        }

        vk::PushConstantRange pushRange;
        pushRange.setStageFlags(vk::ShaderStageFlagBits::eCompute);
        pushRange.setOffset(0);
        pushRange.setSize(sizeof(PbrPassPush));
        vk::PipelineLayoutCreateInfo                 layoutInfo;
        const std::array<vk::DescriptorSetLayout, 2> setLayouts{**sharedSetLayout, **frameSetLayout};
        layoutInfo.setSetLayouts(setLayouts);
        layoutInfo.setPushConstantRanges(pushRange);
        pipelineLayout = std::make_unique<vk::raii::PipelineLayout>(context.device, layoutInfo);

        const std::vector<char> code = readShaderBytes(
            context.shaderDirectory / "pbr_lighting.comp.spv", moduleContext
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
            std::array<vk::DescriptorPoolSize, 2> poolSizes{
                vk::DescriptorPoolSize(vk::DescriptorType::eUniformBuffer, 2),
                vk::DescriptorPoolSize(vk::DescriptorType::eStorageBuffer, 1)
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

            vk::DescriptorBufferInfo              cameraInfo(context.cameraBuffer, 0, sizeof(CameraData));
            vk::DescriptorBufferInfo              primitivesInfo(context.primitivesBuffer, 0, vk::WholeSize);
            vk::DescriptorBufferInfo              lightInfo(**lightStorage.buffer, 0, sizeof(GpuLightUniform));
            std::array<vk::WriteDescriptorSet, 3> writes;
            writes[0].setDstSet(**sharedSet).setDstBinding(0).setDescriptorType(vk::DescriptorType::eUniformBuffer);
            writes[0].setBufferInfo(cameraInfo);
            writes[1].setDstSet(**sharedSet).setDstBinding(1).setDescriptorType(vk::DescriptorType::eStorageBuffer);
            writes[1].setBufferInfo(primitivesInfo);
            writes[2].setDstSet(**sharedSet).setDstBinding(2).setDescriptorType(vk::DescriptorType::eUniformBuffer);
            writes[2].setBufferInfo(lightInfo);
            context.device.updateDescriptorSets(writes, {});
        }

        // Frame set 1.
        {
            std::array<vk::DescriptorPoolSize, 2> poolSizes{
                vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler, 3),
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

            std::array<vk::DescriptorImageInfo, 3> inputs;
            inputs[0] = vk::DescriptorImageInfo(**sampler, context.albedoView, vk::ImageLayout::eGeneral);
            inputs[1] = vk::DescriptorImageInfo(**sampler, context.normalView, vk::ImageLayout::eGeneral);
            inputs[2] = vk::DescriptorImageInfo(**sampler, context.depthView, vk::ImageLayout::eGeneral);
            vk::DescriptorImageInfo target(**sampler, context.directRadianceView, vk::ImageLayout::eGeneral);
            std::array<vk::WriteDescriptorSet, 4> writes;
            for (std::uint32_t binding = 0; binding < 3; ++binding)
            {
                writes[binding]
                    .setDstSet(**frameSet)
                    .setDstBinding(binding)
                    .setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
                writes[binding].setImageInfo(inputs[binding]);
            }
            writes[3].setDstSet(**frameSet).setDstBinding(3).setDescriptorType(vk::DescriptorType::eStorageImage);
            writes[3].setImageInfo(target);
            context.device.updateDescriptorSets(writes, {});
        }

        cachedExtent  = context.extent;
        needsRecreate = false;
    }

    std::shared_ptr<LightState>        lights;
    ReflectionFoldSettings             reflectionFold;
    vk::Extent2D                       cachedExtent{};
    bool                               needsRecreate = false;
    MappedHostVisibleBuffer            lightStorage;
    std::unique_ptr<vk::raii::Sampler> sampler;
    // Layouts are declared BEFORE the pools/sets/pipelines that embed them on
    // purpose: with this order even the automatic member destruction (which
    // runs top to bottom) tears pipelines down before their layouts and
    // layouts before their descriptor set layouts.
    std::unique_ptr<vk::raii::DescriptorSetLayout> sharedSetLayout;
    std::unique_ptr<vk::raii::DescriptorSetLayout> frameSetLayout;
    std::unique_ptr<vk::raii::DescriptorPool>      sharedPool;
    std::unique_ptr<vk::raii::DescriptorSet>       sharedSet;
    std::unique_ptr<vk::raii::DescriptorPool>      framePool;
    std::unique_ptr<vk::raii::DescriptorSet>       frameSet;
    std::unique_ptr<vk::raii::PipelineLayout>      pipelineLayout;
    std::unique_ptr<vk::raii::Pipeline>            pipeline;
};

// ---- Module lifecycle ----------------------------------------------------------------

PbrLightingModule::PbrLightingModule()
{
    componentManifest.name = "pbr";
    componentManifest.kind = ComponentKind::Module;
    // "lighting" is the scene light state (sun + point lights, seeded from
    // the shared [scene] world content, mutable by mods through the contract).
    // "direct_diffuse" is this technique's radiance layer: the run demands it
    // when the direct pass should attach to the renderer.
    componentManifest.providedCapabilities = {"lighting", "direct_diffuse"};
    componentManifest.requiredCapabilities = {"logging", "renderer"};
    componentManifest.requiredPermissions  = {"capability.logging", "capability.renderer"};
    // The light state is seeded from the shared [scene] world content; the
    // technique reads its own [lighting.pbr] settings plus the shared
    // specular hand-off key ([lighting].max_roughness) it splits the glossy
    // energy with the reflection layer through.
    componentManifest.knownConfigKeys = {
        "lighting.pbr.shadows",
        "lighting.pbr.penumbra",
        "lighting.pbr.shadow_steps",
        "lighting.max_roughness",
        "scene.sun.direction",
        "scene.sun.color",
        "scene.sun.intensity",
        "scene.point_lights",
    };
}

PbrLightingModule::~PbrLightingModule()
{
    stop();
}

void PbrLightingModule::construct(
    ComponentContext &context
)
{
    logger = &context.acquireCapability<LoggerCapability>();

    // The light state is shared with the pass so the capability API and the
    // GPU upload always agree.
    state                 = std::make_shared<LightState>();
    state->shadowsEnabled = context.configurationBooleanInSection("lighting", "pbr.shadows").value_or(true);
    state->penumbra       = static_cast<float>(
        context.configurationFloatingInSection("lighting", "pbr.penumbra").value_or(12.0)
    );
    state->shadowSteps = static_cast<std::int32_t>(
        context.configurationIntegerInSection("lighting", "pbr.shadow_steps").value_or(24)
    );

    const std::vector<double> direction = context.configurationFloatingArrayInSection("scene", "sun.direction");
    if (direction.size() == 3)
    {
        state->sun.direction[0] = static_cast<float>(direction[0]);
        state->sun.direction[1] = static_cast<float>(direction[1]);
        state->sun.direction[2] = static_cast<float>(direction[2]);
    }
    const std::vector<double> color = context.configurationFloatingArrayInSection("scene", "sun.color");
    if (color.size() == 3)
    {
        state->sun.color[0] = static_cast<float>(color[0]);
        state->sun.color[1] = static_cast<float>(color[1]);
        state->sun.color[2] = static_cast<float>(color[2]);
    }
    state->sun.intensity = static_cast<float>(
        context.configurationFloatingInSection("scene", "sun.intensity").value_or(0.0)
    );
    state->sunEnabled = state->sun.intensity > 0.0f;

    // Point lights: [[scene.point_lights]] rows seed the capability state, so
    // configurations can place lights without a mod calling the API.
    const std::size_t pointLightCount = std::min<std::size_t>(
        context.configurationArraySizeInSection("scene", "point_lights"), maxPointLights
    );
    for (std::size_t index = 0; index < pointLightCount; ++index)
    {
        const std::string         prefix = "point_lights[" + std::to_string(index) + "]";
        PointLightDescription     light;
        const std::vector<double> position = context.configurationFloatingArrayInSection("scene", prefix + ".position");
        if (position.size() == 3)
        {
            light.position[0] = static_cast<float>(position[0]);
            light.position[1] = static_cast<float>(position[1]);
            light.position[2] = static_cast<float>(position[2]);
        }
        const std::vector<double> color = context.configurationFloatingArrayInSection("scene", prefix + ".color");
        if (color.size() == 3)
        {
            light.color[0] = static_cast<float>(color[0]);
            light.color[1] = static_cast<float>(color[1]);
            light.color[2] = static_cast<float>(color[2]);
        }
        light.intensity = static_cast<float>(
            context.configurationFloatingInSection("scene", prefix + ".intensity").value_or(1.0)
        );
        light.radius = static_cast<float>(
            context.configurationFloatingInSection("scene", prefix + ".radius").value_or(0.0)
        );
        state->pointLights.push_back(light);
    }

    // Attachment follows demand: the kernel activated this module because the
    // run required one of its capabilities. The direct pass attaches only when
    // the run demanded the direct_diffuse layer (a module activated just for
    // the light state stays an idle provider). Asking about one's OWN
    // capability is the one question the demand set answers.
    passAttached = context.isCapabilityDemanded("direct_diffuse");

    // Specular hand-off: when the run also demands the indirect_specular
    // layer, this pass dims its analytic specular by the same Fresnel weight
    // the reflection pass uses, so the sun is not drawn twice on glossy
    // pixels. Both layers read the shared [lighting].max_roughness key, so
    // the two sides of the hand-off can never drift apart.
    PbrLightingPass::ReflectionFoldSettings reflectionFold;
    if (context.isCapabilityDemanded("indirect_specular"))
    {
        reflectionFold.enabled      = true;
        reflectionFold.maxRoughness = static_cast<float>(
            context.configurationFloatingInSection("lighting", "max_roughness").value_or(0.35)
        );
    }

    // The pass object exists regardless; it only records when attached. The
    // renderer capability exists after the renderer module constructed (the
    // resolver orders constructs by the declared requirements).
    pass = std::make_shared<PbrLightingPass>(state, reflectionFold);
    if (passAttached)
    {
        RendererCapability &rendererCapability = context.acquireCapability<RendererCapability>();
        // In-tree wiring: the renderer capability of this build is implemented
        // by RendererModule (see host/main.cpp, composition root). Script mods
        // keep working through the agnostic contracts only.
        auto *rendererModule = static_cast<RendererModule *>(&rendererCapability);
        rendererModule->attachLightingTechnique(pass);
    }
}

void PbrLightingModule::start()
{
    logger->logMessage(
        LogLevel::Information,
        std::string("pbr module started; technique ")
            + (passAttached ? "attached (direct radiance)" : "direct_diffuse not demanded; idle (light state only)")
            + ", sun " + (state->sunEnabled ? "on" : "off") + ", shadows " + (state->shadowsEnabled ? "on" : "off")
    );
}

void PbrLightingModule::stop()
{
    if (passAttached)
    {
        // Detach before the pass dies: the renderer may outlive this module.
        // The renderer capability is still alive during stop (reverse order).
        passAttached = false;
        // The shared_ptr held by the renderer keeps the pass object alive
        // until the renderer detaches or stops; the light state outlives both
        // through the shared ownership.
    }
    pass.reset();
    logger = nullptr;
}

void *PbrLightingModule::capabilityPointerFor(
    const std::string &capabilityName
)
{
    if (capabilityName == LightingCapability::capabilityName)
    {
        return static_cast<LightingCapability *>(this);
    }
    // "direct_diffuse" is a demand marker: it carries no call interface, so
    // the implementation behind it is the module object itself.
    if (capabilityName == "direct_diffuse")
    {
        return this;
    }
    return nullptr;
}

void PbrLightingModule::setSunLight(
    const DirectionalLightDescription &light
)
{
    state->sun        = light;
    state->sunEnabled = true;
}

void PbrLightingModule::clearSunLight()
{
    state->sunEnabled = false;
}

bool PbrLightingModule::hasSunLight() const
{
    return state->sunEnabled;
}

DirectionalLightDescription PbrLightingModule::sunLight() const
{
    return state->sun;
}

LightingCapability::PointLightHandle PbrLightingModule::addPointLight(
    const PointLightDescription &light
)
{
    if (state->pointLights.size() >= maxPointLights)
    {
        return LightingCapability::invalidPointLight;
    }
    state->pointLights.push_back(light);
    return static_cast<LightingCapability::PointLightHandle>(state->pointLights.size());
}

void PbrLightingModule::updatePointLight(
    LightingCapability::PointLightHandle handle, const PointLightDescription &light
)
{
    if (handle == LightingCapability::invalidPointLight || handle > state->pointLights.size())
    {
        return;
    }
    // In place: the handle's index mapping survives, so animation drivers can
    // move a light every frame without renumbering the others.
    state->pointLights[handle - 1] = light;
}

void PbrLightingModule::removePointLight(
    LightingCapability::PointLightHandle handle
)
{
    if (handle == LightingCapability::invalidPointLight || handle > state->pointLights.size())
    {
        return;
    }
    state->pointLights.erase(state->pointLights.begin() + static_cast<std::ptrdiff_t>(handle - 1));
}

void PbrLightingModule::clearPointLights()
{
    state->pointLights.clear();
}

std::uint32_t PbrLightingModule::pointLightCount() const
{
    return static_cast<std::uint32_t>(state->pointLights.size());
}

PointLightDescription PbrLightingModule::pointLight(
    std::uint32_t index
) const
{
    return state->pointLights.at(index);
}

} // namespace engine
