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

vk::raii::ShaderModule createShaderModuleFromDevice(
    vk::raii::Device &device, const std::filesystem::path &path
)
{
    const std::vector<char>    bytes = readShaderBytes(path, logPrefix);
    vk::ShaderModuleCreateInfo moduleInfo;
    moduleInfo.setCodeSize(bytes.size()).setPCode(reinterpret_cast<const std::uint32_t *>(bytes.data()));
    return vk::raii::ShaderModule(device, moduleInfo);
}

void RendererState::createPipelines()
{
    // Explicit teardown in strict reverse dependency order before rebuilding:
    // pipelines embed their layouts, layouts embed the descriptor set layouts,
    // and sets embed all three. Member assignment alone would destroy in
    // declaration order, which can delete a layout under a still-alive
    // pipeline (docs/robustness.md, invariant 25). Called after a waitIdle from
    // recreateSwapchain and once during initialize, so this is always safe.
    compositeDescriptorSet.reset();
    compositeDescriptorPool.reset();
    compositePipeline.reset();
    compositePipelineLayout.reset();
    compositeSetLayout.reset();
    sceneDescriptorSet.reset();
    sceneDescriptorPool.reset();
    scenePipeline.reset();
    scenePipelineLayout.reset();
    sceneSetLayout.reset();

    createScenePipeline();
    createCompositePipeline();
}

void RendererState::createScenePipeline()
{
    // Scene pipeline: geometry pass with three color attachments + depth.
    {
        std::array<vk::DescriptorSetLayoutBinding, 4> bindings;
        // Camera uniform: vertex (transform) and fragment (scene_common's
        // declarations the fragment includes; the block itself is unused
        // there but part of the shader interface).
        bindings[0]
            .setBinding(sceneCameraBinding)
            .setDescriptorType(vk::DescriptorType::eUniformBuffer)
            .setDescriptorCount(1);
        bindings[0].setStageFlags(vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment);
        bindings[1]
            .setBinding(scenePrimitivesBinding)
            .setDescriptorType(vk::DescriptorType::eStorageBuffer)
            .setDescriptorCount(1);
        bindings[1].setStageFlags(vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment);
        bindings[2]
            .setBinding(sceneAlbedoArrayBinding)
            .setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
            .setDescriptorCount(1);
        bindings[2].setStageFlags(vk::ShaderStageFlagBits::eFragment);
        bindings[3]
            .setBinding(sceneDataArrayBinding)
            .setDescriptorType(vk::DescriptorType::eCombinedImageSampler)
            .setDescriptorCount(1);
        bindings[3].setStageFlags(vk::ShaderStageFlagBits::eFragment);
        vk::DescriptorSetLayoutCreateInfo setInfo;
        setInfo.setBindings(bindings);
        sceneSetLayout = std::make_unique<vk::raii::DescriptorSetLayout>(*device, setInfo);

        vk::PushConstantRange pushRange;
        pushRange.setStageFlags(vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment);
        pushRange.setOffset(0);
        pushRange.setSize(sizeof(ObjectPush));
        vk::PipelineLayoutCreateInfo layoutInfo;
        layoutInfo.setSetLayouts(**sceneSetLayout);
        layoutInfo.setPushConstantRanges(pushRange);
        scenePipelineLayout = std::make_unique<vk::raii::PipelineLayout>(*device, layoutInfo);

        vk::raii::ShaderModule vertexModule = createShaderModuleFromDevice(
            *device, resolvedShaderDirectory / "scene.vert.spv"
        );
        vk::raii::ShaderModule fragmentModule = createShaderModuleFromDevice(
            *device, resolvedShaderDirectory / "scene.frag.spv"
        );
        vk::PipelineShaderStageCreateInfo vertexStage;
        vertexStage.setStage(vk::ShaderStageFlagBits::eVertex).setModule(*vertexModule).setPName("main");
        vk::PipelineShaderStageCreateInfo fragmentStage;
        fragmentStage.setStage(vk::ShaderStageFlagBits::eFragment).setModule(*fragmentModule).setPName("main");
        const std::array<vk::PipelineShaderStageCreateInfo, 2> stages = {vertexStage, fragmentStage};

        vk::VertexInputBindingDescription vertexBinding;
        vertexBinding.setBinding(0);
        vertexBinding.setStride(meshFloatsPerVertex * sizeof(float));
        vertexBinding.setInputRate(vk::VertexInputRate::eVertex);
        std::array<vk::VertexInputAttributeDescription, 2> attributes;
        attributes[0].setBinding(0).setLocation(0).setFormat(vk::Format::eR32G32B32Sfloat).setOffset(0);
        attributes[1].setBinding(0).setLocation(1).setFormat(vk::Format::eR32G32B32Sfloat).setOffset(3 * sizeof(float));
        vk::PipelineVertexInputStateCreateInfo vertexInput;
        vertexInput.setVertexBindingDescriptions(vertexBinding);
        vertexInput.setVertexAttributeDescriptions(attributes);

        vk::PipelineInputAssemblyStateCreateInfo inputAssembly;
        inputAssembly.setTopology(vk::PrimitiveTopology::eTriangleList);

        vk::PipelineViewportStateCreateInfo viewportState;
        viewportState.setViewportCount(1).setScissorCount(1);

        vk::PipelineRasterizationStateCreateInfo rasterization;
        rasterization.setPolygonMode(vk::PolygonMode::eFill);
        rasterization.setCullMode(vk::CullModeFlagBits::eNone);
        rasterization.setFrontFace(vk::FrontFace::eCounterClockwise);
        rasterization.setLineWidth(1.0f);

        vk::PipelineMultisampleStateCreateInfo multisampling;
        multisampling.setRasterizationSamples(vk::SampleCountFlagBits::e1);

        vk::PipelineDepthStencilStateCreateInfo depthState;
        depthState.setDepthTestEnable(true);
        depthState.setDepthWriteEnable(true);
        depthState.setDepthCompareOp(vk::CompareOp::eLess);
        depthState.setMinDepthBounds(0.0f);
        depthState.setMaxDepthBounds(1.0f);

        std::array<vk::PipelineColorBlendAttachmentState, 3> blendAttachments;
        for (vk::PipelineColorBlendAttachmentState &attachment : blendAttachments)
        {
            attachment.setBlendEnable(false);
            attachment.setColorWriteMask(
                vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG | vk::ColorComponentFlagBits::eB
                | vk::ColorComponentFlagBits::eA
            );
        }
        vk::PipelineColorBlendStateCreateInfo colorBlend;
        colorBlend.setAttachmentCount(static_cast<std::uint32_t>(blendAttachments.size()));
        colorBlend.setPAttachments(blendAttachments.data());

        std::array<vk::DynamicState, 2>    dynamicStates = {vk::DynamicState::eViewport, vk::DynamicState::eScissor};
        vk::PipelineDynamicStateCreateInfo dynamicState;
        dynamicState.setDynamicStates(dynamicStates);

        std::array<vk::Format, 3>       colorFormats = {hdrColorFormat, gbufferAlbedoFormat, hdrColorFormat};
        vk::PipelineRenderingCreateInfo renderingInfo;
        renderingInfo.setColorAttachmentFormats(colorFormats);
        renderingInfo.setDepthAttachmentFormat(gbufferDepthFormat);

        vk::GraphicsPipelineCreateInfo pipelineInfo;
        pipelineInfo.setPNext(&renderingInfo);
        pipelineInfo.setStages(stages);
        pipelineInfo.setPVertexInputState(&vertexInput);
        pipelineInfo.setPInputAssemblyState(&inputAssembly);
        pipelineInfo.setPViewportState(&viewportState);
        pipelineInfo.setPRasterizationState(&rasterization);
        pipelineInfo.setPMultisampleState(&multisampling);
        pipelineInfo.setPDepthStencilState(&depthState);
        pipelineInfo.setPColorBlendState(&colorBlend);
        pipelineInfo.setPDynamicState(&dynamicState);
        pipelineInfo.setLayout(**scenePipelineLayout);
        pipelineInfo.setRenderPass(nullptr);

        scenePipeline = std::make_unique<vk::raii::Pipeline>(*device, nullptr, pipelineInfo);

        vk::DescriptorPoolSize                uniformPoolSize(vk::DescriptorType::eUniformBuffer, 1);
        vk::DescriptorPoolSize                storagePoolSize(vk::DescriptorType::eStorageBuffer, 1);
        vk::DescriptorPoolSize                arrayPoolSize(vk::DescriptorType::eCombinedImageSampler, 2);
        std::array<vk::DescriptorPoolSize, 3> scenePoolSizes{uniformPoolSize, storagePoolSize, arrayPoolSize};
        vk::DescriptorPoolCreateInfo          poolInfo;
        poolInfo.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
        poolInfo.setMaxSets(1);
        poolInfo.setPoolSizes(scenePoolSizes);
        sceneDescriptorPool = std::make_unique<vk::raii::DescriptorPool>(*device, poolInfo);
        vk::DescriptorSetAllocateInfo setAllocate;
        setAllocate.setDescriptorPool(**sceneDescriptorPool);
        setAllocate.setSetLayouts(**sceneSetLayout);
        sceneDescriptorSet = std::make_unique<vk::raii::DescriptorSet>(
            std::move(vk::raii::DescriptorSets(*device, setAllocate).front())
        );

        vk::DescriptorBufferInfo cameraBufferInfo(**cameraBufferStorage.buffer, 0, sizeof(CameraData));
        vk::DescriptorBufferInfo primitivesBufferInfo(**primitivesBuffer.buffer, 0, vk::WholeSize);
        vk::DescriptorImageInfo  albedoArrayInfo(
            **textureSampler, *albedoArrayImage.view, vk::ImageLayout::eShaderReadOnlyOptimal
        );
        vk::DescriptorImageInfo dataArrayInfo(
            **textureSampler, *dataArrayImage.view, vk::ImageLayout::eShaderReadOnlyOptimal
        );
        std::array<vk::WriteDescriptorSet, 4> sceneWrites;
        sceneWrites[0].setDstSet(**sceneDescriptorSet);
        sceneWrites[0].setDstBinding(sceneCameraBinding);
        sceneWrites[0].setDescriptorType(vk::DescriptorType::eUniformBuffer);
        sceneWrites[0].setBufferInfo(cameraBufferInfo);
        sceneWrites[1].setDstSet(**sceneDescriptorSet);
        sceneWrites[1].setDstBinding(scenePrimitivesBinding);
        sceneWrites[1].setDescriptorType(vk::DescriptorType::eStorageBuffer);
        sceneWrites[1].setBufferInfo(primitivesBufferInfo);
        sceneWrites[2].setDstSet(**sceneDescriptorSet);
        sceneWrites[2].setDstBinding(sceneAlbedoArrayBinding);
        sceneWrites[2].setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
        sceneWrites[2].setImageInfo(albedoArrayInfo);
        sceneWrites[3].setDstSet(**sceneDescriptorSet);
        sceneWrites[3].setDstBinding(sceneDataArrayBinding);
        sceneWrites[3].setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
        sceneWrites[3].setImageInfo(dataArrayInfo);
        device->updateDescriptorSets(sceneWrites, {});
    }
}

void RendererState::createCompositePipeline()
{
    // Composite pipeline: reads the lighting targets, writes the swapchain.
    {
        std::array<vk::DescriptorSetLayoutBinding, compositeCameraBinding + 1> bindings;
        for (std::uint32_t index = 0; index < compositeImageBindingCount; ++index)
        {
            bindings[index].setBinding(index);
            bindings[index].setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
            bindings[index].setDescriptorCount(1);
            bindings[index].setStageFlags(vk::ShaderStageFlagBits::eFragment);
        }
        bindings[compositeEnvironmentBinding]
            .setBinding(compositeEnvironmentBinding)
            .setDescriptorType(vk::DescriptorType::eUniformBuffer)
            .setDescriptorCount(1);
        bindings[compositeEnvironmentBinding].setStageFlags(vk::ShaderStageFlagBits::eFragment);
        bindings[compositeCameraBinding]
            .setBinding(compositeCameraBinding)
            .setDescriptorType(vk::DescriptorType::eUniformBuffer)
            .setDescriptorCount(1);
        bindings[compositeCameraBinding].setStageFlags(vk::ShaderStageFlagBits::eFragment);
        vk::DescriptorSetLayoutCreateInfo setInfo;
        setInfo.setBindings(bindings);
        compositeSetLayout = std::make_unique<vk::raii::DescriptorSetLayout>(*device, setInfo);

        vk::PushConstantRange pushRange;
        pushRange.setStageFlags(vk::ShaderStageFlagBits::eFragment);
        pushRange.setOffset(0);
        pushRange.setSize(sizeof(CompositePush));
        vk::PipelineLayoutCreateInfo layoutInfo;
        layoutInfo.setSetLayouts(**compositeSetLayout);
        layoutInfo.setPushConstantRanges(pushRange);
        compositePipelineLayout = std::make_unique<vk::raii::PipelineLayout>(*device, layoutInfo);

        vk::raii::ShaderModule vertexModule = createShaderModuleFromDevice(
            *device, resolvedShaderDirectory / "scene_composite.vert.spv"
        );
        vk::raii::ShaderModule fragmentModule = createShaderModuleFromDevice(
            *device, resolvedShaderDirectory / "scene_composite.frag.spv"
        );
        vk::PipelineShaderStageCreateInfo vertexStage;
        vertexStage.setStage(vk::ShaderStageFlagBits::eVertex).setModule(*vertexModule).setPName("main");
        vk::PipelineShaderStageCreateInfo fragmentStage;
        fragmentStage.setStage(vk::ShaderStageFlagBits::eFragment).setModule(*fragmentModule).setPName("main");
        const std::array<vk::PipelineShaderStageCreateInfo, 2> stages = {vertexStage, fragmentStage};

        vk::PipelineVertexInputStateCreateInfo   vertexInput;
        vk::PipelineInputAssemblyStateCreateInfo inputAssembly;
        inputAssembly.setTopology(vk::PrimitiveTopology::eTriangleList);
        vk::PipelineViewportStateCreateInfo viewportState;
        viewportState.setViewportCount(1).setScissorCount(1);
        vk::PipelineRasterizationStateCreateInfo rasterization;
        rasterization.setPolygonMode(vk::PolygonMode::eFill);
        rasterization.setCullMode(vk::CullModeFlagBits::eNone);
        rasterization.setLineWidth(1.0f);
        vk::PipelineMultisampleStateCreateInfo multisampling;
        multisampling.setRasterizationSamples(vk::SampleCountFlagBits::e1);
        std::array<vk::PipelineColorBlendAttachmentState, 1> blendAttachments;
        blendAttachments[0].setBlendEnable(false);
        blendAttachments[0].setColorWriteMask(
            vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG | vk::ColorComponentFlagBits::eB
            | vk::ColorComponentFlagBits::eA
        );
        vk::PipelineColorBlendStateCreateInfo colorBlend;
        colorBlend.setAttachmentCount(1).setPAttachments(blendAttachments.data());
        std::array<vk::DynamicState, 2>    dynamicStates = {vk::DynamicState::eViewport, vk::DynamicState::eScissor};
        vk::PipelineDynamicStateCreateInfo dynamicState;
        dynamicState.setDynamicStates(dynamicStates);

        vk::PipelineRenderingCreateInfo renderingInfo;
        renderingInfo.setColorAttachmentFormats(swapchainFormat);

        vk::GraphicsPipelineCreateInfo pipelineInfo;
        pipelineInfo.setPNext(&renderingInfo);
        pipelineInfo.setStages(stages);
        pipelineInfo.setPVertexInputState(&vertexInput);
        pipelineInfo.setPInputAssemblyState(&inputAssembly);
        pipelineInfo.setPViewportState(&viewportState);
        pipelineInfo.setPRasterizationState(&rasterization);
        pipelineInfo.setPMultisampleState(&multisampling);
        pipelineInfo.setPColorBlendState(&colorBlend);
        pipelineInfo.setPDynamicState(&dynamicState);
        pipelineInfo.setLayout(**compositePipelineLayout);
        pipelineInfo.setRenderPass(nullptr);

        compositePipeline = std::make_unique<vk::raii::Pipeline>(*device, nullptr, pipelineInfo);

        std::array<vk::DescriptorPoolSize, 2> compositePoolSizes{
            vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler, compositeImageBindingCount),
            vk::DescriptorPoolSize(
                vk::DescriptorType::eUniformBuffer, compositeCameraBinding + 1 - compositeImageBindingCount
            )
        };
        vk::DescriptorPoolCreateInfo poolInfo;
        poolInfo.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet);
        poolInfo.setMaxSets(1);
        poolInfo.setPoolSizes(compositePoolSizes);
        compositeDescriptorPool = std::make_unique<vk::raii::DescriptorPool>(*device, poolInfo);
        vk::DescriptorSetAllocateInfo setAllocate;
        setAllocate.setDescriptorPool(**compositeDescriptorPool);
        setAllocate.setSetLayouts(**compositeSetLayout);
        compositeDescriptorSet = std::make_unique<vk::raii::DescriptorSet>(
            std::move(vk::raii::DescriptorSets(*device, setAllocate).front())
        );
        writeCompositeDescriptors();
    }

    if (bloomEnabled)
    {
        createBloomPipelines();
    }
}

void RendererState::createBloomPipelines()
{
    // Strict reverse-order teardown first (docs/robustness.md, invariant 25):
    // sets, pools, pipelines, layouts, set layouts.
    bloomExtractDescriptorSet.reset();
    bloomBlurSetA.reset();
    bloomBlurSetB.reset();
    bloomExtractDescriptorPool.reset();
    bloomBlurDescriptorPool.reset();
    bloomExtractPipeline.reset();
    bloomBlurPipeline.reset();
    bloomExtractPipelineLayout.reset();
    bloomBlurPipelineLayout.reset();
    bloomExtractSetLayout.reset();
    bloomBlurSetLayout.reset();
    bloomFinalDescriptorSet.reset();
    bloomFinalDescriptorPool.reset();
    bloomFinalPipeline.reset();
    bloomFinalPipelineLayout.reset();
    bloomFinalSetLayout.reset();

    // ---- bright extract (compute) --------------------------------------------
    {
        std::array<vk::DescriptorSetLayoutBinding, 2> bindings;
        bindings[0].setBinding(0).setDescriptorType(vk::DescriptorType::eCombinedImageSampler).setDescriptorCount(1);
        bindings[0].setStageFlags(vk::ShaderStageFlagBits::eCompute);
        bindings[1].setBinding(1).setDescriptorType(vk::DescriptorType::eStorageImage).setDescriptorCount(1);
        bindings[1].setStageFlags(vk::ShaderStageFlagBits::eCompute);
        vk::DescriptorSetLayoutCreateInfo setInfo;
        setInfo.setBindings(bindings);
        bloomExtractSetLayout = std::make_unique<vk::raii::DescriptorSetLayout>(*device, setInfo);

        vk::PushConstantRange pushRange;
        pushRange.setStageFlags(vk::ShaderStageFlagBits::eCompute);
        pushRange.setOffset(0);
        pushRange.setSize(sizeof(vk::DeviceSize) > 0 ? 16 : 16); // one vec4
        vk::PipelineLayoutCreateInfo layoutInfo;
        layoutInfo.setSetLayouts(**bloomExtractSetLayout);
        layoutInfo.setPushConstantRanges(pushRange);
        bloomExtractPipelineLayout = std::make_unique<vk::raii::PipelineLayout>(*device, layoutInfo);

        vk::raii::ShaderModule module = createShaderModuleFromDevice(
            *device, resolvedShaderDirectory / "scene_bloom_extract.comp.spv"
        );
        vk::PipelineShaderStageCreateInfo stage;
        stage.setStage(vk::ShaderStageFlagBits::eCompute).setModule(*module).setPName("main");
        vk::ComputePipelineCreateInfo pipelineInfo;
        pipelineInfo.setStage(stage);
        pipelineInfo.setLayout(**bloomExtractPipelineLayout);
        bloomExtractPipeline = std::make_unique<vk::raii::Pipeline>(*device, nullptr, pipelineInfo);
    }

    // ---- separable blur (compute) ---------------------------------------------
    {
        std::array<vk::DescriptorSetLayoutBinding, 2> bindings;
        bindings[0].setBinding(0).setDescriptorType(vk::DescriptorType::eCombinedImageSampler).setDescriptorCount(1);
        bindings[0].setStageFlags(vk::ShaderStageFlagBits::eCompute);
        bindings[1].setBinding(1).setDescriptorType(vk::DescriptorType::eStorageImage).setDescriptorCount(1);
        bindings[1].setStageFlags(vk::ShaderStageFlagBits::eCompute);
        vk::DescriptorSetLayoutCreateInfo setInfo;
        setInfo.setBindings(bindings);
        bloomBlurSetLayout = std::make_unique<vk::raii::DescriptorSetLayout>(*device, setInfo);

        vk::PushConstantRange pushRange;
        pushRange.setStageFlags(vk::ShaderStageFlagBits::eCompute);
        pushRange.setOffset(0);
        pushRange.setSize(16); // one vec4
        vk::PipelineLayoutCreateInfo layoutInfo;
        layoutInfo.setSetLayouts(**bloomBlurSetLayout);
        layoutInfo.setPushConstantRanges(pushRange);
        bloomBlurPipelineLayout = std::make_unique<vk::raii::PipelineLayout>(*device, layoutInfo);

        vk::raii::ShaderModule module = createShaderModuleFromDevice(
            *device, resolvedShaderDirectory / "scene_bloom_blur.comp.spv"
        );
        vk::PipelineShaderStageCreateInfo stage;
        stage.setStage(vk::ShaderStageFlagBits::eCompute).setModule(*module).setPName("main");
        vk::ComputePipelineCreateInfo pipelineInfo;
        pipelineInfo.setStage(stage);
        pipelineInfo.setLayout(**bloomBlurPipelineLayout);
        bloomBlurPipeline = std::make_unique<vk::raii::Pipeline>(*device, nullptr, pipelineInfo);
    }

    // ---- final composite (graphics, into the swapchain) -----------------------
    {
        std::array<vk::DescriptorSetLayoutBinding, 2> bindings;
        for (std::uint32_t binding = 0; binding < 2; ++binding)
        {
            bindings[binding].setBinding(binding).setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
            bindings[binding].setDescriptorCount(1);
            bindings[binding].setStageFlags(vk::ShaderStageFlagBits::eFragment);
        }
        vk::DescriptorSetLayoutCreateInfo setInfo;
        setInfo.setBindings(bindings);
        bloomFinalSetLayout = std::make_unique<vk::raii::DescriptorSetLayout>(*device, setInfo);

        vk::PushConstantRange pushRange;
        pushRange.setStageFlags(vk::ShaderStageFlagBits::eFragment);
        pushRange.setOffset(0);
        pushRange.setSize(sizeof(CompositePush));
        vk::PipelineLayoutCreateInfo layoutInfo;
        layoutInfo.setSetLayouts(**bloomFinalSetLayout);
        layoutInfo.setPushConstantRanges(pushRange);
        bloomFinalPipelineLayout = std::make_unique<vk::raii::PipelineLayout>(*device, layoutInfo);

        vk::raii::ShaderModule vertexModule = createShaderModuleFromDevice(
            *device, resolvedShaderDirectory / "scene_composite.vert.spv"
        );
        vk::raii::ShaderModule fragmentModule = createShaderModuleFromDevice(
            *device, resolvedShaderDirectory / "scene_composite_final.frag.spv"
        );
        vk::PipelineShaderStageCreateInfo vertexStage;
        vertexStage.setStage(vk::ShaderStageFlagBits::eVertex).setModule(*vertexModule).setPName("main");
        vk::PipelineShaderStageCreateInfo fragmentStage;
        fragmentStage.setStage(vk::ShaderStageFlagBits::eFragment).setModule(*fragmentModule).setPName("main");
        const std::array<vk::PipelineShaderStageCreateInfo, 2> stages = {vertexStage, fragmentStage};

        vk::PipelineVertexInputStateCreateInfo   vertexInput;
        vk::PipelineInputAssemblyStateCreateInfo inputAssembly;
        inputAssembly.setTopology(vk::PrimitiveTopology::eTriangleList);
        vk::PipelineViewportStateCreateInfo viewportState;
        viewportState.setViewportCount(1).setScissorCount(1);
        vk::PipelineRasterizationStateCreateInfo rasterization;
        rasterization.setPolygonMode(vk::PolygonMode::eFill);
        rasterization.setCullMode(vk::CullModeFlagBits::eNone);
        rasterization.setLineWidth(1.0f);
        vk::PipelineMultisampleStateCreateInfo multisampling;
        multisampling.setRasterizationSamples(vk::SampleCountFlagBits::e1);
        std::array<vk::PipelineColorBlendAttachmentState, 1> blendAttachments;
        blendAttachments[0].setBlendEnable(false);
        blendAttachments[0].setColorWriteMask(
            vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG | vk::ColorComponentFlagBits::eB
            | vk::ColorComponentFlagBits::eA
        );
        vk::PipelineColorBlendStateCreateInfo colorBlend;
        colorBlend.setAttachmentCount(1).setPAttachments(blendAttachments.data());
        std::array<vk::DynamicState, 2>    dynamicStates = {vk::DynamicState::eViewport, vk::DynamicState::eScissor};
        vk::PipelineDynamicStateCreateInfo dynamicState;
        dynamicState.setDynamicStates(dynamicStates);

        vk::PipelineRenderingCreateInfo renderingInfo;
        renderingInfo.setColorAttachmentFormats(swapchainFormat);

        vk::GraphicsPipelineCreateInfo pipelineInfo;
        pipelineInfo.setPNext(&renderingInfo);
        pipelineInfo.setStages(stages);
        pipelineInfo.setPVertexInputState(&vertexInput);
        pipelineInfo.setPInputAssemblyState(&inputAssembly);
        pipelineInfo.setPViewportState(&viewportState);
        pipelineInfo.setPRasterizationState(&rasterization);
        pipelineInfo.setPMultisampleState(&multisampling);
        pipelineInfo.setPColorBlendState(&colorBlend);
        pipelineInfo.setPDynamicState(&dynamicState);
        pipelineInfo.setLayout(**bloomFinalPipelineLayout);
        bloomFinalPipeline = std::make_unique<vk::raii::Pipeline>(*device, nullptr, pipelineInfo);
    }

    writeBloomDescriptors();
}

void RendererState::writeBloomDescriptors()
{
    if (bloomExtractSetLayout == nullptr || hdrSceneImage.image == nullptr)
    {
        return;
    }
    // Extract: read the HDR scene, write bloom A.
    {
        vk::DescriptorPoolSize poolSize(vk::DescriptorType::eCombinedImageSampler, 1);
        vk::DescriptorPoolSize storageSize(vk::DescriptorType::eStorageImage, 1);
        std::array<vk::DescriptorPoolSize, 2> poolSizes{poolSize, storageSize};
        vk::DescriptorPoolCreateInfo poolInfo;
        poolInfo.setMaxSets(1).setPoolSizes(poolSizes);
        bloomExtractDescriptorPool = std::make_unique<vk::raii::DescriptorPool>(*device, poolInfo);
        vk::DescriptorSetAllocateInfo setAllocate;
        setAllocate.setDescriptorPool(**bloomExtractDescriptorPool);
        setAllocate.setSetLayouts(**bloomExtractSetLayout);
        bloomExtractDescriptorSet = std::make_unique<vk::raii::DescriptorSet>(
            std::move(vk::raii::DescriptorSets(*device, setAllocate).front())
        );
        std::array<vk::WriteDescriptorSet, 2> writes;
        vk::DescriptorImageInfo hdrInfo(**linearSampler, *hdrSceneImage.view, vk::ImageLayout::eGeneral);
        writes[0].setDstSet(**bloomExtractDescriptorSet).setDstBinding(0);
        writes[0].setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
        writes[0].setImageInfo(hdrInfo);
        vk::DescriptorImageInfo writeAInfo(vk::Sampler{nullptr}, *bloomImageA.view, vk::ImageLayout::eGeneral);
        writes[1].setDstSet(**bloomExtractDescriptorSet).setDstBinding(1);
        writes[1].setDescriptorType(vk::DescriptorType::eStorageImage);
        writes[1].setImageInfo(writeAInfo);
        device->updateDescriptorSets(writes, {});
    }
    // Blur: two sets (read A -> write B, read B -> write A).
    {
        std::array<vk::DescriptorPoolSize, 2> poolSizes{
            vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler, 2),
            vk::DescriptorPoolSize(vk::DescriptorType::eStorageImage, 2)};
        vk::DescriptorPoolCreateInfo poolInfo;
        poolInfo.setMaxSets(2).setPoolSizes(poolSizes);
        bloomBlurDescriptorPool = std::make_unique<vk::raii::DescriptorPool>(*device, poolInfo);
        std::array<vk::DescriptorSetLayout, 2> layouts{**bloomBlurSetLayout, **bloomBlurSetLayout};
        vk::DescriptorSetAllocateInfo setAllocate;
        setAllocate.setDescriptorPool(**bloomBlurDescriptorPool);
        setAllocate.setSetLayouts(layouts);
        std::vector<vk::raii::DescriptorSet> sets =
            vk::raii::DescriptorSets(*device, setAllocate);
        bloomBlurSetA = std::make_unique<vk::raii::DescriptorSet>(std::move(sets.front()));
        bloomBlurSetB = std::make_unique<vk::raii::DescriptorSet>(std::move(sets.back()));

        std::array<vk::WriteDescriptorSet, 4> writes;
        vk::DescriptorImageInfo readAInfo(**linearSampler, *bloomImageA.view, vk::ImageLayout::eGeneral);
        vk::DescriptorImageInfo writeBInfo(vk::Sampler{nullptr}, *bloomImageB.view, vk::ImageLayout::eGeneral);
        vk::DescriptorImageInfo readBInfo(**linearSampler, *bloomImageB.view, vk::ImageLayout::eGeneral);
        vk::DescriptorImageInfo writeAInfo(vk::Sampler{nullptr}, *bloomImageA.view, vk::ImageLayout::eGeneral);
        writes[0].setDstSet(**bloomBlurSetA).setDstBinding(0);
        writes[0].setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
        writes[0].setImageInfo(readAInfo);
        writes[1].setDstSet(**bloomBlurSetA).setDstBinding(1);
        writes[1].setDescriptorType(vk::DescriptorType::eStorageImage);
        writes[1].setImageInfo(writeBInfo);
        writes[2].setDstSet(**bloomBlurSetB).setDstBinding(0);
        writes[2].setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
        writes[2].setImageInfo(readBInfo);
        writes[3].setDstSet(**bloomBlurSetB).setDstBinding(1);
        writes[3].setDescriptorType(vk::DescriptorType::eStorageImage);
        writes[3].setImageInfo(writeAInfo);
        device->updateDescriptorSets(writes, {});
    }
    // Final: read the HDR scene + the blurred result.
    {
        std::array<vk::DescriptorPoolSize, 1> poolSizes{
            vk::DescriptorPoolSize(vk::DescriptorType::eCombinedImageSampler, 2)};
        vk::DescriptorPoolCreateInfo poolInfo;
        poolInfo.setMaxSets(1).setPoolSizes(poolSizes);
        bloomFinalDescriptorPool = std::make_unique<vk::raii::DescriptorPool>(*device, poolInfo);
        vk::DescriptorSetAllocateInfo setAllocate;
        setAllocate.setDescriptorPool(**bloomFinalDescriptorPool);
        setAllocate.setSetLayouts(**bloomFinalSetLayout);
        bloomFinalDescriptorSet = std::make_unique<vk::raii::DescriptorSet>(
            std::move(vk::raii::DescriptorSets(*device, setAllocate).front())
        );
        std::array<vk::WriteDescriptorSet, 2> writes;
        vk::DescriptorImageInfo hdrInfo(**linearSampler, *hdrSceneImage.view, vk::ImageLayout::eGeneral);
        vk::DescriptorImageInfo bloomInfo(**linearSampler, *bloomImageA.view, vk::ImageLayout::eGeneral);
        writes[0].setDstSet(**bloomFinalDescriptorSet).setDstBinding(0);
        writes[0].setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
        writes[0].setImageInfo(hdrInfo);
        writes[1].setDstSet(**bloomFinalDescriptorSet).setDstBinding(1);
        writes[1].setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
        writes[1].setImageInfo(bloomInfo);
        device->updateDescriptorSets(writes, {});
    }
}

void RendererState::writeCompositeDescriptors()
{
    std::array<vk::DescriptorImageInfo, compositeImageBindingCount> images;
    images[0] = vk::DescriptorImageInfo(**linearSampler, *albedoImage.view, vk::ImageLayout::eGeneral);
    images[1] = vk::DescriptorImageInfo(**linearSampler, *emissiveImage.view, vk::ImageLayout::eGeneral);
    images[2] = vk::DescriptorImageInfo(
        **linearSampler,
        hasDirectRadianceTarget ? *directRadianceImage.view : *dummyImage.view,
        vk::ImageLayout::eGeneral
    );
    images[3] = vk::DescriptorImageInfo(
        **linearSampler,
        hasIndirectDiffuseTarget ? *indirectRadianceImage.view : *dummyImage.view,
        vk::ImageLayout::eGeneral
    );
    images[4] = vk::DescriptorImageInfo(**linearSampler, *depthImage.view, vk::ImageLayout::eGeneral);
    images[5] = vk::DescriptorImageInfo(**linearSampler, *normalImage.view, vk::ImageLayout::eGeneral);
    images[6] = vk::DescriptorImageInfo(
        **linearSampler,
        hasIndirectSpecularTarget ? *indirectSpecularImage.view : *dummyImage.view,
        vk::ImageLayout::eGeneral
    );

    std::array<vk::WriteDescriptorSet, compositeCameraBinding + 1> writes;
    for (std::uint32_t binding = 0; binding < compositeImageBindingCount; ++binding)
    {
        writes[binding].setDstSet(**compositeDescriptorSet);
        writes[binding].setDstBinding(binding);
        writes[binding].setDescriptorType(vk::DescriptorType::eCombinedImageSampler);
        writes[binding].setImageInfo(images[binding]);
    }
    vk::DescriptorBufferInfo environmentInfo(**environmentBufferStorage.buffer, 0, sizeof(EnvironmentData));
    writes[compositeEnvironmentBinding].setDstSet(**compositeDescriptorSet);
    writes[compositeEnvironmentBinding].setDstBinding(compositeEnvironmentBinding);
    writes[compositeEnvironmentBinding].setDescriptorType(vk::DescriptorType::eUniformBuffer);
    writes[compositeEnvironmentBinding].setBufferInfo(environmentInfo);
    vk::DescriptorBufferInfo cameraInfo(**cameraBufferStorage.buffer, 0, sizeof(CameraData));
    writes[compositeCameraBinding].setDstSet(**compositeDescriptorSet);
    writes[compositeCameraBinding].setDstBinding(compositeCameraBinding);
    writes[compositeCameraBinding].setDescriptorType(vk::DescriptorType::eUniformBuffer);
    writes[compositeCameraBinding].setBufferInfo(cameraInfo);
    device->updateDescriptorSets(writes, {});
}
} // namespace engine
