// fur_rendering.cpp - shell-fur asset + draw glue (see fur_rendering.hpp).

#include "fur_rendering.hpp"

#include <engine/renderer/capabilities.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace demo
{

using engine::renderer::RendererHandle;
using engine::renderer::RenderCapability;
using engine::renderer::TextureDescription;
using engine::renderer::TextureFilter;
using engine::renderer::TextureAddress;
using engine::renderer::TexturePixelFormat;
using engine::renderer::TextureHandle;
using engine::renderer::invalidTexture;

bool FurRendering::create(const RenderCapability &renderCapability, RendererHandle rendererHandle,
    const engine::fur::FurCapability *furCapability)
{
    furCapability_ = furCapability;
    renderCapability_ = &renderCapability;
    rendererHandle_ = rendererHandle;
    if (furCapability_ == nullptr)
    {
        // The fur module is not in this build: the demo keeps running with
        // its legacy vertex-tint fur (graceful degradation, documented in
        // the module overview).
        std::fprintf(stderr, "demo: fur capability absent; shell fur disabled\n");
        return true;
    }

    // One strand texture per shipped preset, generated deterministically and
    // uploaded into the renderer. The fur texture rides the material's
    // baseColorTexture slot: rgb = strand albedo variation, alpha = the
    // strand height field the shell layers crop against.
    static const char *presetNames[] = { "plush", "shaggy", "velvety" };
    textures_.clear();
    textures_.reserve(3);
    for (std::uint32_t presetIndex = 0; presetIndex < static_cast<std::uint32_t>(engine::fur::FurPreset::count);
         ++presetIndex)
    {
        const engine::fur::FurParameters parameters =
            furCapability_->presetParameters(furCapability_->self, static_cast<engine::fur::FurPreset>(presetIndex));

        constexpr std::uint32_t textureSize = 256;
        std::vector<std::uint8_t> pixels(static_cast<std::size_t>(textureSize) * textureSize * 4u);
        engine::fur::FurTextureRequest request{};
        request.parameters = parameters;
        request.width      = textureSize;
        request.height     = textureSize;
        if (!furCapability_->generateFurTexture(
                furCapability_->self, &request, pixels.data(), pixels.size()))
        {
            std::fprintf(stderr, "demo: fur texture generation failed for preset %s\n", presetNames[presetIndex]);
            return false;
        }

        TextureDescription description{};
        description.width      = textureSize;
        description.height     = textureSize;
        description.pixels     = pixels.data();
        description.filter     = TextureFilter::Linear;
        description.address    = TextureAddress::Repeat;
        description.pixelFormat = TexturePixelFormat::RGBA8;
        TextureHandle handle = renderCapability.createTexture(
            renderCapability.self, rendererHandle_, &description, nullptr);
        if (handle == invalidTexture)
        {
            std::fprintf(stderr, "demo: fur texture upload failed for preset %s\n", presetNames[presetIndex]);
            return false;
        }

        PresetFur entry;
        entry.parameters = parameters;
        entry.texture    = handle;
        entry.name       = presetNames[presetIndex];
        textures_.push_back(entry);
    }
    return true;
}

void FurRendering::destroy(const RenderCapability &renderCapability, RendererHandle rendererHandle)
{
    for (const PresetFur &entry : textures_)
    {
        if (entry.texture != invalidTexture)
        {
            renderCapability.destroyTexture(renderCapability.self, rendererHandle, entry.texture);
        }
    }
    textures_.clear();
    furCapability_   = nullptr;
    renderCapability_ = nullptr;
    rendererHandle_ = engine::renderer::invalidRenderer;
}

const FurRendering::PresetFur *FurRendering::presetFur(engine::fur::FurPreset preset) const
{
    const std::uint32_t index = static_cast<std::uint32_t>(preset);
    if (index >= textures_.size())
    {
        return nullptr;
    }
    return &textures_[index];
}

// Per-part strand-length scale: the same coat reads wrong at one length on
// every part - ear fur is short, tails carry the longest strands, necks sit
// between. Everything else (heads, animal bodies) uses the preset as-is.
float FurRendering::partLengthScale(const char *partId)
{
    if (partId == nullptr)
    {
        return 1.0f;
    }
    if (std::strncmp(partId, "ear_", 4) == 0)
    {
        return 0.55f;
    }
    if (std::strncmp(partId, "tail_", 5) == 0)
    {
        return 1.2f;
    }
    if (std::strncmp(partId, "neck_", 5) == 0)
    {
        return 0.9f;
    }
    return 1.0f;
}

void FurRendering::drawPartFur(engine::renderer::MeshHandle mesh, const float *modelMatrix,
    const engine::renderer::MeshMaterial &baseMaterial, const char *partId, float geometryRadius,
    float actorRadius, const float cameraPosition[3])
{
    lastDecision_ = engine::fur::FurLodDecision{};
    lastAngularSize_ = 0.0f;
    if (!enabled() || furCapability_ == nullptr || modelMatrix == nullptr || mesh == engine::renderer::invalidMesh)
    {
        return;
    }
    const PresetFur *fur = presetFur(preset_);
    if (fur == nullptr || fur->texture == invalidTexture)
    {
        return;
    }

    // World radius: the actor-level world bounds radius when provided
    // (already transformed by the character module), else the part-local
    // bounds radius scaled by the model basis (average axis length - the
    // same estimate the renderer's frustum cull uses); both add the strand
    // length so displaced shells stay inside the metric.
    float radius = 0.0f;
    if (actorRadius > 0.0f)
    {
        radius = actorRadius + fur->parameters.strandLength;
    }
    else
    {
        const glm::vec4 basisX(modelMatrix[0], modelMatrix[1], modelMatrix[2], modelMatrix[3]);
        const glm::vec4 basisY(modelMatrix[4], modelMatrix[5], modelMatrix[6], modelMatrix[7]);
        const glm::vec4 basisZ(modelMatrix[8], modelMatrix[9], modelMatrix[10], modelMatrix[11]);
        const float scale = (glm::length(glm::vec3(basisX)) + glm::length(glm::vec3(basisY))
                               + glm::length(glm::vec3(basisZ)))
                          / 3.0f;
        radius = std::max(geometryRadius * scale + fur->parameters.strandLength, 1e-3f);
    }
    const glm::vec4 worldCenter = glm::make_mat4(modelMatrix) * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    const float radius_ = std::max(radius, 1e-3f);
    const float distance = std::max(
        glm::length(glm::vec3(worldCenter) - glm::make_vec3(cameraPosition)), 1e-3f);
    const float angularSize = radius_ / distance;
    lastAngularSize_ = angularSize;

    engine::fur::FurLodDecision decision =
        lodEnabled_ ? furCapability_->evaluateFurLod(furCapability_->self, angularSize)
                    : engine::fur::FurLodDecision{};
    lastDecision_ = decision;
    if (layerOverride_ != 0u)
    {
        decision.layerCount     = layerOverride_;
        decision.outerLayerFade = 1.0f;
        decision.furHeightScale = 1.0f;
    }
    if (decision.layerCount == 0u)
    {
        return;
    }

    // The fur material: the base part's material with the strand texture in
    // the baseColorTexture slot. Vertex colors and the base color factor
    // still tint the strands (fur.frag multiplies all three).
    engine::renderer::MeshMaterial furMaterial = baseMaterial;
    furMaterial.baseColorTexture = fur->texture;

    engine::renderer::FurLayerParams layerParams;
    layerParams.layerCount      = decision.layerCount;
    layerParams.furHeight       = fur->parameters.strandLength * partLengthScale(partId) * lengthScale_
                                * decision.furHeightScale;
    layerParams.gravityStrength = fur->parameters.gravityStrength;
    layerParams.densityBoost    = fur->parameters.densityBoost;
    layerParams.outerLayerFade  = decision.outerLayerFade;

    // NOTE: the class holds the render capability through the screen's
    // pointer captured at create(); the draw call routes through it and is
    // clamped by the renderer module's fur_max_layers behind us.
    renderCapability_->drawFur(renderCapability_->self,
        rendererHandle_,
        mesh,
        modelMatrix,
        &furMaterial,
        &layerParams);
}

} // namespace demo
