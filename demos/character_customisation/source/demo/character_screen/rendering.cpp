#include "../character_screen_internal.hpp"
#include "../character_content.hpp"
#include "../character_screen.hpp"
#include <engine/renderer/capabilities.hpp>

#include <cstdint>
#include <glm/glm.hpp>

#include <algorithm>

namespace demo
{
using namespace screen_detail;

void CharacterScreen::drawCharacter()
{
    // skin.tone indexes a five-swatch palette (see the panel); the tint is a
    // draw-time material multiplier on the skin-tagged parts, so vertex
    // colors keep their shading detail underneath. body.coat switches the
    // bare torso, limbs, and the neck lofts between the skin palette and the
    // fur coat - the coat is a change-time vertex-colour repaint
    // (applyDeformations, the same palette as the panel's fur.coat picker),
    // so here those parts draw untinted and matte instead of a flat material
    // tint. The animal grafts (species heads, ears, tails, the bodies) keep
    // their authored colours untouched: multiplying them by the palette
    // darkened the already-coloured fur and broke the match with the
    // repainted neck column at the atlas weld.
    float bodyCoat = 0.0f;
    character_->getParameter(character_->self, characterHandle_, "body.coat", &bodyCoat);
    const bool furryBody = bodyCoat >= 0.5f;
    float      skinTone  = 0.0f;
    character_->getParameter(character_->self, characterHandle_, "skin.tone", &skinTone);
    static const glm::vec3 toneTints[5] = {
        {1.00f, 1.00f, 1.00f},
        {0.92f, 0.80f, 0.68f},
        {0.82f, 0.68f, 0.55f},
        {0.66f, 0.52f, 0.42f},
        {0.42f, 0.40f, 0.42f},
    };
    const int           toneIndex = std::clamp(static_cast<int>(skinTone + 0.5f), 0, 4);
    const std::uint32_t count     = character_->instanceCount(character_->self, characterHandle_);
    for (std::uint32_t index = 0; index < count; ++index)
    {
        const InstanceId id     = character_->instanceId(character_->self, characterHandle_, index);
        const char      *partId = nullptr;
        if (!character_->instanceInfo(character_->self, characterHandle_, id, nullptr, &partId, nullptr))
        {
            continue;
        }
        const PartRenderInfo *renderInfo = content_.renderInfoForPart(partId);
        if (renderInfo == nullptr)
        {
            continue;
        }
        const MeshHandle mesh = content_.meshForGeometry(renderInfo->geometryId);
        if (mesh == engine::renderer::invalidMesh)
        {
            continue;
        }
        float matrix[16] = {};
        if (!character_->instanceTransform(character_->self, characterHandle_, id, matrix))
        {
            continue;
        }
        MeshMaterial material = renderInfo->material;
        if (isSkinPart(partId) && !furryBody)
        {
            const glm::vec3 &tint = toneTints[toneIndex];
            material.baseColor[0] *= tint.r;
            material.baseColor[1] *= tint.g;
            material.baseColor[2] *= tint.b;
        }
        else if (furryBody && isSkinPart(partId))
        {
            // The fur coat's colours are baked into the vertex colours; only
            // the matte finish rides here.
            material.roughnessFactor = 0.92f;
        }
        render_->drawMesh(render_->self, rendererHandle_, mesh, matrix, &material);
    }
}

// ---- UI ---------------------------------------------------------------------

} // namespace demo
