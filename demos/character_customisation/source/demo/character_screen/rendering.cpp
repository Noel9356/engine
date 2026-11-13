#include "../character_screen_internal.hpp"
#include "../geometry.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <utility>

namespace demo
{
using namespace screen_detail;

void CharacterScreen::drawCharacter()
{
    // skin.tone indexes a five-swatch palette (see the panel); the tint is a
    // draw-time material multiplier on the skin-tagged parts, so vertex
    // colors keep their shading detail underneath. fur.coat does the same
    // for the downloaded animal grafts (species heads/ears/tails and the
    // animal bodies) - the furry coat palette multiplies the pack's own base
    // colors.
    float       skinTone = 0.0f;
    character_->getParameter(character_->self, characterHandle_, "skin.tone", &skinTone);
    static const glm::vec3 toneTints[5] = {
        {1.00f, 1.00f, 1.00f},
        {0.92f, 0.80f, 0.68f},
        {0.82f, 0.68f, 0.55f},
        {0.66f, 0.52f, 0.42f},
        {0.42f, 0.40f, 0.42f},
    };
    float    furCoat = 0.0f;
    character_->getParameter(character_->self, characterHandle_, "fur.coat", &furCoat);
    // Neutral, rust red, snow white, charcoal, cream, silver blue.
    static const glm::vec3 coatTints[6] = {
        {1.00f, 1.00f, 1.00f},
        {0.94f, 0.62f, 0.42f},
        {1.04f, 1.04f, 1.02f},
        {0.46f, 0.44f, 0.48f},
        {0.96f, 0.88f, 0.72f},
        {0.72f, 0.78f, 0.88f},
    };
    const int toneIndex = std::clamp(static_cast<int>(skinTone + 0.5f), 0, 4);
    const int coatIndex = std::clamp(static_cast<int>(furCoat + 0.5f), 0, 5);
    const std::uint32_t count = character_->instanceCount(character_->self, characterHandle_);
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
        if (isSkinPart(partId))
        {
            const glm::vec3 &tint = toneTints[toneIndex];
            material.baseColor[0] *= tint.r;
            material.baseColor[1] *= tint.g;
            material.baseColor[2] *= tint.b;
        }
        else if (isFurPart(partId))
        {
            const glm::vec3 &tint = coatTints[coatIndex];
            material.baseColor[0] *= tint.r;
            material.baseColor[1] *= tint.g;
            material.baseColor[2] *= tint.b;
        }
        render_->drawMesh(render_->self, rendererHandle_, mesh, matrix, &material);
    }
}

// ---- UI ---------------------------------------------------------------------

} // namespace demo
