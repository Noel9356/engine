#include "character_stage.hpp"

#include "geometry.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cstdio>
#include <utility>

namespace demo
{

void CharacterStage::create(
    const engine::renderer::RenderCapability &renderer, engine::renderer::RendererHandle rendererHandle)
{
    destroy();
    renderer_       = &renderer;
    rendererHandle_ = rendererHandle;

    // The inward-facing sky dome is authored once and reused across frames;
    // the floor and sun are independent stage resources, not screen state.
    const MeshData floorGeometry = makeBoxMesh(
        glm::vec3(0.0f, -0.06f, 0.0f), glm::vec3(6.0f, 0.06f, 6.0f), glm::vec3(0.42f, 0.44f, 0.48f), 4.0f);
    MeshData skyGeometry = makeSphereMesh(
        glm::vec3(0.0f), 900.0f, 24, 12, glm::vec3(0.55f, 0.64f, 0.78f), glm::vec3(0.24f, 0.38f, 0.62f));
    for (std::size_t index = 0; index + 2 < skyGeometry.indices.size(); index += 3)
    {
        std::swap(skyGeometry.indices[index], skyGeometry.indices[index + 1]);
        Vertex &vertex = skyGeometry.vertices[skyGeometry.indices[index]];
        vertex.normal *= -1.0f;
    }
    const MeshData sunGeometry = makeSphereMesh(
        glm::vec3(280.0f, 350.0f, -640.0f), 16.0f, 14, 8, glm::vec3(4.0f, 3.6f, 3.0f), glm::vec3(4.0f, 3.6f, 3.0f));
    // The contact shadow: a gradient sphere (floor colour at the equator,
    // dark at the zenith) flattened into a disc at draw time - the vertex
    // blend becomes the radial falloff.
    const MeshData contactShadowGeometry
        = makeSphereMesh(glm::vec3(0.0f), 1.0f, 28, 10, glm::vec3(0.42f, 0.44f, 0.48f), glm::vec3(0.20f, 0.21f, 0.23f));

    struct StageUpload
    {
        const MeshData               *geometry;
        engine::renderer::MeshHandle *target;
    };
    const StageUpload uploads[] = {
        {&floorGeometry, &floorMesh_},
        {&skyGeometry, &skyDomeMesh_},
        {&sunGeometry, &sunDiscMesh_},
        {&contactShadowGeometry, &contactShadowMesh_},
    };
    for (const StageUpload &upload : uploads)
    {
        const engine::renderer::MeshDescription description = upload.geometry->description();
        engine::renderer::RenderError           error       = engine::renderer::RenderError::None;
        *upload.target = renderer_->createMesh(renderer_->self, rendererHandle_, &description, &error);
        if (*upload.target == engine::renderer::invalidMesh)
        {
            std::fprintf(stderr, "demo: character stage mesh upload failed (%u)\n", static_cast<unsigned>(error));
        }
    }
}

void CharacterStage::destroy()
{
    if (renderer_ != nullptr && rendererHandle_ != engine::renderer::invalidRenderer)
    {
        if (floorMesh_ != engine::renderer::invalidMesh)
            renderer_->destroyMesh(renderer_->self, rendererHandle_, floorMesh_);
        if (skyDomeMesh_ != engine::renderer::invalidMesh)
            renderer_->destroyMesh(renderer_->self, rendererHandle_, skyDomeMesh_);
        if (sunDiscMesh_ != engine::renderer::invalidMesh)
            renderer_->destroyMesh(renderer_->self, rendererHandle_, sunDiscMesh_);
        if (contactShadowMesh_ != engine::renderer::invalidMesh)
            renderer_->destroyMesh(renderer_->self, rendererHandle_, contactShadowMesh_);
    }
    floorMesh_         = engine::renderer::invalidMesh;
    skyDomeMesh_       = engine::renderer::invalidMesh;
    sunDiscMesh_       = engine::renderer::invalidMesh;
    contactShadowMesh_ = engine::renderer::invalidMesh;
    renderer_          = nullptr;
    rendererHandle_    = engine::renderer::invalidRenderer;
}

void CharacterStage::drawFloor() const
{
    static const engine::renderer::MeshMaterial floorMaterial{
        .baseColor                = {1.0f, 1.0f, 1.0f},
        .baseColorTexture         = engine::renderer::invalidTexture,
        .metallicFactor           = 0.0f,
        .roughnessFactor          = 0.92f,
        .metallicRoughnessTexture = engine::renderer::invalidTexture,
        .normalTexture            = engine::renderer::invalidTexture,
        .normalScale              = 1.0f,
        .occlusionTexture         = engine::renderer::invalidTexture,
        .occlusionStrength        = 1.0f,
        .emissiveColor            = {0.0f, 0.0f, 0.0f},
        .emissiveTexture          = engine::renderer::invalidTexture,
        .unlit                    = false,
    };
    if (renderer_ != nullptr && floorMesh_ != engine::renderer::invalidMesh)
    {
        const glm::mat4 identity(1.0f);
        renderer_->drawMesh(renderer_->self, rendererHandle_, floorMesh_, glm::value_ptr(identity), &floorMaterial);
    }
}

void CharacterStage::drawSkyAndSun() const
{
    static const engine::renderer::MeshMaterial skyMaterial{
        .baseColor                = {1.0f, 1.0f, 1.0f},
        .baseColorTexture         = engine::renderer::invalidTexture,
        .metallicFactor           = 0.0f,
        .roughnessFactor          = 1.0f,
        .metallicRoughnessTexture = engine::renderer::invalidTexture,
        .normalTexture            = engine::renderer::invalidTexture,
        .normalScale              = 1.0f,
        .occlusionTexture         = engine::renderer::invalidTexture,
        .occlusionStrength        = 1.0f,
        .emissiveColor            = {0.0f, 0.0f, 0.0f},
        .emissiveTexture          = engine::renderer::invalidTexture,
        .unlit                    = true,
    };
    static const engine::renderer::MeshMaterial sunMaterial{
        .baseColor                = {4.0f, 3.6f, 3.0f},
        .baseColorTexture         = engine::renderer::invalidTexture,
        .metallicFactor           = 0.0f,
        .roughnessFactor          = 1.0f,
        .metallicRoughnessTexture = engine::renderer::invalidTexture,
        .normalTexture            = engine::renderer::invalidTexture,
        .normalScale              = 1.0f,
        .occlusionTexture         = engine::renderer::invalidTexture,
        .occlusionStrength        = 1.0f,
        .emissiveColor            = {0.0f, 0.0f, 0.0f},
        .emissiveTexture          = engine::renderer::invalidTexture,
        .unlit                    = true,
    };

    if (renderer_ == nullptr)
        return;
    const glm::mat4 identity(1.0f);
    if (skyDomeMesh_ != engine::renderer::invalidMesh)
        renderer_->drawMesh(renderer_->self, rendererHandle_, skyDomeMesh_, glm::value_ptr(identity), &skyMaterial);
    if (sunDiscMesh_ != engine::renderer::invalidMesh)
        renderer_->drawMesh(renderer_->self, rendererHandle_, sunDiscMesh_, glm::value_ptr(identity), &sunMaterial);
}

void CharacterStage::drawContactShadow(float centerX, float centerZ, float radius) const
{
    static const engine::renderer::MeshMaterial shadowMaterial{
        .baseColor                = {1.0f, 1.0f, 1.0f},
        .baseColorTexture         = engine::renderer::invalidTexture,
        .metallicFactor           = 0.0f,
        .roughnessFactor          = 1.0f,
        .metallicRoughnessTexture = engine::renderer::invalidTexture,
        .normalTexture            = engine::renderer::invalidTexture,
        .normalScale              = 1.0f,
        .occlusionTexture         = engine::renderer::invalidTexture,
        .occlusionStrength        = 1.0f,
        .emissiveColor            = {0.0f, 0.0f, 0.0f},
        .emissiveTexture          = engine::renderer::invalidTexture,
        .unlit                    = true,
    };
    if (renderer_ == nullptr || contactShadowMesh_ == engine::renderer::invalidMesh || radius <= 0.02f)
    {
        return;
    }
    // Flatten the gradient sphere into the ground disc: the rim keeps the
    // floor's own colour so the disc dissolves into the tiles, the centre
    // darkens. The disc rides a few millimetres above the floor plane so the
    // depth test resolves it cleanly.
    const glm::vec3 scale(radius, std::clamp(radius * 0.03f, 0.006f, 0.03f), radius);
    const glm::mat4 transform
        = glm::translate(glm::mat4(1.0f), glm::vec3(centerX, 0.004f, centerZ)) * glm::scale(glm::mat4(1.0f), scale);
    renderer_->drawMesh(
        renderer_->self, rendererHandle_, contactShadowMesh_, glm::value_ptr(transform), &shadowMaterial);
}

} // namespace demo
