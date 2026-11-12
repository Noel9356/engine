// Renderer-owned studio backdrop for the character customisation screen.
// Keeping floor/sky/sun resources and draw policy here prevents the screen
// controller from also owning static stage geometry.
#ifndef ENGINE_DEMO_CHARACTER_STAGE_HPP
#define ENGINE_DEMO_CHARACTER_STAGE_HPP

#include <engine/renderer/capabilities.hpp>

namespace demo
{

class CharacterStage final
{
public:
    void create(const engine::renderer::RenderCapability &renderer, engine::renderer::RendererHandle rendererHandle);
    void destroy();

    void drawFloor() const;
    void drawSkyAndSun() const;

private:
    const engine::renderer::RenderCapability *renderer_ = nullptr;
    engine::renderer::RendererHandle rendererHandle_ = engine::renderer::invalidRenderer;
    engine::renderer::MeshHandle floorMesh_ = engine::renderer::invalidMesh;
    engine::renderer::MeshHandle skyDomeMesh_ = engine::renderer::invalidMesh;
    engine::renderer::MeshHandle sunDiscMesh_ = engine::renderer::invalidMesh;
};

} // namespace demo

#endif
