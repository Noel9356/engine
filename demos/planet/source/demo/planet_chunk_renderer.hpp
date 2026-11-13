// Renderer-side cache for the planet module's currently visible chunk meshes.
// The planet module owns CPU chunk data; this class owns only GPU mesh handles
// and the stable per-draw material/transform storage used by the renderer.
#ifndef ENGINE_DEMO_PLANET_CHUNK_RENDERER_HPP
#define ENGINE_DEMO_PLANET_CHUNK_RENDERER_HPP

#include <engine/planet/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>

#include <glm/glm.hpp>

#include <cstdint>
#include <deque>
#include <map>
#include <utility>
#include <vector>

namespace demo::planet_demo
{

class PlanetChunkRenderer final
{
public:
    PlanetChunkRenderer(engine::planet::PlanetCapability &planetCapability, engine::planet::PlanetHandle planetHandle,
        engine::renderer::RenderCapability &renderCapability, engine::renderer::RendererHandle rendererHandle,
        const engine::planet::PlanetGenerationParams &generationParams);
    ~PlanetChunkRenderer();

    PlanetChunkRenderer(const PlanetChunkRenderer &) = delete;
    PlanetChunkRenderer &operator=(const PlanetChunkRenderer &) = delete;

    void updateVisibleChunks();
    void rebuildDrawCommands();
    void destroy();

    [[nodiscard]] const std::vector<engine::renderer::RenderDrawCommand> &drawCommands() const noexcept
    {
        return drawCommands_;
    }

private:
    struct ChunkMeshHandles
    {
        engine::renderer::MeshHandle terrain = engine::renderer::invalidMesh;
        engine::renderer::MeshHandle water = engine::renderer::invalidMesh;
        glm::mat4 terrainTransform{1.0f};
        glm::mat4 waterTransform{1.0f};
        std::uint64_t lastSeenUpdate = 0; // update that last listed this chunk as visible
    };

    void destroyChunkMeshes(ChunkMeshHandles &handles);

    engine::planet::PlanetCapability *planet_ = nullptr;
    engine::planet::PlanetHandle planetHandle_ = engine::planet::invalidPlanet;
    engine::renderer::RenderCapability *renderer_ = nullptr;
    engine::renderer::RendererHandle rendererHandle_ = engine::renderer::invalidRenderer;
    engine::planet::PlanetGenerationParams params_{};

    std::map<engine::planet::ChunkKey, ChunkMeshHandles> uploadedMeshes_;
    std::vector<engine::planet::PlanetChunkView> visibleViews_; // reused every update
    std::uint64_t updateCounter_ = 0;
    // Draw commands retain pointers to materials until endFrame. Deques keep
    // references stable while the command list is being assembled.
    std::deque<engine::renderer::MeshMaterial> terrainMaterials_;
    std::deque<engine::renderer::MeshMaterial> waterMaterials_;
    std::vector<engine::renderer::RenderDrawCommand> drawCommands_;
};

} // namespace demo::planet_demo

#endif
