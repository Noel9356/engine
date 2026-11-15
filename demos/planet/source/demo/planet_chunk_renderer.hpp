// Renderer-side cache for the planet module's currently visible chunk meshes.
// The planet module owns CPU chunk data; this class owns only GPU mesh handles
// and the stable per-draw material/transform storage used by the renderer.
#pragma once

#include <engine/planet/capabilities.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/material.hpp>

#include <glm/glm.hpp>

#include <cstdint>
#include <deque>
#include <map>
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

    // Sky backdrop (see engine/planet/atmosphere.hpp): a static gradient
    // sphere on the unit ball, kept centered on the eye and scaled per frame
    // so its rim always sits beyond the farthest terrain the eye can see but
    // inside the far plane. Drawn first, unlit (vertex colors only), so it
    // never shades or occludes - it only fills the pixels the terrain does
    // not reach: the sky itself, plus any chunk still streaming in.
    void ensureSkyDome();
    // Places and tints the dome for this frame's eye. eyePlanetLocal is the
    // eye in planet-local space, up the local gravity gradient, sunDirection
    // the pbr sun's direction. Reports the horizon-haze color for the frame
    // clear color and returns false when the sky is off (atmosphere disabled,
    // eye outside the atmosphere), leaving the clear color at black.
    bool updateSkyDome(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &up, const glm::dvec3 &sunDirection,
        bool sunEnabled, double nearZ, double farZ, float outClearColor[3]);

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
        std::uint64_t lastSeenUpdate = 0;  // update that last listed this chunk as visible
        std::uint64_t terrainHash = 0;     // content hash of the uploaded terrain mesh
        std::uint64_t waterHash = 0;       // content hash of the uploaded water mesh
    };

    void destroyChunkMeshes(ChunkMeshHandles &handles);
    void destroySkyDome();

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

    // Sky dome: unit-radius gradient sphere (positions on the unit ball,
    // colors from the atmosphere model's up-gradient) shared by every frame;
    // the per-frame transform places, orients and scales it.
    engine::renderer::MeshHandle skyDome_ = engine::renderer::invalidMesh;
    glm::mat4 skyDomeTransform_{1.0f};
    bool skyDomeActive_ = false;
    std::deque<engine::renderer::MeshMaterial> skyDomeMaterials_;
};

} // namespace demo::planet_demo

