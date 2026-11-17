// Renderer-side cache for the planet module's currently visible chunk meshes.
// The planet module owns CPU chunk data; this class owns only GPU mesh handles
// and the stable per-draw material/transform storage used by the renderer.
#pragma once

#include <engine/planet/atmosphere.hpp>
#include <engine/planet/capabilities.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/material.hpp>

#include <glm/glm.hpp>

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
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
    // Assembles the frame's draw commands: backdrop shells first (sky dome,
    // sky cloud deck, space-only atmosphere limb), then the world-anchored
    // cloud shell, then chunk terrain/water. viewProjection frustum-culls the
    // chunk draws (the planet's own selection spans the whole visible disk,
    // including the half behind the camera); eyeAltitudeMeters scales the
    // water material's roughness (a tight sun glint from orbit, the
    // configured sea texture close up).
    void rebuildDrawCommands(const glm::mat4 &viewProjection, double eyeAltitudeMeters);
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

    // Cloud layer (see engine/planet/atmosphere.hpp CloudField): a static
    // shell above the surface meshed once per run from the module's coverage
    // field - quads whose corners all read clear are skipped, so the layer is
    // geometry with organic cloud-shaped edges. Vertices carry the coverage-
    // shaded cloud color and radial normals, and the draw is LIT, so the sun
    // shades the banks (bright tops on the day side, dim banks at the limb).
    // World-anchored: macro (globe/orbit) cameras see the swirling layer
    // against the surface from outside the shell.
    void ensureCloudLayer();
    // The shell is built for cameras OUTSIDE it: a camera between the
    // surface and the shell would see the far side of the sphere through the
    // sky above the horizon (the demo renders terrain two-sided, so the
    // inside-out quads are not culled), so ground views hide the layer.
    void setCloudLayerVisible(bool visible);

    // Atmosphere limb (space views): a hemisphere around the planet at the
    // top of the atmosphere, drawn unlit with depth writes off ahead of
    // everything else. Its vertex gradient runs from the horizon color down
    // to black at the rim, so the ring between the terrain silhouette and
    // the shell's own silhouette reads as glowing air instead of a hard
    // terrain edge on black; the opaque terrain/cloud draws overwrite the
    // disk interior. Rotated toward the eye per frame, active only when the
    // eye is outside the atmosphere (the sky dome owns in-air views).
    void ensureAtmosphereShell();
    void updateAtmosphereShell(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &sunDirection, bool sunEnabled);

    // Sky cloud deck (ground views): the same contoured coverage mesh as the
    // world-anchored shell, but built on the unit ball in the sky dome's
    // frame and drawn unlit ahead of the terrain, so standing cameras see
    // cloud banks overhead instead of a bare gradient. The dome's per-frame
    // update rotates, scales and tints it with the sky (see updateSkyDome).
    void ensureSkyCloudLayer(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &sunDirection);
    void setSkyCloudLayerVisible(bool visible);

    // Star dome (space views): a dense unit sphere whose vertex colors sample
    // the module's star field (see atmosphere.hpp starFieldColor), drawn
    // unlit with depth writes off behind everything else. Active only when
    // the atmosphere weight has faded (the sky dome owns in-air views and
    // covers it completely); the per-frame transform follows the eye.
    void ensureStarDome();
    void updateStarDome(const glm::dvec3 &eyePlanetLocal, double farZ);

    // Sun disc (backdrop): a bright disc with a soft halo placed along the
    // sun direction at the backdrop distance, tinted by the atmosphere
    // model's sun tint (gold at low sun, dark at night) and the configured
    // sun color. Drawn after the sky dome and star dome but before the cloud
    // deck, so banks occlude the sun; depth writes off like every backdrop.
    void ensureSunDisc();
    void updateSunDisc(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &up, const glm::dvec3 &sunDirection,
        const float sunColor[3], bool sunEnabled, double farZ);

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
    void destroyCloudLayer();
    void destroyAtmosphereShell();
    void destroySkyCloudLayer();
    void destroyStarDome();
    void destroySunDisc();

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

    // Cloud layer: static shell mesh from the module's cloud coverage field
    // (built once; world-anchored, so it never follows the camera).
    engine::renderer::MeshHandle cloudLayer_ = engine::renderer::invalidMesh;
    glm::mat4 cloudLayerTransform_{1.0f};
    bool cloudLayerActive_ = false;
    bool cloudLayerVisible_ = false;
    std::deque<engine::renderer::MeshMaterial> cloudLayerMaterials_;
    std::unique_ptr<engine::planet::CloudField> cloudField_;

    // Atmosphere limb hemisphere (space backdrop; see ensureAtmosphereShell).
    engine::renderer::MeshHandle atmosphereShell_ = engine::renderer::invalidMesh;
    glm::mat4 atmosphereShellTransform_{1.0f};
    bool atmosphereShellActive_ = false;
    std::deque<engine::renderer::MeshMaterial> atmosphereShellMaterials_;

    // Sky cloud deck (ground backdrop; see ensureSkyCloudLayer): unit-ball
    // mesh, placed and rotated by the sky dome update every frame.
    engine::renderer::MeshHandle skyCloudLayer_ = engine::renderer::invalidMesh;
    glm::mat4 skyCloudTransform_{1.0f};
    bool skyCloudActive_ = false;
    bool skyCloudVisible_ = false;
    std::deque<engine::renderer::MeshMaterial> skyCloudMaterials_;

    // Star dome (space backdrop; see ensureStarDome).
    engine::renderer::MeshHandle starDome_ = engine::renderer::invalidMesh;
    glm::mat4 starDomeTransform_{1.0f};
    bool starDomeActive_ = false;
    std::deque<engine::renderer::MeshMaterial> starDomeMaterials_;

    // Sun disc (backdrop): a small bright disc with a soft halo, placed
    // along the sun direction at the backdrop distance and tinted by the
    // atmosphere model's sun tint (gold at low sun, neutral at noon, dark at
    // night). Drawn after the sky dome (and star dome) but before the cloud
    // deck, so banks occlude the sun.
    engine::renderer::MeshHandle sunDisc_ = engine::renderer::invalidMesh;
    glm::mat4 sunDiscTransform_{1.0f};
    bool sunDiscActive_ = false;
    std::deque<engine::renderer::MeshMaterial> sunDiscMaterials_;
    double sunDiscAngularRadiusRadians_ = 0.0;
};

} // namespace demo::planet_demo

