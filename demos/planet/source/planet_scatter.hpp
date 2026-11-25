// Surface scatter system for the planet demo: the ground-level real-world
// elements the terrain's color field cannot express - vegetation (trees,
// bushes, grass tufts), scattered surface rocks, bird flocks, breaking-wave
// foam (whitecaps and shore surf bands), valley mist and the sun-aligned
// contact shadows that tie the scattered geometry to the ground.
//
// The planet module owns the density rules (which climate state carries which
// element and how much; see the surface-scatter block in PlanetGenerationParams);
// this system scatters the demo's own geometry against them. Placement is a
// pure function of (planet seed, world-space cell, planet radius): the trees,
// bushes, grass and rocks of a fixed cell evaluate identically on every
// rebuild, so the world does not reshuffle as the eye moves - cells only pop
// in at the annulus boundary. The transient sea-state elements (whitecaps,
// surf, mist) re-roll on a short interval instead, which is exactly the churn
// real breaking foam shows.
//
// The foliage sprites are DOWNLOADED photographic cutouts (Poly Haven, CC0;
// see assets/textures/vegetation): one 2x2 atlas per group - four tree species
// (broadleaf, acacia, conifer, dry broadleaf), four bush species (green
// shrub, dry shrub, fern, scrub) and four grass cards (lush tuft, dry tuft,
// low sprouts, sparse dry). Every instance picks its species from its site's
// biome, so different climates grow visibly different flora. When the atlas
// files are missing the system falls back to the seed-generated procedural
// sprites so the demo still runs.
//
// Everything renders through the demo's canonical PlanetVertex pipeline: one
// mesh per element group, alpha-cutout billboards for foliage (the material's
// alphaCutoff masks the sprite textures), unlit translucent sprites for foam
// and mist, and a per-frame vertex-updated mesh for the birds.
//
// Frame-cost discipline: the rebuild is deterministic and bounded (candidate
// counts derive from the configured radii, not from the visible chunk set),
// the bird mesh is the only per-frame geometry work (a few hundred vertices),
// and space views skip the system entirely (nothing on the ground is visible
// from orbit that these elements express).
#pragma once

#include <engine/planet/capabilities.hpp>
#include <engine/planet/density.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/mesh.hpp>

#include <glm/glm.hpp>

#include <filesystem>
#include <vector>

namespace engine::planet
{
struct PlanetCapability;
struct ClimateSample;
}

namespace demo::planet_demo
{

class PlanetScatterSystem
{
public:
    PlanetScatterSystem(engine::planet::PlanetCapability &planetCapability, engine::planet::PlanetHandle planet,
        const engine::planet::PlanetGenerationParams &params);

    // Binds the renderer and loads the vegetation atlases (assetDirectory =
    // the demo's assets/textures folder; the vegetation/ subfolder holds the
    // photographic species atlases). Safe to call again; a second call is a
    // no-op. When the renderer cannot host the system (no capability, failed
    // texture or mesh creation) every update degrades to a no-op and
    // drawCommands() stays empty.
    void initialize(engine::renderer::RenderCapability &renderCapability, engine::renderer::RendererHandle rendererHandle,
        const std::filesystem::path &assetDirectory);
    void shutdown();

    // Per-frame update. eyePlanetLocal is the eye in planet-local meters;
    // sunDirection the scene's sun direction (unit length, world-oriented);
    // sunEnabled gates the day-only elements (birds roost at night); the
    // deltaSeconds window advances the sea-state churn and the flock flight.
    void update(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &sunDirection, bool sunEnabled, double deltaSeconds);

    // Draw commands referencing this system's meshes (materials point into
    // this system's pools; valid until the next update). Opaque commands
    // first, transparent last - the backend routes each draw by its material
    // regardless of position, the same contract the aurora rides. The anchors
    // run parallel to the commands (the renderer composes the shifted model
    // matrices from them).
    const std::vector<engine::renderer::RenderDrawCommand> &drawCommands() const { return drawCommands_; }
    const std::vector<glm::dvec3> &commandAnchors() const { return commandAnchors_; }

    // True while the eye sits inside the atmosphere's lower band, where the
    // ground-level elements are visible (space views skip the whole system).
    bool groundLevel() const { return groundLevel_; }

private:
    // One drawable element group: a mesh in eye-anchored planet-local
    // coordinates (the model matrix carries the anchor), its material, and
    // the flip that marks a rebuild as vertex-updated (bird mesh) instead of
    // re-uploaded (everything else).
    struct ScatterMesh
    {
        engine::renderer::MeshHandle mesh = engine::renderer::invalidMesh;
        engine::renderer::MeshMaterial material{};
        glm::dvec3 anchorPlanetLocal{0.0};
        bool vertexUpdated = false; // refresh via updateMeshVertices, not re-create
        bool active = false;
    };

    // Deterministic hash stream: SplitMix64 over (seed, salt, cell, jitter
    // lane) - every placement decision keys off the world-space cell, never
    // off the rebuild iteration, so cells do not reshuffle between rebuilds.
    // Public: the placement walk (a free template) keys candidates with it.
public:
    static std::uint64_t cellHash(std::uint64_t seed, std::uint64_t salt, std::int64_t x, std::int64_t y, std::int64_t z);
    static double hashUnit(std::uint64_t hash, std::uint64_t lane); // [0, 1)

private:

    // Foliage sprite textures. The PRIMARY source is the downloaded
    // photographic atlases (tree/bush/grass, 2x2 species tiles each; see
    // assets/textures/vegetation) loaded from the asset directory; the
    // seed-generated procedural sprites remain as the fallback when those
    // files are missing, so the demo degrades instead of vanishing.
    // foam/mist/shadow sprites stay procedural in both cases (photographic
    // foam banks exist but the blobs read identically at scatter scale).
    void loadVegetationAtlases(const std::filesystem::path &assetDirectory);
    void buildSpriteTextures(); // fallback: seed-generated foliage sprites
    void buildSeaStateSprites(); // foam/mist/shadow, procedural in every configuration

    // Replicates one fallback sprite into the 2x2 tile grid the scatter's
    // species UVs address (the photographic atlases ship that layout).
    engine::renderer::TextureHandle createSpriteAtlas(const std::vector<std::uint8_t> &sprite,
        std::uint32_t spriteSize);

    // Rebuilds (re-scatters) every stationary element group for a new eye
    // anchor, and refreshes the transient sea-state groups.
    void rebuild(const glm::dvec3 &eyePlanetLocal);
    void scatterVegetation(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &eyeDirection,
        const glm::dvec3 &east, const glm::dvec3 &north);
    void scatterRocks(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &eyeDirection,
        const glm::dvec3 &east, const glm::dvec3 &north);

    // One sun-aligned contact shadow (a soft dark quad laid tangent to the
    // ground under a scattered element, stretched along the away-from-sun
    // direction by the site's sun elevation). Appends into the shared shadow
    // scratch the rebuild uploads as one mesh; a no-op when the strength is
    // zero, the sun is at/below the horizon, or the textures are missing.
    void appendShadow(const glm::dvec3 &groundPoint, const glm::dvec3 &up, double sizeMeters, double heightMeters);
    void scatterWhitecaps(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &eyeDirection,
        const glm::dvec3 &east, const glm::dvec3 &north, bool partial);
    void scatterSurf(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &eyeDirection,
        const glm::dvec3 &east, const glm::dvec3 &north);
    void scatterMist(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &eyeDirection,
        const glm::dvec3 &east, const glm::dvec3 &north, double sunUp);
    void scatterBirdFlocks(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &eyeDirection,
        const glm::dvec3 &east, const glm::dvec3 &north);

    // Mesh builders. Quad/sprite writers append into shared buffers, then
    // upload (or update) the mesh. anchorPlanetLocal is the eye at rebuild;
    // every vertex is stored eye-relative so the f32 pipeline sees small
    // coordinates exactly like the streamed chunk meshes do. doubleSided
    // keeps both faces shaded AND flips the shading normal toward the eye on
    // back-facing fragments - right for unlit sea/sky sprites (the normal is
    // unused) and wrong for lit foliage, whose lighting normal is
    // deliberately the ground's radial up: the flip would point it into the
    // ground the moment a fragment rises above the eye's tangent plane and
    // drop the draw to ambient-only black (the first-person walk view reads
    // exactly that on up-slope tufts). Lit scatter groups pass false - the
    // camera always sits inside those meshes' bounds, so the backend's
    // majority-vote back-face culling cannot fire either.
    void uploadMesh(ScatterMesh &slot, std::vector<engine::planet::PlanetVertex> &vertices,
        std::vector<std::uint32_t> &indices, bool alphaCutout, bool unlit, bool transparent, float opacity,
        bool doubleSided = true);

    // Surface probe along a unit planet-local direction (the capability's
    // sampleSurfaceRadius, wrapped so every gate reads as one line).
    bool sampleSurfaceRadius(const glm::dvec3 &direction, double &outRadius) const;

    // Per-frame bird animation: advance the flock paths and rewrite the bird
    // mesh's vertices (fixed count) through updateMeshVertices.
    void animateBirds(double deltaSeconds);

    // Rewrites the bird mesh's vertices from the flocks' current flight state
    // (fixed vertex count; the updateMeshVertices path refreshes in place).
    void appendBirdGeometry(std::vector<engine::planet::PlanetVertex> &vertices, std::vector<std::uint32_t> &indices);

    // Composes the draw command list from the active meshes.
    void composeDrawCommands();

    engine::planet::PlanetCapability &capability_;
    engine::planet::PlanetHandle planet_ = 0;
    const engine::planet::PlanetGenerationParams &params_;
    // The module's density field instance used ONLY for cloud-shadow
    // sampling: every scattered element shades with the same shadow field
    // the terrain's vertex albedo carries, so a cloud bank darkens the
    // ground AND the plants standing on it together (an unshaded sprite
    // glows like a sticker against its own shaded ground). Pure field
    // sampling; no streaming state.
    engine::planet::DensityField shadowField_;

    engine::renderer::RenderCapability *renderCapability_ = nullptr;
    engine::renderer::RendererHandle rendererHandle_ = 0;

    engine::renderer::TextureHandle treeTexture_ = engine::renderer::invalidTexture;
    engine::renderer::TextureHandle bushTexture_ = engine::renderer::invalidTexture;
    engine::renderer::TextureHandle grassTexture_ = engine::renderer::invalidTexture;
    engine::renderer::TextureHandle foamTexture_ = engine::renderer::invalidTexture;
    engine::renderer::TextureHandle mistTexture_ = engine::renderer::invalidTexture;
    engine::renderer::TextureHandle shadowTexture_ = engine::renderer::invalidTexture;

    ScatterMesh treeMesh_;
    ScatterMesh bushMesh_;
    ScatterMesh grassMesh_;
    ScatterMesh rockMesh_;
    ScatterMesh shadowMesh_;
    ScatterMesh whitecapMesh_;
    ScatterMesh surfMesh_;
    ScatterMesh mistMesh_;
    ScatterMesh birdMesh_;

    // The contact-shadow scratch the rebuild fills through appendShadow and
    // uploads as one mesh at the end (cleared per rebuild, not per frame).
    std::vector<engine::planet::PlanetVertex> shadowScratch_;
    std::vector<std::uint32_t> shadowIndexScratch_;

    std::vector<engine::renderer::RenderDrawCommand> drawCommands_;
    // Per-command planet-local anchor the renderer composes the shifted model
    // matrix from (the f64 eye-relative shift keeps planet-scale precision);
    // parallel to drawCommands_.
    std::vector<glm::dvec3> commandAnchors_;

    // One foam slot: bound to its lattice cell at the anchor rebuild, alive
    // or dead by a staggered quarter-of-the-pool churn (see scatterWhitecaps).
    struct WhitecapSlot
    {
        glm::dvec3 direction{0.0};   // unit planet-local direction of the patch
        double halfLength = 0.0;
        double halfWidth = 0.0;
        std::uint32_t slotIndex = 0;
        bool alive = false;
    };

    // Bird flocks (flight state advanced every frame; geometry rewritten from
    // it). Empty until the first ground-level rebuild places them.
    struct BirdFlock
    {
        glm::dvec3 centerPlanetLocal{0.0}; // path center
        double pathRadiusMeters = 100.0;
        double angularPhase = 0.0;         // current angle along the path
        double angularSpeed = 0.15;        // radians per second
        double flightHeightMeters = 40.0;  // above the ground under the center
        std::uint32_t birdCount = 9;
        double flapHertz = 3.0;
        double flapPhase = 0.0;
        double birdScale = 1.0;
    };
    std::vector<BirdFlock> flocks_;
    std::vector<WhitecapSlot> whitecapSlots_;

    glm::dvec3 anchorEye_{0.0};      // the rebuild anchor (planet-local)
    double anchorEyeRadius_ = 0.0;   // its radial distance from the planet center
    bool hasAnchor_ = false;         // any rebuild happened yet
    glm::dvec3 lastSunDirection_{0.0, 1.0, 0.0}; // the scene sun at the last update
    double refreshAccumulator_ = 0.0;
    std::uint32_t refreshCounter_ = 0;
    bool groundLevel_ = false;
    bool initialized_ = false;
    bool rendererBound_ = false;
};

} // namespace demo::planet_demo
