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
// Re-anchor cost model: each stationary family (trees, bushes, grass) keeps
// its OWN anchor and a per-cell placement cache, and at most one family
// re-anchors per update. A re-anchor only evaluates the cells that entered
// the ring (the leading annulus); every cached cell re-emits as pure
// arithmetic with the distance-driven terms (ground fade, aerial haze,
// contact-shadow ribbons) replayed against the new anchor, so a moving eye
// pays a small bounded cost per ring shift instead of a full re-scatter.
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
// one family re-anchors per update at most, the caches keep every re-anchor
// to its leading annulus, the bird mesh is the only per-frame geometry work
// (a few hundred vertices), and space views skip the system entirely
// (nothing on the ground is visible from orbit that these elements express).
#pragma once

#include <engine/planet/capabilities.hpp>
#include <engine/planet/density.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/mesh.hpp>

#include <glm/glm.hpp>

#include <array>
#include <cstdint>
#include <filesystem>
#include <unordered_map>
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
    // viewForwardPlanetLocal is the camera's sight axis (eye-to-target, unit
    // length; a zero vector means "unknown"): the near-ring population
    // guarantee ranks the cells it plants by falling inside the camera's
    // forward hemisphere first, so the guaranteed landmark vegetation lands
    // on screen instead of behind the walker.
    void update(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &viewForwardPlanetLocal,
        const glm::dvec3 &sunDirection, bool sunEnabled, double deltaSeconds);

    // Drops every family anchor so the next update re-scatters around the
    // current eye (the in-app view switch lands the eye somewhere new; the
    // per-cell caches keep the world-keyed placements identical wherever the
    // rings still overlap).
    void forceRebuild();

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
        // The stream sizes the live mesh was created with. The in-place
        // vertex refresh only covers a mesh whose vertex AND index counts
        // still match (the renderer rejects a partial or oversized write),
        // so a rebuild whose population changed recreates the mesh instead -
        // a count-mismatched refresh would leave the previous geometry
        // behind while the anchor moves on, sliding the whole stale mesh
        // with the eye.
        std::uint32_t uploadedVertexCount = 0;
        std::uint32_t uploadedIndexCount = 0;
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

    // ---- placement caches -------------------------------------------------
    //
    // A cell's placement (site, size, tint, shadow ribbon) is a pure function
    // of the cell and the planet, so every fully-evaluated card is cached by
    // cell identity and re-emitted as arithmetic at any anchor. The
    // distance-driven terms (the foliage ground fade, the aerial haze, the
    // shadow's anchor-relative lift) apply at EMIT time against the anchor
    // the mesh is being baked for, so a re-anchored family reproduces exactly
    // what a fresh walk at that anchor would show for its cached cells.

    // Identity of one lattice cell: the planet-centered cell coordinates plus
    // the sub-lattice (the grass family walks a near disc and a mid annulus
    // on different cell sizes; the sub index keeps their integer coordinates
    // from colliding).
    struct CellKey
    {
        std::array<std::int64_t, 4> value{};
        bool operator==(const CellKey &other) const noexcept { return value == other.value; }
    };
    struct CellKeyHash
    {
        std::size_t operator()(const CellKey &key) const noexcept;
    };

    // The contact shadow under one card: a ribbon of world-space quads draped
    // along the away-from-sun direction, baked once per placement (the ground
    // march is the expensive part; the sun direction is carried along so a
    // changed sun drops the ribbon instead of replaying it stale). strips == 0
    // means the placement casts no readable shadow.
    struct ShadowRibbon
    {
        glm::dvec3 corners[4][4]{}; // up to four strips x four world corners
        glm::dvec3 normals[4]{};
        float uv[4][4][2]{};
        glm::dvec3 sun{0.0}; // the direction the ribbon was baked against
        std::uint32_t strips = 0;
    };

    // One fully-evaluated billboard (a tree, a companion tree, a bush, or one
    // card of a grass tuft's crossed pair). fixedRight marks the families
    // that orient the card by an explicit in-plane right vector (the grass
    // crossed cards) instead of spherical billboarding toward the anchor.
    struct PlacedCard
    {
        glm::dvec3 base{0.0};          // planet-local card base (burial applied)
        glm::dvec3 up{0.0, 1.0, 0.0};  // the site's radial up
        glm::dvec3 right{0.0};         // explicit card right (fixedRight only)
        double halfWidth = 0.0;
        double height = 0.0;
        std::array<float, 3> speciesColor{}; // tint x cloud shade (pre ground fade, pre haze)
        std::array<float, 3> groundColor{};  // the ground-fade target (site albedo x 0.9)
        std::uint32_t uvTile = 0;
        bool mirrorU = false;
        bool fixedRight = false;
        float rootDarken = 0.0f;
        ShadowRibbon shadowRibbon; // baked contact shadow (see above)
    };

    // The per-family placement cache: one entry per planted cell, holding the
    // cell's fully-evaluated cards (a tree plus its companion, a bush, or a
    // grass tuft's crossed pair).
    using PlacementCache = std::unordered_map<CellKey, std::vector<PlacedCard>, CellKeyHash>;

    // The stationary families the scheduler re-anchors independently: wide
    // rings (trees) re-anchor on a wide step, the near-field grass follows
    // the configured rebuild distance, and at most one family re-anchors per
    // update so no frame pays for two ring shifts at once.
    enum class Family
    {
        Trees = 0,
        Bushes,
        Grass,
        Rocks,
        Count
    };

    // Foliage emit styling: the ground-fade band and the aerial-haze fold
    // differ per family (trees fade hardest, grass not at all), and both key
    // on the card's distance from the anchor the mesh is baked against.
    struct FoliageEmitStyle
    {
        double groundFadeStartMeters = 0.0;
        double groundFadeEndMeters = 0.0;
        double groundFadeWeight = 0.0;
        bool aerialHaze = false;
        glm::vec3 hazeColor{};
        double hazeScaleMeters = 1.0;
        double hazeAsymptote = 0.0;
    };

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

    // Tangent frame (east/north) at one eye position; the shared basis every
    // walk's cell jitter and grass card orientation derive from.
    void tangentFrameAt(const glm::dvec3 &eyePlanetLocal, glm::dvec3 &outEyeDirection, glm::dvec3 &outEast,
        glm::dvec3 &outNorth) const;

    // Bakes one placement's contact-shadow ribbon (the pure part of the old
    // appendShadow: gates, sun geometry, and the ground march; no scratch
    // writes). The replay into the shadow mesh happens in
    // rebuildShadowMesh.
    void bakeShadowRibbon(const glm::dvec3 &groundPoint, const glm::dvec3 &up, double sizeMeters, double heightMeters,
        ShadowRibbon &outRibbon) const;

    // Emits one cached card into a family mesh: the distance-driven ground
    // fade and aerial haze apply here (against the bake anchor), then the
    // billboard goes through pushBillboard with the card's own orientation
    // mode. Card distance is measured from the card base to the anchor.
    void emitFoliageCard(std::vector<engine::planet::PlanetVertex> &vertices, std::vector<std::uint32_t> &indices,
        const PlacedCard &card, const glm::dvec3 &anchor, const FoliageEmitStyle &style) const;

    // Family rebuilds: walk the ring at the family's new anchor, re-emit
    // cached cells, evaluate only the cells that entered the ring, and swap
    // the cache to the emitted set (cells outside the ring drop). Trees and
    // bushes carry the near-ring minimum population (see the walk comments);
    // viewForward ranks it into the camera's forward hemisphere.
    void rebuildTrees(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &viewForward);
    void rebuildBushes(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &viewForward);
    void rebuildGrass(const glm::dvec3 &eyePlanetLocal);
    void rebuildRocks(const glm::dvec3 &eyePlanetLocal);

    // The transient groups re-roll by design (real foam turns over): they
    // re-scatter together on the configured rebuild distance.
    void rebuildTransients(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &viewForward);

    // Replays every family's cached shadow ribbons (plus the rocks' baked
    // ribbons) into the one contact-shadow mesh, anchored at the given eye.
    // Pure arithmetic: the ground march happened at bake time.
    void rebuildShadowMesh(const glm::dvec3 &eyePlanetLocal);

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

    // The contact-shadow scratch the ribbon replay fills and uploads as one
    // mesh (cleared per shadow rebuild, not per frame).
    std::vector<engine::planet::PlanetVertex> shadowScratch_;
    std::vector<std::uint32_t> shadowIndexScratch_;

    // Per-family scheduling state: each family keeps its own anchor (the
    // re-anchor threshold derives from its ring radius - wide rings shift
    // rarely) and its placement cache. The rocks walk directly (a bounded
    // handful of cells) and carry their baked shadow ribbons in a plain list.
    glm::dvec3 familyAnchor_[static_cast<std::size_t>(Family::Count)]{};
    bool familyAnchored_[static_cast<std::size_t>(Family::Count)] = {};
    PlacementCache familyCache_[static_cast<std::size_t>(Family::Count)];
    std::vector<ShadowRibbon> rockShadowRibbons_;

    // The transient groups' shared anchor (whitecaps, surf, mist, flocks).
    glm::dvec3 transientAnchor_{0.0};
    bool transientAnchored_ = false;

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

    glm::dvec3 viewForward_{0.0};    // camera sight axis (planet-local; zero = unknown)
    glm::dvec3 lastSunDirection_{0.0, 1.0, 0.0}; // the scene sun at the last update
    double refreshAccumulator_ = 0.0;
    std::uint32_t refreshCounter_ = 0;
    bool groundLevel_ = false;
    // The eye's ground clearance vs the grass ring radius: the sward is a
    // near-field system (its ring hugs the eye), and from higher than the
    // ring's own radius a tuft subtends under a pixel, so the grass family
    // stands down until the eye descends again - the same rule in every view.
    bool grassAllowed_ = true;
    bool initialized_ = false;
    bool rendererBound_ = false;
};

} // namespace demo::planet_demo
