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

// The lighting the backdrop shells bake into their vertex colors (from the
// pbr capability's environment snapshot, so the baked shells photograph like
// the lit geometry under the same sun and sky). sunRadianceOverPi is the
// sun's color x intensity / pi - the Lambertian diffuse reflectance of a
// white surface under that sun.
struct BackdropLighting
{
    glm::dvec3 sunDirection{0.0, 1.0, 0.0};
    glm::dvec3 sunRadianceOverPi{0.0, 0.0, 0.0};
    glm::dvec3 ambientSky{0.0, 0.0, 0.0};
    glm::dvec3 ambientGround{0.0, 0.0, 0.0};
    bool sunEnabled = false;
};

// One scanned texture variant of a terrain surface group: the detail map
// the material multiplies the vertex colors with, the rock group's optional
// OpenGL-convention tangent-space normal map, and the per-channel mean
// compensation read back from the renderer's textureMean query (the gated
// material divides its base color by the SELECTED variant's mean, so the
// multiply preserves the vertex albedo on average and only adds detail).
struct TerrainTextureVariant
{
    engine::renderer::TextureHandle detail = engine::renderer::invalidTexture;
    engine::renderer::TextureHandle normal = engine::renderer::invalidTexture;
    float meanCompensation[3] = {1.0f, 1.0f, 1.0f};
    // Biome affinity bitmask (1u << biome): the per-chunk pick only draws
    // variants whose mask covers the chunk's center biome, so a grassland's
    // sward never lands on a desert chunk and vice versa. 0 = matches every
    // biome (the untyped fallback the unaffinitized scans keep).
    std::uint32_t biomeMask = 0u;
};

class PlanetChunkRenderer final
{
public:
    PlanetChunkRenderer(engine::planet::PlanetCapability &planetCapability, engine::planet::PlanetHandle planetHandle,
        engine::renderer::RenderCapability &renderCapability, engine::renderer::RendererHandle rendererHandle,
        const engine::planet::PlanetGenerationParams &generationParams);
    ~PlanetChunkRenderer();

    PlanetChunkRenderer(const PlanetChunkRenderer &) = delete;
    PlanetChunkRenderer &operator=(const PlanetChunkRenderer &) = delete;

    // Configures the terrain surface-group texture VARIANTS the chunk
    // materials reference (see the surface-group partition in the planet
    // module's meshing.hpp): each group carries a set of scanned variants
    // and every chunk's draw picks one deterministically from the chunk
    // address, so neighboring chunks photograph different scans and no
    // single scan repeats across the visible disk. invalidTexture variants
    // are skipped; a group with no loadable variant draws untextured (the
    // material then samples the renderer's neutral white stand-in, so the
    // vertex colors stand alone).
    // The per-group gate distances (meters) keep the multiply off beyond
    // them; a zero/negative gate distance keeps the texture off everywhere.
    // Each variant's meanCompensation carries its detail scan's mean color
    // as 1/mean (read back from the renderer's textureMean query): a chunk
    // inside the gate divides its material base color by the SELECTED
    // variant's mean, so the texture multiply preserves the vertex albedo
    // on average and only adds detail, and the gate boundary cannot read.
    // groundReliefMaxDistanceMeters gates the ground variants' NORMAL maps
    // separately and tighter than the albedo gate (see setTerrainTextures'
    // implementation note); a zero/negative value keeps the relief off.
    void setTerrainTextures(const std::vector<TerrainTextureVariant> &groundVariants,
        const std::vector<TerrainTextureVariant> &rockVariants, double groundTextureMaxDistanceMeters,
        double rockTextureMaxDistanceMeters, double groundReliefMaxDistanceMeters);

    // Updates the texture gate distances after the initial setTerrainTextures
    // registration: the in-app view switch re-applies the framing (a view
    // family carries its own field of view, which rescales every gate). The
    // variants and their mean compensations stay as registered.
    void setTerrainTextureGates(double groundTextureMaxDistanceMeters,
        double rockTextureMaxDistanceMeters, double groundReliefMaxDistanceMeters);

    // Installs the ground's near-field relief overlay (MeshMaterial's detail
    // triple): a much finer-tiling normal map whose influence fades in as the
    // eye approaches - full at detailFadeStartMeters, gone at
    // detailFadeEndMeters - so the ground gains real grain during the
    // descent instead of re-showing the same base tiling at every distance.
    // detailUvScale multiplies the ground UVs into the overlay's tiling
    // (tileMetersGround / detailTileMeters); invalidTexture disables the
    // overlay. The overlay rides every ground chunk's material; the per-pixel
    // distance fade keeps far chunks untouched.
    void setGroundDetailNormal(engine::renderer::TextureHandle detailNormal, float detailUvScale,
        float detailFadeStartMeters, float detailFadeEndMeters, float detailStrength);

    // Installs the environment lighting the world-anchored cloud shell bakes
    // into its vertex colors (see BackdropLighting). Called before
    // ensureCloudLayer; the shell's material draws unlit, so the per-pixel
    // PBR evaluation over the disk-sized shell is replaced by the vertex
    // bake (its matte material carries no visible specular to lose).
    void setBackdropLighting(const BackdropLighting &lighting);

    void updateVisibleChunks();
    // Assembles the frame's draw commands. Opaque scene geometry goes first
    // (chunk terrain surface groups + water, near to far), then the
    // world-anchored cloud shell and the sky cloud deck (both depth-tested
    // against the terrain, so each shell's far side rejects early), and the
    // unlit backdrop shells last (star dome, sun disc, sky dome - every
    // pixel they cover that terrain already drew depth-fails). The
    // atmosphere limb shell STAYS ahead of the scene: its disk interior is
    // meant to be overwritten by the opaque draws. viewProjection
    // frustum-culls the chunk draws (the planet's own selection spans the
    // whole visible disk, including the half behind the camera);
    // eyePlanetLocal drives the per-chunk aerial perspective (distant
    // terrain fades toward the horizon haze through the material's base
    // color and emissive terms - the renderer's shading is albedo * light,
    // so the fade composes exact fog: baseColor scales the terrain down by
    // (1 - w) and the emissive term adds hazeColor * w). sunTint is the
    // same tint the sky dome carries this frame.
    void rebuildDrawCommands(const glm::mat4 &viewProjection, const glm::dvec3 &eyePlanetLocal,
        const engine::planet::Vec3 &sunTint, const glm::dvec3 &renderShift);
    void destroy();

    // Sky backdrop (see engine/planet/atmosphere.hpp): a static gradient
    // sphere on the unit ball, kept centered on the eye and scaled per frame
    // so its rim always sits beyond the farthest terrain the eye can see but
    // inside the far plane. Drawn first, unlit (vertex colors only), so it
    // never shades or occludes - it only fills the pixels the terrain does
    // not reach: the sky itself, plus any chunk still streaming in. The bake
    // is sun-aware (see the sun-aware skyGradientColor twin): the canonical
    // sun sits on the dome's +X meridian at the sun's own local elevation,
    // and the per-frame rotation turns that meridian toward the real sun so
    // the glare wedge and the warm horizon land on the sun's azimuth.
    void ensureSkyDome(const glm::dvec3 &sunDirection, bool sunEnabled, const glm::dvec3 &up);
    // Rebakes the dome's vertex colors for a sun whose local elevation has
    // drifted from the baked one (a long interactive session walking toward
    // a pole); the threshold is small enough that capture runs never trigger
    // it twice.
    void rebakeSkyDomeColors(double sunUp);
    // Places and tints the dome for this frame's eye. eyePlanetLocal is the
    // eye in planet-local space, up the local gravity gradient, sunDirection
    // the pbr sun's direction. Reports the horizon-haze color for the frame
    // clear color and returns false when the sky is off (atmosphere disabled,
    // eye outside the atmosphere), leaving the clear color at black.
    bool updateSkyDome(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &up, const glm::dvec3 &sunDirection,
        bool sunEnabled, double nearZ, double farZ, float outClearColor[3]);

    // Cloud layer (see engine/planet/atmosphere.hpp CloudField): a static
    // shell above the surface, drawn PER-PIXEL through the baked cloud
    // texture (ensureCloudTexture) - the shell's UV grid hands every pixel
    // to the texture, whose alpha carries the full-resolution coverage
    // field (bank bodies, wisp octaves, cyclone arms) and whose RGB carries
    // the sun-lit bake (thickness ramp, sun-facing gradient, terminator
    // silver lining). Quads whose corners all read clear skip, so the mesh
    // only covers the cloudy provinces. World-anchored: macro
    // (globe/orbit) cameras see the layer against the surface from outside
    // the shell, alpha-blended over the terrain and the limb glow.
    void ensureCloudLayer(const glm::dvec3 &sunDirection);
    // The shell is built for cameras OUTSIDE it: a camera between the
    // surface and the shell would see the far side of the sphere through the
    // sky above the horizon (the demo renders terrain two-sided, so the
    // inside-out quads are not culled), so ground views hide the layer.
    void setCloudLayerVisible(bool visible);

    // The bake behind every cloud draw (shell and deck): the module's
    // coverage field sampled per equirect texel with the texel-only wisp
    // octaves and cyclone shear, lit by the run's fixed sun. The pixel copy
    // stays owned by the renderer so mesh gates can consult it without
    // going through the texture handle.
    void ensureCloudTexture(const glm::dvec3 &sunDirection);

    // Close-range sea surface detail (see waterNormalTexture_): a pair of
    // tileable sheets - wave normals and albedo variation - the near-water
    // draws sample; baked once, gated per draw by distance.
    void ensureWaterDetailTextures();

    // The sky deck's own bake (see deckTexture_): the coverage field with
    // ground-sky lighting for overhead cameras, sampled where the eye's rays
    // pierce the cloud shell (the chord geometry the deck's source comment
    // describes).
    void ensureDeckTexture(const glm::dvec3 &sunDirection, const glm::dmat3 &domeToPlanet,
        const glm::dvec3 &eyePlanetLocal);

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

    // Moon companion (backdrop, LIT): a small sphere the module's orbital
    // model places on the sky (see atmosphere.hpp moonDirection). The draw
    // rides the regular lit path - the same sun that lights the terrain
    // lights the sphere, so the phase, the terminator and the crescent are
    // the shading's own output, at every eye height and for every orbit
    // position, with no baked phase to desynchronize. Placed like the sun
    // disc at the backdrop distance along the moon direction (the sky
    // dome's own radius in air, between the disc and the star dome in
    // space), depth writes ON: banks in front hide it, and it hides the
    // stars behind it. The material tints with the atmosphere's direct
    // transmittance keyed on the moon's own elevation (the regolith's
    // reflected light passes the same air column the sun's direct light
    // does), so a horizon moon reddens exactly the way the low sun does.
    void ensureMoon();
    void updateMoon(const glm::dvec3 &eyePlanetLocal, const glm::dvec3 &up, const glm::dvec3 &moonDirection,
        bool sunEnabled, double farZ);

    // Aurora curtain (polar night glow): a translucent emissive ring above
    // each spin pole (see atmosphere.hpp aurora constants) whose vertex
    // colors bake the oxygen-green foot, the violet top, the azimuthal ray
    // striations and the night gate against the run's sun direction -
    // space views read a green arc along the dark limb while day-side and
    // twilight ground views stay clean. Drawn in the renderer's transparent
    // pass over the opaque scene, depth-tested so the planet hides the far
    // side of the rings. A space-view backdrop like the limb shell: once the
    // eye is inside the air the ring's sky-projected footprint only blends
    // a near-transparent wash over the horizon haze (nothing it shows pays
    // for its raster), so updateAurora hides it there.
    void ensureAurora(const glm::dvec3 &sunDirection);
    void updateAurora(const glm::dvec3 &eyePlanetLocal);

    // High cirrus deck (space views): a translucent streak veil above the
    // cumulus shell meshed from the module's cirrus field (see
    // atmosphere.hpp cirrusStreakWeight) - the deck's alpha texture carries
    // the filament intensity times a day-side gate, so the veil reads as
    // wisps over the sunlit planet and composites to nothing over the night
    // hemisphere, exactly the aurora's intensity-texture route. Drawn in the
    // renderer's transparent pass, depth-tested against the terrain and the
    // cumulus shell, depth writes off; like the aurora it hides once the eye
    // is inside the air (the sky deck owns in-air skies).
    void ensureCirrusLayer(const glm::dvec3 &sunDirection);
    void updateCirrusLayer(const glm::dvec3 &eyePlanetLocal);

    // Surface scatter (see planet_scatter.hpp): the ground-level life the
    // terrain's color field cannot express - vegetation, surface rocks, bird
    // flocks, breaking-wave foam and valley mist. The system owns its meshes,
    // materials and update cadence; this renderer only composes the shifted
    // model matrices and appends the draw commands (opaque scatter rides the
    // opaque scene pass, the translucent foam/mist commands land in the
    // transparent tail like the aurora). Null keeps the scene bare ground.
    void setSurfaceScatter(class PlanetScatterSystem *scatter);

    [[nodiscard]] const std::vector<engine::renderer::RenderDrawCommand> &drawCommands() const noexcept
    {
        return drawCommands_;
    }

private:
    struct ChunkMeshHandles
    {
        engine::renderer::MeshHandle terrain = engine::renderer::invalidMesh;
        engine::renderer::MeshHandle rock = engine::renderer::invalidMesh;
        engine::renderer::MeshHandle water = engine::renderer::invalidMesh;
        glm::mat4 terrainTransform{1.0f};
        glm::mat4 waterTransform{1.0f};
        // The same placement in f64 (planet-local + center), set whenever a
        // transform is built: the camera-relative pass composes against it,
        // so chunk placement never quantizes at planet scale. The rock
        // group shares the terrain placement (both terrain groups draw with
        // the terrainTransform).
        glm::dvec3 placementCenter{0.0};
        bool terrainPlacementBuilt = false; // terrainTransform carries the chunk's placement
        bool waterPlacementBuilt = false;   // waterTransform carries the chunk's placement
        std::uint64_t lastSeenUpdate = 0;  // update that last listed this chunk as visible
        std::uint64_t terrainHash = 0;     // content hash of the uploaded ground-group mesh
        // Snow-gate scratch (computed lazily at first ground-material emit):
        // whether the chunk's center classifies as the snow biome. A snow
        // chunk's ground group draws UNTEXTURED - the grass detail pair's
        // blade streaks magnified over a snowfield photograph as combed
        // parallel lines (the meltwater-flow texture no snowpack shows).
        // Mutable: the emit pass sees the map's values const, and the cache
        // only memoizes a pure function of the chunk key.
        mutable bool groundSnowy = false;
        mutable bool groundSnowyBuilt = false;
        // The chunk's center biome (lazily classified with the snow gate:
        // one climate sample memoizes both reads). The ground variant pick
        // draws from the biome's own texture pool (see the emit pass).
        mutable std::uint32_t groundBiomeSlot = 0u;
        std::uint64_t rockHash = 0;        // content hash of the uploaded rock-group mesh
        std::uint64_t waterHash = 0;       // content hash of the uploaded water mesh
    };

    void destroyChunkMeshes(ChunkMeshHandles &handles);
    void destroySkyDome();
    void destroyCloudLayer();
    void destroyAtmosphereShell();
    void destroySkyCloudLayer();
    void destroyStarDome();
    void destroySunDisc();
    void destroyMoon();
    void destroyAurora();
    void destroyCirrusLayer();
    void destroyCloudTexture();
    void destroyWaterDetailTextures();

    engine::planet::PlanetCapability *planet_ = nullptr;
    engine::planet::PlanetHandle planetHandle_ = engine::planet::invalidPlanet;
    engine::renderer::RenderCapability *renderer_ = nullptr;
    engine::renderer::RendererHandle rendererHandle_ = engine::renderer::invalidRenderer;
    engine::planet::PlanetGenerationParams params_{};

    std::map<engine::planet::ChunkKey, ChunkMeshHandles> uploadedMeshes_;
    // Per-frame scratch for the near-to-far chunk ordering (see
    // rebuildDrawCommands): the frustum-surviving chunks with their squared
    // eye distances, sorted before any command storage is emitted. Reused
    // across frames like the matrix and material stores.
    struct VisibleChunkDraw
    {
        const std::pair<const engine::planet::ChunkKey, ChunkMeshHandles> *entry = nullptr;
        double distanceSquared = 0.0;
    };
    std::vector<VisibleChunkDraw> frameVisibleDraws_;
    // Per-frame scratch for the far-to-near transparent-tail ordering (see
    // rebuildDrawCommands): the scatter system's transparent commands (foam,
    // mist) with their squared eye distances, sorted farthest first so the
    // alpha-blended banks composite back-to-front. Reused across frames.
    struct TransparentScatterDraw
    {
        std::size_t index = 0;
        double distanceSquared = 0.0;
    };
    std::vector<TransparentScatterDraw> frameTransparentScatter_;
    // Surface scatter command plumbing (see setSurfaceScatter): per-command
    // composed matrices, refreshed inside rebuildDrawCommands from the
    // scatter system's anchors. Borrowed pointer; the demo owns the system.
    class PlanetScatterSystem *surfaceScatter_ = nullptr;
    std::vector<glm::mat4> frameScatterMatrices_;

    // Per-frame model matrices for the camera-relative pass. Sized exactly
    // before the commands that point into them are emitted, so the pointers
    // stay stable until the frame's draw consumes them.
    std::vector<glm::mat4> frameChunkMatrices_;
    std::vector<glm::mat4> frameBackdropMatrices_;
    std::vector<engine::planet::PlanetChunkView> visibleViews_; // reused every update
    std::uint64_t updateCounter_ = 0;
    // Capped upload-failure reports (see updateVisibleChunks).
    std::uint32_t uploadFailureReports_ = 0;
    // Draw commands retain pointers to materials until endFrame. Deques keep
    // references stable while the command list is being assembled.
    std::deque<engine::renderer::MeshMaterial> terrainMaterials_;
    std::deque<engine::renderer::MeshMaterial> rockMaterials_;
    std::deque<engine::renderer::MeshMaterial> waterMaterials_;
    std::vector<engine::renderer::RenderDrawCommand> drawCommands_;

    // Environment lighting for the baked cloud shell (see
    // setBackdropLighting).
    BackdropLighting backdropLighting_{};
    bool backdropLightingValid_ = false;

    // Terrain surface-group texture variants (see setTerrainTextures): the
    // per-group variant sets the chunk draws pick from, and the per-group
    // gate distances within which the multiply applies.
    std::vector<TerrainTextureVariant> groundTextureVariants_;
    std::vector<TerrainTextureVariant> rockTextureVariants_;
    double groundTextureMaxDistanceMetersSquared_ = 0.0;
    double groundReliefMaxDistanceMetersSquared_ = 0.0;
    double rockTextureMaxDistanceMetersSquared_ = 0.0;
    // Near-field relief overlay on the ground group (setGroundDetailNormal).
    engine::renderer::TextureHandle groundDetailNormal_ = engine::renderer::invalidTexture;
    float groundDetailUvScale_ = 0.0f;
    float groundDetailFadeStartMeters_ = 2.0f;
    float groundDetailFadeEndMeters_ = 40.0f;
    float groundDetailStrength_ = 1.0f;

    // Sky dome: unit-radius gradient sphere (positions on the unit ball,
    // colors from the atmosphere model's sun-aware gradient) shared by every
    // frame; the per-frame transform places, orients and scales it. The
    // baked sun's local elevation rides along so a drifting sun rebakes the
    // colors instead of wearing the wrong elevation's glare wedge.
    engine::renderer::MeshHandle skyDome_ = engine::renderer::invalidMesh;
    glm::mat4 skyDomeTransform_{1.0f};
    bool skyDomeActive_ = false;
    double skyDomeBakedSunUp_ = 2.0; // sentinel: nothing baked yet
    std::deque<engine::renderer::MeshMaterial> skyDomeMaterials_;

    // Cloud layer: world-anchored shell drawn from the module's cloud
    // coverage field, per-pixel through the baked cloud texture (see
    // cloudTexture_): the shell mesh carries equirectangular UVs into the
    // texture, whose alpha holds the coverage and RGB the sun-lit bake.
    engine::renderer::MeshHandle cloudLayer_ = engine::renderer::invalidMesh;
    glm::mat4 cloudLayerTransform_{1.0f};
    bool cloudLayerActive_ = false;
    bool cloudLayerVisible_ = false;
    std::deque<engine::renderer::MeshMaterial> cloudLayerMaterials_;
    std::unique_ptr<engine::planet::CloudField> cloudField_;

    // Cloud texture (shared by the world shell and the sky deck): the
    // module's coverage field baked once at startup into an equirectangular
    // RGBA tile - alpha carries per-texel coverage (bank bodies, ragged
    // wisp fringes, cyclone arms), RGB the cloud color under the run's fixed
    // sun (thickness ramp, sun-facing gradient, terminator silver lining).
    // The pixel copy stays owned here so the mesh builders can gate quads
    // and remap UVs against the same bake the shell draws.
    engine::renderer::TextureHandle cloudTexture_ = engine::renderer::invalidTexture;
    std::vector<std::uint8_t> cloudTexturePixels_;
    std::uint32_t cloudTextureWidth_ = 0;
    std::uint32_t cloudTextureHeight_ = 0;

    // The deck's own bake (see ensureSkyCloudLayer): the same coverage
    // field with GROUND-SKY lighting - a lifted ambient floor so the whole
    // dome's banks read bright against the blue (real midday decks
    // scatter the sky's own light), the sun side a touch brighter, a
    // subtle warm rim toward the low-sun horizon. The space bake's
    // terminator contrast paints the anti-sun sky nearly black - right
    // for orbit, wrong overhead. The pixel copy stays owned here (like the
    // space bake's own copy): the deck's vertex colors bilinear this
    // array per rebuild, so the bake must outlive its texture upload.
    engine::renderer::TextureHandle deckTexture_ = engine::renderer::invalidTexture;
    std::vector<std::uint8_t> deckTexturePixels_;
    std::uint32_t deckTextureWidth_ = 0;
    std::uint32_t deckTextureHeight_ = 0;

    // Water surface detail (see setWaterDetailTextures): a tileable wave-
    // normal texture and a low-frequency albedo variation sheet the near-
    // water draws sample, so close-range seas ripple under the sun instead
    // of reading as a flat painted sheet. Both gate by distance like the
    // terrain scans (minified normal maps alias into open-ocean sparkle).
    engine::renderer::TextureHandle waterNormalTexture_ = engine::renderer::invalidTexture;
    engine::renderer::TextureHandle waterAlbedoTexture_ = engine::renderer::invalidTexture;
    double waterTextureMaxDistanceMetersSquared_ = 0.0;

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
    // The disc's radial halo (alpha falloff; see ensureSunDisc): the disc
    // blends at opacity x texel alpha, fading into the sky behind it.
    engine::renderer::TextureHandle sunHaloTexture_ = engine::renderer::invalidTexture;
    glm::mat4 sunDiscTransform_{1.0f};
    bool sunDiscActive_ = false;
    std::deque<engine::renderer::MeshMaterial> sunDiscMaterials_;
    double sunDiscAngularRadiusRadians_ = 0.0;

    // Moon companion (backdrop, lit; see ensureMoon): unit-ball sphere with
    // baked regolith albedo (maria provinces, crater mottling), placed and
    // scaled per frame along the module's orbital moon direction.
    engine::renderer::MeshHandle moon_ = engine::renderer::invalidMesh;
    glm::mat4 moonTransform_{1.0f};
    bool moonActive_ = false;
    std::deque<engine::renderer::MeshMaterial> moonMaterials_;

    // Aurora curtain rings (see ensureAurora): planet-local translucent
    // emissive mesh, world-anchored at the planet center like the cloud
    // shell, static after upload (the night gate is baked against the run's
    // fixed sun direction).
    engine::renderer::MeshHandle aurora_ = engine::renderer::invalidMesh;
    engine::renderer::TextureHandle auroraTexture_ = engine::renderer::invalidTexture;
    glm::mat4 auroraTransform_{1.0f};
    bool auroraActive_ = false;
    std::deque<engine::renderer::MeshMaterial> auroraMaterials_;

    // High cirrus deck (see ensureCirrusLayer): planet-local translucent
    // streak veil above the cumulus shell, world-anchored at the planet
    // center and static after upload (the day-side gate is baked against
    // the run's fixed sun direction, like the aurora's night gate).
    engine::renderer::MeshHandle cirrusLayer_ = engine::renderer::invalidMesh;
    engine::renderer::TextureHandle cirrusTexture_ = engine::renderer::invalidTexture;
    glm::mat4 cirrusLayerTransform_{1.0f};
    bool cirrusLayerActive_ = false;
    std::deque<engine::renderer::MeshMaterial> cirrusLayerMaterials_;
};

} // namespace demo::planet_demo

