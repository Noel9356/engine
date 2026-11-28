// The lighting demo's procedural scene: a broad ground field, a deterministic
// scatter of thousands of static and dynamic instances, a set of direct
// lights, and a reflection/refraction showcase cluster (a stone reflecting
// pool with a transmissive water surface, glass and polished-metal pieces) -
// everything registered with the radiance_cache and reflection_field modules
// so the engine's whole lighting pipeline (sun + shadow maps, direct lights,
// probe-grid and gather indirect, reflection field with Snell refraction,
// photon caustics through the transmissive surfaces) runs over it.

#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <vector>

#include <glm/glm.hpp>

#include <engine/direct_lighting/capabilities.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/radiance_cache/capabilities.hpp>
#include <engine/reflection_field/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>

namespace demo::lighting_demo
{

struct LightingSceneCounts
{
    std::uint32_t groundTiles        = 0;
    std::uint32_t objects            = 0;
    std::uint32_t dynamicObjects     = 0;
    std::uint32_t lights             = 0;
    std::uint32_t triangles          = 0;
    std::uint32_t giSurfaces         = 0;
    std::uint32_t reflectionSurfaces = 0;
    std::uint32_t showcaseSurfaces   = 0; // pool + water + glass/chrome pieces
    std::uint32_t transmissiveSurfaces = 0; // registered with opacity below 1
};

struct LightingSceneParameters
{
    std::uint64_t seed        = 20261126u;
    std::uint32_t objectCount = 2048;
    double       dynamicShare = 0.05;
    std::uint32_t lightCount  = 8;
    double       fieldMeters  = 140.0;
    // The reflection/refraction showcase cluster around the pool. The
    // instance scatter keeps a matching clearing so the cluster owns the
    // middle of the field.
    bool   showcaseEnabled = true;
    double showcasePoolCenterZMeters = 10.0;
};

// Owns every renderable the demo shows. All geometry and every registered
// surface live in the ANCHOR-RELATIVE frame: the scene's ground-plane anchor
// (the demo's --origin) never enters the numbers here - the renderer, the
// camera and both lighting modules all evaluate in one small-magnitude
// frame, so float32 precision holds at any placement and the lighting caches
// stay stable while the camera moves.
class LightingScene final
{
public:
    LightingScene(engine::renderer::RenderCapability &renderCapability,
        engine::renderer::RendererHandle rendererHandle, const LightingSceneParameters &parameters);
    ~LightingScene();

    LightingScene(const LightingScene &)            = delete;
    LightingScene &operator=(const LightingScene &) = delete;

    // The lighting modules the scene registers with. Null entries (module
    // absent) disable the matching registrations; call before initialize.
    void bindLightingModules(engine::radiance_cache::RadianceCacheCapability *giCapability,
        engine::reflection_field::ReflectionFieldCapability *reflectionCapability,
        engine::direct_lighting::DirectLightingCapability *directLightingCapability);

    // Builds the meshes and textures, registers every surface and light.
    // assetDirectory is the demo's assets/textures folder; the ground scans
    // load from there and the scene degrades to vertex-color ground when the
    // files are missing. Returns false when the renderer cannot host the
    // basic mesh set (the demo then exits).
    bool initialize(const std::filesystem::path &assetDirectory);

    // Advances the animated instances and the orbiting lights, re-poses the
    // draw list, and pushes the dynamic transforms into the lighting
    // modules. simSeconds drives the motion. cameraEyeAnchorRelative is the
    // frame's camera position: the sky dome re-centers on it every frame
    // (a skybox must surround the eye, or its lower bands wrap behind the
    // camera and the rasterizer's screen-space banding breaks on them).
    void update(double simSeconds, const glm::dvec3 &cameraEyeAnchorRelative);

    // The animated pool's per-update pass: re-bakes the water vertex normals
    // at the simulation clock's phase (deep-water dispersion, two crossing
    // swell trains plus a gentle chop - see the water build block), re-uploads
    // the mesh, and re-describes the surface for both lighting registries so
    // the refraction and the caustic photon pass follow the moving normals.
    void updateWater(double simSeconds);

    // The frame's draw commands (valid until the next update).
    const std::vector<engine::renderer::RenderDrawCommand> &drawCommands() const { return drawCommands_; }

    [[nodiscard]] LightingSceneCounts counts() const { return counts_; }

    // The showcase pool's anchor-relative center (the demo's showcase camera
    // poses frame this point). Returns the zero vector when the showcase is
    // disabled.
    [[nodiscard]] glm::dvec3 showcaseFocus() const;

    // The showcase cluster's clearing radius: the instance scatter keeps its
    // placements at least this far from the anchor while the showcase is on
    // (the pool, its curb and the glass pieces all live inside it).
    [[nodiscard]] double showcaseClearingMeters() const;

    // Bakes the sky backdrop from the live pbr environment (call after a
    // successful initialize; a second call is a no-op). The dome is unlit
    // vertex-colored geometry keyed to the environment's own sun and
    // hemisphere colors - a zenith-to-horizon gradient, a warm glare wedge
    // around the sun direction, and a below-horizon haze - drawn behind
    // every scene command without writing depth.
    void buildSkyBackdrop(const engine::pbr::PbrEnvironment &environment);

    // Determinism audit: placement, materials and light tracks reproduce
    // from the seed alone. Returns the number of failures (0 = pass).
    std::uint32_t selfTest();

private:
    struct Instance
    {
        glm::dvec3 basePosition; // anchor-relative, on the ground
        glm::mat4  dynamicTransform{}; // last posed matrix (dynamic only)
        float      yaw     = 0.0f;
        float      scale   = 1.0f;
        float      phase   = 0.0f; // animation phase (dynamic only)
        float      bobMeters = 0.0f; // dynamic amplitude
        std::uint8_t meshSlot     = 0; // 0 box, 1 sphere, 2 column
        std::uint8_t materialSlot = 0;
        bool        dynamic = false;
        engine::radiance_cache::GiSurfaceHandle giSurface = engine::radiance_cache::invalidGiSurface;
        engine::reflection_field::ReflectionSurfaceHandle reflectionSurface
            = engine::reflection_field::invalidReflectionSurface;
    };

    // One showcase piece: a prototype mesh posed once (the glass sphere, the
    // glass column, the chrome sphere) with its own material and surface
    // registrations, separate from the scattered instances.
    struct ShowcasePiece
    {
        glm::dvec3 position{0.0};
        float      yaw   = 0.0f;
        float      scale = 1.0f;
        std::uint8_t meshSlot     = 0;
        std::uint8_t materialSlot = 0; // indexes showcaseMaterials_
        engine::radiance_cache::GiSurfaceHandle giSurface = engine::radiance_cache::invalidGiSurface;
        engine::reflection_field::ReflectionSurfaceHandle reflectionSurface
            = engine::reflection_field::invalidReflectionSurface;
    };

    // Borrowed CPU geometry of one showcase mesh (the lighting modules copy
    // these arrays at registration; kept alive for the documentation value of
    // the prototype table, same contract as the instance prototypes).
    struct ShowcaseGeometry
    {
        std::vector<glm::vec3>        positions;
        std::vector<glm::vec3>        normals;
        std::vector<glm::vec3>        colors;
        std::vector<std::uint32_t>    indices;
    };

    // The surface material facts both lighting registries describe a surface
    // with (the material palette and the showcase materials both reduce to
    // this; the renderer's MeshMaterial keeps its own per-draw copy).
    struct SurfaceFacts
    {
        float albedoColor[3]   = {1.0f, 1.0f, 1.0f};
        float emissiveColor[3] = {0.0f, 0.0f, 0.0f};
        float emissiveIntensity = 0.0f;
        float roughness = 1.0f;
        float metallic  = 0.0f;
        float opacity   = 1.0f;
        float indexOfRefraction = 1.5f;
        bool  dynamic   = false;
    };

    struct LightTrack
    {
        engine::direct_lighting::DirectLightHandle handle = engine::direct_lighting::invalidDirectLight;
        bool   orbiting                       = false;
        double angularSpeedRadiansPerSecond   = 0.0;
        double phase                          = 0.0;
        double radiusMeters                   = 0.0;
        float  heightMeters                   = 4.0f;
        engine::direct_lighting::DirectLightDescription description{};
    };

    bool loadGroundTextures(const std::filesystem::path &assetDirectory);
    void placeInstances();
    void placeLights();
    bool buildShowcase();
    bool registerSurfaces();
    bool registerBorrowedGeometry(const std::vector<glm::vec3> &positions, const std::vector<glm::vec3> &normals,
        const std::vector<glm::vec3> &colors, const std::vector<std::uint32_t> &indices, const glm::mat4 &transform,
        const SurfaceFacts &facts, engine::radiance_cache::GiSurfaceHandle &outGiSurface,
        engine::reflection_field::ReflectionSurfaceHandle &outReflectionSurface);
    void buildStaticMaterials();
    void composeDrawCommands();
    [[nodiscard]] glm::mat4 instanceTransform(const Instance &instance, double simSeconds) const;

    engine::renderer::RenderCapability  &renderCapability_;
    engine::renderer::RendererHandle     rendererHandle_;
    LightingSceneParameters              parameters_;

    engine::radiance_cache::RadianceCacheCapability      *giCapability_             = nullptr;
    engine::reflection_field::ReflectionFieldCapability  *reflectionCapability_     = nullptr;
    engine::direct_lighting::DirectLightingCapability    *directLightingCapability_ = nullptr;

    engine::renderer::MeshHandle   groundTileMesh_ = engine::renderer::invalidMesh;
    engine::renderer::MeshHandle   objectMeshes_[3] = {engine::renderer::invalidMesh,
        engine::renderer::invalidMesh, engine::renderer::invalidMesh};
    engine::renderer::TextureHandle groundAlbedo_[2] = {engine::renderer::invalidTexture,
        engine::renderer::invalidTexture};
    engine::renderer::TextureHandle groundNormal_    = engine::renderer::invalidTexture;

    // The showcase cluster (see buildShowcase): stone curb, pale pool floor
    // (the caustics receiver), transmissive rippled water (the caustics
    // transmitter and the refraction interface), and the posed glass/chrome
    // pieces reusing the object prototypes. showcaseMaterials_ holds one
    // renderer material per showcase material slot; showcaseMaterialBase_
    // anchors them inside materialStore_.
    engine::renderer::MeshHandle poolCurbMesh_  = engine::renderer::invalidMesh;
    engine::renderer::MeshHandle poolFloorMesh_ = engine::renderer::invalidMesh;
    engine::renderer::MeshHandle poolWaterMesh_ = engine::renderer::invalidMesh;
    ShowcaseGeometry             poolCurbGeometry_;
    ShowcaseGeometry             poolFloorGeometry_;
    ShowcaseGeometry             poolWaterGeometry_;
    std::vector<ShowcasePiece>   showcasePieces_;
    glm::dvec3                   showcasePoolCenter_{0.0, 0.0, 0.0};
    bool                         showcaseBuilt_ = false;

    // Animated water: the ripple normals re-bake every update (the mesh
    // geometry stays a plane at the water line) from a directional wave
    // spectrum whose phase advances with the simulation clock, so the glints,
    // the refraction and the caustic net all drift like a real pool instead
    // of sitting frozen in the bake. poolWaterVertexScratch_ carries the
    // whole vertex buffer (12 floats per vertex, the canonical layout); the
    // update rewrites only the normal floats before the upload, and the
    // registries re-describe the surface so the traced refraction and the
    // photon pass follow the moving normals.
    std::vector<float>                        poolWaterVertexScratch_;
    std::uint32_t                             poolWaterCellsX_ = 0;
    std::uint32_t                             poolWaterCellsZ_ = 0;
    float                                     poolWaterHalfX_ = 0.0f;
    float                                     poolWaterHalfZ_ = 0.0f;
    float                                     poolWaterY_     = 0.0f;
    double                                    poolWaterLastBakeSeconds = -1.0;
    engine::radiance_cache::GiSurfaceHandle           poolWaterGiSurface_
        = engine::radiance_cache::invalidGiSurface;
    engine::reflection_field::ReflectionSurfaceHandle poolWaterReflectionSurface_
        = engine::reflection_field::invalidReflectionSurface;

    // Sky backdrop (see buildSkyBackdrop): one unlit dome mesh, its own
    // material slot outside materialStore_ (the store is rebuilt by
    // buildStaticMaterials; the dome material outlives those rebuilds), and
    // the sun's billboard quad. The dome's model matrix re-centers on skyEye_
    // every update; the quad re-aims at the eye from sunDirectionWorld_.
    engine::renderer::MeshHandle    skyDomeMesh_ = engine::renderer::invalidMesh;
    engine::renderer::MeshMaterial skyBackdropMaterial_{};
    engine::renderer::MeshHandle    sunDiscMesh_ = engine::renderer::invalidMesh;
    engine::renderer::MeshMaterial sunDiscMaterial_{};
    glm::dvec3                      skyEye_{0.0};
    glm::vec3                       sunDirectionWorld_{0.0f, 0.0f, 0.0f};
    bool                            sunDiscActive_ = false;

    // Borrowed geometry the lighting modules copied at registration (kept
    // alive for the documentation value of the prototype table; the modules
    // own their copies).
    std::array<std::vector<glm::vec3>, 3>         prototypePositions_;
    std::array<std::vector<glm::vec3>, 3>         prototypeNormals_;
    std::array<std::vector<glm::vec3>, 3>         prototypeColors_;
    std::array<std::vector<std::uint32_t>, 3>     prototypeIndices_;
    std::array<float, 3>                          prototypeHalfHeights_{};
    std::array<std::uint32_t, 3>                  prototypeTriangles_{};

    std::vector<Instance>    instances_;
    std::vector<LightTrack>  lightTracks_;

    // The showcase material palette (see buildShowcase): stone, pale floor,
    // water, glass, chrome - converted to renderer materials in
    // buildStaticMaterials and to SurfaceFacts at registration.
    struct ShowcaseMaterialSlot
    {
        float baseColor[3]      = {1.0f, 1.0f, 1.0f};
        float roughness         = 0.8f;
        float metallic          = 0.0f;
        float opacity           = 1.0f;
        float indexOfRefraction = 1.5f;
    };
    std::vector<ShowcaseMaterialSlot> showcaseMaterials_;
    std::uint32_t                     showcaseMaterialBase_ = 0;

    // The ground tile transforms: a fixed grid centered on the anchor.
    std::vector<glm::mat4>   groundTransforms_;
    std::vector<engine::radiance_cache::GiSurfaceHandle>             groundGiSurfaces_;
    std::vector<engine::reflection_field::ReflectionSurfaceHandle>   groundReflectionSurfaces_;

    // Per-frame command state. The stores are reserved once and cleared per
    // frame, so every command's pointer stays valid until the frame is
    // submitted (a reallocation mid-frame would leave the earlier commands
    // reading freed memory).
    std::vector<glm::mat4>                             matrixStore_;
    std::vector<engine::renderer::MeshMaterial>        materialStore_;
    std::uint32_t                                      objectMaterialBase_ = 0;
    std::vector<engine::renderer::RenderDrawCommand>   drawCommands_;

    LightingSceneCounts counts_{};
    double fieldRadiusMeters_ = 140.0;
    double clearingMeters_    = 6.0; // instance-free radius around the anchor
    bool  initialized_        = false;
};

} // namespace demo::lighting_demo
