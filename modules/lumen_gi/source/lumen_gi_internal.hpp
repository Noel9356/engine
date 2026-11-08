// Internal state of the lumen_gi module, shared by its translation units.
//
// The pipeline mirrors Lumen's stages scaled to a CPU implementation:
//
//   surfaces (registry)  ->  per-surface cards (6-face axis-aligned capture,
//   Lumen's documented fallback placement) -> budgeted card capture (albedo,
//   emissive, normal, depth) -> amortized direct lighting on card texels ->
//   indirect lighting propagation (cosine gathers reading direct + previous
//   indirect, one bounce per frame, N+2 through feedback) -> world-space
//   radiance cache (camera-centered clipmap of octahedral probes + SH2) ->
//   screen-space final gather (module debug view) -> composed debug image.
//
// Rays trace the registered triangle scene through per-surface BVHs (traces
// into local space through the inverse transform, so dynamic transforms never
// rebuild anything); shading at every hit goes through the card surface cache
// exactly like Lumen's card lookup.

#ifndef ENGINE_LUMEN_GI_INTERNAL_HPP
#define ENGINE_LUMEN_GI_INTERNAL_HPP

#include <engine/lumen_gi/capabilities.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/direct_lighting/capabilities.hpp>

#include <glm/geometric.hpp>
#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

namespace engine
{
namespace lumen_gi
{

inline constexpr float invalidCardDepth = std::numeric_limits<float>::max();


// Every configurable [lumen_gi] value; filled by the module from
// configuration and refreshed on live reload. See docs/configuration.md.
struct GiSettings
{
    bool          enabled                    = true;
    // Static-frame skip: when camera, scene, and environment are all
    // unchanged and the caches have converged, evaluate() returns early
    // instead of re-blending the same radiance. See docs/configuration.md.
    bool          skipUnchangedFrames        = true;
    std::uint32_t quality                    = 3;  // QualityPreset::High
    std::uint32_t maxSurfaces                = 256;
    std::uint64_t maxTrianglesPerSurface     = 200000;
    std::uint32_t maxCardsPerSurface        = 12;
    float         texelsPerMeter             = 24.0f;
    std::uint32_t maxTexelsPerCardAxis       = 128;
    std::uint64_t captureTexelsPerFrame      = 262144;
    std::uint64_t directTexelsPerFrame       = 65536;
    std::uint64_t indirectTexelsPerFrame     = 65536;
    bool          sunShadows                 = true;
    bool          pointLightShadows          = false;
    float         lightingUpdateSpeed        = 1.0f;
    float         indirectTemporalBlend      = 0.8f;
    float         maxTraceDistance           = 50.0f;
    float         probeSpacing               = 2.0f;
    std::uint32_t probeDirectionResolution   = 16;
    std::uint32_t probeUpdatesPerFrame       = 48;
    float         probeHysteresis            = 0.9f;
    float         probeLevel1SpacingFactor   = 4.0f;
    float         probeLevel1ExtentFactor    = 2.0f;
    float         screenRayDistance          = 2.0f;
    std::uint32_t gatherScreenScale          = 8;
    std::uint32_t gatherRaysPerProbe         = 8;
    bool          gatherImportanceSampling   = true;
    float         gatherTemporalBlend        = 0.85f;
    std::uint32_t gatherSpatialFilter        = 3;
    float         gatherPlaneDepthTolerance  = 0.35f;
    std::uint32_t debugInternalWidth         = 320;
    std::uint32_t debugInternalHeight        = 180;
    std::uint32_t referenceSamples           = 64;
    std::uint32_t referenceBounces           = 4;
    bool          debugIncludeAmbient        = false;
};

// One captured card: an axis-aligned ortho projection of its surface.
// U/V span the card plane, W points outward; capture rays look along -W from
// slightly outside the near plane. All buffers are texelWidth x texelHeight.
struct GiCard
{
    // Card placement (world space).
    glm::vec3    origin{0.0f};
    glm::vec3    axisU{1.0f, 0.0f, 0.0f};
    glm::vec3    axisV{0.0f, 1.0f, 0.0f};
    glm::vec3    axisW{0.0f, 0.0f, 1.0f}; // outward; capture direction is -axisW
    float        width      = 1.0f;
    float        height     = 1.0f;
    float        depth      = 1.0f;      // AABB thickness along -axisW
    std::uint32_t surfaceIndex = 0;

    // Capture buffers. depthTexels stores the hit distance per texel
    // (invalidCardDepth = no surface); `depth` above is the card thickness.
    std::uint32_t        texelWidth  = 4;
    std::uint32_t        texelHeight = 4;
    std::vector<float>   depthTexels;
    std::vector<glm::vec3> albedo;
    std::vector<glm::vec3> emissive;
    std::vector<glm::vec3> normal;
    std::vector<glm::vec3> directRadiance;   // Lumen "Lumen Scene" direct term
    std::vector<glm::vec3> indirectRadiance; // propagated bounce term (double-buffered)
    std::vector<glm::vec3> indirectPrevious; // read side while writing indirectRadiance
    std::vector<glm::vec3> scratch;          // staging for gather writes

    // Amortization stamps (frame indices; 0 = never touched).
    std::uint64_t lastCaptureFrame  = 0;
    std::uint64_t lastDirectFrame   = 0;
    std::uint64_t lastIndirectFrame = 0;
    float         cameraDistance    = 0.0f;
};

// Triangle soup in local space plus its BVH. Traces transform the ray into
// local space, so dynamic surfaces only update their transform.
struct GiSurfaceGeometry
{
    std::vector<glm::vec3>    positions;
    std::vector<glm::vec3>    normals;   // smooth vertex normals in local space
    std::vector<glm::vec3>    colors;    // vertex colors (white when absent)
    std::vector<std::uint32_t> indices;

    // Interior nodes: leftOrFirst = left child node index (right is left+1),
    // count = 0. Leaves: leftOrFirst = first triangle in bvhOrder, count > 0.
    struct BvhNode
    {
        glm::vec3      boundsMin{0.0f};
        glm::vec3      boundsMax{0.0f};
        std::uint32_t  leftOrFirst = 0;
        std::uint32_t  count       = 0;
    };
    std::vector<BvhNode>       bvh;
    std::vector<std::uint32_t> bvhOrder; // triangle indices in leaf order
    glm::vec3                  boundsMin{0.0f};
    glm::vec3                  boundsMax{0.0f};
};

struct GiSurface
{
    GiSurfaceHandle   handle = invalidGiSurface;
    GiSurfaceGeometry geometry;
    glm::mat4         transform{1.0f};
    glm::mat4         inverseTransform{1.0f};
    glm::mat3         normalMatrix{1.0f}; // inverse-transpose of upper 3x3
    glm::vec3         albedo{1.0f};
    glm::vec3         emissive{0.0f};
    std::uint32_t     flagBits     = 0;
    bool              excluded     = false;
    bool              unlit        = false;
    bool              dynamic      = false;
    bool              cardsBuilt   = false;
    glm::vec3         worldMin{0.0f};
    glm::vec3         worldMax{0.0f};
    std::uint32_t     firstCard    = 0; // range into GiPipeline::cards
    std::uint32_t     cardCount    = 0;
};

// Snapshot of the pbr environment + direct lights taken each evaluate().
struct GiEnvironmentSnapshot
{
    bool     hasSun       = false;
    bool     sunEnabled   = false;
    glm::vec3 sunDirection{0.45f, 0.62f, 0.35f}; // toward the sun (unit)
    glm::vec3 sunColor{1.0f, 0.96f, 0.88f};
    float     sunIntensity = 3.0f;
    glm::vec3 ambientSky{0.55f, 0.65f, 0.80f};
    glm::vec3 ambientGround{0.35f, 0.30f, 0.25f};
    float     ambientIntensity = 0.5f;
    bool      hasDirectLights  = false;
    std::vector<direct_lighting::DirectLight> directLights;
    float            exposure   = 1.0f;
    pbr::ToneMapping toneMapping = pbr::ToneMapping::AcesFilmic;

    // Hemisphere skylight standing in for Lumen's Skylight (the environment
    // is the engine's only "infinitely far" lighting source).
    glm::vec3 skyRadiance(const glm::vec3 &direction) const
    {
        const float up = direction.y * 0.5f + 0.5f;
        return glm::mix(ambientGround, ambientSky, up) * ambientIntensity;
    }
    glm::vec3 sunRadiance() const
    {
        return (hasSun && sunEnabled) ? sunColor * sunIntensity : glm::vec3(0.0f);
    }
};

// World-space ray with normalized direction; hits reference the surface array.
struct GiRay
{
    glm::vec3 origin{0.0f};
    glm::vec3 direction{0.0f, 0.0f, -1.0f};
    float     tMax = std::numeric_limits<float>::max();
};

struct GiHit
{
    bool          hit          = false;
    float         t            = 0.0f;
    std::uint32_t surfaceIndex = 0;
    std::uint32_t triangleIndex = 0;
    glm::vec3     position{0.0f};
    glm::vec3     normal{0.0f, 1.0f, 0.0f};   // world smooth normal
    glm::vec3     geometricNormal{0.0f, 1.0f, 0.0f};
};

// Ray-primitive core shared by every stage. Implemented in
// lumen_gi_scene.cpp; parallel stages call these freely (all inputs are
// immutable while stages run).
bool rayTriangle(const GiRay &ray, const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2, float &outT,
    glm::vec3 &outBarycentric);
bool rayBounds(const glm::vec3 &origin, const glm::vec3 &inverseDirection, const glm::vec3 &boundsMin,
    const glm::vec3 &boundsMax, float tMax);

// Closest-hit trace against every non-excluded surface; tMax limits the ray.
GiHit traceScene(const struct GiPipelineState &state, const GiRay &ray);
// Any-hit test used for shadows; returns true when geometry blocks the ray.
bool traceSceneOccluded(const struct GiPipelineState &state, const GiRay &ray, std::uint32_t excludedSurface);

// The whole pipeline state owned by the module instance. Mutators (surface
// registry, setView, evaluate, config) hold moduleMutex_; stage tasks touch
// disjoint buffers and read only state frozen before the graph executes.
struct GiPipelineState
{
    GiSettings settings;

    // Registry (mirrors the module's registry under the module mutex).
    std::vector<GiSurface>   surfaces;
    std::vector<GiCard>      cards;
    std::uint32_t            registryCapacity = 256;
    GiSurfaceHandle          nextHandle       = 1;
    std::uint64_t            frameIndex       = 0;

    // Camera + environment snapshot for the running frame. The previous
    // frame's camera drives the final gather's temporal reprojection.
    bool      hasView   = false;
    glm::mat4 viewProjection{1.0f};
    glm::vec3 cameraPosition{0.0f};
    glm::vec3 cameraForward{0.0f, 0.0f, -1.0f};
    glm::mat4 previousViewProjection{1.0f};
    glm::vec3 previousCameraPosition{0.0f};
    bool      hasPreviousView = false;
    // Static-frame skip change detection: setView marks viewDirty when the
    // passed matrices differ from the previous call, surface mutators mark
    // sceneDirty, and evaluate() consumes both. framesSinceDirty counts
    // consecutive non-dirty evaluates (the convergence gate for the skip).
    bool          viewDirty  = true;
    bool          sceneDirty = true;
    std::uint32_t framesSinceDirty = 0;
    GiEnvironmentSnapshot environment;
    bool        environmentValid = false;
    std::uint64_t environmentVersion = 0;
    std::uint64_t directLightsVersion = 0;

    // World-space radiance cache: two camera-centered clip levels. Probes sit
    // on a fixed lattice anchored to the camera (snapped to spacing), so a
    // probe's array slot keeps a stable world position between snaps.
    struct ProbeLevel
    {
        float                    spacing = 2.0f;
        std::array<std::uint32_t, 3> dimensions{0u, 0u, 0u};
        glm::vec3                anchor{0.0f}; // snapped lattice origin
        struct Probe
        {
            glm::vec3 position{0.0f};
            std::vector<glm::vec3> radiance;  // octahedral map (directionResolution^2)
            std::vector<glm::vec3> radiancePrevious;
            std::vector<float>     sh2;       // 9 x 3 floats, basis order of GiProbe
            float                  validity   = 0.0f;
            std::uint64_t          lastUpdate = 0;
            bool                   valid      = false;
        };
        std::vector<Probe> probes;
        std::uint32_t      updateCursor = 0;
    };
    ProbeLevel level0;
    ProbeLevel level1;

    // Screen-space final gather (debug view) buffers at internal resolution.
    std::uint32_t viewWidth  = 0;
    std::uint32_t viewHeight = 0;
    std::uint32_t screenProbeWidth  = 0;
    std::uint32_t screenProbeHeight = 0;
    std::uint32_t previousViewWidth  = 0;
    std::uint32_t previousViewHeight = 0;
    std::vector<float>     viewDepth;       // linear distance; inf = background
    std::vector<glm::vec3> viewNormal;
    std::vector<glm::vec3> viewWorldPosition;
    std::vector<glm::vec3> viewAlbedo;
    std::vector<glm::vec3> viewEmissive;
    std::vector<std::uint8_t> viewFlags;    // bit0 unlit, bit1 background
    std::vector<std::uint32_t> viewSurfaceIndex;
    std::vector<glm::vec3> screenProbeRadiance; // per probe: persistent octahedral radiance
    std::vector<glm::vec3> screenProbeRadiancePrevious; // reserved for reset-on-move
    std::vector<float>     screenProbeSh2;      // per probe: 9 x 3
    std::vector<float>     screenProbeIrradiance; // per probe: 3 (cosine-lobed)
    std::vector<std::uint8_t> screenProbeValid;
    std::vector<glm::vec3> gatherIrradiance;    // per pixel indirect irradiance
    std::vector<glm::vec3> gatherIrradiancePrevious;
    std::vector<float>     previousViewDepth;
    std::vector<glm::vec3> composedImage;       // FinalImage/GiOnly share this

    // Octahedral direction tables: the world cache uses
    // directionResolution^2 directions (Lumen's 32x32 world probes); screen
    // probes use their own coarser 8x8 map (Lumen's screen probe resolution).
    std::vector<glm::vec3> probeDirections;
    std::vector<float>     probeDirectionSolidAngles;
    std::vector<glm::vec3> screenProbeDirections;

    // Statistics, accumulated by parallel stages through fetch_add.
    std::atomic<std::uint32_t> captureTexelsThisFrame{0};
    std::atomic<std::uint32_t> directTexelsThisFrame{0};
    std::atomic<std::uint32_t> indirectTexelsThisFrame{0};
    std::atomic<std::uint32_t> probeTracesThisFrame{0};
    std::atomic<std::uint32_t> screenProbesThisFrame{0};
    std::atomic<std::uint32_t> gatherRaysThisFrame{0};
    std::atomic<std::uint64_t> totalRaysTraced{0};
    std::atomic<std::uint32_t> framesSkippedStatic{0};
};

// ---- scene.cpp ------------------------------------------------------------
void buildSurfaceGeometry(GiSurfaceGeometry &geometry, const GiSurfaceDescription &description,
    std::uint64_t maxTriangles, GiError *error);
void buildSurfaceBvh(GiSurfaceGeometry &geometry);
void updateSurfaceWorldBounds(GiSurface &surface);
void placeSurfaceCards(GiPipelineState &state, GiSurface &surface, std::uint32_t surfaceIndex);
void buildProbeDirectionTable(GiPipelineState &state);
void buildProbeLevels(GiPipelineState &state);
void snapProbeLevels(GiPipelineState &state, const glm::vec3 &cameraPosition);
void captureCard(GiPipelineState &state, GiCard &card, const GiSurface &surface, std::uint32_t texelBegin,
    std::uint32_t texelEnd);

// One amortized work unit: a contiguous texel range of one card.
struct GiTexelWork
{
    std::uint32_t cardIndex  = 0;
    std::uint32_t texelBegin = 0;
    std::uint32_t texelEnd   = 0;
};

// ---- debug.cpp ------------------------------------------------------------
void rasterizeViewGBuffer(GiPipelineState &state);
void traceScreenProbes(GiPipelineState &state);
void resolvePixelIrradiance(GiPipelineState &state);
void composeDebugView(GiPipelineState &state, GiDebugView view, std::vector<std::uint8_t> &outRgb);
void referencePathTrace(GiPipelineState &state, std::vector<std::uint8_t> &outRgb);
bool exportDebugView(GiPipelineState &state, GiDebugView view, const char *filePath, GiError *error);

// ---- lighting.cpp ---------------------------------------------------------
void projectRadianceToSh2(const glm::vec3 *directions, const glm::vec3 *radiance, std::size_t count, float *outSh2);
bool sunShadowed(const GiPipelineState &state, const glm::vec3 &position, const glm::vec3 &normal,
    std::uint32_t excludedSurface);
void sampleSurfaceAlbedo(const GiPipelineState &state, std::uint32_t surfaceIndex, const glm::vec3 &position,
    const glm::vec3 &normal, glm::vec3 &outAlbedo);
void gatherDirectLightingChunk(GiPipelineState &state, const GiTexelWork *work, std::uint32_t count);
void gatherIndirectLightingChunk(GiPipelineState &state, const GiTexelWork *work, std::uint32_t count);
void updateProbeChunk(GiPipelineState &state, std::uint32_t levelIndex, std::uint32_t begin, std::uint32_t end);
void radianceAtHit(const GiPipelineState &state, const GiHit &hit, bool includeIndirect, glm::vec3 &outRadiance);
void sampleSurfaceCache(const GiPipelineState &state, std::uint32_t surfaceIndex, const glm::vec3 &position,
    const glm::vec3 &normal, bool includeIndirect, glm::vec3 &outRadiance);

} // namespace lumen_gi
} // namespace engine

#endif
