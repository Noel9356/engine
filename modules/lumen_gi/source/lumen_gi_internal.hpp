// Internal state of the lumen_gi module, shared by its translation units.
//
// The pipeline mirrors Lumen's stages scaled to a CPU implementation:
//
//   surfaces (registry)  ->  per-surface cards (6-face axis-aligned capture in
//   the surface's LOCAL space, Lumen's documented fallback placement, with
//   several depth layers so concave meshes keep their interior faces) ->
//   incremental card capture (albedo, normal, depth; a card is captured once
//   and again only when its surface changes) -> amortized direct lighting on
//   card texels -> indirect lighting propagation (cosine gathers reading
//   direct + the committed indirect radiance, one bounce per update, N+2
//   through feedback) -> world-space radiance cache (camera-centered
//   clipmap of octahedral probes with toroidal scrolling + SH2) -> optional
//   screen-space final gather (module debug view, on demand) -> composed
//   debug image.
//
// Rays trace the registered triangle scene through a top-level BVH over the
// surfaces and per-surface BVHs (rays transform into each surface's local
// space, so dynamic transforms never rebuild a surface BVH); shading at every
// hit goes through the card surface cache exactly like Lumen's card lookup.

#ifndef ENGINE_LUMEN_GI_INTERNAL_HPP
#define ENGINE_LUMEN_GI_INTERNAL_HPP

#include <engine/kernel/task_scheduler.hpp>
#include <engine/lumen_gi/capabilities.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/direct_lighting/capabilities.hpp>

#include <glm/geometric.hpp>
#include <glm/ext/matrix_float3x3.hpp>
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
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace engine
{
namespace lumen_gi
{

inline constexpr float kPi               = 3.14159265358979f;
inline constexpr float invalidCardDepth  = std::numeric_limits<float>::max();
inline constexpr std::uint32_t noSurface = 0xffffffffu;
// Fixed probe clip box edge (the renderer's fixed probe buffer holds 2 x 9^3).
inline constexpr std::uint32_t probeGridDimension = 9;
inline constexpr std::uint32_t maxCardLayers      = 4;

// ---- small shared helpers (hash, sampling, octahedral mapping) ---------------

inline std::uint64_t hashSeed(std::uint64_t seed)
{
    std::uint64_t hash = seed * 0x9E3779B97F4A7C15ull;
    hash ^= hash >> 30;
    hash *= 0xBF58476D1CE4E5B9ull;
    hash ^= hash >> 27;
    hash *= 0x94D049BB133111EBull;
    hash ^= hash >> 31;
    return hash;
}

// Uniform [0, 1) float; advances the seed.
inline float randomFloat(std::uint64_t &seed)
{
    seed = hashSeed(seed + 0x632BE59BD9B4E019ull);
    return static_cast<float>(seed >> 40) * (1.0f / 16777216.0f);
}

// Malley cosine-hemisphere sample around +Z from two uniform numbers.
inline glm::vec3 cosineHemisphere(float u, float v)
{
    const float radius = std::sqrt(std::max(0.0f, u));
    const float phi    = v * 2.0f * kPi;
    return glm::vec3(radius * std::cos(phi), radius * std::sin(phi), std::sqrt(std::max(0.0f, 1.0f - u)));
}

// Orthonormal basis with `normal` as the third column (Duff et al. 2017;
// branchless and free of the discontinuity of the older cross-product axis
// switch).
inline glm::mat3 makeBasis(const glm::vec3 &normal)
{
    const float     sign = normal.z >= 0.0f ? 1.0f : -1.0f;
    const float     a    = -1.0f / (sign + normal.z);
    const float     b    = normal.x * normal.y * a;
    const glm::vec3 tangent(1.0f + sign * normal.x * normal.x * a, sign * b, -sign * normal.x);
    const glm::vec3 bitangent(b, sign + normal.y * normal.y * a, -normal.y);
    return glm::mat3(tangent, bitangent, normal);
}

// Octahedral mapping (unit sphere <-> [-1, 1]^2). decode/encode are exact
// inverses of each other; every consumer of probe maps goes through these two
// so the direction tables and the lookups cannot disagree.
inline glm::vec3 octahedralDecode(float u, float v)
{
    glm::vec3 direction(u, v, 1.0f - std::fabs(u) - std::fabs(v));
    if (direction.z < 0.0f)
    {
        const float wrappedX = (1.0f - std::fabs(v)) * (u >= 0.0f ? 1.0f : -1.0f);
        const float wrappedY = (1.0f - std::fabs(u)) * (v >= 0.0f ? 1.0f : -1.0f);
        direction.x          = wrappedX;
        direction.y          = wrappedY;
    }
    const float length = glm::length(direction);
    return length > 1.0e-8f ? direction / length : glm::vec3(0.0f, 0.0f, 1.0f);
}

inline glm::vec2 octahedralEncode(const glm::vec3 &direction)
{
    const float     sum = std::max(std::fabs(direction.x) + std::fabs(direction.y) + std::fabs(direction.z), 1.0e-8f);
    glm::vec3       p   = direction / sum;
    if (p.z < 0.0f)
    {
        const float wrappedX = (1.0f - std::fabs(p.y)) * (p.x >= 0.0f ? 1.0f : -1.0f);
        const float wrappedY = (1.0f - std::fabs(p.x)) * (p.y >= 0.0f ? 1.0f : -1.0f);
        p.x                  = wrappedX;
        p.y                  = wrappedY;
    }
    return glm::vec2(p.x, p.y);
}

// Texel index of a direction in a resolution^2 octahedral map.
inline std::uint32_t octahedralTexelIndex(const glm::vec3 &direction, std::uint32_t resolution)
{
    const glm::vec2 uv = octahedralEncode(direction);
    const auto      toTexel = [resolution](float value)
    {
        const float scaled = (value * 0.5f + 0.5f) * static_cast<float>(resolution);
        return static_cast<std::uint32_t>(std::clamp(scaled, 0.0f, static_cast<float>(resolution) - 1.0f));
    };
    return toTexel(uv.y) * resolution + toTexel(uv.x);
}

// ---- settings ------------------------------------------------------------------

// Every configurable [lumen_gi] value; filled by the module from
// configuration and refreshed on live reload. The defaults below match
// config.toml and docs/configuration.md. See docs/configuration.md.
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
    std::uint32_t maxCardsPerSurface         = 12;
    float         texelsPerMeter             = 24.0f;
    std::uint32_t maxTexelsPerCardAxis       = 128;
    std::uint32_t cardDepthLayers            = 2;
    std::uint64_t captureTexelsPerFrame      = 262144;
    std::uint64_t directTexelsPerFrame       = 65536;
    std::uint64_t indirectTexelsPerFrame     = 65536;
    std::uint32_t indirectRaysPerTexel       = 4;
    bool          sunShadows                 = true;
    bool          pointLightShadows          = false;
    float         lightingUpdateSpeed        = 1.0f;
    float         indirectTemporalBlend      = 0.8f;
    float         maxTraceDistance           = 50.0f;
    float         probeSpacing               = 2.0f;
    std::uint32_t probeDirectionResolution   = 32;
    std::uint32_t probeUpdatesPerFrame       = 64;
    std::uint32_t probeRaysPerUpdate         = 256;
    float         probeHysteresis            = 0.2f;
    float         probeLevel1SpacingFactor   = 4.0f;
    float         screenRayDistance          = 2.0f;
    std::uint32_t gatherScreenScale          = 8;
    std::uint32_t gatherRaysPerProbe         = 16;
    bool          gatherImportanceSampling   = true;
    float         gatherTemporalBlend        = 0.85f;
    std::uint32_t gatherSpatialFilter        = 3;
    float         gatherPlaneDepthTolerance  = 0.35f;
    std::uint32_t debugInternalWidth         = 320;
    std::uint32_t debugInternalHeight        = 180;
    std::uint32_t referenceSamples           = 64;
    std::uint32_t referenceBounces           = 4;
    bool          debugIncludeAmbient        = false;
    bool          continuousGather           = false;
    std::uint32_t exportGatherFrames         = 16;
};

// ---- cards -----------------------------------------------------------------------

// One captured card: an axis-aligned ortho projection of its surface, placed
// in the surface's LOCAL space so a moving instance carries its cache with
// it. U/V span the card plane, W points outward; capture rays look along -W
// from slightly outside the near plane. Each (x, y) texel stores up to
// `layers` front-facing hits ordered by depth, so concave meshes (a room
// modelled as one mesh) keep faces that sit behind a nearer one. Per-texel
// arrays hold layers * texelWidth * texelHeight entries, layer-major.
struct GiCard
{
    glm::vec3     origin{0.0f};
    glm::vec3     axisU{1.0f, 0.0f, 0.0f};
    glm::vec3     axisV{0.0f, 1.0f, 0.0f};
    glm::vec3     axisW{0.0f, 0.0f, 1.0f};
    float         width    = 1.0f;
    float         height   = 1.0f;
    float         depth    = 1.0f;   // thickness of the bounds along -axisW (local units)
    float         nearBias = 1.0e-3f;
    std::uint32_t axisIndex = 2;     // local axis that axisW lies on (0..2)

    std::uint32_t texelWidth  = 4;
    std::uint32_t texelHeight = 4;
    std::uint32_t layers      = 1;

    // depthTexels is the distance from the card plane to the hit, along -W,
    // in local units (invalidCardDepth = no surface in that layer).
    std::vector<float>        depthTexels;
    std::vector<glm::vec3>    albedo;
    std::vector<glm::vec3>    normal;            // local-space smooth normal
    std::vector<glm::vec3>    directRadiance;    // Lumen "Lumen Scene" direct term (+ emission)
    std::vector<glm::vec3>    indirectRadiance;  // committed propagated bounce term
    std::vector<std::uint8_t> indirectSamples;   // updates seen (history ramp, saturates at 255)

    // Amortization progress. Capture is incremental (a cursor over the xy
    // plane); the lighting cursors rotate over all layered texels so every
    // texel is refreshed once per cycle regardless of the per-frame budget.
    std::uint32_t captureCursor    = 0;
    bool          captureComplete  = false;
    std::uint32_t validTexels      = 0;    // valid layered texels after capture
    std::uint32_t directCursor     = 0;
    std::uint32_t indirectCursor   = 0;
    std::uint64_t lastCaptureFrame  = 0;
    std::uint64_t lastDirectFrame   = 0;   // 0 = stale: relight first
    std::uint64_t lastIndirectFrame = 0;
    float         cameraDistance    = 0.0f;

    std::size_t planeTexels() const
    {
        return static_cast<std::size_t>(texelWidth) * texelHeight;
    }
    std::size_t totalTexels() const
    {
        return planeTexels() * layers;
    }
};

// ---- geometry ----------------------------------------------------------------------

// Triangle stored in BVH leaf order for cache-friendly traversal: the vertex
// and its two edges (Moller-Trumbore) plus the source triangle index.
struct GiPackedTriangle
{
    glm::vec3     v0{0.0f};
    glm::vec3     edge1{0.0f};
    glm::vec3     edge2{0.0f};
    std::uint32_t triangle = 0;
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
    // count = 0. Leaves: leftOrFirst = first entry in `packed`, count > 0.
    struct BvhNode
    {
        glm::vec3      boundsMin{0.0f};
        glm::vec3      boundsMax{0.0f};
        std::uint32_t  leftOrFirst = 0;
        std::uint32_t  count       = 0;
    };
    std::vector<BvhNode>          bvh;
    std::vector<GiPackedTriangle> packed;
    glm::vec3                     boundsMin{0.0f};
    glm::vec3                     boundsMax{0.0f};
};

struct GiSurface
{
    GiSurfaceHandle   handle = invalidGiSurface;
    GiSurfaceGeometry geometry;
    glm::mat4         transform{1.0f};
    glm::mat4         inverseTransform{1.0f};
    glm::mat3         linear{1.0f};       // upper 3x3 of transform
    glm::mat3         normalMatrix{1.0f}; // inverse-transpose of the upper 3x3
    glm::vec3         axisScale{1.0f};    // world length of each local axis
    glm::vec3         albedo{1.0f};
    glm::vec3         emissive{0.0f};
    std::uint32_t     flagBits     = 0;
    bool              excluded     = false;
    bool              unlit        = false;
    bool              dynamic      = false;
    glm::vec3         worldMin{0.0f};
    glm::vec3         worldMax{0.0f};
    std::vector<GiCard> cards;            // local-space cards of this surface
};

// Top-level acceleration structure over the surfaces' world bounds.
struct GiTlasNode
{
    glm::vec3     boundsMin{0.0f};
    glm::vec3     boundsMax{0.0f};
    std::uint32_t leftOrFirst = 0;
    std::uint32_t count       = 0;
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
    // True when the ray struck the back of the surface (both the smooth and
    // the geometric normal face away from the ray): Lumen's anti-leak rule
    // returns no radiance for those hits.
    bool          backface     = false;
    float         t            = 0.0f;
    std::uint32_t surfaceIndex = 0;
    std::uint32_t triangleIndex = 0;
    glm::vec3     position{0.0f};
    glm::vec3     normal{0.0f, 1.0f, 0.0f};   // world smooth normal
    glm::vec3     geometricNormal{0.0f, 1.0f, 0.0f}; // world, faces the ray
    glm::vec3     localPosition{0.0f};        // hit point in the surface's local space
    glm::vec3     localNormal{0.0f, 1.0f, 0.0f};     // local smooth normal
};

struct GiPipelineState;

// Ray-primitive core shared by every stage. Implemented in
// lumen_gi_scene.cpp; parallel stages call these freely (all inputs are
// immutable while stages run).
bool rayTriangle(const GiRay &ray, const GiPackedTriangle &triangle, float &outT, glm::vec3 &outBarycentric);

// Closest-hit trace against every non-excluded surface; tMax limits the ray.
GiHit traceScene(const GiPipelineState &state, const GiRay &ray);
// Any-hit test used for shadows; returns true when geometry blocks the ray.
bool traceSceneOccluded(const GiPipelineState &state, const GiRay &ray, std::uint32_t excludedSurface);

// ---- probes ------------------------------------------------------------------------

struct GiProbeSnapshot
{
    GiProbeHeader          header{};
    std::vector<GiProbe>   probes;
    std::uint64_t          version = 0;
};

// One camera-centered clip level. The lattice scrolls toroidally: a probe's
// slot is its integer lattice cell modulo the dimension, so crossing a cell
// only recycles the one plane of probes entering the box instead of
// invalidating every probe (the old "re-anchor everything" behavior).
struct GiProbeLevel
{
    struct Probe
    {
        glm::ivec3             cell{0};          // lattice cell this slot currently holds
        glm::vec3              position{0.0f};
        std::vector<glm::vec3> radiance;         // octahedral map (directionResolution^2)
        std::array<float, 27>  sh2{};            // 9 x 3, basis order of GiProbe
        float                  validity   = 0.0f;
        std::uint64_t          lastUpdate = 0;
        std::uint32_t          phase      = 0;   // rotating ray stratum
        bool                   fresh      = true;  // no data yet: next update traces every direction
        bool                   enclosed   = false; // sits inside geometry (mostly backface hits)
    };
    float                        spacing = 2.0f;
    std::array<std::uint32_t, 3> dimensions{0u, 0u, 0u};
    glm::ivec3                   originCell{0};   // lattice cell of logical probe (0, 0, 0)
    glm::vec3                    anchor{0.0f};    // originCell * spacing
    std::vector<Probe>           probes;          // slot-indexed (toroidal)
    std::uint32_t                updateCursor = 0;

    std::uint32_t dimension() const
    {
        return dimensions[0];
    }
    // Slot holding lattice cell `cell` (valid for cells inside the box).
    std::uint32_t slotOfCell(const glm::ivec3 &cell) const
    {
        const std::int32_t dim = static_cast<std::int32_t>(dimensions[0]);
        const auto wrap = [dim](std::int32_t value) { return ((value % dim) + dim) % dim; };
        return static_cast<std::uint32_t>((wrap(cell.z) * dim + wrap(cell.y)) * dim + wrap(cell.x));
    }
    // Slot of the logical probe (x, y, z) in [0, dim)^3.
    std::uint32_t slotOfLogical(std::uint32_t x, std::uint32_t y, std::uint32_t z) const
    {
        return slotOfCell(originCell + glm::ivec3(static_cast<int>(x), static_cast<int>(y), static_cast<int>(z)));
    }
};

// ---- pipeline state ---------------------------------------------------------------

struct GiCardRef
{
    std::uint32_t surface = 0;
    std::uint32_t card    = 0;
};

// One amortized work unit: a contiguous texel range of one card (flat card
// index into GiPipelineState::cardRefs).
struct GiTexelWork
{
    std::uint32_t cardIndex  = 0;
    std::uint32_t texelBegin = 0;
    std::uint32_t texelEnd   = 0;
};

// The whole pipeline state owned by the module instance. Mutators (surface
// registry, setView, evaluate, config) hold the module mutex; stage tasks
// touch disjoint buffers and read only state frozen before they run.
struct GiPipelineState
{
    GiSettings settings;

    // Registry (mirrors the module's registry under the module mutex).
    std::vector<GiSurface>   surfaces;
    std::vector<GiCardRef>   cardRefs;        // flat view over every surface's cards
    std::uint32_t            registryCapacity = 256;
    GiSurfaceHandle          nextHandle       = 1;
    std::uint64_t            frameIndex       = 0;

    // Top-level BVH over the non-excluded surfaces (rebuilt lazily).
    std::vector<GiTlasNode>    tlas;
    std::vector<std::uint32_t> tlasSurfaces;
    bool                       tlasDirty = true;

    // Camera + environment snapshot for the running frame.
    bool      hasView   = false;
    glm::mat4 viewProjection{1.0f};
    glm::vec3 cameraPosition{0.0f};
    // Static-frame skip change detection: setView marks viewDirty when the
    // passed matrices differ from the previous call, surface mutators mark
    // sceneDirty, and evaluate() consumes both. framesSinceDirty counts
    // consecutive non-dirty evaluates; convergenceFrames is how many such
    // frames the caches need to settle (derived from the budgets and the
    // amount of cached texels / probes).
    bool          viewDirty  = true;
    bool          sceneDirty = true;
    bool          hasPreviousView = false;
    std::uint32_t framesSinceDirty = 0;
    std::uint32_t convergenceFrames = 0;
    GiEnvironmentSnapshot environment;
    bool        environmentValid = false;
    std::uint64_t environmentVersion = 0;
    std::uint64_t directLightsVersion = 0;

    // World-space radiance cache: two camera-centered clip levels.
    GiProbeLevel level0;
    GiProbeLevel level1;

    // Octahedral direction tables: the world cache uses
    // directionResolution^2 directions (Lumen's 32x32 world probes); screen
    // probes use their own coarser 8x8 map (Lumen's screen probe resolution).
    // Weights are the exact octahedral solid angles, normalized to 4 pi.
    std::vector<glm::vec3> probeDirections;
    std::vector<float>     probeDirectionWeights;
    std::vector<glm::vec3> screenProbeDirections;
    std::vector<float>     screenProbeDirectionWeights;

    // Screen-space final gather (debug view) buffers at internal resolution.
    std::uint32_t viewWidth  = 0;
    std::uint32_t viewHeight = 0;
    std::uint32_t screenProbeWidth  = 0;
    std::uint32_t screenProbeHeight = 0;
    std::vector<float>     viewDepth;       // distance to the camera; max = background
    std::vector<glm::vec3> viewNormal;
    std::vector<glm::vec3> viewWorldPosition;
    std::vector<glm::vec3> viewAlbedo;
    std::vector<glm::vec3> viewEmissive;
    std::vector<std::uint8_t> viewFlags;    // bit0 unlit
    std::vector<std::uint32_t> viewSurfaceIndex;
    std::vector<glm::vec3> screenProbeRadiance;   // per probe: persistent octahedral radiance
    std::vector<glm::vec3> screenProbePosition;   // per probe: where its map was last traced
    std::vector<std::uint32_t> screenProbePixel;  // per probe: pixel index of the placement
    std::vector<float>     screenProbeSh2;        // per probe: 9 x 3
    std::vector<std::uint8_t> screenProbeValid;
    std::vector<glm::vec3> gatherIrradiance;      // per pixel indirect irradiance
    std::vector<glm::vec3> gatherIrradiancePrevious;
    // Temporal history of the final gather (self-contained: the gather runs on
    // demand, so it keeps its own previous camera).
    bool          gatherHasHistory = false;
    glm::mat4     gatherPreviousViewProjection{1.0f};
    std::uint32_t gatherPreviousWidth  = 0;
    std::uint32_t gatherPreviousHeight = 0;
    std::vector<float> gatherPreviousDepth;
    std::uint64_t gatherFrame = 0;
    std::uint32_t gatherHistoryLength = 0; // accumulated gather passes (history ramp-up)

    // Statistics, accumulated by parallel stages through fetch_add.
    std::atomic<std::uint32_t> captureTexelsThisFrame{0};
    std::atomic<std::uint32_t> directTexelsThisFrame{0};
    std::atomic<std::uint32_t> indirectTexelsThisFrame{0};
    std::atomic<std::uint32_t> indirectRaysThisFrame{0};
    std::atomic<std::uint32_t> probeTracesThisFrame{0};
    std::atomic<std::uint32_t> screenProbesThisFrame{0};
    std::atomic<std::uint32_t> gatherRaysThisFrame{0};
    std::atomic<std::uint64_t> totalRaysTraced{0};
    std::atomic<std::uint32_t> framesSkippedStatic{0};

    GiCard &card(std::uint32_t flatIndex)
    {
        const GiCardRef reference = cardRefs[flatIndex];
        return surfaces[reference.surface].cards[reference.card];
    }
    const GiCard &card(std::uint32_t flatIndex) const
    {
        const GiCardRef reference = cardRefs[flatIndex];
        return surfaces[reference.surface].cards[reference.card];
    }
    const GiSurface &cardSurface(std::uint32_t flatIndex) const
    {
        return surfaces[cardRefs[flatIndex].surface];
    }
};

// ---- parallel helper --------------------------------------------------------------

// Runs fn(begin, end) over [0, count) in chunks of `grain` with dynamic
// (atomic-counter) scheduling, so uneven ray-tracing costs balance across
// workers. The calling thread participates; with no running scheduler (or a
// small range) everything runs inline. Rethrows the first exception.
template <typename Fn>
void parallelRange(TaskScheduler *scheduler, std::size_t count, std::size_t grain, Fn &&fn)
{
    if (count == 0)
    {
        return;
    }
    grain = std::max<std::size_t>(grain, 1);
    if (scheduler == nullptr || !scheduler->isRunning() || count <= grain)
    {
        fn(static_cast<std::size_t>(0), count);
        return;
    }
    const std::size_t         chunks = (count + grain - 1) / grain;
    std::atomic<std::size_t>  next{0};
    std::exception_ptr        failure;
    std::mutex                failureMutex;
    auto runner = [&]()
    {
        try
        {
            for (;;)
            {
                const std::size_t begin = next.fetch_add(grain, std::memory_order_relaxed);
                if (begin >= count)
                {
                    break;
                }
                fn(begin, std::min(begin + grain, count));
            }
        }
        catch (...)
        {
            std::lock_guard<std::mutex> lock(failureMutex);
            if (!failure)
            {
                failure = std::current_exception();
            }
            next.store(count, std::memory_order_relaxed);
        }
    };
    const std::size_t helpers = std::min<std::size_t>(scheduler->statistics().workerCount, chunks - 1);
    TaskGroup         group;
    for (std::size_t helper = 0; helper < helpers; ++helper)
    {
        scheduler->schedule(group, runner);
    }
    runner();
    group.wait(*scheduler);
    if (failure)
    {
        std::rethrow_exception(failure);
    }
}

// ---- scene.cpp -------------------------------------------------------------------

void buildSurfaceGeometry(GiSurfaceGeometry &geometry, const GiSurfaceDescription &description,
    std::uint64_t maxTriangles, GiError *error);
void buildSurfaceBvh(GiSurfaceGeometry &geometry);
void updateSurfaceTransform(GiSurface &surface, const glm::mat4 &transform);
void updateSurfaceWorldBounds(GiSurface &surface);
// Rebuilds the card set of one surface (drops its captured data).
void placeSurfaceCards(const GiSettings &settings, GiSurface &surface);
// Restarts capture + lighting of every card of one surface.
void invalidateSurfaceCards(GiSurface &surface);
void rebuildCardRefs(GiPipelineState &state);
void ensureSceneTlas(GiPipelineState &state);
void buildProbeDirectionTable(GiPipelineState &state);
// (Re)allocates both probe levels from the current settings.
void buildProbeLevels(GiPipelineState &state);
// Scrolls both clip levels to the camera, recycling only the probes whose
// lattice cell changed.
void snapProbeLevels(GiPipelineState &state, const glm::vec3 &cameraPosition);
void buildProbeSnapshot(const GiPipelineState &state, GiProbeSnapshot &snapshot);

// Captures xy texels [texelBegin, texelEnd) of one card (all layers).
void captureCardTexels(const GiSurface &surface, GiCard &card, std::uint32_t texelBegin, std::uint32_t texelEnd);
// Local-space position / world-space normal of one layered card texel.
glm::vec3 cardTexelLocalPosition(const GiCard &card, std::uint32_t flatTexel);

// ---- pipeline.cpp ------------------------------------------------------------------

// Runs one amortized GI frame: capture, direct, indirect + probe stages.
// Blocks until the stages finish. Returns false when a stage failed.
bool runGiFrame(GiPipelineState &state, TaskScheduler *scheduler);

// ---- debug.cpp ---------------------------------------------------------------------

// Rasterizes the module's GBuffer for the current view (always) and, when
// gatherPasses > 0, runs that many screen-probe + resolve passes so the
// temporal accumulation converges.
void prepareDebugBuffers(GiPipelineState &state, TaskScheduler *scheduler, std::uint32_t gatherPasses);
void composeDebugView(
    GiPipelineState &state, TaskScheduler *scheduler, GiDebugView view, std::vector<std::uint8_t> &outRgb);
void referencePathTrace(GiPipelineState &state, TaskScheduler *scheduler, std::vector<std::uint8_t> &outRgb);
bool exportDebugView(
    GiPipelineState &state, TaskScheduler *scheduler, GiDebugView view, const char *filePath, GiError *error);

// ---- lighting.cpp ------------------------------------------------------------------

// Projects a radiance array onto the 9-coefficient SH2 basis (GiProbe order,
// pre-multiplied by the basis normalization constants so evaluateSh2Irradiance
// is a plain polynomial) using per-direction solid-angle weights, then
// suppresses ringing so the irradiance never goes negative.
void projectRadianceToSh2(
    const glm::vec3 *directions, const glm::vec3 *radiance, const float *weights, std::size_t count, float *outSh2);
glm::vec3 directLightRadiance(const GiPipelineState &state, const glm::vec3 &position, const glm::vec3 &normal);
bool sunShadowed(const GiPipelineState &state, const glm::vec3 &position, const glm::vec3 &normal);

// Surface cache lookups at a point given in the surface's LOCAL space.
void sampleSurfaceCacheLocal(const GiPipelineState &state, const GiSurface &surface, const glm::vec3 &localPosition,
    const glm::vec3 &localNormal, bool includeIndirect, glm::vec3 &outRadiance);
void sampleSurfaceAlbedoLocal(const GiPipelineState &state, const GiSurface &surface, const glm::vec3 &localPosition,
    const glm::vec3 &localNormal, glm::vec3 &outAlbedo);
// World-space convenience wrappers (debug views).
void sampleSurfaceCache(const GiPipelineState &state, std::uint32_t surfaceIndex, const glm::vec3 &position,
    const glm::vec3 &normal, bool includeIndirect, glm::vec3 &outRadiance);
void sampleSurfaceAlbedo(const GiPipelineState &state, std::uint32_t surfaceIndex, const glm::vec3 &position,
    const glm::vec3 &normal, glm::vec3 &outAlbedo);
void radianceAtHit(const GiPipelineState &state, const GiHit &hit, bool includeIndirect, glm::vec3 &outRadiance);

void gatherDirectLightingUnit(GiPipelineState &state, const GiTexelWork &unit);
// Writes the new indirect radiance of the unit's texels into `scratch`
// (one entry per texel of the unit); commitIndirectUnit copies it over the
// cache after every stage task finished, so concurrent traces never observe a
// half-updated cache.
void gatherIndirectLightingUnit(GiPipelineState &state, const GiTexelWork &unit, glm::vec3 *scratch);
void commitIndirectUnit(GiPipelineState &state, const GiTexelWork &unit, const glm::vec3 *scratch);
void updateProbe(GiPipelineState &state, std::uint32_t levelIndex, std::uint32_t slot);

// Trilinear interpolation of the world radiance cache at a world position:
// returns false when the position lies outside the level or no probe around
// it holds valid data.
bool worldCacheLevelRadiance(const GiPipelineState &state, const GiProbeLevel &level, const glm::vec3 &position,
    const glm::vec3 &direction, glm::vec3 &outRadiance);
// SH2 irradiance of the world cache at a position/normal; levels are tried
// from the finest to the coarsest. Returns false when no level covers it.
bool worldCacheIrradiance(
    const GiPipelineState &state, const glm::vec3 &position, const glm::vec3 &normal, glm::vec3 &outIrradiance);

} // namespace lumen_gi
} // namespace engine

#endif
