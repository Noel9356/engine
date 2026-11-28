// Internal state of the radiance_cache module, shared by its translation units.
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

#pragma once

#include <cstddef>
#include <engine/direct_lighting/capabilities.hpp>
#include <engine/kernel/task_scheduler.hpp>
#include <engine/radiance_cache/capabilities.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/renderer/lighting_cpu_pipeline.hpp>
#include <engine/renderer/lighting_ray_core.hpp>

#include <exception>
#include <glm/ext/matrix_float3x3.hpp>
#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>

namespace engine
{
namespace radiance_cache
{

inline constexpr float invalidCardDepth  = std::numeric_limits<float>::max();
inline constexpr std::uint32_t noSurface = 0xffffffffu;
// Default probe clip box edge (probes.clip_dimension; odd so a probe sits
// exactly on the camera axis). Consumers read the lattice extent from the
// exported probe header, so a configured dimension flows to every walker.
inline constexpr std::uint32_t probeGridDimension = 9;
inline constexpr std::uint32_t maxCardLayers      = 4;
// Octahedral map resolution bounds of the world probes (the documented
// config range; the exporter stores the resolution in the probe header).
inline constexpr std::uint32_t minProbeDirectionResolution = 4;
inline constexpr std::uint32_t maxProbeDirectionResolution = 64;
// Screen probes keep Lumen's fixed 8x8 directional map.
inline constexpr std::uint32_t screenProbeResolution       = 8;

// ---- small shared helpers ----------------------------------------------------

// The RNG, sampling frames, and intersection kernels are shared with
// reflection_field through engine/renderer/lighting_ray_core.hpp so both
// modules draw from the same distributions and accept the same hits.
using engine::lighting::kPi;
using engine::lighting::hashSeed;
using engine::lighting::randomFloat;
using engine::lighting::makeBasis;
using engine::lighting::cosineHemisphere;
using engine::lighting::safeInverseDirection;
using engine::lighting::slabTest;
using engine::lighting::rayTriangle;
using engine::lighting::qualityScale;
using engine::lighting::parallelRange;
using engine::lighting::toneMapColor;
using engine::lighting::toByte;
using engine::lighting::gammaEncode;
using engine::lighting::storePixel;

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

// Every configurable [radiance_cache] value; filled by the module from
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
    // Total layered texels one surface's card set may hold: placeSurfaceCards
    // lowers the texel density until the set fits, bounding the per-surface
    // memory a single registered mesh can claim (cards.max_texels_per_surface).
    std::uint64_t maxTexelsPerSurface        = 524288;
    std::uint64_t captureTexelsPerFrame      = 262144;
    std::uint64_t directTexelsPerFrame       = 65536;
    std::uint64_t indirectTexelsPerFrame     = 65536;
    std::uint32_t indirectRaysPerTexel       = 1;
    bool          sunShadows                 = true;
    bool          pointLightShadows          = false;
    float         lightingUpdateSpeed        = 1.0f;
    // Wall-clock budget for the whole evaluate (lighting.update_budget_milliseconds).
    // 0 keeps the fixed per-frame texel budgets; a positive value engages
    // the adaptive governor: the measured evaluate cost feeds back into the
    // direct/indirect texel budgets (bounded multiplicative steps), so the
    // cache refreshes at whatever rate THIS machine can afford instead of a
    // configured texel count that only fits some hardware. The configured
    // budgets (after quality/update_speed scaling) remain the ceiling.
    double        lightingUpdateBudgetMilliseconds = 0.0;
    float         indirectTemporalBlend      = 0.85f;
    float         maxTraceDistance           = 50.0f;
    // Surface offset applied to shadow, gather, and probe rays (see
    // trace.bias_m). One bias behind every trace site keeps the acne/leak
    // trade-off a single tuning decision.
    float         traceBias                  = 0.02f;
    // How many interfaces a gather/probe/shadow ray may cross through
    // transparent or translucent surfaces (trace.max_transmission_segments).
    // Rays past the budget treat the next opaque surface as blocking.
    std::uint32_t maxTransmissionSegments      = 3;
    float         probeSpacing               = 2.0f;
    // Clip box edge per axis of both probe levels (probes.clip_dimension,
    // odd): each level holds dimension^3 probes. Structural - a change
    // rebuilds the levels; see rebuildPipelineAfterConfigChangeLocked.
    std::uint32_t probeClipDimension       = probeGridDimension;
    std::uint32_t probeDirectionResolution   = 32;
    std::uint32_t probeUpdatesPerFrame       = 64;
    std::uint32_t probeRaysPerUpdate         = 256;
    float         probeHysteresis            = 0.2f;
    // Near-surface clearance a relocated probe keeps off the geometry it was
    // pushed away from, as a fraction of the level spacing (see
    // probes.relocation_margin). Also the margin added when clearing a slab.
    float         probeRelocationMargin      = 0.05f;
    // Dynamic relight: probes whose lattice position falls inside a moved or
    // re-created surface's swept world volume, expanded by this radius (see
    // probes.dynamic_relight_radius_m), retrace from scratch so moving or
    // deforming geometry does not linger in the world radiance cache for a
    // full stratum cycle. 0 disables the marking.
    float         dynamicRelightRadius      = 2.0f;
    float         probeLevel1SpacingFactor   = 4.0f;
    // Screen-space final gather rays resolve their full local neighborhood:
    // the distance must reach every surface that shades the traced point (a
    // room's far wall, an outdoor occluder) so a ray's card hit and its
    // world-cache fallback agree with the neighbors' - a reach that stops at
    // the first few meters mixes hit radiance and cache radiance per probe
    // and shows up as low-frequency patches on indirect-lit surfaces.
    float         screenRayDistance          = 20.0f;
    std::uint32_t gatherScreenScale          = 8;
    std::uint32_t gatherRaysPerProbe         = 32;
    bool          gatherImportanceSampling   = true;
    float         gatherTemporalBlend        = 0.85f;
    // Per-probe accumulation weight (see gather.probe_temporal_blend): the
    // weight of a probe direction texel's previous radiance when the pass
    // retraces it, so a probe's SH2 field converges over passes instead of
    // re-showing each pass' single-sample noise through the interpolation.
    float         gatherProbeTemporalBlend   = 0.9f;
    std::uint32_t gatherSpatialFilter        = 3;
    float         gatherPlaneDepthTolerance  = 0.35f;
    // Probe-interpolation footprint radius in probe cells (see
    // gather.interpolation_radius): the pixel loop blends the
    // (2r+1)x(2r+1) neighborhood's SH2 under the depth-plane and
    // normal-alignment gates, so neighboring probes' accumulated traces
    // average before the shading sees them.
    std::uint32_t gatherInterpolationRadius = 2;
    // The exported screen-space gather field (the "radiance_cache.gather"
    // capability): field_enabled gates the per-evaluate pass, field_scale
    // divides the pushed viewport resolution (2 halves each axis; the
    // interpolation and filters upscale it back), and the viewport itself
    // lives in the pipeline state - the field only runs once a consumer
    // pushed one, so module hosts without a gather renderer never pay it.
    bool          gatherFieldEnabled         = true;
    std::uint32_t gatherFieldScale           = 4;
    // Caustics (see [radiance_cache.caustics]): photon-differential splatting of
    // light focused by transmissive surfaces into the card surface cache.
    bool          causticsEnabled            = true;
    std::uint32_t causticsPhotonsPerFrame     = 1024;
    float         causticsTemporalBlend       = 0.9f;
    float         causticsFootprintMinM       = 0.02f;
    float         causticsFootprintMaxM       = 0.75f;
    bool          causticsEmissiveSources     = true;
    std::uint32_t debugInternalWidth         = 320;
    std::uint32_t debugInternalHeight        = 180;
    std::uint32_t referenceSamples           = 64;
    std::uint32_t referenceBounces           = 4;
    bool          debugIncludeAmbient        = false;
    bool          continuousGather           = false;
    std::uint32_t exportGatherFrames         = 16;
    // Diagnostic dump of the exported gather field (debug.export_field_prefix):
    // when non-empty, every publish writes <prefix>_irradiance/_occlusion/_validity.ppm
    // (overwritten each publish, so the final files are the last publish).
    std::string   debugExportFieldPrefix;
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
    // Caustic irradiance deposited by the photon pass: causticIrradiance is
    // the temporal-blended state (energy per texel, NOT yet multiplied by the
    // texel albedo / pi) and causticRadiance the cached outgoing radiance the
    // surface-cache reads blend in with the direct term (albedo / pi * E),
    // refreshed whenever the state changes so the read paths never pay the
    // conversion. causticSamples counts the blend updates (history ramp).
    std::vector<glm::vec3>    causticIrradiance;
    std::vector<glm::vec3>    causticRadiance;
    std::vector<std::uint8_t> causticSamples;
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

struct GatherClipVertex
{
    glm::vec4 clip{0.0f};
    glm::vec3 world{0.0f};
    glm::vec3 bary{0.0f};
};

struct GatherScreenVertex
{
    glm::vec2 pixel{0.0f};
    float     inverseW = 0.0f;
    glm::vec3 world{0.0f};
    glm::vec3 bary{0.0f};
};

struct GatherScreenTriangle
{
    GatherScreenVertex vertex[3];
    std::uint32_t surface  = 0;
    std::uint32_t triangle = 0;
};

struct GiSurface
{
    GiSurfaceHandle   handle = invalidGiSurface;
    GiSurfaceGeometry geometry;
    glm::mat4         transform{1.0f};
    glm::mat4         inverseTransform{1.0f};
    glm::mat3         linear{1.0f};       // upper 3x3 of transform
    glm::mat3         normalMatrix{1.0f}; // inverse-transpose of the upper 3x3
    // Cached pieces of the forward and inverse transforms so the per-ray and
    // per-texel hot loops multiply a mat3 and add a translation instead of
    // building a mat3 view of a mat4 and multiplying a full 4x4 by a w=1
    // point on every use. Filled together with the matrices above by
    // updateSurfaceTransform; identical arithmetic, hoisted once.
    glm::vec3         translation{0.0f};        // column 3 of transform
    glm::mat3         inverseLinear{1.0f};      // upper 3x3 of inverseTransform
    glm::vec3         inverseTranslation{0.0f}; // column 3 of inverseTransform
    // Exact-identity transform: local space coincides with world space, so
    // hot paths skip the per-ray / per-texel local<->world arithmetic (the
    // world-space values are already the local ones). Set together with the
    // matrices by updateSurfaceTransform.
    bool              localIsWorld = true;
    glm::vec3         axisScale{1.0f};    // world length of each local axis
    glm::vec3         albedo{1.0f};
    glm::vec3         emissive{0.0f};
    // Transmission pair (see GiSurfaceDescription): gather, probe, and
    // shadow rays transmit through surfaces with opacity below 1.
    float             opacity      = 1.0f;
    float             indexOfRefraction = 1.5f;
    // True while the caustics pass owns this surface's transmitted light
    // (opacity below 1): the direct, gather, and probe rays then treat it
    // as a stopper so its focused share is counted exactly once - by the
    // photon deposits. Maintained with the caustics transmitter list at
    // frame-plan time (see runGiFrame).
    bool              causticTransmitter = false;
    // The photon pass deposited (or holds) caustic state on this surface's
    // cards: the gather's full-res pass samples the caustic irradiance only
    // for receiver surfaces, so caustic-free scenes (the common case) pay
    // nothing. Cleared together with the card caustic arrays.
    bool              causticReceiver = false;
    std::uint32_t     flagBits     = 0;
    bool              excluded     = false;
    bool              unlit        = false;
    bool              dynamic      = false;
    glm::vec3         worldMin{0.0f};
    glm::vec3         worldMax{0.0f};
    std::vector<GiCard> cards;            // local-space cards of this surface

    // Gather-view raster caches (the reflection field's pattern): the
    // surface's projected screen triangles are rebuilt only when the gather
    // raster serial changes (view / registry / field size) or the surface's
    // exact transform changes; the world positions follow the transform
    // alone, the clip positions the transform and the view.
    std::uint64_t              gatherRasterSerial = 0;
    std::uint64_t              gatherTransformBits = 0;
    std::vector<glm::vec3>     gatherWorldPositions;
    std::vector<glm::vec4>     gatherClipPositions;
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
struct GiEnvironmentSnapshot : engine::lighting::EnvironmentCore
{
    // Direct light with the per-light constants the shading loop needs,
    // shared with reflection_field through engine/renderer/lighting_ray_core.hpp
    // (normalized direction, cone cosines) prepared at snapshot time, so
    // per-texel evaluation does not re-normalize or re-trigonometrise every
    // frame. Inactive lights (zero intensity or black color) are dropped.
    // The sun, skylight, and tone-mapping fields come from the shared
    // EnvironmentCore so the two lumen modules shade the environment
    // identically.
    using PreparedDirectLight = engine::lighting::PreparedLight;

    bool      hasDirectLights  = false;
    std::vector<direct_lighting::DirectLight> directLights;
    std::vector<PreparedDirectLight>          preparedLights;
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
// radiance_cache_scene.cpp; parallel stages call these freely (all inputs are
// immutable while stages run).

// Lean closest-hit payload for the per-frame GI passes. The surface-cache
// shading (radianceAtHit) reads only the hit's local-space position, its
// local normal, and the backface flag, so the gather, the probe updates, and
// the screen-space final gather trace through this cheaper hit instead of
// the full GiHit: the world-space normal chain (two normal-matrix products,
// three normalizations, and the geometric-normal orientation fix-up per
// hit) never runs on those rays. The debug views keep the full traceScene.
// t is the world-space hit distance (the probe relocation rules need it).
struct GiShadingHit
{
    bool          hit          = false;
    bool          backface     = false;
    std::uint32_t surfaceIndex = 0;
    float         t            = 0.0f;
    glm::vec3     localPosition{0.0f};
    glm::vec3     localNormal{0.0f, 1.0f, 0.0f};
};

// Closest hit against every non-excluded surface, carrying only the fields
// the surface-cache shading needs (see GiShadingHit).
GiShadingHit traceSceneShading(const GiPipelineState &state, const GiRay &ray);
// Transmittance (0..1) of a shadow ray through the scene: transparent
// surfaces multiply the remaining light by (1 - opacity) per covering hit
// instead of blocking it; the first opaque surface (or an exhausted
// transmission budget) blocks.
float traceSceneTransmittance(const GiPipelineState &state, const GiRay &ray);

// Closest-hit trace against every non-excluded surface; tMax limits the ray.
GiHit traceScene(const GiPipelineState &state, const GiRay &ray);

// ---- probes ------------------------------------------------------------------------

struct GiProbeSnapshot
{
    GiProbeHeader          header{};
    std::vector<GiProbe>   probes;
    std::uint64_t          version = 0;
};

// Published gather-field snapshot (the copyIrradianceField read side): the
// filtered irradiance and validity packed as radianceGatherTexelFloats floats
// per texel, plus the header and version of the pass that resolved it. Copy
// side and evaluation publish under the same separate snapshot mutex the
// probe snapshot uses, so a renderer polling every frame never waits for a
// running evaluate().
struct GiGatherSnapshot
{
    RadianceGatherHeader   header{};
    std::vector<float>     field;
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
        // DDGI-style relocation offset (Majercik et al. 2019): probes inside
        // geometry or grazing a surface are nudged off it so their rays see
        // front faces and the exported validity recovers. The offset moves
        // only the ray origin; the exported lattice stays regular.
        glm::vec3              offset{0.0f};
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

// Priority key ordering cards into the lighting budgets: stale cards first,
// then nearer ones.
struct GiCardOrderKey
{
    std::uint64_t ageFrames;
    float         cameraDistance;
    std::uint32_t index;
};

// One probe update queued for the frame.
struct GiProbeItem
{
    std::uint32_t level;
    std::uint32_t slot;
};

// The whole pipeline state owned by the module instance. Mutators (surface
// registry, setView, evaluate, config) hold the module mutex; stage tasks
// touch disjoint buffers and read only state frozen before they run.
struct GiPipelineState
{
    GiSettings settings;

    // Adaptive budget governor (settings.lightingUpdateBudgetMilliseconds):
    // the ceilings are the quality/update_speed-scaled budgets
    // applySettingsToState last resolved, and the scale adapts toward the
    // wall-clock budget in bounded multiplicative steps (see evaluateImpl's
    // feedback block). 1.0 = run at the full configured budgets.
    std::uint64_t captureTexelCeiling   = 0;
    std::uint64_t directTexelCeiling    = 0;
    std::uint64_t indirectTexelCeiling  = 0;
    std::uint32_t probeUpdateCeiling    = 0;
    std::uint32_t causticsPhotonCeiling = 0;
    std::uint32_t gatherScreenScaleCeiling = 8;
    std::uint32_t gatherRayCeiling      = 0;
    double        governorScale         = 1.0;
    double        governorMeasuredMs    = 0.0;

    // Registry (mirrors the module's registry under the module mutex).
    std::vector<GiSurface>   surfaces;
    std::vector<GiCardRef>   cardRefs;        // flat view over every surface's cards
    // Flat-texel prefix over cardRefs (rebuilt with cardRefs): cardTexelOffsets
    // holds card i's first texel index and totalCardTexels the grand total,
    // so the caustics staging buffers index texels across cards directly.
    std::vector<std::uint64_t> cardTexelOffsets;
    std::uint64_t              totalCardTexels = 0;
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

    // Gather-view raster caches: `gatherRasterSerial` retires every
    // per-surface projected-triangle cache when the view, the registry, or
    // the field size changes (the reflection field's rasterSerial pattern).
    std::uint64_t                          gatherRasterSerial = 1;
    std::uint32_t                          gatherRasterWidth  = 0;
    std::uint32_t                          gatherRasterHeight = 0;
    glm::mat4                              gatherRasterViewProjection{1.0f};
    bool                                   gatherRasterViewValid = false;
    std::vector<std::vector<GatherScreenTriangle>> gatherRasterTriangles;
    std::vector<GatherScreenTriangle>      gatherRasterConcat;
    std::vector<std::vector<std::uint32_t>> gatherRasterBins;

    // World-space radiance cache: two camera-centered clip levels.
    GiProbeLevel level0;
    GiProbeLevel level1;

    // Octahedral direction tables: the world cache uses
    // directionResolution^2 directions (Lumen's 32x32 world probes); screen
    // probes use their own coarser 8x8 map (Lumen's screen probe resolution).
    // Weights are the exact octahedral solid angles, normalized to 4 pi. The
    // basis tables hold the SH2 projection weight of each direction
    // (normalization constant squared times the basis polynomial, see
    // projectRadianceToSh2), precomputed so per-probe projections skip the
    // per-direction trigonometry.
    std::vector<glm::vec3> probeDirections;
    std::vector<float>     probeDirectionWeights;
    std::vector<float>     probeDirectionBasis;   // directions.size() * 9
    std::vector<glm::vec3> screenProbeDirections;
    std::vector<float>     screenProbeDirectionWeights;
    std::vector<float>     screenProbeDirectionBasis;

    // Screen-space final gather buffers at the resolved view resolution
    // (viewport / gatherFieldScale when a consumer pushed one, the debug
    // internal size otherwise; the exported field and the debug views read
    // the same buffers).
    std::uint32_t viewWidth  = 0;
    std::uint32_t viewHeight = 0;
    // Viewport pushed through the gather capability (0 = none). When set,
    // the GBuffer and gather field run at viewport / settings.gatherFieldScale
    // and the debug views inherit that resolution (one rasterizer, one
    // gather, every consumer); the fixed debug internal size only serves
    // viewport-less hosts.
    std::uint32_t gatherViewportWidth  = 0;
    std::uint32_t gatherViewportHeight = 0;
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
    std::vector<std::uint32_t> screenProbeSurface; // per probe: surface the placement hit
    std::vector<float>     screenProbeSh2;        // per probe: 9 x 3
    // Per probe: the share of the pass' rays that found no surface within
    // the contact radius (1 = open hemisphere around the placement). The
    // pixel pass interpolates it exactly like the SH2 and exports it as the
    // gather field's contact occlusion (see radianceGatherTexelFloats).
    std::vector<float>     screenProbeOpenness;
    std::vector<std::uint8_t> screenProbeValid;
    std::vector<glm::vec3> gatherIrradiance;      // per pixel indirect irradiance (raw accumulation)
    std::vector<glm::vec3> gatherIrradiancePrevious;
    std::vector<glm::vec3> gatherFiltered;        // per pixel display value (bilateral-filtered)
    // Per-pixel validity of the exported field (see radianceGatherTexelFloats):
    // 1 where the GBuffer holds a surface and the gather produced a sample,
    // 0 for background texels the renderer's bilinear walk refuses.
    std::vector<std::uint8_t> gatherFieldValid;
    // Per-pixel contact occlusion of the exported field: the interpolated
    // openness of the neighborhood's screen probes (the tight dark band
    // where surfaces meet that the irradiance averages away).
    std::vector<float>     gatherFieldOcclusion;
    // Version of the exported gather field: bumped every pass that resolves
    // it, frozen while the static-frame skip holds evaluate() early. Copy
    // side and renderer snapshot both gate on it, the copyProbes contract.
    std::uint64_t gatherFieldVersion = 0;
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

    // Per-frame work lists and scratch, reused across frames: capacities
    // persist (the largest frame sets the footprint) and each stage clears
    // its list before refilling, so a steady-state frame does not hit the
    // allocator. Task stages read disjoint entries; no stage runs while a
    // later stage rebuilds one of these lists.
    std::vector<std::uint32_t>  scratchPendingCards;
    std::vector<GiTexelWork>    scratchCaptureWork;
    std::vector<std::uint32_t>  scratchFinishingCards;
    std::vector<GiCardOrderKey> scratchDirectKeys;
    std::vector<GiCardOrderKey> scratchIndirectKeys;
    std::vector<GiTexelWork>    scratchDirectWork;
    std::vector<GiTexelWork>    scratchIndirectWork;
    std::vector<std::size_t>    scratchOffsets;
    std::vector<glm::vec3>      scratchRadiance;   // indirect gather output, committed after the stage
    std::vector<GiProbeItem>    scratchProbeItems;
    std::vector<std::uint8_t>   scratchTaken;      // probe refresh dedup flags (zeroed per level)
    // Caustics scratch (see radiance_cache_caustics.cpp): transmitters and the
    // per-frame deposit staging, reused across frames.
    std::vector<std::uint32_t>  scratchCausticTransmitters;
    std::vector<std::uint32_t>  scratchCausticEmissiveCards; // flat card indices with emission
    std::vector<glm::vec3>      scratchCausticFrame;        // per-flat-texel frame energy estimate
    std::vector<std::uint64_t>  scratchCausticTouched;      // flat texel indices deposited this frame
    // Caustics pass bookkeeping: the emitter-card list and transmitter list
    // rebuild when the registry changes; causticsDecayFrames keeps the
    // decay/commit pass alive this many frames after the last deposit (a
    // caustic whose source vanished keeps fading instead of ghosting, and a
    // long-idle field stops paying the pass entirely).
    std::uint32_t               causticsDecayFrames     = 0;
    bool                        causticTransmittersDirty = true;
    bool                        causticEmissiveCardsDirty = true;

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
    // First flat-texel index of one flat card index (0 when the offsets are
    // not built yet).
    std::uint64_t cardTexelBase(std::uint32_t flatIndex) const
    {
        return flatIndex < cardTexelOffsets.size() ? cardTexelOffsets[flatIndex] : 0;
    }
};

// The task-parallel range helper and the debug-image composition helpers are
// shared with reflection_field through engine/renderer/lighting_cpu_pipeline.hpp.

// ---- scene.cpp -------------------------------------------------------------------

void buildSurfaceGeometry(GiSurfaceGeometry &geometry, const GiSurfaceDescription &description,
    std::uint64_t maxTriangles, GiError *error);
void buildSurfaceBvh(GiSurfaceGeometry &geometry);
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
// Local-space lit point of one layered card texel; the (x, y) plane cell is
// passed in because the lighting loops track it incrementally (the flat
// index still selects the depth layer, so the caller hands both over).
glm::vec3 cardTexelLocalPositionXY(const GiCard &card, std::uint32_t x, std::uint32_t y, std::size_t texel);

// ---- caustics.cpp --------------------------------------------------------------------

// Rebuilds the caustics transmitter list (and the per-surface
// causticTransmitter flags the direct / gather / probe rays read); called at
// frame-plan time while the pipeline state is exclusive, so the pass that
// runs later in the same frame and the direct stage that runs earlier agree.
void rebuildCausticsTransmitters(GiPipelineState &state);

// Runs one amortized caustics photon pass (see radiance_cache_caustics.cpp): spawns
// photon differentials from the sun, the direct lights, and emissive card
// texels, walks them through the transmissive surfaces with the same Snell /
// total-internal-reflection rules the gather rays use, and deposits the focused
// flux into the receivers' card texels as caustic irradiance. Mutates only the
// caustic arrays of the cards; deposits are staged per worker and blended in
// a single-threaded commit.
void runCausticsPass(GiPipelineState &state, TaskScheduler *scheduler);
// Reads the deposited caustic irradiance of one surface at a world-space
// point (the FinalImage debug view's direct term composes it); returns false
// when no card texel covers the point.
bool sampleSurfaceCausticsIrradiance(
    const GiPipelineState &state, std::uint32_t surfaceIndex, const glm::vec3 &position, const glm::vec3 &normal,
    glm::vec3 &outIrradiance);

// ---- pipeline.cpp ------------------------------------------------------------------

// Runs one amortized GI frame: capture, direct, indirect + probe stages.
// Blocks until the stages finish. Returns false when a stage failed.
bool runGiFrame(GiPipelineState &state, TaskScheduler *scheduler);

// Applies one model transform to a surface and refreshes the derived
// matrices/bounds. Returns false (leaving the surface untouched) when the
// transform is singular - its inverse or normal matrix is not finite.
bool updateSurfaceTransform(GiSurface &surface, const glm::mat4 &transform);

// Dynamic relight marking: every probe of both levels whose lattice position
// lies inside the given world volume re-traces from scratch on its next update
// (fresh probes are served first by the planning pass), so a moved, deformed,
// created, or destroyed surface refreshes the world radiance cache around it
// instead of waiting out the rotating stratum cycle.
void markProbesNearVolume(GiPipelineState &state, const glm::vec3 &minBounds, const glm::vec3 &maxBounds);

// ---- gather.cpp ---------------------------------------------------------------------

// Rasterizes the module's GBuffer for the current view (always) and, when
// gatherPasses > 0, runs that many screen-probe + resolve passes so the
// temporal accumulation converges. The GBuffer resolution is the pushed
// gather viewport divided by gather.field_scale when a consumer set one,
// else the fixed debug internal size.
void runGatherPasses(GiPipelineState &state, TaskScheduler *scheduler, std::uint32_t gatherPasses);
void composeDebugView(
    GiPipelineState &state, TaskScheduler *scheduler, GiDebugView view, std::vector<std::uint8_t> &outRgb);
void referencePathTrace(GiPipelineState &state, TaskScheduler *scheduler, std::vector<std::uint8_t> &outRgb);
bool exportDebugView(
    GiPipelineState &state, TaskScheduler *scheduler, GiDebugView view, const char *filePath, GiError *error);

// ---- lighting.cpp ------------------------------------------------------------------

// Projects a radiance array onto the 9-coefficient SH2 basis (GiProbe order,
// pre-multiplied by the basis normalization constants so evaluateSh2Irradiance
// is a plain polynomial) using per-direction solid-angle weights and the
// per-direction basis table that buildProbeDirectionTable fills, then
// suppresses ringing so the irradiance never goes negative.
void projectRadianceToSh2(
    const float *basis, const glm::vec3 *radiance, const float *weights, std::size_t count, float *outSh2);
glm::vec3 directLightRadiance(const GiPipelineState &state, const glm::vec3 &position, const glm::vec3 &normal);
// Shadow transmittance toward the sun (0..1): transparent surfaces soften
// the shadow by their (1 - opacity) share instead of blocking it.
float sunShadowTransmittance(const GiPipelineState &state, const glm::vec3 &position, const glm::vec3 &normal);

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
void radianceAtHit(const GiPipelineState &state, const GiShadingHit &hit, bool includeIndirect, glm::vec3 &outRadiance);

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

} // namespace radiance_cache
} // namespace engine

