// Internal state of the lumen_reflection module, shared by its translation
// units.
//
// The pipeline mirrors Lumen's reflection solve scaled to a CPU
// implementation:
//
//   surfaces (registry) -> a screen-space GBuffer rasterized for the frame's
//   camera at 1/screen_scale resolution -> per-pixel GGX-importance-sampled
//   reflection rays ("stochastic integration") resolved through the hybrid
//   tracing pipeline (screen traces against the module's own GBuffer first,
//   then a scene trace against the per-surface BVHs) -> hit lighting (sun +
//   direct lights + the lumen_gi probe-grid diffuse GI + emission, so
//   reflections carry the scene's global illumination) -> rough lobes skip
//   rays and resample the lumen_gi world radiance cache instead (Lumen's
//   radiance-cache reuse band) -> denoising: BRDF-weighted spatial reuse,
//   temporal accumulation with reprojection + neighborhood clamp +
//   disocclusion handling, and a variance-driven bilateral filter with
//   double strength on freshly revealed pixels -> the exported reflection
//   field (indirect specular radiance per screen texel).
//
// Rays trace the registered triangle scene through a top-level BVH over the
// surfaces and per-surface BVHs (rays transform into each surface's local
// space, so dynamic transforms never rebuild a surface BVH).
//
// Lumen reference: the stage structure, roughness bands, and denoising chain
// follow Epic's published Lumen design ("Lumen: Real-time Global Illumination
// in Unreal Engine 5", SIGGRAPH 2022 Advances course; "Lumen Technical
// Details", Epic Games documentation) scaled to a CPU implementation.

#pragma once

#include <cstddef>
#include <engine/direct_lighting/capabilities.hpp>
#include <engine/kernel/task_scheduler.hpp>
#include <engine/lumen_gi/capabilities.hpp>
#include <engine/lumen_reflection/capabilities.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/renderer/lumen_cpu_pipeline.hpp>
#include <engine/renderer/lumen_ray_core.hpp>

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
namespace lumen_reflection
{

inline constexpr std::uint32_t noSurface = 0xffffffffu;
// Specular-response constants shared by the tracing pass and the debug views.
inline constexpr float kMinimumRoughness = 0.045f; // matches the renderer's clamp floor
inline constexpr float kDielectricF0 = 0.04f;

// ---- small shared helpers --------------------------------------------------------

// The RNG, sampling frames, intersection kernels, luminance weight, and
// quality preset table are shared with lumen_gi through
// engine/renderer/lumen_ray_core.hpp so both modules draw from the same
// distributions and accept the same hits.
using engine::lumen::kPi;
using engine::lumen::hashSeed;
using engine::lumen::randomFloat;
using engine::lumen::makeBasis;
using engine::lumen::luminance;
using engine::lumen::qualityScale;
using engine::lumen::safeInverseDirection;
using engine::lumen::slabTest;
using engine::lumen::rayTriangle;

// Directional radiance of one probe's SH2 projection along `direction`: the
// stored coefficients are the standard SH coefficients pre-multiplied by
// their basis normalization constants (the lumen_gi convention), so the plain
// polynomial below evaluates the radiance and a constant radiance L projects
// to exactly C00 = L. sh2 is 27 floats in the GiProbe [basis][channel] order.
inline void evaluateSh2Radiance(const float sh2[27], const glm::vec3 &direction, glm::vec3 &outRadiance)
{
    const float x = direction.x;
    const float y = direction.y;
    const float z = direction.z;
    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;

    for (std::uint32_t channel = 0; channel < 3; ++channel)
    {
        const float *coefficient = sh2 + channel;
        outRadiance[channel] = coefficient[0]
                             + coefficient[3] * y + coefficient[6] * z + coefficient[9] * x
                             + coefficient[12] * x * y + coefficient[15] * y * z
                             + coefficient[18] * (3.0f * zz - 1.0f) + coefficient[21] * x * z
                             + coefficient[24] * (xx - yy);
    }
}

// ---- settings ----------------------------------------------------------------------

// Every configurable [lumen_reflection] value; filled by the module from
// configuration and refreshed on live reload. The defaults below match
// config.toml and docs/configuration.md.
struct ReflectionSettings
{
    bool          enabled               = true;
    // Static-frame skip: with camera, scene, and environment unchanged and
    // the temporal history converged, evaluate() returns early instead of
    // re-tracing the same rays.
    bool          skipUnchangedFrames   = true;
    std::uint32_t quality               = 3;  // QualityPreset::High

    std::uint32_t maxSurfaces           = 256;
    std::uint64_t maxTrianglesPerSurface = 200000;

    // Field resolution: 1/screen_scale of the viewport in each axis.
    std::uint32_t screenScale            = 4;
    // Stochastic rays per field pixel per frame; temporal accumulation is
    // the effective sample multiplier.
    std::uint32_t raysPerPixel           = 1;
    // Screen traces resolve only what the frame GBuffer covers within this
    // radius; longer rays fall through to the scene trace.
    float         screenRayDistance      = 2.0f;

    // Roughness bands (see docs): rays below rayBandMax, radiance-cache
    // lookups between rayBandMax and maxTrace, no response above maxTrace.
    float         roughRayBandMax        = 0.3f;
    float         roughMaxTrace          = 0.4f;

    float         maxTraceDistance       = 50.0f;
    float         traceBias              = 0.02f;

    // Screen traces: marching step budget and thin-occluder thickness (the
    // depth slack a ray may pass behind a surface and still count as a hit).
    std::uint32_t screenTraceMaxSteps    = 48;
    float         screenTraceThickness   = 0.5f;
    // Screen-trace tier toggle (Lumen's Screen Traces switch): off sends
    // every dedicated ray straight to the scene trace.
    bool          screenTracesEnabled    = true;

    // Temporal accumulation: history weight and neighborhood clamp width
    // (standard deviations of the current 3x3 neighborhood).
    float         temporalStrength       = 0.9f;
    float         neighborhoodClampSigma = 1.0f;

    // Luminance variance threshold above which the bilateral filter smooths.
    float         varianceThreshold      = 0.02f;

    // Spatial reuse radius in pixels (BRDF-weighted 3x3 at radius 1) and the
    // variance-driven bilateral filter.
    std::uint32_t spatialReuseRadius     = 1;
    bool          bilateralEnabled       = true;
    std::uint32_t bilateralRadius        = 2;

    std::uint32_t referenceSamples       = 16;
};

// ---- scene -------------------------------------------------------------------------

// Triangle stored in BVH leaf order for cache-friendly traversal (Moller-
// Trumbore inputs) plus the source triangle index.
struct RfPackedTriangle
{
    glm::vec3     v0{0.0f};
    glm::vec3     edge1{0.0f};
    glm::vec3     edge2{0.0f};
    std::uint32_t triangle = 0;
};

// Triangle soup in local space plus its BVH. Traces transform the ray into
// local space, so dynamic surfaces only update their transform.
struct RfSurfaceGeometry
{
    std::vector<glm::vec3>    positions;
    std::vector<glm::vec3>    normals; // smooth vertex normals in local space
    std::vector<glm::vec3>    colors;   // vertex colors (white when absent)
    std::vector<std::uint32_t> indices;

    // Interior nodes: leftOrFirst = left child node index (right is left+1),
    // count = 0. Leaves: leftOrFirst = first entry in `packed`, count > 0.
    struct BvhNode
    {
        glm::vec3     boundsMin{0.0f};
        glm::vec3     boundsMax{0.0f};
        std::uint32_t leftOrFirst = 0;
        std::uint32_t count       = 0;
    };
    std::vector<BvhNode>          bvh;
    std::vector<RfPackedTriangle> packed;
    glm::vec3                     boundsMin{0.0f};
    glm::vec3                     boundsMax{0.0f};
};

struct RfSurface
{
    ReflectionSurfaceHandle handle = invalidReflectionSurface;
    RfSurfaceGeometry        geometry;
    glm::mat4                transform{1.0f};
    glm::mat4                inverseTransform{1.0f};
    glm::mat3                inverseLinear{1.0f};       // upper 3x3 of inverseTransform
    glm::vec3                inverseTranslation{0.0f}; // column 3 of inverseTransform
    // Exact-identity transform: local space coincides with world space, so
    // the hot per-ray path skips the local<->world arithmetic.
    bool                     localIsWorld = true;
    glm::vec3                albedo{1.0f};
    glm::vec3                emissive{0.0f};
    float                    roughness    = 1.0f;
    float                    metallic     = 0.0f;
    std::uint32_t            flagBits     = 0;
    bool                     excluded     = false;
    bool                     unlit        = false;
    bool                     dynamic      = false;
    glm::vec3                worldMin{0.0f};
    glm::vec3                worldMax{0.0f};
    // GBuffer raster caches: world-space vertex positions depend only on the
    // transform and clip-space positions only on the view, so both are kept
    // per surface and refreshed on demand. `rasterBuiltSerial` records the
    // raster serial the projected triangle cache was built at (0 = stale);
    // `rasterWorldDirty` marks the world positions for recomputation.
    std::vector<glm::vec3>   rasterWorldPositions;
    std::vector<glm::vec4>   rasterClipPositions;
    std::uint64_t            rasterBuiltSerial = 0;
    bool                     rasterWorldDirty   = true;
};

// Top-level acceleration structure over the non-excluded surfaces.
struct RfTlasNode
{
    glm::vec3     boundsMin{0.0f};
    glm::vec3     boundsMax{0.0f};
    std::uint32_t leftOrFirst = 0;
    std::uint32_t count       = 0;
};

// ---- environment snapshot ----------------------------------------------------------

// Snapshot of the pbr environment + direct lights taken each evaluate().
struct RfEnvironmentSnapshot
{
    struct PreparedDirectLight
    {
        direct_lighting::DirectLightType type = direct_lighting::DirectLightType::Directional;
        glm::vec3 color{0.0f};
        float     intensity   = 0.0f;
        glm::vec3 direction{0.0f}; // Directional: toward the light. Spot: cone axis.
        glm::vec3 position{0.0f};
        float     range       = 0.0f;
        float     innerCosine = 0.0f;
        float     outerCosine = 0.0f;
        float     coneDivisor = 1.0e-5f;
    };

    bool     hasSun       = false;
    bool     sunEnabled   = false;
    glm::vec3 sunDirection{0.45f, 0.62f, 0.35f}; // toward the sun (unit)
    glm::vec3 sunColor{1.0f, 0.96f, 0.88f};
    float    sunIntensity = 3.0f;
    glm::vec3 ambientSky{0.55f, 0.65f, 0.80f};
    glm::vec3 ambientGround{0.35f, 0.30f, 0.25f};
    float    ambientIntensity = 0.5f;
    // Exposure + tone mapping of the pbr environment, applied by the debug
    // views (the renderer applies its own to the composited pixels).
    float    exposure      = 1.0f;
    pbr::ToneMapping toneMapping = pbr::ToneMapping::AcesFilmic;
    std::vector<PreparedDirectLight> preparedLights;

    // Hemisphere skylight standing in for Lumen's Skylight on ray miss.
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

// ---- lumen_gi probe grid copy ------------------------------------------------------

// Owned copy of the lumen_gi probe-grid snapshot, refreshed per frame when
// the provider's version changes. Level descriptors mirror the renderer's
// buildProbeLevels so lookups agree with the renderer's indirect term.
struct RfProbeLevel
{
    glm::vec3 anchor{0.0f};
    glm::ivec3 dims{0};
    float     spacing = 2.0f;
    std::uint32_t offset = 0;
};

struct RfProbeGrid
{
    lumen_gi::GiProbeHeader header{};
    std::vector<lumen_gi::GiProbe> probes;
    RfProbeLevel level0;
    RfProbeLevel level1;
    bool levelsBuilt = false;
    std::uint64_t version = 0;

    bool valid() const
    {
        return levelsBuilt && !probes.empty();
    }
};

// ---- frame GBuffer rasterization types ---------------------------------------------

// Vertices carried from local space through the near-plane clip to the screen
// projection (perspective-correct interpolation keeps world position and
// barycentric attributes).
struct RfClipVertex
{
    glm::vec4 clip{0.0f};
    glm::vec3 world{0.0f};
    glm::vec3 bary{0.0f};
};

struct RfScreenVertex
{
    glm::vec2 pixel{0.0f};
    float     inverseW = 0.0f;
    glm::vec3 world{0.0f};
    glm::vec3 bary{0.0f};
};

struct RfScreenTriangle
{
    RfScreenVertex vertex[3];
    std::uint32_t  surface  = 0;
    std::uint32_t  triangle = 0;
};

// A screen-trace hit whose radiance is resolved after the lazy shading pass:
// the texel that cast the ray, the texel the ray landed on, and the Fresnel
// weight of the sample.
struct RfScreenDeferred
{
    std::uint32_t sourceTexel = 0;
    std::uint32_t hitTexel   = 0;
    glm::vec3     fresnel{0.0f};
};

// ---- rays and hits ------------------------------------------------------------------

struct RfRay
{
    glm::vec3 origin{0.0f};
    glm::vec3 direction{0.0f, 0.0f, -1.0f};
    float     tMax = std::numeric_limits<float>::max();
};

// Closest-hit payload: everything hit lighting needs (world position and
// normal for shading, albedo with vertex colors interpolated at the hit).
struct RfHit
{
    bool          hit          = false;
    bool          backface     = false; // both smooth and geometric normal face away
    float         t            = 0.0f;
    std::uint32_t surfaceIndex = noSurface;
    glm::vec3     position{0.0f};
    glm::vec3     normal{0.0f, 1.0f, 0.0f};     // world smooth normal, faces the ray
    glm::vec3     albedo{1.0f};                  // surface albedo * vertex colors at the hit
};

// ---- frame GBuffer ------------------------------------------------------------------

constexpr float rfBackgroundDepth = std::numeric_limits<float>::max();

inline bool rfIsBackground(float depth)
{
    return depth >= rfBackgroundDepth * 0.5f;
}

// ---- resolve kinds ------------------------------------------------------------------

// Where a field texel's radiance came from (the ResolveMask debug view).
enum class RfResolve : std::uint8_t
{
    None = 0,     // background / roughness above max trace: no specular term
    ScreenTrace,  // resolved against the frame GBuffer (Lumen's screen traces)
    SceneTrace,   // resolved by tracing the scene (Lumen's Lumen Scene path)
    Cache,       // rough band: lumen_gi world radiance cache reuse
    Sky          // ray escaped: skylight fallback
};

// ---- pipeline state ------------------------------------------------------------------

// The whole pipeline state owned by the module instance. Mutators hold the
// module mutex; parallel stage tasks touch disjoint buffers and read only
// state frozen before they run.
struct RfPipelineState
{
    ReflectionSettings settings;

    // Registry.
    std::vector<RfSurface>   surfaces;
    std::uint32_t            registryCapacity = 256;
    ReflectionSurfaceHandle  nextHandle       = 1;
    std::uint64_t            frameIndex       = 0;

    // Top-level BVH over the non-excluded surfaces (rebuilt lazily).
    std::vector<RfTlasNode>    tlas;
    std::vector<std::uint32_t> tlasSurfaces;
    bool                       tlasDirty = true;

    // GBuffer raster caches: `rasterSerial` is bumped whenever the view, the
    // registry, or the field resolution changes, so each surface rebuilds its
    // projected triangles only when something they depend on actually moved.
    std::uint64_t                             rasterSerial = 1;
    std::uint32_t                             rasterFieldWidth  = 0;
    std::uint32_t                             rasterFieldHeight = 0;
    std::vector<std::vector<RfScreenTriangle>> rasterTriangles;
    std::vector<RfScreenTriangle>             rasterConcat;
    std::vector<std::vector<std::uint32_t>>    rasterBins;

    // Camera + viewport for the running frame.
    bool      hasView   = false;
    glm::mat4 viewProjection{1.0f};
    glm::vec3 cameraPosition{0.0f};
    std::uint32_t viewportWidth  = 0;
    std::uint32_t viewportHeight = 0;
    // Static-frame skip change detection, mirroring lumen_gi's contract.
    bool          viewDirty  = true;
    bool          sceneDirty = true;
    bool          hasPreviousView = false;
    std::uint32_t framesSinceDirty = 0;
    // Static-frame skip convergence threshold (frames of clean input before
    // evaluate() may freeze the field): derived from the temporal blend so a
    // frozen field is a converged one. applySettingsToState fills it from
    // settings.temporalStrength - the published frame blends the fresh trace
    // at weight (1 - strength)^N after N clean frames, so the threshold is
    // where that weight drops below 1% (clamped to 1..128; a disabled
    // temporal filter converges immediately).
    std::uint32_t convergenceFrames = 1;
    RfEnvironmentSnapshot environment;
    bool        environmentValid = false;
    std::uint64_t environmentVersion = 0;
    std::uint64_t directLightsVersion = 0;

    // lumen_gi probe grid copy (diffuse GI seen in reflections + the rough
    // radiance-cache band). Refreshed per frame; stays invalid without the
    // provider module, and the pipeline falls back to the analytic sky.
    RfProbeGrid probeGrid;

    // Frame GBuffer + field, all at field resolution.
    std::uint32_t fieldWidth  = 0;
    std::uint32_t fieldHeight = 0;
    std::vector<float>     gbufferDepth;       // camera distance; max = background
    std::vector<glm::vec3> gbufferPosition;
    std::vector<glm::vec3> gbufferNormal;
    std::vector<glm::vec3> gbufferAlbedo;
    std::vector<float>     gbufferRoughness;
    std::vector<float>     gbufferMetallic;
    std::vector<glm::vec3> gbufferEmissive;
    std::vector<std::uint8_t> gbufferUnlit;
    std::vector<std::uint32_t> gbufferSurface;

    // Raw stochastic trace output (before denoising) and per-texel metadata.
    std::vector<glm::vec3> fieldRaw;
    std::vector<RfResolve> fieldResolve;
    std::vector<std::uint8_t> fieldValid;      // 1 = texel carries specular response
    std::vector<float>     fieldRayCount;      // rays accumulated into the raw value this frame

    // Denoised, published field (the exported buffer).
    std::vector<glm::vec3> fieldFiltered;

    // Temporal history of the field (reprojected via the stored matrices).
    bool          historyValid = false;
    glm::mat4     historyViewProjection{1.0f};
    std::uint32_t historyWidth  = 0;
    std::uint32_t historyHeight = 0;
    std::vector<glm::vec3> historyField;
    std::vector<float>     historyDepth;
    std::vector<glm::vec3> scratchSpatial; // spatial reuse output, committed per frame
    std::vector<std::uint8_t> scratchDisoccluded; // temporal pass output flags
    std::vector<float>     scratchVariance;       // bilateral pass input

    // Lazy screen color: only texels actually hit by screen traces are shaded
    // (the whole-GBuffer shading the traces sample would otherwise dominate
    // the frame). `screenNeedShade` is the per-frame request mask,
    // `screenDeferredRows` collects the per-row deferred contributions that
    // are applied once the requested texels hold their shaded color.
    std::vector<glm::vec3>                  screenColor;
    std::vector<std::uint8_t>               screenNeedShade;
    std::vector<std::vector<RfScreenDeferred>> screenDeferredRows;

    // Per-texel luminance of the buffer the denoise passes weight against
    // (fieldRaw for spatial reuse, fieldFiltered for the bilateral pass).
    std::vector<float>                      fieldLuminance;

    // Statistics, accumulated by parallel stages through fetch_add.
    std::atomic<std::uint32_t> pixelsTracedThisFrame{0};
    std::atomic<std::uint32_t> raysTracedThisFrame{0};
    std::atomic<std::uint32_t> raysReusedCacheThisFrame{0};
    std::atomic<std::uint32_t> screenTraceHitsThisFrame{0};
    std::atomic<std::uint32_t> sceneTraceHitsThisFrame{0};
    std::atomic<std::uint64_t> totalRaysTraced{0};
    std::atomic<std::uint32_t> framesSkippedStatic{0};
};

// The task-parallel range helper and the debug-image composition helpers are
// shared with lumen_gi through engine/renderer/lumen_cpu_pipeline.hpp.
using engine::lumen::parallelRange;
using engine::lumen::toneMapColor;
using engine::lumen::toByte;
using engine::lumen::gammaEncode;
using engine::lumen::storePixel;

// ---- scene.cpp ----------------------------------------------------------------------

void rfBuildSurfaceGeometry(RfSurfaceGeometry &geometry, const ReflectionSurfaceDescription &description,
    std::uint64_t maxTriangles, ReflectionError *error);
void rfBuildSurfaceBvh(RfSurfaceGeometry &geometry);
void rfUpdateSurfaceTransform(RfSurface &surface, const glm::mat4 &transform);
void rfUpdateSurfaceWorldBounds(RfSurface &surface);
void rfEnsureSceneTlas(RfPipelineState &state);

// Closest hit against every non-excluded surface; tMax limits the ray. The
// normal is flipped to face the ray (reflection rays legitimately see back
// faces; `backface` reports when both normals pointed away).
RfHit rfTraceScene(const RfPipelineState &state, const RfRay &ray);

// Copies the provider's probe grid into the state (no-op without the
// capability); returns true when the copy changed.
bool rfRefreshProbeGrid(RfPipelineState &state, const lumen_gi::LumenGiCapability *capability);

// Cosine-lobed SH2 irradiance of the probe grid at a world position/normal
// (validity-weighted trilinear over the finer covered level); returns false
// when no probe around the position holds data.
bool rfProbeIrradiance(
    const RfPipelineState &state, const glm::vec3 &position, const glm::vec3 &normal, glm::vec3 &outIrradiance);

// Directional SH2 radiance of the probe grid at a world position along one
// direction (validity-weighted trilinear); returns false when uncovered.
bool rfProbeRadiance(
    const RfPipelineState &state, const glm::vec3 &position, const glm::vec3 &direction, glm::vec3 &outRadiance);

// ---- pipeline.cpp -------------------------------------------------------------------

// Rasterizes the frame GBuffer (depth, position, normal, albedo, PBR specular
// parameters, emission) for the current view at field resolution.
void rfRasterizeGBuffer(RfPipelineState &state, TaskScheduler *scheduler);

// GGX visible-normal importance sampling (Heitz 2018): draws a reflection
// direction from the visible GGX lobe of (view, normal, alpha) using two
// uniforms; returns the sampling weight-free direction (the estimator's PDF
// cancels the lobe, so the contribution is the traced radiance itself). The
// tangent frame (`basis` with `localView`, both hoisted per texel by the
// caller) is passed in because it is identical for every ray of a texel.
glm::vec3 rfSampleGgxVndf(const glm::mat3 &basis, const glm::vec3 &localView, float alpha, std::uint64_t &seed);

// Runs the stochastic reflection pass: for each field texel whose roughness
// is inside the traced bands, samples the GGX lobe and resolves the radiance
// through screen traces -> scene traces -> skylight, or through the probe
// radiance cache for the rough band. Fills fieldRaw/fieldResolve/fieldValid.
void rfTraceReflections(RfPipelineState &state, TaskScheduler *scheduler);

// Radiance a reflection ray sees when it lands on a surface (Lumen's hit
// lighting): direct sun + direct lights (the renderer's formulas), the
// lumen_gi probe-grid diffuse GI, emission, and the hemispheric ambient
// fallback while the probe grid carries no data. View-dependent specular at
// the hit is deliberately not evaluated - Lumen's cached scene lighting
// carries diffuse lighting and emission only.
glm::vec3 rfShadeHitPoint(const RfPipelineState &state, const glm::vec3 &position, const glm::vec3 &normal,
    const glm::vec3 &albedo, const glm::vec3 &emissive);

// Radiance of a scene-traced hit (anti-leak: backface hits return nothing,
// lumen_gi's rule; unlit hits emit their albedo).
glm::vec3 rfRadianceAtHit(const RfPipelineState &state, const RfHit &hit);

// Fresnel weight of one GGX sample (Schlick on the view/half-vector cosine),
// applied at trace time so dielectrics and metals both get the right share;
// `f0` is hoisted per texel by the caller.
glm::vec3 rfFresnelWeight(const glm::vec3 &view, const glm::vec3 &sampledDirection, const glm::vec3 &f0);

// Runs one complete reflection frame: GBuffer + trace + denoise; publishes
// fieldFiltered. Returns false when a stage failed.
bool rfRunReflectionFrame(RfPipelineState &state, TaskScheduler *scheduler);

// ---- denoise.cpp --------------------------------------------------------------------

// BRDF-weighted spatial reuse over screen-space neighbors (Lumen's spatial
// reuse pass): reads fieldRaw, writes scratchSpatial + per-texel variance.
void rfSpatialReuse(RfPipelineState &state, TaskScheduler *scheduler);

// Temporal accumulation (Lumen's temporal pass): reprojects the previous
// frame's field with the stored matrices, validates it against the current
// depth (disocclusion), clamps the history to the current neighborhood, and
// blends; writes fieldFiltered and marks freshly revealed texels, where the
// bilateral filter then runs at double strength.
void rfTemporalAccumulate(RfPipelineState &state, TaskScheduler *scheduler);

// Variance-driven bilateral filter (Lumen's last-ditch pass): smooths only
// high-variance texels with tone-mapped (firefly-suppressing) weights.
void rfbilateralFilter(RfPipelineState &state, TaskScheduler *scheduler);

// ---- debug.cpp ----------------------------------------------------------------------

// Composes one debug view into RGB bytes at the field resolution (the
// module's working resolution; the exported PPM reports it); Reference
// brute-forces reference_samples rays per texel as ground truth.
bool rfComposeDebugView(
    RfPipelineState &state, TaskScheduler *scheduler, ReflectionDebugView view, std::vector<std::uint8_t> &outRgb);

} // namespace lumen_reflection
} // namespace engine
