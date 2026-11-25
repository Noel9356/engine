// Internal state of the reflection_field module, shared by its translation
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
//   direct lights + the radiance_cache probe-grid diffuse GI + emission, so
//   reflections carry the scene's global illumination) -> rough lobes skip
//   rays and resample the radiance_cache world radiance cache instead (Lumen's
//   radiance-cache reuse band) -> transmission: rays that hit a transparent
//   or translucent surface continue through it (screen traces pass through
//   the surface's GBuffer texels with its attenuation; scene traces refract
//   by Snell's law with a Fresnel-weighted reflect/transmit split, and
//   rough refraction blurs the transmitted lobe) -> denoising: BRDF-weighted
//   spatial reuse, temporal accumulation with reprojection + neighborhood
//   clamp + disocclusion handling, and a variance-driven bilateral filter
//   with double strength on freshly revealed pixels -> the exported
//   reflection field (indirect specular radiance per screen texel).
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
#include <engine/radiance_cache/capabilities.hpp>
#include <engine/reflection_field/capabilities.hpp>
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
namespace reflection_field
{

inline constexpr std::uint32_t noSurface = 0xffffffffu;
// Specular-response constants shared by the tracing pass and the debug views.
inline constexpr float kMinimumRoughness = 0.045f; // matches the renderer's clamp floor
inline constexpr float kDielectricF0 = 0.04f;
// ---- small shared helpers --------------------------------------------------------

// The RNG, sampling frames, intersection kernels, luminance weight, and
// quality preset table are shared with radiance_cache through
// engine/renderer/lighting_ray_core.hpp so both modules draw from the same
// distributions and accept the same hits.
using engine::lighting::kPi;
using engine::lighting::hashSeed;
using engine::lighting::randomFloat;
using engine::lighting::makeBasis;
using engine::lighting::luminance;
using engine::lighting::qualityScale;
using engine::lighting::safeInverseDirection;
using engine::lighting::slabTest;
using engine::lighting::rayTriangle;

// Directional radiance of one probe's SH2 projection along `direction`: the
// stored coefficients are the standard SH coefficients pre-multiplied by
// their basis normalization constants (the radiance_cache convention), so the plain
// polynomial below evaluates the radiance and a constant radiance L projects
// to exactly C00 = L. sh2 is 27 floats in the GiProbe [basis][channel] order.
inline void evaluateSh2Radiance(const float sh2[27], const glm::vec3 &direction, glm::vec3 &outRadiance)
{
    // The shared vectorized twin (lighting_ray_core.hpp): the three channels
    // of each basis term are contiguous in the [basis][channel] layout, so
    // the evaluation runs them as packed lanes of one vector - the exact
    // scalar operation sequence per channel, a third of the instructions.
    outRadiance = engine::lighting::evaluateSh2RadianceVec3(
        sh2, engine::lighting::sh2BasisTermsFor(direction));
}

// ---- settings ----------------------------------------------------------------------

// Every configurable [reflection_field] value; filled by the module from
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
    std::uint32_t screenScale            = 2;
    // Stochastic rays per field pixel per frame; temporal accumulation is
    // the effective sample multiplier.
    std::uint32_t raysPerPixel           = 1;
    // Sub-texel-jittered taps per texel for the near-mirror band (roughness
    // below roughness.ray_band_max, transmissive texels included): a mirror
    // surface's traced content is high-frequency at field resolution, so one
    // texel-center sample aliases bright/dark edges into single-texel
    // outliers; the jittered taps integrate the texel's world footprint and
    // the temporal accumulation converges them (the per-frame rejitter makes
    // the taps independent draws). 1 disables the supersampling.
    std::uint32_t mirrorBandTaps         = 4;
    // Specular-lobe antialiasing floors (a GGX alpha each; see
    // trace.lobe_alpha_floor / trace.transmission_lobe_alpha_floor): every
    // traced lobe integrates at least this wide so a near-mirror texel's
    // near-deterministic direction cannot alias the high-contrast scene it
    // reflects into single-texel outliers at field resolution. The
    // transmitted floor is wider than the reflected one because refraction
    // magnifies the texel footprint (a curved solid bends neighboring entry
    // texels apart), so transmitted content carries finer structure at the
    // same field resolution.
    float         lobeAlphaFloor          = 0.02f;
    float         transmissionLobeAlphaFloor = 0.03f;
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
    // Grazing-rejection cosine (screen_trace.grazing_limit): a candidate hit
    // whose stored normal meets the ray at a shallower angle than this is
    // refused and the march continues. Depth comparisons alone accept those
    // intersections - a near-parallel ray slides behind a floor texel by
    // microns at every step, so grazing floors accept marching-band hits
    // instead of the real reflector - and the refused chain falls through to
    // the scene trace, which resolves the true reflection. 0 disables the
    // gate (every depth hit accepts).
    float         screenTraceGrazingLimit = 0.25f;
    // Screen-trace tier toggle (Lumen's Screen Traces switch): off sends
    // every dedicated ray straight to the scene trace.
    bool          screenTracesEnabled    = true;

    // Transmission: how many hops a ray chain may take through transparent
    // or translucent surfaces (screen pass-throughs and refracted scene hops
    // both count). A chain that exhausts the budget stops tracing and the
    // world radiance cache answers its transmittance share along the would-be
    // continuation, so deep glass chains fade toward the room instead of
    // absorbing to black.
    std::uint32_t maxTransmissionSegments = 3;

    // Temporal accumulation: history weight and neighborhood clamp width
    // (standard deviations of the current 3x3 neighborhood, widened to the
    // neighborhood's own min/max range so deterministic high-dynamic-range
    // structure survives the clamp).
    float         temporalStrength       = 0.98f;
    float         neighborhoodClampSigma = 1.0f;

    // Angular radius of the sun disc the reflection module's sky answer
    // carries (the scene's visible sun geometry is excluded from tracing by
    // the environment contract, so a mirror surface catches the sun only
    // through this analytic disc).
    float         skySunDiscDegrees      = 2.3f;

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
    // Inverse-transpose of the transform's upper 3x3: the correct matrix for
    // transforming normals under non-uniform scale (the plain linear matrix
    // skews them). Cached per surface like the other derived pieces.
    glm::mat3                normalMatrix{1.0f};
    // Exact-identity transform: local space coincides with world space, so
    // the hot per-ray path skips the local<->world arithmetic.
    bool                     localIsWorld = true;
    glm::vec3                albedo{1.0f};
    glm::vec3                emissive{0.0f};
    float                    roughness    = 1.0f;
    float                    metallic     = 0.0f;
    // Transmission pair (see ReflectionSurfaceDescription): opacity below 1
    // lets rays pass through with the (1 - opacity) share, tinted by the
    // albedo; indexOfRefraction bends the transmitted direction at every
    // interface.
    float                    opacity      = 1.0f;
    float                    indexOfRefraction = 1.5f;
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
// The sun/skylight/tone-mapping fields come from the shared
// engine::lighting::EnvironmentCore so this module and radiance_cache shade the
// environment identically.
struct RfEnvironmentSnapshot : engine::lighting::EnvironmentCore
{
    // Direct light with the per-light constants the hit-shading loop needs,
    // shared with radiance_cache through engine/renderer/lighting_ray_core.hpp, so the
    // two modules' direct terms stay identical.
    using PreparedDirectLight = engine::lighting::PreparedLight;

    std::vector<PreparedDirectLight> preparedLights;
};

// ---- radiance_cache probe grid copy ------------------------------------------------------

// Owned copy of the radiance_cache probe-grid snapshot, refreshed per frame when
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
    radiance_cache::GiProbeHeader header{};
    std::vector<radiance_cache::GiProbe> probes;
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
// the texel that cast the ray, the texel the ray landed on, and the full
// chain weight the contribution carries - the source-interface share (the
// Fresnel weight of a reflected lobe or the transmitted share of a
// refraction lobe) times every transmission tint accumulated along the way.
struct RfScreenDeferred
{
    std::uint32_t sourceTexel = 0;
    std::uint32_t hitTexel   = 0;
    glm::vec3     tint{1.0f};
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
    Cache,       // rough band: radiance_cache world radiance cache reuse
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
    // Static-frame skip change detection, mirroring radiance_cache's contract.
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

    // radiance_cache probe grid copy (diffuse GI seen in reflections + the rough
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
    std::vector<float>     gbufferOpacity;    // 1 = opaque; screen traces pass through below 1
    std::vector<float>     gbufferIor;        // index of refraction of the texel's medium (>= 1)
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
    // Per-texel frames of usable history behind fieldFiltered: the temporal
    // pass blends at N/(N+1) (capped at temporal.strength), so the counter is
    // the accumulated-sample depth of the published texel. Disocclusions,
    // revealed texels, and change detections restart it at zero.
    std::vector<std::uint32_t> historyLength;
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
    std::atomic<std::uint32_t> transmissionSegmentsThisFrame{0};
    std::atomic<std::uint64_t> totalRaysTraced{0};
    std::atomic<std::uint32_t> framesSkippedStatic{0};
};

// The task-parallel range helper and the debug-image composition helpers are
// shared with radiance_cache through engine/renderer/lighting_cpu_pipeline.hpp.
using engine::lighting::parallelRange;
using engine::lighting::toneMapColor;
using engine::lighting::toByte;
using engine::lighting::gammaEncode;
using engine::lighting::storePixel;

// ---- scene.cpp ----------------------------------------------------------------------

void rfBuildSurfaceGeometry(RfSurfaceGeometry &geometry, const ReflectionSurfaceDescription &description,
    std::uint64_t maxTriangles, ReflectionError *error);
void rfBuildSurfaceBvh(RfSurfaceGeometry &geometry);
// Applies one model transform to a surface and refreshes the derived
// matrices/bounds. Returns false (leaving the surface untouched) when the
// transform is singular - its inverse or normal matrix is not finite, so every
// slab test against the derived bounds would fail closed and the surface would
// silently vanish from all rays (the same contract radiance_cache's transform update
// carries).
bool rfUpdateSurfaceTransform(RfSurface &surface, const glm::mat4 &transform);
void rfUpdateSurfaceWorldBounds(RfSurface &surface);
void rfEnsureSceneTlas(RfPipelineState &state);

// Closest hit against every non-excluded surface; tMax limits the ray. The
// normal is flipped to face the ray (reflection rays legitimately see back
// faces; `backface` reports when both normals pointed away).
RfHit rfTraceScene(const RfPipelineState &state, const RfRay &ray);

// Copies the provider's probe grid into the state (no-op without the
// capability); returns true when the copy changed.
bool rfRefreshProbeGrid(RfPipelineState &state, const radiance_cache::RadianceCacheCapability *capability);

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

// Dielectric Fresnel reflectance at normal incidence for an index of
// refraction ratio: F0 = ((n1 - n2) / (n1 + n2))^2, the base the Schlick
// term evaluates from on transparent surfaces.
inline float rfDielectricF0FromIor(float enteringIor, float exitingIor)
{
    const float ratio = (enteringIor - exitingIor) / (enteringIor + exitingIor);
    return glm::clamp(ratio * ratio, 0.0f, 1.0f);
}

// Schlick Fresnel reflectance of one scalar channel.
inline float rfFresnelSchlickScalar(float cosine, float f0)
{
    const float complement = 1.0f - cosine;
    const float powered = complement * complement * complement * complement * complement; // (1-cos)^5
    return f0 + (1.0f - f0) * powered;
}

// Sky radiance for reflection rays: the environment's hemispheric skylight
// plus the sun's disc when the direction looks straight at it. The sun disc
// geometry a scene draws for the eye is excluded from tracing by the
// environment contract (both modules answer those rays from the environment),
// so mirror surfaces would never catch the sun without the analytic disc
// here; the GI probes must not see it (their skylight deliberately excludes
// the direct sun term the pbr sun adds), so only the reflection module's
// sky answers carry the disc.
glm::vec3 rfSkyRadiance(const RfPipelineState &state, const glm::vec3 &direction);

// Runs the stochastic reflection pass: for each field texel whose roughness
// is inside the traced bands, samples the GGX lobe and resolves the radiance
// through screen traces -> scene traces -> skylight, or through the probe
// radiance cache for the rough band. Transparent surfaces extend both
// tiers: screen traces pass through their texels with the surface's
// attenuation, and scene traces refract through them (Fresnel-weighted
// reflect/transmit split, Snell bending, rough transmitted lobes for
// translucent surfaces). Fills fieldRaw/fieldResolve/fieldValid.
void rfTraceReflections(RfPipelineState &state, TaskScheduler *scheduler);

// First-segment split of one sampled reflection ray on a transparent texel
// (the estimator both the ray pass and the Reference debug view share, so
// they cannot diverge): the reflected share is DETERMINISTIC - every ray
// traces the reflected chain at its Fresnel weight - and one stochastic pick
// resolves the `(1 - Fresnel)` share between transmission (probability
// `(1 - opacity)`, weight the surface albedo) and absorption. Splitting the
// deterministic term out of the pick removes the estimator's dominant
// variance: a one-pick split traces the reflected chain only when a rare
// draw lands on it, at full weight (the pick probability divides out), so
// the occasional draw that catches a bright emitter - a lamp's mirror image,
// radiance ~7 in a ~0.9 room - swings a texel's running mean far past the
// signal, and no practical accumulation window smooths that. The split
// estimator's reflected chain returns the same radiance every frame and the
// remaining pick varies between two similar radiance levels. The expectation
// is unchanged: `fresnel * L_reflected + (1 - fresnel) * (1 - opacity) *
// albedo * L_transmitted`. Writes the reflected chain's direction (the
// sampled lobe) and origin (the normal-offset origin the screen march starts
// from) with its Fresnel weight; when the pick chooses transmission,
// `outTransmit` is set and the refracted chain's direction (blurred by the
// texel's roughness with the lobe antialiasing floor underneath), origin
// (offset into the medium), and weight (the `(1 - Fresnel)` share tinted by
// the albedo, the enter-only rule) are written. The refracted lobe's tangent
// frame is passed in prebuilt (every ray of a texel shares it).
inline void rfSplitFirstSegment(bool transmits, const glm::vec3 &fresnel, float texelOpacity,
    const glm::vec3 &albedo, const glm::vec3 &direction, float alpha,
    const glm::mat3 &refractedBasis, const glm::vec3 &position, const glm::vec3 &rayOrigin,
    float traceBias, float transmissionLobeFloor, glm::vec3 &outDirection, glm::vec3 &outOrigin,
    glm::vec3 &outWeight, bool &outTransmit, glm::vec3 &outTransmitDirection, glm::vec3 &outTransmitOrigin,
    glm::vec3 &outTransmitWeight, std::uint64_t &seed)
{
    outDirection = direction;
    outOrigin    = rayOrigin;
    outWeight    = fresnel;
    outTransmit  = false;
    if (!transmits)
    {
        // Opaque texel: the Fresnel-weighted reflected lobe is the whole
        // response (the standard specular estimator).
        return;
    }
    // The `(1 - Fresnel)` share: transmission with probability `(1 -
    // opacity)` (the pick probability carries the share), absorption
    // otherwise. The transmitted chain's weight stays the share's tint.
    if (randomFloat(seed) < 1.0f - texelOpacity)
    {
        outTransmit           = true;
        outTransmitDirection  = rfSampleGgxVndf(refractedBasis, glm::vec3(0.0f, 0.0f, 1.0f),
            std::max(alpha, transmissionLobeFloor), seed);
        outTransmitOrigin     = position + outTransmitDirection * traceBias;
        outTransmitWeight     = (glm::vec3(1.0f) - fresnel) * albedo;
    }
}

// Radiance a reflection ray sees when it lands on a surface (Lumen's hit
// lighting): direct sun + direct lights (the renderer's formulas), the
// radiance_cache probe-grid diffuse GI, emission, and the hemispheric ambient
// fallback while the probe grid carries no data. The direct sun matches the
// renderer's own direct term (unoccluded): the field is composited onto the
// renderer's shaded pixels, so a reflection must not carry occlusion the
// presented image does not show - the occluded sun belongs to the module's
// own composed debug views and to a renderer with a shadow system. The
// optional specular block
// adds the surface's own view-dependent response: for roughness inside the
// traced band the world radiance cache answers along the mirrored view
// direction, weighted by Schlick Fresnel of the surface's base reflectivity
// (Lumen's cache carries the response of reflective hits; without it a
// reflection ray landing on a mirror reads black). `viewDirection` points
// from the hit toward the viewer; the zero vector skips the specular term.
glm::vec3 rfShadeHitPoint(const RfPipelineState &state, const glm::vec3 &position, const glm::vec3 &normal,
    const glm::vec3 &albedo, const glm::vec3 &emissive, const glm::vec3 &viewDirection = glm::vec3(0.0f),
    float roughness = 1.0f, float metallic = 0.0f);

// Radiance of a scene-traced hit (anti-leak: backface hits return nothing,
// radiance_cache's rule; unlit hits emit their albedo). `incomingDirection` is the
// ray's travel direction; the hit's own specular samples the cache along its
// mirror.
glm::vec3 rfRadianceAtHit(const RfPipelineState &state, const RfHit &hit, const glm::vec3 &incomingDirection);

// Result of one ray chain: the resolved radiance (chain weight already
// included) plus the deferred screen hits the lazy shading pass resolves;
// `tint` is the weight each deferred contribution carries, `segments` the
// transmission events the chain walked, `resolve` the terminal tier. A chain
// that splits at a transparent interface resolves both branches, so a result
// can carry several deferred hits; the array is bounded by the transmission
// budget's leaf count (each budgeted interface at most doubles the leaves).
struct RfChainDeferredHit
{
    std::uint32_t hitTexel = 0;
    glm::vec3     tint{1.0f};
};
inline constexpr std::uint32_t rfChainDeferredCapacity = 8u;
struct RfChainResolve
{
    glm::vec3           radiance{0.0f};
    RfChainDeferredHit  deferred[rfChainDeferredCapacity];
    std::uint32_t       deferredCount = 0;
    std::uint32_t       segments      = 0;
    RfResolve           resolve       = RfResolve::None;

    void add(const RfChainResolve &other)
    {
        radiance += other.radiance;
        for (std::uint32_t entry = 0; entry < other.deferredCount; ++entry)
        {
            if (deferredCount < rfChainDeferredCapacity)
            {
                deferred[deferredCount++] = other.deferred[entry];
            }
        }
        segments += other.segments;
        if (resolve == RfResolve::None)
        {
            resolve = other.resolve;
        }
    }
};

// One ray chain through the tracing tiers with transmission (see
// reflection_pipeline.cpp for the full contract): screen traces against the
// frame GBuffer (transparent texels pass through with the surface's
// attenuation), then scene traces where transparent surfaces split the ray
// Fresnel-style into a reflected and a refracted branch (Snell bending,
// total internal reflection, rough transmitted lobes for translucent
// surfaces). The chain weight multiplies every terminal radiance so the
// stochastic picks stay unbiased; `remainingSegments` bounds how many more
// interfaces the chain may cross.
RfChainResolve resolveRayChain(const RfPipelineState &state, const glm::vec4 &clipOrigin, const glm::vec3 &origin,
    const glm::vec3 &direction, const glm::vec3 &weight, std::uint32_t remainingSegments, bool allowScreenTrace,
    std::uint64_t &seed);

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
void rfBilateralFilter(RfPipelineState &state, TaskScheduler *scheduler);

// ---- debug.cpp ----------------------------------------------------------------------

// Composes one debug view into RGB bytes at the field resolution (the
// module's working resolution; the exported PPM reports it); Reference
// brute-forces reference_samples rays per texel as ground truth.
bool rfComposeDebugView(
    RfPipelineState &state, TaskScheduler *scheduler, ReflectionDebugView view, std::vector<std::uint8_t> &outRgb);

} // namespace reflection_field
} // namespace engine
