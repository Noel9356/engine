// Public capability surface of the reflection_field module.
//
// Declares the POD types and function-pointer table consumers receive
// through ParticipantContext::acquireCapability for the "reflection_field"
// capability: a registry of reflection scene surfaces (mesh geometry plus
// the PBR specular and transmission parameters reflections need), a
// camera-driven frame evaluation that runs a Lumen-style reflection
// pipeline (screen-space tracing first, then scene ray tracing with
// refraction through transmissive surfaces, radiance-cache reuse for rough
// lobes, hit shading with the radiance_cache probe grid, spatial reuse, temporal
// accumulation, and a variance-driven bilateral filter), a screen-space
// reflection field snapshot copy API for renderers, statistics, and
// debug-view PPM exports.

#pragma once

#include <engine/renderer/device_profile.hpp>

#include <cstdint>

namespace engine
{
namespace reflection_field
{

// ABI version of this capability struct; consumers must match it.
// ABI 2: copyField's buffer grew to four floats per texel - the radiance
// triplet plus the texel's surface roughness - so renderers can gate the
// bilinear field sample on material similarity instead of blending the
// field across surface boundaries (a rough wall texel must not darken the
// mirror image texel beside it).
// ABI 3: ReflectionSurfaceDescription grew the transmission pair
// (opacity, indexOfRefraction) so rays transmit through transparent and
// translucent surfaces (screen traces pass through with the surface's
// attenuation; scene traces refract by Snell's law with a Fresnel-weighted
// reflect/transmit split), ReflectionStatistics reports the transmission
// segments the frame walked, and reflectionFieldRoughnessBand publishes the
// renderer-side bilinear gate's half-width.
inline constexpr std::uint32_t reflectionFieldCapabilityAbi = 3u;
using ReflectionSurfaceHandle                                = std::uint32_t;
inline constexpr ReflectionSurfaceHandle invalidReflectionSurface = 0;

// Texel stride of copyField's output buffer: the radiance RGB triplet
// followed by the texel's surface roughness, interleaved in row-major texel
// order. Buffers handed to copyField must hold width * height * this many
// floats, and readers must index with the same stride - the single source of
// truth for both sides of the capability boundary.
inline constexpr std::uint32_t reflectionFieldTexelFloats = 4u;
// Roughness value the module reports for texels with no traced response
// (background, unlit, or above the traced roughness band). A renderer's
// field sample treats these texels as absent instead of blending them in.
inline constexpr float reflectionFieldNeutralRoughness = 1.0f;
// Half-width of the roughness band a renderer's bilinear field sample
// accepts around the shaded pixel's own roughness: field texels whose
// surface roughness differs from the pixel's by more than this were traced
// for a different lobe, and blending them in would smear one material's
// specular response across a neighboring material's silhouette. The value
// matches the module's own spatial-reuse roughness gate so both sides
// agree on what "same material" means.
inline constexpr float reflectionFieldRoughnessBand = 0.15f;

// ReflectionSurfaceDescription.flagBits values, sharing radiance_cache's semantics:
// Unlit surfaces emit their albedo as radiance; Excluded surfaces are
// invisible to reflection rays (decorative sky/sun geometry the environment
// answers); Dynamic surfaces expect per-frame transform updates.
inline constexpr std::uint32_t reflectionSurfaceFlagUnlit    = 1u;
inline constexpr std::uint32_t reflectionSurfaceFlagExcluded = 2u;
inline constexpr std::uint32_t reflectionSurfaceFlagDynamic  = 4u;

// Status codes written through optional ReflectionError out-parameters:
// malformed description (non-finite floats, bad counts, degenerate geometry,
// out-of-range roughness/metallic), unknown handle, the module or handle
// space being full, a missing camera for evaluate, or a failed debug export.
enum class ReflectionError : std::uint32_t
{
    None = 0,
    InvalidDescription,
    InvalidHandle,
    CapacityReached,
    ViewNotSet,
    ExportFailed,
    EvaluationFailed // a pipeline stage threw or the scheduler rejected work
};

// Scene surface registered into the reflection world. Geometry is borrowed
// for the duration of the create/update call and copied into the module;
// strides follow the radiance_cache convention (0 excludes an optional attribute:
// missing normals are generated from triangle geometry, missing colors
// default to white; the required positions read as tightly packed triplets).
// roughness/metallic are the PBR specular parameters the GGX ray sampling
// and the exported response are evaluated with.
struct ReflectionSurfaceDescription
{
    const float         *positions           = nullptr;
    std::uint32_t        positionStrideBytes = 0;
    const float         *normals            = nullptr;
    std::uint32_t        normalStrideBytes   = 0;
    const float         *colors             = nullptr;
    std::uint32_t        colorStrideBytes   = 0;
    const std::uint32_t *indices            = nullptr;
    std::uint32_t        vertexCount         = 0;
    std::uint32_t        indexCount          = 0;

    float         transform[16]     = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f};
    float         albedoColor[3]    = {1.0f, 1.0f, 1.0f};
    float         emissiveColor[3]  = {0.0f, 0.0f, 0.0f};
    float         emissiveIntensity = 0.0f;
    // PBR specular parameters: roughness in [0, 1], metallic in [0, 1].
    // Roughness 0 requests the sharpest mirror the pipeline supports.
    float         roughness         = 1.0f;
    float         metallic          = 0.0f;
    // Transmission pair: opacity in [0, 1] (1 = opaque; below 1 the surface
    // transmits the (1 - opacity) share of ray energy, tinted by the
    // albedo color), and the medium's index of refraction (>= 1; refraction
    // bends by Snell's law with this ratio, total internal reflection
    // included). Opacity 1 surfaces never transmit.
    float         opacity           = 1.0f;
    float         indexOfRefraction = 1.5f;
    std::uint32_t flagBits          = 0;
};

// Layout of the exported reflection field: a screen-space buffer of indirect
// specular radiance produced for one camera. Each texel is the expected
// GGX-lobe response (importance-sampled estimator, Fresnel included) of the
// surface its ray chain saw - the term a renderer adds to its shaded pixel.
// copyField writes four floats per texel in row-major texel order: the
// radiance RGB triplet followed by the texel's surface roughness, so the
// renderer's bilinear sample can refuse texels from materials outside the
// shaded pixel's roughness band instead of blending across surface
// boundaries (texels with no response - background, unlit, or above the
// traced roughness band - report 1.0). viewProjection is the column-major
// matrix the field was evaluated with (the same convention as setView),
// cameraPosition the matching eye point. maxRoughness echoes the configured
// roughness.max_trace: surfaces rougher than that carry no specular
// environment response and read as zero.
struct ReflectionFieldHeader
{
    std::uint32_t width            = 0;
    std::uint32_t height           = 0;
    float         viewProjection[16] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f};
    float         cameraPosition[3]  = {0.0f, 0.0f, 0.0f};
    float         maxRoughness       = 0.4f;
};

// Frame counters and cumulative totals. Per-frame fields are reset by
// evaluate(); totals accumulate. raysTracedThisFrame counts the dedicated
// reflection rays (the rough radiance-cache band reports its lookups in
// raysReusedCacheThisFrame), and the three hit counters split those rays by
// where they resolved: screen traces, scene traces, and the sky fallback.
struct ReflectionStatistics
{
    std::uint64_t frameIndex                 = 0;
    std::uint32_t surfaces                   = 0;
    std::uint64_t triangles                  = 0;
    std::uint32_t fieldWidth                 = 0;
    std::uint32_t fieldHeight                = 0;
    std::uint32_t pixelsTracedThisFrame      = 0;
    std::uint32_t raysTracedThisFrame        = 0;
    std::uint32_t raysReusedCacheThisFrame   = 0;
    std::uint32_t screenTraceHitsThisFrame   = 0;
    std::uint32_t sceneTraceHitsThisFrame    = 0;
    // Transmission segments the frame's ray chains walked: every hop a ray
    // took through a transparent or translucent surface (screen pass-through
    // and refracted scene hops both count), summed over the frame. Zero
    // unless the scene registers surfaces with opacity below 1.
    std::uint32_t transmissionSegmentsThisFrame = 0;
    std::uint64_t totalRaysTraced            = 0;
    std::uint32_t framesSkippedStatic        = 0;
    double        lastEvaluateMilliseconds   = 0.0;
};

// Device-resolved budgets produced by resolveProfile: explicit configuration
// wins; automatic values scale the field resolution and ray budget by device
// class (a CPU device halves the linear resolution, a discrete GPU runs the
// configured values, everything in between takes one step down).
struct ReflectionProfile
{
    std::uint32_t screenScale       = 4;
    std::uint32_t raysPerPixel      = 1;
    float         temporalStrength  = 0.9f;
};

// Debug views written by debugExport as binary P6 PPM files at the configured
// internal resolution. Composite is the module's own shaded image (direct +
// radiance_cache indirect + reflection field, through the pbr tone mapping);
// ReflectionsOnly isolates the field; ResolveMask colors pixels by where
// their radiance came from (screen trace / scene trace / radiance cache /
// sky); Reference brute-forces many rays per pixel as ground truth for
// validation of the filtered field.
enum class ReflectionDebugView : std::uint32_t
{
    Composite = 0,
    ReflectionsOnly,
    ResolveMask,
    Reference
};

// Function-pointer table for the "reflection_field" capability. Mutators and
// evaluate serialize against each other on one module mutex; copyField and
// statistics are the read side. Calls never throw; failures are reported
// through the error out-parameters.
struct ReflectionFieldCapability
{
    static constexpr const char *capabilityName = "reflection_field";

    std::uint32_t abi  = reflectionFieldCapabilityAbi;
    void         *self = nullptr;

    ReflectionSurfaceHandle (*createSurface)(
        void *self, const ReflectionSurfaceDescription *description, ReflectionError *error);
    bool (*updateSurface)(
        void *self, ReflectionSurfaceHandle surface, const ReflectionSurfaceDescription *description,
        ReflectionError *error);

    // Fast per-frame transform path for Dynamic surfaces; geometry and
    // material are kept. Returns false for unknown handles.
    bool (*updateSurfaceTransform)(void *self, ReflectionSurfaceHandle surface, const float *transformColumnMajor16);

    bool (*destroySurface)(void *self, ReflectionSurfaceHandle surface);

    // Camera and viewport the field is evaluated for; viewProjection is
    // column-major 4x4 (the renderer's own convention), position the world
    // eye, and viewportWidth/Height the render target the field will be
    // composited onto (the field is sized from them and screen_scale).
    bool (*setView)(void *self, const float *viewProjectionColumnMajor16, const float *cameraPosition3,
        std::uint32_t viewportWidth, std::uint32_t viewportHeight);

    // Runs one Lumen-style reflection frame on the kernel task scheduler and
    // blocks until finished (the render_graph::execute pattern). Requires
    // setView.
    bool (*evaluate)(void *self, double deltaSeconds, ReflectionError *error);

    // Snapshot read of the reflection field. With a null/zero-capacity
    // buffer it still reports the version and fills outHeader, so callers
    // can size their buffer and detect changes. The buffer layout is four
    // floats per texel: the linear-HDR radiance triplet followed by the
    // texel's surface roughness (1.0 for texels that carry no response -
    // background, unlit, or above the traced roughness band - which lets the
    // renderer gate its bilinear sample on material similarity). Returns the
    // number of texels copied (truncated to capacity).
    std::uint32_t (*copyField)(void *self, float *output, std::uint32_t capacityTexels,
        ReflectionFieldHeader *outHeader, std::uint64_t *outVersion);

    bool (*statistics)(void *self, ReflectionStatistics *outStatistics);
    bool (*resolveProfile)(void *self, const renderer::DeviceProfile *device, ReflectionProfile *outProfile);

    // Writes one debug view as a binary PPM file. The view reflects the most
    // recent evaluate() state; export failures (bad path) report ExportFailed.
    bool (*debugExport)(
        void *self, ReflectionDebugView view, const char *filePath, ReflectionError *error);
};

} // namespace reflection_field
} // namespace engine
