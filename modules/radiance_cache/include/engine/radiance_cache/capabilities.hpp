// Public capability surface of the radiance_cache module.
//
// Declares the POD types and function-pointer tables consumers receive
// through ParticipantContext::acquireCapability for the "radiance_cache" capability
// (ABI 4): a registry of GI scene surfaces, a camera-driven frame evaluation that runs
// a Lumen-style dynamic global illumination pipeline (card-based surface
// cache, amortized direct/indirect lighting, world-space radiance probes,
// screen-space final gather), a probe-grid snapshot copy API for renderers,
// device-based profile resolution, statistics, and debug-view PPM exports.

#pragma once

#include <engine/renderer/device_profile.hpp>

#include <algorithm>
#include <cstdint>

namespace engine
{
namespace radiance_cache
{

// ABI version of this capability struct; consumers must match it. History:
// 1 -> 2 added GiStatistics::framesSkippedStatic (the static-frame skip
// reports how many evaluate() calls returned early because the camera, the
// scene, and the environment were all unchanged and the caches converged).
// 2 -> 3 added GiError::EvaluationFailed and GiStatistics::indirectRaysThisFrame,
// and defined GiProbe::sh2 as irradiance-ready coefficients (see GiProbe).
// 3 -> 4 added GiSurfaceDescription's transmission pair (opacity,
// indexOfRefraction): gather and probe rays transmit through surfaces with
// opacity below 1 (attenuated by the (1 - opacity) share, tinted by the
// albedo, bent once by Snell's law with the index ratio), and shadow rays
// soften by the same transmittance instead of being blocked.
// 4 -> 5 added sampleCausticsIrradiance: the photon caustics pass's
// deposited irradiance, readable per world position so coupled renderers
// (the reflection field's refracted chains) light receiver hits with the
// focused patterns a transmitter casts onto them.
inline constexpr std::uint32_t radianceCacheCapabilityAbi = 5u;
using GiSurfaceHandle                               = std::uint32_t;
inline constexpr GiSurfaceHandle invalidGiSurface   = 0;

// GiSurfaceDescription.flagBits values. Unlit surfaces emit their albedo as
// radiance (renderer "unlit" semantics); Excluded surfaces are invisible to
// GI rays entirely (decorative sky/sun geometry the environment already
// covers); Dynamic surfaces expect per-frame transform updates.
inline constexpr std::uint32_t giSurfaceFlagUnlit    = 1u;
inline constexpr std::uint32_t giSurfaceFlagExcluded = 2u;
inline constexpr std::uint32_t giSurfaceFlagDynamic  = 4u;

// Status codes written through optional GiError out-parameters: malformed
// description (non-finite floats, bad counts, degenerate geometry), unknown
// handle, the module or handle space being full, a missing camera for
// evaluate, or a failed debug export.
enum class GiError : std::uint32_t
{
    None = 0,
    InvalidDescription,
    InvalidHandle,
    CapacityReached,
    ViewNotSet,
    ExportFailed,
    EvaluationFailed   // a pipeline stage threw or the scheduler rejected work
};

// Scene surface registered into the GI world. Geometry is borrowed for the
// duration of the create/update call and copied into the module; positions
// are required as float xyz triplets plus per-attribute byte strides. A
// stride of 0 excludes an optional attribute (missing normals are then
// generated from triangle geometry, missing colors default to white); for
// the required positions it instead reads as tightly packed triplets.
// transform is a column-major 4x4 model matrix. emissiveColor and
// emissiveIntensity combine into the surface's emission; albedoColor tints
// vertex colors (white when colors are absent). opacity below 1 makes the
// surface transmissive: gather, probe, and shadow rays pass through with
// the (1 - opacity) share (tinted by albedoColor), bent by Snell's law with
// indexOfRefraction (>= 1) at the entering interface.
struct GiSurfaceDescription
{
    const float         *positions           = nullptr;
    std::uint32_t        positionStrideBytes = 0;
    const float         *normals            = nullptr;
    std::uint32_t        normalStrideBytes   = 0;
    const float         *colors             = nullptr;
    std::uint32_t        colorStrideBytes    = 0;
    const std::uint32_t *indices             = nullptr;
    std::uint32_t        vertexCount         = 0;
    std::uint32_t        indexCount          = 0;

    float         transform[16]     = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f};
    float         albedoColor[3]    = {1.0f, 1.0f, 1.0f};
    float         emissiveColor[3]  = {0.0f, 0.0f, 0.0f};
    float         emissiveIntensity = 0.0f;
    // Transmission pair: opacity in [0, 1] (1 = opaque; below 1 rays
    // transmit the (1 - opacity) share, tinted by the albedo color), and the
    // medium's index of refraction (>= 1) for the refracted continuation.
    float         opacity           = 1.0f;
    float         indexOfRefraction = 1.5f;
    std::uint32_t flagBits          = 0;
};

// One radiance probe of the exported world-space cache. sh2 holds the
// spherical-harmonic-2 projection of the probe's incoming RADIANCE as nine
// (basis index, RGB) triplets ordered C00, C1-1, C10, C11, C2-2, C2-1, C20,
// C21, C22, each coefficient already multiplied by its basis normalization
// constant, so evaluateSh2Irradiance is a plain polynomial and a constant
// radiance L evaluates to exactly pi * L. validity is 0 for probes without
// data or inside geometry and 1 for fully usable probes (it is reduced for
// probes that partly look into a wall).
struct GiProbe
{
    float         position[3] = {0.0f, 0.0f, 0.0f};
    float         sh2[9][3]   = {};
    float         validity    = 0.0f;
    std::uint32_t level       = 0;
};

// Layout of the exported probe grid: camera-centered, uniformly spaced probe
// boxes per clip level (level 0 = fine spacing, level 1 = level1_spacing_factor
// times coarser, covering that many times the extent). Probes are exported in
// logical lattice order (x fastest, then y, then z), level 0 first, then
// level 1; probe 0 of a level sits at its lattice anchor.
struct GiProbeHeader
{
    float         gridCenter[3]       = {0.0f, 0.0f, 0.0f};
    float         level0Spacing       = 2.0f;
    std::uint32_t level0Dimensions[3] = {0u, 0u, 0u};
    float         level1Spacing       = 8.0f;
    std::uint32_t level1Dimensions[3] = {0u, 0u, 0u};
    std::uint32_t directionResolution = 16u;
    std::uint32_t probeCount          = 0u;
    std::uint32_t level0ProbeOffset   = 0u;
    std::uint32_t level1ProbeOffset   = 0u;
};

// Frame counters and cumulative totals. The per-frame fields are reset by
// evaluate() (screenProbesThisFrame / gatherRaysThisFrame are non-zero only
// when the debug gather ran that frame); totalRaysTraced and frameIndex accumulate, and
// framesSkippedStatic counts the evaluate() calls that returned early under
// the static-frame skip (camera, scene, and environment unchanged, caches
// converged).
struct GiStatistics
{
    std::uint64_t frameIndex               = 0;
    std::uint32_t surfaces                 = 0;
    std::uint64_t triangles                = 0;
    std::uint32_t cards                    = 0;
    std::uint64_t cardTexels               = 0;
    std::uint32_t captureTexelsThisFrame   = 0;
    std::uint32_t directTexelsThisFrame    = 0;
    std::uint32_t indirectTexelsThisFrame  = 0;
    std::uint32_t indirectRaysThisFrame    = 0;
    std::uint32_t probeTracesThisFrame     = 0;
    std::uint32_t screenProbesThisFrame    = 0;
    std::uint32_t gatherRaysThisFrame      = 0;
    std::uint64_t totalRaysTraced          = 0;
    std::uint32_t framesSkippedStatic      = 0;
    double        lastEvaluateMilliseconds = 0.0;
};

// Device-resolved budgets produced by resolveProfile: explicit configuration
// wins; automatic values scale the configured budgets by device class.
struct GiProfile
{
    std::uint32_t captureTexelsPerFrame  = 262144u;
    std::uint32_t directTexelsPerFrame   = 65536u;
    std::uint32_t indirectTexelsPerFrame = 65536u;
    std::uint32_t probeUpdatesPerFrame   = 48u;
};

// Debug views written by debugExport as binary P6 PPM files at the configured
// internal resolution. FinalImage is the composed image (sun + direct lights +
// GI + emissive through the pbr tone mapping); GiOnly isolates the indirect
// term; RadianceScene renders the surface cache as the ray-tracing proxy sees it
// (direct + indirect radiance); SurfaceCache shows captured albedo/coverage;
// ProbeIrradiance visualizes the world probe grid; Reference path traces the
// same camera as ground truth for validation.
enum class GiDebugView : std::uint32_t
{
    FinalImage = 0,
    GiOnly,
    RadianceScene,
    SurfaceCache,
    ProbeIrradiance,
    Reference
};

// Function-pointer table for the "radiance_cache" capability. Mutators and
// evaluate serialize against each other on one module mutex; copyProbes and
// statistics are the read side. Calls never throw; failures are reported
// through the error out-parameters.
struct RadianceCacheCapability
{
    static constexpr const char *capabilityName = "radiance_cache";

    std::uint32_t abi  = radianceCacheCapabilityAbi;
    void         *self = nullptr;

    GiSurfaceHandle (*createSurface)(void *self, const GiSurfaceDescription *description, GiError *error);
    bool (*updateSurface)(void *self, GiSurfaceHandle surface, const GiSurfaceDescription *description, GiError *error);

    // Fast per-frame transform path for Dynamic surfaces; geometry and
    // material are kept. Returns false for unknown handles.
    bool (*updateSurfaceTransform)(void *self, GiSurfaceHandle surface, const float *transformColumnMajor16);

    bool (*destroySurface)(void *self, GiSurfaceHandle surface);

    // Camera used for probe placement, capture priority, and the screen-space
    // final gather. viewProjection is column-major 4x4; position is world eye.
    bool (*setView)(void *self, const float *viewProjectionColumnMajor16, const float *cameraPosition3);

    // Runs one Lumen-style GI frame on the kernel task scheduler and blocks
    // until finished (the render_graph::execute pattern). Requires setView.
    bool (*evaluate)(void *self, double deltaSeconds, GiError *error);

    // Snapshot read of the world-space radiance cache. With a null/zero
    // capacity buffer it still reports the version and fills outHeader, so
    // callers can size their buffer and detect changes. Returns the number of
    // probes copied (truncated to capacity).
    std::uint32_t (*copyProbes)(
        void *self, GiProbe *output, std::uint32_t capacity, GiProbeHeader *outHeader, std::uint64_t *outVersion);

    bool (*statistics)(void *self, GiStatistics *outStatistics);
    bool (*resolveProfile)(void *self, const renderer::DeviceProfile *device, GiProfile *outProfile);

    // Writes one debug view as a binary PPM file. The view reflects the most
    // recent evaluate() state; export failures (bad path) report ExportFailed.
    bool (*debugExport)(void *self, GiDebugView view, const char *filePath, GiError *error);

    // Samples the photon caustics pass's deposited irradiance at a world-space
    // position: the caustic receiver surfaces whose bounds contain the
    // position are read with the same card-texel bilinear the gather's display
    // composite applies, and the first receiver with caustic state at the
    // position answers. Returns false when no receiver covers the position or
    // none holds caustic state (outIrradiance stays untouched then). The read
    // side of the registry, like copyProbes: it never waits on a running
    // evaluate, and callers between evaluates (the reflection field's frame)
    // see the last deposited state. A Lambertian caller turns the irradiance
    // into outgoing radiance as albedo / pi * E.
    bool (*sampleCausticsIrradiance)(
        void *self, const float *position3, const float *normal3, float *outIrradiance3);
};

// The screen-space final gather exported as a second capability: the
// per-pixel indirect irradiance field the module's Lumen-style gather
// (screen probes on a jittered grid, short surface-cache rays, world-cache
// misses, depth-plane interpolation, temporal accumulation, bilateral
// filter) resolves for one camera. The field carries the contact-scale
// structure the world probe lattice is too coarse to resolve - short-range
// occlusion at surface meet lines and the caustic irradiance the photon
// pass deposited on the card texels the gather rays land on - so renderers
// that composite it replace their coarse probe-walk ambient with it. ABI 1.
inline constexpr std::uint32_t radianceGatherCapabilityAbi = 1u;
inline constexpr std::uint32_t radianceGatherTexelFloats   = 5u;

// Layout of the exported gather field: width x height texels in row-major
// order, each radianceGatherTexelFloats floats - the irradiance RGB triplet,
// a validity in 0..1 (0 for background texels and texels whose neighborhood
// never produced a sample; renderers refuse them and keep their fallback
// ambient), and the contact occlusion in 0..1: the share of the texel's
// gather rays that found no surface within the near-contact radius (1 = the
// neighborhood's upper hemisphere is open, 0 = every ray met geometry right
// next to the shaded point). A Lambertian renderer multiplies its indirect
// term by it to keep the tight dark bands where surfaces meet, which the
// irradiance alone averages away at field resolution. viewProjection is the column-major matrix of the
// setView the field was evaluated with, cameraPosition the matching eye
// point. The irradiance is the indirect-only term: a Lambertian renderer
// turns it into outgoing radiance as albedo / pi * E, exactly like the
// probe-grid irradiance of the shared evaluation helpers.
struct RadianceGatherHeader
{
    std::uint32_t width            = 0;
    std::uint32_t height           = 0;
    float         viewProjection[16] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f};
    float         cameraPosition[3]  = {0.0f, 0.0f, 0.0f};
};

// Function-pointer table for the "radiance_cache.gather" capability. The
// field evaluation is activated by setViewport: a renderer that intends to
// composite the gather pushes its output resolution once (and on every
// resize), and every evaluate() afterwards resolves the field at that size
// divided by the configured gather.field_scale. Without a viewport the pass
// never runs, so the capability costs nothing for consumers that only read
// the world probe grid. Mutators serialize against evaluate on the same
// module mutex as the radiance_cache capability.
struct RadianceGatherCapability
{
    static constexpr const char *capabilityName = "radiance_cache.gather";

    std::uint32_t abi  = radianceGatherCapabilityAbi;
    void         *self = nullptr;

    // Output resolution the field is evaluated for (the renderer's target
    // size; the pass divides it by [radiance_cache.gather] field_scale).
    // Zero dimensions deactivate the field.
    bool (*setViewport)(void *self, std::uint32_t viewportWidth, std::uint32_t viewportHeight);

    // Snapshot read of the gather field. With a null/zero-capacity buffer it
    // still fills outHeader and reports the version, so callers can size
    // their buffer and detect changes (the version freezes together with the
    // static-frame skip). The buffer layout is radianceGatherTexelFloats
    // floats per texel - the irradiance triplet followed by the validity -
    // in row-major order. Returns the number of texels copied (truncated to
    // capacity).
    std::uint32_t (*copyIrradianceField)(void *self, float *output, std::uint32_t capacityTexels,
        RadianceGatherHeader *outHeader, std::uint64_t *outVersion);
};

// Core of the cosine-lobed SH2 irradiance evaluation below; the sh2 array
// holds 9 RGB coefficient triplets in the GiProbe basis order.
inline void evaluateSh2Irradiance(const float sh2[27], const float normal[3], float outIrradiance[3])
{
    constexpr float k0 = 3.141592654f;  // pi      (band 0)
    constexpr float k1 = 2.094395102f;  // 2pi/3   (band 1)
    constexpr float k2 = 0.785398163f;  // pi/4    (band 2)

    const float x  = normal[0];
    const float y  = normal[1];
    const float z  = normal[2];
    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;

    for (std::uint32_t channel = 0; channel < 3; ++channel)
    {
        const float *coefficient = sh2 + channel;
        outIrradiance[channel]   = k0 * coefficient[0]
                               + k1 * (coefficient[3] * y + coefficient[6] * z + coefficient[9] * x)
                               + k2 * (coefficient[12] * x * y + coefficient[15] * y * z
                                       + coefficient[18] * (3.0f * zz - 1.0f) + coefficient[21] * x * z
                                       + coefficient[24] * (xx - yy));
    }
}

// Evaluates the cosine-lobed irradiance of a probe's SH2 radiance along
// normal (the Ramamoorthi & Hanrahan convolution of a clamped cosine lobe
// with band constants A0 = pi, A1 = 2pi/3, A2 = pi/4). Shared with consumers
// so a renderer hook and the module agree on the representation; returns
// irradiance, which a Lambertian BRDF turns into outgoing radiance as
// albedo / pi * irradiance. The sh2 basis order matches GiProbe.
inline void evaluateProbeIrradiance(const GiProbe &probe, const float normal[3], float outIrradiance[3])
{
    evaluateSh2Irradiance(&probe.sh2[0][0], normal, outIrradiance);
}

// Tangent-plane factor of the probe interpolation weights, part of the
// exported-grid convention the module's world cache, the renderer's probe
// walk, and reflection_field's hit lighting all share (the DDGI anti-leak
// rule): a probe on the far side of the shaded point's tangent plane
// describes the space behind the surface, and blending it in lights surfaces
// through walls. `toProbe` is the shaded point to probe offset and
// `inverseSpacing` the reciprocal of the level's lattice spacing (hoisted by
// the callers - the factor runs per probe corner); the alignment is measured
// against the lattice scale instead of the exact probe distance (an offset
// of one spacing along the normal reaches the saturated ends, which keeps
// the per-probe cost free of the square root a normalized direction would
// need while preserving the zero crossing exactly at the tangent plane).
// Probes lying in the plane (a lattice row sitting exactly on a floor) keep
// half weight. The alignment form below is the factored variant hot paths
// use: the pixel-level base alignment (normal . (anchor - position), scaled)
// plus the corner's unit offsets dotted with the normal, which reproduces
// the direct form without re-deriving the corner position per probe.
inline float probeTangentPlaneAlignmentFactor(float alignment)
{
    const float scaled = alignment * 0.5f + 0.5f;
    return scaled < 0.0f ? 0.0f : (scaled > 1.0f ? 1.0f : scaled);
}

inline float probeTangentPlaneFactor(const float normal[3], const float toProbe[3], float inverseSpacing)
{
    const float alignment = (normal[0] * toProbe[0] + normal[1] * toProbe[1] + normal[2] * toProbe[2]) * inverseSpacing;
    return probeTangentPlaneAlignmentFactor(alignment);
}

} // namespace radiance_cache
} // namespace engine

