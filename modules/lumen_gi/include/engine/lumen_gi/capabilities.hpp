// Public capability surface of the lumen_gi module.
//
// Declares the POD types and function-pointer tables consumers receive
// through ParticipantContext::acquireCapability for the "lumen_gi" capability
// (ABI 3): a registry of GI scene surfaces, a camera-driven frame evaluation that runs
// a Lumen-style dynamic global illumination pipeline (card-based surface
// cache, amortized direct/indirect lighting, world-space radiance probes,
// screen-space final gather), a probe-grid snapshot copy API for renderers,
// device-based profile resolution, statistics, and debug-view PPM exports.

#ifndef ENGINE_LUMEN_GI_CAPABILITIES_HPP
#define ENGINE_LUMEN_GI_CAPABILITIES_HPP

#include <engine/renderer/device_profile.hpp>

#include <cstdint>

namespace engine
{
namespace lumen_gi
{

// ABI version of this capability struct; consumers must match it. History:
// 1 -> 2 added GiStatistics::framesSkippedStatic (the static-frame skip
// reports how many evaluate() calls returned early because the camera, the
// scene, and the environment were all unchanged and the caches converged).
// 2 -> 3 added GiError::EvaluationFailed and GiStatistics::indirectRaysThisFrame,
// and defined GiProbe::sh2 as irradiance-ready coefficients (see GiProbe).
inline constexpr std::uint32_t lumenGiCapabilityAbi = 3u;
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
// are required as float xyz triplets plus per-attribute byte strides (0
// excludes the attribute; missing normals are generated from triangle
// geometry). transform is a column-major 4x4 model matrix. emissiveColor and
// emissiveIntensity combine into the surface's emission; albedoColor tints
// vertex colors (white when colors are absent).
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
// term; LumenScene renders the surface cache as the ray-tracing proxy sees it
// (direct + indirect radiance); SurfaceCache shows captured albedo/coverage;
// ProbeIrradiance visualizes the world probe grid; Reference path traces the
// same camera as ground truth for validation.
enum class GiDebugView : std::uint32_t
{
    FinalImage = 0,
    GiOnly,
    LumenScene,
    SurfaceCache,
    ProbeIrradiance,
    Reference
};

// Function-pointer table for the "lumen_gi" capability. Mutators and
// evaluate serialize against each other on one module mutex; copyProbes and
// statistics are the read side. Calls never throw; failures are reported
// through the error out-parameters.
struct LumenGiCapability
{
    static constexpr const char *capabilityName = "lumen_gi";

    std::uint32_t abi  = lumenGiCapabilityAbi;
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

} // namespace lumen_gi
} // namespace engine

#endif
