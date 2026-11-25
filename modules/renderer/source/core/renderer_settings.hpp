// The [renderer] settings model: every configuration knob in one struct,
// read from the layered configuration store (or derived from the host
// hardware when [renderer] auto_config is on). renderer.cpp holds a
// RendererSettings and re-reads it on construct and on every
// ConfigurationReloadedEvent that touches a renderer.* key.
//
// Field names mirror their configuration keys (max_frames_in_flight ->
// maxFramesInFlight), and the headless rasterizer's implementation-tuning
// keys carry the headless_ prefix both in config and here; the conceptually
// backend-agnostic keys (culling, LOD policy, quality) stay unprefixed.

#pragma once

#include "render_shared.hpp"

#include <engine/kernel/participant_interface.hpp>
#include <engine/renderer/draw_sort.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace engine
{
namespace renderer
{

// What the configuration asked for, before hardware awareness runs: "auto"
// resolves to a concrete backend per renderer (see renderer.cpp's backend
// selection); the explicit values never fall back silently.
enum class ConfiguredBackend
{
    Auto, // hardware-aware: Vulkan when the machine has a real GPU, headless otherwise
    Vulkan,
    Headless
};

// Every [renderer] knob. The member initializers are the built-in defaults
// and match modules/renderer/config.toml, so a reload that removes a key
// restores the documented value.
struct RendererSettings
{
    // Backend selection and device bias.
    ConfiguredBackend   backend        = ConfiguredBackend::Auto;
    DevicePreference    devicePreference = DevicePreference::Auto;
    bool                vsync          = false;
    bool                validation     = false;
    SwapchainPresentMode presentMode   = SwapchainPresentMode::Auto;
    bool                autoConfig     = false;

    // Frame pipeline.
    std::uint32_t maxFramesInFlight                 = 3;
    std::uint32_t maxPendingUploads                 = 64;
    std::uint32_t maxTexturesPerRenderer            = 256;
    std::uint32_t maxMaterialSetsPerRenderer        = 256;
    std::size_t   maxPipelineCacheBytes             = 64u * 1024u * 1024u;
    std::uint32_t memoryBudgetUpdateIntervalFrames  = 120;

    // Quality.
    std::uint32_t msaaSamples     = 4;
    float         anisotropyLevel = 1.0f;
    bool          srgbOutput      = false;

    // Geometry processing (backend-agnostic policy; the headless backend
    // implements the LOD chains today).
    bool                          frustumCulling = true;
    bool                          backFaceCulling = true;
    bool                          lodEnabled     = true;
    std::size_t                   lodMinTriangles = 4096;
    std::array<float, 4>          lodThresholds  = {0.02f, 0.05f, 0.1f, 0.2f};
    std::array<std::uint32_t, 3>  lodStrides     = {4u, 16u, 64u};

    // Lighting integration hooks.
    bool giIndirectLighting        = true;
    bool reflectionIndirectSpecular = true;
    // The radiance_cache screen-space gather field (the final gather's
    // per-pixel indirect irradiance, sampled ahead of the probe-walk
    // ambient).
    bool gatherIndirectLighting    = true;

    // Presentation post pass (headless backend): a soft bloom over the
    // tone-mapped output - bright-pass threshold, quarter-resolution
    // separable blur, weighted add-back.
    bool  bloomEnabled    = true;
    float bloomStrength   = 0.35f;
    float bloomThreshold  = 0.8f;

    // Sun shadow mapping (headless backend): the frame's opaque lit draws
    // rasterize into an orthographic sun-aligned depth map the next frame's
    // sun term shades through (percentage-closer filter). shadowFilterRadius
    // is the filter width in map texels; the rebuild is skipped while the
    // caster set and the sun are unchanged.
    bool          sunShadowsEnabled        = true;
    std::uint32_t shadowMapResolution      = 1024;
    float         shadowFilterRadiusTexels = 1.5f;
    // Zenith sky-visibility shadows (the ambient term's grounding map).
    bool          skyShadowsEnabled        = true;

    // Camera response presentation pass (headless backend): the photographic
    // layer over the finished output - a contrast curve pivoting at mid-gray,
    // a radial lens falloff, and deterministic sensor grain.
    float cameraContrast = 1.0f;
    float cameraVignette = 0.18f;
    float cameraGrain    = 0.01f;

    // Film antialiasing presentation pass (headless backend): a
    // luminance-directed edge blend over the finished rows, smoothing the
    // staircase the rasterizer leaves on screen-space diagonal silhouettes.
    // The strength scales the cross-edge stage that softens the one-texel
    // hard step a straight silhouette keeps between supersample columns
    // (0 keeps the along-edge blend only).
    bool  filmAntialiasingEnabled  = false;
    float filmAntialiasingStrength = 1.0f;

    // Supersample resolve filter (headless backend): the sample-block average
    // runs through the bilinear tent kernel over the 3x3 block neighborhood
    // instead of the plain box average, spreading every silhouette over a
    // two-texel ramp the way a sensor's optical low-pass filter does.
    bool resolveTentEnabled = true;

    // Multiplier on the gather field's contact-occlusion channel (headless
    // backend composites the field; 1 keeps the module's reported occlusion).
    float gatherOcclusionStrength = 1.0f;

    // Draw scheduling.
    bool                drawSortEnabled         = true;
    RenderDrawSortMode  drawSortMode            = RenderDrawSortMode::MeshThenMaterial;
    std::uint32_t       drawSortMinimumCommands = 2;
    bool                instanceBatchingEnabled = true;
    std::uint32_t       instanceBatchMinimumCount = 4;
    std::uint32_t       instanceStreamMegabytes = 4;
    bool                textureMipmapsEnabled   = true;

    // Shell fur.
    bool          furEnabled       = true;
    std::uint32_t furMaximumLayers = 16;

    // GPU diagnostics.
    bool gpuTimestampsEnabled = false;

    // Headless rasterizer tuning. headlessTileParallel's built-in default is
    // the below-four-cores value; the reader resolves the adaptive default
    // (on from four logical processors) when the key is absent.
    std::size_t  headlessTransformCacheBudgetBytes     = 256u * 1024u * 1024u;
    bool         headlessTileParallel                  = false;
    bool         headlessBatchedTileRasterization      = true;
    std::uint32_t headlessParallelTriangleThreshold    = 4096;
    std::size_t  headlessPixelsPerLockGroup            = 16;
    std::uint32_t headlessBandSplitTriangleThreshold   = 4096;
    float        headlessBandSplitMinimumScreenFraction = 0.125f;
    std::uint32_t headlessParallelTransformVertexThreshold = 8192;
    std::uint32_t headlessTilePixels                   = 64;
    std::size_t  headlessMaxBatchTileRecords           = 512u * 1024u;

    // Content-owned clear color (beginFrame with frame == nullptr).
    std::array<float, 4> clearColor = {0.0f, 0.0f, 0.0f, 1.0f};
};

// Reads one [renderer] settings set from the participant's configuration
// view. With [renderer] auto_config = true every performance, quality and
// memory knob is derived from the detected hardware instead (GPU probe,
// logical processor count, installed RAM) and the other tuning keys have no
// effect; clear_color stays content-owned and is always honored.
// Out-of-range numeric values are clamped to their validated ranges with a
// stderr warning; unknown enum strings warn and fall back.
RendererSettings readRendererSettings(ParticipantContext &context);

} // namespace renderer
} // namespace engine
