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
    // MSAA-style interior shading for the supersampled headless target:
    // output pixels whose whole sample block one triangle covers shade
    // once at the block centre instead of once per sample (coverage, depth
    // and the resolve stay per sample, so edges keep their anti-aliasing).
    bool         headlessMsaaInteriorShading           = true;
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
