// Headless (software) renderer backend: a CPU rasterizer that writes RGBA8
// pixels straight into the window module's surface buffer. Same PBR shading
// model as the Vulkan fragment shader, plus frustum/back-face culling,
// distance-based LOD, a vertex transform cache, and optional task-scheduler
// parallelism. Used when the surface API is not Vulkan or "headless" is
// configured.

#pragma once

// The tile-parallel rasterizer's shared pipeline (prepareFanTriangles /
// rasterizeFanRecord / shadeTrianglePixel / fanPixelWindow) must inline into
// its callers to keep the per-triangle/per-pixel cost of a single fused loop
// (an outlined call per covered pixel measurably regresses the frame time).
#if defined(__GNUC__) || defined(__clang__)
#define ENGINE_RASTERIZER_INLINE inline __attribute__((always_inline))
#else
#define ENGINE_RASTERIZER_INLINE inline
#endif

#include "headless_types.hpp"
#include "render_backend.hpp"
#include "render_shared.hpp"
#include "sun_shadows.hpp"
#include "tile_bins.hpp"
#include "wide_shading.hpp"

#include <engine/direct_lighting/capabilities.hpp>
#include <engine/kernel/task_scheduler.hpp>
#include <engine/pbr/capabilities.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/device_profile.hpp>
#include <engine/renderer/material.hpp>
#include <engine/renderer/mesh.hpp>
#include <engine/window/capabilities.hpp>

#include <glm/ext/matrix_float3x3.hpp>
#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/ext/vector_float4.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <new>
#include <unordered_map>
#include <vector>

namespace engine
{
namespace renderer
{

// Per-tile amortization of the probe-grid ambient walk (see GiAmbientSample)
// over a two-tier grid of sample neighborhoods: 8x8-sample blocks first, 4x4
// sample quads second (4x4 and 2x2 output pixels at the default 2x2
// supersampling). One entry per block or quad holds the last walk's inputs
// and result; a sample of the record (clipped triangle) that published the
// entry reuses it outright (the record's surface is continuous, so its gates
// pass by construction), a sample of another record reuses it when the
// same-plane and same-normal gates pass, and both failing runs the exact
// per-pixel walk and publishes it into the tier it missed. A flat expanse
// therefore pays one probe walk per block, a curvature edge or silhouette
// pays one per quad, and a pixel that fails both tiers runs the exact
// evaluation. One cache lives per rasterizing worker and is reset when it
// moves to its next tile.
struct GiTileCache
{
    struct Entry
    {
        glm::vec3     position{0.0f};
        glm::vec3     normal{0.0f, 1.0f, 0.0f};
        glm::vec3     irradiance{0.0f};
        float         coverage = 0.0f;
        float         occlusion = 1.0f;
        // Record (clipped triangle) that published this entry. A reader of
        // the SAME record sits on the same continuous surface, so its gate
        // outcome is decided by the tag alone: the tier check becomes a
        // compare instead of the plane/normal arithmetic, while a
        // cross-record reader keeps the full gates.
        std::uint32_t recordTag = 0u;
        bool          valid     = false;
    };

    // Same-plane gate: the normal projection of the position delta between
    // the cached walk's input and this pixel's stays below this many meters,
    // so a reused sample differs from the exact walk only by the curvature
    // the gate tolerates.
    static constexpr float kPlaneTolerance = 0.05f;
    // Same-normal gate: the dot between the cached walk's normal and this
    // pixel's. The block tier shares the quad tier's tolerance; a 4x4-sample
    // block spans two output pixels, whose normals agree to this dot wherever
    // the surface is locally flat.
    static constexpr float kNormalTolerance = 0.995f;

    std::uint32_t      quadStride = 0;  // 4x4-sample quad columns of the owning tile
    std::vector<Entry> quadEntries;
    std::uint32_t      blockStride = 0; // 8x8-sample block columns of the owning tile
    std::vector<Entry> blockEntries;

    // `sampleColumns`/`sampleRows` are the owning tile's extents in
    // framebuffer samples; the tier grids round up so partial blocks and
    // quads at a tile's right/bottom border still index safely. At the
    // default 2x2 supersampling the tiers are the documented 4x4-pixel
    // blocks and 2x2-pixel quads; other sample scales keep the same sample
    // extents (the record tags and gates, not the tier geometry, decide
    // correctness).
    void resetForTile(std::uint32_t sampleColumns, std::uint32_t sampleRows)
    {
        quadStride = std::max((sampleColumns + 3u) / 4u, 1u);
        quadEntries.assign(
            static_cast<std::size_t>(quadStride) * std::max((sampleRows + 3u) / 4u, 1u), Entry{});
        blockStride = std::max((sampleColumns + 7u) / 8u, 1u);
        blockEntries.assign(
            static_cast<std::size_t>(blockStride) * std::max((sampleRows + 7u) / 8u, 1u), Entry{});
    }

    // Resolves the ambient sample for one shaded sample at band-relative
    // coordinates (x, y): the block tier answers when its tag or gates pass,
    // the quad tier when the block's fail, and both failing runs `walk` at
    // this sample's exact inputs and publishes the result into both tiers
    // (the quad tier keeps serving the samples that only missed the block,
    // and the block tier gets a fresh chance at flat content on the next
    // sample). `walk` is (position, normal, outIrradiance, outCoverage) ->
    // bool, the GiAmbientSample contract of giProbeGridIrradiance.
    template <typename Walk>
    GiAmbientSample resolve(
        std::int32_t x, std::int32_t y, std::uint32_t recordTag, const glm::vec3 &position,
        const glm::vec3 &normal, Walk &&walk)
    {
        GiAmbientSample sample;
        if (!blockEntries.empty())
        {
            Entry &block = blockEntries[static_cast<std::size_t>(y >> 3) * blockStride
                + static_cast<std::size_t>(x >> 3)];
            if (block.valid && (block.recordTag == recordTag
                    || (glm::dot(normal, block.normal) >= kNormalTolerance
                        && std::fabs(glm::dot(normal, block.position - position)) <= kPlaneTolerance)))
            {
                sample.irradiance = block.irradiance;
                sample.coverage   = block.coverage;
                sample.occlusion  = block.occlusion;
                sample.valid      = true;
                return sample;
            }
        }
        if (!quadEntries.empty())
        {
            Entry &quad = quadEntries[static_cast<std::size_t>(y >> 2) * quadStride
                + static_cast<std::size_t>(x >> 2)];
            const glm::vec3 positionDelta = quad.position - position;
            if (quad.valid && (quad.recordTag == recordTag
                    || (glm::dot(normal, quad.normal) >= kNormalTolerance
                        && std::fabs(glm::dot(normal, positionDelta)) <= kPlaneTolerance)))
            {
                sample.irradiance = quad.irradiance;
                sample.coverage   = quad.coverage;
                sample.occlusion  = quad.occlusion;
                sample.valid      = true;
                return sample;
            }
            sample.valid = walk(position, normal, sample.irradiance, sample.coverage, sample.occlusion);
            quad.position   = position;
            quad.normal     = normal;
            quad.irradiance = sample.irradiance;
            quad.coverage   = sample.coverage;
            quad.occlusion  = sample.occlusion;
            quad.recordTag  = recordTag;
            quad.valid      = true;
        }
        else
        {
            sample.valid = walk(position, normal, sample.irradiance, sample.coverage, sample.occlusion);
        }
        if (!blockEntries.empty())
        {
            Entry &block = blockEntries[static_cast<std::size_t>(y >> 3) * blockStride
                + static_cast<std::size_t>(x >> 3)];
            block.position   = position;
            block.normal     = normal;
            block.irradiance = sample.irradiance;
            block.coverage   = sample.coverage;
            block.occlusion  = sample.occlusion;
            block.recordTag  = recordTag;
            block.valid      = true;
        }
        return sample;
    }
};

// The per-tile GI cache plus the owning record's tag, threaded through the
// rasterizer cores as one pointer: the GI tile cache amortizes the
// probe-grid ambient walk (the caller resolves it before shading), and the
// tag identifies the record (clipped triangle) whose continuous surface the
// cache's trust check keys on.
struct TileShadingCache
{
    GiTileCache   *gi        = nullptr;
    std::uint32_t  recordTag = 0u;
};

// Options forwarded from [renderer] configuration at renderer creation.
struct HeadlessBackendOptions
{
    // Cap for the per-mesh transform cache (see the cache entry comment by
    // transformCaches_): the sum of all cached meshes' entries stays within
    // this budget, least-recently-used entries evict when a new mesh does
    // not fit.
    std::size_t transformCacheBudgetBytes = 256u * 1024u * 1024u;
    bool        frustumCullingEnabled     = true;
    bool        backFaceCullingEnabled    = true;

    // Anti-aliasing: the renderer.msaa_samples request (1..8). The headless
    // path maps it onto an integer supersampling grid - the smallest grid of
    // {1x1, 2x1, 2x2, 4x2} holding at least the requested count (1->1, 2->2,
    // 3/4->4, 5..8->8 samples per pixel) - rasterizes that many pixels per
    // output pixel and averages each block at presentation, the software
    // counterpart of the Vulkan backend's multisample resolve.
    std::uint32_t msaaSamples             = 1;

    bool           lodEnabled                = true;
    // Tile-based parallel rasterization for single draws
    // (renderer.headless_tile_parallel, default adaptive: on from four
    // logical processors, off below - the serial per-draw binning phase
    // loses to the band/triangle-range split on two-core hosts, measured
    // ~+55% frame time). Bins one draw's triangles into 64x64 screen tiles
    // with exclusive per-worker ownership (no per-pixel locks).
    bool           tileParallelEnabled       = false;
    // Batched tile rasterization for whole draw lists
    // (renderer.headless_batched_tile_rasterization, default on): one
    // binning pass over every item of a drawMeshBatch call, then every
    // worker rasterizes its tiles with per-tile submission order preserved.
    // Unlike the per-draw tile path this pays off from two workers up: the
    // fork/join and the binning run once per frame instead of once per
    // draw, which is what makes many-small-draw scenes (streamed chunk
    // meshes) scale across the machine.
    bool           batchedTileRasterizationEnabled = true;
    // 16-wide AVX-512 shading runs (renderer.headless_wide_runs, default
    // on): shade sixteen horizontally consecutive samples of a run-eligible
    // record per pass through the wide kernel (see wide_shading.hpp) when
    // the running CPU carries the AVX-512 foundation. The kernel mirrors
    // the 4-wide pass operation for operation, so the wide path returns
    // bit-identical colors; records the wide path does not model keep the
    // existing 4-wide runs.
    bool           wideRunsEnabled = true;
    TaskScheduler *taskScheduler             = nullptr; // borrowed; parallelism disabled when null/not running
    std::uint32_t  parallelTriangleThreshold = 4096;    // triangles per draw before splitting across workers
    std::size_t    pixelsPerLockGroup        = 16;      // pixels per depth lock during parallel rasterization
    std::size_t    lodMinTriangles
        = 4096; // meshes below this triangle count get no LOD chain

    // Draws with at most this many triangles are split by framebuffer bands
    // instead of triangle ranges: full-screen, few-triangle draws (floor,
    // sky dome) have no parallelism without it, and their per-pixel shading
    // dominates the frame. 0 disables band splitting.
    std::uint32_t bandSplitTriangleThreshold = 4096;

    // A draw only pays for a band split when its projected screen extent is
    // at least this fraction of the framebuffer height (small far geometry
    // would spend more on the fork/join than it saves).
    float bandSplitMinimumScreenFraction = 0.125f;

    // Meshes at or above this many vertices transform across workers when a
    // scheduler is available (renderer.headless_parallel_transform_vertices).
    std::uint32_t parallelTransformVertexThreshold = 8192;

    // Screen tile edge for the tile-parallel paths (renderer.headless_tile_pixels);
    // kept a power of two so the tile grid aligns with the framebuffer.
    std::uint32_t tilePixels = 64;

    // Cap on the batched tile path's record scratch (renderer.
    // headless_batch_tile_records): one record is one rasterisable fan
    // triangle, so this bounds the transient memory of the whole-draw-list
    // batching; chunks that would exceed it fall back to the per-draw paths
    // with identical output.
    std::size_t    maxBatchTileRecords         = 512u * 1024u;

    // Presentation bloom (renderer.bloom / bloom_strength / bloom_threshold):
    // a soft glare over the tone-mapped output - bright-pass at the
    // threshold, a quarter-resolution separable Gaussian, and the weighted
    // add-back. Runs after the supersample resolve in endFrame, on the
    // presentation band parallelism.
    bool           bloomEnabled                = true;
    float          bloomStrength               = 0.12f;
    float          bloomThreshold              = 0.85f;

    // Directional sun shadow mapping (renderer.sun_shadows /
    // shadow_map_resolution / shadow_filter_radius): the frame's opaque lit
    // draws rasterize into an orthographic sun-aligned depth map at endFrame
    // and the next frame's sun term shades through a percentage-closer
    // filter. Static scenes skip the rebuild (a hash over the caster set).
    bool           sunShadowsEnabled           = true;
    std::uint32_t  shadowMapResolution         = 1024;
    float          shadowFilterRadiusTexels    = 1.5f;

    // Zenith sky-visibility shadows (renderer.sky_shadows): the same
    // orthographic depth-map machinery aimed straight down, shading the
    // hemispheric ambient / GI diffuse term by what blocks the sky above a
    // point. Contact-scale grounding for panel- and sky-lit scenes where
    // the sun term is off and the probe lattice is too coarse for the tight
    // bands under props.
    bool           skyShadowsEnabled           = true;

    // Camera response (renderer.contrast / vignette / grain): the
    // presentation pass a photograph adds after the tone map - a pivot-point
    // contrast curve, a radial lens falloff, and deterministic sensor grain,
    // applied in linear space over the locked surface rows after the bloom.
    float          cameraContrast              = 1.0f;
    float          cameraVignette              = 0.18f;
    float          cameraGrain                 = 0.01f;

    // Film antialiasing (renderer.film_aa / renderer.film_aa_strength): a
    // luminance-directed edge blend over the finished rows, smoothing the
    // staircase the rasterizer leaves on screen-space diagonal silhouettes.
    // The strength scales the second, cross-edge stage that softens the
    // one-texel hard step a straight silhouette keeps when it lands between
    // supersample columns (0 restores the along-edge-only blend). Off by
    // default; the cost is one extra read-mostly pass over the surface.
    bool           filmAntialiasingEnabled     = false;
    float          filmAntialiasingStrength    = 1.0f;

    // Supersample resolve filter (renderer.headless_resolve_tent): the
    // sample-block average runs through the bilinear tent kernel
    // [1 2 1; 2 4 2; 1 2 1] / 16 over the 3x3 block neighborhood instead of
    // the plain box average. A box resolve reproduces the sample grid's own
    // hard steps (a straight silhouette landing between sample columns keeps
    // a one-texel jump, and shallow diagonals quantize into two-texel
    // terraces); the tent spreads each silhouette over a two-texel ramp the
    // way a sensor's optical low-pass filter does. On by default; the cost
    // is a second separable pass over the sample sums in the resolve.
    bool           resolveTentEnabled          = true;

    // Multiplier on the gather field's contact-occlusion channel
    // (renderer.gather_occlusion_strength): 1 keeps the module's reported
    // occlusion, above 1 deepens the contact bands the ambient term carries.
    float          gatherOcclusionStrength     = 1.0f;

    // Angular-size LOD cutoffs (renderer.lod_thresholds) and the triangle
    // strides of the decimated chains (renderer.lod_strides); level 0 is
    // always the full index list.
    std::array<float, 4>         lodThresholds = {0.02f, 0.05f, 0.1f, 0.2f};
    std::array<std::uint32_t, 3> lodStrides    = {4u, 16u, 64u};
};

// All entry points are main-thread only. drawMeshInstances is implemented as
// a loop over drawMesh. beginFrame (re)allocates the color/depth buffers and
// must be called before any draw; endFrame blits into the window surface.
class HeadlessBackend final : public RenderBackend
{
public:
    HeadlessBackend(window::WindowHandle surfaceHandle, window::WindowCapability *windowCapability,
        const HeadlessBackendOptions &options)
        : tileWidth_(options.tilePixels),
          tileHeight_(options.tilePixels),
          surface_(surfaceHandle),
          transformCacheBudgetBytes_(options.transformCacheBudgetBytes),
          frustumCullingEnabled_(options.frustumCullingEnabled),
          backFaceCullingEnabled_(options.backFaceCullingEnabled),
          lodEnabled_(options.lodEnabled),
          tileParallelEnabled_(options.tileParallelEnabled),
          batchedTileRasterizationEnabled_(options.batchedTileRasterizationEnabled),
          wideRunsActive_(options.wideRunsEnabled && wideshading::avx512Available()),
          taskScheduler_(options.taskScheduler),
          window_(windowCapability),
          parallelTriangleThreshold_(options.parallelTriangleThreshold),
          bandSplitTriangleThreshold_(options.bandSplitTriangleThreshold),
          bandSplitMinimumScreenFraction_(options.bandSplitMinimumScreenFraction),
          pixelsPerLockGroup_(options.pixelsPerLockGroup),
          lodMinTriangles_(options.lodMinTriangles),
          parallelTransformVertexThreshold_(options.parallelTransformVertexThreshold),
          lodStrides_(options.lodStrides),
          lodThresholds_(options.lodThresholds),
          bloomEnabled_(options.bloomEnabled),
          bloomStrength_(options.bloomStrength),
          bloomThreshold_(options.bloomThreshold),
          sunShadowsEnabled_(options.sunShadowsEnabled),
          skyShadowsEnabled_(options.skyShadowsEnabled),
          shadowMapResolution_(options.shadowMapResolution),
          shadowFilterRadiusTexels_(options.shadowFilterRadiusTexels),
          cameraContrast_(options.cameraContrast),
          cameraVignette_(options.cameraVignette),
          cameraGrain_(options.cameraGrain),
          filmAA_(options.filmAntialiasingEnabled),
          filmAAStrength_(std::clamp(options.filmAntialiasingStrength, 0.0f, 2.0f)),
          resolveTent_(options.resolveTentEnabled),
          gatherOcclusionStrength_(options.gatherOcclusionStrength),
          maxBatchTileRecords_(options.maxBatchTileRecords)
    {
        // Supersampling grid from the requested sample count (see
        // HeadlessBackendOptions::msaaSamples); sampleScaleX_/Y_ stay 1 when
        // the request is out of range, leaving the unfiltered path.
        const std::uint32_t samples = options.msaaSamples >= 1u && options.msaaSamples <= 8u
            ? options.msaaSamples
            : 1u;
        if (samples >= 5u)
        {
            sampleScaleX_ = 4u;
            sampleScaleY_ = 2u;
        }
        else if (samples >= 3u)
        {
            sampleScaleX_ = 2u;
            sampleScaleY_ = 2u;
        }
        else if (samples == 2u)
        {
            sampleScaleX_ = 2u;
            sampleScaleY_ = 1u;
        }
        directLights_.reserve(direct_lighting::maxDirectLights);
    }

    const char *name() const override
    {
        return "headless";
    }

    window::WindowHandle surface() const override
    {
        return surface_;
    }

    DeviceClass deviceClass() const override
    {
        return DeviceClass::Cpu;
    }

    const char *deviceName() const override
    {
        return "software rasterizer (CPU)";
    }
    DeviceProfile deviceProfile() const override
    {
        DeviceProfile profile{};
        profile.deviceClass = DeviceClass::Cpu;
        return profile;
    }

    void setViewport(std::uint32_t width, std::uint32_t height) override
    {
        viewportWidth  = width;
        viewportHeight = height;
    }

    void setCamera(const CameraMatrices &camera) override;

    void setDirectLights(const direct_lighting::DirectLight *lights, std::uint32_t count) override;

    // Copies the borrowed radiance_cache probe-grid snapshot; the rasterizer's
    // pbrShade calls then sample it per pixel (validity-weighted trilinear
    // SH2 irradiance). An empty snapshot keeps the legacy hemispheric
    // ambient. The prepared view (level descriptors + bounds constants) is
    // rebuilt here so per-pixel shading reads ready-made constants.
    void setProbeGrid(const GiProbeGrid &grid) override
    {
        // The documented invalid snapshot is {nullptr, 0}; a non-zero count
        // with a null pointer is an upstream contract violation and is
        // rejected (kept as the previous snapshot) instead of reading
        // through null.
        if (grid.probes == nullptr && grid.probeCount != 0)
        {
            return;
        }
        // Copy into a fresh buffer and swap on success: the snapshot copy is
        // the one allocation in this setter, and an allocation failure must
        // keep the previous snapshot (the frame_inputs feeder's documented
        // keep-previous semantics) instead of throwing std::bad_alloc through
        // the renderer capability boundary into the application's frame
        // loop.
        std::vector<radiance_cache::GiProbe> incoming;
        try
        {
            incoming.assign(grid.probes, grid.probes + grid.probeCount);
        }
        catch (const std::bad_alloc &)
        {
            return;
        }
        probeGridProbes_ = std::move(incoming);
        GiProbeGrid copied;
        copied.header     = grid.header;
        copied.probeCount = grid.probeCount;
        copied.probes     = probeGridProbes_.empty() ? nullptr : probeGridProbes_.data();
        probeGrid_        = copied;
        // Prepared once per snapshot: the level descriptors and the level-0
        // bounds-test constants, so per-pixel shading stops rebuilding them
        // for every shaded sample.
        preparedProbeGrid_ = prepareGiProbeGrid(probeGrid_);
    }

    // Copies the borrowed reflection_field field; the rasterizer's pixel
    // path then adds the indirect specular radiance (bilinear field sample
    // through the field's own camera) to every shaded opaque pixel after the
    // PBR pass and before tone mapping. An empty snapshot keeps the legacy
    // specular-only shading.
    void setReflectionField(const ReflectionField &field) override
    {
        if (!field.valid())
        {
            reflectionField_           = ReflectionField{};
            reflectionRadiance_.clear();
            preparedReflectionField_   = PreparedReflectionField{};
            return;
        }
        // The borrowed field is interleaved texels (radiance RGB +
        // roughness; see reflectionFieldTexelFloats), so the copy keeps the
        // whole stride and the pixel-path sampler indexes it the same way.
        // The copy lands in a fresh buffer first and swaps in on success: an
        // allocation failure keeps the previous snapshot (the frame_inputs
        // feeder's documented keep-previous semantics) instead of throwing
        // std::bad_alloc through the renderer capability boundary into the
        // application's frame loop.
        const std::size_t texelCount
            = static_cast<std::size_t>(field.header.width) * field.header.height;
        std::vector<float> incoming;
        try
        {
            incoming.assign(
                field.radiance, field.radiance + texelCount * reflection_field::reflectionFieldTexelFloats);
        }
        catch (const std::bad_alloc &)
        {
            return;
        }
        reflectionRadiance_      = std::move(incoming);
        reflectionField_.header = field.header;
        reflectionField_.radiance = reflectionRadiance_.data();
        // Prepared once per snapshot: the field's view-projection, so the
        // per-pixel bilinear sample stops rebuilding the matrix.
        preparedReflectionField_ = prepareReflectionField(reflectionField_);
    }

    // Copies the borrowed radiance_cache gather field; the pixel path's GI
    // ambient then resolves the gather sample first (validity-gated bilinear
    // through the field's own camera) and falls back to the probe-walk
    // ambient wherever the field refuses the point. An empty snapshot keeps
    // the probe-walk ambient bit-identical.
    void setGatherField(const GatherField &field) override
    {
        if (!field.valid())
        {
            gatherField_            = GatherField{};
            gatherIrradiance_.clear();
            preparedGatherField_    = PreparedGatherField{};
            preparedGatherField_.occlusionStrength = gatherOcclusionStrength_;
            return;
        }
        // The borrowed field is interleaved texels (irradiance RGB +
        // validity; see radianceGatherTexelFloats), so the copy keeps the
        // whole stride and the sampler indexes it the same way. The copy
        // lands in a fresh buffer first and swaps in on success: an
        // allocation failure keeps the previous snapshot (the frame_inputs
        // feeder's documented keep-previous semantics) instead of throwing
        // std::bad_alloc through the renderer capability boundary into the
        // application's frame loop.
        const std::size_t texelCount
            = static_cast<std::size_t>(field.header.width) * field.header.height;
        std::vector<float> incoming;
        try
        {
            incoming.assign(
                field.irradiance, field.irradiance + texelCount * radiance_cache::radianceGatherTexelFloats);
        }
        catch (const std::bad_alloc &)
        {
            return;
        }
        gatherIrradiance_         = std::move(incoming);
        gatherField_.header    = field.header;
        gatherField_.irradiance = gatherIrradiance_.data();
        // Prepared once per snapshot: the field's view-projection, so the
        // per-pixel bilinear sample stops rebuilding the matrix.
        preparedGatherField_      = prepareGatherField(gatherField_);
        preparedGatherField_.occlusionStrength = gatherOcclusionStrength_;
    }

    // The software rasterizer composites the gather field (see
    // setGatherField), so the module core pushes its viewport to the
    // radiance_cache.gather capability for this backend.
    bool consumesGatherField() const override
    {
        return true;
    }

    void setEnvironment(const pbr::PbrEnvironment &environment) override
    {
        environment_    = environment;
        lightDirection_ = glm::normalize(glm::make_vec3(environment.sun.direction));
        sunRadiance_    = glm::make_vec3(environment.sun.color) * environment.sun.intensity
                        * (environment.sun.enabled ? 1.0f : 0.0f);
        ambientSky_     = glm::make_vec3(environment.ambient.skyColor) * environment.ambient.intensity;
        ambientGround_  = glm::make_vec3(environment.ambient.groundColor) * environment.ambient.intensity;
        sunShadowSunActive_ = sunShadowsEnabled_ && environment.sun.enabled
            && (sunRadiance_.x > 0.0f || sunRadiance_.y > 0.0f || sunRadiance_.z > 0.0f);
    }

    void beginFrame(const FrameParameters &frame) override;
    void drawMesh(MeshHandle mesh, const float *modelMatrix, const MeshMaterial &material) override;
    void drawMeshInstances(MeshHandle mesh, const float *modelMatrices, std::uint32_t instanceCount,
        const MeshMaterial &material) override;
    void drawMeshBatch(const MeshBatchItem *items, std::uint32_t itemCount) override;
    void drawFur(MeshHandle mesh, const float *modelMatrix, const MeshMaterial &material,
        const FurLayerParams &params) override;
    void drawFurInstances(MeshHandle mesh, const float *modelMatrices, std::uint32_t instanceCount,
        const MeshMaterial &material, const FurLayerParams &params) override;
    void drawOverlay(const OverlayDrawData &data) override;
    void endFrame() override;

    bool createMesh(MeshHandle id, const MeshDescription &description, RenderError *error) override;
    void destroyMesh(MeshHandle mesh) override;
    bool updateMeshVertices(MeshHandle mesh, const void *vertices, std::uint32_t vertexCount, RenderError *error) override;

    bool createTexture(TextureHandle id, const TextureDescription &description, RenderError *error) override;
    void destroyTexture(TextureHandle texture) override;

    FrameStatistics statistics() const override
    {
        return statistics_;
    }

    HeadlessRenderStatistics headlessStatistics() const
    {
        HeadlessRenderStatistics statistics;
        statistics.framesRendered      = statistics_.framesRendered;
        statistics.trianglesSubmitted  = statistics_.trianglesSubmitted;
        statistics.trianglesRasterized = statistics_.trianglesRasterized;
        return statistics;
    }

private:
    // One vertex after vertex shading: clip-space position plus world-space
    // position/normal for lighting.
    struct TransformedVertex
    {
        glm::vec4 clip{0.0f};
        glm::vec3 worldPosition{0.0f};
        glm::vec3 worldNormal{0.0f};
    };

    // ---- tile-parallel path (opt-in; see HeadlessBackendOptions) ----
    // Screen-tile edge in pixels (renderer.headless_tile_pixels, a power of two):
    // a tile's colour/depth working set stays cache-local, and exclusive
    // ownership needs no per-pixel locks.
    std::uint32_t tileWidth_  = 64;
    std::uint32_t tileHeight_ = 64;

    // One fully shaded corner of a fan triangle (cache entry plus the
    // mesh-sourced varyings). No default member initializers: records are
    // fully assigned on emission.
    struct RasterVertex
    {
        glm::vec4 clip;
        glm::vec3 worldPosition;
        glm::vec3 worldNormal;
        glm::vec3 color;
        glm::vec2 uv;
    };

    // One rasterisable (fan) triangle after transform/near-clip/projection.
    // Produced identically by the tile binning pass and consumed by the tile
    // rasterizer core. setupIndex selects the RasterSetup (material,
    // matrices, culling flags) the record belongs to: single-draw tile paths
    // always emit 0 against a one-element setup array, while the batched
    // tile path bins records from many draws and each record carries its
    // own draw's setup index.
    struct TileBinRecord
    {
        RasterVertex  corners[3];
        float         screenX[3];
        float         screenY[3];
        float         depth[3];
        float         inverseClipW[3];
        glm::vec3     tangent;
        glm::vec3     bitangent;
        float         tangentInverseMax;
        // Perspective-correct UV gradients over the screen triangle (see
        // HeadlessTexture::sampleTrilinear): uv = P(x, y) / S(x, y) with P an
        // affine vec2 and S an affine scalar, so the per-pixel derivative is
        // (gradient - uv * Sgrad) / S. The numerators carry uv * inverseW per
        // corner - the same 1/w-weighted form the attribute interpolation
        // itself uses.
        float         uvNumeratorX[2];
        float         uvNumeratorY[2];
        float         uvDenominatorX;
        float         uvDenominatorY;
        // Mottle footprint (mottle_noise.hpp): the world meters one
        // supersample covers over this record - the record's world extent
        // over its screen extent, computed at emission. The noise floors
        // its octave wavelengths at it so minified draws band-limit the
        // mottle instead of aliasing it into speckle. 0 disables the pass.
        float         noiseFootprintMeters = 0.0f;
        std::uint32_t sourceTriangle;
        std::uint32_t setupIndex;
        // Run-shading dispatch flag (see rasterizeFanRecordRuns), computed at
        // emission: the draw's shading path fits the 4-wide run kernel and
        // the record's framebuffer window amortizes a run. Read per record
        // by the tile loops - a flag read keeps the loop's hoisted state
        // intact (an eligibility CALL per record measurably regressed the
        // inlined per-sample core's codegen).
        bool          shadeRuns = false;
        // Top-left fill rule bits (see fillRuleBitsFor): bit0 = the edge
        // opposite corner 0 accepts boundary pixels, bit1 = edge opposite
        // corner 1, bit2 = edge opposite corner 2. Computed at emission from
        // the screen-space corners; the coverage tests turn the inclusive
        // >= 0 check strict for the edges the rule assigns to the neighbour.
        std::uint8_t fillRuleBits = 0;
    };

    struct StripResult
    {
        std::uint64_t trianglesRasterized = 0; // per-worker partial count, summed by the caller
    };

    // Everything rasterizeTriangles needs for one draw call, gathered once by
    // prepareRasterization: matrices, texture pointers (never null - neutral
    // stand-ins when a map is absent), material factors, and the flags that
    // decide culling and transform-cache use.
    struct RasterSetup
    {
        std::size_t            triangleCount           = 0;
        const std::uint32_t   *indexData               = nullptr;
        glm::mat4              model                   = glm::mat4(1.0f);
        glm::mat4              viewProjection          = glm::mat4(1.0f);
        glm::mat3              modelNormalMatrix       = glm::mat3(1.0f);
        bool                   unlit                   = false;
        // Depth writes follow the material's depthWrite flag; depth testing
        // always applies. Backdrop passes (atmosphere shell, sky dome) draw
        // with writes off so the opaque passes drawn after them own the
        // silhouette interior.
        bool                   depthWrite              = true;
        // Transparent draws (material opacity below 1) alpha blend over the
        // framebuffer with depth writes off; depth testing still applies so
        // opaque geometry in front of the surface keeps occluding it.
        bool                   transparent             = false;
        float                  opacity                 = 1.0f;
        // Per-mesh transform-cache view for this draw: when non-null the
        // geometry pipeline reads the transformed vertices from it instead
        // of transforming per corner (see the .cpp's prepareRasterization).
        bool                   useTransformCache       = false;
        const TransformedVertex *transformCache       = nullptr;
        std::size_t            vertexCount             = 0;
        // Shell-fur draws: when fur is true, vertices are displaced along
        // their object-space normals by (layer/layerCount) * furHeight before
        // the model transform, strand tips droop toward world -Y by
        // gravity * height^2, and the pixel path alpha-blends the strand
        // crop instead of writing depth (mirroring shaders/fur.vert /
        // fur.frag). viewProjectionWorld is the RAW view-projection (the
        // model multiplied out) because the gravity offset happens in world
        // space, after the model transform.
        bool                   fur                     = false;
        float                  furLayerRatio           = 0.0f;
        float                  furLayerIndex           = 0.0f;
        float                  furHeight               = 0.0f;
        float                  furGravity              = 0.0f;
        float                  furDensityBoost         = 6.0f;
        float                  furOuterFade            = 1.0f;
        glm::mat4              viewProjectionWorld     = glm::mat4(1.0f);
        // World-space bounds sphere (valid only while the mesh's local bounds
        // are valid), shared by the frustum pass and the winding vote.
        glm::vec3              boundsWorldCenter       = glm::vec3(0.0f);
        float                  boundsWorldRadius       = 0.0f;
        const HeadlessTexture *baseColorMap            = nullptr;
        const HeadlessTexture *normalMap               = nullptr;
        const HeadlessTexture *metallicRoughnessMap    = nullptr;
        const HeadlessTexture *occlusionMap            = nullptr;
        const HeadlessTexture *emissiveMap             = nullptr;
        glm::vec3              baseColorFactor         = glm::vec3(1.0f);
        float                  metallicFactor          = 0.0f;
        float                  roughnessFactor         = 1.0f;
        // Procedural surface mottle (MeshMaterial::noiseScaleMeters / -
        // Strength, mottle_noise.hpp): the per-draw half of the pass. The
        // per-record half (the sample footprint) rides TileBinRecord.
        float                  noiseScaleMeters        = 0.0f;
        float                  noiseStrength           = 0.0f;
        float                  normalScale             = 1.0f;
        float                  occlusionStrength       = 1.0f;
        glm::vec3              emissiveColor           = glm::vec3(0.0f);
        // Fabric (cloth) shading pair: sheenFactor > 0 blends the GGX
        // specular toward the Charlie/Neubelt fabric lobe and softens the
        // direct-light terminator with the wrap-diffuse weight; doubleSided
        // skips back-face culling and flips back-facing normals toward the
        // viewer (shaders/mesh.frag's uCloth pair).
        float                  sheenFactor             = 0.0f;
        bool                   doubleSided             = false;
        // Alpha-cutout masking (MeshMaterial::alphaCutoff, mesh.frag's
        // uCloth.z): textured opaque draws cull pixels whose sampled
        // base-color alpha lands below the cutoff; 0 keeps every texel.
        float                  alphaCutoff             = 0.0f;
        // Per-pixel distance fog (MeshMaterial's fog triple): the per-sample
        // weight below runs on the sample's own interpolated position, after
        // every lighting and emissive term, in linear space.
        glm::vec3              fogColor                = glm::vec3(0.0f);
        float                  fogScaleMeters          = 1000.0f;
        float                  fogAsymptote            = 0.0f;
        bool                   hasBaseColorMap         = false;
        bool                   hasNormalMap            = false;
        bool                   hasMetallicRoughnessMap = false;
        bool                   hasEmissiveMap          = false;
        bool                   hasEmissive             = false;
        bool                   hasOcclusion            = false;
        glm::vec3              fallbackTangentNormal   = glm::vec3(0.0f, 0.0f, 1.0f);

        bool  enableBackFaceCull = false;
        float frontSign          = 0.0f; // +1/-1 winding convention detected from vertex normals
    };

    [[nodiscard]] std::uint32_t parallelWorkerCount() const noexcept;

    // World-space bounds sphere of a draw, computed once per draw call: the
    // LOD pick in drawMesh needs it, and so do frustum culling and the
    // winding vote inside prepareRasterization. Passing the instance computed
    // for the LOD pick saves the recomputation.
    struct DrawBounds
    {
        glm::vec3 center;
        float     radius;
    };

    // Computes the draw's matrices, culls against the frustum, transforms
    // vertices (through the cache when it pays off), and probes winding to
    // decide whether back-face culling is safe. Returns false when the draw
    // is fully culled or the framebuffer is gone. furParams != nullptr marks
    // a shell-fur draw: furLayer (1..layerCount) is the shell layer this call
    // rasterizes - the transform runs once per layer, so fur draws stamp a
    // fresh transform generation per call and reuse the cache within one
    // layer's triangles.
    bool prepareRasterization(const HeadlessMesh &mesh, const float *modelMatrix, const MeshMaterial &material,
        const std::uint32_t *activeIndices, std::size_t activeIndexCount, RasterSetup &outSetup,
        const DrawBounds *precomputedBounds = nullptr, const FurLayerParams *furParams = nullptr,
        float furLayer = 0.0f);

    // ---- tile-parallel path (opt-in) ----
    // Shared geometry pipeline: transform/clip/project one draw triangle
    // into 0..2 rasterisable fan triangles (near-plane clipping can turn one
    // triangle into a two-triangle fan); back-face culling and degenerate
    // fans drop here. Const, read-only on shared state - safe concurrently.
    // Hot inputs are value-copied into locals before any store to the
    // escaping outFans array (stores through it may alias the setup/cache/
    // mesh reads as far as TBAA is concerned - measured 5x regression of the
    // geometry half without this).
    ENGINE_RASTERIZER_INLINE std::size_t prepareFanTriangles(
        const HeadlessMesh &mesh, const RasterSetup &setup, std::size_t triangleIndex, TileBinRecord (&outFans)[6]) const;

    // Shared rasteriser core: scan one fan record's bounding box clamped to
    // the inclusive pixel window; barycentric coverage with edge-function
    // increments, depth-test, shade, write. Tile owners pass their tile;
    // writes only touch the window, so concurrent calls with disjoint
    // windows never share a pixel. giQuads, when non-null, amortizes the
    // probe-grid ambient walk over its two-tier sample cache (see GiTileCache).
    ENGINE_RASTERIZER_INLINE bool rasterizeFanRecord(const RasterSetup &setup, const TileBinRecord &record,
        std::int32_t xBandBegin, std::int32_t xBandEnd, std::int32_t yBandBegin, std::int32_t yBandEnd,
        TileShadingCache *tiles = nullptr);

    // Run-shading eligibility (the rasterizeFanRecordRuns contract): the
    // kernel models any window width (the wide passes mask the final
    // partial group; the 4-wide core's leftover columns run per sample),
    // so eligibility is purely about the DRAW: no fur, no cloth sheen, no
    // direct lights, and no textured transparency (the lane loop's
    // texel-alpha modulation has no wide twin). Evaluated at emission.
    bool recordRunsEligible(const RasterSetup &setup) const;

    // The 4-wide run variant of rasterizeFanRecord (see quad_shading.hpp):
    // identical window, coverage, depth and commit semantics - run-eligible
    // records shade four consecutive samples per SoA pass. Deliberately
    // outlined (not ENGINE_RASTERIZER_INLINE): keeping the run machinery out
    // of the shared core preserves the per-sample loop's codegen.
    bool rasterizeFanRecordRuns(const RasterSetup &setup, const TileBinRecord &record,
        std::int32_t xBandBegin, std::int32_t xBandEnd, std::int32_t yBandBegin, std::int32_t yBandEnd,
        TileShadingCache *tiles = nullptr);

    // Integer pixel window (inclusive) a fan record can write, clamped to
    // the framebuffer. Shared verbatim by the rasteriser core and the tile
    // binning pass so the bins cover exactly the pixels the core may touch.
    ENGINE_RASTERIZER_INLINE static void fanPixelWindow(const TileBinRecord &record, std::uint32_t framebufferWidth,
        std::uint32_t framebufferHeight, std::int32_t &firstX, std::int32_t &lastX, std::int32_t &firstY,
        std::int32_t &lastY);

    // Shared shading kernel: perspective-correct varying interpolation,
    // material maps, PBR, emissive, tone map + gamma. Const; the caller owns
    // the depth/colour write. giSample, when non-null, replaces the
    // per-pixel probe-grid walk with a caller-evaluated quad sample.
    ENGINE_RASTERIZER_INLINE glm::vec3 shadeTrianglePixel(const RasterSetup &setup, const TileBinRecord &record,
        float weightA, float weightB, float inverseWA, float inverseWB, float inverseWC,
        bool *outFieldTraced = nullptr, const GiAmbientSample *giSample = nullptr,
        const glm::vec3 *interpolatedPair = nullptr, const glm::vec3 *geometryNormalPair = nullptr) const;

    // The GI ambient walk the tile cache and the per-pixel path amortize:
    // the gather field's irradiance first (the radiance_cache final gather,
    // contact-scale occlusion and caustics included), the coarse probe-grid
    // walk wherever the field refuses the position (off its frustum, invalid
    // footprint). The GiAmbientSample contract of giProbeGridIrradiance, so
    // the same cache tiers serve both sources.
    bool resolveGiAmbient(const glm::vec3 &position, const glm::vec3 &normal, glm::vec3 &outIrradiance,
        float &outCoverage, float &outOcclusion) const;

    // Presentation bloom over the locked surface's encoded output rows
    // (see endFrame): bright-pass, quarter-res separable Gaussian, weighted
    // add-back. Defined in the endFrame translation unit.
    void applyPresentationBloom(const window::SurfacePixels &pixels, std::uint32_t columns, std::uint32_t rows);

    // Sun visibility of one shaded point through the built shadow map
    // (sunshadows::sunShadowVisibility's backend-side wrapper: inactive maps
    // and an inactive sun read as fully lit). Const; reads the shared map.
    ENGINE_SHADING_INLINE float sunShadowVisibility(const glm::vec3 &worldPosition, const glm::vec3 &normal) const
    {
        if (!sunShadowMap_.valid)
        {
            return 1.0f;
        }
        return sunshadows::sunShadowVisibility(sunShadowMap_, worldPosition, normal, shadowFilterRadiusTexels_);
    }

    // Zenith sky visibility of one shaded point through the sky-visibility
    // map (the ambient term's grounding factor; inactive maps read as fully
    // open sky).
    ENGINE_SHADING_INLINE float skyShadowVisibility(const glm::vec3 &worldPosition, const glm::vec3 &normal) const
    {
        if (!skyShadowMap_.valid)
        {
            return 1.0f;
        }
        return sunshadows::sunShadowVisibility(skyShadowMap_, worldPosition, normal, shadowFilterRadiusTexels_);
    }

    // Camera response presentation pass (see endFrame): the pivot-point
    // contrast curve, radial vignette, and deterministic sensor grain over
    // the locked surface's encoded rows, in linear space. Defined in the
    // endFrame translation unit.
    void applyCameraResponse(const window::SurfacePixels &pixels, std::uint32_t columns, std::uint32_t rows);
    void applyFilmAntialias(const window::SurfacePixels &pixels, std::uint32_t columns, std::uint32_t rows);

    // Tent supersample resolve (renderer.headless_resolve_tent, see
    // HeadlessBackendOptions::resolveTentEnabled): the sample-block sums run
    // through the separable [1 2 1; 2 4 2; 1 2 1] / 16 kernel across the 3x3
    // block neighborhood, edge blocks clamped. `scratch` is the calling
    // band's private slice (six rows of outputColumns uint16 entries: three
    // raw block-sum rows plus their three horizontally folded rows), so
    // concurrently running bands never share state. Defined in the endFrame
    // translation unit.
    void resolveRowsTent(const std::uint8_t *samples, std::uint32_t framebufferWidth,
        window::SurfacePixels &pixels, std::uint32_t outputColumns, std::uint32_t outputRows,
        std::uint32_t rowBegin, std::uint32_t rowEnd, std::uint16_t *scratch) const;

    // Records one draw as a shadow caster (sun_shadows.hpp's exclusion rule:
    // unlit and transparent draws do not block the sun) and rebuilds the maps
    // at endFrame when the caster set or the light directions changed.
    // Defined in the endFrame translation unit.
    void recordSunShadowCaster(MeshHandle mesh, const float *modelMatrix, const MeshMaterial &material);
    void updateSunShadowMap();
    // One map build over the frame's shared caster geometries: the raster
    // splits across the task scheduler's workers when one is running (the
    // atomic per-cell min-store keeps the map exactly the serial build's),
    // the plain serial build covers every other host.
    void buildShadowMapInParallel(sunshadows::SunShadowMap &map,
        const std::vector<sunshadows::SunShadowGeometry> &geometries, const glm::vec3 &direction,
        const glm::vec3 *receiverMin, const glm::vec3 *receiverMax);
    std::uint64_t sunShadowCastersHash() const;
    std::uint64_t sunShadowStateHash() const;
    std::uint64_t skyShadowStateHash() const;
    std::vector<sunshadows::SunShadowGeometry> sunShadowGeometries() const;
    void sunShadowReceiverBox(glm::vec3 &receiverMin, glm::vec3 &receiverMax, bool &haveBox) const;

    // Phase 1 (calling thread): geometry pipeline over every triangle, then
    // two-pass binning into screen tiles (per-tile lists keep ascending
    // record order, preserving the sequential path's per-pixel triangle
    // order). Returns false on allocation failure or over-cap draws (caller
    // falls back to the default paths). Every record carries setupIndex 0;
    // the caller rasterizes against a one-element setup array.
    bool buildTileRecords(const HeadlessMesh &mesh, const RasterSetup &setup);

    // Batched phase 1 (calling thread): same pipeline over every triangle of
    // every prepared item in batchSetups_ (records appended in submission
    // order, each stamped with its draw's setup index), then the same
    // two-pass binning. Item preparation (mesh lookup, LOD pick,
    // prepareRasterization) happens interleaved in drawMeshBatch before this
    // runs, so the transform cache still holds each item's vertices while
    // its records are emitted. Returns false on allocation failure or when
    // the records exceed the batch cap (caller falls back to per-draw
    // paths for the chunk).
    bool buildBatchTileRecords();

    // Shared phase-1 tail of both tile paths: two-pass binning of the
    // current tileRecords_ into the 64x64 tile grid. Returns false on an
    // empty grid or an allocation failure.
    bool binTileRecords();

    // Phase 2 (one worker): rasterise exactly the tiles with
    // index % workerCount == workerIndex. Records resolve their RasterSetup
    // through record.setupIndex into the setups array (one element for the
    // single-draw paths, one per batched item for the batch). Exclusive
    // tile ownership: no locks. Partial statistics land in *outResult.
    void rasterizeTileRecords(
        const RasterSetup *setups, std::uint32_t workerIndex, std::uint32_t workerCount, StripResult *outResult);

    // Rasterizes triangles [triStart, triEnd) with per-pixel depth testing.
    // With useAtomicDepth the thread takes a spin lock per pixel group
    // (pixelLocks_) before the depth/color update; single-threaded callers
    // pass false and no result target. yBandBegin/yBandEnd (exclusive begin,
    // inclusive end) restrict pixel rows so several workers can share one
    // draw by splitting the framebuffer; the defaults cover every row.
    void rasterizeTriangles(const HeadlessMesh &mesh, const RasterSetup &setup, std::size_t triStart,
        std::size_t triEnd, bool useAtomicDepth, StripResult *outResult, std::uint32_t yBandBegin = 0,
        std::uint32_t yBandEnd = 0xFFFFFFFFu);


    // Textures resolve to neutral stand-ins instead of failing; `neutral`
    // picks which stand-in (white vs flat normal) fits the slot.
    const HeadlessTexture *mapFor(TextureHandle handle, const HeadlessTexture &neutral) const;

    // Angular-size LOD pick shared by the single-draw and batch paths (see
    // the .cpp): returns the index list to rasterize and the draw's
    // world-space bounds sphere when the mesh carries valid bounds.
    const std::vector<std::uint32_t> *activeIndicesForDraw(
        const HeadlessMesh &meshData, const float *modelMatrix, DrawBounds &outBounds, bool &outHaveBounds) const;

    window::WindowHandle surface_                   = 0;
    std::size_t          transformCacheBudgetBytes_ = 256u * 1024u * 1024u; // replaced by options in the ctor
    bool                 frustumCullingEnabled_     = true;
    bool                 backFaceCullingEnabled_    = true;
    bool                 lodEnabled_                = true;
    // Both tile-path flags mirror HeadlessBackendOptions (the constructor
    // always initializes them from it; the defaults here only matter for
    // readers).
    bool                 tileParallelEnabled_       = false;
    bool                 batchedTileRasterizationEnabled_ = true;
    // The wide kernel's effective gate: the configured flag AND the running
    // CPU's AVX-512 foundation (wideshading::avx512Available). Set once in the
    // constructor; the rasterizer consults it per record.
    bool                 wideRunsActive_            = false;

    TaskScheduler *taskScheduler_ = nullptr;

    window::WindowCapability *window_ = nullptr; // borrowed; provides surface pixel lock/unlock

    std::uint32_t parallelTriangleThreshold_      = 4096;
    std::uint32_t bandSplitTriangleThreshold_     = 4096;
    float         bandSplitMinimumScreenFraction_ = 0.125f;
    std::size_t   pixelsPerLockGroup_             = 16;
    std::size_t   lodMinTriangles_                = 4096;
    // Vertex count at which the transform stage parallelizes (see
    // prepareRasterization).
    std::uint32_t parallelTransformVertexThreshold_ = 8192;
    // Triangle strides of the decimated LOD chains (renderer.lod_strides).
    std::array<std::uint32_t, 3> lodStrides_ = {4u, 16u, 64u};
    // LOD angular-size cutoffs (boundsRadius/dist): pick coarser LOD as the value shrinks
    // (renderer.lod_thresholds).
    std::array<float, 4> lodThresholds_ = {0.02f, 0.05f, 0.1f, 0.2f};

    pbr::PbrEnvironment              environment_{};
    CameraMatrices                   camera_{};
    bool                             cameraInitialized_ = false;
    glm::mat4                        viewProjection_{0.0f};
    std::array<glm::vec4, 4>         frustumPlanes_{};
    glm::vec3                        cameraPosition_{0.0f};
    glm::vec3                        lightDirection_{0.0f};
    glm::vec3                        sunRadiance_{0.0f};
    glm::vec3                        ambientSky_{0.0f};
    glm::vec3                        ambientGround_{0.0f};
    std::vector<PreparedDirectLight> directLights_;

    // radiance_cache probe-grid hook: owned probe copy plus the prepared view
    // passed to pbrShade. Kept valid between frames; setProbeGrid replaces
    // both wholesale.
    GiProbeGrid                     probeGrid_{};
    PreparedGiProbeGrid             preparedProbeGrid_{};
    std::vector<radiance_cache::GiProbe> probeGridProbes_;  

    // reflection_field field hook: owned radiance copy plus the borrowed view
    // the pixel path samples. Kept valid between frames; setReflectionField
    // replaces it wholesale.
    std::vector<float>                 reflectionRadiance_;
    ReflectionField                    reflectionField_{};
    // Prepared view of reflectionField_ (the field's own view-projection
    // built once per snapshot); rebuilt by setReflectionField.
    PreparedReflectionField            preparedReflectionField_{};

    // radiance_cache gather-field hook: owned irradiance copy plus the
    // prepared view the pixel path's GI ambient resolves first. Kept valid
    // between frames; setGatherField replaces it wholesale.
    std::vector<float>                 gatherIrradiance_;
    GatherField                        gatherField_{};
    // Prepared view of gatherField_ (the field's own view-projection built
    // once per snapshot); rebuilt by setGatherField.
    PreparedGatherField                preparedGatherField_{};

    std::uint32_t viewportWidth  = 0;
    std::uint32_t viewportHeight = 0;

    // CPU framebuffer: RGBA8 color, float depth (1 = far), and one lock byte
    // per pixelsPerLockGroup_ pixels for parallel depth updates. The buffers
    // carry the supersampled resolution (framebuffer = viewport scaled by
    // sampleScaleX_/Y_); endFrame averages each block down to the surface.
    std::vector<std::uint8_t> color;
    std::vector<float>        depth;
    std::uint32_t             framebufferWidth  = 0;
    std::uint32_t             framebufferHeight = 0;

    // Supersampling factors per output pixel (see HeadlessBackendOptions::
    // msaaSamples); 1/1 reproduces the legacy pixel-for-pixel presentation.
    std::uint32_t sampleScaleX_ = 1;
    std::uint32_t sampleScaleY_ = 1;

    std::vector<std::uint8_t> pixelLocks;

    // Reused across parallel draws: per-worker partial rasterized-triangle
    // counts. Member (not local) so a frame with many parallel draws does
    // not hit the allocator 60+ times (allocation spikes land in the P99).
    std::vector<StripResult> rasterResultScratch_;

    // Presentation bloom scratch (see applyPresentationBloom): the
    // quarter-res bright energy and its blur scratch, kept across frames so
    // steady-state endFrames allocate nothing.
    std::vector<glm::vec3> bloomBright_;
    std::vector<glm::vec3> bloomScratch_;
    bool                   bloomEnabled_    = true;
    float                  bloomStrength_   = 0.12f;
    float                  bloomThreshold_  = 0.85f;

    // Sun shadow mapping (sun_shadows.hpp): the casters recorded by this
    // frame's draws (cleared in beginFrame, hashed and rebuilt in endFrame),
    // the map the current frame shades through (built by the PREVIOUS
    // endFrame - the caster set only exists once every draw is seen), and
    // the hash the map was built for (the static-scene rebuild skip).
    std::vector<sunshadows::SunShadowCaster> sunShadowCasters_;
    sunshadows::SunShadowMap                 sunShadowMap_;
    sunshadows::SunShadowMap                 skyShadowMap_;
    std::uint64_t                            sunShadowBuiltHash_ = 0;
    std::uint64_t                            skyShadowBuiltHash_ = 0;
    bool                                     sunShadowSunActive_ = false;
    bool                                     sunShadowsEnabled_ = true;
    bool                                     skyShadowsEnabled_ = true;
    std::uint32_t                            shadowMapResolution_ = 1024;
    float                                    shadowFilterRadiusTexels_ = 1.5f;

    // Film antialiasing scratch (applyFilmAntialias): a snapshot of the
    // finished rows every pass reads from, so band-parallel workers never
    // sample a row a neighbor already rewrote (the result stays
    // deterministic regardless of scheduling).
    std::vector<std::uint8_t> filmAASource_;

    // Camera response pass (applyCameraResponse): contrast pivot curve,
    // radial vignette, deterministic sensor grain.
    float cameraContrast_          = 1.0f;
    float cameraVignette_          = 0.18f;
    float cameraGrain_             = 0.01f;
    bool  filmAA_                  = false;
    // Film AA cross-edge stage strength (film_aa_strength; see
    // applyFilmAntialias) and the tent supersample resolve gate
    // (headless_resolve_tent; see resolveRowsTent).
    float filmAAStrength_          = 1.0f;
    bool  resolveTent_             = true;

    // Tent resolve scratch (resolveRowsTent): per-band slices of 16-bit
    // sample-block sums (three raw rows plus three folded rows per band),
    // grown once per size in endFrame before the bands dispatch.
    mutable std::vector<std::uint16_t> tentRowSums_;

    // Gather field contact-occlusion multiplier (setGatherField bakes it
    // into the prepared field).
    float gatherOcclusionStrength_ = 1.0f;

    // Tile-parallel scratch, reused across draws/frames (no steady-state
    // allocation; capacity persists and the largest tile-parallel draw sets
    // the footprint). tileRecords_ holds the fan records of the current
    // draw (<= 2x its triangle count); tileOffsets_/tileItems_ are the
    // per-tile record lists.
    std::vector<TileBinRecord> tileRecords_;
    std::vector<std::uint32_t> tileOffsets_;
    std::vector<std::uint32_t> tileItems_;
    std::vector<tilebins::RecordTileRange> tileRangeScratch_;
    // Draws that could exceed the record cap fall back to the default
    // paths instead of growing the scratch without bound.
    static constexpr std::size_t maxTileRecords_ = 1u << 20;

    // Batched tile-rasterization state (drawMeshBatch). batchSetups_ holds
    // one RasterSetup per surviving batch item (indexed by
    // TileBinRecord::setupIndex) and is filled by the serial prepare loop
    // before any worker reads it; the records themselves bake each item's
    // transformed corners, so workers never need the meshes. The batch
    // record cap (renderer.headless_batch_tile_records) bounds the per-chunk
    // scratch: a full streamed frame (LOD chunks plus the backdrop shells)
    // routinely carries several hundred thousand triangles, so the default
    // cap is sized to swallow a whole typical frame in one flush; larger
    // frames still flush in chunks, which preserves submission order across
    // chunk boundaries - each chunk rasterizes completely before the next
    // begins, exactly like the per-draw replay it replaces.
    std::vector<RasterSetup> batchSetups_;
    std::size_t              maxBatchTileRecords_ = 512u * 1024u;

    std::unordered_map<MeshHandle, HeadlessMesh> meshes;

    // ---- per-mesh transform cache -------------------------------------
    // One entry per cached mesh, surviving draws and frames: the world
    // stage (model transform + normal matrix) is keyed on the exact model
    // matrix bits plus the draw kind (unlit, shell-fur layer parameters),
    // and the clip stage is keyed separately on whichever matrix produced
    // clip (the combined view-projection, or the raw one for fur). A static
    // model matrix under a static camera hits both tiers; a camera move
    // reprojects the clip tier without re-running the model stage; a moving
    // or shapeshifting object simply misses and re-transforms. The total
    // footprint is bounded by transformCacheBudgetBytes_ (raised entries
    // evict least-recently-touched ones; a mesh larger than the whole
    // budget is never cached and keeps the direct per-vertex path).
    struct TransformCacheEntry
    {
        glm::mat4 worldModel{0.0f};
        bool      worldValid     = false;
        bool      worldUnlit     = false;
        bool      fur            = false;
        float     furLayerRatio  = 0.0f;
        float     furHeight      = 0.0f;
        float     furGravity     = 0.0f;
        glm::mat4 clipMatrix{0.0f};
        bool      clipValid      = false;
        std::vector<TransformedVertex> vertices;
        std::uint64_t                   lastTouch = 0;
    };
    std::unordered_map<MeshHandle, TransformCacheEntry> transformCaches_;
    std::size_t   transformCacheBytesUsed_ = 0;
    std::uint64_t transformCacheClock_     = 1;

    // Finds or creates this draw's cache entry (nullptr when the mesh cannot
    // be cached); existing entries only refresh their LRU stamp.
    [[nodiscard]] TransformCacheEntry *transformCacheEntryFor(const HeadlessMesh &mesh, std::size_t vertexCount);

    // Drops a mesh's entry (destroy, or re-creation under the same handle).
    void forgetTransformCache(MeshHandle mesh);

    std::unordered_map<TextureHandle, HeadlessTexture> textures;

    HeadlessTexture neutralWhite_ = neutralWhiteTexture();
    HeadlessTexture flatNormal_   = flatNormalTexture();

    FrameStatistics statistics_{};
};

} // namespace renderer
} // namespace engine

