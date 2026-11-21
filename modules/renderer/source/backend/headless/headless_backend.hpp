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

#include <array>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace engine
{
namespace renderer
{

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
    // MSAA-style interior shading for the supersampled target: an output
    // pixel whose entire sample block one triangle covers (and whose every
    // sample passes the depth test) shades once at the block centre and
    // commits the one color to all samples - coverage, per-sample depth and
    // the averaging resolve stay per sample, so silhouettes keep the exact
    // supersampled anti-aliasing while interior pixels drop the redundant
    // per-sample shading (the software twin of hardware MSAA, where the
    // pixel shader runs once per pixel and only coverage/depth run per
    // sample). Transparent and shell-fur draws keep the per-sample path:
    // their per-sample blends read the samples' own background colors.
    bool           msaaInteriorShadingEnabled      = true;
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
    std::size_t maxBatchTileRecords = 512u * 1024u;

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
          msaaInteriorShadingEnabled_(options.msaaInteriorShadingEnabled && options.msaaSamples >= 3u
              && options.msaaSamples <= 4u),
          taskScheduler_(options.taskScheduler),
          window_(windowCapability),
          bandSplitTriangleThreshold_(options.bandSplitTriangleThreshold),
          bandSplitMinimumScreenFraction_(options.bandSplitMinimumScreenFraction),
          lodMinTriangles_(options.lodMinTriangles),
          parallelTransformVertexThreshold_(options.parallelTransformVertexThreshold),
          lodStrides_(options.lodStrides),
          lodThresholds_(options.lodThresholds),
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

    // Copies the borrowed lumen_gi probe-grid snapshot; the rasterizer's
    // pbrShade calls then sample it per pixel (validity-weighted trilinear
    // SH2 irradiance). An empty snapshot keeps the legacy hemispheric
    // ambient. The prepared view (level descriptors + bounds constants) is
    // rebuilt here so per-pixel shading reads ready-made constants.
    void setProbeGrid(const GiProbeGrid &grid) override
    {
        probeGridProbes_.assign(grid.probes, grid.probes + grid.probeCount);
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

    // Copies the borrowed lumen_reflection field; the rasterizer's pixel
    // path then adds the indirect specular radiance (bilinear field sample
    // through the field's own camera) to every shaded opaque pixel after the
    // PBR pass and before tone mapping. An empty snapshot keeps the legacy
    // specular-only shading.
    void setReflectionField(const ReflectionField &field) override
    {
        reflectionField_      = ReflectionField{};
        reflectionRadiance_.clear();
        if (field.valid())
        {
            // The borrowed field is interleaved texels (radiance RGB +
            // roughness; see reflectionFieldTexelFloats), so the copy keeps
            // the whole stride and the pixel-path sampler indexes it the
            // same way.
            const std::size_t texelCount
                = static_cast<std::size_t>(field.header.width) * field.header.height;
            reflectionRadiance_.assign(
                field.radiance, field.radiance + texelCount * lumen_reflection::reflectionFieldTexelFloats);
            reflectionField_.header = field.header;
            reflectionField_.radiance = reflectionRadiance_.data();
        }
        // Prepared once per snapshot: the field's view-projection, so the
        // per-pixel bilinear sample stops rebuilding the matrix.
        preparedReflectionField_ = prepareReflectionField(reflectionField_);
    }

    void setEnvironment(const pbr::PbrEnvironment &environment) override
    {
        environment_    = environment;
        lightDirection_ = glm::normalize(glm::make_vec3(environment.sun.direction));
        sunRadiance_    = glm::make_vec3(environment.sun.color) * environment.sun.intensity
                        * (environment.sun.enabled ? 1.0f : 0.0f);
        ambientSky_     = glm::make_vec3(environment.ambient.skyColor) * environment.ambient.intensity;
        ambientGround_  = glm::make_vec3(environment.ambient.groundColor) * environment.ambient.intensity;
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

    // Reports the creation-time mean color stored on the texture record
    // (see HeadlessTexture::meanRGB).
    bool textureMean(TextureHandle texture, float outMeanRGB[3]) const override;

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
        std::uint32_t sourceTriangle;
        std::uint32_t setupIndex;
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

    // Per-tile occlusion state for the tile-parallel raster paths (see
    // rasterizeFanRecord): coverage bits for the samples that carry
    // depth-written geometry and, once every sample of the tile is covered,
    // the tile's farthest depth at that moment. A later record whose NEAREST
    // corner depth lies beyond that farthest depth is fully hidden behind
    // geometry the tile already drew (its every sample would fail the
    // per-sample depth test), so its whole bounding-box scan is skipped.
    // Depth writes only lower sample values, so the stored farthest depth
    // stays a conservative upper bound of the tile's true maximum. One
    // 64-sample word per tile row: tiles wider or taller than 64 samples
    // (renderer.headless_tile_pixels above 64) leave the state unused.
    struct TileOcclusion
    {
        std::array<std::uint64_t, 64> coveredWords{};
        std::uint32_t coveredCount  = 0;
        std::uint32_t sampleCount   = 0; // 0 = occlusion tracking disabled
        bool         closed         = false;
        float        farthestDepth  = 1.0f;
    };

    // Shared rasteriser core: scan one fan record's bounding box clamped to
    // the inclusive pixel window; barycentric coverage with edge-function
    // increments, depth-test, shade, write. Tile owners pass their tile and
    // its occlusion state; writes only touch the window, so concurrent calls
    // with disjoint windows never share a pixel.
    ENGINE_RASTERIZER_INLINE bool rasterizeFanRecord(const RasterSetup &setup, const TileBinRecord &record,
        std::int32_t xBandBegin, std::int32_t xBandEnd, std::int32_t yBandBegin, std::int32_t yBandEnd,
        TileOcclusion *occlusion = nullptr);

    // Integer pixel window (inclusive) a fan record can write, clamped to
    // the framebuffer. Shared verbatim by the rasteriser core and the tile
    // binning pass so the bins cover exactly the pixels the core may touch.
    ENGINE_RASTERIZER_INLINE static void fanPixelWindow(const TileBinRecord &record, std::uint32_t framebufferWidth,
        std::uint32_t framebufferHeight, std::int32_t &firstX, std::int32_t &lastX, std::int32_t &firstY,
        std::int32_t &lastY);

    // Shared shading kernel: perspective-correct varying interpolation,
    // material maps, PBR, emissive, tone map + gamma. Const; the caller owns
    // the depth/colour write.
    ENGINE_RASTERIZER_INLINE glm::vec3 shadeTrianglePixel(const RasterSetup &setup, const TileBinRecord &record,
        float weightA, float weightB, float inverseWA, float inverseWB, float inverseWC,
        bool *outFieldTraced = nullptr) const;

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

    void rasterizeMesh(const HeadlessMesh &mesh, const float *modelMatrix, const MeshMaterial &material);

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
    // Mirrors HeadlessBackendOptions::msaaInteriorShadingEnabled (see there);
    // the constructor ANDs it with the 2x2 grid requirement, so the flag is
    // only ever true on the sample grid the fast path walks.
    bool                 msaaInteriorShadingEnabled_ = false;

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

    // lumen_gi probe-grid hook: owned probe copy plus the prepared view
    // passed to pbrShade. Kept valid between frames; setProbeGrid replaces
    // both wholesale.
    GiProbeGrid                     probeGrid_{};
    PreparedGiProbeGrid             preparedProbeGrid_{};
    std::vector<lumen_gi::GiProbe> probeGridProbes_;  

    // lumen_reflection field hook: owned radiance copy plus the borrowed view
    // the pixel path samples. Kept valid between frames; setReflectionField
    // replaces it wholesale.
    std::vector<float>                 reflectionRadiance_;
    ReflectionField                    reflectionField_{};
    // Prepared view of reflectionField_ (the field's own view-projection
    // built once per snapshot); rebuilt by setReflectionField.
    PreparedReflectionField            preparedReflectionField_{};

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

    // Tile-parallel scratch, reused across draws/frames (no steady-state
    // allocation; capacity persists and the largest tile-parallel draw sets
    // the footprint). tileRecords_ holds the fan records of the current
    // draw (<= 2x its triangle count); tileOffsets_/tileItems_ are the
    // per-tile record lists.
    std::vector<TileBinRecord> tileRecords_;
    std::vector<std::uint32_t> tileOffsets_;
    std::vector<std::uint32_t> tileItems_;
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

