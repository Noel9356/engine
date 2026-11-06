// Headless (software) render backend: a CPU rasterizer that writes RGBA8
// pixels straight into the window module's surface buffer. Same PBR shading
// model as the Vulkan fragment shader, plus frustum/back-face culling,
// distance-based LOD, a vertex transform cache, and optional task-scheduler
// parallelism. Used when the surface API is not Vulkan or "headless" is
// configured.

#ifndef ENGINE_RENDER_MODULE_HEADLESS_BACKEND_HPP
#define ENGINE_RENDER_MODULE_HEADLESS_BACKEND_HPP

#include "engine/direct_lighting/capabilities.hpp"
#include "engine/pbr/capabilities.hpp"
#include "engine/render/capabilities.hpp"
#include "engine/render/device_profile.hpp"
#include "engine/render/material.hpp"
#include "engine/render/mesh.hpp"
#include "render_backend.hpp"
#include "render_shared.hpp"

#include <engine/kernel/task_scheduler.hpp>
#include <engine/window/capabilities.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <glm/ext/matrix_float3x3.hpp>
#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/ext/vector_float4.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <unordered_map>
#include <vector>

namespace engine
{
namespace render
{

// CPU-side mesh copy in planar (SoA) layout for rasterizer locality, with
// bounding-sphere data for culling/LOD. lodIndices holds decimated index
// lists (every 4th/16th/64th triangle) generated at creation.
struct HeadlessMesh
{
    std::vector<float>         positions;
    std::vector<float>         normals;
    std::vector<float>         colors;
    std::vector<float>         uvs;
    std::vector<std::uint32_t> indices;
    glm::vec3                  boundsCenter{0.0f};
    float                      boundsRadius = 0.0f;
    bool                       boundsValid  = false;

    std::vector<std::vector<std::uint32_t>> lodIndices;

    [[nodiscard]] std::size_t vertexCount() const noexcept
    {
        return positions.size() / 3;
    }
};

// CPU-side texture. pixels is the RGBA8 upload copy (unused by sampling);
// pixelsRGB plus the mipData chain (bilinearly downsampled, halved each
// level) are what the sampler reads.
struct HeadlessTexture
{
    std::uint32_t                   width  = 0;
    std::uint32_t                   height = 0;
    std::vector<std::uint8_t>       pixels;
    std::vector<float>              pixelsRGB;
    std::uint32_t                   mipLevelCount = 1;
    std::vector<std::vector<float>> mipData;
    std::vector<std::uint32_t>      mipWidths;
    std::vector<std::uint32_t>      mipHeights;

    TextureFilter  filter  = TextureFilter::Linear;
    TextureAddress address = TextureAddress::Repeat;

    // Bilinear or nearest sample with repeat/clamp addressing. mipLevel 0 or
    // an out-of-range level samples the base image; returns white when empty.
    glm::vec3 sample(float u, float v, std::uint32_t mipLevel = 0) const;
};

// 1x1 stand-ins so materials without maps still sample something sensible.
inline HeadlessTexture neutralWhiteTexture()
{
    HeadlessTexture texture;
    texture.width     = 1;
    texture.height    = 1;
    texture.pixels    = {255, 255, 255, 255};
    texture.pixelsRGB = {1.0f, 1.0f, 1.0f};
    texture.filter    = TextureFilter::Nearest;
    texture.address   = TextureAddress::Repeat;
    return texture;
}

inline HeadlessTexture flatNormalTexture()
{
    HeadlessTexture texture;
    texture.width     = 1;
    texture.height    = 1;
    texture.pixels    = {128, 128, 255, 255};
    texture.pixelsRGB = {0.5f, 0.5f, 1.0f};
    texture.filter    = TextureFilter::Nearest;
    texture.address   = TextureAddress::Repeat;
    return texture;
}

// Options forwarded from [render] configuration at renderer creation.
struct HeadlessBackendOptions
{
    std::size_t transformCacheBudgetBytes = 32u * 1024u * 1024u; // cap for the vertex transform cache
    bool        frustumCullingEnabled     = true;
    bool        backFaceCullingEnabled    = true;

    bool           lodEnabled                = true;
    TaskScheduler *taskScheduler             = nullptr; // borrowed; parallelism disabled when null/not running
    std::uint32_t  parallelTriangleThreshold = 4096;    // triangles per draw before splitting across workers
    std::size_t    pixelsPerLockGroup        = 16;      // pixels per depth lock during parallel rasterization
    std::size_t    lodMinTriangles
        = 4096; // plumbed in but currently unused; LOD generation uses a fixed 4096*3-index threshold

    // Draws with at most this many triangles are split by framebuffer bands
    // instead of triangle ranges: full-screen, few-triangle draws (floor,
    // sky dome) have no parallelism without it, and their per-pixel shading
    // dominates the frame. 0 disables band splitting.
    std::uint32_t bandSplitTriangleThreshold = 4096;

    // A draw only pays for a band split when its projected screen extent is
    // at least this fraction of the framebuffer height (small far geometry
    // would spend more on the fork/join than it saves).
    float bandSplitMinimumScreenFraction = 0.125f;
};

// All entry points are main-thread only. drawMeshInstances is implemented as
// a loop over drawMesh. beginFrame (re)allocates the color/depth buffers and
// must be called before any draw; endFrame blits into the window surface.
class HeadlessBackend final : public RenderBackend
{
public:
    HeadlessBackend(window::WindowHandle surfaceHandle, window::WindowCapability *windowCapability,
        const HeadlessBackendOptions &options)
        : surface_(surfaceHandle),
          transformCacheBudgetBytes_(options.transformCacheBudgetBytes),
          frustumCullingEnabled_(options.frustumCullingEnabled),
          backFaceCullingEnabled_(options.backFaceCullingEnabled),
          lodEnabled_(options.lodEnabled),
          taskScheduler_(options.taskScheduler),
          bandSplitTriangleThreshold_(options.bandSplitTriangleThreshold),
          bandSplitMinimumScreenFraction_(options.bandSplitMinimumScreenFraction),
          window_(windowCapability)
    {
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
    void drawOverlay(const OverlayDrawData &data) override;
    void endFrame() override;

    bool createMesh(MeshHandle id, const MeshDescription &description, RenderError *error) override;
    void destroyMesh(MeshHandle mesh) override;

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
        bool                   useTransformCache       = false;
        std::uint32_t          transformGeneration     = 0;
        std::size_t            vertexCount             = 0;
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

    // Computes the draw's matrices, culls against the frustum, transforms
    // vertices (through the cache when it pays off), and probes winding to
    // decide whether back-face culling is safe. Returns false when the draw
    // is fully culled or the framebuffer is gone.
    bool prepareRasterization(const HeadlessMesh &mesh, const float *modelMatrix, const MeshMaterial &material,
        const std::uint32_t *activeIndices, std::size_t activeIndexCount, RasterSetup &outSetup);

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

    window::WindowHandle surface_                   = 0;
    std::size_t          transformCacheBudgetBytes_ = 32u * 1024u * 1024u;
    bool                 frustumCullingEnabled_     = true;
    bool                 backFaceCullingEnabled_    = true;
    bool                 lodEnabled_                = true;

    TaskScheduler *taskScheduler_ = nullptr;

    window::WindowCapability *window_ = nullptr; // borrowed; provides surface pixel lock/unlock

    std::uint32_t parallelTriangleThreshold_      = 4096;
    std::uint32_t bandSplitTriangleThreshold_     = 4096;
    float         bandSplitMinimumScreenFraction_ = 0.125f;
    std::size_t   pixelsPerLockGroup_             = 16;
    std::size_t   lodMinTriangles_                = 4096;
    // LOD angular-size cutoffs (boundsRadius/dist): pick coarser LOD as the value shrinks.
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

    std::uint32_t viewportWidth  = 0;
    std::uint32_t viewportHeight = 0;

    // CPU framebuffer: RGBA8 color, float depth (1 = far), and one lock byte
    // per pixelsPerLockGroup_ pixels for parallel depth updates.
    std::vector<std::uint8_t> color;
    std::vector<float>        depth;
    std::uint32_t             framebufferWidth  = 0;
    std::uint32_t             framebufferHeight = 0;

    std::vector<std::uint8_t> pixelLocks;

    // Reused across parallel draws: per-worker partial rasterized-triangle
    // counts. Member (not local) so a frame with many parallel draws does
    // not hit the allocator 60+ times (allocation spikes land in the P99).
    std::vector<StripResult> rasterResultScratch_;

    std::unordered_map<MeshHandle, HeadlessMesh> meshes;

    // Transform cache: vertices shaded for the current generation (stamp
    // array lets draws reuse transforms across overlapping index ranges).
    std::vector<TransformedVertex> transformedVertices_;
    std::vector<std::uint32_t>     transformedVertexStamps_;
    std::uint32_t                  transformGeneration_ = 0;

    std::unordered_map<TextureHandle, HeadlessTexture> textures;

    HeadlessTexture neutralWhite_ = neutralWhiteTexture();
    HeadlessTexture flatNormal_   = flatNormalTexture();

    FrameStatistics statistics_{};
};

} // namespace render
} // namespace engine

#endif
