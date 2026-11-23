// Headless backend implementation: a multithreaded CPU rasterizer.
//
// Pipeline per draw: prepareRasterization (culling, transform cache, texture
// resolution) then rasterizeTriangles (Sutherland-Hodgman-style near clip,
// scanline rasterization with perspective-correct interpolation, PBR shading
// via pbr_shading.hpp, tile-locked depth updates when parallel). Frames are
// blitted into the window module's locked surface pixels.

#include "headless_backend.hpp"
#include "render_shared.hpp"
#include "tile_bins.hpp"

#include <engine/direct_lighting/capabilities.hpp>
#include <engine/kernel/task_scheduler.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/material.hpp>
#include <engine/renderer/mesh.hpp>
#include <engine/window/capabilities.hpp>

#include <glm/common.hpp>
#include <glm/exponential.hpp>
#include <glm/ext/matrix_float3x3.hpp>
#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/vector_float2.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/ext/vector_float4.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <new>
#include <thread>
#include <utility>
#include <vector>

// Helps the compiler keep color/depth pointers non-aliased in the rasterizer.
#if defined(__GNUC__) || defined(__clang__)
#define ENGINE_RESTRICT __restrict__
#else
#define ENGINE_RESTRICT
#endif

namespace engine
{
namespace renderer
{

namespace
{
// One shell-fur displaced vertex: the exact CPU mirror of the displacement
// section in shaders/fur.vert (parity contract, like pbrShade <-> mesh.frag).
// layerRatio = layer / layerCount in 0..1; the per-vertex jitter hash breaks
// up the concentric shell layering; gravity droops the tip toward world -Y,
// quadratically in the strand height, AFTER the model transform.
struct FurDisplacedVertex
{
    glm::vec4 clip;
    glm::vec3 worldPosition;
};

inline FurDisplacedVertex furDisplaceVertex(const glm::vec3 &position, const glm::vec3 &normal,
    const glm::mat4 &model, const glm::mat4 &viewProjectionWorld, float layerRatio, float furHeight, float gravity)
{
    const float jitter = glm::fract(
        glm::sin(glm::dot(position, glm::vec3(12.9898f, 78.233f, 37.719f))) * 43758.5453f);
    const float height    = furHeight * layerRatio * (0.7f + 0.6f * jitter);
    const glm::vec3 displaced = position + normal * height;
    glm::vec4 world        = model * glm::vec4(displaced, 1.0f);
    world.y -= gravity * height * height;
    FurDisplacedVertex result;
    result.clip          = viewProjectionWorld * world;
    result.worldPosition = glm::vec3(world);
    return result;
}

// Bitwise matrix comparison for transform-cache keys. A conservative
// mismatch (-0.0 vs 0.0 and NaN payloads) only costs a re-transform.
bool transformMatricesEqual(const glm::mat4 &a, const glm::mat4 &b)
{
    return std::memcmp(&a, &b, sizeof(glm::mat4)) == 0;
}

// Sutherland-Hodgman clip of one polygon against a single half-plane given
// as a distance functor (negative = outside). Interpolates every vertex
// attribute along the cut edge, exactly like the near-plane clip walk.
template <typename Vertex, typename Distance>
inline std::uint32_t clipPolygonAgainstPlane(
    const Vertex *input, std::uint32_t count, Vertex *output, Distance distanceOf)
{
    std::uint32_t outCount = 0;
    for (std::uint32_t current = 0; current < count; ++current)
    {
        const std::uint32_t next    = (current + 1u) % count;
        const Vertex       &edgeStart = input[current];
        const Vertex       &edgeEnd   = input[next];
        const float startDistance = distanceOf(edgeStart);
        const float endDistance   = distanceOf(edgeEnd);
        const bool  startInside   = startDistance >= 0.0f;
        const bool  endInside     = endDistance >= 0.0f;
        if (startInside)
        {
            output[outCount++] = edgeStart;
        }
        if (startInside != endInside)
        {
            const float t          = startDistance / (startDistance - endDistance);
            Vertex      &crossing  = output[outCount++];
            crossing.clip          = glm::mix(edgeStart.clip, edgeEnd.clip, t);
            crossing.worldPosition = glm::mix(edgeStart.worldPosition, edgeEnd.worldPosition, t);
            crossing.worldNormal   = glm::mix(edgeStart.worldNormal, edgeEnd.worldNormal, t);
            crossing.color         = glm::mix(edgeStart.color, edgeEnd.color, t);
            crossing.uv            = glm::mix(edgeStart.uv, edgeEnd.uv, t);
        }
    }
    return outCount;
}

// Guard-band screen-rect clip: clips a (near-plane-clipped) polygon against
// the framebuffer rectangle expressed in clip space - the four planes
// x = ±w, y = ±w, i.e. NDC |x|,|y| <= 1. A near-plane crossing survives the
// near clip with a w a hair above the near plane and projects to
// astronomical screen coordinates; the rasterizer's bounding-box clamp then
// turns the fan's scan window into the whole framebuffer for a sliver
// triangle (an eye-centered dome's horizon ring measured as full-screen
// scans per degenerate fan). Clipping here keeps every projected vertex
// inside the screen so the scan window is tight. Output is unchanged: the
// removed region is off-screen (those pixels were never shaded), and
// clipping is linear along the polygon's edges, so every surviving pixel
// interpolates the same attributes it did before. scratchA/scratchB must
// hold at least 9 vertices (a 4-gon grows by at most one vertex per plane);
// outPolygon points at the clipped polygon (input, scratchA or scratchB).
template <typename Vertex>
inline std::uint32_t clipPolygonToFramebuffer(const Vertex *input, std::uint32_t count, Vertex *scratchA,
    Vertex *scratchB, const Vertex *&outPolygon)
{
    const Vertex *currentPoly = input;
    Vertex       *writeBuffer = scratchA;
    std::uint32_t currentCount = clipPolygonAgainstPlane(currentPoly, count, writeBuffer,
        [](const Vertex &vertex) { return vertex.clip.x + vertex.clip.w; });
    currentPoly = writeBuffer;
    writeBuffer = writeBuffer == scratchA ? scratchB : scratchA;
    if (currentCount >= 3)
    {
        currentCount = clipPolygonAgainstPlane(currentPoly, currentCount, writeBuffer,
            [](const Vertex &vertex) { return vertex.clip.w - vertex.clip.x; });
        currentPoly  = writeBuffer;
        writeBuffer  = writeBuffer == scratchA ? scratchB : scratchA;
    }
    if (currentCount >= 3)
    {
        currentCount = clipPolygonAgainstPlane(currentPoly, currentCount, writeBuffer,
            [](const Vertex &vertex) { return vertex.clip.y + vertex.clip.w; });
        currentPoly  = writeBuffer;
        writeBuffer  = writeBuffer == scratchA ? scratchB : scratchA;
    }
    if (currentCount >= 3)
    {
        currentCount = clipPolygonAgainstPlane(currentPoly, currentCount, writeBuffer,
            [](const Vertex &vertex) { return vertex.clip.w - vertex.clip.y; });
        currentPoly  = writeBuffer;
    }
    outPolygon = currentPoly;
    return currentCount;
}
} // namespace

// Precomputes shading inputs: radiance = color * intensity, inverse-squared
// range falloff (range clamped away from zero), cone cosines for spots.
void HeadlessBackend::setDirectLights(const direct_lighting::DirectLight *lights, std::uint32_t count)
{
    count = std::min(count, direct_lighting::maxDirectLights);
    directLights_.clear();
    if (lights != nullptr && count != 0)
    {
        directLights_.resize(count);
        for (std::uint32_t index = 0; index < count; ++index)
        {
            const direct_lighting::DirectLightDescription &source = lights[index].description;
            PreparedDirectLight                           &target = directLights_[index];
            target.type                                           = source.type;
            target.position                                       = glm::make_vec3(source.position);
            target.direction                                      = glm::make_vec3(source.direction);
            target.radiance                                       = glm::make_vec3(source.color) * source.intensity;
            const float safeRange                                 = std::max(source.range, 1.0e-3f);
            target.inverseRangeSquared                            = 1.0f / (safeRange * safeRange);
            target.innerConeCosine                                = std::cos(source.innerConeRadians);
            target.outerConeCosine                                = std::cos(source.outerConeRadians);
        }
    }
}

// Caches the view-projection matrix and extracts the four side frustum
// planes (Gribb-Hartmann rows) for later sphere culling. Identical camera
// data short-circuits, which also keeps the frustum stable.
void HeadlessBackend::setCamera(const CameraMatrices &camera)
{
    if (cameraInitialized_ && std::memcmp(&camera_, &camera, sizeof(CameraMatrices)) == 0)
    {
        return;
    }

    camera_            = camera;
    cameraInitialized_ = true;
    viewProjection_    = glm::make_mat4(camera.projection) * glm::make_mat4(camera.view);
    cameraPosition_    = glm::make_vec3(camera.position);
    const glm::vec4 row0(viewProjection_[0][0], viewProjection_[1][0], viewProjection_[2][0], viewProjection_[3][0]);
    const glm::vec4 row1(viewProjection_[0][1], viewProjection_[1][1], viewProjection_[2][1], viewProjection_[3][1]);
    const glm::vec4 row3(viewProjection_[0][3], viewProjection_[1][3], viewProjection_[2][3], viewProjection_[3][3]);

    frustumPlanes_ = {row3 + row0, row3 - row0, row3 + row1, row3 - row1};
    for (glm::vec4 &plane : frustumPlanes_)
    {
        const float length = glm::length(glm::vec3(plane));
        if (std::isfinite(length) && length > 1e-8f)
        {
            plane /= length;
        }
        else
        {
            plane = glm::vec4(0.0f);
        }
    }
}

// (Re)allocates the CPU framebuffer when the viewport changed (all
// overflow/bad_alloc paths leave the framebuffer empty, which silently skips
// drawing), then clears color to the clear color, depth to 1.0, and the
// parallel-rasterization lock bytes to 0.
void HeadlessBackend::beginFrame(const FrameParameters &frame)
{
    std::uint32_t width  = viewportWidth;
    std::uint32_t height = viewportHeight;
    if (width == 0 || height == 0)
    {
        std::uint32_t surfaceWidth  = 0;
        std::uint32_t surfaceHeight = 0;
        window_->getFramebufferSize(window_->self, surface_, &surfaceWidth, &surfaceHeight);
        if (width == 0)
        {
            width = surfaceWidth;
        }
        if (height == 0)
        {
            height = surfaceHeight;
        }
    }
    if (width == 0 || height == 0)
    {
        return;
    }
    // Supersampled framebuffer: the rasterizers already size everything from
    // framebufferWidth/Height, so scaling here turns every shaded pixel into a
    // sample and endFrame's resolve is the only presentation-side change.
    const std::uint32_t sampleScaleX = sampleScaleX_;
    const std::uint32_t sampleScaleY = sampleScaleY_;
    const std::uint64_t bufferWidth64  = static_cast<std::uint64_t>(width) * sampleScaleX;
    const std::uint64_t bufferHeight64 = static_cast<std::uint64_t>(height) * sampleScaleY;
    if (bufferWidth64 > static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max())
        || bufferHeight64 > static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()))
    {
        framebufferWidth = framebufferHeight = 0;
        color.clear();
        depth.clear();
        pixelLocks.clear();
        return;
    }
    width  = static_cast<std::uint32_t>(bufferWidth64);
    height = static_cast<std::uint32_t>(bufferHeight64);
    if (framebufferWidth != width || framebufferHeight != height)
    {
        if (width > static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max())
            || height > static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max()))
        {
            framebufferWidth = framebufferHeight = 0;
            color.clear();
            depth.clear();
            pixelLocks.clear();
            return;
        }
        const std::size_t maximum = std::numeric_limits<std::size_t>::max();
        if (static_cast<std::size_t>(width) > maximum / height
            || static_cast<std::size_t>(width) * height > maximum / 4)
        {
            framebufferWidth = framebufferHeight = 0;
            color.clear();
            depth.clear();
            pixelLocks.clear();
            return;
        }
        const std::size_t pixelCount = static_cast<std::size_t>(width) * height;
        if (pixelCount * 4 > color.max_size() || pixelCount > depth.max_size())
        {
            framebufferWidth = framebufferHeight = 0;
            color.clear();
            depth.clear();
            pixelLocks.clear();
            return;
        }
        try
        {
            std::vector<std::uint8_t> newColor(pixelCount * 4, 0);
            std::vector<float>        newDepth(pixelCount, 1.0f);
            const std::size_t         lockCount = (pixelCount + pixelsPerLockGroup_ - 1) / pixelsPerLockGroup_;
            std::vector<std::uint8_t> newLocks(lockCount, 0);
            color.swap(newColor);
            depth.swap(newDepth);
            pixelLocks.swap(newLocks);
            framebufferWidth  = width;
            framebufferHeight = height;
        }
        catch (const std::bad_alloc &)
        {
            framebufferWidth = framebufferHeight = 0;
            color.clear();
            depth.clear();
            pixelLocks.clear();
            return;
        }
    }
    const std::uint8_t red   = toByte(frame.clearColor[0]);
    const std::uint8_t green = toByte(frame.clearColor[1]);
    const std::uint8_t blue  = toByte(frame.clearColor[2]);
    const std::uint8_t alpha = toByte(frame.clearColor[3]);

    // One word per pixel: the pattern is built by copying the four bytes in
    // memory order and written back the same way, so byte order cancels out.
    std::uint8_t clearBytes[4] = {red, green, blue, alpha};
    std::uint32_t clearWord    = 0;
    std::memcpy(&clearWord, clearBytes, sizeof(clearWord));
    std::uint32_t *const colorWordsBegin = reinterpret_cast<std::uint32_t *>(color.data());
    const std::size_t    pixelsPerRow    = framebufferWidth;
    // The clear is a pure store sweep over disjoint row ranges, so it splits
    // across the task scheduler exactly like the resolve in endFrame does;
    // without a scheduler (or on single-worker hosts) it stays serial.
    const bool clearInParallel = taskScheduler_ != nullptr && taskScheduler_->isRunning()
        && parallelWorkerCount() >= 2u && framebufferHeight >= 2u;
    if (clearInParallel)
    {
        const std::uint32_t clearWorkers = parallelWorkerCount();
        const std::uint32_t clearGrain   = std::max<std::uint32_t>(
            1u, static_cast<std::uint32_t>(framebufferHeight / (clearWorkers * 4u)));
        float *const depthBegin = depth.data();
        taskScheduler_->parallelFor(framebufferHeight,
            clearGrain,
            [this, colorWordsBegin, depthBegin, clearWord, pixelsPerRow](std::size_t rowBegin, std::size_t rowEnd)
            {
                std::fill(colorWordsBegin + rowBegin * pixelsPerRow,
                    colorWordsBegin + rowEnd * pixelsPerRow,
                    clearWord);
                std::fill(depthBegin + rowBegin * pixelsPerRow, depthBegin + rowEnd * pixelsPerRow, 1.0f);
            });
    }
    else
    {
        std::fill(colorWordsBegin,
            reinterpret_cast<std::uint32_t *>(color.data() + color.size()),
            clearWord);
        std::fill(depth.begin(), depth.end(), 1.0f);
    }

    std::fill(pixelLocks.begin(), pixelLocks.end(), 0);
}

// Presents: copies the CPU raster into the window surface's locked pixel
// buffer. With supersampling active, each output pixel is the average of its
// sample block - the software counterpart of the Vulkan backend's
// DONT_CARE-store + RESOLVE_MODE_AVERAGE multisample resolve (averaging the
// encoded UNORM samples, matching the space the fur path blends in).
// Matching full-surface strides (the common case) move the frame in one
// memcpy; mismatching strides fall back to a row-by-row copy.
void HeadlessBackend::endFrame()
{
    window::SurfacePixels pixels;
    if (!window_->lockSurfacePixels(window_->self, surface_, &pixels))
    {
        return;
    }
    const std::uint32_t rows    = std::min(framebufferHeight, pixels.height);
    const std::uint32_t columns = std::min(framebufferWidth, pixels.width);
    if (sampleScaleX_ == 1u && sampleScaleY_ == 1u)
    {
        if (rows == framebufferHeight && columns == framebufferWidth
            && pixels.strideBytes == static_cast<std::size_t>(framebufferWidth) * 4u)
        {
            std::memcpy(pixels.rgba, color.data(), static_cast<std::size_t>(rows) * pixels.strideBytes);
        }
        else
        {
            for (std::uint32_t y = 0; y < rows; ++y)
            {
                std::memcpy(pixels.rgba + static_cast<std::size_t>(y) * pixels.strideBytes,
                    color.data() + static_cast<std::size_t>(y) * framebufferWidth * 4,
                    static_cast<std::size_t>(columns) * 4);
            }
        }
    }
    else
    {
        // Resolve: framebufferWidth/Height are sample-space; each output pixel
        // averages its sampleScaleX x sampleScaleY block of encoded samples.
        // The sample count of the supersampling grid is always a power of two,
        // so the rounding average ((sum + half) / count) is exact as a shift;
        // rows are split across workers when a scheduler is available (each
        // band writes disjoint rows of the locked surface).
        const std::uint32_t outputRows    = std::min(framebufferHeight / sampleScaleY_, pixels.height);
        const std::uint32_t outputColumns = std::min(framebufferWidth / sampleScaleX_, pixels.width);
        const std::uint32_t sampleCount   = sampleScaleX_ * sampleScaleY_;
        std::uint32_t       sampleShift   = 0;
        bool                sampleCountIsPowerOfTwo = false;
        switch (sampleCount)
        {
        case 1u: sampleShift = 0u; sampleCountIsPowerOfTwo = true; break;
        case 2u: sampleShift = 1u; sampleCountIsPowerOfTwo = true; break;
        case 4u: sampleShift = 2u; sampleCountIsPowerOfTwo = true; break;
        case 8u: sampleShift = 3u; sampleCountIsPowerOfTwo = true; break;
        default: sampleCountIsPowerOfTwo = false; break;
        }
        const auto resolveRows = [&](std::uint32_t rowBegin, std::uint32_t rowEnd)
        {
            for (std::uint32_t y = rowBegin; y < rowEnd; ++y)
            {
                std::uint8_t *destination = pixels.rgba + static_cast<std::size_t>(y) * pixels.strideBytes;
                for (std::uint32_t x = 0; x < outputColumns; ++x)
                {
                    std::uint32_t accumulated[4] = {0, 0, 0, 0};
                    const std::uint8_t *sampleBase
                        = color.data()
                        + (static_cast<std::size_t>(y) * sampleScaleY_ * framebufferWidth
                           + static_cast<std::size_t>(x) * sampleScaleX_)
                            * 4;
                    for (std::uint32_t sampleY = 0; sampleY < sampleScaleY_; ++sampleY)
                    {
                        const std::uint8_t *sampleRow
                            = sampleBase + static_cast<std::size_t>(sampleY) * framebufferWidth * 4;
                        for (std::uint32_t sampleX = 0; sampleX < sampleScaleX_; ++sampleX)
                        {
                            const std::uint8_t *sample = sampleRow + static_cast<std::size_t>(sampleX) * 4;
                            accumulated[0] += sample[0];
                            accumulated[1] += sample[1];
                            accumulated[2] += sample[2];
                            accumulated[3] += sample[3];
                        }
                    }
                    if (sampleCountIsPowerOfTwo)
                    {
                        const std::uint32_t half = 1u << (sampleShift - 1u);
                        destination[0] = static_cast<std::uint8_t>((accumulated[0] + half) >> sampleShift);
                        destination[1] = static_cast<std::uint8_t>((accumulated[1] + half) >> sampleShift);
                        destination[2] = static_cast<std::uint8_t>((accumulated[2] + half) >> sampleShift);
                        destination[3] = static_cast<std::uint8_t>((accumulated[3] + half) >> sampleShift);
                    }
                    else
                    {
                        const std::uint32_t half = sampleCount / 2u;
                        destination[0] = static_cast<std::uint8_t>((accumulated[0] + half) / sampleCount);
                        destination[1] = static_cast<std::uint8_t>((accumulated[1] + half) / sampleCount);
                        destination[2] = static_cast<std::uint8_t>((accumulated[2] + half) / sampleCount);
                        destination[3] = static_cast<std::uint8_t>((accumulated[3] + half) / sampleCount);
                    }
                    destination += 4;
                }
            }
        };
        const bool resolveInParallel = taskScheduler_ != nullptr && taskScheduler_->isRunning()
            && parallelWorkerCount() >= 2u && outputRows >= 2u;
        if (resolveInParallel)
        {
            const std::uint32_t workers    = parallelWorkerCount();
            TaskGroup resolveGroup;
            const std::uint32_t bandCount = workers;
            const std::uint32_t bandWidth = (outputRows + bandCount - 1u) / bandCount;
            for (std::uint32_t band = 1; band < bandCount; ++band)
            {
                const std::uint32_t begin = band * bandWidth;
                const std::uint32_t end   = std::min(begin + bandWidth, outputRows);
                if (begin >= end)
                {
                    break;
                }
                if (!taskScheduler_->schedule(resolveGroup, [this, &resolveRows, begin, end]()
                        { resolveRows(begin, end); }))
                {
                    resolveRows(begin, end);
                }
            }
            resolveRows(0u, std::min(bandWidth, outputRows));
            resolveGroup.wait(*taskScheduler_);
        }
        else
        {
            resolveRows(0u, outputRows);
        }
    }
    window_->unlockSurfacePixels(window_->self, surface_);
    ++statistics_.framesRendered;
}

// Copies the interleaved vertices into planar arrays, computes the bounding
// sphere (used for frustum culling, back-face heuristics, and LOD), and for
// large meshes decimates LOD chains by keeping every 4th/16th/64th triangle.
bool HeadlessBackend::createMesh(MeshHandle id, const MeshDescription &description, RenderError *error)
{
    try
    {
        HeadlessMesh      mesh;
        const auto       *src = static_cast<const float *>(description.vertices);
        const auto       *idx = static_cast<const std::uint32_t *>(description.indices);
        const std::size_t vc  = description.vertexCount;

        mesh.positions.resize(vc * 3);
        mesh.normals.resize(vc * 3);
        mesh.colors.resize(vc * 3);
        mesh.uvs.resize(vc * 2);
        for (std::size_t v = 0; v < vc; ++v)
        {
            const float *s            = src + v * meshVertexFloatCount;
            mesh.positions[v * 3 + 0] = s[0];
            mesh.positions[v * 3 + 1] = s[1];
            mesh.positions[v * 3 + 2] = s[2];
            mesh.normals[v * 3 + 0]   = s[3];
            mesh.normals[v * 3 + 1]   = s[4];
            mesh.normals[v * 3 + 2]   = s[5];
            mesh.colors[v * 3 + 0]    = s[6];
            mesh.colors[v * 3 + 1]    = s[7];
            mesh.colors[v * 3 + 2]    = s[8];
            mesh.uvs[v * 2 + 0]       = s[9];
            mesh.uvs[v * 2 + 1]       = s[10];
        }
        mesh.indices.assign(idx, idx + description.indexCount);

        const float *pos = mesh.positions.data();
        glm::vec3    minimum(std::numeric_limits<float>::max());
        glm::vec3    maximum(std::numeric_limits<float>::lowest());
        for (std::size_t v = 0; v < vc; ++v)
        {
            const glm::vec3 p(pos[v * 3], pos[v * 3 + 1], pos[v * 3 + 2]);
            minimum = glm::min(minimum, p);
            maximum = glm::max(maximum, p);
        }
        mesh.boundsCenter   = (minimum + maximum) * 0.5f;
        float radiusSquared = 0.0f;
        for (std::size_t v = 0; v < vc; ++v)
        {
            const glm::vec3 delta = glm::vec3(pos[v * 3], pos[v * 3 + 1], pos[v * 3 + 2]) - mesh.boundsCenter;
            radiusSquared         = std::max(radiusSquared, glm::dot(delta, delta));
        }
        mesh.boundsRadius = std::sqrt(radiusSquared);
        mesh.boundsValid  = std::isfinite(mesh.boundsRadius);

        if (lodEnabled_ && mesh.indices.size() >= lodMinTriangles_ * 3)
        {
            // LOD 0 is the full mesh; each level strides over the triangle
            // list, so quality drops 4x/16x/64x with no spatial coherence.
            mesh.lodIndices.push_back(mesh.indices);
            const std::size_t fullCount = mesh.indices.size();
            const std::size_t strides[] = {lodStrides_[0], lodStrides_[1], lodStrides_[2]};
            for (int level = 0; level < 3; ++level)
            {
                const std::size_t triStride   = strides[level];
                const std::size_t triCount    = fullCount / 3;
                const std::size_t lodTriCount = triCount / triStride;
                if (lodTriCount < 1)
                    break;
                std::vector<std::uint32_t> lodIdx;
                lodIdx.reserve(lodTriCount * 3);
                for (std::size_t t = 0; t < triCount; t += triStride)
                {
                    lodIdx.push_back(mesh.indices[t * 3 + 0]);
                    lodIdx.push_back(mesh.indices[t * 3 + 1]);
                    lodIdx.push_back(mesh.indices[t * 3 + 2]);
                }
                if (lodIdx.size() >= 3)
                    mesh.lodIndices.push_back(std::move(lodIdx));
            }
        }

        mesh.handle = id;
        // A re-creation under an existing handle is rejected the same way the
        // Vulkan backend rejects it: the caller owns the handle lifecycle, and
        // silently keeping (or half-replacing) stale geometry would hide the
        // mistake. Stale transform-cache state is still dropped so a failed
        // call cannot leave cached transforms against new geometry.
        if (meshes.find(id) != meshes.end())
        {
            forgetTransformCache(id);
            reportError(error, RenderError::CreationFailed);
            return false;
        }
        forgetTransformCache(id);
        meshes.emplace(id, std::move(mesh));
        reportError(error, RenderError::None);
        return true;
    }
    catch (const std::bad_alloc &)
    {
        reportError(error, RenderError::CreationFailed);
        return false;
    }
}

void HeadlessBackend::destroyMesh(MeshHandle mesh)
{
    forgetTransformCache(mesh);
    meshes.erase(mesh);
}

// Dynamic-geometry refresh (the cloth path): replaces the whole vertex
// stream from the canonical interleaved layout, recomputes the bounding
// sphere, and drops every cache keyed on the old geometry (the transform
// cache and the winding-vote memos). The index buffer and the decimated LOD
// lists reference it unchanged, so they stay valid.
bool HeadlessBackend::updateMeshVertices(MeshHandle mesh, const void *vertices, std::uint32_t vertexCount, RenderError *error)
{
    const auto found = meshes.find(mesh);
    if (found == meshes.end())
    {
        reportError(error, RenderError::InvalidMesh);
        return false;
    }
    HeadlessMesh      &record = found->second;
    const std::size_t  count  = record.vertexCount();
    if (vertices == nullptr || vertexCount == 0 || vertexCount != count)
    {
        // The update must cover the whole creation stream: a partial or
        // oversized write would desynchronize the vertex layout (or leave
        // stale vertices behind the new tail).
        reportError(error, RenderError::InvalidMesh);
        return false;
    }
    try
    {
        const auto *src = static_cast<const float *>(vertices);
        record.positions.resize(count * 3);
        record.normals.resize(count * 3);
        record.colors.resize(count * 3);
        record.uvs.resize(count * 2);
        for (std::size_t v = 0; v < count; ++v)
        {
            const float *s                = src + v * meshVertexFloatCount;
            record.positions[v * 3 + 0]   = s[0];
            record.positions[v * 3 + 1]   = s[1];
            record.positions[v * 3 + 2]   = s[2];
            record.normals[v * 3 + 0]     = s[3];
            record.normals[v * 3 + 1]     = s[4];
            record.normals[v * 3 + 2]     = s[5];
            record.colors[v * 3 + 0]      = s[6];
            record.colors[v * 3 + 1]      = s[7];
            record.colors[v * 3 + 2]      = s[8];
            record.uvs[v * 2 + 0]         = s[9];
            record.uvs[v * 2 + 1]         = s[10];
        }

        glm::vec3 minimum(std::numeric_limits<float>::max());
        glm::vec3 maximum(std::numeric_limits<float>::lowest());
        for (std::size_t v = 0; v < count; ++v)
        {
            const glm::vec3 p(record.positions[v * 3], record.positions[v * 3 + 1], record.positions[v * 3 + 2]);
            minimum = glm::min(minimum, p);
            maximum = glm::max(maximum, p);
        }
        record.boundsCenter   = (minimum + maximum) * 0.5f;
        float radiusSquared   = 0.0f;
        for (std::size_t v = 0; v < count; ++v)
        {
            const glm::vec3 delta = glm::vec3(record.positions[v * 3], record.positions[v * 3 + 1],
                                      record.positions[v * 3 + 2])
                                    - record.boundsCenter;
            radiusSquared = std::max(radiusSquared, glm::dot(delta, delta));
        }
        record.boundsRadius = std::sqrt(radiusSquared);
        record.boundsValid  = std::isfinite(record.boundsRadius);

        // The geometry changed: cached world-space transforms and winding
        // votes derived from the old positions are invalid from this call on.
        forgetTransformCache(mesh);
        for (HeadlessMesh::WindingVoteMemo &memo : record.windingVotes)
        {
            memo.sources.fill(nullptr);
            memo.signs.fill(0.0f);
        }
        reportError(error, RenderError::None);
        return true;
    }
    catch (const std::bad_alloc &)
    {
        reportError(error, RenderError::CreationFailed);
        return false;
    }
}

// Stores the RGBA8 copy and builds the float mip chain (box-filtered
// quarters) used by the sampler.
bool HeadlessBackend::createTexture(TextureHandle id, const TextureDescription &description, RenderError *error)
{
    if (description.pixels == nullptr || description.width == 0 || description.height == 0)
    {
        reportError(error, RenderError::InvalidTexture);
        return false;
    }
    try
    {
        HeadlessTexture texture;
        texture.width   = description.width;
        texture.height  = description.height;
        texture.filter  = description.filter;
        texture.address = description.address;
        // Power-of-two extents let the sampler wrap repeat coordinates through
        // an AND mask instead of a division/modulo pair (identical results;
        // see HeadlessTexture::wrappedCoordinate). Clamp addressing keeps the
        // masks empty so the clamp path is taken.
        const bool pow2Width  = (description.width & (description.width - 1u)) == 0u;
        const bool pow2Height = (description.height & (description.height - 1u)) == 0u;
        texture.wrapMaskX  = pow2Width ? description.width - 1u : 0u;
        texture.wrapMaskY  = pow2Height ? description.height - 1u : 0u;
        texture.repeatPow2 = texture.address == TextureAddress::Repeat && pow2Width && pow2Height;
        texture.pixels.assign(description.pixels,
            description.pixels + static_cast<std::size_t>(description.width) * description.height * 4);
        {
            const std::size_t pc = static_cast<std::size_t>(description.width) * description.height;
            texture.pixelsRGB.resize(pc * 3);
            constexpr float inv = 1.0f / 255.0f;
            for (std::size_t i = 0; i < pc; ++i)
            {
                texture.pixelsRGB[i * 3]     = static_cast<float>(description.pixels[i * 4]) * inv;
                texture.pixelsRGB[i * 3 + 1] = static_cast<float>(description.pixels[i * 4 + 1]) * inv;
                texture.pixelsRGB[i * 3 + 2] = static_cast<float>(description.pixels[i * 4 + 2]) * inv;
            }
        }
        // Mip chain (box-filtered quarters, level 1 down to 1x1): minified
        // fragments sample the level whose footprint matches their screen
        // texel instead of a shimmering base-level fetch (see
        // HeadlessTexture::sampleTrilinear). Non-square and non-pow2 sources
        // still chain cleanly - each level halves its own extents.
        {
            std::vector<HeadlessTexture::MipLevel> chain;
            std::uint32_t levelWidth  = description.width;
            std::uint32_t levelHeight = description.height;
            const float   *source     = texture.pixelsRGB.data();
            while (levelWidth > 1u || levelHeight > 1u)
            {
                const std::uint32_t nextWidth  = std::max(1u, levelWidth / 2u);
                const std::uint32_t nextHeight = std::max(1u, levelHeight / 2u);
                HeadlessTexture::MipLevel mip;
                mip.width  = nextWidth;
                mip.height = nextHeight;
                mip.rgb.assign(static_cast<std::size_t>(nextWidth) * nextHeight * 3, 0.0f);
                for (std::uint32_t y = 0; y < nextHeight; ++y)
                {
                    for (std::uint32_t x = 0; x < nextWidth; ++x)
                    {
                        // 2x2 box; odd extents clamp their far tap to the
                        // last texel so edge texels keep full weight.
                        const std::uint32_t x0 = x * 2u;
                        const std::uint32_t y0 = y * 2u;
                        const std::uint32_t x1 = std::min(x0 + 1u, levelWidth - 1u);
                        const std::uint32_t y1 = std::min(y0 + 1u, levelHeight - 1u);
                        for (std::uint32_t channel = 0; channel < 3u; ++channel)
                        {
                            const float a = source[(static_cast<std::size_t>(y0) * levelWidth + x0) * 3 + channel];
                            const float b = source[(static_cast<std::size_t>(y0) * levelWidth + x1) * 3 + channel];
                            const float c = source[(static_cast<std::size_t>(y1) * levelWidth + x0) * 3 + channel];
                            const float d = source[(static_cast<std::size_t>(y1) * levelWidth + x1) * 3 + channel];
                            mip.rgb[(static_cast<std::size_t>(y) * nextWidth + x) * 3 + channel] = (a + b + c + d) * 0.25f;
                        }
                    }
                }
                chain.push_back(std::move(mip));
                source      = chain.back().rgb.data();
                levelWidth  = nextWidth;
                levelHeight = nextHeight;
            }
            texture.mipChain = std::move(chain);
        }
        textures.emplace(id, std::move(texture));
        reportError(error, RenderError::None);
        return true;
    }
    catch (const std::bad_alloc &)
    {
        reportError(error, RenderError::CreationFailed);
        return false;
    }
}

void HeadlessBackend::destroyTexture(TextureHandle texture)
{
    textures.erase(texture);
}

// Worker-count hint for the parallel-rasterization triggers: lock-free (the
// scheduler publishes its live worker count in an atomic set at start/stop).
// The previous implementation called statistics() here - once or twice per
// draw call - which took the scheduler's lifecycle mutex, its global mutex,
// and every worker queue mutex just to read a value that is constant while
// the scheduler is running: pure lock traffic against the workers doing the
// actual frame work.
std::uint32_t HeadlessBackend::parallelWorkerCount() const noexcept
{
    const unsigned hardware = std::thread::hardware_concurrency();
    if (hardware <= 1u)
        return 1u;
    const std::uint32_t workers = taskScheduler_ != nullptr ? taskScheduler_->runningWorkerCountHint() : 0u;
    return workers != 0u ? workers + 1u : 1u;
}

// Angular-size LOD pick shared by the single-draw and batch paths: returns
// the index list to rasterize (the base list or an LOD-chain level chosen by
// the mesh's apparent size) and, when the mesh carries valid bounds, the
// draw's world-space bounds sphere the decision was made from (reused by
// frustum culling and the winding vote inside prepareRasterization).
const std::vector<std::uint32_t> *HeadlessBackend::activeIndicesForDraw(
    const HeadlessMesh &meshData, const float *modelMatrix, DrawBounds &outBounds, bool &outHaveBounds) const
{
    const std::vector<std::uint32_t> *activeIndices = &meshData.indices;
    outBounds   = DrawBounds{glm::vec3(0.0f), 0.0f};
    outHaveBounds = false;
    if (lodEnabled_ && !meshData.lodIndices.empty() && meshData.boundsValid && cameraInitialized_)
    {
        const glm::mat4 model = glm::make_mat4(modelMatrix);
        outBounds.center      = glm::vec3(model * glm::vec4(meshData.boundsCenter, 1.0f));
        const glm::vec3 basisX = glm::vec3(model[0]);
        const glm::vec3 basisY = glm::vec3(model[1]);
        const glm::vec3 basisZ = glm::vec3(model[2]);
        const float     worldScale
            = std::sqrt(glm::dot(basisX, basisX) + glm::dot(basisY, basisY) + glm::dot(basisZ, basisZ));
        outBounds.radius = meshData.boundsRadius * worldScale;
        outHaveBounds    = true;
        const float dist        = glm::distance(cameraPosition_, outBounds.center);
        const float angularSize = dist > 0.0f ? outBounds.radius / dist : 1.0f; // ~screen fraction the mesh spans
        int         lodLevel    = 0;
        if (angularSize < lodThresholds_[0])
            lodLevel = static_cast<int>(meshData.lodIndices.size()) - 1;
        else if (angularSize < lodThresholds_[1])
            lodLevel = std::min(3, static_cast<int>(meshData.lodIndices.size()) - 1);
        else if (angularSize < lodThresholds_[2])
            lodLevel = std::min(2, static_cast<int>(meshData.lodIndices.size()) - 1);
        else if (angularSize < lodThresholds_[3])
            lodLevel = std::min(1, static_cast<int>(meshData.lodIndices.size()) - 1);
        if (lodLevel > 0)
            activeIndices = &meshData.lodIndices[lodLevel];
    }
    return activeIndices;
}

// One draw: pick an LOD by the mesh's angular size (scaled bounds radius /
// distance), then rasterize - splitting the triangle range across the task
// scheduler when the draw is big enough (each worker depth-tests under a
// pixel-group lock).
void HeadlessBackend::drawMesh(MeshHandle mesh, const float *modelMatrix, const MeshMaterial &material)
{
    const auto found = meshes.find(mesh);
    if (found == meshes.end())
    {
        return;
    }
    const HeadlessMesh &meshData = found->second;

    DrawBounds bounds{glm::vec3(0.0f), 0.0f};
    bool haveBounds = false;
    const std::vector<std::uint32_t> *activeIndices = activeIndicesForDraw(meshData, modelMatrix, bounds, haveBounds);

    const std::size_t triCount = activeIndices->size() / 3;
    statistics_.trianglesSubmitted += triCount;

    RasterSetup setup;
    if (!prepareRasterization(meshData, modelMatrix, material, activeIndices->data(), activeIndices->size(), setup,
            haveBounds ? &bounds : nullptr))
    {
        return;
    }

    const std::uint32_t workers = parallelWorkerCount();
    const bool useParallel      = workers >= 2u && triCount >= parallelTriangleThreshold_ && taskScheduler_ != nullptr
                               && taskScheduler_->isRunning();

    // Band splitting: draws with few triangles but large screen coverage
    // (floor, sky dome) cannot be parallelized by triangle ranges - there is
    // nothing to split. Their cost is per-pixel shading, so split the draw's
    // projected row range into one band per worker; each worker scans the
    // (small) triangle list and rasterizes only its band's rows. Splitting
    // the extent (not the whole framebuffer) keeps the bands balanced when
    // the draw covers only part of the screen - a floor that spans the
    // bottom half would otherwise hand one worker every shaded pixel. Cheap
    // far geometry is excluded by the screen-extent test.
    bool  useBandParallel = false;
    float extentMinY      = 0.0f;
    float extentMaxY      = 0.0f;
    if (!useParallel && workers >= 2u && triCount >= 2u && bandSplitTriangleThreshold_ != 0
        && triCount <= bandSplitTriangleThreshold_ && setup.useTransformCache && taskScheduler_ != nullptr
        && taskScheduler_->isRunning())
    {
        // Projected screen-space row extent from the cached clip positions
        // (band candidates have few triangles, so this scan is cheap). The
        // scan mirrors the rasterizer's near-plane clip exactly - including
        // the interpolated crossings - so every row a clipped triangle can
        // touch lands inside the band range. Skipping the crossings would
        // under-report the extent for geometry near the camera and leave
        // rows un-rasterized.
        float minY = std::numeric_limits<float>::max();
        float maxY = std::numeric_limits<float>::lowest();
        bool  any  = false;
        const auto includeScreenY = [&](float clipY, float clipW)
        {
            const float screenY
                = (1.0f - (clipY / clipW * 0.5f + 0.5f)) * static_cast<float>(framebufferHeight);
            if (!std::isfinite(screenY))
            {
                // Degenerate projection (w at/near zero): the rasterizer's
                // bounding-box clamp keeps it inside the framebuffer, so
                // treat the extent as the full height.
                minY = 0.0f;
                maxY = static_cast<float>(framebufferHeight);
            }
            else
            {
                minY = std::min(minY, screenY);
                maxY = std::max(maxY, screenY);
            }
            any = true;
        };
        for (std::size_t index = 0; index + 2 < activeIndices->size(); index += 3)
        {
            glm::vec4 cornerClip[3];
            bool      cornerValid[3];
            for (std::uint32_t corner = 0; corner < 3; ++corner)
            {
                const std::uint32_t vertexIndex = (*activeIndices)[index + corner];
                cornerValid[corner]             = vertexIndex < setup.vertexCount;
                cornerClip[corner]              = cornerValid[corner] ? setup.transformCache[vertexIndex].clip : glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
            }
            for (std::uint32_t corner = 0; corner < 3; ++corner)
            {
                if (!cornerValid[corner])
                    continue;
                const glm::vec4 &clip    = cornerClip[corner];
                const float      dist    = clip.w + clip.z;
                // Same inclusion rule as the rasterizer's clip walk: a corner
                // at dist >= 0 survives clipping and is projected (whatever
                // its w - degenerate values degrade to a full-height extent).
                if (dist >= 0.0f)
                {
                    includeScreenY(clip.y, clip.w);
                }
                // Edge to the next corner: if it crosses the near plane, the
                // interpolated crossing can project outside the surviving
                // corners' hull (geometry close to the camera), so it must
                // widen the extent.
                const std::uint32_t next = (corner + 1u) % 3u;
                if (!cornerValid[next])
                    continue;
                const glm::vec4 &nextClip = cornerClip[next];
                const float      nextDist = nextClip.w + nextClip.z;
                if ((dist >= 0.0f) != (nextDist >= 0.0f))
                {
                    const float t         = dist / (dist - nextDist);
                    const float crossingY = glm::mix(clip.y, nextClip.y, t);
                    const float crossingW = glm::mix(clip.w, nextClip.w, t);
                    includeScreenY(crossingY, crossingW);
                }
            }
        }
        if (any)
        {
            const float extent = maxY - minY;
            useBandParallel    = extent >= static_cast<float>(framebufferHeight) * bandSplitMinimumScreenFraction_;
            extentMinY         = minY;
            extentMaxY         = maxY;
        }
    }
    // Tile-parallel path (renderer.headless_tile_parallel, default adaptive:
    // on from four logical processors): triggers mirror the default paths
    // exactly (big draws by triangle count, few-triangle wide-coverage draws
    // by projected extent), but the execution bins triangles into 64x64
    // screen tiles with exclusive per-worker ownership - no per-pixel locks.
    // Falls back to the default paths when disabled, the binning allocation
    // fails, or the draw exceeds the record cap. Whole draw lists take the
    // batched variant of this path instead (drawMeshBatch).
    if (tileParallelEnabled_ && workers >= 2u && taskScheduler_ != nullptr && taskScheduler_->isRunning()
        && (useParallel || useBandParallel) && buildTileRecords(meshData, setup))
    {
        if (rasterResultScratch_.size() < workers)
            rasterResultScratch_.resize(workers);
        for (std::uint32_t workerIndex = 0; workerIndex < workers; ++workerIndex)
            rasterResultScratch_[workerIndex].trianglesRasterized = 0;
        TaskGroup tileGroup;
        for (std::uint32_t workerIndex = 1; workerIndex < workers; ++workerIndex)
        {
            const std::uint32_t index = workerIndex;
            // Tiles are owned exclusively by worker index, so a share whose
            // submission is rejected can run inline on the calling thread
            // without any new races.
            if (!taskScheduler_->schedule(tileGroup,
                    [this, &setup, index, workers]()
                    { rasterizeTileRecords(&setup, index, workers, &rasterResultScratch_[index]); }))
            {
                rasterizeTileRecords(&setup, index, workers, &rasterResultScratch_[index]);
            }
        }
        rasterizeTileRecords(&setup, 0u, workers, &rasterResultScratch_[0]);
        tileGroup.wait(*taskScheduler_);
        std::uint64_t totalRasterized = 0;
        for (std::uint32_t workerIndex = 0; workerIndex < workers; ++workerIndex)
            totalRasterized += rasterResultScratch_[workerIndex].trianglesRasterized;
        statistics_.trianglesRasterized += totalRasterized;
        return;
    }

    if (useBandParallel)
    {
        if (rasterResultScratch_.size() < workers)
            rasterResultScratch_.resize(workers);
        TaskGroup group;
        // Band boundaries over the projected extent, rounded outward so no
        // coverage falls between bands; a short extent yields empty bands
        // (their row loop starts past its end and exits immediately).
        const std::uint32_t bandTop    = std::min<std::uint32_t>(framebufferHeight - 1u,
            static_cast<std::uint32_t>(std::max(0.0f, std::floor(extentMinY))));
        const std::uint32_t bandBottom = std::min(framebufferHeight - 1u,
            static_cast<std::uint32_t>(std::ceil(extentMaxY)));
        const std::uint32_t extentRows = bandBottom >= bandTop ? bandBottom - bandTop + 1u : 1u;
        const std::uint32_t bandRows   = std::max<std::uint32_t>(1u, extentRows / workers);
        for (std::uint32_t workerIndex = 1; workerIndex < workers; ++workerIndex)
        {
            const std::uint32_t begin = std::min<std::uint32_t>(framebufferHeight, bandTop + workerIndex * bandRows);
            const std::uint32_t end   = workerIndex + 1u < workers ? std::min<std::uint32_t>(framebufferHeight, begin + bandRows)
                                                                   : bandBottom + 1u;
            // A rejected submission (scheduler stopping between the trigger
            // check and the schedule, or the outstanding bound hit) used to
            // silently drop this band - its rows would never rasterize.
            // Bands are disjoint, so running the share inline is race-free.
            if (!taskScheduler_->schedule(group,
                    [this, &meshData, &setup, &triCount, begin, end, workerIndex]()
                    {
                        rasterizeTriangles(meshData, setup, 0, triCount, true, &rasterResultScratch_[workerIndex], begin, end - 1u);
                    }))
            {
                rasterizeTriangles(meshData, setup, 0, triCount, true, &rasterResultScratch_[workerIndex], begin, end - 1u);
            }
        }
        rasterizeTriangles(meshData, setup, 0, triCount, true, &rasterResultScratch_[0], bandTop,
            std::min<std::uint32_t>(bandBottom, bandTop + bandRows - 1u));
        group.wait(*taskScheduler_);
        std::uint64_t totalRasterized = 0;
        for (std::uint32_t workerIndex = 0; workerIndex < workers; ++workerIndex)
            totalRasterized += rasterResultScratch_[workerIndex].trianglesRasterized;
        statistics_.trianglesRasterized += totalRasterized;
        return;
    }

    if (!useParallel)
    {
        rasterizeTriangles(meshData, setup, 0, triCount, false, nullptr);
        return;
    }

    if (rasterResultScratch_.size() < workers)
        rasterResultScratch_.resize(workers);
    std::vector<StripResult> &results = rasterResultScratch_;
    for (std::uint32_t workerIndex = 0; workerIndex < workers; ++workerIndex)
        results[workerIndex].trianglesRasterized = 0;
    const std::size_t        baseChunk = triCount / workers;
    const std::size_t        remainder = triCount % workers;
    std::size_t              triCursor = 0;
    TaskGroup                group;
    for (std::uint32_t i = 1; i < workers; ++i)
    {
        const std::size_t chunk    = baseChunk + (i < remainder ? 1 : 0);
        const std::size_t triStart = triCursor;
        triCursor += chunk;
        const std::size_t triEnd = triCursor;
        if (triStart >= triEnd)
        {
            continue;
        }
        const std::uint32_t workerIndex = i;
        // A rejected submission used to silently drop this triangle range
        // (its geometry would never rasterize). Ranges are disjoint, so
        // running the share inline is race-free.
        if (!taskScheduler_->schedule(group,
                [this, &meshData, &setup, triStart, triEnd, &results, workerIndex]()
                { rasterizeTriangles(meshData, setup, triStart, triEnd, true, &results[workerIndex]); }))
        {
            rasterizeTriangles(meshData, setup, triStart, triEnd, true, &results[workerIndex]);
        }
    }

    {
        const std::size_t triStart = triCursor;
        const std::size_t triEnd   = triCount;
        rasterizeTriangles(meshData, setup, triStart, triEnd, true, &results[0]);
    }
    group.wait(*taskScheduler_);
    std::uint64_t totalRasterized = 0;
    for (const StripResult &r : results)
    {
        totalRasterized += r.trianglesRasterized;
    }
    statistics_.trianglesRasterized += totalRasterized;
}

// Instance draws on the CPU backend replay as repeated drawMesh calls. Each
// instance carries its own model matrix, so the shared transform cache
// cannot carry work across instances (every call stamps a fresh generation
// and re-transforms the vertices); the cache still deduplicates within one
// call's overlapping index ranges. Whole-command-list batching happens one
// level up, in drawMeshBatch.
void HeadlessBackend::drawMeshInstances(
    MeshHandle mesh, const float *modelMatrices, std::uint32_t instanceCount, const MeshMaterial &material)
{
    if (modelMatrices == nullptr)
    {
        return;
    }
    for (std::uint32_t index = 0; index < instanceCount; ++index)
    {
        drawMesh(mesh, modelMatrices + static_cast<std::size_t>(index) * 16u, material);
    }
}

// Batched tile-parallel rasterization of a whole submitted draw list (see
// RenderBackend::drawMeshBatch). Phases, per chunk of items:
//   1. (calling thread) per item: mesh lookup, LOD pick,
//      prepareRasterization, then immediate fan-record emission - preparing
//      and emitting one item at a time keeps the shared transform cache
//      valid for exactly the item whose records are being baked, exactly
//      like the per-draw replay interleaves them. Items accumulate until
//      the next one would overflow the batch record cap; oversized single
//      items (and any allocation failure) fall back to the per-draw paths,
//      which produce identical output.
//   2. (calling thread) one two-pass binning of the chunk's records into
//      64x64 tiles; per-tile lists keep submission order (ascending record
//      ids), so every pixel sees the same triangle order the sequential
//      replay would show it - equal-depth writes, blended backdrop layers
//      and depth-early rejections all resolve identically.
//   3. (all workers) each worker rasterizes the tiles with
//      index % workerCount == workerIndex; exclusive tile ownership means
//      no per-pixel locks.
// Chunks rasterize to completion in order, so submission order also holds
// across chunk boundaries. This is the path that makes many-small-draw
// scenes (streamed LOD chunks, scattered props) use the whole machine: no
// single draw is large enough for the per-draw parallel thresholds, but the
// frame as a whole is.
void HeadlessBackend::drawMeshBatch(const MeshBatchItem *items, std::uint32_t itemCount)
{
    if (items == nullptr)
    {
        return;
    }
    const std::uint32_t workers = parallelWorkerCount();
    if (!batchedTileRasterizationEnabled_ || workers < 2u || itemCount < 2u || taskScheduler_ == nullptr
        || !taskScheduler_->isRunning() || framebufferWidth == 0 || framebufferHeight == 0)
    {
        RenderBackend::drawMeshBatch(items, itemCount);
        return;
    }

    const MeshMaterial defaultMaterial;
    std::uint32_t     index = 0;
    while (index < itemCount)
    {
        // ---- chunk preparation (serial) --------------------------------------
        // [index, consumed) are the items of this chunk; consumed only
        // advances past items whose records are in tileRecords_ (or that
        // were safely skipped/handled inline).
        std::uint32_t consumed = index;
        batchSetups_.clear();
        tileRecords_.clear();
        while (consumed < itemCount)
        {
            const MeshBatchItem &item = items[consumed];
            if (item.modelMatrix == nullptr || item.mesh == invalidMesh)
            {
                // The sequential replay skips these without touching
                // statistics; so does the batch.
                ++consumed;
                continue;
            }
            const auto found = meshes.find(item.mesh);
            if (found == meshes.end())
            {
                // Unknown mesh: drawMesh returns before its submitted-count,
                // and so does the batch.
                ++consumed;
                continue;
            }
            const HeadlessMesh &meshData = found->second;

            DrawBounds                             bounds{glm::vec3(0.0f), 0.0f};
            bool                                   haveBounds = false;
            const std::vector<std::uint32_t> *activeIndices
                = activeIndicesForDraw(meshData, item.modelMatrix, bounds, haveBounds);
            const std::size_t triCount = activeIndices->size() / 3;
            const std::size_t recordEstimate = triCount * 6u; // worst case: near-clip + screen-rect clip fans
            if (recordEstimate > maxBatchTileRecords_)
            {
                // A single draw too big for the batch scratch: the per-draw
                // paths have their own larger cap and their own trigger
                // thresholds, and this item has not been counted yet, so
                // replaying it through drawMesh keeps the statistics exact.
                if (consumed == index)
                {
                    drawMesh(item.mesh,
                        item.modelMatrix,
                        item.material != nullptr ? *item.material : defaultMaterial);
                    ++consumed;
                    index = consumed;
                    continue;
                }
                break; // flush the chunk first; this item starts the next one
            }
            if (!tileRecords_.empty() && tileRecords_.size() + recordEstimate > maxBatchTileRecords_)
            {
                break; // chunk full: flush, then this item starts the next one
            }

            statistics_.trianglesSubmitted += triCount;
            RasterSetup setup;
            const MeshMaterial &material = item.material != nullptr ? *item.material : defaultMaterial;
            if (!prepareRasterization(meshData, item.modelMatrix, material, activeIndices->data(),
                    activeIndices->size(), setup, haveBounds ? &bounds : nullptr))
            {
                // Culled or unusable: submitted was already counted, exactly
                // like drawMesh counts before its own prepare.
                ++consumed;
                continue;
            }

            const std::uint32_t setupIndex = static_cast<std::uint32_t>(batchSetups_.size());
            const std::size_t   recordsBefore = tileRecords_.size();
            try
            {
                TileBinRecord fans[6]; // near-clip + screen-rect clip fans, see prepareFanTriangles
                for (std::size_t triangle = 0; triangle < triCount; ++triangle)
                {
                    const std::size_t fanCount = prepareFanTriangles(meshData, setup, triangle, fans);
                    for (std::size_t fan = 0; fan < fanCount; ++fan)
                    {
                        fans[fan].setupIndex = setupIndex;
                        tileRecords_.push_back(fans[fan]);
                    }
                }
                batchSetups_.push_back(std::move(setup));
            }
            catch (const std::bad_alloc &)
            {
                // Drop the partially emitted item (its records are
                // incomplete), refund its submitted count and hand it to the
                // single-draw path, then flush what complete items did make
                // it into the chunk.
                tileRecords_.resize(recordsBefore);
                statistics_.trianglesSubmitted -= triCount;
                drawMesh(item.mesh, item.modelMatrix, material);
                ++consumed;
                break;
            }
            ++consumed;
        }

        // ---- chunk flush ------------------------------------------------------
        if (tileRecords_.empty())
        {
            index = consumed;
            continue;
        }
        if (!buildBatchTileRecords())
        {
            // Binning refused the chunk (allocation failure or an un-binnable
            // record span). The records themselves are complete geometry in
            // submission order, so replay them sequentially over the whole
            // framebuffer - the same pixels in the same order as the
            // parallel path, one record at a time.
            for (const TileBinRecord &record : tileRecords_)
            {
                if (rasterizeFanRecord(batchSetups_[record.setupIndex], record, 0,
                        static_cast<std::int32_t>(framebufferWidth) - 1, 0,
                        static_cast<std::int32_t>(framebufferHeight) - 1))
                {
                    ++statistics_.trianglesRasterized;
                }
            }
            index = consumed;
            continue;
        }

        try
        {
            if (rasterResultScratch_.size() < workers)
                rasterResultScratch_.resize(workers);
        }
        catch (const std::bad_alloc &)
        {
            // No per-worker result slots: the counts cannot be gathered, but
            // the pixels still must be. Replay the binned records
            // sequentially (the per-record loop above) so the chunk draws
            // exactly once either way.
            for (const TileBinRecord &record : tileRecords_)
            {
                if (rasterizeFanRecord(batchSetups_[record.setupIndex], record, 0,
                        static_cast<std::int32_t>(framebufferWidth) - 1, 0,
                        static_cast<std::int32_t>(framebufferHeight) - 1))
                {
                    ++statistics_.trianglesRasterized;
                }
            }
            index = consumed;
            continue;
        }
        for (std::uint32_t workerIndex = 0; workerIndex < workers; ++workerIndex)
            rasterResultScratch_[workerIndex].trianglesRasterized = 0;
        TaskGroup tileGroup;
        const RasterSetup *setups = batchSetups_.data();
        for (std::uint32_t workerIndex = 1; workerIndex < workers; ++workerIndex)
        {
            const std::uint32_t batchWorkerIndex = workerIndex;
            // Tiles are owned exclusively by worker index, so a share whose
            // submission is rejected can run inline on the calling thread
            // without any new races.
            if (!taskScheduler_->schedule(tileGroup,
                    [this, setups, batchWorkerIndex, workers]()
                    { rasterizeTileRecords(setups, batchWorkerIndex, workers, &rasterResultScratch_[batchWorkerIndex]); }))
            {
                rasterizeTileRecords(setups, batchWorkerIndex, workers, &rasterResultScratch_[batchWorkerIndex]);
            }
        }
        rasterizeTileRecords(setups, 0u, workers, &rasterResultScratch_[0]);
        tileGroup.wait(*taskScheduler_);
        std::uint64_t totalRasterized = 0;
        for (std::uint32_t workerIndex = 0; workerIndex < workers; ++workerIndex)
            totalRasterized += rasterResultScratch_[workerIndex].trianglesRasterized;
        statistics_.trianglesRasterized += totalRasterized;
        index = consumed;
    }
}

// Shell-fur stack on the CPU rasterizer: one rasterization pass per shell
// layer, each with displaced vertices, the strand-crop alpha, and alpha
// blending (no depth write). Mirrors the Vulkan fur pipeline exactly - same
// furShade/furAlpha formulas, same displacement, same blend space - so both
// backends produce the same image (the parity contract in mesh.frag's header
// comment, extended by fur.frag and backends.md).
//
// Sequential rasterization only: the shell stack is fill-rate-bound and the
// band/tile splits parallelize per-draw pixel work, not the layer loop; fur
// draws reuse the transform cache per layer (each layer stamps a fresh
// generation), which is where the CPU cost actually amortizes.
void HeadlessBackend::drawFur(MeshHandle mesh, const float *modelMatrix, const MeshMaterial &material,
    const FurLayerParams &params)
{
    const auto found = meshes.find(mesh);
    if (found == meshes.end() || modelMatrix == nullptr || framebufferWidth == 0 || framebufferHeight == 0)
    {
        return;
    }
    const HeadlessMesh &meshData = found->second;

    // Same hard layer bound as the Vulkan backend: a corrupted params must
    // not turn one call into a pathological loop.
    const std::uint32_t layerCount = std::clamp(params.layerCount, 0u, 64u);
    if (layerCount == 0)
    {
        return;
    }

    for (std::uint32_t layer = 1; layer <= layerCount; ++layer)
    {
        RasterSetup setup;
        if (!prepareRasterization(meshData,
                modelMatrix,
                material,
                meshData.indices.data(),
                meshData.indices.size(),
                setup,
                nullptr,
                &params,
                static_cast<float>(layer)))
        {
            return;
        }
        // Submitted counts per layer whose triangles were actually handed to
        // the rasterizer; a failed prepare aborts the stack without counting
        // layers that never ran.
        statistics_.trianglesSubmitted += setup.triangleCount;
        rasterizeTriangles(meshData, setup, 0, setup.triangleCount, false, nullptr);
        statistics_.trianglesRasterized += setup.triangleCount;
    }
}

// Fur across several instances is repeated drawFur calls (same convention as
// drawMeshInstances on this backend).
void HeadlessBackend::drawFurInstances(MeshHandle mesh, const float *modelMatrices, std::uint32_t instanceCount,
    const MeshMaterial &material, const FurLayerParams &params)
{
    if (modelMatrices == nullptr)
    {
        return;
    }
    for (std::uint32_t index = 0; index < instanceCount; ++index)
    {
        drawFur(mesh, modelMatrices + static_cast<std::size_t>(index) * 16u, material, params);
    }
}

// Software overlay rasterizer: maps display pixels to framebuffer pixels,
// scissors each command's clip rectangle, then rasterizes indexed triangles
// with barycentric coverage, texture sampling, vertex colors, and "over"
// alpha blending into the color buffer.
void HeadlessBackend::drawOverlay(const OverlayDrawData &data)
{
    if (framebufferWidth == 0 || framebufferHeight == 0 || data.vertices == nullptr || data.indices == nullptr
        || data.commands == nullptr || data.displaySize[0] <= 0.0f || data.displaySize[1] <= 0.0f
        || data.framebufferScale[0] <= 0.0f || data.framebufferScale[1] <= 0.0f)
    {
        return;
    }

    // The overlay's display-to-framebuffer scale lands in surface pixels; the
    // supersampled framebuffer adds its sample factors on top, so UI quads
    // cover the same screen area and the resolve keeps them crisp.
    const float scaleX  = data.framebufferScale[0] * static_cast<float>(sampleScaleX_);
    const float scaleY  = data.framebufferScale[1] * static_cast<float>(sampleScaleY_);
    const auto  channel = [](std::uint32_t colorValue, unsigned shift)
    { return static_cast<float>((colorValue >> shift) & 0xffu) / 255.0f; };
    const auto edge = [](float ax, float ay, float bx, float by, float px, float py)
    { return (px - ax) * (by - ay) - (py - ay) * (bx - ax); };

    for (std::uint32_t commandIndex = 0; commandIndex < data.commandCount; ++commandIndex)
    {
        const OverlayDrawCommand &draw = data.commands[commandIndex];
        if (draw.elementCount < 3 || draw.indexOffset > data.indexCount
            || draw.elementCount > data.indexCount - draw.indexOffset)
        {
            continue;
        }

        const float clipMinX = (draw.clipRectangle[0] - data.displayPosition[0]) * scaleX;
        const float clipMinY = (draw.clipRectangle[1] - data.displayPosition[1]) * scaleY;
        const float clipMaxX = (draw.clipRectangle[2] - data.displayPosition[0]) * scaleX;
        const float clipMaxY = (draw.clipRectangle[3] - data.displayPosition[1]) * scaleY;
        if (!std::isfinite(clipMinX) || !std::isfinite(clipMinY) || !std::isfinite(clipMaxX)
            || !std::isfinite(clipMaxY))
        {
            continue;
        }
        const float        boundedLeft   = std::clamp(clipMinX, 0.0f, static_cast<float>(framebufferWidth));
        const float        boundedTop    = std::clamp(clipMinY, 0.0f, static_cast<float>(framebufferHeight));
        const float        boundedRight  = std::clamp(clipMaxX, 0.0f, static_cast<float>(framebufferWidth));
        const float        boundedBottom = std::clamp(clipMaxY, 0.0f, static_cast<float>(framebufferHeight));
        const std::int32_t clipLeft      = static_cast<std::int32_t>(std::floor(boundedLeft));
        const std::int32_t clipTop       = static_cast<std::int32_t>(std::floor(boundedTop));
        const std::int32_t clipRight     = static_cast<std::int32_t>(std::ceil(boundedRight));
        const std::int32_t clipBottom    = static_cast<std::int32_t>(std::ceil(boundedBottom));
        if (clipLeft >= clipRight || clipTop >= clipBottom)
            continue;

        const HeadlessTexture *texture = mapFor(draw.texture, neutralWhite_);
        if (texture == nullptr || texture->width == 0 || texture->height == 0 || texture->pixels.size() < 4)
            texture = &neutralWhite_;

        const auto sampleRgba = [texture](float u, float v, float out[4])
        {
            if (texture->address == TextureAddress::Repeat)
            {
                u -= std::floor(u);
                v -= std::floor(v);
            }
            u                   = std::clamp(u, 0.0f, 1.0f);
            v                   = std::clamp(v, 0.0f, 1.0f);
            const float tx      = u * static_cast<float>(texture->width - 1);
            const float ty      = v * static_cast<float>(texture->height - 1);
            const auto  pixelAt = [texture](std::uint32_t x, std::uint32_t y)
            { return texture->pixels.data() + (static_cast<std::size_t>(y) * texture->width + x) * 4; };
            if (texture->filter == TextureFilter::Nearest || (texture->width == 1 && texture->height == 1))
            {
                const std::uint32_t x = static_cast<std::uint32_t>(
                    std::clamp(std::lround(tx), 0l, static_cast<long>(texture->width - 1)));
                const std::uint32_t y = static_cast<std::uint32_t>(
                    std::clamp(std::lround(ty), 0l, static_cast<long>(texture->height - 1)));
                const std::uint8_t *pixel = pixelAt(x, y);
                for (int c = 0; c < 4; ++c)
                    out[c] = static_cast<float>(pixel[c]) / 255.0f;
                return;
            }
            const std::uint32_t x0  = static_cast<std::uint32_t>(std::floor(tx));
            const std::uint32_t y0  = static_cast<std::uint32_t>(std::floor(ty));
            const std::uint32_t x1  = std::min(x0 + 1, texture->width - 1);
            const std::uint32_t y1  = std::min(y0 + 1, texture->height - 1);
            const float         fx  = tx - static_cast<float>(x0);
            const float         fy  = ty - static_cast<float>(y0);
            const std::uint8_t *p00 = pixelAt(x0, y0);
            const std::uint8_t *p10 = pixelAt(x1, y0);
            const std::uint8_t *p01 = pixelAt(x0, y1);
            const std::uint8_t *p11 = pixelAt(x1, y1);
            for (int c = 0; c < 4; ++c)
            {
                const float a = static_cast<float>(p00[c]) * (1.0f - fx) + static_cast<float>(p10[c]) * fx;
                const float b = static_cast<float>(p01[c]) * (1.0f - fx) + static_cast<float>(p11[c]) * fx;
                out[c]        = (a * (1.0f - fy) + b * fy) / 255.0f;
            }
        };

        const std::uint32_t triangleCount = draw.elementCount / 3;
        for (std::uint32_t triangle = 0; triangle < triangleCount; ++triangle)
        {
            const std::uint32_t indexBase = draw.indexOffset + triangle * 3;
            const std::uint64_t ia64      = static_cast<std::uint64_t>(data.indices[indexBase]) + draw.vertexOffset;
            const std::uint64_t ib64      = static_cast<std::uint64_t>(data.indices[indexBase + 1]) + draw.vertexOffset;
            const std::uint64_t ic64      = static_cast<std::uint64_t>(data.indices[indexBase + 2]) + draw.vertexOffset;
            if (ia64 >= data.vertexCount || ib64 >= data.vertexCount || ic64 >= data.vertexCount)
                continue;
            const OverlayVertex &a    = data.vertices[ia64];
            const OverlayVertex &b    = data.vertices[ib64];
            const OverlayVertex &c    = data.vertices[ic64];
            const float          ax   = (a.position[0] - data.displayPosition[0]) * scaleX;
            const float          ay   = (a.position[1] - data.displayPosition[1]) * scaleY;
            const float          bx   = (b.position[0] - data.displayPosition[0]) * scaleX;
            const float          by   = (b.position[1] - data.displayPosition[1]) * scaleY;
            const float          cx   = (c.position[0] - data.displayPosition[0]) * scaleX;
            const float          cy   = (c.position[1] - data.displayPosition[1]) * scaleY;
            const float          area = edge(ax, ay, bx, by, cx, cy);
            if (std::abs(area) <= 1.0e-8f)
                continue;

            const std::int32_t left = std::max(clipLeft, static_cast<std::int32_t>(std::floor(std::min({ax, bx, cx}))));
            const std::int32_t top  = std::max(clipTop, static_cast<std::int32_t>(std::floor(std::min({ay, by, cy}))));
            const std::int32_t right
                = std::min(clipRight, static_cast<std::int32_t>(std::ceil(std::max({ax, bx, cx}))));
            const std::int32_t bottom
                = std::min(clipBottom, static_cast<std::int32_t>(std::ceil(std::max({ay, by, cy}))));
            if (left >= right || top >= bottom)
                continue;

            // Per-vertex RGBA channels are constant across the triangle's
            // pixels: decode them once instead of per pixel.
            const float vertexChannels[3][4]
                = {{channel(a.color, 0), channel(a.color, 8), channel(a.color, 16), channel(a.color, 24)},
                    {channel(b.color, 0), channel(b.color, 8), channel(b.color, 16), channel(b.color, 24)},
                    {channel(c.color, 0), channel(c.color, 8), channel(c.color, 16), channel(c.color, 24)}};

            bool wroteAnyPixel = false;
            for (std::int32_t y = top; y < bottom; ++y)
            {
                for (std::int32_t x = left; x < right; ++x)
                {
                    const float px = static_cast<float>(x) + 0.5f;
                    const float py = static_cast<float>(y) + 0.5f;
                    const float wa = edge(bx, by, cx, cy, px, py) / area;
                    const float wb = edge(cx, cy, ax, ay, px, py) / area;
                    const float wc = 1.0f - wa - wb;
                    if (wa < -1.0e-5f || wb < -1.0e-5f || wc < -1.0e-5f)
                        continue;

                    const float u = a.uv[0] * wa + b.uv[0] * wb + c.uv[0] * wc;
                    const float v = a.uv[1] * wa + b.uv[1] * wb + c.uv[1] * wc;
                    float       texel[4]{};
                    sampleRgba(u, v, texel);
                    const float vertexColor[4]
                        = {vertexChannels[0][0] * wa + vertexChannels[1][0] * wb + vertexChannels[2][0] * wc,
                            vertexChannels[0][1] * wa + vertexChannels[1][1] * wb + vertexChannels[2][1] * wc,
                            vertexChannels[0][2] * wa + vertexChannels[1][2] * wb + vertexChannels[2][2] * wc,
                            vertexChannels[0][3] * wa + vertexChannels[1][3] * wb + vertexChannels[2][3] * wc};
                    const float alpha = std::clamp(texel[3] * vertexColor[3], 0.0f, 1.0f);
                    if (alpha <= 0.0f)
                        continue;
                    std::uint8_t *destination
                        = color.data()
                        + (static_cast<std::size_t>(y) * framebufferWidth + static_cast<std::uint32_t>(x)) * 4;
                    for (std::uint32_t component = 0; component < 3; ++component)
                    {
                        const float source     = texel[component] * vertexColor[component];
                        const float target     = static_cast<float>(destination[component]) / 255.0f;
                        destination[component] = static_cast<std::uint8_t>(
                            std::lround(std::clamp(source * alpha + target * (1.0f - alpha), 0.0f, 1.0f) * 255.0f));
                    }
                    const float destinationAlpha = static_cast<float>(destination[3]) / 255.0f;
                    destination[3]
                        = static_cast<std::uint8_t>(std::lround((alpha + destinationAlpha * (1.0f - alpha)) * 255.0f));
                    wroteAnyPixel = true;
                }
            }
            // Submitted counts every processed triangle; rasterized only the
            // ones that actually shaded at least one pixel, matching the mesh
            // paths' semantics.
            ++statistics_.trianglesSubmitted;
            if (wroteAnyPixel)
            {
                ++statistics_.trianglesRasterized;
            }
        }
    }
}

// Textures resolve to neutral stand-ins instead of failing; `neutral`
// picks which stand-in (white vs flat normal) fits the slot.
const HeadlessTexture *HeadlessBackend::mapFor(TextureHandle handle, const HeadlessTexture &neutral) const
{
    if (handle == invalidTexture)
    {
        return &neutral;
    }
    const auto found = textures.find(handle);
    return found != textures.end() ? &found->second : &neutral;
}

// Finds or creates this draw's transform-cache entry. Existing entries
// refresh their LRU stamp and come back as-is; a new entry first evicts
// least-recently-touched ones until it fits the budget, and a mesh larger
// than the whole budget is never cached at all.
HeadlessBackend::TransformCacheEntry *HeadlessBackend::transformCacheEntryFor(
    const HeadlessMesh &mesh, std::size_t vertexCount)
{
    if (mesh.handle == invalidMesh)
    {
        return nullptr;
    }
    const MeshHandle handle = mesh.handle;
    const auto       found  = transformCaches_.find(handle);
    if (found != transformCaches_.end())
    {
        found->second.lastTouch = ++transformCacheClock_;
        return &found->second;
    }
    const std::size_t entryBytes = vertexCount * sizeof(TransformedVertex);
    if (entryBytes > transformCacheBudgetBytes_)
    {
        return nullptr;
    }
    while (transformCacheBytesUsed_ + entryBytes > transformCacheBudgetBytes_)
    {
        auto victim = transformCaches_.end();
        for (auto it = transformCaches_.begin(); it != transformCaches_.end(); ++it)
        {
            if (victim == transformCaches_.end() || it->second.lastTouch < victim->second.lastTouch)
            {
                victim = it;
            }
        }
        if (victim == transformCaches_.end())
        {
            break;
        }
        transformCacheBytesUsed_ -= victim->second.vertices.size() * sizeof(TransformedVertex);
        transformCaches_.erase(victim);
    }
    if (transformCacheBytesUsed_ + entryBytes > transformCacheBudgetBytes_)
    {
        return nullptr;
    }
    try
    {
        TransformCacheEntry &entry = transformCaches_[handle];
        entry.vertices.resize(vertexCount);
        transformCacheBytesUsed_ += entryBytes;
        entry.lastTouch   = ++transformCacheClock_;
        entry.worldValid  = false;
        entry.clipValid   = false;
        return &entry;
    }
    catch (const std::bad_alloc &)
    {
        transformCaches_.erase(handle);
        return nullptr;
    }
}

void HeadlessBackend::forgetTransformCache(MeshHandle mesh)
{
    const auto found = transformCaches_.find(mesh);
    if (found == transformCaches_.end())
    {
        return;
    }
    transformCacheBytesUsed_ -= found->second.vertices.size() * sizeof(TransformedVertex);
    transformCaches_.erase(found);
}

// Gathers everything rasterizeTriangles needs. Order matters: frustum cull,
// transform the vertices (through the persistent per-mesh cache when it fits
// the budget), resolve textures to stand-ins, then probe a spread of the
// mesh's triangles against their vertex normals to decide if back-face
// culling is safe (only attempted when the whole mesh is in front of the
// camera). A majority over up to 64 samples replaces the old single-triangle
// probe: one sample on a sharp crease (where a smooth vertex normal leans
// across the edge) used to flip the sign and cull nearly every front face.
bool HeadlessBackend::prepareRasterization(const HeadlessMesh &mesh, const float *modelMatrix,
    const MeshMaterial &material, const std::uint32_t *activeIndices, std::size_t activeIndexCount,
    RasterSetup &outSetup, const DrawBounds *precomputedBounds, const FurLayerParams *furParams, float furLayer)
{
    if (framebufferWidth == 0 || framebufferHeight == 0)
    {
        return false;
    }

    const std::size_t triangleCount = activeIndexCount / 3;

    outSetup.triangleCount     = triangleCount;
    outSetup.indexData         = activeIndices;
    outSetup.model             = glm::make_mat4(modelMatrix);
    outSetup.viewProjection    = viewProjection_ * outSetup.model;
    outSetup.viewProjectionWorld = viewProjection_;
    // SAFETY: inverse-transpose normal transform, mirroring mesh.vert /
    // fur.vert. mat3(model) is wrong for non-uniformly scaled models (sheared
    // normals light incorrectly); the physically correct transform is
    // transpose(inverse(mat3(model))). For uniformly scaled models (the common
    // case) the result matches up to a scalar and the shading stage
    // renormalizes, so only direction matters. Singular matrices surface as
    // non-finite vertex normals, which the per-vertex fallback below replaces
    // with the up vector.
    outSetup.modelNormalMatrix = material.unlit ? glm::mat3(1.0f)
                                                : glm::transpose(glm::inverse(glm::mat3(outSetup.model)));
    outSetup.unlit             = material.unlit;
    // Transparent draws alpha blend with depth writes off (the fur commit's
    // rule, applied to whole meshes): the blend must see what the opaque
    // scene behind the surface wrote, and co-transparent surfaces must not
    // occlude each other. Depth testing still applies, so opaque geometry
    // in front of the surface keeps occluding it. Consumers order their
    // transparent draws after the opaque pass, far to near (the documented
    // contract, same as the fur stack's).
    outSetup.opacity           = glm::clamp(material.opacity, 0.0f, 1.0f);
    outSetup.transparent       = outSetup.opacity < 1.0f;
    outSetup.depthWrite        = material.depthWrite && !outSetup.transparent;
    // Shell-fur state: the transform below displaces along the object-space
    // normals once per layer; a fur draw stamps a fresh transform generation
    // so the cache (reused across triangles within the layer) never carries
    // a previous layer's displaced positions.
    outSetup.fur               = furParams != nullptr;
    outSetup.furLayerRatio     = outSetup.fur ? glm::clamp(furLayer / glm::max(float(furParams->layerCount), 1.0f), 0.0f, 1.0f) : 0.0f;
    outSetup.furLayerIndex     = outSetup.fur ? furLayer : 0.0f;
    outSetup.furHeight         = outSetup.fur ? furParams->furHeight : 0.0f;
    outSetup.furGravity        = outSetup.fur ? furParams->gravityStrength : 0.0f;
    outSetup.furDensityBoost   = outSetup.fur ? furParams->densityBoost : 6.0f;
    outSetup.furOuterFade      = outSetup.fur ? furParams->outerLayerFade : 1.0f;

    // World-space bounds sphere, computed at most once per draw. drawMesh
    // already builds one for its LOD pick and passes it in; otherwise it is
    // derived here on first use by frustum culling or the winding vote.
    bool boundsReady = false;
    const auto ensureWorldBounds = [&]()
    {
        if (boundsReady)
            return;
        if (precomputedBounds != nullptr)
        {
            outSetup.boundsWorldCenter = precomputedBounds->center;
            outSetup.boundsWorldRadius = precomputedBounds->radius;
        }
        else
        {
            const glm::vec4 worldCenter4 = outSetup.model * glm::vec4(mesh.boundsCenter, 1.0f);
            outSetup.boundsWorldCenter   = glm::vec3(worldCenter4);
            const glm::vec3 basisX       = glm::vec3(outSetup.model[0]);
            const glm::vec3 basisY       = glm::vec3(outSetup.model[1]);
            const glm::vec3 basisZ       = glm::vec3(outSetup.model[2]);
            const float     worldScale   = std::sqrt(
                glm::dot(basisX, basisX) + glm::dot(basisY, basisY) + glm::dot(basisZ, basisZ));
            outSetup.boundsWorldRadius = mesh.boundsRadius * worldScale;
        }
        boundsReady = true;
    };

    if (frustumCullingEnabled_ && cameraInitialized_ && mesh.boundsValid && triangleCount >= 4)
    {
        ensureWorldBounds();
        const glm::vec4 worldCenter(outSetup.boundsWorldCenter, 1.0f);
        if (sphereOutsideFrustum(frustumPlanes_.data(), worldCenter, outSetup.boundsWorldRadius))
        {
            return false;
        }
    }

    const std::size_t vertexCount = mesh.vertexCount();
    outSetup.vertexCount         = vertexCount;
    outSetup.useTransformCache   = false;
    outSetup.transformCache      = nullptr;
    if (triangleCount >= 2 && activeIndexCount > vertexCount
        && transformCacheBudgetBytes_ >= sizeof(TransformedVertex)
        && vertexCount <= transformCacheBudgetBytes_ / sizeof(TransformedVertex))
    {
        // Persistent per-mesh transform cache: the entry survives draws and
        // frames, so a static model matrix under a static camera skips the
        // vertex stage entirely, and a camera move reprojects the clip tier
        // without re-running the model stage. Draws that cannot be cached
        // keep the direct per-vertex path inside the rasterizers.
        TransformCacheEntry *entry = transformCacheEntryFor(mesh, vertexCount);
        if (entry != nullptr)
        {
            // Fur clips through the raw view-projection (gravity lands in
            // world space, after the model transform); regular draws clip
            // through the combined view-projection of the local position.
            const glm::mat4 clipMatrix = outSetup.fur ? outSetup.viewProjectionWorld : outSetup.viewProjection;
            const bool worldKeyMatches = entry->worldValid
                && transformMatricesEqual(entry->worldModel, outSetup.model) && entry->fur == outSetup.fur
                && entry->worldUnlit == outSetup.unlit
                && (!outSetup.fur
                    || (entry->furLayerRatio == outSetup.furLayerRatio && entry->furHeight == outSetup.furHeight
                        && entry->furGravity == outSetup.furGravity));
            const bool clipKeyMatches = entry->clipValid && transformMatricesEqual(entry->clipMatrix, clipMatrix);
            try
            {
                if (worldKeyMatches && !clipKeyMatches)
                {
                    // World stage hit, clip tier miss (the camera moved):
                    // reproject the stored stage. The per-vertex expressions
                    // are the ones the full pass uses, so the values are
                    // bit-identical to a full re-transform of the same draw.
                    const float *const posData = mesh.positions.data();
                    if (outSetup.fur)
                    {
                        for (std::size_t v = 0; v < vertexCount; ++v)
                        {
                            entry->vertices[v].clip
                                = clipMatrix * glm::vec4(entry->vertices[v].worldPosition, 1.0f);
                        }
                    }
                    else
                    {
                        for (std::size_t v = 0; v < vertexCount; ++v)
                        {
                            const glm::vec4 localPos(
                                posData[v * 3], posData[v * 3 + 1], posData[v * 3 + 2], 1.0f);
                            entry->vertices[v].clip = clipMatrix * localPos;
                        }
                    }
                    entry->clipMatrix = clipMatrix;
                    entry->clipValid  = true;
                }
                else if (!worldKeyMatches)
                {
                    // World stage miss: full transform, writing the persistent
                    // entry (the original per-vertex expressions).
                    entry->vertices.resize(vertexCount);
                    const glm::mat4 &model             = outSetup.model;
                    const glm::mat4 &viewProjection    = outSetup.viewProjection;
                    const glm::mat3 &modelNormalMatrix = outSetup.modelNormalMatrix;
                    const bool       unlit             = outSetup.unlit;
                    const float     *posData           = mesh.positions.data();
                    const float     *nrmData           = mesh.normals.data();
                    const std::size_t hwWorkers     = parallelWorkerCount();
                    const bool parallelizeTransform = hwWorkers >= 2u
                        && vertexCount >= parallelTransformVertexThreshold_ && taskScheduler_ != nullptr
                        && taskScheduler_->isRunning();
                    auto transformRange = [&](std::size_t vStart, std::size_t vEnd)
                    {
                        for (std::size_t v = vStart; v < vEnd; ++v)
                        {
                            const glm::vec3 localPos3(posData[v * 3], posData[v * 3 + 1], posData[v * 3 + 2]);
                            TransformedVertex &out = entry->vertices[v];
                            if (outSetup.fur)
                            {
                                // Shell-fur displacement (fur.vert mirror): outward
                                // along the object-space normal, gravity droop in
                                // world space, clip from the RAW view-projection.
                                const FurDisplacedVertex displaced = furDisplaceVertex(localPos3,
                                    glm::make_vec3(nrmData + v * 3),
                                    outSetup.model,
                                    outSetup.viewProjectionWorld,
                                    outSetup.furLayerRatio,
                                    outSetup.furHeight,
                                    outSetup.furGravity);
                                out.clip          = displaced.clip;
                                out.worldPosition = displaced.worldPosition;
                            }
                            else
                            {
                                const glm::vec4 localPos(localPos3, 1.0f);
                                out.clip          = viewProjection * localPos;
                                out.worldPosition = glm::vec3(model * localPos);
                            }
                            if (!unlit)
                            {
                                const glm::vec3 normalRaw           = modelNormalMatrix * glm::make_vec3(nrmData + v * 3);
                                const float     normalLengthSquared = glm::dot(normalRaw, normalRaw);
                                out.worldNormal = normalLengthSquared > 0.0f && std::isfinite(normalLengthSquared)
                                                    ? normalRaw * glm::inversesqrt(normalLengthSquared)
                                                    : glm::vec3(0.0f, 1.0f, 0.0f);
                            }
                        }
                    };
                    if (parallelizeTransform)
                    {
                        // Chunk size follows the threshold but stays bounded:
                        // tiny chunks would spend the parallel win on task
                        // bookkeeping, huge ones would not split small meshes.
                        const std::uint32_t transformGrain = std::clamp(parallelTransformVertexThreshold_, 256u, 8192u);
                        taskScheduler_->parallelFor(vertexCount, transformGrain, transformRange);
                    }
                    else
                    {
                        transformRange(0, vertexCount);
                    }
                    entry->worldModel    = outSetup.model;
                    entry->worldUnlit    = outSetup.unlit;
                    entry->fur           = outSetup.fur;
                    entry->furLayerRatio = outSetup.furLayerRatio;
                    entry->furHeight     = outSetup.furHeight;
                    entry->furGravity    = outSetup.furGravity;
                    entry->worldValid    = true;
                    entry->clipMatrix    = clipMatrix;
                    entry->clipValid     = true;
                }
                outSetup.useTransformCache = true;
                outSetup.transformCache    = entry->vertices.data();
            }
            catch (const std::bad_alloc &)
            {
                entry->worldValid = false;
                entry->clipValid  = false;
                outSetup.useTransformCache = false;
                outSetup.transformCache    = nullptr;
            }
        }
    }

    outSetup.baseColorMap         = mapFor(material.baseColorTexture, neutralWhite_);
    outSetup.normalMap            = mapFor(material.normalTexture, flatNormal_);
    outSetup.metallicRoughnessMap = mapFor(material.metallicRoughnessTexture, neutralWhite_);
    outSetup.occlusionMap         = mapFor(material.occlusionTexture, neutralWhite_);
    outSetup.emissiveMap          = mapFor(material.emissiveTexture, neutralWhite_);

    outSetup.baseColorFactor         = glm::make_vec3(material.baseColor);
    outSetup.metallicFactor          = material.metallicFactor;
    outSetup.roughnessFactor         = material.roughnessFactor;
    outSetup.normalScale             = material.normalScale;
    outSetup.occlusionStrength       = material.occlusionStrength;
    outSetup.emissiveColor           = glm::make_vec3(material.emissiveColor);
    outSetup.sheenFactor             = glm::clamp(material.sheenFactor, 0.0f, 1.0f);
    outSetup.doubleSided             = material.doubleSided;
    outSetup.hasBaseColorMap         = outSetup.baseColorMap != &neutralWhite_;
    outSetup.hasNormalMap            = outSetup.normalMap != &flatNormal_;
    outSetup.hasMetallicRoughnessMap = outSetup.metallicRoughnessMap != &neutralWhite_;
    outSetup.hasEmissiveMap          = outSetup.emissiveMap != &neutralWhite_;
    outSetup.hasEmissive
        = outSetup.emissiveColor.x != 0.0f || outSetup.emissiveColor.y != 0.0f || outSetup.emissiveColor.z != 0.0f;
    outSetup.hasOcclusion
        = !outSetup.unlit && outSetup.occlusionStrength != 0.0f && outSetup.occlusionMap != &neutralWhite_;

    outSetup.fallbackTangentNormal = glm::vec3(0.0f, 0.0f, 1.0f);
    if (!outSetup.unlit && !outSetup.hasNormalMap)
    {
        const glm::vec3 normalMapSample = flatNormal_.sample(0.0f, 0.0f) * 2.0f - 1.0f;
        outSetup.fallbackTangentNormal  = glm::normalize(glm::vec3(
            normalMapSample.x * outSetup.normalScale, normalMapSample.y * outSetup.normalScale, normalMapSample.z));
    }

    outSetup.enableBackFaceCull = false;
    outSetup.frontSign          = 0.0f;
    // Double-sided draws (cloth sheets, flags, thin shells) never cull: their
    // fragments may face either way, and the pixel path flips normals toward
    // the viewer instead (the mesh.frag twin of the glTF doubleSided rule).
    if (cameraInitialized_ && !outSetup.doubleSided)
    {
        // A generator that knows its own winding convention (see
        // MeshMaterial::frontFaceWinding) pins the culling sign directly:
        // exact per-triangle culling, no heuristics. This bypasses the
        // bounds and majority-vote machinery below, whose vote cannot
        // classify rugged procedural meshes reliably. The declared sign
        // needs only vertex POSITIONS (a face normal cross product), so it
        // works for unlit backdrops too - shells and domes drawn by
        // position-only vertex colors - where the vote path (which reads
        // smooth vertex normals) cannot apply.
        if (material.frontFaceWinding > 0.0f || material.frontFaceWinding < 0.0f)
        {
            outSetup.frontSign          = material.frontFaceWinding;
            outSetup.enableBackFaceCull = true; // winding convention declared; culling is safe
        }
        else if (!outSetup.unlit && backFaceCullingEnabled_ && outSetup.useTransformCache && mesh.boundsValid
            && triangleCount >= 4)
        {
            ensureWorldBounds();
            const float cameraDist = glm::distance(cameraPosition_, outSetup.boundsWorldCenter);
            if (cameraDist > outSetup.boundsWorldRadius)
            {
                // Majority vote over a spread of triangles: face winding
                // against vertex normals identifies the outward orientation
                // camera-independently (screen-space signs cannot - front and
                // back faces balance on a closed mesh). A crease triangle
                // whose smooth corner normals lean across the edge loses the
                // vote instead of flipping the whole mesh.
                //
                // The outcome is a pure function of the mesh geometry, the
                // sampled index list and the sign of the model's linear part
                // (((M a) x (M b)) . (M c) = det(M) . ((a x b) . c)), so it is
                // memoized per mesh and re-derived only when the transform
                // mirrors or the index subset changes.
                const unsigned determinantSlot = glm::determinant(glm::mat3(outSetup.model)) < 0.0f ? 1u : 0u;
                HeadlessMesh::WindingVoteMemo &memo = mesh.windingVotes[determinantSlot];
                float frontSign = 0.0f;
                bool  memoHit   = false;
                for (std::size_t slot = 0; slot < HeadlessMesh::WindingVoteMemo::capacity; ++slot)
                {
                    if (memo.sources[slot] == activeIndices)
                    {
                        frontSign = memo.signs[slot];
                        memoHit   = true;
                        break;
                    }
                }
                if (!memoHit)
                {
                    const std::size_t sampleStride = std::max<std::size_t>(1, triangleCount / 64);
                    std::uint32_t     positive = 0, negative = 0;
                    for (std::size_t t = 0; t < triangleCount; t += sampleStride)
                    {
                        const std::uint32_t i0 = activeIndices[t * 3 + 0];
                        const std::uint32_t i1 = activeIndices[t * 3 + 1];
                        const std::uint32_t i2 = activeIndices[t * 3 + 2];
                        if (i0 >= vertexCount || i1 >= vertexCount || i2 >= vertexCount)
                            continue;
                        const TransformedVertex *const cache = outSetup.transformCache;
                        const glm::vec3 &wp0 = cache[i0].worldPosition;
                        const glm::vec3 &wp1 = cache[i1].worldPosition;
                        const glm::vec3 &wp2 = cache[i2].worldPosition;
                        const glm::vec3  faceNormal = glm::cross(wp1 - wp0, wp2 - wp0);
                        const float      fnLen2     = glm::dot(faceNormal, faceNormal);
                        if (fnLen2 < 1e-12f)
                            continue;
                        const glm::vec3 &wn0 = cache[i0].worldNormal;
                        const glm::vec3 &wn1 = cache[i1].worldNormal;
                        const glm::vec3 &wn2 = cache[i2].worldNormal;
                        const glm::vec3  avgNormal = wn0 + wn1 + wn2;
                        const float      anLen2    = glm::dot(avgNormal, avgNormal);
                        if (anLen2 < 1e-12f)
                            continue;
                        const float dotFN = glm::dot(faceNormal, avgNormal);
                        if (dotFN > 1e-6f)
                            ++positive;
                        else if (dotFN < -1e-6f)
                            ++negative;
                    }
                    if (positive > negative && positive > 0)
                        frontSign = 1.0f;
                    else if (negative > positive && negative > 0)
                        frontSign = -1.0f;
                    // Record the outcome for this index list, reusing the first
                    // free slot; when every slot is taken, slot 0 is the victim
                    // so a ninth index list still gets memoized instead of
                    // re-running the vote on every draw. A fixed victim is
                    // enough because the pipeline only ever samples the base
                    // list plus its (at most four) LOD lists per determinant
                    // slot - the eviction path exists to degrade gracefully,
                    // not because it is expected.
                    std::size_t memoSlot = HeadlessMesh::WindingVoteMemo::capacity;
                    for (std::size_t slot = 0; slot < HeadlessMesh::WindingVoteMemo::capacity; ++slot)
                    {
                        if (memo.sources[slot] == nullptr)
                        {
                            memoSlot = slot;
                            break;
                        }
                    }
                    if (memoSlot == HeadlessMesh::WindingVoteMemo::capacity)
                    {
                        memoSlot = 0;
                    }
                    memo.sources[memoSlot] = activeIndices;
                    memo.signs[memoSlot]   = frontSign;
                }
                if (frontSign != 0.0f)
                {
                    outSetup.frontSign          = frontSign;
                    outSetup.enableBackFaceCull = true; // winding convention confirmed; culling is safe
                }
            }
        }
    }
    return true;
}

// Whole-mesh single-threaded rasterization wrapper (drawMesh uses the
// parallel-split path above).
void HeadlessBackend::rasterizeMesh(const HeadlessMesh &mesh, const float *modelMatrix, const MeshMaterial &material)
{
    statistics_.trianglesSubmitted += mesh.indices.size() / 3;
    RasterSetup setup;
    if (!prepareRasterization(mesh, modelMatrix, material, mesh.indices.data(), mesh.indices.size(), setup))
    {
        return;
    }
    rasterizeTriangles(mesh, setup, 0, setup.triangleCount, false, nullptr);
}

// The rasterizer core. For each triangle: shade/transform corners (reusing
// the transform cache when allowed), clip against the near plane, project to
// screen, build a screen-space tangent frame for normal maps, then scan the
// bounding box testing barycentric coverage with edge-function increments.
// Depth and color writes go through the pixel-group spin lock when parallel.
void HeadlessBackend::rasterizeTriangles(const HeadlessMesh &mesh, const RasterSetup &setup, std::size_t triStart,
    std::size_t triEnd, bool useAtomicDepth, StripResult *outResult, std::uint32_t yBandBegin, std::uint32_t yBandEnd)
{
    if (framebufferWidth == 0 || framebufferHeight == 0)
    {
        return;
    }
    const std::size_t triangleCount = setup.triangleCount;
    if (triStart > triEnd)
        triStart = triEnd;
    if (triEnd > triangleCount)
        triEnd = triangleCount;
    if (yBandBegin >= framebufferHeight)
        return; // this band starts past the framebuffer: nothing to do
    yBandEnd = std::min(yBandEnd, framebufferHeight - 1u);

    const glm::mat4  &model              = setup.model;
    const glm::mat4  &viewProjection     = setup.viewProjection;
    const glm::mat3  &modelNormalMatrix  = setup.modelNormalMatrix;
    const bool        unlit              = setup.unlit;
    const bool        writeDepth         = setup.depthWrite;
    const bool        enableBackFaceCull = setup.enableBackFaceCull;
    const float       frontSign          = setup.frontSign;
    const bool        useTransformCache  = setup.useTransformCache;
    const std::size_t vertexCount        = setup.vertexCount;
    const TransformedVertex *const availableTransformCache = setup.transformCache;
    const float *const posData   = mesh.positions.data();
    const float *const nrmData   = mesh.normals.data();
    const float *const colorData = mesh.colors.data();
    const float *const uvData    = mesh.uvs.data();

    const HeadlessTexture *const baseColorMap            = setup.baseColorMap;
    const HeadlessTexture *const normalMap               = setup.normalMap;
    const HeadlessTexture *const metallicRoughnessMap    = setup.metallicRoughnessMap;
    const HeadlessTexture *const occlusionMap            = setup.occlusionMap;
    const HeadlessTexture *const emissiveMap             = setup.emissiveMap;
    const glm::vec3              baseColorFactor         = setup.baseColorFactor;
    const float                  metallicFactor          = setup.metallicFactor;
    const float                  roughnessFactor         = setup.roughnessFactor;
    const float                  normalScale             = setup.normalScale;
    const float                  occlusionStrength       = setup.occlusionStrength;
    const glm::vec3              emissiveColor           = setup.emissiveColor;
    const bool                   hasBaseColorMap         = setup.hasBaseColorMap;
    const bool                   hasNormalMap            = setup.hasNormalMap;
    const bool                   hasMetallicRoughnessMap = setup.hasMetallicRoughnessMap;
    const bool                   hasEmissiveMap          = setup.hasEmissiveMap;
    const bool                   hasEmissive             = setup.hasEmissive;
    const bool                   hasOcclusion            = setup.hasOcclusion;
    float *const ENGINE_RESTRICT        depthBuffer       = this->depth.data();
    std::uint8_t *const ENGINE_RESTRICT colorBuffer       = this->color.data();
    const std::uint32_t                 framebufferWidth  = this->framebufferWidth;
    const std::uint32_t                 framebufferHeight = this->framebufferHeight;

    for (std::size_t triangle = triStart; triangle < triEnd; ++triangle)
    {
        const std::uint32_t *index = setup.indexData + triangle * 3;

        struct ClipVertex
        {
            glm::vec4 clip;
            glm::vec3 worldPosition;
            glm::vec3 worldNormal;
            glm::vec3 color;
            glm::vec2 uv;
        };
        ClipVertex corners[3];
        for (std::uint32_t corner = 0; corner < 3; ++corner)
        {
            const std::uint32_t vertexIndex = index[corner];
            if (useTransformCache && vertexIndex < vertexCount)
            {
                const TransformedVertex &transformed = availableTransformCache[vertexIndex];
                corners[corner].clip                 = transformed.clip;
                corners[corner].worldPosition        = transformed.worldPosition;
                corners[corner].worldNormal          = unlit ? glm::vec3(0.0f) : transformed.worldNormal;
                // Unlit draws skip per-pixel position interpolation, so the
                // corners' world positions were dropped - except when the
                // draw declares its winding: the per-triangle cull test
                // reads them (see prepareRasterization).
                if (unlit && !setup.enableBackFaceCull)
                    corners[corner].worldPosition = glm::vec3(0.0f);
            }
            else
            {
                const glm::vec3 localPosition3(
                    posData[vertexIndex * 3], posData[vertexIndex * 3 + 1], posData[vertexIndex * 3 + 2]);
                if (setup.fur)
                {
                    // Shell-fur displacement (fur.vert mirror), same as the
                    // transform-cache path above.
                    const FurDisplacedVertex displaced = furDisplaceVertex(localPosition3,
                        glm::make_vec3(nrmData + vertexIndex * 3),
                        model,
                        setup.viewProjectionWorld,
                        setup.furLayerRatio,
                        setup.furHeight,
                        setup.furGravity);
                    corners[corner].clip          = displaced.clip;
                    corners[corner].worldPosition = displaced.worldPosition;
                }
                else
                {
                    const glm::vec4 localPosition(localPosition3, 1.0f);
                    corners[corner].clip          = viewProjection * localPosition;
                    corners[corner].worldPosition = glm::vec3(model * localPosition);
                }
                if (!unlit)
                {
                    const glm::vec3 normalRaw           = modelNormalMatrix * glm::make_vec3(nrmData + vertexIndex * 3);
                    const float     normalLengthSquared = glm::dot(normalRaw, normalRaw);
                    corners[corner].worldNormal = normalLengthSquared > 0.0f && std::isfinite(normalLengthSquared)
                                                    ? normalRaw * glm::inversesqrt(normalLengthSquared)
                                                    : glm::vec3(0.0f, 1.0f, 0.0f);
                }
                else
                {
                    corners[corner].worldNormal   = glm::vec3(0.0f);
                    if (!setup.enableBackFaceCull)
                    {
                        corners[corner].worldPosition = glm::vec3(0.0f);
                    }
                }
            }
            corners[corner].color = glm::make_vec3(colorData + vertexIndex * 3);
            corners[corner].uv    = glm::make_vec2(uvData + vertexIndex * 2);
        }

        if (enableBackFaceCull)
        {
            const glm::vec3 faceNormal = glm::cross(corners[1].worldPosition - corners[0].worldPosition,
                corners[2].worldPosition - corners[0].worldPosition);
            const glm::vec3 viewDir    = cameraPosition_ - corners[0].worldPosition;
            if (glm::dot(faceNormal, viewDir) * frontSign <= 0.0f)
            {
                continue;
            }
        }

        ClipVertex    clipped[9]; // near-clip emits up to 4; the screen-rect clip result (up to 8) is copied back
        std::uint32_t clippedCount = 0;
        // Near-plane clip: w + z >= 0 is inside. Fully-inside triangles skip
        // the Sutherland-Hodgman walk; otherwise emit inside corners and the
        // interpolated edge crossings (max 4 vertices out).
        const float dist0 = corners[0].clip.w + corners[0].clip.z;
        const float dist1 = corners[1].clip.w + corners[1].clip.z;
        const float dist2 = corners[2].clip.w + corners[2].clip.z;
        if (dist0 >= 0.0f && dist1 >= 0.0f && dist2 >= 0.0f)
        {
            clipped[0]   = corners[0];
            clipped[1]   = corners[1];
            clipped[2]   = corners[2];
            clippedCount = 3;
        }
        else
        {
            for (std::uint32_t current = 0; current < 3; ++current)
            {
                const std::uint32_t next      = (current + 1) % 3;
                const ClipVertex   &edgeStart = corners[current];
                const ClipVertex   &edgeEnd   = corners[next];

                const float startDistance = edgeStart.clip.w + edgeStart.clip.z;
                const float endDistance   = edgeEnd.clip.w + edgeEnd.clip.z;
                const bool  startInside   = startDistance >= 0.0f;
                const bool  endInside     = endDistance >= 0.0f;
                if (startInside)
                {
                    clipped[clippedCount++] = edgeStart;
                }
                if (startInside != endInside)
                {
                    const float t          = startDistance / (startDistance - endDistance);
                    ClipVertex &crossing   = clipped[clippedCount++];
                    crossing.clip          = glm::mix(edgeStart.clip, edgeEnd.clip, t);
                    crossing.worldPosition = glm::mix(edgeStart.worldPosition, edgeEnd.worldPosition, t);
                    crossing.worldNormal   = glm::mix(edgeStart.worldNormal, edgeEnd.worldNormal, t);
                    crossing.color         = glm::mix(edgeStart.color, edgeEnd.color, t);
                    crossing.uv            = glm::mix(edgeStart.uv, edgeEnd.uv, t);
                }
            }
        }
        if (clippedCount < 3)
        {
            continue;
        }

        glm::vec3 screen[9];
        float     depth[9];
        float     clipW[9];
        const auto projectPolygon = [&screen, &depth, &clipW, framebufferWidth, framebufferHeight](const ClipVertex *polygon, std::uint32_t vertexCount)
        {
            for (std::uint32_t vertex = 0; vertex < vertexCount; ++vertex)
            {
                clipW[vertex]        = polygon[vertex].clip.w;
                const float inverseW = 1.0f / clipW[vertex];
                const float ndcX     = polygon[vertex].clip.x * inverseW;
                const float ndcY     = polygon[vertex].clip.y * inverseW;
                const float ndcZ     = polygon[vertex].clip.z * inverseW;
                screen[vertex]       = glm::vec3((ndcX * 0.5f + 0.5f) * static_cast<float>(framebufferWidth),
                    (1.0f - (ndcY * 0.5f + 0.5f)) * static_cast<float>(framebufferHeight),
                    0.0f);
                depth[vertex]        = ndcZ * 0.5f + 0.5f;
            }
        };
        projectPolygon(clipped, clippedCount);
        // Guard-band screen-rect clip: a near-plane crossing projects through
        // a w a hair above the near plane and lands at astronomical screen
        // coordinates, so the fan's clamped scan window becomes the whole
        // framebuffer for a sliver triangle (an eye-centered dome's horizon
        // ring measured as full-screen scans per degenerate fan). When any
        // projected vertex escapes the screen, clip the polygon against the
        // framebuffer rectangle in clip space; the removed region is
        // off-screen, so every surviving pixel interpolates identically.
        {
            bool insideScreen = true;
            const float maxScreenX = static_cast<float>(framebufferWidth) - 1.0f;
            const float maxScreenY = static_cast<float>(framebufferHeight) - 1.0f;
            for (std::uint32_t vertex = 0; vertex < clippedCount && insideScreen; ++vertex)
            {
                insideScreen = screen[vertex].x >= 0.0f && screen[vertex].x <= maxScreenX && screen[vertex].y >= 0.0f
                    && screen[vertex].y <= maxScreenY;
            }
            if (!insideScreen)
            {
                ClipVertex         rectScratchA[9];
                ClipVertex         rectScratchB[9];
                const ClipVertex  *rectPolygon = nullptr;
                const std::uint32_t rectCount
                    = clipPolygonToFramebuffer(clipped, clippedCount, rectScratchA, rectScratchB, rectPolygon);
                if (rectCount < 3)
                {
                    continue;
                }
                clippedCount = rectCount;
                projectPolygon(rectPolygon, clippedCount);
                for (std::uint32_t vertex = 0; vertex < clippedCount; ++vertex)
                {
                    clipped[vertex] = rectPolygon[vertex];
                }
            }
        }

        glm::vec3 tangentVector(1.0f, 0.0f, 0.0f);
        glm::vec3 bitangentVector(0.0f, 1.0f, 0.0f);
        float     tangentInverseMax = 1.0f;
        if (!unlit)
        {
            const glm::vec3 edge1         = corners[1].worldPosition - corners[0].worldPosition;
            const glm::vec3 edge2         = corners[2].worldPosition - corners[0].worldPosition;
            const glm::vec2 uvDelta1      = corners[1].uv - corners[0].uv;
            const glm::vec2 uvDelta2      = corners[2].uv - corners[0].uv;
            const float     uvDeterminant = uvDelta1.x * uvDelta2.y - uvDelta2.x * uvDelta1.y;
            if (std::fabs(uvDeterminant) > 1e-12f)
            {
                const float uvScale = 1.0f / uvDeterminant;
                tangentVector       = (edge1 * uvDelta2.y - edge2 * uvDelta1.y) * uvScale;
                bitangentVector     = (edge2 * uvDelta1.x - edge1 * uvDelta2.x) * uvScale;
            }
            const float maximumTangentLengthSquared
                = std::max(glm::dot(tangentVector, tangentVector), glm::dot(bitangentVector, bitangentVector));
            tangentInverseMax = 1.0f / (std::sqrt(maximumTangentLengthSquared) + 1e-12f);
        }

        bool rasterized = false;
        for (std::uint32_t fan = 1; fan + 1 < clippedCount; ++fan)
        {
            const std::uint32_t cornerA = 0;
            const std::uint32_t cornerB = fan;
            const std::uint32_t cornerC = fan + 1;

            const float area = (screen[cornerB].x - screen[cornerA].x) * (screen[cornerC].y - screen[cornerA].y)
                             - (screen[cornerC].x - screen[cornerA].x) * (screen[cornerB].y - screen[cornerA].y);
            if (std::fabs(area) < 1e-9f)
            {
                continue;
            }
            const float inverseArea = 1.0f / area;

            const float minX = std::max(0.0f, std::min({screen[cornerA].x, screen[cornerB].x, screen[cornerC].x}));
            const float maxX = std::min(static_cast<float>(framebufferWidth) - 1.0f,
                std::max({screen[cornerA].x, screen[cornerB].x, screen[cornerC].x}));
            const float minY = std::max(0.0f, std::min({screen[cornerA].y, screen[cornerB].y, screen[cornerC].y}));
            const float maxY = std::min(static_cast<float>(framebufferHeight) - 1.0f,
                std::max({screen[cornerA].y, screen[cornerB].y, screen[cornerC].y}));
            if (minX > maxX || minY > maxY)
            {
                continue;
            }

            const std::int32_t firstX = static_cast<std::int32_t>(minX);
            const std::int32_t lastX  = static_cast<std::int32_t>(maxX);
            // Band restriction: clamp the row range to this worker's band.
            const std::int32_t firstY = std::max<std::int32_t>(static_cast<std::int32_t>(minY), static_cast<std::int32_t>(yBandBegin));
            const std::int32_t lastY  = std::min<std::int32_t>(static_cast<std::int32_t>(maxY), static_cast<std::int32_t>(yBandEnd));
            if (firstX > lastX || firstY > lastY)
            {
                continue;
            }

            const float inverseWA = 1.0f / clipW[cornerA];
            const float inverseWB = 1.0f / clipW[cornerB];
            const float inverseWC = 1.0f / clipW[cornerC];

            // UV gradients for mip selection (same derivation as
            // TileBinRecord's; see HeadlessTexture::sampleTrilinear).
            const float gradAx = -(screen[cornerC].y - screen[cornerB].y) * inverseArea;
            const float gradAy = (screen[cornerC].x - screen[cornerB].x) * inverseArea;
            const float gradBx = -(screen[cornerA].y - screen[cornerC].y) * inverseArea;
            const float gradBy = (screen[cornerA].x - screen[cornerC].x) * inverseArea;
            const float gradCx = -(gradAx + gradBx);
            const float gradCy = -(gradAy + gradBy);
            const float uvNumeratorX0 = gradAx * inverseWA * clipped[cornerA].uv.x
                + gradBx * inverseWB * clipped[cornerB].uv.x + gradCx * inverseWC * clipped[cornerC].uv.x;
            const float uvNumeratorX1 = gradAx * inverseWA * clipped[cornerA].uv.y
                + gradBx * inverseWB * clipped[cornerB].uv.y + gradCx * inverseWC * clipped[cornerC].uv.y;
            const float uvNumeratorY0 = gradAy * inverseWA * clipped[cornerA].uv.x
                + gradBy * inverseWB * clipped[cornerB].uv.x + gradCy * inverseWC * clipped[cornerC].uv.x;
            const float uvNumeratorY1 = gradAy * inverseWA * clipped[cornerA].uv.y
                + gradBy * inverseWB * clipped[cornerB].uv.y + gradCy * inverseWC * clipped[cornerC].uv.y;
            const float uvDenominatorX = gradAx * inverseWA + gradBx * inverseWB + gradCx * inverseWC;
            const float uvDenominatorY = gradAy * inverseWA + gradBy * inverseWB + gradCy * inverseWC;

            const float weightADx = -(screen[cornerC].y - screen[cornerB].y) * inverseArea;
            const float weightBDx = -(screen[cornerA].y - screen[cornerC].y) * inverseArea;
            const float sampleX   = static_cast<float>(firstX) + 0.5f;
            const float depthADx
                = weightADx * depth[cornerA] + weightBDx * depth[cornerB] - (weightADx + weightBDx) * depth[cornerC];

            for (std::int32_t y = firstY; y <= lastY; ++y)
            {
                const float sampleY  = static_cast<float>(y) + 0.5f;
                const float offsetAX = sampleX - screen[cornerB].x;
                const float offsetAY = sampleY - screen[cornerB].y;
                const float offsetBX = sampleX - screen[cornerC].x;
                const float offsetBY = sampleY - screen[cornerC].y;
                float       weightA  = ((screen[cornerC].x - screen[cornerB].x) * offsetAY
                                           - (screen[cornerC].y - screen[cornerB].y) * offsetAX)
                                     * inverseArea;
                float       weightB  = ((screen[cornerA].x - screen[cornerC].x) * offsetBY
                                           - (screen[cornerA].y - screen[cornerC].y) * offsetBX)
                                     * inverseArea;
                float       pixelDepth
                    = weightA * depth[cornerA] + weightB * depth[cornerB] + (1.0f - weightA - weightB) * depth[cornerC];
                std::size_t pixelIndex = static_cast<std::size_t>(y) * framebufferWidth + firstX;

                for (std::int32_t x = firstX; x <= lastX; ++x, ++pixelIndex)
                {
#if defined(__GNUC__) || defined(__clang__)
                    __builtin_prefetch(&depthBuffer[pixelIndex + 8], 0, 0);
#endif
                    const float weightC = 1.0f - weightA - weightB;
                    if (weightA >= 0.0f && weightB >= 0.0f && weightC >= 0.0f)
                    {
                        const float currentDepth = depthBuffer[pixelIndex];
                        if (pixelDepth >= currentDepth)
                        {
                            // Behind geometry: advance the edge functions now and
                            // skip shading (covered pixels advance at the loop tail).
                            weightA += weightADx;
                            weightB += weightBDx;
                            pixelDepth += depthADx;
                            continue;
                        }

                        // Perspective-correct attributes: interpolate 1/w-weighted
                        // varyings, then divide by the interpolated 1/w.
                        const float inverseWSum = weightA * inverseWA + weightB * inverseWB + weightC * inverseWC;
                        const float inversePerspective = 1.0f / inverseWSum;
                        const float correctedA         = weightA * inverseWA * inversePerspective;
                        const float correctedB         = weightB * inverseWB * inversePerspective;
                        const float correctedC         = 1.0f - correctedA - correctedB;

                        const glm::vec2 interpolatedUv    = correctedA * clipped[cornerA].uv
                                                          + correctedB * clipped[cornerB].uv
                                                          + correctedC * clipped[cornerC].uv;
                        // Per-pixel UV derivatives (quotient rule over the
                        // affine numerator/denominator; see sampleTrilinear).
                        glm::vec2 uvDerivativeX(0.0f);
                        glm::vec2 uvDerivativeY(0.0f);
                        if (hasBaseColorMap || hasNormalMap || hasMetallicRoughnessMap || hasOcclusion || hasEmissiveMap)
                        {
                            uvDerivativeX = glm::vec2(uvNumeratorX0 - interpolatedUv.x * uvDenominatorX,
                                uvNumeratorX1 - interpolatedUv.y * uvDenominatorX) * inversePerspective;
                            uvDerivativeY = glm::vec2(uvNumeratorY0 - interpolatedUv.x * uvDenominatorY,
                                uvNumeratorY1 - interpolatedUv.y * uvDenominatorY) * inversePerspective;
                        }
                        const glm::vec3 interpolatedColor = correctedA * clipped[cornerA].color
                                                          + correctedB * clipped[cornerB].color
                                                          + correctedC * clipped[cornerC].color;
                        glm::vec3       baseColor         = interpolatedColor * baseColorFactor;
                        if (hasBaseColorMap)
                        {
                            baseColor *= baseColorMap->sampleTrilinear(interpolatedUv.x, interpolatedUv.y, uvDerivativeX, uvDerivativeY);
                        }

                        // Shell-fur strand crop (fur.frag mirror): the layer
                        // alpha first - a fully transparent texel skips the
                        // shading exactly like the shader's discard. field =
                        // the fur texture's alpha (strand height); the neutral
                        // stand-in reports 1 (a fur-less field stays opaque).
                        float furPixelAlpha = 1.0f;
                        // The shaded pixel's reflection-field response flag
                        // (transmissive commits read it; see the transparent
                        // commit below). Reset per pixel because the shading
                        // site only sets it when the field sample returned a
                        // traced response.
                        bool bandFieldTraced = false;
                        if (setup.fur)
                        {
                            // Jittered strand field (furStrandField <->
                            // fur.frag furFieldJitter): the per-layer hash
                            // breaks the shell-band artifact up into noise.
                            const float strandField = hasBaseColorMap
                                ? furStrandField(baseColorMap->sampleAlpha(interpolatedUv.x, interpolatedUv.y),
                                    interpolatedUv,
                                    setup.furLayerIndex)
                                : 1.0f;
                            furPixelAlpha = furAlpha(strandField,
                                setup.furLayerRatio,
                                setup.furDensityBoost,
                                setup.furOuterFade);
                            if (furPixelAlpha <= 0.0039f)
                            {
                                weightA += weightADx;
                                weightB += weightBDx;
                                pixelDepth += depthADx;
                                continue;
                            }
                        }

                        glm::vec3 shaded = baseColor;
                        if (!unlit)
                        {
                            const glm::vec3 interpolatedPosition = correctedA * clipped[cornerA].worldPosition
                                                                 + correctedB * clipped[cornerB].worldPosition
                                                                 + correctedC * clipped[cornerC].worldPosition;
                            const glm::vec3 interpolatedNormal   = correctedA * clipped[cornerA].worldNormal
                                                                 + correctedB * clipped[cornerB].worldNormal
                                                                 + correctedC * clipped[cornerC].worldNormal;
                            const glm::vec3 geometryNormal       = glm::normalize(interpolatedNormal);

                            if (setup.fur)
                            {
                                // Shell-fur shading (furShade <-> fur.frag):
                                // half-Lambert diffuse + Kajiya-Kay specular
                                // against the strand tangent (= the shell
                                // normal), hemispheric ambient, direct lights.
                                // No normal map, no occlusion map, no lumen_gi
                                // probe walk - same perf contract as fur.frag.
                                const float strandShade = glm::mix(0.60f, 1.05f, setup.furLayerRatio);
                                shaded = furShade(geometryNormal,
                                    interpolatedPosition,
                                    cameraPosition_,
                                    lightDirection_,
                                    sunRadiance_,
                                    ambientSky_,
                                    ambientGround_,
                                    baseColor,
                                    strandShade,
                                    directLights_.data(),
                                    static_cast<std::uint32_t>(directLights_.size()));
                            }
                            else
                            {
                            // Without a normal map the surface normal IS the
                            // interpolated geometry normal: skip the TBN
                            // construction and its per-pixel normalize (the
                            // common case for colored/vertex-lit parts).
                            glm::vec3 normal = geometryNormal;
                            // Two-sided draws (cloth sheets, flags): flip the
                            // normal toward the viewer on back-facing
                            // fragments before the TBN is built - the mesh.frag
                            // twin of the glTF doubleSided rule (back-face
                            // normals are reversed before lighting).
                            if (setup.doubleSided)
                            {
                                const glm::vec3 viewForFlip = glm::normalize(cameraPosition_ - interpolatedPosition);
                                if (glm::dot(normal, viewForFlip) < 0.0f)
                                {
                                    normal = -normal;
                                }
                            }
                            if (hasNormalMap)
                            {
                                const glm::vec3 normalMapSample
                                    = normalMap->sampleTrilinear(interpolatedUv.x, interpolatedUv.y, uvDerivativeX, uvDerivativeY) * 2.0f - 1.0f;
                                const glm::vec3 tangentNormal = glm::normalize(glm::vec3(normalMapSample.x * normalScale,
                                    normalMapSample.y * normalScale,
                                    normalMapSample.z));
                                // The (possibly flipped) oriented normal is the
                                // TBN's third axis, exactly like mesh.frag builds
                                // its tangent frame from the flipped geometry
                                // normal on two-sided draws.
                                const glm::mat3 tbn(
                                    tangentVector * tangentInverseMax, bitangentVector * tangentInverseMax,
                                    normal);
                                normal = glm::normalize(tbn * tangentNormal);
                            }

                            float metallic  = metallicFactor;
                            float roughness = roughnessFactor;
                            if (hasMetallicRoughnessMap)
                            {
                                const glm::vec3 metallicRoughnessSample
                                    = metallicRoughnessMap->sampleTrilinear(interpolatedUv.x, interpolatedUv.y, uvDerivativeX, uvDerivativeY);
                                metallic *= metallicRoughnessSample.b;
                                roughness *= metallicRoughnessSample.g;
                            }
                            metallic  = std::clamp(metallic, 0.0f, 1.0f);
                            roughness = std::clamp(roughness, minimumRoughness, 1.0f);
                            const float occlusion
                                = hasOcclusion
                                    ? 1.0f
                                          - occlusionStrength
                                                * (1.0f - occlusionMap->sampleTrilinear(interpolatedUv.x, interpolatedUv.y, uvDerivativeX, uvDerivativeY).r)
                                    : 1.0f;
                            shaded = pbrShade(normal,
                                interpolatedPosition,
                                cameraPosition_,
                                lightDirection_,
                                sunRadiance_,
                                ambientSky_,
                                ambientGround_,
                                baseColor,
                                metallic,
                                roughness,
                                occlusion,
                                directLights_.data(),
                                static_cast<std::uint32_t>(directLights_.size()),
                                preparedProbeGrid_.valid ? &preparedProbeGrid_ : nullptr,
                                setup.opacity,
                                setup.sheenFactor);
                            // Indirect specular (lumen_reflection field hook):
                            // the field texel this pixel projects into carries
                            // the expected GGX-lobe response of its material,
                            // so the add is the whole composite (linear space,
                            // before tone mapping - see shadeTrianglePixel's
                            // twin of this block).
                            if (preparedReflectionField_.valid)
                            {
                                shaded += sampleReflectionField(
                                    preparedReflectionField_, interpolatedPosition, roughness, &bandFieldTraced);
                            }
                            }
                        }

                        if (hasEmissive)
                        {
                            if (hasEmissiveMap)
                            {
                                shaded += emissiveColor * emissiveMap->sampleTrilinear(interpolatedUv.x, interpolatedUv.y, uvDerivativeX, uvDerivativeY);
                            }
                            else
                            {
                                shaded += emissiveColor;
                            }
                        }
                        // Output transform: tone map here; the commit paths
                        // encode to bytes (see encodeColorRgba; the CPU
                        // raster always writes UNORM targets; see
                        // SRGB_OUTPUT in mesh.frag for the GPU-side switch).
                        shaded = toneMapColor(shaded, environment_.exposure, environment_.toneMapping);

                        if (setup.fur)
                        {
                            // Shell-fur commit: alpha blend AFTER tone map +
                            // gamma encode - the same space the fur pipeline's
                            // fixed-function blending operates in on the UNORM
                            // target - and NO depth write (transparent layers
                            // must not occlude each other or later geometry).
                            const glm::vec3 encoded = fastSrgbEncode(glm::max(shaded, glm::vec3(0.0f)));
                            std::uint8_t *pixel  = colorBuffer + pixelIndex * 4;
                            const float   oneMinus = 1.0f - furPixelAlpha;
                            pixel[0] = toByte(encoded.x * furPixelAlpha + static_cast<float>(pixel[0]) / 255.0f * oneMinus);
                            pixel[1] = toByte(encoded.y * furPixelAlpha + static_cast<float>(pixel[1]) / 255.0f * oneMinus);
                            pixel[2] = toByte(encoded.z * furPixelAlpha + static_cast<float>(pixel[2]) / 255.0f * oneMinus);
                            pixel[3] = 255;
                            rasterized = true;
                            weightA += weightADx;
                            weightB += weightBDx;
                            pixelDepth += depthADx;
                            continue;
                        }

                        if (setup.transparent)
                        {
                            std::uint8_t *pixel = colorBuffer + pixelIndex * 4;
                            if (bandFieldTraced)
                            {
                                // Transmissive commit: the field's response
                                // carries the interface's reflected share and
                                // the refracted transmitted share, so the
                                // shaded result commits directly (blending the
                                // background in too would double it).
                                *reinterpret_cast<std::uint32_t *>(pixel) = encodeColorRgba(shaded);
                            }
                            else
                            {
                                // No traced response (outside the traced
                                // roughness band, or no reflection module):
                                // the classic encoded-space alpha blend, with
                                // the material's opacity as the blend factor
                                // (the depth write is already off through the
                                // setup; the pre-shade depth test above still
                                // applied, so opaque geometry in front keeps
                                // occluding).
                                const glm::vec3 encoded = fastSrgbEncode(glm::max(shaded, glm::vec3(0.0f)));
                                const float   oneMinus = 1.0f - setup.opacity;
                                pixel[0] = toByte(encoded.x * setup.opacity + static_cast<float>(pixel[0]) / 255.0f * oneMinus);
                                pixel[1] = toByte(encoded.y * setup.opacity + static_cast<float>(pixel[1]) / 255.0f * oneMinus);
                                pixel[2] = toByte(encoded.z * setup.opacity + static_cast<float>(pixel[2]) / 255.0f * oneMinus);
                                pixel[3] = 255;
                            }
                            rasterized = true;
                            weightA += weightADx;
                            weightB += weightBDx;
                            pixelDepth += depthADx;
                            continue;
                        }

                        // Opaque commit: one packed word (the composed
                        // sRGB+quantize table; identical output to the byte
                        // stores it replaces).
                        const std::uint32_t packed = encodeColorRgba(shaded);
                        if (useAtomicDepth)
                        {
                            // Parallel path: one spin lock per pixel group keeps
                            // depth-test/write pairs from interleaving.
                            const std::size_t             lockIndex = pixelIndex / pixelsPerLockGroup_;
                            std::atomic_ref<std::uint8_t> lockRef(pixelLocks[lockIndex]);
                            std::uint8_t                  expected = 0;
                            while (!lockRef.compare_exchange_weak(
                                expected, 1, std::memory_order_acquire, std::memory_order_relaxed))
                            {
                                expected = 0;
                            }

                            if (pixelDepth < depthBuffer[pixelIndex])
                            {
                                if (writeDepth)
                                {
                                    depthBuffer[pixelIndex] = pixelDepth;
                                }
                                std::atomic_ref<std::uint32_t> pixelWord(
                                    *reinterpret_cast<std::uint32_t *>(colorBuffer + pixelIndex * 4));
                                pixelWord.store(packed, std::memory_order_relaxed);
                                rasterized = true;
                            }
                            lockRef.store(0, std::memory_order_release);
                        }
                        else
                        {
                            if (pixelDepth < depthBuffer[pixelIndex])
                            {
                                if (writeDepth)
                                {
                                    depthBuffer[pixelIndex] = pixelDepth;
                                }
                                *reinterpret_cast<std::uint32_t *>(colorBuffer + pixelIndex * 4) = packed;
                                rasterized = true;
                            }
                        }
                    }

                    weightA += weightADx;
                    weightB += weightBDx;
                    pixelDepth += depthADx;
                }
            }
        }
        if (rasterized)
        {
            if (outResult != nullptr)
            {
                ++outResult->trianglesRasterized;
            }
            else
            {
                ++statistics_.trianglesRasterized;
            }
        }
    }
}

// Shared geometry pipeline. For one draw triangle: shade/transform corners
// (reusing the transform cache when allowed), cull back faces, clip against
// the near plane, project to screen, build the screen-space tangent frame
// for normal maps, and emit 0..2 rasterisable fan triangles (near-plane
// clipping can turn one triangle into a two-triangle fan; degenerate fans
// are dropped here, exactly where the previous single-loop core skipped
// them). The sequential path and the tile binning pass run this verbatim,
// so both rasterisers see identical geometry.
//
// Every hot input is value-copied into locals BEFORE any store to the
// escaping outFans array: stores through outFans may alias the setup, mesh
// and transform-cache reads as far as TBAA is concerned, so reads left in
// memory would be re-loaded after every corner store (measured 5x
// regression of the geometry half). The previous single-loop core had no
// escaped stores in the geometry half at all and got this for free.
std::size_t HeadlessBackend::prepareFanTriangles(
    const HeadlessMesh &mesh, const RasterSetup &setup, std::size_t triangleIndex, TileBinRecord (&outFans)[6]) const
{
    // Inputs (value copies - registers, not aliasable memory).
    const glm::mat4   model             = setup.model;
    const glm::mat4   viewProjection    = setup.viewProjection;
    const glm::mat3   modelNormalMatrix = setup.modelNormalMatrix;
    const bool        unlit             = setup.unlit;
    const bool        useTransformCache = setup.useTransformCache;
    const std::size_t vertexCount       = setup.vertexCount;
    const bool        enableBackFaceCull = setup.enableBackFaceCull;
    const float       frontSign          = setup.frontSign;
    const glm::vec3   cameraPosition     = cameraPosition_;
    const std::uint32_t framebufferWidthLocal  = framebufferWidth;  // this-> read once
    const std::uint32_t framebufferHeightLocal = framebufferHeight;
    const std::uint32_t *const indexData = setup.indexData;
    const float *const  posData          = mesh.positions.data();
    const float *const  nrmData          = mesh.normals.data();
    const float *const  colorData        = mesh.colors.data();
    const float *const  uvData           = mesh.uvs.data();
    // Cache data pointer hoisted with the rest (the cache is read-only here;
    // the per-mesh entry whose vertices this draw transforms through).
    const TransformedVertex *const transformCache = setup.transformCache;

    const std::uint32_t *index = indexData + triangleIndex * 3;

    RasterVertex corners[3]; // local, never escapes: scalarised into registers
    for (std::uint32_t corner = 0; corner < 3; ++corner)
    {
        const std::uint32_t vertexIndex = index[corner];
        if (useTransformCache && vertexIndex < vertexCount)
        {
            const TransformedVertex &transformed = transformCache[vertexIndex];
            corners[corner].clip                 = transformed.clip;
            corners[corner].worldPosition        = transformed.worldPosition;
            corners[corner].worldNormal          = unlit ? glm::vec3(0.0f) : transformed.worldNormal;
            // Keep world positions for declared-winding unlit draws (the
            // per-triangle cull test reads them; see rasterizeTriangles).
            if (unlit && !enableBackFaceCull)
                corners[corner].worldPosition = glm::vec3(0.0f);
        }
        else
        {
            const glm::vec4 localPosition(
                posData[vertexIndex * 3], posData[vertexIndex * 3 + 1], posData[vertexIndex * 3 + 2], 1.0f);
            corners[corner].clip          = viewProjection * localPosition;
            corners[corner].worldPosition = glm::vec3(model * localPosition);
            if (!unlit)
            {
                const glm::vec3 normalRaw           = modelNormalMatrix * glm::make_vec3(nrmData + vertexIndex * 3);
                const float     normalLengthSquared = glm::dot(normalRaw, normalRaw);
                corners[corner].worldNormal = normalLengthSquared > 0.0f && std::isfinite(normalLengthSquared)
                                                ? normalRaw * glm::inversesqrt(normalLengthSquared)
                                                : glm::vec3(0.0f, 1.0f, 0.0f);
            }
            else
            {
                corners[corner].worldNormal   = glm::vec3(0.0f);
                if (!enableBackFaceCull)
                {
                    corners[corner].worldPosition = glm::vec3(0.0f);
                }
            }
        }
        corners[corner].color = glm::make_vec3(colorData + vertexIndex * 3);
        corners[corner].uv    = glm::make_vec2(uvData + vertexIndex * 2);
    }

    if (enableBackFaceCull)
    {
        const glm::vec3 faceNormal = glm::cross(corners[1].worldPosition - corners[0].worldPosition,
            corners[2].worldPosition - corners[0].worldPosition);
        const glm::vec3 viewDir    = cameraPosition - corners[0].worldPosition;
        if (glm::dot(faceNormal, viewDir) * frontSign <= 0.0f)
        {
            return 0;
        }
    }

    RasterVertex  clipped[9]; // near-clip emits up to 4; the screen-rect clip result (up to 8) is copied back
    std::uint32_t clippedCount = 0;
    // Near-plane clip: w + z >= 0 is inside. Fully-inside triangles skip
    // the Sutherland-Hodgman walk; otherwise emit inside corners and the
    // interpolated edge crossings (max 4 vertices out).
    const float dist0 = corners[0].clip.w + corners[0].clip.z;
    const float dist1 = corners[1].clip.w + corners[1].clip.z;
    const float dist2 = corners[2].clip.w + corners[2].clip.z;
    if (dist0 >= 0.0f && dist1 >= 0.0f && dist2 >= 0.0f)
    {
        clipped[0]   = corners[0];
        clipped[1]   = corners[1];
        clipped[2]   = corners[2];
        clippedCount = 3;
    }
    else
    {
        for (std::uint32_t current = 0; current < 3; ++current)
        {
            const std::uint32_t next      = (current + 1) % 3;
            const RasterVertex &edgeStart = corners[current];
            const RasterVertex &edgeEnd   = corners[next];

            const float startDistance = edgeStart.clip.w + edgeStart.clip.z;
            const float endDistance   = edgeEnd.clip.w + edgeEnd.clip.z;
            const bool  startInside   = startDistance >= 0.0f;
            const bool  endInside     = endDistance >= 0.0f;
            if (startInside)
            {
                clipped[clippedCount++] = edgeStart;
            }
            if (startInside != endInside)
            {
                const float t          = startDistance / (startDistance - endDistance);
                RasterVertex &crossing = clipped[clippedCount++];
                crossing.clip          = glm::mix(edgeStart.clip, edgeEnd.clip, t);
                crossing.worldPosition = glm::mix(edgeStart.worldPosition, edgeEnd.worldPosition, t);
                crossing.worldNormal   = glm::mix(edgeStart.worldNormal, edgeEnd.worldNormal, t);
                crossing.color         = glm::mix(edgeStart.color, edgeEnd.color, t);
                crossing.uv            = glm::mix(edgeStart.uv, edgeEnd.uv, t);
            }
        }
    }
    if (clippedCount < 3)
    {
        return 0;
    }

    float screenX[9];
    float screenY[9];
    float depth[9];
    float clipW[9];
    float inverseW[9];
    const auto projectPolygon = [&](const RasterVertex *polygon, std::uint32_t vertexCount)
    {
        for (std::uint32_t vertex = 0; vertex < vertexCount; ++vertex)
        {
            clipW[vertex]    = polygon[vertex].clip.w;
            inverseW[vertex] = 1.0f / clipW[vertex];
            const float ndcX = polygon[vertex].clip.x * inverseW[vertex];
            const float ndcY = polygon[vertex].clip.y * inverseW[vertex];
            const float ndcZ = polygon[vertex].clip.z * inverseW[vertex];
            screenX[vertex]  = (ndcX * 0.5f + 0.5f) * static_cast<float>(framebufferWidthLocal);
            screenY[vertex]  = (1.0f - (ndcY * 0.5f + 0.5f)) * static_cast<float>(framebufferHeightLocal);
            depth[vertex]    = ndcZ * 0.5f + 0.5f;
        }
    };
    projectPolygon(clipped, clippedCount);
    // Guard-band screen-rect clip (see rasterizeTriangles): a near-plane
    // crossing projects to astronomical screen coordinates and the fan's
    // clamped pixel window becomes the whole framebuffer for a sliver
    // triangle. Clipping against the framebuffer rectangle in clip space
    // keeps every record's window tight; off-screen pixels were never
    // shaded, so output is unchanged.
    {
        bool insideScreen = true;
        const float maxScreenX = static_cast<float>(framebufferWidthLocal) - 1.0f;
        const float maxScreenY = static_cast<float>(framebufferHeightLocal) - 1.0f;
        for (std::uint32_t vertex = 0; vertex < clippedCount && insideScreen; ++vertex)
        {
            insideScreen = screenX[vertex] >= 0.0f && screenX[vertex] <= maxScreenX && screenY[vertex] >= 0.0f
                && screenY[vertex] <= maxScreenY;
        }
        if (!insideScreen)
        {
            RasterVertex         rectScratchA[9];
            RasterVertex         rectScratchB[9];
            const RasterVertex  *rectPolygon = nullptr;
            const std::uint32_t  rectCount
                = clipPolygonToFramebuffer(clipped, clippedCount, rectScratchA, rectScratchB, rectPolygon);
            if (rectCount < 3)
            {
                return 0;
            }
            clippedCount = rectCount;
            projectPolygon(rectPolygon, clippedCount);
            for (std::uint32_t vertex = 0; vertex < clippedCount; ++vertex)
            {
                clipped[vertex] = rectPolygon[vertex];
            }
        }
    }

    glm::vec3 tangentVector(1.0f, 0.0f, 0.0f);
    glm::vec3 bitangentVector(0.0f, 1.0f, 0.0f);
    float     tangentInverseMax = 1.0f;
    if (!unlit)
    {
        const glm::vec3 edge1         = corners[1].worldPosition - corners[0].worldPosition;
        const glm::vec3 edge2         = corners[2].worldPosition - corners[0].worldPosition;
        const glm::vec2 uvDelta1      = corners[1].uv - corners[0].uv;
        const glm::vec2 uvDelta2      = corners[2].uv - corners[0].uv;
        const float     uvDeterminant = uvDelta1.x * uvDelta2.y - uvDelta2.x * uvDelta1.y;
        if (std::fabs(uvDeterminant) > 1e-12f)
        {
            const float uvScale = 1.0f / uvDeterminant;
            tangentVector       = (edge1 * uvDelta2.y - edge2 * uvDelta1.y) * uvScale;
            bitangentVector     = (edge2 * uvDelta1.x - edge1 * uvDelta2.x) * uvScale;
        }
        const float maximumTangentLengthSquared
            = std::max(glm::dot(tangentVector, tangentVector), glm::dot(bitangentVector, bitangentVector));
        tangentInverseMax = 1.0f / (std::sqrt(maximumTangentLengthSquared) + 1e-12f);
    }

    std::size_t fanCount = 0;
    for (std::uint32_t fan = 1; fan + 1 < clippedCount; ++fan)
    {
        const std::uint32_t cornerA = 0;
        const std::uint32_t cornerB = fan;
        const std::uint32_t cornerC = fan + 1;

        const float area = (screenX[cornerB] - screenX[cornerA]) * (screenY[cornerC] - screenY[cornerA])
                         - (screenX[cornerC] - screenX[cornerA]) * (screenY[cornerB] - screenY[cornerA]);
        if (std::fabs(area) < 1e-9f)
        {
            continue;
        }

        TileBinRecord     &record      = outFans[fanCount++];
        const std::uint32_t sources[3] = {cornerA, cornerB, cornerC};
        record.corners[0]              = clipped[cornerA];
        record.corners[1]              = clipped[cornerB];
        record.corners[2]              = clipped[cornerC];
        for (std::uint32_t corner = 0; corner < 3; ++corner)
        {
            record.screenX[corner]      = screenX[sources[corner]];
            record.screenY[corner]      = screenY[sources[corner]];
            record.depth[corner]        = depth[sources[corner]];
            record.inverseClipW[corner] = inverseW[sources[corner]];
        }
        record.tangent           = tangentVector;
        record.bitangent         = bitangentVector;
        record.tangentInverseMax = tangentInverseMax;
        // UV gradients for mip selection (see TileBinRecord): the barycentric
        // weights are affine over the screen triangle, so the 1/w-weighted UV
        // numerator P and denominator S are affine too - only their gradients
        // are needed per pixel (the quotient rule turns them into dUv/dx and
        // dUv/dy).
        {
            const float invArea = 1.0f / area;
            const float gradAx = -(record.screenY[2] - record.screenY[1]) * invArea;
            const float gradAy = (record.screenX[2] - record.screenX[1]) * invArea;
            const float gradBx = -(record.screenY[0] - record.screenY[2]) * invArea;
            const float gradBy = (record.screenX[0] - record.screenX[2]) * invArea;
            const float gradCx = -(gradAx + gradBx);
            const float gradCy = -(gradAy + gradBy);
            const float wA = record.inverseClipW[0], wB = record.inverseClipW[1], wC = record.inverseClipW[2];
            record.uvNumeratorX[0] = gradAx * wA * record.corners[0].uv.x + gradBx * wB * record.corners[1].uv.x
                + gradCx * wC * record.corners[2].uv.x;
            record.uvNumeratorX[1] = gradAx * wA * record.corners[0].uv.y + gradBx * wB * record.corners[1].uv.y
                + gradCx * wC * record.corners[2].uv.y;
            record.uvNumeratorY[0] = gradAy * wA * record.corners[0].uv.x + gradBy * wB * record.corners[1].uv.x
                + gradCy * wC * record.corners[2].uv.x;
            record.uvNumeratorY[1] = gradAy * wA * record.corners[0].uv.y + gradBy * wB * record.corners[1].uv.y
                + gradCy * wC * record.corners[2].uv.y;
            record.uvDenominatorX = gradAx * wA + gradBx * wB + gradCx * wC;
            record.uvDenominatorY = gradAy * wA + gradBy * wB + gradCy * wC;
        }
        record.sourceTriangle    = static_cast<std::uint32_t>(triangleIndex);
    }
    return fanCount;
}

// Integer pixel window a fan record can write, clamped to the framebuffer
// (same clamp order as the previous single-loop core: clamp the float box,
// then truncate to integers). Shared by the rasterizer core and the tile
// binning pass so the bins always cover exactly the pixels the core may
// touch.
void HeadlessBackend::fanPixelWindow(const TileBinRecord &record, std::uint32_t framebufferWidth,
    std::uint32_t framebufferHeight, std::int32_t &firstX, std::int32_t &lastX, std::int32_t &firstY,
    std::int32_t &lastY)
{
    const float minX = std::max(0.0f, std::min({record.screenX[0], record.screenX[1], record.screenX[2]}));
    const float maxX = std::min(static_cast<float>(framebufferWidth) - 1.0f,
        std::max({record.screenX[0], record.screenX[1], record.screenX[2]}));
    const float minY = std::max(0.0f, std::min({record.screenY[0], record.screenY[1], record.screenY[2]}));
    const float maxY = std::min(static_cast<float>(framebufferHeight) - 1.0f,
        std::max({record.screenY[0], record.screenY[1], record.screenY[2]}));
    firstX = static_cast<std::int32_t>(minX);
    lastX  = static_cast<std::int32_t>(maxX);
    firstY = static_cast<std::int32_t>(minY);
    lastY  = static_cast<std::int32_t>(maxY);
}

// Shared shading kernel: perspective-correct attribute interpolation
// (1/w-weighted varyings, divided by the interpolated 1/w), material map
// sampling, PBR lighting, emissive, then tone map + manual gamma (the CPU
// raster always writes UNORM targets; see SRGB_OUTPUT in mesh.frag for the
// GPU-side switch). Pure function of the record, the weights and the draw
// setup - no framebuffer access, safe to call concurrently.
//
// Setup inputs are value-copied into locals first: inlined into the pixel
// loop, these become registers hoisted out of the x-loop (the depth/colour
// stores are restrict-qualified, so nothing inside the loop can clobber
// them). Reading setup fields directly would re-load them per pixel.
glm::vec3 HeadlessBackend::shadeTrianglePixel(const RasterSetup &setup, const TileBinRecord &record, float weightA,
    float weightB, float inverseWA, float inverseWB, float inverseWC, bool *outFieldTraced,
    const GiAmbientSample *giSample) const
{
    const RasterVertex &cornerA = record.corners[0];
    const RasterVertex &cornerB = record.corners[1];
    const RasterVertex &cornerC = record.corners[2];

    // Setup inputs (value copies; see function comment).
    const bool        unlit             = setup.unlit;
    const bool        hasBaseColorMap         = setup.hasBaseColorMap;
    const HeadlessTexture *baseColorMap       = setup.baseColorMap;
    const glm::vec3   baseColorFactor         = setup.baseColorFactor;
    const bool        hasNormalMap            = setup.hasNormalMap;
    const HeadlessTexture *normalMap          = setup.normalMap;
    const float       normalScale             = setup.normalScale;
    const bool        hasMetallicRoughnessMap = setup.hasMetallicRoughnessMap;
    const HeadlessTexture *metallicRoughnessMap = setup.metallicRoughnessMap;
    const float       metallicFactor          = setup.metallicFactor;
    const float       roughnessFactor         = setup.roughnessFactor;
    const bool        hasOcclusion            = setup.hasOcclusion;
    const float       occlusionStrength       = setup.occlusionStrength;
    const HeadlessTexture *occlusionMap       = setup.occlusionMap;
    const bool        hasEmissive             = setup.hasEmissive;
    const bool        hasEmissiveMap          = setup.hasEmissiveMap;
    const glm::vec3   emissiveColor           = setup.emissiveColor;
    const HeadlessTexture *emissiveMap        = setup.emissiveMap;
    const glm::vec3   cameraPosition          = cameraPosition_;
    const glm::vec3   lightDirection          = lightDirection_;
    const glm::vec3   sunRadiance             = sunRadiance_;
    const glm::vec3   ambientSky              = ambientSky_;
    const glm::vec3   ambientGround           = ambientGround_;
    const PreparedDirectLight *const lights   = directLights_.data();
    const std::uint32_t            lightCount = static_cast<std::uint32_t>(directLights_.size());
    const glm::vec3   tangentScaled    = record.tangent * record.tangentInverseMax;
    const glm::vec3   bitangentScaled  = record.bitangent * record.tangentInverseMax;

    const float weightC            = 1.0f - weightA - weightB;
    const float inverseWSum        = weightA * inverseWA + weightB * inverseWB + weightC * inverseWC;
    const float inversePerspective = 1.0f / inverseWSum;
    const float correctedA         = weightA * inverseWA * inversePerspective;
    const float correctedB         = weightB * inverseWB * inversePerspective;
    const float correctedC         = 1.0f - correctedA - correctedB;

    const glm::vec2 interpolatedUv    = correctedA * cornerA.uv + correctedB * cornerB.uv + correctedC * cornerC.uv;
    // Per-pixel UV derivatives from the record's affine gradients (see
    // TileBinRecord): the quotient rule over P/S yields dUv/dx and dUv/dy at
    // this pixel, which the mip-selecting sampler turns into a level.
    glm::vec2 uvDerivativeX(0.0f);
    glm::vec2 uvDerivativeY(0.0f);
    if (hasBaseColorMap || hasNormalMap || hasMetallicRoughnessMap || hasOcclusion || hasEmissiveMap)
    {
        const float invS = inversePerspective;
        uvDerivativeX = glm::vec2(record.uvNumeratorX[0] - interpolatedUv.x * record.uvDenominatorX,
            record.uvNumeratorX[1] - interpolatedUv.y * record.uvDenominatorX) * invS;
        uvDerivativeY = glm::vec2(record.uvNumeratorY[0] - interpolatedUv.x * record.uvDenominatorY,
            record.uvNumeratorY[1] - interpolatedUv.y * record.uvDenominatorY) * invS;
    }
    const glm::vec3 interpolatedColor = correctedA * cornerA.color + correctedB * cornerB.color
                                      + correctedC * cornerC.color;
    glm::vec3       baseColor         = interpolatedColor * baseColorFactor;
    if (hasBaseColorMap)
    {
        baseColor *= baseColorMap->sampleTrilinear(interpolatedUv.x, interpolatedUv.y, uvDerivativeX, uvDerivativeY);
    }

    glm::vec3 shaded = baseColor;
    if (!unlit)
    {
        const glm::vec3 interpolatedPosition = correctedA * cornerA.worldPosition + correctedB * cornerB.worldPosition
                                             + correctedC * cornerC.worldPosition;
        const glm::vec3 interpolatedNormal   = correctedA * cornerA.worldNormal + correctedB * cornerB.worldNormal
                                             + correctedC * cornerC.worldNormal;
        const glm::vec3 geometryNormal       = glm::normalize(interpolatedNormal);

        // Without a normal map the surface normal IS the interpolated
        // geometry normal: skip the TBN construction and its per-pixel
        // normalize (the common case for colored/vertex-lit parts).
        glm::vec3 normal = geometryNormal;
        // Two-sided draws (cloth sheets, flags): flip the normal toward the
        // viewer on back-facing fragments before the TBN is built - the
        // mesh.frag twin of the glTF doubleSided rule.
        if (setup.doubleSided)
        {
            const glm::vec3 viewForFlip = glm::normalize(cameraPosition - interpolatedPosition);
            if (glm::dot(normal, viewForFlip) < 0.0f)
            {
                normal = -normal;
            }
        }
        if (hasNormalMap)
        {
            const glm::vec3 normalMapSample = normalMap->sampleTrilinear(interpolatedUv.x, interpolatedUv.y, uvDerivativeX, uvDerivativeY) * 2.0f - 1.0f;
            const glm::vec3 tangentNormal = glm::normalize(glm::vec3(normalMapSample.x * normalScale,
                normalMapSample.y * normalScale,
                normalMapSample.z));
            // The (possibly flipped) oriented normal is the TBN's third
            // axis, matching mesh.frag's flipped-geometry-normal frame.
            const glm::mat3 tbn(tangentScaled, bitangentScaled, normal);
            normal = glm::normalize(tbn * tangentNormal);
        }

        float metallic  = metallicFactor;
        float roughness = roughnessFactor;
        if (hasMetallicRoughnessMap)
        {
            const glm::vec3 metallicRoughnessSample = metallicRoughnessMap->sampleTrilinear(interpolatedUv.x, interpolatedUv.y, uvDerivativeX, uvDerivativeY);
            metallic *= metallicRoughnessSample.b;
            roughness *= metallicRoughnessSample.g;
        }
        metallic  = std::clamp(metallic, 0.0f, 1.0f);
        roughness = std::clamp(roughness, minimumRoughness, 1.0f);
        const float occlusion
            = hasOcclusion
                  ? 1.0f
                        - occlusionStrength * (1.0f - occlusionMap->sampleTrilinear(interpolatedUv.x, interpolatedUv.y, uvDerivativeX, uvDerivativeY).r)
                  : 1.0f;
        shaded = pbrShade(normal,
            interpolatedPosition,
            cameraPosition,
            lightDirection,
            sunRadiance,
            ambientSky,
            ambientGround,
            baseColor,
            metallic,
            roughness,
            occlusion,
            lights,
            lightCount,
            giSample == nullptr && preparedProbeGrid_.valid ? &preparedProbeGrid_ : nullptr,
            setup.opacity,
            setup.sheenFactor,
            giSample);

        // Indirect specular (lumen_reflection field hook): the field texel
        // this pixel projects into carries the expected GGX-lobe response of
        // its material, so the add is the whole composite. Linear space, per
        // the module's estimator contract; tone mapping happens below.
        if (preparedReflectionField_.valid)
        {
            bool fieldTraced = false;
            shaded += sampleReflectionField(preparedReflectionField_, interpolatedPosition, roughness, &fieldTraced);
            if (outFieldTraced != nullptr)
            {
                *outFieldTraced = fieldTraced;
            }
        }
        else if (outFieldTraced != nullptr)
        {
            *outFieldTraced = false;
        }
    }

    if (hasEmissive)
    {
        if (hasEmissiveMap)
        {
            shaded += emissiveColor * emissiveMap->sampleTrilinear(interpolatedUv.x, interpolatedUv.y, uvDerivativeX, uvDerivativeY);
        }
        else
        {
            shaded += emissiveColor;
        }
    }
    // Tone map only: the commit paths fold the sRGB encode and the byte
    // quantization into one table (encodeColorRgba); the shell-fur path
    // blends in encoded space and applies fastSrgbEncode itself.
    return toneMapColor(shaded, environment_.exposure, environment_.toneMapping);
}

// Shared rasterizer core. Scans the fan record's bounding box clamped to the
// inclusive pixel window [xBandBegin..xBandEnd] x [yBandBegin..yBandEnd],
// testing barycentric coverage with edge-function increments, depth-testing,
// shading through shadeTrianglePixel, and writing depth + colour. The
// sequential path passes the whole framebuffer; tile owners pass their tile
// (exclusive ownership means the writes need no locks). Returns whether at
// least one pixel was written.
bool HeadlessBackend::rasterizeFanRecord(const RasterSetup &setup, const TileBinRecord &record,
    std::int32_t xBandBegin, std::int32_t xBandEnd, std::int32_t yBandBegin, std::int32_t yBandEnd,
    GiAmortizationCache *giAmortization)
{
    if (framebufferWidth == 0 || framebufferHeight == 0)
    {
        return false;
    }

    std::int32_t firstX = 0;
    std::int32_t lastX  = 0;
    std::int32_t firstY = 0;
    std::int32_t lastY  = 0;
    fanPixelWindow(record, framebufferWidth, framebufferHeight, firstX, lastX, firstY, lastY);
    if (firstX > lastX || firstY > lastY)
    {
        return false;
    }
    // Window restriction (tile owners pass a tile; the sequential path passes
    // the whole framebuffer, which leaves the window unchanged because
    // fanPixelWindow already clamped it to the framebuffer).
    firstX = std::max<std::int32_t>(firstX, xBandBegin);
    lastX  = std::min<std::int32_t>(lastX, xBandEnd);
    firstY = std::max<std::int32_t>(firstY, yBandBegin);
    lastY  = std::min<std::int32_t>(lastY, yBandEnd);
    if (firstX > lastX || firstY > lastY)
    {
        return false;
    }

    // Record fields are value-copied into locals before the scan loops: the
    // depth/colour stores below are restrict-qualified, so these become
    // registers (reading record.* directly would re-load per row/pixel).
    // Constant indices only, so the arrays scalarise.
    const float screenXA = record.screenX[0];
    const float screenXB = record.screenX[1];
    const float screenXC = record.screenX[2];
    const float screenYA = record.screenY[0];
    const float screenYB = record.screenY[1];
    const float screenYC = record.screenY[2];
    const float depthA   = record.depth[0];
    const float depthB   = record.depth[1];
    const float depthC   = record.depth[2];
    const float inverseWA = record.inverseClipW[0];
    const float inverseWB = record.inverseClipW[1];
    const float inverseWC = record.inverseClipW[2];
    const bool  writeDepth = setup.depthWrite;

    // Same area expression prepareFanTriangles used, on the same stored
    // screen coordinates: bit-identical value, so |area| >= 1e-9 holds here.
    const float area = (screenXB - screenXA) * (screenYC - screenYA)
                     - (screenXC - screenXA) * (screenYB - screenYA);
    const float inverseArea = 1.0f / area;

    float *const ENGINE_RESTRICT        depthBuffer = this->depth.data();
    std::uint8_t *const ENGINE_RESTRICT colorBuffer = this->color.data();

    const float weightADx = -(screenYC - screenYB) * inverseArea;
    const float weightBDx = -(screenYA - screenYC) * inverseArea;
    const float sampleX   = static_cast<float>(firstX) + 0.5f;
    const float depthADx  = weightADx * depthA + weightBDx * depthB - (weightADx + weightBDx) * depthC;

    bool wrote = false;
    // The shaded pixel's reflection-field response flag (transmissive
    // commits read it; see the transparent commit below).
    bool fieldTraced = false;
    // Probe-grid amortization tiers (see GiAmortizationCache): only draws
    // whose shading normal IS the interpolated geometry normal can share a
    // cached sample (a normal map perturbs it per pixel, a two-sided draw
    // flips it on back-facing fragments; unlit and fur draws never consult
    // the probe grid). The coarse block tier first, the quad tier next.
    const bool giQuadEligible = giAmortization != nullptr && preparedProbeGrid_.valid && !setup.unlit && !setup.fur
        && !setup.hasNormalMap && !setup.doubleSided;
    GiQuadCache::Entry *const blockEntries
        = giQuadEligible && !giAmortization->blocks.entries.empty() ? giAmortization->blocks.entries.data() : nullptr;
    const std::uint32_t blockStride = giAmortization != nullptr ? giAmortization->blocks.stride : 0u;
    GiQuadCache::Entry *const quadEntries
        = giQuadEligible && !giAmortization->quads.entries.empty() ? giAmortization->quads.entries.data() : nullptr;
    const std::uint32_t quadStride = giAmortization != nullptr ? giAmortization->quads.stride : 0u;
    for (std::int32_t y = firstY; y <= lastY; ++y)
    {
        const float sampleY  = static_cast<float>(y) + 0.5f;
        const float offsetAX = sampleX - screenXB;
        const float offsetAY = sampleY - screenYB;
        const float offsetBX = sampleX - screenXC;
        const float offsetBY = sampleY - screenYC;
        float       weightA  = ((screenXC - screenXB) * offsetAY - (screenYC - screenYB) * offsetAX) * inverseArea;
        float       weightB  = ((screenXA - screenXC) * offsetBY - (screenYA - screenYC) * offsetBX) * inverseArea;
        float       pixelDepth = weightA * depthA + weightB * depthB + (1.0f - weightA - weightB) * depthC;
        std::size_t pixelIndex = static_cast<std::size_t>(y) * framebufferWidth + firstX;

        for (std::int32_t x = firstX; x <= lastX; ++x, ++pixelIndex)
        {
#if defined(__GNUC__) || defined(__clang__)
            __builtin_prefetch(&depthBuffer[pixelIndex + 8], 0, 0);
#endif
            const float weightC = 1.0f - weightA - weightB;
            if (weightA >= 0.0f && weightB >= 0.0f && weightC >= 0.0f)
            {
                const float currentDepth = depthBuffer[pixelIndex];
                if (pixelDepth >= currentDepth)
                {
                    // Behind geometry: advance the edge functions now and
                    // skip shading (covered pixels advance at the loop tail).
                    weightA += weightADx;
                    weightB += weightBDx;
                    pixelDepth += depthADx;
                    continue;
                }

                // Perspective-correct attributes are interpolated inside the
                // shading kernel, which returns the tone-mapped linear color;
                // the commit folds the sRGB encode and quantization into one
                // packed word. The pre-shade depth test above is the early-out;
                // the commit reuses it directly: within one record scan (and
                // one tile) the calling thread is the only depth writer, so the
                // pixel's depth still holds the value that test read.
                //
                // Tiered probe-grid amortization: interpolate this pixel's
                // surface pair (the same perspective-correct math the shading
                // kernel runs) and reuse the coarse 4x4 block's cached walk
                // when its gates pass, else the 2x2 quad's, else evaluate the
                // walk and publish it to the tier that missed it. The gates -
                // same plane (the normal projection of the position delta
                // stays small) and a near-identical normal - are what the
                // walk's own inputs vary smoothly over, so a passing
                // neighbor returns the sample the per-pixel walk would have
                // produced to within the gates' tolerance.
                GiAmbientSample quadSample;
                const GiAmbientSample *giSample = nullptr;
                if (quadEntries != nullptr)
                {
                    const float inverseWSum = weightA * inverseWA + weightB * inverseWB + weightC * inverseWC;
                    const float inversePerspective = 1.0f / inverseWSum;
                    const float correctedA         = weightA * inverseWA * inversePerspective;
                    const float correctedB         = weightB * inverseWB * inversePerspective;
                    const float correctedC         = 1.0f - correctedA - correctedB;
                    const RasterVertex &cornerA    = record.corners[0];
                    const RasterVertex &cornerB    = record.corners[1];
                    const RasterVertex &cornerC    = record.corners[2];
                    const glm::vec3 pixelPosition  = correctedA * cornerA.worldPosition
                                                    + correctedB * cornerB.worldPosition
                                                    + correctedC * cornerC.worldPosition;
                    const glm::vec3 pixelNormalRaw = correctedA * cornerA.worldNormal
                                                    + correctedB * cornerB.worldNormal
                                                    + correctedC * cornerC.worldNormal;
                    const float     pixelNormalLength = glm::length(pixelNormalRaw);
                    const glm::vec3 pixelNormal = pixelNormalLength > 1.0e-8f ? pixelNormalRaw / pixelNormalLength
                                                                          : glm::vec3(0.0f, 1.0f, 0.0f);
                    GiQuadCache::Entry &blockEntry
                        = blockEntries[static_cast<std::size_t>((y - yBandBegin) >> 2) * blockStride
                                      + static_cast<std::size_t>((x - xBandBegin) >> 2)];
                    const glm::vec3 blockPositionDelta = blockEntry.position - pixelPosition;
                    if (blockEntry.valid && glm::dot(pixelNormal, blockEntry.normal) >= 0.995f
                        && std::fabs(glm::dot(pixelNormal, blockPositionDelta)) <= 0.05f)
                    {
                        quadSample.irradiance = blockEntry.irradiance;
                        quadSample.coverage   = blockEntry.coverage;
                        quadSample.valid      = true;
                    }
                    else
                    {
                        GiQuadCache::Entry &entry
                            = quadEntries[static_cast<std::size_t>((y - yBandBegin) >> 1) * quadStride
                                          + static_cast<std::size_t>((x - xBandBegin) >> 1)];
                        const glm::vec3 positionDelta = entry.position - pixelPosition;
                        if (entry.valid && glm::dot(pixelNormal, entry.normal) >= 0.995f
                            && std::fabs(glm::dot(pixelNormal, positionDelta)) <= 0.05f)
                        {
                            quadSample.irradiance = entry.irradiance;
                            quadSample.coverage   = entry.coverage;
                            quadSample.valid      = true;
                        }
                        else
                        {
                            quadSample.valid = giProbeGridIrradiance(
                                preparedProbeGrid_, pixelPosition, pixelNormal, quadSample.irradiance, quadSample.coverage);
                            entry.position   = pixelPosition;
                            entry.normal     = pixelNormal;
                            entry.irradiance = quadSample.irradiance;
                            entry.coverage   = quadSample.coverage;
                            entry.valid      = true;
                        }
                        // A rejected coarse block still learns the walk for
                        // its remaining pixels (curved regions converge back
                        // toward block sharing as the block's pixels agree).
                        blockEntry.position   = pixelPosition;
                        blockEntry.normal     = pixelNormal;
                        blockEntry.irradiance = quadSample.irradiance;
                        blockEntry.coverage   = quadSample.coverage;
                        blockEntry.valid      = true;
                    }
                    giSample = &quadSample;
                }
                const glm::vec3 shaded
                    = shadeTrianglePixel(setup, record, weightA, weightB, inverseWA, inverseWB, inverseWC,
                        &fieldTraced, giSample);

                if (setup.transparent)
                {
                    std::uint8_t *pixel = colorBuffer + pixelIndex * 4;
                    if (fieldTraced)
                    {
                        // Transmissive commit: the reflection field's response
                        // at this pixel already contains what the surface
                        // shows of the scene behind it (the reflected share
                        // off the interface plus the transmitted share
                        // refracted through it), so the shaded result commits
                        // directly - blending the raw background in as well
                        // would show the scene behind twice. Depth writes stay
                        // off: co-transmissive surfaces stack through the
                        // field's chains, not through occlusion.
                        *reinterpret_cast<std::uint32_t *>(pixel) = encodeColorRgba(shaded);
                    }
                    else
                    {
                        // No traced response here (the surface is outside the
                        // traced roughness band, or the reflection module is
                        // absent): the classic alpha blend over the
                        // framebuffer. Tile ownership keeps the per-pixel
                        // order equal to the submission order, so
                        // co-transparent surfaces stack far-to-near exactly as
                        // the sequential path blends them.
                        const glm::vec3 encoded = fastSrgbEncode(glm::max(shaded, glm::vec3(0.0f)));
                        const float     oneMinus = 1.0f - setup.opacity;
                        pixel[0] = toByte(encoded.x * setup.opacity + static_cast<float>(pixel[0]) / 255.0f * oneMinus);
                        pixel[1] = toByte(encoded.y * setup.opacity + static_cast<float>(pixel[1]) / 255.0f * oneMinus);
                        pixel[2] = toByte(encoded.z * setup.opacity + static_cast<float>(pixel[2]) / 255.0f * oneMinus);
                        pixel[3] = 255;
                    }
                    wrote = true;
                }
                else
                {
                    if (writeDepth)
                    {
                        depthBuffer[pixelIndex] = pixelDepth;
                    }
                    *reinterpret_cast<std::uint32_t *>(colorBuffer + pixelIndex * 4) = encodeColorRgba(shaded);
                    wrote                                                            = true;
                }
            }

            weightA += weightADx;
            weightB += weightBDx;
            pixelDepth += depthADx;
        }
    }
    return wrote;
}


// Tile-parallel path, phase 1: run the shared geometry pipeline over every
// triangle (identical to the sequential path), then bin the fan records into
// 64x64 screen tiles with a two-pass count + scatter. Per-tile lists keep
// ascending record order, so the per-pixel triangle order inside a tile
// equals the sequential path's - equal-depth writes resolve identically.
bool HeadlessBackend::buildTileRecords(const HeadlessMesh &mesh, const RasterSetup &setup)
{
    if (framebufferWidth == 0 || framebufferHeight == 0)
    {
        return false;
    }
    const std::size_t triangleCount = setup.triangleCount;
    if (triangleCount == 0)
    {
        return false;
    }
    // Worst case every triangle clips into a six-triangle fan (near-plane
    // clip followed by the screen-rect clip). Draws that could exceed the
    // record cap fall back to the sequential path instead of growing the
    // persistent scratch without bound.
    if (triangleCount > maxTileRecords_ / 6u)
    {
        return false;
    }

    try
    {
        tileRecords_.clear();
        tileRecords_.reserve(triangleCount * 2u);
        TileBinRecord fans[6]; // reused across triangles; see rasterizeTriangles
        for (std::size_t triangle = 0; triangle < triangleCount; ++triangle)
        {
            const std::size_t fanCount = prepareFanTriangles(mesh, setup, triangle, fans);
            for (std::size_t fan = 0; fan < fanCount; ++fan)
            {
                // Single-draw tile paths rasterize against a one-element
                // setup array; every record belongs to that one draw.
                fans[fan].setupIndex = 0u;
                tileRecords_.push_back(fans[fan]);
            }
        }
    }
    catch (const std::bad_alloc &)
    {
        tileRecords_.clear();
        tileItems_.clear();
        tileOffsets_.clear();
        return false;
    }

    return binTileRecords();
}

// Batched tile-parallel path, phase 1: the batch's records are already in
// tileRecords_ (drawMeshBatch emitted them per item, stamped with each
// draw's setup index); only the shared binning remains.
bool HeadlessBackend::buildBatchTileRecords()
{
    return binTileRecords();
}

// Shared phase-1 tail: two-pass binning of the current tileRecords_ into the
// 64x64 tile grid. Returns false on an empty grid or an allocation failure
// (callers fall back or replay the records sequentially).
bool HeadlessBackend::binTileRecords()
{
    const tilebins::TileGrid grid = tilebins::makeGrid(framebufferWidth, framebufferHeight, tileWidth_, tileHeight_);
    if (tilebins::tileCount(grid) == 0)
    {
        return false;
    }

    const bool binned = tilebins::binRecordTileRanges(tileRecords_.size(),
        [this, &grid](std::size_t record, std::uint32_t &tileXBegin, std::uint32_t &tileXEnd,
            std::uint32_t &tileYBegin, std::uint32_t &tileYEnd)
        {
            std::int32_t firstX = 0;
            std::int32_t lastX  = 0;
            std::int32_t firstY = 0;
            std::int32_t lastY  = 0;
            fanPixelWindow(tileRecords_[record], framebufferWidth, framebufferHeight, firstX, lastX, firstY, lastY);
            return tilebins::tileRangeForPixelRect(
                grid, firstX, lastX, firstY, lastY, tileXBegin, tileXEnd, tileYBegin, tileYEnd);
        },
        grid, tileOffsets_, tileItems_);
    if (!binned)
    {
        tileOffsets_.clear();
        tileItems_.clear();
        return false;
    }
    return true;
}

// Tile-parallel path, phase 2: rasterise exactly the tiles this worker owns
// (index % workerCount == workerIndex). Exclusive tile ownership means the
// depth/colour writes need no locks; each tile's working set is cache-local.
// Each record resolves its RasterSetup through record.setupIndex (the
// single-draw paths pass a one-element array; the batch passes one setup per
// item).
void HeadlessBackend::rasterizeTileRecords(
    const RasterSetup *setups, std::uint32_t workerIndex, std::uint32_t workerCount, StripResult *outResult)
{
    if (outResult == nullptr || workerCount == 0u || setups == nullptr)
    {
        return;
    }
    const tilebins::TileGrid grid  = tilebins::makeGrid(framebufferWidth, framebufferHeight, tileWidth_, tileHeight_);
    const std::uint32_t      tiles = tilebins::tileCount(grid);
    if (tiles == 0u || tileOffsets_.size() != static_cast<std::size_t>(tiles) + 1u)
    {
        return; // no binned draw; the framebuffer cannot change between phases (main-thread only)
    }

    const std::int32_t framebufferWidthInt  = static_cast<std::int32_t>(framebufferWidth);
    const std::int32_t framebufferHeightInt = static_cast<std::int32_t>(framebufferHeight);
    std::uint64_t      written              = 0;
    for (std::uint32_t tile = 0; tile < tiles; ++tile)
    {
        // Checkerboard ownership ((column + row) % workerCount): a strided
        // walk over the flat tile index would hand each worker every
        // Nth COLUMN, and coverage-heavy bands (a planet's disk, a cloud
        // deck) sit in contiguous row ranges - the checkerboard interleaves
        // any contiguous region across every worker instead.
        const std::uint32_t tileX = tile % grid.tilesX;
        const std::uint32_t tileY = tile / grid.tilesX;
        if ((tileX + tileY) % workerCount != workerIndex)
        {
            continue;
        }
        const std::uint32_t begin = tileOffsets_[tile];
        const std::uint32_t end   = tileOffsets_[tile + 1u];
        if (begin == end)
        {
            continue;
        }
        const std::int32_t  xBandBegin = static_cast<std::int32_t>(tileX * tileWidth_);
        const std::int32_t  xBandEnd = static_cast<std::int32_t>(
            std::min<std::int64_t>(framebufferWidthInt, static_cast<std::int64_t>(xBandBegin) + tileWidth_) - 1);
        const std::int32_t yBandBegin = static_cast<std::int32_t>(tileY * tileHeight_);
        const std::int32_t yBandEnd = static_cast<std::int32_t>(
            std::min<std::int64_t>(framebufferHeightInt, static_cast<std::int64_t>(yBandBegin) + tileHeight_) - 1);
        // One probe-grid amortization cache per worker thread, reset per
        // owned tile: the amortization is only valid within a tile's pixels
        // (see GiAmortizationCache), and concurrent tile owners must never
        // share entries. The thread-local keeps its capacity across tiles
        // and frames, so a steady-state tile costs one allocation-free clear.
        static thread_local GiAmortizationCache amortization;
        if (preparedProbeGrid_.valid)
        {
            amortization.resetForTile(tileWidth_, tileHeight_);
        }
        for (std::uint32_t entry = begin; entry < end; ++entry)
        {
            const TileBinRecord &record = tileRecords_[tileItems_[entry]];
            if (rasterizeFanRecord(setups[record.setupIndex], record, xBandBegin, xBandEnd, yBandBegin, yBandEnd,
                    preparedProbeGrid_.valid ? &amortization : nullptr))
            {
                // Counted per (record, tile): a fan record spanning several
                // tiles owned by different workers is counted once per tile
                // it wrote in - the same span semantics the previous band
                // split used for multi-band triangles.
                ++written;
            }
        }
    }
    outResult->trianglesRasterized += written;
}

} // namespace renderer
} // namespace engine
