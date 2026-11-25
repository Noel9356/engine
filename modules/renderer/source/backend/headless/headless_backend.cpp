// Headless backend implementation: a multithreaded CPU rasterizer.
//
// Pipeline per draw: prepareRasterization (culling, transform cache, texture
// resolution) then rasterizeTriangles (Sutherland-Hodgman-style near clip,
// scanline rasterization with perspective-correct interpolation, PBR shading
// via pbr_shading.hpp, tile-locked depth updates when parallel). Frames are
// blitted into the window module's locked surface pixels.

#include "headless_backend.hpp"
#include "quad_shading.hpp"
#include "render_shared.hpp"
#include "tile_bins.hpp"

#include <engine/direct_lighting/capabilities.hpp>
#include <engine/kernel/task_scheduler.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/material.hpp>
#include <engine/renderer/mesh.hpp>
#include <engine/window/capabilities.hpp>

#if defined(__SSSE3__) || defined(__AVX2__) || defined(__SSE2__) || defined(__x86_64__) || defined(_M_X64) \
    || defined(_M_AMD64)
#include <immintrin.h>
#endif

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

    // Shadow casters accumulate per frame (see sun_shadows.hpp): the list
    // starts empty so the endFrame build hashes exactly what this frame
    // drew, and the capacity survives for allocation-free steady states.
    sunShadowCasters_.clear();
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
        const auto resolveRows = [&](std::uint32_t rowBegin, std::uint32_t rowEnd, std::uint32_t bandIndex)
        {
            // Tent resolve (renderer.headless_resolve_tent, the default):
            // the sample-block sums run through the separable bilinear tent
            // [1 2 1; 2 4 2; 1 2 1] / 16 across the 3x3 block neighborhood.
            // A box resolve reproduces the sample grid's own hard steps (a
            // straight silhouette landing between sample columns keeps a
            // one-texel jump, and shallow diagonals quantize into two-texel
            // terraces the film blend cannot fully smooth); the tent spreads
            // every silhouette over a two-texel ramp the way a sensor's
            // optical low-pass filter does, before the film blend sees it.
            // Each band works on its own scratch slice (bands run
            // concurrently).
            if (resolveTent_)
            {
                resolveRowsTent(color.data(), framebufferWidth, pixels, outputColumns, outputRows, rowBegin,
                    rowEnd, tentRowSums_.data() + static_cast<std::size_t>(bandIndex) * outputColumns * 4u * 6u);
                return;
            }
            // 2x2 supersampling (the default): a widened-add fast path computes
            // the same rounding average the generic loop below computes - the
            // per-sample bytes widen to 16-bit lanes exactly, the sums stay
            // exact (max 4 * 255 + half fits 16 bits), and the +half >> shift
            // rounding is the same arithmetic - while covering two output
            // pixels per SIMD pass. _mm_shuffle_epi8 needs SSSE3; builds
            // without it keep the generic loop.
#if defined(__SSSE3__) || defined(__AVX2__)
            const bool resolve22 = sampleScaleX_ == 2u && sampleScaleY_ == 2u && sampleCountIsPowerOfTwo
                && outputColumns >= 2u;
            if (resolve22)
            {
                // Byte masks: the 8-byte swap adds each sample pair's matching
                // channels (output pixel's sums land in lanes 0-3), and the
                // final gather picks the four packed bytes of each output
                // pixel out of the pack result.
                const __m128i swapGroups = _mm_setr_epi8(8, 9, 10, 11, 12, 13, 14, 15, 0, 1, 2, 3, 4, 5, 6, 7);
                const __m128i gatherBytes = _mm_setr_epi8(0, 1, 2, 3, 8, 9, 10, 11, 0, 0, 0, 0, 0, 0, 0, 0);
                const __m128i zero = _mm_setzero_si128();
                const __m128i halfWord = _mm_set1_epi16(static_cast<short>(1u << (sampleShift - 1u)));
                for (std::uint32_t y = rowBegin; y < rowEnd; ++y)
                {
                    std::uint8_t *destination = pixels.rgba + static_cast<std::size_t>(y) * pixels.strideBytes;
                    const std::uint8_t *rowA = color.data()
                        + static_cast<std::size_t>(y * 2u) * framebufferWidth * 4u;
                    const std::uint8_t *rowB = rowA + static_cast<std::size_t>(framebufferWidth) * 4u;
                    std::uint32_t x = 0;
                    for (; x + 2u <= outputColumns; x += 2u)
                    {
                        const __m128i samplesA = _mm_loadu_si128(reinterpret_cast<const __m128i *>(rowA + x * 8u));
                        const __m128i samplesB = _mm_loadu_si128(reinterpret_cast<const __m128i *>(rowB + x * 8u));
                        // Per-byte row sums (bytes widen to 16-bit lanes).
                        const __m128i sumLo = _mm_add_epi16(
                            _mm_unpacklo_epi8(samplesA, zero), _mm_unpacklo_epi8(samplesB, zero));
                        const __m128i sumHi = _mm_add_epi16(
                            _mm_unpackhi_epi8(samplesA, zero), _mm_unpackhi_epi8(samplesB, zero));
                        // Sample-pair sums: each output pixel's four samples
                        // are two 4-byte groups; the swap adds group to group.
                        const __m128i pairsLo = _mm_add_epi16(sumLo, _mm_shuffle_epi8(sumLo, swapGroups));
                        const __m128i pairsHi = _mm_add_epi16(sumHi, _mm_shuffle_epi8(sumHi, swapGroups));
                        // Rounding shift, then pack each lane's low byte.
                        const __m128i roundedLo = _mm_srli_epi16(_mm_add_epi16(pairsLo, halfWord), sampleShift);
                        const __m128i roundedHi = _mm_srli_epi16(_mm_add_epi16(pairsHi, halfWord), sampleShift);
                        const __m128i packed = _mm_shuffle_epi8(
                            _mm_packus_epi16(roundedLo, roundedHi), gatherBytes);
                        _mm_storel_epi64(reinterpret_cast<__m128i *>(destination + x * 4u), packed);
                    }
                    // Odd trailing column: one output pixel from four samples.
                    if (x < outputColumns)
                    {
                        const std::uint8_t *sampleBase = rowA + static_cast<std::size_t>(x) * 8u;
                        std::uint32_t accumulated[4] = {0, 0, 0, 0};
                        for (std::uint32_t subY = 0; subY < 2u; ++subY)
                        {
                            const std::uint8_t *sampleRow
                                = sampleBase + static_cast<std::size_t>(subY) * framebufferWidth * 4u;
                            for (std::uint32_t subX = 0; subX < 2u; ++subX)
                            {
                                const std::uint8_t *sample = sampleRow + static_cast<std::size_t>(subX) * 4u;
                                accumulated[0] += sample[0];
                                accumulated[1] += sample[1];
                                accumulated[2] += sample[2];
                                accumulated[3] += sample[3];
                            }
                        }
                        const std::uint32_t half = 1u << (sampleShift - 1u);
                        destination[x * 4u + 0] = static_cast<std::uint8_t>((accumulated[0] + half) >> sampleShift);
                        destination[x * 4u + 1] = static_cast<std::uint8_t>((accumulated[1] + half) >> sampleShift);
                        destination[x * 4u + 2] = static_cast<std::uint8_t>((accumulated[2] + half) >> sampleShift);
                        destination[x * 4u + 3] = static_cast<std::uint8_t>((accumulated[3] + half) >> sampleShift);
                    }
                }
                return;
            }
#endif
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
        // The tent resolve's per-band scratch is sized once here (the band
        // workers below index their own slice): three rows of raw block sums
        // plus their three horizontally folded rows, per band.
        std::size_t tentBandStride = 0;
        if (resolveTent_)
        {
            tentBandStride = static_cast<std::size_t>(outputColumns) * 4u * 6u;
            const std::size_t tentTotal = tentBandStride * (parallelWorkerCount() + 1u);
            if (tentRowSums_.size() < tentTotal)
            {
                tentRowSums_.assign(tentTotal, 0);
            }
        }
        (void)tentBandStride; // the resolve lambda slices by band index itself
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
                if (!taskScheduler_->schedule(resolveGroup, [this, &resolveRows, begin, end, band]()
                        { resolveRows(begin, end, band); }))
                {
                    resolveRows(begin, end, band);
                }
            }
            resolveRows(0u, std::min(bandWidth, outputRows), 0u);
            resolveGroup.wait(*taskScheduler_);
        }
        else
        {
            resolveRows(0u, outputRows, 0u);
        }
    }
    // Presentation bloom (renderer.bloom): the resolve above produced the
    // tone-mapped, sRGB-encoded output rows of the locked surface; the bloom
    // bright-passes them (decoded back to linear through the shared encode
    // grid's inverse), blurs the bright energy at quarter resolution, and
    // adds the weighted glare back before the surface unlocks. The whole
    // pass reads and writes the same locked rows, so it serializes cleanly
    // after the resolve bands.
    if (bloomEnabled_ && bloomStrength_ > 0.0f && columns > 8u && rows > 8u)
    {
        applyPresentationBloom(pixels, columns, rows);
    }
    // Film antialiasing (renderer.film_aa): the luminance-directed edge
    // blend over the finished, bloomed rows - the rasterizer's screen-space
    // diagonal staircase smoothed before the camera response layers the
    // contrast curve, the lens falloff, and the grain on top.
    if (filmAA_ && columns > 4u && rows > 4u)
    {
        applyFilmAntialias(pixels, columns, rows);
    }
    // Camera response (renderer.contrast / vignette / grain): the
    // photographic layer over the finished, bloomed output - the contrast
    // pivot curve, the radial lens falloff, and deterministic sensor grain,
    // decoded to linear and re-encoded through the shared byte table. Runs
    // after the bloom so the glare carries the same lens response.
    if ((cameraContrast_ != 1.0f || cameraVignette_ > 0.0f || cameraGrain_ > 0.0f) && columns > 1u && rows > 1u)
    {
        applyCameraResponse(pixels, columns, rows);
    }
    // Sun shadow map rebuild (renderer.sun_shadows): the casters recorded by
    // this frame's draws rasterize into the map the NEXT frame shades
    // through; a hash over the caster set and the sun skips the rebuild
    // while the scene is static.
    updateSunShadowMap();
    window_->unlockSurfacePixels(window_->self, surface_);
    ++statistics_.framesRendered;
}

// The presentation bloom pass (see the endFrame call site): bright-pass +
// quarter-res separable Gaussian + weighted add-back over the locked
// surface's encoded rows. The decode side of the sRGB grid is a 256-entry
// inverse table built like srgb_tables' forward tables; the encode side
// reuses byteEncode so the output quantization matches every other commit
// path. The pass keeps its scratch across frames (steady-state frames
// allocate nothing).
void HeadlessBackend::applyPresentationBloom(const window::SurfacePixels &pixels, std::uint32_t columns,
    std::uint32_t rows)
{
    const std::uint32_t smallWidth  = std::max(columns / 4u, 1u);
    const std::uint32_t smallHeight = std::max(rows / 4u, 1u);
    const std::size_t   smallCount  = static_cast<std::size_t>(smallWidth) * smallHeight;
    if (bloomBright_.size() != smallCount)
    {
        bloomBright_.assign(smallCount, glm::vec3(0.0f));
        bloomScratch_.assign(smallCount, glm::vec3(0.0f));
    }
    // Inverse of the fast encode table: fastSrgbEncode quantizes through the
    // 256-step grid, so the decode of a committed byte is that grid point's
    // linear value (one shared table, built on first use).
    static const std::array<float, 256> decode = []
    {
        std::array<float, 256> table{};
        for (std::uint32_t index = 0; index < 256; ++index)
        {
            table[index] = std::pow(static_cast<float>(index) / 255.0f, 2.2f);
        }
        return table;
    }();
    const float threshold = std::clamp(bloomThreshold_, 0.0f, 1.0f);

    // Bright pass: each quarter-res cell averages its 4x4 block of output
    // pixels in linear space and keeps the energy above the threshold (the
    // per-channel excess over the threshold, the soft-knee form every
    // presentation bloom uses). Fully dark cells store zero so the blur
    // treats them as empty space.
    for (std::uint32_t cellY = 0; cellY < smallHeight; ++cellY)
    {
        for (std::uint32_t cellX = 0; cellX < smallWidth; ++cellX)
        {
            glm::vec3 sum(0.0f);
            std::uint32_t count = 0;
            const std::uint32_t rowBegin = cellY * 4u;
            const std::uint32_t rowEnd   = std::min(rowBegin + 4u, rows);
            const std::uint32_t columnBegin = cellX * 4u;
            const std::uint32_t columnEnd   = std::min(columnBegin + 4u, columns);
            for (std::uint32_t y = rowBegin; y < rowEnd; ++y)
            {
                const std::uint8_t *row = pixels.rgba + static_cast<std::size_t>(y) * pixels.strideBytes;
                for (std::uint32_t x = columnBegin; x < columnEnd; ++x)
                {
                    const std::uint8_t *pixel = row + static_cast<std::size_t>(x) * 4u;
                    sum.r += decode[pixel[0]];
                    sum.g += decode[pixel[1]];
                    sum.b += decode[pixel[2]];
                    ++count;
                }
            }
            glm::vec3 energy(0.0f);
            if (count != 0)
            {
                const glm::vec3 average = sum * (1.0f / static_cast<float>(count));
                for (std::uint32_t axis = 0; axis < 3; ++axis)
                {
                    energy[axis] = std::max(average[axis] - threshold, 0.0f);
                }
            }
            bloomBright_[static_cast<std::size_t>(cellY) * smallWidth + cellX] = energy;
        }
    }

    // Separable Gaussian: a symmetric 5-tap kernel (exp(-d^2 / (2 * 1.5^2))
    // normalized over the taps), horizontal into the scratch then vertical
    // back into the bright buffer.
    constexpr float kKernel[5] = {0.093099f, 0.228636f, 0.356530f, 0.228636f, 0.093099f};
    const auto blurAxis = [&](bool horizontal)
    {
        for (std::uint32_t y = 0; y < smallHeight; ++y)
        {
            for (std::uint32_t x = 0; x < smallWidth; ++x)
            {
                glm::vec3 sum(0.0f);
                for (std::int32_t tap = -2; tap <= 2; ++tap)
                {
                    std::int32_t sampleX = horizontal ? static_cast<std::int32_t>(x) + tap : static_cast<std::int32_t>(x);
                    std::int32_t sampleY = horizontal ? static_cast<std::int32_t>(y)
                                                      : static_cast<std::int32_t>(y) + tap;
                    sampleX = std::clamp(sampleX, 0, static_cast<std::int32_t>(smallWidth) - 1);
                    sampleY = std::clamp(sampleY, 0, static_cast<std::int32_t>(smallHeight) - 1);
                    sum += bloomBright_[static_cast<std::size_t>(sampleY) * smallWidth
                              + static_cast<std::size_t>(sampleX)]
                        * kKernel[tap + 2];
                }
                bloomScratch_[static_cast<std::size_t>(y) * smallWidth + x] = sum;
            }
        }
        std::swap(bloomBright_, bloomScratch_);
    };
    blurAxis(true);
    blurAxis(false);

    // Add-back: every output pixel bilinearly samples the blurred energy at
    // its quarter-res position, scales by the strength, and adds it to its
    // decoded linear color before re-encoding through the shared byte table.
    const std::uint8_t *encodeTable = srgb_tables::byteEncode.values;
    const float         strength    = bloomStrength_;
    for (std::uint32_t y = 0; y < rows; ++y)
    {
        std::uint8_t *row = pixels.rgba + static_cast<std::size_t>(y) * pixels.strideBytes;
        for (std::uint32_t x = 0; x < columns; ++x)
        {
            const float u = (static_cast<float>(x) + 0.5f) * 0.25f - 0.5f;
            const float v = (static_cast<float>(y) + 0.5f) * 0.25f - 0.5f;
            const std::int32_t x0 = std::clamp(static_cast<std::int32_t>(std::floor(u)), 0,
                static_cast<std::int32_t>(smallWidth) - 1);
            const std::int32_t y0 = std::clamp(static_cast<std::int32_t>(std::floor(v)), 0,
                static_cast<std::int32_t>(smallHeight) - 1);
            const std::int32_t x1 = std::min(x0 + 1, static_cast<std::int32_t>(smallWidth) - 1);
            const std::int32_t y1 = std::min(y0 + 1, static_cast<std::int32_t>(smallHeight) - 1);
            const float fx = std::clamp(u - static_cast<float>(x0), 0.0f, 1.0f);
            const float fy = std::clamp(v - static_cast<float>(y0), 0.0f, 1.0f);
            const glm::vec3 *t00 = &bloomBright_[static_cast<std::size_t>(y0) * smallWidth + x0];
            const glm::vec3 *t10 = &bloomBright_[static_cast<std::size_t>(y0) * smallWidth + x1];
            const glm::vec3 *t01 = &bloomBright_[static_cast<std::size_t>(y1) * smallWidth + x0];
            const glm::vec3 *t11 = &bloomBright_[static_cast<std::size_t>(y1) * smallWidth + x1];
            const glm::vec3 glare = (*t00 * ((1.0f - fx) * (1.0f - fy)) + *t10 * (fx * (1.0f - fy))
                + *t01 * ((1.0f - fx) * fy) + *t11 * (fx * fy))
                * strength;
            std::uint8_t *pixel = row + static_cast<std::size_t>(x) * 4u;
            for (std::uint32_t channel = 0; channel < 3; ++channel)
            {
                const float linear = decode[pixel[channel]] + glare[channel];
                const float clamped = linear < 0.0f ? 0.0f : (linear > 1.0f ? 1.0f : linear);
                pixel[channel] = encodeTable[static_cast<std::uint32_t>(clamped * 255.0f + 0.5f)];
            }
        }
    }
}

// The camera response pass (see the endFrame call site): the photographic
// layer over the finished output. Every pixel decodes to linear through the
// shared sRGB grid's inverse (the same table the bloom builds; a local twin
// keeps each presentation pass self-contained), applies the pivot-point
// contrast curve, the radial lens falloff, and one deterministic grain
// sample, then re-encodes through the shared byte table. The grain hash
// mixes the frame counter, so an animated view shows uncorrelated sensor
// noise while a converged static frame stays bit-stable across exports.
// Rows are split across workers exactly like the resolve bands: every pixel
// reads and writes only its own cell.
void HeadlessBackend::applyCameraResponse(const window::SurfacePixels &pixels, std::uint32_t columns,
    std::uint32_t rows)
{
    static const std::array<float, 256> decode = []
    {
        std::array<float, 256> table{};
        for (std::uint32_t index = 0; index < 256; ++index)
        {
            table[index] = std::pow(static_cast<float>(index) / 255.0f, 2.2f);
        }
        return table;
    }();
    const std::uint8_t *encodeTable = srgb_tables::byteEncode.values;

    const float contrast   = cameraContrast_ > 0.05f ? cameraContrast_ : 1.0f;
    const float vignette   = std::clamp(cameraVignette_, 0.0f, 1.0f);
    const float grain      = std::clamp(cameraGrain_, 0.0f, 0.2f);
    // Contrast pivot: the sRGB mid-gray linear value (0.18), the exposure
    // midpoint every tone curve pivots around.
    constexpr float kPivot = 0.18f;
    const std::uint32_t frameSeed = static_cast<std::uint32_t>(statistics_.framesRendered);

    const auto processRows = [&](std::uint32_t rowBegin, std::uint32_t rowEnd)
    {
        for (std::uint32_t y = rowBegin; y < rowEnd; ++y)
        {
            std::uint8_t *row = pixels.rgba + static_cast<std::size_t>(y) * pixels.strideBytes;
            const float normalizedY = (static_cast<float>(y) + 0.5f) / static_cast<float>(rows) * 2.0f - 1.0f;
            for (std::uint32_t x = 0; x < columns; ++x)
            {
                std::uint8_t *pixel = row + static_cast<std::size_t>(x) * 4u;
                glm::vec3 linear(decode[pixel[0]], decode[pixel[1]], decode[pixel[2]]);
                if (contrast != 1.0f)
                {
                    linear = (linear - glm::vec3(kPivot)) * contrast + glm::vec3(kPivot);
                    linear = glm::clamp(linear, glm::vec3(0.0f), glm::vec3(1.0f));
                }
                if (vignette > 0.0f)
                {
                    const float normalizedX
                        = (static_cast<float>(x) + 0.5f) / static_cast<float>(columns) * 2.0f - 1.0f;
                    const float radius = std::sqrt(normalizedX * normalizedX + normalizedY * normalizedY);
                    // Smooth falloff from the frame's mid-radius to its corner:
                    // centers stay untouched, corners lose the full strength.
                    const float t = glm::smoothstep(0.45f, 1.35f, radius);
                    linear *= 1.0f - vignette * t;
                }
                if (grain > 0.0f)
                {
                    // Wang hash over (pixel, frame): uniform in [0, 1), fixed
                    // for a given frame and pixel. Film-like amplitude peaks
                    // in the midtones and fades in the extremes.
                    std::uint32_t state = frameSeed * 0x9E3779B9u + x * 0x85EBCA6Bu + y * 0xC2B2AE35u;
                    state ^= state >> 15;
                    state *= 0x2545F491u;
                    state ^= state >> 13;
                    const float noise  = static_cast<float>(state & 0x00FFFFFFu) * (1.0f / 16777216.0f) - 0.5f;
                    const float luma   = linear.x * 0.299f + linear.y * 0.587f + linear.z * 0.114f;
                    const float midtone = 4.0f * luma * (1.0f - luma);
                    // One gain per pixel, not one delta per channel: scaling
                    // all three channels by the same factor keeps the hue,
                    // so the grain reads as luminance-density film grain
                    // instead of per-channel colored sensor noise.
                    linear *= 1.0f + grain * (0.35f + 0.65f * midtone) * noise;
                }
                for (std::uint32_t channel = 0; channel < 3; ++channel)
                {
                    const float clamped
                        = linear[channel] < 0.0f ? 0.0f : (linear[channel] > 1.0f ? 1.0f : linear[channel]);
                    pixel[channel] = encodeTable[static_cast<std::uint32_t>(clamped * 255.0f + 0.5f)];
                }
            }
        }
    };

    const bool inParallel = taskScheduler_ != nullptr && taskScheduler_->isRunning() && parallelWorkerCount() >= 2u
        && rows >= 2u;
    if (inParallel)
    {
        const std::uint32_t workers    = parallelWorkerCount();
        const std::uint32_t bandCount  = workers;
        const std::uint32_t bandWidth  = (rows + bandCount - 1u) / bandCount;
        TaskGroup group;
        for (std::uint32_t band = 1; band < bandCount; ++band)
        {
            const std::uint32_t begin = band * bandWidth;
            const std::uint32_t end   = std::min(begin + bandWidth, rows);
            if (begin >= end)
            {
                break;
            }
            if (!taskScheduler_->schedule(group, [this, &processRows, begin, end]()
                    { processRows(begin, end); }))
            {
                processRows(begin, end);
            }
        }
        processRows(0u, std::min(bandWidth, rows));
        group.wait(*taskScheduler_);
    }
    else
    {
        processRows(0u, rows);
    }
}

// The film antialiasing pass (see the endFrame call site): a
// luminance-directed edge blend over the finished, bloomed rows, in the
// encoded (non-linear) space the blend math expects. A snapshot of the rows
// feeds every sample, so band-parallel workers always read the pre-pass
// image and the output stays deterministic regardless of which worker
// touched which row first. Per pixel: five luminance taps bracket the local
// contrast, four corner taps orient the blend axis, and the kernel mixes the
// center with samples offset along the edge normal - an FXAA-class
// formulation whose range check falls back to the higher-quality pair of
// thirds-samples wherever the full blend would overshoot the local bracket,
// so silhouettes keep their shape while the diagonals lose the staircase.
// Flat neighborhoods take the early-out and are never touched (the grain the
// camera response adds afterwards keeps its per-pixel detail).
// Tent supersample resolve (see HeadlessBackendOptions::resolveTentEnabled
// and the endFrame call site): the separable [1 2 1; 2 4 2; 1 2 1] / 16
// kernel over the 3x3 neighborhood of sample-block sums, edge blocks
// clamped. The kernel runs on 16-bit block sums (a block holds at most
// sampleCount * 255 per channel, so the widest intermediate stays inside 16
// bits for every supported grid): each output row's three block rows are
// computed once, folded horizontally, and combined vertically. `scratch` is
// the calling band's private slice (see the header); each band keeps a
// sliding three-row window so a row's sums and folds are computed exactly
// once, and concurrent bands never share state, so the pass stays
// deterministic under scheduling.
void HeadlessBackend::resolveRowsTent(const std::uint8_t *samples, std::uint32_t framebufferWidth,
    window::SurfacePixels &pixels, std::uint32_t outputColumns, std::uint32_t outputRows,
    std::uint32_t rowBegin, std::uint32_t rowEnd, std::uint16_t *scratch) const
{
    const std::size_t rowStride = static_cast<std::size_t>(outputColumns) * 4u;
    // Scratch layout: three rows of raw block sums followed by their three
    // horizontally folded rows (the calling band's private slice).
    std::uint16_t *rawSums[3]    = {scratch, scratch + rowStride, scratch + rowStride * 2u};
    std::uint16_t *foldedSums[3] = {scratch + rowStride * 3u, scratch + rowStride * 4u,
        scratch + rowStride * 5u};
    // Block sums of one output row's sampleScaleX_ x sampleScaleY_ sample
    // block (the same sum the box resolve rounds down to bytes).
    const auto computeRowSums = [this, samples, framebufferWidth, outputColumns](
        std::uint32_t outY, std::uint16_t *dest)
    {
        const std::uint8_t *rowBase
            = samples + static_cast<std::size_t>(outY) * sampleScaleY_ * framebufferWidth * 4u;
        for (std::uint32_t x = 0; x < outputColumns; ++x)
        {
            const std::uint8_t *blockBase = rowBase + static_cast<std::size_t>(x) * sampleScaleX_ * 4u;
            std::uint32_t accumulated[4] = {0, 0, 0, 0};
            for (std::uint32_t sampleY = 0; sampleY < sampleScaleY_; ++sampleY)
            {
                const std::uint8_t *sample
                    = blockBase + static_cast<std::size_t>(sampleY) * framebufferWidth * 4u;
                for (std::uint32_t sampleX = 0; sampleX < sampleScaleX_; ++sampleX)
                {
                    accumulated[0] += sample[0];
                    accumulated[1] += sample[1];
                    accumulated[2] += sample[2];
                    accumulated[3] += sample[3];
                    sample += 4;
                }
            }
            dest[x * 4u + 0] = static_cast<std::uint16_t>(accumulated[0]);
            dest[x * 4u + 1] = static_cast<std::uint16_t>(accumulated[1]);
            dest[x * 4u + 2] = static_cast<std::uint16_t>(accumulated[2]);
            dest[x * 4u + 3] = static_cast<std::uint16_t>(accumulated[3]);
        }
    };
    // Horizontal [1 2 1] fold of a raw block-sum row, edge columns clamped.
    const auto foldRowHorizontally = [outputColumns](const std::uint16_t *sums, std::uint16_t *folded)
    {
        for (std::uint32_t x = 0; x < outputColumns; ++x)
        {
            const std::size_t west = static_cast<std::size_t>(x > 0u ? x - 1u : 0u) * 4u;
            const std::size_t here = static_cast<std::size_t>(x) * 4u;
            const std::size_t east = static_cast<std::size_t>(x + 1u < outputColumns ? x + 1u : x) * 4u;
            for (std::uint32_t channel = 0; channel < 4; ++channel)
            {
                folded[here + channel] = static_cast<std::uint16_t>(sums[west + channel]
                    + 2u * sums[here + channel] + sums[east + channel]);
            }
        }
    };
    // Sliding window over output rows: the clamped south (y-1), center (y),
    // and north (y+1) block rows, each raw-summed and folded once.
    // The kernel's 16 weight units divide the 3x3 sum of BLOCK sums, and a
    // block sum carries sampleCount samples - the total shift is 4 + the
    // block's log2 sample count (mirroring the box resolve's sampleShift).
    const std::uint32_t sampleCount = sampleScaleX_ * sampleScaleY_;
    const std::uint32_t tentShift  = 4u + (sampleCount == 2u ? 1u
        : sampleCount == 4u ? 2u : sampleCount == 8u ? 3u : 0u);
    const std::uint32_t tentRound  = 1u << (tentShift - 1u);
    std::uint32_t southIndex   = rowBegin > 0u ? rowBegin - 1u : 0u;
    std::uint32_t centerIndex  = rowBegin;
    std::uint32_t northIndex   = rowBegin + 1u < outputRows ? rowBegin + 1u : rowBegin;
    computeRowSums(southIndex, rawSums[0]);
    foldRowHorizontally(rawSums[0], foldedSums[0]);
    computeRowSums(centerIndex, rawSums[1]);
    foldRowHorizontally(rawSums[1], foldedSums[1]);
    computeRowSums(northIndex, rawSums[2]);
    foldRowHorizontally(rawSums[2], foldedSums[2]);
    for (std::uint32_t y = rowBegin; y < rowEnd; ++y)
    {
        std::uint8_t *destination = pixels.rgba + static_cast<std::size_t>(y) * pixels.strideBytes;
        // Vertical [1 2 1] over the three folded rows, rounding to bytes.
        for (std::uint32_t x = 0; x < outputColumns; ++x)
        {
            const std::size_t index = static_cast<std::size_t>(x) * 4u;
            destination[index + 0] = static_cast<std::uint8_t>(
                (foldedSums[0][index + 0] + 2u * foldedSums[1][index + 0] + foldedSums[2][index + 0]
                    + tentRound) >> tentShift);
            destination[index + 1] = static_cast<std::uint8_t>(
                (foldedSums[0][index + 1] + 2u * foldedSums[1][index + 1] + foldedSums[2][index + 1]
                    + tentRound) >> tentShift);
            destination[index + 2] = static_cast<std::uint8_t>(
                (foldedSums[0][index + 2] + 2u * foldedSums[1][index + 2] + foldedSums[2][index + 2]
                    + tentRound) >> tentShift);
            destination[index + 3] = static_cast<std::uint8_t>(
                (foldedSums[0][index + 3] + 2u * foldedSums[1][index + 3] + foldedSums[2][index + 3]
                    + tentRound) >> tentShift);
        }
        if (y + 1u < rowEnd)
        {
            // Slide the row window one output row down (the north edge
            // clamps at the bottom): recycle the south row's buffers.
            const std::uint32_t nextNorth = y + 2u < outputRows ? y + 2u : outputRows - 1u;
            std::uint16_t *const oldestRaw    = rawSums[0];
            std::uint16_t *const oldestFolded = foldedSums[0];
            rawSums[0]    = rawSums[1];
            rawSums[1]    = rawSums[2];
            rawSums[2]    = oldestRaw;
            foldedSums[0] = foldedSums[1];
            foldedSums[1] = foldedSums[2];
            foldedSums[2] = oldestFolded;
            computeRowSums(nextNorth, rawSums[2]);
            foldRowHorizontally(rawSums[2], foldedSums[2]);
        }
    }
}

void HeadlessBackend::applyFilmAntialias(const window::SurfacePixels &pixels, std::uint32_t columns,
    std::uint32_t rows)
{
    const std::size_t byteCount = static_cast<std::size_t>(rows) * pixels.strideBytes;
    if (filmAASource_.size() != byteCount)
    {
        filmAASource_.resize(byteCount);
    }
    std::memcpy(filmAASource_.data(), pixels.rgba, byteCount);
    const std::uint8_t *source = filmAASource_.data();

    // Bilinear sample of the snapshot at sub-pixel coordinates: the edge
    // blend offsets are fractional texel steps, and a nearest-texel fetch
    // would collapse them back onto the center pixel.
    const auto samplePixel = [source, &pixels, columns, rows](float x, float y, glm::vec3 &outColor, float &outLuma)
    {
        const float clampedX = glm::clamp(x, 0.0f, static_cast<float>(columns) - 1.0f);
        const float clampedY = glm::clamp(y, 0.0f, static_cast<float>(rows) - 1.0f);
        const std::int32_t x0 = static_cast<std::int32_t>(clampedX);
        const std::int32_t y0 = static_cast<std::int32_t>(clampedY);
        const std::int32_t x1 = std::min(x0 + 1, static_cast<std::int32_t>(columns) - 1);
        const std::int32_t y1 = std::min(y0 + 1, static_cast<std::int32_t>(rows) - 1);
        const float fractionX = clampedX - static_cast<float>(x0);
        const float fractionY = clampedY - static_cast<float>(y0);
        const auto texel = [source, &pixels](std::int32_t x, std::int32_t y)
        {
            const std::uint8_t *t = source + static_cast<std::size_t>(y) * pixels.strideBytes
                + static_cast<std::size_t>(x) * 4u;
            return glm::vec3(static_cast<float>(t[0]), static_cast<float>(t[1]), static_cast<float>(t[2]));
        };
        const glm::vec3 top    = glm::mix(texel(x0, y0), texel(x1, y0), fractionX);
        const glm::vec3 bottom = glm::mix(texel(x0, y1), texel(x1, y1), fractionX);
        outColor = glm::mix(top, bottom, fractionY) * (1.0f / 255.0f);
        // Perceptual luma of the encoded channels: the edge math compares
        // what the eye sees, not linear radiance.
        outLuma = outColor.x * 0.299f + outColor.y * 0.587f + outColor.z * 0.114f;
    };

    const auto processRows = [&](std::uint32_t rowBegin, std::uint32_t rowEnd)
    {
        for (std::uint32_t y = rowBegin; y < rowEnd; ++y)
        {
            std::uint8_t *row = pixels.rgba + static_cast<std::size_t>(y) * pixels.strideBytes;
            const float centerY = static_cast<float>(y);
            for (std::uint32_t x = 0; x < columns; ++x)
            {
                std::uint8_t *pixel = row + static_cast<std::size_t>(x) * 4u;
                const float centerX = static_cast<float>(x);
                glm::vec3 color;
                float luma;
                glm::vec3 northColor, southColor, westColor, eastColor;
                float northLuma, southLuma, westLuma, eastLuma;
                samplePixel(centerX, centerY, color, luma);
                samplePixel(centerX, centerY - 1.0f, northColor, northLuma);
                samplePixel(centerX, centerY + 1.0f, southColor, southLuma);
                samplePixel(centerX - 1.0f, centerY, westColor, westLuma);
                samplePixel(centerX + 1.0f, centerY, eastColor, eastLuma);
                const float minimum = std::min({luma, northLuma, southLuma, westLuma, eastLuma});
                const float maximum = std::max({luma, northLuma, southLuma, westLuma, eastLuma});
                // Flat neighborhood (the common case): keep the pixel.
                if (maximum - minimum < 0.043f)
                {
                    continue;
                }
                float northWestLuma, northEastLuma, southWestLuma, southEastLuma;
                glm::vec3 cornerColor;
                samplePixel(centerX - 1.0f, centerY - 1.0f, cornerColor, northWestLuma);
                samplePixel(centerX + 1.0f, centerY - 1.0f, cornerColor, northEastLuma);
                samplePixel(centerX - 1.0f, centerY + 1.0f, cornerColor, southWestLuma);
                samplePixel(centerX + 1.0f, centerY + 1.0f, cornerColor, southEastLuma);
                // Edge normal direction from the corner luma differences
                // (the classic console-quality formulation), in texel units.
                // The clamp is 4, not the console's 8: this renderer's GI
                // gradients are smooth and wide, and a half-pair reaching 4
                // texels away read as defocus instead of antialiasing - 4
                // caps the full blend at a two-texel span, still enough to
                // hide the rasterizer's one-texel staircase.
                glm::vec2 direction(-(northWestLuma + northEastLuma - southWestLuma - southEastLuma),
                    northWestLuma + southWestLuma - northEastLuma - southEastLuma);
                const float directionReduce = std::max(
                    (northWestLuma + northEastLuma + southWestLuma + southEastLuma) * (0.125f * 0.25f), 1.0f / 128.0f);
                const float inverseDirectionMinimum
                    = 1.0f / (std::min(std::abs(direction.x), std::abs(direction.y)) + directionReduce);
                direction = glm::clamp(direction * inverseDirectionMinimum, glm::vec2(-4.0f), glm::vec2(4.0f));
                const auto blend = [&samplePixel, centerX, centerY, &direction](float offset)
                {
                    glm::vec3 sampledColor;
                    float sampledLuma;
                    samplePixel(centerX + direction.x * offset, centerY + direction.y * offset, sampledColor,
                        sampledLuma);
                    return sampledColor;
                };
                const glm::vec3 thirdsPair = (blend(1.0f / 3.0f - 0.5f) + blend(2.0f / 3.0f - 0.5f)) * 0.5f;
                const glm::vec3 halfPair   = thirdsPair * 0.5f + (blend(-0.5f) + blend(0.5f)) * 0.25f;
                const float halfPairLuma
                    = halfPair.x * 0.299f + halfPair.y * 0.587f + halfPair.z * 0.114f;
                // The full blend must stay inside the local contrast bracket;
                // outside it the thirds pair is the safe answer.
                const glm::vec3 chosen = (halfPairLuma < minimum || halfPairLuma > maximum) ? thirdsPair : halfPair;
                // Cross-edge softening (film_aa_strength): the along-edge
                // blend above runs parallel to a straight silhouette and
                // cannot touch the one-texel hard step a straight edge keeps
                // when it lands between supersample columns - the tent
                // resolve spreads that step, and this stage catches whatever
                // hard boundary remains. The pixel blends toward the mean of
                // its two neighbors ACROSS the edge, but only when it sits ON
                // the boundary: one across neighbor matches the pixel (its
                // own side's plateau) while the other lies on the far side.
                // A thin bright feature (an emissive strip, a specular slit)
                // has both across neighbors far from the pixel and is left
                // alone; a smooth GI gradient's deltas sit mid-bracket on
                // both sides and never qualify either. The weight ramps from
                // the flat-skip threshold to half a texel at strong contrast.
                glm::vec3 finalColor = chosen;
                const float contrast = maximum - minimum;
                const float crossWeight
                    = filmAAStrength_ * glm::clamp((contrast - 0.043f) * 4.0f, 0.0f, 0.5f);
                if (crossWeight > 0.0f)
                {
                    // Across the edge = along the luma gradient: east/west
                    // when the edge runs vertically, north/south otherwise.
                    const bool edgeRunsVertically
                        = std::abs(eastLuma - westLuma) >= std::abs(southLuma - northLuma);
                    const float nearDelta = edgeRunsVertically ? std::abs(luma - westLuma)
                                                               : std::abs(luma - northLuma);
                    const float farDelta  = edgeRunsVertically ? std::abs(luma - eastLuma)
                                                               : std::abs(luma - southLuma);
                    const bool boundaryPixel
                        = (nearDelta <= contrast * 0.25f && farDelta >= contrast * 0.6f)
                        || (farDelta <= contrast * 0.25f && nearDelta >= contrast * 0.6f);
                    if (boundaryPixel)
                    {
                        const glm::vec3 acrossMean = edgeRunsVertically ? (westColor + eastColor) * 0.5f
                                                                        : (northColor + southColor) * 0.5f;
                        finalColor = glm::mix(chosen, acrossMean, crossWeight);
                    }
                }
                // Direct quantization: the blend already lives in the encoded
                // display space the samples came from - routing it through
                // the shared encode table would apply the transfer curve a
                // second time and lift every blended edge toward white.
                for (std::uint32_t channel = 0; channel < 3; ++channel)
                {
                    const float clamped
                        = finalColor[channel] < 0.0f ? 0.0f
                          : (finalColor[channel] > 1.0f ? 1.0f : finalColor[channel]);
                    pixel[channel] = static_cast<std::uint8_t>(clamped * 255.0f + 0.5f);
                }
            }
        }
    };

    const bool inParallel = taskScheduler_ != nullptr && taskScheduler_->isRunning() && parallelWorkerCount() >= 2u
        && rows >= 2u;
    if (inParallel)
    {
        const std::uint32_t workers    = parallelWorkerCount();
        const std::uint32_t bandCount  = workers;
        const std::uint32_t bandWidth  = (rows + bandCount - 1u) / bandCount;
        TaskGroup group;
        for (std::uint32_t band = 1; band < bandCount; ++band)
        {
            const std::uint32_t begin = band * bandWidth;
            const std::uint32_t end   = std::min(begin + bandWidth, rows);
            if (begin >= end)
            {
                break;
            }
            if (!taskScheduler_->schedule(group, [this, &processRows, begin, end]()
                    { processRows(begin, end); }))
            {
                processRows(begin, end);
            }
        }
        processRows(0u, std::min(bandWidth, rows));
        group.wait(*taskScheduler_);
    }
    else
    {
        processRows(0u, rows);
    }
}

// Records one submitted draw as a sun-shadow caster. The exclusion rule
// mirrors the lighting modules' shadow-ray semantics: unlit draws are the
// decorative geometry the environment already accounts for (sky domes, emissive
// panels), and transparent draws transmit their share instead of blocking -
// a hard shadow from either would darken what the lighting already handles.
// Recording happens before the frustum cull inside prepareRasterization: a
// mesh outside the view frustum still casts a visible shadow into it.
void HeadlessBackend::recordSunShadowCaster(MeshHandle mesh, const float *modelMatrix, const MeshMaterial &material)
{
    if ((!sunShadowSunActive_ && !skyShadowsEnabled_) || mesh == 0 || modelMatrix == nullptr)
    {
        return;
    }
    if (material.unlit || material.opacity < 0.999f)
    {
        return;
    }
    sunshadows::SunShadowCaster caster;
    caster.mesh = mesh;
    std::memcpy(caster.transform, modelMatrix, sizeof(caster.transform));
    sunShadowCasters_.push_back(caster);
}

// FNV-1a over the shared caster state (casters, camera, resolution): the
// body both shadow-map hashes reuse, so the sun map additionally mixes in
// the sun direction and the sky map stays direction-fixed.
std::uint64_t HeadlessBackend::sunShadowCastersHash() const
{
    const auto mix = [](std::uint64_t hash, const float *values, std::size_t count)
    {
        for (std::size_t index = 0; index < count; ++index)
        {
            std::uint32_t bits = 0;
            std::memcpy(&bits, values + index, sizeof(bits));
            hash ^= bits;
            hash *= 0x00000100000001B3ull;
        }
        return hash;
    };
    std::uint64_t hash = 0xCBF29CE484222325ull;
    hash ^= shadowMapResolution_;
    hash *= 0x00000100000001B3ull;
    hash ^= sunShadowCasters_.size();
    hash *= 0x00000100000001B3ull;
    for (const sunshadows::SunShadowCaster &caster : sunShadowCasters_)
    {
        hash ^= caster.mesh;
        hash *= 0x00000100000001B3ull;
        hash = mix(hash, caster.transform, 16);
    }
    const float camera[3] = {cameraPosition_.x, cameraPosition_.y, cameraPosition_.z};
    hash                  = mix(hash, camera, 3);
    const float viewProjection[16] = {viewProjection_[0][0], viewProjection_[1][0], viewProjection_[2][0],
        viewProjection_[3][0], viewProjection_[0][1], viewProjection_[1][1], viewProjection_[2][1],
        viewProjection_[3][1], viewProjection_[0][2], viewProjection_[1][2], viewProjection_[2][2],
        viewProjection_[3][2], viewProjection_[0][3], viewProjection_[1][3], viewProjection_[2][3],
        viewProjection_[3][3]};
    hash = mix(hash, viewProjection, 16);
    return hash;
}

// The sun map's hash: the shared caster state plus the sun's on/off state
// and direction. Two frames with the same hash produce the same map, so the
// rebuild is skipped (the static-scene amortization).
std::uint64_t HeadlessBackend::sunShadowStateHash() const
{
    std::uint64_t hash = sunShadowCastersHash();
    hash ^= sunShadowSunActive_ ? 1u : 0u;
    hash *= 0x00000100000001B3ull;
    const float sun[3] = {lightDirection_.x, lightDirection_.y, lightDirection_.z};
    for (std::size_t index = 0; index < 3; ++index)
    {
        std::uint32_t bits = 0;
        std::memcpy(&bits, sun + index, sizeof(bits));
        hash ^= bits;
        hash *= 0x00000100000001B3ull;
    }
    return hash;
}

// The sky-visibility map's hash: the shared caster state alone (the map's
// direction is the fixed world up).
std::uint64_t HeadlessBackend::skyShadowStateHash() const
{
    return sunShadowCastersHash() ^ 0x5BF8CA9A17C1ull;
}

// Resolves the casters into build-ready geometry (the mesh lookups and the
// exact world bounds boxes; see sun_shadows.hpp).
std::vector<sunshadows::SunShadowGeometry> HeadlessBackend::sunShadowGeometries() const
{
    std::vector<sunshadows::SunShadowGeometry> geometries;
    geometries.reserve(sunShadowCasters_.size());
    for (const sunshadows::SunShadowCaster &caster : sunShadowCasters_)
    {
        const auto found = meshes.find(caster.mesh);
        if (found == meshes.end())
        {
            continue;
        }
        const HeadlessMesh &mesh = found->second;
        sunshadows::SunShadowGeometry geometry;
        geometry.positions            = mesh.positions.data();
        geometry.positionCount        = mesh.positions.size() / 3;
        geometry.indices              = mesh.indices.data();
        geometry.indexCount           = mesh.indices.size();
        geometry.transformColumnMajor = caster.transform;
        const glm::mat4 model         = glm::make_mat4(caster.transform);
        // The mesh's local bounds box (center +/- radius, the bounding
        // sphere's conservative box) transformed by the model matrix: exact
        // under any rotation/scale, so the orthographic fit measures the
        // caster's true footprint.
        const glm::vec3 localMin = mesh.boundsCenter - glm::vec3(mesh.boundsRadius);
        const glm::vec3 localMax = mesh.boundsCenter + glm::vec3(mesh.boundsRadius);
        geometry.worldBoundsMin = glm::vec3(std::numeric_limits<float>::max());
        geometry.worldBoundsMax = glm::vec3(std::numeric_limits<float>::lowest());
        for (std::int32_t dz = 0; dz < 2; ++dz)
        {
            for (std::int32_t dy = 0; dy < 2; ++dy)
            {
                for (std::int32_t dx = 0; dx < 2; ++dx)
                {
                    const glm::vec3 localCorner(dx ? localMax.x : localMin.x,
                        dy ? localMax.y : localMin.y,
                        dz ? localMax.z : localMin.z);
                    const glm::vec3 worldCorner(model * glm::vec4(localCorner, 1.0f));
                    geometry.worldBoundsMin = glm::min(geometry.worldBoundsMin, worldCorner);
                    geometry.worldBoundsMax = glm::max(geometry.worldBoundsMax, worldCorner);
                }
            }
        }
        if (mesh.boundsValid && std::isfinite(geometry.worldBoundsMin.x) && std::isfinite(geometry.worldBoundsMax.x))
        {
            geometries.push_back(geometry);
        }
    }
    return geometries;
}

// The receiver box the orthographic u/v fits follow: the view frustum's
// world bounds, from the frame's view-projection (the same matrix every
// draw clipped through). A singular inverse falls back to the casters' own
// fit.
void HeadlessBackend::sunShadowReceiverBox(glm::vec3 &receiverMin, glm::vec3 &receiverMax, bool &haveBox) const
{
    receiverMin = glm::vec3(0.0f);
    receiverMax = glm::vec3(0.0f);
    haveBox     = false;
    if (!cameraInitialized_)
    {
        return;
    }
    const glm::mat4 inverseViewProjection = glm::inverse(viewProjection_);
    if (!std::isfinite(inverseViewProjection[0][0]))
    {
        return;
    }
    glm::vec3 corners[8];
    std::size_t corner = 0;
    for (std::int32_t dz = -1; dz <= 1; dz += 2)
    {
        for (std::int32_t dy = -1; dy <= 1; dy += 2)
        {
            for (std::int32_t dx = -1; dx <= 1; dx += 2)
            {
                const glm::vec4 world = inverseViewProjection * glm::vec4(
                    static_cast<float>(dx), static_cast<float>(dy), static_cast<float>(dz), 1.0f);
                if (world.w > 1.0e-6f)
                {
                    corners[corner++] = glm::vec3(world) / world.w;
                }
            }
        }
    }
    if (corner != 8)
    {
        return;
    }
    receiverMin = corners[0];
    receiverMax = corners[0];
    for (std::size_t index = 1; index < corner; ++index)
    {
        receiverMin = glm::min(receiverMin, corners[index]);
        receiverMax = glm::max(receiverMax, corners[index]);
    }
    haveBox = std::isfinite(receiverMin.x) && std::isfinite(receiverMax.x);
}

// Builds the shadow maps from the frame's recorded casters (see
// sun_shadows.hpp). Called at endFrame: the maps this build produces are the
// ones the NEXT frame shades through, and matching hashes skip each build
// independently - a static scene pays each rasterization exactly once.
void HeadlessBackend::updateSunShadowMap()
{
    if (sunShadowCasters_.empty() || (!sunShadowSunActive_ && !skyShadowsEnabled_))
    {
        sunShadowMap_.valid = false;
        sunShadowBuiltHash_ = 0;
        skyShadowMap_.valid = false;
        skyShadowBuiltHash_ = 0;
        return;
    }
    glm::vec3 receiverMin(0.0f);
    glm::vec3 receiverMax(0.0f);
    bool      haveReceiverBox = false;
    sunShadowReceiverBox(receiverMin, receiverMax, haveReceiverBox);

    if (sunShadowSunActive_)
    {
        const std::uint64_t hash = sunShadowStateHash();
        if (!sunShadowMap_.valid || hash != sunShadowBuiltHash_)
        {
            sunshadows::buildSunShadowMap(sunShadowMap_,
                sunShadowGeometries(),
                shadowMapResolution_,
                lightDirection_,
                haveReceiverBox ? &receiverMin : nullptr,
                haveReceiverBox ? &receiverMax : nullptr);
            sunShadowBuiltHash_ = hash;
        }
    }
    else
    {
        sunShadowMap_.valid = false;
        sunShadowBuiltHash_ = 0;
    }

    if (skyShadowsEnabled_)
    {
        const std::uint64_t hash = skyShadowStateHash();
        if (!skyShadowMap_.valid || hash != skyShadowBuiltHash_)
        {
            // Zenith direction: the sky's radiance arrives from straight
            // above, so the map measures what stands between each point and
            // the open sky.
            sunshadows::buildSunShadowMap(skyShadowMap_,
                sunShadowGeometries(),
                shadowMapResolution_,
                glm::vec3(0.0f, 1.0f, 0.0f),
                haveReceiverBox ? &receiverMin : nullptr,
                haveReceiverBox ? &receiverMax : nullptr);
            skyShadowBuiltHash_ = hash;
        }
    }
    else
    {
        skyShadowMap_.valid = false;
        skyShadowBuiltHash_ = 0;
    }

    // Debug export (ENGINE_SHADOW_DEBUG=path): the built depth maps as PNGs,
    // grayscale (1 = empty/far). Diagnostic aid for shadow-map bring-up.
    if (const char *shadowDebug = std::getenv("ENGINE_SHADOW_DEBUG"); shadowDebug != nullptr)
    {
        const auto exportMap = [&](const sunshadows::SunShadowMap &map, const char *suffix)
        {
            if (!map.valid)
            {
                return;
            }
            std::string path(shadowDebug);
            path += suffix;
            std::vector<std::uint8_t> pixels(static_cast<std::size_t>(map.resolution) * map.resolution * 4);
            for (std::size_t index = 0; index < pixels.size(); index += 4)
            {
                const std::uint8_t gray = static_cast<std::uint8_t>(
                    std::clamp(map.depth[index / 4], 0.0f, 1.0f) * 255.0f + 0.5f);
                pixels[index] = gray;
                pixels[index + 1] = gray;
                pixels[index + 2] = gray;
                pixels[index + 3] = 255;
            }
            util::writePngRgba(path.c_str(), map.resolution, map.resolution, pixels.data(),
                static_cast<std::size_t>(map.resolution) * 4, false, false);
            std::fprintf(stderr, "shadow map %s: %ux%u texel=%.4f\n", suffix, map.resolution, map.resolution,
                map.texelWorldSize);
        };
        exportMap(sunShadowMap_, ".sun.png");
        exportMap(skyShadowMap_, ".sky.png");
    }
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

    // Shadow caster recording (sun_shadows.hpp): before the frustum cull
    // inside prepareRasterization - a mesh outside the view still casts a
    // visible shadow into it.
    recordSunShadowCaster(mesh, modelMatrix, material);

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
            recordSunShadowCaster(item.mesh, item.modelMatrix, material);
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
                if (record.shadeRuns
                        ? rasterizeFanRecordRuns(batchSetups_[record.setupIndex], record, 0,
                            static_cast<std::int32_t>(framebufferWidth) - 1, 0,
                            static_cast<std::int32_t>(framebufferHeight) - 1, nullptr)
                        : rasterizeFanRecord(batchSetups_[record.setupIndex], record, 0,
                            static_cast<std::int32_t>(framebufferWidth) - 1, 0,
                            static_cast<std::int32_t>(framebufferHeight) - 1, nullptr))
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
                if (record.shadeRuns
                        ? rasterizeFanRecordRuns(batchSetups_[record.setupIndex], record, 0,
                            static_cast<std::int32_t>(framebufferWidth) - 1, 0,
                            static_cast<std::int32_t>(framebufferHeight) - 1, nullptr)
                        : rasterizeFanRecord(batchSetups_[record.setupIndex], record, 0,
                            static_cast<std::int32_t>(framebufferWidth) - 1, 0,
                            static_cast<std::int32_t>(framebufferHeight) - 1, nullptr))
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
    outSetup.alphaCutoff             = glm::clamp(material.alphaCutoff, 0.0f, 1.0f);

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
    // Alpha cutout threshold, 0 when inactive (no cutoff, transparent draws,
    // or no base-color map): the per-pixel check folds to one compare.
    const float alphaCutoffActive = (setup.alphaCutoff > 0.0f && !setup.transparent && setup.hasBaseColorMap)
        ? setup.alphaCutoff
        : 0.0f;
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
                        // Alpha cutout (MeshMaterial::alphaCutoff, mesh.frag's
                        // uCloth.z discard mirror): a texel below the cutoff
                        // skips shading and every commit exactly like the
                        // fur crop below - opaque draws keep depth writes, so
                        // masked foliage's own depth is consistent everywhere.
                        // Transparent draws blend instead; the cutoff does not
                        // apply to them.
                        if (alphaCutoffActive)
                        {
                            if (baseColorMap->sampleAlpha(interpolatedUv.x, interpolatedUv.y) < alphaCutoffActive)
                            {
                                weightA += weightADx;
                                weightB += weightBDx;
                                pixelDepth += depthADx;
                                continue;
                            }
                        }
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
                                // No normal map, no occlusion map, no radiance_cache
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
                            // Indirect specular (reflection_field field hook):
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
                                // the material's opacity TIMES the base-color
                                // map's texel alpha as the blend factor (the
                                // mesh.frag oColor.a twin: uCloth.w * texel.a
                                // on the GPU side) - a textured transparent
                                // sprite (foam patch, mist bank) fades per
                                // texel instead of compositing as a solid
                                // card. Fully transparent texels skip the
                                // blend entirely.
                                const float texelAlpha = setup.hasBaseColorMap
                                    ? setup.baseColorMap->sampleAlpha(interpolatedUv.x, interpolatedUv.y)
                                    : 1.0f;
                                const float effectiveOpacity = setup.opacity * texelAlpha;
                                if (effectiveOpacity > 0.0039f)
                                {
                                    const glm::vec3 encoded = fastSrgbEncode(glm::max(shaded, glm::vec3(0.0f)));
                                    const float   oneMinus = 1.0f - effectiveOpacity;
                                    pixel[0] = toByte(encoded.x * effectiveOpacity + static_cast<float>(pixel[0]) / 255.0f * oneMinus);
                                    pixel[1] = toByte(encoded.y * effectiveOpacity + static_cast<float>(pixel[1]) / 255.0f * oneMinus);
                                    pixel[2] = toByte(encoded.z * effectiveOpacity + static_cast<float>(pixel[2]) / 255.0f * oneMinus);
                                    pixel[3] = 255;
                                }
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
        // Run-shading dispatch flag (see rasterizeFanRecordRuns): computed at
        // emission, where the draw's setup and the record's window are both
        // known; the rasterization loops read only the flag.
        record.shadeRuns = recordRunsEligible(setup, record);
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
// The GI ambient walk the tile cache and the per-pixel path amortize (see
// the header declaration): gather field first, probe-grid walk as the
// fallback. Pure read of the prepared snapshots, safe to call concurrently.
bool HeadlessBackend::resolveGiAmbient(const glm::vec3 &position, const glm::vec3 &normal, glm::vec3 &outIrradiance,
    float &outCoverage, float &outOcclusion) const
{
    if (preparedGatherField_.valid)
    {
        glm::vec3 gatherIrradiance(0.0f);
        float     gatherCoverage = 0.0f;
        float     gatherOcclusion = 1.0f;
        if (sampleGatherField(preparedGatherField_, position, gatherIrradiance, gatherCoverage, gatherOcclusion))
        {
            outIrradiance = gatherIrradiance;
            outCoverage   = gatherCoverage;
            outOcclusion  = gatherOcclusion;
            return true;
        }
    }
    outOcclusion = 1.0f;
    return giProbeGridIrradiance(preparedProbeGrid_, position, normal, outIrradiance, outCoverage);
}

glm::vec3 HeadlessBackend::shadeTrianglePixel(const RasterSetup &setup, const TileBinRecord &record, float weightA,
    float weightB, float inverseWA, float inverseWB, float inverseWC, bool *outFieldTraced,
    const GiAmbientSample *giSample, const glm::vec3 *interpolatedPair, const glm::vec3 *geometryNormalPair) const
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
    glm::vec3         sunRadiance             = sunRadiance_;
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
        // The interpolated surface pair, when the caller already computed it
        // (the tile cache's prologue needs the same values for its gates), is
        // passed in instead of being re-derived: the caller evaluates the
        // exact expressions below, so the values are identical.
        const glm::vec3 interpolatedPosition = interpolatedPair != nullptr
            ? *interpolatedPair
            : correctedA * cornerA.worldPosition + correctedB * cornerB.worldPosition
                + correctedC * cornerC.worldPosition;
        const glm::vec3 geometryNormal = geometryNormalPair != nullptr
            ? *geometryNormalPair
            : glm::normalize(correctedA * cornerA.worldNormal + correctedB * cornerB.worldNormal
                + correctedC * cornerC.worldNormal);

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

        // Sun shadow map (sun_shadows.hpp): the sun's diffuse and specular
        // terms scale by the percentage-closer visibility of this point.
        // The local sunRadiance copy the caller value-caches takes the
        // factor, so pbrShade's math stays untouched.
        if (sunShadowMap_.valid && sunRadiance.x + sunRadiance.y + sunRadiance.z > 0.0f)
        {
            sunRadiance *= sunShadowVisibility(interpolatedPosition, normal);
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
        float occlusion
            = hasOcclusion
                  ? 1.0f
                        - occlusionStrength * (1.0f - occlusionMap->sampleTrilinear(interpolatedUv.x, interpolatedUv.y, uvDerivativeX, uvDerivativeY).r)
                  : 1.0f;
        // Zenith sky visibility (sun_shadows.hpp): the HEMISPHERIC ambient
        // is the sky's diffuse approximation, so what stands between this
        // point and the open sky scales it (the GI ambient - light bounced
        // from every direction - stays untouched; an emissive-lit interior
        // keeps its bounce, an open field loses its sky fill under props).
        glm::vec3 ambientSkySky    = ambientSky;
        glm::vec3 ambientGroundSky = ambientGround;
        if (skyShadowMap_.valid && normal.y > 0.0f)
        {
            const float skyVisible = skyShadowVisibility(interpolatedPosition, normal);
            ambientSkySky *= skyVisible;
            ambientGroundSky *= skyVisible;
        }
        // Gather field (radiance_cache.gather hook): the final gather's
        // per-pixel irradiance replaces the probe-walk ambient wherever the
        // field covers the position - the contact-scale structure (short-
        // range occlusion, caustic irradiance) the coarse probe lattice
        // cannot resolve. pbrShade consumes it as a GiAmbientSample; the
        // probe-walk grid path stays the fallback for refused positions.
        GiAmbientSample gatherSample;
        const GiAmbientSample *effectiveGiSample = giSample;
        if (effectiveGiSample == nullptr && preparedGatherField_.valid
            && sampleGatherField(preparedGatherField_, interpolatedPosition, gatherSample.irradiance,
                gatherSample.coverage, gatherSample.occlusion))
        {
            gatherSample.valid = true;
            effectiveGiSample  = &gatherSample;
        }
        shaded = pbrShade(normal,
            interpolatedPosition,
            cameraPosition,
            lightDirection,
            sunRadiance,
            ambientSkySky,
            ambientGroundSky,
            baseColor,
            metallic,
            roughness,
            occlusion,
            lights,
            lightCount,
            effectiveGiSample == nullptr && preparedProbeGrid_.valid ? &preparedProbeGrid_ : nullptr,
            setup.opacity,
            setup.sheenFactor,
            effectiveGiSample);

        // Indirect specular (reflection_field field hook): the field texel
        // this pixel projects into carries the expected GGX-lobe response of
        // its material, so the add is the whole composite. Linear space, per
        // the module's estimator contract; tone mapping happens below. The
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
// Run-shading dispatch predicate: the draw's shading path fits the 4-wide
// run kernel (no normal map, no cloth sheen, no direct lights, and either an
// probe grid included - the lane loop resolves each lane's ambient sample
// through the tile cache, so the walk amortizes exactly like the per-sample
// path instead of dropping to the hemispheric ambient), and the record's
// framebuffer window is wide enough to amortize a
// run (narrow records keep the per-sample core; their lanes mostly reject
// and the run bookkeeping would cost more than the SoA pass saves).
// Alpha-cutout draws FIT the kernel: the lane loop fetches each lane's texel
// alpha and drops masked lanes exactly like the per-sample core's discard -
// foliage canopies shade four samples per pass instead of one.
// Evaluated once per record at emission (the shadeRuns flag), never inside
// the rasterization loops - an eligibility call per record measurably
// regressed the inlined per-sample core's codegen.
bool HeadlessBackend::recordRunsEligible(const RasterSetup &setup, const TileBinRecord &record) const
{
    // Textured transparent draws stay on the per-sample cores: their commit
    // modulates the blend by the base-color map's texel alpha, a per-lane
    // fetch the run kernel's shared commit does not carry.
    if (setup.fur || setup.sheenFactor > 0.0f
        || (setup.transparent && setup.hasBaseColorMap) || !directLights_.empty())
    {
        return false;
    }
    std::int32_t firstX = 0;
    std::int32_t lastX  = 0;
    std::int32_t firstY = 0;
    std::int32_t lastY  = 0;
    fanPixelWindow(record, framebufferWidth, framebufferHeight, firstX, lastX, firstY, lastY);
    return lastX - firstX + 1 >= 16;
}

bool HeadlessBackend::rasterizeFanRecord(const RasterSetup &setup, const TileBinRecord &record,
    std::int32_t xBandBegin, std::int32_t xBandEnd, std::int32_t yBandBegin, std::int32_t yBandEnd,
    TileShadingCache *tiles)
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
    // Alpha cutout threshold, 0 when inactive (see the sequential core's
    // hoist): one compare per covered pixel. Runs-path records reach this
    // function only for narrow windows (recordRunsEligible routes wide ones
    // to the run kernel, whose lane loop carries its own check).
    const float alphaCutoffActive = (setup.alphaCutoff > 0.0f && !setup.transparent) ? setup.alphaCutoff : 0.0f;
    // The shaded pixel's reflection-field response flag (transmissive
    // commits read it; see the transparent commit below).
    bool fieldTraced = false;
    // Two-tier amortization of the probe-grid ambient (see GiTileCache): only
    // draws whose shading normal IS the interpolated geometry normal can
    // share a cached sample (a normal map perturbs it per pixel, a two-sided
    // draw flips it on back-facing fragments; unlit and fur draws never
    // consult the probe grid). The field cache applies to every lit draw:
    // its gates (roughness agreement, position tolerance) refuse any sample
    // the pixel's own material would answer differently.
    const bool giCacheEligible = tiles != nullptr && tiles->gi != nullptr && preparedProbeGrid_.valid
        && !setup.unlit && !setup.fur && !setup.hasNormalMap && !setup.doubleSided;
    GiTileCache *const tileCache = giCacheEligible ? tiles->gi : nullptr;
    // The interpolated surface pair is computed once per covered pixel and
    // shared by the cache resolve and the shading kernel (which would
    // otherwise re-derive the same perspective-correct values); the locals
    // live outside the scan loops so a passing pixel writes them in place.
    // Every lit draw under a probe grid needs the pair, cache-eligible or
    // not (a normal-mapped or two-sided draw still shades with the geometry
    // normal and world position).
    const bool surfacePairNeeded = preparedProbeGrid_.valid && !setup.unlit && !setup.fur;
    glm::vec3   pairPosition(0.0f);
    glm::vec3   pairNormal(0.0f, 1.0f, 0.0f);
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

                // Alpha cutout (MeshMaterial::alphaCutoff, mesh.frag's
                // discard mirror): a texel below the cutoff skips the
                // shading kernel and every commit, exactly like the
                // sequential core's check. The perspective-correct UV
                // mirrors the shading kernel's own interpolation.
                if (alphaCutoffActive)
                {
                    const float cutInverseW = weightA * inverseWA + weightB * inverseWB + weightC * inverseWC;
                    const float cutW = 1.0f / cutInverseW;
                    const float cutA = weightA * inverseWA * cutW;
                    const float cutB = weightB * inverseWB * cutW;
                    const float cutC = 1.0f - cutA - cutB;
                    const glm::vec2 cutUv = cutA * record.corners[0].uv + cutB * record.corners[1].uv
                        + cutC * record.corners[2].uv;
                    if (setup.baseColorMap->sampleAlpha(cutUv.x, cutUv.y) < alphaCutoffActive)
                    {
                        weightA += weightADx;
                        weightB += weightBDx;
                        pixelDepth += depthADx;
                        continue;
                    }
                }

                // Perspective-correct attributes are interpolated inside the
                // shading kernel, which returns the tone-mapped linear color;
                // the commit folds the sRGB encode and quantization into one
                // packed word. The pre-shade depth test above is the early-out;
                // the commit reuses it directly: within one record scan (and
                // one tile) the calling thread is the only depth writer, so the
                // pixel's depth still holds the value that test read.
                //
                // Quad-amortized probe-grid ambient: interpolate this pixel's
                // surface pair (the same perspective-correct math the shading
                // kernel runs) and either reuse the quad's cached walk or
                // evaluate and publish it. The gates - same plane (the normal
                // projection of the position delta stays small) and a
                // near-identical normal - are what the walk's own inputs vary
                // smoothly over, so a passing neighbor returns the sample the
                // per-pixel walk would have produced to within the gates'
                // tolerance. The interpolated surface pair computed here is
                // the same pair the shading kernel needs (the exact
                // perspective-correct expressions), so it is passed through
                // instead of being re-derived per pixel.
                GiAmbientSample         quadSample;
                const GiAmbientSample  *giSample        = nullptr;
                const glm::vec3        *positionHint    = nullptr;
                const glm::vec3        *geometryHint    = nullptr;
                if (surfacePairNeeded)
                {
                    const float inverseWSum = weightA * inverseWA + weightB * inverseWB + weightC * inverseWC;
                    const float inversePerspective = 1.0f / inverseWSum;
                    const float correctedA         = weightA * inverseWA * inversePerspective;
                    const float correctedB         = weightB * inverseWB * inversePerspective;
                    const float correctedC         = 1.0f - correctedA - correctedB;
                    const RasterVertex &cornerA    = record.corners[0];
                    const RasterVertex &cornerB    = record.corners[1];
                    const RasterVertex &cornerC    = record.corners[2];
                    pairPosition = correctedA * cornerA.worldPosition + correctedB * cornerB.worldPosition
                                 + correctedC * cornerC.worldPosition;
                    const glm::vec3 pixelNormalRaw = correctedA * cornerA.worldNormal
                                                    + correctedB * cornerB.worldNormal
                                                    + correctedC * cornerC.worldNormal;
                    const float     pixelNormalLength = glm::length(pixelNormalRaw);
                    pairNormal = pixelNormalLength > 1.0e-8f ? pixelNormalRaw / pixelNormalLength
                                                             : glm::vec3(0.0f, 1.0f, 0.0f);
                    positionHint = &pairPosition;
                    geometryHint = &pairNormal;
                    if (tileCache != nullptr)
                    {
                        quadSample = tileCache->resolve(
                            x - xBandBegin, y - yBandBegin, tiles->recordTag, pairPosition, pairNormal,
                            [this](const glm::vec3 &walkPosition, const glm::vec3 &walkNormal, glm::vec3 &outIrradiance,
                                float &outCoverage, float &outOcclusion) -> bool
                            {
                                return resolveGiAmbient(walkPosition, walkNormal, outIrradiance, outCoverage,
                                    outOcclusion);
                            });
                        giSample = &quadSample;
                    }
                }
                const glm::vec3 shaded
                    = shadeTrianglePixel(setup, record, weightA, weightB, inverseWA, inverseWB, inverseWC,
                        &fieldTraced, giSample, positionHint, geometryHint);

                if (setup.transparent)
                {
                    // Perspective-correct UV for the texel-alpha modulation
                    // the commit below applies (the same interpolation the
                    // shading kernel runs; computed only for transparent
                    // textured draws).
                    const float transparentInverseW
                        = weightA * inverseWA + weightB * inverseWB + weightC * inverseWC;
                    const float transparentW = 1.0f / transparentInverseW;
                    const float transparentA = weightA * inverseWA * transparentW;
                    const float transparentB = weightB * inverseWB * transparentW;
                    const float transparentC = 1.0f - transparentA - transparentB;
                    const glm::vec2 transparentUv = transparentA * record.corners[0].uv
                        + transparentB * record.corners[1].uv + transparentC * record.corners[2].uv;
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
                        // framebuffer, with the material's opacity TIMES the
                        // base-color map's texel alpha (the sequential core's
                        // rule and mesh.frag's oColor.a twin). Tile ownership
                        // keeps the per-pixel order equal to the submission
                        // order, so co-transparent surfaces stack far-to-near
                        // exactly as the sequential path blends them.
                        const float texelAlpha = setup.hasBaseColorMap
                            ? setup.baseColorMap->sampleAlpha(transparentUv.x, transparentUv.y)
                            : 1.0f;
                        const float effectiveOpacity = setup.opacity * texelAlpha;
                        if (effectiveOpacity > 0.0039f)
                        {
                            const glm::vec3 encoded = fastSrgbEncode(glm::max(shaded, glm::vec3(0.0f)));
                            const float     oneMinus = 1.0f - effectiveOpacity;
                            pixel[0] = toByte(encoded.x * effectiveOpacity + static_cast<float>(pixel[0]) / 255.0f * oneMinus);
                            pixel[1] = toByte(encoded.y * effectiveOpacity + static_cast<float>(pixel[1]) / 255.0f * oneMinus);
                            pixel[2] = toByte(encoded.z * effectiveOpacity + static_cast<float>(pixel[2]) / 255.0f * oneMinus);
                            pixel[3] = 255;
                        }
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


bool HeadlessBackend::rasterizeFanRecordRuns(const RasterSetup &setup, const TileBinRecord &record,
    std::int32_t xBandBegin, std::int32_t xBandEnd, std::int32_t yBandBegin, std::int32_t yBandEnd,
    TileShadingCache *tiles)
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
    // Alpha cutout threshold, 0 when inactive (see the sequential core's
    // hoist): the lane loop compares one fetched texel alpha per accepted
    // lane, and the leftover-column tail keeps the per-sample core's check.
    const float alphaCutoffActive = (setup.alphaCutoff > 0.0f && !setup.transparent) ? setup.alphaCutoff : 0.0f;
    // The shaded pixel's reflection-field response flag (transmissive
    // commits read it; see the transparent commit below).
    bool fieldTraced = false;
    // Two-tier amortization of the probe-grid ambient (see GiTileCache): only
    // draws whose shading normal IS the interpolated geometry normal can
    // share a cached sample (a normal map perturbs it per pixel, a two-sided
    // draw flips it on back-facing fragments; unlit and fur draws never
    // consult the probe grid). The field cache applies to every lit draw:
    // its gates (roughness agreement, position tolerance) refuse any sample
    // the pixel's own material would answer differently.
    const bool giCacheEligible = tiles != nullptr && tiles->gi != nullptr && preparedProbeGrid_.valid
        && !setup.unlit && !setup.fur && !setup.hasNormalMap && !setup.doubleSided;
    GiTileCache *const tileCache = giCacheEligible ? tiles->gi : nullptr;
    // The interpolated surface pair is computed once per covered pixel and
    // shared by the cache resolve and the shading kernel (which would
    // otherwise re-derive the same perspective-correct values); the locals
    // live outside the scan loops so a passing pixel writes them in place.
    // Every lit draw under a probe grid needs the pair, cache-eligible or
    // not (a normal-mapped or two-sided draw still shades with the geometry
    // normal and world position).
    const bool surfacePairNeeded = preparedProbeGrid_.valid && !setup.unlit && !setup.fur;
    glm::vec3   pairPosition(0.0f);
    glm::vec3   pairNormal(0.0f, 1.0f, 0.0f);

    // 4-wide shading runs (see quad_shading.hpp): the run kernel models the
    // whole !unlit shading path for draws without direct lights, cloth sheen,
    // or an unamortized probe-grid walk, and shades four horizontally
    // consecutive samples of one record together in a single SoA pass -
    // normal-mapped draws included, with the caller running the per-lane TBN
    // chain (the fetch needs per-sample UVs) and the kernel lighting the
    // finished normals. Alpha-cutout draws join the same passes: the lane
    // loop fetches each lane's texel alpha first and drops masked lanes, so
    // a canopy shades only its surviving lanes. What the kernel does not
    // model (the albedo and map fetches themselves, the reflection field,
    // emissive, tone mapping, the commits) stays per sample with the caller.
    const RasterVertex &cornerA = record.corners[0];
    const RasterVertex &cornerB = record.corners[1];
    const RasterVertex &cornerC = record.corners[2];
    const glm::vec3    runPositions[3] = {cornerA.worldPosition, cornerB.worldPosition, cornerC.worldPosition};
    const glm::vec3    runNormals[3]    = {cornerA.worldNormal, cornerB.worldNormal, cornerC.worldNormal};
    const glm::vec3    runColors[3]     = {cornerA.color, cornerB.color, cornerC.color};
    // The dispatcher (recordRunsEligible) guarantees this variant only sees
    // records whose draws the run kernel models and whose windows amortize a
    // 4-wide run.
    const bool hasBaseColorMap = setup.hasBaseColorMap;
    // Normal-mapped lit lanes fetch the map and build the TBN in the run loop
    // (shadeTrianglePixel's exact normal chain, per lane); the kernel then
    // lights the finished normals. Unlit lanes never fetch (the per-sample
    // path only reads the map inside !unlit).
    const bool normalMapped = setup.hasNormalMap && !setup.unlit;
    const HeadlessTexture *const normalMap = setup.normalMap;
    const float normalScale = setup.normalScale;
    const glm::vec3 tangentScaled = record.tangent * record.tangentInverseMax;
    const glm::vec3 bitangentScaled = record.bitangent * record.tangentInverseMax;
    const bool hasMetallicRoughnessMap = setup.hasMetallicRoughnessMap;
    const bool hasOcclusion = setup.hasOcclusion;
    const bool hasEmissiveMap = setup.hasEmissiveMap;
    const bool anySampledMap = hasBaseColorMap || normalMapped || hasMetallicRoughnessMap || hasOcclusion || hasEmissiveMap;
    shadingrun::RunContext runContext;
    runContext.cameraPosition  = cameraPosition_;
    runContext.lightDirection  = lightDirection_;
    runContext.sunRadiance     = sunRadiance_;
    runContext.ambientSky      = ambientSky_;
    runContext.ambientGround   = ambientGround_;
    runContext.baseColorFactor = setup.baseColorFactor;
    runContext.opacity         = setup.opacity;
    runContext.sheenFactor     = setup.sheenFactor;
    runContext.doubleSided     = setup.doubleSided;
    runContext.albedoFilled    = hasBaseColorMap;
    runContext.unlit           = setup.unlit;
    runContext.normalMapped    = normalMapped;
    runContext.exposure        = environment_.exposure;
    runContext.toneMapping     = environment_.toneMapping;
    // The reflection-field response and emissive add in LINEAR space (the
    // per-sample path applies them before the output transform), so the run
    // defers the transform to the caller whenever those paths are live.
    runContext.toneMapInside = !(setup.hasEmissive || (preparedReflectionField_.valid && !setup.unlit));

    // The commit both paths share: transparent draws blend over the
    // framebuffer, opaque draws write depth and the packed color. The depth
    // value is the caller's interpolated sample depth.
    const auto commitSample = [&](const glm::vec3 &shaded, float sampleDepth, std::size_t index, bool sampleFieldTraced)
    {
        if (setup.transparent)
        {
            std::uint8_t *pixel = colorBuffer + index * 4;
            if (sampleFieldTraced)
            {
                // Transmissive commit: the reflection field's response at
                // this pixel already contains what the surface shows of the
                // scene behind it (the reflected share off the interface plus
                // the transmitted share refracted through it), so the shaded
                // result commits directly - blending the raw background in
                // as well would show the scene behind twice. Depth writes
                // stay off: co-transmissive surfaces stack through the
                // field's chains, not through occlusion.
                *reinterpret_cast<std::uint32_t *>(pixel) = encodeColorRgba(shaded);
            }
            else
            {
                // No traced response here (the surface is outside the traced
                // roughness band, or the reflection module is absent): the
                // classic alpha blend over the framebuffer. Tile ownership
                // keeps the per-pixel order equal to the submission order, so
                // co-transparent surfaces stack far-to-near exactly as the
                // sequential path blends them.
                const glm::vec3 encoded = fastSrgbEncode(glm::max(shaded, glm::vec3(0.0f)));
                const float     oneMinus = 1.0f - setup.opacity;
                pixel[0] = toByte(encoded.x * setup.opacity + static_cast<float>(pixel[0]) / 255.0f * oneMinus);
                pixel[1] = toByte(encoded.y * setup.opacity + static_cast<float>(pixel[1]) / 255.0f * oneMinus);
                pixel[2] = toByte(encoded.z * setup.opacity + static_cast<float>(pixel[2]) / 255.0f * oneMinus);
                pixel[3] = 255;
            }
        }
        else
        {
            if (writeDepth)
            {
                depthBuffer[index] = sampleDepth;
            }
            *reinterpret_cast<std::uint32_t *>(colorBuffer + index * 4) = encodeColorRgba(shaded);
        }
        wrote = true;
    };

    // The run path's per-lane epilogue: reflection field, emissive, tone map,
    // commit (the per-sample path's tail, per lane).
    const auto commitRunLane = [&](const glm::vec3 &shaded, float sampleDepth, std::size_t index,
        const float laneWeightA_, const float laneWeightB_, const float runRoughness, const glm::vec2 &laneUv,
        const glm::vec2 &laneUvDerivativeX, const glm::vec2 &laneUvDerivativeY)
    {
        glm::vec3 shadedOut = shaded;
        bool      sampleFieldTraced = false;
        if (preparedReflectionField_.valid && !setup.unlit)
        {
            // The reflection field's add is the whole composite (see the
            // per-sample path); the lane's perspective-correct position feeds
            // the field sample.
            const float weightC = 1.0f - laneWeightA_ - laneWeightB_;
            const float inverseWSum = laneWeightA_ * inverseWA + laneWeightB_ * inverseWB + weightC * inverseWC;
            const float inversePerspective = 1.0f / inverseWSum;
            const float correctedA = laneWeightA_ * inverseWA * inversePerspective;
            const float correctedB = laneWeightB_ * inverseWB * inversePerspective;
            const float correctedC = 1.0f - correctedA - correctedB;
            const glm::vec3 pixelPosition = correctedA * cornerA.worldPosition + correctedB * cornerB.worldPosition
                + correctedC * cornerC.worldPosition;
            const float roughnessClamped = std::clamp(runRoughness, minimumRoughness, 1.0f);
            bool traced = false;
            shadedOut += sampleReflectionField(preparedReflectionField_, pixelPosition, roughnessClamped, &traced);
            sampleFieldTraced = traced;
        }
        if (setup.hasEmissive)
        {
            if (hasEmissiveMap)
            {
                shadedOut += setup.emissiveColor
                    * setup.emissiveMap->sampleTrilinear(laneUv.x, laneUv.y, laneUvDerivativeX, laneUvDerivativeY);
            }
            else
            {
                shadedOut += setup.emissiveColor;
            }
        }
        // The run kernel already applied the output transform (exposure +
        // tone map) unless it deferred it behind the linear-space adds above.
        if (!runContext.toneMapInside)
        {
            shadedOut = toneMapColor(shadedOut, environment_.exposure, environment_.toneMapping);
        }
        commitSample(shadedOut, sampleDepth, index, sampleFieldTraced);
    };

    // Scratch for the 4-wide runs below, hoisted out of the scan loops: the
    // struct's zero-initialization pays once per record instead of once per
    // run (the shadeNormal fields tripled its footprint), and every field an
    // accepted lane reads is still written fresh in the lane loop - giValid
    // is the one field the old per-run zero-init used to clear for lanes the
    // quad-cache block does not touch, so the lane loop clears it
    // explicitly; discarded lanes carry garbage the committed outputs never
    // read.
    shadingrun::RunInputs run{};
    glm::vec2             laneUv[4];
    glm::vec2             laneUvDerivativeX[4];
    glm::vec2             laneUvDerivativeY[4];
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

        {
            // Runs of four horizontally consecutive samples; leftover columns
            // fall through to the per-sample loop below. The lane weights and
            // depths step through the same incremental sequence the per-sample
            // loop's tail stepping produces, so a lane sees exactly the value
            // the per-sample scan would compute for it.
            const std::int32_t runEnd = firstX + static_cast<std::int32_t>((lastX - firstX + 1) & ~3);
            for (std::int32_t x = firstX; x < runEnd; x += 4, pixelIndex += 4)
            {
                float laneWeightA[4];
                float laneWeightB[4];
                float laneDepth[4];
                float stepWeightA = weightA;
                float stepWeightB = weightB;
                float stepDepth   = pixelDepth;
                for (int lane = 0; lane < 4; ++lane)
                {
                    laneWeightA[lane] = stepWeightA;
                    laneWeightB[lane] = stepWeightB;
                    laneDepth[lane]    = stepDepth;
                    stepWeightA += weightADx;
                    stepWeightB += weightBDx;
                    stepDepth   += depthADx;
                }
                bool accepted[4] = {false, false, false, false};
                bool anyAccepted = false;
                for (int lane = 0; lane < 4; ++lane)
                {
                    const float weightC = 1.0f - laneWeightA[lane] - laneWeightB[lane];
                    if (weightC >= 0.0f && laneWeightA[lane] >= 0.0f && laneWeightB[lane] >= 0.0f
                        && laneDepth[lane] < depthBuffer[pixelIndex + static_cast<std::size_t>(lane)])
                    {
                        accepted[lane] = true;
                        anyAccepted    = true;
                    }
                }
                if (!anyAccepted)
                {
                    weightA    = stepWeightA;
                    weightB    = stepWeightB;
                    pixelDepth = stepDepth;
                    continue;
                }

                bool anyShaded = false;
                for (int lane = 0; lane < 4; ++lane)
                {
                    run.active[lane] = accepted[lane];
                    if (!accepted[lane])
                    {
                        continue;
                    }
                    // Fresh per run: the hoisted scratch keeps the previous
                    // run's probe-grid verdict otherwise (see the scratch
                    // hoist comment above).
                    run.giValid[lane] = false;
                    run.giOcclusion[lane] = 1.0f;
                    const float weightC = 1.0f - laneWeightA[lane] - laneWeightB[lane];
                    const float inverseWSum = laneWeightA[lane] * inverseWA + laneWeightB[lane] * inverseWB
                                             + weightC * inverseWC;
                    const float inversePerspective = 1.0f / inverseWSum;
                    const float correctedA = laneWeightA[lane] * inverseWA * inversePerspective;
                    const float correctedB = laneWeightB[lane] * inverseWB * inversePerspective;
                    const float correctedC = 1.0f - correctedA - correctedB;
                    run.correctedA[lane] = correctedA;
                    run.correctedB[lane] = correctedB;
                    run.correctedC[lane] = correctedC;

                    // Alpha cutout (MeshMaterial::alphaCutoff, mesh.frag's
                    // discard mirror): a texel below the cutoff drops the
                    // lane before any other fetch - no albedo read, no map
                    // chain, no shading, no commit, exactly the per-sample
                    // core's skip. The perspective-correct UV mirrors the
                    // shading kernel's own interpolation.
                    if (alphaCutoffActive)
                    {
                        const glm::vec2 cutUv = correctedA * cornerA.uv + correctedB * cornerB.uv
                            + correctedC * cornerC.uv;
                        if (setup.baseColorMap->sampleAlpha(cutUv.x, cutUv.y) < alphaCutoffActive)
                        {
                            run.active[lane] = false;
                            continue;
                        }
                    }
                    anyShaded = true;

                    float metallic  = setup.metallicFactor;
                    float roughness = setup.roughnessFactor;
                    float occlusion = 1.0f;
                    if (anySampledMap)
                    {
                        laneUv[lane] = correctedA * cornerA.uv + correctedB * cornerB.uv + correctedC * cornerC.uv;
                        // Per-lane UV derivatives from the record's affine
                        // gradients (the quotient rule; see the per-sample
                        // path).
                        laneUvDerivativeX[lane] = glm::vec2(record.uvNumeratorX[0] - laneUv[lane].x * record.uvDenominatorX,
                            record.uvNumeratorX[1] - laneUv[lane].y * record.uvDenominatorX) * inversePerspective;
                        laneUvDerivativeY[lane] = glm::vec2(record.uvNumeratorY[0] - laneUv[lane].x * record.uvDenominatorY,
                            record.uvNumeratorY[1] - laneUv[lane].y * record.uvDenominatorY) * inversePerspective;
                        if (hasBaseColorMap)
                        {
                            const glm::vec3 interpolatedColor = correctedA * cornerA.color + correctedB * cornerB.color
                                + correctedC * cornerC.color;
                            const glm::vec3 textured = interpolatedColor * setup.baseColorFactor
                                * setup.baseColorMap->sampleTrilinear(laneUv[lane].x, laneUv[lane].y,
                                    laneUvDerivativeX[lane], laneUvDerivativeY[lane]);
                            run.albedo[0][lane] = textured.r;
                            run.albedo[1][lane] = textured.g;
                            run.albedo[2][lane] = textured.b;
                        }
                        // Unlit draws stop at the albedo (the per-sample
                        // path's `if (!unlit)`): the material maps only load
                        // for lit lanes.
                        if (!setup.unlit && (hasMetallicRoughnessMap || hasOcclusion))
                        {
                            if (hasMetallicRoughnessMap)
                            {
                                const glm::vec3 sample = setup.metallicRoughnessMap->sampleTrilinear(laneUv[lane].x,
                                    laneUv[lane].y, laneUvDerivativeX[lane], laneUvDerivativeY[lane]);
                                metallic  *= sample.b;
                                roughness *= sample.g;
                            }
                            if (hasOcclusion)
                            {
                                occlusion = 1.0f - setup.occlusionStrength
                                    * (1.0f - setup.occlusionMap->sampleTrilinear(laneUv[lane].x, laneUv[lane].y,
                                                 laneUvDerivativeX[lane], laneUvDerivativeY[lane]).r);
                            }
                        }
                        // Normal-mapped lanes: the per-sample path's normal
                        // chain, per lane (shadeTrianglePixel's twin block -
                        // the exact expressions, so each lane's normal bits
                        // equal the value the per-sample scan computes). The
                        // kernel then lights the finished normals in SoA.
                        if (normalMapped)
                        {
                            const glm::vec3 interpolatedPosition = correctedA * cornerA.worldPosition
                                + correctedB * cornerB.worldPosition + correctedC * cornerC.worldPosition;
                            const glm::vec3 interpolatedNormal = correctedA * cornerA.worldNormal
                                + correctedB * cornerB.worldNormal + correctedC * cornerC.worldNormal;
                            const glm::vec3 geometryNormal = glm::normalize(interpolatedNormal);
                            glm::vec3       orientedNormal = geometryNormal;
                            if (setup.doubleSided)
                            {
                                const glm::vec3 viewForFlip = glm::normalize(cameraPosition_ - interpolatedPosition);
                                if (glm::dot(orientedNormal, viewForFlip) < 0.0f)
                                {
                                    orientedNormal = -orientedNormal;
                                }
                            }
                            const glm::vec3 normalMapSample = normalMap->sampleTrilinear(laneUv[lane].x,
                                laneUv[lane].y, laneUvDerivativeX[lane], laneUvDerivativeY[lane]) * 2.0f - 1.0f;
                            const glm::vec3 tangentNormal = glm::normalize(glm::vec3(normalMapSample.x * normalScale,
                                normalMapSample.y * normalScale,
                                normalMapSample.z));
                            // The (possibly flipped) oriented normal is the
                            // TBN's third axis, matching the per-sample path's
                            // flipped-geometry-normal frame.
                            const glm::mat3 tbn(tangentScaled, bitangentScaled, orientedNormal);
                            const glm::vec3 shadedNormal = glm::normalize(tbn * tangentNormal);
                            run.shadeNormal[0][lane] = shadedNormal.x;
                            run.shadeNormal[1][lane] = shadedNormal.y;
                            run.shadeNormal[2][lane] = shadedNormal.z;
                        }
                    }
                    run.metallic[lane]  = metallic;
                    run.roughness[lane] = roughness;
                    run.occlusion[lane] = occlusion;

                    // The lane's interpolated surface pair, shared by the
                    // sun-shadow visibility below and the tile-cache walk
                    // (the same perspective-correct math the shading kernel
                    // runs).
                    const bool laneNeedsSurfacePair
                        = ((sunShadowMap_.valid || skyShadowMap_.valid) && !setup.unlit) || tileCache != nullptr;
                    glm::vec3 pixelPosition(0.0f);
                    glm::vec3 pixelNormal(0.0f, 1.0f, 0.0f);
                    if (laneNeedsSurfacePair)
                    {
                        pixelPosition = correctedA * cornerA.worldPosition
                            + correctedB * cornerB.worldPosition + correctedC * cornerC.worldPosition;
                        const glm::vec3 pixelNormalRaw = correctedA * cornerA.worldNormal
                            + correctedB * cornerB.worldNormal + correctedC * cornerC.worldNormal;
                        const float     pixelNormalLength = glm::length(pixelNormalRaw);
                        pixelNormal = pixelNormalLength > 1.0e-8f
                            ? pixelNormalRaw / pixelNormalLength
                            : glm::vec3(0.0f, 1.0f, 0.0f);
                    }

                    // Sun shadow visibility (sun_shadows.hpp), per lane: the
                    // run kernel multiplies the sun weight by it, exactly
                    // where the per-sample path scales its sunRadiance copy.
                    // The zenith sky visibility (the hemispheric ambient's
                    // grounding factor) rides its own lane field.
                    if (!setup.unlit)
                    {
                        if (sunShadowMap_.valid)
                        {
                            run.sunVisibility[lane] = sunShadowVisibility(pixelPosition, pixelNormal);
                        }
                        if (skyShadowMap_.valid && pixelNormal.y > 0.0f)
                        {
                            run.skyVisibility[lane] = skyShadowVisibility(pixelPosition, pixelNormal);
                        }
                    }

                    // Probe-grid tile-cache sample: the per-sample path's
                    // resolve, per lane.
                    if (tileCache != nullptr)
                    {
                        const GiAmbientSample laneSample = tileCache->resolve(
                            x + lane - xBandBegin, y - yBandBegin, tiles->recordTag, pixelPosition, pixelNormal,
                            [this](const glm::vec3 &walkPosition, const glm::vec3 &walkNormal,
                                glm::vec3 &outIrradiance, float &outCoverage, float &outOcclusion) -> bool
                            {
                                return resolveGiAmbient(walkPosition, walkNormal, outIrradiance, outCoverage,
                                    outOcclusion);
                            });
                        run.giValid[lane]         = laneSample.valid;
                        run.giCoverage[lane]      = laneSample.coverage;
                        run.giOcclusion[lane]     = laneSample.occlusion;
                        run.giIrradiance[0][lane] = laneSample.irradiance.x;
                        run.giIrradiance[1][lane] = laneSample.irradiance.y;
                        run.giIrradiance[2][lane] = laneSample.irradiance.z;
                    }
                }

                // Every accepted lane can sit behind its own cutout mask; a
                // run whose lanes all dropped skips the kernel and commits
                // outright.
                if (anyShaded)
                {
                    glm::vec3 shadedRun[4];
                    shadingrun::shadeRun4(runContext, runPositions, runNormals, runColors, run, shadedRun);

                    for (int lane = 0; lane < 4; ++lane)
                    {
                        if (!run.active[lane])
                        {
                            continue;
                        }
                        commitRunLane(shadedRun[lane], laneDepth[lane], pixelIndex + static_cast<std::size_t>(lane),
                            laneWeightA[lane], laneWeightB[lane], run.roughness[lane], laneUv[lane],
                            laneUvDerivativeX[lane], laneUvDerivativeY[lane]);
                    }
                }

                weightA    = stepWeightA;
                weightB    = stepWeightB;
                pixelDepth = stepDepth;
            }
        }

        for (std::int32_t x = firstX + ((lastX - firstX + 1) & ~3); x <= lastX; ++x, ++pixelIndex)
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

                // Alpha cutout for the leftover columns (the lane loop above
                // cutout-tests its lanes; this tail keeps the per-sample
                // core's check so a record's tail pixels discard exactly
                // like its run pixels).
                if (alphaCutoffActive)
                {
                    const float cutInverseW = weightA * inverseWA + weightB * inverseWB + weightC * inverseWC;
                    const float cutW = 1.0f / cutInverseW;
                    const float cutA = weightA * inverseWA * cutW;
                    const float cutB = weightB * inverseWB * cutW;
                    const float cutC = 1.0f - cutA - cutB;
                    const glm::vec2 cutUv = cutA * record.corners[0].uv + cutB * record.corners[1].uv
                        + cutC * record.corners[2].uv;
                    if (setup.baseColorMap->sampleAlpha(cutUv.x, cutUv.y) < alphaCutoffActive)
                    {
                        weightA += weightADx;
                        weightB += weightBDx;
                        pixelDepth += depthADx;
                        continue;
                    }
                }

                // Perspective-correct attributes are interpolated inside the
                // shading kernel, which returns the tone-mapped linear color;
                // the commit folds the sRGB encode and quantization into one
                // packed word. The pre-shade depth test above is the early-out;
                // the commit reuses it directly: within one record scan (and
                // one tile) the calling thread is the only depth writer, so the
                // pixel's depth still holds the value that test read.
                //
                // Quad-amortized probe-grid ambient: interpolate this pixel's
                // surface pair (the same perspective-correct math the shading
                // kernel runs) and either reuse the quad's cached walk or
                // evaluate and publish it. The gates - same plane (the normal
                // projection of the position delta stays small) and a
                // near-identical normal - are what the walk's own inputs vary
                // smoothly over, so a passing neighbor returns the sample the
                // per-pixel walk would have produced to within the gates'
                // tolerance.
                GiAmbientSample         quadSample;
                const GiAmbientSample  *giSample     = nullptr;
                const glm::vec3        *positionHint = nullptr;
                const glm::vec3        *geometryHint = nullptr;
                if (surfacePairNeeded)
                {
                    const float inverseWSum = weightA * inverseWA + weightB * inverseWB + weightC * inverseWC;
                    const float inversePerspective = 1.0f / inverseWSum;
                    const float correctedA         = weightA * inverseWA * inversePerspective;
                    const float correctedB         = weightB * inverseWB * inversePerspective;
                    const float correctedC         = 1.0f - correctedA - correctedB;
                    pairPosition = correctedA * cornerA.worldPosition + correctedB * cornerB.worldPosition
                                 + correctedC * cornerC.worldPosition;
                    const glm::vec3 pixelNormalRaw = correctedA * cornerA.worldNormal
                                                    + correctedB * cornerB.worldNormal
                                                    + correctedC * cornerC.worldNormal;
                    const float     pixelNormalLength = glm::length(pixelNormalRaw);
                    pairNormal = pixelNormalLength > 1.0e-8f ? pixelNormalRaw / pixelNormalLength
                                                             : glm::vec3(0.0f, 1.0f, 0.0f);
                    positionHint = &pairPosition;
                    geometryHint = &pairNormal;
                    if (tileCache != nullptr)
                    {
                        quadSample = tileCache->resolve(
                            x - xBandBegin, y - yBandBegin, tiles->recordTag, pairPosition, pairNormal,
                            [this](const glm::vec3 &walkPosition, const glm::vec3 &walkNormal, glm::vec3 &outIrradiance,
                                float &outCoverage, float &outOcclusion) -> bool
                            {
                                return resolveGiAmbient(walkPosition, walkNormal, outIrradiance, outCoverage,
                                    outOcclusion);
                            });
                        giSample = &quadSample;
                    }
                }
                const glm::vec3 shaded
                    = shadeTrianglePixel(setup, record, weightA, weightB, inverseWA, inverseWB, inverseWC,
                        &fieldTraced, giSample, positionHint, geometryHint);

                if (setup.transparent)
                {
                    // Perspective-correct UV for the texel-alpha modulation
                    // the commit below applies (the same interpolation the
                    // shading kernel runs; computed only for transparent
                    // textured draws).
                    const float transparentInverseW
                        = weightA * inverseWA + weightB * inverseWB + weightC * inverseWC;
                    const float transparentW = 1.0f / transparentInverseW;
                    const float transparentA = weightA * inverseWA * transparentW;
                    const float transparentB = weightB * inverseWB * transparentW;
                    const float transparentC = 1.0f - transparentA - transparentB;
                    const glm::vec2 transparentUv = transparentA * record.corners[0].uv
                        + transparentB * record.corners[1].uv + transparentC * record.corners[2].uv;
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
                        // framebuffer, with the material's opacity TIMES the
                        // base-color map's texel alpha (the sequential core's
                        // rule and mesh.frag's oColor.a twin). Tile ownership
                        // keeps the per-pixel order equal to the submission
                        // order, so co-transparent surfaces stack far-to-near
                        // exactly as the sequential path blends them.
                        const float texelAlpha = setup.hasBaseColorMap
                            ? setup.baseColorMap->sampleAlpha(transparentUv.x, transparentUv.y)
                            : 1.0f;
                        const float effectiveOpacity = setup.opacity * texelAlpha;
                        if (effectiveOpacity > 0.0039f)
                        {
                            const glm::vec3 encoded = fastSrgbEncode(glm::max(shaded, glm::vec3(0.0f)));
                            const float     oneMinus = 1.0f - effectiveOpacity;
                            pixel[0] = toByte(encoded.x * effectiveOpacity + static_cast<float>(pixel[0]) / 255.0f * oneMinus);
                            pixel[1] = toByte(encoded.y * effectiveOpacity + static_cast<float>(pixel[1]) / 255.0f * oneMinus);
                            pixel[2] = toByte(encoded.z * effectiveOpacity + static_cast<float>(pixel[2]) / 255.0f * oneMinus);
                            pixel[3] = 255;
                        }
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
        // One probe-grid tile cache per worker thread, reset per owned tile:
        // the amortization is only valid within a tile's samples (see
        // GiTileCache), and concurrent tile owners must never share entries.
        // The thread-local keeps its capacity across tiles and frames, so a
        // steady-state tile costs one allocation-free clear.
        static thread_local GiTileCache tileCache;
        if (preparedProbeGrid_.valid)
        {
            tileCache.resetForTile(tileWidth_, tileHeight_);
        }
        // Run-shading dispatch (see rasterizeFanRecordRuns): wide records of
        // run-eligible draws shade in 4-wide SoA passes; every other record
        // (and every narrow window) keeps the shared per-sample core, whose
        // codegen this dispatch keeps pristine by keeping the run machinery
        // in its own outlined function.
        TileShadingCache tileShading;
        tileShading.gi = preparedProbeGrid_.valid ? &tileCache : nullptr;
        for (std::uint32_t entry = begin; entry < end; ++entry)
        {
            const TileBinRecord &record = tileRecords_[tileItems_[entry]];
            // The record's chunk index identifies its (single, continuous)
            // surface to the caches' tag checks.
            tileShading.recordTag = tileItems_[entry];
            const bool rasterized = record.shadeRuns
                ? rasterizeFanRecordRuns(setups[record.setupIndex], record, xBandBegin, xBandEnd, yBandBegin,
                    yBandEnd, &tileShading)
                : rasterizeFanRecord(setups[record.setupIndex], record, xBandBegin, xBandEnd, yBandBegin, yBandEnd,
                    &tileShading);
            if (rasterized)
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
