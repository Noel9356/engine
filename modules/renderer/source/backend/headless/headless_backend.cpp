// Headless backend implementation: a multithreaded CPU rasterizer.
//
// Pipeline per draw: prepareRasterization (culling, transform cache, texture
// resolution) then rasterizeTriangles (Sutherland-Hodgman-style near clip,
// scanline rasterization with perspective-correct interpolation, PBR shading
// via render_shared.hpp, tile-locked depth updates when parallel). Frames are
// blitted into the window module's locked surface pixels.

#include "headless_backend.hpp"
#include "tile_bins.hpp"
#include "engine/direct_lighting/capabilities.hpp"
#include "engine/kernel/task_scheduler.hpp"
#include "engine/renderer/capabilities.hpp"
#include "engine/renderer/material.hpp"
#include "engine/renderer/mesh.hpp"
#include "engine/window/capabilities.hpp"
#include "render_shared.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <glm/common.hpp>
#include <glm/exponential.hpp>
#include <glm/ext/matrix_float3x3.hpp>
#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/vector_float2.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/ext/vector_float4.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/type_ptr.hpp>
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
} // namespace

// CPU texture sampler: wrap or clamp UVs, then nearest or bilinear filter
// against the requested mip level (level 0 = the base image). mipData holds
// only the generated (downsampled) levels - level 0 reads pixelsRGB, level
// N>=1 reads mipData[N-1]. Returns white when empty.
glm::vec3 HeadlessTexture::sample(float u, float v, std::uint32_t mipLevel) const
{
    const float  *data;
    std::uint32_t mipW, mipH;
    if (mipLevel == 0 || mipData.empty() || mipLevel - 1u >= mipData.size())
    {
        data = pixelsRGB.data();
        mipW = width;
        mipH = height;
    }
    else
    {
        const std::uint32_t mipIndex = mipLevel - 1u;
        data = mipData[mipIndex].data();
        mipW = mipWidths[mipIndex];
        mipH = mipHeights[mipIndex];
    }
    if (mipW == 0 || mipH == 0 || data == nullptr)
        return glm::vec3(1.0f);
    if (mipW == 1 && mipH == 1)
        return glm::vec3(data[0], data[1], data[2]);
    if (address == TextureAddress::Repeat)
    {
        u -= std::floor(u);
        v -= std::floor(v);
    }
    else
    {
        u = std::clamp(u, 0.0f, 1.0f);
        v = std::clamp(v, 0.0f, 1.0f);
    }
    const float texelX    = u * static_cast<float>(mipW) - 0.5f;
    const float texelY    = v * static_cast<float>(mipH) - 0.5f;
    const auto  addrCoord = [&](std::int32_t cc, std::uint32_t e) -> std::uint32_t
    {
        const std::int32_t lim = static_cast<std::int32_t>(e);
        if (address == TextureAddress::Repeat)
        {
            const std::int32_t w = cc % lim;
            return static_cast<std::uint32_t>(w < 0 ? w + lim : w);
        }
        return static_cast<std::uint32_t>(std::clamp(cc, 0, lim - 1));
    };
    const auto pixelAt = [data, mipW](std::uint32_t x, std::uint32_t y) -> const float *
    { return data + (static_cast<std::size_t>(y) * mipW + x) * 3; };
    if (filter == TextureFilter::Nearest)
    {
        const auto   x = addrCoord(static_cast<std::int32_t>(std::floor(texelX + 0.5f)), mipW);
        const auto   y = addrCoord(static_cast<std::int32_t>(std::floor(texelY + 0.5f)), mipH);
        const float *p = pixelAt(x, y);
        return glm::vec3(p[0], p[1], p[2]);
    }
    const auto          x0i = static_cast<std::int32_t>(std::floor(texelX));
    const auto          y0i = static_cast<std::int32_t>(std::floor(texelY));
    const float         fx  = texelX - static_cast<float>(x0i);
    const float         fy  = texelY - static_cast<float>(y0i);
    const std::uint32_t x0 = addrCoord(x0i, mipW), x1 = addrCoord(x0i + 1, mipW);
    const std::uint32_t y0 = addrCoord(y0i, mipH), y1 = addrCoord(y0i + 1, mipH);
    const float        *p00 = pixelAt(x0, y0), *p10 = pixelAt(x1, y0), *p01 = pixelAt(x0, y1), *p11 = pixelAt(x1, y1);
    const float         omx = 1.0f - fx, omy = 1.0f - fy;
    const float         w00 = omx * omy, w10 = fx * omy, w01 = omx * fy, w11 = fx * fy;
    return glm::vec3(p00[0] * w00 + p10[0] * w10 + p01[0] * w01 + p11[0] * w11,
        p00[1] * w00 + p10[1] * w10 + p01[1] * w01 + p11[1] * w11,
        p00[2] * w00 + p10[2] * w10 + p01[2] * w01 + p11[2] * w11);
}

// Alpha-channel sample over the RGBA8 upload copy: same wrap/clamp, same
// nearest/bilinear filtering as sample() (mip level 0). The fur strand field
// lives in the fur texture's alpha channel (see shaders/fur.frag), so the
// headless fur path samples this instead of the RGB mip chain.
float HeadlessTexture::sampleAlpha(float u, float v) const
{
    if (width == 0 || height == 0 || pixels.size() < static_cast<std::size_t>(width) * height * 4)
        return 1.0f;
    if (width == 1 && height == 1)
        return static_cast<float>(pixels[3]) / 255.0f;
    if (address == TextureAddress::Repeat)
    {
        u -= std::floor(u);
        v -= std::floor(v);
    }
    else
    {
        u = std::clamp(u, 0.0f, 1.0f);
        v = std::clamp(v, 0.0f, 1.0f);
    }
    const float texelX = u * static_cast<float>(width) - 0.5f;
    const float texelY = v * static_cast<float>(height) - 0.5f;
    const auto  addrCoord = [&](std::int32_t cc, std::uint32_t e) -> std::uint32_t
    {
        const std::int32_t lim = static_cast<std::int32_t>(e);
        if (address == TextureAddress::Repeat)
        {
            const std::int32_t w = cc % lim;
            return static_cast<std::uint32_t>(w < 0 ? w + lim : w);
        }
        return static_cast<std::uint32_t>(std::clamp(cc, 0, lim - 1));
    };
    const auto alphaAt = [&](std::uint32_t x, std::uint32_t y) -> float
    { return static_cast<float>(pixels[(static_cast<std::size_t>(y) * width + x) * 4 + 3]) / 255.0f; };
    if (filter == TextureFilter::Nearest)
    {
        const auto x = addrCoord(static_cast<std::int32_t>(std::floor(texelX + 0.5f)), width);
        const auto y = addrCoord(static_cast<std::int32_t>(std::floor(texelY + 0.5f)), height);
        return alphaAt(x, y);
    }
    const auto          x0i = static_cast<std::int32_t>(std::floor(texelX));
    const auto          y0i = static_cast<std::int32_t>(std::floor(texelY));
    const float         fx  = texelX - static_cast<float>(x0i);
    const float         fy  = texelY - static_cast<float>(y0i);
    const std::uint32_t x0 = addrCoord(x0i, width), x1 = addrCoord(x0i + 1, width);
    const std::uint32_t y0 = addrCoord(y0i, height), y1 = addrCoord(y0i + 1, height);
    const float a00 = alphaAt(x0, y0), a10 = alphaAt(x1, y0), a01 = alphaAt(x0, y1), a11 = alphaAt(x1, y1);
    const float omx = 1.0f - fx, omy = 1.0f - fy;
    return a00 * omx * omy + a10 * fx * omy + a01 * omx * fy + a11 * fx * fy;
}

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
    std::fill(reinterpret_cast<std::uint32_t *>(color.data()),
        reinterpret_cast<std::uint32_t *>(color.data() + color.size()),
        clearWord);
    std::fill(depth.begin(), depth.end(), 1.0f);

    std::fill(pixelLocks.begin(), pixelLocks.end(), 0);
}

// Presents: copies the CPU raster into the window surface's locked pixel
// buffer. Matching full-surface strides (the common case) move the frame in
// one memcpy; mismatching strides fall back to a row-by-row copy.
void HeadlessBackend::endFrame()
{
    window::SurfacePixels pixels;
    if (!window_->lockSurfacePixels(window_->self, surface_, &pixels))
    {
        return;
    }
    const std::uint32_t rows    = std::min(framebufferHeight, pixels.height);
    const std::uint32_t columns = std::min(framebufferWidth, pixels.width);
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
            const std::size_t strides[] = {4, 16, 64};
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
    meshes.erase(mesh);
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
            if (description.width > 1 && description.height > 1)
            {
                // mipData holds only the generated (downsampled) levels, not
                // the base image - the sampler reads pixelsRGB for mip level 0
                // directly, so duplicating it in mipData[0] was wasted memory
                // (3 floats per pixel, e.g. ~12 MiB on a 1024x1024 texture).
                // The first downsample reads from pixelsRGB; subsequent ones
                // read from the previous mip in mipData.
                const std::vector<float> *prev = &texture.pixelsRGB;
                std::uint32_t             cw   = description.width, ch = description.height;
                while (cw > 1 || ch > 1)
                {
                    const std::uint32_t nw = std::max(1u, cw / 2), nh = std::max(1u, ch / 2);
                    std::vector<float>  nxt(static_cast<std::size_t>(nw) * nh * 3);
                    for (std::uint32_t y = 0; y < nh; ++y)
                        for (std::uint32_t x = 0; x < nw; ++x)
                        {
                            const std::uint32_t sx0 = x * 2, sy0 = y * 2, sx1 = std::min(sx0 + 1, cw - 1),
                                                sy1 = std::min(sy0 + 1, ch - 1);
                            for (int cc = 0; cc < 3; ++cc)
                                nxt[(y * nw + x) * 3 + cc]
                                    = ((*prev)[(sy0 * cw + sx0) * 3 + cc] + (*prev)[(sy0 * cw + sx1) * 3 + cc]
                                          + (*prev)[(sy1 * cw + sx0) * 3 + cc] + (*prev)[(sy1 * cw + sx1) * 3 + cc])
                                    * 0.25f;
                        }
                    texture.mipData.push_back(std::move(nxt));
                    texture.mipWidths.push_back(nw);
                    texture.mipHeights.push_back(nh);
                    prev = &texture.mipData.back();
                    cw = nw;
                    ch = nh;
                }
                texture.mipLevelCount = static_cast<std::uint32_t>(texture.mipData.size()) + 1u; // +1 for the base level
            }
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

    const std::vector<std::uint32_t> *activeIndices = &meshData.indices;
    DrawBounds                        bounds{glm::vec3(0.0f), 0.0f};
    bool                              haveBounds = false;
    if (lodEnabled_ && !meshData.lodIndices.empty() && meshData.boundsValid && cameraInitialized_)
    {
        const glm::mat4 model = glm::make_mat4(modelMatrix);
        bounds.center         = glm::vec3(model * glm::vec4(meshData.boundsCenter, 1.0f));
        const glm::vec3 basisX = glm::vec3(model[0]);
        const glm::vec3 basisY = glm::vec3(model[1]);
        const glm::vec3 basisZ = glm::vec3(model[2]);
        const float     worldScale
            = std::sqrt(glm::dot(basisX, basisX) + glm::dot(basisY, basisY) + glm::dot(basisZ, basisZ));
        bounds.radius = meshData.boundsRadius * worldScale;
        haveBounds    = true;
        const float dist        = glm::distance(cameraPosition_, bounds.center);
        const float angularSize = dist > 0.0f ? bounds.radius / dist : 1.0f; // ~screen fraction the mesh spans
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
                cornerClip[corner]              = cornerValid[corner] ? transformedVertices_[vertexIndex].clip : glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
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
    // Tile-parallel path (renderer.headless_tile_parallel, default
    // ON): triggers mirror the default paths exactly (big draws by triangle
    // count, few-triangle wide-coverage draws by projected extent), but the
    // execution bins triangles into 64x64 screen tiles with exclusive
    // per-worker ownership - no per-pixel locks. Falls back to the default
    // paths when disabled, the binning allocation fails, or the draw exceeds
    // the record cap.
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
                    [this, &meshData, &setup, index, workers]()
                    { rasterizeTileRecords(setup, index, workers, &rasterResultScratch_[index]); }))
            {
                rasterizeTileRecords(setup, index, workers, &rasterResultScratch_[index]);
            }
        }
        rasterizeTileRecords(setup, 0u, workers, &rasterResultScratch_[0]);
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

// Instance draws are just repeated drawMesh calls on the CPU backend (the
// transform cache still deduplicates the vertex shading).
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

    const std::size_t triCount = meshData.indices.size() / 3;
    statistics_.trianglesSubmitted += static_cast<std::uint64_t>(triCount) * layerCount;

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

    const float scaleX  = data.framebufferScale[0];
    const float scaleY  = data.framebufferScale[1];
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
                }
            }
            ++statistics_.trianglesSubmitted;
            ++statistics_.trianglesRasterized;
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

// Gathers everything rasterizeTriangles needs. Order matters: frustum cull,
// transform the vertices (through the generation-stamped cache when it fits
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
    outSetup.modelNormalMatrix = material.unlit ? glm::mat3(1.0f) : glm::mat3(outSetup.model);
    outSetup.unlit             = material.unlit;
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
        const float     worldRadius = outSetup.boundsWorldRadius;
        for (const glm::vec4 &plane : frustumPlanes_)
        {
            const float signedDistance = glm::dot(plane, worldCenter);
            if (std::isfinite(signedDistance) && signedDistance < -worldRadius)
            {
                return false;
            }
        }
    }

    const std::size_t vertexCount         = mesh.vertexCount();
    std::uint32_t     transformGeneration = transformGeneration_ + 1;
    // Per-vertex cache cost: the transformed data plus its generation stamp.
    constexpr std::size_t transformCacheEntryBytes = sizeof(TransformedVertex) + sizeof(std::uint32_t);
    const bool transformCacheWithinBudget          = transformCacheBudgetBytes_ >= transformCacheEntryBytes
                                                  && vertexCount <= transformCacheBudgetBytes_ / transformCacheEntryBytes;
    outSetup.vertexCount                           = vertexCount;
    outSetup.useTransformCache                     = false;
    if (triangleCount >= 2 && activeIndexCount > vertexCount && transformCacheWithinBudget)
    {
        try
        {
            if (transformedVertices_.size() < vertexCount)
            {
                transformedVertices_.resize(vertexCount);
            }
            if (transformedVertexStamps_.size() < vertexCount)
            {
                transformedVertexStamps_.resize(vertexCount, 0);
            }
            if (transformGeneration == 0)
            {
                std::fill(transformedVertexStamps_.begin(), transformedVertexStamps_.end(), 0);
                transformGeneration = 1;
            }
            transformGeneration_         = transformGeneration;
            outSetup.transformGeneration = transformGeneration;
            outSetup.useTransformCache   = true;

            const glm::mat4  &model             = outSetup.model;
            const glm::mat4  &viewProjection    = outSetup.viewProjection;
            const glm::mat3  &modelNormalMatrix = outSetup.modelNormalMatrix;
            const bool        unlit             = outSetup.unlit;
            const float      *posData           = mesh.positions.data();
            const float      *nrmData           = mesh.normals.data();
            const std::size_t hwWorkers         = parallelWorkerCount();
            const bool        parallelizeTransform
                = hwWorkers >= 2u && vertexCount >= 8192u && taskScheduler_ != nullptr && taskScheduler_->isRunning();
            auto transformRange = [&](std::size_t vStart, std::size_t vEnd)
            {
                for (std::size_t v = vStart; v < vEnd; ++v)
                {
                    // Stamp guard: vertices already transformed this draw (or
                    // this generation) are reused instead of recomputed.
                    if (transformedVertexStamps_[v] == transformGeneration)
                    {
                        continue;
                    }
                    const glm::vec3 localPos3(posData[v * 3], posData[v * 3 + 1], posData[v * 3 + 2]);
                    TransformedVertex &out = transformedVertices_[v];
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
                    transformedVertexStamps_[v] = transformGeneration;
                }
            };
            if (parallelizeTransform)
            {
                taskScheduler_->parallelFor(vertexCount, 8192u, transformRange);
            }
            else
            {
                transformRange(0, vertexCount);
            }
        }
        catch (const std::bad_alloc &)
        {
            outSetup.useTransformCache = false;
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
    if (backFaceCullingEnabled_ && !outSetup.unlit && outSetup.useTransformCache && cameraInitialized_
        && mesh.boundsValid && triangleCount >= 4)
    {
        ensureWorldBounds();
        const float cameraDist = glm::distance(cameraPosition_, outSetup.boundsWorldCenter);
        if (cameraDist > outSetup.boundsWorldRadius)
        {
            // Majority vote over a spread of triangles: face winding against
            // vertex normals identifies the outward orientation
            // camera-independently (screen-space signs cannot - front and
            // back faces balance on a closed mesh). A crease triangle whose
            // smooth corner normals lean across the edge loses the vote
            // instead of flipping the whole mesh.
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
                    const glm::vec3 &wp0 = transformedVertices_[i0].worldPosition;
                    const glm::vec3 &wp1 = transformedVertices_[i1].worldPosition;
                    const glm::vec3 &wp2 = transformedVertices_[i2].worldPosition;
                    const glm::vec3  faceNormal = glm::cross(wp1 - wp0, wp2 - wp0);
                    const float      fnLen2     = glm::dot(faceNormal, faceNormal);
                    if (fnLen2 < 1e-12f)
                        continue;
                    const glm::vec3 &wn0 = transformedVertices_[i0].worldNormal;
                    const glm::vec3 &wn1 = transformedVertices_[i1].worldNormal;
                    const glm::vec3 &wn2 = transformedVertices_[i2].worldNormal;
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
                // free slot (or slot 0 once every slot is taken).
                for (std::size_t slot = 0; slot < HeadlessMesh::WindingVoteMemo::capacity; ++slot)
                {
                    if (memo.sources[slot] == nullptr)
                    {
                        memo.sources[slot] = activeIndices;
                        memo.signs[slot]   = frontSign;
                        break;
                    }
                }
            }
            if (frontSign != 0.0f)
            {
                outSetup.frontSign          = frontSign;
                outSetup.enableBackFaceCull = true; // winding convention confirmed; culling is safe
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
    const bool        enableBackFaceCull = setup.enableBackFaceCull;
    const float       frontSign          = setup.frontSign;
    const bool        useTransformCache  = setup.useTransformCache;
    const std::size_t vertexCount        = setup.vertexCount;
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
                const TransformedVertex &transformed = transformedVertices_[vertexIndex];
                corners[corner].clip                 = transformed.clip;
                corners[corner].worldPosition        = transformed.worldPosition;
                corners[corner].worldNormal          = unlit ? glm::vec3(0.0f) : transformed.worldNormal;
                if (unlit)
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
                    corners[corner].worldPosition = glm::vec3(0.0f);
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

        ClipVertex    clipped[4];
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

        glm::vec3 screen[4];
        float     depth[4];
        float     clipW[4];
        for (std::uint32_t vertex = 0; vertex < clippedCount; ++vertex)
        {
            clipW[vertex]        = clipped[vertex].clip.w;
            const float inverseW = 1.0f / clipW[vertex];
            const float ndcX     = clipped[vertex].clip.x * inverseW;
            const float ndcY     = clipped[vertex].clip.y * inverseW;
            const float ndcZ     = clipped[vertex].clip.z * inverseW;
            screen[vertex]       = glm::vec3((ndcX * 0.5f + 0.5f) * static_cast<float>(framebufferWidth),
                (1.0f - (ndcY * 0.5f + 0.5f)) * static_cast<float>(framebufferHeight),
                0.0f);
            depth[vertex]        = ndcZ * 0.5f + 0.5f;
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
                        const glm::vec3 interpolatedColor = correctedA * clipped[cornerA].color
                                                          + correctedB * clipped[cornerB].color
                                                          + correctedC * clipped[cornerC].color;
                        glm::vec3       baseColor         = interpolatedColor * baseColorFactor;
                        if (hasBaseColorMap)
                        {
                            baseColor *= baseColorMap->sample(interpolatedUv.x, interpolatedUv.y);
                        }

                        // Shell-fur strand crop (fur.frag mirror): the layer
                        // alpha first - a fully transparent texel skips the
                        // shading exactly like the shader's discard. field =
                        // the fur texture's alpha (strand height); the neutral
                        // stand-in reports 1 (a fur-less field stays opaque).
                        float furPixelAlpha = 1.0f;
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
                            if (hasNormalMap)
                            {
                                const glm::vec3 normalMapSample
                                    = normalMap->sample(interpolatedUv.x, interpolatedUv.y) * 2.0f - 1.0f;
                                const glm::vec3 tangentNormal = glm::normalize(glm::vec3(normalMapSample.x * normalScale,
                                    normalMapSample.y * normalScale,
                                    normalMapSample.z));
                                const glm::mat3 tbn(
                                    tangentVector * tangentInverseMax, bitangentVector * tangentInverseMax,
                                    geometryNormal);
                                normal = glm::normalize(tbn * tangentNormal);
                            }

                            float metallic  = metallicFactor;
                            float roughness = roughnessFactor;
                            if (hasMetallicRoughnessMap)
                            {
                                const glm::vec3 metallicRoughnessSample
                                    = metallicRoughnessMap->sample(interpolatedUv.x, interpolatedUv.y);
                                metallic *= metallicRoughnessSample.b;
                                roughness *= metallicRoughnessSample.g;
                            }
                            metallic  = std::clamp(metallic, 0.0f, 1.0f);
                            roughness = std::clamp(roughness, minimumRoughness, 1.0f);
                            const float occlusion
                                = hasOcclusion
                                    ? 1.0f
                                          - occlusionStrength
                                                * (1.0f - occlusionMap->sample(interpolatedUv.x, interpolatedUv.y).r)
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
                                probeGrid_.valid() ? &probeGrid_ : nullptr);
                            }
                        }

                        if (hasEmissive)
                        {
                            if (hasEmissiveMap)
                            {
                                shaded += emissiveColor * emissiveMap->sample(interpolatedUv.x, interpolatedUv.y);
                            }
                            else
                            {
                                shaded += emissiveColor;
                            }
                        }
                        // Output transform: tone map, then the fast sRGB
                        // lookup (see fastSrgbEncode; the CPU raster always
                        // writes UNORM targets; see SRGB_OUTPUT in mesh.frag
                        // for the GPU-side switch).
                        shaded = toneMapColor(shaded, environment_.exposure, environment_.toneMapping);
                        shaded = fastSrgbEncode(glm::max(shaded, glm::vec3(0.0f)));

                        if (setup.fur)
                        {
                            // Shell-fur commit: alpha blend AFTER tone map +
                            // gamma encode - the same space the fur pipeline's
                            // fixed-function blending operates in on the UNORM
                            // target - and NO depth write (transparent layers
                            // must not occlude each other or later geometry).
                            std::uint8_t *pixel  = colorBuffer + pixelIndex * 4;
                            const float   oneMinus = 1.0f - furPixelAlpha;
                            pixel[0] = toByte(shaded.x * furPixelAlpha + static_cast<float>(pixel[0]) / 255.0f * oneMinus);
                            pixel[1] = toByte(shaded.y * furPixelAlpha + static_cast<float>(pixel[1]) / 255.0f * oneMinus);
                            pixel[2] = toByte(shaded.z * furPixelAlpha + static_cast<float>(pixel[2]) / 255.0f * oneMinus);
                            pixel[3] = 255;
                            rasterized = true;
                            weightA += weightADx;
                            weightB += weightBDx;
                            pixelDepth += depthADx;
                            continue;
                        }

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
                                depthBuffer[pixelIndex] = pixelDepth;
                                std::uint8_t *pixel     = colorBuffer + pixelIndex * 4;
                                pixel[0]                = toByte(shaded.x);
                                pixel[1]                = toByte(shaded.y);
                                pixel[2]                = toByte(shaded.z);
                                pixel[3]                = 255;
                                rasterized              = true;
                            }
                            lockRef.store(0, std::memory_order_release);
                        }
                        else
                        {
                            if (pixelDepth < depthBuffer[pixelIndex])
                            {
                                depthBuffer[pixelIndex] = pixelDepth;
                                std::uint8_t *pixel     = colorBuffer + pixelIndex * 4;
                                pixel[0]                = toByte(shaded.x);
                                pixel[1]                = toByte(shaded.y);
                                pixel[2]                = toByte(shaded.z);
                                pixel[3]                = 255;
                                rasterized              = true;
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
    const HeadlessMesh &mesh, const RasterSetup &setup, std::size_t triangleIndex, TileBinRecord (&outFans)[2]) const
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
    // Cache data pointer hoisted with the rest (the cache is read-only here).
    const TransformedVertex *const transformCache = transformedVertices_.data();

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
            if (unlit)
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
                corners[corner].worldPosition = glm::vec3(0.0f);
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

    RasterVertex  clipped[4];
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

    float screenX[4];
    float screenY[4];
    float depth[4];
    float clipW[4];
    float inverseW[4];
    for (std::uint32_t vertex = 0; vertex < clippedCount; ++vertex)
    {
        clipW[vertex]    = clipped[vertex].clip.w;
        inverseW[vertex] = 1.0f / clipW[vertex];
        const float ndcX = clipped[vertex].clip.x * inverseW[vertex];
        const float ndcY = clipped[vertex].clip.y * inverseW[vertex];
        const float ndcZ = clipped[vertex].clip.z * inverseW[vertex];
        screenX[vertex]  = (ndcX * 0.5f + 0.5f) * static_cast<float>(framebufferWidthLocal);
        screenY[vertex]  = (1.0f - (ndcY * 0.5f + 0.5f)) * static_cast<float>(framebufferHeightLocal);
        depth[vertex]    = ndcZ * 0.5f + 0.5f;
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
    float weightB, float inverseWA, float inverseWB, float inverseWC) const
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
    const glm::vec3 interpolatedColor = correctedA * cornerA.color + correctedB * cornerB.color
                                      + correctedC * cornerC.color;
    glm::vec3       baseColor         = interpolatedColor * baseColorFactor;
    if (hasBaseColorMap)
    {
        baseColor *= baseColorMap->sample(interpolatedUv.x, interpolatedUv.y);
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
        if (hasNormalMap)
        {
            const glm::vec3 normalMapSample = normalMap->sample(interpolatedUv.x, interpolatedUv.y) * 2.0f - 1.0f;
            const glm::vec3 tangentNormal = glm::normalize(glm::vec3(normalMapSample.x * normalScale,
                normalMapSample.y * normalScale,
                normalMapSample.z));
            const glm::mat3 tbn(tangentScaled, bitangentScaled, geometryNormal);
            normal = glm::normalize(tbn * tangentNormal);
        }

        float metallic  = metallicFactor;
        float roughness = roughnessFactor;
        if (hasMetallicRoughnessMap)
        {
            const glm::vec3 metallicRoughnessSample = metallicRoughnessMap->sample(interpolatedUv.x, interpolatedUv.y);
            metallic *= metallicRoughnessSample.b;
            roughness *= metallicRoughnessSample.g;
        }
        metallic  = std::clamp(metallic, 0.0f, 1.0f);
        roughness = std::clamp(roughness, minimumRoughness, 1.0f);
        const float occlusion
            = hasOcclusion
                  ? 1.0f
                        - occlusionStrength * (1.0f - occlusionMap->sample(interpolatedUv.x, interpolatedUv.y).r)
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
            probeGrid_.valid() ? &probeGrid_ : nullptr);
    }

    if (hasEmissive)
    {
        if (hasEmissiveMap)
        {
            shaded += emissiveColor * emissiveMap->sample(interpolatedUv.x, interpolatedUv.y);
        }
        else
        {
            shaded += emissiveColor;
        }
    }
    shaded = toneMapColor(shaded, environment_.exposure, environment_.toneMapping);
    shaded = fastSrgbEncode(glm::max(shaded, glm::vec3(0.0f)));
    return shaded;
}

// Shared rasterizer core. Scans the fan record's bounding box clamped to the
// inclusive pixel window [xBandBegin..xBandEnd] x [yBandBegin..yBandEnd],
// testing barycentric coverage with edge-function increments, depth-testing,
// shading through shadeTrianglePixel, and writing depth + colour. The
// sequential path passes the whole framebuffer; tile owners pass their tile
// (exclusive ownership means the writes need no locks). Returns whether at
// least one pixel was written.
bool HeadlessBackend::rasterizeFanRecord(const RasterSetup &setup, const TileBinRecord &record,
    std::int32_t xBandBegin, std::int32_t xBandEnd, std::int32_t yBandBegin, std::int32_t yBandEnd)
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
                // shading kernel; the depth re-check below mirrors the
                // previous write path exactly (the pre-shade test above is
                // the early-out, this is the commit).
                const glm::vec3 shaded
                    = shadeTrianglePixel(setup, record, weightA, weightB, inverseWA, inverseWB, inverseWC);

                if (pixelDepth < depthBuffer[pixelIndex])
                {
                    depthBuffer[pixelIndex] = pixelDepth;
                    std::uint8_t *pixel     = colorBuffer + pixelIndex * 4;
                    pixel[0]                = toByte(shaded.x);
                    pixel[1]                = toByte(shaded.y);
                    pixel[2]                = toByte(shaded.z);
                    pixel[3]                = 255;
                    wrote                   = true;
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
    // Worst case every triangle clips into a two-triangle fan. Draws that
    // could exceed the record cap fall back to the sequential path instead
    // of growing the persistent scratch without bound.
    if (triangleCount > maxTileRecords_ / 2u)
    {
        return false;
    }
    const tilebins::TileGrid grid = tilebins::makeGrid(framebufferWidth, framebufferHeight, tileWidth_, tileHeight_);
    if (tilebins::tileCount(grid) == 0)
    {
        return false;
    }

    try
    {
        tileRecords_.clear();
        tileRecords_.reserve(triangleCount * 2u);
        TileBinRecord fans[2]; // reused across triangles; see rasterizeTriangles
        for (std::size_t triangle = 0; triangle < triangleCount; ++triangle)
        {
            const std::size_t fanCount = prepareFanTriangles(mesh, setup, triangle, fans);
            for (std::size_t fan = 0; fan < fanCount; ++fan)
            {
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
void HeadlessBackend::rasterizeTileRecords(
    const RasterSetup &setup, std::uint32_t workerIndex, std::uint32_t workerCount, StripResult *outResult)
{
    if (outResult == nullptr || workerCount == 0u)
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
    for (std::uint32_t tile = workerIndex; tile < tiles; tile += workerCount)
    {
        const std::uint32_t begin = tileOffsets_[tile];
        const std::uint32_t end   = tileOffsets_[tile + 1u];
        if (begin == end)
        {
            continue;
        }
        const std::uint32_t tileX      = tile % grid.tilesX;
        const std::uint32_t tileY      = tile / grid.tilesX;
        const std::int32_t  xBandBegin = static_cast<std::int32_t>(tileX * tileWidth_);
        const std::int32_t  xBandEnd = static_cast<std::int32_t>(
            std::min<std::int64_t>(framebufferWidthInt, static_cast<std::int64_t>(xBandBegin) + tileWidth_) - 1);
        const std::int32_t yBandBegin = static_cast<std::int32_t>(tileY * tileHeight_);
        const std::int32_t yBandEnd = static_cast<std::int32_t>(
            std::min<std::int64_t>(framebufferHeightInt, static_cast<std::int64_t>(yBandBegin) + tileHeight_) - 1);
        for (std::uint32_t entry = begin; entry < end; ++entry)
        {
            const TileBinRecord &record = tileRecords_[tileItems_[entry]];
            if (rasterizeFanRecord(setup, record, xBandBegin, xBandEnd, yBandBegin, yBandEnd))
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
