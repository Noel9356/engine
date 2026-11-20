// Headless backend data types: CPU-side mesh and texture representations
// shared between the headless backend and its rasterizer core. Extracted from
// headless_backend.hpp so the rasterizer's inner types and the mesh/texture
// records can be reused without dragging in the full HeadlessBackend class.
//
// HeadlessMesh is a planar (SoA) layout for rasterizer locality, with
// bounding-sphere data for culling/LOD. lodIndices holds decimated index
// lists (every 4th/16th/64th triangle) generated at creation.
//
// HeadlessTexture keeps the RGBA8 upload copy plus the float RGB view read by
// the sampler.

#pragma once

#include <engine/renderer/capabilities.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <glm/ext/vector_float2.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/geometric.hpp>
#include <vector>

namespace engine
{
namespace renderer
{

// CPU-side mesh copy in planar (SoA) layout for rasterizer locality, with
// bounding-sphere data for culling/LOD. lodIndices holds decimated index
// lists (every 4th/16th/64th triangle) generated at creation.
struct HeadlessMesh
{
    // The owning renderer handle; lets the transform cache key its per-mesh
    // entry without plumbing the handle through every draw entry point.
    MeshHandle                 handle = invalidMesh;
    std::vector<float>         positions;
    std::vector<float>         normals;
    std::vector<float>         colors;
    std::vector<float>         uvs;
    std::vector<std::uint32_t> indices;
    glm::vec3                  boundsCenter{0.0f};
    float                      boundsRadius = 0.0f;
    bool                       boundsValid  = false;

    std::vector<std::vector<std::uint32_t>> lodIndices;

    // Memo for the back-face winding vote (see HeadlessBackend's
    // prepareRasterization): the vote compares face normals against vertex
    // normals in world space, and ((M a) x (M b)) . (M c) = det(M) . ((a x b)
    // . c) for any invertible linear part M, so its outcome is a pure
    // function of the mesh geometry, the sampled index list, and the sign of
    // the model matrix determinant. It is therefore computed once per (index
    // list, determinant sign) pair and reused. Slot 0 = positive
    // determinant, slot 1 = negative (mirrored) transform; each slot holds
    // one entry per index list the mesh is drawn with (base plus LODs).
    struct WindingVoteMemo
    {
        static constexpr std::size_t capacity = 8;
        std::array<const std::uint32_t *, capacity> sources{};
        std::array<float, capacity>                 signs{};
    };
    mutable std::array<WindingVoteMemo, 2> windingVotes;

    [[nodiscard]] std::size_t vertexCount() const noexcept
    {
        return positions.size() / 3;
    }
};

// CPU-side texture. pixels is the RGBA8 upload copy (read by sampleAlpha,
// e.g. the fur strand field); pixelsRGB is what the color sampler reads as
// mip level 0, and mipChain holds the box-filtered quarters for levels 1..n
// (see sampleTrilinear).
struct HeadlessTexture
{
    // One box-filtered quarter of the previous level; level 0 lives in
    // pixelsRGB, so the chain starts at level 1.
    struct MipLevel
    {
        std::uint32_t      width  = 0;
        std::uint32_t      height = 0;
        std::vector<float> rgb;
    };

    std::uint32_t                   width  = 0;
    std::uint32_t                   height = 0;
    std::vector<std::uint8_t>       pixels;
    std::vector<float>              pixelsRGB;
    std::vector<MipLevel>           mipChain;
    TextureFilter  filter  = TextureFilter::Linear;
    TextureAddress address = TextureAddress::Repeat;

    // Power-of-two extents switch the sampler's repeat addressing from a
    // division/modulo pair to an AND mask (same result for any int32 texel
    // coordinate; see wrappedCoordinate). Set by createTexture; the 1x1
    // stand-ins leave them at their defaults, which the early-out never
    // reaches.
    std::uint32_t wrapMaskX = 0;
    std::uint32_t wrapMaskY = 0;
    bool          repeatPow2 = false;

    // Bilinear or nearest sample with repeat/clamp addressing; returns white
    // when empty. Defined inline in the header: the rasterizer calls it per
    // covered pixel, and an out-of-line definition compiles to a real call
    // per sampled texel.
    [[nodiscard]] glm::vec3 sample(float u, float v) const
    {
        const float *data = pixelsRGB.data();
        const std::uint32_t mipW = width;
        const std::uint32_t mipH = height;
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
        const float texelX = u * static_cast<float>(mipW) - 0.5f;
        const float texelY = v * static_cast<float>(mipH) - 0.5f;
        const auto pixelAt = [data, mipW](std::uint32_t x, std::uint32_t y) -> const float *
        { return data + (static_cast<std::size_t>(y) * mipW + x) * 3; };
        if (filter == TextureFilter::Nearest)
        {
            const auto x = wrappedCoordinate(static_cast<std::int32_t>(std::floor(texelX + 0.5f)), mipW, wrapMaskX);
            const auto y = wrappedCoordinate(static_cast<std::int32_t>(std::floor(texelY + 0.5f)), mipH, wrapMaskY);
            const float *p = pixelAt(x, y);
            return glm::vec3(p[0], p[1], p[2]);
        }
        const auto          x0i = static_cast<std::int32_t>(std::floor(texelX));
        const auto          y0i = static_cast<std::int32_t>(std::floor(texelY));
        const float         fx  = texelX - static_cast<float>(x0i);
        const float         fy  = texelY - static_cast<float>(y0i);
        const std::uint32_t x0 = wrappedCoordinate(x0i, mipW, wrapMaskX), x1 = wrappedCoordinate(x0i + 1, mipW, wrapMaskX);
        const std::uint32_t y0 = wrappedCoordinate(y0i, mipH, wrapMaskY), y1 = wrappedCoordinate(y0i + 1, mipH, wrapMaskY);
        const float        *p00 = pixelAt(x0, y0), *p10 = pixelAt(x1, y0), *p01 = pixelAt(x0, y1), *p11 = pixelAt(x1, y1);
        const float         omx = 1.0f - fx, omy = 1.0f - fy;
        const float         w00 = omx * omy, w10 = fx * omy, w01 = omx * fy, w11 = fx * fy;
        return glm::vec3(p00[0] * w00 + p10[0] * w10 + p01[0] * w01 + p11[0] * w11,
            p00[1] * w00 + p10[1] * w10 + p01[1] * w01 + p11[1] * w11,
            p00[2] * w00 + p10[2] * w10 + p01[2] * w01 + p11[2] * w11);
    }

    // Trilinear sampling for screen-mapped fragments: uvDx / uvDy are the
    // perspective-correct UV derivatives per screen pixel (the rasterizer
    // derives them from the same barycentric gradients that interpolate the
    // UV itself). The larger axis footprint selects the mip level, and the
    // level blends bilinearly across the floor/ceil pair - the CPU twin of
    // the GPU sampler's linear-mipmap-linear filtering. Callers without
    // derivatives (fur strand fields, flat stand-ins) keep the base-level
    // sample(): their fetches are screen-aligned or constant.
    [[nodiscard]] glm::vec3 sampleTrilinear(float u, float v, const glm::vec2 &uvDx, const glm::vec2 &uvDy) const
    {
        if (width == 0 || height == 0 || pixelsRGB.empty() || filter != TextureFilter::Linear
            || mipChain.empty() || (width == 1 && height == 1))
        {
            return sample(u, v);
        }
        // Isotropic footprint: the longer of the two screen-axis footprints,
        // in level-0 texels. Sub-texel footprints stay on the base level.
        // The comparisons run on SQUARED footprints: the two common extremes
        // (full minification onto the top level, magnification onto the base
        // level) then resolve with two multiplies and a compare, and the
        // sqrt/log2 pair only runs inside the blend band between them.
        const float footprintX2 = uvDx.x * uvDx.x * static_cast<float>(width * width)
            + uvDx.y * uvDx.y * static_cast<float>(height * height);
        const float footprintY2 = uvDy.x * uvDy.x * static_cast<float>(width * width)
            + uvDy.y * uvDy.y * static_cast<float>(height * height);
        const float footprintSquared = std::max(footprintX2, footprintY2);
        const float maxLevel         = static_cast<float>(mipChain.size());
        const float topFootprint     = std::exp2(maxLevel); // footprint that selects the top level
        if (footprintSquared >= topFootprint * topFootprint)
        {
            return sampleBilinearAt(static_cast<std::uint32_t>(maxLevel), u, v);
        }
        if (footprintSquared <= 1.0f)
        {
            return sample(u, v);
        }
        const float level = std::min(maxLevel, std::log2(std::sqrt(footprintSquared)));
        const auto  low   = static_cast<std::uint32_t>(level);
        const float blend = level - static_cast<float>(low);
        if (low >= mipChain.size() || blend <= 0.0f)
        {
            return sampleBilinearAt(low, u, v);
        }
        const glm::vec3 coarse = sampleBilinearAt(low, u, v);
        const glm::vec3 fine   = sampleBilinearAt(low + 1 <= mipChain.size() ? low + 1 : low, u, v);
        return fine * (1.0f - blend) + coarse * blend;
    }

    // Alpha-channel sample from the RGBA8 upload copy, same addressing and
    // filtering as sample(). Alpha always samples the base image - matching
    // the fur pipeline, whose strand-field fetch (the fur texture's alpha)
    // runs at the base level on the GPU side too. Returns 1 when empty.
    [[nodiscard]] float sampleAlpha(float u, float v) const
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
        const auto alphaAt = [&](std::uint32_t x, std::uint32_t y) -> float
        { return static_cast<float>(pixels[(static_cast<std::size_t>(y) * width + x) * 4 + 3]) / 255.0f; };
        if (filter == TextureFilter::Nearest)
        {
            const auto x = wrappedCoordinate(static_cast<std::int32_t>(std::floor(texelX + 0.5f)), width, wrapMaskX);
            const auto y = wrappedCoordinate(static_cast<std::int32_t>(std::floor(texelY + 0.5f)), height, wrapMaskY);
            return alphaAt(x, y);
        }
        const auto          x0i = static_cast<std::int32_t>(std::floor(texelX));
        const auto          y0i = static_cast<std::int32_t>(std::floor(texelY));
        const float         fx  = texelX - static_cast<float>(x0i);
        const float         fy  = texelY - static_cast<float>(y0i);
        const std::uint32_t x0 = wrappedCoordinate(x0i, width, wrapMaskX), x1 = wrappedCoordinate(x0i + 1, width, wrapMaskX);
        const std::uint32_t y0 = wrappedCoordinate(y0i, height, wrapMaskY), y1 = wrappedCoordinate(y0i + 1, height, wrapMaskY);
        const float a00 = alphaAt(x0, y0), a10 = alphaAt(x1, y0), a01 = alphaAt(x0, y1), a11 = alphaAt(x1, y1);
        const float omx = 1.0f - fx, omy = 1.0f - fy;
        return a00 * omx * omy + a10 * fx * omy + a01 * omx * fy + a11 * fx * fy;
    }

private:
    // Bilinear fetch from one mip level (0 = the base image, k = mipChain[k-1])
    // with this sampler's addressing; shares the wrap logic with sample().
    [[nodiscard]] glm::vec3 sampleBilinearAt(std::uint32_t level, float u, float v) const
    {
        std::uint32_t   levelWidth = 0;
        std::uint32_t   levelHeight = 0;
        const float    *data = nullptr;
        if (level == 0u)
        {
            levelWidth  = width;
            levelHeight = height;
            data        = pixelsRGB.data();
        }
        else if (level - 1u < mipChain.size())
        {
            const MipLevel &mip = mipChain[level - 1u];
            levelWidth  = mip.width;
            levelHeight = mip.height;
            data        = mip.rgb.data();
        }
        if (levelWidth == 0 || levelHeight == 0 || data == nullptr)
        {
            return glm::vec3(1.0f);
        }
        if (levelWidth == 1u && levelHeight == 1u)
        {
            return glm::vec3(data[0], data[1], data[2]);
        }
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
        // Power-of-two levels keep the mask trick: every level of a pow2 base
        // is pow2 itself, so each level derives its own mask from its extent.
        const std::uint32_t maskX = (levelWidth & (levelWidth - 1u)) == 0u ? levelWidth - 1u : 0u;
        const std::uint32_t maskY = (levelHeight & (levelHeight - 1u)) == 0u ? levelHeight - 1u : 0u;
        const float texelX = u * static_cast<float>(levelWidth) - 0.5f;
        const float texelY = v * static_cast<float>(levelHeight) - 0.5f;
        const auto pixelAt = [data, levelWidth](std::uint32_t x, std::uint32_t y) -> const float *
        { return data + (static_cast<std::size_t>(y) * levelWidth + x) * 3; };
        const auto x0i = static_cast<std::int32_t>(std::floor(texelX));
        const auto y0i = static_cast<std::int32_t>(std::floor(texelY));
        const float fx = texelX - static_cast<float>(x0i);
        const float fy = texelY - static_cast<float>(y0i);
        const std::uint32_t x0 = wrappedCoordinate(x0i, levelWidth, maskX), x1 = wrappedCoordinate(x0i + 1, levelWidth, maskX);
        const std::uint32_t y0 = wrappedCoordinate(y0i, levelHeight, maskY), y1 = wrappedCoordinate(y0i + 1, levelHeight, maskY);
        const float *p00 = pixelAt(x0, y0), *p10 = pixelAt(x1, y0), *p01 = pixelAt(x0, y1), *p11 = pixelAt(x1, y1);
        const float omx = 1.0f - fx, omy = 1.0f - fy;
        const float w00 = omx * omy, w10 = fx * omy, w01 = omx * fy, w11 = fx * fy;
        return glm::vec3(p00[0] * w00 + p10[0] * w10 + p01[0] * w01 + p11[0] * w11,
            p00[1] * w00 + p10[1] * w10 + p01[1] * w01 + p11[1] * w11,
            p00[2] * w00 + p10[2] * w10 + p01[2] * w01 + p11[2] * w11);
    }

    // Maps one texel coordinate onto the texture under this sampler's
    // addressing: repeat wraps the coordinate into [0, extent), clamp pins
    // it to the edge. Shared by the color and alpha samplers so both walk
    // coordinates identically. Power-of-two repeats wrap through the mask
    // (identical result to the modulo for any int32 coordinate, negative
    // included, in two's complement).
    [[nodiscard]] std::uint32_t wrappedCoordinate(std::int32_t coordinate, std::uint32_t extent, std::uint32_t pow2Mask) const
    {
        const std::int32_t limit = static_cast<std::int32_t>(extent);
        if (address == TextureAddress::Repeat)
        {
            if (pow2Mask != 0u)
            {
                return static_cast<std::uint32_t>(coordinate) & pow2Mask;
            }
            const std::int32_t wrapped = coordinate % limit;
            return static_cast<std::uint32_t>(wrapped < 0 ? wrapped + limit : wrapped);
        }
        return static_cast<std::uint32_t>(std::clamp(coordinate, 0, limit - 1));
    }
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

} // namespace renderer
} // namespace engine

