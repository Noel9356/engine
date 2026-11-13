// Headless backend data types: CPU-side mesh and texture representations
// shared between the headless backend and its rasterizer core. Extracted from
// headless_backend.hpp so the rasterizer's inner types and the mesh/texture
// records can be reused without dragging in the full HeadlessBackend class.
//
// HeadlessMesh is a planar (SoA) layout for rasterizer locality, with
// bounding-sphere data for culling/LOD. lodIndices holds decimated index
// lists (every 4th/16th/64th triangle) generated at creation.
//
// HeadlessTexture keeps the RGBA8 upload copy plus a float RGB mip chain
// (box-filtered, halved each level) read by the sampler.

#ifndef ENGINE_RENDERER_MODULE_HEADLESS_TYPES_HPP
#define ENGINE_RENDERER_MODULE_HEADLESS_TYPES_HPP

#include "engine/renderer/capabilities.hpp"
#include "engine/renderer/material.hpp"
#include "engine/renderer/mesh.hpp"

#include <glm/ext/vector_float3.hpp>
#include <glm/geometric.hpp>
#include <array>
#include <cstddef>
#include <cstdint>
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

} // namespace renderer
} // namespace engine

#endif
