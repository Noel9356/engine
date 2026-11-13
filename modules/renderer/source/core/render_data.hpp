// Mesh/texture normalization shared by both backends' createMesh/
// createTexture paths: converts arbitrary vertex layouts into the module's
// 11-float interleaved vertex (position, normal, color, uv) and arbitrary
// pixel layouts into tightly packed top-left RGBA8.

#ifndef ENGINE_RENDERER_MODULE_DATA_HPP
#define ENGINE_RENDERER_MODULE_DATA_HPP

#include "engine/renderer/mesh.hpp"
#include <engine/renderer/capabilities.hpp>

#include <cstdint>
#include <vector>

namespace engine
{
namespace renderer
{

// A mesh after prepareMeshData(). Either the normalized interleaved copy is
// filled (vertices) or, when the source was already canonical 11-float data,
// the source pointers are kept and vertices stays empty; description()
// stitches whichever representation is live into a MeshDescription.
struct PreparedMeshData
{
    std::vector<float>         vertices;
    std::vector<std::uint32_t> indices;

    const void   *sourceVertices = nullptr; // borrowed from the caller when no copy is needed
    const void   *sourceIndices  = nullptr;
    std::uint32_t vertexCount    = 0;
    std::uint32_t indexCount     = 0;

    MeshDescription description() const;
};

// A texture after prepareTextureData(): empty pixels means the source was
// already canonical RGBA8/top-left/tightly packed; otherwise pixels holds the
// converted copy and description() points at it.
struct PreparedTextureData
{
    std::vector<std::uint8_t> pixels;
    TextureDescription        source;

    TextureDescription description() const;
};

// Both validate thoroughly (finite values, in-range indices, stride/size
// overflow) and report RenderError::InvalidMesh/InvalidTexture through the
// out-parameter on failure.
bool prepareMeshData(const MeshDescription &source, PreparedMeshData &prepared, RenderError *error);

bool prepareTextureData(const TextureDescription &source, PreparedTextureData &prepared, RenderError *error);

} // namespace renderer
} // namespace engine

#endif
