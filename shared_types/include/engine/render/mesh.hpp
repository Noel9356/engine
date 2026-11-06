// Shared mesh upload description (engine::render).
//
// Describes interleaved vertex data (attribute table + stride) and index
// data without depending on any renderer API.

#ifndef ENGINE_RENDER_MESH_HPP
#define ENGINE_RENDER_MESH_HPP

#include <cstdint>

namespace engine::render
{

// Canonical packed vertex size in floats, used by mesh converters.
inline constexpr std::uint32_t meshVertexFloatCount = 11;

enum class VertexSemantic : std::uint32_t
{
    Position = 0,
    Normal,
    Color,
    TexCoord0
};

// Attribute component format (normalized byte formats included).
enum class VertexFormat : std::uint32_t
{
    Float32x2 = 0,
    Float32x3,
    Float32x4,
    UNorm8x4,
    SNorm8x4
};

// One attribute: semantic, format and byte offset within the vertex stride.
struct VertexAttribute
{
    VertexSemantic semantic    = VertexSemantic::Position;
    VertexFormat   format      = VertexFormat::Float32x3;
    std::uint32_t  offsetBytes = 0;
};

enum class MeshIndexFormat : std::uint32_t
{
    UInt16 = 0,
    UInt32
};

// Non-owning view of CPU vertex/index buffers handed to the renderer for upload
// (callers keep the memory alive until the upload completes).
struct MeshDescription
{
    const void   *vertices    = nullptr;
    std::uint32_t vertexCount = 0;

    const void     *indices     = nullptr;
    std::uint32_t   indexCount  = 0;
    MeshIndexFormat indexFormat = MeshIndexFormat::UInt32;

    std::uint32_t          vertexStrideBytes    = 0;
    const VertexAttribute *vertexAttributes     = nullptr;
    std::uint32_t          vertexAttributeCount = 0;
};

} // namespace engine::render

#endif
