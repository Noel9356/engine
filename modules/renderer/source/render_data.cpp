// Mesh/texture normalization for the renderer capability's create paths.
//
// prepareMeshData folds arbitrary vertex layouts (interleaved 11-float data
// or a strided attribute list) into the module's canonical interleaved
// vertex; prepareTextureData converts any supported pixel format/origin into
// tightly packed top-left RGBA8. Both validate aggressively so backends can
// trust what they receive.

#include "render_data.hpp"
#include "engine/renderer/capabilities.hpp"
#include "engine/renderer/mesh.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace engine
{
namespace renderer
{
namespace
{

void setError(RenderError *error, RenderError value)
{
    if (error != nullptr)
    {
        *error = value;
    }
}

struct VertexAttributeView
{
    const VertexAttribute *attribute = nullptr; // null means the semantic is absent
    std::uint32_t          byteSize  = 0;       // size of the attribute in the source stride
};

// Byte size of one attribute value; false for an unknown format.
bool vertexFormatSize(VertexFormat format, std::uint32_t &size)
{
    switch (format)
    {
    case VertexFormat::Float32x2:
        size = 2 * sizeof(float);
        return true;
    case VertexFormat::Float32x3:
        size = 3 * sizeof(float);
        return true;
    case VertexFormat::Float32x4:
        size = 4 * sizeof(float);
        return true;
    case VertexFormat::UNorm8x4:
    case VertexFormat::SNorm8x4:
        size = 4 * sizeof(std::uint8_t);
        return true;
    }
    return false;
}

// Which vertex formats are legal for each semantic (positions need 3+ floats,
// colors may be 8-bit unorm, etc.).
bool formatSupportsSemantic(VertexSemantic semantic, VertexFormat format)
{
    switch (semantic)
    {
    case VertexSemantic::Position:
        return format == VertexFormat::Float32x3 || format == VertexFormat::Float32x4;
    case VertexSemantic::Normal:
        return format == VertexFormat::Float32x3 || format == VertexFormat::Float32x4
            || format == VertexFormat::SNorm8x4;
    case VertexSemantic::Color:
        return format == VertexFormat::Float32x3 || format == VertexFormat::Float32x4
            || format == VertexFormat::UNorm8x4;
    case VertexSemantic::TexCoord0:
        return format == VertexFormat::Float32x2;
    }
    return false;
}

const std::uint8_t *attributeAddress(const std::uint8_t *vertex, const VertexAttribute &attribute)
{
    return vertex + attribute.offsetBytes;
}

// Reads a 3-component attribute, widening snorm/unorm byte formats to
// floats (snorm re-centers to [-1, 1] by dividing by 127).
void readFloat3(const std::uint8_t *vertex, const VertexAttribute &attribute, float out[3])
{
    if (attribute.format == VertexFormat::SNorm8x4)
    {
        std::int8_t values[4]{};
        std::memcpy(values, attributeAddress(vertex, attribute), sizeof(values));
        for (std::uint32_t i = 0; i < 3; ++i)
        {
            out[i] = std::max(-1.0f, static_cast<float>(values[i]) / 127.0f);
        }
        return;
    }

    if (attribute.format == VertexFormat::UNorm8x4)
    {
        std::uint8_t values[4]{};
        std::memcpy(values, attributeAddress(vertex, attribute), sizeof(values));
        for (std::uint32_t i = 0; i < 3; ++i)
        {
            out[i] = static_cast<float>(values[i]) / 255.0f;
        }
        return;
    }

    std::memcpy(out, attributeAddress(vertex, attribute), 3 * sizeof(float));
}

void readFloat2(const std::uint8_t *vertex, const VertexAttribute &attribute, float out[2])
{
    std::memcpy(out, attributeAddress(vertex, attribute), 2 * sizeof(float));
}

bool finite3(const float value[3])
{
    return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

// Returns false on any malformed index buffer. With no indices supplied it
// generates the identity sequence (0..vertexCount-1); 32-bit indices that are
// already aligned and in range are borrowed instead of copied.
bool resolveIndices(const MeshDescription &source, PreparedMeshData &prepared)
{
    prepared.sourceIndices = nullptr;
    prepared.indexCount    = 0;

    if (source.indexCount == 0)
    {
        if (source.indices != nullptr || source.vertexCount > prepared.indices.max_size())
        {
            return false;
        }
        prepared.indices.resize(source.vertexCount);
        for (std::uint32_t i = 0; i < source.vertexCount; ++i)
        {
            prepared.indices[i] = i;
        }
        prepared.indexCount = source.vertexCount;
        return prepared.indexCount >= 3 && prepared.indexCount % 3 == 0;
    }

    if (source.indices == nullptr || source.indexCount < 3 || source.indexCount % 3 != 0
        || source.indexCount > prepared.indices.max_size())
    {
        return false;
    }

    if (source.indexFormat == MeshIndexFormat::UInt32
        && reinterpret_cast<std::uintptr_t>(source.indices) % alignof(std::uint32_t) == 0)
    {
        const auto *sourceIndices = static_cast<const std::uint32_t *>(source.indices);
        for (std::uint32_t i = 0; i < source.indexCount; ++i)
        {
            if (sourceIndices[i] >= source.vertexCount)
            {
                return false;
            }
        }
        prepared.sourceIndices = source.indices;
        prepared.indexCount    = source.indexCount;
        return true;
    }

    prepared.indices.resize(source.indexCount);
    const auto *sourceBytes = static_cast<const std::uint8_t *>(source.indices);
    switch (source.indexFormat)
    {
    case MeshIndexFormat::UInt16: {
        for (std::uint32_t i = 0; i < source.indexCount; ++i)
        {
            std::uint16_t index = 0;
            std::memcpy(&index, sourceBytes + static_cast<std::size_t>(i) * sizeof(index), sizeof(index));
            if (index >= source.vertexCount)
            {
                return false;
            }
            prepared.indices[i] = index;
        }
        break;
    }
    case MeshIndexFormat::UInt32: {
        for (std::uint32_t i = 0; i < source.indexCount; ++i)
        {
            std::uint32_t index = 0;
            std::memcpy(&index, sourceBytes + static_cast<std::size_t>(i) * sizeof(index), sizeof(index));
            if (index >= source.vertexCount)
            {
                return false;
            }
            prepared.indices[i] = index;
        }
        break;
    }
    default:
        return false;
    }

    prepared.indexCount = source.indexCount;
    return true;
}

// Accumulates per-triangle cross products into the normal slots, then
// normalizes; degenerate normals fall back to +Y. Fails on out-of-range indices.
bool generateNormals(std::vector<float> &vertices, const std::vector<std::uint32_t> &indices, std::uint32_t vertexCount)
{
    for (std::size_t triangle = 0; triangle < indices.size(); triangle += 3)
    {
        const std::uint32_t index0 = indices[triangle + 0];
        const std::uint32_t index1 = indices[triangle + 1];
        const std::uint32_t index2 = indices[triangle + 2];
        if (index0 >= vertexCount || index1 >= vertexCount || index2 >= vertexCount)
        {
            return false;
        }

        const float *vertex0 = vertices.data() + static_cast<std::size_t>(index0) * meshVertexFloatCount;
        const float *vertex1 = vertices.data() + static_cast<std::size_t>(index1) * meshVertexFloatCount;
        const float *vertex2 = vertices.data() + static_cast<std::size_t>(index2) * meshVertexFloatCount;

        const float edge1[3] = {
            vertex1[0] - vertex0[0],
            vertex1[1] - vertex0[1],
            vertex1[2] - vertex0[2],
        };
        const float edge2[3] = {
            vertex2[0] - vertex0[0],
            vertex2[1] - vertex0[1],
            vertex2[2] - vertex0[2],
        };
        const float normal[3] = {
            edge1[1] * edge2[2] - edge1[2] * edge2[1],
            edge1[2] * edge2[0] - edge1[0] * edge2[2],
            edge1[0] * edge2[1] - edge1[1] * edge2[0],
        };

        float *target0 = vertices.data() + static_cast<std::size_t>(index0) * meshVertexFloatCount + 3;
        float *target1 = vertices.data() + static_cast<std::size_t>(index1) * meshVertexFloatCount + 3;
        float *target2 = vertices.data() + static_cast<std::size_t>(index2) * meshVertexFloatCount + 3;
        target0[0] += normal[0];
        target0[1] += normal[1];
        target0[2] += normal[2];
        target1[0] += normal[0];
        target1[1] += normal[1];
        target1[2] += normal[2];
        target2[0] += normal[0];
        target2[1] += normal[1];
        target2[2] += normal[2];
    }

    for (std::uint32_t i = 0; i < vertexCount; ++i)
    {
        float      *normal        = vertices.data() + static_cast<std::size_t>(i) * meshVertexFloatCount + 3;
        const float lengthSquared = normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2];
        if (lengthSquared > 1e-20f && std::isfinite(lengthSquared))
        {
            const float inverseLength = 1.0f / std::sqrt(lengthSquared);
            normal[0] *= inverseLength;
            normal[1] *= inverseLength;
            normal[2] *= inverseLength;
        }
        else
        {
            normal[0] = 0.0f;
            normal[1] = 0.0f;
            normal[2] = 1.0f;
        }
    }
    return true;
}

// Bytes per pixel for the source pixel formats (0 = unknown format).
std::uint32_t textureBytesPerPixel(TexturePixelFormat format)
{
    switch (format)
    {
    case TexturePixelFormat::R8:
        return 1;
    case TexturePixelFormat::RG8:
        return 2;
    case TexturePixelFormat::RGB8:
        return 3;
    case TexturePixelFormat::RGBA8:
    case TexturePixelFormat::BGRA8:
        return 4;
    }
    return 0;
}

} // namespace

// description() prefers the normalized copies but falls back to the borrowed
// source pointers when no conversion was needed.
MeshDescription PreparedMeshData::description() const
{
    MeshDescription result;
    result.vertices    = vertices.empty() ? sourceVertices : vertices.data();
    result.vertexCount = vertexCount;
    result.indices     = indices.empty() ? sourceIndices : indices.data();
    result.indexCount  = indexCount;
    result.indexFormat = MeshIndexFormat::UInt32;
    return result;
}

TextureDescription PreparedTextureData::description() const
{
    TextureDescription result = source;
    if (!pixels.empty())
    {
        result.pixels = pixels.data();
    }
    result.pixelFormat    = TexturePixelFormat::RGBA8;
    result.origin         = TextureOrigin::TopLeft;
    result.rowStrideBytes = 0;
    return result;
}

bool prepareMeshData(const MeshDescription &source, PreparedMeshData &prepared, RenderError *error)
{
    // Two input shapes: stride 0 means already-interleaved 11-float data
    // (validated and borrowed when aligned); otherwise the attribute list is
    // repacked below into the canonical interleaved layout, one vertex at a
    // time, starting from position 0 / normal +Z / color white / uv 0.
    prepared.vertices.clear();
    prepared.indices.clear();
    prepared.sourceVertices = nullptr;
    prepared.sourceIndices  = nullptr;
    prepared.vertexCount    = 0;
    prepared.indexCount     = 0;

    if (source.vertices == nullptr || source.vertexCount == 0)
    {
        setError(error, RenderError::InvalidMesh);
        return false;
    }

    bool hasNormalAttribute = false;
    if (source.vertexStrideBytes == 0)
    {
        if (source.vertexAttributes != nullptr || source.vertexAttributeCount != 0)
        {
            setError(error, RenderError::InvalidMesh);
            return false;
        }
        const std::size_t floatCount = static_cast<std::size_t>(source.vertexCount) * meshVertexFloatCount;
        if (floatCount > prepared.vertices.max_size())
        {
            setError(error, RenderError::InvalidMesh);
            return false;
        }
        const bool   aligned      = reinterpret_cast<std::uintptr_t>(source.vertices) % alignof(float) == 0;
        const float *sourceFloats = aligned ? static_cast<const float *>(source.vertices) : nullptr;
        if (!aligned)
        {
            prepared.vertices.resize(floatCount);
            std::memcpy(prepared.vertices.data(), source.vertices, floatCount * sizeof(float));
            sourceFloats = prepared.vertices.data();
        }
        for (std::size_t i = 0; i < floatCount; ++i)
        {
            if (!std::isfinite(sourceFloats[i]))
            {
                setError(error, RenderError::InvalidMesh);
                prepared.vertices.clear();
                return false;
            }
        }
        if (aligned)
        {
            prepared.sourceVertices = source.vertices;
        }
    }
    else
    {
        if (source.vertexAttributes == nullptr || source.vertexAttributeCount == 0
            || source.vertexCount > std::numeric_limits<std::size_t>::max() / source.vertexStrideBytes)
        {
            setError(error, RenderError::InvalidMesh);
            return false;
        }

        VertexAttributeView attributes[4]{};
        bool                hasPosition = false;
        bool                hasNormal   = false;
        for (std::uint32_t i = 0; i < source.vertexAttributeCount; ++i)
        {
            const VertexAttribute &attribute = source.vertexAttributes[i];
            if (static_cast<std::uint32_t>(attribute.semantic) > static_cast<std::uint32_t>(VertexSemantic::TexCoord0))
            {
                setError(error, RenderError::InvalidMesh);
                return false;
            }
            const std::size_t semanticIndex = static_cast<std::size_t>(attribute.semantic);
            if (attributes[semanticIndex].attribute != nullptr
                || !formatSupportsSemantic(attribute.semantic, attribute.format)
                || !vertexFormatSize(attribute.format, attributes[semanticIndex].byteSize)
                || static_cast<std::uint64_t>(attribute.offsetBytes) + attributes[semanticIndex].byteSize
                       > source.vertexStrideBytes)
            {
                setError(error, RenderError::InvalidMesh);
                return false;
            }
            attributes[semanticIndex].attribute = &attribute;
            hasPosition |= attribute.semantic == VertexSemantic::Position;
            hasNormal |= attribute.semantic == VertexSemantic::Normal;
        }
        hasNormalAttribute = hasNormal;
        if (!hasPosition)
        {
            setError(error, RenderError::InvalidMesh);
            return false;
        }

        const std::size_t vertexFloatCount = static_cast<std::size_t>(source.vertexCount) * meshVertexFloatCount;
        if (vertexFloatCount > prepared.vertices.max_size())
        {
            setError(error, RenderError::InvalidMesh);
            return false;
        }
        prepared.vertices.resize(vertexFloatCount);
        const auto *sourceBytes = static_cast<const std::uint8_t *>(source.vertices);
        for (std::uint32_t i = 0; i < source.vertexCount; ++i)
        {
            const std::uint8_t *vertex = sourceBytes + static_cast<std::size_t>(i) * source.vertexStrideBytes;
            float              *target = prepared.vertices.data() + static_cast<std::size_t>(i) * meshVertexFloatCount;

            target[0] = target[1] = target[2] = 0.0f;
            target[3] = target[4] = 0.0f;
            target[5]             = 1.0f;
            target[6] = target[7] = target[8] = 1.0f;
            target[9] = target[10] = 0.0f;

            if (attributes[static_cast<std::size_t>(VertexSemantic::Position)].attribute != nullptr)
            {
                readFloat3(vertex, *attributes[static_cast<std::size_t>(VertexSemantic::Position)].attribute, target);
            }
            if (attributes[static_cast<std::size_t>(VertexSemantic::Normal)].attribute != nullptr)
            {
                readFloat3(vertex, *attributes[static_cast<std::size_t>(VertexSemantic::Normal)].attribute, target + 3);
            }
            if (attributes[static_cast<std::size_t>(VertexSemantic::Color)].attribute != nullptr)
            {
                readFloat3(vertex, *attributes[static_cast<std::size_t>(VertexSemantic::Color)].attribute, target + 6);
            }
            if (attributes[static_cast<std::size_t>(VertexSemantic::TexCoord0)].attribute != nullptr)
            {
                readFloat2(
                    vertex, *attributes[static_cast<std::size_t>(VertexSemantic::TexCoord0)].attribute, target + 9);
            }
            if (!finite3(target) || (hasNormal && !finite3(target + 3)) || !std::isfinite(target[9])
                || !std::isfinite(target[10]) || !finite3(target + 6))
            {
                setError(error, RenderError::InvalidMesh);
                prepared.vertices.clear();
                return false;
            }
        }
    }

    if (!resolveIndices(source, prepared))
    {
        setError(error, RenderError::InvalidMesh);
        prepared.vertices.clear();
        prepared.indices.clear();
        prepared.sourceIndices = nullptr;
        return false;
    }

    if (!prepared.vertices.empty() && source.vertexStrideBytes != 0)
    {
        // Attribute-list input: the index buffer must be materialized when
        // normals still have to be generated (generateNormals walks vertices
        // through the indices), otherwise the borrowed source indices stay.
        if (!hasNormalAttribute && prepared.indices.empty() && prepared.sourceIndices != nullptr)
        {
            const auto *indices = static_cast<const std::uint32_t *>(prepared.sourceIndices);
            prepared.indices.assign(indices, indices + prepared.indexCount);
            prepared.sourceIndices = nullptr;
        }
        if (hasNormalAttribute)
        {
            for (std::uint32_t i = 0; i < source.vertexCount; ++i)
            {
                float      *normal = prepared.vertices.data() + static_cast<std::size_t>(i) * meshVertexFloatCount + 3;
                const float lengthSquared = normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2];
                if (lengthSquared > 1e-20f && std::isfinite(lengthSquared))
                {
                    const float inverseLength = 1.0f / std::sqrt(lengthSquared);
                    normal[0] *= inverseLength;
                    normal[1] *= inverseLength;
                    normal[2] *= inverseLength;
                }
                else
                {
                    normal[0] = 0.0f;
                    normal[1] = 0.0f;
                    normal[2] = 1.0f;
                }
            }
        }
        else if (!generateNormals(prepared.vertices, prepared.indices, source.vertexCount))
        {
            setError(error, RenderError::InvalidMesh);
            prepared.vertices.clear();
            prepared.indices.clear();
            return false;
        }
    }

    prepared.vertexCount = source.vertexCount;
    setError(error, RenderError::None);
    return true;
}

bool prepareTextureData(const TextureDescription &source, PreparedTextureData &prepared, RenderError *error)
{
    prepared.pixels.clear();
    prepared.source = source;

    if (source.pixels == nullptr || source.width == 0 || source.height == 0)
    {
        setError(error, RenderError::InvalidTexture);
        return false;
    }

    const std::uint32_t bytesPerPixel = textureBytesPerPixel(source.pixelFormat);
    if (bytesPerPixel == 0
        || static_cast<std::uint32_t>(source.origin) > static_cast<std::uint32_t>(TextureOrigin::BottomLeft)
        || source.width > std::numeric_limits<std::uint32_t>::max() / bytesPerPixel)
    {
        setError(error, RenderError::InvalidTexture);
        return false;
    }

    const std::uint32_t tightRowBytes = source.width * bytesPerPixel;
    const std::uint32_t sourceStride  = source.rowStrideBytes == 0 ? tightRowBytes : source.rowStrideBytes;
    if (sourceStride < tightRowBytes || source.height > std::numeric_limits<std::size_t>::max() / sourceStride
        || source.height > std::numeric_limits<std::size_t>::max() / source.width / 4)
    {
        setError(error, RenderError::InvalidTexture);
        return false;
    }

    // Canonical input (RGBA8, top-left, tightly packed) is passed through
    // with no copy; everything else is converted row by row below.
    const bool isCanonical = source.pixelFormat == TexturePixelFormat::RGBA8 && source.origin == TextureOrigin::TopLeft
                          && sourceStride == source.width * 4;
    if (isCanonical)
    {
        setError(error, RenderError::None);
        return true;
    }

    const std::size_t destinationSize = static_cast<std::size_t>(source.width) * source.height * 4;
    if (destinationSize > prepared.pixels.max_size())
    {
        setError(error, RenderError::InvalidTexture);
        return false;
    }
    prepared.pixels.resize(destinationSize);

    if (source.pixelFormat == TexturePixelFormat::RGBA8)
    {
        for (std::uint32_t y = 0; y < source.height; ++y)
        {
            const std::uint32_t sourceY        = source.origin == TextureOrigin::TopLeft ? y : source.height - 1 - y;
            const std::uint8_t *sourceRow      = source.pixels + static_cast<std::size_t>(sourceY) * sourceStride;
            std::uint8_t       *destinationRow = prepared.pixels.data() + static_cast<std::size_t>(y) * tightRowBytes;
            std::memcpy(destinationRow, sourceRow, tightRowBytes);
        }
        setError(error, RenderError::None);
        return true;
    }

    for (std::uint32_t y = 0; y < source.height; ++y)
    {
        const std::uint32_t sourceY        = source.origin == TextureOrigin::TopLeft ? y : source.height - 1 - y;
        const std::uint8_t *sourceRow      = source.pixels + static_cast<std::size_t>(sourceY) * sourceStride;
        std::uint8_t       *destinationRow = prepared.pixels.data() + static_cast<std::size_t>(y) * source.width * 4;

        for (std::uint32_t x = 0; x < source.width; ++x)
        {
            const std::uint8_t *pixel = sourceRow + static_cast<std::size_t>(x) * bytesPerPixel;
            std::uint8_t       *rgba  = destinationRow + static_cast<std::size_t>(x) * 4;
            switch (source.pixelFormat)
            {
            case TexturePixelFormat::R8:
                rgba[0] = rgba[1] = rgba[2] = pixel[0];
                rgba[3]                     = 255;
                break;
            case TexturePixelFormat::RG8:
                rgba[0] = pixel[0];
                rgba[1] = pixel[1];
                rgba[2] = 0;
                rgba[3] = 255;
                break;
            case TexturePixelFormat::RGB8:
                rgba[0] = pixel[0];
                rgba[1] = pixel[1];
                rgba[2] = pixel[2];
                rgba[3] = 255;
                break;
            case TexturePixelFormat::RGBA8:
                std::memcpy(rgba, pixel, 4);
                break;
            case TexturePixelFormat::BGRA8:
                rgba[0] = pixel[2];
                rgba[1] = pixel[1];
                rgba[2] = pixel[0];
                rgba[3] = pixel[3];
                break;
            }
        }
    }

    setError(error, RenderError::None);
    return true;
}

} // namespace renderer
} // namespace engine
