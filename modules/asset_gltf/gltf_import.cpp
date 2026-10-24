#include "gltf_import.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace engine
{

namespace
{

std::array<float, 3> vec3FromArray(
    const JsonValue *value, const std::array<float, 3> &fallback
)
{
    if (value == nullptr || !value->isArray() || value->array.size() < 3)
    {
        return fallback;
    }
    return {
        static_cast<float>(value->array[0].numberOr(0.0)),
        static_cast<float>(value->array[1].numberOr(0.0)),
        static_cast<float>(value->array[2].numberOr(0.0)),
    };
}

std::array<float, 4> vec4FromArray(
    const JsonValue *value, const std::array<float, 4> &fallback
)
{
    if (value == nullptr || !value->isArray() || value->array.size() < 4)
    {
        return fallback;
    }
    return {
        static_cast<float>(value->array[0].numberOr(0.0)),
        static_cast<float>(value->array[1].numberOr(0.0)),
        static_cast<float>(value->array[2].numberOr(0.0)),
        static_cast<float>(value->array[3].numberOr(0.0)),
    };
}

// One node's resolved world transform plus its glTF index.
struct NodeRecord
{
    GltfMat4            worldTransform{};
    bool                hasMesh    = false;
    std::size_t         meshIndex  = 0;
    std::string         name;
    std::vector<std::size_t> children;
};

// Reads one node's LOCAL transform (matrix or TRS, exactly as the glTF 2.0
// spec prescribes: when matrix is present, TRS MUST be ignored).
GltfMat4 localTransformOf(
    const JsonValue &node
)
{
    const JsonValue *matrix = node.find("matrix");
    if (matrix != nullptr && matrix->isArray() && matrix->array.size() == 16)
    {
        GltfMat4 result{};
        for (std::size_t index = 0; index < 16; ++index)
        {
            result[index] = static_cast<float>(matrix->array[index].numberOr(0.0));
        }
        return result;
    }
    const std::array<float, 3> translation = vec3FromArray(node.find("translation"), {0.0f, 0.0f, 0.0f});
    // glTF rotation is (x, y, z, w) with w last.
    const std::array<float, 4> rotation =
        vec4FromArray(node.find("rotation"), {0.0f, 0.0f, 0.0f, 1.0f});
    const std::array<float, 3> scale = vec3FromArray(node.find("scale"), {1.0f, 1.0f, 1.0f});
    return gltfComposeTrs(translation, rotation, scale);
}

// Builds the node table with composed world transforms: one depth-first walk,
// each world transform computed exactly once from its parent's.
void collectNodes(
    const JsonValue               &nodesArray,
    std::size_t                    nodeIndex,
    const GltfMat4               &parentTransform,
    std::vector<NodeRecord>       &outNodes,
    std::vector<bool>             &visited,
    std::string                   &errorReport
)
{
    if (nodeIndex >= nodesArray.array.size())
    {
        errorReport = "node index " + std::to_string(nodeIndex) + " is out of range";
        return;
    }
    if (visited[nodeIndex])
    {
        // A cycle is a malformed file (the spec forbids it); cutting the walk
        // here keeps the import finite without a crash.
        return;
    }
    visited[nodeIndex] = true;

    const JsonValue &node = nodesArray.array[nodeIndex];
    NodeRecord       record;
    record.name       = node.find("name") != nullptr ? node.find("name")->stringOr("") : "";
    record.worldTransform = gltfMultiplyMatrix(parentTransform, localTransformOf(node));

    const JsonValue *mesh = node.find("mesh");
    if (mesh != nullptr && mesh->isNumber())
    {
        record.hasMesh   = true;
        record.meshIndex = static_cast<std::size_t>(mesh->number);
    }

    const JsonValue *children = node.find("children");
    if (children != nullptr && children->isArray())
    {
        for (const JsonValue &child : children->array)
        {
            if (child.isNumber())
            {
                record.children.push_back(static_cast<std::size_t>(child.number));
            }
        }
    }

    // Copy the child list before recursing: the record lives in a vector that
    // recursion may reallocate.
    const std::vector<std::size_t> childIndices = record.children;
    const bool                     hasMesh      = record.hasMesh;
    const std::size_t              meshIndex    = record.meshIndex;
    const std::string              name         = record.name;
    const GltfMat4                 world        = record.worldTransform;

    outNodes[nodeIndex]                = NodeRecord{};
    outNodes[nodeIndex].worldTransform = world;
    outNodes[nodeIndex].hasMesh        = hasMesh;
    outNodes[nodeIndex].meshIndex      = meshIndex;
    outNodes[nodeIndex].name           = name;
    outNodes[nodeIndex].children       = childIndices;

    for (std::size_t child : childIndices)
    {
        collectNodes(nodesArray, child, world, outNodes, visited, errorReport);
        if (!errorReport.empty())
        {
            return;
        }
    }
}

// Material factors of one glTF material index (or the default material).
struct ImportedMaterial
{
    std::array<float, 3> albedo    = {0.8f, 0.8f, 0.8f};
    float                roughness = 1.0f;
    float                metallic  = 0.0f;
    float                emissive  = 0.0f;
};

ImportedMaterial materialOf(
    const JsonValue *materialsArray, std::size_t materialIndex
)
{
    ImportedMaterial imported;
    if (materialsArray == nullptr || !materialsArray->isArray() || materialIndex >= materialsArray->array.size())
    {
        return imported;
    }
    const JsonValue &material = materialsArray->array[materialIndex];
    const JsonValue *pbr      = material.find("pbrMetallicRoughness");
    if (pbr != nullptr && pbr->isObject())
    {
        // baseColorFactor is RGBA; the alpha channel has no counterpart in
        // the engine's material model and is ignored.
        const std::array<float, 3> baseColor =
            vec3FromArray(pbr->find("baseColorFactor"), {1.0f, 1.0f, 1.0f});
        imported.albedo = baseColor;
        // find() returns null when the key is absent - the spec DEFAULTS
        // (metallic 1.0, roughness 1.0) apply, exactly like any other
        // optional factor here. The earlier unguarded dereference of
        // find(...)->numberOr crashed on documents that omit either key.
        const JsonValue *metallic  = pbr->find("metallicFactor");
        const JsonValue *roughness = pbr->find("roughnessFactor");
        imported.metallic  = metallic != nullptr ? static_cast<float>(metallic->numberOr(1.0)) : 1.0f;
        imported.roughness = roughness != nullptr ? static_cast<float>(roughness->numberOr(1.0)) : 1.0f;
    }
    const std::array<float, 3> emissiveFactor =
        vec3FromArray(material.find("emissiveFactor"), {0.0f, 0.0f, 0.0f});
    // The engine's emissive is scalar radiance (albedo * strength); the glTF
    // factor is RGB. The strongest channel drives the strength - an honest
    // approximation for the emissive surfaces content authors model.
    imported.emissive =
        std::max(emissiveFactor[0], std::max(emissiveFactor[1], emissiveFactor[2]));
    return imported;
}

// ---- Full-mesh fidelity: buffer decoding and accessor reading --------------

// Base64 value table: -1 outside the alphabet, '=' handled by the caller.
int base64ValueOf(
    char character
)
{
    if (character >= 'A' && character <= 'Z')
    {
        return character - 'A';
    }
    if (character >= 'a' && character <= 'z')
    {
        return character - 'a' + 26;
    }
    if (character >= '0' && character <= '9')
    {
        return character - '0' + 52;
    }
    if (character == '+')
    {
        return 62;
    }
    if (character == '/')
    {
        return 63;
    }
    return -1;
}

} // namespace

bool gltfDecodeBase64(
    const std::string &text, std::vector<std::uint8_t> &outBytes, std::string &errorReport
)
{
    outBytes.clear();
    outBytes.reserve(text.size() / 4 * 3);
    std::uint32_t accumulator = 0;
    int           bits        = 0;
    int           padding     = 0;
    for (const char character : text)
    {
        if (character == ' ' || character == '\t' || character == '\r' || character == '\n')
        {
            continue; // RFC 4648 allows wrapping whitespace; be liberal here
        }
        if (character == '=')
        {
            ++padding;
            continue;
        }
        if (padding > 0)
        {
            errorReport = "base64 payload has data characters after padding";
            return false;
        }
        const int value = base64ValueOf(character);
        if (value < 0)
        {
            errorReport = "base64 payload contains a character outside the alphabet";
            return false;
        }
        accumulator = (accumulator << 6) | static_cast<std::uint32_t>(value);
        bits += 6;
        if (bits >= 8)
        {
            bits -= 8;
            outBytes.push_back(static_cast<std::uint8_t>((accumulator >> bits) & 0xFFu));
        }
    }
    if (padding > 2)
    {
        errorReport = "base64 payload has more than two padding characters";
        return false;
    }
    return true;
}

namespace
{

// One primitive's decoded vertex data: positions and normals are per-vertex
// floats, indices reference them (empty when the primitive is non-indexed -
// the reader then treats the vertices as a triangle list in order).
struct MeshPrimitiveData
{
    std::vector<float>         positions; // 3 floats per vertex
    std::vector<float>         normals;   // 3 floats per vertex; may be empty
    std::vector<std::uint32_t> indices;   // triangle list; may be empty
};

// Grows the local-space AABB of one primitive from its POSITION accessor's
// min/max (both are JSON fields - the bounding-proxy path never decodes
// buffer views).
bool accessorBounds(
    const JsonValue &accessor, std::array<float, 3> &outMin, std::array<float, 3> &outMax
)
{
    const JsonValue *min = accessor.find("min");
    const JsonValue *max = accessor.find("max");
    if (min == nullptr || max == nullptr || !min->isArray() || !max->isArray() || min->array.size() < 3
        || max->array.size() < 3)
    {
        return false;
    }
    outMin = {
        static_cast<float>(min->array[0].numberOr(0.0)),
        static_cast<float>(min->array[1].numberOr(0.0)),
        static_cast<float>(min->array[2].numberOr(0.0)),
    };
    outMax = {
        static_cast<float>(max->array[0].numberOr(0.0)),
        static_cast<float>(max->array[1].numberOr(0.0)),
        static_cast<float>(max->array[2].numberOr(0.0)),
    };
    return true;
}

// Reads one accessor's raw span: resolves bufferView + buffer, validates the
// component/type pair and the byte bounds, and returns the pointer into the
// decoded buffer. Everything the caller does with the data stays local - the
// decoded buffers live in the caller's vector for the whole import.
const std::uint8_t *accessorSpan(
    const JsonValue                         &document,
    const std::vector<std::vector<std::uint8_t>> &bufferData,
    std::size_t                              accessorIndex,
    std::uint32_t                            expectedComponentType,
    const char                              *expectedType,
    std::size_t                             &outElementCount,
    std::size_t                             &outStrideBytes,
    const std::string                       &sourceName,
    std::string                             &errorReport
)
{
    const JsonValue *accessorsArray = document.find("accessors");
    if (accessorsArray == nullptr || !accessorsArray->isArray() || accessorIndex >= accessorsArray->array.size())
    {
        errorReport = sourceName + ": accessor index " + std::to_string(accessorIndex) + " is out of range";
        return nullptr;
    }
    const JsonValue &accessor = accessorsArray->array[accessorIndex];
    const double     count    = accessor.find("count") != nullptr ? accessor.find("count")->numberOr(0.0) : 0.0;
    if (count <= 0.0)
    {
        errorReport = sourceName + ": accessor " + std::to_string(accessorIndex) + " has no usable count";
        return nullptr;
    }
    const double componentType =
        accessor.find("componentType") != nullptr ? accessor.find("componentType")->numberOr(0.0) : 0.0;
    const std::string type = accessor.find("type") != nullptr ? accessor.find("type")->stringOr("") : "";
    if (static_cast<std::uint32_t>(componentType) != expectedComponentType
        || (expectedType != nullptr && type != expectedType))
    {
        errorReport = sourceName + ": accessor " + std::to_string(accessorIndex) + " is " + type
                      + "/" + std::to_string(static_cast<unsigned>(componentType)) + ", expected "
                      + expectedType + "/" + std::to_string(expectedComponentType);
        return nullptr;
    }
    const std::size_t componentSize = expectedComponentType == 5126u ? 4u
                                      : expectedComponentType == 5125u ? 4u
                                      : expectedComponentType == 5123u ? 2u
                                      : 1u;
    const std::size_t componentCount = std::string(expectedType) == "VEC3" ? 3u
                                       : std::string(expectedType) == "SCALAR" ? 1u
                                       : 4u;
    const std::size_t elementSize = componentSize * componentCount;

    const double bufferViewIndex =
        accessor.find("bufferView") != nullptr ? accessor.find("bufferView")->numberOr(-1.0) : -1.0;
    const JsonValue *bufferViewsArray = document.find("bufferViews");
    if (bufferViewIndex < 0.0 || bufferViewsArray == nullptr || !bufferViewsArray->isArray()
        || bufferViewIndex >= static_cast<double>(bufferViewsArray->array.size()))
    {
        errorReport = sourceName + ": accessor " + std::to_string(accessorIndex)
            + " names no usable bufferView";
        return nullptr;
    }
    const JsonValue &bufferView = bufferViewsArray->array[static_cast<std::size_t>(bufferViewIndex)];
    const double     bufferIndex =
        bufferView.find("buffer") != nullptr ? bufferView.find("buffer")->numberOr(-1.0) : -1.0;
    if (bufferIndex < 0.0 || bufferIndex >= static_cast<double>(bufferData.size()))
    {
        errorReport = sourceName + ": bufferView " + std::to_string(static_cast<long long>(bufferViewIndex))
            + " names no usable buffer";
        return nullptr;
    }
    const std::vector<std::uint8_t> &sourceBuffer = bufferData[static_cast<std::size_t>(bufferIndex)];
    std::size_t viewOffset = bufferView.find("byteOffset") != nullptr
                                 ? static_cast<std::size_t>(bufferView.find("byteOffset")->numberOr(0.0))
                                 : 0u;
    std::size_t accessorOffset = accessor.find("byteOffset") != nullptr
                                     ? static_cast<std::size_t>(accessor.find("byteOffset")->numberOr(0.0))
                                     : 0u;
    // A bufferView.byteStride larger than the element size means the accessor
    // is interleaved with other attributes (the glTF vertex layout); the
    // element's own bytes stay contiguous, only the step grows.
    const double strideValue = bufferView.find("byteStride") != nullptr
                                   ? bufferView.find("byteStride")->numberOr(0.0)
                                   : 0.0;
    outStrideBytes = strideValue > static_cast<double>(elementSize) ? static_cast<std::size_t>(strideValue)
                                                                    : elementSize;
    outElementCount = static_cast<std::size_t>(count);
    const std::size_t requiredBytes =
        outElementCount > 0 ? (outElementCount - 1) * outStrideBytes + elementSize : 0;
    if (viewOffset + accessorOffset + requiredBytes > sourceBuffer.size())
    {
        errorReport = sourceName + ": accessor " + std::to_string(accessorIndex)
            + " reads past the end of its buffer";
        return nullptr;
    }
    return sourceBuffer.data() + viewOffset + accessorOffset;
}

// Reads one primitive's positions/normals/indices into MeshPrimitiveData.
// Positions are required; normals are optional (computed below when absent);
// indices are optional (sequential when absent). Only TRIANGLES (mode 4) is
// accepted at mesh fidelity - a mode this renderer cannot draw would be a
// silent lie.
bool readPrimitiveMesh(
    const JsonValue                         &document,
    const JsonValue                         &primitive,
    const std::vector<std::vector<std::uint8_t>> &bufferData,
    MeshPrimitiveData                       &out,
    const std::string                       &sourceName,
    std::string                             &errorReport
)
{
    const double mode = primitive.find("mode") != nullptr ? primitive.find("mode")->numberOr(4.0) : 4.0;
    if (mode != 4.0)
    {
        errorReport = sourceName + ": primitive mode " + std::to_string(static_cast<long long>(mode))
            + ": only triangles (mode 4) are imported at mesh fidelity"
            + " (bounds-only import accepts every primitive)";
        return false;
    }
    const JsonValue *attributes = primitive.find("attributes");
    if (attributes == nullptr || !attributes->isObject())
    {
        errorReport = sourceName + ": primitive has no attributes";
        return false;
    }
    const JsonValue *positionAccessor = attributes->find("POSITION");
    if (positionAccessor == nullptr || !positionAccessor->isNumber())
    {
        errorReport = sourceName + ": primitive has no POSITION attribute";
        return false;
    }
    std::size_t positionCount = 0;
    std::size_t positionStride = 0;
    const std::uint8_t *positionBytes = accessorSpan(
        document, bufferData, static_cast<std::size_t>(positionAccessor->number), 5126u, "VEC3",
        positionCount, positionStride, sourceName, errorReport
    );
    if (positionBytes == nullptr)
    {
        return false;
    }
    out.positions.resize(positionCount * 3u);
    for (std::size_t vertex = 0; vertex < positionCount; ++vertex)
    {
        const float *source = reinterpret_cast<const float *>(positionBytes + vertex * positionStride);
        out.positions[vertex * 3u + 0u] = source[0];
        out.positions[vertex * 3u + 1u] = source[1];
        out.positions[vertex * 3u + 2u] = source[2];
    }

    const JsonValue *normalAccessor = attributes->find("NORMAL");
    if (normalAccessor != nullptr && normalAccessor->isNumber())
    {
        std::size_t normalCount = 0;
        std::size_t normalStride = 0;
        const std::uint8_t *normalBytes = accessorSpan(
            document, bufferData, static_cast<std::size_t>(normalAccessor->number), 5126u, "VEC3",
            normalCount, normalStride, sourceName, errorReport
        );
        if (normalBytes == nullptr)
        {
            return false;
        }
        if (normalCount != positionCount)
        {
            errorReport = sourceName + ": NORMAL accessor covers " + std::to_string(normalCount)
                + " vertices but POSITION covers " + std::to_string(positionCount);
            return false;
        }
        out.normals.resize(normalCount * 3u);
        for (std::size_t vertex = 0; vertex < normalCount; ++vertex)
        {
            const float *source = reinterpret_cast<const float *>(normalBytes + vertex * normalStride);
            out.normals[vertex * 3u + 0u] = source[0];
            out.normals[vertex * 3u + 1u] = source[1];
            out.normals[vertex * 3u + 2u] = source[2];
        }
    }

    const JsonValue *indexAccessor = primitive.find("indices");
    if (indexAccessor != nullptr && indexAccessor->isNumber())
    {
        // Any of the three index component types; the type check runs inside
        // accessorSpan per component, so try them in spec order.
        struct IndexFormat
        {
            std::uint32_t componentType;
            std::size_t   size;
        };
        const IndexFormat formats[] = {{5125u, 4u}, {5123u, 2u}, {5121u, 1u}};
        for (const IndexFormat &format : formats)
        {
            std::size_t indexCount = 0;
            std::size_t indexStride = 0;
            const std::uint8_t *indexBytes = accessorSpan(
                document, bufferData, static_cast<std::size_t>(indexAccessor->number), format.componentType, "SCALAR",
                indexCount, indexStride, sourceName, errorReport
            );
            if (indexBytes == nullptr)
            {
                errorReport.clear(); // wrong component type: try the next one
                continue;
            }
            out.indices.resize(indexCount);
            for (std::size_t index = 0; index < indexCount; ++index)
            {
                std::uint32_t value = 0;
                if (format.size == 4u)
                {
                    value = *reinterpret_cast<const std::uint32_t *>(indexBytes + index * indexStride);
                }
                else if (format.size == 2u)
                {
                    value = *reinterpret_cast<const std::uint16_t *>(indexBytes + index * indexStride);
                }
                else
                {
                    value = *(indexBytes + index * indexStride);
                }
                if (value >= positionCount)
                {
                    errorReport = sourceName + ": index " + std::to_string(value)
                        + " is out of range (" + std::to_string(positionCount) + " vertices)";
                    return false;
                }
                out.indices[index] = value;
            }
            return true;
        }
        if (errorReport.empty())
        {
            errorReport = sourceName + ": indices accessor has no supported component type";
        }
        return false;
    }
    // Non-indexed: the vertices are the triangle list in order.
    out.indices.resize(positionCount);
    for (std::size_t index = 0; index < positionCount; ++index)
    {
        out.indices[index] = static_cast<std::uint32_t>(index);
    }
    return true;
}

// Area-weighted smooth normals for a triangle soup without NORMAL data: one
// accumulation pass, then normalize. Degenerate triangles contribute nothing;
// a vertex no triangle touches keeps the up fallback.
void computeSmoothNormals(
    const std::vector<float> &positions, const std::vector<std::uint32_t> &indices, std::vector<float> &outNormals
)
{
    outNormals.assign(positions.size(), 0.0f);
    const std::size_t vertexCount = positions.size() / 3u;
    for (std::size_t triangle = 0; triangle + 2u < indices.size(); triangle += 3u)
    {
        const std::uint32_t i0 = indices[triangle + 0u];
        const std::uint32_t i1 = indices[triangle + 1u];
        const std::uint32_t i2 = indices[triangle + 2u];
        if (i0 >= vertexCount || i1 >= vertexCount || i2 >= vertexCount)
        {
            continue;
        }
        const float a[3] = {positions[i1 * 3u] - positions[i0 * 3u],
                            positions[i1 * 3u + 1u] - positions[i0 * 3u + 1u],
                            positions[i1 * 3u + 2u] - positions[i0 * 3u + 2u]};
        const float b[3] = {positions[i2 * 3u] - positions[i0 * 3u],
                            positions[i2 * 3u + 1u] - positions[i0 * 3u + 1u],
                            positions[i2 * 3u + 2u] - positions[i0 * 3u + 2u]};
        const float cross[3] = {a[1] * b[2] - a[2] * b[1],
                                a[2] * b[0] - a[0] * b[2],
                                a[0] * b[1] - a[1] * b[0]};
        for (const std::uint32_t corner : {i0, i1, i2})
        {
            outNormals[corner * 3u + 0u] += cross[0];
            outNormals[corner * 3u + 1u] += cross[1];
            outNormals[corner * 3u + 2u] += cross[2];
        }
    }
    for (std::size_t vertex = 0; vertex < vertexCount; ++vertex)
    {
        const float length = std::sqrt(outNormals[vertex * 3u] * outNormals[vertex * 3u]
                                       + outNormals[vertex * 3u + 1u] * outNormals[vertex * 3u + 1u]
                                       + outNormals[vertex * 3u + 2u] * outNormals[vertex * 3u + 2u]);
        if (length > 1e-12f)
        {
            outNormals[vertex * 3u + 0u] /= length;
            outNormals[vertex * 3u + 1u] /= length;
            outNormals[vertex * 3u + 2u] /= length;
        }
        else
        {
            outNormals[vertex * 3u + 0u] = 0.0f;
            outNormals[vertex * 3u + 1u] = 1.0f;
            outNormals[vertex * 3u + 2u] = 0.0f;
        }
    }
}

} // namespace

GltfMat4 gltfMultiplyMatrix(
    const GltfMat4 &a, const GltfMat4 &b
)
{
    // Column-major: result[column * 4 + row].
    GltfMat4 result{};
    for (std::size_t column = 0; column < 4; ++column)
    {
        for (std::size_t row = 0; row < 4; ++row)
        {
            float sum = 0.0f;
            for (std::size_t k = 0; k < 4; ++k)
            {
                sum += a[k * 4 + row] * b[column * 4 + k];
            }
            result[column * 4 + row] = sum;
        }
    }
    return result;
}

GltfMat4 gltfComposeTrs(
    const std::array<float, 3> &translation,
    const std::array<float, 4> &rotationQuaternion,
    const std::array<float, 3> &scale
)
{
    // Unit quaternion to 3x3 rotation (row-major within the 3x3 block).
    const float x = rotationQuaternion[0];
    const float y = rotationQuaternion[1];
    const float z = rotationQuaternion[2];
    const float w = rotationQuaternion[3];
    const float xx = x * x, yy = y * y, zz = z * z;
    const float xy = x * y, xz = x * z, yz = y * z;
    const float wx = w * x, wy = w * y, wz = w * z;

    GltfMat4 result{};
    result[0]  = (1.0f - 2.0f * (yy + zz)) * scale[0]; // column 0, row 0
    result[1]  = (2.0f * (xy + wz)) * scale[0];        // column 0, row 1
    result[2]  = (2.0f * (xz - wy)) * scale[0];        // column 0, row 2
    result[4]  = (2.0f * (xy - wz)) * scale[1];        // column 1, row 0
    result[5]  = (1.0f - 2.0f * (xx + zz)) * scale[1];
    result[6]  = (2.0f * (yz + wx)) * scale[1];
    result[8]  = (2.0f * (xz + wy)) * scale[2];
    result[9]  = (2.0f * (yz - wx)) * scale[2];
    result[10] = (1.0f - 2.0f * (xx + yy)) * scale[2];
    result[12] = translation[0];
    result[13] = translation[1];
    result[14] = translation[2];
    result[15] = 1.0f;
    return result;
}

std::array<float, 3> gltfTransformPoint(
    const GltfMat4 &matrix, const std::array<float, 3> &point
)
{
    return {
        matrix[0] * point[0] + matrix[4] * point[1] + matrix[8] * point[2] + matrix[12],
        matrix[1] * point[0] + matrix[5] * point[1] + matrix[9] * point[2] + matrix[13],
        matrix[2] * point[0] + matrix[6] * point[1] + matrix[10] * point[2] + matrix[14],
    };
}

std::uint32_t gltfImportIntoScene(
    const JsonValue &document,
    SceneCapability &scene,
    const std::string &sourceName,
    std::uint32_t fidelity,
    std::string &errorReport
)
{
    // The three top-level arrays the importer consumes.
    const JsonValue *nodesArray     = document.find("nodes");
    const JsonValue *meshesArray    = document.find("meshes");
    const JsonValue *materialsArray = document.find("materials");
    if (nodesArray == nullptr || !nodesArray->isArray())
    {
        errorReport = sourceName + ": the document has no nodes array - nothing to import";
        return 0;
    }
    if (meshesArray == nullptr || !meshesArray->isArray())
    {
        errorReport = sourceName + ": the document has no meshes array - nothing to import";
        return 0;
    }

    // The root nodes: the default scene, or scene 0, or every parentless node
    // (the spec's fallback for files without a scenes array).
    std::vector<std::size_t> rootIndices;
    const JsonValue         *scenesArray = document.find("scenes");
    if (scenesArray != nullptr && scenesArray->isArray() && !scenesArray->array.empty())
    {
        const double sceneNumber = document.find("scene") != nullptr && document.find("scene")->isNumber()
                                       ? document.find("scene")->number
                                       : 0.0;
        const std::size_t sceneIndex =
            sceneNumber >= 0.0 && sceneNumber < static_cast<double>(scenesArray->array.size())
                ? static_cast<std::size_t>(sceneNumber)
                : 0;
        const JsonValue *scene = &scenesArray->array[sceneIndex];
        const JsonValue *roots = scene->find("nodes");
        if (roots != nullptr && roots->isArray())
        {
            for (const JsonValue &root : roots->array)
            {
                if (root.isNumber())
                {
                    rootIndices.push_back(static_cast<std::size_t>(root.number));
                }
            }
        }
    }
    if (rootIndices.empty())
    {
        // Fallback: every node no other node lists as a child.
        std::vector<bool> isChild(nodesArray->array.size(), false);
        for (const JsonValue &node : nodesArray->array)
        {
            const JsonValue *children = node.find("children");
            if (children == nullptr || !children->isArray())
            {
                continue;
            }
            for (const JsonValue &child : children->array)
            {
                if (child.isNumber() && child.number >= 0.0
                    && child.number < static_cast<double>(isChild.size()))
                {
                    isChild[static_cast<std::size_t>(child.number)] = true;
                }
            }
        }
        for (std::size_t index = 0; index < isChild.size(); ++index)
        {
            if (!isChild[index])
            {
                rootIndices.push_back(index);
            }
        }
    }
    if (rootIndices.empty())
    {
        errorReport = sourceName + ": the document defines no root nodes";
        return 0;
    }

    // One world-transform walk over the whole hierarchy.
    std::vector<NodeRecord> nodes(nodesArray->array.size());
    std::vector<bool>       visited(nodesArray->array.size(), false);
    GltfMat4                identity{};
    identity[0] = identity[5] = identity[10] = identity[15] = 1.0f;
    for (std::size_t root : rootIndices)
    {
        collectNodes(*nodesArray, root, identity, nodes, visited, errorReport);
        if (!errorReport.empty())
        {
            errorReport = sourceName + ": " + errorReport;
            return 0;
        }
    }

    // Spawn one object per mesh primitive, at the requested fidelity.
    std::uint32_t spawned = 0;

    // Full-mesh fidelity needs the embedded buffers: decoded up front, once
    // per buffer (a document may reference several). External URIs are
    // refused: the .gltf JSON form stays self-contained (the module README
    // documents the mapping).
    const JsonValue *buffersArray = document.find("buffers");
    std::vector<std::vector<std::uint8_t>> bufferData;
    if (fidelity == asset_fidelity::full_mesh)
    {
        if (buffersArray == nullptr || !buffersArray->isArray() || buffersArray->array.empty())
        {
            errorReport = sourceName
                + ": the document has no buffers array; full-mesh import needs embedded buffer data"
                + " (use bounding-proxy fidelity, or export with embedded buffers)";
            return 0;
        }
        bufferData.resize(buffersArray->array.size());
        for (std::size_t bufferIndex = 0; bufferIndex < buffersArray->array.size(); ++bufferIndex)
        {
            const JsonValue &buffer = buffersArray->array[bufferIndex];
            const std::string uri = buffer.find("uri") != nullptr ? buffer.find("uri")->stringOr("") : "";
            constexpr const char *dataPrefix    = "data:";
            constexpr const char *base64Marker  = ";base64,";
            const std::size_t markerPosition = uri.find(base64Marker);
            if (uri.rfind(dataPrefix, 0) != 0 || markerPosition == std::string::npos)
            {
                errorReport = sourceName + ": buffer " + std::to_string(bufferIndex)
                    + " references external data; full-mesh import reads embedded data URIs only ("
                    + "data:application/octet-stream;base64,...)";
                return 0;
            }
            std::string decodeError;
            if (!gltfDecodeBase64(
                    uri.substr(markerPosition + std::string(base64Marker).size()), bufferData[bufferIndex], decodeError
                ))
            {
                errorReport = sourceName + ": buffer " + std::to_string(bufferIndex) + ": " + decodeError;
                return 0;
            }
        }
    }

    for (std::size_t nodeIndex = 0; nodeIndex < nodes.size(); ++nodeIndex)
    {
        const NodeRecord &node = nodes[nodeIndex];
        if (!node.hasMesh || node.meshIndex >= meshesArray->array.size())
        {
            continue;
        }
        const JsonValue &mesh = meshesArray->array[node.meshIndex];
        const JsonValue *primitives = mesh.find("primitives");
        if (primitives == nullptr || !primitives->isArray())
        {
            continue;
        }
        std::size_t primitiveIndex = 0;
        for (const JsonValue &primitive : primitives->array)
        {
            const JsonValue *materialIndex = primitive.find("material");
            const ImportedMaterial material = materialIndex != nullptr && materialIndex->isNumber()
                                                  ? materialOf(materialsArray, static_cast<std::size_t>(materialIndex->number))
                                                  : ImportedMaterial{};

            // ---- Full-mesh fidelity: register the triangles, instance them
            if (fidelity == asset_fidelity::full_mesh)
            {
                MeshPrimitiveData meshData;
                if (readPrimitiveMesh(document, primitive, bufferData, meshData, sourceName, errorReport))
                {
                    // Bake the node transform into the vertices EXCEPT the
                    // translation: the engine's rotation model is Y-only, so
                    // a baked transform (any rotation, any scale) always
                    // lands exactly. The object's position carries the node
                    // origin plus the mesh centroid; the mesh data is
                    // CENTERED on the object's origin (the registry's proxy
                    // box and UV parameterization assume a centered mesh).
                    const std::array<float, 3> nodeOrigin = gltfTransformPoint(node.worldTransform, {0.0f, 0.0f, 0.0f});
                    const std::size_t vertexCount = meshData.positions.size() / 3u;
                    std::vector<float> baked(meshData.positions.size());
                    std::array<float, 3> minimum{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                                                 std::numeric_limits<float>::max()};
                    std::array<float, 3> maximum{-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
                                                 -std::numeric_limits<float>::max()};
                    for (std::size_t vertex = 0; vertex < vertexCount; ++vertex)
                    {
                        const std::array<float, 3> world = gltfTransformPoint(
                            node.worldTransform,
                            {meshData.positions[vertex * 3u + 0u], meshData.positions[vertex * 3u + 1u],
                             meshData.positions[vertex * 3u + 2u]}
                        );
                        for (std::size_t axis = 0; axis < 3; ++axis)
                        {
                            baked[vertex * 3u + axis] = world[axis] - nodeOrigin[axis];
                            minimum[axis] = std::min(minimum[axis], baked[vertex * 3u + axis]);
                            maximum[axis] = std::max(maximum[axis], baked[vertex * 3u + axis]);
                        }
                    }
                    const std::array<float, 3> centroid{(minimum[0] + maximum[0]) * 0.5f,
                                                        (minimum[1] + maximum[1]) * 0.5f,
                                                        (minimum[2] + maximum[2]) * 0.5f};
                    for (std::size_t vertex = 0; vertex < vertexCount; ++vertex)
                    {
                        for (std::size_t axis = 0; axis < 3; ++axis)
                        {
                            baked[vertex * 3u + axis] -= centroid[axis];
                        }
                    }
                    // Normals: rotate/scale with the vertices. The glTF data
                    // carries object-space normals; the baked transform's
                    // linear part applies through the inverse-transpose - for
                    // the transforms content ships (rigid, uniform scale)
                    // that is the same 3x3 renormalized, and the renderer
                    // normalizes per-vertex anyway. Recompute when the source
                    // had none.
                    std::vector<float> normals = meshData.normals;
                    if (normals.empty())
                    {
                        computeSmoothNormals(baked, meshData.indices, normals);
                    }
                    else
                    {
                        for (std::size_t vertex = 0; vertex < vertexCount; ++vertex)
                        {
                            const std::array<float, 3> source{normals[vertex * 3u + 0u], normals[vertex * 3u + 1u],
                                                              normals[vertex * 3u + 2u]};
                            // The upper 3x3 of the world transform, applied
                            // per column (column-major storage), then
                            // normalized - exact for rigid + uniform scale.
                            for (std::size_t axis = 0; axis < 3; ++axis)
                            {
                                normals[vertex * 3u + axis] =
                                    node.worldTransform[axis * 4u + 0u] * source[0]
                                    + node.worldTransform[axis * 4u + 1u] * source[1]
                                    + node.worldTransform[axis * 4u + 2u] * source[2];
                            }
                        }
                    }

                    SceneMeshDescriptor meshDescriptor;
                    meshDescriptor.vertices.resize(vertexCount * 6u);
                    for (std::size_t vertex = 0; vertex < vertexCount; ++vertex)
                    {
                        for (std::size_t axis = 0; axis < 3; ++axis)
                        {
                            meshDescriptor.vertices[vertex * 6u + axis] = baked[vertex * 3u + axis];
                            meshDescriptor.vertices[vertex * 6u + 3u + axis] = normals[vertex * 3u + axis];
                        }
                    }
                    meshDescriptor.indices = meshData.indices;
                    const std::string meshName = (node.name.empty() ? "mesh" : node.name) + "#n"
                        + std::to_string(nodeIndex) + "p" + std::to_string(primitiveIndex);
                    const SceneCapability::MeshHandle meshHandle = scene.addMesh(meshName, meshDescriptor);
                    if (meshHandle != SceneCapability::invalidMesh)
                    {
                        SceneObjectDescriptor object;
                        object.type       = scene_object_type::mesh;
                        object.meshHandle = meshHandle;
                        object.position   = {nodeOrigin[0] + centroid[0], nodeOrigin[1] + centroid[1],
                                             nodeOrigin[2] + centroid[2]};
                        object.size       = {std::max((maximum[0] - minimum[0]) * 0.5f, 0.001f),
                                             std::max((maximum[1] - minimum[1]) * 0.5f, 0.001f),
                                             std::max((maximum[2] - minimum[2]) * 0.5f, 0.001f)};
                        object.albedo           = material.albedo;
                        object.roughness        = material.roughness;
                        object.metallic         = material.metallic;
                        object.emissiveStrength = material.emissive;
                        if (scene.addObject(object) == SceneCapability::invalidObject)
                        {
                            return spawned;
                        }
                        ++spawned;
                        ++primitiveIndex;
                        continue;
                    }
                    // invalidMesh: the mesh registry is full or refused the
                    // data (its refusal is logged precisely there). Fall
                    // through to the bounding-box proxy so the rest of the
                    // level still arrives - a documented partial import.
                }
                else
                {
                    // Structural read failure at mesh fidelity: precise
                    // failure of the whole import (the caller reports it).
                    errorReport = sourceName + ": " + errorReport;
                    return 0;
                }
            }

            // ---- Bounding-proxy fidelity (and the mesh registry's fallback):
            // one box from the POSITION accessor's min/max.
            const JsonValue *attributes = primitive.find("attributes");
            if (attributes == nullptr || !attributes->isObject())
            {
                ++primitiveIndex;
                continue;
            }
            const JsonValue *positionAccessor = attributes->find("POSITION");
            if (positionAccessor == nullptr || !positionAccessor->isNumber())
            {
                ++primitiveIndex;
                continue; // a primitive without positions draws nothing anywhere
            }
            const JsonValue *accessorsArray = document.find("accessors");
            if (accessorsArray == nullptr || !accessorsArray->isArray()
                || positionAccessor->number < 0.0
                || positionAccessor->number >= static_cast<double>(accessorsArray->array.size()))
            {
                ++primitiveIndex;
                continue;
            }
            std::array<float, 3> localMin{};
            std::array<float, 3> localMax{};
            if (!accessorBounds(accessorsArray->array[static_cast<std::size_t>(positionAccessor->number)], localMin, localMax))
            {
                ++primitiveIndex;
                continue; // no min/max: the accessor was written without bounds
            }

            // All 8 corners through the world transform, then the world AABB
            // (a rotated box's bounds are the corners' bounds).
            std::array<float, 3> worldMin{std::numeric_limits<float>::max(),
                                          std::numeric_limits<float>::max(),
                                          std::numeric_limits<float>::max()};
            std::array<float, 3> worldMax{-std::numeric_limits<float>::max(),
                                          -std::numeric_limits<float>::max(),
                                          -std::numeric_limits<float>::max()};
            for (int corner = 0; corner < 8; ++corner)
            {
                const std::array<float, 3> local{corner & 1 ? localMax[0] : localMin[0],
                                                 corner & 2 ? localMax[1] : localMin[1],
                                                 corner & 4 ? localMax[2] : localMin[2]};
                const std::array<float, 3> world = gltfTransformPoint(node.worldTransform, local);
                for (std::size_t axis = 0; axis < 3; ++axis)
                {
                    worldMin[axis] = std::min(worldMin[axis], world[axis]);
                    worldMax[axis] = std::max(worldMax[axis], world[axis]);
                }
            }

            SceneObjectDescriptor object;
            object.type    = scene_object_type::box;
            object.size    = {std::max((worldMax[0] - worldMin[0]) * 0.5f, 0.001f),
                              std::max((worldMax[1] - worldMin[1]) * 0.5f, 0.001f),
                              std::max((worldMax[2] - worldMin[2]) * 0.5f, 0.001f)};
            object.position = {(worldMin[0] + worldMax[0]) * 0.5f,
                               (worldMin[1] + worldMax[1]) * 0.5f,
                               (worldMin[2] + worldMax[2]) * 0.5f};
            object.albedo           = material.albedo;
            object.roughness        = material.roughness;
            object.metallic         = material.metallic;
            object.emissiveStrength = material.emissive;

            if (scene.addObject(object) == SceneCapability::invalidObject)
            {
                // Registry full: report what arrived. The caller decides
                // whether that is acceptable for this asset.
                return spawned;
            }
            ++spawned;
            ++primitiveIndex;
        }
    }
    return spawned;
}

} // namespace engine
