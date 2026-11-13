// STL importer for both binary and ASCII flavors.
//
// Facets are appended into a single part with bit-exact welding on the transformed
// position (plus the face normal, except when smooth normals are generated).
// Normals come from the file, from the geometry, or are regenerated afterwards
// according to normal_mode.

#include "model_import_internal.hpp"

#include <algorithm>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine::models::detail
{
namespace
{
// Weld key over the transformed position's bit patterns (-0.0 folded to +0.0) and
// the face normal. In smooth mode the normal fields are zeroed so corners weld
// across faces and smooth normals can accumulate over shared vertices.
struct StlPositionKey
{
    std::uint32_t x  = 0;
    std::uint32_t y  = 0;
    std::uint32_t z  = 0;
    std::uint32_t nx = 0;
    std::uint32_t ny = 0;
    std::uint32_t nz = 0;

    bool operator==(const StlPositionKey &) const = default;
};

// Splitmix64-style mix of the six key words.
struct StlPositionKeyHash
{
    std::size_t operator()(const StlPositionKey &key) const noexcept
    {
        std::uint64_t value = static_cast<std::uint64_t>(key.x) << 32 | key.y;
        value ^= static_cast<std::uint64_t>(key.z) * 0x9e3779b97f4a7c15ull;
        value ^= static_cast<std::uint64_t>(key.nx) * 0xbf58476d1ce4e5b9ull;
        value ^= static_cast<std::uint64_t>(key.ny) * 0x94d049bb133111ebull;
        value ^= static_cast<std::uint64_t>(key.nz) * 0x369dea0f31a53f85ull;
        value ^= value >> 30;
        value *= 0xbf58476d1ce4e5b9ull;
        value ^= value >> 27;
        value *= 0x94d049bb133111ebull;
        value ^= value >> 31;
        return static_cast<std::size_t>(value);
    }
};

// One parsed facet: facet normal plus three vertices, straight from the file.
struct StlTriangle
{
    float normal[3]{};
    float vertices[3][3]{};
};

} // namespace

// Imports one STL file (binary or ASCII, auto-detected by size arithmetic) into a
// single part named after the file stem.
bool importStlFile(const std::filesystem::path &path, ModelRecord &record, const ModelLoadOptions &options,
    const ModelTransform &transform, ModelLoadError *error)
{
    std::error_code     fileError;
    const std::uint64_t fileSize = std::filesystem::file_size(path, fileError);
    if (fileError)
    {
        reportError(error, ModelLoadError::ParseFailed);
        return false;
    }

    ModelPart part;
    part.name = path.stem().string();

    // STL carries no shared vertices: welding is needed for smooth normals, and
    // flat mode disables welding because every corner needs its own vertex.
    const bool smooth = options.normalMode == ModelNormalMode::GenerateSmooth;
    const bool flat   = options.normalMode == ModelNormalMode::GenerateFlat;
    std::unordered_map<StlPositionKey, std::uint32_t, StlPositionKeyHash> weldedVertices;
    const bool    canWeldVertices    = !flat && (options.weldVertices || smooth);
    std::uint64_t triangleCount      = 0;
    bool          indexLimitExceeded = false;

    // Transforms and appends one facet: weld-or-emit each corner, accumulate bounds.
    // Returns false on non-finite input or when the 32-bit index space would overflow.
    auto appendTriangle = [&](const StlTriangle &triangle) -> bool
    {
        if (triangleCount >= std::numeric_limits<std::uint32_t>::max() / 3u)
        {
            indexLimitExceeded = true;
            return false;
        }
        float transformedTriangle[3][3]{};
        for (std::size_t vertex = 0; vertex < 3; ++vertex)
        {
            for (const float component : triangle.vertices[vertex])
            {
                if (!std::isfinite(component))
                {
                    return false;
                }
            }
            transformPosition(transform, triangle.vertices[vertex], transformedTriangle[vertex]);
            for (const float component : transformedTriangle[vertex])
            {
                if (!std::isfinite(component))
                {
                    return false;
                }
            }
        }
        float geometricNormal[3]{};
        auto  calculateGeometricNormal = [&]()
        {
            const float edgeAB[3]
                = {transformedTriangle[1][0] - transformedTriangle[0][0],
                    transformedTriangle[1][1] - transformedTriangle[0][1],
                    transformedTriangle[1][2] - transformedTriangle[0][2]};
            const float edgeAC[3]
                = {transformedTriangle[2][0] - transformedTriangle[0][0],
                    transformedTriangle[2][1] - transformedTriangle[0][1],
                    transformedTriangle[2][2] - transformedTriangle[0][2]};
            geometricNormal[0] = edgeAB[1] * edgeAC[2] - edgeAB[2] * edgeAC[1];
            geometricNormal[1] = edgeAB[2] * edgeAC[0] - edgeAB[0] * edgeAC[2];
            geometricNormal[2] = edgeAB[0] * edgeAC[1] - edgeAB[1] * edgeAC[0];
            normalizeVector(geometricNormal);
        };

        // Prefer the file's facet normal; degenerate (near-zero) normals fall back to
        // the geometric one. Flat/smooth modes always derive normals from geometry.
        float sourceNormal[3] = {0.0f, 1.0f, 0.0f};
        if (!flat && !smooth)
        {
            if (!std::isfinite(triangle.normal[0]) || !std::isfinite(triangle.normal[1])
                || !std::isfinite(triangle.normal[2]))
            {
                return false;
            }
            transformNormal(transform, triangle.normal, sourceNormal);
            if (std::abs(triangle.normal[0]) + std::abs(triangle.normal[1]) + std::abs(triangle.normal[2]) <= 1e-12f)
            {
                calculateGeometricNormal();
                std::copy(geometricNormal, geometricNormal + 3, sourceNormal);
            }
        }
        else
        {
            calculateGeometricNormal();
            std::copy(geometricNormal, geometricNormal + 3, sourceNormal);
            // Mirrored transforms flip triangle handedness, so flat normals flip too.
            if (flat && transform.reverseWinding)
            {
                sourceNormal[0] = -sourceNormal[0];
                sourceNormal[1] = -sourceNormal[1];
                sourceNormal[2] = -sourceNormal[2];
            }
        }

        for (std::size_t vertex = 0; vertex < 3; ++vertex)
        {
            const float   *position = transformedTriangle[vertex];
            // Fold -0.0f into +0.0f on both positions and normals so welding
            // treats signed-zero normals identically (a face with normal
            // (0,-0,1) welds the same as one with (0,0,1)).
            StlPositionKey key{
                .x  = std::bit_cast<std::uint32_t>(canonicalizeZero(position[0])),
                .y  = std::bit_cast<std::uint32_t>(canonicalizeZero(position[1])),
                .z  = std::bit_cast<std::uint32_t>(canonicalizeZero(position[2])),
                .nx = smooth ? 0u : std::bit_cast<std::uint32_t>(canonicalizeZero(sourceNormal[0])),
                .ny = smooth ? 0u : std::bit_cast<std::uint32_t>(canonicalizeZero(sourceNormal[1])),
                .nz = smooth ? 0u : std::bit_cast<std::uint32_t>(canonicalizeZero(sourceNormal[2]))};
            const auto found = canWeldVertices ? weldedVertices.find(key) : weldedVertices.end();
            if (found != weldedVertices.end())
            {
                part.indices.push_back(found->second);
                continue;
            }

            const std::uint32_t localIndex
                = static_cast<std::uint32_t>(part.vertices.size() / renderer::meshVertexFloatCount);
            const std::size_t offset = part.vertices.size();
            part.vertices.resize(offset + renderer::meshVertexFloatCount);
            std::copy(position, position + 3, part.vertices.begin() + offset);
            std::copy(sourceNormal, sourceNormal + 3, part.vertices.begin() + offset + 3);
            part.vertices[offset + 6]  = 1.0f;
            part.vertices[offset + 7]  = 1.0f;
            part.vertices[offset + 8]  = 1.0f;
            part.vertices[offset + 9]  = 0.0f;
            part.vertices[offset + 10] = 0.0f;
            part.indices.push_back(localIndex);
            if (canWeldVertices)
            {
                weldedVertices.emplace(key, localIndex);
            }
        }
        ++triangleCount;
        return true;
    };

    // Binary STL: 80-byte header, uint32 facet count, then 50 bytes per facet.
    // The file counts as binary only when its size matches that arithmetic exactly.
    bool          binaryFile          = false;
    std::uint32_t binaryTriangleCount = 0;
    if (fileSize >= 84)
    {
        std::ifstream input(path, std::ios::binary);
        if (input)
        {
            input.seekg(80, std::ios::beg);
            unsigned char countBytes[4]{};
            input.read(reinterpret_cast<char *>(countBytes), sizeof(countBytes));
            if (input)
            {
                binaryTriangleCount = static_cast<std::uint32_t>(countBytes[0])
                                    | (static_cast<std::uint32_t>(countBytes[1]) << 8)
                                    | (static_cast<std::uint32_t>(countBytes[2]) << 16)
                                    | (static_cast<std::uint32_t>(countBytes[3]) << 24);
                binaryFile          = 84ull + static_cast<std::uint64_t>(binaryTriangleCount) * 50ull == fileSize;
            }
        }
    }

    if (binaryFile)
    {
        if (binaryTriangleCount > std::numeric_limits<std::uint32_t>::max() / 3u)
        {
            reportError(error, ModelLoadError::FileTooLarge);
            return false;
        }
        const std::size_t maximumVertices = static_cast<std::size_t>(binaryTriangleCount) * 3;
        if (maximumVertices > maximumVertexElementCount / renderer::meshVertexFloatCount
            || maximumVertices > maximumVertexElementCount)
        {
            reportError(error, ModelLoadError::FileTooLarge);
            return false;
        }
        // Reserve up front; welded meshes usually end far below the 3x worst case.
        constexpr std::size_t initialWeldCapacity = 262144;
        const std::size_t     initialVertexCapacity
            = canWeldVertices ? std::min(maximumVertices, initialWeldCapacity) : maximumVertices;
        part.vertices.reserve(initialVertexCapacity * renderer::meshVertexFloatCount);
        part.indices.reserve(maximumVertices);
        if (canWeldVertices)
        {
            weldedVertices.reserve(initialVertexCapacity);
        }

        std::ifstream input(path, std::ios::binary);
        input.seekg(84, std::ios::beg);
        auto readFloat = [](const unsigned char *bytes)
        {
            const std::uint32_t bits
                = static_cast<std::uint32_t>(bytes[0]) | (static_cast<std::uint32_t>(bytes[1]) << 8)
                | (static_cast<std::uint32_t>(bytes[2]) << 16) | (static_cast<std::uint32_t>(bytes[3]) << 24);
            return std::bit_cast<float>(bits);
        };
        // Read in fixed-size batches instead of one stream read per facet.
        constexpr std::size_t      trianglesPerBatch = 2048;
        std::vector<unsigned char> batch(trianglesPerBatch * 50);
        std::uint32_t              triangleIndex = 0;
        while (triangleIndex < binaryTriangleCount)
        {
            const std::size_t batchCount = std::min<std::size_t>(
                trianglesPerBatch, static_cast<std::size_t>(binaryTriangleCount - triangleIndex));
            const std::size_t batchBytes = batchCount * 50;
            input.read(reinterpret_cast<char *>(batch.data()), static_cast<std::streamsize>(batchBytes));
            if (input.gcount() != static_cast<std::streamsize>(batchBytes))
            {
                reportError(error, ModelLoadError::ParseFailed);
                return false;
            }
            for (std::size_t batchIndex = 0; batchIndex < batchCount; ++batchIndex, ++triangleIndex)
            {
                const unsigned char *triangleBytes = batch.data() + batchIndex * 50;
                StlTriangle          triangle;
                for (std::size_t axis = 0; axis < 3; ++axis)
                {
                    triangle.normal[axis] = readFloat(triangleBytes + axis * 4);
                }
                for (std::size_t vertex = 0; vertex < 3; ++vertex)
                {
                    for (std::size_t axis = 0; axis < 3; ++axis)
                    {
                        const std::size_t offset        = 12 + (vertex * 3 + axis) * 4;
                        triangle.vertices[vertex][axis] = readFloat(triangleBytes + offset);
                    }
                }
                if (!appendTriangle(triangle))
                {
                    reportError(error, indexLimitExceeded ? ModelLoadError::FileTooLarge : ModelLoadError::ParseFailed);
                    return false;
                }
            }
        }
    }
    else
    {
        std::ifstream input(path);
        if (!input)
        {
            reportError(error, ModelLoadError::ParseFailed);
            return false;
        }
        // Minimal whitespace tokenizer and from_chars float parsing; the facet
        // structure (facet/outer/loop/vertex x3/endloop/endfacet) must match exactly.
        std::string line;
        auto        nextToken = [](std::string_view &fields, std::string_view &token)
        {
            while (!fields.empty() && std::isspace(static_cast<unsigned char>(fields.front())) != 0)
            {
                fields.remove_prefix(1);
            }
            if (fields.empty())
            {
                return false;
            }
            const std::size_t end = fields.find_first_of(" \t\r\n\f\v");
            token                 = fields.substr(0, end);
            fields.remove_prefix(end == std::string_view::npos ? fields.size() : end);
            return true;
        };
        auto nextFloat = [&](std::string_view &fields, float &value)
        {
            std::string_view ignored;
            while (!fields.empty() && std::isspace(static_cast<unsigned char>(fields.front())) != 0)
            {
                fields.remove_prefix(1);
            }
            if (!fields.empty() && fields.front() == '+')
            {
                fields.remove_prefix(1);
            }
            const char *begin  = fields.data();
            const char *end    = begin + fields.size();
            const auto  parsed = std::from_chars(begin, end, value, std::chars_format::general);
            if (parsed.ec != std::errc{} || parsed.ptr == begin)
            {
                return false;
            }
            fields.remove_prefix(static_cast<std::size_t>(parsed.ptr - begin));
            return true;
        };
        auto onlyExpectedTags = [&](const char *expectedFirst, const char *expectedSecond)
        {
            if (!std::getline(input, line))
            {
                return false;
            }
            std::string_view fields(line);
            std::string_view token;
            if (!nextToken(fields, token) || token != expectedFirst)
            {
                return false;
            }
            if (expectedSecond[0] != '\0' && (!nextToken(fields, token) || token != expectedSecond))
            {
                return false;
            }
            return !nextToken(fields, token);
        };
        while (std::getline(input, line))
        {
            std::string_view fields(line);
            std::string_view token;
            if (!nextToken(fields, token) || token != "facet")
            {
                continue;
            }
            StlTriangle triangle;
            if (!nextToken(fields, token) || token != "normal" || !nextFloat(fields, triangle.normal[0])
                || !nextFloat(fields, triangle.normal[1]) || !nextFloat(fields, triangle.normal[2])
                || nextToken(fields, token) || !onlyExpectedTags("outer", "loop"))
            {
                reportError(error, ModelLoadError::ParseFailed);
                return false;
            }
            for (std::size_t vertex = 0; vertex < 3; ++vertex)
            {
                if (!std::getline(input, line))
                {
                    reportError(error, ModelLoadError::ParseFailed);
                    return false;
                }
                fields = line;
                if (!nextToken(fields, token) || token != "vertex" || !nextFloat(fields, triangle.vertices[vertex][0])
                    || !nextFloat(fields, triangle.vertices[vertex][1])
                    || !nextFloat(fields, triangle.vertices[vertex][2]) || nextToken(fields, token))
                {
                    reportError(error, ModelLoadError::ParseFailed);
                    return false;
                }
            }
            if (!onlyExpectedTags("endloop", "") || !onlyExpectedTags("endfacet", "") || !appendTriangle(triangle))
            {
                reportError(error, indexLimitExceeded ? ModelLoadError::FileTooLarge : ModelLoadError::ParseFailed);
                return false;
            }
        }
        if (input.bad())
        {
            reportError(error, ModelLoadError::ParseFailed);
            return false;
        }
    }

    if (triangleCount == 0)
    {
        reportError(error, ModelLoadError::NoGeometry);
        return false;
    }
    // Winding is flipped only after all facets are in, so weld keys stay consistent.
    if (transform.reverseWinding)
    {
        reverseTriangleWinding(part.indices);
    }
    if (options.importMaterials)
    {
        part.triangleMaterials.assign(static_cast<std::size_t>(triangleCount), invalidMaterial);
    }
    // Smooth mode replaces every per-face normal with the accumulated smooth one.
    if (smooth)
    {
        std::vector<float> smoothNormals;
        if (!accumulateIndexedSmoothNormals(part.vertices, part.indices, smoothNormals))
        {
            reportError(error, ModelLoadError::ParseFailed);
            return false;
        }
        for (std::size_t vertex = 0; vertex < part.vertices.size() / renderer::meshVertexFloatCount; ++vertex)
        {
            std::copy_n(
                smoothNormals.data() + vertex * 3, 3, part.vertices.data() + vertex * renderer::meshVertexFloatCount + 3);
        }
    }
    record.parts.push_back(std::move(part));
    // Bounds are accumulated from the final vertex buffer in one pass. This is
    // cheaper than touching every corner inside appendTriangle and keeps the
    // facet loop tight. `record.parts.back()` is the part we just moved in,
    // so its vertex buffer holds the transformed positions.
    accumulateBounds(record.parts.back(), record);
    reportError(error, ModelLoadError::None);
    return true;
}

} // namespace engine::models::detail
