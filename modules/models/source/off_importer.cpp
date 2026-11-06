// OFF/COFF importer: plain-text polygon meshes with optional per-vertex colors
// (COFF). Polygons are fan-triangulated and emitted as a single part named after
// the file stem; normals are always generated because OFF carries none.

#include "model_import_internal.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <istream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace engine::models::detail
{
namespace
{
// Whitespace-delimited tokenizer that skips '#' comments to end of line.
bool nextToken(std::istream &input, std::string &token)
{
    token.clear();
    for (;;)
    {
        const int next = input.peek();
        if (next == std::char_traits<char>::eof())
        {
            return false;
        }
        const unsigned char character = static_cast<unsigned char>(next);
        if (std::isspace(character))
        {
            input.get();
            continue;
        }
        if (character == '#')
        {
            input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }
        break;
    }
    while (true)
    {
        const int next = input.peek();
        if (next == std::char_traits<char>::eof() || std::isspace(static_cast<unsigned char>(next)) || next == '#')
        {
            break;
        }
        token.push_back(static_cast<char>(input.get()));
    }
    return !token.empty();
}

// Strict decimal parse with a uint32 overflow guard.
bool parseUnsigned(const std::string &token, std::uint32_t &value)
{
    if (token.empty())
    {
        return false;
    }
    std::uint64_t parsed = 0;
    for (const unsigned char character : token)
    {
        if (character < '0' || character > '9')
        {
            return false;
        }
        parsed = parsed * 10u + static_cast<std::uint64_t>(character - '0');
        if (parsed > std::numeric_limits<std::uint32_t>::max())
        {
            return false;
        }
    }
    value = static_cast<std::uint32_t>(parsed);
    return true;
}

// strtod-based parse that rejects ERANGE, trailing garbage, and non-finite values.
bool parseFiniteFloat(const std::string &token, float &value)
{
    errno               = 0;
    char        *end    = nullptr;
    const double parsed = std::strtod(token.c_str(), &end);
    if (errno == ERANGE || end == token.c_str() || *end != '\0' || !std::isfinite(parsed)
        || std::abs(parsed) > std::numeric_limits<float>::max())
    {
        return false;
    }
    value = static_cast<float>(parsed);
    return std::isfinite(value);
}
} // namespace

// Imports an OFF/COFF file; colors are read only for COFF with importVertexColors.
bool importOffFile(const std::filesystem::path &path, ModelRecord &record, const ModelLoadOptions &options,
    const ModelTransform &transform, ModelLoadError *error)
{
    std::ifstream input(path, std::ios::binary);
    if (!input)
    {
        reportError(error, ModelLoadError::FileNotFound);
        return false;
    }

    std::string token;
    if (!nextToken(input, token) || (token != "OFF" && token != "COFF"))
    {
        reportError(error, ModelLoadError::ParseFailed);
        return false;
    }
    const bool hasColors = token == "COFF" && options.importVertexColors;

    std::uint32_t vertexCount = 0;
    std::uint32_t faceCount   = 0;
    std::uint32_t edgeCount   = 0;
    if (!nextToken(input, token) || !parseUnsigned(token, vertexCount) || !nextToken(input, token)
        || !parseUnsigned(token, faceCount) || !nextToken(input, token) || !parseUnsigned(token, edgeCount)
        || vertexCount == 0 || faceCount == 0)
    {
        reportError(error, ModelLoadError::ParseFailed);
        return false;
    }
    // Header edge count is parsed but unused.
    (void)edgeCount;

    if (vertexCount > std::vector<std::array<float, 11>>().max_size()
        || faceCount > std::vector<std::uint32_t>().max_size() / 3u)
    {
        reportError(error, ModelLoadError::FileTooLarge);
        return false;
    }

    std::vector<std::array<float, 11>> vertices(vertexCount);
    for (auto &vertex : vertices)
    {
        vertex = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f};
    }
    for (std::uint32_t index = 0; index < vertexCount; ++index)
    {
        std::array<float, 3> position{};
        for (float &component : position)
        {
            if (!nextToken(input, token) || !parseFiniteFloat(token, component))
            {
                reportError(error, ModelLoadError::ParseFailed);
                return false;
            }
        }
        transformPosition(transform, position.data(), vertices[index].data());
        if (!std::isfinite(vertices[index][0]) || !std::isfinite(vertices[index][1])
            || !std::isfinite(vertices[index][2]))
        {
            reportError(error, ModelLoadError::ParseFailed);
            return false;
        }
        if (hasColors)
        {
            float color[4] = {1.0f, 1.0f, 1.0f, 1.0f};
            for (std::size_t channel = 0; channel < 3; ++channel)
            {
                if (!nextToken(input, token) || !parseFiniteFloat(token, color[channel]) || color[channel] < 0.0f)
                {
                    reportError(error, ModelLoadError::ParseFailed);
                    return false;
                }
            }

            // COFF colors may be 0..1 floats or 0..255 bytes; scale down when any
            // channel exceeds 1. Anything after the color on the line is ignored.
            const float scale = std::max({color[0], color[1], color[2]}) > 1.0f ? (1.0f / 255.0f) : 1.0f;
            for (std::size_t channel = 0; channel < 3; ++channel)
            {
                vertices[index][6 + channel] = std::clamp(color[channel] * scale, 0.0f, 1.0f);
            }

            input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }

    std::vector<std::uint32_t> indices;
    indices.reserve(static_cast<std::size_t>(faceCount) * 6u);
    // Fan-triangulate each polygon around its first vertex.
    for (std::uint32_t face = 0; face < faceCount; ++face)
    {
        std::uint32_t cornerCount = 0;
        if (!nextToken(input, token) || !parseUnsigned(token, cornerCount) || cornerCount < 3)
        {
            reportError(error, ModelLoadError::ParseFailed);
            return false;
        }
        std::uint32_t first    = 0;
        std::uint32_t previous = 0;
        for (std::uint32_t corner = 0; corner < cornerCount; ++corner)
        {
            std::uint32_t vertex = 0;
            if (!nextToken(input, token) || !parseUnsigned(token, vertex) || vertex >= vertexCount)
            {
                reportError(error, ModelLoadError::ParseFailed);
                return false;
            }
            if (corner == 0)
            {
                first = vertex;
            }
            else if (corner == 1)
            {
                previous = vertex;
            }
            else
            {
                indices.push_back(first);
                indices.push_back(previous);
                indices.push_back(vertex);
                previous = vertex;
            }
        }
    }

    if (indices.empty())
    {
        reportError(error, ModelLoadError::NoGeometry);
        return false;
    }
    if (transform.reverseWinding)
    {
        reverseTriangleWinding(indices);
    }

    std::vector<float> interleaved;
    interleaved.resize(static_cast<std::size_t>(vertexCount) * render::meshVertexFloatCount);
    for (std::size_t index = 0; index < vertexCount; ++index)
    {
        std::copy(
            vertices[index].begin(), vertices[index].end(), interleaved.begin() + index * render::meshVertexFloatCount);
    }
    // OFF has no normals: generate flat (unshared vertices) or smooth per
    // normal_mode; PreserveOnly keeps the +Y placeholder from the vertex defaults.
    if (options.normalMode != ModelNormalMode::PreserveOnly)
    {
        std::vector<float> normals;
        if (options.normalMode == ModelNormalMode::GenerateFlat)
        {
            std::vector<float> flatVertices;
            flatVertices.reserve(indices.size() * render::meshVertexFloatCount);
            for (std::size_t corner = 0; corner + 2 < indices.size(); corner += 3)
            {
                const std::uint32_t ia = indices[corner + 0];
                const std::uint32_t ib = indices[corner + 1];
                const std::uint32_t ic = indices[corner + 2];
                const float *a     = interleaved.data() + static_cast<std::size_t>(ia) * render::meshVertexFloatCount;
                const float *b     = interleaved.data() + static_cast<std::size_t>(ib) * render::meshVertexFloatCount;
                const float *c     = interleaved.data() + static_cast<std::size_t>(ic) * render::meshVertexFloatCount;
                const float  ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
                const float  ac[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
                float        normal[3]
                    = {ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0]};
                normalizeVector(normal);
                for (std::size_t triangleCorner = 0; triangleCorner < 3; ++triangleCorner)
                {
                    const std::uint32_t sourceIndex = indices[corner + triangleCorner];
                    const float        *source
                        = interleaved.data() + static_cast<std::size_t>(sourceIndex) * render::meshVertexFloatCount;
                    const std::size_t base = flatVertices.size();
                    flatVertices.insert(flatVertices.end(), source, source + render::meshVertexFloatCount);
                    std::copy(normal, normal + 3, flatVertices.begin() + base + 3);
                }
            }
            // Expanded vertices are addressed by sequential identity indices.
            interleaved = std::move(flatVertices);
            indices.resize(interleaved.size() / render::meshVertexFloatCount);
            for (std::size_t index = 0; index < indices.size(); ++index)
            {
                indices[index] = static_cast<std::uint32_t>(index);
            }
        }
        else
        {
            accumulateSmoothNormalsInterleaved(interleaved, indices, normals);
            for (std::size_t vertex = 0; vertex < normals.size() / 3; ++vertex)
            {
                std::copy_n(
                    normals.data() + vertex * 3, 3, interleaved.data() + vertex * render::meshVertexFloatCount + 3);
            }
        }
    }

    ModelPart part;
    part.vertices = std::move(interleaved);
    part.indices  = std::move(indices);
    part.name     = path.stem().string();
    for (std::size_t offset = 0; offset < part.vertices.size(); offset += render::meshVertexFloatCount)
    {
        const float *position = part.vertices.data() + offset;
        if (offset == 0)
        {
            std::copy_n(position, 3, record.minimum);
            std::copy_n(position, 3, record.maximum);
        }
        else
        {
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                record.minimum[axis] = std::min(record.minimum[axis], position[axis]);
                record.maximum[axis] = std::max(record.maximum[axis], position[axis]);
            }
        }
    }
    record.parts.push_back(std::move(part));
    reportError(error, ModelLoadError::None);
    return true;
}

} // namespace engine::models::detail
