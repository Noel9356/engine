// PLY importer for ascii, binary little-endian, and binary big-endian encodings,
// with a hand-rolled header/element parser.
//
// Reads the "vertex" and "face" elements (positions required; normals, colors,
// and uvs optional) and emits a single part named after the file stem.

#include "model_import_internal.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <istream>
#include <limits>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace engine::models::detail
{

bool importPlyFile(const std::filesystem::path &path, ModelRecord &record, const ModelLoadOptions &options,
    const ModelTransform &transform, ModelLoadError *error)
{
    std::error_code     fileError;
    const std::uint64_t fileSize = std::filesystem::file_size(path, fileError);
    if (fileError)
    {
        reportError(error, ModelLoadError::ParseFailed);
        return false;
    }
    // One header property: PLY value type plus, for lists, the count type.
    struct PlyProperty
    {
        std::string name;
        std::string valueType;
        std::string countType;
        bool        list = false;
    };
    // One header element ("vertex", "face", ...): count and property list.
    struct PlyElement
    {
        std::string              name;
        std::uint64_t            count = 0;
        std::vector<PlyProperty> properties;
    };
    // Selected by the "format" line; binary scalars are assembled byte-wise.
    enum class PlyEncoding
    {
        Ascii,
        LittleEndian,
        BigEndian
    };

    std::ifstream input(path, std::ios::binary);
    if (!input)
    {
        reportError(error, ModelLoadError::ParseFailed);
        return false;
    }
    std::string line;
    if (!std::getline(input, line) || line != "ply")
    {
        reportError(error, ModelLoadError::ParseFailed);
        return false;
    }
    PlyEncoding             encoding    = PlyEncoding::Ascii;
    bool                    haveFormat  = false;
    bool                    endedHeader = false;
    std::vector<PlyElement> elements;
    PlyElement             *currentElement = nullptr;
    auto                    supportedType  = [](const std::string &type)
    {
        return type == "char" || type == "int8" || type == "uchar" || type == "uint8" || type == "short"
            || type == "int16" || type == "ushort" || type == "uint16" || type == "int" || type == "int32"
            || type == "uint" || type == "uint32" || type == "float" || type == "float32" || type == "double"
            || type == "float64";
    };
    // Header walk: format line, then element/property declarations; comment and
    // obj_info lines are skipped and the walk stops at end_header.
    while (std::getline(input, line))
    {
        std::istringstream fields(line);
        std::string        token;
        if (!(fields >> token) || token == "comment" || token == "obj_info")
        {
            continue;
        }
        if (token == "format")
        {
            std::string format;
            std::string version;
            if (!(fields >> format >> version) || version != "1.0")
            {
                reportError(error, ModelLoadError::ParseFailed);
                return false;
            }
            if (format == "ascii")
            {
                encoding = PlyEncoding::Ascii;
            }
            else if (format == "binary_little_endian")
            {
                encoding = PlyEncoding::LittleEndian;
            }
            else if (format == "binary_big_endian")
            {
                encoding = PlyEncoding::BigEndian;
            }
            else
            {
                reportError(error, ModelLoadError::ParseFailed);
                return false;
            }
            haveFormat = true;
        }
        else if (token == "element")
        {
            PlyElement element;
            if (!(fields >> element.name >> element.count) || element.count > std::numeric_limits<std::uint32_t>::max())
            {
                reportError(error, ModelLoadError::FileTooLarge);
                return false;
            }
            elements.push_back(std::move(element));
            currentElement = &elements.back();
        }
        else if (token == "property")
        {
            if (currentElement == nullptr)
            {
                reportError(error, ModelLoadError::ParseFailed);
                return false;
            }
            PlyProperty property;
            if (!(fields >> property.valueType))
            {
                reportError(error, ModelLoadError::ParseFailed);
                return false;
            }
            if (property.valueType == "list")
            {
                property.list = true;
                if (!(fields >> property.countType >> property.valueType >> property.name))
                {
                    reportError(error, ModelLoadError::ParseFailed);
                    return false;
                }
                if (!supportedType(property.countType))
                {
                    reportError(error, ModelLoadError::ParseFailed);
                    return false;
                }
            }
            else if (!(fields >> property.name))
            {
                reportError(error, ModelLoadError::ParseFailed);
                return false;
            }
            if (!supportedType(property.valueType))
            {
                reportError(error, ModelLoadError::ParseFailed);
                return false;
            }
            currentElement->properties.push_back(std::move(property));
        }
        else if (token == "end_header")
        {
            endedHeader = true;
            break;
        }
    }
    if (!haveFormat || !endedHeader)
    {
        reportError(error, ModelLoadError::ParseFailed);
        return false;
    }

    // Reads one scalar of the given PLY type in the file's encoding into a double.
    // Binary values are assembled little-endian (big-endian bytes reversed) and
    // every accepted value must be finite.
    auto readScalar = [&](const std::string &type, double &value) -> bool
    {
        if (encoding == PlyEncoding::Ascii)
        {
            return static_cast<bool>(input >> value) && std::isfinite(value);
        }
        auto readBytes = [&](unsigned char *destination, std::size_t count)
        {
            input.read(reinterpret_cast<char *>(destination), static_cast<std::streamsize>(count));
            if (!input)
            {
                return false;
            }
            if (encoding == PlyEncoding::BigEndian)
            {
                std::reverse(destination, destination + count);
            }
            return true;
        };
        if (type == "char" || type == "int8")
        {
            std::int8_t raw{};
            input.read(reinterpret_cast<char *>(&raw), 1);
            value = raw;
        }
        else if (type == "uchar" || type == "uint8")
        {
            std::uint8_t raw{};
            input.read(reinterpret_cast<char *>(&raw), 1);
            value = raw;
        }
        else if (type == "short" || type == "int16")
        {
            unsigned char bytes[2]{};
            if (!readBytes(bytes, sizeof(bytes)))
                return false;
            const std::uint16_t bits = static_cast<std::uint16_t>(bytes[0]) | static_cast<std::uint16_t>(bytes[1] << 8);
            value                    = std::bit_cast<std::int16_t>(bits);
        }
        else if (type == "ushort" || type == "uint16")
        {
            unsigned char bytes[2]{};
            if (!readBytes(bytes, sizeof(bytes)))
                return false;
            value = static_cast<std::uint16_t>(bytes[0]) | static_cast<std::uint16_t>(bytes[1] << 8);
        }
        else if (type == "int" || type == "int32")
        {
            unsigned char bytes[4]{};
            if (!readBytes(bytes, sizeof(bytes)))
                return false;
            const std::uint32_t bits
                = static_cast<std::uint32_t>(bytes[0]) | (static_cast<std::uint32_t>(bytes[1]) << 8)
                | (static_cast<std::uint32_t>(bytes[2]) << 16) | (static_cast<std::uint32_t>(bytes[3]) << 24);
            value = std::bit_cast<std::int32_t>(bits);
        }
        else if (type == "uint" || type == "uint32")
        {
            unsigned char bytes[4]{};
            if (!readBytes(bytes, sizeof(bytes)))
                return false;
            value = static_cast<std::uint32_t>(bytes[0]) | (static_cast<std::uint32_t>(bytes[1]) << 8)
                  | (static_cast<std::uint32_t>(bytes[2]) << 16) | (static_cast<std::uint32_t>(bytes[3]) << 24);
        }
        else if (type == "float" || type == "float32")
        {
            unsigned char bytes[4]{};
            if (!readBytes(bytes, sizeof(bytes)))
                return false;
            const std::uint32_t bits
                = static_cast<std::uint32_t>(bytes[0]) | (static_cast<std::uint32_t>(bytes[1]) << 8)
                | (static_cast<std::uint32_t>(bytes[2]) << 16) | (static_cast<std::uint32_t>(bytes[3]) << 24);
            value = std::bit_cast<float>(bits);
        }
        else if (type == "double" || type == "float64")
        {
            unsigned char bytes[8]{};
            if (!readBytes(bytes, sizeof(bytes)))
                return false;
            std::uint64_t bits = 0;
            for (std::size_t index = 0; index < 8; ++index)
            {
                bits |= static_cast<std::uint64_t>(bytes[index]) << (index * 8);
            }
            value = std::bit_cast<double>(bits);
        }
        else
        {
            return false;
        }
        return static_cast<bool>(input) && std::isfinite(value);
    };
    // List counts must be non-negative whole numbers within uint32 range.
    auto readCount = [&](const std::string &type, std::uint64_t &count)
    {
        double value = 0.0;
        if (!readScalar(type, value) || value < 0.0 || value > std::numeric_limits<std::uint32_t>::max()
            || std::floor(value) != value)
        {
            return false;
        }
        count = static_cast<std::uint64_t>(value);
        return true;
    };
    auto propertyIndex = [](const PlyElement &element, const char *name)
    {
        for (std::size_t index = 0; index < element.properties.size(); ++index)
        {
            if (element.properties[index].name == name)
                return index;
        }
        return element.properties.size();
    };

    // Only "vertex" and "face" are interpreted; other elements are consumed
    // field-by-field to keep the stream aligned and then discarded.
    const PlyElement *vertexElement = nullptr;
    const PlyElement *faceElement   = nullptr;
    std::uint64_t     vertexCount   = 0;
    std::uint64_t     faceCount     = 0;
    for (const PlyElement &element : elements)
    {
        if (element.name == "vertex")
        {
            vertexElement = &element;
            vertexCount   = element.count;
        }
        else if (element.name == "face")
        {
            faceElement = &element;
            faceCount   = element.count;
        }
    }
    if (vertexElement == nullptr || faceElement == nullptr || vertexCount == 0 || faceCount == 0)
    {
        reportError(error, ModelLoadError::NoGeometry);
        return false;
    }
    if (vertexCount > std::vector<std::array<float, 11>>().max_size())
    {
        reportError(error, ModelLoadError::FileTooLarge);
        return false;
    }
    const std::size_t xIndex = propertyIndex(*vertexElement, "x");
    const std::size_t yIndex = propertyIndex(*vertexElement, "y");
    const std::size_t zIndex = propertyIndex(*vertexElement, "z");
    if (xIndex == vertexElement->properties.size() || yIndex == vertexElement->properties.size()
        || zIndex == vertexElement->properties.size())
    {
        reportError(error, ModelLoadError::ParseFailed);
        return false;
    }
    // First matching name wins; covers the aliased uv and face-index properties.
    auto findAnyProperty = [&](const PlyElement &element, std::initializer_list<const char *> names)
    {
        for (const char *name : names)
        {
            const std::size_t index = propertyIndex(element, name);
            if (index < element.properties.size())
                return index;
        }
        return element.properties.size();
    };
    const std::size_t nxIndex          = propertyIndex(*vertexElement, "nx");
    const std::size_t nyIndex          = propertyIndex(*vertexElement, "ny");
    const std::size_t nzIndex          = propertyIndex(*vertexElement, "nz");
    const bool        hasSourceNormals = nxIndex < vertexElement->properties.size()
                                      && nyIndex < vertexElement->properties.size()
                                      && nzIndex < vertexElement->properties.size();
    const std::size_t uIndex           = findAnyProperty(*vertexElement, {"u", "s", "texture_u"});
    const std::size_t vIndex           = findAnyProperty(*vertexElement, {"v", "t", "texture_v"});
    const bool        hasUvs   = uIndex < vertexElement->properties.size() && vIndex < vertexElement->properties.size();
    const std::size_t redIndex = propertyIndex(*vertexElement, "red");
    const std::size_t greenIndex        = propertyIndex(*vertexElement, "green");
    const std::size_t blueIndex         = propertyIndex(*vertexElement, "blue");
    const bool        hasColors         = options.importVertexColors && redIndex < vertexElement->properties.size()
                                       && greenIndex < vertexElement->properties.size()
                                       && blueIndex < vertexElement->properties.size();
    const std::size_t faceIndexProperty = findAnyProperty(*faceElement, {"vertex_indices", "vertex_index"});

    // Per-source-vertex staging: x y z nx ny nz r g b u v (defaults: +Y normal,
    // white color, zero uv).
    std::vector<std::array<float, 11>> sourceVertices(static_cast<std::size_t>(vertexCount));
    for (auto &vertex : sourceVertices)
    {
        vertex = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f};
    }
    std::vector<std::uint32_t> sourceIndices;
    if (faceCount <= std::numeric_limits<std::size_t>::max() / 3)
    {
        sourceIndices.reserve(static_cast<std::size_t>(faceCount) * 3);
    }
    // Integer color types are normalized by their full-scale value.
    auto colorScale = [](const std::string &type)
    {
        if (type == "uchar" || type == "uint8")
            return 1.0f / 255.0f;
        if (type == "ushort" || type == "uint16")
            return 1.0f / 65535.0f;
        if (type == "uint" || type == "uint32")
            return 1.0f / 4294967295.0f;
        return 1.0f;
    };
    const float redScale   = hasColors ? colorScale(vertexElement->properties[redIndex].valueType) : 1.0f;
    const float greenScale = hasColors ? colorScale(vertexElement->properties[greenIndex].valueType) : 1.0f;
    const float blueScale  = hasColors ? colorScale(vertexElement->properties[blueIndex].valueType) : 1.0f;
    for (const PlyElement &element : elements)
    {
        for (std::uint64_t row = 0; row < element.count; ++row)
        {
            if (element.name == "vertex")
            {
                auto &vertex = sourceVertices[static_cast<std::size_t>(row)];
                for (std::size_t property = 0; property < element.properties.size(); ++property)
                {
                    const PlyProperty &descriptor = element.properties[property];
                    if (descriptor.list)
                    {
                        std::uint64_t count = 0;
                        if (!readCount(descriptor.countType, count))
                        {
                            reportError(error, ModelLoadError::ParseFailed);
                            return false;
                        }
                        for (std::uint64_t index = 0; index < count; ++index)
                        {
                            double ignored = 0.0;
                            if (!readScalar(descriptor.valueType, ignored))
                            {
                                reportError(error, ModelLoadError::ParseFailed);
                                return false;
                            }
                        }
                        continue;
                    }
                    double value = 0.0;
                    if (!readScalar(descriptor.valueType, value))
                    {
                        reportError(error, ModelLoadError::ParseFailed);
                        return false;
                    }
                    const float converted = static_cast<float>(value);
                    if (property == xIndex)
                        vertex[0] = converted;
                    else if (property == yIndex)
                        vertex[1] = converted;
                    else if (property == zIndex)
                        vertex[2] = converted;
                    else if (hasSourceNormals && property == nxIndex)
                        vertex[3] = converted;
                    else if (hasSourceNormals && property == nyIndex)
                        vertex[4] = converted;
                    else if (hasSourceNormals && property == nzIndex)
                        vertex[5] = converted;
                    else if (hasColors && property == redIndex)
                        vertex[6] = converted * redScale;
                    else if (hasColors && property == greenIndex)
                        vertex[7] = converted * greenScale;
                    else if (hasColors && property == blueIndex)
                        vertex[8] = converted * blueScale;
                    else if (hasUvs && property == uIndex)
                        vertex[9] = converted;
                    else if (hasUvs && property == vIndex)
                        vertex[10] = converted;
                }
            }
            else if (element.name == "face")
            {
                // Faces are polygon lists; fan-triangulate around the first vertex.
                std::vector<std::uint32_t> faceIndices;
                for (std::size_t property = 0; property < element.properties.size(); ++property)
                {
                    const PlyProperty &descriptor = element.properties[property];
                    if (descriptor.list)
                    {
                        std::uint64_t count = 0;
                        if (!readCount(descriptor.countType, count) || count > vertexCount + 1)
                        {
                            reportError(error, ModelLoadError::ParseFailed);
                            return false;
                        }
                        if (property == faceIndexProperty)
                        {
                            faceIndices.reserve(static_cast<std::size_t>(count));
                        }
                        for (std::uint64_t index = 0; index < count; ++index)
                        {
                            double value = 0.0;
                            if (!readScalar(descriptor.valueType, value) || value < 0.0
                                || value >= static_cast<double>(vertexCount) || std::floor(value) != value)
                            {
                                reportError(error, ModelLoadError::ParseFailed);
                                return false;
                            }
                            if (property == faceIndexProperty)
                            {
                                faceIndices.push_back(static_cast<std::uint32_t>(value));
                            }
                        }
                    }
                    else
                    {
                        double ignored = 0.0;
                        if (!readScalar(descriptor.valueType, ignored))
                        {
                            reportError(error, ModelLoadError::ParseFailed);
                            return false;
                        }
                    }
                }
                if (faceIndexProperty == element.properties.size() || faceIndices.size() < 3)
                {
                    reportError(error, ModelLoadError::ParseFailed);
                    return false;
                }
                const std::size_t additionalIndices = (faceIndices.size() - 2) * 3;
                if (additionalIndices > sourceIndices.max_size() - sourceIndices.size())
                {
                    reportError(error, ModelLoadError::FileTooLarge);
                    return false;
                }
                for (std::size_t corner = 1; corner + 1 < faceIndices.size(); ++corner)
                {
                    sourceIndices.push_back(faceIndices[0]);
                    sourceIndices.push_back(faceIndices[corner]);
                    sourceIndices.push_back(faceIndices[corner + 1]);
                }
            }
            else
            {
                // Unknown element: read (and discard) each field to stay aligned.
                for (const PlyProperty &descriptor : element.properties)
                {
                    if (descriptor.list)
                    {
                        std::uint64_t count = 0;
                        if (!readCount(descriptor.countType, count) || count > fileSize)
                        {
                            reportError(error, ModelLoadError::ParseFailed);
                            return false;
                        }
                        for (std::uint64_t index = 0; index < count; ++index)
                        {
                            double ignored = 0.0;
                            if (!readScalar(descriptor.valueType, ignored))
                            {
                                reportError(error, ModelLoadError::ParseFailed);
                                return false;
                            }
                        }
                    }
                    else
                    {
                        double ignored = 0.0;
                        if (!readScalar(descriptor.valueType, ignored))
                        {
                            reportError(error, ModelLoadError::ParseFailed);
                            return false;
                        }
                    }
                }
            }
        }
    }
    if (sourceIndices.empty() || sourceIndices.size() % 3 != 0
        || sourceIndices.size() > std::numeric_limits<std::uint32_t>::max())
    {
        reportError(error, ModelLoadError::NoGeometry);
        return false;
    }

    // Normal handling: PLY normals are per-vertex. Flat mode expands to unshared
    // vertices, smooth mode generates, preserve mode transforms the file normals.
    const bool generateSmooth  = options.normalMode == ModelNormalMode::GenerateSmooth
                              || (options.normalMode == ModelNormalMode::PreserveOrGenerateSmooth && !hasSourceNormals);
    const bool preserveNormals = hasSourceNormals
                              && (options.normalMode == ModelNormalMode::PreserveOnly
                                  || options.normalMode == ModelNormalMode::PreserveOrGenerateSmooth);
    const bool flat            = options.normalMode == ModelNormalMode::GenerateFlat;
    std::vector<float> positions(static_cast<std::size_t>(vertexCount) * 3);
    for (std::size_t index = 0; index < sourceVertices.size(); ++index)
    {
        const auto &source      = sourceVertices[index];
        float      *destination = positions.data() + index * 3;
        transformPosition(transform, source.data(), destination);
        if (!std::isfinite(destination[0]) || !std::isfinite(destination[1]) || !std::isfinite(destination[2]))
        {
            reportError(error, ModelLoadError::ParseFailed);
            return false;
        }
    }

    ModelPart part;
    part.name = path.stem().string();
    // Flat mode: one expanded vertex per corner with sequential (identity) indices.
    if (flat)
    {
        const std::size_t vertexCountOut = sourceIndices.size();
        if (vertexCountOut > std::vector<float>().max_size() / render::meshVertexFloatCount)
        {
            reportError(error, ModelLoadError::FileTooLarge);
            return false;
        }
        part.vertices.reserve(vertexCountOut * render::meshVertexFloatCount);
        part.indices.reserve(vertexCountOut);
        for (std::size_t triangle = 0; triangle < sourceIndices.size(); triangle += 3)
        {
            const std::uint32_t ia    = sourceIndices[triangle];
            const std::uint32_t ib    = sourceIndices[triangle + 1];
            const std::uint32_t ic    = sourceIndices[triangle + 2];
            const float        *a     = positions.data() + static_cast<std::size_t>(ia) * 3;
            const float        *b     = positions.data() + static_cast<std::size_t>(ib) * 3;
            const float        *c     = positions.data() + static_cast<std::size_t>(ic) * 3;
            const float         ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
            const float         ac[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
            float               normal[3]
                = {ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0]};
            normalizeVector(normal);
            if (transform.reverseWinding)
            {
                normal[0] = -normal[0];
                normal[1] = -normal[1];
                normal[2] = -normal[2];
            }
            for (std::size_t corner = 0; corner < 3; ++corner)
            {
                const std::uint32_t sourceIndex = sourceIndices[triangle + corner];
                const auto         &source      = sourceVertices[sourceIndex];
                const float        *position    = positions.data() + static_cast<std::size_t>(sourceIndex) * 3;
                const std::size_t   offset      = part.vertices.size();
                part.vertices.resize(offset + render::meshVertexFloatCount);
                std::copy_n(position, 3, part.vertices.data() + offset);
                std::copy_n(normal, 3, part.vertices.data() + offset + 3);
                std::copy_n(source.data() + 6, 3, part.vertices.data() + offset + 6);
                const float v              = options.flipTextureV ? 1.0f - source[10] : source[10];
                part.vertices[offset + 9]  = source[9] * options.uvScale[0] + options.uvOffset[0];
                part.vertices[offset + 10] = v * options.uvScale[1] + options.uvOffset[1];
                part.indices.push_back(static_cast<std::uint32_t>(part.indices.size()));
            }
        }
    }
    else
    {
        // Area-weighted smooth normals from the transformed positions.
        std::vector<float> smoothNormals;
        if (generateSmooth)
        {
            smoothNormals.assign(static_cast<std::size_t>(vertexCount) * 3, 0.0f);
            for (std::size_t triangle = 0; triangle < sourceIndices.size(); triangle += 3)
            {
                const std::uint32_t ia    = sourceIndices[triangle];
                const std::uint32_t ib    = sourceIndices[triangle + 1];
                const std::uint32_t ic    = sourceIndices[triangle + 2];
                const float        *a     = positions.data() + static_cast<std::size_t>(ia) * 3;
                const float        *b     = positions.data() + static_cast<std::size_t>(ib) * 3;
                const float        *c     = positions.data() + static_cast<std::size_t>(ic) * 3;
                const float         ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
                const float         ac[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
                const float         normal[3]
                    = {ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0]};
                for (const std::uint32_t index : {ia, ib, ic})
                {
                    float *sum = smoothNormals.data() + static_cast<std::size_t>(index) * 3;
                    sum[0] += normal[0];
                    sum[1] += normal[1];
                    sum[2] += normal[2];
                }
            }
            for (std::size_t index = 0; index < vertexCount; ++index)
            {
                normalizeVector(smoothNormals.data() + index * 3);
            }
            // Mirrored import flips handedness, so generated normals flip with it.
            if (transform.reverseWinding)
            {
                for (float &component : smoothNormals)
                {
                    component = -component;
                }
            }
        }
        const std::size_t outputCount = static_cast<std::size_t>(vertexCount);
        if (outputCount > std::vector<float>().max_size() / render::meshVertexFloatCount)
        {
            reportError(error, ModelLoadError::FileTooLarge);
            return false;
        }
        part.vertices.resize(outputCount * render::meshVertexFloatCount);
        for (std::size_t index = 0; index < outputCount; ++index)
        {
            const auto &source      = sourceVertices[index];
            float      *destination = part.vertices.data() + index * render::meshVertexFloatCount;
            std::copy_n(positions.data() + index * 3, 3, destination);
            if (generateSmooth)
            {
                std::copy_n(smoothNormals.data() + index * 3, 3, destination + 3);
            }
            else if (preserveNormals)
            {
                transformNormal(transform, source.data() + 3, destination + 3);
            }
            else
            {
                destination[3] = 0.0f;
                destination[4] = 1.0f;
                destination[5] = 0.0f;
            }
            std::copy_n(source.data() + 6, 3, destination + 6);
            const float v   = options.flipTextureV ? 1.0f - source[10] : source[10];
            destination[9]  = source[9] * options.uvScale[0] + options.uvOffset[0];
            destination[10] = v * options.uvScale[1] + options.uvOffset[1];
        }
        part.indices = std::move(sourceIndices);
        if (transform.reverseWinding)
        {
            reverseTriangleWinding(part.indices);
        }
    }
    // PLY has no materials; every triangle records invalidMaterial when enabled.
    if (options.importMaterials)
    {
        part.triangleMaterials.assign(part.indices.size() / 3, invalidMaterial);
    }
    bool haveBounds = false;
    for (std::size_t offset = 0; offset + 2 < part.vertices.size(); offset += render::meshVertexFloatCount)
    {
        const float x = part.vertices[offset];
        const float y = part.vertices[offset + 1];
        const float z = part.vertices[offset + 2];
        if (!haveBounds)
        {
            record.minimum[0] = record.maximum[0] = x;
            record.minimum[1] = record.maximum[1] = y;
            record.minimum[2] = record.maximum[2] = z;
            haveBounds                            = true;
        }
        else
        {
            record.minimum[0] = std::min(record.minimum[0], x);
            record.maximum[0] = std::max(record.maximum[0], x);
            record.minimum[1] = std::min(record.minimum[1], y);
            record.maximum[1] = std::max(record.maximum[1], y);
            record.minimum[2] = std::min(record.minimum[2], z);
            record.maximum[2] = std::max(record.maximum[2], z);
        }
    }
    record.parts.push_back(std::move(part));
    reportError(error, ModelLoadError::None);
    return true;
}

} // namespace engine::models::detail
