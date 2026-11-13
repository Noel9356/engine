// Wavefront OBJ/MTL importer built on tinyobjloader (the library implementation
// is instantiated in this translation unit).
//
// Converts OBJ shapes into ModelParts and MTL materials into module material
// records: welding by position/normal/uv keys, smooth/flat normal generation,
// option transforms, and per-triangle material indices.

#include "model_import_internal.hpp"

// Instantiates the tinyobjloader implementation for this module.
#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine::models::detail
{
namespace
{
// Weld key for one OBJ corner: the source position/normal/uv indices. flatCorner
// (1-based corner number) keeps vertices unshared when flat normals are wanted.
struct ObjVertexKey
{
    int           position   = -1;
    int           normal     = -1;
    int           uv         = -1;
    std::uint32_t flatCorner = 0;

    bool operator==(const ObjVertexKey &) const = default;
};

// 64-bit mix of the key fields (boost-style combine plus a splitmix64 finalizer).
struct ObjVertexKeyHash
{
    std::size_t operator()(const ObjVertexKey &key) const noexcept
    {
        std::uint64_t value = static_cast<std::uint32_t>(key.position) + 0x9e3779b97f4a7c15ull;
        value ^= static_cast<std::uint32_t>(key.normal) + 0x9e3779b97f4a7c15ull + (value << 6) + (value >> 2);
        value ^= static_cast<std::uint32_t>(key.uv) + 0x9e3779b97f4a7c15ull + (value << 6) + (value >> 2);
        value ^= static_cast<std::uint64_t>(key.flatCorner) + 0x9e3779b97f4a7c15ull + (value << 6) + (value >> 2);
        value ^= value >> 30;
        value *= 0xbf58476d1ce4e5b9ull;
        value ^= value >> 27;
        value *= 0x94d049bb133111ebull;
        value ^= value >> 31;
        return static_cast<std::size_t>(value);
    }
};

// Area-weighted smooth normals for one shape, accumulated over its corners by
// position index. touchedFlags/touchedIndices record which accumulators were
// cleared so the scratch buffers can be reused across shapes without resizing.
void accumulateSmoothNormals(const std::vector<tinyobj::real_t> &positions,
    const std::vector<tinyobj::index_t> &corners, std::vector<float> &outNormals,
    std::vector<std::uint8_t> &touchedFlags, std::vector<std::uint32_t> &touchedIndices)
{
    if (outNormals.size() != positions.size())
    {
        outNormals.resize(positions.size(), 0.0f);
    }
    if (touchedFlags.size() != positions.size() / 3)
    {
        touchedFlags.resize(positions.size() / 3, 0);
    }
    touchedIndices.clear();
    for (std::size_t corner = 0; corner + 2 < corners.size(); corner += 3)
    {
        const std::uint32_t indexA = static_cast<std::uint32_t>(corners[corner + 0].vertex_index);
        const std::uint32_t indexB = static_cast<std::uint32_t>(corners[corner + 1].vertex_index);
        const std::uint32_t indexC = static_cast<std::uint32_t>(corners[corner + 2].vertex_index);

        const tinyobj::real_t *a = positions.data() + static_cast<std::size_t>(indexA) * 3;
        const tinyobj::real_t *b = positions.data() + static_cast<std::size_t>(indexB) * 3;
        const tinyobj::real_t *c = positions.data() + static_cast<std::size_t>(indexC) * 3;
        float normal[3]{};
        crossProduct3(a, b, c, normal);
        for (const std::uint32_t index : {indexA, indexB, indexC})
        {
            if (touchedFlags[index] == 0)
            {
                touchedFlags[index]      = 1;
                const std::size_t offset = static_cast<std::size_t>(index) * 3;
                outNormals[offset + 0]   = 0.0f;
                outNormals[offset + 1]   = 0.0f;
                outNormals[offset + 2]   = 0.0f;
                touchedIndices.push_back(index);
            }
            const std::size_t offset = static_cast<std::size_t>(index) * 3;
            outNormals[offset + 0] += normal[0];
            outNormals[offset + 1] += normal[1];
            outNormals[offset + 2] += normal[2];
        }
    }
    for (const std::uint32_t index : touchedIndices)
    {
        const std::size_t offset    = static_cast<std::size_t>(index) * 3;
        float             normal[3] = {outNormals[offset + 0], outNormals[offset + 1], outNormals[offset + 2]};
        normalizeVector(normal);
        std::copy(normal, normal + 3, outNormals.begin() + offset);
    }
}

} // namespace

// Imports one OBJ file: MTL materials first (when enabled), then each non-empty
// shape as a part. Reports ParseFailed for malformed data and NoGeometry when no
// shape produced triangles.
bool importObjFile(const std::filesystem::path &path, ModelRecord &record, const ModelLoadOptions &options,
    const ModelTransform &transform, const std::string &defaultMaterialSearchPath, ModelLoadError *error)
{
    tinyobj::ObjReaderConfig config;
    // Faces are requested pre-triangulated; vertex colors are optional.
    config.triangulate  = true;
    config.vertex_color = options.importVertexColors;
    // A per-load override wins over the module-configured MTL search path.
    const char *materialSearchPath = options.materialSearchPath != nullptr && options.materialSearchPath[0] != '\0'
                                       ? options.materialSearchPath
                                       : defaultMaterialSearchPath.c_str();
    if (materialSearchPath[0] != '\0')
    {
        config.mtl_search_path = materialSearchPath;
    }

    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(path.string(), config))
    {
        if (!reader.Error().empty())
        {
            std::fprintf(stderr, "models module: %s: %s\n", path.string().c_str(), reader.Error().c_str());
        }
        reportError(error, ModelLoadError::ParseFailed);
        return false;
    }
    if (!reader.Warning().empty())
    {
        std::fprintf(stderr, "models module: %s: %s\n", path.string().c_str(), reader.Warning().c_str());
    }

    const tinyobj::attrib_t                &attrib    = reader.GetAttrib();
    const std::vector<tinyobj::shape_t>    &shapes    = reader.GetShapes();
    const std::vector<tinyobj::material_t> &materials = reader.GetMaterials();
    if (options.importMaterials)
    {
        // Copies one MTL texture option block: finite floats with fallbacks and the
        // colorspace string mapped onto the shared enum.
        auto copyTextureOptions = [](const tinyobj::texture_option_t &source)
        {
            ModelMaterialTextureOptions result;
            auto                        toFiniteFloat = [](tinyobj::real_t value, float fallback)
            {
                const double converted = static_cast<double>(value);
                return std::isfinite(converted) ? static_cast<float>(std::clamp(converted,
                                                      -static_cast<double>(std::numeric_limits<float>::max()),
                                                      static_cast<double>(std::numeric_limits<float>::max())))
                                                : fallback;
            };
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                result.offset[axis]     = toFiniteFloat(source.origin_offset[axis], 0.0f);
                result.scale[axis]      = toFiniteFloat(source.scale[axis], 1.0f);
                result.turbulence[axis] = toFiniteFloat(source.turbulence[axis], 0.0f);
            }
            result.sharpness         = toFiniteFloat(source.sharpness, 1.0f);
            result.brightness        = toFiniteFloat(source.brightness, 0.0f);
            result.contrast          = toFiniteFloat(source.contrast, 1.0f);
            result.bumpMultiplier    = toFiniteFloat(source.bump_multiplier, 1.0f);
            result.textureResolution = source.texture_resolution;
            result.textureType       = static_cast<std::uint32_t>(source.type);
            result.imageChannel      = source.imfchan;
            result.clamp             = source.clamp;
            result.blendU            = source.blendu;
            result.blendV            = source.blendv;
            if (source.colorspace == "linear")
            {
                result.colorSpace = ModelTextureColorSpace::Linear;
            }
            else if (source.colorspace == "sRGB" || source.colorspace == "srgb" || source.colorspace == "SRGB")
            {
                result.colorSpace = ModelTextureColorSpace::Srgb;
            }
            return result;
        };
        record.materials.reserve(materials.size());
        for (const tinyobj::material_t &source : materials)
        {
            ModelMaterialRecord material;
            material.name  = source.name;
            auto copyColor = [](const auto *input, float *output)
            {
                for (std::size_t channel = 0; channel < 3; ++channel)
                {
                    const double value = static_cast<double>(input[channel]);
                    output[channel] = std::isfinite(value) ? static_cast<float>(std::clamp(value, 0.0, 1.0e20)) : 0.0f;
                }
            };
            copyColor(source.diffuse, material.factors.baseColor);
            copyColor(source.ambient, material.factors.ambient);
            copyColor(source.emission, material.factors.emissive);
            copyColor(source.specular, material.factors.specular);
            copyColor(source.transmittance, material.factors.transmittance);
            // OBJ has no PBR fields, so infer them: metallic defaults to 1 when a
            // metallic map exists; roughness comes from the file, or from shininess
            // via a GGX-style conversion, or stays at the default.
            const double metallic            = static_cast<double>(source.metallic);
            const double roughness           = static_cast<double>(source.roughness);
            const double shininess           = static_cast<double>(source.shininess);
            const bool   hasMetallicTexture  = !source.metallic_texname.empty();
            const bool   hasRoughnessTexture = !source.roughness_texname.empty();
            material.factors.metallic
                = std::isfinite(metallic)
                    ? static_cast<float>(std::clamp(metallic == 0.0 && hasMetallicTexture ? 1.0 : metallic, 0.0, 1.0))
                    : 0.0f;
            material.factors.hasPbrFactors = (std::isfinite(metallic) && metallic > 0.0)
                                          || (std::isfinite(roughness) && roughness > 0.0) || hasRoughnessTexture
                                          || hasMetallicTexture;
            if (hasRoughnessTexture && std::isfinite(roughness) && roughness == 0.0)
            {
                material.factors.roughness = 1.0f;
            }
            else if (material.factors.hasPbrFactors && std::isfinite(roughness))
            {
                material.factors.roughness = static_cast<float>(std::clamp(roughness, 0.0, 1.0));
            }
            else if (std::isfinite(shininess) && shininess > 0.0)
            {
                material.factors.roughness = static_cast<float>(std::sqrt(2.0 / (std::min(shininess, 1.0e12) + 2.0)));
            }
            if (std::isfinite(shininess))
            {
                material.factors.shininess = static_cast<float>(std::clamp(shininess, 0.0, 1.0e12));
            }
            material.factors.opacity
                = std::isfinite(static_cast<double>(source.dissolve))
                    ? static_cast<float>(std::clamp(static_cast<double>(source.dissolve), 0.0, 1.0))
                    : 1.0f;
            material.factors.indexOfRefraction
                = std::isfinite(static_cast<double>(source.ior)) && source.ior > 0.0
                    ? static_cast<float>(std::clamp(static_cast<double>(source.ior), 1.0e-4, 1.0e4))
                    : 1.5f;
            material.factors.illuminationModel = source.illum >= 0 ? static_cast<std::uint32_t>(source.illum) : 2u;
            auto copyFactor                    = [](tinyobj::real_t value, float fallback)
            {
                return std::isfinite(static_cast<double>(value))
                         ? static_cast<float>(std::clamp(static_cast<double>(value), 0.0, 1.0))
                         : fallback;
            };
            material.factors.sheen              = copyFactor(source.sheen, 0.0f);
            material.factors.clearcoat          = copyFactor(source.clearcoat_thickness, 0.0f);
            material.factors.clearcoatRoughness = copyFactor(source.clearcoat_roughness, 0.0f);
            material.factors.anisotropy         = copyFactor(source.anisotropy, 0.0f);
            material.factors.anisotropyRotation = std::isfinite(static_cast<double>(source.anisotropy_rotation))
                                                    ? static_cast<float>(source.anisotropy_rotation)
                                                    : 0.0f;
            material.texturePaths[static_cast<std::size_t>(ModelMaterialTextureSlot::Ambient)] = source.ambient_texname;
            material.texturePaths[static_cast<std::size_t>(ModelMaterialTextureSlot::BaseColor)]
                = source.diffuse_texname;
            material.texturePaths[static_cast<std::size_t>(ModelMaterialTextureSlot::Specular)]
                = source.specular_texname;
            material.texturePaths[static_cast<std::size_t>(ModelMaterialTextureSlot::SpecularHighlight)]
                = source.specular_highlight_texname;
            // A bump map stands in when no dedicated normal map is present.
            material.texturePaths[static_cast<std::size_t>(ModelMaterialTextureSlot::Normal)]
                = source.normal_texname.empty() ? source.bump_texname : source.normal_texname;
            material.texturePaths[static_cast<std::size_t>(ModelMaterialTextureSlot::Roughness)]
                = source.roughness_texname;
            material.texturePaths[static_cast<std::size_t>(ModelMaterialTextureSlot::Metallic)]
                = source.metallic_texname;
            material.texturePaths[static_cast<std::size_t>(ModelMaterialTextureSlot::Emissive)]
                = source.emissive_texname;
            material.texturePaths[static_cast<std::size_t>(ModelMaterialTextureSlot::Opacity)] = source.alpha_texname;
            material.texturePaths[static_cast<std::size_t>(ModelMaterialTextureSlot::Displacement)]
                = source.displacement_texname;
            material.texturePaths[static_cast<std::size_t>(ModelMaterialTextureSlot::Reflection)]
                = source.reflection_texname;
            material.textureOptions[static_cast<std::size_t>(ModelMaterialTextureSlot::Ambient)]
                = copyTextureOptions(source.ambient_texopt);
            material.textureOptions[static_cast<std::size_t>(ModelMaterialTextureSlot::BaseColor)]
                = copyTextureOptions(source.diffuse_texopt);
            material.textureOptions[static_cast<std::size_t>(ModelMaterialTextureSlot::Specular)]
                = copyTextureOptions(source.specular_texopt);
            material.textureOptions[static_cast<std::size_t>(ModelMaterialTextureSlot::SpecularHighlight)]
                = copyTextureOptions(source.specular_highlight_texopt);
            material.textureOptions[static_cast<std::size_t>(ModelMaterialTextureSlot::Roughness)]
                = copyTextureOptions(source.roughness_texopt);
            material.textureOptions[static_cast<std::size_t>(ModelMaterialTextureSlot::Metallic)]
                = copyTextureOptions(source.metallic_texopt);
            material.textureOptions[static_cast<std::size_t>(ModelMaterialTextureSlot::Emissive)]
                = copyTextureOptions(source.emissive_texopt);
            material.textureOptions[static_cast<std::size_t>(ModelMaterialTextureSlot::Opacity)]
                = copyTextureOptions(source.alpha_texopt);
            material.textureOptions[static_cast<std::size_t>(ModelMaterialTextureSlot::Displacement)]
                = copyTextureOptions(source.displacement_texopt);
            material.textureOptions[static_cast<std::size_t>(ModelMaterialTextureSlot::Reflection)]
                = copyTextureOptions(source.reflection_texopt);
            material.textureOptions[static_cast<std::size_t>(ModelMaterialTextureSlot::Sheen)]
                = copyTextureOptions(source.sheen_texopt);
            material.textureOptions[static_cast<std::size_t>(ModelMaterialTextureSlot::Normal)]
                = copyTextureOptions(source.normal_texname.empty() ? source.bump_texopt : source.normal_texopt);
            record.materials.push_back(std::move(material));
        }
    }

    std::vector<float>         smoothNormalScratch;
    std::vector<std::uint8_t>  smoothNormalTouchedFlags;
    std::vector<std::uint32_t> smoothNormalTouchedIndices;
    for (const tinyobj::shape_t &shape : shapes)
    {
        const std::size_t indexCount = shape.mesh.indices.size();
        if (indexCount == 0)
        {
            continue;
        }
        if (indexCount % 3 != 0 || indexCount > std::numeric_limits<std::uint32_t>::max())
        {
            reportError(error, ModelLoadError::ParseFailed);
            return false;
        }

        bool anyMissingNormals = false;
        bool allMissingNormals = true;
        bool anyUvs            = false;
        // Validate every corner up front: triangle-aligned index count, in-range
        // indices, and finite positions/normals/uvs/colors.
        for (const tinyobj::index_t &index : shape.mesh.indices)
        {
            if (index.vertex_index < 0 || static_cast<std::size_t>(index.vertex_index) >= attrib.vertices.size() / 3)
            {
                reportError(error, ModelLoadError::ParseFailed);
                return false;
            }
            const tinyobj::real_t *position = attrib.vertices.data() + static_cast<std::size_t>(index.vertex_index) * 3;
            if (!std::isfinite(position[0]) || !std::isfinite(position[1]) || !std::isfinite(position[2]))
            {
                reportError(error, ModelLoadError::ParseFailed);
                return false;
            }
            if (index.normal_index >= 0)
            {
                allMissingNormals = false;
                if (static_cast<std::size_t>(index.normal_index) >= attrib.normals.size() / 3)
                {
                    reportError(error, ModelLoadError::ParseFailed);
                    return false;
                }
                const tinyobj::real_t *normal
                    = attrib.normals.data() + static_cast<std::size_t>(index.normal_index) * 3;
                if (!std::isfinite(normal[0]) || !std::isfinite(normal[1]) || !std::isfinite(normal[2]))
                {
                    reportError(error, ModelLoadError::ParseFailed);
                    return false;
                }
            }
            else
            {
                anyMissingNormals = true;
            }
            if (options.importVertexColors
                && attrib.colors.size() >= (static_cast<std::size_t>(index.vertex_index) + 1) * 3)
            {
                const tinyobj::real_t *color = attrib.colors.data() + static_cast<std::size_t>(index.vertex_index) * 3;
                if (!std::isfinite(color[0]) || !std::isfinite(color[1]) || !std::isfinite(color[2]))
                {
                    reportError(error, ModelLoadError::ParseFailed);
                    return false;
                }
            }
            if (index.texcoord_index >= 0)
            {
                if (static_cast<std::size_t>(index.texcoord_index) >= attrib.texcoords.size() / 2)
                {
                    reportError(error, ModelLoadError::ParseFailed);
                    return false;
                }
                const tinyobj::real_t *uv
                    = attrib.texcoords.data() + static_cast<std::size_t>(index.texcoord_index) * 2;
                if (!std::isfinite(uv[0]) || !std::isfinite(uv[1]))
                {
                    reportError(error, ModelLoadError::ParseFailed);
                    return false;
                }
                anyUvs = true;
            }
        }

        // Welding collapses corners with identical keys; flat mode disables it because
        // every corner needs its own vertex. The fast path welds by position index
        // alone when the shape has neither normals nor uvs at all.
        std::unordered_map<ObjVertexKey, std::uint32_t, ObjVertexKeyHash> localVertices;
        std::vector<std::uint32_t>                                        positionRemap;
        std::vector<float>                                                vertices;
        std::vector<std::uint32_t>                                        indices;
        vertices.reserve(indexCount * renderer::meshVertexFloatCount);
        indices.reserve(indexCount);
        const bool weldVertices = options.weldVertices && options.normalMode != ModelNormalMode::GenerateFlat;

        const bool simplePositionKeys = weldVertices && allMissingNormals && !anyUvs;
        if (simplePositionKeys)
        {
            positionRemap.assign(attrib.vertices.size() / 3, invalidMaterial);
        }
        else if (weldVertices)
        {
            localVertices.reserve(std::min(indexCount, attrib.vertices.size() / 3));
        }

        // Which normals to emit: preserved from the file, generated smooth (also for
        // missing ones in PreserveOrGenerateSmooth mode), or flat per triangle.
        const bool preserveSourceNormals = options.normalMode == ModelNormalMode::PreserveOrGenerateSmooth
                                        || options.normalMode == ModelNormalMode::PreserveOnly;
        const bool generateSmoothNormals
            = options.normalMode == ModelNormalMode::GenerateSmooth
           || (options.normalMode == ModelNormalMode::PreserveOrGenerateSmooth && anyMissingNormals);
        const bool generateFlatNormals = options.normalMode == ModelNormalMode::GenerateFlat;
        if (generateSmoothNormals)
        {
            accumulateSmoothNormals(attrib.vertices,
                shape.mesh.indices,
                smoothNormalScratch,
                smoothNormalTouchedFlags,
                smoothNormalTouchedIndices);
        }

        // One precomputed face normal per triangle for flat shading.
        std::vector<float> flatNormals;
        if (generateFlatNormals)
        {
            flatNormals.resize((indexCount / 3) * 3);
            for (std::size_t corner = 0; corner + 2 < indexCount; corner += 3)
            {
                const tinyobj::index_t &indexA = shape.mesh.indices[corner + 0];
                const tinyobj::index_t &indexB = shape.mesh.indices[corner + 1];
                const tinyobj::index_t &indexC = shape.mesh.indices[corner + 2];
                const tinyobj::real_t  *a = attrib.vertices.data() + static_cast<std::size_t>(indexA.vertex_index) * 3;
                const tinyobj::real_t  *b = attrib.vertices.data() + static_cast<std::size_t>(indexB.vertex_index) * 3;
                const tinyobj::real_t  *c = attrib.vertices.data() + static_cast<std::size_t>(indexC.vertex_index) * 3;
                float faceNormal[3]{};
                crossProduct3(a, b, c, faceNormal);
                normalizeVector(faceNormal);
                std::copy(faceNormal, faceNormal + 3, flatNormals.begin() + (corner / 3) * 3);
            }
        }

        for (std::size_t corner = 0; corner < indexCount; ++corner)
        {
            const tinyobj::index_t &index = shape.mesh.indices[corner];

            // Corners whose normal will be generated weld by position (-1 in the key);
            // corners with an explicit file normal keep it in the key. Flat corners
            // never weld (a unique flatCorner per corner).
            const ObjVertexKey key{
                .position   = index.vertex_index,
                .normal     = generateSmoothNormals
                                   && (options.normalMode == ModelNormalMode::GenerateSmooth || index.normal_index < 0)
                                ? -1
                                : index.normal_index,
                .uv         = index.texcoord_index,
                .flatCorner = generateFlatNormals ? static_cast<std::uint32_t>(corner + 1) : 0u};

            if (simplePositionKeys)
            {
                const std::uint32_t remapped = positionRemap[static_cast<std::size_t>(index.vertex_index)];
                if (remapped != invalidMaterial)
                {
                    indices.push_back(remapped);
                    continue;
                }
            }
            else
            {
                const auto inserted = weldVertices ? localVertices.find(key) : localVertices.end();
                if (inserted != localVertices.end())
                {
                    indices.push_back(inserted->second);
                    continue;
                }
            }

            const std::uint32_t localIndex = static_cast<std::uint32_t>(vertices.size() / renderer::meshVertexFloatCount);
            const tinyobj::real_t *position = attrib.vertices.data() + static_cast<std::size_t>(index.vertex_index) * 3;
            if (!std::isfinite(position[0]) || !std::isfinite(position[1]) || !std::isfinite(position[2]))
            {
                reportError(error, ModelLoadError::ParseFailed);
                return false;
            }
            const std::size_t vertexOffset = vertices.size();
            vertices.resize(vertexOffset + renderer::meshVertexFloatCount);
            transformPosition(transform, position, vertices.data() + vertexOffset);
            if (!std::isfinite(vertices[vertexOffset + 0]) || !std::isfinite(vertices[vertexOffset + 1])
                || !std::isfinite(vertices[vertexOffset + 2]))
            {
                reportError(error, ModelLoadError::ParseFailed);
                return false;
            }

            // Normal source for a new vertex: flat face normal, generated smooth
            // normal, or the file's own; +Y until one of them applies.
            float sourceNormal[3] = {0.0f, 1.0f, 0.0f};
            if (generateFlatNormals)
            {
                std::copy_n(flatNormals.data() + (corner / 3) * 3, 3, sourceNormal);
            }
            else if (generateSmoothNormals
                     && (options.normalMode == ModelNormalMode::GenerateSmooth || index.normal_index < 0))
            {
                std::copy_n(
                    smoothNormalScratch.data() + static_cast<std::size_t>(index.vertex_index) * 3, 3, sourceNormal);
            }
            else if (preserveSourceNormals && index.normal_index >= 0)
            {
                const tinyobj::real_t *normal
                    = attrib.normals.data() + static_cast<std::size_t>(index.normal_index) * 3;
                std::copy_n(normal, 3, sourceNormal);
            }
            transformNormal(transform, sourceNormal, vertices.data() + vertexOffset + 3);

            float *color = vertices.data() + vertexOffset + 6;
            color[0] = color[1] = color[2] = 1.0f;
            if (options.importVertexColors
                && attrib.colors.size() >= (static_cast<std::size_t>(index.vertex_index) + 1) * 3)
            {
                const tinyobj::real_t *sourceColor
                    = attrib.colors.data() + static_cast<std::size_t>(index.vertex_index) * 3;
                color[0] = sourceColor[0];
                color[1] = sourceColor[1];
                color[2] = sourceColor[2];
            }

            // OBJ v runs bottom-up; flipTextureV converts to the renderer's convention.
            if (anyUvs && index.texcoord_index >= 0)
            {
                const tinyobj::real_t *uv
                    = attrib.texcoords.data() + static_cast<std::size_t>(index.texcoord_index) * 2;
                const float v               = options.flipTextureV ? 1.0f - uv[1] : uv[1];
                vertices[vertexOffset + 9]  = uv[0] * options.uvScale[0] + options.uvOffset[0];
                vertices[vertexOffset + 10] = v * options.uvScale[1] + options.uvOffset[1];
            }
            else
            {
                vertices[vertexOffset + 9]  = 0.0f;
                vertices[vertexOffset + 10] = 0.0f;
            }

            if (simplePositionKeys)
            {
                positionRemap[static_cast<std::size_t>(index.vertex_index)] = localIndex;
            }
            else if (weldVertices)
            {
                localVertices.emplace(key, localIndex);
            }
            indices.push_back(localIndex);
        }

        // Reset the per-shape touched bookkeeping so the next shape reuses the scratch.
        if (generateSmoothNormals)
        {
            for (const std::uint32_t index : smoothNormalTouchedIndices)
            {
                smoothNormalTouchedFlags[index] = 0;
            }
            smoothNormalTouchedIndices.clear();
        }
        if (vertices.empty())
        {
            continue;
        }

        if (transform.reverseWinding)
        {
            reverseTriangleWinding(indices);
        }
        // One material index per triangle; missing or unmapped ids become invalidMaterial.
        std::vector<std::uint32_t> triangleMaterials;
        if (options.importMaterials)
        {
            const std::size_t triangleCount = indexCount / 3;
            triangleMaterials.reserve(triangleCount);
            for (std::size_t triangle = 0; triangle < triangleCount; ++triangle)
            {
                const int sourceMaterial
                    = triangle < shape.mesh.material_ids.size() ? shape.mesh.material_ids[triangle] : -1;
                triangleMaterials.push_back(
                    sourceMaterial >= 0 && static_cast<std::size_t>(sourceMaterial) < record.materials.size()
                        ? static_cast<std::uint32_t>(sourceMaterial)
                        : invalidMaterial);
            }
        }
        ModelPart part{
            .vertices          = std::move(vertices),
            .indices           = std::move(indices),
            .name              = shape.name,
            .triangleMaterials = std::move(triangleMaterials)};
        // Unnamed shapes get a stable "part N" name.
        if (part.name.empty())
        {
            part.name = "part " + std::to_string(record.parts.size());
        }
        accumulateBounds(part, record);
        record.parts.push_back(std::move(part));
    }

    // An OBJ with no renderable shape is NoGeometry, not a success.
    if (record.parts.empty())
    {
        reportError(error, ModelLoadError::NoGeometry);
        return false;
    }
    reportError(error, ModelLoadError::None);
    return true;
}

} // namespace engine::models::detail
