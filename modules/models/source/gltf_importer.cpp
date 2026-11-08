// glTF 2.0 importer built on cgltf (declarations only; the parser implementation
// is compiled in cgltf_implementation.cpp).
//
// Walks the default scene, folds each node's world transform into its mesh
// primitives, and converts them into parts with materials and per-triangle
// material indices.

#include "model_import_internal.hpp"

#include <cgltf.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace engine::models::detail
{
namespace
{
// Column-major 4x4 in cgltf's float[16] convention (translation at [12..14]).
using Matrix4 = std::array<float, 16>;

bool finiteMatrix(const Matrix4 &matrix)
{
    return std::all_of(matrix.begin(), matrix.end(), [](float value) { return std::isfinite(value); });
}

// Transforms a direction by the inverse-transpose of the matrix's upper-left 3x3
// (built from cofactors); fails on singular matrices.
bool transformNodeNormal(const Matrix4 &matrix, const float *source, float *destination)
{
    const float c0[3]        = {matrix[0], matrix[1], matrix[2]};
    const float c1[3]        = {matrix[4], matrix[5], matrix[6]};
    const float c2[3]        = {matrix[8], matrix[9], matrix[10]};
    const float cofactor0[3] = {
        c1[1] * c2[2] - c1[2] * c2[1],
        c1[2] * c2[0] - c1[0] * c2[2],
        c1[0] * c2[1] - c1[1] * c2[0],
    };
    const float cofactor1[3] = {
        c2[1] * c0[2] - c2[2] * c0[1],
        c2[2] * c0[0] - c2[0] * c0[2],
        c2[0] * c0[1] - c2[1] * c0[0],
    };
    const float cofactor2[3] = {
        c0[1] * c1[2] - c0[2] * c1[1],
        c0[2] * c1[0] - c0[0] * c1[2],
        c0[0] * c1[1] - c0[1] * c1[0],
    };
    const float determinant = c0[0] * cofactor0[0] + c0[1] * cofactor0[1] + c0[2] * cofactor0[2];
    if (!std::isfinite(determinant) || std::abs(determinant) <= 1.0e-12f)
    {
        return false;
    }
    const float inverseDeterminant = 1.0f / determinant;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        destination[axis] = (cofactor0[axis] * source[0] + cofactor1[axis] * source[1] + cofactor2[axis] * source[2])
                          * inverseDeterminant;
    }
    return std::isfinite(destination[0]) && std::isfinite(destination[1]) && std::isfinite(destination[2]);
}

// Applies the full 4x4 with perspective divide by w; fails when w or the result
// is degenerate or non-finite.
bool transformNodePosition(const Matrix4 &matrix, const float *source, float *destination)
{
    const float x = source[0];
    const float y = source[1];
    const float z = source[2];
    const float w = matrix[3] * x + matrix[7] * y + matrix[11] * z + matrix[15];
    if (!std::isfinite(w) || std::abs(w) <= 1.0e-12f)
    {
        return false;
    }
    const float inverseW = 1.0f / w;
    destination[0]       = (matrix[0] * x + matrix[4] * y + matrix[8] * z + matrix[12]) * inverseW;
    destination[1]       = (matrix[1] * x + matrix[5] * y + matrix[9] * z + matrix[13]) * inverseW;
    destination[2]       = (matrix[2] * x + matrix[6] * y + matrix[10] * z + matrix[14]) * inverseW;
    return std::isfinite(destination[0]) && std::isfinite(destination[1]) && std::isfinite(destination[2]);
}

// Determinant of the upper-left 3x3; negative means the node mirrors geometry.
float nodeDeterminant(const Matrix4 &matrix)
{
    const float c0[3] = {matrix[0], matrix[1], matrix[2]};
    const float c1[3] = {matrix[4], matrix[5], matrix[6]};
    const float c2[3] = {matrix[8], matrix[9], matrix[10]};
    return c0[0] * (c1[1] * c2[2] - c1[2] * c2[1]) + c0[1] * (c1[2] * c2[0] - c1[0] * c2[2])
         + c0[2] * (c1[0] * c2[1] - c1[1] * c2[0]);
}

const cgltf_accessor *findAccessor(const cgltf_primitive &primitive, cgltf_attribute_type type)
{
    return cgltf_find_accessor(&primitive, type, 0);
}

// Unpacks an accessor into `components`-tuples of floats via cgltf; any decode
// failure or short read fails the load.
bool unpackFloats(const cgltf_accessor *accessor, std::size_t components, std::vector<float> &output)
{
    if (accessor == nullptr || components == 0
        || accessor->count > std::numeric_limits<std::size_t>::max() / components)
    {
        return false;
    }
    const std::size_t floatCount = static_cast<std::size_t>(accessor->count) * components;
    output.resize(floatCount);
    return cgltf_accessor_unpack_floats(accessor, output.data(), output.size()) == output.size();
}

// Image URI of a texture view, preferring the KHR_texture_basisu/webp images
// when the default image carries no URI.
std::string textureUri(const cgltf_texture_view &view)
{
    if (view.texture == nullptr)
    {
        return {};
    }
    const cgltf_image *image = view.texture->image;
    if ((image == nullptr || image->uri == nullptr) && view.texture->has_basisu)
    {
        image = view.texture->basisu_image;
    }
    if ((image == nullptr || image->uri == nullptr) && view.texture->has_webp)
    {
        image = view.texture->webp_image;
    }
    return image != nullptr && image->uri != nullptr ? image->uri : std::string{};
}

// Copies one texture view into a material slot: URI, KHR_texture_transform
// (offset/scale/rotation/texCoord), sampler filters/wraps, plus the view scales
// that map onto bumpMultiplier (normal) and brightness (ambient/occlusion).
void copyMaterialTexture(
    const cgltf_texture_view &source, ModelMaterialRecord &destination, ModelMaterialTextureSlot slot)
{
    destination.texturePaths[static_cast<std::size_t>(slot)] = textureUri(source);
    ModelMaterialTextureOptions &options = destination.textureOptions[static_cast<std::size_t>(slot)];
    options.texCoord                     = source.texcoord >= 0 ? static_cast<std::uint32_t>(source.texcoord) : 0u;
    if (source.has_transform)
    {
        options.offset[0] = source.transform.offset[0];
        options.offset[1] = source.transform.offset[1];
        options.scale[0]  = source.transform.scale[0];
        options.scale[1]  = source.transform.scale[1];
        options.rotation  = source.transform.rotation;
        if (source.transform.has_texcoord && source.transform.texcoord >= 0)
        {
            options.texCoord = static_cast<std::uint32_t>(source.transform.texcoord);
        }
    }
    if (source.texture != nullptr && source.texture->sampler != nullptr)
    {
        options.samplerMinFilter = static_cast<std::uint32_t>(source.texture->sampler->min_filter);
        options.samplerMagFilter = static_cast<std::uint32_t>(source.texture->sampler->mag_filter);
        options.wrapU            = static_cast<std::uint32_t>(source.texture->sampler->wrap_s);
        options.wrapV            = static_cast<std::uint32_t>(source.texture->sampler->wrap_t);
    }
    if (slot == ModelMaterialTextureSlot::Normal)
    {
        options.bumpMultiplier = source.scale;
    }
    else if (slot == ModelMaterialTextureSlot::Ambient)
    {
        options.brightness = source.scale;
    }
}

// Converts one cgltf material: core PBR metallic-roughness factors, alpha mode,
// and the supported extensions (ior, clearcoat, sheen, transmission, specular,
// iridescence, volume, emissive strength, anisotropy, diffuse transmission,
// dispersion). Returns false on non-finite inputs.
bool importMaterial(const cgltf_material &source, ModelMaterialRecord &destination)
{
    destination.name                        = source.name != nullptr ? source.name : "material";
    const cgltf_pbr_metallic_roughness &pbr = source.pbr_metallic_roughness;
    for (std::size_t channel = 0; channel < 3; ++channel)
    {
        const float baseColor = pbr.base_color_factor[channel];
        const float emissive  = source.emissive_factor[channel];
        if (!std::isfinite(baseColor) || !std::isfinite(emissive))
        {
            return false;
        }
        destination.factors.baseColor[channel] = std::clamp(baseColor, 0.0f, 1.0f);
        destination.factors.emissive[channel]  = std::max(emissive, 0.0f);
    }
    destination.factors.opacity       = std::clamp(pbr.base_color_factor[3], 0.0f, 1.0f);
    destination.factors.metallic      = std::clamp(pbr.metallic_factor, 0.0f, 1.0f);
    destination.factors.roughness     = std::clamp(pbr.roughness_factor, 0.0f, 1.0f);
    destination.factors.hasPbrFactors = true;
    destination.factors.unlit         = source.unlit != 0;
    destination.factors.doubleSided   = source.double_sided != 0;
    destination.factors.alphaMode     = static_cast<std::uint32_t>(source.alpha_mode);
    destination.factors.alphaCutoff
        = std::isfinite(source.alpha_cutoff) ? std::clamp(source.alpha_cutoff, 0.0f, 1.0f) : 0.5f;
    if (source.has_ior && std::isfinite(source.ior.ior) && source.ior.ior > 0.0f)
    {
        destination.factors.indexOfRefraction = source.ior.ior;
    }
    if (source.has_clearcoat)
    {
        destination.factors.clearcoat          = std::clamp(source.clearcoat.clearcoat_factor, 0.0f, 1.0f);
        destination.factors.clearcoatRoughness = std::clamp(source.clearcoat.clearcoat_roughness_factor, 0.0f, 1.0f);
    }
    if (source.has_sheen)
    {
        destination.factors.sheenRoughness = std::clamp(source.sheen.sheen_roughness_factor, 0.0f, 1.0f);
        std::copy_n(source.sheen.sheen_color_factor, 3, destination.factors.sheenColor);
        destination.factors.sheen
            = std::clamp(std::max(destination.factors.sheenColor[0],
                             std::max(destination.factors.sheenColor[1], destination.factors.sheenColor[2])),
                0.0f,
                1.0f);
        copyMaterialTexture(source.sheen.sheen_color_texture, destination, ModelMaterialTextureSlot::SheenColor);
        copyMaterialTexture(
            source.sheen.sheen_roughness_texture, destination, ModelMaterialTextureSlot::SheenRoughness);
    }
    if (source.has_transmission)
    {
        destination.factors.transmission = std::clamp(source.transmission.transmission_factor, 0.0f, 1.0f);
        copyMaterialTexture(
            source.transmission.transmission_texture, destination, ModelMaterialTextureSlot::Transmission);
    }
    if (source.has_specular)
    {
        destination.factors.specularFactor = std::clamp(source.specular.specular_factor, 0.0f, 1.0f);
        std::copy_n(source.specular.specular_color_factor, 3, destination.factors.specular);
        copyMaterialTexture(source.specular.specular_texture, destination, ModelMaterialTextureSlot::Specular);
        copyMaterialTexture(
            source.specular.specular_color_texture, destination, ModelMaterialTextureSlot::SpecularColor);
    }
    if (source.has_iridescence)
    {
        destination.factors.iridescence                 = std::clamp(source.iridescence.iridescence_factor, 0.0f, 1.0f);
        destination.factors.iridescenceIor              = source.iridescence.iridescence_ior;
        destination.factors.iridescenceThicknessMinimum = source.iridescence.iridescence_thickness_min;
        destination.factors.iridescenceThicknessMaximum = source.iridescence.iridescence_thickness_max;
        copyMaterialTexture(source.iridescence.iridescence_texture, destination, ModelMaterialTextureSlot::Iridescence);
        copyMaterialTexture(source.iridescence.iridescence_thickness_texture,
            destination,
            ModelMaterialTextureSlot::IridescenceThickness);
    }
    if (source.has_volume)
    {
        destination.factors.thickness = std::clamp(source.volume.thickness_factor, 0.0f, 1.0f);
        std::copy_n(source.volume.attenuation_color, 3, destination.factors.attenuationColor);
        destination.factors.attenuationDistance = source.volume.attenuation_distance;
        copyMaterialTexture(source.volume.thickness_texture, destination, ModelMaterialTextureSlot::Thickness);
    }
    // Emissive strength multiplies the emissive factor; guard against overflow.
    if (source.has_emissive_strength)
    {
        if (!std::isfinite(source.emissive_strength.emissive_strength))
        {
            return false;
        }
        destination.factors.emissiveStrength = std::max(source.emissive_strength.emissive_strength, 0.0f);
        for (float &channel : destination.factors.emissive)
        {
            channel *= destination.factors.emissiveStrength;
            if (!std::isfinite(channel))
            {
                return false;
            }
        }
    }
    if (source.has_anisotropy)
    {
        destination.factors.anisotropy         = std::clamp(source.anisotropy.anisotropy_strength, 0.0f, 1.0f);
        destination.factors.anisotropyRotation = source.anisotropy.anisotropy_rotation;
        copyMaterialTexture(source.anisotropy.anisotropy_texture, destination, ModelMaterialTextureSlot::Anisotropy);
    }
    if (source.has_diffuse_transmission)
    {
        destination.factors.diffuseTransmission
            = std::clamp(source.diffuse_transmission.diffuse_transmission_factor, 0.0f, 1.0f);
        std::copy_n(source.diffuse_transmission.diffuse_transmission_color_factor,
            3,
            destination.factors.diffuseTransmissionColor);
        copyMaterialTexture(source.diffuse_transmission.diffuse_transmission_texture,
            destination,
            ModelMaterialTextureSlot::DiffuseTransmission);
        copyMaterialTexture(source.diffuse_transmission.diffuse_transmission_color_texture,
            destination,
            ModelMaterialTextureSlot::DiffuseTransmissionColor);
    }
    if (source.has_dispersion)
    {
        destination.factors.dispersion = std::max(source.dispersion.dispersion, 0.0f);
    }
    copyMaterialTexture(source.clearcoat.clearcoat_texture, destination, ModelMaterialTextureSlot::Clearcoat);
    copyMaterialTexture(
        source.clearcoat.clearcoat_roughness_texture, destination, ModelMaterialTextureSlot::ClearcoatRoughness);
    copyMaterialTexture(
        source.clearcoat.clearcoat_normal_texture, destination, ModelMaterialTextureSlot::ClearcoatNormal);
    copyMaterialTexture(pbr.base_color_texture, destination, ModelMaterialTextureSlot::BaseColor);
    copyMaterialTexture(pbr.metallic_roughness_texture, destination, ModelMaterialTextureSlot::Metallic);
    // glTF stores metallic and roughness in one texture; expose it in both slots.
    destination.texturePaths[static_cast<std::size_t>(ModelMaterialTextureSlot::Roughness)]
        = destination.texturePaths[static_cast<std::size_t>(ModelMaterialTextureSlot::Metallic)];
    destination.textureOptions[static_cast<std::size_t>(ModelMaterialTextureSlot::Roughness)]
        = destination.textureOptions[static_cast<std::size_t>(ModelMaterialTextureSlot::Metallic)];
    copyMaterialTexture(source.normal_texture, destination, ModelMaterialTextureSlot::Normal);
    copyMaterialTexture(source.occlusion_texture, destination, ModelMaterialTextureSlot::Ambient);
    copyMaterialTexture(source.emissive_texture, destination, ModelMaterialTextureSlot::Emissive);
    return true;
}

// Expands the primitive's indices (or a sequential range) into a triangle list.
// Strip and fan topologies are converted with the standard alternating flip and
// degenerate triangles are dropped; out-of-range indices fail the load.
bool buildIndices(const cgltf_primitive &primitive, std::uint32_t vertexCount, std::vector<std::uint32_t> &indices)
{
    std::vector<std::uint32_t> sourceIndices;
    if (primitive.indices != nullptr)
    {
        if (primitive.indices->count > std::numeric_limits<std::uint32_t>::max())
        {
            return false;
        }
        sourceIndices.resize(static_cast<std::size_t>(primitive.indices->count));
        if (cgltf_accessor_unpack_indices(
                primitive.indices, sourceIndices.data(), sizeof(std::uint32_t), sourceIndices.size())
            != sourceIndices.size())
        {
            return false;
        }
    }
    else
    {
        sourceIndices.resize(vertexCount);
        for (std::uint32_t index = 0; index < vertexCount; ++index)
        {
            sourceIndices[index] = index;
        }
    }
    for (const std::uint32_t index : sourceIndices)
    {
        if (index >= vertexCount)
        {
            return false;
        }
    }

    indices.clear();
    if (primitive.type == cgltf_primitive_type_triangles)
    {
        if (sourceIndices.size() % 3 != 0)
        {
            return false;
        }
        indices = std::move(sourceIndices);
    }
    else if (primitive.type == cgltf_primitive_type_triangle_strip)
    {
        if (sourceIndices.size() < 3)
        {
            return true;
        }
        indices.reserve((sourceIndices.size() - 2) * 3);
        for (std::size_t index = 0; index + 2 < sourceIndices.size(); ++index)
        {
            const std::uint32_t a = sourceIndices[index + (index & 1u)];
            const std::uint32_t b = sourceIndices[index + ((index & 1u) ? 0u : 1u)];
            const std::uint32_t c = sourceIndices[index + 2];
            if (a != b && b != c && a != c)
            {
                indices.insert(indices.end(), {a, b, c});
            }
        }
    }
    else if (primitive.type == cgltf_primitive_type_triangle_fan)
    {
        if (sourceIndices.size() < 3)
        {
            return true;
        }
        indices.reserve((sourceIndices.size() - 2) * 3);
        const std::uint32_t center = sourceIndices.front();
        for (std::size_t index = 1; index + 1 < sourceIndices.size(); ++index)
        {
            const std::uint32_t b = sourceIndices[index];
            const std::uint32_t c = sourceIndices[index + 1];
            if (center != b && b != c && center != c)
            {
                indices.insert(indices.end(), {center, b, c});
            }
        }
    }
    else
    {
        return false;
    }
    return !indices.empty();
}

// Converts one primitive into a part. Draco compression and morph targets are
// rejected; positions must be vec3, and normal/uv/color accessors must match the
// vertex count if present.
bool appendPrimitive(const cgltf_data &data, const cgltf_mesh &mesh, const cgltf_primitive &primitive,
    const Matrix4 &nodeMatrix, const std::string &nodeName, ModelRecord &record, const ModelLoadOptions &options,
    const ModelTransform &transform, bool *hasGeometry)
{
    // Content this importer cannot decode.
    if (primitive.has_draco_mesh_compression || primitive.targets_count != 0)
    {
        return false;
    }
    const cgltf_accessor *positionAccessor = findAccessor(primitive, cgltf_attribute_type_position);
    if (positionAccessor == nullptr || positionAccessor->type != cgltf_type_vec3 || positionAccessor->count == 0
        || positionAccessor->count > std::numeric_limits<std::uint32_t>::max())
    {
        return false;
    }
    const std::size_t vertexCount = static_cast<std::size_t>(positionAccessor->count);
    if (vertexCount > std::vector<float>().max_size() / renderer::meshVertexFloatCount)
    {
        return false;
    }
    const cgltf_accessor *normalAccessor = findAccessor(primitive, cgltf_attribute_type_normal);
    const cgltf_accessor *uvAccessor     = findAccessor(primitive, cgltf_attribute_type_texcoord);
    const cgltf_accessor *colorAccessor
        = options.importVertexColors ? findAccessor(primitive, cgltf_attribute_type_color) : nullptr;
    if ((normalAccessor != nullptr && (normalAccessor->type != cgltf_type_vec3 || normalAccessor->count != vertexCount))
        || (uvAccessor != nullptr && (uvAccessor->type != cgltf_type_vec2 || uvAccessor->count != vertexCount))
        || (colorAccessor != nullptr
            && ((colorAccessor->type != cgltf_type_vec3 && colorAccessor->type != cgltf_type_vec4)
                || colorAccessor->count != vertexCount)))
    {
        return false;
    }

    std::vector<float> positions;
    std::vector<float> normals;
    std::vector<float> uvs;
    std::vector<float> colors;
    if (!unpackFloats(positionAccessor, 3, positions)
        || (normalAccessor != nullptr && !unpackFloats(normalAccessor, 3, normals))
        || (uvAccessor != nullptr && !unpackFloats(uvAccessor, 2, uvs))
        || (colorAccessor != nullptr
            && !unpackFloats(colorAccessor, cgltf_num_components(colorAccessor->type), colors)))
    {
        return false;
    }

    ModelPart part;
    part.name = nodeName.empty() ? (mesh.name != nullptr ? mesh.name : "mesh") : nodeName;
    part.vertices.resize(vertexCount * renderer::meshVertexFloatCount);
    // Normals are preserved through the node matrix (inverse-transpose) unless the
    // mode forces generation; a failed node transform falls back to smooth normals.
    const bool preserveNormals  = normalAccessor != nullptr && options.normalMode != ModelNormalMode::GenerateSmooth
                               && options.normalMode != ModelNormalMode::GenerateFlat;
    bool       nodeNormalsValid = true;
    for (std::size_t index = 0; index < vertexCount; ++index)
    {
        const float *sourcePosition = positions.data() + index * 3;
        float        nodePosition[3]{};
        float       *vertex = part.vertices.data() + index * renderer::meshVertexFloatCount;
        // Node world transform first, then the option transform on top.
        if (!transformNodePosition(nodeMatrix, sourcePosition, nodePosition))
        {
            return false;
        }
        transformPosition(transform, nodePosition, vertex);
        if (!std::isfinite(vertex[0]) || !std::isfinite(vertex[1]) || !std::isfinite(vertex[2]))
        {
            return false;
        }
        vertex[3] = 0.0f;
        vertex[4] = 1.0f;
        vertex[5] = 0.0f;
        if (preserveNormals)
        {
            float nodeNormal[3]{};
            if (transformNodeNormal(nodeMatrix, normals.data() + index * 3, nodeNormal))
            {
                transformNormal(transform, nodeNormal, vertex + 3);
            }
            else
            {
                nodeNormalsValid = false;
            }
        }
        vertex[6] = vertex[7] = vertex[8] = 1.0f;
        if (colorAccessor != nullptr)
        {
            const std::size_t colorStride = cgltf_num_components(colorAccessor->type);
            const float      *color       = colors.data() + index * colorStride;
            vertex[6]                     = std::clamp(color[0], 0.0f, 1.0f);
            vertex[7]                     = std::clamp(color[1], 0.0f, 1.0f);
            vertex[8]                     = std::clamp(color[2], 0.0f, 1.0f);
        }
        if (uvAccessor != nullptr)
        {
            const float u = uvs[index * 2 + 0];
            const float v = uvs[index * 2 + 1];
            vertex[9]     = u * options.uvScale[0] + options.uvOffset[0];
            vertex[10]    = (options.flipTextureV ? 1.0f - v : v) * options.uvScale[1] + options.uvOffset[1];
        }
    }

    if (!buildIndices(primitive, static_cast<std::uint32_t>(vertexCount), part.indices))
    {
        return false;
    }
    // Two mirrored sources cancel out: flip only when exactly one of the option
    // transform and the node matrix mirrors the geometry.
    if (transform.reverseWinding != (nodeDeterminant(nodeMatrix) < 0.0f))
    {
        reverseTriangleWinding(part.indices);
    }
    // Regenerate normals whenever the preserved ones are absent or unusable.
    const bool generateSmooth = options.normalMode == ModelNormalMode::GenerateSmooth
                             || (options.normalMode == ModelNormalMode::PreserveOrGenerateSmooth && !preserveNormals)
                             || (preserveNormals && !nodeNormalsValid);
    if (generateSmooth)
    {
        std::vector<float> generatedNormals;
        accumulateSmoothNormalsInterleaved(part.vertices, part.indices, generatedNormals);
        for (std::size_t index = 0; index < vertexCount; ++index)
        {
            std::copy_n(generatedNormals.data() + index * 3,
                3,
                part.vertices.data() + index * renderer::meshVertexFloatCount + 3);
        }
    }
    // Flat mode: expand to one vertex per corner carrying the face normal.
    if (options.normalMode == ModelNormalMode::GenerateFlat)
    {
        std::vector<float> flatVertices;
        flatVertices.resize(part.indices.size() * renderer::meshVertexFloatCount);
        for (std::size_t triangle = 0; triangle < part.indices.size(); triangle += 3)
        {
            const std::uint32_t indexA = part.indices[triangle + 0];
            const std::uint32_t indexB = part.indices[triangle + 1];
            const std::uint32_t indexC = part.indices[triangle + 2];
            const float *a     = part.vertices.data() + static_cast<std::size_t>(indexA) * renderer::meshVertexFloatCount;
            const float *b     = part.vertices.data() + static_cast<std::size_t>(indexB) * renderer::meshVertexFloatCount;
            const float *c     = part.vertices.data() + static_cast<std::size_t>(indexC) * renderer::meshVertexFloatCount;
            const float  ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
            const float  ac[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
            float        normal[3] = {
                ab[1] * ac[2] - ab[2] * ac[1],
                ab[2] * ac[0] - ab[0] * ac[2],
                ab[0] * ac[1] - ab[1] * ac[0],
            };
            normalizeVector(normal);
            for (std::size_t corner = 0; corner < 3; ++corner)
            {
                const float *source
                    = part.vertices.data()
                    + static_cast<std::size_t>(part.indices[triangle + corner]) * renderer::meshVertexFloatCount;
                float *destination = flatVertices.data() + (triangle + corner) * renderer::meshVertexFloatCount;
                std::copy_n(source, renderer::meshVertexFloatCount, destination);
                std::copy_n(normal, 3, destination + 3);
            }
        }
        part.vertices = std::move(flatVertices);
        part.indices.resize(part.vertices.size() / renderer::meshVertexFloatCount);
        for (std::size_t index = 0; index < part.indices.size(); ++index)
        {
            part.indices[index] = static_cast<std::uint32_t>(index);
        }
    }

    // Triangles record the primitive's material by its index in the file's list.
    if (options.importMaterials && primitive.material != nullptr)
    {
        const auto materialOffset = primitive.material - data.materials;
        if (materialOffset < 0 || static_cast<std::size_t>(materialOffset) >= data.materials_count)
        {
            return false;
        }
        const std::uint32_t materialIndex = static_cast<std::uint32_t>(materialOffset);
        part.triangleMaterials.assign(part.indices.size() / 3, materialIndex);
    }
    // Primitives that degenerated to zero triangles produce no part.
    if (part.indices.empty())
    {
        return true;
    }
    record.parts.push_back(std::move(part));
    *hasGeometry = true;
    return true;
}
} // namespace

// The allow-list of "extensionsRequired" entries this importer understands;
// anything else fails the load rather than importing wrong data.
bool isSupportedRequiredExtension(const char *extension)
{
    if (extension == nullptr)
    {
        return false;
    }
    constexpr const char *supported[] = {
        "KHR_materials_clearcoat",
        "KHR_materials_dispersion",
        "KHR_materials_emissive_strength",
        "KHR_materials_ior",
        "KHR_materials_iridescence",
        "KHR_materials_sheen",
        "KHR_materials_specular",
        "KHR_materials_transmission",
        "KHR_materials_unlit",
        "KHR_materials_volume",
        "KHR_materials_diffuse_transmission",
        "KHR_materials_anisotropy",
        "KHR_texture_transform",
        "KHR_mesh_quantization",
    };
    return std::any_of(std::begin(supported),
        std::end(supported),
        [extension](const char *candidate) { return std::string(extension) == candidate; });
}

// Loads and validates the glTF document, imports materials, then walks the scene
// (default scene, else first scene, else root nodes, else all meshes with identity
// transforms) importing every mesh primitive as a part.
bool importGltfFile(const std::filesystem::path &path, ModelRecord &record, const ModelLoadOptions &options,
    const ModelTransform &transform, ModelLoadError *error)
{
    // cgltf owns the parsed document; the unique_ptr frees it on every exit path.
    cgltf_options parserOptions{};
    cgltf_data   *rawData = nullptr;
    if (cgltf_parse_file(&parserOptions, path.string().c_str(), &rawData) != cgltf_result_success || rawData == nullptr)
    {
        reportError(error, ModelLoadError::ParseFailed);
        return false;
    }
    std::unique_ptr<cgltf_data, decltype(&cgltf_free)> data(rawData, &cgltf_free);
    if (cgltf_load_buffers(&parserOptions, data.get(), path.string().c_str()) != cgltf_result_success
        || cgltf_validate(data.get()) != cgltf_result_success)
    {
        reportError(error, ModelLoadError::ParseFailed);
        return false;
    }
    for (std::size_t index = 0; index < data->extensions_required_count; ++index)
    {
        if (!isSupportedRequiredExtension(data->extensions_required[index]))
        {
            reportError(error, ModelLoadError::ParseFailed);
            return false;
        }
    }
    // Meshopt-compressed buffer views are rejected instead of decoded.
    for (std::size_t index = 0; index < data->buffer_views_count; ++index)
    {
        if (data->buffer_views[index].has_meshopt_compression)
        {
            reportError(error, ModelLoadError::ParseFailed);
            return false;
        }
    }

    if (options.importMaterials)
    {
        record.materials.reserve(data->materials_count);
        for (std::size_t index = 0; index < data->materials_count; ++index)
        {
            ModelMaterialRecord material;
            if (!importMaterial(data->materials[index], material))
            {
                reportError(error, ModelLoadError::ParseFailed);
                return false;
            }
            record.materials.push_back(std::move(material));
        }
    }

    bool          hasGeometry = false;
    const Matrix4 identity
        = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const auto importMesh = [&](const cgltf_mesh &mesh, const Matrix4 &nodeMatrix, const std::string &nodeName)
    {
        for (std::size_t primitiveIndex = 0; primitiveIndex < mesh.primitives_count; ++primitiveIndex)
        {
            if (!appendPrimitive(*data,
                    mesh,
                    mesh.primitives[primitiveIndex],
                    nodeMatrix,
                    nodeName,
                    record,
                    options,
                    transform,
                    &hasGeometry))
            {
                return false;
            }
        }
        return true;
    };

    // Depth-first scene walk with cycle detection; skinned meshes and
    // EXT_mesh_gpu_instancing nodes are rejected as unsupported.
    std::unordered_set<const cgltf_node *> visitedNodes;
    const auto                             visitNode = [&](const auto &self, const cgltf_node *node) -> bool
    {
        if (node == nullptr)
        {
            return false;
        }
        if (!visitedNodes.insert(node).second)
        {
            return false;
        }
        if ((node->skin != nullptr && node->mesh != nullptr) || node->has_mesh_gpu_instancing)
        {
            return false;
        }
        Matrix4 nodeMatrix{};
        cgltf_node_transform_world(node, nodeMatrix.data());
        if (!finiteMatrix(nodeMatrix))
        {
            return false;
        }
        if (node->mesh != nullptr)
        {
            if (!importMesh(*node->mesh, nodeMatrix, node->name != nullptr ? node->name : ""))
            {
                return false;
            }
        }
        for (std::size_t child = 0; child < node->children_count; ++child)
        {
            if (!self(self, node->children[child]))
            {
                return false;
            }
        }
        return true;
    };

    // Fallback order: default scene, first scene, unparented nodes, raw meshes.
    const cgltf_scene *scene = data->scene;
    if (scene == nullptr && data->scenes_count != 0)
    {
        scene = &data->scenes[0];
    }
    if (scene != nullptr)
    {
        for (std::size_t index = 0; index < scene->nodes_count; ++index)
        {
            if (!visitNode(visitNode, scene->nodes[index]))
            {
                reportError(error, ModelLoadError::ParseFailed);
                return false;
            }
        }
    }
    else if (data->nodes_count != 0)
    {
        for (std::size_t index = 0; index < data->nodes_count; ++index)
        {
            if (data->nodes[index].parent == nullptr && !visitNode(visitNode, &data->nodes[index]))
            {
                reportError(error, ModelLoadError::ParseFailed);
                return false;
            }
        }
    }
    else
    {
        for (std::size_t index = 0; index < data->meshes_count; ++index)
        {
            if (!importMesh(
                    data->meshes[index], identity, data->meshes[index].name != nullptr ? data->meshes[index].name : ""))
            {
                reportError(error, ModelLoadError::ParseFailed);
                return false;
            }
        }
    }

    if (!hasGeometry || record.parts.empty())
    {
        reportError(error, ModelLoadError::NoGeometry);
        return false;
    }
    reportError(error, ModelLoadError::None);
    return true;
}

} // namespace engine::models::detail
