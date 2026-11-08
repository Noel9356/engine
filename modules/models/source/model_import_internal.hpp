// Shared internal types and helpers for the models importers.
//
// Every format importer (obj/stl/ply/off/gltf) turns its file into a ModelRecord
// using these building blocks: option-derived transforms, normal generation and
// index utilities. models.cpp drives the importers and owns the finished records.

#ifndef ENGINE_MODELS_MODEL_IMPORT_INTERNAL_HPP
#define ENGINE_MODELS_MODEL_IMPORT_INTERNAL_HPP

#include <engine/models/capabilities.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace engine::models::detail
{

// One renderable piece of a loaded model. Vertices are interleaved with
// renderer::meshVertexFloatCount (11) floats per vertex: position, normal, color,
// uv. Indices are a uint32 triangle list; triangleMaterials has one entry per
// triangle (index into ModelRecord::materials, or invalidMaterial).
struct ModelPart
{
    std::vector<float>         vertices;
    std::vector<std::uint32_t> indices;
    std::string                name;
    std::vector<std::uint32_t> triangleMaterials;
};

// Module-side material record: the consumer-facing factors plus the raw texture
// URIs and per-slot sampling/transform options gathered from the source file.
struct ModelMaterialRecord
{
    ModelMaterial                                                                                      factors;
    std::string                                                                                        name;
    std::array<std::string, static_cast<std::size_t>(ModelMaterialTextureSlot::Count)>                 texturePaths;
    std::array<ModelMaterialTextureOptions, static_cast<std::size_t>(ModelMaterialTextureSlot::Count)> textureOptions;
};

// Meshoptimizer pass selection for the post-import optimization step
// (module configuration, not per-load options).
struct ModelOptimizationOptions
{
    bool  vertexCache       = true;  // reorder triangles for post-transform cache hits
    bool  overdraw          = true;  // then reorder again to reduce pixel overdraw
    bool  vertexFetch       = true;  // finally reorder vertices for sequential GPU fetch
    float overdrawThreshold = 1.05f; // allowed overdraw increase for the overdraw pass
};

// Everything owned by one loaded model handle: post-transform bounds and
// statistics, the parts in source order, and the materials in source order.
struct ModelRecord
{
    float minimum[3] = {0.0f, 0.0f, 0.0f};
    float maximum[3] = {0.0f, 0.0f, 0.0f};

    ModelStatistics                  statistics;
    std::vector<ModelPart>           parts;
    std::vector<ModelMaterialRecord> materials;
};

// Precomputed option transform shared by all importers. positionMatrix is the
// row-major rotation (from the unit quaternion) times per-axis scale; normalMatrix
// is the rotation divided by the scale (correct for transforming directions under
// non-uniform scale); positions are transformed pivot-relative.
struct ModelTransform
{
    float positionMatrix[3][3]{};
    float normalMatrix[3][3]{};
    float translation[3]{};
    float pivot[3]{};
    bool  reverseWinding = false;
};

// Sanity bound applied to scale/translation/pivot/uv values (|value| must not
// exceed this) so imports cannot overflow float arithmetic downstream.
inline constexpr double maximumTransformMagnitude = 1.0e12;

// Normalizes a 3-vector in place (double precision). Degenerate inputs fall back
// to `fallback`, or +Y when none is given.
inline void normalizeVector(float *value, const float *fallback = nullptr)
{
    const double length = std::hypot(
        std::hypot(static_cast<double>(value[0]), static_cast<double>(value[1])), static_cast<double>(value[2]));
    if (std::isfinite(length) && length > 1e-12)
    {
        value[0] = static_cast<float>(value[0] / length);
        value[1] = static_cast<float>(value[1] / length);
        value[2] = static_cast<float>(value[2] / length);
    }
    else if (fallback != nullptr)
    {
        value[0] = fallback[0];
        value[1] = fallback[1];
        value[2] = fallback[2];
    }
    else
    {
        value[0] = 0.0f;
        value[1] = 1.0f;
        value[2] = 0.0f;
    }
}

// Builds the option transform. Winding reversal happens for ModelWindingMode::Reverse
// and, in Automatic mode, when an odd number of scale axes is negative (handedness
// flip); Preserve always keeps the source order.
inline ModelTransform makeTransform(const ModelLoadOptions &options)
{
    const float x = options.rotation[0];
    const float y = options.rotation[1];
    const float z = options.rotation[2];
    const float w = options.rotation[3];
    const float rotation[3][3]
        = {{1.0f - 2.0f * (y * y + z * z), 2.0f * (x * y - z * w), 2.0f * (x * z + y * w)},
            {2.0f * (x * y + z * w), 1.0f - 2.0f * (x * x + z * z), 2.0f * (y * z - x * w)},
            {2.0f * (x * z - y * w), 2.0f * (y * z + x * w), 1.0f - 2.0f * (x * x + y * y)}};
    ModelTransform transform;
    for (std::size_t row = 0; row < 3; ++row)
    {
        transform.translation[row] = options.translation[row];
        transform.pivot[row]       = options.pivot[row];
        for (std::size_t column = 0; column < 3; ++column)
        {
            transform.positionMatrix[row][column] = rotation[row][column] * options.scale[column];
            // Dividing by scale is correct for normals; zero scales collapse that axis to 0.
            transform.normalMatrix[row][column]
                = std::abs(options.scale[column]) > 1e-12f ? rotation[row][column] / options.scale[column] : 0.0f;
        }
    }
    const bool negativeScaleParity = (options.scale[0] < 0.0f) ^ (options.scale[1] < 0.0f) ^ (options.scale[2] < 0.0f);
    transform.reverseWinding       = options.windingMode == ModelWindingMode::Reverse
                                  || (options.windingMode == ModelWindingMode::Automatic && negativeScaleParity);
    if (options.windingMode == ModelWindingMode::Preserve)
    {
        transform.reverseWinding = false;
    }
    return transform;
}

// Applies the option transform: subtract pivot, rotate+scale, add translation.
template <typename Real>
inline void transformPosition(const ModelTransform &transform, const Real *source, float *destination)
{
    const float x = source[0] - transform.pivot[0];
    const float y = source[1] - transform.pivot[1];
    const float z = source[2] - transform.pivot[2];
    for (std::size_t row = 0; row < 3; ++row)
    {
        destination[row] = transform.positionMatrix[row][0] * x + transform.positionMatrix[row][1] * y
                         + transform.positionMatrix[row][2] * z + transform.translation[row];
    }
}

// Transforms a direction by the normal matrix and renormalizes (importers hand
// this unnormalized source normals or geometric face normals).
template <typename Real>
inline void transformNormal(const ModelTransform &transform, const Real *source, float *destination)
{
    for (std::size_t row = 0; row < 3; ++row)
    {
        destination[row] = transform.normalMatrix[row][0] * source[0] + transform.normalMatrix[row][1] * source[1]
                         + transform.normalMatrix[row][2] * source[2];
    }
    normalizeVector(destination);
}

// Swaps the last two corners of every triangle, flipping front-face order.
inline void reverseTriangleWinding(std::vector<std::uint32_t> &indices)
{
    for (std::size_t index = 0; index + 2 < indices.size(); index += 3)
    {
        std::swap(indices[index + 1], indices[index + 2]);
    }
}

// Accumulates area-weighted smooth normals over an interleaved vertex/index pair
// (face normals summed per vertex, then normalized). Used by the stl/off/gltf
// paths that keep a shared vertex pool.
inline void accumulateSmoothNormalsInterleaved(
    const std::vector<float> &vertices, const std::vector<std::uint32_t> &indices, std::vector<float> &outNormals)
{
    const std::size_t vertexCount = vertices.size() / renderer::meshVertexFloatCount;
    outNormals.assign(vertexCount * 3, 0.0f);
    for (std::size_t corner = 0; corner + 2 < indices.size(); corner += 3)
    {
        const std::uint32_t indexA = indices[corner + 0];
        const std::uint32_t indexB = indices[corner + 1];
        const std::uint32_t indexC = indices[corner + 2];
        const float        *a      = vertices.data() + static_cast<std::size_t>(indexA) * renderer::meshVertexFloatCount;
        const float        *b      = vertices.data() + static_cast<std::size_t>(indexB) * renderer::meshVertexFloatCount;
        const float        *c      = vertices.data() + static_cast<std::size_t>(indexC) * renderer::meshVertexFloatCount;
        const float         ab[3]  = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        const float         ac[3]  = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
        const float         normal[3]
            = {ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0]};
        for (const std::uint32_t index : {indexA, indexB, indexC})
        {
            const std::size_t offset = static_cast<std::size_t>(index) * 3;
            outNormals[offset + 0] += normal[0];
            outNormals[offset + 1] += normal[1];
            outNormals[offset + 2] += normal[2];
        }
    }
    for (std::size_t offset = 0; offset + 2 < outNormals.size(); offset += 3)
    {
        float normal[3] = {outNormals[offset + 0], outNormals[offset + 1], outNormals[offset + 2]};
        normalizeVector(normal);
        std::copy(normal, normal + 3, outNormals.begin() + offset);
    }
}

// Writes an error code through an optional out-parameter.
inline void reportError(ModelLoadError *error, ModelLoadError value)
{
    if (error != nullptr)
    {
        *error = value;
    }
}

// One importer per format: parse `path` into `record` (parts, materials, bounds),
// applying `options` and the precomputed `transform`. Return false on failure with
// *error set; an empty import is NoGeometry.
bool importObjFile(const std::filesystem::path &path, ModelRecord &record, const ModelLoadOptions &options,
    const ModelTransform &transform, const std::string &defaultMaterialSearchPath, ModelLoadError *error);
bool importStlFile(const std::filesystem::path &path, ModelRecord &record, const ModelLoadOptions &options,
    const ModelTransform &transform, ModelLoadError *error);
bool importPlyFile(const std::filesystem::path &path, ModelRecord &record, const ModelLoadOptions &options,
    const ModelTransform &transform, ModelLoadError *error);
bool importOffFile(const std::filesystem::path &path, ModelRecord &record, const ModelLoadOptions &options,
    const ModelTransform &transform, ModelLoadError *error);
bool importGltfFile(const std::filesystem::path &path, ModelRecord &record, const ModelLoadOptions &options,
    const ModelTransform &transform, ModelLoadError *error);
void optimizeModelRecord(ModelRecord &record, const ModelOptimizationOptions &options);

} // namespace engine::models::detail

#endif
