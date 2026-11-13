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
#include <limits>
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
// (module configuration, not per-load options). The passes run in a fixed order:
// vertex cache -> overdraw -> simplify -> vertex fetch. Simplify is the only pass
// that can change the vertex/index count; the others only reorder in place.
struct ModelOptimizationOptions
{
    bool  vertexCache       = true;  // reorder triangles for post-transform cache hits
    bool  overdraw          = true;  // then reorder again to reduce pixel overdraw
    bool  vertexFetch       = true;  // finally reorder vertices for sequential GPU fetch
    float overdrawThreshold = 1.05f; // allowed overdraw increase for the overdraw pass

    // Triangle-count reduction via meshopt_simplify. Skipped for parts whose
    // triangles do not map to a single fully opaque material (blending order
    // would be destroyed), and skipped for parts with fewer than 6 triangles
    // (meshopt_simplify needs at least one collapsible edge).
    bool  simplify            = false;
    // Target index count as a fraction of the source index count, in 0..1.
    // 0 keeps nothing (meshopt stops at the topology floor); 1 keeps the mesh
    // intact. A typical LOD setting is 0.25..0.5.
    float simplifyTargetRatio = 0.5f;
    // Relative error threshold (in mesh's own scale) accepted during collapses.
    // Smaller values preserve shape better at the cost of fewer reductions; the
    // meshopt default of 1e-2f is a reasonable starting point.
    float simplifyTargetError = 1.0e-2f;
    // When true, passes meshopt_SimplifyLockBorder so boundary vertices (mesh
    // edges, attribute seams) are never collapsed. Improves LOD seam behaviour
    // at the cost of fewer reductions.
    bool  simplifyLockBorder = false;
};

// Everything owned by one loaded model handle: post-transform bounds and
// statistics, the parts in source order, and the materials in source order.
// `haveBounds` tracks whether `minimum`/`maximum` have been seeded by at least
// one vertex; importers set it via `accumulateBounds`.
struct ModelRecord
{
    float minimum[3] = {0.0f, 0.0f, 0.0f};
    float maximum[3] = {0.0f, 0.0f, 0.0f};
    bool  haveBounds = false;

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

// Practical upper bound on vertex/index element counts. Used in place of
// `std::vector<T>().max_size()` (which constructs a temporary vector just to
// read an implementation-defined constant) for overflow guards.
inline constexpr std::size_t maximumVertexElementCount
    = static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max());

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

// Computes an unnormalized triangle cross product from three position vectors.
template <typename Real>
inline void crossProduct3(const Real *a, const Real *b, const Real *c, float *destination) noexcept
{
    const double ab[3] = {static_cast<double>(b[0]) - static_cast<double>(a[0]),
        static_cast<double>(b[1]) - static_cast<double>(a[1]),
        static_cast<double>(b[2]) - static_cast<double>(a[2])};
    const double ac[3] = {static_cast<double>(c[0]) - static_cast<double>(a[0]),
        static_cast<double>(c[1]) - static_cast<double>(a[1]),
        static_cast<double>(c[2]) - static_cast<double>(a[2])};
    destination[0] = static_cast<float>(ab[1] * ac[2] - ab[2] * ac[1]);
    destination[1] = static_cast<float>(ab[2] * ac[0] - ab[0] * ac[2]);
    destination[2] = static_cast<float>(ab[0] * ac[1] - ab[1] * ac[0]);
}

// Computes a unit face normal. Degenerate triangles use the same +Y fallback as
// other generated normals in the import pipeline.
inline void computeFaceNormal(const float *a, const float *b, const float *c, float *destination) noexcept
{
    crossProduct3(a, b, c, destination);
    normalizeVector(destination);
}

// Accumulates area-weighted smooth normals for indexed position data. vertexStride
// and positionOffset describe either a packed position array or an interleaved
// vertex array. Returns false when the source layout or an index is invalid.
inline bool accumulateIndexedSmoothNormals(const std::vector<float> &vertices,
    const std::vector<std::uint32_t> &indices, std::vector<float> &outNormals,
    std::size_t vertexStride = renderer::meshVertexFloatCount, std::size_t positionOffset = 0)
{
    outNormals.clear();
    if (positionOffset > vertexStride || vertexStride - positionOffset < 3 || vertices.size() % vertexStride != 0
        || indices.size() % 3 != 0)
    {
        return false;
    }

    const std::size_t vertexCount = vertices.size() / vertexStride;
    if (vertexCount > maximumVertexElementCount / 3)
    {
        return false;
    }
    for (const std::uint32_t index : indices)
    {
        if (index >= vertexCount)
        {
            return false;
        }
    }

    outNormals.assign(vertexCount * 3, 0.0f);
    for (std::size_t triangle = 0; triangle < indices.size(); triangle += 3)
    {
        const std::uint32_t indexA = indices[triangle + 0];
        const std::uint32_t indexB = indices[triangle + 1];
        const std::uint32_t indexC = indices[triangle + 2];
        const float *a = vertices.data() + static_cast<std::size_t>(indexA) * vertexStride + positionOffset;
        const float *b = vertices.data() + static_cast<std::size_t>(indexB) * vertexStride + positionOffset;
        const float *c = vertices.data() + static_cast<std::size_t>(indexC) * vertexStride + positionOffset;
        float normal[3]{};
        crossProduct3(a, b, c, normal);
        for (const std::uint32_t index : {indexA, indexB, indexC})
        {
            float *sum = outNormals.data() + static_cast<std::size_t>(index) * 3;
            sum[0] += normal[0];
            sum[1] += normal[1];
            sum[2] += normal[2];
        }
    }
    for (std::size_t vertex = 0; vertex < vertexCount; ++vertex)
    {
        normalizeVector(outNormals.data() + vertex * 3);
    }
    return true;
}

// Expands an indexed part to one vertex per triangle corner and writes a flat
// face normal to each expanded vertex. Triangle-material order is unchanged.
inline bool expandFlatShading(ModelPart &part)
{
    constexpr std::size_t vertexStride = renderer::meshVertexFloatCount;
    const std::size_t sourceVertexCount = part.vertices.size() / vertexStride;
    if (part.vertices.size() % vertexStride != 0 || part.indices.size() % 3 != 0
        || static_cast<std::uint64_t>(part.indices.size()) > static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()) + 1ull
        || part.indices.size() > maximumVertexElementCount / vertexStride)
    {
        return false;
    }
    for (const std::uint32_t index : part.indices)
    {
        if (index >= sourceVertexCount)
        {
            return false;
        }
    }

    std::vector<float> flatVertices(part.indices.size() * vertexStride);
    for (std::size_t triangle = 0; triangle < part.indices.size(); triangle += 3)
    {
        const std::uint32_t indexA = part.indices[triangle + 0];
        const std::uint32_t indexB = part.indices[triangle + 1];
        const std::uint32_t indexC = part.indices[triangle + 2];
        const float *a = part.vertices.data() + static_cast<std::size_t>(indexA) * vertexStride;
        const float *b = part.vertices.data() + static_cast<std::size_t>(indexB) * vertexStride;
        const float *c = part.vertices.data() + static_cast<std::size_t>(indexC) * vertexStride;
        float normal[3]{};
        computeFaceNormal(a, b, c, normal);
        for (std::size_t corner = 0; corner < 3; ++corner)
        {
            const std::uint32_t sourceIndex = part.indices[triangle + corner];
            const float *source = part.vertices.data() + static_cast<std::size_t>(sourceIndex) * vertexStride;
            float *destination = flatVertices.data() + (triangle + corner) * vertexStride;
            std::copy_n(source, vertexStride, destination);
            std::copy_n(normal, 3, destination + 3);
        }
    }

    part.vertices = std::move(flatVertices);
    for (std::size_t index = 0; index < part.indices.size(); ++index)
    {
        part.indices[index] = static_cast<std::uint32_t>(index);
    }
    return true;
}

// Writes an error code through an optional out-parameter.
inline void reportError(ModelLoadError *error, ModelLoadError value)
{
    if (error != nullptr)
    {
        *error = value;
    }
}

// Folds -0.0f into +0.0f so welding keys treat signed zeros identically.
inline float canonicalizeZero(float value) noexcept
{
    return value == 0.0f ? 0.0f : value;
}

// Accumulates an axis-aligned bounds box from the interleaved vertex stream of
// `part` into `record.minimum`/`record.maximum`, threading the seeded/not-seeded
// state through `record.haveBounds`. Safe to call on empty parts (no-op). All
// importers use this so bounds handling is identical across formats.
inline void accumulateBounds(const ModelPart &part, ModelRecord &record) noexcept
{
    constexpr std::size_t vertexStride = renderer::meshVertexFloatCount;
    if (part.vertices.empty() || part.vertices.size() % vertexStride != 0)
    {
        return;
    }
    for (std::size_t offset = 0; offset + 2 < part.vertices.size(); offset += vertexStride)
    {
        const float x = part.vertices[offset + 0];
        const float y = part.vertices[offset + 1];
        const float z = part.vertices[offset + 2];
        if (!record.haveBounds)
        {
            record.minimum[0] = record.maximum[0] = x;
            record.minimum[1] = record.maximum[1] = y;
            record.minimum[2] = record.maximum[2] = z;
            record.haveBounds                    = true;
        }
        else
        {
            record.minimum[0] = std::min(record.minimum[0], x);
            record.minimum[1] = std::min(record.minimum[1], y);
            record.minimum[2] = std::min(record.minimum[2], z);
            record.maximum[0] = std::max(record.maximum[0], x);
            record.maximum[1] = std::max(record.maximum[1], y);
            record.maximum[2] = std::max(record.maximum[2], z);
        }
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
