// meshoptimizer-backed post-import optimization.
//
// Runs over every part of a freshly imported ModelRecord: vertex-cache and
// overdraw passes reorder triangles, the vertex-fetch pass reorders vertices.
// Triangle reordering is skipped for materials where triangle order matters.

#include "model_import_internal.hpp"

#include <meshoptimizer.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace engine::models::detail
{
namespace
{
// True when the part either has no per-triangle materials or every triangle uses
// the same one.
bool hasSingleTriangleMaterial(const ModelPart &part)
{
    return part.triangleMaterials.empty()
        || std::all_of(part.triangleMaterials.begin(),
            part.triangleMaterials.end(),
            [&part](std::uint32_t material) { return material == part.triangleMaterials.front(); });
}

// Optimizes one part in place. Guards reject malformed parts (bad sizes, out-of-
// range indices) instead of letting the optimizer see them. Triangle reordering
// (cache/overdraw) only runs when the part maps to a single opaque material with
// full opacity; alpha-blended or unassigned parts keep their triangle order.
void optimizePart(ModelPart &part, const ModelRecord &record, const ModelOptimizationOptions &options)
{
    constexpr std::size_t vertexStride = render::meshVertexFloatCount * sizeof(float);
    if (part.indices.size() < 3 || part.indices.size() % 3 != 0 || part.vertices.empty()
        || part.vertices.size() % render::meshVertexFloatCount != 0)
    {
        return;
    }

    const std::size_t vertexCount = part.vertices.size() / render::meshVertexFloatCount;
    if (vertexCount > std::numeric_limits<unsigned int>::max()
        || part.indices.size() > std::numeric_limits<std::size_t>::max() / sizeof(unsigned int))
    {
        return;
    }
    if (std::any_of(part.indices.begin(),
            part.indices.end(),
            [vertexCount](std::uint32_t index) { return index >= vertexCount; }))
    {
        return;
    }
    if (!part.triangleMaterials.empty() && part.triangleMaterials.size() != part.indices.size() / 3)
    {
        return;
    }

    bool canReorderTriangles = hasSingleTriangleMaterial(part);
    if (canReorderTriangles && !part.triangleMaterials.empty())
    {
        const std::uint32_t materialIndex = part.triangleMaterials.front();
        if (materialIndex >= record.materials.size())
        {
            canReorderTriangles = false;
        }
        else
        {
            const ModelMaterial &material = record.materials[materialIndex].factors;
            canReorderTriangles           = material.alphaMode != 2u && material.opacity >= 1.0f;
        }
    }
    if (canReorderTriangles)
    {
        if (options.vertexCache)
        {
            meshopt_optimizeVertexCache(part.indices.data(), part.indices.data(), part.indices.size(), vertexCount);
        }
        if (options.overdraw)
        {
            meshopt_optimizeOverdraw(part.indices.data(),
                part.indices.data(),
                part.indices.size(),
                part.vertices.data(),
                vertexCount,
                vertexStride,
                options.overdrawThreshold);
        }
    }

    // Vertex fetch: reorder vertices along the index buffer. Skipped entirely when
    // the remap turns out to be the identity.
    if (!options.vertexFetch)
    {
        return;
    }

    std::vector<unsigned int> remap(vertexCount);
    const std::size_t         optimizedVertexCount
        = meshopt_optimizeVertexFetchRemap(remap.data(), part.indices.data(), part.indices.size(), vertexCount);
    if (optimizedVertexCount == 0 || optimizedVertexCount > vertexCount)
    {
        return;
    }

    if (optimizedVertexCount == vertexCount)
    {
        bool identityRemap = true;
        for (std::size_t index = 0; index < vertexCount; ++index)
        {
            if (remap[index] != index)
            {
                identityRemap = false;
                break;
            }
        }
        if (identityRemap)
        {
            return;
        }
    }

    std::vector<float>         optimizedVertices(optimizedVertexCount * render::meshVertexFloatCount);
    std::vector<std::uint32_t> optimizedIndices(part.indices.size());
    meshopt_remapVertexBuffer(optimizedVertices.data(), part.vertices.data(), vertexCount, vertexStride, remap.data());
    meshopt_remapIndexBuffer(optimizedIndices.data(), part.indices.data(), part.indices.size(), remap.data());
    part.vertices = std::move(optimizedVertices);
    part.indices  = std::move(optimizedIndices);
}
} // namespace

// Applies the configured passes to every part of the record.
void optimizeModelRecord(ModelRecord &record, const ModelOptimizationOptions &options)
{
    for (ModelPart &part : record.parts)
    {
        optimizePart(part, record, options);
    }
}

} // namespace engine::models::detail
