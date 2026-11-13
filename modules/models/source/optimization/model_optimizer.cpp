// meshoptimizer-backed post-import optimization.
//
// Runs over every part of a freshly imported ModelRecord in a fixed order:
// vertex-cache and overdraw reorder triangles, simplification reduces the
// triangle count, and the vertex-fetch pass reorders vertices along the final
// index order. Triangle reordering and simplification are skipped for parts
// where triangle order matters (alpha-blended, multi-material, or unassigned
// materials); vertex fetch always runs because it preserves triangle order.

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

// True when the part maps to a single fully opaque material so triangle order
// can be freely changed. Empty material arrays and unassigned materials
// (invalidMaterial) are treated as opaque: the renderer draws them in source
// order, but there is no blending to preserve.
bool canReorderTriangles(const ModelPart &part, const ModelRecord &record)
{
    if (!hasSingleTriangleMaterial(part))
    {
        return false;
    }
    if (part.triangleMaterials.empty())
    {
        return true;
    }
    const std::uint32_t materialIndex = part.triangleMaterials.front();
    if (materialIndex == invalidMaterial || materialIndex >= record.materials.size())
    {
        return true;
    }
    const ModelMaterial &material = record.materials[materialIndex].factors;
    return material.alphaMode != 2u && material.opacity >= 1.0f;
}

// Optimizes one part in place. Guards reject malformed parts (bad sizes, out-of-
// range indices) instead of letting the optimizer see them. Triangle reordering
// (cache/overdraw) and simplification only run when canReorderTriangles is true;
// vertex fetch always runs.
void optimizePart(ModelPart &part, const ModelRecord &record, const ModelOptimizationOptions &options)
{
    constexpr std::size_t vertexStride = renderer::meshVertexFloatCount * sizeof(float);
    if (part.indices.size() < 3 || part.indices.size() % 3 != 0 || part.vertices.empty()
        || part.vertices.size() % renderer::meshVertexFloatCount != 0)
    {
        return;
    }

    const std::size_t vertexCount = part.vertices.size() / renderer::meshVertexFloatCount;
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

    const bool reorderable = canReorderTriangles(part, record);
    if (reorderable)
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
        // Simplification reduces the triangle count. meshopt_simplify writes a
        // new index buffer that may reference fewer vertices than the source;
        // the unused vertices are reclaimed by the vertex-fetch pass below.
        // Skipped for tiny meshes (no collapsible edges) and when the target
        // ratio leaves the mesh intact.
        if (options.simplify && part.indices.size() >= 6
            && options.simplifyTargetRatio >= 0.0f && options.simplifyTargetRatio < 1.0f)
        {
            const std::size_t targetIndexCount
                = static_cast<std::size_t>(static_cast<double>(part.indices.size()) * options.simplifyTargetRatio);
            // meshopt_simplify never grows the buffer; allocate the source size
            // and shrink to the returned count.
            std::vector<std::uint32_t> simplified(part.indices.size());
            const unsigned int         flags = options.simplifyLockBorder ? meshopt_SimplifyLockBorder : 0u;
            float                      resultError = 0.0f;
            const std::size_t          simplifiedCount
                = meshopt_simplify(simplified.data(),
                    part.indices.data(),
                    part.indices.size(),
                    part.vertices.data(),
                    vertexCount,
                    vertexStride,
                    targetIndexCount,
                    options.simplifyTargetError,
                    flags,
                    &resultError);
            if (simplifiedCount != 0 && simplifiedCount < part.indices.size())
            {
                simplified.resize(simplifiedCount);
                part.indices = std::move(simplified);
                // Simplification rewrites the triangle set, so the old
                // per-triangle material array no longer lines up. Because
                // simplification only runs on parts that map to a single
                // material (canReorderTriangles), we can safely re-stamp that
                // one material across the new triangle count. Empty arrays
                // (no materials) stay empty.
                if (!part.triangleMaterials.empty())
                {
                    const std::uint32_t materialIndex = part.triangleMaterials.front();
                    part.triangleMaterials.assign(part.indices.size() / 3, materialIndex);
                }
                // Re-run vertex cache on the simplified topology for best
                // cache behaviour; the overdraw pass above ran on the old
                // triangle order and is not re-run (cheap to skip).
                if (options.vertexCache)
                {
                    meshopt_optimizeVertexCache(
                        part.indices.data(), part.indices.data(), part.indices.size(), vertexCount);
                }
            }
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

    std::vector<float>         optimizedVertices(optimizedVertexCount * renderer::meshVertexFloatCount);
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
