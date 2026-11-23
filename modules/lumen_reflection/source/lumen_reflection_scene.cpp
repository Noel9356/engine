// lumen_reflection scene side: the surface registry's geometry storage and
// the ray-tracing core (per-surface local-space BVHs under a top-level BVH),
// plus the lumen_gi probe-grid copy and its irradiance/radiance lookups.
//
// The tracing structure deliberately mirrors lumen_gi's scene module: rays
// transform into each surface's local space so a moving instance never
// rebuilds its surface BVH, and an exact-identity transform skips the
// local<->world arithmetic on the hot path.

#include "lumen_reflection_internal.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <vector>

namespace engine
{
namespace lumen_reflection
{

namespace
{

// safeInverseDirection, slabTest, and the Moller-Trumbore kernel come from
// engine/renderer/lumen_ray_core.hpp (shared with lumen_gi).

constexpr std::uint32_t traversalStackSize = 96;

// Attribute strides must hold their three floats and stay sane (an interleaved
// vertex layout has no reason to jump pages between attributes); a smaller
// stride would read past the attribute the index addresses.
constexpr std::uint32_t minimumAttributeStrideBytes = 3 * sizeof(float);
constexpr std::uint32_t maximumAttributeStrideBytes = 4096;
// Registry descriptions are module-internal API input, not trusted data: a
// vertex count above this is rejected before any allocation is sized from it.
constexpr std::uint32_t maximumDescriptionVertices = 1u << 24;

// Traversal of one surface's local-space BVH; ray.tMax shrinks to the
// closest hit.
bool traverseSurface(const RfSurfaceGeometry &geometry, RfRay &ray, const glm::vec3 &inverseDirection,
    std::uint32_t &outTriangle, glm::vec3 &outBarycentric)
{
    if (geometry.bvh.empty())
    {
        return false;
    }
    float entry = 0.0f;
    if (!slabTest(ray.origin, inverseDirection, geometry.boundsMin, geometry.boundsMax, ray.tMax, entry))
    {
        return false;
    }
    std::array<std::uint32_t, traversalStackSize> stack;
    std::uint32_t                                 stackSize = 0;
    stack[stackSize++]                                      = 0;
    bool found                                              = false;
    while (stackSize > 0)
    {
        const RfSurfaceGeometry::BvhNode &node = geometry.bvh[stack[--stackSize]];
        if (node.count > 0)
        {
            const RfPackedTriangle *triangles = geometry.packed.data() + node.leftOrFirst;
            for (std::uint32_t local = 0; local < node.count; ++local)
            {
                float     t = 0.0f;
                glm::vec3 barycentric{};
                if (rayTriangle(ray, triangles[local], t, barycentric))
                {
                    ray.tMax       = t;
                    outTriangle    = triangles[local].triangle;
                    outBarycentric = barycentric;
                    found          = true;
                }
            }
            continue;
        }
        const RfSurfaceGeometry::BvhNode &left  = geometry.bvh[node.leftOrFirst];
        const RfSurfaceGeometry::BvhNode &right = geometry.bvh[node.leftOrFirst + 1];
        float                             leftNear  = 0.0f;
        float                             rightNear = 0.0f;
        const bool hitLeft  = slabTest(ray.origin, inverseDirection, left.boundsMin, left.boundsMax, ray.tMax, leftNear);
        const bool hitRight = slabTest(ray.origin, inverseDirection, right.boundsMin, right.boundsMax, ray.tMax, rightNear);
        if (hitLeft && hitRight)
        {
            // Push the farther child first so the nearer one pops next. The
            // pushes are guarded: a median-split BVH over N triangles is about
            // log2(N/4) deep, so the stack cannot fill in practice, but a
            // degenerate split must never write past it either.
            if (leftNear <= rightNear)
            {
                if (stackSize < traversalStackSize)
                {
                    stack[stackSize++] = node.leftOrFirst + 1;
                }
                if (stackSize < traversalStackSize)
                {
                    stack[stackSize++] = node.leftOrFirst;
                }
            }
            else
            {
                if (stackSize < traversalStackSize)
                {
                    stack[stackSize++] = node.leftOrFirst;
                }
                if (stackSize < traversalStackSize)
                {
                    stack[stackSize++] = node.leftOrFirst + 1;
                }
            }
        }
        else if (hitLeft)
        {
            if (stackSize < traversalStackSize)
            {
                stack[stackSize++] = node.leftOrFirst;
            }
        }
        else if (hitRight)
        {
            if (stackSize < traversalStackSize)
            {
                stack[stackSize++] = node.leftOrFirst + 1;
            }
        }
    }
    return found;
}

struct SurfaceHit
{
    bool          hit          = false;
    float         t            = 0.0f;
    std::uint32_t surfaceIndex = noSurface;
    std::uint32_t triangleIndex = 0;
    glm::vec3     barycentric{0.0f};
};

// Traces one world ray through the top-level BVH and the surface BVHs.
SurfaceHit traceSurfaces(const RfPipelineState &state, const RfRay &worldRay)
{
    SurfaceHit result;
    if (state.tlas.empty())
    {
        return result;
    }
    const glm::vec3 inverseDirection = safeInverseDirection(worldRay.direction);
    float           bestT            = worldRay.tMax;
    float           entry            = 0.0f;
    const RfTlasNode &root            = state.tlas[0];
    if (!slabTest(worldRay.origin, inverseDirection, root.boundsMin, root.boundsMax, bestT, entry))
    {
        return result;
    }
    std::array<std::uint32_t, traversalStackSize> stack;
    std::uint32_t                                 stackSize = 0;
    stack[stackSize++]                                      = 0;
    while (stackSize > 0)
    {
        const RfTlasNode &node = state.tlas[stack[--stackSize]];
        if (node.count > 0)
        {
            for (std::uint32_t local = 0; local < node.count; ++local)
            {
                const std::uint32_t surfaceIndex = state.tlasSurfaces[node.leftOrFirst + local];
                const RfSurface   &surface = state.surfaces[surfaceIndex];
                float worldEntry = 0.0f;
                if (!slabTest(worldRay.origin, inverseDirection, surface.worldMin, surface.worldMax, bestT, worldEntry))
                {
                    continue;
                }
                glm::vec3 localOrigin;
                glm::vec3 localDirection;
                float     directionLength = 1.0f;
                if (surface.localIsWorld)
                {
                    localOrigin    = worldRay.origin;
                    localDirection = worldRay.direction;
                }
                else
                {
                    localOrigin    = surface.inverseLinear * worldRay.origin + surface.inverseTranslation;
                    localDirection = surface.inverseLinear * worldRay.direction;
                    directionLength = glm::length(localDirection);
                    if (directionLength < 1.0e-8f)
                    {
                        continue;
                    }
                }
                RfRay localRay{localOrigin,
                    surface.localIsWorld ? localDirection : localDirection / directionLength,
                    bestT * directionLength};
                std::uint32_t triangle = 0;
                glm::vec3     barycentric{};
                if (traverseSurface(surface.geometry, localRay,
                        surface.localIsWorld ? inverseDirection : safeInverseDirection(localRay.direction), triangle,
                        barycentric))
                {
                    result.hit          = true;
                    result.surfaceIndex = surfaceIndex;
                    bestT                = localRay.tMax / directionLength;
                    result.t             = bestT;
                    result.triangleIndex = triangle;
                    result.barycentric    = barycentric;
                }
            }
            continue;
        }
        const RfTlasNode &left  = state.tlas[node.leftOrFirst];
        const RfTlasNode &right = state.tlas[node.leftOrFirst + 1];
        float             leftNear  = 0.0f;
        float             rightNear = 0.0f;
        const bool hitLeft  = slabTest(worldRay.origin, inverseDirection, left.boundsMin, left.boundsMax, bestT, leftNear);
        const bool hitRight = slabTest(worldRay.origin, inverseDirection, right.boundsMin, right.boundsMax, bestT, rightNear);
        if (hitLeft && hitRight)
        {
            if (leftNear <= rightNear)
            {
                if (stackSize < traversalStackSize)
                {
                    stack[stackSize++] = node.leftOrFirst + 1;
                }
                if (stackSize < traversalStackSize)
                {
                    stack[stackSize++] = node.leftOrFirst;
                }
            }
            else
            {
                if (stackSize < traversalStackSize)
                {
                    stack[stackSize++] = node.leftOrFirst;
                }
                if (stackSize < traversalStackSize)
                {
                    stack[stackSize++] = node.leftOrFirst + 1;
                }
            }
        }
        else if (hitLeft)
        {
            if (stackSize < traversalStackSize)
            {
                stack[stackSize++] = node.leftOrFirst;
            }
        }
        else if (hitRight)
        {
            if (stackSize < traversalStackSize)
            {
                stack[stackSize++] = node.leftOrFirst + 1;
            }
        }
    }
    return result;
}

} // namespace

// ---- public trace entry point -------------------------------------------------------

RfHit rfTraceScene(const RfPipelineState &state, const RfRay &ray)
{
    RfHit result;
    const SurfaceHit surfaceHit = traceSurfaces(state, ray);
    result.hit          = surfaceHit.hit;
    result.t           = surfaceHit.t;
    result.surfaceIndex = surfaceHit.surfaceIndex;
    if (!surfaceHit.hit)
    {
        return result;
    }
    const RfSurface         &surface  = state.surfaces[surfaceHit.surfaceIndex];
    const RfSurfaceGeometry &geometry = surface.geometry;
    const std::uint32_t i0 = geometry.indices[surfaceHit.triangleIndex * 3 + 0];
    const std::uint32_t i1 = geometry.indices[surfaceHit.triangleIndex * 3 + 1];
    const std::uint32_t i2 = geometry.indices[surfaceHit.triangleIndex * 3 + 2];
    const glm::vec3    &b  = surfaceHit.barycentric;

    glm::vec3 localPosition = geometry.positions[i0] * b.x + geometry.positions[i1] * b.y
                            + geometry.positions[i2] * b.z;
    glm::vec3 localNormal = geometry.normals[i0] * b.x + geometry.normals[i1] * b.y
                           + geometry.normals[i2] * b.z;
    const glm::vec3 localAlbedo = geometry.colors[i0] * b.x + geometry.colors[i1] * b.y
                                 + geometry.colors[i2] * b.z;
    const glm::vec3 geometric = glm::cross(
        geometry.positions[i1] - geometry.positions[i0], geometry.positions[i2] - geometry.positions[i0]);
    const float geometricLength = glm::length(geometric);
    const float smoothLength = glm::length(localNormal);
    if (smoothLength < 1.0e-8f)
    {
        localNormal = geometricLength > 1.0e-12f ? geometric / geometricLength : glm::vec3(0.0f, 1.0f, 0.0f);
    }
    else
    {
        localNormal /= smoothLength;
    }

    result.position = glm::vec3(surface.transform * glm::vec4(localPosition, 1.0f));
    // Normals transform through the inverse transpose: the plain linear
    // matrix skews them under non-uniform scale.
    result.normal   = surface.localIsWorld ? localNormal : glm::normalize(surface.normalMatrix * localNormal);
    result.albedo   = surface.albedo * localAlbedo;
    // Reflection rays start on surfaces and legitimately strike the back of
    // thin geometry (the inside of the room's outer shell, seen from outside
    // through a miss). Report a backface only when BOTH the smooth and the
    // geometric normal face away, mirroring lumen_gi's anti-leak rule.
    const glm::vec3 worldGeometric = geometricLength > 1.0e-12f
        ? (surface.localIsWorld ? geometric / geometricLength
                                : glm::normalize(surface.normalMatrix * geometric))
        : result.normal;
    const bool smoothFaces  = glm::dot(result.normal, ray.direction) <= 0.0f;
    const bool geometricFaces = glm::dot(worldGeometric, ray.direction) <= 0.0f;
    result.backface = !smoothFaces && !geometricFaces;
    if (!smoothFaces && geometricFaces)
    {
        // Smooth normal disagrees with the winding: trust the smooth one for
        // shading but keep the reported normal facing the ray so the caller's
        // hemisphere logic stays well defined.
        result.normal = -result.normal;
    }
    return result;
}

// ---- geometry build -----------------------------------------------------------------

void rfBuildSurfaceGeometry(RfSurfaceGeometry &geometry, const ReflectionSurfaceDescription &description,
    std::uint64_t maxTriangles, ReflectionError *error)
{
    auto fail = [&](ReflectionError value)
    {
        if (error != nullptr)
        {
            *error = value;
        }
    };
    if (description.positions == nullptr || description.indices == nullptr || description.vertexCount == 0
        || description.indexCount < 3 || description.indexCount % 3 != 0)
    {
        fail(ReflectionError::InvalidDescription);
        return;
    }
    const std::uint64_t triangleCount = description.indexCount / 3;
    if (triangleCount > maxTriangles)
    {
        fail(ReflectionError::InvalidDescription);
        return;
    }
    if (description.positionStrideBytes % sizeof(float) != 0
        || (description.positionStrideBytes != 0
            && (description.positionStrideBytes < minimumAttributeStrideBytes
                || description.positionStrideBytes > maximumAttributeStrideBytes))
        || description.vertexCount > maximumDescriptionVertices)
    {
        fail(ReflectionError::InvalidDescription);
        return;
    }
    const std::size_t positionStride = description.positionStrideBytes != 0
                                         ? description.positionStrideBytes / sizeof(float)
                                         : 3;
    const bool hasNormals = description.normals != nullptr && description.normalStrideBytes != 0;
    const bool hasColors  = description.colors != nullptr && description.colorStrideBytes != 0;
    if (hasNormals && (description.normalStrideBytes % sizeof(float) != 0
            || description.normalStrideBytes < minimumAttributeStrideBytes
            || description.normalStrideBytes > maximumAttributeStrideBytes))
    {
        fail(ReflectionError::InvalidDescription);
        return;
    }
    if (hasColors && (description.colorStrideBytes % sizeof(float) != 0
            || description.colorStrideBytes < minimumAttributeStrideBytes
            || description.colorStrideBytes > maximumAttributeStrideBytes))
    {
        fail(ReflectionError::InvalidDescription);
        return;
    }
    for (std::uint32_t index = 0; index < description.indexCount; ++index)
    {
        if (description.indices[index] >= description.vertexCount)
        {
            fail(ReflectionError::InvalidDescription);
            return;
        }
    }
    for (std::uint32_t vertex = 0; vertex < description.vertexCount; ++vertex)
    {
        const float *position = description.positions + static_cast<std::size_t>(vertex) * positionStride;
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            if (!std::isfinite(position[axis]))
            {
                fail(ReflectionError::InvalidDescription);
                return;
            }
        }
    }
    const std::size_t normalStride = description.normalStrideBytes != 0 ? description.normalStrideBytes / sizeof(float) : 3;
    const std::size_t colorStride  = description.colorStrideBytes != 0 ? description.colorStrideBytes / sizeof(float) : 3;
    if (hasNormals)
    {
        for (std::uint32_t vertex = 0; vertex < description.vertexCount; ++vertex)
        {
            const float *source = description.normals + static_cast<std::size_t>(vertex) * normalStride;
            if (!std::isfinite(source[0]) || !std::isfinite(source[1]) || !std::isfinite(source[2]))
            {
                fail(ReflectionError::InvalidDescription);
                return;
            }
        }
    }
    if (hasColors)
    {
        for (std::uint32_t vertex = 0; vertex < description.vertexCount; ++vertex)
        {
            const float *source = description.colors + static_cast<std::size_t>(vertex) * colorStride;
            if (!std::isfinite(source[0]) || !std::isfinite(source[1]) || !std::isfinite(source[2]))
            {
                fail(ReflectionError::InvalidDescription);
                return;
            }
        }
    }

    geometry.positions.resize(description.vertexCount);
    for (std::uint32_t vertex = 0; vertex < description.vertexCount; ++vertex)
    {
        const float *source = description.positions + static_cast<std::size_t>(vertex) * positionStride;
        geometry.positions[vertex] = glm::vec3(source[0], source[1], source[2]);
    }
    geometry.indices.assign(description.indices, description.indices + description.indexCount);

    if (hasNormals)
    {
        geometry.normals.resize(description.vertexCount);
        for (std::uint32_t vertex = 0; vertex < description.vertexCount; ++vertex)
        {
            const float *source = description.normals + static_cast<std::size_t>(vertex) * normalStride;
            glm::vec3    normal(source[0], source[1], source[2]);
            const float  length = glm::length(normal);
            geometry.normals[vertex] = length > 1.0e-8f ? normal / length : glm::vec3(0.0f, 1.0f, 0.0f);
        }
    }
    else
    {
        // Smooth normals from triangle geometry (the renderer's
        // prepareMeshData does the same for missing normals).
        geometry.normals.assign(description.vertexCount, glm::vec3(0.0f));
        for (std::uint32_t triangle = 0; triangle < triangleCount; ++triangle)
        {
            const std::uint32_t i0 = geometry.indices[triangle * 3 + 0];
            const std::uint32_t i1 = geometry.indices[triangle * 3 + 1];
            const std::uint32_t i2 = geometry.indices[triangle * 3 + 2];
            const glm::vec3    faceNormal = glm::cross(
                geometry.positions[i1] - geometry.positions[i0], geometry.positions[i2] - geometry.positions[i0]);
            geometry.normals[i0] += faceNormal;
            geometry.normals[i1] += faceNormal;
            geometry.normals[i2] += faceNormal;
        }
        for (glm::vec3 &normal : geometry.normals)
        {
            const float length = glm::length(normal);
            normal = length > 1.0e-8f ? normal / length : glm::vec3(0.0f, 1.0f, 0.0f);
        }
    }

    geometry.colors.assign(description.vertexCount, glm::vec3(1.0f));
    if (hasColors)
    {
        for (std::uint32_t vertex = 0; vertex < description.vertexCount; ++vertex)
        {
            const float *source = description.colors + static_cast<std::size_t>(vertex) * colorStride;
            geometry.colors[vertex] = glm::clamp(glm::vec3(source[0], source[1], source[2]),
                glm::vec3(0.0f), glm::vec3(1.0e6f));
        }
    }
}

void rfBuildSurfaceBvh(RfSurfaceGeometry &geometry)
{
    const std::size_t triangleCount = geometry.indices.size() / 3;
    geometry.bvh.clear();
    geometry.packed.clear();
    if (triangleCount == 0)
    {
        geometry.boundsMin = geometry.boundsMax = glm::vec3(0.0f);
        return;
    }
    geometry.packed.resize(triangleCount);
    for (std::uint32_t triangle = 0; triangle < triangleCount; ++triangle)
    {
        const std::uint32_t i0 = geometry.indices[triangle * 3 + 0];
        const std::uint32_t i1 = geometry.indices[triangle * 3 + 1];
        const std::uint32_t i2 = geometry.indices[triangle * 3 + 2];
        RfPackedTriangle &packed = geometry.packed[triangle];
        packed.v0      = geometry.positions[i0];
        packed.edge1   = geometry.positions[i1] - geometry.positions[i0];
        packed.edge2   = geometry.positions[i2] - geometry.positions[i0];
        packed.triangle = triangle;
    }

    struct BuildEntry
    {
        std::uint32_t first;
        std::uint32_t count;
        std::uint32_t node;
    };

    // Node bounds over full triangle extents with median splits on the
    // widest centroid axis, matching lumen_gi's build shape: enough for the
    // few hundred triangles a reflection surface carries.
    geometry.bvh.assign(triangleCount * 2, RfSurfaceGeometry::BvhNode{});
    std::uint32_t        nodeCount = 0; // highest allocated node index
    std::vector<BuildEntry> stack;
    stack.push_back({0u, static_cast<std::uint32_t>(triangleCount), 0u});
    while (!stack.empty())
    {
        const BuildEntry entry = stack.back();
        stack.pop_back();
        const std::uint32_t first = entry.first;
        const std::uint32_t count = entry.count;
        const std::uint32_t node  = entry.node;

        // Bounds of this subtree's triangles (full extent, not centroids).
        glm::vec3 boundsMin(std::numeric_limits<float>::max());
        glm::vec3 boundsMax(-std::numeric_limits<float>::max());
        for (std::uint32_t local = 0; local < count; ++local)
        {
            const RfPackedTriangle &packed = geometry.packed[first + local];
            boundsMin = glm::min(boundsMin, glm::min(packed.v0, glm::min(packed.v0 + packed.edge1, packed.v0 + packed.edge2)));
            boundsMax = glm::max(boundsMax, glm::max(packed.v0, glm::max(packed.v0 + packed.edge1, packed.v0 + packed.edge2)));
        }
        if (boundsMin.x > boundsMax.x || boundsMin.y > boundsMax.y || boundsMin.z > boundsMax.z)
        {
            boundsMin = boundsMax = glm::vec3(0.0f);
        }
        geometry.bvh[node].boundsMin = boundsMin;
        geometry.bvh[node].boundsMax = boundsMax;

        if (count <= 4)
        {
            geometry.bvh[node].count       = count;
            geometry.bvh[node].leftOrFirst = first;
            continue;
        }

        // Widest centroid axis median split; the subtree's triangles are
        // reordered in place so both children own contiguous packed ranges.
        glm::vec3 centroidMin(std::numeric_limits<float>::max());
        glm::vec3 centroidMax(-std::numeric_limits<float>::max());
        for (std::uint32_t local = 0; local < count; ++local)
        {
            const glm::vec3 centroid = geometry.packed[first + local].v0
                + (geometry.packed[first + local].edge1 + geometry.packed[first + local].edge2) * (1.0f / 3.0f);
            centroidMin = glm::min(centroidMin, centroid);
            centroidMax = glm::max(centroidMax, centroid);
        }
        const glm::vec3 extent = centroidMax - centroidMin;
        std::uint32_t axis = 0;
        if (extent.y > extent.x && extent.y >= extent.z)
        {
            axis = 1;
        }
        else if (extent.z > extent.x && extent.z > extent.y)
        {
            axis = 2;
        }
        std::sort(geometry.packed.begin() + first, geometry.packed.begin() + first + count,
            [&](const RfPackedTriangle &l, const RfPackedTriangle &r)
            {
                const float lc = l.v0[axis] + (l.edge1[axis] + l.edge2[axis]) / 3.0f;
                const float rc = r.v0[axis] + (r.edge1[axis] + r.edge2[axis]) / 3.0f;
                return lc < rc;
            });
        const std::uint32_t split     = count / 2;
        const std::uint32_t childBase = nodeCount + 1;
        geometry.bvh[node].count       = 0;
        geometry.bvh[node].leftOrFirst = childBase;
        nodeCount                      = childBase + 1;
        // Left child pops first (LIFO) so nearer-side traversal order holds.
        stack.push_back({first + split, count - split, childBase + 1});
        stack.push_back({first, split, childBase});
    }
    geometry.bvh.resize(nodeCount + 1);

    geometry.boundsMin = glm::vec3(std::numeric_limits<float>::max());
    geometry.boundsMax = glm::vec3(-std::numeric_limits<float>::max());
    for (const RfPackedTriangle &packed : geometry.packed)
    {
        geometry.boundsMin = glm::min(geometry.boundsMin, glm::min(packed.v0, glm::min(packed.v0 + packed.edge1, packed.v0 + packed.edge2)));
        geometry.boundsMax = glm::max(geometry.boundsMax, glm::max(packed.v0, glm::max(packed.v0 + packed.edge1, packed.v0 + packed.edge2)));
    }
    if (geometry.boundsMin.x > geometry.boundsMax.x || geometry.boundsMin.y > geometry.boundsMax.y
        || geometry.boundsMin.z > geometry.boundsMax.z)
    {
        geometry.boundsMin = geometry.boundsMax = glm::vec3(0.0f);
    }
}

void rfUpdateSurfaceWorldBounds(RfSurface &surface)
{
    const glm::vec3 localCenter = (surface.geometry.boundsMin + surface.geometry.boundsMax) * 0.5f;
    const glm::vec3 localHalf   = (surface.geometry.boundsMax - surface.geometry.boundsMin) * 0.5f;
    glm::vec3       worldMin(std::numeric_limits<float>::max());
    glm::vec3       worldMax(-std::numeric_limits<float>::max());
    for (std::uint32_t corner = 0; corner < 8; ++corner)
    {
        const glm::vec3 local(localCenter.x + ((corner & 1u) ? localHalf.x : -localHalf.x),
            localCenter.y + ((corner & 2u) ? localHalf.y : -localHalf.y),
            localCenter.z + ((corner & 4u) ? localHalf.z : -localHalf.z));
        const glm::vec3 world = glm::vec3(surface.transform * glm::vec4(local, 1.0f));
        worldMin = glm::min(worldMin, world);
        worldMax = glm::max(worldMax, world);
    }
    if (worldMin.x > worldMax.x)
    {
        worldMin = worldMax = glm::vec3(0.0f);
    }
    surface.worldMin = worldMin;
    surface.worldMax = worldMax;
}

bool rfUpdateSurfaceTransform(RfSurface &surface, const glm::mat4 &transform)
{
    // The derived matrices are validated before anything is committed: a
    // singular transform produces non-finite values, and NaN bounds make every
    // slab test fail closed, silently removing the surface from all rays. The
    // caller keeps the previous transform instead (see the header contract).
    const glm::mat4 inverse       = glm::inverse(transform);
    const glm::mat3 normalMatrix  = glm::inverseTranspose(glm::mat3(transform));
    const auto      allFinite    = [](const float *elements, std::uint32_t count)
    {
        for (std::uint32_t element = 0; element < count; ++element)
        {
            if (!std::isfinite(elements[element]))
            {
                return false;
            }
        }
        return true;
    };
    if (!allFinite(glm::value_ptr(transform), 16u) || !allFinite(glm::value_ptr(inverse), 16u)
        || !allFinite(glm::value_ptr(normalMatrix), 9u))
    {
        return false;
    }
    surface.transform       = transform;
    surface.inverseTransform = inverse;
    surface.inverseLinear   = glm::mat3(inverse);
    surface.inverseTranslation = glm::vec3(inverse[3]);
    surface.normalMatrix    = normalMatrix;
    surface.localIsWorld    = transform == glm::mat4(1.0f);
    rfUpdateSurfaceWorldBounds(surface);
    // The raster caches depend on the transform: the projected triangles and
    // the world-space vertex positions rebuild on the next raster pass.
    surface.rasterBuiltSerial = 0;
    surface.rasterWorldDirty  = true;
    return true;
}

void rfEnsureSceneTlas(RfPipelineState &state)
{
    if (!state.tlasDirty)
    {
        return;
    }
    state.tlas.clear();
    state.tlasSurfaces.clear();
    for (std::size_t index = 0; index < state.surfaces.size(); ++index)
    {
        if (!state.surfaces[index].excluded)
        {
            state.tlasSurfaces.push_back(static_cast<std::uint32_t>(index));
        }
    }
    if (state.tlasSurfaces.empty())
    {
        state.tlasDirty = false;
        return;
    }

    // A binary BVH over the surface world bounds, median-split on the widest
    // centroid axis with leaves of up to four surfaces - the same shape as
    // the per-surface builds. A ray then slab-tests only the surface cells
    // its sweep actually intersects, however far the registry grows, instead
    // of scanning every surface (the flat leaf list the module started with).
    struct BuildEntry
    {
        std::uint32_t first;
        std::uint32_t count;
        std::uint32_t node;
    };
    const std::uint32_t surfaceCount = static_cast<std::uint32_t>(state.tlasSurfaces.size());
    state.tlas.assign(static_cast<std::size_t>(surfaceCount) * 2, RfTlasNode{});
    std::uint32_t            nodeCount = 0;
    std::vector<BuildEntry> stack;
    stack.push_back({0u, surfaceCount, 0u});
    while (!stack.empty())
    {
        const BuildEntry entry = stack.back();
        stack.pop_back();
        const std::uint32_t first = entry.first;
        const std::uint32_t count = entry.count;
        const std::uint32_t node  = entry.node;

        glm::vec3 boundsMin(std::numeric_limits<float>::max());
        glm::vec3 boundsMax(-std::numeric_limits<float>::max());
        for (std::uint32_t local = 0; local < count; ++local)
        {
            const RfSurface &surface = state.surfaces[state.tlasSurfaces[first + local]];
            boundsMin = glm::min(boundsMin, surface.worldMin);
            boundsMax = glm::max(boundsMax, surface.worldMax);
        }
        if (boundsMin.x > boundsMax.x || boundsMin.y > boundsMax.y || boundsMin.z > boundsMax.z)
        {
            boundsMin = boundsMax = glm::vec3(0.0f);
        }
        state.tlas[node].boundsMin = boundsMin;
        state.tlas[node].boundsMax = boundsMax;

        if (count <= 4)
        {
            state.tlas[node].count       = count;
            state.tlas[node].leftOrFirst = first;
            continue;
        }

        glm::vec3 centroidMin(std::numeric_limits<float>::max());
        glm::vec3 centroidMax(-std::numeric_limits<float>::max());
        for (std::uint32_t local = 0; local < count; ++local)
        {
            const RfSurface &surface = state.surfaces[state.tlasSurfaces[first + local]];
            const glm::vec3 centroid = (surface.worldMin + surface.worldMax) * 0.5f;
            centroidMin = glm::min(centroidMin, centroid);
            centroidMax = glm::max(centroidMax, centroid);
        }
        const glm::vec3 extent = centroidMax - centroidMin;
        std::uint32_t axis = 0;
        if (extent.y > extent.x && extent.y >= extent.z)
        {
            axis = 1;
        }
        else if (extent.z > extent.x && extent.z > extent.y)
        {
            axis = 2;
        }
        std::sort(state.tlasSurfaces.begin() + first, state.tlasSurfaces.begin() + first + count,
            [&](std::uint32_t left, std::uint32_t right)
            {
                const glm::vec3 leftCentroid = (state.surfaces[left].worldMin + state.surfaces[left].worldMax) * 0.5f;
                const glm::vec3 rightCentroid = (state.surfaces[right].worldMin + state.surfaces[right].worldMax) * 0.5f;
                return leftCentroid[axis] < rightCentroid[axis];
            });
        const std::uint32_t split     = count / 2;
        const std::uint32_t childBase = nodeCount + 1;
        state.tlas[node].count       = 0;
        state.tlas[node].leftOrFirst = childBase;
        nodeCount                    = childBase + 1;
        // Left child pops first (LIFO) so the nearer-side traversal holds.
        stack.push_back({first + split, count - split, childBase + 1});
        stack.push_back({first, split, childBase});
    }
    state.tlas.resize(nodeCount + 1);
    state.tlasDirty = false;
}

// ---- probe grid copy + lookups ------------------------------------------------------

bool rfRefreshProbeGrid(RfPipelineState &state, const lumen_gi::LumenGiCapability *capability)
{
    if (capability == nullptr || capability->copyProbes == nullptr)
    {
        return false;
    }
    std::uint64_t version = 0;
    lumen_gi::GiProbeHeader header{};
    capability->copyProbes(capability->self, nullptr, 0, &header, &version);
    if (state.probeGrid.valid() && version == state.probeGrid.version)
    {
        return false; // unchanged copy
    }
    RfProbeGrid grid;
    grid.header = header;
    if (header.probeCount != 0)
    {
        grid.probes.resize(header.probeCount);
        const std::uint32_t copied = capability->copyProbes(capability->self,
            grid.probes.data(), header.probeCount, &grid.header, &version);
        grid.probes.resize(std::min<std::uint32_t>(copied, header.probeCount));
    }
    grid.version = version;

    // Level descriptors (renderer's buildProbeLevels semantics).
    if (!grid.probes.empty() && header.level0Dimensions[0] >= 2u && header.level1Dimensions[0] >= 2u
        && header.level0ProbeOffset < grid.probes.size() && header.level1ProbeOffset < grid.probes.size()
        && header.level1ProbeOffset > header.level0ProbeOffset)
    {
        grid.level0.anchor = glm::make_vec3(grid.probes[header.level0ProbeOffset].position);
        grid.level0.dims   = glm::ivec3(static_cast<int>(header.level0Dimensions[0]),
            static_cast<int>(header.level0Dimensions[1]), static_cast<int>(header.level0Dimensions[2]));
        grid.level0.spacing = header.level0Spacing;
        grid.level0.offset  = header.level0ProbeOffset;
        grid.level1.anchor  = glm::make_vec3(grid.probes[header.level1ProbeOffset].position);
        grid.level1.dims    = glm::ivec3(static_cast<int>(header.level1Dimensions[0]),
            static_cast<int>(header.level1Dimensions[1]), static_cast<int>(header.level1Dimensions[2]));
        grid.level1.spacing = header.level1Spacing;
        grid.level1.offset  = header.level1ProbeOffset;
        grid.levelsBuilt    = true;
    }
    state.probeGrid = std::move(grid);
    return true;
}

namespace
{

// Trilinear footprint of one position: the eight surrounding lattice probes'
// SH2 blocks and validities gathered into per-thread contiguous storage.
// Hit shading evaluates these lookups for every traced ray (and the debug
// views for every texel), and rays of one surface overwhelmingly share their
// lattice cell, so gathering once per cell removes the scattered probe-array
// loads from the per-ray path. The gathered values are the probes' own data
// verbatim and the evaluation keeps the direct walk's corner order, weights,
// and arithmetic, so results are identical. The cache identity pins the grid
// version (the provider republishes on content change), the level descriptor,
// and the base corner.
struct CachedFootprint
{
    std::uint64_t version = 0;
    glm::vec3     anchor{0.0f};
    float         spacing = 0.0f;
    glm::ivec3    dims{0};
    std::uint32_t offset  = 0;
    glm::ivec3    base{0};
    bool          filled = false;
    glm::vec3     sh2[8][9]{};
    float         validity[8]{};
};

// Picks the lattice level for a world position (renderer semantics: level 0
// inside its half-cell-extended bounds, level 1 everywhere else).
const RfProbeLevel *probeLevelFor(const RfProbeGrid &grid, const glm::vec3 &position)
{
    if (!grid.levelsBuilt)
    {
        return nullptr;
    }
    const glm::vec3 local   = position - grid.level0.anchor;
    const glm::vec3 extent0 = glm::vec3(grid.level0.dims - 1) * grid.level0.spacing;
    const float     halfCell0 = grid.level0.spacing * 0.5f;
    const bool insideLevel0 = glm::all(glm::bvec4(local.x >= -halfCell0, local.y >= -halfCell0,
        local.z >= -halfCell0, true))
        && glm::all(glm::bvec4(local.x <= extent0.x + halfCell0, local.y <= extent0.y + halfCell0,
            local.z <= extent0.z + halfCell0, true));
    return insideLevel0 ? &grid.level0 : &grid.level1;
}

// Gathers (or replays from the cache) the footprint of `position` on `level`:
// fills the fraction and the per-slot weights (lattice x validity) and the
// weight sum, in the corner order the direct gather used. Returns false when
// no probe of the footprint carries data.
bool probeFootprint(const RfPipelineState &state, const RfProbeLevel &level, const glm::vec3 &position,
    CachedFootprint &cache, float weights[8], float &weightSum)
{
    const RfProbeGrid &grid = state.probeGrid;
    const glm::vec3 cell     = (position - level.anchor) / level.spacing;
    const glm::ivec3 base    = glm::clamp(glm::ivec3(glm::floor(cell)), glm::ivec3(0), level.dims - glm::ivec3(2));
    const glm::vec3 fraction = glm::clamp(cell - glm::vec3(base), 0.0f, 1.0f);
    if (!cache.filled || cache.version != grid.version || cache.base != base
        || cache.spacing != level.spacing || cache.dims != level.dims || cache.offset != level.offset
        || cache.anchor != level.anchor)
    {
        cache.version = grid.version;
        cache.anchor  = level.anchor;
        cache.spacing = level.spacing;
        cache.dims    = level.dims;
        cache.offset  = level.offset;
        cache.base    = base;
        for (int cornerZ = 0; cornerZ < 2; ++cornerZ)
        {
            for (int cornerY = 0; cornerY < 2; ++cornerY)
            {
                for (int cornerX = 0; cornerX < 2; ++cornerX)
                {
                    const glm::ivec3 corner = base + glm::ivec3(cornerX, cornerY, cornerZ);
                    const std::size_t probeIndex = static_cast<std::size_t>(level.offset)
                        + static_cast<std::size_t>(corner.x + level.dims.x * (corner.y + level.dims.y * corner.z));
                    const std::uint32_t slot = static_cast<std::uint32_t>(cornerZ * 4 + cornerY * 2 + cornerX);
                    if (probeIndex >= grid.probes.size())
                    {
                        cache.validity[slot] = 0.0f;
                        std::fill(std::begin(cache.sh2[slot]), std::end(cache.sh2[slot]), glm::vec3(0.0f));
                        continue;
                    }
                    const lumen_gi::GiProbe &probe = grid.probes[probeIndex];
                    for (std::uint32_t band = 0; band < 9; ++band)
                    {
                        cache.sh2[slot][band] = glm::vec3(probe.sh2[band][0], probe.sh2[band][1], probe.sh2[band][2]);
                    }
                    cache.validity[slot] = probe.validity;
                }
            }
        }
        cache.filled = true;
    }
    weightSum = 0.0f;
    for (std::uint32_t slot = 0; slot < 8; ++slot)
    {
        const float latticeWeight
            = (static_cast<float>(slot & 1u) * fraction.x + (1.0f - static_cast<float>(slot & 1u)) * (1.0f - fraction.x))
              * (static_cast<float>((slot >> 1) & 1u) * fraction.y
                  + (1.0f - static_cast<float>((slot >> 1) & 1u)) * (1.0f - fraction.y))
              * (static_cast<float>((slot >> 2) & 1u) * fraction.z
                  + (1.0f - static_cast<float>((slot >> 2) & 1u)) * (1.0f - fraction.z));
        weights[slot] = latticeWeight * cache.validity[slot];
        if (cache.validity[slot] > 0.0f)
        {
            weightSum += weights[slot];
        }
        else
        {
            weights[slot] = 0.0f;
        }
    }
    return weightSum > 1.0e-6f;
}

} // namespace

bool rfProbeIrradiance(
    const RfPipelineState &state, const glm::vec3 &position, const glm::vec3 &normal, glm::vec3 &outIrradiance)
{
    outIrradiance = glm::vec3(0.0f);
    const RfProbeGrid &grid = state.probeGrid;
    const RfProbeLevel *level = probeLevelFor(grid, position);
    if (level == nullptr)
    {
        return false;
    }
    static thread_local CachedFootprint irradianceCache;
    float    weights[8]   = {};
    float    weightSum    = 0.0f;
    if (!probeFootprint(state, *level, position, irradianceCache, weights, weightSum))
    {
        return false;
    }
    const float normalArray[3] = {normal.x, normal.y, normal.z};
    // The tangent-plane (geometric) term of the probe weights comes from the
    // shared lumen_gi::probeTangentPlaneFactor - the exported-grid convention
    // the provider's world cache and the renderer's probe walk also apply -
    // and each valid corner evaluates into one vec3 accumulator (the
    // register-friendly shape; see the renderer twin in gi_probe_sampling.hpp).
    const float inverseSpacing = 1.0f / std::max(level->spacing, 1.0e-5f);
    float     valid = 0.0f;
    glm::vec3 sum(0.0f);
    for (std::uint32_t slot = 0; slot < 8; ++slot)
    {
        if (weights[slot] <= 0.0f)
        {
            continue;
        }
        const glm::ivec3 corner(irradianceCache.base + glm::ivec3(static_cast<int>(slot & 1u),
            static_cast<int>((slot >> 1) & 1u), static_cast<int>((slot >> 2) & 1u)));
        const glm::vec3 toProbe = (level->anchor + glm::vec3(corner) * level->spacing) - position;
        const float weight
            = weights[slot] * lumen_gi::probeTangentPlaneFactor(&normal.x, &toProbe.x, inverseSpacing);
        if (weight <= 1.0e-8f)
        {
            continue;
        }
        const float blend = weight / weightSum;
        glm::vec3    irradiance;
        lumen_gi::evaluateSh2Irradiance(&irradianceCache.sh2[slot][0].x, normalArray, glm::value_ptr(irradiance));
        sum += irradiance * blend;
        valid += irradianceCache.validity[slot] * blend;
    }
    if (valid <= 0.0f)
    {
        return false;
    }
    outIrradiance = sum;
    return true;
}

bool rfProbeRadiance(
    const RfPipelineState &state, const glm::vec3 &position, const glm::vec3 &direction, glm::vec3 &outRadiance)
{
    outRadiance = glm::vec3(0.0f);
    const RfProbeGrid &grid = state.probeGrid;
    const RfProbeLevel *level = probeLevelFor(grid, position);
    if (level == nullptr)
    {
        return false;
    }
    static thread_local CachedFootprint radianceCache;
    float    weights[8]   = {};
    float    weightSum    = 0.0f;
    if (!probeFootprint(state, *level, position, radianceCache, weights, weightSum))
    {
        return false;
    }
    float valid = 0.0f;
    for (std::uint32_t slot = 0; slot < 8; ++slot)
    {
        if (weights[slot] <= 0.0f)
        {
            continue;
        }
        glm::vec3 radiance;
        evaluateSh2Radiance(&radianceCache.sh2[slot][0].x, direction, radiance);
        const float blend = weights[slot] / weightSum;
        outRadiance += radiance * blend;
        valid += radianceCache.validity[slot] * blend;
    }
    return valid > 0.0f;
}

} // namespace lumen_reflection
} // namespace engine
