// lumen_gi scene side: geometry ingestion, per-surface BVH build, card
// placement (Lumen's six-face axis-aligned fallback projection), probe
// lattice management, and budgeted card capture.
//
// Geometry is stored in the surface's local space; every world-space query
// transforms its ray through the inverse model matrix, so dynamic surfaces
// never rebuild anything. Capture casts one ortho ray per texel against the
// surface's own triangles and stores a GBuffer-like card representation
// (albedo, emissive, world normal, depth) exactly like Lumen's card capture
// output; invalid texels carry depth = invalidCardDepth.

#include "lumen_gi_internal.hpp"

#include <glm/geometric.hpp>
#include <glm/glm.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace engine
{
namespace lumen_gi
{

// ---- ray helpers -----------------------------------------------------------

bool rayTriangle(const GiRay &ray, const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2, float &outT,
    glm::vec3 &outBarycentric)
{
    const glm::vec3 edge1 = v1 - v0;
    const glm::vec3 edge2 = v2 - v0;
    const glm::vec3 pVec  = glm::cross(ray.direction, edge2);
    const float     determinant = glm::dot(edge1, pVec);
    if (std::fabs(determinant) < 1.0e-12f)
    {
        return false;
    }
    const float     inverseDeterminant = 1.0f / determinant;
    const glm::vec3 tVec               = ray.origin - v0;
    const float     u                  = glm::dot(tVec, pVec) * inverseDeterminant;
    if (u < 0.0f || u > 1.0f)
    {
        return false;
    }
    const glm::vec3 qVec = glm::cross(tVec, edge1);
    const float     v    = glm::dot(ray.direction, qVec) * inverseDeterminant;
    if (v < 0.0f || u + v > 1.0f)
    {
        return false;
    }
    const float t = glm::dot(edge2, qVec) * inverseDeterminant;
    if (t < 1.0e-4f || t > ray.tMax)
    {
        return false;
    }
    outT              = t;
    outBarycentric    = glm::vec3(1.0f - u - v, u, v);
    return true;
}

bool rayBounds(const glm::vec3 &origin, const glm::vec3 &inverseDirection, const glm::vec3 &boundsMin,
    const glm::vec3 &boundsMax, float tMax)
{
    float tNear = 0.0f;
    float tFar  = tMax;
    for (std::uint32_t axis = 0; axis < 3; ++axis)
    {
        const float directionAxis = inverseDirection[axis];
        float       t0            = (boundsMin[axis] - origin[axis]) * directionAxis;
        float       t1            = (boundsMax[axis] - origin[axis]) * directionAxis;
        if (directionAxis < 0.0f)
        {
            std::swap(t0, t1);
        }
        tNear = std::max(tNear, t0);
        tFar  = std::min(tFar, t1);
        if (tNear > tFar)
        {
            return false;
        }
    }
    return true;
}

namespace
{

glm::vec3 safeInverseDirection(const glm::vec3 &direction)
{
    glm::vec3 inverse;
    for (std::uint32_t axis = 0; axis < 3; ++axis)
    {
        inverse[axis] = std::fabs(direction[axis]) > 1.0e-8f ? 1.0f / direction[axis] : 1.0e8f;
    }
    return inverse;
}

// Closest-hit traversal of one surface's local-space BVH.
bool traceSurfaceClosest(const GiSurface &surface, GiRay &ray, std::uint32_t &outTriangle, glm::vec3 &outBarycentric)
{
    const GiSurfaceGeometry &geometry = surface.geometry;
    if (geometry.bvh.empty() || geometry.indices.empty())
    {
        return false;
    }
    const glm::vec3 inverseDirection = safeInverseDirection(ray.direction);
    if (!rayBounds(ray.origin, inverseDirection, geometry.boundsMin, geometry.boundsMax, ray.tMax))
    {
        return false;
    }
    std::array<std::uint32_t, 64> stack{};
    std::uint32_t                 stackSize = 0;
    stack[stackSize++]                  = 0;
    bool          found                 = false;
    while (stackSize > 0)
    {
        const std::uint32_t      nodeIndex = stack[--stackSize];
        const GiSurfaceGeometry::BvhNode &node      = geometry.bvh[nodeIndex];
        if (node.count > 0)
        {
            for (std::uint32_t local = 0; local < node.count; ++local)
            {
                const std::uint32_t triangle = geometry.bvhOrder[node.leftOrFirst + local];
                const glm::vec3 &   a        = geometry.positions[geometry.indices[triangle * 3 + 0]];
                const glm::vec3 &   b        = geometry.positions[geometry.indices[triangle * 3 + 1]];
                const glm::vec3 &   c        = geometry.positions[geometry.indices[triangle * 3 + 2]];
                float               t        = 0.0f;
                glm::vec3           barycentric{};
                if (rayTriangle(ray, a, b, c, t, barycentric))
                {
                    ray.tMax = t;
                    outTriangle       = triangle;
                    outBarycentric    = barycentric;
                    found             = true;
                }
            }
            continue;
        }
        if (!rayBounds(ray.origin, inverseDirection, node.boundsMin, node.boundsMax, ray.tMax))
        {
            continue;
        }
        stack[stackSize++] = node.leftOrFirst;
        stack[stackSize++] = node.leftOrFirst + 1;
    }
    return found;
}

// Any-hit traversal (shadows): stops at the first accepted hit.
bool traceSurfaceOccluded(const GiSurface &surface, GiRay &ray)
{
    const GiSurfaceGeometry &geometry = surface.geometry;
    if (geometry.bvh.empty() || geometry.indices.empty())
    {
        return false;
    }
    const glm::vec3 inverseDirection = safeInverseDirection(ray.direction);
    if (!rayBounds(ray.origin, inverseDirection, geometry.boundsMin, geometry.boundsMax, ray.tMax))
    {
        return false;
    }
    std::array<std::uint32_t, 64> stack{};
    std::uint32_t                 stackSize = 0;
    stack[stackSize++]                  = 0;
    while (stackSize > 0)
    {
        const std::uint32_t               nodeIndex = stack[--stackSize];
        const GiSurfaceGeometry::BvhNode &node      = geometry.bvh[nodeIndex];
        if (node.count > 0)
        {
            for (std::uint32_t local = 0; local < node.count; ++local)
            {
                const std::uint32_t triangle = geometry.bvhOrder[node.leftOrFirst + local];
                const glm::vec3 &   a        = geometry.positions[geometry.indices[triangle * 3 + 0]];
                const glm::vec3 &   b        = geometry.positions[geometry.indices[triangle * 3 + 1]];
                const glm::vec3 &   c        = geometry.positions[geometry.indices[triangle * 3 + 2]];
                float               t        = 0.0f;
                glm::vec3           barycentric{};
                if (rayTriangle(ray, a, b, c, t, barycentric))
                {
                    return true;
                }
            }
            continue;
        }
        if (!rayBounds(ray.origin, inverseDirection, node.boundsMin, node.boundsMax, ray.tMax))
        {
            continue;
        }
        stack[stackSize++] = node.leftOrFirst;
        stack[stackSize++] = node.leftOrFirst + 1;
    }
    return false;
}

struct SurfaceHit
{
    bool          hit          = false;
    float         t            = 0.0f; // world-space distance
    std::uint32_t surfaceIndex = 0;
    std::uint32_t triangleIndex = 0;
    glm::vec3     barycentric{0.0f};
};

SurfaceHit traceSurfaces(const GiPipelineState &state, const GiRay &worldRay, bool anyHit)
{
    SurfaceHit result;
    float      bestT = worldRay.tMax;
    for (std::uint32_t surfaceIndex = 0; surfaceIndex < state.surfaces.size(); ++surfaceIndex)
    {
        const GiSurface &surface = state.surfaces[surfaceIndex];
        if (surface.excluded)
        {
            continue;
        }
        // Transform the ray into the surface's local space; local t values
        // scale by the local direction length and convert back on return.
        const glm::vec3 localOrigin     = glm::vec3(surface.inverseTransform * glm::vec4(worldRay.origin, 1.0f));
        const glm::vec3 localDirection  = glm::mat3(surface.inverseTransform) * worldRay.direction;
        const float     directionLength = glm::length(localDirection);
        if (directionLength < 1.0e-8f)
        {
            continue;
        }
        GiRay         localRay{localOrigin, localDirection / directionLength, bestT * directionLength};
        std::uint32_t triangle    = 0;
        glm::vec3     barycentric{};
        if (anyHit)
        {
            if (traceSurfaceOccluded(surface, localRay))
            {
                result.hit          = true;
                result.surfaceIndex = surfaceIndex;
                return result;
            }
            continue;
        }
        if (traceSurfaceClosest(surface, localRay, triangle, barycentric))
        {
            bestT                = localRay.tMax / directionLength;
            result.hit           = true;
            result.t             = bestT;
            result.surfaceIndex  = surfaceIndex;
            result.triangleIndex = triangle;
            result.barycentric   = barycentric;
        }
    }
    return result;
}

} // namespace

// ---- public trace entry points ---------------------------------------------

GiHit traceScene(const GiPipelineState &state, const GiRay &ray)
{
    const SurfaceHit surfaceHit = traceSurfaces(state, ray, false);
    GiHit            result;
    result.hit          = surfaceHit.hit;
    result.t            = surfaceHit.t;
    result.surfaceIndex = surfaceHit.surfaceIndex;
    result.triangleIndex = surfaceHit.triangleIndex;
    if (surfaceHit.hit)
    {
        const GiSurface &surface = state.surfaces[surfaceHit.surfaceIndex];
        result.position          = ray.origin + ray.direction * surfaceHit.t;
        // Smooth normal from the triangle's vertex normals, transformed by the
        // surface normal matrix (world space).
        const GiSurfaceGeometry &geometry = surface.geometry;
        const glm::vec3 &        n0       = geometry.normals[geometry.indices[surfaceHit.triangleIndex * 3 + 0]];
        const glm::vec3 &        n1       = geometry.normals[geometry.indices[surfaceHit.triangleIndex * 3 + 1]];
        const glm::vec3 &        n2       = geometry.normals[geometry.indices[surfaceHit.triangleIndex * 3 + 2]];
        const glm::vec3          localNormal = glm::normalize(
            n0 * surfaceHit.barycentric.x + n1 * surfaceHit.barycentric.y + n2 * surfaceHit.barycentric.z);
        result.normal          = glm::normalize(surface.normalMatrix * localNormal);
        const glm::vec3 &      p0 = geometry.positions[geometry.indices[surfaceHit.triangleIndex * 3 + 0]];
        const glm::vec3 &      p1 = geometry.positions[geometry.indices[surfaceHit.triangleIndex * 3 + 1]];
        const glm::vec3 &      p2 = geometry.positions[geometry.indices[surfaceHit.triangleIndex * 3 + 2]];
        const glm::vec3        edge1World = surface.normalMatrix * glm::cross(p1 - p0, p2 - p0);
        result.geometricNormal = glm::normalize(edge1World);
        // Make the geometric normal face the ray for two-sided triangle tests.
        if (glm::dot(result.geometricNormal, ray.direction) > 0.0f)
        {
            result.geometricNormal = -result.geometricNormal;
        }
    }
    return result;
}

bool traceSceneOccluded(const GiPipelineState &state, const GiRay &ray, std::uint32_t excludedSurface)
{
    for (std::uint32_t surfaceIndex = 0; surfaceIndex < state.surfaces.size(); ++surfaceIndex)
    {
        const GiSurface &surface = state.surfaces[surfaceIndex];
        if (surface.excluded || surfaceIndex == excludedSurface)
        {
            continue;
        }
        const glm::vec3 localOrigin    = glm::vec3(surface.inverseTransform * glm::vec4(ray.origin, 1.0f));
        const glm::vec3 localDirection = glm::mat3(surface.inverseTransform) * ray.direction;
        const float     directionLength = glm::length(localDirection);
        if (directionLength < 1.0e-8f)
        {
            continue;
        }
        GiRay         localRay{localOrigin, localDirection / directionLength, ray.tMax * directionLength};
        if (traceSurfaceOccluded(surface, localRay))
        {
            return true;
        }
    }
    return false;
}

// ---- geometry ingestion ----------------------------------------------------

void buildSurfaceGeometry(GiSurfaceGeometry &geometry, const GiSurfaceDescription &description,
    std::uint64_t maxTriangles, GiError *error)
{
    auto fail = [&](GiError value)
    {
        if (error != nullptr)
        {
            *error = value;
        }
    };
    if (description.positions == nullptr || description.indices == nullptr || description.vertexCount == 0
        || description.indexCount < 3 || description.indexCount % 3 != 0)
    {
        fail(GiError::InvalidDescription);
        return;
    }
    const std::uint64_t triangleCount = description.indexCount / 3;
    if (triangleCount > maxTriangles)
    {
        fail(GiError::InvalidDescription);
        return;
    }
    const std::size_t positionStride = description.positionStrideBytes != 0
                                         ? description.positionStrideBytes / sizeof(float)
                                         : 3;
    if (description.positionStrideBytes % sizeof(float) != 0)
    {
        fail(GiError::InvalidDescription);
        return;
    }
    const bool hasNormals = description.normals != nullptr;
    const bool hasColors  = description.colors != nullptr;
    if (hasNormals && description.normalStrideBytes % sizeof(float) != 0)
    {
        fail(GiError::InvalidDescription);
        return;
    }
    if (hasColors && description.colorStrideBytes % sizeof(float) != 0)
    {
        fail(GiError::InvalidDescription);
        return;
    }
    for (std::uint32_t index = 0; index < description.indexCount; ++index)
    {
        if (description.indices[index] >= description.vertexCount)
        {
            fail(GiError::InvalidDescription);
            return;
        }
    }
    // All floats must be finite before any math touches them.
    for (std::uint32_t vertex = 0; vertex < description.vertexCount; ++vertex)
    {
        const float *position = description.positions + static_cast<std::size_t>(vertex) * positionStride;
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            if (!std::isfinite(position[axis]))
            {
                fail(GiError::InvalidDescription);
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
        const std::size_t normalStride = description.normalStrideBytes != 0
                                           ? description.normalStrideBytes / sizeof(float)
                                           : 3;
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
        // Generate smooth normals from triangle geometry (renderer's
        // prepareMeshData does the same for missing normals).
        geometry.normals.assign(description.vertexCount, glm::vec3(0.0f));
        for (std::uint32_t triangle = 0; triangle < triangleCount; ++triangle)
        {
            const std::uint32_t i0 = geometry.indices[triangle * 3 + 0];
            const std::uint32_t i1 = geometry.indices[triangle * 3 + 1];
            const std::uint32_t i2 = geometry.indices[triangle * 3 + 2];
            const glm::vec3     faceNormal = glm::cross(geometry.positions[i1] - geometry.positions[i0],
                geometry.positions[i2] - geometry.positions[i0]);
            geometry.normals[i0] += faceNormal;
            geometry.normals[i1] += faceNormal;
            geometry.normals[i2] += faceNormal;
        }
        for (glm::vec3 &normal : geometry.normals)
        {
            const float length = glm::length(normal);
            normal = length > 1.0e-12f ? normal / length : glm::vec3(0.0f, 1.0f, 0.0f);
        }
    }

    if (hasColors)
    {
        const std::size_t colorStride = description.colorStrideBytes != 0 ? description.colorStrideBytes / sizeof(float)
                                                                          : 3;
        geometry.colors.resize(description.vertexCount);
        for (std::uint32_t vertex = 0; vertex < description.vertexCount; ++vertex)
        {
            const float *source = description.colors + static_cast<std::size_t>(vertex) * colorStride;
            geometry.colors[vertex] =
                glm::vec3(std::max(source[0], 0.0f), std::max(source[1], 0.0f), std::max(source[2], 0.0f));
        }
    }
    else
    {
        geometry.colors.assign(description.vertexCount, glm::vec3(1.0f));
    }
    if (error != nullptr)
    {
        *error = GiError::None;
    }
}

void buildSurfaceBvh(GiSurfaceGeometry &geometry)
{
    const std::uint32_t triangleCount = static_cast<std::uint32_t>(geometry.indices.size() / 3);
    geometry.bvh.clear();
    geometry.bvhOrder.clear();
    if (triangleCount == 0)
    {
        return;
    }
    geometry.bvhOrder.resize(triangleCount);
    for (std::uint32_t triangle = 0; triangle < triangleCount; ++triangle)
    {
        geometry.bvhOrder[triangle] = triangle;
    }
    // Centroid bounds per triangle for splitting decisions.
    std::vector<glm::vec3>      centroids(triangleCount);
    std::vector<glm::vec3>      triangleMin(triangleCount);
    std::vector<glm::vec3>      triangleMax(triangleCount);
    glm::vec3                   globalMin(std::numeric_limits<float>::max());
    glm::vec3                   globalMax(-std::numeric_limits<float>::max());
    for (std::uint32_t triangle = 0; triangle < triangleCount; ++triangle)
    {
        const glm::vec3 &a = geometry.positions[geometry.indices[triangle * 3 + 0]];
        const glm::vec3 &b = geometry.positions[geometry.indices[triangle * 3 + 1]];
        const glm::vec3 &c = geometry.positions[geometry.indices[triangle * 3 + 2]];
        triangleMin[triangle] = glm::min(a, glm::min(b, c));
        triangleMax[triangle] = glm::max(a, glm::max(b, c));
        centroids[triangle]   = (a + b + c) * (1.0f / 3.0f);
        globalMin             = glm::min(globalMin, triangleMin[triangle]);
        globalMax             = glm::max(globalMax, triangleMax[triangle]);
    }
    geometry.boundsMin = globalMin;
    geometry.boundsMax = globalMax;

    // Recursive median split on the widest centroid axis (deterministic; the
    // scene sizes here do not justify an SAH build).
    struct BuildEntry
    {
        std::uint32_t nodeIndex;
        std::uint32_t begin;
        std::uint32_t end;
    };
    geometry.bvh.reserve(2 * triangleCount);
    auto makeNode = [&](const glm::vec3 &boundsMin, const glm::vec3 &boundsMax) -> std::uint32_t
    {
        GiSurfaceGeometry::BvhNode node;
        node.boundsMin    = boundsMin;
        node.boundsMax    = boundsMax;
        node.leftOrFirst  = 0;
        node.count        = 0;
        geometry.bvh.push_back(node);
        return static_cast<std::uint32_t>(geometry.bvh.size() - 1);
    };
    std::vector<BuildEntry> stack;
    const std::uint32_t     root = makeNode(globalMin, globalMax);
    stack.push_back(BuildEntry{root, 0, triangleCount});
    while (!stack.empty())
    {
        const BuildEntry entry = stack.back();
        stack.pop_back();
        const std::uint32_t span = entry.end - entry.begin;
        if (span <= 4)
        {
            GiSurfaceGeometry::BvhNode &node = geometry.bvh[entry.nodeIndex];
            node.leftOrFirst                 = entry.begin;
            node.count                       = span;
            continue;
        }
        glm::vec3 centroidMin(std::numeric_limits<float>::max());
        glm::vec3 centroidMax(-std::numeric_limits<float>::max());
        for (std::uint32_t i = entry.begin; i < entry.end; ++i)
        {
            const glm::vec3 &centroid = centroids[geometry.bvhOrder[i]];
            centroidMin = glm::min(centroidMin, centroid);
            centroidMax = glm::max(centroidMax, centroid);
        }
        const glm::vec3 extent = centroidMax - centroidMin;
        std::uint32_t   axis   = 0;
        if (extent.y >= extent.x && extent.y >= extent.z)
        {
            axis = 1;
        }
        else if (extent.z >= extent.x && extent.z >= extent.y)
        {
            axis = 2;
        }
        const std::uint32_t middle = entry.begin + span / 2;
        std::nth_element(geometry.bvhOrder.begin() + static_cast<std::ptrdiff_t>(entry.begin),
            geometry.bvhOrder.begin() + static_cast<std::ptrdiff_t>(middle),
            geometry.bvhOrder.begin() + static_cast<std::ptrdiff_t>(entry.end),
            [&](std::uint32_t left, std::uint32_t right)
            { return centroids[left][axis] < centroids[right][axis]; });
        glm::vec3 leftMin(std::numeric_limits<float>::max());
        glm::vec3 leftMax(-std::numeric_limits<float>::max());
        glm::vec3 rightMin(std::numeric_limits<float>::max());
        glm::vec3 rightMax(-std::numeric_limits<float>::max());
        for (std::uint32_t i = entry.begin; i < middle; ++i)
        {
            leftMin = glm::min(leftMin, triangleMin[geometry.bvhOrder[i]]);
            leftMax = glm::max(leftMax, triangleMax[geometry.bvhOrder[i]]);
        }
        for (std::uint32_t i = middle; i < entry.end; ++i)
        {
            rightMin = glm::min(rightMin, triangleMin[geometry.bvhOrder[i]]);
            rightMax = glm::max(rightMax, triangleMax[geometry.bvhOrder[i]]);
        }
        const std::uint32_t leftIndex  = makeNode(leftMin, leftMax);
        const std::uint32_t rightIndex = makeNode(rightMin, rightMax);
        geometry.bvh[entry.nodeIndex].leftOrFirst = leftIndex;
        geometry.bvh[entry.nodeIndex].count       = 0;
        stack.push_back(BuildEntry{leftIndex, entry.begin, middle});
        stack.push_back(BuildEntry{rightIndex, middle, entry.end});
    }
}

void updateSurfaceWorldBounds(GiSurface &surface)
{
    const GiSurfaceGeometry &geometry = surface.geometry;
    glm::vec3                worldMin(std::numeric_limits<float>::max());
    glm::vec3                worldMax(-std::numeric_limits<float>::max());
    for (std::uint32_t corner = 0; corner < 8; ++corner)
    {
        const glm::vec3 local((corner & 1u) ? geometry.boundsMax.x : geometry.boundsMin.x,
            (corner & 2u) ? geometry.boundsMax.y : geometry.boundsMin.y,
            (corner & 4u) ? geometry.boundsMax.z : geometry.boundsMin.z);
        const glm::vec3 world = glm::vec3(surface.transform * glm::vec4(local, 1.0f));
        worldMin              = glm::min(worldMin, world);
        worldMax              = glm::max(worldMax, world);
    }
    surface.worldMin = worldMin;
    surface.worldMax = worldMax;
}

// ---- card placement ---------------------------------------------------------

// Places the surface's cards as axis-aligned ortho projections of its world
// AABB: six face cards, with the largest faces subdivided when a card would
// exceed maxTexelsPerCardAxis texels along an axis. This is Lumen's
// documented fallback placement (used when import-time surfel clustering is
// unavailable or fails); total cards are clamped to maxCardsPerSurface.
void placeSurfaceCards(GiPipelineState &state, GiSurface &surface, std::uint32_t surfaceIndex)
{
    surface.firstCard = static_cast<std::uint32_t>(state.cards.size());
    surface.cardCount = 0;
    surface.cardsBuilt = true;
    if (surface.excluded)
    {
        return;
    }

    const glm::vec3 boundsMin = surface.worldMin;
    const glm::vec3 boundsMax = surface.worldMax;
    const glm::vec3 extent    = glm::max(boundsMax - boundsMin, glm::vec3(1.0e-3f));
    const float     density   = state.settings.texelsPerMeter;
    const std::uint32_t maxAxisTexels = std::max(state.settings.maxTexelsPerCardAxis, 4u);

    struct Candidate
    {
        bool          baseCard = false; // one of the six unsplit faces
        std::uint32_t axis     = 0;     // outward axis
        std::uint32_t sign     = 0;     // 0 = negative face, 1 = positive face
        glm::vec3     origin{0.0f};
        glm::vec3     axisU{0.0f};
        glm::vec3     axisV{0.0f};
        glm::vec3     axisW{0.0f};
        float         width  = 0.0f;
        float         height = 0.0f;
        std::uint32_t texelWidth = 0;
        std::uint32_t texelHeight = 0;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(6);
    for (std::uint32_t axis = 0; axis < 3; ++axis)
    {
        const std::uint32_t uAxis = (axis + 1) % 3;
        const std::uint32_t vAxis = (axis + 2) % 3;
        for (std::uint32_t sign = 0; sign < 2; ++sign)
        {
            Candidate candidate;
            candidate.baseCard = true;
            candidate.axis     = axis;
            candidate.sign     = sign;
            candidate.axisW[axis] = sign == 0 ? -1.0f : 1.0f;
            candidate.axisU[uAxis] = 1.0f;
            candidate.axisV[vAxis] = 1.0f;
            candidate.origin[axis] = sign == 0 ? boundsMin[axis] : boundsMax[axis];
            candidate.origin[uAxis] = 0.5f * (boundsMin[uAxis] + boundsMax[uAxis]);
            candidate.origin[vAxis] = 0.5f * (boundsMin[vAxis] + boundsMax[vAxis]);
            candidate.width         = extent[uAxis];
            candidate.height        = extent[vAxis];
            candidate.texelWidth =
                std::clamp(static_cast<std::uint32_t>(std::ceil(candidate.width * density)), 4u, 4096u);
            candidate.texelHeight =
                std::clamp(static_cast<std::uint32_t>(std::ceil(candidate.height * density)), 4u, 4096u);
            candidates.push_back(candidate);
        }
    }

    // Expand split candidates: a face longer than maxAxisTexels is subdivided
    // along its longer card axis so every piece stays within the budget.
    std::vector<Candidate> split;
    split.reserve(candidates.size() * 2);
    for (Candidate &candidate : candidates)
    {
        std::uint32_t splitU = 1;
        std::uint32_t splitV = 1;
        while (candidate.texelWidth / splitU > maxAxisTexels)
        {
            ++splitU;
        }
        while (candidate.texelHeight / splitV > maxAxisTexels)
        {
            ++splitV;
        }
        const std::uint32_t splits = splitU * splitV;
        if (splits == 1)
        {
            split.push_back(candidate);
            continue;
        }
        for (std::uint32_t v = 0; v < splitV; ++v)
        {
            for (std::uint32_t u = 0; u < splitU; ++u)
            {
                Candidate piece               = candidate;
                piece.baseCard                = false;
                const float pieceWidth        = candidate.width / static_cast<float>(splitU);
                const float pieceHeight       = candidate.height / static_cast<float>(splitV);
                const float uOffset           = (-0.5f + (static_cast<float>(u) + 0.5f) / static_cast<float>(splitU))
                                      * candidate.width;
                const float vOffset           = (-0.5f + (static_cast<float>(v) + 0.5f) / static_cast<float>(splitV))
                                      * candidate.height;
                piece.origin += candidate.axisU * uOffset + candidate.axisV * vOffset;
                piece.width                   = pieceWidth;
                piece.height                  = pieceHeight;
                piece.texelWidth              = std::clamp(
                    static_cast<std::uint32_t>(std::ceil(pieceWidth * density)), 4u, maxAxisTexels);
                piece.texelHeight             = std::clamp(
                    static_cast<std::uint32_t>(std::ceil(pieceHeight * density)), 4u, maxAxisTexels);
                split.push_back(piece);
            }
        }
    }

    // Clamp to the configured per-surface budget: base cards first, then the
    // split pieces with the largest texel counts.
    std::vector<Candidate *> ordered;
    ordered.reserve(split.size());
    for (Candidate &candidate : split)
    {
        ordered.push_back(&candidate);
    }
    const std::uint32_t maxCards = std::max(state.settings.maxCardsPerSurface, 1u);
    std::stable_sort(ordered.begin(),
        ordered.end(),
        [](const Candidate *left, const Candidate *right)
        {
            if (left->baseCard != right->baseCard)
            {
                return left->baseCard;
            }
            const std::uint64_t leftTexels = static_cast<std::uint64_t>(left->texelWidth) * left->texelHeight;
            const std::uint64_t rightTexels = static_cast<std::uint64_t>(right->texelWidth) * right->texelHeight;
            return leftTexels > rightTexels;
        });
    if (ordered.size() > maxCards)
    {
        ordered.resize(maxCards);
    }

    surface.firstCard = static_cast<std::uint32_t>(state.cards.size());
    for (const Candidate *candidate : ordered)
    {
        GiCard card;
        card.surfaceIndex = surfaceIndex;
        card.origin       = candidate->origin;
        card.axisU        = candidate->axisU;
        card.axisV        = candidate->axisV;
        card.axisW        = candidate->axisW;
        card.width        = candidate->width;
        card.height       = candidate->height;
        card.depth        = extent[candidate->axis] + 1.0e-2f;
        card.texelWidth   = candidate->texelWidth;
        card.texelHeight  = candidate->texelHeight;
        const std::size_t texels = static_cast<std::size_t>(card.texelWidth) * card.texelHeight;
        card.depthTexels.assign(texels, invalidCardDepth);
        card.albedo.assign(texels, glm::vec3(0.0f));
        card.emissive.assign(texels, glm::vec3(0.0f));
        card.normal.assign(texels, glm::vec3(0.0f, 1.0f, 0.0f));
        card.directRadiance.assign(texels, glm::vec3(0.0f));
        card.indirectRadiance.assign(texels, glm::vec3(0.0f));
        card.indirectPrevious.assign(texels, glm::vec3(0.0f));
        state.cards.push_back(std::move(card));
    }
    surface.cardCount = static_cast<std::uint32_t>(state.cards.size()) - surface.firstCard;
}

// ---- probe lattice -----------------------------------------------------------

void buildProbeDirectionTable(GiPipelineState &state)
{
    const std::uint32_t resolution = std::max(state.settings.probeDirectionResolution, 4u);
    state.probeDirections.clear();
    state.probeDirectionSolidAngles.clear();
    state.probeDirections.reserve(static_cast<std::size_t>(resolution) * resolution);
    state.probeDirectionSolidAngles.reserve(static_cast<std::size_t>(resolution) * resolution);
    // Octahedral mapping: texel centers of a resolution^2 map on the unit
    // octahedron unfolded to the [-1, 1]^2 square; sign of z folds.
    for (std::uint32_t y = 0; y < resolution; ++y)
    {
        for (std::uint32_t x = 0; x < resolution; ++x)
        {
            const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(resolution) * 2.0f - 1.0f;
            const float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(resolution) * 2.0f - 1.0f;
            glm::vec3   direction;
            direction.x = u;
            direction.y = v;
            direction.z = 1.0f - std::fabs(u) - std::fabs(v);
            if (direction.z < 0.0f)
            {
                const float wrappedX = (1.0f - std::fabs(v)) * (u >= 0.0f ? 1.0f : -1.0f);
                const float wrappedY = (1.0f - std::fabs(u)) * (v >= 0.0f ? 1.0f : -1.0f);
                direction.x          = wrappedX;
                direction.y          = wrappedY;
            }
            const float length = glm::length(direction);
            direction          = length > 1.0e-8f ? direction / length : glm::vec3(0.0f, 0.0f, 1.0f);
            state.probeDirections.push_back(direction);
            // Solid angle of one octahedral texel: 4 pi / texel count is the
            // uniform-atlas approximation used for SH2 projection weights.
            state.probeDirectionSolidAngles.push_back(4.0f * 3.141592654f
                / static_cast<float>(resolution * resolution));
        }
    }
    state.settings.probeDirectionResolution = resolution;

    // Screen probe direction table: fixed 8x8 octahedral map (Lumen's screen
    // probe directional resolution).
    constexpr std::uint32_t screenResolution = 8;
    state.screenProbeDirections.clear();
    state.screenProbeDirections.reserve(screenResolution * screenResolution);
    for (std::uint32_t y = 0; y < screenResolution; ++y)
    {
        for (std::uint32_t x = 0; x < screenResolution; ++x)
        {
            const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(screenResolution) * 2.0f - 1.0f;
            const float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(screenResolution) * 2.0f - 1.0f;
            glm::vec3   direction;
            direction.x = u;
            direction.y = v;
            direction.z = 1.0f - std::fabs(u) - std::fabs(v);
            if (direction.z < 0.0f)
            {
                const float wrappedX = (1.0f - std::fabs(v)) * (u >= 0.0f ? 1.0f : -1.0f);
                const float wrappedY = (1.0f - std::fabs(u)) * (v >= 0.0f ? 1.0f : -1.0f);
                direction.x          = wrappedX;
                direction.y          = wrappedY;
            }
            const float length = glm::length(direction);
            direction          = length > 1.0e-8f ? direction / length : glm::vec3(0.0f, 0.0f, 1.0f);
            state.screenProbeDirections.push_back(direction);
        }
    }
}

void buildProbeLevels(GiPipelineState &state)
{
    // Two camera-centered clip levels of 9^3 probes each (an odd dimension
    // keeps a probe exactly on the camera axis). Level 0 covers the near field
    // at the configured spacing; level 1 covers a four-times-larger volume at
    // the coarser spacing. Bounded memory: 2 x 729 probes x 256 directions.
    constexpr std::uint32_t probeGridDimension = 9;
    auto buildLevel = [&](GiPipelineState::ProbeLevel &level, float spacing)
    {
        level.spacing         = std::max(spacing, 0.25f);
        level.dimensions      = {probeGridDimension, probeGridDimension, probeGridDimension};
        level.probes.resize(static_cast<std::size_t>(probeGridDimension) * probeGridDimension * probeGridDimension);
        level.updateCursor    = 0;
    };
    buildLevel(state.level0, state.settings.probeSpacing);
    buildLevel(state.level1, state.settings.probeSpacing * std::max(state.settings.probeLevel1SpacingFactor, 2.0f));
    GiPipelineState::ProbeLevel *const levels[2] = {&state.level0, &state.level1};
    for (GiPipelineState::ProbeLevel *level : levels)
    {
        for (GiPipelineState::ProbeLevel::Probe &probe : level->probes)
        {
            probe.radiance.assign(state.probeDirections.size(), glm::vec3(0.0f));
            probe.radiancePrevious.assign(state.probeDirections.size(), glm::vec3(0.0f));
            probe.sh2.assign(27, 0.0f);
        }
    }
}

void snapProbeLevels(GiPipelineState &state, const glm::vec3 &cameraPosition)
{
    auto snapLevel = [&](GiPipelineState::ProbeLevel &level)
    {
        // Anchor the lattice so the camera sits at the center probe; probes
        // keep stable world positions until the camera crosses a cell.
        const std::uint32_t half     = level.dimensions[0] / 2;
        const glm::vec3     snapped  = (glm::floor(cameraPosition / level.spacing)
                                - static_cast<float>(half))
                                * level.spacing;
        level.anchor          = snapped;
        const std::uint32_t dim = level.dimensions[0];
        for (std::uint32_t z = 0; z < dim; ++z)
        {
            for (std::uint32_t y = 0; y < dim; ++y)
            {
                for (std::uint32_t x = 0; x < dim; ++x)
                {
                    const std::size_t index = (static_cast<std::size_t>(z) * dim + y) * dim + x;
                    GiPipelineState::ProbeLevel::Probe &probe = level.probes[index];
                    const glm::vec3 position = snapped + glm::vec3(static_cast<float>(x), static_cast<float>(y),
                                                               static_cast<float>(z))
                                               * level.spacing;
                    if (!probe.valid || glm::distance(probe.position, position) > 1.0e-4f)
                    {
                        // A snapped probe moves to a new world position: the
                        // cached radiance no longer matches (Lumen relies on
                        // probe hysteresis for the same effect); decay it and
                        // let the amortized updates repopulate.
                        probe.position  = position;
                        probe.validity  = 0.0f;
                        probe.valid     = true;
                    }
                }
            }
        }
    };
    snapLevel(state.level0);
    snapLevel(state.level1);
}

// ---- card capture ------------------------------------------------------------

// Captures texels [texelBegin, texelEnd) of one card: an ortho ray per texel
// against the surface's own triangles, storing depth, albedo, emissive, and
// world normal. Invalid texels keep invalidCardDepth and are skipped by the
// lighting and sampling passes.
void captureCard(GiPipelineState &state, GiCard &card, const GiSurface &surface, std::uint32_t texelBegin,
    std::uint32_t texelEnd)
{
    // The state carries the capture settings shared by the card families;
    // this family reads everything from the surface, so the parameter is
    // deliberately unused here.
    (void)state;
    if (surface.excluded)
    {
        return;
    }
    const std::uint32_t texelWidth  = card.texelWidth;
    const std::uint32_t texelHeight = card.texelHeight;
    const glm::vec3     inward      = -card.axisW;
    // Slightly outside the near plane so surface faces on the card plane are
    // found (Lumen applies the same capture bias).
    const float nearBias  = std::max(1.0e-3f, card.depth * 1.0e-3f);
    const float rayLength = card.depth + 2.0f * nearBias;
    for (std::uint32_t texel = texelBegin; texel < texelEnd && texel < texelWidth * texelHeight; ++texel)
    {
        const std::uint32_t x = texel % texelWidth;
        const std::uint32_t y = texel / texelWidth;
        const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(texelWidth) - 0.5f;
        const float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(texelHeight) - 0.5f;
        const glm::vec3 planePoint = card.origin + card.axisU * (u * card.width) + card.axisV * (v * card.height);
        GiRay           ray{planePoint + card.axisW * nearBias, inward, rayLength};
        std::uint32_t   triangle    = 0;
        glm::vec3       barycentric{};
        const GiSurfaceGeometry &geometry = surface.geometry;
        bool        found       = false;
        {
            // Closest hit against this surface only (local space).
            const glm::vec3 localOrigin    = glm::vec3(surface.inverseTransform * glm::vec4(ray.origin, 1.0f));
            const glm::vec3 localDirection = glm::mat3(surface.inverseTransform) * ray.direction;
            const float     directionLength = glm::length(localDirection);
            GiRay           localRay{localOrigin, localDirection / directionLength, ray.tMax * directionLength};
            found = traceSurfaceClosest(surface, localRay, triangle, barycentric);
            ray.tMax = found ? localRay.tMax / directionLength : ray.tMax;
        }
        if (!found)
        {
            card.depthTexels[texel]   = invalidCardDepth;
            card.albedo[texel]  = glm::vec3(0.0f);
            card.emissive[texel] = glm::vec3(0.0f);
            continue;
        }
        card.depthTexels[texel] = ray.tMax;
        const std::uint32_t i0 = geometry.indices[triangle * 3 + 0];
        const std::uint32_t i1 = geometry.indices[triangle * 3 + 1];
        const std::uint32_t i2 = geometry.indices[triangle * 3 + 2];
        const glm::vec3 localNormal = glm::normalize(geometry.normals[i0] * barycentric.x
            + geometry.normals[i1] * barycentric.y + geometry.normals[i2] * barycentric.z);
        card.normal[texel] = glm::normalize(surface.normalMatrix * localNormal);
        if (glm::dot(card.normal[texel], inward) > 0.0f)
        {
            // Faces pointing along the ray (interior shells seen from inside)
            // are not usable capture coverage; Lumen rejects backface
            // captures too.
            card.depthTexels[texel]  = invalidCardDepth;
            card.albedo[texel] = glm::vec3(0.0f);
            continue;
        }
        const glm::vec3 vertexColor = geometry.colors[i0] * barycentric.x + geometry.colors[i1] * barycentric.y
                                    + geometry.colors[i2] * barycentric.z;
        card.albedo[texel]   = vertexColor * surface.albedo;
        card.emissive[texel] = surface.emissive;
    }
}

} // namespace lumen_gi
} // namespace engine
