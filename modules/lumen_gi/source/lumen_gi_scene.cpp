// lumen_gi scene side: geometry ingestion, per-surface BVH build (binned
// SAH), the top-level surface BVH, ray tracing, card placement (Lumen's
// six-face axis-aligned fallback projection, in the surface's local space),
// card capture, and the probe lattice with toroidal scrolling.
//
// Geometry is stored in the surface's local space; every world-space query
// transforms its ray through the inverse model matrix, so dynamic surfaces
// never rebuild a surface BVH. Cards live in the same local space, so a moving
// instance carries its captured albedo / normal / depth (and cached lighting)
// with it; capture runs once per surface change instead of every frame.

#include <engine/lumen_gi/capabilities.hpp>
#include "lumen_gi_internal.hpp"

#include <cstddef>
#include <cstdint>
#include <glm/geometric.hpp>
#include <glm/glm.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace engine
{
namespace lumen_gi
{


// ---- ray helpers -----------------------------------------------------------
// safeInverseDirection, slabTest, and the Moller-Trumbore kernel come from
// engine/renderer/lumen_ray_core.hpp (shared with lumen_reflection).

namespace
{

constexpr std::uint32_t traversalStackSize = 96;

// Traversal of one surface's local-space BVH. AnyHit stops at the first
// accepted hit (shadows); otherwise ray.tMax shrinks to the closest hit.
// The caller passes the ray's precomputed inverse direction: rays against an
// identity-transform surface reuse the top-level inverse, so only
// transformed surfaces pay the three divisions again.
template <bool AnyHit>
bool traverseSurface(const GiSurfaceGeometry &geometry, GiRay &ray, const glm::vec3 &inverseDirection,
    std::uint32_t &outTriangle, glm::vec3 &outBarycentric)
{
    if (geometry.bvh.empty())
    {
        return false;
    }
    float           entry            = 0.0f;
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
        const GiSurfaceGeometry::BvhNode &node = geometry.bvh[stack[--stackSize]];
        if (node.count > 0)
        {
            const GiPackedTriangle *triangles = geometry.packed.data() + node.leftOrFirst;
            for (std::uint32_t local = 0; local < node.count; ++local)
            {
                float     t = 0.0f;
                glm::vec3 barycentric{};
                if (rayTriangle(ray, triangles[local], t, barycentric))
                {
                    if constexpr (AnyHit)
                    {
                        return true;
                    }
                    ray.tMax        = t;
                    outTriangle     = triangles[local].triangle;
                    outBarycentric  = barycentric;
                    found           = true;
                }
            }
            continue;
        }
        const GiSurfaceGeometry::BvhNode &left  = geometry.bvh[node.leftOrFirst];
        const GiSurfaceGeometry::BvhNode &right = geometry.bvh[node.leftOrFirst + 1];
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
    float         t            = 0.0f; // world-space distance
    std::uint32_t surfaceIndex = 0;
    std::uint32_t triangleIndex = 0;
    glm::vec3     barycentric{0.0f};
};

// Barycentric position, smooth normal, and raw geometric normal of a triangle
// hit in the surface's local space, shared by the full and the lean trace
// front-ends so the two can never drift apart.
struct LocalShading
{
    glm::vec3 position{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f}; // normalized smooth normal
    glm::vec3 geometric{0.0f};           // raw cross product, unnormalized
    float     geometricLength = 0.0f;
};

LocalShading localShadingAt(const GiSurfaceGeometry &geometry, std::uint32_t triangleIndex, const glm::vec3 &b)
{
    LocalShading      shading;
    const std::uint32_t i0 = geometry.indices[triangleIndex * 3 + 0];
    const std::uint32_t i1 = geometry.indices[triangleIndex * 3 + 1];
    const std::uint32_t i2 = geometry.indices[triangleIndex * 3 + 2];
    shading.position = geometry.positions[i0] * b.x + geometry.positions[i1] * b.y + geometry.positions[i2] * b.z;

    // Smooth normal from the triangle's vertex normals.
    glm::vec3 smooth = geometry.normals[i0] * b.x + geometry.normals[i1] * b.y + geometry.normals[i2] * b.z;
    shading.geometric       = glm::cross(
        geometry.positions[i1] - geometry.positions[i0], geometry.positions[i2] - geometry.positions[i0]);
    shading.geometricLength = glm::length(shading.geometric);
    const float smoothLength = glm::length(smooth);
    shading.normal = smoothLength > 1.0e-8f
        ? smooth / smoothLength
        : (shading.geometricLength > 1.0e-12f ? shading.geometric / shading.geometricLength
                                             : glm::vec3(0.0f, 1.0f, 0.0f));
    return shading;
}

// Traces one world ray through the top-level BVH and the surface BVHs.
template <bool AnyHit>
SurfaceHit traceSurfaces(const GiPipelineState &state, const GiRay &worldRay, std::uint32_t excludedSurface)
{
    SurfaceHit result;
    if (state.tlas.empty())
    {
        return result;
    }
    const glm::vec3 inverseDirection = safeInverseDirection(worldRay.direction);
    float           bestT            = worldRay.tMax;
    float           entry            = 0.0f;
    const GiTlasNode &root           = state.tlas[0];
    if (!slabTest(worldRay.origin, inverseDirection, root.boundsMin, root.boundsMax, bestT, entry))
    {
        return result;
    }
    std::array<std::uint32_t, traversalStackSize> stack;
    std::uint32_t                                 stackSize = 0;
    stack[stackSize++]                                      = 0;
    while (stackSize > 0)
    {
        const GiTlasNode &node = state.tlas[stack[--stackSize]];
        if (node.count > 0)
        {
            for (std::uint32_t local = 0; local < node.count; ++local)
            {
                const std::uint32_t surfaceIndex = state.tlasSurfaces[node.leftOrFirst + local];
                if (surfaceIndex == excludedSurface)
                {
                    continue;
                }
                const GiSurface &surface = state.surfaces[surfaceIndex];
                // World-space bounds test before any per-surface transform:
                // the TLAS leaf slab only proves the leaf's combined bounds
                // are grazed; individual surfaces inside it are frequently
                // missed, and the inverse-transform work is the dominant
                // per-leaf cost for those.
                float worldEntry = 0.0f;
                if (!slabTest(worldRay.origin, inverseDirection, surface.worldMin, surface.worldMax, bestT, worldEntry))
                {
                    continue;
                }
                // Transform the ray into the surface's local space; local t
                // values scale by the local direction length. The cached
                // inverse pieces replace the per-ray mat3 view of the mat4 and
                // the full 4x4 by-w=1 point product (same arithmetic), and an
                // identity transform skips both: a world-space ray is already
                // a valid local ray with direction length exactly one, so the
                // top-level inverse direction is reused by the BVH traversal.
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
                GiRay localRay{localOrigin,
                    surface.localIsWorld ? localDirection : localDirection / directionLength,
                    bestT * directionLength};
                std::uint32_t triangle = 0;
                glm::vec3     barycentric{};
                if (traverseSurface<AnyHit>(surface.geometry, localRay,
                        surface.localIsWorld ? inverseDirection : safeInverseDirection(localRay.direction), triangle,
                        barycentric))
                {
                    result.hit          = true;
                    result.surfaceIndex = surfaceIndex;
                    if constexpr (AnyHit)
                    {
                        return result;
                    }
                    bestT                = localRay.tMax / directionLength;
                    result.t             = bestT;
                    result.triangleIndex = triangle;
                    result.barycentric   = barycentric;
                }
            }
            continue;
        }
        const GiTlasNode &left  = state.tlas[node.leftOrFirst];
        const GiTlasNode &right = state.tlas[node.leftOrFirst + 1];
        float             leftNear  = 0.0f;
        float             rightNear = 0.0f;
        const bool hitLeft  = slabTest(worldRay.origin, inverseDirection, left.boundsMin, left.boundsMax, bestT, leftNear);
        const bool hitRight = slabTest(worldRay.origin, inverseDirection, right.boundsMin, right.boundsMax, bestT, rightNear);
        if (hitLeft && hitRight)
        {
            // Farther child first (see traverseSurface); the pushes are
            // guarded the same way.
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

// ---- public trace entry points ---------------------------------------------

GiHit traceScene(const GiPipelineState &state, const GiRay &ray)
{
    const SurfaceHit surfaceHit = traceSurfaces<false>(state, ray, noSurface);
    GiHit            result;
    result.hit           = surfaceHit.hit;
    result.t             = surfaceHit.t;
    result.surfaceIndex  = surfaceHit.surfaceIndex;
    result.triangleIndex = surfaceHit.triangleIndex;
    if (!surfaceHit.hit)
    {
        return result;
    }
    const GiSurface   &surface  = state.surfaces[surfaceHit.surfaceIndex];
    const LocalShading shading  = localShadingAt(surface.geometry, surfaceHit.triangleIndex, surfaceHit.barycentric);
    result.position      = ray.origin + ray.direction * surfaceHit.t;
    result.localPosition = shading.position;
    result.localNormal   = shading.normal;
    result.normal        = glm::normalize(surface.normalMatrix * shading.normal);

    glm::vec3 geometricNormal = result.normal;
    if (shading.geometricLength > 1.0e-12f)
    {
        geometricNormal = glm::normalize(surface.normalMatrix * (shading.geometric / shading.geometricLength));
    }
    // Backface = the ray hits the back of the surface (both normals agree).
    // The geometric normal handed back always faces the ray, so consumers can
    // use it for biasing without caring about winding.
    result.backface = glm::dot(result.normal, ray.direction) > 0.0f && glm::dot(geometricNormal, ray.direction) > 0.0f;
    if (glm::dot(geometricNormal, ray.direction) > 0.0f)
    {
        geometricNormal = -geometricNormal;
    }
    result.geometricNormal = geometricNormal;
    return result;
}

// Lean variant of traceScene for the per-frame GI rays (see GiShadingHit):
// identical closest-hit selection and identical local position / normal
// interpolation, but the world-space normal chain is skipped and the
// backface classification runs on the same dots taken in LOCAL space.
//
// The sign of a world-space dot(normalMatrix * n, d) equals the sign of the
// local dot(n, inverseLinear * d): with normalMatrix = (L^-1)^T and
// inverseLinear = L^-1, dot((L^-1)^T n, d) = n . (L^-1 d) exactly, and
// normalizing the world normal never changes the sign of the dot. The
// degenerate-geometric fallback (use the smooth normal) mirrors traceScene,
// and card capture applies the same local-space rule.
GiShadingHit traceSceneShading(const GiPipelineState &state, const GiRay &ray)
{
    const SurfaceHit surfaceHit = traceSurfaces<false>(state, ray, noSurface);
    GiShadingHit     result;
    result.hit          = surfaceHit.hit;
    result.surfaceIndex = surfaceHit.surfaceIndex;
    result.t            = surfaceHit.t;
    if (!surfaceHit.hit)
    {
        result.t = 0.0f;
        return result;
    }
    const GiSurface   &surface  = state.surfaces[surfaceHit.surfaceIndex];
    const LocalShading shading  = localShadingAt(surface.geometry, surfaceHit.triangleIndex, surfaceHit.barycentric);
    result.localPosition = shading.position;
    result.localNormal   = shading.normal;

    const glm::vec3 localRayDirection
        = surface.localIsWorld ? ray.direction : surface.inverseLinear * ray.direction;
    result.backface = glm::dot(shading.normal, localRayDirection) > 0.0f
        && (shading.geometricLength <= 1.0e-12f || glm::dot(shading.geometric, localRayDirection) > 0.0f);
    return result;
}

// Transmittance of a shadow ray: transparent surfaces attenuate the
// remaining light by (1 - opacity) per covering hit instead of blocking it;
// the first opaque surface (or an exhausted transmission budget) blocks the
// rest. Each continuation advances past the covering hit by the trace bias
// so the same pane is not re-hit.
float traceSceneTransmittance(const GiPipelineState &state, const GiRay &ray)
{
    float transmittance = 1.0f;
    GiRay  remaining{ray.origin, ray.direction, ray.tMax};
    const std::uint32_t budget = state.settings.maxTransmissionSegments + 1u;
    for (std::uint32_t segment = 0; segment < budget; ++segment)
    {
        const SurfaceHit surfaceHit = traceSurfaces<false>(state, remaining, noSurface);
        if (!surfaceHit.hit)
        {
            return transmittance;
        }
        const GiSurface &surface = state.surfaces[surfaceHit.surfaceIndex];
        if (surface.opacity >= 1.0f)
        {
            return 0.0f;
        }
        transmittance *= 1.0f - glm::clamp(surface.opacity, 0.0f, 1.0f);
        if (transmittance <= 1.0e-4f)
        {
            return 0.0f;
        }
        const float step = surfaceHit.t + state.settings.traceBias;
        remaining.origin    = remaining.origin + remaining.direction * step;
        remaining.tMax      = remaining.tMax > step ? remaining.tMax - step : 0.0f;
        if (remaining.tMax <= 0.0f)
        {
            return transmittance;
        }
    }
    return 0.0f;
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
    // Registry descriptions are module-internal API input, not trusted data:
    // the vertex count is bounded before any allocation is sized from it, and
    // every attribute stride must hold its three floats while staying sane (an
    // interleaved vertex layout has no reason to jump pages between
    // attributes; a smaller stride would read past the attribute an index
    // addresses).
    if (description.vertexCount > (1u << 24))
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
    constexpr std::uint32_t minimumAttributeStrideBytes = 3 * sizeof(float);
    constexpr std::uint32_t maximumAttributeStrideBytes = 4096;
    if (description.positionStrideBytes % sizeof(float) != 0
        || (description.positionStrideBytes != 0
            && (description.positionStrideBytes < minimumAttributeStrideBytes
                || description.positionStrideBytes > maximumAttributeStrideBytes)))
    {
        fail(GiError::InvalidDescription);
        return;
    }
    const std::size_t positionStride = description.positionStrideBytes != 0
                                         ? description.positionStrideBytes / sizeof(float)
                                         : 3;
    // Stride 0 excludes the attribute (the documented contract): normals are
    // then generated from the triangle geometry and colors default to white.
    // Positions are required, so a zero position stride reads as tightly packed
    // float triplets instead.
    const bool hasNormals = description.normals != nullptr && description.normalStrideBytes != 0;
    const bool hasColors  = description.colors != nullptr && description.colorStrideBytes != 0;
    if (hasNormals && (description.normalStrideBytes % sizeof(float) != 0
            || description.normalStrideBytes < minimumAttributeStrideBytes
            || description.normalStrideBytes > maximumAttributeStrideBytes))
    {
        fail(GiError::InvalidDescription);
        return;
    }
    if (hasColors && (description.colorStrideBytes % sizeof(float) != 0
            || description.colorStrideBytes < minimumAttributeStrideBytes
            || description.colorStrideBytes > maximumAttributeStrideBytes))
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
    const std::size_t normalStride = description.normalStrideBytes != 0 ? description.normalStrideBytes / sizeof(float) : 3;
    const std::size_t colorStride  = description.colorStrideBytes != 0 ? description.colorStrideBytes / sizeof(float) : 3;
    if (hasNormals)
    {
        for (std::uint32_t vertex = 0; vertex < description.vertexCount; ++vertex)
        {
            const float *source = description.normals + static_cast<std::size_t>(vertex) * normalStride;
            if (!std::isfinite(source[0]) || !std::isfinite(source[1]) || !std::isfinite(source[2]))
            {
                fail(GiError::InvalidDescription);
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

namespace
{

float halfSurfaceArea(const glm::vec3 &boundsMin, const glm::vec3 &boundsMax)
{
    const glm::vec3 extent = glm::max(boundsMax - boundsMin, glm::vec3(0.0f));
    return extent.x * extent.y + extent.y * extent.z + extent.z * extent.x;
}

} // namespace

// Binned-SAH build (16 bins on the widest centroid axis). Falls back to a
// median split when the centroids are degenerate or a partition comes out
// one-sided, and to a hard median split past a depth limit so the traversal
// stack can never overflow. Triangles end up in `packed` in leaf order.
void buildSurfaceBvh(GiSurfaceGeometry &geometry)
{
    const std::uint32_t triangleCount = static_cast<std::uint32_t>(geometry.indices.size() / 3);
    geometry.bvh.clear();
    geometry.packed.clear();
    if (triangleCount == 0)
    {
        return;
    }
    std::vector<std::uint32_t> order(triangleCount);
    std::vector<glm::vec3>     centroids(triangleCount);
    std::vector<glm::vec3>     triangleMin(triangleCount);
    std::vector<glm::vec3>     triangleMax(triangleCount);
    glm::vec3                  globalMin(std::numeric_limits<float>::max());
    glm::vec3                  globalMax(-std::numeric_limits<float>::max());
    for (std::uint32_t triangle = 0; triangle < triangleCount; ++triangle)
    {
        order[triangle]        = triangle;
        const glm::vec3 &a     = geometry.positions[geometry.indices[triangle * 3 + 0]];
        const glm::vec3 &b     = geometry.positions[geometry.indices[triangle * 3 + 1]];
        const glm::vec3 &c     = geometry.positions[geometry.indices[triangle * 3 + 2]];
        triangleMin[triangle]  = glm::min(a, glm::min(b, c));
        triangleMax[triangle]  = glm::max(a, glm::max(b, c));
        centroids[triangle]    = (a + b + c) * (1.0f / 3.0f);
        globalMin              = glm::min(globalMin, triangleMin[triangle]);
        globalMax              = glm::max(globalMax, triangleMax[triangle]);
    }
    geometry.boundsMin = globalMin;
    geometry.boundsMax = globalMax;

    struct BuildEntry
    {
        std::uint32_t nodeIndex;
        std::uint32_t begin;
        std::uint32_t end;
        std::uint32_t depth;
    };
    constexpr std::uint32_t leafSize     = 4;
    constexpr std::uint32_t maxDepth     = 48;
    constexpr std::uint32_t binCount     = 16;
    geometry.bvh.reserve(2 * static_cast<std::size_t>(triangleCount) + 2);
    auto makeNode = [&](const glm::vec3 &boundsMin, const glm::vec3 &boundsMax) -> std::uint32_t
    {
        GiSurfaceGeometry::BvhNode node;
        node.boundsMin = boundsMin;
        node.boundsMax = boundsMax;
        geometry.bvh.push_back(node);
        return static_cast<std::uint32_t>(geometry.bvh.size() - 1);
    };
    std::vector<BuildEntry> stack;
    stack.push_back(BuildEntry{makeNode(globalMin, globalMax), 0, triangleCount, 0});
    while (!stack.empty())
    {
        const BuildEntry entry = stack.back();
        stack.pop_back();
        const std::uint32_t span = entry.end - entry.begin;
        auto makeLeaf = [&]()
        {
            GiSurfaceGeometry::BvhNode &node = geometry.bvh[entry.nodeIndex];
            node.leftOrFirst                 = entry.begin;
            node.count                       = span;
        };
        if (span <= leafSize)
        {
            makeLeaf();
            continue;
        }
        glm::vec3 centroidMin(std::numeric_limits<float>::max());
        glm::vec3 centroidMax(-std::numeric_limits<float>::max());
        for (std::uint32_t i = entry.begin; i < entry.end; ++i)
        {
            centroidMin = glm::min(centroidMin, centroids[order[i]]);
            centroidMax = glm::max(centroidMax, centroids[order[i]]);
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

        std::uint32_t middle     = entry.begin + span / 2;
        bool          partitioned = false;
        if (extent[axis] > 1.0e-9f && entry.depth < maxDepth)
        {
            struct Bin
            {
                glm::vec3     boundsMin{std::numeric_limits<float>::max()};
                glm::vec3     boundsMax{-std::numeric_limits<float>::max()};
                std::uint32_t count = 0;
            };
            std::array<Bin, binCount> bins{};
            const float               scale = static_cast<float>(binCount) * (1.0f - 1.0e-5f) / extent[axis];
            auto binOf = [&](std::uint32_t triangle)
            {
                return std::min(binCount - 1, static_cast<std::uint32_t>((centroids[triangle][axis] - centroidMin[axis]) * scale));
            };
            for (std::uint32_t i = entry.begin; i < entry.end; ++i)
            {
                Bin &bin = bins[binOf(order[i])];
                bin.boundsMin = glm::min(bin.boundsMin, triangleMin[order[i]]);
                bin.boundsMax = glm::max(bin.boundsMax, triangleMax[order[i]]);
                ++bin.count;
            }
            // Sweep: left areas/counts, then right, choosing the cheapest plane.
            std::array<float, binCount - 1>         leftCost{};
            std::array<std::uint32_t, binCount - 1> leftCount{};
            glm::vec3     runMin(std::numeric_limits<float>::max());
            glm::vec3     runMax(-std::numeric_limits<float>::max());
            std::uint32_t runCount = 0;
            for (std::uint32_t split = 0; split + 1 < binCount; ++split)
            {
                runMin = glm::min(runMin, bins[split].boundsMin);
                runMax = glm::max(runMax, bins[split].boundsMax);
                runCount += bins[split].count;
                leftCount[split] = runCount;
                leftCost[split]  = runCount > 0 ? halfSurfaceArea(runMin, runMax) * static_cast<float>(runCount) : 0.0f;
            }
            runMin   = glm::vec3(std::numeric_limits<float>::max());
            runMax   = glm::vec3(-std::numeric_limits<float>::max());
            runCount = 0;
            float         bestCost  = std::numeric_limits<float>::max();
            std::uint32_t bestSplit = 0;
            for (std::uint32_t split = binCount - 1; split > 0; --split)
            {
                runMin = glm::min(runMin, bins[split].boundsMin);
                runMax = glm::max(runMax, bins[split].boundsMax);
                runCount += bins[split].count;
                const std::uint32_t leftN = leftCount[split - 1];
                if (leftN == 0 || runCount == 0)
                {
                    continue;
                }
                const float cost = leftCost[split - 1] + halfSurfaceArea(runMin, runMax) * static_cast<float>(runCount);
                if (cost < bestCost)
                {
                    bestCost  = cost;
                    bestSplit = split - 1;
                }
            }
            if (bestCost < std::numeric_limits<float>::max())
            {
                const glm::vec3 nodeMin = geometry.bvh[entry.nodeIndex].boundsMin;
                const glm::vec3 nodeMax = geometry.bvh[entry.nodeIndex].boundsMax;
                const float parentArea  = std::max(halfSurfaceArea(nodeMin, nodeMax), 1.0e-12f);
                const float splitCost   = 0.5f + bestCost / parentArea;
                if (span <= 8 && static_cast<float>(span) <= splitCost)
                {
                    makeLeaf();
                    continue;
                }
                const auto pivot = std::partition(order.begin() + static_cast<std::ptrdiff_t>(entry.begin),
                    order.begin() + static_cast<std::ptrdiff_t>(entry.end),
                    [&](std::uint32_t triangle) { return binOf(triangle) <= bestSplit; });
                const std::uint32_t split = static_cast<std::uint32_t>(pivot - order.begin());
                if (split > entry.begin && split < entry.end)
                {
                    middle      = split;
                    partitioned = true;
                }
            }
        }
        if (!partitioned)
        {
            std::nth_element(order.begin() + static_cast<std::ptrdiff_t>(entry.begin),
                order.begin() + static_cast<std::ptrdiff_t>(middle),
                order.begin() + static_cast<std::ptrdiff_t>(entry.end),
                [&](std::uint32_t left, std::uint32_t right)
                { return centroids[left][axis] < centroids[right][axis]; });
        }
        glm::vec3 leftMin(std::numeric_limits<float>::max());
        glm::vec3 leftMax(-std::numeric_limits<float>::max());
        glm::vec3 rightMin(std::numeric_limits<float>::max());
        glm::vec3 rightMax(-std::numeric_limits<float>::max());
        for (std::uint32_t i = entry.begin; i < middle; ++i)
        {
            leftMin = glm::min(leftMin, triangleMin[order[i]]);
            leftMax = glm::max(leftMax, triangleMax[order[i]]);
        }
        for (std::uint32_t i = middle; i < entry.end; ++i)
        {
            rightMin = glm::min(rightMin, triangleMin[order[i]]);
            rightMax = glm::max(rightMax, triangleMax[order[i]]);
        }
        const std::uint32_t leftIndex  = makeNode(leftMin, leftMax);
        const std::uint32_t rightIndex = makeNode(rightMin, rightMax);
        geometry.bvh[entry.nodeIndex].leftOrFirst = leftIndex;
        geometry.bvh[entry.nodeIndex].count       = 0;
        stack.push_back(BuildEntry{leftIndex, entry.begin, middle, entry.depth + 1});
        stack.push_back(BuildEntry{rightIndex, middle, entry.end, entry.depth + 1});
    }

    geometry.packed.resize(triangleCount);
    for (std::uint32_t slot = 0; slot < triangleCount; ++slot)
    {
        const std::uint32_t triangle = order[slot];
        const glm::vec3    &a        = geometry.positions[geometry.indices[triangle * 3 + 0]];
        const glm::vec3    &b        = geometry.positions[geometry.indices[triangle * 3 + 1]];
        const glm::vec3    &c        = geometry.positions[geometry.indices[triangle * 3 + 2]];
        geometry.packed[slot]        = GiPackedTriangle{a, b - a, c - a, triangle};
    }
}

// ---- surface transforms --------------------------------------------------------

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

bool updateSurfaceTransform(GiSurface &surface, const glm::mat4 &transform)
{
    // Validate the derived matrices before committing anything: a singular
    // transform (zero scale, collapsed basis) produces a non-finite inverse,
    // and every slab test against NaN bounds fails closed, so the surface
    // would silently vanish from all rays. The caller keeps the previous
    // transform instead.
    const glm::mat4 inverse        = glm::inverse(transform);
    const glm::mat3 linear        = glm::mat3(transform);
    const glm::mat3 normalMatrix  = glm::inverseTranspose(linear);
    const auto      allFinite4    = [](const glm::mat4 &matrix)
    {
        const float *elements = glm::value_ptr(matrix);
        for (std::uint32_t element = 0; element < 16; ++element)
        {
            if (!std::isfinite(elements[element]))
            {
                return false;
            }
        }
        return true;
    };
    const auto allFinite3 = [](const glm::mat3 &matrix)
    {
        const float *elements = glm::value_ptr(matrix);
        for (std::uint32_t element = 0; element < 9; ++element)
        {
            if (!std::isfinite(elements[element]))
            {
                return false;
            }
        }
        return true;
    };
    if (!allFinite4(transform) || !allFinite4(inverse) || !allFinite3(normalMatrix))
    {
        return false;
    }
    surface.transform        = transform;
    surface.inverseTransform = inverse;
    surface.linear           = linear;
    surface.normalMatrix     = normalMatrix;
    // Cached transform pieces for the per-ray / per-texel hot paths: the
    // translation columns and the upper 3x3 of the inverse. A point through
    // inverseLinear + inverseTranslation performs the same three dot products
    // the 4x4-by-w=1 product performs, so results are identical while the
    // hot loops stop re-deriving a mat3 from the mat4 on every use.
    surface.translation         = glm::vec3(transform[3]);
    surface.inverseLinear      = glm::mat3(surface.inverseTransform);
    surface.inverseTranslation = glm::vec3(surface.inverseTransform[3]);
    // Exact-identity transform: local space coincides with world space, so
    // per-ray local transforms collapse to copies (a world-space ray is
    // already a valid local ray) and the local direction length is exactly
    // one. Static geometry registered in world space - the common case - then
    // pays neither the two mat3 products nor the length / division per ray.
    surface.localIsWorld = transform == glm::mat4(1.0f);
    surface.axisScale    = glm::vec3(glm::length(surface.linear[0]), glm::length(surface.linear[1]),
        glm::length(surface.linear[2]));
    updateSurfaceWorldBounds(surface);
    // The cards are local-space, so a moved surface keeps its captured data;
    // only the cached lighting is stale (new shadows / light angles). Mark it
    // for the front of the relight queue.
    for (GiCard &card : surface.cards)
    {
        card.lastDirectFrame   = 0;
        card.lastIndirectFrame = 0;
    }
    return true;
}

// ---- dynamic relight marking ---------------------------------------------

void markProbesNearVolume(GiPipelineState &state, const glm::vec3 &minBounds, const glm::vec3 &maxBounds)
{
    // Every probe whose lattice position lies inside the volume re-traces
    // from scratch on its next update (the planning pass serves fresh probes
    // first and probeUpdatesPerFrame bounds the work), so a moved, deformed,
    // created, or destroyed surface refreshes the world radiance cache
    // around it within a few frames instead of waiting out the rotating
    // stratum cycle.
    for (GiProbeLevel *level : {&state.level0, &state.level1})
    {
        for (GiProbeLevel::Probe &probe : level->probes)
        {
            if (probe.position.x >= minBounds.x && probe.position.x <= maxBounds.x
                && probe.position.y >= minBounds.y && probe.position.y <= maxBounds.y
                && probe.position.z >= minBounds.z && probe.position.z <= maxBounds.z)
            {
                probe.fresh = true;
            }
        }
    }
}

// ---- card placement ---------------------------------------------------------

void invalidateSurfaceCards(GiSurface &surface)
{
    for (GiCard &card : surface.cards)
    {
        card.captureCursor     = 0;
        card.captureComplete   = false;
        card.validTexels       = 0;
        card.directCursor      = 0;
        card.indirectCursor    = 0;
        card.lastCaptureFrame  = 0;
        card.lastDirectFrame   = 0;
        card.lastIndirectFrame = 0;
        // Until recaptured the card must not contribute stale coverage.
        std::fill(card.depthTexels.begin(), card.depthTexels.end(), invalidCardDepth);
        std::fill(card.directRadiance.begin(), card.directRadiance.end(), glm::vec3(0.0f));
        std::fill(card.indirectRadiance.begin(), card.indirectRadiance.end(), glm::vec3(0.0f));
        std::fill(card.indirectSamples.begin(), card.indirectSamples.end(), static_cast<std::uint8_t>(0));
    }
}

namespace
{

struct CardCandidate
{
    std::uint32_t axis     = 0;     // outward local axis
    std::uint32_t sign     = 0;     // 0 = negative face, 1 = positive face
    glm::vec3     origin{0.0f};
    glm::vec3     axisU{0.0f};
    glm::vec3     axisV{0.0f};
    glm::vec3     axisW{0.0f};
    float         width  = 0.0f;
    float         height = 0.0f;
    std::uint32_t texelWidth  = 0;
    std::uint32_t texelHeight = 0;
    bool          split       = false;
};

// Six face cards at the given texel density, with faces longer than the axis
// limit subdivided so every piece stays within it.
std::vector<CardCandidate> buildCardCandidates(const glm::vec3 &boundsMin, const glm::vec3 &boundsMax,
    const glm::vec3 &extent, const glm::vec3 &axisScale, float density, std::uint32_t maxAxisTexels)
{
    std::vector<CardCandidate> pieces;
    pieces.reserve(12);
    for (std::uint32_t axis = 0; axis < 3; ++axis)
    {
        const std::uint32_t uAxis = (axis + 1) % 3;
        const std::uint32_t vAxis = (axis + 2) % 3;
        for (std::uint32_t sign = 0; sign < 2; ++sign)
        {
            CardCandidate face;
            face.axis                 = axis;
            face.sign                 = sign;
            face.axisW[axis]          = sign == 0 ? -1.0f : 1.0f;
            face.axisU[uAxis]         = 1.0f;
            face.axisV[vAxis]         = 1.0f;
            face.origin[axis]         = sign == 0 ? boundsMin[axis] : boundsMax[axis];
            face.origin[uAxis]        = 0.5f * (boundsMin[uAxis] + boundsMax[uAxis]);
            face.origin[vAxis]        = 0.5f * (boundsMin[vAxis] + boundsMax[vAxis]);
            face.width                = extent[uAxis];
            face.height               = extent[vAxis];
            // Texel counts follow the WORLD size of the face so a scaled
            // instance keeps a consistent density. The count is clamped in
            // the float domain before the integer conversion: an extreme
            // extent or instance scale would otherwise overflow the cast.
            const float worldWidth    = extent[uAxis] * axisScale[uAxis];
            const float worldHeight   = extent[vAxis] * axisScale[vAxis];
            face.texelWidth  = static_cast<std::uint32_t>(
                std::clamp(std::ceil(worldWidth * density), 4.0f, 4096.0f));
            face.texelHeight = static_cast<std::uint32_t>(
                std::clamp(std::ceil(worldHeight * density), 4.0f, 4096.0f));

            std::uint32_t splitU = 1;
            std::uint32_t splitV = 1;
            while (face.texelWidth / splitU > maxAxisTexels)
            {
                ++splitU;
            }
            while (face.texelHeight / splitV > maxAxisTexels)
            {
                ++splitV;
            }
            if (splitU * splitV == 1)
            {
                pieces.push_back(face);
                continue;
            }
            for (std::uint32_t v = 0; v < splitV; ++v)
            {
                for (std::uint32_t u = 0; u < splitU; ++u)
                {
                    CardCandidate piece = face;
                    piece.split         = true;
                    piece.width         = face.width / static_cast<float>(splitU);
                    piece.height        = face.height / static_cast<float>(splitV);
                    const float uOffset = (-0.5f + (static_cast<float>(u) + 0.5f) / static_cast<float>(splitU)) * face.width;
                    const float vOffset = (-0.5f + (static_cast<float>(v) + 0.5f) / static_cast<float>(splitV)) * face.height;
                    piece.origin += face.axisU * uOffset + face.axisV * vOffset;
                    piece.texelWidth = static_cast<std::uint32_t>(std::clamp(
                        std::ceil(piece.width * axisScale[uAxis] * density), 4.0f, static_cast<float>(maxAxisTexels)));
                    piece.texelHeight = static_cast<std::uint32_t>(std::clamp(
                        std::ceil(piece.height * axisScale[vAxis] * density), 4.0f, static_cast<float>(maxAxisTexels)));
                    pieces.push_back(piece);
                }
            }
        }
    }
    return pieces;
}

} // namespace

// Places the surface's cards as axis-aligned ortho projections of its LOCAL
// bounds: six face cards, with big faces subdivided when a card would exceed
// maxTexelsPerCardAxis texels along an axis. This is Lumen's documented
// fallback placement. When the pieces would exceed cards.per_surface_max the
// texel density is lowered until everything fits, so every face stays covered
// (dropping whole pieces would leave large parts of a big surface - often the
// visible top face - without any cache).
void placeSurfaceCards(const GiSettings &settings, GiSurface &surface)
{
    surface.cards.clear();
    if (surface.excluded || surface.geometry.bvh.empty())
    {
        return;
    }
    const glm::vec3 boundsMin = surface.geometry.boundsMin;
    const glm::vec3 boundsMax = surface.geometry.boundsMax;
    const float     largestExtent = std::max({boundsMax.x - boundsMin.x, boundsMax.y - boundsMin.y,
        boundsMax.z - boundsMin.z, 1.0e-3f});
    const glm::vec3 extent = glm::max(boundsMax - boundsMin, glm::vec3(largestExtent * 1.0e-3f));
    const glm::vec3 axisScale = glm::max(surface.axisScale, glm::vec3(1.0e-4f));
    const std::uint32_t maxAxisTexels = std::max(settings.maxTexelsPerCardAxis, 4u);
    const std::uint32_t maxCards      = std::max(settings.maxCardsPerSurface, 1u);
    const std::uint32_t layers        = std::clamp(settings.cardDepthLayers, 1u, maxCardLayers);

    float density = settings.texelsPerMeter;
    std::vector<CardCandidate> pieces = buildCardCandidates(boundsMin, boundsMax, extent, axisScale, density, maxAxisTexels);
    for (std::uint32_t attempt = 0; attempt < 24 && pieces.size() > maxCards && density > 0.05f; ++attempt)
    {
        density *= 0.75f;
        pieces = buildCardCandidates(boundsMin, boundsMax, extent, axisScale, density, maxAxisTexels);
    }
    if (pieces.size() > maxCards)
    {
        // Fewer cards than faces allowed: keep the largest.
        std::stable_sort(pieces.begin(), pieces.end(),
            [](const CardCandidate &left, const CardCandidate &right)
            {
                return static_cast<std::uint64_t>(left.texelWidth) * left.texelHeight
                     > static_cast<std::uint64_t>(right.texelWidth) * right.texelHeight;
            });
        pieces.resize(maxCards);
    }

    surface.cards.reserve(pieces.size());
    for (const CardCandidate &candidate : pieces)
    {
        GiCard card;
        card.origin       = candidate.origin;
        card.axisU        = candidate.axisU;
        card.axisV        = candidate.axisV;
        card.axisW        = candidate.axisW;
        card.width        = candidate.width;
        card.height       = candidate.height;
        card.axisIndex    = candidate.axis;
        card.nearBias     = std::max(largestExtent * 1.0e-3f, 1.0e-5f);
        card.depth        = extent[candidate.axis] + 2.0f * card.nearBias;
        card.texelWidth   = candidate.texelWidth;
        card.texelHeight  = candidate.texelHeight;
        card.layers       = layers;
        const std::size_t texels = card.totalTexels();
        card.depthTexels.assign(texels, invalidCardDepth);
        card.albedo.assign(texels, glm::vec3(0.0f));
        card.normal.assign(texels, glm::vec3(0.0f, 1.0f, 0.0f));
        card.directRadiance.assign(texels, glm::vec3(0.0f));
        card.indirectRadiance.assign(texels, glm::vec3(0.0f));
        card.indirectSamples.assign(texels, static_cast<std::uint8_t>(0));
        surface.cards.push_back(std::move(card));
    }
}

void rebuildCardRefs(GiPipelineState &state)
{
    state.cardRefs.clear();
    for (std::uint32_t surfaceIndex = 0; surfaceIndex < state.surfaces.size(); ++surfaceIndex)
    {
        const GiSurface &surface = state.surfaces[surfaceIndex];
        for (std::uint32_t cardIndex = 0; cardIndex < surface.cards.size(); ++cardIndex)
        {
            state.cardRefs.push_back(GiCardRef{surfaceIndex, cardIndex});
        }
    }
}

// ---- top-level BVH ------------------------------------------------------------

void ensureSceneTlas(GiPipelineState &state)
{
    if (!state.tlasDirty)
    {
        return;
    }
    // Built into locals and swapped in on success: a failed allocation half
    // way must leave the previous tree in place (with the dirty flag still
    // set) instead of an empty TLAS that silently swallows every ray.
    std::vector<GiTlasNode>    tlas;
    std::vector<std::uint32_t> tlasSurfaces;
    for (std::uint32_t index = 0; index < state.surfaces.size(); ++index)
    {
        const GiSurface &surface = state.surfaces[index];
        if (!surface.excluded && !surface.geometry.bvh.empty())
        {
            tlasSurfaces.push_back(index);
        }
    }
    if (!tlasSurfaces.empty())
    {
        struct BuildEntry
        {
            std::uint32_t nodeIndex;
            std::uint32_t begin;
            std::uint32_t end;
        };
        auto boundsOf = [&](std::uint32_t begin, std::uint32_t end, glm::vec3 &outMin, glm::vec3 &outMax)
        {
            outMin = glm::vec3(std::numeric_limits<float>::max());
            outMax = glm::vec3(-std::numeric_limits<float>::max());
            for (std::uint32_t i = begin; i < end; ++i)
            {
                const GiSurface &surface = state.surfaces[tlasSurfaces[i]];
                outMin                   = glm::min(outMin, surface.worldMin);
                outMax                   = glm::max(outMax, surface.worldMax);
            }
        };
        auto makeNode = [&](std::uint32_t begin, std::uint32_t end) -> std::uint32_t
        {
            GiTlasNode node;
            boundsOf(begin, end, node.boundsMin, node.boundsMax);
            tlas.push_back(node);
            return static_cast<std::uint32_t>(tlas.size() - 1);
        };
        std::vector<BuildEntry> stack;
        stack.push_back(BuildEntry{makeNode(0, static_cast<std::uint32_t>(tlasSurfaces.size())), 0,
            static_cast<std::uint32_t>(tlasSurfaces.size())});
        while (!stack.empty())
        {
            const BuildEntry entry = stack.back();
            stack.pop_back();
            const std::uint32_t span = entry.end - entry.begin;
            if (span <= 2)
            {
                tlas[entry.nodeIndex].leftOrFirst = entry.begin;
                tlas[entry.nodeIndex].count       = span;
                continue;
            }
            const glm::vec3 extent = tlas[entry.nodeIndex].boundsMax - tlas[entry.nodeIndex].boundsMin;
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
            std::nth_element(tlasSurfaces.begin() + static_cast<std::ptrdiff_t>(entry.begin),
                tlasSurfaces.begin() + static_cast<std::ptrdiff_t>(middle),
                tlasSurfaces.begin() + static_cast<std::ptrdiff_t>(entry.end),
                [&](std::uint32_t left, std::uint32_t right)
                {
                    const GiSurface &a = state.surfaces[left];
                    const GiSurface &b = state.surfaces[right];
                    return a.worldMin[axis] + a.worldMax[axis] < b.worldMin[axis] + b.worldMax[axis];
                });
            const std::uint32_t leftIndex  = makeNode(entry.begin, middle);
            const std::uint32_t rightIndex = makeNode(middle, entry.end);
            tlas[entry.nodeIndex].leftOrFirst = leftIndex;
            tlas[entry.nodeIndex].count       = 0;
            stack.push_back(BuildEntry{leftIndex, entry.begin, middle});
            stack.push_back(BuildEntry{rightIndex, middle, entry.end});
        }
    }
    state.tlas         = std::move(tlas);
    state.tlasSurfaces = std::move(tlasSurfaces);
    state.tlasDirty    = false;
}

// ---- probe lattice -----------------------------------------------------------

namespace
{

// Fills a resolution^2 octahedral table: unit directions at texel centers,
// the exact solid angle of every texel (normalized to sum to 4 pi), and the
// per-direction SH2 projection basis values (normalization constant squared
// times the basis polynomial - the stored coefficient folds the basis, see
// projectRadianceToSh2). The octahedron-plane element maps to the sphere with
// dOmega = dA / |p|^3, so texels near the diagonals cover less solid angle
// than the ones at the axes; using a flat 4 pi / N weight (as before) skews
// every SH projection.
void fillOctahedralTable(std::uint32_t resolution, std::vector<glm::vec3> &directions, std::vector<float> &weights,
    std::vector<float> &basis)
{
    constexpr float kConstants[9] = {0.282094792f, 0.488602512f, 0.488602512f, 0.488602512f,
        1.092548431f, 1.092548431f, 0.315391565f, 1.092548431f, 0.546274215f};
    directions.clear();
    weights.clear();
    basis.clear();
    const std::size_t count = static_cast<std::size_t>(resolution) * resolution;
    directions.reserve(count);
    weights.reserve(count);
    basis.reserve(count * 9);
    double total = 0.0;
    for (std::uint32_t y = 0; y < resolution; ++y)
    {
        for (std::uint32_t x = 0; x < resolution; ++x)
        {
            const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(resolution) * 2.0f - 1.0f;
            const float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(resolution) * 2.0f - 1.0f;
            glm::vec3   p(u, v, 1.0f - std::fabs(u) - std::fabs(v));
            if (p.z < 0.0f)
            {
                p.x = (1.0f - std::fabs(v)) * (u >= 0.0f ? 1.0f : -1.0f);
                p.y = (1.0f - std::fabs(u)) * (v >= 0.0f ? 1.0f : -1.0f);
            }
            const float length = std::max(glm::length(p), 1.0e-8f);
            const glm::vec3 direction = p / length;
            directions.push_back(direction);
            const float weight = 1.0f / (length * length * length);
            weights.push_back(weight);
            total += weight;
            const float dx = direction.x;
            const float dy = direction.y;
            const float dz = direction.z;
            const float polynomial[9] = {1.0f, dy, dz, dx, dx * dy, dy * dz, 3.0f * dz * dz - 1.0f, dx * dz, dx * dx - dy * dy};
            for (std::uint32_t coefficient = 0; coefficient < 9; ++coefficient)
            {
                basis.push_back(kConstants[coefficient] * polynomial[coefficient] * kConstants[coefficient]);
            }
        }
    }
    const float normalization = total > 0.0 ? static_cast<float>(4.0 * 3.14159265358979 / total) : 0.0f;
    for (float &weight : weights)
    {
        weight *= normalization;
    }
}

} // namespace

void buildProbeDirectionTable(GiPipelineState &state)
{
    const std::uint32_t resolution = std::clamp(state.settings.probeDirectionResolution,
        minProbeDirectionResolution, maxProbeDirectionResolution);
    fillOctahedralTable(resolution, state.probeDirections, state.probeDirectionWeights, state.probeDirectionBasis);
    state.settings.probeDirectionResolution = resolution;
    // Screen probes: fixed 8x8 octahedral map (Lumen's screen probe directional
    // resolution).
    fillOctahedralTable(screenProbeResolution, state.screenProbeDirections, state.screenProbeDirectionWeights,
        state.screenProbeDirectionBasis);
}

void buildProbeLevels(GiPipelineState &state)
{
    // Two camera-centered clip levels of 9^3 probes each (an odd dimension
    // keeps a probe exactly on the camera axis). Level 0 covers the near field
    // at the configured spacing; level 1 uses a coarser spacing
    // (probes.level1_spacing_factor x) and so covers that many times the
    // extent. The dimension is fixed by the renderer's probe buffer.
    auto buildLevel = [&](GiProbeLevel &level, float spacing)
    {
        level.spacing    = std::max(spacing, 0.25f);
        level.dimensions = {probeGridDimension, probeGridDimension, probeGridDimension};
        level.probes.assign(
            static_cast<std::size_t>(probeGridDimension) * probeGridDimension * probeGridDimension, GiProbeLevel::Probe{});
        level.updateCursor = 0;
        level.originCell   = glm::ivec3(0);
        for (GiProbeLevel::Probe &probe : level.probes)
        {
            probe.radiance.assign(state.probeDirections.size(), glm::vec3(0.0f));
            // Sentinel: no lattice cell assigned yet (forces the first snap to
            // initialize every slot).
            probe.cell = glm::ivec3(std::numeric_limits<std::int32_t>::min());
        }
    };
    buildLevel(state.level0, state.settings.probeSpacing);
    buildLevel(state.level1, state.settings.probeSpacing * std::max(state.settings.probeLevel1SpacingFactor, 2.0f));
}

void snapProbeLevels(GiPipelineState &state, const glm::vec3 &cameraPosition)
{
    auto snapLevel = [&](GiProbeLevel &level)
    {
        if (level.probes.empty())
        {
            return;
        }
        const std::int32_t dim  = static_cast<std::int32_t>(level.dimensions[0]);
        const std::int32_t half = dim / 2;
        // The scaled camera is clamped into int32 range before the cell
        // conversion: a camera millions of spacing units out still scrolls
        // the toroidal lattice correctly, but the raw floor would overflow.
        const float maxCell    = static_cast<float>(std::numeric_limits<std::int32_t>::max() / 4);
        const glm::vec3 scaled = glm::clamp(cameraPosition / level.spacing, glm::vec3(-maxCell), glm::vec3(maxCell));
        const glm::ivec3   origin = glm::ivec3(glm::floor(scaled)) - glm::ivec3(half);
        level.originCell = origin;
        for (std::int32_t z = 0; z < dim; ++z)
        {
            for (std::int32_t y = 0; y < dim; ++y)
            {
                for (std::int32_t x = 0; x < dim; ++x)
                {
                    const glm::ivec3 cell = origin + glm::ivec3(x, y, z);
                    GiProbeLevel::Probe &probe = level.probes[level.slotOfCell(cell)];
                    if (probe.cell != cell)
                    {
                        // The slot now holds a different lattice cell (it
                        // scrolled in): its old radiance describes another
                        // place, so drop it and let the amortized updates
                        // repopulate it.
                        probe.cell       = cell;
                        probe.position   = glm::vec3(cell) * level.spacing;
                        probe.offset     = glm::vec3(0.0f);
                        probe.validity   = 0.0f;
                        probe.fresh      = true;
                        probe.enclosed   = false;
                        probe.phase      = 0;
                        probe.lastUpdate = 0;
                        probe.sh2.fill(0.0f);
                        std::fill(probe.radiance.begin(), probe.radiance.end(), glm::vec3(0.0f));
                    }
                }
            }
        }
    };
    snapLevel(state.level0);
    snapLevel(state.level1);
}

void buildProbeSnapshot(const GiPipelineState &state, GiProbeSnapshot &snapshot)
{
    snapshot.probes.clear();
    GiProbeHeader &header = snapshot.header;
    header                = GiProbeHeader{};
    if (state.level0.probes.empty() || state.level1.probes.empty())
    {
        // Levels not built yet (no configuration applied): export the empty
        // header rather than touching the lattice math.
        return;
    }
    header.gridCenter[0]  = state.cameraPosition.x;
    header.gridCenter[1]  = state.cameraPosition.y;
    header.gridCenter[2]  = state.cameraPosition.z;
    header.level0Spacing  = state.level0.spacing;
    header.level1Spacing  = state.level1.spacing;
    for (std::uint32_t axis = 0; axis < 3; ++axis)
    {
        header.level0Dimensions[axis] = state.level0.dimensions[axis];
        header.level1Dimensions[axis] = state.level1.dimensions[axis];
    }
    header.directionResolution = state.settings.probeDirectionResolution;
    const std::uint32_t level0Count = static_cast<std::uint32_t>(state.level0.probes.size());
    const std::uint32_t level1Count = static_cast<std::uint32_t>(state.level1.probes.size());
    header.level0ProbeOffset = 0;
    header.level1ProbeOffset = level0Count;
    header.probeCount        = level0Count + level1Count;
    snapshot.probes.reserve(header.probeCount);
    // Export in logical lattice order (x fastest, then y, then z): the
    // renderer indexes the grid as x + dim * (y + dim * z) from probe 0's
    // position, independent of the toroidal storage order.
    auto appendLevel = [&](const GiProbeLevel &level, std::uint32_t levelIndex)
    {
        const std::uint32_t dim = level.dimensions[0];
        for (std::uint32_t z = 0; z < dim; ++z)
        {
            for (std::uint32_t y = 0; y < dim; ++y)
            {
                for (std::uint32_t x = 0; x < dim; ++x)
                {
                    const GiProbeLevel::Probe &probe = level.probes[level.slotOfLogical(x, y, z)];
                    GiProbe                    target;
                    const glm::vec3 position = glm::vec3(level.originCell + glm::ivec3(static_cast<int>(x),
                                                   static_cast<int>(y), static_cast<int>(z)))
                        * level.spacing;
                    target.position[0] = position.x;
                    target.position[1] = position.y;
                    target.position[2] = position.z;
                    for (std::uint32_t coefficient = 0; coefficient < 27; ++coefficient)
                    {
                        target.sh2[coefficient / 3][coefficient % 3] = probe.sh2[coefficient];
                    }
                    target.validity = (probe.cell == level.originCell + glm::ivec3(static_cast<int>(x),
                                           static_cast<int>(y), static_cast<int>(z)))
                        ? probe.validity
                        : 0.0f;
                    target.level    = levelIndex;
                    snapshot.probes.push_back(target);
                }
            }
        }
    };
    appendLevel(state.level0, 0);
    appendLevel(state.level1, 1);
}

// ---- card capture --------------------------------------------------------------

// Local-space lit point of one card texel with the (x, y) cell already known
// (the lighting loops track the cell incrementally, so the hot path never
// pays the flat-index divisions). Same arithmetic as the flat-index variant.
glm::vec3 cardTexelLocalPositionXY(const GiCard &card, std::uint32_t x, std::uint32_t y, std::size_t texel)
{
    const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(card.texelWidth) - 0.5f;
    const float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(card.texelHeight) - 0.5f;
    const glm::vec3 planePoint = card.origin + card.axisU * (u * card.width) + card.axisV * (v * card.height);
    // The lit point is the actual surface behind the plane (depth below it),
    // not the plane point: for surfaces inset from the card plane the plane
    // point lies in free space or inside another object.
    return planePoint - card.axisW * card.depthTexels[texel];
}

// Captures xy texels [texelBegin, texelEnd) of one card, all depth layers: a
// local-space ortho ray per texel keeps the first `layers` front-facing hits
// (faces whose normal points toward the card). Backfaces are stepped through,
// which is what lets a room modelled as one mesh keep its floor behind the
// ceiling's top face.
void captureCardTexels(const GiSurface &surface, GiCard &card, std::uint32_t texelBegin, std::uint32_t texelEnd)
{
    if (surface.excluded)
    {
        return;
    }
    const GiSurfaceGeometry &geometry = surface.geometry;
    const std::uint32_t texelWidth    = card.texelWidth;
    const std::uint32_t plane         = static_cast<std::uint32_t>(card.planeTexels());
    const glm::vec3     inward        = -card.axisW;
    const float         rayLength     = card.depth + 2.0f * card.nearBias;
    const float         restart       = std::max(card.nearBias, 1.0e-5f);
    // Every capture ray shares the ortho direction, so its inverse (the BVH
    // slab input) is computed once per card instead of once per ray.
    const glm::vec3     inverseInward = safeInverseDirection(inward);
    for (std::uint32_t texel = texelBegin; texel < texelEnd && texel < plane; ++texel)
    {
        const std::uint32_t x = texel % texelWidth;
        const std::uint32_t y = texel / texelWidth;
        const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(texelWidth) - 0.5f;
        const float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(card.texelHeight) - 0.5f;
        const glm::vec3 planePoint = card.origin + card.axisU * (u * card.width) + card.axisV * (v * card.height);
        const glm::vec3 start      = planePoint + card.axisW * card.nearBias;

        std::uint32_t layer   = 0;
        float         offset  = 0.0f;
        // The step loop always advances (every hit moves the origin at least
        // one epsilon plus the restart bias), so it terminates on its own;
        // the iteration bound only keeps stacked coplanar backfaces - a
        // common baked-model pathology - from spending hundreds of
        // traversals per texel. Layers left over stay invalid.
        const std::uint32_t stepLimit = 64u + 8u * card.layers;
        std::uint32_t       steps     = 0;
        while (layer < card.layers && steps++ < stepLimit)
        {
            GiRay         ray{start + inward * offset, inward, rayLength - offset};
            if (ray.tMax <= 0.0f)
            {
                break;
            }
            std::uint32_t triangle = 0;
            glm::vec3     barycentric{};
            if (!traverseSurface<false>(geometry, ray, inverseInward, triangle, barycentric))
            {
                break;
            }
            const float hitDistance = offset + ray.tMax;
            offset                  = hitDistance + restart;

            const std::uint32_t i0 = geometry.indices[triangle * 3 + 0];
            const std::uint32_t i1 = geometry.indices[triangle * 3 + 1];
            const std::uint32_t i2 = geometry.indices[triangle * 3 + 2];
            glm::vec3 smoothNormal = geometry.normals[i0] * barycentric.x + geometry.normals[i1] * barycentric.y
                                   + geometry.normals[i2] * barycentric.z;
            const glm::vec3 geometric = glm::cross(
                geometry.positions[i1] - geometry.positions[i0], geometry.positions[i2] - geometry.positions[i0]);
            const float smoothLength    = glm::length(smoothNormal);
            const float geometricLength = glm::length(geometric);
            smoothNormal = smoothLength > 1.0e-8f
                ? smoothNormal / smoothLength
                : (geometricLength > 1.0e-12f ? geometric / geometricLength : glm::vec3(0.0f, 1.0f, 0.0f));
            // Same rule as traceScene(): a hit is a backface only when the
            // smooth and the geometric normal both face along the ray.
            const bool backface = glm::dot(smoothNormal, inward) > 0.0f
                && (geometricLength <= 1.0e-12f || glm::dot(geometric, inward) > 0.0f);
            if (backface)
            {
                continue; // step through: a nearer back face hides nothing useful
            }
            const std::size_t slot = static_cast<std::size_t>(layer) * plane + texel;
            card.depthTexels[slot] = std::max(hitDistance - card.nearBias, 0.0f);
            card.normal[slot]      = smoothNormal;
            card.albedo[slot]      = (geometry.colors[i0] * barycentric.x + geometry.colors[i1] * barycentric.y
                                         + geometry.colors[i2] * barycentric.z)
                * surface.albedo;
            ++layer;
        }
        for (; layer < card.layers; ++layer)
        {
            const std::size_t slot = static_cast<std::size_t>(layer) * plane + texel;
            card.depthTexels[slot] = invalidCardDepth;
            card.albedo[slot]      = glm::vec3(0.0f);
        }
    }
}

} // namespace lumen_gi
} // namespace engine
