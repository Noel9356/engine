// types.hpp - shared value types for the spatial_index module.
//
// Handles, hit records, and query counters used by the static BVH, the dynamic
// AABB tree, and the combined scene index. Header-only, no dependencies.

#ifndef ENGINE_SPATIAL_INDEX_TYPES_HPP
#define ENGINE_SPATIAL_INDEX_TYPES_HPP

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace engine::spatial
{

// Sentinel node index inside the trees (also used for "no leaf" in item maps).
inline constexpr std::uint32_t invalidNode = std::numeric_limits<std::uint32_t>::max();
// Opaque proxy id for dynamic-tree entries: generation in the high 32 bits,
// (slot + 1) in the low 32 bits, so stale handles fail to resolve.
using ProxyHandle                         = std::uint64_t;
inline constexpr ProxyHandle invalidProxy = 0;

// World-space ray. Direction need not be normalized; ray intersections report
// physical distance along its normalized direction. A positive infinity maximum
// distance means unbounded. Zero-length and non-finite directions are rejected.
struct Ray3
{
    std::array<float, 3> origin{};
    std::array<float, 3> direction{0.0f, 0.0f, -1.0f};
    float maxDistance = std::numeric_limits<float>::infinity();
};


// Inward-facing plane in the convention dot(normal, point) + distance >= 0
// for points inside a volume. Normal need not be normalized.
struct Plane3
{
    std::array<float, 3> normal{0.0f, 0.0f, 1.0f};
    float distance = 0.0f;
};

// Six-plane view frustum. Plane order is left, right, bottom, top, near, far;
// callers may use any consistent ordering because each plane is tested.
struct Frustum3
{
    std::array<Plane3, 6> planes{};

    [[nodiscard]] bool valid() const noexcept
    {
        for (const Plane3 &plane : planes)
        {
            if (!std::isfinite(plane.normal[0]) || !std::isfinite(plane.normal[1])
                || !std::isfinite(plane.normal[2]) || !std::isfinite(plane.distance))
                return false;
            const double lengthSquared = static_cast<double>(plane.normal[0]) * plane.normal[0]
                + static_cast<double>(plane.normal[1]) * plane.normal[1]
                + static_cast<double>(plane.normal[2]) * plane.normal[2];
            if (!std::isfinite(lengthSquared))
                return false;
            if (lengthSquared <= std::numeric_limits<double>::min() && plane.distance < 0.0f)
                return false;
        }
        return true;
    }
};

// Per-query traversal counters returned alongside results (profiling/telemetry).
struct QueryStatistics
{
    std::uint64_t nodesVisited  = 0; // Internal nodes + leaves examined.
    std::uint64_t leavesVisited = 0; // Leaf entries (items/proxies) tested.
    std::uint64_t hits          = 0; // Entries appended to the output vector.
};

// One query match from the dynamic tree: the proxy handle plus the payload
// supplied at insert time.
struct SpatialHit
{
    ProxyHandle   handle       = invalidProxy;
    std::uint64_t userData     = 0;
    std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max();
};

// Dynamic-tree ray hit. Distance is measured in world units from ray.origin.
struct DynamicRayHit
{
    ProxyHandle   handle       = invalidProxy;
    std::uint64_t userData     = 0;
    std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max();
    float         distance     = 0.0f;
};

// Static-BVH ray hit; itemIndex refers to the original build() input order.
struct StaticRayHit
{
    std::uint32_t itemIndex    = invalidNode;
    std::uint64_t userData     = 0;
    std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max();
    float         distance     = 0.0f;
};

// Tree health snapshot for the dynamic tree (see DynamicAabbTree::statistics);
// used to decide when a full rebuild is worthwhile.
struct DynamicTreeStatistics
{
    std::uint64_t nodesVisited     = 0;   // Total nodes walked by statistics().
    std::uint64_t leaves           = 0;   // Number of proxies (leaf nodes).
    std::uint32_t height           = 0;   // Height of the tree (edges from root to leaf).
    std::uint32_t maximumBalance   = 0;   // Largest |height(left) - height(right)| seen.
    double        surfaceAreaRatio = 0.0; // Sum of node areas / root area; ~1 is ideal.
};

} // namespace engine::spatial

#endif
