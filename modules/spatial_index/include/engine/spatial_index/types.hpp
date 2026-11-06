// types.hpp - shared value types for the spatial_index module.
//
// Handles, hit records, and query counters used by the static BVH, the dynamic
// AABB tree, and the combined scene index. Header-only, no dependencies.

#ifndef ENGINE_SPATIAL_INDEX_TYPES_HPP
#define ENGINE_SPATIAL_INDEX_TYPES_HPP

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
