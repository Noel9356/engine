// scene_spatial_index.hpp - combined static/dynamic spatial index for scenes.
//
// Pairs a StaticBvh (immutable structure, refit-only) with a DynamicAabbTree
// (moving objects) and merges their query results into one hit list tagged by
// SceneMobility. Header-only.

#ifndef ENGINE_SPATIAL_INDEX_SCENE_SPATIAL_INDEX_HPP
#define ENGINE_SPATIAL_INDEX_SCENE_SPATIAL_INDEX_HPP

#include <engine/spatial_index/dynamic_aabb_tree.hpp>
#include <engine/spatial_index/static_bvh.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>

namespace engine::spatial
{

// Which subtree a SceneSpatialHit came from (and which id field is meaningful).
enum class SceneMobility : std::uint8_t
{
    Static, // Hit from the static BVH: read staticIndex.
    Dynamic // Hit from the dynamic tree: read dynamicHandle.
};

// One query match. For Static hits staticIndex indexes the items given to
// buildStatic; for Dynamic hits dynamicHandle is the proxy handle.
struct SceneSpatialHit
{
    SceneMobility mobility      = SceneMobility::Static;
    std::uint32_t staticIndex   = invalidNode;
    ProxyHandle   dynamicHandle = invalidProxy;
    std::uint64_t userData      = 0;
    std::uint64_t categoryMask  = std::numeric_limits<std::uint64_t>::max();
};

// Scene-level ray hit. staticIndex is meaningful for Static; dynamicHandle
// is meaningful for Dynamic. Distance is measured in world units.
struct SceneSpatialRayHit
{
    SceneMobility mobility = SceneMobility::Static;
    std::uint32_t staticIndex = invalidNode;
    ProxyHandle dynamicHandle = invalidProxy;
    std::uint64_t userData = 0;
    std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max();
    float distance = 0.0f;
};

// Caller-owned temporary storage reused across queries to avoid per-query
// allocations; it holds no data between calls and is reset by every query.
struct SceneSpatialQueryScratch
{
    std::vector<std::uint32_t> staticIndices;
    std::vector<SpatialHit>    dynamicHits;

    void reserve(std::size_t staticCount, std::size_t dynamicCount)
    {
        staticIndices.reserve(staticCount);
        dynamicHits.reserve(dynamicCount);
    }
};

// Caller-owned ray-query buffers; reserve to the expected number of hits to
// keep steady-state queries allocation-free.
struct SceneSpatialRayScratch
{
    std::vector<StaticRayHit> staticHits;
    std::vector<DynamicRayHit> dynamicHits;

    void reserve(std::size_t staticCount, std::size_t dynamicCount)
    {
        staticHits.reserve(staticCount);
        dynamicHits.reserve(dynamicCount);
    }
};

// Facade over StaticBvh + DynamicAabbTree. Static geometry is built once (and
// only refit afterwards); dynamic proxies are inserted/updated/removed freely.
// Not internally synchronized.
class SceneSpatialIndex final
{
public:
    // Dynamic side tuning is delegated to the dynamic tree options.
    explicit SceneSpatialIndex(const DynamicAabbTreeOptions &dynamicOptions = {}) noexcept
        : dynamic_(dynamicOptions)
    {}

    // (Re)builds the static half from full items; see StaticBvh::build.
    bool buildStatic(std::span<const SpatialItem> items, const StaticBvhBuildOptions &options = {})
    {
        return static_.build(items, options);
    }

    // Static refits: full-tree (refitStatic) or per-item partial (refitStaticItems).
    bool refitStatic(std::span<const Aabb> bounds) noexcept
    {
        return static_.refit(bounds);
    }
    bool refitStaticItems(std::span<const SpatialItemUpdate> updates) noexcept
    {
        return static_.refitItems(updates);
    }

    // Dynamic half: thin wrappers over the DynamicAabbTree equivalents.
    ProxyHandle insertDynamic(const Aabb &bounds, std::uint64_t userData,
        std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max())
    {
        return dynamic_.insert(bounds, userData, categoryMask);
    }

    bool updateDynamic(ProxyHandle handle, const Aabb &bounds, const std::array<float, 3> &displacement = {})
    {
        return dynamic_.update(handle, bounds, displacement);
    }

    bool updateDynamicMany(std::span<const DynamicProxyUpdate> updates)
    {
        return dynamic_.updateMany(updates);
    }

    bool removeDynamic(ProxyHandle handle)
    {
        return dynamic_.remove(handle);
    }
    void setDynamicOptions(const DynamicAabbTreeOptions &options) noexcept
    {
        dynamic_.setOptions(options);
    }

    [[nodiscard]] DynamicAabbTreeOptions dynamicOptions() const noexcept
    {
        return dynamic_.options();
    }

    bool setDynamicCategoryMask(ProxyHandle handle, std::uint64_t mask) noexcept
    {
        return dynamic_.setCategoryMask(handle, mask);
    }
    bool rebuildDynamic()
    {
        return dynamic_.rebuild();
    }
    [[nodiscard]] bool needsDynamicRebuild(
        std::uint32_t maximumHeight = 64u, double maximumSurfaceAreaRatio = 0.0) const
    {
        return dynamic_.needsRebuild(maximumHeight, maximumSurfaceAreaRatio);
    }

    // Queries both trees and appends tagged hits to `out` (static first, then
    // dynamic). The returned statistics sum both traversals. `scratch` is
    // caller-owned temporary storage; `out` is not cleared by the query.
    QueryStatistics query(const Aabb &area, std::vector<SceneSpatialHit> &out, SceneSpatialQueryScratch &scratch,
        std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max()) const
    {
        scratch.staticIndices.clear();
        scratch.dynamicHits.clear();
        QueryStatistics result = static_.query(area, scratch.staticIndices, categoryMask);
        for (const std::uint32_t index : scratch.staticIndices)
        {
            const SpatialItem *item = static_.item(index);
            if (item != nullptr)
                out.push_back({SceneMobility::Static, index, invalidProxy, item->userData, item->categoryMask});
        }
        const QueryStatistics dynamicStats = dynamic_.query(area, scratch.dynamicHits, categoryMask);
        result.nodesVisited += dynamicStats.nodesVisited;
        result.leavesVisited += dynamicStats.leavesVisited;
        result.hits += dynamicStats.hits;
        for (const SpatialHit &hit : scratch.dynamicHits)
            out.push_back({SceneMobility::Dynamic, invalidNode, hit.handle, hit.userData, hit.categoryMask});
        return result;
    }

    // Sphere query across static and dynamic geometry. Results are appended
    // static-first, then dynamic; scratch buffers are cleared and reused.
    QueryStatistics querySphere(const std::array<float, 3> &center, float radius,
        std::vector<SceneSpatialHit> &out, SceneSpatialQueryScratch &scratch,
        std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max()) const
    {
        scratch.staticIndices.clear();
        scratch.dynamicHits.clear();
        QueryStatistics result = static_.querySphere(center, radius, scratch.staticIndices, categoryMask);
        for (const std::uint32_t index : scratch.staticIndices)
        {
            const SpatialItem *item = static_.item(index);
            if (item != nullptr)
                out.push_back({SceneMobility::Static, index, invalidProxy, item->userData, item->categoryMask});
        }
        const QueryStatistics dynamicStats = dynamic_.querySphere(center, radius, scratch.dynamicHits, categoryMask);
        result.nodesVisited += dynamicStats.nodesVisited;
        result.leavesVisited += dynamicStats.leavesVisited;
        result.hits += dynamicStats.hits;
        for (const SpatialHit &hit : scratch.dynamicHits)
            out.push_back({SceneMobility::Dynamic, invalidNode, hit.handle, hit.userData, hit.categoryMask});
        return result;
    }

    // Frustum query across static and dynamic geometry. Frustum planes point
    // inward; invalid plane data yields an empty result.
    QueryStatistics queryFrustum(const Frustum3 &frustum, std::vector<SceneSpatialHit> &out,
        SceneSpatialQueryScratch &scratch,
        std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max()) const
    {
        scratch.staticIndices.clear();
        scratch.dynamicHits.clear();
        QueryStatistics result = static_.queryFrustum(frustum, scratch.staticIndices, categoryMask);
        for (const std::uint32_t index : scratch.staticIndices)
        {
            const SpatialItem *item = static_.item(index);
            if (item != nullptr)
                out.push_back({SceneMobility::Static, index, invalidProxy, item->userData, item->categoryMask});
        }
        const QueryStatistics dynamicStats = dynamic_.queryFrustum(frustum, scratch.dynamicHits, categoryMask);
        result.nodesVisited += dynamicStats.nodesVisited;
        result.leavesVisited += dynamicStats.leavesVisited;
        result.hits += dynamicStats.hits;
        for (const SpatialHit &hit : scratch.dynamicHits)
            out.push_back({SceneMobility::Dynamic, invalidNode, hit.handle, hit.userData, hit.categoryMask});
        return result;
    }

    // Ray query across static and dynamic geometry. Results are appended in
    // static-then-dynamic grouping, not distance-sorted; sort by distance when
    // nearest-first processing is required. Scratch buffers are cleared/reused.
    QueryStatistics raycast(const Ray3 &ray, std::vector<SceneSpatialRayHit> &out,
        SceneSpatialRayScratch &scratch,
        std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max()) const
    {
        scratch.staticHits.clear();
        scratch.dynamicHits.clear();
        QueryStatistics result = static_.raycast(ray, scratch.staticHits, categoryMask);
        for (const StaticRayHit &hit : scratch.staticHits)
            out.push_back({SceneMobility::Static, hit.itemIndex, invalidProxy, hit.userData, hit.categoryMask, hit.distance});
        const QueryStatistics dynamicStats = dynamic_.raycast(ray, scratch.dynamicHits, categoryMask);
        result.nodesVisited += dynamicStats.nodesVisited;
        result.leavesVisited += dynamicStats.leavesVisited;
        result.hits += dynamicStats.hits;
        for (const DynamicRayHit &hit : scratch.dynamicHits)
            out.push_back({SceneMobility::Dynamic, invalidNode, hit.handle, hit.userData, hit.categoryMask, hit.distance});
        return result;
    }

    // Read-only access to the underlying trees for specialized traversals.
    [[nodiscard]] std::size_t staticSize() const noexcept
    {
        return static_.size();
    }
    [[nodiscard]] std::size_t dynamicSize() const noexcept
    {
        return dynamic_.size();
    }
    [[nodiscard]] const StaticBvh &staticTree() const noexcept
    {
        return static_;
    }
    [[nodiscard]] const DynamicAabbTree &dynamicTree() const noexcept
    {
        return dynamic_;
    }

private:
    StaticBvh       static_;
    DynamicAabbTree dynamic_;
};

} // namespace engine::spatial

#endif
