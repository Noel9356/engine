// static_bvh.hpp - static bounding volume hierarchy over indexed items.
//
// Built once with build() (median or binned-SAH splits), then queried many
// times. Bounds of existing items can be refreshed in bulk (refit) or
// incrementally (refitItems) without changing the tree structure. Header-only.

#ifndef ENGINE_SPATIAL_INDEX_STATIC_BVH_HPP
#define ENGINE_SPATIAL_INDEX_STATIC_BVH_HPP

#include <engine/spatial_index/aabb.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <numeric>
#include <span>
#include <vector>

namespace engine::spatial
{

// Leaf splitting strategy used while building.
enum class StaticBvhSplitStrategy : std::uint8_t
{
    Median,   // Split at the centroid median of the widest centroid axis.
    BinnedSah // Surface-area heuristic over binned candidate splits (better tree, slower build).
};

// Build knobs; values are clamped internally (leafSize 1-64, bucketCount 4-32).
struct StaticBvhBuildOptions
{
    std::uint32_t          leafSize    = 4; // Maximum items kept in one leaf node.
    StaticBvhSplitStrategy strategy    = StaticBvhSplitStrategy::Median;
    std::uint32_t          bucketCount = 12; // SAH bins per axis when strategy is BinnedSah.
};

// One indexed object in the BVH. categoryMask lets queries filter by category
// cheaply (nodes keep the OR of their items' masks).
struct SpatialItem
{
    Aabb          bounds{};
    std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max();
    std::uint64_t userData     = 0; // Opaque caller payload, reachable via item().
};

// Incremental item change applied by refitItems (index refers to the position
// the item had at build time; the item order is fixed after build).
struct SpatialItemUpdate
{
    std::uint32_t itemIndex = invalidNode;
    Aabb          bounds{};
    std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max();
};

// Immutable-structure BVH: node indices stay valid until the next build().
// Not internally synchronized: queries are const, but callers must not mutate
// the tree from another thread while querying.
class StaticBvh final
{
public:
    // Convenience overload using default options except leafSize.
    bool build(std::span<const Aabb> bounds, std::uint32_t leafSize = 4)
    {
        StaticBvhBuildOptions options{};
        options.leafSize = leafSize;
        return build(bounds, options);
    }

    // Builds the tree from plain boxes (masks default to "all categories",
    // userData to the item index) or from full SpatialItems. Any invalid box,
    // size overflow, or allocation failure clears the tree and returns false.
    bool build(std::span<const Aabb> bounds, const StaticBvhBuildOptions &options)
    {
        clear();
        if (bounds.size() > static_cast<std::size_t>(invalidNode) - 1u)
            return false;
        try
        {
            items_.reserve(bounds.size());
            for (std::size_t index = 0; index < bounds.size(); ++index)
            {
                if (!bounds[index].valid())
                {
                    clear();
                    return false;
                }
                items_.push_back({bounds[index], std::numeric_limits<std::uint64_t>::max(), index});
            }
            return buildNodes(options);
        }
        catch (...)
        {
            clear();
            return false;
        }
    }

    bool build(std::span<const SpatialItem> items, const StaticBvhBuildOptions &options = {})
    {
        clear();
        if (items.size() > static_cast<std::size_t>(invalidNode) - 1u)
            return false;
        try
        {
            items_.reserve(items.size());
            for (const SpatialItem &item : items)
            {
                if (!item.bounds.valid())
                {
                    clear();
                    return false;
                }
                items_.push_back(item);
            }
            return buildNodes(options);
        }
        catch (...)
        {
            clear();
            return false;
        }
    }

    // Resets the tree to empty (vector capacity is retained for reuse).
    void clear() noexcept
    {
        items_.clear();
        order_.clear();
        nodes_.clear();
        itemLeaf_.clear();
        refitMarks_.clear();
        refitScratch_.clear();
    }

    [[nodiscard]] std::size_t size() const noexcept
    {
        return items_.size();
    }
    [[nodiscard]] bool empty() const noexcept
    {
        return items_.empty();
    }
    [[nodiscard]] std::size_t nodeCount() const noexcept
    {
        return nodes_.size();
    }
    [[nodiscard]] StaticBvhBuildOptions buildOptions() const noexcept
    {
        return options_;
    }

    // Recomputes every node bound/mask bottom-up from new bounds for all items
    // (structure unchanged). Fails without changes if sizes mismatch or any new
    // bound is invalid.
    bool refit(std::span<const Aabb> updatedBounds) noexcept
    {
        if (updatedBounds.size() != items_.size())
            return false;
        for (const Aabb &bounds : updatedBounds)
            if (!bounds.valid())
                return false;
        for (std::size_t index = 0; index < items_.size(); ++index)
            items_[index].bounds = updatedBounds[index];
        // Nodes are stored child-before-parent, so a reverse walk refits the
        // whole tree in one pass.
        for (std::size_t reverse = nodes_.size(); reverse != 0; --reverse)
        {
            Node &node = nodes_[reverse - 1u];
            if (node.count != 0u)
            {
                Aabb          combined = items_[order_[node.first]].bounds;
                std::uint64_t mask     = 0;
                for (std::uint32_t offset = 0; offset < node.count; ++offset)
                {
                    const SpatialItem &item = items_[order_[node.first + offset]];
                    combined                = merge(combined, item.bounds);
                    mask |= item.categoryMask;
                }
                node.bounds       = combined;
                node.categoryMask = mask;
            }
            else
            {
                node.bounds       = merge(nodes_[node.left].bounds, nodes_[node.right].bounds);
                node.categoryMask = nodes_[node.left].categoryMask | nodes_[node.right].categoryMask;
            }
        }
        return true;
    }

    // Partial refit: applies per-item updates, then recomputes bounds only along
    // the affected root paths (paths deduplicated via generation marks, then
    // reprocessed child-first). Fails without changes if any update is out of
    // range or carries an invalid bound.
    bool refitItems(std::span<const SpatialItemUpdate> updates) noexcept
    {
        for (const SpatialItemUpdate &update : updates)
            if (update.itemIndex >= items_.size() || !update.bounds.valid())
                return false;
        if (updates.empty())
            return true;

        for (const SpatialItemUpdate &update : updates)
        {
            SpatialItem &item = items_[update.itemIndex];
            item.bounds       = update.bounds;
            item.categoryMask = update.categoryMask;
        }

        // Collect the unique ancestor chain of every updated item; the stamp
        // trick avoids visiting shared ancestors twice.
        ++refitGeneration_;
        if (refitGeneration_ == 0u)
        {
            std::fill(refitMarks_.begin(), refitMarks_.end(), 0u);
            refitGeneration_ = 1u;
        }
        refitScratch_.clear();
        for (const SpatialItemUpdate &update : updates)
        {
            std::uint32_t nodeIndex = itemLeaf_[update.itemIndex];
            while (nodeIndex != invalidNode)
            {
                if (refitMarks_[nodeIndex] != refitGeneration_)
                {
                    refitMarks_[nodeIndex] = refitGeneration_;
                    refitScratch_.push_back(nodeIndex);
                }
                nodeIndex = nodes_[nodeIndex].parent;
            }
        }

        // Descending index order == child-before-parent, so each node sees
        // already-updated children.
        std::sort(refitScratch_.begin(), refitScratch_.end(), std::greater<>());
        for (const std::uint32_t nodeIndex : refitScratch_)
        {
            Node &node = nodes_[nodeIndex];
            if (node.count != 0u)
            {
                Aabb          combined = items_[order_[node.first]].bounds;
                std::uint64_t mask     = 0;
                for (std::uint32_t offset = 0; offset < node.count; ++offset)
                {
                    const SpatialItem &item = items_[order_[node.first + offset]];
                    combined                = merge(combined, item.bounds);
                    mask |= item.categoryMask;
                }
                node.bounds       = combined;
                node.categoryMask = mask;
            }
            else
            {
                node.bounds       = merge(nodes_[node.left].bounds, nodes_[node.right].bounds);
                node.categoryMask = nodes_[node.left].categoryMask | nodes_[node.right].categoryMask;
            }
        }
        return true;
    }

    // Area query: appends the indices of items whose bounds intersect `area`
    // and whose categoryMask overlaps `categoryMask` (0 disables the query).
    // Traversal is iterative with a fixed 128-entry stack; a tree deeper than
    // that aborts the query with partial results (visible in the statistics).
    QueryStatistics query(const Aabb &area, std::vector<std::uint32_t> &out,
        std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max()) const
    {
        QueryStatistics stats{};
        if (!area.valid() || nodes_.empty() || categoryMask == 0)
            return stats;
        std::array<std::uint32_t, 128> stack{};
        std::size_t                    stackSize = 0;
        stack[stackSize++]                       = 0u;
        while (stackSize != 0)
        {
            const std::uint32_t index = stack[--stackSize];
            const Node         &node  = nodes_[index];
            ++stats.nodesVisited;
            if ((node.categoryMask & categoryMask) == 0 || !node.bounds.intersects(area))
                continue;
            if (node.count != 0u)
            {
                for (std::uint32_t offset = 0; offset < node.count; ++offset)
                {
                    const std::uint32_t itemIndex = order_[node.first + offset];
                    ++stats.leavesVisited;
                    const SpatialItem &item = items_[itemIndex];
                    if ((item.categoryMask & categoryMask) != 0 && item.bounds.intersects(area))
                    {
                        out.push_back(itemIndex);
                        ++stats.hits;
                    }
                }
            }
            else
            {
                if (stackSize + 2u > stack.size())
                    return stats;
                stack[stackSize++] = node.right;
                stack[stackSize++] = node.left;
            }
        }
        return stats;
    }

    // Sphere query: appends original item indices whose AABBs intersect the
    // sphere. Internal bounds prune traversal; item bounds are tested exactly.
    QueryStatistics querySphere(const std::array<float, 3> &center, float radius,
        std::vector<std::uint32_t> &out,
        std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max()) const
    {
        QueryStatistics stats{};
        if (nodes_.empty() || categoryMask == 0 || !std::isfinite(center[0])
            || !std::isfinite(center[1]) || !std::isfinite(center[2])
            || !std::isfinite(radius) || radius < 0.0f)
            return stats;
        std::array<std::uint32_t, 128> stack{};
        std::size_t stackSize = 0;
        stack[stackSize++] = 0u;
        while (stackSize != 0)
        {
            const std::uint32_t index = stack[--stackSize];
            const Node &node = nodes_[index];
            ++stats.nodesVisited;
            if ((node.categoryMask & categoryMask) == 0 || !query_detail::intersectsSphereValid(node.bounds, center, radius))
                continue;
            if (node.count != 0u)
            {
                for (std::uint32_t offset = 0; offset < node.count; ++offset)
                {
                    const std::uint32_t itemIndex = order_[node.first + offset];
                    ++stats.leavesVisited;
                    const SpatialItem &item = items_[itemIndex];
                    if ((item.categoryMask & categoryMask) != 0
                        && query_detail::intersectsSphereValid(item.bounds, center, radius))
                    {
                        out.push_back(itemIndex);
                        ++stats.hits;
                    }
                }
            }
            else
            {
                if (stackSize + 2u > stack.size())
                    return stats;
                stack[stackSize++] = node.right;
                stack[stackSize++] = node.left;
            }
        }
        return stats;
    }

    // Frustum query: appends original item indices whose AABBs intersect all
    // inward-facing planes. Category masks are used at both node and item level.
    QueryStatistics queryFrustum(const Frustum3 &frustum, std::vector<std::uint32_t> &out,
        std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max()) const
    {
        QueryStatistics stats{};
        if (!frustum.valid() || nodes_.empty() || categoryMask == 0)
            return stats;
        std::array<std::uint32_t, 128> stack{};
        std::size_t stackSize = 0;
        stack[stackSize++] = 0u;
        while (stackSize != 0)
        {
            const std::uint32_t index = stack[--stackSize];
            const Node &node = nodes_[index];
            ++stats.nodesVisited;
            if ((node.categoryMask & categoryMask) == 0 || !query_detail::intersectsFrustumValid(node.bounds, frustum))
                continue;
            if (node.count != 0u)
            {
                for (std::uint32_t offset = 0; offset < node.count; ++offset)
                {
                    const std::uint32_t itemIndex = order_[node.first + offset];
                    ++stats.leavesVisited;
                    const SpatialItem &item = items_[itemIndex];
                    if ((item.categoryMask & categoryMask) != 0 && query_detail::intersectsFrustumValid(item.bounds, frustum))
                    {
                        out.push_back(itemIndex);
                        ++stats.hits;
                    }
                }
            }
            else
            {
                if (stackSize + 2u > stack.size())
                    return stats;
                stack[stackSize++] = node.right;
                stack[stackSize++] = node.left;
            }
        }
        return stats;
    }

    // Ray query: appends exact item-bound hits in BVH traversal order.
    // Distance is in world units; out is not cleared. Traversal uses a fixed
    // stack and returns partial results if it would overflow.
    QueryStatistics raycast(const Ray3 &ray, std::vector<StaticRayHit> &out,
        std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max()) const
    {
        QueryStatistics stats{};
        if (nodes_.empty() || categoryMask == 0)
            return stats;
        std::array<std::uint32_t, 128> stack{};
        std::size_t stackSize = 0;
        stack[stackSize++] = 0u;
        while (stackSize != 0)
        {
            const std::uint32_t index = stack[--stackSize];
            const Node &node = nodes_[index];
            ++stats.nodesVisited;
            if ((node.categoryMask & categoryMask) == 0 || !intersectRay(node.bounds, ray))
                continue;
            if (node.count != 0u)
            {
                for (std::uint32_t offset = 0; offset < node.count; ++offset)
                {
                    const std::uint32_t itemIndex = order_[node.first + offset];
                    ++stats.leavesVisited;
                    const SpatialItem &item = items_[itemIndex];
                    float distance = 0.0f;
                    if ((item.categoryMask & categoryMask) != 0 && intersectRay(item.bounds, ray, &distance))
                    {
                        out.push_back({itemIndex, item.userData, item.categoryMask, distance});
                        ++stats.hits;
                    }
                }
            }
            else
            {
                if (stackSize + 2u > stack.size())
                    return stats;
                stack[stackSize++] = node.right;
                stack[stackSize++] = node.left;
            }
        }
        return stats;
    }

    // Item accessor by original index; nullptr when out of range.
    [[nodiscard]] const SpatialItem *item(std::uint32_t index) const noexcept
    {
        return index < items_.size() ? &items_[index] : nullptr;
    }

private:
    // Flat node array, child-before-parent; nodes_[0] is the root. count != 0
    // marks a leaf covering items[order_[first, first+count)]; otherwise
    // left/right are child indices. categoryMask is the OR over the subtree for
    // cheap query rejection.
    struct Node
    {
        Aabb          bounds{};
        std::uint64_t categoryMask = 0;
        std::uint32_t parent       = invalidNode;
        std::uint32_t left         = invalidNode;
        std::uint32_t right        = invalidNode;
        std::uint32_t first        = 0; // Leaf only: start offset into order_.
        std::uint32_t count        = 0; // Leaf only: item count; 0 for internal nodes.
    };

    // Per-axis SAH bucket accumulator used by the BinnedSah strategy.
    struct Bin
    {
        Aabb          bounds{};
        std::uint32_t count       = 0;
        bool          initialized = false;
    };

    // Clamps options, lays order_ out as identity, builds recursively, then
    // records which leaf owns each item (itemLeaf_) for refitItems.
    bool buildNodes(const StaticBvhBuildOptions &requested)
    {
        options_             = requested;
        options_.leafSize    = std::clamp(options_.leafSize, 1u, 64u);
        options_.bucketCount = std::clamp(options_.bucketCount, 4u, 32u);
        order_.resize(items_.size());
        std::iota(order_.begin(), order_.end(), 0u);
        if (!order_.empty())
        {
            nodes_.reserve(order_.size() * 2u);
            buildNode(0u, static_cast<std::uint32_t>(order_.size()));
        }
        itemLeaf_.assign(items_.size(), invalidNode);
        for (std::uint32_t nodeIndex = 0; nodeIndex < nodes_.size(); ++nodeIndex)
        {
            const Node &node = nodes_[nodeIndex];
            if (node.count != 0u)
                for (std::uint32_t offset = 0; offset < node.count; ++offset)
                    itemLeaf_[order_[node.first + offset]] = nodeIndex;
        }
        refitMarks_.assign(nodes_.size(), 0u);
        refitScratch_.clear();
        refitScratch_.reserve(nodes_.size());
        return true;
    }

    // Recursively creates the node covering order_[first, first+count). Chooses
    // the split axis from the centroid extent; BinnedSah evaluates binned split
    // costs per axis and keeps the cheapest partition whose sides each hold at
    // least a quarter of the items, falling back to a median split otherwise.
    std::uint32_t buildNode(std::uint32_t first, std::uint32_t count)
    {
        const std::uint32_t nodeIndex = static_cast<std::uint32_t>(nodes_.size());
        nodes_.push_back({});
        Aabb                 nodeBounds  = items_[order_[first]].bounds;
        std::array<float, 3> centroidMin = nodeBounds.center();
        std::array<float, 3> centroidMax = centroidMin;
        std::uint64_t        nodeMask    = 0;
        for (std::uint32_t offset = 0; offset < count; ++offset)
        {
            const SpatialItem &item = items_[order_[first + offset]];
            nodeBounds              = merge(nodeBounds, item.bounds);
            nodeMask |= item.categoryMask;
            const auto center = item.bounds.center();
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                centroidMin[axis] = std::min(centroidMin[axis], center[axis]);
                centroidMax[axis] = std::max(centroidMax[axis], center[axis]);
            }
        }
        nodes_[nodeIndex].bounds       = nodeBounds;
        nodes_[nodeIndex].categoryMask = nodeMask;
        if (count <= options_.leafSize)
        {
            nodes_[nodeIndex].first = first;
            nodes_[nodeIndex].count = count;
            return nodeIndex;
        }

        std::size_t axis = 0;
        if (centroidMax[1] - centroidMin[1] > centroidMax[axis] - centroidMin[axis])
            axis = 1;
        if (centroidMax[2] - centroidMin[2] > centroidMax[axis] - centroidMin[axis])
            axis = 2;
        std::uint32_t middle         = first + count / 2u;
        bool          sahPartitioned = false;

        if (options_.strategy == StaticBvhSplitStrategy::BinnedSah)
        {
            double              bestCost    = std::numeric_limits<double>::infinity();
            std::size_t         bestAxis    = 0;
            std::uint32_t       bestSplit   = 0;
            const std::uint32_t bucketCount = options_.bucketCount;
            for (std::size_t candidateAxis = 0; candidateAxis < 3; ++candidateAxis)
            {
                const double extent
                    = static_cast<double>(centroidMax[candidateAxis]) - static_cast<double>(centroidMin[candidateAxis]);
                if (!(extent > 1.0e-12) || !std::isfinite(extent))
                    continue;
                // Bin item bounds by centroid position, then accumulate
                // prefix/suffix bounds and counts for every candidate split.
                std::array<Bin, 32> bins{};
                for (std::uint32_t offset = 0; offset < count; ++offset)
                {
                    const std::uint32_t itemIndex  = order_[first + offset];
                    const double        center     = items_[itemIndex].bounds.center()[candidateAxis];
                    const double        normalized = (center - centroidMin[candidateAxis]) / extent;
                    const auto          bucket     = std::min(
                        bucketCount - 1u, static_cast<std::uint32_t>(std::max(0.0, normalized) * bucketCount));
                    Bin &bin = bins[bucket];
                    if (!bin.initialized)
                    {
                        bin.bounds      = items_[itemIndex].bounds;
                        bin.initialized = true;
                    }
                    else
                        bin.bounds = merge(bin.bounds, items_[itemIndex].bounds);
                    ++bin.count;
                }
                std::array<Aabb, 32>          prefixBounds{}, suffixBounds{};
                std::array<std::uint32_t, 32> prefixCounts{}, suffixCounts{};
                bool                          prefixValid = false;
                Aabb                          accumulated{};
                std::uint32_t                 accumulatedCount = 0;
                for (std::uint32_t bucket = 0; bucket < bucketCount; ++bucket)
                {
                    if (bins[bucket].count != 0)
                    {
                        accumulated = prefixValid ? merge(accumulated, bins[bucket].bounds) : bins[bucket].bounds;
                        prefixValid = true;
                    }
                    accumulatedCount += bins[bucket].count;
                    prefixBounds[bucket] = accumulated;
                    prefixCounts[bucket] = accumulatedCount;
                }
                bool suffixValid = false;
                accumulatedCount = 0;
                for (std::uint32_t reverse = bucketCount; reverse != 0; --reverse)
                {
                    const std::uint32_t bucket = reverse - 1u;
                    if (bins[bucket].count != 0)
                    {
                        accumulated = suffixValid ? merge(accumulated, bins[bucket].bounds) : bins[bucket].bounds;
                        suffixValid = true;
                    }
                    accumulatedCount += bins[bucket].count;
                    suffixBounds[bucket] = accumulated;
                    suffixCounts[bucket] = accumulatedCount;
                }
                // Cost = left area * left count + right area * right count.
                for (std::uint32_t split = 0; split + 1u < bucketCount; ++split)
                {
                    const std::uint32_t leftCount  = prefixCounts[split];
                    const std::uint32_t rightCount = suffixCounts[split + 1u];
                    if (leftCount == 0 || rightCount == 0)
                        continue;
                    const double cost = static_cast<double>(prefixBounds[split].surfaceArea()) * leftCount
                                      + static_cast<double>(suffixBounds[split + 1u].surfaceArea()) * rightCount;
                    if (cost < bestCost)
                    {
                        bestCost  = cost;
                        bestAxis  = candidateAxis;
                        bestSplit = split;
                    }
                }
            }
            if (std::isfinite(bestCost))
            {
                axis                   = bestAxis;
                const double extent    = static_cast<double>(centroidMax[axis]) - centroidMin[axis];
                const auto   bucketFor = [&](std::uint32_t itemIndex)
                {
                    const double center     = items_[itemIndex].bounds.center()[axis];
                    const double normalized = (center - centroidMin[axis]) / extent;
                    return std::min(options_.bucketCount - 1u,
                        static_cast<std::uint32_t>(std::max(0.0, normalized) * options_.bucketCount));
                };
                auto partition                 = std::partition(order_.begin() + first,
                    order_.begin() + first + count,
                    [&](std::uint32_t itemIndex) { return bucketFor(itemIndex) <= bestSplit; });
                middle                         = static_cast<std::uint32_t>(partition - order_.begin());
                const std::uint32_t leftCount  = middle - first;
                const std::uint32_t rightCount = first + count - middle;

                // Guard against lopsided SAH partitions on clustered centroids.
                const std::uint32_t minimumSide = std::max(1u, count / 4u);
                sahPartitioned                  = leftCount >= minimumSide && rightCount >= minimumSide;
            }
        }

        if (!sahPartitioned)
        {
            // Median fallback: nth_element on the centroid coordinate; ties are
            // broken by item index so the partition is deterministic.
            middle = first + count / 2u;
            std::nth_element(order_.begin() + first,
                order_.begin() + middle,
                order_.begin() + first + count,
                [&](std::uint32_t lhs, std::uint32_t rhs)
                {
                    const float a = items_[lhs].bounds.center()[axis];
                    const float b = items_[rhs].bounds.center()[axis];
                    return a == b ? lhs < rhs : a < b;
                });
        }
        const std::uint32_t left  = buildNode(first, middle - first);
        const std::uint32_t right = buildNode(middle, first + count - middle);
        nodes_[nodeIndex].left    = left;
        nodes_[nodeIndex].right   = right;
        nodes_[left].parent       = nodeIndex;
        nodes_[right].parent      = nodeIndex;
        return nodeIndex;
    }

    std::vector<SpatialItem>   items_;               // Stable storage; index == caller's item index.
    std::vector<std::uint32_t> order_;               // Item indices in leaf order; fixed after build.
    std::vector<Node>          nodes_;               // Child-before-parent flat array; root at 0.
    std::vector<std::uint32_t> itemLeaf_;            // Item index -> leaf node index.
    std::vector<std::uint32_t> refitMarks_;          // Per-node last-refit generation stamp.
    std::vector<std::uint32_t> refitScratch_;        // Reused node list for partial refits.
    std::uint32_t              refitGeneration_ = 0; // Current refitItems() stamp.
    StaticBvhBuildOptions      options_{};
};

} // namespace engine::spatial

#endif
