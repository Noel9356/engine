#ifndef ENGINE_MODULES_BVH_BINARY_BVH_HPP
#define ENGINE_MODULES_BVH_BINARY_BVH_HPP

// The implementation behind "bvh.binary": a compact axis-aligned bounding
// box hierarchy for CPU-side spatial queries. Pure logic over plain float
// arrays: no Vulkan, no backend.
//
// The structure is a binary BVH over caller-supplied AABBs. The split
// STRATEGY is configurable ([bvh].split in the configuration, applied by the
// module; see bvh.cpp) because build shape and query cost trade off against
// each other:
//
// - "median"   splits the longest axis at the object median. Perfectly
//              balanced, deterministic down to the index tiebreak - the
//              historical default.
// - "midpoint" splits the longest axis at the range's centroid midpoint.
//              Same build cost class as median (about half of it, in fact),
//              adapts the plane to where the boxes actually are, and wins
//              the engine's build-per-call workload at every measured box
//              count. Degenerate ranges (no center on one side of the
//              plane) fall back to the median split.
// - "sah"      picks the (axis, plane) pair that minimizes the binned surface
//              area heuristic - the expected traversal cost of the subtree.
//              The best QUERY throughput once the box count grows, at a
//              markedly higher build cost; the right choice for a consumer
//              that builds once and queries many times.
//
// The shipped default is "midpoint": the benchmark harness
// (modules/bvh/benchmark.cpp, spread and clustered scenes at the registry
// cap of 32 boxes and at the 256/1024 stress counts) measured the engine's
// actual contract - build a hierarchy, then run the batch's overlap
// queries - and the midpoint tree's total (build + query) cost is the
// lowest at every measured operating point, e.g. 256 spread boxes:
// midpoint ~29 us vs median ~56 us vs SAH ~269 us per build+batch call.
// SAH's edge is pure query throughput; under a per-call rebuild that edge
// never pays for its build.
//
// Every strategy produces single-box leaves over the same node structure, so
// query results are identical regardless of strategy (the overlap test is
// exact): the choice only moves traversal cost around, never correctness.

#include "bvh_capability.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>

namespace engine
{

// The split strategy, chosen once per build.
enum class BvhSplitMethod : std::uint8_t
{
    Median,   // longest axis, object median
    Midpoint, // longest axis, centroid midpoint of the range
    Sah       // binned surface area heuristic, best of the three axes
};

// The hierarchy. Build once per query batch (32 boxes: microseconds), query
// many times. Build takes the boxes by value and keeps its own copy - the
// caller's geometry may be reordered scenes, the hierarchy is a stable
// snapshot.
class BoundingVolumeHierarchy final
{
public:
    // Bin count of the SAH strategy: large enough that the sweep sees the
    // box distribution, small enough that a 32-box node still fills several
    // boxes per bin (a bin census coarser than the data is noise).
    static constexpr std::uint32_t sahBinCount = 16;

    // Builds the hierarchy over the given boxes. Empty input yields an empty
    // hierarchy that answers every query with no results. The method selects
    // the split strategy (see the file comment); the shipped default is the
    // benchmark winner, midpoint.
    void build(
        std::vector<AxisAlignedBoundingBox> boxes,
        BvhSplitMethod                      method = BvhSplitMethod::Midpoint
    )
    {
        boxes_ = std::move(boxes);
        nodes_.clear();
        splitMethod = method;
        if (boxes_.empty())
        {
            return;
        }
        nodes_.reserve(boxes_.size() * 2);
        // Leaf order = original box index; the split sorts index lists, never
        // the caller's boxes.
        std::vector<std::uint32_t> order(boxes_.size());
        for (std::uint32_t index = 0; index < static_cast<std::uint32_t>(boxes_.size()); ++index)
        {
            order[index] = index;
        }
        buildRange(order.data(), static_cast<std::uint32_t>(order.size()));
    }

    std::size_t boxCount() const
    {
        return boxes_.size();
    }

    const AxisAlignedBoundingBox &box(
        std::size_t index
    ) const
    {
        return boxes_[index];
    }

    // Appends every box overlapping the query box (exact test, no false
    // positives) to outIndices. The results are unordered - callers that
    // iterate symmetric pairs must not act twice on one pair.
    void queryOverlaps(
        const AxisAlignedBoundingBox &query, std::vector<std::uint32_t> &outIndices
    ) const
    {
        if (nodes_.empty())
        {
            return;
        }
        // Iterative walk. Every node's skipSubtree is one past its whole
        // subtree, so both "rejected" and "finished reporting" continue at
        // the same place - a leaf or a completed right subtree can never run
        // the cursor out of bounds.
        static constexpr std::uint32_t none = 0xFFFFFFFFu;
        std::uint32_t                  nodeIndex = 0;
        while (nodeIndex < nodes_.size())
        {
            const Node &node = nodes_[nodeIndex];
            if (!boxOverlaps(node.bounds, query))
            {
                nodeIndex = node.skipSubtree;
                continue;
            }
            if (node.boxIndex != none)
            {
                outIndices.push_back(node.boxIndex);
                nodeIndex = node.skipSubtree;
                continue;
            }
            ++nodeIndex; // internal and overlapping: descend into the left child
        }
    }

private:
    struct Node
    {
        AxisAlignedBoundingBox bounds{};
        std::uint32_t          boxIndex    = 0xFFFFFFFFu; // leaf payload
        std::uint32_t          skipSubtree = 0xFFFFFFFFu; // next node when this subtree is rejected
    };

    // Builds one subtree over order[0..count); returns the range's root node
    // position. Leaves hold one box; internal nodes split by the configured
    // strategy.
    std::uint32_t buildRange(
        std::uint32_t *order, std::uint32_t count
    )
    {
        AxisAlignedBoundingBox rangeBounds = boxes_[order[0]];
        for (std::uint32_t index = 1; index < count; ++index)
        {
            const AxisAlignedBoundingBox &box = boxes_[order[index]];
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                rangeBounds.minPoint[axis] = std::min(rangeBounds.minPoint[axis], box.minPoint[axis]);
                rangeBounds.maxPoint[axis] = std::max(rangeBounds.maxPoint[axis], box.maxPoint[axis]);
            }
        }

        const std::uint32_t nodePosition = static_cast<std::uint32_t>(nodes_.size());
        nodes_.push_back(Node{.bounds = rangeBounds, .boxIndex = 0xFFFFFFFFu, .skipSubtree = 0xFFFFFFFFu});
        if (count == 1)
        {
            nodes_[nodePosition].boxIndex    = order[0];
            nodes_[nodePosition].skipSubtree = nodePosition + 1; // one past this leaf
            return nodePosition;
        }

        // The split. Every strategy partitions `order` in place around a
        // position in (0, count) and recurses; a degenerate range (all
        // centers coincident on the chosen plane) falls back to the median
        // so the recursion always makes progress.
        std::uint32_t middle = 0;
        switch (splitMethod)
        {
        case BvhSplitMethod::Median:
            middle = medianSplit(order, count, rangeBounds);
            break;
        case BvhSplitMethod::Midpoint:
            middle = midpointSplit(order, count, rangeBounds);
            break;
        case BvhSplitMethod::Sah:
            middle = sahSplit(order, count, rangeBounds);
            break;
        }

        // The left subtree's root position is not needed: internal nodes are
        // walked linearly (the query descends by ++nodeIndex), only the right
        // child's skip value feeds the parent's skipSubtree.
        buildRange(order, middle);
        const std::uint32_t rightPosition = buildRange(order + middle, count - middle);
        // Rejecting this subtree skips past the right child (which itself
        // skips its subtree): the parent's skip is one past the last node of
        // its range, which is exactly the right child's skip value after the
        // right subtree was built.
        nodes_[nodePosition].skipSubtree = nodes_[rightPosition].skipSubtree;
        return nodePosition;
    }

    // Longest axis of the range, shared by every strategy's axis pre-choice
    // and by the median fallback.
    static std::size_t longestAxis(
        const AxisAlignedBoundingBox &rangeBounds
    )
    {
        std::size_t axis          = 0;
        float       longestExtent = -1.0f;
        for (std::size_t candidate = 0; candidate < 3; ++candidate)
        {
            const float extent = rangeBounds.maxPoint[candidate] - rangeBounds.minPoint[candidate];
            if (extent > longestExtent)
            {
                longestExtent = extent;
                axis          = candidate;
            }
        }
        return axis;
    }

    // Deterministic center comparator on one axis (index as tiebreak keeps
    // identical boxes reproducibly ordered).
    std::uint32_t medianSplit(
        std::uint32_t *order, std::uint32_t count, const AxisAlignedBoundingBox &rangeBounds
    )
    {
        const std::size_t axis = longestAxis(rangeBounds);
        const std::uint32_t middle = count / 2;
        std::nth_element(
            order,
            order + middle,
            order + count,
            [this, axis](std::uint32_t left, std::uint32_t right)
            {
                const float leftCenter  = boxes_[left].center()[axis];
                const float rightCenter = boxes_[right].center()[axis];
                if (leftCenter != rightCenter)
                {
                    return leftCenter < rightCenter;
                }
                return left < right;
            }
        );
        return middle;
    }

    // Splits the longest axis at the range's centroid midpoint. A plane no
    // box center lands left of (or all do) would recurse forever, so a
    // degenerate partition falls back to the median split.
    std::uint32_t midpointSplit(
        std::uint32_t *order, std::uint32_t count, const AxisAlignedBoundingBox &rangeBounds
    )
    {
        const std::size_t axis     = longestAxis(rangeBounds);
        const float       midpoint = 0.5f * (rangeBounds.minPoint[axis] + rangeBounds.maxPoint[axis]);
        const auto        firstRight = std::partition(
            order,
            order + count,
            [this, axis, midpoint](std::uint32_t index)
            { return boxes_[index].center()[axis] < midpoint; }
        );
        const std::uint32_t middle = static_cast<std::uint32_t>(firstRight - order);
        if (middle == 0 || middle == count)
        {
            return medianSplit(order, count, rangeBounds);
        }
        return middle;
    }

    // Binned surface area heuristic, evaluated on all three axes. Every box
    // enters one bin per axis by its center; a left-to-right sweep accumulates
    // each candidate split's cost
    //
    //     cost(edge) = C_traverse + (SA(left) * N(left) + SA(right) * N(right))
    //                  / SA(range)
    //
    // and the cheapest (axis, edge) wins. Single-box leaves make the leaf
    // cost side of the heuristic constant, so the SAH here is a split PLANE
    // selector - it shapes the tree's expected traversal cost, it does not
    // decide leaf size. A candidate that strands every box on one side falls
    // back to the median split.
    std::uint32_t sahSplit(
        std::uint32_t *order, std::uint32_t count, const AxisAlignedBoundingBox &rangeBounds
    )
    {
        struct Bin
        {
            AxisAlignedBoundingBox bounds{};
            std::uint32_t          count = 0;
        };
        constexpr float traversalCost = 1.0f;
        constexpr float infinity      = std::numeric_limits<float>::infinity();

        float         bestCost = infinity;
        std::size_t   bestAxis = 0;
        std::uint32_t bestEdge = 0;
        bool          found    = false;

        const float rangeSurface = surfaceArea(rangeBounds);
        const float inverseRangeSurface = rangeSurface > 0.0f ? 1.0f / rangeSurface : 0.0f;

        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            const float axisMin    = rangeBounds.minPoint[axis];
            const float axisExtent = rangeBounds.maxPoint[axis] - axisMin;
            if (axisExtent <= 0.0f)
            {
                continue; // degenerate axis: every center identical, no split
            }

            Bin bins[sahBinCount];
            for (Bin &sahBin : bins)
            {
                sahBin = Bin{};
            }
            const float binScale = static_cast<float>(sahBinCount) / axisExtent;
            for (std::uint32_t index = 0; index < count; ++index)
            {
                std::uint32_t binIndex = static_cast<std::uint32_t>(
                    (boxes_[order[index]].center()[axis] - axisMin) * binScale
                );
                binIndex = std::min(binIndex, sahBinCount - 1); // the exact upper edge lands in the last bin
                if (bins[binIndex].count == 0)
                {
                    bins[binIndex].bounds = boxes_[order[index]];
                }
                else
                {
                    grow(bins[binIndex].bounds, boxes_[order[index]]);
                }
                ++bins[binIndex].count;
            }

            // Prefix areas/counts (bins 0..edge) and suffix areas/counts
            // (bins edge+1..last); edge in [0, sahBinCount - 2] so both sides
            // are non-empty as bin RANGES (individual boxes may still land
            // on one side only - checked below through the partition).
            float         leftArea[sahBinCount - 1];
            std::uint32_t leftCount[sahBinCount - 1];
            float         rightArea[sahBinCount - 1];
            std::uint32_t rightCount[sahBinCount - 1];

            AxisAlignedBoundingBox accumulated{};
            std::uint32_t          accumulatedCount = 0;
            for (std::uint32_t edge = 0; edge < sahBinCount - 1; ++edge)
            {
                if (bins[edge].count > 0)
                {
                    if (accumulatedCount == 0)
                    {
                        accumulated = bins[edge].bounds;
                    }
                    else
                    {
                        grow(accumulated, bins[edge].bounds);
                    }
                    accumulatedCount += bins[edge].count;
                }
                leftArea[edge]  = accumulatedCount > 0 ? surfaceArea(accumulated) : 0.0f;
                leftCount[edge] = accumulatedCount;
            }
            accumulated      = AxisAlignedBoundingBox{};
            accumulatedCount = 0;
            for (std::uint32_t reverse = sahBinCount - 1; reverse > 0; --reverse)
            {
                const std::uint32_t edge = reverse - 1; // right side starts at bins[edge + 1]
                if (bins[reverse].count > 0)
                {
                    if (accumulatedCount == 0)
                    {
                        accumulated = bins[reverse].bounds;
                    }
                    else
                    {
                        grow(accumulated, bins[reverse].bounds);
                    }
                    accumulatedCount += bins[reverse].count;
                }
                rightArea[edge]  = accumulatedCount > 0 ? surfaceArea(accumulated) : 0.0f;
                rightCount[edge] = accumulatedCount;
            }

            for (std::uint32_t edge = 0; edge < sahBinCount - 1; ++edge)
            {
                if (leftCount[edge] == 0 || rightCount[edge] == 0)
                {
                    continue;
                }
                const float cost = traversalCost
                                 + (leftArea[edge] * static_cast<float>(leftCount[edge])
                                    + rightArea[edge] * static_cast<float>(rightCount[edge]))
                                       * inverseRangeSurface;
                if (cost < bestCost)
                {
                    bestCost  = cost;
                    bestAxis  = axis;
                    bestEdge  = edge;
                    found     = true;
                }
            }
        }

        if (!found)
        {
            return medianSplit(order, count, rangeBounds);
        }

        // Partition by the winning bin edge: boxes whose centers fall in
        // bins 0..bestEdge go left. Binning and partitioning share one
        // formula, so the sweep's side populations match the actual split.
        const float axisMin    = rangeBounds.minPoint[bestAxis];
        const float axisExtent = rangeBounds.maxPoint[bestAxis] - axisMin;
        const float binScale   = static_cast<float>(sahBinCount) / axisExtent;
        const auto  firstRight = std::partition(
            order,
            order + count,
            [this, bestAxis, axisMin, binScale, bestEdge](std::uint32_t index)
            {
                const std::uint32_t bin = std::min(
                    static_cast<std::uint32_t>((boxes_[index].center()[bestAxis] - axisMin) * binScale),
                    sahBinCount - 1
                );
                return bin <= bestEdge;
            }
        );
        const std::uint32_t middle = static_cast<std::uint32_t>(firstRight - order);
        if (middle == 0 || middle == count)
        {
            return medianSplit(order, count, rangeBounds);
        }
        return middle;
    }

    static void grow(
        AxisAlignedBoundingBox &bounds, const AxisAlignedBoundingBox &box
    )
    {
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            bounds.minPoint[axis] = std::min(bounds.minPoint[axis], box.minPoint[axis]);
            bounds.maxPoint[axis] = std::max(bounds.maxPoint[axis], box.maxPoint[axis]);
        }
    }

    static float surfaceArea(
        const AxisAlignedBoundingBox &bounds
    )
    {
        const float x = bounds.maxPoint[0] - bounds.minPoint[0];
        const float y = bounds.maxPoint[1] - bounds.minPoint[1];
        const float z = bounds.maxPoint[2] - bounds.minPoint[2];
        if (x <= 0.0f || y <= 0.0f || z <= 0.0f)
        {
            return 0.0f;
        }
        return 2.0f * (x * y + y * z + z * x);
    }

    BvhSplitMethod splitMethod = BvhSplitMethod::Midpoint;

    std::vector<AxisAlignedBoundingBox> boxes_;
    std::vector<Node>                   nodes_;
};

} // namespace engine

#endif // ENGINE_MODULES_BVH_BINARY_BVH_HPP
