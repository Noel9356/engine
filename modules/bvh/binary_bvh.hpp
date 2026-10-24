#ifndef ENGINE_MODULES_BVH_BINARY_BVH_HPP
#define ENGINE_MODULES_BVH_BINARY_BVH_HPP

// The implementation behind "bvh.binary": a compact axis-aligned bounding
// box hierarchy for CPU-side spatial queries. Pure logic over plain float
// arrays: no Vulkan, no backend, no configuration surface.
//
// The structure is a binary BVH over caller-supplied AABBs, built by median
// split on the longest axis (surface-area heuristics buy little at the
// object counts the engine works with - the registry caps at 32 - and a
// median split is deterministic, which keeps simulation runs reproducible).

#include "bvh_capability.hpp"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace engine
{

// The hierarchy. Build once per query batch (32 boxes: microseconds), query
// many times. Build takes the boxes by value and keeps its own copy - the
// caller's geometry may be reordered scenes, the hierarchy is a stable
// snapshot.
class BoundingVolumeHierarchy final
{
public:
    // Builds the hierarchy over the given boxes. Empty input yields an empty
    // hierarchy that answers every query with no results.
    void build(
        std::vector<AxisAlignedBoundingBox> boxes
    )
    {
        boxes_ = std::move(boxes);
        nodes_.clear();
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
    // position. Leaves hold one box; internal nodes split on the longest
    // axis at the median.
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

        // Longest axis, median split. A deterministic comparator on center
        // coordinates (with the index as tiebreak) keeps identical boxes
        // reproducibly ordered.
        std::size_t      axis     = 0;
        {
            float longestExtent = -1.0f;
            for (std::size_t candidate = 0; candidate < 3; ++candidate)
            {
                const float extent = rangeBounds.maxPoint[candidate] - rangeBounds.minPoint[candidate];
                if (extent > longestExtent)
                {
                    longestExtent = extent;
                    axis          = candidate;
                }
            }
        }
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

        const std::uint32_t leftPosition  = buildRange(order, middle);
        const std::uint32_t rightPosition = buildRange(order + middle, count - middle);
        // Rejecting this subtree skips past the right child (which itself
        // skips its subtree): the parent's skip is one past the last node of
        // its range, which is exactly the right child's skip value after the
        // right subtree was built.
        nodes_[nodePosition].skipSubtree = nodes_[rightPosition].skipSubtree;
        return nodePosition;
    }

    std::vector<AxisAlignedBoundingBox> boxes_;
    std::vector<Node>                   nodes_;
};

} // namespace engine

#endif // ENGINE_MODULES_BVH_BINARY_BVH_HPP
