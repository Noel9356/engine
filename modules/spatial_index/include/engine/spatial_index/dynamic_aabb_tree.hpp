// dynamic_aabb_tree.hpp - incrementally updated AABB tree for moving proxies.
//
// Classic dynamic BVH: leaves store proxies with fat bounds, inserts/updates
// descend by cheapest area increase, and ancestors are rebalanced with
// rotations. Handles are generation-tagged so stale handles fail safely.
// Header-only; not internally synchronized.

#ifndef ENGINE_SPATIAL_INDEX_DYNAMIC_AABB_TREE_HPP
#define ENGINE_SPATIAL_INDEX_DYNAMIC_AABB_TREE_HPP

#include <engine/spatial_index/aabb.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>

namespace engine::spatial
{

// Tree tuning; setters clamp the values (fatMargin 0-10000, multiplier 0-1000).
struct DynamicAabbTreeOptions
{
    float fatMargin = 0.1f; // Extra world-unit padding grown around every new/updated bound.

    float displacementMultiplier = 2.0f; // How much predicted motion is pre-grown into the fat bound.
};

// Batched proxy change for updateMany: new bounds plus the per-axis velocity/
// displacement estimate used to pre-grow the replacement fat bound.
struct DynamicProxyUpdate
{
    ProxyHandle          handle = invalidProxy;
    Aabb                 bounds{};
    std::array<float, 3> displacement{};
};

// Dynamic AABB tree over ProxyHandle entries. insert/update/remove keep the
// tree balanced via rotations; query() visits fat bounds but confirms hits on
// the tight bounds, so fat padding never produces false hits.
class DynamicAabbTree final
{
public:
    explicit DynamicAabbTree(float fatMargin = 0.1f) noexcept
    {
        setOptions({fatMargin, 2.0f});
    }

    explicit DynamicAabbTree(const DynamicAabbTreeOptions &options) noexcept
    {
        setOptions(options);
    }

    void setOptions(const DynamicAabbTreeOptions &options) noexcept
    {
        setFatMargin(options.fatMargin);
        setDisplacementMultiplier(options.displacementMultiplier);
    }

    [[nodiscard]] DynamicAabbTreeOptions options() const noexcept
    {
        return {fatMargin_, displacementMultiplier_};
    }

    void setFatMargin(float value) noexcept
    {
        fatMargin_ = std::isfinite(value) ? std::clamp(value, 0.0f, 10000.0f) : 0.1f;
    }

    [[nodiscard]] float fatMargin() const noexcept
    {
        return fatMargin_;
    }

    void setDisplacementMultiplier(float value) noexcept
    {
        displacementMultiplier_ = std::isfinite(value) ? std::clamp(value, 0.0f, 1000.0f) : 2.0f;
    }

    [[nodiscard]] float displacementMultiplier() const noexcept
    {
        return displacementMultiplier_;
    }
    [[nodiscard]] std::size_t size() const noexcept
    {
        return proxyCount_;
    }
    [[nodiscard]] bool empty() const noexcept
    {
        return proxyCount_ == 0;
    }

    // Adds a proxy with the given bounds. The stored bound is fattened by the
    // configured margin. Returns invalidProxy for invalid bounds, tree overflow,
    // or allocation failure; otherwise a handle valid until the proxy is freed.
    ProxyHandle insert(const Aabb &bounds, std::uint64_t userData,
        std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max())
    {
        if (!bounds.valid())
            return invalidProxy;

        if (nodes_.size() > static_cast<std::size_t>(invalidNode) - 3u)
            return invalidProxy;
        try
        {
            nodes_.reserve(nodes_.size() + (root_ == invalidNode ? 1u : 2u));
        }
        catch (...)
        {
            return invalidProxy;
        }
        const std::uint32_t leaf = allocateNode();
        if (leaf == invalidNode)
            return invalidProxy;
        Node &node        = nodes_[leaf];
        node.tightBounds  = bounds;
        node.bounds       = fatten(bounds, {});
        node.userData     = userData;
        node.categoryMask = categoryMask;
        node.height       = 0;
        node.left = node.right = invalidNode;
        insertLeaf(leaf);
        ++proxyCount_;
        return makeHandle(leaf, node.generation);
    }

    // Removes a proxy. The slot goes back to the free list and its generation
    // is bumped, so outstanding handles for it stop resolving.
    bool remove(ProxyHandle handle)
    {
        const std::uint32_t leaf = resolve(handle);
        if (leaf == invalidNode)
            return false;
        removeLeaf(leaf);
        freeNode(leaf);
        --proxyCount_;
        return true;
    }

    // Changes a proxy's category mask and recomputes the OR masks of all its
    // ancestors so query filtering stays correct.
    bool setCategoryMask(ProxyHandle handle, std::uint64_t categoryMask) noexcept
    {
        const std::uint32_t leaf = resolve(handle);
        if (leaf == invalidNode)
            return false;
        nodes_[leaf].categoryMask = categoryMask;
        std::uint32_t parent      = nodes_[leaf].parent;
        while (parent != invalidNode)
        {
            Node &node        = nodes_[parent];
            node.categoryMask = nodes_[node.left].categoryMask | nodes_[node.right].categoryMask;
            parent            = node.parent;
        }
        return true;
    }

    // Moves a proxy. If the new bounds still fit inside the current fat bound
    // the tree is untouched (only the tight bound is refreshed); otherwise the
    // leaf is removed and reinserted, pre-grown along `displacement` to reduce
    // future reinserts.
    bool update(ProxyHandle handle, const Aabb &bounds, const std::array<float, 3> &displacement = {})
    {
        if (!bounds.valid())
            return false;
        const std::uint32_t leaf = resolve(handle);
        if (leaf == invalidNode)
            return false;
        if (nodes_[leaf].bounds.contains(bounds))
        {
            nodes_[leaf].tightBounds = bounds;
            return true;
        }
        removeLeaf(leaf);
        nodes_[leaf].tightBounds = bounds;
        nodes_[leaf].bounds      = fatten(bounds, displacement);
        insertLeaf(leaf);
        return true;
    }

    // All-or-nothing batch of update(): every handle/bound is validated first,
    // then applied; nothing changes if any entry is invalid.
    bool updateMany(std::span<const DynamicProxyUpdate> updates)
    {
        for (const DynamicProxyUpdate &update : updates)
            if (!update.bounds.valid() || resolve(update.handle) == invalidNode)
                return false;
        for (const DynamicProxyUpdate &update : updates)
        {
            const std::uint32_t leaf = resolve(update.handle);
            if (nodes_[leaf].bounds.contains(update.bounds))
                nodes_[leaf].tightBounds = update.bounds;
            else
            {
                removeLeaf(leaf);
                nodes_[leaf].tightBounds = update.bounds;
                nodes_[leaf].bounds      = fatten(update.bounds, update.displacement);
                insertLeaf(leaf);
            }
        }
        return true;
    }

    // Area query over the tree; same fixed-stack traversal as StaticBvh (early
    // out with partial results if a path would overflow the 128-entry stack).
    QueryStatistics query(const Aabb &area, std::vector<SpatialHit> &out,
        std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max()) const
    {
        QueryStatistics stats{};
        if (!area.valid() || root_ == invalidNode || categoryMask == 0)
            return stats;
        std::array<std::uint32_t, 128> stack{};
        std::size_t                    stackSize = 0;
        stack[stackSize++]                       = root_;
        while (stackSize != 0)
        {
            const std::uint32_t index = stack[--stackSize];
            const Node         &node  = nodes_[index];
            ++stats.nodesVisited;
            if ((node.categoryMask & categoryMask) == 0 || !node.bounds.intersects(area))
                continue;
            if (node.left == invalidNode)
            {
                ++stats.leavesVisited;
                if ((node.categoryMask & categoryMask) != 0 && node.tightBounds.intersects(area))
                {
                    out.push_back({makeHandle(index, node.generation), node.userData, node.categoryMask});
                    ++stats.hits;
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

    // Sphere query over dynamic proxies. The fat bounds prune traversal and
    // tight bounds confirm each leaf hit before its generation-safe handle is emitted.
    QueryStatistics querySphere(const std::array<float, 3> &center, float radius,
        std::vector<SpatialHit> &out,
        std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max()) const
    {
        QueryStatistics stats{};
        if (root_ == invalidNode || categoryMask == 0 || !std::isfinite(center[0])
            || !std::isfinite(center[1]) || !std::isfinite(center[2])
            || !std::isfinite(radius) || radius < 0.0f)
            return stats;
        std::array<std::uint32_t, 128> stack{};
        std::size_t stackSize = 0;
        stack[stackSize++] = root_;
        while (stackSize != 0)
        {
            const std::uint32_t index = stack[--stackSize];
            const Node &node = nodes_[index];
            ++stats.nodesVisited;
            if ((node.categoryMask & categoryMask) == 0 || !query_detail::intersectsSphereValid(node.bounds, center, radius))
                continue;
            if (node.left == invalidNode)
            {
                ++stats.leavesVisited;
                if ((node.categoryMask & categoryMask) != 0
                    && query_detail::intersectsSphereValid(node.tightBounds, center, radius))
                {
                    out.push_back({makeHandle(index, node.generation), node.userData, node.categoryMask});
                    ++stats.hits;
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

    // Frustum query over dynamic proxies. Fat bounds are used for pruning and
    // tight bounds are used for the final leaf test.
    QueryStatistics queryFrustum(const Frustum3 &frustum, std::vector<SpatialHit> &out,
        std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max()) const
    {
        QueryStatistics stats{};
        if (!frustum.valid() || root_ == invalidNode || categoryMask == 0)
            return stats;
        std::array<std::uint32_t, 128> stack{};
        std::size_t stackSize = 0;
        stack[stackSize++] = root_;
        while (stackSize != 0)
        {
            const std::uint32_t index = stack[--stackSize];
            const Node &node = nodes_[index];
            ++stats.nodesVisited;
            if ((node.categoryMask & categoryMask) == 0 || !query_detail::intersectsFrustumValid(node.bounds, frustum))
                continue;
            if (node.left == invalidNode)
            {
                ++stats.leavesVisited;
                if ((node.categoryMask & categoryMask) != 0 && query_detail::intersectsFrustumValid(node.tightBounds, frustum))
                {
                    out.push_back({makeHandle(index, node.generation), node.userData, node.categoryMask});
                    ++stats.hits;
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

    // Full top-down rebuild with median splits, recycling the existing leaf
    // nodes in place and reusing the old internal-node slots as the new internal
    // nodes. Returns false (tree unchanged) on allocation failure or if the
    // leaf walk disagrees with proxyCount_.
    bool rebuild()
    {
        if (proxyCount_ < 2)
            return true;
        try
        {
            std::vector<std::uint32_t> leaves;
            leaves.reserve(proxyCount_);
            std::vector<std::uint32_t> stack;
            stack.reserve(nodes_.size());
            stack.push_back(root_);
            while (!stack.empty())
            {
                const std::uint32_t index = stack.back();
                stack.pop_back();
                const Node &node = nodes_[index];
                if (node.left == invalidNode)
                    leaves.push_back(index);
                else
                {
                    stack.push_back(node.left);
                    stack.push_back(node.right);
                }
            }
            if (leaves.size() != proxyCount_)
                return false;

            std::vector<std::uint8_t> isLeaf(nodes_.size(), 0u);
            for (const std::uint32_t index : leaves)
                isLeaf[index] = 1u;
            std::vector<std::uint32_t> internalSlots;
            internalSlots.reserve(nodes_.size() - leaves.size());
            std::vector<Node> rebuilt = nodes_;
            for (std::uint32_t index = 0; index < rebuilt.size(); ++index)
            {
                if (isLeaf[index] != 0u)
                {
                    Node &leaf  = rebuilt[index];
                    leaf.parent = leaf.left = leaf.right = invalidNode;
                    leaf.next                            = invalidNode;
                    leaf.height                          = 0;
                }
                else
                {
                    const std::uint32_t generation = rebuilt[index].generation;
                    rebuilt[index]                 = Node{};
                    rebuilt[index].generation      = generation == 0 ? 1u : generation;
                    rebuilt[index].height          = -1;
                    internalSlots.push_back(index);
                }
            }
            if (internalSlots.size() < leaves.size() - 1u)
                return false;
            std::size_t nextInternal = 0;
            auto        buildRange
                = [&](auto &&self, std::size_t first, std::size_t last, std::uint32_t parent) -> std::uint32_t
            {
                if (last - first == 1u)
                {
                    const std::uint32_t leaf = leaves[first];
                    rebuilt[leaf].parent     = parent;
                    return leaf;
                }
                std::array<float, 3> centroidMin = rebuilt[leaves[first]].bounds.center();
                std::array<float, 3> centroidMax = centroidMin;
                Aabb                 combined    = rebuilt[leaves[first]].bounds;
                for (std::size_t offset = first; offset < last; ++offset)
                {
                    const Node &leaf  = rebuilt[leaves[offset]];
                    combined          = merge(combined, leaf.bounds);
                    const auto center = leaf.bounds.center();
                    for (std::size_t axis = 0; axis < 3; ++axis)
                    {
                        centroidMin[axis] = std::min(centroidMin[axis], center[axis]);
                        centroidMax[axis] = std::max(centroidMax[axis], center[axis]);
                    }
                }
                std::size_t axis = 0;
                if (centroidMax[1] - centroidMin[1] > centroidMax[axis] - centroidMin[axis])
                    axis = 1;
                if (centroidMax[2] - centroidMin[2] > centroidMax[axis] - centroidMin[axis])
                    axis = 2;
                const std::size_t middle = first + (last - first) / 2u;
                std::nth_element(leaves.begin() + static_cast<std::ptrdiff_t>(first),
                    leaves.begin() + static_cast<std::ptrdiff_t>(middle),
                    leaves.begin() + static_cast<std::ptrdiff_t>(last),
                    [&](std::uint32_t lhs, std::uint32_t rhs)
                    {
                        const float a = rebuilt[lhs].bounds.center()[axis];
                        const float b = rebuilt[rhs].bounds.center()[axis];
                        return a == b ? lhs < rhs : a < b;
                    });
                const std::uint32_t index = internalSlots[nextInternal++];
                Node               &node  = rebuilt[index];
                node.parent               = parent;
                node.left = node.right    = invalidNode;
                node.next                 = invalidNode;
                node.height               = 1;
                node.bounds               = combined;
                node.categoryMask         = 0;
                const std::uint32_t left  = self(self, first, middle, index);
                const std::uint32_t right = self(self, middle, last, index);
                node.left                 = left;
                node.right                = right;
                node.height               = 1 + std::max(rebuilt[left].height, rebuilt[right].height);
                node.bounds               = merge(rebuilt[left].bounds, rebuilt[right].bounds);
                node.categoryMask         = rebuilt[left].categoryMask | rebuilt[right].categoryMask;
                return index;
            };
            const std::uint32_t newRoot     = buildRange(buildRange, 0u, leaves.size(), invalidNode);
            std::uint32_t       newFreeList = invalidNode;
            for (std::size_t reverse = rebuilt.size(); reverse != 0; --reverse)
            {
                Node &node = rebuilt[reverse - 1u];
                if (node.height < 0)
                {
                    node.next   = newFreeList;
                    newFreeList = static_cast<std::uint32_t>(reverse - 1u);
                }
            }
            nodes_.swap(rebuilt);
            root_     = newRoot;
            freeList_ = newFreeList;
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    // Ray query: appends exact tight-bound hits in tree traversal order.
    // categoryMask filters whole subtrees; out is not cleared. Traversal uses
    // a fixed stack and returns partial results if it would overflow.
    QueryStatistics raycast(const Ray3 &ray, std::vector<DynamicRayHit> &out,
        std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max()) const
    {
        QueryStatistics stats{};
        if (root_ == invalidNode || categoryMask == 0)
            return stats;
        std::array<std::uint32_t, 128> stack{};
        std::size_t stackSize = 0;
        stack[stackSize++] = root_;
        while (stackSize != 0)
        {
            const std::uint32_t index = stack[--stackSize];
            const Node &node = nodes_[index];
            ++stats.nodesVisited;
            if ((node.categoryMask & categoryMask) == 0 || !intersectRay(node.bounds, ray))
                continue;
            if (node.left == invalidNode)
            {
                ++stats.leavesVisited;
                float distance = 0.0f;
                if (intersectRay(node.tightBounds, ray, &distance))
                {
                    out.push_back({makeHandle(index, node.generation), node.userData, node.categoryMask, distance});
                    ++stats.hits;
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

    // Tree quality snapshot: node count, height, worst sibling-height
    // difference, and the summed-node-area / root-area ratio.
    [[nodiscard]] DynamicTreeStatistics statistics() const
    {
        DynamicTreeStatistics result{};
        if (root_ == invalidNode)
            return result;
        struct Pending
        {
            std::uint32_t index;
            std::uint32_t depth;
        };
        std::vector<Pending> stack;
        stack.reserve(nodes_.size());
        stack.push_back({root_, 0u});
        double areaSum = 0.0;
        while (!stack.empty())
        {
            const Pending pending = stack.back();
            stack.pop_back();
            const Node &node = nodes_[pending.index];
            ++result.nodesVisited;
            result.height = std::max(result.height, pending.depth);
            areaSum += node.bounds.surfaceArea();
            if (node.left == invalidNode)
                ++result.leaves;
            else
            {
                const int balanceValue = std::abs(nodes_[node.left].height - nodes_[node.right].height);
                result.maximumBalance  = std::max(result.maximumBalance, static_cast<std::uint32_t>(balanceValue));
                stack.push_back({node.left, pending.depth + 1u});
                stack.push_back({node.right, pending.depth + 1u});
            }
        }
        const double rootArea = nodes_[root_].bounds.surfaceArea();
        result.surfaceAreaRatio
            = rootArea > 0.0 ? areaSum / rootArea : (areaSum > 0.0 ? std::numeric_limits<double>::infinity() : 0.0);
        return result;
    }

    // Heuristic rebuild hint: true when the height exceeds maximumHeight or the
    // surface-area ratio exceeds maximumSurfaceAreaRatio (<= 0 picks the default
    // 8 * log2(proxies + 1)). Never triggers with fewer than two proxies.
    [[nodiscard]] bool needsRebuild(std::uint32_t maximumHeight = 64u, double maximumSurfaceAreaRatio = 0.0) const
    {
        if (proxyCount_ < 2)
            return false;

        if (!std::isfinite(maximumSurfaceAreaRatio) || maximumSurfaceAreaRatio <= 0.0)
            maximumSurfaceAreaRatio = 8.0 * std::log2(static_cast<double>(proxyCount_) + 1.0);
        const DynamicTreeStatistics current = statistics();
        return current.height > maximumHeight || current.surfaceAreaRatio > maximumSurfaceAreaRatio;
    }

    // Structural integrity check: parent/height/bounds/mask invariants and
    // exactly proxyCount_ leaves reachable from the root.
    [[nodiscard]] bool valid() const noexcept
    {
        if (root_ == invalidNode)
            return proxyCount_ == 0;
        std::size_t leaves = 0;
        return validateNode(root_, invalidNode, leaves) && leaves == proxyCount_;
    }

private:
    // One tree node. Leaves (height == 0) hold a proxy; internal nodes (height
    // >= 1) only aggregate. height < 0 marks a slot on the free list (or a
    // rebuild scratch slot). bounds is the fat traversal bound, tightBounds the
    // exact proxy bound used for hit tests; next links freed slots.
    struct Node
    {
        Aabb          bounds{};
        Aabb          tightBounds{};
        std::uint64_t userData     = 0;
        std::uint64_t categoryMask = std::numeric_limits<std::uint64_t>::max();
        std::uint32_t parent       = invalidNode;
        std::uint32_t left         = invalidNode;
        std::uint32_t right        = invalidNode;
        std::uint32_t next         = invalidNode; // Free-list link while unallocated.
        std::uint32_t generation   = 1;           // Stamped into handles; bumped on free.
        int           height       = -1;
    };

    // Handle encoding: generation in the high 32 bits, (slot + 1) in the low
    // 32 bits (0 encodes the invalid handle).
    static ProxyHandle makeHandle(std::uint32_t slot, std::uint32_t generation) noexcept
    {
        return (static_cast<std::uint64_t>(generation) << 32u) | (static_cast<std::uint64_t>(slot) + 1u);
    }

    // Decodes a handle back to its slot, rejecting stale generations, freed or
    // internal slots, and out-of-range indices; invalidNode means "no such leaf".
    std::uint32_t resolve(ProxyHandle handle) const noexcept
    {
        if (handle == invalidProxy)
            return invalidNode;
        const std::uint32_t encodedSlot = static_cast<std::uint32_t>(handle & 0xffffffffu);
        const std::uint32_t generation  = static_cast<std::uint32_t>(handle >> 32u);
        if (encodedSlot == 0 || generation == 0)
            return invalidNode;
        const std::uint32_t slot = encodedSlot - 1u;
        if (slot >= nodes_.size() || nodes_[slot].height != 0 || nodes_[slot].left != invalidNode
            || nodes_[slot].right != invalidNode || nodes_[slot].generation != generation)
            return invalidNode;
        return slot;
    }

    // Takes a slot from the free list (keeping its generation) or appends a new
    // one; invalidNode when the tree size limit is reached.
    std::uint32_t allocateNode()
    {
        if (freeList_ != invalidNode)
        {
            const std::uint32_t index      = freeList_;
            freeList_                      = nodes_[index].next;
            const std::uint32_t generation = nodes_[index].generation;
            nodes_[index]                  = Node{};
            nodes_[index].generation       = generation;
            nodes_[index].height           = 0;
            return index;
        }
        if (nodes_.size() >= static_cast<std::size_t>(invalidNode) - 1u)
            return invalidNode;
        nodes_.push_back({});
        nodes_.back().height = 0;
        return static_cast<std::uint32_t>(nodes_.size() - 1u);
    }

    // Returns a slot to the free list and invalidates all handles to it.
    void freeNode(std::uint32_t index) noexcept
    {
        Node &node  = nodes_[index];
        node.height = -1;
        node.parent = node.left = node.right = invalidNode;
        node.userData                        = 0;
        node.tightBounds                     = {};
        ++node.generation;
        if (node.generation == 0)
            node.generation = 1;
        node.next = freeList_;
        freeList_ = index;
    }

    // Fat bound for (re)insertion: fatMargin on every side plus, per axis, the
    // predicted displacement (multiplied by displacementMultiplier, sign-aware).
    Aabb fatten(const Aabb &bounds, const std::array<float, 3> &displacement) const noexcept
    {
        Aabb             result = expanded(bounds, fatMargin_);
        constexpr double limit  = std::numeric_limits<float>::max();
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            const double delta = std::isfinite(displacement[axis])
                                   ? static_cast<double>(displacement[axis]) * displacementMultiplier_
                                   : 0.0;
            if (delta < 0.0)
                result.minimum[axis]
                    = static_cast<float>(std::max(-limit, static_cast<double>(result.minimum[axis]) + delta));
            else
                result.maximum[axis]
                    = static_cast<float>(std::min(limit, static_cast<double>(result.maximum[axis]) + delta));
        }
        return result;
    }

    // Standard dynamic-BVH leaf insert. Descends from the root choosing the
    // child with the smallest inherited area increase, stops early when making
    // the current node the sibling is cheapest, splices in a new parent, then
    // refits and rebalances the ancestor chain. Leaves the tree unchanged if
    // the new parent node cannot be allocated.
    void insertLeaf(std::uint32_t leaf)
    {
        if (root_ == invalidNode)
        {
            root_                = leaf;
            nodes_[root_].parent = invalidNode;
            return;
        }
        const Aabb    leafBounds = nodes_[leaf].bounds;
        std::uint32_t sibling    = root_;
        while (nodes_[sibling].left != invalidNode)
        {
            const Node  &node       = nodes_[sibling];
            const double inherited  = 2.0 * (merge(node.bounds, leafBounds).surfaceArea() - node.bounds.surfaceArea());
            const double leftCost   = nodes_[node.left].left == invalidNode
                                        ? merge(nodes_[node.left].bounds, leafBounds).surfaceArea() + inherited
                                        : merge(nodes_[node.left].bounds, leafBounds).surfaceArea()
                                              - nodes_[node.left].bounds.surfaceArea() + inherited;
            const double rightCost  = nodes_[node.right].left == invalidNode
                                        ? merge(nodes_[node.right].bounds, leafBounds).surfaceArea() + inherited
                                        : merge(nodes_[node.right].bounds, leafBounds).surfaceArea()
                                              - nodes_[node.right].bounds.surfaceArea() + inherited;
            const double parentCost = 2.0 * merge(node.bounds, leafBounds).surfaceArea();
            if (parentCost < leftCost && parentCost < rightCost)
                break;
            sibling = leftCost < rightCost ? node.left : node.right;
        }

        const std::uint32_t oldParent = nodes_[sibling].parent;
        const std::uint32_t newParent = allocateNode();
        if (newParent == invalidNode)
            return;
        nodes_[newParent].parent       = oldParent;
        nodes_[newParent].bounds       = merge(leafBounds, nodes_[sibling].bounds);
        nodes_[newParent].height       = nodes_[sibling].height + 1;
        nodes_[newParent].categoryMask = nodes_[sibling].categoryMask | nodes_[leaf].categoryMask;
        nodes_[newParent].left         = sibling;
        nodes_[newParent].right        = leaf;
        nodes_[sibling].parent         = newParent;
        nodes_[leaf].parent            = newParent;
        if (oldParent == invalidNode)
            root_ = newParent;
        else if (nodes_[oldParent].left == sibling)
            nodes_[oldParent].left = newParent;
        else
            nodes_[oldParent].right = newParent;
        refitAndBalance(nodes_[newParent].parent);
    }

    // Detaches a leaf and replaces its parent with the leaf's sibling, then
    // rebalances from the grandparent; removing the root empties the tree.
    void removeLeaf(std::uint32_t leaf)
    {
        if (leaf == root_)
        {
            root_               = invalidNode;
            nodes_[leaf].parent = invalidNode;
            return;
        }
        const std::uint32_t parent      = nodes_[leaf].parent;
        const std::uint32_t grandParent = nodes_[parent].parent;
        const std::uint32_t sibling     = nodes_[parent].left == leaf ? nodes_[parent].right : nodes_[parent].left;
        if (grandParent != invalidNode)
        {
            if (nodes_[grandParent].left == parent)
                nodes_[grandParent].left = sibling;
            else
                nodes_[grandParent].right = sibling;
            nodes_[sibling].parent = grandParent;
            freeNode(parent);
            refitAndBalance(grandParent);
        }
        else
        {
            root_                  = sibling;
            nodes_[sibling].parent = invalidNode;
            freeNode(parent);
        }
        nodes_[leaf].parent = invalidNode;
    }

    // Single AVL-style rotation around `index` when its children's heights
    // differ by more than one; returns the new subtree root and rewrites the
    // bounds/masks/heights of the two rotated nodes.
    std::uint32_t balance(std::uint32_t index)
    {
        Node &a = nodes_[index];
        if (a.left == invalidNode || a.height < 2)
            return index;
        const std::uint32_t bIndex = a.left;
        const std::uint32_t cIndex = a.right;
        Node               &b      = nodes_[bIndex];
        Node               &c      = nodes_[cIndex];
        const int           delta  = c.height - b.height;
        if (delta > 1)
        {
            const std::uint32_t fIndex = c.left;
            const std::uint32_t gIndex = c.right;
            Node               &f      = nodes_[fIndex];
            Node               &g      = nodes_[gIndex];
            c.left                     = index;
            c.parent                   = a.parent;
            a.parent                   = cIndex;
            if (c.parent != invalidNode)
            {
                if (nodes_[c.parent].left == index)
                    nodes_[c.parent].left = cIndex;
                else
                    nodes_[c.parent].right = cIndex;
            }
            else
                root_ = cIndex;
            if (f.height > g.height)
            {
                c.right        = fIndex;
                a.right        = gIndex;
                g.parent       = index;
                f.parent       = cIndex;
                a.bounds       = merge(b.bounds, g.bounds);
                c.bounds       = merge(a.bounds, f.bounds);
                a.categoryMask = b.categoryMask | g.categoryMask;
                c.categoryMask = a.categoryMask | f.categoryMask;
                a.height       = 1 + std::max(b.height, g.height);
                c.height       = 1 + std::max(a.height, f.height);
            }
            else
            {
                c.right        = gIndex;
                a.right        = fIndex;
                f.parent       = index;
                g.parent       = cIndex;
                a.bounds       = merge(b.bounds, f.bounds);
                c.bounds       = merge(a.bounds, g.bounds);
                a.categoryMask = b.categoryMask | f.categoryMask;
                c.categoryMask = a.categoryMask | g.categoryMask;
                a.height       = 1 + std::max(b.height, f.height);
                c.height       = 1 + std::max(a.height, g.height);
            }
            return cIndex;
        }
        if (delta < -1)
        {
            const std::uint32_t dIndex = b.left;
            const std::uint32_t eIndex = b.right;
            Node               &d      = nodes_[dIndex];
            Node               &e      = nodes_[eIndex];
            b.left                     = index;
            b.parent                   = a.parent;
            a.parent                   = bIndex;
            if (b.parent != invalidNode)
            {
                if (nodes_[b.parent].left == index)
                    nodes_[b.parent].left = bIndex;
                else
                    nodes_[b.parent].right = bIndex;
            }
            else
                root_ = bIndex;
            if (d.height > e.height)
            {
                b.right        = dIndex;
                a.left         = eIndex;
                e.parent       = index;
                d.parent       = bIndex;
                a.bounds       = merge(c.bounds, e.bounds);
                b.bounds       = merge(a.bounds, d.bounds);
                a.categoryMask = c.categoryMask | e.categoryMask;
                b.categoryMask = a.categoryMask | d.categoryMask;
                a.height       = 1 + std::max(c.height, e.height);
                b.height       = 1 + std::max(a.height, d.height);
            }
            else
            {
                b.right        = eIndex;
                a.left         = dIndex;
                d.parent       = index;
                e.parent       = bIndex;
                a.bounds       = merge(c.bounds, d.bounds);
                b.bounds       = merge(a.bounds, e.bounds);
                a.categoryMask = c.categoryMask | d.categoryMask;
                b.categoryMask = a.categoryMask | e.categoryMask;
                a.height       = 1 + std::max(c.height, d.height);
                b.height       = 1 + std::max(a.height, e.height);
            }
            return bIndex;
        }
        return index;
    }

    // Rotates and refits from `index` up to the root.
    void refitAndBalance(std::uint32_t index)
    {
        while (index != invalidNode)
        {
            index      = balance(index);
            Node &node = nodes_[index];
            if (node.left != invalidNode)
            {
                node.height       = 1 + std::max(nodes_[node.left].height, nodes_[node.right].height);
                node.bounds       = merge(nodes_[node.left].bounds, nodes_[node.right].bounds);
                node.categoryMask = nodes_[node.left].categoryMask | nodes_[node.right].categoryMask;
            }
            index = node.parent;
        }
    }

    // Recursive invariant check used by valid(): parent links, heights,
    // containment, and mask aggregation.
    bool validateNode(std::uint32_t index, std::uint32_t parent, std::size_t &leaves) const noexcept
    {
        if (index >= nodes_.size())
            return false;
        const Node &node = nodes_[index];
        if (node.height < 0 || node.parent != parent || !node.bounds.valid())
            return false;
        if (node.left == invalidNode)
        {
            ++leaves;
            return node.right == invalidNode && node.height == 0 && node.tightBounds.valid()
                && node.bounds.contains(node.tightBounds);
        }
        if (node.right == invalidNode || node.left >= nodes_.size() || node.right >= nodes_.size())
            return false;
        const Node &left  = nodes_[node.left];
        const Node &right = nodes_[node.right];
        if (!node.bounds.contains(left.bounds) || !node.bounds.contains(right.bounds))
            return false;
        if (node.height != 1 + std::max(left.height, right.height))
            return false;
        if (node.categoryMask != (left.categoryMask | right.categoryMask))
            return false;
        return validateNode(node.left, index, leaves) && validateNode(node.right, index, leaves);
    }

    std::vector<Node> nodes_;
    std::uint32_t     root_                   = invalidNode;
    std::uint32_t     freeList_               = invalidNode; // Head of the freed-slot chain (via Node::next).
    std::size_t       proxyCount_             = 0;
    float             fatMargin_              = 0.1f;
    float             displacementMultiplier_ = 2.0f;
};

} // namespace engine::spatial

#endif
