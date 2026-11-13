// aabb.hpp - axis-aligned bounding box primitive for the spatial_index module.
//
// Value type plus the free functions (merge, expanded) the tree/BVH builders
// rely on. Coordinates are floats; all operations are noexcept and branch-free
// of allocation.

#ifndef ENGINE_SPATIAL_INDEX_AABB_HPP
#define ENGINE_SPATIAL_INDEX_AABB_HPP

#include <engine/spatial_index/types.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace engine::spatial
{

// Axis-aligned bounding box in 3D, stored as component-wise min/max.
struct Aabb
{
    std::array<float, 3> minimum{};
    std::array<float, 3> maximum{};

    // True when all six bounds are finite and min <= max on every axis.
    [[nodiscard]] bool valid() const noexcept
    {
        for (std::size_t axis = 0; axis < 3; ++axis)
            if (!std::isfinite(minimum[axis]) || !std::isfinite(maximum[axis]) || minimum[axis] > maximum[axis])
                return false;
        return true;
    }

    // Midpoint per axis; computed in double to avoid overflow at ±FLT_MAX corners.
    [[nodiscard]] std::array<float, 3> center() const noexcept
    {
        return {
            static_cast<float>((static_cast<double>(minimum[0]) + maximum[0]) * 0.5),
            static_cast<float>((static_cast<double>(minimum[1]) + maximum[1]) * 0.5),
            static_cast<float>((static_cast<double>(minimum[2]) + maximum[2]) * 0.5)};
    }

    // Surface area of the box face sum; degenerate axes contribute 0. Used as
    // the cost heuristic by the tree builders (SAH) and quality metrics.
    [[nodiscard]] double surfaceArea() const noexcept
    {
        const double x = std::max(0.0, static_cast<double>(maximum[0]) - minimum[0]);
        const double y = std::max(0.0, static_cast<double>(maximum[1]) - minimum[1]);
        const double z = std::max(0.0, static_cast<double>(maximum[2]) - minimum[2]);
        return 2.0 * (x * y + y * z + z * x);
    }

    // Inclusive on both sides: touching boxes count as intersecting.
    [[nodiscard]] bool intersects(const Aabb &other) const noexcept
    {
        return minimum[0] <= other.maximum[0] && maximum[0] >= other.minimum[0] && minimum[1] <= other.maximum[1]
            && maximum[1] >= other.minimum[1] && minimum[2] <= other.maximum[2] && maximum[2] >= other.minimum[2];
    }

    [[nodiscard]] bool contains(const Aabb &other) const noexcept
    {
        return minimum[0] <= other.minimum[0] && minimum[1] <= other.minimum[1] && minimum[2] <= other.minimum[2]
            && maximum[0] >= other.maximum[0] && maximum[1] >= other.maximum[1] && maximum[2] >= other.maximum[2];
    }
};

// Intersects a ray with an AABB using a double-precision slab test. The
// returned distance is in world units, independent of direction magnitude.
// A ray starting inside the box reports distance 0.
[[nodiscard]] inline bool intersectRay(const Aabb &bounds, const Ray3 &ray, float *outDistance = nullptr) noexcept
{
    if (!bounds.valid() || !std::isfinite(ray.origin[0]) || !std::isfinite(ray.origin[1])
        || !std::isfinite(ray.origin[2]) || !std::isfinite(ray.direction[0])
        || !std::isfinite(ray.direction[1]) || !std::isfinite(ray.direction[2])
        || std::isnan(ray.maxDistance) || ray.maxDistance < 0.0f)
        return false;

    const double dx = ray.direction[0];
    const double dy = ray.direction[1];
    const double dz = ray.direction[2];
    const double directionLength = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!std::isfinite(directionLength) || directionLength <= 1.0e-20)
        return false;

    const double inverseLength = 1.0 / directionLength;
    const double direction[3] = {dx * inverseLength, dy * inverseLength, dz * inverseLength};
    double entry = 0.0;
    double exit = static_cast<double>(ray.maxDistance);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        const double origin = ray.origin[axis];
        if (std::abs(direction[axis]) <= 1.0e-15)
        {
            if (origin < bounds.minimum[axis] || origin > bounds.maximum[axis])
                return false;
            continue;
        }
        double first = (static_cast<double>(bounds.minimum[axis]) - origin) / direction[axis];
        double last = (static_cast<double>(bounds.maximum[axis]) - origin) / direction[axis];
        if (first > last)
            std::swap(first, last);
        entry = std::max(entry, first);
        exit = std::min(exit, last);
        if (entry > exit)
            return false;
    }
    if (exit < 0.0 || entry > static_cast<double>(ray.maxDistance)
        || entry > static_cast<double>(std::numeric_limits<float>::max()))
        return false;
    if (outDistance != nullptr)
        *outDistance = static_cast<float>(entry);
    return true;
}

namespace query_detail
{

// Traversal stack for recursive tree walks: a fixed array for the common
// shallow-tree case, migrating wholesale (order-preserving) to a per-thread
// heap buffer when a tree deeper than the array is encountered. This keeps
// deep or skewed trees from silently truncating query results while never
// allocating in the common case.
class TraversalStack
{
public:
    explicit TraversalStack(std::uint32_t root) noexcept
    {
        array_[arraySize_] = root;
        arraySize_         = 1u;
    }

    [[nodiscard]] bool empty() const noexcept
    {
        return arraySize_ == 0u && heap_.empty();
    }

    [[nodiscard]] std::uint32_t pop() noexcept
    {
        if (heap_.empty())
            return array_[--arraySize_];
        const std::uint32_t value = heap_.back();
        heap_.pop_back();
        return value;
    }

    // Pushes both children; the right child is visited first, matching the
    // classic `stack[right]; stack[left]` push order of fixed-array walks.
    // Returns false only when migrating to the heap buffer fails (allocation
    // failure); callers keep their previous overflow behaviour in that case.
    [[nodiscard]] bool pushChildren(std::uint32_t right, std::uint32_t left)
    {
        if (heap_.empty() && arraySize_ + 2u <= array_.size())
        {
            array_[arraySize_++] = right;
            array_[arraySize_++] = left;
            return true;
        }
        try
        {
            if (heap_.empty())
                heap_.assign(array_.begin(), array_.begin() + static_cast<std::ptrdiff_t>(arraySize_));
            heap_.push_back(right);
            heap_.push_back(left);
        }
        catch (...)
        {
            return false;
        }
        arraySize_ = 0u;
        return true;
    }

private:
    std::array<std::uint32_t, 128> array_{};
    std::size_t                    arraySize_ = 0u;
    std::vector<std::uint32_t>     heap_;
};

[[nodiscard]] inline bool intersectsSphereValid(const Aabb &bounds, const std::array<float, 3> &center,
    float radius) noexcept
{
    double distanceSquared = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        const double coordinate = center[axis];
        const double closest = std::clamp(coordinate, static_cast<double>(bounds.minimum[axis]),
            static_cast<double>(bounds.maximum[axis]));
        const double delta = coordinate - closest;
        distanceSquared += delta * delta;
    }
    return distanceSquared <= static_cast<double>(radius) * radius;
}

[[nodiscard]] inline bool intersectsFrustumValid(const Aabb &bounds, const Frustum3 &frustum) noexcept
{
    for (const Plane3 &plane : frustum.planes)
    {
        const double x = plane.normal[0] >= 0.0f ? bounds.maximum[0] : bounds.minimum[0];
        const double y = plane.normal[1] >= 0.0f ? bounds.maximum[1] : bounds.minimum[1];
        const double z = plane.normal[2] >= 0.0f ? bounds.maximum[2] : bounds.minimum[2];
        const double signedDistance = static_cast<double>(plane.normal[0]) * x
            + static_cast<double>(plane.normal[1]) * y + static_cast<double>(plane.normal[2]) * z
            + plane.distance;
        if (signedDistance < 0.0)
            return false;
    }
    return true;
}

} // namespace query_detail

// Exact sphere/AABB overlap test. A sphere radius must be finite and
// non-negative; the squared-distance calculation uses double precision.
[[nodiscard]] inline bool intersectsSphere(const Aabb &bounds, const std::array<float, 3> &center,
    float radius) noexcept
{
    return bounds.valid() && std::isfinite(center[0]) && std::isfinite(center[1])
        && std::isfinite(center[2]) && std::isfinite(radius) && radius >= 0.0f
        && query_detail::intersectsSphereValid(bounds, center, radius);
}

// Conservative AABB/view-frustum test. Plane normals point inward and the
// positive support vertex is tested against each plane. Invalid frusta reject.
[[nodiscard]] inline bool intersectsFrustum(const Aabb &bounds, const Frustum3 &frustum) noexcept
{
    return bounds.valid() && frustum.valid() && query_detail::intersectsFrustumValid(bounds, frustum);
}

// Component-wise union of two boxes.
[[nodiscard]] inline Aabb merge(const Aabb &lhs, const Aabb &rhs) noexcept
{
    Aabb result;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        result.minimum[axis] = std::min(lhs.minimum[axis], rhs.minimum[axis]);
        result.maximum[axis] = std::max(lhs.maximum[axis], rhs.maximum[axis]);
    }
    return result;
}

// Grows the box by margin on every side, clamped to finite float range;
// non-finite or negative margins are treated as 0.
[[nodiscard]] inline Aabb expanded(const Aabb &bounds, float margin) noexcept
{
    Aabb result           = bounds;
    margin                = std::isfinite(margin) ? std::max(0.0f, margin) : 0.0f;
    constexpr float limit = std::numeric_limits<float>::max();
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        result.minimum[axis] = static_cast<float>(
            std::max(-static_cast<double>(limit), static_cast<double>(result.minimum[axis]) - margin));
        result.maximum[axis] = static_cast<float>(
            std::min(static_cast<double>(limit), static_cast<double>(result.maximum[axis]) + margin));
    }
    return result;
}

} // namespace engine::spatial

#endif
