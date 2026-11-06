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
