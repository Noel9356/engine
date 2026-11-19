// broadphase.cpp - uniform spatial hash pair collection. Each solid body is
// inserted into every hash cell its AABB touches; dynamic and kinematic
// bodies then query their own cells for candidates. Sleeping pairs and
// filter-rejected pairs are dropped before the AABB test, and the surviving
// pairs are sorted and deduplicated so downstream stages stay deterministic.
//
// The grid is rebuilt every step into caller-owned scratch vectors: one flat
// array of (cell key, body index) entries sorted by key, plus the bucket
// offsets the binary searches need. No per-step allocations happen once the
// scratch capacities have warmed up.

#include "broadphase.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace engine::physics_rigid
{
namespace
{

constexpr std::uint64_t kCellCoordinateBits = 21; // cells per axis before wrap
constexpr std::uint64_t kCellCoordinateMask = (1ull << kCellCoordinateBits) - 1ull;

inline std::int64_t worldToCell(double value, double inverseCellSize)
{
    // Floor to the cell grid; the coordinate range covers +/- 2^20 cells,
    // far beyond any world this module's budgets allow.
    const double cell = std::floor(value * inverseCellSize);
    return static_cast<std::int64_t>(std::clamp(cell, -1048576.0, 1048575.0));
}

inline std::uint64_t makeCellKey(std::int64_t x, std::int64_t y, std::int64_t z)
{
    const std::uint64_t ux = static_cast<std::uint64_t>(x) & kCellCoordinateMask;
    const std::uint64_t uy = static_cast<std::uint64_t>(y) & kCellCoordinateMask;
    const std::uint64_t uz = static_cast<std::uint64_t>(z) & kCellCoordinateMask;
    return (ux << (2 * kCellCoordinateBits)) | (uy << kCellCoordinateBits) | uz;
}

inline std::uint64_t makeBodyPairKey(std::size_t indexA, std::size_t indexB)
{
    const std::size_t smaller = indexA < indexB ? indexA : indexB;
    const std::size_t larger  = indexA < indexB ? indexB : indexA;
    return (static_cast<std::uint64_t>(smaller) << 32) | static_cast<std::uint64_t>(larger);
}

bool collisionFilterAllows(const Body &a, const Body &b)
{
    return (a.categoryBits & b.maskBits) != 0u && (b.categoryBits & a.maskBits) != 0u;
}

// Sphere-AABB separation along the axes: the squared distance from the
// sphere center to the box, zero when the center is inside.
double sphereAabbDistanceSquared(const math::Vec3 &center, const math::Vec3 &minimum, const math::Vec3 &maximum)
{
    const double x = std::max(std::max(minimum.x - center.x, center.x - maximum.x), 0.0);
    const double y = std::max(std::max(minimum.y - center.y, center.y - maximum.y), 0.0);
    const double z = std::max(std::max(minimum.z - center.z, center.z - maximum.z), 0.0);
    return x * x + y * y + z * z;
}

} // namespace

bool aabbOverlaps(
    const math::Vec3 &minimumA, const math::Vec3 &maximumA, const math::Vec3 &minimumB, const math::Vec3 &maximumB)
{
    return minimumA.x <= maximumB.x && maximumA.x >= minimumB.x && minimumA.y <= maximumB.y && maximumA.y >= minimumB.y
        && minimumA.z <= maximumB.z && maximumA.z >= minimumB.z;
}

void collectBroadphasePairs(const World &world, double cellSize, BroadphaseScratch &scratch,
    std::vector<std::pair<std::size_t, std::size_t>> &outPairs)
{
    outPairs.clear();
    scratch.cellEntries.clear();
    if (cellSize <= 0.0 || world.bodies.empty())
    {
        return;
    }
    const double inverseCellSize = 1.0 / cellSize;

    // Insert every solid body (planes are unbounded and handled separately)
    // as one entry per touched cell, then sort the flat entry array by cell
    // key so every cell's candidates form one contiguous bucket.
    for (std::size_t index = 0; index < world.bodies.size(); ++index)
    {
        const Body &body = world.bodies[index];
        if (body.shape.kind == ShapeKind::Plane)
        {
            continue;
        }
        const std::int64_t cellXStart = worldToCell(body.aabbMinimum.x, inverseCellSize);
        const std::int64_t cellXEnd   = worldToCell(body.aabbMaximum.x, inverseCellSize);
        const std::int64_t cellYStart = worldToCell(body.aabbMinimum.y, inverseCellSize);
        const std::int64_t cellYEnd   = worldToCell(body.aabbMaximum.y, inverseCellSize);
        const std::int64_t cellZStart = worldToCell(body.aabbMinimum.z, inverseCellSize);
        const std::int64_t cellZEnd   = worldToCell(body.aabbMaximum.z, inverseCellSize);
        for (std::int64_t cellZ = cellZStart; cellZ <= cellZEnd; ++cellZ)
        {
            for (std::int64_t cellY = cellYStart; cellY <= cellYEnd; ++cellY)
            {
                for (std::int64_t cellX = cellXStart; cellX <= cellXEnd; ++cellX)
                {
                    scratch.cellEntries.emplace_back(makeCellKey(cellX, cellY, cellZ), static_cast<std::uint32_t>(index));
                }
            }
        }
    }
    if (scratch.cellEntries.empty())
    {
        return;
    }
    std::sort(
        scratch.cellEntries.begin(), scratch.cellEntries.end(), [](const BroadphaseEntry &a, const BroadphaseEntry &b)
        {
            return a.cellKey != b.cellKey ? a.cellKey < b.cellKey : a.bodyIndex < b.bodyIndex;
        });

    // Query: one probe per non-static solid body. The sorted entry array
    // makes every cell's candidates one contiguous run, located with a binary
    // search and walked in body order.
    for (std::size_t index = 0; index < world.bodies.size(); ++index)
    {
        const Body &body = world.bodies[index];
        if (body.shape.kind == ShapeKind::Plane || body.kind == BodyKind::Static)
        {
            continue; // static bodies never initiate pairs
        }
        const std::int64_t cellXStart = worldToCell(body.aabbMinimum.x, inverseCellSize);
        const std::int64_t cellXEnd   = worldToCell(body.aabbMaximum.x, inverseCellSize);
        const std::int64_t cellYStart = worldToCell(body.aabbMinimum.y, inverseCellSize);
        const std::int64_t cellYEnd   = worldToCell(body.aabbMaximum.y, inverseCellSize);
        const std::int64_t cellZStart = worldToCell(body.aabbMinimum.z, inverseCellSize);
        const std::int64_t cellZEnd   = worldToCell(body.aabbMaximum.z, inverseCellSize);
        for (std::int64_t cellZ = cellZStart; cellZ <= cellZEnd; ++cellZ)
        {
            for (std::int64_t cellY = cellYStart; cellY <= cellYEnd; ++cellY)
            {
                for (std::int64_t cellX = cellXStart; cellX <= cellXEnd; ++cellX)
                {
                    const std::uint64_t cellKey = makeCellKey(cellX, cellY, cellZ);
                    auto entry                 = std::lower_bound(scratch.cellEntries.begin(),
                        scratch.cellEntries.end(),
                        cellKey,
                        [](const BroadphaseEntry &candidate, std::uint64_t key) { return candidate.cellKey < key; });
                    for (; entry != scratch.cellEntries.end() && entry->cellKey == cellKey; ++entry)
                    {
                        const std::size_t candidateIndex = entry->bodyIndex;
                        if (candidateIndex == index)
                        {
                            continue;
                        }
                        const Body &candidate = world.bodies[candidateIndex];
                        if (candidate.kind == BodyKind::Static && body.kind == BodyKind::Static)
                        {
                            continue;
                        }
                        if (!candidate.awake && !body.awake)
                        {
                            continue;
                        }
                        if (!collisionFilterAllows(candidate, body))
                        {
                            continue;
                        }
                        if (!aabbOverlaps(
                                candidate.aabbMinimum, candidate.aabbMaximum, body.aabbMinimum, body.aabbMaximum))
                        {
                            continue;
                        }
                        const std::size_t indexA = std::min<std::size_t>(index, candidateIndex);
                        const std::size_t indexB = std::max<std::size_t>(index, candidateIndex);
                        outPairs.emplace_back(indexA, indexB);
                    }
                }
            }
        }
    }

    std::sort(outPairs.begin(), outPairs.end());
    outPairs.erase(std::unique(outPairs.begin(), outPairs.end()), outPairs.end());
}

bool sphereMayOverlapShape(
    const math::Vec3 &center, double radius, const math::Vec3 &aabbMinimum, const math::Vec3 &aabbMaximum)
{
    if (!(radius > 0.0))
    {
        return false;
    }
    return sphereAabbDistanceSquared(center, aabbMinimum, aabbMaximum) <= radius * radius;
}

} // namespace engine::physics_rigid
