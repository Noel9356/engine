// broadphase.cpp - uniform spatial hash pair collection. Each solid body is
// inserted into every hash cell its AABB touches; dynamic and kinematic
// bodies then query their own cells for candidates. Sleeping pairs and
// filter-rejected pairs are dropped before the AABB test, and the surviving
// pairs are sorted so downstream stages stay deterministic.

#include "broadphase.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>

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

} // namespace

bool aabbOverlaps(
    const math::Vec3 &minimumA, const math::Vec3 &maximumA, const math::Vec3 &minimumB, const math::Vec3 &maximumB)
{
    return minimumA.x <= maximumB.x && maximumA.x >= minimumB.x && minimumA.y <= maximumB.y && maximumA.y >= minimumB.y
        && minimumA.z <= maximumB.z && maximumA.z >= minimumB.z;
}

void collectBroadphasePairs(
    const World &world, double cellSize, std::vector<std::pair<std::size_t, std::size_t>> &outPairs)
{
    outPairs.clear();
    if (cellSize <= 0.0 || world.bodies.empty())
    {
        return;
    }
    const double inverseCellSize = 1.0 / cellSize;

    // Grid cells shared by the insert and query passes; the bucket vectors
    // are reused across steps through clear().
    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> grid;
    grid.reserve(world.bodies.size() * 2);

    // Insert every solid body (planes are unbounded and handled separately).
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
                    grid[makeCellKey(cellX, cellY, cellZ)].push_back(static_cast<std::uint32_t>(index));
                }
            }
        }
    }

    // Query: one probe per solid body, deduplicated pair keys.
    std::unordered_set<std::uint64_t> seenPairs;
    seenPairs.reserve(world.bodies.size() * 2);
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
                    const auto cellIterator = grid.find(makeCellKey(cellX, cellY, cellZ));
                    if (cellIterator == grid.end())
                    {
                        continue;
                    }
                    for (const std::uint32_t candidateIndex : cellIterator->second)
                    {
                        if (static_cast<std::size_t>(candidateIndex) == index)
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
                        const std::uint64_t pairKey = makeBodyPairKey(index, candidateIndex);
                        if (!seenPairs.insert(pairKey).second)
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
}

} // namespace engine::physics_rigid
