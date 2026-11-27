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
//
// Cell coordinates span +/- 2^30 cells per axis; beyond that range the
// coordinates clamp to the boundary cells, so far-flung bodies still meet
// every candidate they overlap, only through coarser buckets. At the
// default 2-meter cell the representable extent is roughly +/- 1 billion
// meters per axis. The three coordinate lanes mix into one 64-bit key;
// distinct cells sharing a key merely merge their candidate buckets, which
// the AABB overlap test rejects deterministically.
//
// Bodies whose AABB spans more grid cells than the per-body entry budget
// never enter the grid: the span, not the coordinate range, is what a huge
// body (or a small configured cell size) multiplies into an unbounded
// entry count. Oversized bodies pair directly against every solid body
// instead (same filters as the grid query), so they still meet every
// candidate they overlap at a bounded O(bodies) cost each, and the pair
// output stays exact and deterministic either way.

#include "broadphase.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace engine::physics_rigid
{
namespace
{

constexpr std::uint64_t kCellCoordinateBits = 31; // representable coordinate lanes per axis
constexpr std::int64_t  kCellCoordinateMinimum = -(1ll << (kCellCoordinateBits - 1));
constexpr std::int64_t  kCellCoordinateMaximum = (1ll << (kCellCoordinateBits - 1)) - 1ll;

// Per-body grid entry budget. A body touching more cells than this skips
// the grid and pairs directly. 4096 entries (12 bytes each) caps a
// non-oversized body at ~48 KiB of grid entries while keeping every
// reasonably sized body - a 30 m box at the default 2 m cell spans 16^3 =
// 4096 cells - in the grid; thin slabs stay in the grid at any length
// because only the product of the three spans counts.
constexpr std::int64_t kOversizedEntryBudget = 4096;

// True when a body touching spanX x spanY x spanZ cells exceeds the entry
// budget. The staged comparisons keep every product inside int64: once
// spanX (or spanX * spanY) is known to be within the budget, multiplying
// the next span (at most 2^31) cannot overflow.
inline bool spanExceedsBudget(std::int64_t spanX, std::int64_t spanY, std::int64_t spanZ)
{
    if (spanX > kOversizedEntryBudget)
    {
        return true;
    }
    if (spanX * spanY > kOversizedEntryBudget)
    {
        return true;
    }
    return spanX * spanY * spanZ > kOversizedEntryBudget;
}

inline std::int64_t worldToCell(double value, double inverseCellSize)
{
    // Floor to the cell grid; coordinates beyond the representable range
    // clamp to the boundary cells so insert and query agree everywhere.
    const double cell = std::floor(value * inverseCellSize);
    return static_cast<std::int64_t>(std::clamp(cell,
        static_cast<double>(kCellCoordinateMinimum), static_cast<double>(kCellCoordinateMaximum)));
}

// The clamped cell range a body's AABB touches on one axis, plus the cell
// count of that span (inclusive on both ends).
struct CellRange
{
    std::int64_t start = 0;
    std::int64_t end   = 0;

    std::int64_t count() const { return end - start + 1; }
};

inline CellRange axisCellRange(double minimum, double maximum, double inverseCellSize)
{
    CellRange range;
    range.start = worldToCell(minimum, inverseCellSize);
    range.end   = worldToCell(maximum, inverseCellSize);
    return range;
}

inline std::uint64_t makeCellKey(std::int64_t x, std::int64_t y, std::int64_t z)
{
    // 64-bit mix of the three coordinate lanes (the odd-multiplier mixes
    // the spatially adjacent coordinates across the whole word, so nearby
    // cells rarely share a key).
    std::uint64_t key = static_cast<std::uint64_t>(x) * 0x9E3779B185EBCA87ull;
    key ^= static_cast<std::uint64_t>(y) * 0xC2B2AE3D27D4EB4Full + 0x165667B19E3779F9ull;
    key ^= static_cast<std::uint64_t>(z) * 0xBF58476D1CE4E5B9ull + 0x94D049BB133111EBull;
    return key;
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
    scratch.oversizedBodies.clear();
    scratch.probeOversizedBodies.clear();
    if (cellSize <= 0.0 || world.bodies.empty())
    {
        return;
    }
    const double inverseCellSize = 1.0 / cellSize;

    // Insert every solid body (planes are unbounded and handled separately)
    // as one entry per touched cell, then sort the flat entry array by cell
    // key so every cell's candidates form one contiguous bucket. Bodies
    // whose AABB spans more cells than the entry budget are oversized: they
    // skip the grid entirely and pair directly in the pass below, so the
    // entry count a single body can add stays bounded.
    for (std::size_t index = 0; index < world.bodies.size(); ++index)
    {
        const Body &body = world.bodies[index];
        if (body.shape.kind == ShapeKind::Plane)
        {
            continue;
        }
        const CellRange rangeX = axisCellRange(body.aabbMinimum.x, body.aabbMaximum.x, inverseCellSize);
        const CellRange rangeY = axisCellRange(body.aabbMinimum.y, body.aabbMaximum.y, inverseCellSize);
        const CellRange rangeZ = axisCellRange(body.aabbMinimum.z, body.aabbMaximum.z, inverseCellSize);
        if (spanExceedsBudget(rangeX.count(), rangeY.count(), rangeZ.count()))
        {
            scratch.oversizedBodies.push_back(static_cast<std::uint32_t>(index));
            continue;
        }
        for (std::int64_t cellZ = rangeZ.start; cellZ <= rangeZ.end; ++cellZ)
        {
            for (std::int64_t cellY = rangeY.start; cellY <= rangeY.end; ++cellY)
            {
                for (std::int64_t cellX = rangeX.start; cellX <= rangeX.end; ++cellX)
                {
                    scratch.cellEntries.emplace_back(makeCellKey(cellX, cellY, cellZ), static_cast<std::uint32_t>(index));
                }
            }
        }
    }
    if (!scratch.oversizedBodies.empty())
    {
        // Deterministic direct-pass order regardless of the bodies' layout.
        std::sort(scratch.oversizedBodies.begin(), scratch.oversizedBodies.end());
    }
    if (!scratch.cellEntries.empty())
    {
        std::sort(
            scratch.cellEntries.begin(), scratch.cellEntries.end(), [](const BroadphaseEntry &a, const BroadphaseEntry &b)
            {
                return a.cellKey != b.cellKey ? a.cellKey < b.cellKey : a.bodyIndex < b.bodyIndex;
            });
    }

    // Query: one probe per non-static solid body. The sorted entry array
    // makes every cell's candidates one contiguous run, located with a binary
    // search and walked in body order. Oversized bodies never walk cells -
    // their pairs come from the direct pass below - so this loop stays
    // bounded no matter how large a body is.
    //
    // The probe AABB is the body's shape bounds inflated by the speculative
    // contact margin: two shapes within the margin count as touching (the
    // documented margin semantics), and their AABBs are then within the
    // margin too, so every margin-close pair still meets in a shared cell.
    // Without the inflation the grid dropped a pair the moment its AABBs
    // separated - at ANY positive shape separation for flush contacts - and
    // the speculative contacts the narrowphase is built to produce (resting
    // persistence, gap closing) never ran for non-plane pairs.
    const double margin = world.settings.contactMargin >= 0.0 ? world.settings.contactMargin : 0.0;
    for (std::size_t index = 0; index < world.bodies.size(); ++index)
    {
        const Body &body = world.bodies[index];
        if (body.shape.kind == ShapeKind::Plane || body.kind == BodyKind::Static)
        {
            continue; // static bodies never initiate pairs
        }
        const math::Vec3 probeMinimum{
            body.aabbMinimum.x - margin, body.aabbMinimum.y - margin, body.aabbMinimum.z - margin};
        const math::Vec3 probeMaximum{
            body.aabbMaximum.x + margin, body.aabbMaximum.y + margin, body.aabbMaximum.z + margin};
        const CellRange rangeX = axisCellRange(probeMinimum.x, probeMaximum.x, inverseCellSize);
        const CellRange rangeY = axisCellRange(probeMinimum.y, probeMaximum.y, inverseCellSize);
        const CellRange rangeZ = axisCellRange(probeMinimum.z, probeMaximum.z, inverseCellSize);
        if (spanExceedsBudget(rangeX.count(), rangeY.count(), rangeZ.count()))
        {
            // The margin-inflated probe can exceed the entry budget even
            // though the body's own AABB fit it (the body stayed in the
            // grid). The grid walk is unaffordable, but this body must
            // still initiate its pairs: statics never probe, so a purely
            // skipped probe would drop every static pair - a body like this
            // falls through the statics it should rest on. Route it through
            // the direct pass below instead.
            scratch.probeOversizedBodies.push_back(static_cast<std::uint32_t>(index));
            continue;
        }
        for (std::int64_t cellZ = rangeZ.start; cellZ <= rangeZ.end; ++cellZ)
        {
            for (std::int64_t cellY = rangeY.start; cellY <= rangeY.end; ++cellY)
            {
                for (std::int64_t cellX = rangeX.start; cellX <= rangeX.end; ++cellX)
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
                                candidate.aabbMinimum, candidate.aabbMaximum, probeMinimum, probeMaximum))
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

    // Oversized bodies pair directly against every solid body with the same
    // filters the grid query applies (both-static, both-asleep, category /
    // mask, and the margin-inflated AABB overlap so margin-close pairs
    // survive here exactly as they do in the grid). An oversized body is
    // absent from the grid, so a moving body's cell walk cannot find it;
    // this pass covers oversized x grid, oversized x oversized, and
    // static-oversized x moving pairs alike. The (min, max) normalization
    // plus the final sort/unique keeps the output identical to the
    // grid-only path.
    const auto pairDirectly = [&](std::uint32_t oversizedIndex)
    {
        const Body &oversized = world.bodies[oversizedIndex];
        const math::Vec3 oversizedProbeMinimum{
            oversized.aabbMinimum.x - margin, oversized.aabbMinimum.y - margin, oversized.aabbMinimum.z - margin};
        const math::Vec3 oversizedProbeMaximum{
            oversized.aabbMaximum.x + margin, oversized.aabbMaximum.y + margin, oversized.aabbMaximum.z + margin};
        for (std::size_t index = 0; index < world.bodies.size(); ++index)
        {
            if (index == oversizedIndex)
            {
                continue;
            }
            const Body &body = world.bodies[index];
            if (body.shape.kind == ShapeKind::Plane)
            {
                continue;
            }
            if (body.kind == BodyKind::Static && oversized.kind == BodyKind::Static)
            {
                continue;
            }
            if (!body.awake && !oversized.awake)
            {
                continue;
            }
            if (!collisionFilterAllows(body, oversized))
            {
                continue;
            }
            if (!aabbOverlaps(body.aabbMinimum, body.aabbMaximum, oversizedProbeMinimum, oversizedProbeMaximum))
            {
                continue;
            }
            const std::size_t indexA = std::min<std::size_t>(index, oversizedIndex);
            const std::size_t indexB = std::max<std::size_t>(index, oversizedIndex);
            outPairs.emplace_back(indexA, indexB);
        }
    };
    for (const std::uint32_t oversizedIndex : scratch.oversizedBodies)
    {
        pairDirectly(oversizedIndex);
    }
    // Probe-oversized bodies were skipped by the query walk above; they run
    // the same direct pairing here (ascending index order, so the pass stays
    // deterministic and the final sort/unique deduplicates any pair another
    // probing body already emitted through the grid).
    for (const std::uint32_t oversizedIndex : scratch.probeOversizedBodies)
    {
        pairDirectly(oversizedIndex);
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
