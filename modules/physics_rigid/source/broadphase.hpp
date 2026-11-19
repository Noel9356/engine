// broadphase.hpp - uniform spatial hash broadphase over solid body AABBs.
// Planes are unbounded and never hashed; callers test them separately. The
// output is a deduplicated, index-sorted pair list so narrowphase and the
// solver run in a deterministic order every step.

#pragma once

#include "records.hpp"

#include <cstdint>
#include <utility>
#include <vector>

namespace engine::physics_rigid
{

// One (cell key, body index) entry of the per-step hash grid. The entries
// sort by cell key, so every cell's candidates form one contiguous run.
struct BroadphaseEntry
{
    std::uint64_t cellKey   = 0;
    std::uint32_t bodyIndex = 0;
};

// Scratch owned by the module and reused across steps so the broadphase
// builds its grid without per-step allocations.
struct BroadphaseScratch
{
    std::vector<BroadphaseEntry> cellEntries;
};

// Collects the candidate body pairs of one step: every unordered pair of
// solid bodies whose AABBs overlap a shared spatial-hash cell, filtered by
// motion kind (static-static pairs never collide), sleep state (two sleeping
// bodies are skipped), and the category/mask collision filter. The result is
// sorted by (indexA, indexB) and contains no duplicates. The scratch buffers
// are cleared and refilled every call.
void collectBroadphasePairs(const World &world, double cellSize, BroadphaseScratch &scratch,
    std::vector<std::pair<std::size_t, std::size_t>> &outPairs);

// Conservative sphere-versus-body pre-filter: the body's shape AABB is an
// exact world-space bound of the shape, so a sphere that intersects the
// shape always passes this test. Queries use it to skip narrow-phase work.
bool sphereMayOverlapShape(
    const math::Vec3 &center, double radius, const math::Vec3 &aabbMinimum, const math::Vec3 &aabbMaximum);

// AABB overlap test used by the broadphase and by queries.
bool aabbOverlaps(
    const math::Vec3 &minimumA, const math::Vec3 &maximumA, const math::Vec3 &minimumB, const math::Vec3 &maximumB);

} // namespace engine::physics_rigid
