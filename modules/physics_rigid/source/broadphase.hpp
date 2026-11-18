// broadphase.hpp - uniform spatial hash broadphase over solid body AABBs.
// Planes are unbounded and never hashed; callers test them separately. The
// output is a deduplicated, index-sorted pair list so narrowphase and the
// solver run in a deterministic order every step.

#pragma once

#include "records.hpp"

#include <utility>
#include <vector>

namespace engine::physics_rigid
{

// Collects the candidate body pairs of one step: every unordered pair of
// solid bodies whose AABBs overlap a shared spatial-hash cell, filtered by
// motion kind (static-static pairs never collide), sleep state (two sleeping
// bodies are skipped), and the category/mask collision filter. The result is
// sorted by (indexA, indexB) and contains no duplicates.
void collectBroadphasePairs(
    const World &world, double cellSize, std::vector<std::pair<std::size_t, std::size_t>> &outPairs);

// AABB overlap test used by the broadphase and by queries.
bool aabbOverlaps(
    const math::Vec3 &minimumA, const math::Vec3 &maximumA, const math::Vec3 &minimumB, const math::Vec3 &maximumB);

} // namespace engine::physics_rigid
