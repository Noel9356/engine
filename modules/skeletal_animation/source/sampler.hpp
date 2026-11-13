// Sampling helpers shared by skeletal_animation translation units.
//
// `sampleClipInternal` walks a packed clip against a skeleton bind pose and
// writes a full joint pose. Per-track cursors (when present) avoid the binary
// search for sequential playback; the linear walk is bounded so a stale
// cursor degrades to binary search without producing invalid intervals.
// `extractRootMotion` reuses the shared quaternion-vector rotation helper
// from engine::math rather than open-coding the rotation formula.

#ifndef ENGINE_SKELETAL_ANIMATION_SAMPLER_HPP
#define ENGINE_SKELETAL_ANIMATION_SAMPLER_HPP

#include <engine/skeletal_animation/capabilities.hpp>
#include "math_utils.hpp"
#include "records.hpp"

#include <cstdint>
#include <vector>

namespace engine::skeletal_animation
{

// Locates the keyframe index whose interval contains `sampleTime`. When
// `cursor` is non-null and already in range, the function walks adjacent
// intervals (bounded by `maximumCursorWalk`); otherwise it falls back to
// `std::upper_bound` and writes the result back into `cursor`. The function
// assumes `keyCount >= 2` and `keys` is sorted ascending by `timeSeconds`.
std::uint32_t findLeftKey(const AnimationKeyframe *keys, std::uint32_t keyCount, float sampleTime,
    std::uint32_t *cursor, std::uint32_t maximumCursorWalk) noexcept;

// Samples a clip onto the caller-provided pose buffer. The bind pose of the
// skeleton is copied first; tracked joints overwrite their entries. The
// optional `trackCursors` enables sequential-playback acceleration; pass
// nullptr for stateless sampling (always binary search). Returns true on
// success and sets `*error` on failure.
bool sampleClipInternal(const ClipRecord &clip, const SkeletonRecord &skeleton, float time, bool looping,
    JointDescription *output, std::uint32_t capacity, AnimationError *error,
    std::vector<std::uint32_t> *trackCursors, std::uint32_t maximumCursorWalk) noexcept;

// Computes the translation and rotation delta of `rootJoint` between two clip
// times. The translation is expressed in the start pose's root frame; the
// rotation is `inverse(startRotation) * endRotation`. Returns false (and sets
// `*error`) when the inputs are invalid or the resulting transform is not
// finite.
bool extractRootMotionInternal(const ClipRecord &clip, const SkeletonRecord &skeleton, std::uint32_t rootJoint,
    float startTime, float endTime, bool looping, float outTranslation[3], float outRotation[4],
    AnimationError *error) noexcept;

} // namespace engine::skeletal_animation

#endif
