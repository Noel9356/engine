// Internal record types for the skeletal_animation module.
//
// These structs are the persistent state stored in the module's handle-keyed
// maps. They are kept in a dedicated header so the algorithm helpers (sampling,
// event collection, blending) can reference them without re-declaring them.

#ifndef ENGINE_SKELETAL_ANIMATION_RECORDS_HPP
#define ENGINE_SKELETAL_ANIMATION_RECORDS_HPP

#include <engine/skeletal_animation/capabilities.hpp>
#include "math_utils.hpp"

#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <vector>

namespace engine::skeletal_animation
{

// Per-joint sampling cursor metadata, shared across translation units.
struct SkeletonRecord
{
    std::vector<JointDescription> bindPose;
    std::vector<std::uint32_t>    topologicalOrder;
    // Cached inverse bind pose matrices (column-major 4x4), one per joint.
    // Empty when the module is configured to compute them on demand or when
    // skeleton creation failed to build them.
    std::vector<float>            inverseBindMatrices;
};

// Track metadata references ranges in ClipRecord's packed keyframe buffers.
struct ClipTrack
{
    std::uint32_t          jointIndex         = 0;
    std::uint32_t          firstKeyframe      = 0;
    std::uint32_t          keyframeCount      = 0;
    std::uint32_t          rotationDataOffset = invalidKeyframeCursor;
    AnimationInterpolation interpolation     = AnimationInterpolation::Slerp;
};

struct ClipRecord
{
    std::string                    name;
    float                          duration      = 0.0f;
    std::uint64_t                  keyframeCount = 0;
    std::vector<ClipTrack>         tracks;
    std::vector<AnimationKeyframe> keyframes;
    // Entry i stores 1 / (time[i] - time[i - 1]); the first entry of each
    // track is zero. An unrepresentable reciprocal uses division at sample time.
    std::vector<float> inverseIntervals;
    // Slerp-only intervals are packed independently; angle and reciprocal sine
    // are precomputed at clip creation to remove acos/sin(angle) from sampling.
    std::vector<QuaternionInterval> rotationIntervals;
    std::vector<AnimationEventMarker> eventMarkers;
};

struct PlayerRecord
{
    AnimationClipHandle     clip     = invalidAnimationClip;
    float                   speed    = 1.0f;
    double                  time     = 0.0;
    bool                    looping  = true;
    bool                    playing  = true;
    bool                    finished = false;
    std::vector<std::uint32_t> trackKeyCursors;
    std::deque<AnimationPlayerEvent> events;
    bool eventQueueOverflowed = false;
};

} // namespace engine::skeletal_animation

#endif
