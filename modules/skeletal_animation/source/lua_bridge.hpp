#ifndef ENGINE_SKELETAL_ANIMATION_LUA_BRIDGE_HPP
#define ENGINE_SKELETAL_ANIMATION_LUA_BRIDGE_HPP

#include <engine/script_runtime/capabilities.hpp>
#include <engine/skeletal_animation/capabilities.hpp>

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine::skeletal_animation
{

struct SkeletalAnimationLuaBridgeContext
{
    struct SkeletonBuilder
    {
        std::vector<JointDescription> joints;
        std::vector<std::uint8_t> assigned;
    };

    struct TrackBuilder
    {
        std::uint32_t jointIndex = 0;
        AnimationInterpolation interpolation = AnimationInterpolation::Default;
        std::vector<AnimationKeyframe> keyframes;
    };

    struct ClipBuilder
    {
        std::string name;
        float durationSeconds = 0.0f;
        std::vector<TrackBuilder> tracks;
        std::vector<AnimationEventMarker> eventMarkers;
    };

    script_runtime::ScriptRuntimeCapability *scriptRuntime = nullptr;
    SkeletalAnimationCapability *animation = nullptr;
    std::mutex mutex;
    std::mutex apiMutex;
    std::unordered_map<std::uint32_t, SkeletonBuilder> skeletonBuilders;
    std::unordered_map<std::uint32_t, ClipBuilder> clipBuilders;
    std::uint64_t nextBuilder = 1;
    std::uint32_t maximumJoints = 0;
    std::uint32_t maximumSkeletonBuilders = 0;
    std::uint32_t maximumClipBuilders = 0;
    std::uint32_t maximumTracksPerClip = 0;
    std::uint32_t maximumKeyframesPerTrack = 0;
    std::uint32_t maximumEventMarkersPerClip = 0;
    std::uint64_t maximumPendingEventMarkers = 0;
    std::uint64_t maximumPendingJoints = 0;
    std::uint64_t maximumPendingKeyframes = 0;
    std::uint64_t pendingJoints = 0;
    std::uint64_t pendingKeyframes = 0;
    std::uint64_t pendingEventMarkers = 0;
};

bool initializeSkeletalAnimationLuaRuntime(
    void *userData, script_runtime::ScriptRuntimeHandle runtime) noexcept;
void shutdownSkeletalAnimationLuaBridge(SkeletalAnimationLuaBridgeContext &context) noexcept;

} // namespace engine::skeletal_animation

#endif
