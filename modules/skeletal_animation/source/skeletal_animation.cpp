// skeletal_animation module implementation.
//
// The module owns the handle-keyed maps for skeletons, clips, and players and
// exposes the `SkeletalAnimationCapability` function table. Pose math,
// sampling, event collection, and blending live in dedicated translation
// units (math_utils.hpp, sampler, event_collector, blending) so this file
// stays focused on lifecycle and capability dispatch.

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/math/quaternion.hpp>
#include <engine/skeletal_animation/capabilities.hpp>
#include "blending.hpp"
#include "event_collector.hpp"
#include "lua_bridge.hpp"
#include "math_utils.hpp"
#include "records.hpp"
#include "sampler.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <new>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine::skeletal_animation
{
namespace
{

inline constexpr std::uint32_t invalidKeyframeCursorLocal = invalidKeyframeCursor;

bool takeHandle(std::uint32_t &nextHandle, std::uint32_t &output) noexcept
{
    if (nextHandle == 0)
        return false;
    output = nextHandle;
    nextHandle = nextHandle == std::numeric_limits<std::uint32_t>::max() ? 0u : nextHandle + 1u;
    return true;
}

// Populates `record.inverseBindMatrices` when `cacheInverseBindPose` is true.
// Returns true on success, false if any joint's bind pose is non-invertible
// (the cache is left empty in that case so the public API can still report a
// deterministic error).
bool tryCacheInverseBindPose(SkeletonRecord &record) noexcept
{
    const std::uint32_t jointCount = static_cast<std::uint32_t>(record.bindPose.size());
    if (jointCount == 0)
        return false;

    std::vector<float> globalMatrices(static_cast<std::size_t>(jointCount) * 16);
    for (const std::uint32_t joint : record.topologicalOrder)
    {
        float localMatrix[16];
        makeTransformMatrix(record.bindPose[joint], localMatrix);
        const std::int32_t parent = record.bindPose[joint].parentIndex;
        float *destination = globalMatrices.data() + static_cast<std::size_t>(joint) * 16;
        if (parent < 0)
            std::copy(localMatrix, localMatrix + 16, destination);
        else
        {
            float globalMatrix[16];
            multiplyColumnMajorMatrices(globalMatrices.data() + static_cast<std::size_t>(parent) * 16,
                localMatrix, globalMatrix);
            std::copy(globalMatrix, globalMatrix + 16, destination);
        }
    }

    std::vector<float> inverse(static_cast<std::size_t>(jointCount) * 16);
    for (std::uint32_t joint = 0; joint < jointCount; ++joint)
    {
        const float *global = globalMatrices.data() + static_cast<std::size_t>(joint) * 16;
        float *destination = inverse.data() + static_cast<std::size_t>(joint) * 16;
        if (!invertAffineMatrix(global, destination))
            return false;
    }
    record.inverseBindMatrices = std::move(inverse);
    return true;
}

class SkeletalAnimationModule final : public ModuleInterface
{
public:
    SkeletalAnimationModule()
        : manifest_(makeManifest())
    {
        assignCapabilityPointers();
        skeletons_.max_load_factor(0.7f);
        clips_.max_load_factor(0.7f);
        players_.max_load_factor(0.7f);
    }

    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    void construct(ParticipantContext &context) override
    {
        readPositiveLimit(context, "maximum_joints", 65536, maximumJoints_);
        readPositiveLimit(context, "maximum_clips", 65536, maximumClips_);
        readPositiveLimit(context, "maximum_players", 65536, maximumPlayers_);
        readPositiveLimit(context, "maximum_skeletons", 65536, maximumSkeletons_);
        readPositiveLimit(context, "maximum_tracks_per_clip", 65536, maximumTracksPerClip_);
        readPositiveLimit(context, "maximum_keyframes_per_track", 1'000'000, maximumKeyframesPerTrack_);
        readPositiveLimit(context, "maximum_total_keyframes", 100'000'000, maximumTotalKeyframes_);
        readPositiveLimit(context, "maximum_total_joints", 10'000'000, maximumTotalJoints_);
        readPositiveLimit(context, "maximum_blend_poses", 256, maximumBlendPoses_);
        readPositiveLimit(context, "maximum_total_sampling_cache_entries", 100'000'000,
            maximumTotalSamplingCacheEntries_);
        readPositiveLimit(context, "maximum_event_markers_per_clip", 65536, maximumEventMarkersPerClip_);
        readPositiveLimit(context, "maximum_events_per_player", 65536, maximumEventsPerPlayer_);
        readPositiveLimit(context, "maximum_total_pending_events", 100'000'000, maximumTotalPendingEvents_);
        readPositiveLimit(context, "maximum_total_event_markers", 100'000'000, maximumTotalEventMarkers_);
        if (const auto value = context.configurationIntegerInSection("skeletal_animation", "maximum_cursor_walk");
            value && *value >= 0 && *value <= 1'000'000)
            maximumCursorWalk_ = static_cast<std::uint32_t>(*value);

        samplingCacheEnabled_ = context.configurationBooleanInSection("skeletal_animation", "sampling_cache_enabled")
                                    .value_or(true);
        cacheInverseBindPose_ = context.configurationBooleanInSection(
                                    "skeletal_animation", "cache_inverse_bind_pose")
                                    .value_or(true);
        if (const auto value = context.configurationStringInSection("skeletal_animation", "default_interpolation"))
        {
            if (*value == "slerp")
                defaultInterpolation_ = AnimationInterpolation::Slerp;
            else if (*value == "nlerp")
                defaultInterpolation_ = AnimationInterpolation::Nlerp;
            else if (*value == "step")
                defaultInterpolation_ = AnimationInterpolation::Step;
        }

        scriptRuntime_ = context.tryAcquireCapability<script_runtime::ScriptRuntimeCapability>();
        if (script_runtime::canRegisterRuntimeInitializer(scriptRuntime_))
        {
            luaBridgeContext_.scriptRuntime = scriptRuntime_;
            luaBridgeContext_.animation = &capability_;
            luaBridgeContext_.maximumJoints = maximumJoints_;
            luaBridgeContext_.maximumSkeletonBuilders = maximumSkeletons_;
            luaBridgeContext_.maximumClipBuilders = maximumClips_;
            luaBridgeContext_.maximumTracksPerClip = maximumTracksPerClip_;
            luaBridgeContext_.maximumKeyframesPerTrack = maximumKeyframesPerTrack_;
            luaBridgeContext_.maximumEventMarkersPerClip = maximumEventMarkersPerClip_;
            luaBridgeContext_.maximumPendingEventMarkers = maximumTotalEventMarkers_;
            luaBridgeContext_.maximumPendingJoints = maximumTotalJoints_;
            luaBridgeContext_.maximumPendingKeyframes = maximumTotalKeyframes_;
            runtimeInitializer_ = script_runtime::tryRegisterRuntimeInitializer(
                scriptRuntime_, initializeSkeletalAnimationLuaRuntime, &luaBridgeContext_);
        }

        // Reserving the configured live-object budgets avoids rehash pauses
        // during object creation. Record payloads remain separately sized.
        skeletons_.reserve(maximumSkeletons_);
        clips_.reserve(maximumClips_);
        players_.reserve(maximumPlayers_);
    }

    void start() override
    {}

    void stop() override
    {
        script_runtime::unregisterRuntimeInitializer(scriptRuntime_, runtimeInitializer_);
        shutdownSkeletalAnimationLuaBridge(luaBridgeContext_);
        players_.clear();
        clips_.clear();
        skeletons_.clear();
        totalKeyframes_             = 0;
        totalJoints_                = 0;
        totalSamplingCacheEntries_  = 0;
        totalEventMarkers_          = 0;
        totalPendingEvents_         = 0;
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == SkeletalAnimationCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "skeletal_animation",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"skeletal_animation"},
            .optionalCapabilities = {"script_runtime"}};
    }

    template <typename T>
    static void readPositiveLimit(ParticipantContext &context, const char *key, std::int64_t maximum, T &target)
    {
        if (const auto value = context.configurationIntegerInSection("skeletal_animation", key);
            value && *value > 0 && *value <= maximum)
            target = static_cast<T>(*value);
    }

    void assignCapabilityPointers()
    {
        capability_.abi  = skeletalAnimationCapabilityAbi;
        capability_.self = this;
#define ENGINE_SKELETAL_ANIMATION_BIND(field, method) \
    capability_.field = [](void *self, auto... args) -> decltype(auto) \
    { return static_cast<SkeletalAnimationModule *>(self)->method(args...); }
        ENGINE_SKELETAL_ANIMATION_BIND(createSkeleton, createSkeletonImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(destroySkeleton, destroySkeletonImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(skeletonInfo, skeletonInfoImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(createClip, createClipImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(destroyClip, destroyClipImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(clipDuration, clipDurationImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(sampleClip, sampleClipImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(extractRootMotion, extractRootMotionImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(createPlayer, createPlayerImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(destroyPlayer, destroyPlayerImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(setPlayerClip, setPlayerClipImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(setPlayerSpeed, setPlayerSpeedImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(setPlayerPlaying, setPlayerPlayingImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(advancePlayer, advancePlayerImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(playerState, playerStateImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(evaluatePlayer, evaluatePlayerImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(blendPoses, blendPosesImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(blendPosesMasked, blendPosesMaskedImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(applyAdditivePose, applyAdditivePoseImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(buildGlobalTransforms, buildGlobalTransformsImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(playerEventCount, playerEventCountImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(copyPlayerEvents, copyPlayerEventsImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(clearPlayerEvents, clearPlayerEventsImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(setPlayerTime, setPlayerTimeImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(clipInfo, clipInfoImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(buildInverseBindPose, buildInverseBindPoseImpl);
        ENGINE_SKELETAL_ANIMATION_BIND(computeSkinningMatrices, computeSkinningMatricesImpl);
#undef ENGINE_SKELETAL_ANIMATION_BIND
    }

    static void setError(AnimationError *output, AnimationError value) noexcept
    {
        if (output != nullptr)
            *output = value;
    }

    void releaseSamplingCache(PlayerRecord &player) noexcept
    {
        totalSamplingCacheEntries_ -= player.trackKeyCursors.size();
        std::vector<std::uint32_t>().swap(player.trackKeyCursors);
    }

    void configureSamplingCache(PlayerRecord &player, std::size_t trackCount)
    {
        releaseSamplingCache(player);
        if (!samplingCacheEnabled_ || trackCount == 0
            || trackCount > maximumTotalSamplingCacheEntries_ - totalSamplingCacheEntries_)
            return;

        try
        {
            player.trackKeyCursors.assign(trackCount, invalidKeyframeCursorLocal);
            totalSamplingCacheEntries_ += trackCount;
        }
        catch (const std::bad_alloc &)
        {
            // Caching is an optimization. Allocation pressure falls back to
            // binary-search sampling without failing player creation.
            player.trackKeyCursors.clear();
        }
    }

    SkeletonHandle createSkeletonImpl(const SkeletonDescription *description, AnimationError *error)
    {
        setError(error, AnimationError::None);
        if (description == nullptr || description->joints == nullptr || description->jointCount == 0
            || description->jointCount > maximumJoints_)
        {
            setError(error, AnimationError::InvalidDescription);
            return invalidSkeleton;
        }
        if (skeletons_.size() >= maximumSkeletons_
            || totalJoints_ > maximumTotalJoints_
            || description->jointCount > maximumTotalJoints_ - totalJoints_)
        {
            setError(error, AnimationError::CapacityExceeded);
            return invalidSkeleton;
        }

        SkeletonRecord record;
        record.bindPose.assign(description->joints, description->joints + description->jointCount);
        for (std::uint32_t index = 0; index < description->jointCount; ++index)
        {
            JointDescription &joint = record.bindPose[index];
            if (joint.parentIndex < -1 || joint.parentIndex >= static_cast<std::int32_t>(description->jointCount)
                || joint.parentIndex == static_cast<std::int32_t>(index) || !finitePose(joint))
            {
                setError(error, AnimationError::InvalidDescription);
                return invalidSkeleton;
            }
            normalizeQuaternion(joint.rotation);
        }

        // Build a parent-before-child order once. Joint indices are not required
        // to be topological, so global-pose evaluation cannot assume parents
        // precede children in the caller's array.
        std::vector<std::uint8_t> visitState(description->jointCount, 0);
        std::vector<std::uint32_t> path;
        path.reserve(description->jointCount);
        record.topologicalOrder.reserve(description->jointCount);
        for (std::uint32_t index = 0; index < description->jointCount; ++index)
        {
            if (visitState[index] == 2)
                continue;
            path.clear();
            std::int32_t current = static_cast<std::int32_t>(index);
            while (current >= 0 && visitState[static_cast<std::size_t>(current)] == 0)
            {
                visitState[static_cast<std::size_t>(current)] = 1;
                path.push_back(static_cast<std::uint32_t>(current));
                current = record.bindPose[static_cast<std::size_t>(current)].parentIndex;
            }
            if (current >= 0 && visitState[static_cast<std::size_t>(current)] == 1)
            {
                setError(error, AnimationError::InvalidDescription);
                return invalidSkeleton;
            }
            while (!path.empty())
            {
                const std::uint32_t joint = path.back();
                path.pop_back();
                visitState[joint] = 2;
                record.topologicalOrder.push_back(joint);
            }
        }

        if (cacheInverseBindPose_)
            tryCacheInverseBindPose(record);

        SkeletonHandle handle = invalidSkeleton;
        if (!takeHandle(nextSkeleton_, handle))
        {
            setError(error, AnimationError::CapacityExceeded);
            return invalidSkeleton;
        }
        skeletons_.emplace(handle, std::move(record));
        totalJoints_ += description->jointCount;
        return handle;
    }

    void destroySkeletonImpl(SkeletonHandle handle)
    {
        const auto found = skeletons_.find(handle);
        if (found == skeletons_.end())
            return;
        totalJoints_ -= found->second.bindPose.size();
        skeletons_.erase(found);
    }

    bool skeletonInfoImpl(SkeletonHandle handle, std::uint32_t *outJointCount)
    {
        const auto found = skeletons_.find(handle);
        if (found == skeletons_.end() || outJointCount == nullptr)
            return false;
        *outJointCount = static_cast<std::uint32_t>(found->second.bindPose.size());
        return true;
    }

    AnimationClipHandle createClipImpl(const AnimationClipDescription *description, AnimationError *error)
    {
        setError(error, AnimationError::None);
        if (description == nullptr || !std::isfinite(description->durationSeconds)
            || description->durationSeconds <= 0.0f || (description->trackCount != 0 && description->tracks == nullptr)
            || (description->eventMarkerCount != 0 && description->eventMarkers == nullptr))
        {
            setError(error, AnimationError::InvalidDescription);
            return invalidAnimationClip;
        }
        if (clips_.size() >= maximumClips_ || description->trackCount > maximumTracksPerClip_
            || description->eventMarkerCount > maximumEventMarkersPerClip_
            || totalEventMarkers_ > maximumTotalEventMarkers_
            || description->eventMarkerCount > maximumTotalEventMarkers_ - totalEventMarkers_)
        {
            setError(error, AnimationError::CapacityExceeded);
            return invalidAnimationClip;
        }

        // Sort lightweight input references rather than allocating a bitmap
        // proportional to maximum_joints for every clip.
        std::vector<const AnimationTrackDescription *> orderedTracks;
        orderedTracks.reserve(description->trackCount);
        std::uint64_t requestedKeyframes = 0;
        for (std::uint32_t index = 0; index < description->trackCount; ++index)
        {
            const AnimationTrackDescription &track = description->tracks[index];
            if (track.keyframes == nullptr || track.jointIndex >= maximumJoints_)
            {
                setError(error, AnimationError::InvalidDescription);
                return invalidAnimationClip;
            }
            if (track.keyframeCount == 0 || track.keyframeCount > maximumKeyframesPerTrack_)
            {
                setError(error, AnimationError::CapacityExceeded);
                return invalidAnimationClip;
            }
            if (track.keyframeCount > maximumTotalKeyframes_
                || requestedKeyframes > maximumTotalKeyframes_ - track.keyframeCount)
            {
                setError(error, AnimationError::CapacityExceeded);
                return invalidAnimationClip;
            }
            requestedKeyframes += track.keyframeCount;
            orderedTracks.push_back(&track);
        }
        if (totalKeyframes_ > maximumTotalKeyframes_
            || requestedKeyframes > maximumTotalKeyframes_ - totalKeyframes_)
        {
            setError(error, AnimationError::CapacityExceeded);
            return invalidAnimationClip;
        }

        std::sort(orderedTracks.begin(), orderedTracks.end(), [](const auto *left, const auto *right)
        { return left->jointIndex < right->jointIndex; });
        for (std::size_t index = 1; index < orderedTracks.size(); ++index)
        {
            if (orderedTracks[index - 1]->jointIndex == orderedTracks[index]->jointIndex)
            {
                setError(error, AnimationError::InvalidDescription);
                return invalidAnimationClip;
            }
        }

        std::uint64_t slerpKeyframes = 0;
        for (const AnimationTrackDescription *source : orderedTracks)
        {
            const AnimationInterpolation interpolation = source->interpolation == AnimationInterpolation::Default
                ? defaultInterpolation_
                : source->interpolation;
            if (interpolation != AnimationInterpolation::Slerp && interpolation != AnimationInterpolation::Nlerp
                && interpolation != AnimationInterpolation::Step)
            {
                setError(error, AnimationError::InvalidDescription);
                return invalidAnimationClip;
            }
            if (interpolation == AnimationInterpolation::Slerp)
                slerpKeyframes += source->keyframeCount;
        }

        ClipRecord record;
        record.name          = description->name != nullptr ? description->name : "animation_clip";
        record.duration      = description->durationSeconds;
        record.keyframeCount = requestedKeyframes;
        record.tracks.reserve(description->trackCount);
        record.keyframes.reserve(static_cast<std::size_t>(requestedKeyframes));
        record.inverseIntervals.reserve(static_cast<std::size_t>(requestedKeyframes));
        record.rotationIntervals.reserve(static_cast<std::size_t>(slerpKeyframes));
        record.eventMarkers.reserve(description->eventMarkerCount);
        for (std::uint32_t index = 0; index < description->eventMarkerCount; ++index)
        {
            const AnimationEventMarker marker = description->eventMarkers[index];
            if (!std::isfinite(marker.timeSeconds) || marker.timeSeconds < 0.0f
                || marker.timeSeconds > record.duration)
            {
                setError(error, AnimationError::InvalidDescription);
                return invalidAnimationClip;
            }
            record.eventMarkers.push_back(marker);
        }
        std::stable_sort(record.eventMarkers.begin(), record.eventMarkers.end(),
            [](const AnimationEventMarker &left, const AnimationEventMarker &right)
            { return left.timeSeconds < right.timeSeconds; });

        for (const AnimationTrackDescription *source : orderedTracks)
        {
            AnimationInterpolation interpolation = source->interpolation;
            if (interpolation == AnimationInterpolation::Default)
                interpolation = defaultInterpolation_;
            ClipTrack track{};
            track.jointIndex    = source->jointIndex;
            track.firstKeyframe = static_cast<std::uint32_t>(record.keyframes.size());
            track.keyframeCount = source->keyframeCount;
            track.interpolation = interpolation;
            if (interpolation == AnimationInterpolation::Slerp)
                track.rotationDataOffset = static_cast<std::uint32_t>(record.rotationIntervals.size());

            bool  hasPrevious = false;
            float previousTime = 0.0f;
            for (std::uint32_t keyIndex = 0; keyIndex < source->keyframeCount; ++keyIndex)
            {
                AnimationKeyframe key = source->keyframes[keyIndex];
                if (!std::isfinite(key.timeSeconds) || key.timeSeconds < 0.0f
                    || key.timeSeconds > record.duration || (hasPrevious && key.timeSeconds <= previousTime))
                {
                    setError(error, AnimationError::InvalidDescription);
                    return invalidAnimationClip;
                }
                for (float value : key.translation)
                {
                    if (!std::isfinite(value))
                    {
                        setError(error, AnimationError::InvalidDescription);
                        return invalidAnimationClip;
                    }
                }
                for (float value : key.scale)
                {
                    if (!std::isfinite(value) || std::abs(value) < 1.0e-8f)
                    {
                        setError(error, AnimationError::InvalidDescription);
                        return invalidAnimationClip;
                    }
                }

                double rotationLengthSquared = 0.0;
                for (float value : key.rotation)
                {
                    if (!std::isfinite(value))
                    {
                        setError(error, AnimationError::InvalidDescription);
                        return invalidAnimationClip;
                    }
                    rotationLengthSquared += static_cast<double>(value) * value;
                }
                if (rotationLengthSquared < 1.0e-16 || !std::isfinite(rotationLengthSquared))
                {
                    setError(error, AnimationError::InvalidDescription);
                    return invalidAnimationClip;
                }
                normalizeQuaternion(key.rotation);

                float inverseInterval = 0.0f;
                QuaternionInterval rotationInterval{};
                if (hasPrevious)
                {
                    const double interval = static_cast<double>(key.timeSeconds) - previousTime;
                    const double reciprocal = 1.0 / interval;
                    if (reciprocal <= std::numeric_limits<float>::max())
                        inverseInterval = static_cast<float>(reciprocal);

                    if (interpolation == AnimationInterpolation::Slerp)
                    {
                        const AnimationKeyframe &previousKey = record.keyframes.back();
                        const float cosine = quaternionDot(previousKey.rotation, key.rotation);
                        const float absoluteCosine = std::clamp(std::abs(cosine), 0.0f, 1.0f);
                        if (absoluteCosine <= 0.9995f)
                        {
                            const float angle = std::acos(absoluteCosine);
                            rotationInterval.signedAngle = cosine < 0.0f ? -angle : angle;
                            rotationInterval.inverseSine = 1.0f / std::sin(angle);
                        }
                    }
                }
                record.keyframes.push_back(key);
                record.inverseIntervals.push_back(inverseInterval);
                if (interpolation == AnimationInterpolation::Slerp)
                    record.rotationIntervals.push_back(rotationInterval);
                previousTime = key.timeSeconds;
                hasPrevious = true;
            }
            record.tracks.push_back(track);
        }

        AnimationClipHandle handle = invalidAnimationClip;
        if (!takeHandle(nextClip_, handle))
        {
            setError(error, AnimationError::CapacityExceeded);
            return invalidAnimationClip;
        }
        clips_.emplace(handle, std::move(record));
        totalKeyframes_ += requestedKeyframes;
        totalEventMarkers_ += description->eventMarkerCount;
        return handle;
    }

    void destroyClipImpl(AnimationClipHandle handle)
    {
        const auto clip = clips_.find(handle);
        if (clip == clips_.end())
            return;
        totalKeyframes_ -= clip->second.keyframeCount;
        totalEventMarkers_ -= clip->second.eventMarkers.size();
        clips_.erase(clip);

        // Clip destruction is infrequent. Park only players currently using
        // this clip and release their optional cursor memory immediately.
        for (auto &[id, player] : players_)
        {
            (void)id;
            if (player.clip == handle)
            {
                player.clip    = invalidAnimationClip;
                player.playing = false;
                releaseSamplingCache(player);
            }
        }
    }

    bool clipDurationImpl(AnimationClipHandle handle, float *output)
    {
        const auto found = clips_.find(handle);
        if (found == clips_.end() || output == nullptr)
            return false;
        *output = found->second.duration;
        return true;
    }

    bool clipInfoImpl(AnimationClipHandle handle, AnimationClipInfo *outInfo)
    {
        if (outInfo == nullptr)
            return false;
        const auto found = clips_.find(handle);
        if (found == clips_.end())
            return false;
        const ClipRecord &clip = found->second;
        outInfo->name = clip.name.c_str();
        outInfo->durationSeconds = clip.duration;
        outInfo->trackCount = static_cast<std::uint32_t>(clip.tracks.size());
        outInfo->keyframeCount = clip.keyframeCount;
        outInfo->eventMarkerCount = static_cast<std::uint32_t>(clip.eventMarkers.size());
        return true;
    }

    bool sampleClipImpl(AnimationClipHandle clipHandle, SkeletonHandle skeletonHandle, float time, bool looping,
        JointDescription *output, std::uint32_t capacity, AnimationError *error)
    {
        const auto clip     = clips_.find(clipHandle);
        const auto skeleton = skeletons_.find(skeletonHandle);
        if (clip == clips_.end() || skeleton == skeletons_.end())
        {
            setError(error, AnimationError::InvalidHandle);
            return false;
        }
        return sampleClipInternal(clip->second, skeleton->second, time, looping, output, capacity, error,
            nullptr, maximumCursorWalk_);
    }

    bool extractRootMotionImpl(AnimationClipHandle clipHandle, SkeletonHandle skeletonHandle,
        std::uint32_t rootJoint, float startTime, float endTime, bool looping,
        float outTranslation[3], float outRotation[4], AnimationError *error)
    {
        const auto skeleton = skeletons_.find(skeletonHandle);
        const auto clip = clips_.find(clipHandle);
        if (skeleton == skeletons_.end() || clip == clips_.end())
        {
            setError(error, AnimationError::InvalidHandle);
            return false;
        }
        return extractRootMotionInternal(clip->second, skeleton->second, rootJoint, startTime, endTime, looping,
            outTranslation, outRotation, error);
    }

    AnimationClipHandle findClip(AnimationClipHandle handle) const
    {
        return clips_.contains(handle) ? handle : invalidAnimationClip;
    }

    AnimationPlayerHandle createPlayerImpl(const AnimationPlaybackDescription *description, AnimationError *error)
    {
        setError(error, AnimationError::None);
        if (description == nullptr || findClip(description->clip) == invalidAnimationClip
            || !std::isfinite(description->speed) || !std::isfinite(description->initialTimeSeconds))
        {
            setError(error, AnimationError::InvalidDescription);
            return invalidAnimationPlayer;
        }
        if (players_.size() >= maximumPlayers_)
        {
            setError(error, AnimationError::CapacityExceeded);
            return invalidAnimationPlayer;
        }

        PlayerRecord record;
        record.clip         = description->clip;
        record.speed        = description->speed;
        record.looping      = description->looping;
        record.playing      = description->playing;
        // One lookup for the clip duration + track count, replacing two
        // `clips_.at(record.clip)` calls in the previous implementation.
        const auto clipIter = clips_.find(record.clip);
        const double duration = static_cast<double>(clipIter->second.duration);
        const double initialTime = static_cast<double>(description->initialTimeSeconds);
        if (record.looping)
        {
            record.time = std::fmod(initialTime, duration);
            if (record.time < 0.0)
                record.time += duration;
        }
        else
            record.time = std::clamp(initialTime, 0.0, duration);
        record.finished = !record.looping && record.time >= duration;
        configureSamplingCache(record, clipIter->second.tracks.size());

        AnimationPlayerHandle handle = invalidAnimationPlayer;
        if (!takeHandle(nextPlayer_, handle))
        {
            releaseSamplingCache(record);
            setError(error, AnimationError::CapacityExceeded);
            return invalidAnimationPlayer;
        }
        players_.emplace(handle, std::move(record));
        return handle;
    }

    void destroyPlayerImpl(AnimationPlayerHandle handle)
    {
        const auto found = players_.find(handle);
        if (found == players_.end())
            return;
        releaseSamplingCache(found->second);
        totalPendingEvents_ -= found->second.events.size();
        players_.erase(found);
    }

    bool setPlayerClipImpl(AnimationPlayerHandle handle, AnimationClipHandle clip, bool restart)
    {
        const auto player = players_.find(handle);
        const auto target = clips_.find(clip);
        if (player == players_.end() || target == clips_.end())
            return false;

        PlayerRecord &record = player->second;
        if (record.clip != clip)
        {
            clearPlayerEventsInternal(record);
            releaseSamplingCache(record);
            record.clip = clip;
            configureSamplingCache(record, target->second.tracks.size());
            record.finished = false;
        }
        else if (record.trackKeyCursors.size() == target->second.tracks.size())
            std::fill(record.trackKeyCursors.begin(), record.trackKeyCursors.end(), invalidKeyframeCursorLocal);

        if (restart)
        {
            clearPlayerEventsInternal(record);
            record.time     = 0.0;
            record.finished = false;
        }
        else
        {
            const double duration = static_cast<double>(target->second.duration);
            if (record.looping)
            {
                record.time = std::fmod(record.time, duration);
                if (record.time < 0.0)
                    record.time += duration;
            }
            else
                record.time = std::clamp(record.time, 0.0, duration);
        }
        return true;
    }

    bool setPlayerSpeedImpl(AnimationPlayerHandle handle, float speed)
    {
        const auto player = players_.find(handle);
        if (player == players_.end() || !std::isfinite(speed))
            return false;
        player->second.speed = speed;
        return true;
    }

    bool setPlayerPlayingImpl(AnimationPlayerHandle handle, bool playing)
    {
        const auto player = players_.find(handle);
        if (player == players_.end())
            return false;
        player->second.playing = playing && player->second.clip != invalidAnimationClip && !player->second.finished;
        return true;
    }

    bool setPlayerTimeImpl(AnimationPlayerHandle handle, float timeSeconds)
    {
        const auto player = players_.find(handle);
        if (player == players_.end() || !std::isfinite(timeSeconds))
            return false;
        PlayerRecord &record = player->second;
        const auto clip = clips_.find(record.clip);
        if (clip == clips_.end())
        {
            record.playing = false;
            if (record.clip != invalidAnimationClip)
            {
                record.clip = invalidAnimationClip;
                releaseSamplingCache(record);
            }
            return false;
        }
        clearPlayerEventsInternal(record);
        const double duration = static_cast<double>(clip->second.duration);
        const double requestedTime = static_cast<double>(timeSeconds);
        if (record.looping)
        {
            record.time = std::fmod(requestedTime, duration);
            if (record.time < 0.0)
                record.time += duration;
            record.finished = false;
        }
        else
        {
            record.time = std::clamp(requestedTime, 0.0, duration);
            record.finished = record.time >= duration;
            record.playing = record.playing && !record.finished;
        }
        return true;
    }

    bool advancePlayerImpl(AnimationPlayerHandle handle, float deltaSeconds)
    {
        const auto player = players_.find(handle);
        if (player == players_.end() || !std::isfinite(deltaSeconds) || deltaSeconds < 0.0f)
            return false;

        PlayerRecord &record = player->second;
        const auto clip = clips_.find(record.clip);
        if (clip == clips_.end())
        {
            record.playing = false;
            if (record.clip != invalidAnimationClip)
            {
                record.clip = invalidAnimationClip;
                releaseSamplingCache(record);
            }
            return false;
        }
        if (!record.playing)
            return true;

        const double advancedTime = static_cast<double>(record.time)
            + static_cast<double>(deltaSeconds) * static_cast<double>(record.speed);
        if (!std::isfinite(advancedTime))
            return false;

        const double duration = static_cast<double>(clip->second.duration);
        if (!clip->second.eventMarkers.empty() && advancedTime != record.time)
        {
            collectPlayerEvents(record, clip->first, clip->second, advancedTime, duration,
                maximumEventsPerPlayer_, maximumTotalPendingEvents_, totalPendingEvents_);
        }
        if (record.looping)
        {
            double wrappedTime = std::fmod(advancedTime, duration);
            if (wrappedTime < 0.0)
                wrappedTime += duration;
            record.time = wrappedTime;
        }
        else if (advancedTime >= duration)
        {
            record.time     = duration;
            record.finished = true;
            record.playing  = false;
        }
        else if (advancedTime < 0.0)
        {
            record.time     = 0.0;
            record.finished = true;
            record.playing  = false;
        }
        else
            record.time = advancedTime;
        return true;
    }

    bool playerStateImpl(AnimationPlayerHandle handle, AnimationPlayerState *output)
    {
        const auto player = players_.find(handle);
        if (player == players_.end() || output == nullptr)
            return false;
        const auto clip = clips_.find(player->second.clip);
        output->timeSeconds    = static_cast<float>(player->second.time);
        output->normalizedTime = clip != clips_.end()
            ? static_cast<float>(player->second.time / static_cast<double>(clip->second.duration))
            : 0.0f;
        output->playing        = player->second.playing;
        output->finished       = player->second.finished;
        return true;
    }

    bool evaluatePlayerImpl(AnimationPlayerHandle playerHandle, SkeletonHandle skeleton, JointDescription *output,
        std::uint32_t capacity, AnimationError *error)
    {
        const auto player = players_.find(playerHandle);
        if (player == players_.end())
        {
            setError(error, AnimationError::InvalidHandle);
            return false;
        }
        const auto clip = clips_.find(player->second.clip);
        if (clip == clips_.end())
        {
            setError(error, AnimationError::InvalidHandle);
            return false;
        }
        const auto skeletonIter = skeletons_.find(skeleton);
        if (skeletonIter == skeletons_.end())
        {
            setError(error, AnimationError::InvalidHandle);
            return false;
        }
        auto *cursors = player->second.trackKeyCursors.empty() ? nullptr : &player->second.trackKeyCursors;
        return sampleClipInternal(clip->second, skeletonIter->second, static_cast<float>(player->second.time),
            player->second.looping, output, capacity, error, cursors, maximumCursorWalk_);
    }

    bool blendPosesImpl(const JointDescription *const *poses, const float *weights, std::uint32_t poseCount,
        std::uint32_t jointCount, JointDescription *output, std::uint32_t capacity, AnimationError *error)
    {
        return blendPosesInternal(poses, weights, nullptr, poseCount, jointCount, output, capacity, error,
            maximumJoints_, maximumBlendPoses_);
    }

    bool blendPosesMaskedImpl(const JointDescription *const *poses, const float *weights,
        const float *const *jointMasks, std::uint32_t poseCount, std::uint32_t jointCount,
        JointDescription *output, std::uint32_t capacity, AnimationError *error)
    {
        return blendPosesInternal(poses, weights, jointMasks, poseCount, jointCount, output, capacity, error,
            maximumJoints_, maximumBlendPoses_);
    }

    bool applyAdditivePoseImpl(const JointDescription *basePose, const JointDescription *additivePose,
        const JointDescription *referencePose, const float *jointWeights, std::uint32_t jointCount, float weight,
        JointDescription *output, std::uint32_t capacity, AnimationError *error)
    {
        return applyAdditivePoseInternal(basePose, additivePose, referencePose, jointWeights, jointCount, weight,
            output, capacity, error, maximumJoints_);
    }

    bool buildGlobalTransformsImpl(SkeletonHandle skeletonHandle, const JointDescription *localPose,
        float *outputMatrices, std::uint32_t matrixCapacity, AnimationError *error)
    {
        const auto skeleton = skeletons_.find(skeletonHandle);
        if (skeleton == skeletons_.end())
        {
            setError(error, AnimationError::InvalidHandle);
            return false;
        }
        return buildGlobalTransformsInternal(skeleton->second, localPose, outputMatrices, matrixCapacity, error);
    }

    bool buildInverseBindPoseImpl(SkeletonHandle skeletonHandle, float *outputMatrices,
        std::uint32_t matrixCapacity, AnimationError *error)
    {
        const auto skeleton = skeletons_.find(skeletonHandle);
        if (skeleton == skeletons_.end())
        {
            setError(error, AnimationError::InvalidHandle);
            return false;
        }
        return buildInverseBindPoseInternal(skeleton->second, outputMatrices, matrixCapacity, error);
    }

    bool computeSkinningMatricesImpl(SkeletonHandle skeletonHandle, const JointDescription *localPose,
        float *outputMatrices, std::uint32_t matrixCapacity, AnimationError *error)
    {
        const auto skeleton = skeletons_.find(skeletonHandle);
        if (skeleton == skeletons_.end())
        {
            setError(error, AnimationError::InvalidHandle);
            return false;
        }
        return computeSkinningMatricesInternal(skeleton->second, localPose, outputMatrices, matrixCapacity, error);
    }

    void clearPlayerEventsInternal(PlayerRecord &player) noexcept
    {
        const std::uint64_t dropped = clearPlayerEvents(player);
        totalPendingEvents_ -= std::min<std::uint64_t>(totalPendingEvents_, dropped);
    }

    bool playerEventCountImpl(AnimationPlayerHandle handle, std::uint32_t *outCount, bool *outOverflowed)
    {
        const auto found = players_.find(handle);
        if (found == players_.end() || outCount == nullptr)
            return false;
        *outCount = static_cast<std::uint32_t>(found->second.events.size());
        if (outOverflowed != nullptr)
            *outOverflowed = found->second.eventQueueOverflowed;
        return true;
    }

    bool copyPlayerEventsImpl(AnimationPlayerHandle handle, std::uint32_t firstIndex,
        AnimationPlayerEvent *output, std::uint32_t capacity, std::uint32_t *outCopied)
    {
        const auto found = players_.find(handle);
        if (found == players_.end() || outCopied == nullptr || (capacity != 0 && output == nullptr))
            return false;
        *outCopied = 0;
        if (firstIndex >= found->second.events.size() || capacity == 0)
            return true;
        const std::size_t available = found->second.events.size() - firstIndex;
        const std::size_t count = std::min<std::size_t>(available, capacity);
        auto event = found->second.events.begin() + static_cast<std::ptrdiff_t>(firstIndex);
        for (std::size_t index = 0; index < count; ++index, ++event)
            output[index] = *event;
        *outCopied = static_cast<std::uint32_t>(count);
        return true;
    }

    bool clearPlayerEventsImpl(AnimationPlayerHandle handle)
    {
        const auto found = players_.find(handle);
        if (found == players_.end())
            return false;
        clearPlayerEventsInternal(found->second);
        return true;
    }

    ParticipantManifest manifest_;
    SkeletalAnimationCapability capability_{};
    script_runtime::ScriptRuntimeCapability *scriptRuntime_ = nullptr;
    script_runtime::ScriptRuntimeInitializerHandle runtimeInitializer_ = script_runtime::invalidScriptRuntimeInitializer;
    SkeletalAnimationLuaBridgeContext luaBridgeContext_{};

    std::unordered_map<SkeletonHandle, SkeletonRecord> skeletons_;
    std::unordered_map<AnimationClipHandle, ClipRecord> clips_;
    std::unordered_map<AnimationPlayerHandle, PlayerRecord> players_;

    SkeletonHandle        nextSkeleton_     = 1;
    AnimationClipHandle   nextClip_         = 1;
    AnimationPlayerHandle nextPlayer_       = 1;
    std::uint32_t         maximumJoints_    = 4096;
    std::uint32_t         maximumClips_     = 4096;
    std::uint32_t         maximumPlayers_   = 4096;
    std::uint32_t         maximumSkeletons_ = 4096;
    std::uint32_t         maximumTracksPerClip_     = 4096;
    std::uint32_t         maximumKeyframesPerTrack_ = 65536;
    std::uint64_t         maximumTotalKeyframes_    = 8'000'000;
    std::uint64_t         maximumTotalJoints_       = 1'000'000;
    std::uint32_t         maximumBlendPoses_        = 16;
    std::uint32_t         maximumEventMarkersPerClip_ = 4096;
    std::uint32_t         maximumEventsPerPlayer_ = 256;
    std::uint64_t         maximumTotalEventMarkers_ = 1'000'000;
    std::uint64_t         maximumTotalPendingEvents_ = 1'000'000;
    std::uint64_t         maximumTotalSamplingCacheEntries_ = 1'000'000;
    bool                  samplingCacheEnabled_ = true;
    bool                  cacheInverseBindPose_ = true;
    std::uint32_t         maximumCursorWalk_ = 8;
    AnimationInterpolation defaultInterpolation_ = AnimationInterpolation::Slerp;

    std::uint64_t totalKeyframes_            = 0;
    std::uint64_t totalJoints_               = 0;
    std::uint64_t totalSamplingCacheEntries_  = 0;
    std::uint64_t totalEventMarkers_          = 0;
    std::uint64_t totalPendingEvents_         = 0;
};

} // namespace
} // namespace engine::skeletal_animation

extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const provided[] = {"skeletal_animation"};
    static const char *const optionalCapabilities[] = {"script_runtime"};
    static const engine::EngineModuleEntryTable entry
        = {.abiVersion     = engine::engineModuleAbiVersion,
            .moduleName    = "skeletal_animation",
            .moduleVersion = "0.1.0",
            .createModule  = []() -> engine::ModuleInterface *
            { return new engine::skeletal_animation::SkeletalAnimationModule(); },
            .providedCapabilities    = provided,
            .providedCapabilityCount = 1,
            .requiredCapabilities    = nullptr,
            .requiredCapabilityCount = 0,
            .optionalCapabilities    = optionalCapabilities,
            .optionalCapabilityCount = 1,
            .declaredPermissions     = nullptr,
            .declaredPermissionCount = 0};
    return &entry;
}
