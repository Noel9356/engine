// skeletal_animation module implementation.
//
// Stores skeletons, immutable keyframed clips, playback clocks, and pose blends.
// Clip data is packed into contiguous buffers; per-player sampling cursors make
// sequential playback avoid a binary search for every animated joint.

#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/skeletal_animation/capabilities.hpp>
#include "lua_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
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

using Pose = JointDescription;

inline constexpr std::uint32_t invalidKeyframeCursor = std::numeric_limits<std::uint32_t>::max();

struct QuaternionInterval
{
    float signedAngle  = 0.0f;
    float inverseSine = 0.0f;
};

bool finitePose(const Pose &pose) noexcept
{
    for (float value : pose.translation)
        if (!std::isfinite(value))
            return false;
    for (float value : pose.rotation)
        if (!std::isfinite(value))
            return false;
    for (float value : pose.scale)
        if (!std::isfinite(value))
            return false;

    const double lengthSquared = static_cast<double>(pose.rotation[0]) * pose.rotation[0]
        + static_cast<double>(pose.rotation[1]) * pose.rotation[1]
        + static_cast<double>(pose.rotation[2]) * pose.rotation[2]
        + static_cast<double>(pose.rotation[3]) * pose.rotation[3];
    return lengthSquared > 1.0e-16 && std::isfinite(lengthSquared);
}

void normalizeQuaternion(float value[4]) noexcept
{
    const double lengthSquared = static_cast<double>(value[0]) * value[0]
        + static_cast<double>(value[1]) * value[1] + static_cast<double>(value[2]) * value[2]
        + static_cast<double>(value[3]) * value[3];
    const float inverseLength = static_cast<float>(1.0 / std::sqrt(lengthSquared));
    for (std::uint32_t index = 0; index < 4; ++index)
        value[index] *= inverseLength;
}

float quaternionDot(const float a[4], const float b[4]) noexcept
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

void nlerp(const float a[4], const float bInput[4], float t, float output[4]) noexcept
{
    const float sign = quaternionDot(a, bInput) < 0.0f ? -1.0f : 1.0f;
    for (std::uint32_t index = 0; index < 4; ++index)
        output[index] = a[index] + (bInput[index] * sign - a[index]) * t;
    normalizeQuaternion(output);
}

void quaternionMultiply(const float left[4], const float right[4], float output[4]) noexcept
{
    const float x = left[3] * right[0] + left[0] * right[3] + left[1] * right[2] - left[2] * right[1];
    const float y = left[3] * right[1] - left[0] * right[2] + left[1] * right[3] + left[2] * right[0];
    const float z = left[3] * right[2] + left[0] * right[1] - left[1] * right[0] + left[2] * right[3];
    const float w = left[3] * right[3] - left[0] * right[0] - left[1] * right[1] - left[2] * right[2];
    output[0] = x;
    output[1] = y;
    output[2] = z;
    output[3] = w;
}

void weightedQuaternionDelta(const float deltaInput[4], float weight, float output[4]) noexcept
{
    float delta[4] = {deltaInput[0], deltaInput[1], deltaInput[2], deltaInput[3]};
    if (delta[3] < 0.0f)
    {
        for (float &value : delta)
            value = -value;
    }

    const float halfAngle = std::acos(std::clamp(delta[3], -1.0f, 1.0f));
    const float sine = std::sin(halfAngle);
    if (std::abs(sine) > 1.0e-6f)
    {
        const float factor = std::sin(halfAngle * weight) / sine;
        output[0] = delta[0] * factor;
        output[1] = delta[1] * factor;
        output[2] = delta[2] * factor;
        output[3] = std::cos(halfAngle * weight);
    }
    else
    {
        output[0] = delta[0] * weight;
        output[1] = delta[1] * weight;
        output[2] = delta[2] * weight;
        output[3] = 1.0f;
    }
    normalizeQuaternion(output);
}

void makeTransformMatrix(const Pose &pose, float output[16]) noexcept
{
    float rotation[4] = {pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3]};
    normalizeQuaternion(rotation);
    const float x = rotation[0];
    const float y = rotation[1];
    const float z = rotation[2];
    const float w = rotation[3];

    const float r00 = 1.0f - 2.0f * (y * y + z * z);
    const float r01 = 2.0f * (x * y - z * w);
    const float r02 = 2.0f * (x * z + y * w);
    const float r10 = 2.0f * (x * y + z * w);
    const float r11 = 1.0f - 2.0f * (x * x + z * z);
    const float r12 = 2.0f * (y * z - x * w);
    const float r20 = 2.0f * (x * z - y * w);
    const float r21 = 2.0f * (y * z + x * w);
    const float r22 = 1.0f - 2.0f * (x * x + y * y);

    output[0] = r00 * pose.scale[0];
    output[1] = r10 * pose.scale[0];
    output[2] = r20 * pose.scale[0];
    output[3] = 0.0f;
    output[4] = r01 * pose.scale[1];
    output[5] = r11 * pose.scale[1];
    output[6] = r21 * pose.scale[1];
    output[7] = 0.0f;
    output[8] = r02 * pose.scale[2];
    output[9] = r12 * pose.scale[2];
    output[10] = r22 * pose.scale[2];
    output[11] = 0.0f;
    output[12] = pose.translation[0];
    output[13] = pose.translation[1];
    output[14] = pose.translation[2];
    output[15] = 1.0f;
}

void multiplyColumnMajorMatrices(const float left[16], const float right[16], float output[16]) noexcept
{
    float result[16];
    for (std::uint32_t column = 0; column < 4; ++column)
    {
        for (std::uint32_t row = 0; row < 4; ++row)
        {
            result[column * 4 + row] = left[row] * right[column * 4]
                + left[4 + row] * right[column * 4 + 1]
                + left[8 + row] * right[column * 4 + 2]
                + left[12 + row] * right[column * 4 + 3];
        }
    }
    std::copy(result, result + 16, output);
}

void copyKeyframe(const AnimationKeyframe &key, std::int32_t parentIndex, Pose &output) noexcept
{
    output.parentIndex = parentIndex;
    for (std::uint32_t axis = 0; axis < 3; ++axis)
    {
        output.translation[axis] = key.translation[axis];
        output.scale[axis]       = key.scale[axis];
    }
    for (std::uint32_t axis = 0; axis < 4; ++axis)
        output.rotation[axis] = key.rotation[axis];
}

void interpolateKeyframes(const AnimationKeyframe &left, const AnimationKeyframe &right, float t,
    AnimationInterpolation interpolation, const QuaternionInterval *rotationInterval, std::int32_t parentIndex,
    Pose &output) noexcept
{
    if (interpolation == AnimationInterpolation::Step || t <= 0.0f)
    {
        copyKeyframe(left, parentIndex, output);
        return;
    }
    if (t >= 1.0f)
    {
        copyKeyframe(right, parentIndex, output);
        return;
    }

    output.parentIndex = parentIndex;
    for (std::uint32_t axis = 0; axis < 3; ++axis)
    {
        output.translation[axis] = left.translation[axis] + (right.translation[axis] - left.translation[axis]) * t;
        output.scale[axis]       = left.scale[axis] + (right.scale[axis] - left.scale[axis]) * t;
    }

    if (interpolation == AnimationInterpolation::Nlerp)
    {
        nlerp(left.rotation, right.rotation, t, output.rotation);
    }
    else if (rotationInterval == nullptr || rotationInterval->signedAngle == 0.0f)
    {
        nlerp(left.rotation, right.rotation, t, output.rotation);
    }
    else
    {
        const float angle = std::abs(rotationInterval->signedAngle);
        const float leftWeight = std::sin((1.0f - t) * angle) * rotationInterval->inverseSine;
        const float rightWeight = std::sin(t * rotationInterval->signedAngle) * rotationInterval->inverseSine;
        for (std::uint32_t axis = 0; axis < 4; ++axis)
            output.rotation[axis] = left.rotation[axis] * leftWeight + right.rotation[axis] * rightWeight;
    }
}

bool takeHandle(std::uint32_t &nextHandle, std::uint32_t &output) noexcept
{
    if (nextHandle == 0)
        return false;
    output = nextHandle;
    nextHandle = nextHandle == std::numeric_limits<std::uint32_t>::max() ? 0u : nextHandle + 1u;
    return true;
}

struct SkeletonRecord
{
    std::vector<JointDescription> bindPose;
    std::vector<std::uint32_t>    topologicalOrder;
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
};

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
        if (const auto value = context.configurationIntegerInSection("skeletal_animation", "maximum_cursor_walk");
            value && *value >= 0 && *value <= 1'000'000)
            maximumCursorWalk_ = static_cast<std::uint32_t>(*value);

        samplingCacheEnabled_ = context.configurationBooleanInSection("skeletal_animation", "sampling_cache_enabled")
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
        if (scriptRuntime_ != nullptr && scriptRuntime_->abi == script_runtime::scriptRuntimeCapabilityAbi
            && scriptRuntime_->registerRuntimeInitializer != nullptr)
        {
            luaBridgeContext_.scriptRuntime = scriptRuntime_;
            luaBridgeContext_.animation = &capability_;
            luaBridgeContext_.maximumJoints = maximumJoints_;
            luaBridgeContext_.maximumSkeletonBuilders = maximumSkeletons_;
            luaBridgeContext_.maximumClipBuilders = maximumClips_;
            luaBridgeContext_.maximumTracksPerClip = maximumTracksPerClip_;
            luaBridgeContext_.maximumKeyframesPerTrack = maximumKeyframesPerTrack_;
            luaBridgeContext_.maximumPendingJoints = maximumTotalJoints_;
            luaBridgeContext_.maximumPendingKeyframes = maximumTotalKeyframes_;
            runtimeInitializer_ = scriptRuntime_->registerRuntimeInitializer(
                scriptRuntime_->self, initializeSkeletalAnimationLuaRuntime, &luaBridgeContext_);
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
        if (scriptRuntime_ != nullptr && runtimeInitializer_ != script_runtime::invalidScriptRuntimeInitializer
            && scriptRuntime_->unregisterRuntimeInitializer != nullptr)
        {
            scriptRuntime_->unregisterRuntimeInitializer(scriptRuntime_->self, runtimeInitializer_);
            runtimeInitializer_ = script_runtime::invalidScriptRuntimeInitializer;
        }
        shutdownSkeletalAnimationLuaBridge(luaBridgeContext_);
        players_.clear();
        clips_.clear();
        skeletons_.clear();
        totalKeyframes_             = 0;
        totalJoints_                = 0;
        totalSamplingCacheEntries_  = 0;
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
            .version              = "0.4.0",
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
            player.trackKeyCursors.assign(trackCount, invalidKeyframeCursor);
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
            || description->durationSeconds <= 0.0f || (description->trackCount != 0 && description->tracks == nullptr))
        {
            setError(error, AnimationError::InvalidDescription);
            return invalidAnimationClip;
        }
        if (clips_.size() >= maximumClips_ || description->trackCount > maximumTracksPerClip_)
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
        return handle;
    }

    void destroyClipImpl(AnimationClipHandle handle)
    {
        const auto clip = clips_.find(handle);
        if (clip == clips_.end())
            return;
        totalKeyframes_ -= clip->second.keyframeCount;
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

    std::uint32_t findLeftKey(const AnimationKeyframe *keys, std::uint32_t keyCount, float sampleTime,
        std::uint32_t *cursor) const noexcept
    {
        std::uint32_t leftIndex = cursor != nullptr ? *cursor : invalidKeyframeCursor;
        if (leftIndex < keyCount - 1u)
        {
            if (sampleTime >= keys[leftIndex].timeSeconds && sampleTime < keys[leftIndex + 1u].timeSeconds)
                return leftIndex;

            std::uint32_t steps = 0;
            if (sampleTime >= keys[leftIndex + 1u].timeSeconds)
            {
                while (leftIndex + 1u < keyCount - 1u && sampleTime >= keys[leftIndex + 1u].timeSeconds
                    && steps < maximumCursorWalk_)
                {
                    ++leftIndex;
                    ++steps;
                }
            }
            else
            {
                while (leftIndex > 0u && sampleTime < keys[leftIndex].timeSeconds && steps < maximumCursorWalk_)
                {
                    --leftIndex;
                    ++steps;
                }
            }
            if (sampleTime >= keys[leftIndex].timeSeconds && sampleTime < keys[leftIndex + 1u].timeSeconds)
            {
                if (cursor != nullptr)
                    *cursor = leftIndex;
                return leftIndex;
            }
        }

        const auto upper = std::upper_bound(keys, keys + keyCount, sampleTime,
            [](float value, const AnimationKeyframe &key) { return value < key.timeSeconds; });
        leftIndex = static_cast<std::uint32_t>((upper - keys) - 1);
        if (cursor != nullptr)
            *cursor = leftIndex;
        return leftIndex;
    }

    bool sampleClipInternal(AnimationClipHandle clipHandle, SkeletonHandle skeletonHandle, float time, bool looping,
        JointDescription *output, std::uint32_t capacity, AnimationError *error,
        std::vector<std::uint32_t> *trackCursors)
    {
        setError(error, AnimationError::None);
        const auto clip     = clips_.find(clipHandle);
        const auto skeleton = skeletons_.find(skeletonHandle);
        if (clip == clips_.end() || skeleton == skeletons_.end())
        {
            setError(error, AnimationError::InvalidHandle);
            return false;
        }

        const ClipRecord &clipRecord = clip->second;
        const auto       &bindPose   = skeleton->second.bindPose;
        const std::uint32_t jointCount = static_cast<std::uint32_t>(bindPose.size());
        if (output == nullptr || capacity < jointCount)
        {
            setError(error, AnimationError::BufferTooSmall);
            return false;
        }
        if (!std::isfinite(time))
        {
            setError(error, AnimationError::InvalidDescription);
            return false;
        }
        // Tracks are sorted by joint index, so compatibility is an O(1) check.
        if (!clipRecord.tracks.empty() && clipRecord.tracks.back().jointIndex >= jointCount)
        {
            setError(error, AnimationError::IncompatibleSkeleton);
            return false;
        }

        std::copy(bindPose.begin(), bindPose.end(), output);
        float sampleTime = time;
        if (looping)
        {
            sampleTime = std::fmod(sampleTime, clipRecord.duration);
            if (sampleTime < 0.0f)
                sampleTime += clipRecord.duration;
        }
        else
            sampleTime = std::clamp(sampleTime, 0.0f, clipRecord.duration);

        const bool useCursors = trackCursors != nullptr && trackCursors->size() == clipRecord.tracks.size();
        for (std::size_t trackIndex = 0; trackIndex < clipRecord.tracks.size(); ++trackIndex)
        {
            const ClipTrack &track = clipRecord.tracks[trackIndex];
            const auto *keys = clipRecord.keyframes.data() + track.firstKeyframe;
            const auto keyCount = track.keyframeCount;
            auto *cursor = useCursors ? &(*trackCursors)[trackIndex] : nullptr;
            Pose &pose = output[track.jointIndex];
            const std::int32_t parentIndex = pose.parentIndex;

            if (keyCount == 1u || sampleTime <= keys[0].timeSeconds)
            {
                copyKeyframe(keys[0], parentIndex, pose);
                if (cursor != nullptr)
                    *cursor = 0u;
                continue;
            }
            if (sampleTime >= keys[keyCount - 1u].timeSeconds)
            {
                copyKeyframe(keys[keyCount - 1u], parentIndex, pose);
                if (cursor != nullptr)
                    *cursor = keyCount - 2u;
                continue;
            }

            const std::uint32_t leftIndex = findLeftKey(keys, keyCount, sampleTime, cursor);
            const AnimationKeyframe &left  = keys[leftIndex];
            const AnimationKeyframe &right = keys[leftIndex + 1u];
            const std::size_t intervalIndex = static_cast<std::size_t>(track.firstKeyframe) + leftIndex + 1u;
            const float inverseInterval = clipRecord.inverseIntervals[intervalIndex];
            const float t = inverseInterval > 0.0f
                ? (sampleTime - left.timeSeconds) * inverseInterval
                : (sampleTime - left.timeSeconds) / (right.timeSeconds - left.timeSeconds);
            const QuaternionInterval *rotationInterval = track.rotationDataOffset != invalidKeyframeCursor
                ? &clipRecord.rotationIntervals[static_cast<std::size_t>(track.rotationDataOffset) + leftIndex + 1u]
                : nullptr;
            interpolateKeyframes(left, right, t, track.interpolation, rotationInterval, parentIndex, pose);
        }
        return true;
    }

    bool sampleClipImpl(AnimationClipHandle clipHandle, SkeletonHandle skeletonHandle, float time, bool looping,
        JointDescription *output, std::uint32_t capacity, AnimationError *error)
    {
        return sampleClipInternal(clipHandle, skeletonHandle, time, looping, output, capacity, error, nullptr);
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
        const double duration = static_cast<double>(clips_.at(record.clip).duration);
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
        configureSamplingCache(record, clips_.at(record.clip).tracks.size());

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
            releaseSamplingCache(record);
            record.clip = clip;
            configureSamplingCache(record, target->second.tracks.size());
            record.finished = false;
        }
        else if (record.trackKeyCursors.size() == target->second.tracks.size())
            std::fill(record.trackKeyCursors.begin(), record.trackKeyCursors.end(), invalidKeyframeCursor);

        if (restart)
        {
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
        auto *cursors = player->second.trackKeyCursors.empty() ? nullptr : &player->second.trackKeyCursors;
        return sampleClipInternal(player->second.clip, skeleton, static_cast<float>(player->second.time),
            player->second.looping, output, capacity, error, cursors);
    }

    bool blendPosesImpl(const JointDescription *const *poses, const float *weights, std::uint32_t poseCount,
        std::uint32_t jointCount, JointDescription *output, std::uint32_t capacity, AnimationError *error)
    {
        return blendPosesInternal(poses, weights, nullptr, poseCount, jointCount, output, capacity, error);
    }

    bool blendPosesMaskedImpl(const JointDescription *const *poses, const float *weights,
        const float *const *jointMasks, std::uint32_t poseCount, std::uint32_t jointCount,
        JointDescription *output, std::uint32_t capacity, AnimationError *error)
    {
        return blendPosesInternal(poses, weights, jointMasks, poseCount, jointCount, output, capacity, error);
    }

    bool blendPosesInternal(const JointDescription *const *poses, const float *weights,
        const float *const *jointMasks, std::uint32_t poseCount, std::uint32_t jointCount,
        JointDescription *output, std::uint32_t capacity, AnimationError *error)
    {
        setError(error, AnimationError::None);
        if (poses == nullptr || weights == nullptr || poseCount == 0 || jointCount == 0
            || jointCount > maximumJoints_ || poseCount > maximumBlendPoses_ || output == nullptr
            || capacity < jointCount)
        {
            setError(error, AnimationError::BufferTooSmall);
            return false;
        }

        // Validate all inputs before modifying output. A mask is per input pose
        // and per joint, so each layer can affect an independent skeleton region.
        for (std::uint32_t pose = 0; pose < poseCount; ++pose)
        {
            if (poses[pose] == nullptr || !std::isfinite(weights[pose]) || weights[pose] < 0.0f
                || (jointMasks != nullptr && jointMasks[pose] == nullptr))
            {
                setError(error, AnimationError::InvalidDescription);
                return false;
            }
            for (std::uint32_t joint = 0; joint < jointCount; ++joint)
            {
                if (!finitePose(poses[pose][joint])
                    || (pose != 0 && poses[pose][joint].parentIndex != poses[0][joint].parentIndex))
                {
                    setError(error, AnimationError::InvalidDescription);
                    return false;
                }
                if (jointMasks != nullptr
                    && (!std::isfinite(jointMasks[pose][joint]) || jointMasks[pose][joint] < 0.0f
                        || jointMasks[pose][joint] > 1.0f))
                {
                    setError(error, AnimationError::InvalidDescription);
                    return false;
                }
            }
        }

        for (std::uint32_t joint = 0; joint < jointCount; ++joint)
        {
            double totalWeight = 0.0;
            std::uint32_t referencePose = 0;
            bool hasReference = false;
            for (std::uint32_t pose = 0; pose < poseCount; ++pose)
            {
                const double mask = jointMasks == nullptr ? 1.0 : jointMasks[pose][joint];
                const double effectiveWeight = static_cast<double>(weights[pose]) * mask;
                totalWeight += effectiveWeight;
                if (!hasReference && effectiveWeight > 0.0)
                {
                    referencePose = pose;
                    hasReference = true;
                }
            }
            if (totalWeight <= 1.0e-8)
            {
                output[joint] = poses[0][joint];
                continue;
            }

            Pose &target = output[joint];
            target.parentIndex = poses[0][joint].parentIndex;
            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                target.translation[axis] = 0.0f;
                target.scale[axis] = 0.0f;
            }
            for (std::uint32_t axis = 0; axis < 4; ++axis)
                target.rotation[axis] = 0.0f;

            float referenceRotation[4] = {
                poses[referencePose][joint].rotation[0], poses[referencePose][joint].rotation[1],
                poses[referencePose][joint].rotation[2], poses[referencePose][joint].rotation[3]};
            normalizeQuaternion(referenceRotation);
            for (std::uint32_t pose = 0; pose < poseCount; ++pose)
            {
                const double mask = jointMasks == nullptr ? 1.0 : jointMasks[pose][joint];
                const float weight = static_cast<float>(static_cast<double>(weights[pose]) * mask / totalWeight);
                if (weight == 0.0f)
                    continue;
                const Pose &source = poses[pose][joint];
                float sourceRotation[4] = {
                    source.rotation[0], source.rotation[1], source.rotation[2], source.rotation[3]};
                normalizeQuaternion(sourceRotation);
                const float sign = quaternionDot(sourceRotation, referenceRotation) < 0.0f ? -1.0f : 1.0f;
                for (std::uint32_t axis = 0; axis < 3; ++axis)
                {
                    target.translation[axis] += source.translation[axis] * weight;
                    target.scale[axis] += source.scale[axis] * weight;
                }
                for (std::uint32_t axis = 0; axis < 4; ++axis)
                    target.rotation[axis] += sourceRotation[axis] * (weight * sign);
            }
            normalizeQuaternion(target.rotation);
        }
        return true;
    }

    bool applyAdditivePoseImpl(const JointDescription *basePose, const JointDescription *additivePose,
        const JointDescription *referencePose, const float *jointWeights, std::uint32_t jointCount, float weight,
        JointDescription *output, std::uint32_t capacity, AnimationError *error)
    {
        setError(error, AnimationError::None);
        if (basePose == nullptr || additivePose == nullptr || referencePose == nullptr || output == nullptr
            || jointCount == 0 || jointCount > maximumJoints_ || capacity < jointCount || !std::isfinite(weight))
        {
            setError(error, AnimationError::InvalidDescription);
            return false;
        }

        for (std::uint32_t joint = 0; joint < jointCount; ++joint)
        {
            if (!finitePose(basePose[joint]) || !finitePose(additivePose[joint]) || !finitePose(referencePose[joint])
                || basePose[joint].parentIndex != additivePose[joint].parentIndex
                || basePose[joint].parentIndex != referencePose[joint].parentIndex
                || (jointWeights != nullptr && (!std::isfinite(jointWeights[joint])
                    || jointWeights[joint] < 0.0f || jointWeights[joint] > 1.0f)))
            {
                setError(error, AnimationError::InvalidDescription);
                return false;
            }
        }

        for (std::uint32_t joint = 0; joint < jointCount; ++joint)
        {
            // Snapshot each joint before writing so output may alias any input pose.
            const Pose base = basePose[joint];
            const Pose additive = additivePose[joint];
            const Pose reference = referencePose[joint];
            const float jointWeight = weight * (jointWeights == nullptr ? 1.0f : jointWeights[joint]);
            output[joint] = base;
            if (jointWeight == 0.0f)
                continue;

            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                output[joint].translation[axis] += (additive.translation[axis]
                    - reference.translation[axis]) * jointWeight;
                output[joint].scale[axis] += (additive.scale[axis]
                    - reference.scale[axis]) * jointWeight;
            }

            float baseRotation[4] = {
                base.rotation[0], base.rotation[1], base.rotation[2], base.rotation[3]};
            float referenceRotation[4] = {
                reference.rotation[0], reference.rotation[1], reference.rotation[2], reference.rotation[3]};
            float additiveRotation[4] = {
                additive.rotation[0], additive.rotation[1], additive.rotation[2], additive.rotation[3]};
            normalizeQuaternion(baseRotation);
            normalizeQuaternion(referenceRotation);
            normalizeQuaternion(additiveRotation);

            const float inverseReference[4] = {
                -referenceRotation[0], -referenceRotation[1], -referenceRotation[2], referenceRotation[3]};
            float rotationDelta[4];
            quaternionMultiply(inverseReference, additiveRotation, rotationDelta);
            normalizeQuaternion(rotationDelta);
            float weightedDelta[4];
            weightedQuaternionDelta(rotationDelta, jointWeight, weightedDelta);
            quaternionMultiply(baseRotation, weightedDelta, output[joint].rotation);
            normalizeQuaternion(output[joint].rotation);
        }
        return true;
    }

    bool buildGlobalTransformsImpl(SkeletonHandle skeletonHandle, const JointDescription *localPose,
        float *outputMatrices, std::uint32_t matrixCapacity, AnimationError *error)
    {
        setError(error, AnimationError::None);
        const auto skeleton = skeletons_.find(skeletonHandle);
        if (skeleton == skeletons_.end())
        {
            setError(error, AnimationError::InvalidHandle);
            return false;
        }
        const std::uint32_t jointCount = static_cast<std::uint32_t>(skeleton->second.bindPose.size());
        if (localPose == nullptr || outputMatrices == nullptr)
        {
            setError(error, AnimationError::InvalidDescription);
            return false;
        }
        if (matrixCapacity < jointCount)
        {
            setError(error, AnimationError::BufferTooSmall);
            return false;
        }

        for (std::uint32_t joint = 0; joint < jointCount; ++joint)
        {
            if (!finitePose(localPose[joint])
                || localPose[joint].parentIndex != skeleton->second.bindPose[joint].parentIndex)
            {
                setError(error, AnimationError::InvalidDescription);
                return false;
            }
        }

        for (const std::uint32_t joint : skeleton->second.topologicalOrder)
        {
            float localMatrix[16];
            makeTransformMatrix(localPose[joint], localMatrix);
            const std::int32_t parent = skeleton->second.bindPose[joint].parentIndex;
            float *destination = outputMatrices + static_cast<std::size_t>(joint) * 16;
            if (parent < 0)
            {
                std::copy(localMatrix, localMatrix + 16, destination);
            }
            else
            {
                float globalMatrix[16];
                multiplyColumnMajorMatrices(outputMatrices + static_cast<std::size_t>(parent) * 16,
                    localMatrix, globalMatrix);
                std::copy(globalMatrix, globalMatrix + 16, destination);
            }
        }
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
    std::uint64_t         maximumTotalSamplingCacheEntries_ = 1'000'000;
    bool                  samplingCacheEnabled_ = true;
    std::uint32_t         maximumCursorWalk_ = 8;
    AnimationInterpolation defaultInterpolation_ = AnimationInterpolation::Slerp;

    std::uint64_t totalKeyframes_            = 0;
    std::uint64_t totalJoints_               = 0;
    std::uint64_t totalSamplingCacheEntries_  = 0;
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
            .moduleVersion = "0.4.0",
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
