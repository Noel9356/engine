// Sampling helpers implementation; see sampler.hpp for the contract.

#include "sampler.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <new>

namespace engine::skeletal_animation
{

std::uint32_t findLeftKey(const AnimationKeyframe *keys, std::uint32_t keyCount, float sampleTime,
    std::uint32_t *cursor, std::uint32_t maximumCursorWalk) noexcept
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
                && steps < maximumCursorWalk)
            {
                ++leftIndex;
                ++steps;
            }
        }
        else
        {
            while (leftIndex > 0u && sampleTime < keys[leftIndex].timeSeconds && steps < maximumCursorWalk)
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

bool sampleClipInternal(const ClipRecord &clipRecord, const SkeletonRecord &skeleton, float time, bool looping,
    JointDescription *output, std::uint32_t capacity, AnimationError *error,
    std::vector<std::uint32_t> *trackCursors, std::uint32_t maximumCursorWalk) noexcept
{
    if (error != nullptr)
        *error = AnimationError::None;

    const auto       &bindPose   = skeleton.bindPose;
    const std::uint32_t jointCount = static_cast<std::uint32_t>(bindPose.size());
    if (output == nullptr || capacity < jointCount)
    {
        if (error != nullptr)
            *error = AnimationError::BufferTooSmall;
        return false;
    }
    if (!std::isfinite(time))
    {
        if (error != nullptr)
            *error = AnimationError::InvalidDescription;
        return false;
    }
    // Tracks are sorted by joint index, so compatibility is an O(1) check.
    if (!clipRecord.tracks.empty() && clipRecord.tracks.back().jointIndex >= jointCount)
    {
        if (error != nullptr)
            *error = AnimationError::IncompatibleSkeleton;
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

        const std::uint32_t leftIndex = findLeftKey(keys, keyCount, sampleTime, cursor, maximumCursorWalk);
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

bool extractRootMotionInternal(const ClipRecord &clip, const SkeletonRecord &skeleton, std::uint32_t rootJoint,
    float startTime, float endTime, bool looping, float outTranslation[3], float outRotation[4],
    AnimationError *error) noexcept
{
    if (error != nullptr)
        *error = AnimationError::None;
    if (outTranslation == nullptr || outRotation == nullptr || !std::isfinite(startTime)
        || !std::isfinite(endTime))
    {
        if (error != nullptr)
            *error = AnimationError::InvalidDescription;
        return false;
    }

    const std::uint32_t jointCount = static_cast<std::uint32_t>(skeleton.bindPose.size());
    if (rootJoint >= jointCount)
    {
        if (error != nullptr)
            *error = AnimationError::InvalidDescription;
        return false;
    }
    if (skeleton.bindPose[rootJoint].parentIndex != -1)
    {
        if (error != nullptr)
            *error = AnimationError::InvalidDescription;
        return false;
    }

    // A scratch buffer bounded by the joint count; declaring it inside the
    // function keeps root-motion extraction free of any allocation in the
    // success path of `sampleClipInternal`. The vector is reused across the
    // two samples below.
    try
    {
        std::vector<Pose> startPose(jointCount);
        std::vector<Pose> endPose(jointCount);
        // `maximumCursorWalk` is unused here because we sample without cursors.
        if (!sampleClipInternal(clip, skeleton, startTime, looping, startPose.data(), jointCount, error, nullptr, 0)
            || !sampleClipInternal(clip, skeleton, endTime, looping, endPose.data(), jointCount, error, nullptr, 0))
            return false;

        const Pose &start = startPose[rootJoint];
        const Pose &end = endPose[rootJoint];
        if (!finitePose(start) || !finitePose(end))
        {
            if (error != nullptr)
                *error = AnimationError::InvalidDescription;
            return false;
        }

        float inverseStart[4] = {-start.rotation[0], -start.rotation[1],
            -start.rotation[2], start.rotation[3]};
        normalizeQuaternion(inverseStart);
        const float delta[3] = {end.translation[0] - start.translation[0],
            end.translation[1] - start.translation[1], end.translation[2] - start.translation[2]};
        float rotated[3];
        // The shared quaternion-vector rotation produces the same result as the
        // explicit formula previously inlined here, for unit quaternions.
        rotateVector(inverseStart, delta, rotated);
        float rotationDelta[4]{};
        multiplyQuaternion(inverseStart, end.rotation, rotationDelta);
        normalizeQuaternion(rotationDelta);
        for (float value : rotated)
        {
            if (!std::isfinite(value))
            {
                if (error != nullptr)
                    *error = AnimationError::InvalidDescription;
                return false;
            }
        }
        for (float value : rotationDelta)
        {
            if (!std::isfinite(value))
            {
                if (error != nullptr)
                    *error = AnimationError::InvalidDescription;
                return false;
            }
        }
        std::copy(rotated, rotated + 3, outTranslation);
        std::copy(rotationDelta, rotationDelta + 4, outRotation);
        return true;
    }
    catch (const std::bad_alloc &)
    {
        if (error != nullptr)
            *error = AnimationError::CapacityExceeded;
        return false;
    }
}

} // namespace engine::skeletal_animation
