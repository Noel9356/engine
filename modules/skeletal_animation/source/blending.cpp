// Blending implementation; see blending.hpp for the contract.

#include "blending.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace engine::skeletal_animation
{

namespace
{

// Builds the global bind pose matrices of `skeleton` in column-major 4x4 form
// into `globalMatrices`. `globalMatrices` must hold `jointCount * 16` floats.
void buildGlobalBindMatrices(const SkeletonRecord &skeleton, float *globalMatrices) noexcept
{
    for (const std::uint32_t joint : skeleton.topologicalOrder)
    {
        float localMatrix[16];
        makeTransformMatrix(skeleton.bindPose[joint], localMatrix);
        const std::int32_t parent = skeleton.bindPose[joint].parentIndex;
        float *destination = globalMatrices + static_cast<std::size_t>(joint) * 16;
        if (parent < 0)
            std::copy(localMatrix, localMatrix + 16, destination);
        else
        {
            float globalMatrix[16];
            multiplyColumnMajorMatrices(globalMatrices + static_cast<std::size_t>(parent) * 16,
                localMatrix, globalMatrix);
            std::copy(globalMatrix, globalMatrix + 16, destination);
        }
    }
}

} // namespace

bool blendPosesInternal(const JointDescription *const *poses, const float *weights,
    const float *const *jointMasks, std::uint32_t poseCount, std::uint32_t jointCount,
    JointDescription *output, std::uint32_t capacity, AnimationError *error,
    std::uint32_t maximumJoints, std::uint32_t maximumBlendPoses) noexcept
{
    if (error != nullptr)
        *error = AnimationError::None;
    if (poses == nullptr || weights == nullptr || poseCount == 0 || jointCount == 0
        || jointCount > maximumJoints || poseCount > maximumBlendPoses || output == nullptr
        || capacity < jointCount)
    {
        if (error != nullptr)
            *error = AnimationError::BufferTooSmall;
        return false;
    }

    // Validate all inputs before modifying output. A mask is per input pose
    // and per joint, so each layer can affect an independent skeleton region.
    for (std::uint32_t pose = 0; pose < poseCount; ++pose)
    {
        if (poses[pose] == nullptr || !std::isfinite(weights[pose]) || weights[pose] < 0.0f
            || (jointMasks != nullptr && jointMasks[pose] == nullptr))
        {
            if (error != nullptr)
                *error = AnimationError::InvalidDescription;
            return false;
        }
        for (std::uint32_t joint = 0; joint < jointCount; ++joint)
        {
            if (!finitePose(poses[pose][joint])
                || (pose != 0 && poses[pose][joint].parentIndex != poses[0][joint].parentIndex))
            {
                if (error != nullptr)
                    *error = AnimationError::InvalidDescription;
                return false;
            }
            if (jointMasks != nullptr
                && (!std::isfinite(jointMasks[pose][joint]) || jointMasks[pose][joint] < 0.0f
                    || jointMasks[pose][joint] > 1.0f))
            {
                if (error != nullptr)
                    *error = AnimationError::InvalidDescription;
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

bool applyAdditivePoseInternal(const JointDescription *basePose, const JointDescription *additivePose,
    const JointDescription *referencePose, const float *jointWeights, std::uint32_t jointCount, float weight,
    JointDescription *output, std::uint32_t capacity, AnimationError *error, std::uint32_t maximumJoints) noexcept
{
    if (error != nullptr)
        *error = AnimationError::None;
    if (basePose == nullptr || additivePose == nullptr || referencePose == nullptr || output == nullptr
        || jointCount == 0 || jointCount > maximumJoints || capacity < jointCount || !std::isfinite(weight))
    {
        if (error != nullptr)
            *error = AnimationError::InvalidDescription;
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
            if (error != nullptr)
                *error = AnimationError::InvalidDescription;
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
        multiplyQuaternion(inverseReference, additiveRotation, rotationDelta);
        normalizeQuaternion(rotationDelta);
        float weightedDelta[4];
        weightedQuaternionDelta(rotationDelta, jointWeight, weightedDelta);
        multiplyQuaternion(baseRotation, weightedDelta, output[joint].rotation);
        normalizeQuaternion(output[joint].rotation);
    }
    return true;
}

bool buildGlobalTransformsInternal(const SkeletonRecord &skeleton, const JointDescription *localPose,
    float *outputMatrices, std::uint32_t matrixCapacity, AnimationError *error) noexcept
{
    if (error != nullptr)
        *error = AnimationError::None;

    const std::uint32_t jointCount = static_cast<std::uint32_t>(skeleton.bindPose.size());
    if (localPose == nullptr || outputMatrices == nullptr)
    {
        if (error != nullptr)
            *error = AnimationError::InvalidDescription;
        return false;
    }
    if (matrixCapacity < jointCount)
    {
        if (error != nullptr)
            *error = AnimationError::BufferTooSmall;
        return false;
    }

    for (std::uint32_t joint = 0; joint < jointCount; ++joint)
    {
        if (!finitePose(localPose[joint])
            || localPose[joint].parentIndex != skeleton.bindPose[joint].parentIndex)
        {
            if (error != nullptr)
                *error = AnimationError::InvalidDescription;
            return false;
        }
    }

    for (const std::uint32_t joint : skeleton.topologicalOrder)
    {
        float localMatrix[16];
        makeTransformMatrix(localPose[joint], localMatrix);
        const std::int32_t parent = skeleton.bindPose[joint].parentIndex;
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

bool buildInverseBindPoseInternal(const SkeletonRecord &skeleton, float *outputMatrices,
    std::uint32_t matrixCapacity, AnimationError *error) noexcept
{
    if (error != nullptr)
        *error = AnimationError::None;

    const std::uint32_t jointCount = static_cast<std::uint32_t>(skeleton.bindPose.size());
    if (outputMatrices == nullptr)
    {
        if (error != nullptr)
            *error = AnimationError::InvalidDescription;
        return false;
    }
    if (matrixCapacity < jointCount)
    {
        if (error != nullptr)
            *error = AnimationError::BufferTooSmall;
        return false;
    }

    if (!skeleton.inverseBindMatrices.empty())
    {
        std::copy(skeleton.inverseBindMatrices.begin(), skeleton.inverseBindMatrices.end(),
            outputMatrices);
        return true;
    }

    // No cached inverse bind pose: build the global bind pose on a scratch
    // buffer and invert each joint in place. The scratch avoids re-walking
    // the parent chain for every inversion.
    std::vector<float> globalMatrices(static_cast<std::size_t>(jointCount) * 16);
    buildGlobalBindMatrices(skeleton, globalMatrices.data());
    for (std::uint32_t joint = 0; joint < jointCount; ++joint)
    {
        const float *global = globalMatrices.data() + static_cast<std::size_t>(joint) * 16;
        float *destination = outputMatrices + static_cast<std::size_t>(joint) * 16;
        if (!invertAffineMatrix(global, destination))
        {
            if (error != nullptr)
                *error = AnimationError::InvalidDescription;
            return false;
        }
    }
    return true;
}

bool computeSkinningMatricesInternal(const SkeletonRecord &skeleton, const JointDescription *localPose,
    float *outputMatrices, std::uint32_t matrixCapacity, AnimationError *error) noexcept
{
    if (error != nullptr)
        *error = AnimationError::None;

    const std::uint32_t jointCount = static_cast<std::uint32_t>(skeleton.bindPose.size());
    if (localPose == nullptr || outputMatrices == nullptr)
    {
        if (error != nullptr)
            *error = AnimationError::InvalidDescription;
        return false;
    }
    if (matrixCapacity < jointCount)
    {
        if (error != nullptr)
            *error = AnimationError::BufferTooSmall;
        return false;
    }

    // Reuse the caller's output buffer for the global pass, then compose with
    // the inverse bind pose in place. We need a scratch for the inverse bind
    // pose because inverting into the same buffer we read from would corrupt
    // parent data before children read it.
    if (!buildGlobalTransformsInternal(skeleton, localPose, outputMatrices, jointCount, error))
        return false;

    std::vector<float> inverseBind(static_cast<std::size_t>(jointCount) * 16);
    if (!buildInverseBindPoseInternal(skeleton, inverseBind.data(), jointCount, error))
        return false;

    for (std::uint32_t joint = 0; joint < jointCount; ++joint)
    {
        float *global = outputMatrices + static_cast<std::size_t>(joint) * 16;
        const float *inverse = inverseBind.data() + static_cast<std::size_t>(joint) * 16;
        float skinned[16];
        multiplyColumnMajorMatrices(global, inverse, skinned);
        std::copy(skinned, skinned + 16, global);
    }
    return true;
}

} // namespace engine::skeletal_animation
