// Pose blending, additive composition, and global/skinning transforms.
//
// All operations work on caller-provided pose and matrix buffers so the
// blending helpers stay free of allocation in the hot path. The functions
// take the relevant budget (maximumJoints) as a parameter to avoid coupling
// to the module state.

#ifndef ENGINE_SKELETAL_ANIMATION_BLENDING_HPP
#define ENGINE_SKELETAL_ANIMATION_BLENDING_HPP

#include <engine/skeletal_animation/capabilities.hpp>
#include "math_utils.hpp"
#include "records.hpp"

#include <cstdint>

namespace engine::skeletal_animation
{

// Weighted blend of several poses (translation/scale weighted average,
// hemisphere-corrected quaternion NLERP). `jointMasks` is an optional array
// of poseCount pointers, each holding jointCount values in [0, 1] used as
// per-pose, per-joint weights; pass nullptr for the unmasked blend.
bool blendPosesInternal(const JointDescription *const *poses, const float *weights,
    const float *const *jointMasks, std::uint32_t poseCount, std::uint32_t jointCount,
    JointDescription *output, std::uint32_t capacity, AnimationError *error,
    std::uint32_t maximumJoints, std::uint32_t maximumBlendPoses) noexcept;

// Applies an additive pose relative to `referencePose` onto `basePose`.
// Rotation deltas are applied in local space; `jointWeights` is optional and
// defaults to 1 for every joint. Additive `weight` may be negative or greater
// than 1. Output may alias any input pose; each joint is snapshotted before
// writing the result.
bool applyAdditivePoseInternal(const JointDescription *basePose, const JointDescription *additivePose,
    const JointDescription *referencePose, const float *jointWeights, std::uint32_t jointCount, float weight,
    JointDescription *output, std::uint32_t capacity, AnimationError *error,
    std::uint32_t maximumJoints) noexcept;

// Builds column-major 4x4 global matrices for every joint in `skeleton` by
// composing each joint's local TRS with its parent's global matrix. The
// skeleton's precomputed topological order makes the operation safe even
// when parent joints appear after children in the caller's input array.
bool buildGlobalTransformsInternal(const SkeletonRecord &skeleton, const JointDescription *localPose,
    float *outputMatrices, std::uint32_t matrixCapacity, AnimationError *error) noexcept;

// Builds column-major 4x4 inverse bind pose matrices for every joint in
// `skeleton`. Uses the precomputed cache when present; otherwise computes
// global bind poses and inverts them on demand. Returns false if any
// inversion fails (degenerate bind pose scale).
bool buildInverseBindPoseInternal(const SkeletonRecord &skeleton, float *outputMatrices,
    std::uint32_t matrixCapacity, AnimationError *error) noexcept;

// Computes per-joint skinning matrices: `global(localPose) * inverseBindPose`.
// Falls back to building the inverse bind pose on demand when the skeleton
// has no cached copy; callers that need to compute skinning matrices for many
// frames should pre-build and cache the inverse bind pose once via
// `buildInverseBindPose`.
bool computeSkinningMatricesInternal(const SkeletonRecord &skeleton, const JointDescription *localPose,
    float *outputMatrices, std::uint32_t matrixCapacity, AnimationError *error) noexcept;

} // namespace engine::skeletal_animation

#endif
