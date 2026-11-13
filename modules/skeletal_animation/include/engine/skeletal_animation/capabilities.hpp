// capabilities.hpp - public C capability struct for the skeletal_animation
// module.
//
// Plain-data descriptions plus a struct of function pointers; consumers acquire
// it via ParticipantContext::acquireCapability<SkeletalAnimationCapability>().
// Handles are never reused; 0 means "none".

#ifndef ENGINE_SKELETAL_ANIMATION_CAPABILITIES_HPP
#define ENGINE_SKELETAL_ANIMATION_CAPABILITIES_HPP

#include <cstdint>

namespace engine::skeletal_animation
{

// Increment when any public capability structure or function-table layout changes.
// ABI history: 3 added masked/additive blending and global transforms; 4 adds
// root-motion extraction from a clip interval; 5 adds clip event markers and
// bounded per-player event retrieval; 6 adds explicit player time control,
// clip metadata queries, inverse bind pose matrices, and skinning matrix
// palette generation.
inline constexpr std::uint32_t skeletalAnimationCapabilityAbi = 6u;
using SkeletonHandle                                          = std::uint32_t;
using AnimationClipHandle                                     = std::uint32_t;
using AnimationPlayerHandle                                   = std::uint32_t;
// Invalid-handle sentinels; all real handles start at 1.
inline constexpr SkeletonHandle        invalidSkeleton        = 0;
inline constexpr AnimationClipHandle   invalidAnimationClip   = 0;
inline constexpr AnimationPlayerHandle invalidAnimationPlayer = 0;

// One joint of a skeleton (or one sampled pose joint): local transform relative
// to the parent joint plus parent index (-1 for roots).
struct JointDescription
{
    std::int32_t parentIndex    = -1;
    float        translation[3] = {0.0f, 0.0f, 0.0f};
    float        rotation[4]    = {0.0f, 0.0f, 0.0f, 1.0f}; // Quaternion, x y z w.
    float        scale[3]       = {1.0f, 1.0f, 1.0f};
};

// Skeleton creation input: joints in index order. Each parent index must be in
// [-1, jointCount), must not be the joint itself, and the hierarchy must be
// acyclic (validated at creation).
struct SkeletonDescription
{
    const JointDescription *joints     = nullptr;
    std::uint32_t           jointCount = 0;
};

// One keyframe of a track: local joint transform at timeSeconds (quaternion is
// normalized by the module at clip creation).
struct AnimationKeyframe
{
    float timeSeconds    = 0.0f;
    float translation[3] = {0.0f, 0.0f, 0.0f};
    float rotation[4]    = {0.0f, 0.0f, 0.0f, 1.0f};
    float scale[3]       = {1.0f, 1.0f, 1.0f};
};

// Rotation/transform interpolation policy for one animation track.
// Default uses the module-wide default_interpolation setting.
enum class AnimationInterpolation : std::uint32_t
{
    Default = 0,
    Slerp,
    Nlerp,
    Step
};

// Per-joint animation channel: an ordered list of keyframes for one joint.
struct AnimationTrackDescription
{
    std::uint32_t            jointIndex    = 0;
    const AnimationKeyframe *keyframes     = nullptr;
    std::uint32_t            keyframeCount = 0;
    AnimationInterpolation   interpolation = AnimationInterpolation::Default;
};

// A named event marker embedded in a clip timeline. Multiple markers may share
// a timestamp; their declaration order is preserved. userData is opaque to the
// animation module and is copied into emitted player events.
struct AnimationEventMarker
{
    float         timeSeconds = 0.0f;
    std::uint32_t eventId = 0;
    std::uint32_t userData = 0;
};

// Clip creation input: a name, total duration, one track per animated joint
// (at most one track per joint), and optional timeline event markers.
struct AnimationClipDescription
{
    const char                      *name            = nullptr;
    float                            durationSeconds = 0.0f;
    const AnimationTrackDescription *tracks          = nullptr;
    std::uint32_t                    trackCount      = 0;
    const AnimationEventMarker      *eventMarkers    = nullptr;
    std::uint32_t                    eventMarkerCount = 0;
};

// Clip metadata returned by `clipInfo`. The name pointer references storage
// owned by the clip and remains valid until the clip is destroyed.
struct AnimationClipInfo
{
    const char  *name                 = nullptr;
    float       durationSeconds      = 0.0f;
    std::uint32_t trackCount          = 0;
    std::uint64_t keyframeCount      = 0;
    std::uint32_t eventMarkerCount   = 0;
};

// Event delivered when a player crosses a clip marker. direction is +1 for
// forward playback and -1 for reverse playback. clipTimeSeconds is the marker
// timestamp in the source clip, not the player's unwrapped loop time.
struct AnimationPlayerEvent
{
    AnimationClipHandle clip = invalidAnimationClip;
    std::uint32_t eventId = 0;
    std::uint32_t userData = 0;
    float clipTimeSeconds = 0.0f;
    std::int32_t direction = 1;
};

// Player creation input: which clip to run and how.
struct AnimationPlaybackDescription
{
    AnimationClipHandle clip               = invalidAnimationClip;
    float               speed              = 1.0f; // Playback rate multiplier.
    float               initialTimeSeconds = 0.0f; // Wrapped for looping playback; clamped otherwise.
    bool                looping            = true;
    bool                playing            = true;
};

// Snapshot of a player's clock (see playerState).
struct AnimationPlayerState
{
    float timeSeconds    = 0.0f; // Current playback time.
    float normalizedTime = 0.0f; // timeSeconds / clip duration (0 if the clip is gone).
    bool  playing        = false;
    bool  finished       = false;
};

// Error codes shared by all capability entry points (out-parameters; callers
// must check these, boolean returns only report usage failures).
enum class AnimationError : std::uint32_t
{
    None = 0,
    InvalidHandle,        // Unknown skeleton/clip/player handle.
    InvalidDescription,   // Null pointers, non-finite values, bad hierarchy/keyframes.
    CapacityExceeded,     // Per-object or global budget exhausted.
    IncompatibleSkeleton, // Clip tracks reference joints the skeleton lacks.
    BufferTooSmall        // Output pose buffer smaller than the joint count.
};

// The capability struct. `self` is the module instance; every function takes it
// first. Functions returning bool fail on usage errors (bad handle/null out)
// and report real failures through the AnimationError out-parameter.
struct SkeletalAnimationCapability
{
    static constexpr const char *capabilityName = "skeletal_animation";
    std::uint32_t                abi            = skeletalAnimationCapabilityAbi;
    void                        *self           = nullptr;

    SkeletonHandle (*createSkeleton)(void *, const SkeletonDescription *, AnimationError *);
    void (*destroySkeleton)(void *, SkeletonHandle);
    bool (*skeletonInfo)(void *, SkeletonHandle, std::uint32_t *outJointCount);
    AnimationClipHandle (*createClip)(void *, const AnimationClipDescription *, AnimationError *);
    void (*destroyClip)(void *, AnimationClipHandle);
    bool (*clipDuration)(void *, AnimationClipHandle, float *outDurationSeconds);
    bool (*sampleClip)(void *, AnimationClipHandle, SkeletonHandle, float timeSeconds, bool looping,
        JointDescription *outputPose, std::uint32_t capacity, AnimationError *);

    // Extracts the root joint's local-space translation and rotation delta
    // between two clip times. rootJoint must be a top-level joint. The output
    // translation is expressed in the start pose's root frame; the output
    // quaternion is inverse(startRotation) * endRotation (x, y, z, w). Scale
    // is intentionally excluded. When looping is true, each endpoint is
    // wrapped independently; split intervals at loop boundaries to accumulate
    // continuous cycle motion.
    bool (*extractRootMotion)(void *, AnimationClipHandle, SkeletonHandle, std::uint32_t rootJoint,
        float startTimeSeconds, float endTimeSeconds, bool looping, float outTranslation[3],
        float outRotation[4], AnimationError *);
    AnimationPlayerHandle (*createPlayer)(void *, const AnimationPlaybackDescription *, AnimationError *);
    void (*destroyPlayer)(void *, AnimationPlayerHandle);
    bool (*setPlayerClip)(void *, AnimationPlayerHandle, AnimationClipHandle, bool restart);
    bool (*setPlayerSpeed)(void *, AnimationPlayerHandle, float speed);
    bool (*setPlayerPlaying)(void *, AnimationPlayerHandle, bool playing);
    bool (*advancePlayer)(void *, AnimationPlayerHandle, float deltaSeconds);
    bool (*playerState)(void *, AnimationPlayerHandle, AnimationPlayerState *);
    bool (*evaluatePlayer)(
        void *, AnimationPlayerHandle, SkeletonHandle, JointDescription *, std::uint32_t, AnimationError *);
    bool (*blendPoses)(void *, const JointDescription *const *poses, const float *weights, std::uint32_t poseCount,
        std::uint32_t jointCount, JointDescription *outputPose, std::uint32_t capacity, AnimationError *);

    // Blends animation layers with per-pose, per-joint masks. `jointMasks` is
    // an array of poseCount pointers, each containing jointCount values in [0, 1].
    // A null jointMasks pointer applies the unmasked blend used by blendPoses.
    bool (*blendPosesMasked)(void *, const JointDescription *const *poses, const float *weights,
        const float *const *jointMasks, std::uint32_t poseCount, std::uint32_t jointCount,
        JointDescription *outputPose, std::uint32_t capacity, AnimationError *);

    // Applies an additive pose relative to referencePose onto basePose. Rotation
    // deltas are applied in local space; jointWeights is optional and defaults
    // to 1 for every joint. Additive `weight` may be negative or greater than 1.
    bool (*applyAdditivePose)(void *, const JointDescription *basePose, const JointDescription *additivePose,
        const JointDescription *referencePose, const float *jointWeights, std::uint32_t jointCount, float weight,
        JointDescription *outputPose, std::uint32_t capacity, AnimationError *);

    // Converts local joint TRS values to column-major 4x4 global matrices using
    // the hierarchy stored in the skeleton. `matrixCapacity` counts matrices,
    // not scalar floats; output storage must contain matrixCapacity * 16 floats.
    bool (*buildGlobalTransforms)(void *, SkeletonHandle, const JointDescription *localPose,
        float *outputMatrices, std::uint32_t matrixCapacity, AnimationError *);

    // Player events are queued by advancePlayer when playback crosses clip
    // markers. copyPlayerEvents is non-destructive and copies up to capacity
    // events beginning at firstIndex; outCopied reports the number copied.
    // playerEventCount reports pending events and whether any were dropped due
    // to configured queue budgets. clearPlayerEvents removes pending events
    // and resets the overflow indicator.
    bool (*playerEventCount)(void *, AnimationPlayerHandle, std::uint32_t *outCount, bool *outOverflowed);
    bool (*copyPlayerEvents)(void *, AnimationPlayerHandle, std::uint32_t firstIndex,
        AnimationPlayerEvent *output, std::uint32_t capacity, std::uint32_t *outCopied);
    bool (*clearPlayerEvents)(void *, AnimationPlayerHandle);

    // --- ABI 6 additions ---

    // Sets a player's playback clock to an explicit time. Looping players wrap
    // the supplied time into [0, duration); one-shot players clamp to that
    // range. Pending events are cleared and the finished flag is recomputed
    // (one-shot players with time >= duration finish). Negating playback via a
    // negative time only makes sense for looping clips, where it is equivalent
    // to `duration - |time|`.
    bool (*setPlayerTime)(void *, AnimationPlayerHandle, float timeSeconds);

    // Returns clip metadata (name, duration, track count, keyframe count,
    // event marker count). The name pointer references clip-owned storage and
    // is valid until the clip is destroyed.
    bool (*clipInfo)(void *, AnimationClipHandle, AnimationClipInfo *outInfo);

    // Computes per-joint inverse bind pose matrices in column-major 4x4 form.
    // Each matrix is the inverse of the joint's global bind pose (parent chain
    // composed into the joint's local bind transform). `matrixCapacity` counts
    // matrices; output storage must hold matrixCapacity * 16 floats. The
    // module caches inverse bind poses at skeleton creation when configured to
    // do so, otherwise they are computed on demand.
    bool (*buildInverseBindPose)(void *, SkeletonHandle, float *outputMatrices, std::uint32_t matrixCapacity,
        AnimationError *);

    // Computes per-joint skinning matrices: `global(localPose) * inverseBindPose`.
    // Output is column-major 4x4 matrices, one per joint. `matrixCapacity` counts
    // matrices; output storage must hold matrixCapacity * 16 floats. This is the
    // typical vertex-skinning palette consumed by mesh shaders.
    bool (*computeSkinningMatrices)(void *, SkeletonHandle, const JointDescription *localPose,
        float *outputMatrices, std::uint32_t matrixCapacity, AnimationError *);
};

} // namespace engine::skeletal_animation

#endif
