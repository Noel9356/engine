# skeletal_animation module

skeletal_animation provides joint hierarchies, immutable keyframed clips, playback players, and weighted pose blending. It is a CPU-side animation service: it owns no rendering state, and consumers sample or blend local-space poses before skinning their meshes.

## Capability

The module provides the `skeletal_animation` capability (ABI 3, see `include/engine/skeletal_animation/capabilities.hpp`). All calls take the capability pointer's `self` first; handles are `std::uint32_t`, start at 1, are never reused, and `0` (`invalidSkeleton` / `invalidAnimationClip` / `invalidAnimationPlayer`) means "none".

Capability ABI 3 adds masked pose blending, additive pose application, and global transform generation. Consumers must rebuild against the updated header; binaries built against earlier capability layouts must not use this capability.

| Function | Purpose |
| --- | --- |
| `createSkeleton(description, outError)` | Stores a bind pose (validating the joint hierarchy) and returns a handle. |
| `destroySkeleton(handle)` | Releases the skeleton and its joints budget. |
| `skeletonInfo(handle, outJointCount)` | Joint count of a skeleton. |
| `createClip(description, outError)` | Copies and validates keyframe tracks; returns a clip handle. |
| `destroyClip(handle)` | Releases the clip; players using it are parked. |
| `clipDuration(handle, outDurationSeconds)` | Duration of a clip. |
| `sampleClip(clip, skeleton, time, looping, output, capacity, outError)` | Samples a clip onto a full pose buffer (bind pose + tracked joints). |
| `createPlayer(description, outError)` / `destroyPlayer(handle)` | Playback state machines over one clip. |
| `setPlayerClip(handle, clip, restart)` / `setPlayerSpeed(handle, speed)` / `setPlayerPlaying(handle, playing)` | Player controls. |
| `advancePlayer(handle, deltaSeconds)` | Advances the player clock (`deltaSeconds * speed`). |
| `playerState(handle, outState)` | Time, normalized time, playing, finished. |
| `evaluatePlayer(player, skeleton, output, capacity, outError)` | Samples the player's clip at the player's clock. |
| `blendPoses(poses, weights, poseCount, jointCount, output, capacity, outError)` | Weighted blend of several poses (translation/scale weighted average, hemisphere-corrected quaternion NLERP). |

**Error convention:** functions returning `bool` only report usage problems (unknown handle, null output, buffer too small); everything else is reported by setting the `AnimationError` out-parameter. Callers that need to distinguish failure modes must check the error out-parameter.

## Provided / required / optional capabilities

From `module.toml` and the entry table at the bottom of `source/skeletal_animation.cpp` (both agree):

- **Provided:** `skeletal_animation`.
- **Required:** none.
- **Optional:** `script_runtime` (automatic Lua bridge registration).
- **Declared permissions:** none.

The entry table exports `engineQueryModuleEntryTable()` with `abiVersion = engineModuleAbiVersion` (kernel module ABI 8), module name `skeletal_animation`, version `0.4.0`, and a `createModule` factory for `SkeletalAnimationModule`. The capability ABI is independently versioned as 3 for the masked-blend, additive-pose, and global-transform operations.

## Architecture

| File | Purpose |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capability, declared `config_keys` (read by kernel discovery without loading the library). |
| `config.toml` | Default values for the `[skeletal_animation]` configuration section (see `configuration.md`). |
| `CMakeLists.txt` | Builds `engine_module_skeletal_animation` (a `.so` under `runtime/modules/skeletal_animation/`) and copies `module.toml` next to it. |
| `include/engine/skeletal_animation/capabilities.hpp` | Public capability types: ABI constant, handles, descriptions, player state, error codes, capability struct. |
| `source/skeletal_animation.cpp` | Pose math, packed clip storage, bounded player sampling cursors, module class, capability binding, and entry table. |
| `source/lua_bridge.cpp` / `source/lua_bridge.hpp` | Automatic Lua namespace registration and bounded skeleton/clip builders. |

## How it works

**Skeletons.** `createSkeleton` copies the joint list as the bind pose. Each joint must have a parent index in `[-1, jointCount)` other than itself, finite transform values, and a non-zero rotation quaternion (which is normalized on entry). The hierarchy is verified acyclic per joint with a two-phase walk (mark the path, then confirm); any cycle rejects the whole description. Global and per-skeleton joint budgets are enforced up front.

**Clips.** A clip owns at most one track per joint. At creation every keyframe is validated: strictly increasing times within `[0, duration]`, finite translation/scale, scale magnitude ≥ 1e-8, finite non-zero rotation (normalized on entry). Tracks are sorted by joint index and their keyframes are packed into one contiguous keyframe array per clip; reciprocal time intervals are stored in a parallel array. Slerp tracks also store precomputed shortest-path quaternion interval coefficients, avoiding repeated `acos` and `sin(angle)` work during sampling. This avoids one keyframe allocation per track and makes sampling write joints in ascending order. Duplicate tracks are detected after sorting lightweight track references, without a bitmap proportional to the joint limit. Clip and keyframe budgets (per-track and global) are checked before runtime publication.

**Players.** A player is a small clock over one clip: `speed` multiplier (negative allowed), `looping`, `playing`, `finished`. The internal playback clock accumulates `deltaSeconds * speed` in double precision; the public state and sampling interfaces expose float times. Initial looping offsets are wrapped (including negative offsets), while one-shot offsets are clamped. `advancePlayer` wraps looping players with `fmod` (negative playback wraps forward), and clamps one-shot players to `[0, duration]`, flagging `finished` and stopping at either end. A player whose clip is destroyed is parked: `clip` reset, `playing` false; `evaluatePlayer` then reports `InvalidHandle`.

**Sampling** (`sampleClip` / `evaluatePlayer`). The skeleton's bind pose is copied to the output buffer first, so joints without tracks keep their rest transform. `sampleTime` is wrapped (looping) or clamped. For stateless `sampleClip`, a binary search (`upper_bound`) finds the surrounding keyframes. `evaluatePlayer` can use a per-player cursor for each track: it reuses the previous interval, walks a short distance for nearby time changes, and falls back to binary search for larger seeks or reversals. The cursor cache is optional and globally budgeted. Before the first or after the last keyframe the boundary keyframe is used verbatim. Each track selects `Slerp`, `Nlerp`, or `Step`; `Default` resolves to the module's `default_interpolation` setting. Slerp uses clip-time precomputed quaternion coefficients; Nlerp avoids the trigonometric path. Translation and scale use linear interpolation for both rotational interpolation modes. Clips are skeleton-independent at creation time; if a track's joint index is outside the target skeleton, sampling fails with `IncompatibleSkeleton`.

**Blending** (`blendPoses`). All poses must share the same parent layout and be finite; weights must be finite and non-negative. Input poses are validated before output is written. The blend then traverses input poses in pose-major order so each input pose is read contiguously. Translation and scale use normalized weights; quaternion contributions are hemisphere-aligned against pose 0 before accumulation and the result is normalized (weighted NLERP). A zero total weight copies pose 0. `poseCount` is capped by `maximum_blend_poses`.

**Layered blending.** `blendPosesMasked` applies a separate [0, 1] joint mask to each input pose before normalizing that joint's effective weights. This supports upper/lower-body layers and arbitrary per-joint influence masks without imposing a fixed rig layout. `applyAdditivePose` applies translation/scale deltas and a local-space quaternion delta relative to a supplied reference pose; negative and greater-than-one layer weights are supported, and an optional per-joint mask controls influence.

**Hierarchy evaluation.** `buildGlobalTransforms` converts a local pose into column-major 4x4 global matrices. Skeleton creation computes a topological order, so parent joints may appear after children in the caller's input array. The operation uses caller-owned output storage and no per-call allocation. Matrix layout and coordinate convention remain the consumer's responsibility.

## Threading and lifetime

The module has no locks. Object creation/destruction, `stop()`, and player-control calls must be serialized. Read-only `sampleClip` calls may run concurrently while no object-lifecycle operation is occurring. `evaluatePlayer` updates its player-owned cursor cache, so calls for distinct players may run concurrently, but calls that evaluate or control the same player must be serialized. Records are stored directly in `unordered_map` nodes (without a second per-record heap allocation) and die with the module: `stop()` clears players, clips, and skeletons and resets the budget counters. Handles are monotonic counters and are never reused, so a stale handle is rejected rather than aliasing a newer object. Destroying a clip does not destroy players that reference it — they are parked until given a new clip.

## Lua bridge

When `script_runtime` is available, the module installs a `skeletal_animation` Lua table into each newly created Lua runtime before script execution. Native participants can still use the full C++ capability directly. Lua functions use numeric handles and primitive arguments; errors are numeric `AnimationError` values.

The bridge supports staged skeleton and clip construction (`begin_skeleton`, `set_joint`, `finish_skeleton`; `begin_clip`, `begin_track`, `add_keyframe`, `finish_clip`), cancellation of unfinished builders, object destruction and queries, player creation/control/state, per-joint clip sampling and player evaluation, and per-joint global transform retrieval. Skeleton/clip builder limits reuse the module's configured joint, skeleton, clip, track, and keyframe limits. Pose-wide blending remains available through the native capability because the current Lua native-call ABI does not marshal nested arrays.

```lua
local builder = assert(skeletal_animation.begin_skeleton(1))
assert(skeletal_animation.set_joint(builder, 0, -1,
    0, 0, 0, 0, 0, 0, 1, 1, 1, 1))
local skeleton, skeleton_error = skeletal_animation.finish_skeleton(builder)
assert(skeleton ~= nil, skeleton_error)
```

`script_runtime` is an optional capability. Without it, the module remains fully usable through its C++ API. The complete function reference lives in [lua_bridge.md](lua_bridge.md).

## Configuration

See `configuration.md` for the `[skeletal_animation]` keys. In addition to object/keyframe budgets, projects can disable player sampling caches, cap their total memory, and choose the default rotation interpolation. Per-track interpolation can override the default.

## Integration

The module publishes no events, subscribes to no events, and declares no permissions. Consumers acquire the capability and drive it directly:

```cpp
#include <engine/skeletal_animation/capabilities.hpp>

namespace anim = engine::skeletal_animation;

auto &animation = context.acquireCapability<anim::SkeletalAnimationCapability>();

// Build a two-joint skeleton.
std::vector<anim::JointDescription> joints = {
    {.parentIndex = -1, .translation = {0, 0, 0}, .rotation = {0, 0, 0, 1}, .scale = {1, 1, 1}},
    {.parentIndex = 0,  .translation = {0, 1, 0}, .rotation = {0, 0, 0, 1}, .scale = {1, 1, 1}},
};
anim::SkeletonDescription skeletonDescription{joints.data(), 2};
anim::AnimationError error = anim::AnimationError::None;
const anim::SkeletonHandle skeleton
    = animation.createSkeleton(animation.self, &skeletonDescription, &error);

// A 2-second clip with one track for joint 1.
std::vector<anim::AnimationKeyframe> keys = {
    {.timeSeconds = 0.0f, .translation = {0, 1, 0}, .rotation = {0, 0, 0, 1}, .scale = {1, 1, 1}},
    {.timeSeconds = 2.0f, .translation = {0, 1, 0}, .rotation = {0, 0, 0.7071f, 0.7071f}, .scale = {1, 1, 1}},
};
anim::AnimationTrackDescription track{1, keys.data(), 2, anim::AnimationInterpolation::Nlerp};
anim::AnimationClipDescription clipDescription{"wave", 2.0f, &track, 1};
const anim::AnimationClipHandle clip
    = animation.createClip(animation.self, &clipDescription, &error);

anim::AnimationPlaybackDescription playback{clip, 1.0f, 0.0f, true, true};
const anim::AnimationPlayerHandle player
    = animation.createPlayer(animation.self, &playback, &error);

// Each frame:
animation.advancePlayer(animation.self, player, deltaSeconds);
std::vector<anim::JointDescription> pose(joints.size());
animation.evaluatePlayer(animation.self, player, skeleton, pose.data(),
    static_cast<std::uint32_t>(pose.size()), &error);

// Cross-fade between two clips:
anim::JointDescription blended[2];
const anim::JointDescription *poses[2] = {poseA, poseB};
const float weights[2] = {0.3f, 0.7f};
animation.blendPoses(animation.self, poses, weights, 2, 2, blended, 2, &error);

animation.destroyPlayer(animation.self, player);
animation.destroyClip(animation.self, clip);
animation.destroySkeleton(animation.self, skeleton);
```
