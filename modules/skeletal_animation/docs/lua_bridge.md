# skeletal_animation lua bridge

The skeletal_animation module installs a `skeletal_animation.*` Lua function namespace into every script runtime the engine hosts. Scripts build skeletons and animation clips through builder objects, create playback players, step them per frame, and sample joint poses or global joint transforms. The bridge is optional: without `script_runtime` the C++ `skeletal_animation` capability works unchanged and scripts never see the `skeletal_animation` table.

## Installation

The module declares `optional_capabilities = ["script_runtime"]`. In `construct()` (source/skeletal_animation.cpp) it acquires the capability when present, fills the bridge context with the module's configured budgets (`maximum_joints`, `maximum_skeletons`, `maximum_clips`, `maximum_tracks_per_clip`, `maximum_keyframes_per_track`, `maximum_total_joints`, `maximum_total_keyframes`), and registers `initializeSkeletalAnimationLuaRuntime` (source/lua_bridge.cpp) through `registerRuntimeInitializer`. script_runtime runs the initializer once per runtime - including runtimes created before registration. `stop()` also calls `shutdownSkeletalAnimationLuaBridge`, which drops all pending builders and resets the budget counters.

## Values and errors

- Handles are numbers; `0` means "none". Builder handles (skeleton/clip under construction) come from a separate counter and are only valid inside the bridge; skeleton/clip/player handles are the capability's handles. Creation functions return `nil` in the handle slot on refusal.
- Numbers must be finite; handle/index arguments must be integral and non-negative (joint `parent` may be `-1` for roots). Quaternions are `x, y, z, w` (4 floats); translations and scales are 3 floats each. A joint pose in a result is therefore 10 numbers in the order translation, rotation, scale.
- Interpolation selectors are strings: `""`/`"default"` (use `skeletal_animation.default_interpolation`), `"slerp"`, `"nlerp"`, `"step"`. Unknown strings reject the call.
- Failure reporting mirrors the capability: creation functions return `(handle_or_nil, error_code)` where `error_code` is the numeric `AnimationError` value - 0 `None`, 1 `InvalidHandle`, 2 `InvalidDescription`, 3 `CapacityExceeded`, 4 `IncompatibleSkeleton`, 5 `BufferTooSmall`. The rest return plain booleans or `nil` for unknown handles.
- A malformed call (wrong argument count, wrong types, non-finite floats) is rejected without touching bridge or module state.

## Builder budgets

Script-side building is bounded by the same budgets the native module enforces, so a runaway script cannot pin memory:

- At most `skeletal_animation.maximum_skeletons` skeleton builders and `maximum_clips` clip builders may exist at once.
- `begin_skeleton(joint_count)` is refused when `joint_count` is 0, exceeds `maximum_joints`, or would push the module-wide pending-joint count past `maximum_total_joints`.
- `begin_track` is refused past `maximum_tracks_per_clip` tracks per clip; `add_keyframe` is refused past `maximum_keyframes_per_track` keyframes in one track or past the module-wide pending-keyframe budget (`maximum_total_keyframes`).
- `cancel_builder` releases a builder and returns its budget; `finish_skeleton`/`finish_clip` atomically move the builder into the module. `stop()` clears everything.

## Function reference

Functions are listed alphabetically. `t(3)`/`r(4)`/`s(3)` mark 3 translation floats, 4 rotation quaternion floats, 3 scale floats in that order.

| Function | Arguments | Returns | Notes |
| --- | --- | --- | --- |
| `skeletal_animation.add_keyframe` | builder, track_index, time_seconds, t(3), r(4), s(3) | ok | Appends to the track inside the clip builder; 13 arguments total. |
| `skeletal_animation.advance_player` | player, delta_seconds | ok | Advances playback; `delta_seconds` is seconds. |
| `skeletal_animation.begin_clip` | name, duration_seconds | builder or nil | Empty names and negative durations are refused. |
| `skeletal_animation.begin_skeleton` | joint_count | builder or nil | Allocates `joint_count` unassigned joints. |
| `skeletal_animation.begin_track` | builder, joint_index, interpolation | track_index or nil | Tracks are appended in call order; `track_index` is what `add_keyframe` expects. |
| `skeletal_animation.cancel_builder` | builder | ok | Works for skeleton and clip builders; releases reserved budget. |
| `skeletal_animation.clip_duration` | clip | seconds | Returns `nil` for an unknown clip. |
| `skeletal_animation.create_player` | clip, speed? (=1.0), initial_time? (=0.0), looping? (=false), playing? (=true) | handle or nil, error | Optional trailing arguments may be omitted. |
| `skeletal_animation.destroy_clip` | clip | (none) | |
| `skeletal_animation.destroy_player` | player | (none) | |
| `skeletal_animation.destroy_skeleton` | skeleton | (none) | |
| `skeletal_animation.evaluate_joint` | player, skeleton, joint_index | t(3), r(4), s(3) | Evaluates the player's current pose for `skeleton` and returns joint `joint_index`; `nil` when the handle is unknown or the joint is out of range. |
| `skeletal_animation.finish_clip` | builder | clip or nil, error | Refused (`InvalidDescription`) when the builder has no tracks or any track has no keyframes. |
| `skeletal_animation.finish_skeleton` | builder | skeleton or nil, error | Refused (`InvalidDescription`) while any joint index is still unassigned. |
| `skeletal_animation.global_transform` | player, skeleton, joint_index | 16 numbers | Evaluates the pose, builds global (parent-composed) transforms, and returns joint `joint_index`'s 4x4 matrix as 16 floats. |
| `skeletal_animation.player_state` | player | time_seconds, normalized_time, playing, finished | Returns `nil` for an unknown player. |
| `skeletal_animation.sample_joint` | clip, skeleton, time_seconds, looping, joint_index | t(3), r(4), s(3) | Stateless clip sampling at an arbitrary time; independent of any player. |
| `skeletal_animation.set_joint` | builder, index, parent, t(3), r(4), s(3) | ok | `parent` is `-1` for roots and must not equal `index`; 13 arguments total. |
| `skeletal_animation.set_player_clip` | player, clip, restart | ok | |
| `skeletal_animation.set_player_playing` | player, playing | ok | |
| `skeletal_animation.set_player_speed` | player, speed | ok | |
| `skeletal_animation.skeleton_joint_count` | skeleton | count | Returns `nil` for an unknown skeleton. |

## Limits

- script_runtime caps apply per runtime: at most `script_runtime.maximum_native_functions` (default 256) registered functions - the bridge registers 22 - at most `maximum_native_arguments` (default 32) per call (the widest calls take 13) and at most `maximum_native_results` (default 16) per call (the widest is `skeletal_animation.global_transform` with 16).
- Bridge builder tables serialize on their own mutex; capability calls serialize on the module's API mutex, so script and native callers never race.
- Sampling and evaluation allocate pose scratch per call; `sampling_cache_enabled` only benefits players driven by the native `evaluatePlayer` path, while `sample_joint` is stateless binary search by design.

## Example

```lua
-- A two-joint skeleton: root plus a child offset along Y.
local skel_builder = skeletal_animation.begin_skeleton(2)
skeletal_animation.set_joint(skel_builder, 0, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1)
skeletal_animation.set_joint(skel_builder, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1)
local skeleton, error_code = skeletal_animation.finish_skeleton(skel_builder)
assert(skeleton ~= nil, error_code)

-- A one-track clip that swings the child joint around X.
local clip_builder = skeletal_animation.begin_clip("swing", 2.0)
local track = skeletal_animation.begin_track(clip_builder, 1, "slerp")
for i = 0, 3 do
    local t = i * 0.5
    local half = math.sin(t * math.pi) * 0.5
    skeletal_animation.add_keyframe(clip_builder, track, t,
        0, 1, 0,                                       -- translation
        half, 0, 0, 1,                                 -- rotation (x y z w)
        1, 1, 1)                                       -- scale
end
local clip, clip_error = skeletal_animation.finish_clip(clip_builder)
assert(clip ~= nil, clip_error)

local player = skeletal_animation.create_player(clip, 1.0, 0.0, true)
assert(player ~= nil)

function frame(delta_seconds)
    skeletal_animation.advance_player(player, delta_seconds)
    local x, y, z, qx, qy, qz, qw = skeletal_animation.evaluate_joint(player, skeleton, 1)
    -- upload the joint pose to the renderer / entity transform here
end
```

See [configuration.md](configuration.md) for the budgets behind the bridge and [overview.md](overview.md) for the native capability.
