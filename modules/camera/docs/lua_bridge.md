# camera lua bridge

The camera module installs a `camera.*` Lua function namespace into every script runtime the engine hosts. Scripts create and destroy cameras, switch control modes, place the pose and projection, step the controller with input samples, read back the eye position, project world points, and construct world-space viewport rays. The bridge is optional: without `script_runtime` the C++ `camera` capability works unchanged and scripts never see the `camera` table.

## Installation

The module declares `optional_capabilities = ["script_runtime"]`. In `construct()` (source/camera.cpp) it acquires the capability when present and registers `initializeCameraLuaRuntime` (source/lua_bridge.cpp) through `registerRuntimeInitializer`. script_runtime runs the initializer once per runtime - including runtimes created before registration - and each call registers the ten `camera.*` native functions for that runtime. Unregistering at module shutdown stops future applications.

## Values and errors

- Camera handles are numbers; `0` (`invalidCamera`) means "none" and is never handed out. `camera.create` returns `nil` when the module cannot create a camera. Mutators generally return no values; `camera.set_control_settings` returns a boolean, `camera.view_position` returns three coordinates, and the projection helpers return their documented values or nil values on invalid input.
- Numbers must be finite; angles are degrees, positions are world units, and `delta_seconds` is seconds. Integer-typed arguments must be integral and non-negative.
- Mode selectors are strings: control modes are `"fixed"`, `"orbit"`, `"fly"`; projection names are `"perspective"` or `"orthographic"`. Unknown strings reject the call.
- Booleans must be real booleans. A malformed call (wrong argument count, wrong types) returns without touching camera state.
- The native capability exposes a `CameraError` only from `createCamera`; other entries return their documented values or silently ignore invalid input. The bridge preserves that contract, so scripts do not receive per-call error codes.

## Function reference

Functions are listed alphabetically.

| Function | Arguments | Returns | Notes |
| --- | --- | --- | --- |
| `camera.create` | mode? | handle or nil | Creates a camera from the `[camera]` configuration defaults. The optional mode string (`"fixed"`, `"orbit"`, `"fly"`) is applied to the new camera; omit it to keep the configured default mode. |
| `camera.destroy` | handle | (none) | |
| `camera.look_at` | handle, eye_x, eye_y, eye_z, center_x, center_y, center_z, up_x, up_y, up_z | (none) | Places the pose from an eye point, target point, and up vector (3 floats each; 10 arguments total). |
| `camera.set_control_settings` | handle, move_speed, look_speed, mouse_sensitivity, orbit_distance, orbit_zoom_speed, sprint_multiplier, max_delta_seconds, pitch_limit_degrees | ok | Overrides the per-camera control settings; the values are the same fields the `[camera]` keys default. |
| `camera.set_mode` | handle, mode | (none) | `mode` is `"fixed"`, `"orbit"`, or `"fly"`. Entering orbit re-derives the radius from the current pose; leaving orbit keeps the pose. |
| `camera.set_projection` | handle, projection, vertical, near_z, far_z | (none) | `projection` is `"perspective"` (vertical = FOV in degrees) or `"orthographic"` (vertical = frustum half-height). |
| `camera.update` | handle, move_forward, move_right, move_up, look_horizontal, look_vertical, zoom, mouse_delta_x, mouse_delta_y, sprint, delta_seconds | (none) | One input sample; axes are clamped to [-1, 1], `sprint` is a boolean, and `delta_seconds` is clamped to the camera's `max_delta_seconds`. 11 arguments total. |
| `camera.view_position` | handle, width, height | x, y, z | Eye position from the cached `ViewState` for the given viewport size in pixels. |
| `camera.viewport_to_world_ray` | handle, width, height, x, y | origin_x, origin_y, origin_z, direction_x, direction_y, direction_z | Builds a normalized world-space ray from top-left-origin pixel coordinates. Perspective origin is the camera eye; orthographic origin is the near-plane point. Returns six nil values for invalid input/handle. |
| `camera.world_to_viewport` | handle, width, height, world_x, world_y, world_z | x, y, depth | Projects to top-left-origin pixel coordinates and [0, 1] depth. X/Y may be outside the viewport; returns nil values when the point is on/behind the perspective eye plane or the viewport is invalid. |

## Limits

- script_runtime caps apply per runtime: at most `script_runtime.maximum_native_functions` (default 256) registered functions - the bridge registers 10 - at most `maximum_native_arguments` (default 32) per call (the widest is `camera.update` with 11) and at most `maximum_native_results` (default 16) per call (the widest is `camera.viewport_to_world_ray` with 6).
- Bridge calls serialize on the bridge context mutex, so script calls are orderly among themselves; direct C++ capability calls must still follow the module's single-thread rule (see [overview.md](overview.md), "Threading and lifetime"). Do not call the camera capability from the thread that owns the frame loop while a script call is in flight on another thread.
- `camera.maximum_cameras` bounds live cameras regardless of caller; `camera.create` beyond it returns `nil`.

## Example

```lua
local cam = camera.create("orbit")
if cam == nil then return end

camera.set_projection(cam, "perspective", 55.0, 0.1, 200.0)
camera.look_at(cam, 6.0, 4.0, 6.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0)

function frame(delta_seconds)
    -- orbit input: horizontal spin, zoom wheel
    camera.update(cam, 0.0, 0.0, 0.0, 0.4, 0.0, 0.1, 0.0, 0.0, false, delta_seconds)

    local x, y, z = camera.view_position(cam, 1280, 720)
    print(string.format("eye %.1f %.1f %.1f", x, y, z))
end
```

See [configuration.md](configuration.md) for the `[camera]` defaults and [overview.md](overview.md) for the native capability and control-mode semantics.
