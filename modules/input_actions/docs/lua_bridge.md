# input_actions lua bridge

The input_actions module installs an `input_actions.*` Lua function namespace into every script runtime the engine hosts. Scripts create prioritized action contexts, define actions and bind them to device controls, feed raw device events, advance the frame, and read action state. The bridge is optional: without `script_runtime` the C++ `input_actions` capability works unchanged and scripts never see the `input_actions` table.

## Installation

The module declares `optional_capabilities = ["script_runtime"]`. In `construct()` (source/input_actions.cpp) it acquires the capability when present and registers `initializeInputActionsLuaRuntime` (source/lua_bridge.cpp) through `registerRuntimeInitializer`. script_runtime runs the initializer once per runtime - including runtimes created before registration - so every runtime receives the same fourteen `input_actions.*` functions. Unregistering at module shutdown stops future applications.

## Values and errors

- Context and action handles are numbers; `0` (`invalidInputContext` / `invalidInputAction`) means "none" and is never handed out. Creation functions return `nil` instead of a handle when the request is refused (unknown context, capacity caps).
- Numbers must be finite; integer-typed arguments must be integral. `priority` is a signed integer (higher wins); `device`, `code`, and `component` are non-negative integers.
- String selectors are validated exactly:
    - action types: `"button"`, `"axis1d"`, `"axis2d"`
    - combine modes: `"sum"`, `"maximum"`, `"last"`
    - binding controls: `"key"`, `"mouse_button"`, `"gamepad_button"`, `"gamepad_axis"`, `"mouse_delta_x"`, `"mouse_delta_y"`, `"mouse_wheel_x"`, `"mouse_wheel_y"`, `"touch_contact"`, `"touch_position_x"`, `"touch_position_y"`, `"touch_delta_x"`, `"touch_delta_y"`, `"touch_pressure"`
    - event kinds: `"key"`, `"mouse_button"`, `"gamepad_button"`, `"gamepad_axis"`, `"mouse_delta"`, `"mouse_wheel"`, `"focus_lost"`, `"touch_began"`, `"touch_moved"`, `"touch_ended"`, `"touch_cancelled"`
- Booleans must be real booleans. A malformed call (wrong argument count, wrong types, unknown selector) is rejected without touching input state; unlike the creation functions, the mutation and query functions return plain booleans rather than error codes.

For touch bindings, `code` is a contact id or `0xffffffff` for `anyTouchContact` (the contact id `0xffffffff` is reserved); `device` identifies the touch surface. When forwarding window events, device ids use the reserved `touchDeviceIdBase` mapping described in [overview.md](overview.md).

## Function reference

Functions are listed alphabetically.

| Function | Arguments | Returns | Notes |
| --- | --- | --- | --- |
| `input_actions.action_state` | context, action | value_0, value_1, down, pressed, released | The five results of `InputActionState`. Returns `nil` for an unknown context/action pair. `pressed`/`released` are single-frame edges. |
| `input_actions.advance_frame` | delta_seconds | (none) | Runs the per-frame pipeline (priority evaluation, combine, deadzone/curve shaping, smoothing, consumption) and then resets the accumulated mouse deltas and wheel values. |
| `input_actions.bind_action` | context, action, control, device, code, component, scale, deadzone, response_exponent, invert | ok | `deadzone < 0` selects the module default (gamepad axes only); exponent must be in (0, 8]. Button actions accept only digital controls with component 0. |
| `input_actions.clear_bindings` | context, action | ok | Removes every binding of the action. |
| `input_actions.binding_count` | context, action | count | Returns the number of bindings; returns 0 for an invalid action. |
| `input_actions.binding_at` | context, action, binding_index | control, device, code, component, scale, deadzone, response_exponent, invert | Returns one binding by zero-based index, or `nil` when the action/index is invalid. |
| `input_actions.remove_binding` | context, action, binding_index | ok | Removes one zero-based binding index and resets the action state. |
| `input_actions.create_action` | context, name, type, threshold, combine, smoothing_seconds | handle or nil | `type` and `combine` are the selectors above; `threshold` must be in [0, 1] and `smoothing_seconds` at most `input_actions.maximum_smoothing_seconds`. Duplicate names within a context are refused. |
| `input_actions.create_context` | name, priority, enabled, consume_when_active | handle or nil | Higher priority evaluates first; `consume_when_active` hides the context's active controls from lower-priority contexts for that frame. |
| `input_actions.destroy_context` | handle | (none) | Invalidates the context's action handles. |
| `input_actions.feed_event` | kind, device, code, value, secondary_value, down, [pressure] | ok | Injects one raw event. The optional seventh argument is touch pressure (default 1); touch positions/pressure are normalized to [0, 1]. `focus_lost` wipes all stored state for that device. |
| `input_actions.remove_action` | context, action | ok | |
| `input_actions.reset_device` | device | (none) | Drops all stored state for one device id (reconnect/swap). |
| `input_actions.set_context_enabled` | handle, enabled | ok | Disabling clears the context's action states so no stale edges are observed. |

## Limits

- script_runtime caps apply per runtime: at most `script_runtime.maximum_native_functions` (default 256) registered functions - the bridge registers 14 - at most `maximum_native_arguments` (default 32) per call (the widest is `input_actions.bind_action` with 10) and at most `maximum_native_results` (default 16) per call (the widest is `input_actions.action_state` with 5).
- The module serializes every capability entry with one mutex, so script calls interleave safely with native calls (window/input producer, frame loop) and see one coherent state.
- Context, action, binding, device, and per-device control counts are bounded by the `[input_actions]` configuration keys; refused requests return `nil` / `false`.
- Events fed from a script participate in the same per-frame consumption rules as producer events: feed events before `advance_frame`, and read states after it.

## Example

```lua
local gameplay = input_actions.create_context("gameplay", 10, true, true)
if gameplay == nil then return end

local move = input_actions.create_action(gameplay, "move_x", "axis1d", 0.5, "sum", 0.1)
input_actions.bind_action(gameplay, move, "key", 0, 29, 0, 1.0, -1.0, 1.0, false)  -- D
input_actions.bind_action(gameplay, move, "key", 0, 27, 0, -1.0, -1.0, 1.0, false) -- A

function frame(delta_seconds)
    -- the window producer already fed real device events; scripts can inject too
    input_actions.advance_frame(delta_seconds)

    local value, _, down, pressed = input_actions.action_state(gameplay, move)
    if down then
        print(string.format("move_x %.2f (pressed this frame: %s)", value, tostring(pressed)))
    end
end
```

See [configuration.md](configuration.md) for the `[input_actions]` caps and [overview.md](overview.md) for the evaluation pipeline semantics.
