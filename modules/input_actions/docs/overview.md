# input_actions module

input_actions turns raw device events into virtual game actions. Consumers group actions into prioritized contexts, bind each action to one or more device controls (keys, mouse, gamepad, touch), feed the module raw events or forward window events, and read per-frame action state after `advanceFrame`. Priority ordering, per-frame control consumption, deadzones, response curves, and exponential smoothing make overlapping context stacks (UI over gameplay, etc.) behave predictably without any per-consumer plumbing.

## Capability

The module provides the `input_actions` capability, ABI constant `inputActionsCapabilityAbi = 4`, declared in `modules/input_actions/include/engine/input_actions/capabilities.hpp`. The module library is `engine_module_input_actions`.

Key capability facts:

- Context and action handles are `std::uint32_t` ids starting at 1; the constants `invalidInputContext` / `invalidInputAction` (`0`) are never handed out.
- Every function takes the `self` pointer first; all caller buffers stay caller-owned.
- Frame flow: `feedEvent` or `feedWindowEvent` as events arrive, `advanceFrame` once per frame, then `actionState` per action. `pressed`/`released` are single-frame edges.
- `advanceFrame` consumes the accumulated mouse deltas and wheel values, so those controls read as zero until new events arrive. Mouse absolute position is retained across frames (it stores the most recently reported value).
- `feedWindowEvent` maps keyboard/mouse to `keyboardMouseDevice`, gamepads to `gamepadDeviceIdBase | GLFW joystick id`, and touch surfaces to `touchDeviceIdBase | native device id`; touch device ids above `0x7fffffff` are rejected to preserve the reserved device-id namespace. Raw `feedEvent` callers choose device ids themselves. State for an id is created on demand and dropped by `resetDeviceState`.
- Control and event enums are open-ended: existing values keep their numeric assignments while new kinds (such as `MousePosition` events and `MousePositionX`/`MousePositionY` controls) are appended without changing the ABI constant.

### Runtime rebinding

The binding inspection functions support control-settings screens and user remapping without exposing module-owned storage:

```cpp
const std::uint32_t count = input.bindingCount(input.self, context, action);
std::vector<engine::input_actions::InputBinding> bindings(count);
std::uint32_t required = 0;
if (input.copyBindings(input.self, context, action, bindings.data(), count, &required))
{
    // Display the bindings; indexes remain stable until the binding list changes.
}
input.removeBinding(input.self, context, action, 0); // removes the first binding
```

`bindingCount` returns zero for an invalid action. `copyBindings` returns false for an invalid action or insufficient capacity, reports the required count when the action is valid, and never writes a partial list. `removeBinding` preserves the relative order of remaining bindings and clears the last computed action state.

## Provided / required / optional capabilities

From `module.toml`:

| Field | Value |
| --- | --- |
| provides | `input_actions` |
| requires | (none) |
| optional_capabilities | `script_runtime` |

From the entry table at the bottom of `source/input_actions.cpp` (`engineQueryModuleEntryTable`, kernel ABI `engineModuleAbiVersion = 8`):

| Field | Value |
| --- | --- |
| moduleName / version | `input_actions` / `0.1.0` |
| providedCapabilities | `input_actions` (1) |
| requiredCapabilities | none (0) |
| optionalCapabilities | `script_runtime` (1) |
| declaredPermissions | none (0) |

## Architecture

| File | Role |
| --- | --- |
| `module.toml` | On-disk descriptor: name, version, provided capability, and the declared `config_keys` documenting the configuration surface. Copied next to the built .so by CMake. |
| `config.toml` | Default values for the `[input_actions]` section (see `configuration.md`). |
| `include/engine/input_actions/capabilities.hpp` | Public capability types: ABI constant, handles, event/control/action enums, description and state structs, and the function-pointer vtable. |
| `source/input_actions.cpp` | The module entry point: `InputActionsModule` wires the cooperating pieces together, serializes capability calls with one mutex, binds the C capability vtable, and exports the module entry table. |
| `source/internal.hpp` | Shared private declarations: `TouchState`, `DeviceState`, `ControlKey`, `ControlKeyHash`, and the analog/touch control classifiers used by the registry and the evaluator. |
| `source/configuration.hpp` | Header-only `InputActionsConfiguration` aggregate plus the loader that reads `[input_actions]` keys from `ParticipantContext` with range validation. |
| `source/device_state.hpp` / `source/device_state.cpp` | `DeviceStateRegistry`: per-device control snapshots, raw event ingestion, touch-contact lookup, per-frame delta reset. |
| `source/context_registry.hpp` / `source/context_registry.cpp` | `ContextRegistry`: contexts, actions, bindings, handle allocation, ordered iteration, name-uniqueness tracking. |
| `source/window_adapter.hpp` / `source/window_adapter.cpp` | `feedWindowEvent` translator from `window::WindowEvent` to `InputEvent`(s). |
| `source/frame_evaluator.hpp` / `source/frame_evaluator.cpp` | `FrameEvaluator`: the per-frame pipeline (binding reads, combine modes, shaping, smoothing, threshold, consumption). |
| `source/lua_bridge.cpp`, `source/lua_bridge.hpp` | Optional `script_runtime` bridge for context/action creation, binding, event submission, frame advancement, and action-state queries. |
| `docs/` | This documentation. |

## How it works

### Data model

- `DeviceState` (per device id, owned by `DeviceStateRegistry`): maps of pressed keys, mouse buttons, and gamepad buttons plus stored gamepad axis values, multitouch contact snapshots, accumulated pairs (mouse delta X/Y, wheel X/Y), and the most recently reported absolute cursor position. A cached `controlCount` sums the sizes of every control map and the touch map so the per-device control cap is checked without re-summing five containers per event.
- `ContextRecord` (owned by `ContextRegistry`): name, integer priority, `enabled` flag, `consumeWhenActive` flag, its actions (`ActionRecord`: description, bindings, last computed `InputActionState`), and a parallel name set so duplicate-name detection is constant time.

### Events to device state

`feedEvent` validates the event (finite values, known kind) and stores it: button kinds store `down` per code, `GamepadAxis` stores stick axes in [-1, 1] and trigger axes in [0, 1], `MouseDelta`/`MouseWheel` accumulate X/Y, `MousePosition` records the latest absolute cursor X/Y, and touch events maintain each contact's normalized position, per-frame delta, pressure, and active state. A begin/end pair received within one frame still produces a one-frame active contact, so a tap is not lost. `FocusLost` wipes all stored state for that device. `feedWindowEvent` translates the window module's key, mouse, gamepad, touch, and focus events to these raw events; a `CursorMoved` window event emits both `MouseDelta` (for accumulated relative motion) and `MousePosition` (for absolute cursor coordinates) so consumers can bind either. Unrelated window events are ignored. Unknown devices are created on demand until `maximum_devices` is reached.

### The per-frame pipeline

`advanceFrame(deltaSeconds)` runs in one pass:

1. The delta is clamped to `maximum_frame_delta_seconds` (non-finite or non-positive deltas skip time-based effects such as smoothing).
2. The context list is rebuilt lazily (only after contexts were created or destroyed) sorted by descending priority, then ascending handle.
3. For each context, for each action: bindings are evaluated in insertion order, skipping controls that an earlier context consumed this frame. Each binding value passes through `readBinding` (below) and is merged into the target component with the action's combine mode (`Sum`, `MaximumMagnitude`, or `LastNonZero`).
4. Components are clamped to [-1, 1]; axis actions optionally smooth toward the raw value with `alpha = 1 - exp(-delta / smoothingSeconds)`.
5. `down` is the threshold test: |value[0]| for Button/Axis1D, the Euclidean magnitude for Axis2D (which can reach about 1.41 on the diagonal). `pressed`/`released` are the edges against the previous frame.
6. A context whose action went down collects its active controls; when `consumeWhenActive` is set those controls are marked consumed, so every lower-priority context ignores them for this frame.
7. Finally the accumulated mouse deltas and wheel values reset to zero, so they represent per-frame relative motion.

### Binding value shaping

`readBinding` looks up the control (unknown devices read 0), then:

- GamepadAxis only: applies a deadzone; `deadzone < 0` selects the module default. Stick axes are signed [-1, 1], while trigger axes are unipolar [0, 1]. Values inside the deadzone become 0 and the rest is rescaled so [deadzone, 1] maps to [0, 1], preserving the sign for stick axes.
- `invert` negates, `scale` multiplies, and `responseExponent` applies the curve `copysign(|v|^e, v)`.

### Binding rules

`bindAction` validates the control enum, finite scalars, deadzone in [-1, 1), exponent in (0, 8], and the per-type component rules: Button actions accept only digital controls with component 0 (`TouchContact` is digital; touch position/delta/pressure, gamepad axes, and mouse position are analog), Axis1D requires component 0, and Axis2D accepts components 0 and 1. `createAction` additionally rejects duplicate action names within a context, thresholds outside [0, 1], and smoothing above the configured cap. `setContextEnabled(false)` clears the context's action states, so no stale pressed/released edges are observed while it is off.

### Touch bindings

Touch controls use the binding `device` to select a touch surface and `code` to select a contact id. Set `code = anyTouchContact` (`0xffffffff`; this contact id is reserved) to select the most recently updated active contact, or the latest contact when none remain active. `TouchContact` reads as down while the contact is active, including a begin/end tap wholly contained in one input frame. Position and pressure are normalized to [0, 1]; touch deltas are normalized coordinate differences accumulated until `advanceFrame` completes. `TouchContact` can drive Button actions; the position, delta, and pressure controls drive Axis1D/Axis2D actions.

### Mouse position bindings

`MousePositionX` and `MousePositionY` read the most recently reported absolute cursor position for the binding's `device` (typically `keyboardMouseDevice`). When the window adapter produces the event, coordinates are normalized to the drawable window area [0, 1]; raw `feedEvent` callers may use any unit they prefer. The position is retained across frames, so an Axis2D action bound to `(MousePositionX, MousePositionY)` exposes the cursor location even when no motion occurred this frame. These controls are analog and therefore cannot drive Button actions.

### Window event adapter

After popping each `window::WindowEvent`, call `feedWindowEvent(self, windowHandle, &event)`. Feed events from one window queue only once per event; gamepad events are broadcast to all live window queues. It forwards key and mouse state, cursor deltas and absolute cursor position, wheel offsets, standardized gamepad controls, focus loss, and forwarded touch contacts. A `CursorMoved` window event emits two raw events (`MouseDelta` for accumulated relative motion and `MousePosition` for the absolute coordinate) so a binding can read either. Gamepad ids use `gamepadDeviceIdBase | joystickId`; touch ids use `touchDeviceIdBase | touchDevice`; touch device ids above `0x7fffffff` are rejected rather than aliased. The reserved high-bit ranges are reserved for the adapter; use other device ids for raw `feedEvent` producers. The adapter does not poll a window or native input source itself; call the window module's `pollEvents` first. See the window module overview for touch host integration requirements.

## Lua bridge

When `script_runtime` is present, Lua runtimes receive `input_actions.*` functions for context/action management, binding, raw event submission, frame advancement, and state queries. String selectors are `"button"`, `"axis1d"`, `"axis2d"`; combine modes are `"sum"`, `"maximum"`, `"last"`; controls and event kinds use the names accepted by the bridge. The capability serializes calls internally, so script and native callers share one coherent state. The bridge is omitted when `script_runtime` is unavailable. The complete function reference lives in [lua_bridge.md](lua_bridge.md).

## Threading and lifetime

The module serializes every capability entry point with one mutex. This permits calls from the input thread and Lua callbacks without concurrent mutation of device maps or action contexts; event producers should still batch events before `advanceFrame` to keep frame ordering deterministic. State persists across frames: device snapshots, bindings, and the last computed action states live until `destroyContext`, `removeAction`, `resetDeviceState`, or module `stop()` (which clears contexts and device state; the context order is rebuilt on the next frame). Action handles stay valid until their action or context is destroyed; action names can be fetched via `copyActionName` (buffer includes the terminating NUL). Bindings can be enumerated with `bindingCount` and `copyBindings`, and an individual binding can be removed by its current zero-based index with `removeBinding`. Binding copies are all-or-nothing: `outRequired` receives the number of bindings, and no partial copy occurs when capacity is insufficient. Removing a binding resets the action state until the next frame.

## Configuration

Defaults and validation ranges are in `config.toml`; every key is documented in [configuration.md](configuration.md). The module works with no configuration at all: the built-in defaults equal the `config.toml` values.

## Integration

- Events: none. The module does not publish to or subscribe on the kernel `EventBus`; input reaches it through `feedEvent` calls or `feedWindowEvent` forwarding from the window/input producer.
- Permissions: none declared (`declaredPermissions` is empty in the entry table and `module.toml` declares no permissions).

Consumer usage (all signatures taken from `capabilities.hpp`):

```cpp
#include <engine/input_actions/capabilities.hpp>

// During a participant's construct phase:
auto& input = context.acquireCapability<engine::input_actions::InputActionsCapability>();

engine::input_actions::InputContextDescription contextDesc{};
contextDesc.name              = "gameplay";
contextDesc.priority          = 10;
contextDesc.consumeWhenActive = true;
const auto ctx = input.createContext(input.self, &contextDesc);

engine::input_actions::InputActionDescription actionDesc{};
actionDesc.name             = "move_right";
actionDesc.type             = engine::input_actions::InputActionType::Axis1D;
actionDesc.smoothingSeconds = 0.1f;
const auto action = input.createAction(input.self, ctx, &actionDesc);

engine::input_actions::InputBinding binding{};
binding.control = engine::input_actions::InputControlKind::Key;
binding.device  = 0;
binding.code    = 29; // producer-specific key code
binding.scale   = 1.0f;
input.bindAction(input.self, ctx, action, &binding);

// Each frame, after the window module produced events. windowHandle is the
// handle returned by window.createWindow(...) during application setup.
auto& window = context.acquireCapability<engine::window::WindowCapability>();
engine::window::WindowEvent event{};
while (window.popEvent(window.self, windowHandle, &event))
    input.feedWindowEvent(input.self, windowHandle, &event);
input.advanceFrame(input.self, frameDeltaSeconds);

engine::input_actions::InputActionState state{};
if (input.actionState(input.self, ctx, action, &state) && state.down)
{
    // use state.value[0], or state.pressed for the edge
}
```
