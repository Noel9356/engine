# input_actions module

input_actions turns raw device events into virtual game actions. Consumers group actions into prioritized contexts, bind each action to one or more device controls (keys, mouse, gamepad), feed the module raw events, and read per-frame action state after `advanceFrame`. Priority ordering, per-frame control consumption, deadzones, response curves, and exponential smoothing make overlapping context stacks (UI over gameplay, etc.) behave predictably without any per-consumer plumbing.

## Capability

The module provides the `input_actions` capability, ABI constant `inputActionsCapabilityAbi = 2`, declared in `modules/input_actions/include/engine/input_actions/capabilities.hpp`. The module library is `engine_module_input_actions`.

Key capability facts:

- Context and action handles are `std::uint32_t` ids starting at 1; the constants `invalidInputContext` / `invalidInputAction` (`0`) are never handed out.
- Every function takes the `self` pointer first; all caller buffers stay caller-owned.
- Frame flow: `feedEvent` as events arrive, `advanceFrame` once per frame, then `actionState` per action. `pressed`/`released` are single-frame edges.
- `advanceFrame` consumes the accumulated mouse deltas and wheel values, so those controls read as zero until new events arrive.
- Device ids are producer-chosen; state for an id is created on demand by `feedEvent` and dropped by `resetDeviceState`.

## Provided / required / optional capabilities

From `module.toml`:

| Field | Value |
| --- | --- |
| provides | `input_actions` |
| requires | (none) |

From the entry table at the bottom of `source/input_actions.cpp` (`engineQueryModuleEntryTable`, kernel ABI `engineModuleAbiVersion = 8`):

| Field | Value |
| --- | --- |
| moduleName / version | `input_actions` / `0.1.0` |
| providedCapabilities | `input_actions` (1) |
| requiredCapabilities | none (0) |
| optionalCapabilities | none (0) |
| declaredPermissions | none (0) |

## Architecture

| File | Role |
| --- | --- |
| `module.toml` | On-disk descriptor: name, version, provided capability, and the declared `config_keys` documenting the configuration surface. Copied next to the built .so by CMake. |
| `config.toml` | Default values for the `[input_actions]` section (see `configuration.md`). |
| `include/engine/input_actions/capabilities.hpp` | Public capability types: ABI constant, handles, event/control/action enums, description and state structs, and the function-pointer vtable. |
| `source/input_actions.cpp` | The whole implementation: per-device state, contexts/actions/bindings, the per-frame evaluation pipeline, capability binding, module entry table. |
| `docs/` | This documentation. |

## How it works

### Data model

- `DeviceState` (per device id): maps of pressed keys, mouse buttons, and gamepad buttons plus stored gamepad axis values, and two accumulated pairs (mouse delta X/Y, wheel X/Y). The button/axis maps together are capped by `maximum_controls_per_device`.
- `ContextRecord`: name, integer priority, `enabled` flag, `consumeWhenActive` flag, and its actions (`ActionRecord`: description, bindings, last computed `InputActionState`).

### Events to device state

`feedEvent` validates the event (finite values, known kind) and stores it: button kinds store `down` per code, `GamepadAxis` stores the value clamped to [-1, 1], and `MouseDelta`/`MouseWheel` add `value`/`secondaryValue` into the accumulated X/Y slots (clamped to plus/minus 1e6). `FocusLost` wipes all stored state for that device instead of storing anything. Unknown devices are created on demand until `maximum_devices` is reached.

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

- GamepadAxis only: applies a deadzone; `deadzone < 0` selects the module default. Values inside the deadzone become 0 and the rest is rescaled so [deadzone, 1] maps to [0, 1], preserving the sign.
- `invert` negates, `scale` multiplies, and `responseExponent` applies the curve `copysign(|v|^e, v)`.

### Binding rules

`bindAction` validates the control enum, finite scalars, deadzone in [-1, 1), exponent in (0, 8], and the per-type component rules: Button actions accept only digital controls with component 0 (analog axes are rejected), Axis1D requires component 0, and Axis2D accepts components 0 and 1. `createAction` additionally rejects duplicate action names within a context, thresholds outside [0, 1], and smoothing above the configured cap. `setContextEnabled(false)` clears the context's action states, so no stale pressed/released edges are observed while it is off.

## Threading and lifetime

The module has no internal locking. All capability calls must come from one thread (typically the main or input thread the kernel drives `advanceFrame` from). State persists across frames: device snapshots, bindings, and the last computed action states live until `destroyContext`, `removeAction`, `resetDeviceState`, or module `stop()` (which clears contexts and device state; the context order is rebuilt on the next frame). Action handles stay valid until their action or context is destroyed; action names can be fetched via `copyActionName` (buffer includes the terminating NUL).

## Configuration

Defaults and validation ranges are in `config.toml`; every key is documented in [configuration.md](configuration.md). The module works with no configuration at all: the built-in defaults equal the `config.toml` values.

## Integration

- Events: none. The module does not publish to or subscribe on the kernel `EventBus`; input reaches it exclusively through `feedEvent` calls from the window/input producer.
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

// Each frame, after the window module produced events:
engine::input_actions::InputEvent event{};
event.kind   = engine::input_actions::InputEventKind::Key;
event.device = 0;
event.code   = 29;
event.down   = true;
input.feedEvent(input.self, &event);
input.advanceFrame(input.self, frameDeltaSeconds);

engine::input_actions::InputActionState state{};
if (input.actionState(input.self, ctx, action, &state) && state.down)
{
    // use state.value[0], or state.pressed for the edge
}
```
