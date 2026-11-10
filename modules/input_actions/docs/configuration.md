# input_actions configuration

All keys live in the `[input_actions]` section. The module's `config.toml` supplies the defaults below; the application config or a runtime profile may override them. `module.toml` mirrors every key under `config_keys` for documentation; the kernel validates known keys for engine sections and game-mod manifests only, so unknown `[input_actions]` keys are silently accepted.

Every value is read once in `construct()`; the module does not follow configuration reloads, so restarts are required for changes. Keys are listed alphabetically.

| Key | Type | Valid range (as validated in `construct()`) | Default (`config.toml`) | Effect |
| --- | --- | --- | --- | --- |
| `input_actions.default_deadzone` | float | finite, 0.0 - 1.0 (exclusive upper) | `0.15` | Radial deadzone for `GamepadAxis` bindings whose `deadzone` is negative; inside the deadzone values read 0 and the rest is rescaled to [0, 1] preserving the sign. |
| `input_actions.maximum_actions_per_context` | integer | 1 - 65536 | `1024` | Cap on actions per context; `createAction` rejects beyond it. Used to size the per-frame consumption scratch containers. |
| `input_actions.maximum_bindings_per_action` | integer | 1 - 4096 | `64` | Cap on bindings per action; `bindAction` rejects beyond it. |
| `input_actions.maximum_contexts` | integer | 1 - 4096 | `64` | Cap on live input contexts; `createContext` returns `invalidInputContext` beyond it. Also used to pre-reserve the frame context list. |
| `input_actions.maximum_controls_per_device` | integer | 1 - 65536 | `4096` | Cap on stored controls (keys + mouse buttons + gamepad buttons + axes + touch contacts combined) per device; new controls beyond it are refused. |
| `input_actions.maximum_devices` | integer | 1 - 1024 | `32` | Cap on distinct device ids tracked; `feedEvent` for a new device beyond it returns false. |
| `input_actions.maximum_frame_delta_seconds` | float | finite, 0.0 - 1.0 (exclusive lower) | `0.25` | Clamp applied to the `deltaSeconds` passed to `advanceFrame`; larger deltas are clamped, non-positive deltas skip time-based effects. |
| `input_actions.maximum_smoothing_seconds` | float | finite, 0.0 - 10.0 | `2.0` | Upper bound accepted for an action's `smoothingSeconds` in `createAction`; larger values are rejected. |

## Out-of-range and invalid values

`construct()` reads each key through `ParticipantContext::configurationIntegerInSection` or `configurationFloatingInSection` and applies it only when the value is present, has the expected TOML type (integer keys require integers; float keys accept a TOML float or integer), passes `std::isfinite`, and falls inside the range above. Otherwise the assignment is skipped, the built-in default (identical to the `config.toml` value) remains in effect, and nothing is reported to the caller. There are no other failure paths: the module starts normally with any subset of valid keys.

## How the caps shape behavior

- The caps are enforced at object-creation time, not by eviction: existing contexts, actions, bindings, and device states survive a lowered cap; only new requests beyond it are refused. The Lua bridge (`create_context`, `create_action`, `bind_action`) is subject to the same limits and returns `nil`/`false` when a request is refused.
- `maximum_controls_per_device` bounds the stored state a misbehaving producer can accumulate per device (including simultaneous touch contacts); `resetDevice` (or a `focus_lost` event) frees it.
- `maximum_frame_delta_seconds` exists so a pause or debugger break cannot turn one huge delta into teleporting smoothing; see [overview.md](overview.md) for the per-frame pipeline that consumes it.
