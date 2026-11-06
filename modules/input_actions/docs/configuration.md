# input_actions configuration

All keys live in the `[input_actions]` section. The module's `config.toml` supplies the defaults below; a runtime profile may override them. `module.toml` mirrors every key under `config_keys` for documentation; the kernel validates known keys for engine sections and game-mod manifests only.

| Key | Type | Valid range (as validated in `construct()`) | Default (`config.toml`) | Effect |
| --- | --- | --- | --- | --- |
| `maximum_contexts` | integer | 1..4096 | 64 | Cap on live input contexts; `createContext` returns `invalidInputContext` beyond it. Also used to pre-reserve the frame context list. |
| `maximum_actions_per_context` | integer | 1..65536 | 1024 | Cap on actions per context; `createAction` rejects beyond it. Used to size the per-frame consumption scratch containers. |
| `maximum_bindings_per_action` | integer | 1..4096 | 64 | Cap on bindings per action; `bindAction` rejects beyond it. |
| `maximum_devices` | integer | 1..1024 | 32 | Cap on distinct device ids tracked; `feedEvent` for a new device beyond it returns false. |
| `maximum_controls_per_device` | integer | 1..65536 | 4096 | Cap on stored controls (keys + mouse buttons + gamepad buttons + axes combined) per device; new controls beyond it are refused. |
| `default_deadzone` | float | finite, 0.0..1.0 (exclusive upper) | 0.15 | Radial deadzone for `GamepadAxis` bindings whose `deadzone` is negative; inside the deadzone values read 0. |
| `maximum_smoothing_seconds` | float | finite, 0.0..10.0 | 2.0 | Upper bound accepted for an action's `smoothingSeconds` in `createAction`; larger values are rejected. |
| `maximum_frame_delta_seconds` | float | finite, 0.0..1.0 (exclusive lower) | 0.25 | Clamp applied to the `deltaSeconds` passed to `advanceFrame`; larger deltas are clamped, non-positive deltas skip time-based effects. |

## Invalid and out-of-range values

`construct()` reads each key through `ParticipantContext::configurationIntegerInSection` or `configurationFloatingInSection` and applies it only when the value is present, has the expected TOML type (integer keys require integers; float keys accept a TOML float or integer), passes `std::isfinite`, and falls inside the range above. Otherwise the assignment is skipped, the built-in default (identical to the `config.toml` value) remains in effect, and nothing is reported to the caller. There are no other failure paths: the module starts normally with any subset of valid keys.
