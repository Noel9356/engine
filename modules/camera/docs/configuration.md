# camera configuration

All keys live in the `[camera]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. The `config_keys` list in `module.toml` documents the section's keys, but the current kernel does not validate module sections against it: known-key validation (unknown-key warnings) covers the engine's built-in sections and the `config_keys` of game-mod manifests only, so unknown `[camera]` keys are silently accepted.

Every value is read once per `readConfiguration()` (in `construct()`, and again whenever a `ConfigurationReloadedEvent` touches `camera.*` keys). A value outside its documented range is reported on stderr and ignored - the built-in default stays active; nothing is clamped. Keys are listed alphabetically.

| Key | Type | Valid range (validated in `readConfiguration()`) | Default (`config.toml`) | Effect |
| --- | --- | --- | --- | --- |
| `camera.far_z` | float | finite, `(0, 1.0e9]` | `100.0` | Default far-plane distance. Independently of the range check, `far_z` must exceed `near_z`; otherwise the module warns and uses `max(built-in far plane, near_z + 1)`. |
| `camera.fov_degrees` | float | finite, exclusive `(0.1, 179)` | `55.0` | Default vertical FOV in degrees for perspective cameras (`minimumFovYDegrees`/`maximumFovYDegrees` bounds, endpoints excluded). |
| `camera.look_down_key` | string | any (never read by the module) | `"down"` | Key-binding declaration for consumers; see the note below the table. |
| `camera.look_left_key` | string | any (never read by the module) | `"left"` | As above. |
| `camera.look_right_key` | string | any (never read by the module) | `"right"` | As above. |
| `camera.look_speed` | float | finite, `(0, 3600]` | `60.0` | Rotation speed in degrees per second for axis-driven look input. |
| `camera.look_up_key` | string | any (never read by the module) | `"up"` | As above. |
| `camera.max_delta_seconds` | float | finite, `(0, 5]` | `0.25` | Upper clamp (seconds) applied to `update()`'s `deltaSeconds`, so a pause or debugger break cannot teleport the camera. |
| `camera.maximum_cameras` | integer | 1 - 65536 | `256` | Maximum simultaneously live camera records. Existing cameras survive a lower hot-reloaded limit; new cameras are refused until the live count drops below it. |
| `camera.mode` | string | `"fly"`, `"orbit"`, or `"fixed"` | `"fly"` | Default `CameraControlMode` for newly created cameras. An unknown value prints `unknown [camera] mode ...` and falls back to `"fly"`. |
| `camera.mouse_sensitivity` | float | finite, `(0, 100]` | `0.11` | Degrees of rotation per pixel of mouse movement. |
| `camera.move_backward_key` | string | any (never read by the module) | `"s"` | Key-binding declaration for consumers; see the note below the table. |
| `camera.move_down_key` | string | any (never read by the module) | `"left_control"` | As above. |
| `camera.move_forward_key` | string | any (never read by the module) | `"w"` | As above. |
| `camera.move_left_key` | string | any (never read by the module) | `"a"` | As above. |
| `camera.move_right_key` | string | any (never read by the module) | `"d"` | As above. |
| `camera.move_speed` | float | finite, `(0, 1.0e6]` | `4.0` | Fly-mode movement speed in units per second (`defaultMoveSpeed_`). |
| `camera.move_up_key` | string | any (never read by the module) | `"space"` | As above. |
| `camera.near_z` | float | finite, `(0, 1.0e6]` | `0.1` | Default near-plane distance in world units. |
| `camera.orbit_distance` | float | finite, `(0, 1.0e8]` | `6.0` | Initial orbit radius for orbit cameras without an explicit pose. |
| `camera.orbit_zoom_in_key` | string | any (never read by the module) | `"space"` | Key-binding declaration for consumers; see the note below the table. |
| `camera.orbit_zoom_out_key` | string | any (never read by the module) | `"left_control"` | As above. |
| `camera.orbit_zoom_speed` | float | finite, `(0, 1.0e6]` | `4.0` (tracks `move_speed`) | Radius change per second of zoom input in orbit mode. Reset to the resolved `move_speed` before the key is read, so with no key present it follows `move_speed`. |
| `camera.orthographic_height` | float | finite, `(0, 1.0e8]` | `10.0` | Vertical half-extent of the orthographic frustum. |
| `camera.pitch_limit_degrees` | float | finite, exclusive `(0, 90)` | `89.0` | Pitch clamp in degrees for both axes of look rotation. |
| `camera.sprint_key` | string | any (never read by the module) | `"left_shift"` | Key-binding declaration for consumers; see the note below the table. |
| `camera.sprint_multiplier` | float | finite, `(0, 100]` | `3.0` | Fly-speed multiplier while the sprint input is held. |

## Key-binding keys are declarations, not settings

The thirteen `*_key` entries are declared, defaulted, and mirrored in `config_keys`, but `readConfiguration()` never consumes them: the camera is driven by caller-supplied `CameraInputState` values, not by raw keys. Consumers are free to read these strings from the `[camera]` section to build their own bindings (or to route them through the input_actions module); changing them in config has no effect inside the camera module itself.

## Out-of-range / invalid values

Each scalar key is validated with an explicit range check; failures print a `camera module: ignoring [camera] ...` message on stderr (for `mode`: `unknown [camera] mode ...`) and leave the built-in default in place. The `far_z <= near_z` case is also repaired at load time, not just at camera creation.

`module.toml` `config_keys` mirrors the keys above for documentation; unknown `camera.*` keys are silently accepted by the kernel.

Note that all of these are defaults for *new* cameras. A `ConfigurationReloadedEvent` refreshes them, but cameras that already exist keep their current settings.
