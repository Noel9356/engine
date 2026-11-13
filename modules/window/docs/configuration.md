# window configuration

All keys live in the `[window]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. Every value is read in `construct()` and again whenever a `ConfigurationReloadedEvent` touches a `window.*` key (see `source/core/window_configuration.cpp`, `readWindowModuleConfiguration()`). The keys are mirrored in `module.toml` `config_keys` for documentation; the kernel does not validate module sections against that list (only engine sections and plugin manifests are validated). Keys are listed alphabetically.

| Key | Type | Valid range (validated in `readWindowModuleConfiguration`) | Default (`config.toml`) | Effect |
| --- | --- | --- | --- | --- |
| `window.backend` | string | `auto`, `glfw`, `headless` | `auto` | Selects the backend. `auto` prefers GLFW and falls back to headless; a forced `glfw` that cannot initialize fails window creation. Unknown values print a stderr warning and behave as `auto`. Read when the backend is first initialized; a reload cannot swap the backend of a running system. |
| `window.decorated` | boolean | any TOML boolean | unset | Default for `WindowDescription::decorated` (struct default `true`) when the caller leaves `hasDecorated` false. |
| `window.event_queue_capacity` | integer | 0 - 4294967295 | `4096` | Per-window event queue capacity, used when the caller does not request one explicitly. |
| `window.floating` | boolean | any TOML boolean | unset | Default for `WindowDescription::floating` (struct default `false`) when `hasFloating` is false. |
| `window.focus_on_show` | boolean | any TOML boolean | unset | Default for `WindowDescription::focusOnShow` (struct default `true`, the GLFW focus-on-show hint) when the caller leaves `hasFocusOnShow` false. |
| `window.gamepad_axis_threshold` | floating | 0.0 - 1.0 | `0.0005` | Minimum absolute change a standardized gamepad axis must report before a `GamepadAxisMoved` event is queued; smaller changes are treated as noise. Runtime knob: applies immediately, including after a reload. |
| `window.gamepad_polling` | boolean | any TOML boolean | `true` | Enables the per-frame standardized-gamepad scan in `pollEvents`/`waitEvents`. Turning it off skips the scan entirely (no connect/disconnect, button, or axis events). Runtime knob: applies immediately, including after a reload. |
| `window.height` | integer | 1 - 4294967295 | `720` | Default window height, used when a caller's `WindowDescription::height` is 0. |
| `window.maximized` | boolean | any TOML boolean | unset | Default for `WindowDescription::maximized` (struct default `false`, the GLFW maximized hint) when the caller leaves `hasMaximized` false. |
| `window.maximum_height` | integer | 0 - 2147483647 | `0` | As `maximum_width`, for height. |
| `window.maximum_width` | integer | 0 - 2147483647 | `0` | Default maximum width (0 = no limit); sizes are clamped into it. |
| `window.minimum_height` | integer | 0 - 2147483647 | `0` | As `minimum_width`, for height. |
| `window.minimum_width` | integer | 0 - 2147483647 | `0` | Default minimum width applied at creation when the caller's limit is 0 (`0` = no limit). Sizes are clamped into it. |
| `window.opacity` | floating | 0.0 - 1.0 | unset | Default for `WindowDescription::opacity` (struct default `1.0`, fully opaque) when the caller leaves `hasOpacity` false. Applied when the window is created; `setOpacity` changes it at runtime. |
| `window.position_x` | integer | -2147483648 - 2147483647 | unset | Initial window X position, used when the caller leaves `hasPosition` false. Both `position_x` and `position_y` must be valid; one alone is ignored. |
| `window.position_y` | integer | -2147483648 - 2147483647 | unset | Initial window Y position (paired with `position_x`). |
| `window.resizable` | boolean | any TOML boolean | unset | Default for `WindowDescription::resizable` (struct default `true`) when the caller leaves `hasResizable` false. |
| `window.title` | string | non-empty | `"engine"` | Default title, used when a caller's `WindowDescription::title` is null. |
| `window.visible` | boolean | any TOML boolean | unset | Default for `WindowDescription::visible` (struct default `true`) when `hasVisible` is false. |
| `window.width` | integer | 1 - 4294967295 | `1280` | Default window width, used when a caller's `WindowDescription::width` is 0. |

## Out-of-range and invalid values

A value outside its documented range is reported on stderr (`window module: ignoring [window] <key>; ...`) and ignored - the built-in default shown above stays active. `position_x` / `position_y` are all-or-nothing: if either is missing or out of the int32 range both are ignored. For the seven has-flag keys (`decorated`, `floating`, `focus_on_show`, `maximized`, `opacity`, `resizable`, `visible`) the module cannot distinguish "unset" from a value, so a wrong TOML type simply leaves the key unset and the `WindowDescription` struct default wins.

## How the defaults are applied

At window creation the resolved defaults only fill fields the caller genuinely left unset (the `has*` flags in `WindowDescription`). `eventQueueCapacity` follows the same rule strictly: without `hasEventQueueCapacity` the configured capacity applies no matter what value the struct carries (including its 4096 default); with the flag set, the caller's value wins unchanged, where 0 means an unbounded queue.

Because the values are defaults rather than commands, reloading configuration never resizes or moves a window that already exists; the new values apply to windows created afterwards. The two exceptions are the runtime knobs `gamepad_polling` and `gamepad_axis_threshold`, which the module forwards to the active backend on every reload (and after backend initialization), so they affect the very next event pump. The `backend` key is read when the backend is first initialized (a reload cannot swap the window backend of a running system). See [overview.md](overview.md) for the backend matrix and event delivery.
