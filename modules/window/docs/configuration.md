# window configuration

All keys live in the `[window]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. Every value is re-read in `construct()` and again whenever a `ConfigurationReloadedEvent` touches a `window.*` key (see `source/window.cpp`, `readConfiguration()`). The keys are mirrored in `module.toml` `config_keys` for documentation; the kernel does not validate module sections against that list (only engine sections and game-mod manifests are validated).

| Key | Type | Valid range (validated in `readConfiguration`) | Default (`config.toml`) | Effect |
| --- | --- | --- | --- | --- |
| `backend` | string | `auto`, `glfw`, `headless` | `auto` | Selects the backend. `auto` prefers GLFW and falls back to headless; a forced `glfw` that cannot initialize fails window creation. Unknown values print a stderr warning and behave as `auto`. |
| `width` | integer | 1 – 4294967295 | `1280` | Default window width, used when a caller's `WindowDescription::width` is 0. |
| `height` | integer | 1 – 4294967295 | `720` | Default window height, used when a caller's `WindowDescription::height` is 0. |
| `title` | string | non-empty | `"engine"` | Default title, used when a caller's `WindowDescription::title` is null. |
| `resizable` | boolean | any TOML boolean | unset | Default for `WindowDescription::resizable` (struct default `true`) when the caller leaves `hasResizable` false. |
| `visible` | boolean | any TOML boolean | unset | Default for `WindowDescription::visible` (struct default `true`) when `hasVisible` is false. |
| `decorated` | boolean | any TOML boolean | unset | Default for `WindowDescription::decorated` (struct default `true`) when `hasDecorated` is false. |
| `floating` | boolean | any TOML boolean | unset | Default for `WindowDescription::floating` (struct default `false`) when `hasFloating` is false. |
| `minimum_width` | integer | 0 – 2147483647 | `0` | Default minimum width applied at creation when the caller's limit is 0 (`0` = no limit). Sizes are clamped into it. |
| `minimum_height` | integer | 0 – 2147483647 | `0` | As `minimum_width`, for height. |
| `maximum_width` | integer | 0 – 2147483647 | `0` | Default maximum width (0 = no limit); sizes are clamped into it. |
| `maximum_height` | integer | 0 – 2147483647 | `0` | As `maximum_width`, for height. |
| `event_queue_capacity` | integer | 0 – 4294967295 | `4096` | Per-window event queue capacity, used when the caller does not request one explicitly. |
| `position_x` | integer | -2147483648 – 2147483647 | unset | Initial window X position, used when the caller leaves `hasPosition` false. Both `position_x` and `position_y` must be valid; one alone is ignored. |
| `position_y` | integer | -2147483648 – 2147483647 | unset | Initial window Y position (paired with `position_x`). |

Out-of-range and invalid values: a value outside its documented range is reported on stderr (`window module: ignoring [window] <key>; ...`) and ignored — the built-in default shown above stays active. `position_x` / `position_y` are all-or-nothing: if either is missing or out of the int32 range both are ignored. For the four booleans the module cannot distinguish "unset" from a value (it stores `std::optional<bool>`), so a wrong TOML type simply leaves the key unset and the `WindowDescription` struct default wins.

At window creation the resolved defaults only fill fields the caller genuinely left unset (the `has*` flags in `WindowDescription`). `eventQueueCapacity` follows the same rule strictly: without `hasEventQueueCapacity` the configured capacity applies no matter what value the struct carries (including its 4096 default); with the flag set, the caller's value wins unchanged, where 0 means an unbounded queue.
