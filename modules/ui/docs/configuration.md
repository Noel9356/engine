# ui configuration

The ui module reads four keys from the `[ui]` section. Defaults ship in `config.toml`; the application config or a profile may override any of them. `module.toml` mirrors the keys under `config_keys` for documentation; the kernel validates known keys for engine sections and plugin manifests only, so unknown `[ui]` keys are silently accepted.

The keys are read once in `construct()` (source/ui.cpp). The module does not subscribe to the kernel `ConfigurationReloadedEvent`: context creation freezes the resolved values into each `UiContextRecord`, so a live reload would only affect contexts created afterwards and the host can already override every field per-context via `UiContextDescription`. There is no reload path to consider and no clamping to describe.

| Key | Type | Valid range | Default | Effect |
| --- | --- | --- | --- | --- |
| `ui.save_window_layout` | boolean | `true` / `false` | `false` | Controls ImGui's ini-backed window-layout persistence. `false` (default) disables it (`io.IniFilename = nullptr`): no `imgui.ini` is loaded or written, so startups are deterministic and the frame path contains no file I/O. `true` restores stock ImGui behaviour: the layout is written from inside `NewFrame` roughly every 5 seconds after a layout change - synchronous file I/O in the frame path that shows up as a frame-time spike. Enable it only when overlay layout persistence matters more than the occasional hitch. |
| `ui.ini_filename` | string | any path | `""` (empty) | When `save_window_layout` is true, sets the default ini path used by every context that did not override it through `UiContextDescription::iniFilename`. An empty value falls back to ImGui's built-in `"imgui.ini"`. The string is copied into each `UiContextRecord` so the configuration store may be reloaded without invalidating outstanding contexts. |
| `ui.default_style` | string | `dark` / `light` / `classic` | `dark` | Initial colour theme applied to every new context that did not pin one through `UiContextDescription::initialStyle`. Maps to ImGui's `StyleColorsDark` / `StyleColorsLight` / `StyleColorsClassic`. Unknown values are logged to stderr and fall back to `dark`. |
| `ui.default_font_size` | floating | non-negative finite | `0.0` | Pixel size for the built-in default font added to every new context that did not pin one through `UiContextDescription::defaultFontSize`. `0.0` keeps ImGui's built-in default (13 px). Non-finite or negative values are logged to stderr and fall back to `0.0`. |

## Keys that are not configuration

Everything else the ui module does is configured through the capability API rather than the config file, per host and per context:

- `UiContextDescription` (passed at `createContext`): UI scale, navigation flags, the draw-data reserves and maximums (vertex/index/command counts) that bound one frame's draw data, per-context ini filename override, default font size override, and the initial colour theme override.
- `UiFrameDescription` (passed at every `beginFrame`): display size, framebuffer scale, and delta time.

This keeps per-context memory use host-configurable instead of imposed by one global hardware profile; see [overview.md](overview.md) for the capability calls.
