# ui configuration

The ui module reads exactly one boolean key from the `[ui]` section. Defaults ship in `config.toml`; the application config or a profile may override it. `module.toml` mirrors the key under `config_keys` for documentation; the kernel validates known keys for engine sections and plugin manifests only, so unknown `[ui]` keys are silently accepted.

The key is read once in `construct()` (source/ui.cpp). The module does not subscribe to the kernel `ConfigurationReloadedEvent`, so changing the value requires an application restart; there is no reload path to consider and no clamping to describe.

| Key | Type | Valid range | Default | Effect |
| --- | --- | --- | --- | --- |
| `ui.save_window_layout` | boolean | `true` / `false` | `false` | Controls ImGui's ini-backed window-layout persistence. `false` (default) disables it (`io.IniFilename = nullptr`): no `imgui.ini` is loaded or written, so startups are deterministic and the frame path contains no file I/O. `true` restores stock ImGui behavior: the layout is written from inside `NewFrame` roughly every 5 seconds after a layout change - synchronous file I/O in the frame path that shows up as a frame-time spike. Enable it only when overlay layout persistence matters more than the occasional hitch. |

## Keys that are not configuration

Everything else the ui module does is configured through the capability API rather than the config file, per host and per context:

- `UiContextDescription` (passed at `createContext`): UI scale, navigation flags, and the draw-data reserves and maximums (vertex/index/command counts) that bound one frame's draw data.
- `UiFrameDescription` (passed at every `beginFrame`): display size, framebuffer scale, and delta time.

This keeps per-context memory use host-configurable instead of imposed by one global hardware profile; see [overview.md](overview.md) for the capability calls.
