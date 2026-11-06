# ui configuration

`construct()` in `source/ui.cpp` reads one boolean key; it does not subscribe to the configuration reload event (the value is read once at startup).

| Key | Type | Valid range | Default | Effect |
| --- | --- | --- | --- | --- |
| `save_window_layout` | boolean | true / false | `false` | When false (default), ImGui's automatic `imgui.ini` persistence is disabled (`io.IniFilename = nullptr`): no ini file is loaded or written. When true, ImGui behaves stock and writes `imgui.ini` from inside `NewFrame` roughly every 5 seconds after a window layout change - synchronous file I/O in the frame path, which shows up as a frame-time spike. Enable only if overlay layout persistence matters more than the occasional hitch. |

Notes:

- The module declares no configuration at all: there is no `[ui]` section contract, and the kernel does not validate module sections in any case (it validates only engine sections and game-mod manifests).
- Values a host may still tune at runtime are passed per call through the API: `UiContextDescription` (scale, navigation flags, draw-data reserves and maximums) at `createContext` time and `UiFrameDescription` (display size, framebuffer scale, delta time) at every `beginFrame`.
- If configuration is ever added, it should follow the module convention: defaults in `config.toml`, keys documented in `module.toml` `config_keys`, read once in `construct()` via `ParticipantContext::configuration*InSection`.
