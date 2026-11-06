# ui configuration

`construct()` in `source/ui.cpp` reads one boolean key from `[ui]`; it does not subscribe to the configuration reload event (the value is read once at startup). `module.toml` declares the key in `config_keys`.

| Key | Type | Valid range | Default | Effect |
| --- | --- | --- | --- | --- |
| `ui.save_window_layout` | boolean | true / false | `false` | When false (default), ImGui's automatic `imgui.ini` persistence is disabled (`io.IniFilename = nullptr`): no ini file is loaded or written. When true, ImGui behaves stock and writes `imgui.ini` from inside `NewFrame` roughly every 5 seconds after a window layout change - synchronous file I/O in the frame path, which shows up as a frame-time spike. Enable only if overlay layout persistence matters more than the occasional hitch. |

Notes:

- `ui.save_window_layout` is the module's only configuration key. The same key is listed in `module.toml`; the kernel currently does not validate module sections.
- Values a host may still tune at runtime are passed per call through the API: `UiContextDescription` (scale, navigation flags, draw-data reserves and maximums) at `createContext` time and `UiFrameDescription` (display size, framebuffer scale, delta time) at every `beginFrame`.
- Context memory use remains host-configurable per `UiContextDescription`: reserve and maximum vertex/index/command counts are supplied at `createContext` time rather than imposed by one global hardware profile.
