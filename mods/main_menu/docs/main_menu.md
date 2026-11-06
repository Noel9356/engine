# main_menu mod

The main_menu mod is a **script mod**: its behaviour lives entirely in `main_menu.lua` and is executed by the `script_runtime` module, which proves the engine's Lua bridge end to end (kernel hosting, event forwarding, native function calls in both directions, and shared globals).

## Package layout

| File | Purpose |
| --- | --- |
| `mod.toml` | Manifest. `type = "script"`, `entry = "main_menu.lua"`, `host_capability = "script_runtime"`. Script mods cannot `provides` capabilities; the three required capabilities (`script_runtime`, `ui`, `render`) only order construction and document what the mod depends on. |
| `main_menu.lua` | The whole mod: visibility state, FPS statistics, and the drawing calls. |

## Runtime model

- The kernel discovers the package, resolves its capability requirements, and hands it to the module that provides `host_capability` (`script_runtime`).
- `script_runtime` runs the entry file in a sandboxed Lua runtime (no `io`, `os`, `dofile`, or `loadfile`; libraries: `math`, `string`, `table`, `utf8`) and sets the globals `engine_mod_id`, `engine_mod_version`, `engine_mod_path`.
- Engine events arrive as optional global callbacks; absent ones are skipped:

| Callback | Trigger |
| --- | --- |
| `on_construct()` | After the entry file loaded. |
| `on_start()` | `EngineStartupCompletedEvent`. |
| `on_frame(delta, elapsed)` | `EngineFrameEvent` (per rendered frame; `delta` is the engine's clamped frame delta). |
| `on_fixed_update(delta)` | `EngineFixedUpdateEvent`. |
| `on_key_pressed(name)` | `WindowKeyEvent`, published by the demo for every key press; `name` is the stable lowercase key name (`"escape"`, `"w"`, ...). |
| `on_shutdown()` | `EngineShutdownStartedEvent`. |

## Drawing bridge

The Lua script cannot call the `ui`/`render` capabilities directly. The demo application looks the hosted runtime up with `ScriptRuntimeCapability::hostedRuntime("main_menu")` and registers these natives into it:

| Function (Lua signature) | Effect |
| --- | --- |
| `menu_begin_frame(w, h, scale_x, scale_y, delta)` -> `bool` | Begins a ui frame. |
| `menu_set_next_window_position(x, y, pivot_x, pivot_y)` -> `bool` | Positions the next window. |
| `menu_set_next_window_size(width, height)` -> `bool` | Sizes the next window. |
| `menu_begin_window(title)` -> `bool` | Begins the window (fixed frameless flags); body may be skipped. |
| `menu_text(value)` -> `bool` | Adds a text line. |
| `menu_end_window()` -> `bool` | Ends the window. |
| `menu_end_frame()` -> `bool` | Ends the frame and submits the overlay draw data to the renderer. |

Every call validates its arguments and returns `false` when the bridge is not ready or the arguments are wrong, so a script bug fails the call instead of the process. Before `on_draw` the demo refreshes the frame-geometry globals `menu_display_width`, `menu_display_height`, `menu_framebuffer_scale_x`, `menu_framebuffer_scale_y`, and `menu_delta_seconds`; `on_draw` is called inside the render frame, between the scene draw and frame submission.

## Behaviour

`Escape` toggles the window. Frames are accumulated into per-second buckets; the last 60 completed buckets feed the minimum, maximum, and frame-weighted average FPS lines, mirroring the previous native implementation. A Lua error in `on_draw` disables the menu for the rest of the run (reported once on stderr).
