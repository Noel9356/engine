# lua_scripting module

The script-mod host. This module embeds Lua 5.4 (via sol2) and implements the kernel's `ScriptModHost` surface (`kernel/include/engine/kernel/component_interface.hpp`): for every discovered script mod whose declared `host_capability` resolves to the `scripting` family member this module provides (`scripting.lua`), it creates an isolated Lua state, loads the package's entry file, and exposes a permission-checked `engine` bridge table. The capability publishes no C++ header on purpose: its consumer surface IS the Lua bridge, and the kernel reaches the hosting surface through the kernel-owned `ScriptModHost` interface. Script mods therefore need no build system, no compiler, and no C++ knowledge — a `mod.toml` plus a `.lua` file is a complete mod (`mods/README.md`). A mod never names its host: the kernel resolves the declared host capability to whichever active module provides it.

## The bridge

The `engine` table is the entire script ABI, and every function speaks capability-level concepts (log lines, key names, window pixel sizes, capture requests), never implementation vocabulary — that is what makes script mods implementation-, platform- and vendor-agnostic by construction. The full function reference lives in `mods/README.md`, section "Scripts: the Lua bridge"; the table below lists the core surface.

| Function | Effect |
|---|---|
| `engine.log_trace(msg)` / `log_debug` / `log_information` / `log_warning` / `log_error` | write through the logging capability |
| `engine.subscribe_key_pressed(fn)` / `engine.subscribe_key_released(fn)` | call `fn(key_name)` on key events (published by the input_glfw module) |
| `engine.subscribe_frame_begin(fn)` | call `fn(frame_index)` once per rendered frame (published by the renderer before it records) |
| `engine.window_size()` | returns `width, height` of the main window |
| `engine.request_screenshot()` | queues a screenshot through the screenshot_png module |

Isolation: each script runs in its own Lua state, so script mods cannot see each other's globals, and a crashing script surfaces as a load/call error diagnostic naming the mod instead of taking the app down. Permissions are enforced at the boundary: a script mod's declared grants (for example `logging.write` — see `mods/README.md`) are validated at startup, and a denied permission fails the run precisely; every bridge call re-checks the calling mod's grants before it executes, so a mod without the grant gets a Lua error from the call, never a silent success. The screenshot capability is resolved at the first bridge CALL, not at construct: this module only requires `logging` and `window`, so the kernel's construct order may run it before the screenshot_png module registers its capability, and a construct-time acquire would cache that absence for the process lifetime.

## Wiring

| Manifest field | Value |
|---|---|
| provided capabilities | `scripting` (the script-host surface behind `ScriptModHost`) |
| required capabilities | `logging`, `window` |
| required permissions | `capability.logging`, `capability.window`, plus `capability.<name>` for every capability the bridge can call into (`screenshot`, `input`, `scene`, `lighting`, `renderer`, `animation`, `asset`, `physics`, `audio`, `ui`) |

The module activates on demand: a script mod whose `host_capability` is `scripting` pulls it into the run (nothing else does — a run with no script mods never loads it). The kernel orders every script mod after its host automatically, so a script can assume the modules its bridge calls into are started.

## Configuration

None — the module takes no configuration section. Script mods are discovered from disk (`[engine].mod_directory`, default `mods`), and per-mod behavior lives in each package's `mod.toml` and script, not in `engine.toml`.

## Files

- `lua_scripting.hpp` / `lua_scripting.cpp` — the module, the sol2 bridge, per-mod state management
- `module_plugin.cpp` — the static plugin descriptor (name, factory, provided capabilities)
- `CMakeLists.txt` — target and option wiring (`ENGINE_BUILD_MODULE_LUA_SCRIPTING`); the build acquires Lua 5.4 and sol2 through the dependency tiers in `docs/building.md` (Lua is pinned to 5.4 because sol2 requires it exactly)

The shipped content package `mods/test_scene/` is the minimal live script mod: discovery, manifest validation, hosting and the bridge's log path are exercised by every run that loads it.
