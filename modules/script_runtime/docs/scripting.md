# Writing Lua script plugins

A script plugin is a package directory under a plugins directory containing `plugin.toml` and at least the entry script:

```toml
capabilities    = ["script_runtime"]
engine_api      = 1
entry           = "my_mod.lua"
host_capability = "script_runtime"
id              = "my_mod"
type            = "script"
version         = "0.1.0"
```

Rules enforced by the kernel:

- `type` must be `"script"`; `entry` is a relative path inside the package.
- `host_capability` must be set and must also appear in `capabilities` — this is what orders the hosting module before the plugin.
- Script plugins cannot declare `provides` (only native plugins publish capabilities).
- Every entry in `capabilities` must be provided by some active module or the graph fails to resolve.
- `.lua` files in the package are part of the trust hash; a trusted plugin that changes its scripts must be re-trusted.

## Environment

The entry file runs once in its own sandboxed runtime. Available:

- Globals set by the host: `engine_plugin_id`, `engine_plugin_version`, `engine_plugin_path`.
- Libraries: `math`, `string`, `table`, `utf8`, and a trimmed `base` (`print` works; `dofile`/`loadfile`/`load`/`collectgarbage` do not exist; `string.dump` is removed; no `io`/`os`/`package`/`debug`). The `coroutine` library is available when the host enables it (the engine default keeps it off; set the `ScriptLibraryCoroutine` bit when creating a runtime, or enable it through `script_runtime.default_enabled_libraries`).
- Functions registered by the hosting application (for example the demo's `menu_*` bridge) — check for existence if your plugin must run in several games: `if menu_text ~= nil then ... end`.
- Anything the application registered via `registerNativeFunction` on the runtime returned by `hostedRuntime(pluginId)`.
- Functions installed by the optional modules' Lua bridges when those modules are active: `audio.*`, `camera.*`, `input_actions.*`, and `skeletal_animation.*`. Each module's `docs/lua_bridge.md` documents its exact function set.

## Callbacks

Define only the callbacks you need; every one is optional and events with no matching global are silently skipped.

```lua
function on_construct() end                   -- entry file finished loading
function on_start() end                       -- engine startup completed
function on_frame(delta_seconds, elapsed_seconds) end  -- every rendered frame
function on_fixed_update(delta_seconds) end   -- fixed-rate simulation step
function on_key_pressed(key_name) end         -- "escape", "w", "left_shift", ...
function on_shutdown() end                    -- engine shutdown beginning
```

Timing notes:

- `on_frame` runs when the frame event is published, after the frame's render submission. `delta_seconds` is the engine's clamped frame delta.
- Drawing must happen inside the render frame, so the application calls a dedicated function (like `on_draw`) itself between scene draw and frame submission — see the main_menu plugin for the pattern.
- A runtime error inside a callback is printed once per distinct message and the plugin keeps running; an error that prevents hosting aborts startup.

## Performance and safety

- Scripts share the module-wide memory budget; keep per-frame allocations low and prefer reusing tables.
- The instruction limit applies per execute/call and is checked in batches (`instruction_hook_granularity`, default 1000 Lua instructions) to limit hook overhead: one `on_frame` invocation that loops forever is stopped, but the budget restarts next frame. Lower granularity catches runaway loops sooner at higher overhead.
- Native calls are the expensive boundary; batch data (tables of numbers are not supported — use multiple numbers or strings) rather than calling in a tight loop.
- The sandbox is text-only: scripts are compiled with `luaL_loadbufferx` in `"t"` mode, and the `load` global is removed so a script cannot load binary chunks at runtime.
