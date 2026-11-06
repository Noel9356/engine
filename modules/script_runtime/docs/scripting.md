# Writing Lua script mods

A script mod is a package directory under a mods directory containing `mod.toml` and at least the entry script:

```toml
id              = "my_mod"
version         = "0.1.0"
engine_api      = 1
type            = "script"
entry           = "my_mod.lua"
host_capability = "script_runtime"
capabilities    = ["script_runtime"]
```

Rules enforced by the kernel:

- `type` must be `"script"`; `entry` is a relative path inside the package.
- `host_capability` must be set and must also appear in `capabilities` — this is what orders the hosting module before the mod.
- Script mods cannot declare `provides` (only native mods publish capabilities).
- Every entry in `capabilities` must be provided by some active module or the graph fails to resolve.
- `.lua` files in the package are part of the trust hash; a trusted mod that changes its scripts must be re-trusted.

## Environment

The entry file runs once in its own sandboxed runtime. Available:

- Globals set by the host: `engine_mod_id`, `engine_mod_version`, `engine_mod_path`.
- Libraries: `math`, `string`, `table`, `utf8`, and a trimmed `base` (`print` works; `dofile`/`loadfile`/`collectgarbage` do not exist; no `io`/`os`).
- Functions registered by the hosting application (for example the demo's `menu_*` bridge) — check for existence if your mod must run in several games: `if menu_text ~= nil then ... end`.
- Anything the application registered via `registerNativeFunction` on the runtime returned by `hostedRuntime(modId)`.

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
- Drawing must happen inside the render frame, so the application calls a dedicated function (like `on_draw`) itself between scene draw and frame submission — see the main_menu mod for the pattern.
- A runtime error inside a callback is printed once per distinct message and the mod keeps running; an error that prevents hosting aborts startup.

## Performance and safety

- Scripts share the module-wide memory budget; keep per-frame allocations low and prefer reusing tables.
- The instruction limit applies per execute/call: one `on_frame` invocation that loops forever is stopped, but the budget restarts next frame.
- Native calls are the expensive boundary; batch data (tables of numbers are not supported — use multiple numbers or strings) rather than calling in a tight loop.
