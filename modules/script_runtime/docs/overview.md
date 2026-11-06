# script_runtime module

script_runtime gives the engine an embedded scripting layer: sandboxed Lua 5.5 runtimes for game mods, plus the hosting machinery the kernel uses to run `type = "script"` mods. It is the module that makes the Lua bridge real — native code calls into Lua, Lua calls back into native code, and engine events reach script callbacks.

## Capability surface

The module provides the `script_runtime` capability (ABI 2, see `include/engine/script_runtime/capabilities.hpp`). All calls take the runtime handle first; handles are never reused and `0` (`invalidScriptRuntime`) means "none".

| Function | Purpose |
| --- | --- |
| `createRuntime(description, outError)` | Creates a sandboxed runtime. `nullptr` uses the module defaults from `[script_runtime]` configuration. |
| `destroyRuntime(handle)` | Destroys the runtime and its Lua state. |
| `runtimeInfo(handle, outInfo)` | Memory usage and active limits, for dashboards. |
| `executeString(handle, source, chunkName, ...)` | Compiles and runs a source buffer (`"t"` text chunks only). |
| `executeFile(handle, path, ...)` | Reads a file (size-checked against the script size limit) and runs it. |
| `callFunction(handle, name, ...)` | Calls a global function with no arguments. |
| `registerNativeFunction(handle, name, fn, userData)` | Exposes a C++ function to the script as a global. |
| `setGlobalNumber/String`, `getGlobalNumber/String` | Primitive global exchange. |
| `collectGarbage(handle)` | Forces a full Lua GC cycle. |
| `hostedRuntime(modId)` | Handle of the runtime hosting a kernel script mod (v2). |

**Error convention (important):** for `executeString`, `executeFile`, and `callFunction` the boolean return only reports usage problems (bad handle, bad argument, non-function global) or a truncated error text. Compilation and runtime failures are reported by setting the `ScriptError` out-parameter and filling `errorText`; with a large enough buffer the boolean return is `true` even for a failed script. Callers that need to detect script failures must check the `ScriptError` out-parameter.

## Sandbox

Each runtime is a Lua state created with a custom allocator:

- **Memory:** a per-runtime byte budget and a module-wide byte budget (atomic, shared by all runtimes). Growth beyond either fails the allocation, which Lua surfaces as a memory error.
- **Instructions:** a count hook every 1000 VM instructions stops runaway scripts with an "instruction limit exceeded" error; the budget is re-armed for every execute/call.
- **Libraries:** only `math`, `string`, `table`, and `utf8` (plus a trimmed `base`). `dofile`, `loadfile`, and `collectgarbage` are removed and `io`/`os` are never opened, so scripts cannot touch the filesystem or the process.
- **Values:** scripts and native code exchange only nil, boolean, number, and string; argument/result counts are capped per configuration.
- **Threads:** one mutex per runtime serializes Lua access; runtimes are independent of each other.

## Script mod hosting

The kernel hands every discovered `type = "script"` mod to the module that provides the mod's `host_capability` — normally this module — through `ScriptGameModHost::hostScriptGameMod` during the construct phase. Hosting:

1. Creates a runtime with the module defaults.
2. Sets the globals `engine_mod_id`, `engine_mod_version`, `engine_mod_path`.
3. Executes the entry file (the mod's `entry` from `mod.toml`).
4. Calls `on_construct()` if the script defined it.
5. Registers the runtime under the mod id so `hostedRuntime()` can find it.

The module subscribes to the kernel event bus in `construct()` and forwards events to optional global callbacks; absent callbacks are skipped:

| Engine event | Lua callback |
| --- | --- |
| `EngineStartupCompletedEvent` | `on_start()` |
| `EngineFrameEvent` | `on_frame(delta_seconds, elapsed_seconds)` |
| `EngineFixedUpdateEvent` | `on_fixed_update(delta_seconds)` |
| `WindowKeyEvent` | `on_key_pressed(key_name)` |
| `EngineShutdownStartedEvent` | `on_shutdown()` |

A callback that raises a Lua error is reported once per distinct message on stderr and does not stop the engine. The hosting application can look the runtime up with `hostedRuntime(modId)` and register additional native functions into it — that is how the demo gives the `main_menu` mod its drawing functions (see `mods/main_menu/docs/main_menu.md`).

A scripting error while hosting (runtime creation failed, script did not compile) throws; the kernel reports it as a construct failure and aborts startup with a diagnostic.

## Files

| File | Purpose |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capability, declared `config_keys`. Read by the kernel's module discovery so the descriptor does not depend on loading the library. |
| `config.toml` | Default values for the `[script_runtime]` configuration section (see `configuration.md`). |
| `include/engine/script_runtime/capabilities.hpp` | The public capability types (ABI, errors, values, native function signature). |
| `source/script_runtime.cpp` | The whole implementation: sandbox primitives, capability binding, mod hosting, module entry table. |
