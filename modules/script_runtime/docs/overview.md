# script_runtime module

script_runtime gives the engine an embedded scripting layer: sandboxed Lua 5.5 runtimes for plugins, plus the hosting machinery the kernel uses to run `type = "script"` plugins. It is the module that makes the Lua bridge real — native code calls into Lua, Lua calls back into native code, and engine events reach script callbacks.

## Capability surface

The module provides the `script_runtime` capability (ABI 3, see `include/engine/script_runtime/capabilities.hpp`). All calls take the runtime handle first; handles are never reused and `0` (`invalidScriptRuntime`) means "none".

| Function | Purpose |
| --- | --- |
| `createRuntime(description, outError)` | Creates a sandboxed runtime. `nullptr` uses the module defaults from `[script_runtime]` configuration. |
| `destroyRuntime(handle)` | Destroys the runtime and its Lua state. |
| `runtimeInfo(handle, outInfo)` | Memory usage and active limits, for dashboards. |
| `executeString(handle, source, chunkName, ...)` | Compiles and runs a source buffer (`"t"` text chunks only). |
| `executeFile(handle, path, ...)` | Reads a file (size-checked against the script size limit) and runs it. |
| `callFunction(handle, name, ...)` | Calls a global function with no arguments. |
| `registerNativeFunction(handle, name, fn, userData)` | Exposes a C++ function to the script as a global; dotted names create nested tables (for example, `audio.create_engine` becomes `audio.create_engine`). |
| `setGlobalNumber/String`, `getGlobalNumber/String` | Primitive global exchange. |
| `collectGarbage(handle)` | Forces a full Lua GC cycle. |
| `hostedRuntime(pluginId)` | Handle of the runtime hosting a kernel script plugin (v2). |
| `registerRuntimeInitializer(fn, userData)` | Installs a module-owned bridge into every existing runtime and every runtime created later (v3). |
| `unregisterRuntimeInitializer(handle)` | Stops applying that initializer to runtimes created later. Existing native functions remain registered until their runtime is destroyed. |

**Error convention (important):** for `executeString`, `executeFile`, and `callFunction` the boolean return only reports usage problems (bad handle, bad argument, non-function global) or a truncated error text. Compilation and runtime failures are reported by setting the `ScriptError` out-parameter and filling `errorText`; with a large enough buffer the boolean return is `true` even for a failed script. Callers that need to detect script failures must check the `ScriptError` out-parameter.

## Native bridge value helpers

`include/engine/script_runtime/native_values.hpp` provides shared C++ helpers for module-owned native functions. `number`, `uint32Value`, `int32Value`, and `floatValue` validate numeric types, finiteness, integral ranges, and float representability. `boolean` and `stringValue` validate primitive arguments; `optionalBoolean` and the fallback/required parameters on numeric conversions, implement explicit optional-argument defaults. String views and pointers remain borrowed for the duration of the native call and must not be retained. `resultNumber`, `resultBoolean`, `resultString`, and `resultNil` initialize result slots before writing their value.

These helpers do not alter the `ScriptRuntimeCapability` ABI or the native-function calling convention. Domain-specific enum parsing, argument-count checks, capability calls, and error mapping remain in the owning module's bridge.

## Optional module bridges

Modules that install Lua APIs can use `canRegisterRuntimeInitializer()` and
`tryRegisterRuntimeInitializer()` from the capability header to check provider
compatibility and register their runtime callback. Call
`unregisterRuntimeInitializer()` during module shutdown; it clears the local
initializer handle after attempting removal. The callback context must remain
alive until the module has unregistered it and completed its bridge shutdown.

Modules may declare `script_runtime` as an optional capability and register a runtime initializer during `construct()`. The initializer receives each runtime handle and can register that module's native functions with `registerNativeFunction`. Registration also visits runtimes that already exist; future runtimes receive the initializer before `createRuntime()` returns. The configured `maximum_runtime_initializers` limits registered module hooks. Unregister the hook during module shutdown. It does not remove native functions already installed into existing Lua states, so bridge callbacks must remain safe for the lifetime of those states.

## Sandbox

Each runtime is a Lua state created with a custom allocator:

- **Memory:** a per-runtime byte budget and a module-wide byte budget (atomic, shared by all runtimes). Growth beyond either fails the allocation, which Lua surfaces as a memory error.
- **Instructions:** a count hook every 1000 VM instructions stops runaway scripts with an "instruction limit exceeded" error; the budget is re-armed for every execute/call.
- **Libraries:** only `math`, `string`, `table`, and `utf8` (plus a trimmed `base`). `dofile`, `loadfile`, and `collectgarbage` are removed and `io`/`os` are never opened, so scripts cannot touch the filesystem or the process.
- **Values:** scripts and native code exchange only nil, boolean, number, and string; argument/result counts are capped per configuration.
- **Threads:** one mutex per runtime serializes Lua access; runtimes are independent of each other.

## Script plugin hosting

The kernel hands every discovered `type = "script"` plugin to the module that provides the plugin's `host_capability` — normally this module — through `ScriptPluginHost::hostScriptPlugin` during the construct phase. Hosting:

1. Creates a runtime with the module defaults.
2. Sets the globals `engine_plugin_id`, `engine_plugin_version`, `engine_plugin_path`.
3. Executes the entry file (the plugin's `entry` from `plugin.toml`).
4. Calls `on_construct()` if the script defined it.
5. Registers the runtime under the plugin id so `hostedRuntime()` can find it.

The module subscribes to the kernel event bus in `construct()` and forwards events to optional global callbacks; absent callbacks are skipped:

| Engine event | Lua callback |
| --- | --- |
| `EngineStartupCompletedEvent` | `on_start()` |
| `EngineFrameEvent` | `on_frame(delta_seconds, elapsed_seconds)` |
| `EngineFixedUpdateEvent` | `on_fixed_update(delta_seconds)` |
| `WindowKeyEvent` | `on_key_pressed(key_name)` |
| `EngineShutdownStartedEvent` | `on_shutdown()` |

A callback that raises a Lua error is reported once per distinct message on stderr and does not stop the engine. The hosting application can look the runtime up with `hostedRuntime(pluginId)` and register additional native functions into it — that is how the demo gives the `main_menu` plugin its drawing functions (see `plugins/main_menu/docs/main_menu.md`).

A scripting error while hosting (runtime creation failed, script did not compile) throws; the kernel reports it as a construct failure and aborts startup with a diagnostic.

## Files

| File | Purpose |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capability, declared `config_keys`. Read by the kernel's module discovery so the descriptor does not depend on loading the library. |
| `config.toml` | Default values for the `[script_runtime]` configuration section (see `configuration.md`). |
| `include/engine/script_runtime/capabilities.hpp` | The public capability types (ABI, errors, values, native function signature) and optional-module initializer helpers. |
| `include/engine/script_runtime/native_values.hpp` | Allocation-free argument validation and result-slot helpers for native Lua bridge implementations. |
| `source/script_runtime.cpp` | The whole implementation: sandbox primitives, capability binding, plugin hosting, module entry table. |
