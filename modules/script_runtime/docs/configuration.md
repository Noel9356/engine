# script_runtime configuration

All keys live in the `[script_runtime]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. The `config_keys` list in `module.toml` documents the section's keys, but the current kernel does not validate module sections against it (only engine sections and plugin manifests are validated), so unknown `[script_runtime]` keys are silently accepted.

Every value is read once in `construct()`; the module does not follow configuration reloads. Keys are listed alphabetically within each table.

## Auto config

| Key | Type | Range | Default | Effect |
| --- | --- | --- | --- | --- |
| `script_runtime.auto_config` | boolean | (boolean) | `false` | Hardware-derived memory ceilings. When `true`, `script_runtime.memory_limit_mb` = ~1.6% of installed RAM clamped to 16..256 and `script_runtime.maximum_total_memory_mb` = ~12.5% clamped to 128..4096, and the two memory keys in `[script_runtime]` have no effect. Unknown RAM keeps the built-in ceilings. The remaining keys bound logic rather than memory and stay effective in both modes. A one-line stderr notice reports the derived limits. |

## Manual keys

| Key | Type | Range | Default | Effect |
| --- | --- | --- | --- | --- |
| `script_runtime.instruction_limit` | integer | >= 1000 | `10000000` | VM instructions allowed per execute/call, enforced by the count hook. Runs of pure C code inside one call are not counted. |
| `script_runtime.maximum_native_arguments` | integer | 1 - 32 | `32` | Maximum arguments a native function accepts from Lua. |
| `script_runtime.maximum_native_functions` | integer | 1 - 4096 | `256` | Maximum registered native functions per runtime. |
| `script_runtime.maximum_native_results` | integer | 1 - 16 | `16` | Maximum results a native function may return to Lua. |
| `script_runtime.maximum_runtime_initializers` | integer | 1 - 4096 | `64` | Maximum optional-module Lua bridge initializers that may be registered. Each initializer is applied to existing runtimes and to runtimes created later. |
| `script_runtime.maximum_runtimes` | integer | 1 - 4096 | `256` | Maximum concurrently existing runtimes; `createRuntime` fails with `CapacityExceeded` beyond it. |
| `script_runtime.maximum_script_size_mb` | integer | 1 - 256 | `4` | Largest source (string or file) a runtime will compile, in MiB. |
| `script_runtime.maximum_total_memory_mb` | integer | 1 - 65536 | `1024` | Module-wide heap budget in MiB shared by all runtimes. |
| `script_runtime.memory_limit_mb` | integer | 1 - 4096 | `64` | Per-runtime Lua heap budget in MiB. Allocations beyond the budget fail inside Lua. |

A value outside its documented range is ignored and the built-in default stays active.

## Notes

- Hosted script plugins always use the module defaults; per-plugin limits are not configurable (the manifest has no per-plugin resource fields).
- The two memory budgets are independent: a runtime may be well under `script_runtime.memory_limit_mb` while the module-wide `script_runtime.maximum_total_memory_mb` stops it.
- `script_runtime.maximum_script_size_mb` is also capped internally by the memory limit (a script can never be larger than the heap it runs in).
- The native-function, argument, and result caps directly bound the Lua bridges the optional modules install: every bridge in this engine fits with room to spare (the widest bridge call takes 14 arguments and returns up to 16 results). See [overview.md](overview.md) for the capability surface, [scripting.md](scripting.md) for the script-authoring view, and the per-module [lua_bridge.md](../audio/docs/lua_bridge.md) documents for the bridged functions.
