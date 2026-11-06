# script_runtime configuration

All keys live in the `[script_runtime]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. The `config_keys` list in `module.toml` documents the section's keys, but the current kernel does not validate module sections against it (only engine sections and game-mod manifests are validated), so unknown `[script_runtime]` keys are silently accepted.

Every value is read once in `construct()`. A value outside its documented range is ignored and the built-in default stays active.

| Key | Type | Range | Default | Effect |
| --- | --- | --- | --- | --- |
| `memory_limit_mb` | integer | 1 – 4096 | `64` | Per-runtime Lua heap budget in MiB. Allocations beyond the budget fail inside Lua. |
| `instruction_limit` | integer | ≥ 1000 | `10000000` | VM instructions allowed per execute/call, enforced by the count hook. Runs of pure C code inside one call are not counted. |
| `maximum_script_size_mb` | integer | 1 – 256 | `4` | Largest source (string or file) a runtime will compile, in MiB. |
| `maximum_runtimes` | integer | 1 – 4096 | `256` | Maximum concurrently existing runtimes; `createRuntime` fails with `CapacityExceeded` beyond it. |
| `maximum_total_memory_mb` | integer | 1 – 65536 | `1024` | Module-wide heap budget in MiB shared by all runtimes. |
| `maximum_native_functions` | integer | 1 – 4096 | `256` | Maximum registered native functions per runtime. |
| `maximum_native_arguments` | integer | 1 – 32 | `32` | Maximum arguments a native function accepts from Lua. |
| `maximum_native_results` | integer | 1 – 16 | `16` | Maximum results a native function may return to Lua. |

Notes:

- Hosted script mods always use the module defaults; per-mod limits are not configurable (the manifest has no per-mod resource fields).
- The two memory budgets are independent: a runtime may be well under `memory_limit_mb` while the module-wide `maximum_total_memory_mb` stops it.
- `maximum_script_size_mb` is also capped internally by the memory limit (a script can never be larger than the heap it runs in).
