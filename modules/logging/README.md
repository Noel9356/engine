# logging module

The foundation service of the engine: a leveled, tagged message sink that every other component depends on. The logging module provides the `LoggerCapability` (capability name `logging`, contract in `kernel/include/engine/capabilities/logging_contract.hpp`) and requires nothing itself, so it is always the first component in the dependency graph and is conventionally listed first under `[engine].modules`. Every module manifest declares `capability.logging` as a required permission, and mods that want to write log lines declare `logging` under `capabilities` in their `mod.toml` — the permission system therefore routes all logging through one auditable grant.

## Contract

`LoggerCapability` is deliberately the smallest contract in the tree: one method, `logMessage(LogLevel level, const std::string& message)`. The `LogLevel` enum next to it defines the five levels the engine uses — `trace`, `debug`, `information`, `warning`, `error`. There is no formatting API, no sink selection, and no category argument on the contract; the module owns presentation, and a mod or module that wants structure prefixes the message itself. Keeping the contract this small means a replacement implementation (for example a networked or binary sink) can drop in without any consumer noticing more than the output format.

## Configuration

The module reads its `[logging]` section during startup. Unknown keys are ignored, and a missing section means defaults.

| Key | Type | Default | Meaning |
|---|---|---|---|
| `level` | string | `"information"` | minimum level that is emitted: `trace`, `debug`, `information`, `warning`, `error`; an unrecognized name falls back to `information` |
| `file` | string | (console only) | additional sink: log lines are appended to this file, resolved relative to the executable directory (e.g. `engine.log` writes next to the binary) |

An unrecognized level string is not a startup error — the module falls back to `information` so a typo degrades verbosity instead of failing the run.

## Implementation

The module is intentionally dependency-light: standard library only, no third-party code. Console output goes to standard output and is decorated based on what it is attached to — `standardOutputIsTerminal()` probes with `isatty`/`_isatty`, so interactive runs get human-oriented presentation while redirected or captured runs (CI logs, the headless smoke run) stay plain. Lines are time-stamped with wall-clock time (`std::chrono`/`std::ctime`), prefixed with the level tag, and the configured minimum level filters everything below it. When `[logging].file` is set, the file sink is opened at startup in append mode and every emitted line goes to both sinks; the file is flushed on each write so a crashed run keeps the lines it managed to emit.

The engine's own startup story depends on this module being observable early: the kernel's startup diagnostics (unknown capabilities, ambiguous providers, cycles, permission denials — see `docs/architecture.md`) are written through the logging capability once the logging component is up, which is another reason it must construct first.

## Files

- `logging.hpp` / `logging.cpp` — the module and its capability implementation
- `CMakeLists.txt` — target and option wiring (`ENGINE_BUILD_MODULE_LOGGING`)

## Tests

No dedicated unit suite: the module is exercised continuously by every other test through the kernel integration tests (`tests/kernel_integration_tests.cpp` boot real component graphs with logging attached), and the headless smoke run asserts a clean run with logging active.
