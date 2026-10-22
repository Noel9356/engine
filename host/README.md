# host

The composition root and frame loop. The host is the only place that knows the concrete module set: it registers module factories with the kernel behind CMake options, parses the command line, drives the frame loop, and maps runtime outcomes to exit codes for automation. Everything below it is interchangeable; everything above it is the operating system.

## Composition

`host/main.cpp` registers one factory per compiled-in module — `logging`, `window`, `input`, `renderer`, `screenshot`, `lua_scripting`, `pbr`, `radiance_cascades`, `ray_reflections` — then hands the configured `[engine].modules` list to the kernel, which resolves the graph, constructs and starts the components in dependency order (mods included, discovered from disk), runs the frame loop until the window module reports a close request, and stops everything in reverse order. Excluding a module at build time removes its factory and its registration; a configuration still listing it fails with a precise diagnostic instead of a link error (the availability header, problem 5 in `docs/robustness.md`).

The frame loop per iteration: `pollEvents()` on the window capability, check the close request, render one frame through the renderer capability, present. There is no engine-side thread and no queue — the loop runs on the main thread, which is why CPU-heavy frames surface as an OS "not responding" when the message pump starves (problem 47) and why the renderer's cost knobs (`[lighting.radiance_cascades].probe_stride` above all) matter interactively. Frames are serialized on the device (the renderer waits both in-flight fences before recording — problem 58), so a frame's GPU work never overlaps the next frame's recording or uniform uploads. Runs stay quiet at shutdown; the communication latency and frame-time report is opt-in.

## Command line

| Flag | Effect |
|---|---|
| `--help` | print usage and exit |
| `--validate` | full dry run: layered config load, manifest build (including every `mod.toml` on disk), graph resolution, permission check — no backend is touched; prints `engine configuration is valid` and exits 0 on success. Keys that no component declares are reported as warnings (see `docs/configuration.md`, "Known keys"); warnings never flip the exit code |
| `--config <path>` | load this configuration instead of `config/engine.toml` (resolved relative to the executable directory) |
| `--scene <path>` | world content file replacing the world (`[scene]`) wholesale, overriding both the assembly's `[scene]` and any mod-declared scene; the file may define only `[scene]` (see `docs/configuration.md`, "The layer stack") |
| `--set <key>=<value>` | override one configuration value; repeatable, highest layer, parsed before any backend starts (malformed assignments exit 2). Values use TOML syntax; a bare word becomes a string |
| `--provenance` | validate and additionally print which layer provided each effective configuration value |
| `--smoke-frames <n>` | render exactly `n` frames, queue one screenshot halfway through (exercising readback and PNG write), shut down cleanly and report through the exit codes below |
| `--leak-check` | evaluate the captured frame against the `[leak_test]` regions after rendering (used with `config/leak_test.toml`) |
| `--latency-report` | opt-in communication latency report (per-channel event dispatch mean and percentiles) plus frame-time percentiles at shutdown; the same report is available with `[diagnostics] latency_report = true`, and `--set diagnostics.latency_report=true` is honored exactly like the key |

## Exit codes

| Code | Meaning |
|---|---|
| `0` | clean run (or successful validation) |
| `1` | the frame loop threw |
| `2` | bad usage (unknown flag, missing/invalid argument) |
| `3` | the graphics backend observed validation errors during the run (the renderer observes them through the debug messenger, so a broken pipeline or a disabled feature fails the run even when the driver would have carried on) |
| `4` | the leak check failed — light measured inside the sealed region, or the lit region stayed dark (a dead GI path) |

The renderer's `validationErrorsObserved()` flag is what turns backend misbehavior into exit code 3; `fail_on_validation_error = true` additionally aborts interactively at the first error.

## The `[leak_test]` section

Host-only configuration consumed by `--leak-check`: the captured frame is evaluated against regions in normalized screen coordinates.

| Key | Type | Default | Meaning |
|---|---|---|---|
| `region` | float[4] | (none) | sealed region `[x0, y0, x1, y1]` that must stay dark |
| `max_luminance` | float | `0.02` | maximum luminance (0-1) inside the sealed region |
| `lit_region` | float[4] | (none) | region that must receive light (guards against a dead GI path) |
| `min_lit_luminance` | float | `0.05` | minimum peak luminance (0-1) inside the lit region |

`config/leak_test.toml` renders the sealed room with the GI technique ONLY (`techniques = ["radiance_cascades"]`): the sealed region then proves the transport path cannot leak, and the lit region proves the GI path is alive — with direct lighting attached, a dead GI used to pass invisibly because the sun lit the lit region directly (problem 29 in `docs/robustness.md`).

## Automation surface

The QA harness drives this host headlessly: `scripts/run_headless_smoke.sh` (boots the assembled runtime under Xvfb or the real display, renders N frames of a given config), `scripts/run_leak_check.sh` (the leak scenario end to end), `scripts/run_qa_pose.sh` and the `scripts/qa/qa_capture` script mod (deterministic camera poses and pixel captures for visual verification). The CTest tiers that wrap them are documented in `docs/building.md`, "Testing".

## Files

- `host/main.cpp` — composition root, CLI parsing, frame loop, exit code mapping
- `host/CMakeLists.txt` — the runtime executable target and the `ENGINE_BUILD_DEMO` option (which also gates the example mods)

## Tests

The host is covered through automation rather than unit tests: the cTest tiers `engine_headless_smoke` and `engine_leak_check` run this executable's `--smoke-frames` and `--leak-check` modes and assert on the exit codes.
