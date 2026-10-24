# profiler module

CPU-side instrumentation for the engine: the profiler module provides the `ProfilerCapability` (capability name `profiler`, published in `profiler_capability.hpp` next to this module — the single source of truth every consumer includes) and requires `logging`, which it uses as the report sink. Consumers feed counters and named duration samples during the run; the module owns aggregation and emits one deterministic report through the logging capability at shutdown. Nothing else in the engine knows the module exists: like every module it loads exactly when an active mod's manifest demands `profiler`, and a run that never demands it pays nothing.

## Capability surface

`ProfilerCapability` carries two recording methods and one RAII helper, and no output concept at all:

- `addCounterAmount(name, amount)` — accumulates a monotonically growing amount under a name (`"assets.bytes_read"`, `"spawn.objects"`). The unit is the caller's decision and lives in the name.
- `addSampleSeconds(name, seconds)` — records one completed duration. Repeated calls accumulate count, total and maximum, which is everything a mean needs.
- `ProfilerCapability::ScopedSample` — measures one lexical scope from construction to destruction and records it as a sample. Cheap enough for per-frame guards; a hot loop that would sample every iteration should time the batch once instead.

The calls are plain virtual calls on the pointer the consumer acquired — one vtable indirection, no lookup, no locking. Like the event bus the profiler is single threaded by contract: record from the frame thread, the same thread that publishes events. Aggregation state is deliberately minimal (two `std::map`s) so the shutdown report is deterministic across runs, which makes before/after comparisons of a tuning change readable.

## Configuration

The module reads its `[profiler]` section during startup. Unknown keys are ignored, and a missing section means defaults.

| Key | Type | Default | Meaning |
|---|---|---|---|
| `report_level` | string | `"information"` | log level the shutdown report is emitted at: `trace`, `debug`, `information`, `warning`; an unrecognized name falls back to `information` |

The report goes through the logging capability, so it lands wherever the active logging implementation sends its lines (console, file, network — the profiler never knows). An unrecognized level string is not a startup error — the module falls back to `information` so a typo degrades verbosity instead of failing the run.

## Lifecycle and report timing

The module requires `logging`, so the resolver orders it after the logging provider; `construct` acquires the logger once and caches it. The report is emitted from `stop()`, and stop runs in reverse dependency order — the profiler, as a provider, stops after every component that recorded into it, so the report covers the consumers' whole lifecycle, including the work they do in their own stop phases. A run that never constructs the module (no mod demanded the capability) never emits a report: no logger is cached, and `stop` guards on that.

## Adding measurements from a mod or module

Declare `profiler` in the consuming component's requirements (a native mod lists it under `capabilities` in `mod.toml`, a module lists it in its manifest and its static descriptor), acquire the capability through the component context, and call the recording methods. The worked example in the tree is the `profiler_bench` native mod (`../../mods/profiler_bench/`), which records counters and both scoped and manual samples and lets the shutdown report print them.

## Files

- `profiler_capability.hpp` — the published capability surface every consumer includes
- `profiler.hpp` / `profiler.cpp` — the module and its capability implementation
- `module_plugin.cpp` — the static plugin descriptor (name, factory, provided/required capabilities)
- `CMakeLists.txt` — target wiring (`ENGINE_BUILD_MODULE_PROFILER`), the capability INTERFACE target and the header purity guard
