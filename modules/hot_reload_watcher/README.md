# hot_reload_watcher module - the development-loop watcher

## What this module provides

Capability `hot_reload` (implementation-bound member `hot_reload.watcher`, published in `hot_reload_capability.hpp`): filesystem watching for the edit-compile-see loop. The module polls the trees a run actually uses, and every change surfaces as one `HotReloadEvent { kind, relativePath, modId }` on the event bus — on the MAIN thread, in frame order, like every other event. The module itself only watches and reports; what a change means is the consumer's decision, and the shipped consumers already handle the two interesting kinds:

- the renderer_vulkan module re-creates its pipelines when a compiled `.spv` changes (recompile a shader, alt-tab back, the new pipeline is live);
- the lua_scripting module reloads the changed mod's script live (data carries the package id);
- configuration-shaped changes (`mod.toml`, `scene.toml`, engine/mod `.toml`) are logged: an assembly restart is the honest reload path for configuration.

## What gets watched

The standard trees are watched automatically at start: the runtime shader directory, every discovered mod package (scripts, manifests, scenes, shipped configs) and the configuration directory — so a plain run pays no setup and watches exactly what it loads. `watchPath(engineRelativePath)` adds one file or one directory (recursively, matching the watcher's extension set) for content that lives elsewhere; `watchedFileCount()` reports the set's size. A watch path that does not exist is a precise warning, never a silent no-op.

The kinds, and who can react to each:

| kind | file | shipped reaction |
|---|---|---|
| `shader` | a compiled `.spv` under the runtime shaders | renderer re-creates pipelines |
| `script` | a mod's `.lua` | scripting module reloads that package's script |
| `manifest` | a `mod.toml` | logged (restart to apply) |
| `scene` | a `scene.toml` | logged (restart, or re-import through scripts) |
| `config` | an engine/mod `.toml` | logged (restart to apply) |

## Timing

Polling runs on a worker thread; the kernel's `AsyncTaskDispatcher` bridges results onto the main thread, so the frame loop only ever sees a cheap drain. The interval is `[hot_reload].watch_interval_ms` (default 250, clamped to 50..5000): fast enough that the loop feels live, slow enough that a build writing many files costs one scan, not one event per write.

## Activation

Demand-driven like every module: a mod that wants live reloads adds `hot_reload` to its required capabilities and the watcher joins the run; a run whose mods do not demand it never pays for it. The module requires `logging` and declares `filesystem.read` (watching reads file metadata and contents).

## Files

- `hot_reload_capability.hpp` - the published capability surface (the event, the kinds, `watchPath`)
- `hot_reload.hpp` / `hot_reload.cpp` - the module, the poll loop, the event publication
- `module_plugin.cpp` - the static plugin descriptor
- `CMakeLists.txt` - target wiring (`ENGINE_BUILD_MODULE_HOT_RELOAD_WATCHER`)
