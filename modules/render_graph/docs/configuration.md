# render_graph configuration

All keys live in the `[render_graph]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. The `config_keys` list in `module.toml` documents the section's keys, but the current kernel does not validate module sections against it: known-key validation (unknown-key warnings) covers the engine's built-in sections and the `config_keys` of plugin manifests only, so unknown `[render_graph]` keys are silently accepted.

Values are read once in `construct()` (`source/render_graph.cpp`); the module does not follow configuration reloads, so a restart is needed for changes. Numeric keys are clamped to their upper bound rather than rejected, as shown in the range column. Keys are listed alphabetically.

| Key | Type | Valid range (as read in `construct()`) | Default | Effect |
| --- | --- | --- | --- | --- |
| `render_graph.max_passes` | integer | > 0; clamped to <= 4096 (values <= 0 are ignored) | `256` | Largest pass count `compile`/`execute` accept before `CapacityExceeded`. |
| `render_graph.max_resources` | integer | > 0; clamped to <= 8192 (values <= 0 are ignored) | `512` | Largest resource count `compile`/`execute` accept before `CapacityExceeded`. |
| `render_graph.plan_cache_entries` | integer | 0 - 16 (outside the range the key is ignored) | `8` | Compiled-plan cache capacity in entries (hard-capped at 16 internally); 0 disables caching. Enabled by default so a stable graph shape is analysed once instead of every frame. |
| `render_graph.plan_cache_max_elements` | integer | 0 - 1000000 (outside the range the key is ignored) | `16384` | Graphs with more elements (passes + resources + uses + dependencies) bypass the plan cache instead of being stored. |
| `render_graph.quality` | string | `"low"`, `"medium"`, `"high"`, `"ultra"` (any other value, including `"auto"`, keeps `Auto`) | `"auto"` | Quality preset used by `resolveProfile`: `Auto` derives a device-based scale, explicit presets scale limits by 0.25x/0.5x/1x/2x. |
| `render_graph.transient_aliasing` | boolean | `true` / `false` | `true` | When true, physical slots are reused across transient resources with disjoint lifetimes; when false, every transient resource gets its own slot. |
| `render_graph.transient_budget_mb` | integer | >= 0; clamped to <= 16384 MiB | `512` | Transient memory budget in MiB; allocations beyond it fail with `BudgetExceeded`. |

## Out-of-range and invalid values

`max_passes`, `max_resources`, and `transient_budget_mb` are clamped to their upper bounds (out-of-range-low values keep the default); `quality`, `plan_cache_entries`, and `plan_cache_max_elements` fall back to their defaults when the value is outside the accepted set/range, and `transient_aliasing` defaults to true when the key is absent. Values arriving through the `setSettings` capability entry instead of configuration are validated strictly by `validSettings` (quality <= Ultra, 1 <= maxPasses <= 4096, 1 <= maxResources <= 8192, transient budget <= 16 GiB) and rejected with `false` on any violation.

## Tuning notes

- The plan cache pays off when the graph shape (passes, resources, lifetimes) is stable across frames; a graph that changes shape every frame both misses the cache and evicts entries, so `plan_cache_entries` can stay at its default there.
- `transient_budget_mb` is an analysis-time cap, not an allocation pool: the graph refuses plans whose transient high-water mark exceeds it so the renderer never gets a plan it cannot back with memory.
- Profiles shipped in `config/profiles/*.toml` set `transient_budget_mb` and `plan_cache_max_elements` per hardware class; treat those as the calibrated starting points. See [overview.md](overview.md) for the compile/execute pipeline these keys bound.
