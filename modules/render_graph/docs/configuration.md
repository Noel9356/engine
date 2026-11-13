# render_graph configuration

All keys live in the `[render_graph]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. The `config_keys` list in `module.toml` documents the section's keys, but the current kernel does not validate module sections against it: known-key validation (unknown-key warnings) covers the engine's built-in sections and the `config_keys` of plugin manifests only, so unknown `[render_graph]` keys are silently accepted.

Values are read once in `construct()` (`source/render_graph.cpp`); the module does not follow configuration reloads, so a restart is needed for changes. Numeric keys are clamped to their upper bound rather than rejected, as shown in the range column. Keys are listed alphabetically.

| Key | Type | Valid range (as read in `construct()`) | Default | Effect |
| --- | --- | --- | --- | --- |
| `render_graph.aliasing_strategy` | string | `"best_fit"`, `"first_fit"`, `"worst_fit"` (any other value keeps `best_fit`) | `"best_fit"` | Slot selection strategy for the transient aliasing allocator. `best_fit` picks the smallest free slot that fits the resource (minimal waste); `first_fit` picks the first slot >= size (fastest lookup); `worst_fit` picks the largest slot (spreads usage, leaves small slots open). When no free slot fits, the allocator grows the selected slot and falls back to the largest available. |
| `render_graph.execution_cache` | boolean | `true` / `false` | `true` | When true, a prebuilt `TaskGraph` is kept per cached plan shape and rebound to the current callback each frame, avoiding per-frame node and adjacency allocations. When false, every `execute` builds a fresh `TaskGraph`. |
| `render_graph.max_passes` | integer | > 0; clamped to <= 4096 (values <= 0 are ignored) | `256` | Largest pass count `compile`/`execute` accept before `CapacityExceeded`. |
| `render_graph.max_resources` | integer | > 0; clamped to <= 8192 (values <= 0 are ignored) | `512` | Largest resource count `compile`/`execute` accept before `CapacityExceeded`. |
| `render_graph.plan_cache_entries` | integer | 0 - 16 (outside the range the key is ignored) | `8` | Compiled-plan cache capacity in entries (hard-capped at 16 internally); 0 disables caching. Enabled by default so a stable graph shape is analysed once instead of every frame. |
| `render_graph.plan_cache_max_elements` | integer | 0 - 1000000 (outside the range the key is ignored) | `16384` | Graphs with more elements (passes + resources + edges) bypass the plan cache instead of being stored. |
| `render_graph.quality` | string | `"low"`, `"medium"`, `"high"`, `"ultra"` (any other value, including `"auto"`, keeps `Auto`) | `"auto"` | Quality preset used by `resolveProfile`: `Auto` derives a device-based scale, explicit presets scale limits by 0.25x/0.5x/1x/2x. |
| `render_graph.transient_aliasing` | boolean | `true` / `false` | `true` | When true, physical slots are reused across transient resources with disjoint lifetimes; when false, every transient resource gets its own slot. |
| `render_graph.transient_budget_mb` | integer | >= 0; clamped to <= 16384 MiB | `512` | Transient memory budget in MiB; allocations beyond it fail with `BudgetExceeded`. |

## Out-of-range and invalid values

`max_passes`, `max_resources`, and `transient_budget_mb` are clamped to their upper bounds (out-of-range-low values keep the default); `quality`, `aliasing_strategy`, `plan_cache_entries`, and `plan_cache_max_elements` fall back to their defaults when the value is outside the accepted set/range; `transient_aliasing` and `execution_cache` default to true when the key is absent. Values arriving through the `setSettings` capability entry instead of configuration are validated strictly by `validSettings` (quality <= Ultra, 1 <= maxPasses <= 4096, 1 <= maxResources <= 8192, transient budget <= 16 GiB) and rejected with `false` on any violation. The aliasing strategy and execution cache toggle are configuration-only; they are not exposed through `RenderGraphSettings`.

## Tuning notes

- The plan cache pays off when the graph shape (passes, resources, lifetimes) is stable across frames; a graph that changes shape every frame both misses the cache and evicts entries, so `plan_cache_entries` can stay at its default there.
- `transient_budget_mb` is an analysis-time cap, not an allocation pool: the graph refuses plans whose transient high-water mark exceeds it so the renderer never gets a plan it cannot back with memory.
- `aliasing_strategy` only changes which free slot a transient resource reuses; the slot count and total bytes stay the same for graphs where every free slot fits. The strategy matters when free slots have varying sizes: `best_fit` minimises fragmentation, `worst_fit` spreads load, `first_fit` is the cheapest to evaluate.
- `execution_cache` removes per-frame `TaskGraph` allocation for stable graph shapes; disable it only when diagnosing execution-path issues, since the per-call fallback rebuilds the graph every frame.
- Profiles shipped in `config/profiles/*.toml` set `transient_budget_mb` and `plan_cache_max_elements` per hardware class; treat those as the calibrated starting points. See [overview.md](overview.md) for the compile/execute pipeline these keys bound.
