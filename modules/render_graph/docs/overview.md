# render_graph module

render_graph is the engine's frame-graph planner: it takes passes and resources described as plain structs, validates them, orders them deterministically, packs transient resources into as few physical memory slots as their lifetimes allow (within a configurable budget), and executes the passes across the kernel task scheduler. The module owns no GPU resources and performs no rendering itself - consumers record their own work in the per-pass callback - so it works for any backend and for tests alike.

## Capability

The module provides the `render_graph` capability.

- ABI version: `2`, exposed as the standalone constant `render_graph::renderGraphCapabilityAbi` (also the default of `RenderGraphCapability::abi`)
- Header: `modules/render_graph/include/engine/render_graph/capabilities.hpp`
- Binding: `ParticipantContext::acquireCapability<RenderGraphCapability>()`; every entry takes `capability.self` as its first argument.

| Entry | Purpose |
| --- | --- |
| `settings(self)` | Pointer to the current `RenderGraphSettings` (owned by the module). |
| `setSettings(self, const RenderGraphSettings*)` | Validates against the accepted ranges; clears the plan cache on success; `false` for null or invalid settings. |
| `resolveProfile(self, const renderer::DeviceProfile*, RenderGraphProfile*)` | Scales the configured limits down for the given device; `false` for null pointers. |
| `compile(self, passes, passCount, resources, resourceCount, outPassOrder, passCapacity, outResourceSlots, resourceCapacity, outResult, outError)` | Orders passes and assigns transient slots; outputs need room for `passCount` / `resourceCount` entries; on failure `outError` explains and outputs are not meaningful. |
| `execute(self, passes, passCount, resources, resourceCount, callback, userData, outError)` | Runs every pass in dependency order on the task scheduler; `callback(userData, passIndex)` fires once per pass with the original index; blocks until all callbacks finished. |

## Provided / required / optional capabilities

From `module.toml` and the entry table at the bottom of `source/render_graph.cpp`:

- **Provides:** `render_graph`
- **Requires:** none
- **Optional:** none
- **Declared permissions:** none

Consumers still need the `capability.render_graph` permission granted by the kernel permission system before `acquireCapability` succeeds (the prefix `capability.` is added automatically by `ParticipantContext`).

## Architecture

| File | Role |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capability, declared `config_keys` (documentation; the kernel validates only game-mod manifests' keys). |
| `config.toml` | Default values for the `[render_graph]` configuration section (see `configuration.md`). |
| `include/engine/render_graph/capabilities.hpp` | Public capability types: sentinel, access/error enums, settings/profile/resource/pass structs, compile result, and the `RenderGraphCapability` function-pointer struct. |
| `include/engine/render_graph/compiler.hpp` | `RenderGraphPlan` (analysis result) and `buildRenderGraphPlan`, the entry point into the graph analysis. |
| `source/render_graph_compiler.cpp` | The analysis itself: validation, read/write hazard edges, deterministic topological ordering, transient resource lifetimes (built as the static `engine_render_graph_plan` library). |
| `source/render_graph.cpp` | The module: capability binding, configuration, device-profile resolution, transient aliasing allocator, plan cache, and TaskGraph-based execution. |

## How it works

**Analysis (`buildRenderGraphPlan`).** Pure computation over the caller's arrays, in stages:

1. Validation: at most 4,096 passes and 8,192 resources; resources need a non-zero size and power-of-two alignment; `transient` and `initiallyDefined` are mutually exclusive; pass use/dependency arrays must be consistent; total uses and explicit dependencies are each capped at 1,000,000 (`CapacityExceeded`).
2. Explicit dependencies are inserted as deduplicated edges, then a first topological sort (Kahn's algorithm with a min-heap, so ties resolve to the lowest pass index) detects cycles among them (`DependencyCycle`).
3. Hazard analysis in that order: every read depends on the resource's previous write; every write depends on the previous write and all pending reads (RAW, WAW, and WAR edges). Reading a transient (or not `initiallyDefined`) resource before any write, or using one resource twice in a single pass, is `InvalidInput`.
4. A second topological sort over the combined edge set produces the final execution order; a cycle here means hazard edges contradicted the explicit dependencies.
5. With `includeResourceLifetimes = true`, first/last use of every resource is recorded as a position in the final order, and transient resources are emitted in `resourceOrder` sorted by first use - the walk order for the aliasing allocator. Execution (`execute`) skips this stage.

**Transient aliasing (`compile`).** Walking `resourceOrder`, the allocator retires physical slots whose last use lies before the candidate's first use (a heap ordered by expiry) and pools them per `compatibilityClass`; a candidate reuses a pooled slot of the same class, best-fit by size (falling back to the largest), or gets a fresh slot. Slot size/alignment grow to fit, and both fresh allocations and growth are checked against the transient memory budget (`BudgetExceeded`). With `transientAliasing = false` every transient resource gets its own slot. The result reports the number of physical slots and their combined size in `RenderGraphCompileResult`.

**Device scaling (`resolveProfile`).** With `Auto` quality the scale comes from the device class and device-local memory: non-Vulkan 0.125; Vulkan discrete 1.0, integrated 0.5, otherwise 0.25, further capped by deviceLocalMemoryBytes / 4 GiB. Explicit presets scale by 0.25x (Low), 0.5x (Medium), 1x (High), 2x (Ultra). Scaled pass/resource counts and the transient budget are clamped back to the configured settings; `asyncCompute` is never enabled in this version.

**Plan cache.** Compiled plans are cached keyed by structural equality of the graph (values, not pointers; alignment, classes, access masks, and dependencies all participate). Up to 16 entries, LRU eviction; graphs whose element count (passes + resources + uses + dependencies) exceeds the configured maximum bypass the cache; allocation failures degrade to "no caching" instead of throwing.

**Execution.** `execute` builds a `TaskGraph` with one node per pass wired with the plan's edges and runs it on the kernel `TaskScheduler`, blocking until every callback finished. A throwing callback or a scheduler rejection surfaces as `ExecutionFailed`.

## Threading and lifetime

- `RenderGraphPlanCache` is mutex-protected and `compile`/`execute` are `const`, so concurrent compile/execute calls are safe with respect to the cache. `settings_` itself is not synchronized: do not call `setSettings` concurrently with compile/execute.
- Pass/resource arrays (`uses`, `dependencies`) are borrowed and only need to live for the duration of the call; outputs are caller-owned. Cached plans are held via `std::shared_ptr<const RenderGraphPlan>` inside the cache, so a plan stays valid while the cache owns it.
- `execute` blocks the calling thread until all pass callbacks have run; the callbacks themselves execute on scheduler worker threads, so they must synchronize whatever shared state they touch.
- The module is created by the kernel through the entry table; the scheduler and profiler pointers are borrowed from the `ParticipantContext` in `construct()` and owned by the kernel.

## Configuration

Configurable through the `[render_graph]` section; every key is documented in [configuration.md](configuration.md).

## Integration

- **Events published:** none.
- **Events subscribed:** none (the module does not touch the event bus; it reads configuration once in `construct()`).
- **Permissions declared:** none in the entry table; acquisition of the capability is gated by the implicit `capability.render_graph` permission.
- **Consumers:** the demo application (`demo_character_customisation/source/main.cpp`) is a real consumer. It loads the module as part of its host options, acquires the capability optionally (with the `renderGraphCapabilityAbi` check), applies a device-tuned settings round (`settings` → render `deviceProfile` → `resolveProfile` → `setSettings`), `compile`s a fixed three-pass frame graph once (independent update passes for rocks, tori, and the wandering dragons, each writing a transient scratch resource), and `execute`s it every frame with callbacks dispatched by the original pass index. Any failure degrades to the same updates run inline on the main thread, so the demo keeps rendering without the capability.

Consumer example (real signatures):

```cpp
#include <engine/render_graph/capabilities.hpp>

auto &graph = appContext->acquireCapability<render_graph::RenderGraphCapability>();
if (graph.abi != render_graph::renderGraphCapabilityAbi)
    return; // ABI mismatch: skip rather than call through a stale struct

render_graph::RenderGraphResourceDescription resources[]{ /* size/alignment/class per buffer */ };
render_graph::RenderGraphResourceUse uses0[]{ {0, render_graph::ResourceAccess::Write} };
render_graph::RenderGraphResourceUse uses1[]{ {0, render_graph::ResourceAccess::Read} };
render_graph::RenderGraphPassDescription passes[2]{};
passes[0].uses = uses0; passes[0].useCount = 1;
passes[1].uses = uses1; passes[1].useCount = 1;
passes[1].dependencies    = ...; // pass indices that must run first
passes[1].dependencyCount = ...;

std::uint32_t order[2]{};
std::uint32_t slots[1]{};
render_graph::RenderGraphCompileResult result{};
render_graph::RenderGraphError         error{};
if (graph.compile(graph.self, passes, 2, resources, 1, order, 2, slots, 1, &result, &error))
{
    graph.execute(graph.self, passes, 2, resources, 1,
        [](void *userData, std::uint32_t passIndex) { /* record/pass work */ },
        nullptr, &error);
}
```

`execute` can be called without a prior `compile`; it builds the same plan internally (without lifetimes, since execution does not allocate).
