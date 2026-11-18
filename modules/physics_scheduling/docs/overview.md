# physics_scheduling module

physics_scheduling provides deterministic simulation scheduling: staggered update batches that spread simulated items across fixed steps at configurable intervals, and a five-band fidelity ladder (Cached, Animation, Procedural, Simplified, Full) that maps a consumer-supplied importance score onto the simulation level each item should run at, with demotion hysteresis that keeps boundary scores from flickering between bands. The module is a pure utility over plain item records — it owns no simulation state, requires no other capability, never touches the physics world, and consumers read the due set and the per-item levels before driving their own simulators, so an application can adopt it selectively (one schedule for cloth instances, another for foliage patches and debris) without changing how anything else steps.

## Capability

The module provides the `physics_scheduling` capability (ABI version constant `physicsSchedulingCapabilityAbi = 1`) and requires no other capability; the kernel constructs it wherever an application or module lists `physics_scheduling` in its demand. Schedule handles are `std::uint32_t`, start at 1, are never reused, and `0` (`invalidSchedule`) means "none". Items are identified by the consumer's own `std::uint64_t` id, which the scheduler echoes but never interprets; duplicate ids inside one schedule are rejected. Importance scores and thresholds are dimensionless values clamped to [0, 1]; intervals are whole fixed steps in [1, 65536].

### Frame-rate independence contract

Call `updateSchedule` exactly once per fixed update; every other entry point is a query or a retune that takes effect from the next pass. The due set and the level transitions depend only on the step sequence and the recorded item state, so identical input sequences produce identical schedules — the same property the physics_rigid stepping contract provides, which is what makes scheduled sub-simulation replayable and testable. An item is due in the pass where `(stepIndex + slotIndex) mod updateInterval` is zero, with `slotIndex` the item's registration ordinal: items sharing an interval are spread evenly across the steps instead of bunching into one heavy pass, and the phase survives other items being added or removed.

| Group | Functions | Purpose |
| --- | --- | --- |
| Schedule lifetime | `createSchedule(description, outError)`, `destroySchedule(handle)` | One schedule owns its items, thresholds, and step counter. `ScheduleDescription` sets the default interval, the four threshold boundaries, and the demotion margin; inverted thresholds are rejected. |
| Items | `addItem(schedule, description, outError)`, `removeItem(schedule, id)` | Registration order is the iteration order (compacted on removals); the slot ordinal pins each item's stagger phase. The initial level comes from the initial score. |
| Retunes | `setItemUpdateInterval`, `setItemImportance` | Interval and score changes take effect from the next pass; the score is clamped to [0, 1] and must be finite. |
| Stepping | `updateSchedule(schedule)` | Advances the schedule one fixed step, recomputes the due set, and applies the fidelity transitions; returns the due count. |
| Queries | `isItemDue`, `itemLevel`, `itemCount`, `copyItemStates`, `copyDueItems` | Per-item due and level queries, a full snapshot in registration order, and the due ids of the last pass. |
| Statistics | `scheduleStatistics` | Item count, the last pass's due count, and per-level histograms. |

**Error convention:** functions returning `bool` fail on unknown handles or ids and on null outputs; creation functions also fill the `SchedulingError` out-parameter (`InvalidHandle`, `InvalidDescription`, `CapacityExceeded`, `Unsupported`).

## Provided / required / optional capabilities

From `module.toml` and the entry table at the bottom of `source/physics_scheduling.cpp` (they agree):

- **Provided:** `physics_scheduling`.
- **Required:** none.
- **Optional:** none.
- **Declared permissions:** none.

Consumers acquire the capability through `ParticipantContext::acquireCapability` with the kernel permission `capability.physics_scheduling`, which the application's demand closure grants.

## Architecture

| File | Purpose |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capability, declared `config_keys`. |
| `config.toml` | Default values for the `[physics_scheduling]` configuration section (see `configuration.md`). |
| `CMakeLists.txt` | Builds `engine_module_physics_scheduling` (a `.so` under `runtime/modules/physics_scheduling/`) and copies `module.toml` next to it. |
| `include/engine/physics_scheduling/capabilities.hpp` | Public capability types: ABI constant, handles, descriptions, item states, statistics, capability struct. |
| `source/physics_scheduling.cpp` | Module class (lifecycle, configuration, budgets, capability binding) plus the schedule and item records. |

## How it works

**Staggered batches.** Each item carries an update interval (steps between updates; the schedule default fills the zero passed at registration). On every pass the item is due when `(stepIndex + slotIndex) mod interval` is zero, where `slotIndex` is the item's registration ordinal and `stepIndex` counts the schedule's own passes. Because the ordinals are distinct, items of one interval distribute themselves evenly over the interval's steps — a thousand items at interval 8 put roughly 125 into each step's due set instead of a thousand into one step — and the copy-of-`copyDueItems` list stays in registration order, so consumers that iterate it do deterministic work per step. Changing an interval re-evaluates the phase immediately, and removing an item never shifts another item's phase because ordinals are never reused.

**Fidelity ladder with hysteresis.** The description's four strictly descending thresholds (`fullThreshold`, `simplifiedThreshold`, `proceduralThreshold`, `animationThreshold`) map the clamped score onto the bands: at or above `fullThreshold` the item runs `Full`, below that the next band down wins, and scores under `animationThreshold` sit in `Cached`. Promotions apply the pass the score crosses the boundary. Demotions wait for the score to fall below the current band's entry threshold times `demotionMargin` — with the defaults a `Simplified` item is only demoted under `0.5 x 0.8 = 0.4` — which absorbs the noise that consumer scores carry near a boundary (distance jitter, screen-size oscillation) without hysteresis timers or per-band dwell counts. A margin of zero restores immediate switching in both directions, and the per-level histogram in the statistics lets a consumer watch the population settle.

**Composing with the simulation modules.** The scheduler is intentionally decoupled: a consumer computes each item's importance however the application measures relevance (camera distance, screen coverage, gameplay tag), writes it through `setItemImportance` before the pass, then reads `copyDueItems` and the levels and applies them — stepping a cloth only when due, dropping an emitter to a coarser pool, swapping a character's jiggle chains for a static pose under `Cached`, or pausing an item entirely. Everything else in the engine keeps its own contract: a reduced update rate changes how often a simulator runs, never its fixed step size, which is what keeps constrained systems stable.

## Threading and lifetime

The module has no locks. Object creation and destruction, `stop()`, and `updateSchedule` must be serialized from one thread (the main or simulation thread); the query entry points may run concurrently with each other while no mutation or pass is in flight. Schedules and items live in dense arrays and die with the schedule: `destroySchedule` releases every item; `stop()` clears all schedules and resets the handle counters. Handles are monotonic and never reused, so a stale handle is rejected rather than aliasing a newer object.

A consumer schedules one pass per fixed update:

```cpp
engine::physics_scheduling::SchedulingCapability &scheduling =
    context.acquireCapability<engine::physics_scheduling::SchedulingCapability>();
if (scheduling.abi != engine::physics_scheduling::physicsSchedulingCapabilityAbi)
{
    return;
}
for (std::uint32_t step = 0; step < timing.fixedUpdateCount; ++step)
{
    for (const Item &item : trackedItems)
    {
        scheduling.setItemImportance(scheduling.self, schedule, item.id, item.score());
    }
    const std::uint32_t dueCount = scheduling.updateSchedule(scheduling.self, schedule);
    (void)dueCount;
    for (const Item &item : trackedItems)
    {
        if (scheduling.isItemDue(scheduling.self, schedule, item.id))
        {
            engine::physics_scheduling::FidelityLevel level = engine::physics_scheduling::FidelityLevel::Full;
            scheduling.itemLevel(scheduling.self, schedule, item.id, &level);
            item.simulate(level);
        }
    }
}
```
