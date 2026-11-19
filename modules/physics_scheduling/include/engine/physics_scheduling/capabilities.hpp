// capabilities.hpp - public C capability surface of the physics_scheduling
// module.
//
// Deterministic simulation scheduling for applications that run many
// simulated objects at different rates and detail levels: staggered update
// batches (an item updates every N steps at a phase derived from its
// registration slot, so work is spread evenly across steps), and a five-band
// fidelity ladder (Cached, Animation, Procedural, Simplified, Full) driven by
// a consumer-supplied importance score with demotion hysteresis. The module
// is a pure utility: it owns no simulation state, requires no other
// capability, and never touches the physics world; consumers read the due
// set and the per-item levels and drive their own simulators accordingly.
//
// Frame-rate independence contract: call updateSchedule exactly once per
// fixed update; the due set and the level transitions then depend only on
// the step sequence, so identical input sequences produce identical
// schedules (the demo self-test verifies this).

#pragma once

#include <cstdint>

namespace engine::physics_scheduling
{

// Increment when any public capability structure or function-table layout
// changes; consumers verify it after acquisition.
//
// ABI history:
//   1 - initial surface (schedules, staggered update batches, the fidelity
//       ladder with demotion hysteresis, item states, statistics).
inline constexpr std::uint32_t physicsSchedulingCapabilityAbi = 1u;

using ScheduleHandle = std::uint32_t;

inline constexpr ScheduleHandle invalidSchedule = 0;

enum class SchedulingError : std::uint32_t
{
    None = 0,

    InvalidHandle,      // unknown schedule handle
    InvalidDescription, // null pointers, non-finite values, inverted thresholds, zero interval
    CapacityExceeded,   // configured schedule or item budget exhausted
    Unsupported         // reserved; entry points without an error out-parameter
                        // report rejections through their return value
};

// Fidelity bands, from lowest cost to highest. The meaning per band is the
// consumer's to choose; the intent mirrors the simulation fidelity levels:
// Cached (no continuously running simulation), Animation (predefined motion
// with physical adjustments), Procedural (mathematical models and springs),
// Simplified (reduced geometry or solver complexity), Full (the detailed
// physical model).
enum class FidelityLevel : std::uint32_t
{
    Cached = 0,
    Animation,
    Procedural,
    Simplified,
    Full
};

inline constexpr std::uint32_t kFidelityLevelCount = 5u;

struct ScheduleDescription
{
    // Update interval assigned to items that pass 0 in their ItemDescription
    // (steps between updates; 1 = every step).
    std::uint32_t defaultUpdateInterval = 1;

    // Importance thresholds that map a score in [0, 1] to the fidelity
    // ladder. They must be strictly descending from Full down to Animation;
    // scores below animationThreshold sit in Cached.
    double fullThreshold       = 0.75;
    double simplifiedThreshold = 0.5;
    double proceduralThreshold = 0.25;
    double animationThreshold  = 0.1;

    // Demotion hysteresis fraction (clamped to [0, 1)): an item keeps its
    // current level until its score falls below the level's entry threshold
    // times this margin, which stops boundary scores from flickering the
    // level every step. Zero switches immediately in both directions.
    double demotionMargin = 0.8;
};

struct ItemDescription
{
    // Consumer-owned stable id echoed through the queries; duplicates are
    // rejected. The scheduler never interprets the value.
    std::uint64_t id = 0;

    // Steps between updates (1 = every step); 0 adopts the schedule default.
    std::uint32_t updateInterval = 0;

    // Initial importance score (clamped to [0, 1]).
    double importance = 0.0;
};

// One item's scheduling state after the latest updateSchedule pass.
struct ItemState
{
    std::uint64_t id = 0;

    bool          due = false; // scheduled to run in the pass just executed
    std::uint32_t updateInterval = 0;

    FidelityLevel level = FidelityLevel::Full;
    double        importance = 0.0;
};

struct ScheduleStatistics
{
    std::uint32_t itemCount = 0;
    std::uint32_t dueItemCount = 0;                  // due set size of the last update pass
    std::uint32_t levelCounts[5] = {0, 0, 0, 0, 0}; // indexed by FidelityLevel
};

// The "physics_scheduling" capability. Calls are main-thread only; the
// module takes no internal locks and never calls into other capabilities.
struct SchedulingCapability
{
    static constexpr const char *capabilityName = "physics_scheduling";

    std::uint32_t abi = physicsSchedulingCapabilityAbi;

    void *self = nullptr;

    ScheduleHandle (*createSchedule)(void *self, const ScheduleDescription *description, SchedulingError *error);
    void (*destroySchedule)(void *self, ScheduleHandle schedule);

    // Adds one item; the id must be unique inside the schedule. Items are
    // iterated in registration order, compacted on removals.
    bool (*addItem)(void *self, ScheduleHandle schedule, const ItemDescription *description, SchedulingError *error);
    bool (*removeItem)(void *self, ScheduleHandle schedule, std::uint64_t id);

    // Runtime retunes; both take effect from the next update pass.
    bool (*setItemUpdateInterval)(void *self, ScheduleHandle schedule, std::uint64_t id, std::uint32_t interval);
    bool (*setItemImportance)(void *self, ScheduleHandle schedule, std::uint64_t id, double importance);

    // Advances the schedule one fixed step: computes the due set (staggered
    // by registration slot) and applies the fidelity transitions with
    // hysteresis. Returns the number of due items.
    std::uint32_t (*updateSchedule)(void *self, ScheduleHandle schedule);

    // Per-item queries; unknown ids report false.
    bool (*isItemDue)(void *self, ScheduleHandle schedule, std::uint64_t id);
    bool (*itemLevel)(void *self, ScheduleHandle schedule, std::uint64_t id, FidelityLevel *outLevel);

    // Snapshot of every item in registration order (due flag, interval,
    // level, importance). copyItemStates writes up to capacity states and
    // reports the required count through outRequired.
    bool (*itemCount)(void *self, ScheduleHandle schedule, std::uint32_t *outCount);
    bool (*copyItemStates)(
        void *self, ScheduleHandle schedule, ItemState *outStates, std::uint32_t capacity, std::uint32_t *outRequired);

    // Ids due in the last update pass, in registration order.
    bool (*copyDueItems)(
        void *self, ScheduleHandle schedule, std::uint64_t *outIds, std::uint32_t capacity, std::uint32_t *outRequired);

    bool (*scheduleStatistics)(void *self, ScheduleHandle schedule, ScheduleStatistics *outStatistics);
};

} // namespace engine::physics_scheduling
