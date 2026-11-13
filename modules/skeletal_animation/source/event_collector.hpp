// Player event collection helpers shared by the player-advance path.
//
// `collectPlayerEvents` is the core algorithm that walks the clip's event
// marker list and emits `AnimationPlayerEvent` entries into a player's
// bounded queue. The forward and reverse paths are separated into helper
// lambdas so the loop-boundary edge cases (markers at 0 and duration, multi-
// loop advances) are easy to read.

#ifndef ENGINE_SKELETAL_ANIMATION_EVENT_COLLECTOR_HPP
#define ENGINE_SKELETAL_ANIMATION_EVENT_COLLECTOR_HPP

#include <engine/skeletal_animation/capabilities.hpp>
#include "records.hpp"

#include <cstdint>

namespace engine::skeletal_animation
{

// Removes all queued events from `player` and resets the overflow indicator.
// Returns the number of events that were dropped so the caller can adjust the
// module-wide pending-event counter.
inline std::uint64_t clearPlayerEvents(PlayerRecord &player) noexcept
{
    const std::uint64_t dropped = std::min<std::uint64_t>(player.events.size(), std::numeric_limits<std::uint64_t>::max());
    player.events.clear();
    player.eventQueueOverflowed = false;
    return dropped;
}

// Attempts to enqueue one event. Returns false (and sets the overflow flag)
// when either the per-player or module-wide queue budget would be exceeded;
// the caller is responsible for stopping the collection loop in that case.
// The `totalPendingEvents` argument is the module-wide counter; on success
// the function increments it via the reference.
bool queuePlayerEvent(PlayerRecord &player, AnimationClipHandle clip, const AnimationEventMarker &marker,
    std::int32_t direction, std::uint32_t maximumEventsPerPlayer, std::uint64_t maximumTotalPendingEvents,
    std::uint64_t &totalPendingEvents) noexcept;

// Walks `clip.eventMarkers` between `player.time` and `advancedTime`,
// emitting events into `player.events` according to the marker-crossing
// semantics documented in `overview.md`. `duration` is `clip.duration`.
// Handles loop boundary crossings and the inclusive-at-destination /
// exclusive-at-start rule for both forward and reverse playback.
void collectPlayerEvents(PlayerRecord &player, AnimationClipHandle clipHandle, const ClipRecord &clip,
    double advancedTime, double duration, std::uint32_t maximumEventsPerPlayer,
    std::uint64_t maximumTotalPendingEvents, std::uint64_t &totalPendingEvents) noexcept;

} // namespace engine::skeletal_animation

#endif
