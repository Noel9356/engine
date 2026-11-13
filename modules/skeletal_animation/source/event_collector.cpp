// Event collection implementation; see event_collector.hpp for the contract.

#include "event_collector.hpp"

#include <algorithm>
#include <cstdint>
#include <new>

namespace engine::skeletal_animation
{

bool queuePlayerEvent(PlayerRecord &player, AnimationClipHandle clip, const AnimationEventMarker &marker,
    std::int32_t direction, std::uint32_t maximumEventsPerPlayer, std::uint64_t maximumTotalPendingEvents,
    std::uint64_t &totalPendingEvents) noexcept
{
    if (player.events.size() >= maximumEventsPerPlayer || totalPendingEvents >= maximumTotalPendingEvents)
    {
        player.eventQueueOverflowed = true;
        return false;
    }
    try
    {
        player.events.push_back(AnimationPlayerEvent{
            .clip = clip,
            .eventId = marker.eventId,
            .userData = marker.userData,
            .clipTimeSeconds = marker.timeSeconds,
            .direction = direction});
        ++totalPendingEvents;
        return true;
    }
    catch (const std::bad_alloc &)
    {
        player.eventQueueOverflowed = true;
        return false;
    }
}

void collectPlayerEvents(PlayerRecord &player, AnimationClipHandle clipHandle, const ClipRecord &clip,
    double advancedTime, double duration, std::uint32_t maximumEventsPerPlayer,
    std::uint64_t maximumTotalPendingEvents, std::uint64_t &totalPendingEvents) noexcept
{
    const double startTime = player.time;
    if (!player.looping)
    {
        const double endTime = std::clamp(advancedTime, 0.0, duration);
        if (advancedTime > startTime)
        {
            for (const AnimationEventMarker &marker : clip.eventMarkers)
            {
                const double markerTime = marker.timeSeconds;
                if (markerTime > startTime && markerTime <= endTime
                    && !queuePlayerEvent(player, clipHandle, marker, 1, maximumEventsPerPlayer,
                        maximumTotalPendingEvents, totalPendingEvents))
                    return;
            }
        }
        else
        {
            for (auto marker = clip.eventMarkers.rbegin(); marker != clip.eventMarkers.rend(); ++marker)
            {
                const double markerTime = marker->timeSeconds;
                if (markerTime < startTime && markerTime >= endTime
                    && !queuePlayerEvent(player, clipHandle, *marker, -1, maximumEventsPerPlayer,
                        maximumTotalPendingEvents, totalPendingEvents))
                    return;
            }
        }
        return;
    }

    if (advancedTime > startTime)
    {
        double remaining = advancedTime - startTime;
        double localTime = startTime;
        while (remaining > 0.0)
        {
            const double toBoundary = duration - localTime;
            const double segmentLength = std::min(remaining, toBoundary);
            const double segmentEnd = localTime + segmentLength;
            for (const AnimationEventMarker &marker : clip.eventMarkers)
            {
                const double markerTime = marker.timeSeconds;
                if (markerTime > localTime && markerTime <= segmentEnd
                    && !queuePlayerEvent(player, clipHandle, marker, 1, maximumEventsPerPlayer,
                        maximumTotalPendingEvents, totalPendingEvents))
                    return;
            }
            remaining -= segmentLength;
            if (segmentLength < toBoundary)
                break;

            localTime = 0.0;
            for (const AnimationEventMarker &marker : clip.eventMarkers)
            {
                if (marker.timeSeconds != 0.0f)
                    break;
                if (!queuePlayerEvent(player, clipHandle, marker, 1, maximumEventsPerPlayer,
                        maximumTotalPendingEvents, totalPendingEvents))
                    return;
            }
            if (remaining <= 0.0)
                break;
        }
        return;
    }

    if (advancedTime < startTime)
    {
        double remaining = startTime - advancedTime;
        double localTime = startTime;
        bool initialPosition = true;
        while (remaining > 0.0)
        {
            if (localTime == 0.0)
            {
                // The starting position is excluded by reverse-crossing
                // semantics. A marker at duration belongs to the previous
                // cycle and is therefore not fired at the initial instant.
                if (initialPosition)
                {
                    localTime = duration;
                    initialPosition = false;
                    continue;
                }
            }

            const double toBoundary = localTime;
            const double segmentLength = std::min(remaining, toBoundary);
            const double segmentEnd = localTime - segmentLength;
            std::size_t end = clip.eventMarkers.size();
            while (end > 0)
            {
                const float markerTime = clip.eventMarkers[end - 1].timeSeconds;
                std::size_t begin = end - 1;
                while (begin > 0 && clip.eventMarkers[begin - 1].timeSeconds == markerTime)
                    --begin;
                if (static_cast<double>(markerTime) < localTime
                    && static_cast<double>(markerTime) >= segmentEnd)
                {
                    for (std::size_t index = begin; index < end; ++index)
                    {
                        if (!queuePlayerEvent(player, clipHandle, clip.eventMarkers[index], -1,
                                maximumEventsPerPlayer, maximumTotalPendingEvents, totalPendingEvents))
                            return;
                    }
                }
                end = begin;
            }
            remaining -= segmentLength;
            if (segmentLength < toBoundary)
                break;

            // At a crossed loop boundary, duration markers in the previous
            // cycle share the boundary instant with time-zero markers.
            for (auto marker = clip.eventMarkers.rbegin(); marker != clip.eventMarkers.rend(); ++marker)
            {
                if (marker->timeSeconds != duration)
                    break;
                if (!queuePlayerEvent(player, clipHandle, *marker, -1, maximumEventsPerPlayer,
                        maximumTotalPendingEvents, totalPendingEvents))
                    return;
            }
            localTime = duration;
            initialPosition = false;
            if (remaining <= 0.0)
                break;
        }
    }
}

} // namespace engine::skeletal_animation
