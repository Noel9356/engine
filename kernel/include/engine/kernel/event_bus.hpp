#ifndef ENGINE_KERNEL_EVENT_BUS_HPP
#define ENGINE_KERNEL_EVENT_BUS_HPP

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine
{

using SubscriptionToken = std::uint64_t;

inline constexpr SubscriptionToken invalidSubscriptionToken = 0;

// Communication latency accounting for one channel. The bus is the single
// cross-component communication medium (module-module, mod-module, mod-mod
// all dispatch through it; capability calls are direct virtual calls on a
// pointer acquired once), so these numbers ARE the communication latencies.
// Storage is fixed per channel: two counters plus a bounded ring of recent
// samples - no heap growth, no per-publish allocation.
struct ChannelLatencyStatistics
{
    static constexpr std::size_t sampleCapacity = 128;
    // The per-sample ring stores nanoseconds in 32 bits; larger samples clamp.
    static constexpr std::uint64_t maxRecordableSampleNanoseconds = 0xFFFFFFFFull;

    std::uint64_t deliveries = 0;
    std::uint64_t totalNanoseconds = 0;
    std::uint64_t maxNanoseconds = 0;
    std::uint32_t samples[sampleCapacity] = {};
    std::size_t sampleCount = 0;
    std::size_t samplePosition = 0;

    void record(std::uint64_t nanoseconds)
    {
        ++deliveries;
        totalNanoseconds += nanoseconds;
        maxNanoseconds = std::max(maxNanoseconds, nanoseconds);
        samples[samplePosition] =
            static_cast<std::uint32_t>(std::min<std::uint64_t>(nanoseconds, maxRecordableSampleNanoseconds));
        samplePosition = (samplePosition + 1) % sampleCapacity;
        if (sampleCount < sampleCapacity)
        {
            ++sampleCount;
        }
    }
};

// Prettifies a typeid name into a readable channel name: strips the GCC/Clang
// leading length digits, the MSVC 'struct '/'class ' prefix and the common
// 'engine::' namespace so reports stay stable across compilers.
std::string prettifyChannelName(const std::type_index& eventType);

// Type-erased channel base so the bus can unsubscribe through the RAII guard.
class EventBus;

// A single typed event channel.
//
// Dispatch strategy: handlers are stored as cached function pointer + user data
// pairs inside one contiguous vector. Publishing is therefore a linear scan with
// one indirect call per handler: no std::function, no queue, no allocation once
// the handler set is stable. This is the minimal latency path for same-thread
// communication between modules and mods.
//
// Thread policy: the bus is deliberately single threaded (engine main thread).
// Cross thread delivery is a module concern and may be added later as a
// dedicated delivery module without changing this interface.
template <typename EventType>
class EventChannel final
{
public:
    using HandlerFunction = void (*)(void* userData, const EventType& event);

    EventChannel() = default;

    EventChannel(const EventChannel&) = delete;
    EventChannel& operator=(const EventChannel&) = delete;

    // Subscribe a raw handler: a function taking user data plus the event.
    // Used by free function adapters and by tests.
    SubscriptionToken subscribe(HandlerFunction handler, void* userData)
    {
        return subscribeHandler(handler, userData);
    }

    // Subscribe a bound member function. The trampoline is resolved once at
    // registration time; dispatch pays a single indirect call.
    template <typename ListenerClass, void (ListenerClass::*HandlerMethod)(const EventType&)>
    SubscriptionToken subscribe(ListenerClass* listener)
    {
        return subscribeHandler(&EventChannel::dispatchToMember<ListenerClass, HandlerMethod>, static_cast<void*>(listener));
    }

    // Subscribe a free function taking the event by const reference.
    SubscriptionToken subscribe(void (*freeFunction)(const EventType&))
    {
        return subscribeHandler(&EventChannel::dispatchToFree, reinterpret_cast<void*>(freeFunction));
    }

    // Remove a handler. Safe to call from inside a handler that is currently
    // being dispatched; the removal is deferred until dispatch completes.
    bool unsubscribe(SubscriptionToken token)
    {
        if (dispatching)
        {
            for (HandlerEntry& entry : handlers)
            {
                if (entry.token == token && !entry.pendingRemoval)
                {
                    entry.pendingRemoval = true;
                    deferredRemovals.push_back(token);
                    return true;
                }
            }
            return false;
        }

        for (std::size_t index = 0; index < handlers.size(); ++index)
        {
            if (handlers[index].token == token)
            {
                handlers[index] = handlers.back();
                handlers.pop_back();
                return true;
            }
        }
        return false;
    }

    // Invoke every live handler synchronously in subscription order. When
    // latency recording is enabled (see EventBus::setLatencyRecordingEnabled)
    // the dispatch duration (handler time only, clock overhead excluded by
    // taking the second timestamp after the loop) is recorded into the
    // channel's latency statistics. When it is disabled the clocks are never
    // read and the bookkeeping collapses to one branch - the measurement
    // costs nothing on the hot path of a normal run.
    void publish(const EventType& event)
    {
        const bool measure = recordingFlag != nullptr && *recordingFlag;
        const std::chrono::steady_clock::time_point publishStart =
            measure ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
        dispatching = true;
        for (std::size_t index = 0; index < handlers.size(); ++index)
        {
            const HandlerEntry& entry = handlers[index];
            if (!entry.pendingRemoval)
            {
                entry.invoke(entry.userData, event);
            }
        }
        dispatching = false;

        if (!deferredRemovals.empty())
        {
            const std::vector<SubscriptionToken> removals = std::move(deferredRemovals);
            deferredRemovals.clear();
            for (SubscriptionToken token : removals)
            {
                unsubscribe(token);
            }
        }
        if (measure)
        {
            statistics.record(elapsedNanoseconds(publishStart));
        }
    }

    // Wiring for the bus-level recording switch. The flag is owned by the
    // EventBus so one call re-targets every channel.
    void setRecordingFlag(const bool* flag)
    {
        recordingFlag = flag;
    }

    // Communication latency statistics of this channel (module-module,
    // mod-module and mod-mod dispatches all flow through here).
    const ChannelLatencyStatistics& latencyStatistics() const
    {
        return statistics;
    }

private:
    struct HandlerEntry
    {
        void* userData = nullptr;
        HandlerFunction invoke = nullptr;
        SubscriptionToken token = invalidSubscriptionToken;
        bool pendingRemoval = false;
    };

    SubscriptionToken subscribeHandler(HandlerFunction handler, void* userData)
    {
        const SubscriptionToken token = nextToken++;
        handlers.push_back(HandlerEntry{userData, handler, token, false});
        return token;
    }

    template <typename ListenerClass, void (ListenerClass::*HandlerMethod)(const EventType&)>
    static void dispatchToMember(void* userData, const EventType& event)
    {
        (static_cast<ListenerClass*>(userData)->*HandlerMethod)(event);
    }

    static void dispatchToFree(void* userData, const EventType& event)
    {
        void (*freeFunction)(const EventType&) = reinterpret_cast<void (*)(const EventType&)>(userData);
        freeFunction(event);
    }

    std::vector<HandlerEntry> handlers;
    std::vector<SubscriptionToken> deferredRemovals;
    ChannelLatencyStatistics statistics;
    const bool* recordingFlag = nullptr;
    bool dispatching = false;
    SubscriptionToken nextToken = 1;

    static std::uint64_t elapsedNanoseconds(const std::chrono::steady_clock::time_point& start)
    {
        return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                             std::chrono::steady_clock::now() - start)
                                             .count());
    }
};

// The event bus owns one channel per event type. Channels are heap anchored so
// references handed out once stay valid for the lifetime of the bus: modules
// cache a channel reference at construction and publish with zero lookups.
//
// Communication latency model: publishing is a linear scan over one contiguous
// handler vector with one indirect call per handler - no queue, no allocation,
// no lock (the bus is single threaded by contract). That is the minimum
// achievable latency for in-process, same-thread delivery; queued or threaded
// alternatives were evaluated and rejected because they trade away exactly the
// latency this engine prioritises (see docs/architecture.md).
class EventBus final
{
public:
    EventBus() = default;

    EventBus(const EventBus&) = delete;
    EventBus& operator=(const EventBus&) = delete;

    // Communication latency recording is a diagnostic, not a default: a normal
    // run publishes thousands of events per second and its users do not want a
    // report on shutdown. Disabled (the default) the bus never reads a clock
    // for measurement; enabled it records into the per-channel bounded rings
    // that latencyReport() prints. The host enables it from [diagnostics]
    // latency_report or the --latency-report command line flag.
    void setLatencyRecordingEnabled(bool enabled)
    {
        latencyRecording = enabled;
    }

    [[nodiscard]] bool latencyRecordingEnabled() const
    {
        return latencyRecording;
    }

    // Stable reference to the channel for the given event type. The reference
    // remains valid for the lifetime of the bus; cache it in the caller.
    template <typename EventType>
    EventChannel<EventType>& channel()
    {
        const std::type_index eventType = typeid(EventType);
        auto found = channels.find(eventType);
        if (found == channels.end())
        {
            found = channels.emplace(eventType, std::make_unique<ChannelHolder<EventType>>()).first;
            static_cast<ChannelHolder<EventType>*>(found->second.get())->channel.setRecordingFlag(&latencyRecording);
        }
        return static_cast<ChannelHolder<EventType>*>(found->second.get())->channel;
    }

    // Convenience publish with a lookup per call. Hot paths should cache the
    // channel reference and publish through it directly.
    template <typename EventType>
    void publish(const EventType& event)
    {
        channel<EventType>().publish(event);
    }

    // Unsubscribe without knowing the channel type. Used by EventSubscription.
    void unsubscribeThrough(std::type_index eventType, SubscriptionToken token)
    {
        const auto found = channels.find(eventType);
        if (found != channels.end())
        {
            found->second->unsubscribe(token);
        }
    }

    // Human readable communication latency report: one line per active
    // channel with delivery count, mean and the p50/p95/p99/max of the most
    // recent dispatches. Percentiles come from the bounded per-channel sample
    // ring, so they are estimates over recent history, not lifetime exacts.
    std::string latencyReport() const
    {
        std::string report;
        std::vector<const ChannelHolderBase*> ordered;
        ordered.reserve(channels.size());
        for (const auto& entry : channels)
        {
            ordered.push_back(entry.second.get());
        }
        // Deterministic order: by channel name.
        std::sort(ordered.begin(), ordered.end(), [](const ChannelHolderBase* left, const ChannelHolderBase* right)
                  { return left->reportName() < right->reportName(); });
        for (const ChannelHolderBase* channel : ordered)
        {
            channel->appendReport(report);
        }
        return report;
    }

private:
    struct ChannelHolderBase
    {
        virtual ~ChannelHolderBase() = default;
        virtual bool unsubscribe(SubscriptionToken token) = 0;
        virtual const std::string& reportName() const = 0;
        virtual void appendReport(std::string& report) const = 0;
    };

    template <typename EventType>
    struct ChannelHolder final : ChannelHolderBase
    {
        EventChannel<EventType> channel;
        std::string name = prettifyChannelName(typeid(EventType));

        bool unsubscribe(SubscriptionToken token) override
        {
            return channel.unsubscribe(token);
        }

        const std::string& reportName() const override
        {
            return name;
        }

        void appendReport(std::string& report) const override
        {
            const ChannelLatencyStatistics& stats = channel.latencyStatistics();
            if (stats.deliveries == 0)
            {
                return;
            }
            std::vector<std::uint32_t> recent(stats.samples, stats.samples + stats.sampleCount);
            std::sort(recent.begin(), recent.end());
            const auto percentile = [&recent](std::size_t fraction)
            {
                const std::size_t index = std::min(recent.size() - 1, recent.size() * fraction / 100);
                return recent[index];
            };
            const std::uint64_t mean = stats.totalNanoseconds / stats.deliveries;
            report += "  " + name + ": " + std::to_string(stats.deliveries) + " dispatches, mean " +
                      std::to_string(mean) + " ns, p50 " + std::to_string(percentile(50)) + " ns, p95 " +
                      std::to_string(percentile(95)) + " ns, p99 " + std::to_string(percentile(99)) + " ns, max " +
                      std::to_string(stats.maxNanoseconds) + " ns\n";
        }
    };

    std::unordered_map<std::type_index, std::unique_ptr<ChannelHolderBase>> channels;
    bool latencyRecording = false;
};

// RAII guard for one subscription. Releasing the guard unsubscribes the handler,
// which guarantees that a stopped module or an unloaded mod can never observe
// events through a dangling handler.
class EventSubscription final
{
public:
    EventSubscription() = default;

    EventSubscription(EventBus* ownerBus, std::type_index eventType, SubscriptionToken token)
        : ownerBus(ownerBus), eventType(eventType), token(token)
    {
    }

    ~EventSubscription()
    {
        release();
    }

    EventSubscription(const EventSubscription&) = delete;
    EventSubscription& operator=(const EventSubscription&) = delete;

    EventSubscription(EventSubscription&& other) noexcept
        : ownerBus(other.ownerBus), eventType(other.eventType), token(other.token)
    {
        other.ownerBus = nullptr;
        other.token = invalidSubscriptionToken;
    }

    EventSubscription& operator=(EventSubscription&& other) noexcept
    {
        if (this != &other)
        {
            release();
            ownerBus = other.ownerBus;
            eventType = other.eventType;
            token = other.token;
            other.ownerBus = nullptr;
            other.token = invalidSubscriptionToken;
        }
        return *this;
    }

    void release()
    {
        if (ownerBus != nullptr && token != invalidSubscriptionToken)
        {
            ownerBus->unsubscribeThrough(std::type_index(eventType), token);
        }
        ownerBus = nullptr;
        token = invalidSubscriptionToken;
    }

private:
    EventBus* ownerBus = nullptr;
    std::type_index eventType = typeid(void);
    SubscriptionToken token = invalidSubscriptionToken;
};

} // namespace engine

#endif // ENGINE_KERNEL_EVENT_BUS_HPP
