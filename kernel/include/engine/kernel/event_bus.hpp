// Typed publish/subscribe event bus.
//
// Each event type gets one EventChannel (created on demand, keyed by
// type_index). Handlers run in priority order during publish(); deferred
// events queue per channel and are drained by flushDeferredEvents,
// usually once per frame on the main thread. Optional per-channel latency
// statistics feed the diagnostics report.

#ifndef ENGINE_KERNEL_EVENT_BUS_HPP
#define ENGINE_KERNEL_EVENT_BUS_HPP

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine
{

// Unique per channel; 0 is the invalid token.
using SubscriptionToken = std::uint64_t;

inline constexpr SubscriptionToken invalidSubscriptionToken = 0;

// Lower values run first; equal priorities keep subscription order.
inline constexpr int defaultEventPriority = 0;

// Rolling per-channel dispatch latency samples (nanoseconds) plus totals.
// Not synchronized: only touched by the publishing thread.
struct ChannelLatencyStatistics
{
    // Ring buffer size for the percentile report (p50/p95/p99 over the last
    // 128 dispatches).
    static constexpr std::size_t sampleCapacity = 128;

    // Samples clamp here (~4.29 s) to fit the uint32 ring buffer.
    static constexpr std::uint64_t maxRecordableSampleNanoseconds = 0xFFFFFFFFull;

    std::uint64_t deliveries              = 0;
    std::uint64_t totalNanoseconds        = 0;
    std::uint64_t maxNanoseconds          = 0;
    std::uint32_t samples[sampleCapacity] = {};
    std::size_t   sampleCount             = 0;
    std::size_t   samplePosition          = 0;

    void record(std::uint64_t nanoseconds)
    {
        ++deliveries;
        totalNanoseconds += nanoseconds;
        maxNanoseconds = std::max(maxNanoseconds, nanoseconds);
        samples[samplePosition]
            = static_cast<std::uint32_t>(std::min<std::uint64_t>(nanoseconds, maxRecordableSampleNanoseconds));
        samplePosition = (samplePosition + 1) % sampleCapacity;
        if (sampleCount < sampleCapacity)
        {
            ++sampleCount;
        }
    }
};

// Strips compiler-specific prefixes/namespaces for report output
// (defined in participant_context.cpp).
std::string prettifyChannelName(const std::type_index &eventType);

class EventBus;

// One event type's handler list plus its deferred queue. Not internally
// synchronized: publish/subscribe belong to one thread (the main thread
// by convention); only publishDeferred is safe from other threads.
template <typename EventType>
class EventChannel final
{
public:
    // C-style handler plus a separately typed free-function form. Keeping
    // function pointers typed avoids object-pointer conversions.
    using HandlerFunction = void (*)(void *userData, const EventType &event);
    using FreeFunction    = void (*)(const EventType &event);

    EventChannel() = default;

    EventChannel(const EventChannel &)            = delete;
    EventChannel &operator=(const EventChannel &) = delete;

    SubscriptionToken subscribe(HandlerFunction handler, void *userData)
    {
        return subscribeHandler(handler, userData, nullptr, defaultEventPriority);
    }

    SubscriptionToken subscribe(HandlerFunction handler, void *userData, int priority)
    {
        return subscribeHandler(handler, userData, nullptr, priority);
    }

    template <typename ListenerClass, void (ListenerClass::*HandlerMethod)(const EventType &)>
    SubscriptionToken subscribe(ListenerClass *listener)
    {
        if (listener == nullptr)
            return invalidSubscriptionToken;
        return subscribeHandler(&EventChannel::dispatchToMember<ListenerClass, HandlerMethod>,
            static_cast<void *>(listener), nullptr, defaultEventPriority);
    }

    template <typename ListenerClass, void (ListenerClass::*HandlerMethod)(const EventType &)>
    SubscriptionToken subscribe(ListenerClass *listener, int priority)
    {
        if (listener == nullptr)
            return invalidSubscriptionToken;
        return subscribeHandler(
            &EventChannel::dispatchToMember<ListenerClass, HandlerMethod>, static_cast<void *>(listener), nullptr, priority);
    }

    SubscriptionToken subscribe(FreeFunction freeFunction)
    {
        return subscribeHandler(nullptr, nullptr, freeFunction, defaultEventPriority);
    }

    SubscriptionToken subscribe(FreeFunction freeFunction, int priority)
    {
        return subscribeHandler(nullptr, nullptr, freeFunction, priority);
    }

    // Safe during dispatch: the entry is flagged and removed once the current
    // publish completes (or pulled from pendingSubscriptions).
    bool unsubscribe(SubscriptionToken token)
    {
        if (dispatching)
        {
            for (HandlerEntry &entry : handlers)
            {
                if (entry.token == token && !entry.pendingRemoval)
                {
                    deferredRemovals.push_back(token);
                    entry.pendingRemoval = true;
                    return true;
                }
            }

            for (std::size_t index = 0; index < pendingSubscriptions.size(); ++index)
            {
                if (pendingSubscriptions[index].token == token)
                {
                    pendingSubscriptions.erase(pendingSubscriptions.begin() + static_cast<std::ptrdiff_t>(index));
                    return true;
                }
            }
            return false;
        }

        for (std::size_t index = 0; index < handlers.size(); ++index)
        {
            if (handlers[index].token == token)
            {
                handlers.erase(handlers.begin() + static_cast<std::ptrdiff_t>(index));
                return true;
            }
        }
        return false;
    }

    // Invokes handlers synchronously in priority order; re-entrant publishes
    // are allowed. DispatchGuard applies queued (un)subscriptions afterwards.
    void publish(const EventType &event)
    {
        const bool                                  measure = recordingFlag != nullptr && *recordingFlag;
        const std::chrono::steady_clock::time_point publishStart
            = measure ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
        {
            DispatchGuard guard(*this);
            for (std::size_t index = 0; index < handlers.size(); ++index)
            {
                const HandlerEntry &entry = handlers[index];

                if (!entry.pendingRemoval) [[likely]]
                {
                    if (entry.freeFunction != nullptr)
                        entry.freeFunction(event);
                    else
                        entry.invoke(entry.userData, event);
                }
            }
        }
        if (measure)
        {
            statistics.record(elapsedNanoseconds(publishStart));
        }
    }

    // Thread-safe enqueue onto this channel's deferred queue; delivery happens
    // in flushDeferred() (main thread). Returns false when the bus deferred-
    // event capacity is set and full (the drop is counted).
    bool publishDeferred(const EventType &event)
    {
        return enqueueDeferred(event);
    }

    bool publishDeferred(EventType &&event)
    {
        return enqueueDeferred(std::move(event));
    }

    // Swaps the queue under the lock, then publishes outside it so handlers
    // may enqueue deferred events again without deadlock.
    void flushDeferred()
    {
        {
            std::lock_guard<std::mutex> lock(deferredMutex);
            if (deferredEvents.empty())
            {
                return;
            }
            deferredDrain.swap(deferredEvents);
        }
        for (const EventType &event : deferredDrain)
        {
            publish(event);
        }
        deferredDrain.clear();
    }

    bool hasDeferredEvents() const
    {
        std::lock_guard<std::mutex> lock(deferredMutex);
        return !deferredEvents.empty();
    }

    // Total events rejected by the deferred-queue capacity (0 = unlimited).
    [[nodiscard]] std::size_t droppedDeferredEventCount() const
    {
        return droppedDeferredEvents.load(std::memory_order_relaxed);
    }

    std::size_t subscriberCount() const
    {
        return handlers.size();
    }

    // True when every handler uses the default priority (report shows
    // "subscription order").
    bool defaultPrioritySpan() const
    {
        for (const HandlerEntry &entry : handlers)
        {
            if (entry.priority != defaultEventPriority)
            {
                return false;
            }
        }
        return true;
    }

    int lowestPriority() const
    {
        int lowest = defaultEventPriority;
        for (const HandlerEntry &entry : handlers)
        {
            lowest = std::min(lowest, entry.priority);
        }
        return lowest;
    }

    int highestPriority() const
    {
        int highest = defaultEventPriority;
        for (const HandlerEntry &entry : handlers)
        {
            highest = std::max(highest, entry.priority);
        }
        return highest;
    }

    // Bus-wired switches injected at channel creation; null = disabled.
    void setRecordingFlag(const bool *flag)
    {
        recordingFlag = flag;
    }

    void setDeferredPendingFlag(std::atomic<bool> *flag)
    {
        deferredPendingFlag = flag;
    }

    void setDeferredEventCapacity(const std::atomic<std::size_t> *capacity)
    {
        deferredEventCapacity = capacity;
    }

    const ChannelLatencyStatistics &latencyStatistics() const
    {
        return statistics;
    }

private:
    // Locks deferredMutex; a capacity of 0 (or null pointer) means unbounded.
    template <typename EventValue>
    bool enqueueDeferred(EventValue &&event)
    {
        {
            std::lock_guard<std::mutex> lock(deferredMutex);
            const std::size_t           capacity
                = deferredEventCapacity != nullptr ? deferredEventCapacity->load(std::memory_order_relaxed) : 0;
            if (capacity != 0 && deferredEvents.size() >= capacity)
            {
                droppedDeferredEvents.fetch_add(1, std::memory_order_relaxed);
                return false;
            }
            deferredEvents.emplace_back(std::forward<EventValue>(event));
        }
        if (deferredPendingFlag != nullptr)
        {
            deferredPendingFlag->store(true, std::memory_order_relaxed);
        }
        return true;
    }

    // pendingRemoval: unsubscribed mid-dispatch; skipped until purged.
    struct HandlerEntry
    {
        void             *userData       = nullptr;
        HandlerFunction   invoke         = nullptr;
        FreeFunction      freeFunction   = nullptr;
        SubscriptionToken token          = invalidSubscriptionToken;
        bool              pendingRemoval = false;
        int               priority       = defaultEventPriority;
    };

    // Marks the channel as dispatching; the outermost destructor applies the
    // subscriptions/unsubscriptions queued while handlers ran.
    class DispatchGuard final
    {
    public:
        explicit DispatchGuard(EventChannel &ownerChannel)
            : channel(ownerChannel),
              wasDispatching(ownerChannel.dispatching)
        {
            channel.dispatching = true;
        }

        DispatchGuard(const DispatchGuard &)            = delete;
        DispatchGuard &operator=(const DispatchGuard &) = delete;

        ~DispatchGuard()
        {
            channel.dispatching = wasDispatching;
            if (wasDispatching)
            {
                return;
            }

            if (!channel.pendingSubscriptions.empty())
            {
                const std::size_t count = channel.pendingSubscriptions.size();
                for (std::size_t index = 0; index < count; ++index)
                {
                    channel.insertHandler(std::move(channel.pendingSubscriptions[index]));
                }
                channel.pendingSubscriptions.clear();
            }
            if (!channel.deferredRemovals.empty())
            {
                const std::size_t count = channel.deferredRemovals.size();
                for (std::size_t index = 0; index < count; ++index)
                {
                    channel.unsubscribe(channel.deferredRemovals[index]);
                }
                channel.deferredRemovals.clear();
            }
        }

    private:
        EventChannel &channel;
        bool          wasDispatching;
    };

    // Tokens start at 1 so 0 can flag "no subscription"; entries made while
    // dispatching wait in pendingSubscriptions.
    SubscriptionToken subscribeHandler(
        HandlerFunction handler, void *userData, FreeFunction freeFunction, int priority)
    {
        if ((handler == nullptr) == (freeFunction == nullptr) || nextToken == invalidSubscriptionToken)
            return invalidSubscriptionToken;

        const SubscriptionToken token = nextToken++;
        HandlerEntry            entry{.userData       = userData,
            .invoke                              = handler,
            .freeFunction                        = freeFunction,
            .token                                = token,
            .pendingRemoval                       = false,
            .priority                             = priority};
        if (dispatching)
        {
            handlers.reserve(handlers.size() + pendingSubscriptions.size() + 1u);
            pendingSubscriptions.push_back(std::move(entry));
            return token;
        }
        insertHandler(std::move(entry));
        return token;
    }

    // Stable insertion by ascending priority; equal priorities keep
    // subscription order.
    void insertHandler(HandlerEntry &&entry)
    {
        std::size_t insertPosition = handlers.size();
        for (std::size_t index = 0; index < handlers.size(); ++index)
        {
            if (handlers[index].priority < entry.priority)
            {
                insertPosition = index;
                break;
            }
        }
        handlers.push_back(std::move(entry));
        std::rotate(handlers.begin() + static_cast<std::ptrdiff_t>(insertPosition), handlers.end() - 1, handlers.end());
    }

    template <typename ListenerClass, void (ListenerClass::*HandlerMethod)(const EventType &)>
    static void dispatchToMember(void *userData, const EventType &event)
    {
        (static_cast<ListenerClass *>(userData)->*HandlerMethod)(event);
    }

    // Live handler list; pendingSubscriptions/deferredRemovals hold changes
    // made while dispatching.
    std::vector<HandlerEntry> handlers;

    std::vector<HandlerEntry>      pendingSubscriptions;
    std::vector<SubscriptionToken> deferredRemovals;
    ChannelLatencyStatistics       statistics;
    const bool                    *recordingFlag = nullptr;

    // deferredEvents is the shared queue (producers + main thread);
    // deferredDrain is the main thread's swap buffer.
    mutable std::mutex              deferredMutex;
    std::vector<EventType>          deferredEvents;
    std::vector<EventType>          deferredDrain;
    std::atomic<std::size_t>        droppedDeferredEvents{0};
    const std::atomic<std::size_t> *deferredEventCapacity = nullptr;
    std::atomic<bool>              *deferredPendingFlag   = nullptr;
    bool                            dispatching           = false;
    SubscriptionToken               nextToken             = 1;

    static std::uint64_t elapsedNanoseconds(const std::chrono::steady_clock::time_point &start)
    {
        return static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count());
    }
};

// Type-erased worker queue so the bus can flush/close worker-thread queues
// of any event type.
class WorkerQueueBase
{
public:
    virtual ~WorkerQueueBase() = default;

    virtual void flush() = 0;

    virtual void closeQueue()
    {}
};

// maxPendingEvents = 0 means unbounded.
struct WorkerEventQueueOptions
{
    std::size_t maxPendingEvents = 0;
};

// Bounded queue for worker threads that must not publish directly:
// enqueue() is thread-safe, flush() (main thread, via the bus) publishes
// the batch to the channel. enqueue rejects when closed or full.
template <typename EventType>
class WorkerEventQueue final : public WorkerQueueBase
{
public:
    WorkerEventQueue(EventChannel<EventType> &targetChannel, std::atomic<bool> *busDeferredPendingFlag)
        : WorkerEventQueue(targetChannel, busDeferredPendingFlag, WorkerEventQueueOptions{})
    {}

    WorkerEventQueue(EventChannel<EventType> &targetChannel, std::atomic<bool> *busDeferredPendingFlag,
        WorkerEventQueueOptions queueOptions)
        : channel(targetChannel),
          deferredPendingFlag(busDeferredPendingFlag),
          options(queueOptions)
    {}

    WorkerEventQueue(const WorkerEventQueue &)            = delete;
    WorkerEventQueue &operator=(const WorkerEventQueue &) = delete;

    // Close is checked twice: a cheap acquire test first, then definitively
    // under the mutex so a concurrent closeQueue cannot lose events.
    bool enqueue(EventType event)
    {
        if (closed.load(std::memory_order_acquire))
        {
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            if (closed.load(std::memory_order_relaxed)
                || (options.maxPendingEvents != 0 && pendingEvents.size() >= options.maxPendingEvents))
            {
                rejectedEvents.fetch_add(1, std::memory_order_relaxed);
                return false;
            }
            pendingEvents.push_back(std::move(event));
            pendingCount.fetch_add(1, std::memory_order_relaxed);
            if (deferredPendingFlag != nullptr)
            {
                deferredPendingFlag->store(true, std::memory_order_relaxed);
            }
        }
        return true;
    }

    [[nodiscard]] std::size_t pendingEventCount() const
    {
        return pendingCount.load(std::memory_order_relaxed);
    }

    [[nodiscard]] std::size_t rejectedEventCount() const
    {
        return rejectedEvents.load(std::memory_order_relaxed);
    }

    [[nodiscard]] std::size_t capacity() const
    {
        return options.maxPendingEvents;
    }

    // Swaps pending under the lock and publishes unlocked so handlers can
    // enqueue again; events enqueued mid-flush wait for the next flush.
    void flush() override
    {
        if (pendingCount.load(std::memory_order_relaxed) == 0)
        {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(queueMutex);
            if (pendingEvents.empty())
            {
                return;
            }
            drainBuffer.swap(pendingEvents);
            pendingCount.store(0, std::memory_order_relaxed);
        }
        if (closed.load(std::memory_order_acquire))
        {
            drainBuffer.clear();
            return;
        }
        for (const EventType &event : drainBuffer)
        {
            channel.publish(event);
        }
        drainBuffer.clear();
    }

    // Terminal: rejects all future enqueues and discards pending events
    // (used when the kernel releases its channels).
    void closeQueue() override
    {
        closed.store(true, std::memory_order_release);
        std::lock_guard<std::mutex> lock(queueMutex);
        pendingEvents.clear();
        pendingCount.store(0, std::memory_order_relaxed);
    }

private:
    EventChannel<EventType> &channel;
    std::atomic<bool>       *deferredPendingFlag = nullptr;

    std::atomic<bool>      closed{false};
    std::mutex             queueMutex;
    std::vector<EventType> pendingEvents;

    WorkerEventQueueOptions  options{};
    std::atomic<std::size_t> pendingCount{0};
    std::atomic<std::size_t> rejectedEvents{0};

    std::vector<EventType> drainBuffer;
};

// Owns the per-type channels and worker queues. Channels live until
// releaseChannels(); channel references must not outlive the bus.
class EventBus final
{
public:
    EventBus() = default;

    EventBus(const EventBus &)            = delete;
    EventBus &operator=(const EventBus &) = delete;

    void setLatencyRecordingEnabled(bool enabled)
    {
        latencyRecording = enabled;
    }

    // Maximum queued deferred events per channel; 0 disables the limit.
    void setDeferredEventCapacity(std::size_t maximumPendingPerChannel)
    {
        deferredEventCapacity.store(maximumPendingPerChannel, std::memory_order_relaxed);
    }

    [[nodiscard]] std::size_t deferredEventCapacityLimit() const
    {
        return deferredEventCapacity.load(std::memory_order_relaxed);
    }

    [[nodiscard]] bool latencyRecordingEnabled() const
    {
        return latencyRecording;
    }

    // Creates the channel for EventType on first use and wires the bus-wide
    // flags into it.
    template <typename EventType>
    EventChannel<EventType> &channel()
    {
        const std::type_index eventType = typeid(EventType);
        auto                  found     = channels.find(eventType);
        if (found == channels.end())
        {
            found = channels.emplace(eventType, std::make_unique<ChannelHolder<EventType>>()).first;
            ChannelHolder<EventType> *holder = static_cast<ChannelHolder<EventType> *>(found->second.get());
            holder->channel.setRecordingFlag(&latencyRecording);
            holder->channel.setDeferredPendingFlag(&deferredPending);
            holder->channel.setDeferredEventCapacity(&deferredEventCapacity);

            channelOrder.push_back(holder);
        }
        return static_cast<ChannelHolder<EventType> *>(found->second.get())->channel;
    }

    // Returns nullptr when no channel for EventType has been created yet
    // (does not create one). Useful for callers that want to publish only when
    // a subscriber already exists.
    template <typename EventType>
    EventChannel<EventType> *tryChannel() noexcept
    {
        const auto found = channels.find(typeid(EventType));
        if (found == channels.end())
        {
            return nullptr;
        }
        return static_cast<ChannelHolder<EventType> *>(found->second.get())->channelPtr();
    }

    // True when a channel for EventType has been created (i.e. someone has
    // already subscribed or published).
    template <typename EventType>
    bool hasChannel() const noexcept
    {
        return channels.find(typeid(EventType)) != channels.end();
    }

    // Number of channels created so far (one per event type ever published or
    // subscribed). Lives outside the per-channel subscriber count.
    [[nodiscard]] std::size_t channelCount() const noexcept
    {
        return channels.size();
    }

    // Lock-free "any deferred events pending across every channel" check;
    // cleared by flushDeferredEvents. Cheaper than hasDeferredEvents() on
    // each channel when the bus has many channels and a typical frame has
    // no deferred traffic.
    [[nodiscard]] bool deferredEventPending() const noexcept
    {
        return deferredPending.load(std::memory_order_relaxed);
    }

    template <typename EventType>
    void publish(const EventType &event)
    {
        channel<EventType>().publish(event);
    }

    // Thread-safe: enqueues on the event's channel; delivered by
    // flushDeferredEvents.
    template <typename EventValue>
    bool publishDeferred(EventValue &&event)
    {
        using EventType = std::remove_cvref_t<EventValue>;
        return channel<EventType>().publishDeferred(std::forward<EventValue>(event));
    }

    // Main-thread drain point: flushes every channel's deferred queue and
    // every worker queue. deferredPending makes the common empty case one
    // relaxed atomic load.
    void flushDeferredEvents()
    {
        if (!deferredPending.load(std::memory_order_relaxed))
        {
            return;
        }
        deferredPending.store(false, std::memory_order_relaxed);
        for (ChannelHolderBase *channelHolder : channelOrder)
        {
            channelHolder->flushDeferredChannel();
        }
        for (const std::shared_ptr<WorkerQueueBase> &workerQueue : workerQueues)
        {
            workerQueue->flush();
        }
    }

    // The bus keeps a shared reference for flushing and closes the queue in
    // releaseChannels(); deliver only via flush (main thread).
    template <typename EventType>
    [[nodiscard]] std::shared_ptr<WorkerEventQueue<EventType>> createWorkerQueue()
    {
        return createWorkerQueue<EventType>(WorkerEventQueueOptions{});
    }

    template <typename EventType>
    [[nodiscard]] std::shared_ptr<WorkerEventQueue<EventType>> createWorkerQueue(WorkerEventQueueOptions queueOptions)
    {
        auto queue
            = std::make_shared<WorkerEventQueue<EventType>>(channel<EventType>(), &deferredPending, queueOptions);
        workerQueues.push_back(queue);
        return queue;
    }

    // Type-erased unsubscribe used by EventSubscription without needing the
    // channel template.
    void unsubscribeThrough(std::type_index eventType, SubscriptionToken token)
    {
        const auto found = channels.find(eventType);
        if (found != channels.end())
        {
            found->second->unsubscribe(token);
        }
    }

    // Shutdown: closes worker queues (dropping pending events), then destroys
    // all channels; handlers must not be invoked afterwards.
    void releaseChannels()
    {
        for (const std::shared_ptr<WorkerQueueBase> &workerQueue : workerQueues)
        {
            workerQueue->closeQueue();
        }
        channels.clear();
        channelOrder.clear();
        workerQueues.clear();
        deferredPending.store(false, std::memory_order_relaxed);
    }

    // Diagnostics: channels with active handlers, sorted by name.
    std::string subscriptionReport() const
    {
        std::string                            report;
        std::vector<const ChannelHolderBase *> ordered;
        ordered.reserve(channels.size());
        for (const auto &entry : channels)
        {
            if (entry.second->subscriberCount() > 0)
            {
                ordered.push_back(entry.second.get());
            }
        }
        if (ordered.empty())
        {
            return report;
        }

        std::sort(ordered.begin(),
            ordered.end(),
            [](const ChannelHolderBase *left, const ChannelHolderBase *right)
            { return left->reportName() < right->reportName(); });
        report += "event subscriptions (" + std::to_string(ordered.size()) + " channel(s) with handlers):\n";
        for (const ChannelHolderBase *channelHolder : ordered)
        {
            channelHolder->appendSubscriptionLine(report);
        }
        return report;
    }

    // Diagnostics: per-channel dispatch counts and latency percentiles
    // (populated only while latency recording is enabled).
    std::string latencyReport() const
    {
        std::string                            report;
        std::vector<const ChannelHolderBase *> ordered;
        ordered.reserve(channels.size());
        for (const auto &entry : channels)
        {
            ordered.push_back(entry.second.get());
        }

        std::sort(ordered.begin(),
            ordered.end(),
            [](const ChannelHolderBase *left, const ChannelHolderBase *right)
            { return left->reportName() < right->reportName(); });
        for (const ChannelHolderBase *channel : ordered)
        {
            channel->appendReport(report);
        }
        return report;
    }

private:
    // Type-erasure bridge so EventBus can drive channels without templates.
    struct ChannelHolderBase
    {
        virtual ~ChannelHolderBase()                                       = default;
        virtual bool               unsubscribe(SubscriptionToken token)    = 0;
        virtual const std::string &reportName() const                      = 0;
        virtual void               appendReport(std::string &report) const = 0;

        virtual void flushDeferredChannel() = 0;

        virtual std::size_t subscriberCount() const = 0;

        virtual void appendSubscriptionLine(std::string &report) const = 0;
    };

    template <typename EventType>
    struct ChannelHolder final : ChannelHolderBase
    {
        EventChannel<EventType> channel;
        std::string             name = prettifyChannelName(typeid(EventType));

        EventChannel<EventType> *channelPtr() noexcept
        {
            return &channel;
        }

        bool unsubscribe(SubscriptionToken token) override
        {
            return channel.unsubscribe(token);
        }

        const std::string &reportName() const override
        {
            return name;
        }

        void appendReport(std::string &report) const override
        {
            const ChannelLatencyStatistics &stats = channel.latencyStatistics();
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
            report += "  " + name + ": " + std::to_string(stats.deliveries) + " dispatches, mean "
                    + std::to_string(mean) + " ns, p50 " + std::to_string(percentile(50)) + " ns, p95 "
                    + std::to_string(percentile(95)) + " ns, p99 " + std::to_string(percentile(99)) + " ns, max "
                    + std::to_string(stats.maxNanoseconds) + " ns\n";
        }

        void flushDeferredChannel() override
        {
            channel.flushDeferred();
        }

        std::size_t subscriberCount() const override
        {
            return channel.subscriberCount();
        }

        void appendSubscriptionLine(std::string &report) const override
        {
            report += "  " + name + ": " + std::to_string(channel.subscriberCount()) + " handler(s)";
            if (channel.defaultPrioritySpan())
            {
                report += " (subscription order)\n";
            }
            else
            {
                report += " (priorities " + std::to_string(channel.lowestPriority()) + ".."
                        + std::to_string(channel.highestPriority()) + ")\n";
            }
        }
    };

    // channelOrder preserves creation order for deterministic flushes;
    // channels is the type_index lookup.
    std::unordered_map<std::type_index, std::unique_ptr<ChannelHolderBase>> channels;

    std::vector<ChannelHolderBase *> channelOrder;

    std::vector<std::shared_ptr<WorkerQueueBase>> workerQueues;

    // Set by any enqueue (deferred or worker); cleared by flushDeferredEvents.
    std::atomic<bool>        deferredPending{false};
    std::atomic<std::size_t> deferredEventCapacity{0};
    bool                     latencyRecording = false;
};

// RAII subscription handle: unsubscribes through the bus on destruction.
// Moveable, not copyable.
class EventSubscription final
{
public:
    EventSubscription() = default;

    EventSubscription(EventBus *ownerBus, std::type_index eventType, SubscriptionToken token)
        : ownerBus(ownerBus),
          eventType(eventType),
          token(token)
    {}

    ~EventSubscription()
    {
        release();
    }

    EventSubscription(const EventSubscription &)            = delete;
    EventSubscription &operator=(const EventSubscription &) = delete;

    EventSubscription(EventSubscription &&other) noexcept
        : ownerBus(other.ownerBus),
          eventType(other.eventType),
          token(other.token)
    {
        other.ownerBus = nullptr;
        other.token    = invalidSubscriptionToken;
    }

    EventSubscription &operator=(EventSubscription &&other) noexcept
    {
        if (this != &other)
        {
            release();
            ownerBus       = other.ownerBus;
            eventType      = other.eventType;
            token          = other.token;
            other.ownerBus = nullptr;
            other.token    = invalidSubscriptionToken;
        }
        return *this;
    }

    // Idempotent; safe on default-constructed or already-moved handles.
    void release()
    {
        if (ownerBus != nullptr && token != invalidSubscriptionToken)
        {
            ownerBus->unsubscribeThrough(std::type_index(eventType), token);
        }
        ownerBus = nullptr;
        token    = invalidSubscriptionToken;
    }

private:
    EventBus         *ownerBus  = nullptr;
    std::type_index   eventType = typeid(void);
    SubscriptionToken token     = invalidSubscriptionToken;
};

} // namespace engine

#endif
