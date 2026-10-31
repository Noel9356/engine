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

using SubscriptionToken = std::uint64_t;

inline constexpr SubscriptionToken invalidSubscriptionToken = 0;

inline constexpr int defaultEventPriority = 0;

struct ChannelLatencyStatistics
{
    static constexpr std::size_t sampleCapacity = 128;

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

std::string prettifyChannelName(const std::type_index &eventType);

class EventBus;

template <typename EventType>
class EventChannel final
{
public:
    using HandlerFunction = void (*)(
        void            *userData,
        const EventType &event
    );

    EventChannel() = default;

    EventChannel(const EventChannel &)            = delete;
    EventChannel &operator=(const EventChannel &) = delete;

    SubscriptionToken subscribe(
        HandlerFunction handler,
        void           *userData
    )
    {
        return subscribeHandler(handler, userData, defaultEventPriority);
    }

    SubscriptionToken subscribe(
        HandlerFunction handler,
        void           *userData,
        int             priority
    )
    {
        return subscribeHandler(handler, userData, priority);
    }

    template <
        typename ListenerClass,
        void (ListenerClass::*HandlerMethod)(const EventType &)>
    SubscriptionToken subscribe(ListenerClass *listener)
    {
        return subscribeHandler(
            &EventChannel::dispatchToMember<ListenerClass, HandlerMethod>,
            static_cast<void *>(listener),
            defaultEventPriority
        );
    }

    template <
        typename ListenerClass,
        void (ListenerClass::*HandlerMethod)(const EventType &)>
    SubscriptionToken subscribe(
        ListenerClass *listener,
        int            priority
    )
    {
        return subscribeHandler(
            &EventChannel::dispatchToMember<ListenerClass, HandlerMethod>, static_cast<void *>(listener), priority
        );
    }

    SubscriptionToken subscribe(void (*freeFunction)(const EventType &))
    {
        return subscribeHandler(
            &EventChannel::dispatchToFree, reinterpret_cast<void *>(freeFunction), defaultEventPriority
        );
    }

    SubscriptionToken subscribe(
        void (*freeFunction)(const EventType &),
        int priority
    )
    {
        return subscribeHandler(&EventChannel::dispatchToFree, reinterpret_cast<void *>(freeFunction), priority);
    }

    bool unsubscribe(SubscriptionToken token)
    {
        if (dispatching)
        {
            for (HandlerEntry &entry : handlers)
            {
                if (entry.token == token && !entry.pendingRemoval)
                {
                    entry.pendingRemoval = true;
                    deferredRemovals.push_back(token);
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
                    entry.invoke(entry.userData, event);
                }
            }
        }
        if (measure)
        {
            statistics.record(elapsedNanoseconds(publishStart));
        }
    }

    void publishDeferred(const EventType &event)
    {
        deferredEvents.push_back(event);
        if (deferredPendingFlag != nullptr)
        {
            deferredPendingFlag->store(true, std::memory_order_relaxed);
        }
    }

    void flushDeferred()
    {
        if (deferredEvents.empty())
        {
            return;
        }
        deferredDrain.swap(deferredEvents);
        for (const EventType &event : deferredDrain)
        {
            publish(event);
        }
        deferredDrain.clear();
    }

    bool hasDeferredEvents() const
    {
        return !deferredEvents.empty();
    }

    std::size_t subscriberCount() const
    {
        return handlers.size();
    }

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

    void setRecordingFlag(const bool *flag)
    {
        recordingFlag = flag;
    }

    void setDeferredPendingFlag(std::atomic<bool> *flag)
    {
        deferredPendingFlag = flag;
    }

    const ChannelLatencyStatistics &latencyStatistics() const
    {
        return statistics;
    }

private:
    struct HandlerEntry
    {
        void             *userData       = nullptr;
        HandlerFunction   invoke         = nullptr;
        SubscriptionToken token          = invalidSubscriptionToken;
        bool              pendingRemoval = false;
        int               priority       = defaultEventPriority;
    };

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

    SubscriptionToken subscribeHandler(
        HandlerFunction handler,
        void           *userData,
        int             priority
    )
    {
        const SubscriptionToken token = nextToken++;
        HandlerEntry            entry{
            .userData = userData, .invoke = handler, .token = token, .pendingRemoval = false, .priority = priority
        };
        if (dispatching)
        {
            pendingSubscriptions.push_back(std::move(entry));
            return token;
        }
        insertHandler(std::move(entry));
        return token;
    }

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

    template <
        typename ListenerClass,
        void (ListenerClass::*HandlerMethod)(const EventType &)>
    static void dispatchToMember(
        void            *userData,
        const EventType &event
    )
    {
        (static_cast<ListenerClass *>(userData)->*HandlerMethod)(event);
    }

    static void dispatchToFree(
        void            *userData,
        const EventType &event
    )
    {
        void (*freeFunction)(const EventType &) = reinterpret_cast<void (*)(const EventType &)>(userData);
        freeFunction(event);
    }

    std::vector<HandlerEntry> handlers;

    std::vector<HandlerEntry>      pendingSubscriptions;
    std::vector<SubscriptionToken> deferredRemovals;
    ChannelLatencyStatistics       statistics;
    const bool                    *recordingFlag = nullptr;

    std::vector<EventType> deferredEvents;
    std::vector<EventType> deferredDrain;
    std::atomic<bool>     *deferredPendingFlag = nullptr;
    bool                   dispatching         = false;
    SubscriptionToken      nextToken           = 1;

    static std::uint64_t elapsedNanoseconds(const std::chrono::steady_clock::time_point &start)
    {
        return static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count()
        );
    }
};

class WorkerQueueBase
{
public:
    virtual ~WorkerQueueBase() = default;

    virtual void flush() = 0;

    virtual void closeQueue()
    {}
};

template <typename EventType>
class WorkerEventQueue final : public WorkerQueueBase
{
public:
    WorkerEventQueue(
        EventChannel<EventType> &targetChannel,
        std::atomic<bool>       *busDeferredPendingFlag
    )
        : channel(targetChannel),
          deferredPendingFlag(busDeferredPendingFlag)
    {}

    WorkerEventQueue(const WorkerEventQueue &)            = delete;
    WorkerEventQueue &operator=(const WorkerEventQueue &) = delete;

    void enqueue(EventType event)
    {
        if (closed.load(std::memory_order_acquire))
        {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            if (closed.load(std::memory_order_relaxed))
            {
                return;
            }
            pendingEvents.push_back(std::move(event));
            pendingCount.fetch_add(1, std::memory_order_relaxed);
            if (deferredPendingFlag != nullptr)
            {
                deferredPendingFlag->store(true, std::memory_order_relaxed);
            }
        }
    }

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

    std::atomic<std::size_t> pendingCount{0};

    std::vector<EventType> drainBuffer;
};

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

    [[nodiscard]] bool latencyRecordingEnabled() const
    {
        return latencyRecording;
    }

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

            channelOrder.push_back(holder);
        }
        return static_cast<ChannelHolder<EventType> *>(found->second.get())->channel;
    }

    template <typename EventType>
    void publish(const EventType &event)
    {
        channel<EventType>().publish(event);
    }

    template <typename EventType>
    void publishDeferred(const EventType &event)
    {
        channel<EventType>().publishDeferred(event);
    }

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

    template <typename EventType>
    [[nodiscard]] std::shared_ptr<WorkerEventQueue<EventType>> createWorkerQueue()
    {
        auto queue = std::make_shared<WorkerEventQueue<EventType>>(channel<EventType>(), &deferredPending);
        workerQueues.push_back(queue);
        return queue;
    }

    void unsubscribeThrough(
        std::type_index   eventType,
        SubscriptionToken token
    )
    {
        const auto found = channels.find(eventType);
        if (found != channels.end())
        {
            found->second->unsubscribe(token);
        }
    }

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

        std::sort(ordered.begin(), ordered.end(), [](const ChannelHolderBase *left, const ChannelHolderBase *right) {
            return left->reportName() < right->reportName();
        });
        report += "event subscriptions (" + std::to_string(ordered.size()) + " channel(s) with handlers):\n";
        for (const ChannelHolderBase *channelHolder : ordered)
        {
            channelHolder->appendSubscriptionLine(report);
        }
        return report;
    }

    std::string latencyReport() const
    {
        std::string                            report;
        std::vector<const ChannelHolderBase *> ordered;
        ordered.reserve(channels.size());
        for (const auto &entry : channels)
        {
            ordered.push_back(entry.second.get());
        }

        std::sort(ordered.begin(), ordered.end(), [](const ChannelHolderBase *left, const ChannelHolderBase *right) {
            return left->reportName() < right->reportName();
        });
        for (const ChannelHolderBase *channel : ordered)
        {
            channel->appendReport(report);
        }
        return report;
    }

private:
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
            const auto percentile = [&recent](std::size_t fraction) {
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
            } else
            {
                report += " (priorities " + std::to_string(channel.lowestPriority()) + ".."
                        + std::to_string(channel.highestPriority()) + ")\n";
            }
        }
    };

    std::unordered_map<std::type_index, std::unique_ptr<ChannelHolderBase>> channels;

    std::vector<ChannelHolderBase *> channelOrder;

    std::vector<std::shared_ptr<WorkerQueueBase>> workerQueues;

    std::atomic<bool> deferredPending{false};
    bool              latencyRecording = false;
};

class EventSubscription final
{
public:
    EventSubscription() = default;

    EventSubscription(
        EventBus         *ownerBus,
        std::type_index   eventType,
        SubscriptionToken token
    )
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
