// Deferred/recurring callbacks drained on the main thread.
//
// scheduleAfter/scheduleEvery return RAII ScheduledTask handles;
// callbacks run only inside tick(), which the application calls once per
// frame. Recurring tasks reschedule by their interval, either skipping
// missed ticks or catching up one per tick (RecurringTaskPolicy).

#ifndef ENGINE_KERNEL_MAIN_THREAD_TASK_QUEUE_HPP
#define ENGINE_KERNEL_MAIN_THREAD_TASK_QUEUE_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace engine
{

class MainThreadTaskQueue;

// SkipMissedIntervals: a lagging task jumps to the next unpassed multiple
// of its interval. CatchUpOnePerTick: it fires at most once per tick
// until it catches up.
enum class RecurringTaskPolicy
{
    SkipMissedIntervals,
    CatchUpOnePerTick
};

// maxTasks/maxCallbacksPerTick = 0 mean unbounded.
struct MainThreadTaskQueueOptions
{
    std::size_t maxTasks = 0;

    std::size_t maxCallbacksPerTick = 0;

    RecurringTaskPolicy recurringTaskPolicy = RecurringTaskPolicy::SkipMissedIntervals;
};

// RAII handle; destruction cancels the task. valid() reports whether it
// is still armed. Not copyable; move resets the source.
class ScheduledTask final
{
public:
    ScheduledTask() = default;

    ScheduledTask(MainThreadTaskQueue *ownerScheduler, std::shared_ptr<bool> taskArmed)
        : owner(ownerScheduler),
          armed(std::move(taskArmed))
    {}

    ~ScheduledTask()
    {
        cancel();
    }

    ScheduledTask(const ScheduledTask &)            = delete;
    ScheduledTask &operator=(const ScheduledTask &) = delete;

    ScheduledTask(ScheduledTask &&other) noexcept
        : owner(other.owner),
          armed(std::move(other.armed)),
          lifetime(std::move(other.lifetime)),
          lifetimeGuarded(other.lifetimeGuarded)
    {
        other.owner           = nullptr;
        other.lifetimeGuarded = false;
    }

    ScheduledTask &operator=(ScheduledTask &&other) noexcept
    {
        if (this != &other)
        {
            cancel();
            owner                 = other.owner;
            armed                 = std::move(other.armed);
            lifetime              = std::move(other.lifetime);
            lifetimeGuarded       = other.lifetimeGuarded;
            other.owner           = nullptr;
            other.lifetimeGuarded = false;
        }
        return *this;
    }

    // Defined below MainThreadTaskQueue (needs removeTask). Safe on moved-
    // from and default handles; detaches from the owner so the queue's own
    // destruction cannot double-cancel.
    void cancel();

    [[nodiscard]] bool valid() const
    {
        return armed != nullptr && *armed;
    }

private:
    friend class MainThreadTaskQueue;

    ScheduledTask(MainThreadTaskQueue *ownerScheduler, std::shared_ptr<bool> taskArmed,
        const std::shared_ptr<void> &ownerLifetime)
        : owner(ownerScheduler),
          armed(std::move(taskArmed)),
          lifetime(ownerLifetime),
          lifetimeGuarded(true)
    {}

    MainThreadTaskQueue *owner = nullptr;
    // armed is shared with the queue's TaskEntry: clearing it lazily skips
    // the entry inside tick() without touching the heap.
    std::shared_ptr<bool> armed;
    std::weak_ptr<void>   lifetime;
    bool                  lifetimeGuarded = false;
};

// Main-thread only (tick and schedule must not race); no internal locking.
class MainThreadTaskQueue final
{
public:
    MainThreadTaskQueue() = default;

    explicit MainThreadTaskQueue(MainThreadTaskQueueOptions queueOptions)
        : options_(queueOptions)
    {}

    MainThreadTaskQueue(const MainThreadTaskQueue &)            = delete;
    MainThreadTaskQueue &operator=(const MainThreadTaskQueue &) = delete;

    ~MainThreadTaskQueue()
    {
        clear();
        lifetimeToken_.reset();
    }

    void setOptions(MainThreadTaskQueueOptions queueOptions)
    {
        options_ = queueOptions;
    }

    [[nodiscard]] const MainThreadTaskQueueOptions &options() const
    {
        return options_;
    }

    // One-shot callback after delaySeconds (clamped to >= 0); invalid input
    // or a full queue returns an invalid handle.
    [[nodiscard]] ScheduledTask scheduleAfter(double delaySeconds, std::function<void()> task)
    {
        return scheduleTaskEntry(delaySeconds, 0.0, std::move(task));
    }

    // Repeating callback every intervalSeconds; non-finite or non-positive
    // intervals return an invalid handle.
    [[nodiscard]] ScheduledTask scheduleEvery(double intervalSeconds, std::function<void()> task)
    {
        if (!(intervalSeconds > 0.0) || !std::isfinite(intervalSeconds))
        {
            return ScheduledTask{};
        }
        return scheduleTaskEntry(intervalSeconds, intervalSeconds, std::move(task));
    }

    // Runs every due task. Due entries are popped into dueScratch_ first so a
    // callback may schedule or cancel other tasks safely; a throwing callback
    // disarms itself, re-queues the rest and rethrows.
    void tick()
    {
        if (ticking_ || heap_.empty())
        {
            return;
        }

        const auto now = Clock::now();
        dueScratch_.clear();
        const std::size_t callbackLimit      = options_.maxCallbacksPerTick;
        std::size_t       collectedCallbacks = 0;

        // Cancelled (disarmed) entries are skipped; the per-tick callback limit
        // re-queues the entry that exceeded the budget.
        while (!heap_.empty() && heap_.front().nextDue <= now)
        {
            std::pop_heap(heap_.begin(), heap_.end(), EarlierFirst{});
            TaskEntry entry = std::move(heap_.back());
            heap_.pop_back();
            if (!*entry.armed)
            {
                continue;
            }
            if (callbackLimit != 0 && collectedCallbacks >= callbackLimit)
            {
                insertTask(std::move(entry));
                break;
            }
            dueScratch_.push_back(std::move(entry));
            ++collectedCallbacks;
        }

        if (dueScratch_.empty())
        {
            return;
        }

        ticking_          = true;
        std::size_t index = 0;
        try
        {
            for (; index < dueScratch_.size(); ++index)
            {
                TaskEntry &entry = dueScratch_[index];
                if (!*entry.armed)
                {
                    continue;
                }

                try
                {
                    entry.callback();
                }
                catch (...)
                {
                    disarm(entry);
                    throw;
                }

                // Recurring reschedule: CatchUpOnePerTick advances one interval per tick;
                // SkipMissedIntervals realigns to the next unpassed multiple of the
                // interval so it never fires a burst.
                if (entry.intervalSeconds > 0.0 && *entry.armed)
                {
                    const auto interval        = secondsAsDuration(entry.intervalSeconds);
                    const auto nextIntervalDue = addSaturated(entry.nextDue, interval);
                    if (nextIntervalDue > now || options_.recurringTaskPolicy == RecurringTaskPolicy::CatchUpOnePerTick)
                    {
                        entry.nextDue = nextIntervalDue;
                    }
                    else
                    {
                        const auto elapsed = now - entry.nextDue;
                        entry.nextDue      = addSaturated(now, interval - (elapsed % interval));
                    }
                    insertTask(std::move(entry));
                }
                else if (*entry.armed)
                {
                    disarm(entry);
                }
            }
            // The throwing task is disarmed; still-due tasks return to the heap so
            // the next tick() can retry them.
        }
        catch (...)
        {
            if (index < dueScratch_.size())
            {
                disarm(dueScratch_[index]);
            }
            for (std::size_t remaining = index + 1; remaining < dueScratch_.size(); ++remaining)
            {
                if (*dueScratch_[remaining].armed)
                {
                    insertTask(std::move(dueScratch_[remaining]));
                }
            }
            ticking_ = false;
            dueScratch_.clear();
            compactIfNeeded();
            throw;
        }

        ticking_ = false;
        dueScratch_.clear();
        compactIfNeeded();
    }

    // Disarms every task, including the in-flight batch (when not ticking).
    void clear()
    {
        for (TaskEntry &entry : heap_)
        {
            disarm(entry);
        }
        heap_.clear();

        for (TaskEntry &entry : dueScratch_)
        {
            disarm(entry);
        }
        if (!ticking_)
        {
            dueScratch_.clear();
        }
        activeTaskCount_ = 0;
    }

    // Live (armed) tasks, including any currently executing batch.
    [[nodiscard]] std::size_t taskCount() const
    {
        return activeTaskCount_;
    }

private:
    friend class ScheduledTask;

    using Clock = std::chrono::steady_clock;

    // sequence breaks ties so same-deadline tasks run in schedule order.
    struct TaskEntry
    {
        Clock::time_point     nextDue;
        double                intervalSeconds = 0.0;
        std::function<void()> callback;
        std::shared_ptr<bool> armed;
        std::uint64_t         sequence = 0;
    };

    // Min-heap comparator: earliest nextDue first, then lowest sequence.
    struct EarlierFirst
    {
        bool operator()(const TaskEntry &left, const TaskEntry &right) const
        {
            if (left.nextDue != right.nextDue)
            {
                return left.nextDue > right.nextDue;
            }
            return left.sequence > right.sequence;
        }
    };

    // Converts seconds to a steady_clock duration, clamping to at least one
    // clock tick and to the representable maximum (overflow guard).
    static Clock::duration secondsAsDuration(double seconds)
    {
        if (!(seconds > 0.0))
        {
            return Clock::duration(1);
        }
        const double maximumSeconds = std::chrono::duration<double>(Clock::duration::max()).count();
        if (seconds >= maximumSeconds)
        {
            return Clock::duration::max();
        }
        const Clock::duration converted
            = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
        return converted > Clock::duration::zero() ? converted : Clock::duration(1);
    }

    // Saturating time_point addition (recurring tasks far in the future).
    static Clock::time_point addSaturated(Clock::time_point base, Clock::duration increment)
    {
        const Clock::duration available = Clock::time_point::max() - base;
        return increment >= available ? Clock::time_point::max() : base + increment;
    }

    static Clock::time_point deadlineAfter(double seconds)
    {
        return addSaturated(Clock::now(), secondsAsDuration(seconds));
    }

    // Common path for both schedulers: rejects empty callbacks, non-finite
    // delays and full queues before allocating the shared armed flag.
    ScheduledTask scheduleTaskEntry(double delaySeconds, double intervalSeconds, std::function<void()> task)
    {
        if (!task || !std::isfinite(delaySeconds) || (options_.maxTasks != 0 && activeTaskCount_ >= options_.maxTasks))
        {
            return ScheduledTask{};
        }

        TaskEntry entry;
        entry.nextDue                   = deadlineAfter(std::max(delaySeconds, 0.0));
        entry.intervalSeconds           = intervalSeconds;
        entry.callback                  = std::move(task);
        entry.armed                     = std::make_shared<bool>(true);
        entry.sequence                  = nextSequence_++;
        std::shared_ptr<bool> taskArmed = entry.armed;
        insertTask(std::move(entry));
        ++activeTaskCount_;
        return ScheduledTask(this, std::move(taskArmed), lifetimeToken_);
    }

    void insertTask(TaskEntry entry)
    {
        heap_.push_back(std::move(entry));
        std::push_heap(heap_.begin(), heap_.end(), EarlierFirst{});
    }

    void disarm(TaskEntry &entry)
    {
        if (entry.armed != nullptr && *entry.armed)
        {
            *entry.armed = false;
            if (activeTaskCount_ > 0)
            {
                --activeTaskCount_;
            }
        }
    }

    // Cancel path from ScheduledTask::cancel; heap compaction is deferred
    // while tick() runs (entries are skipped via the armed flag).
    void removeTask(const std::shared_ptr<bool> &taskArmed)
    {
        if (taskArmed == nullptr || !*taskArmed)
        {
            return;
        }
        *taskArmed = false;
        if (activeTaskCount_ > 0)
        {
            --activeTaskCount_;
        }
        if (!ticking_)
        {
            compactIfNeeded();
        }
    }

    // Physically removes disarmed entries once they double the live count
    // (+32 slack), then rebuilds the heap.
    void compactIfNeeded()
    {
        if (heap_.size() <= activeTaskCount_ * 2 + 32)
        {
            return;
        }
        heap_.erase(std::remove_if(heap_.begin(), heap_.end(), [](const TaskEntry &entry) { return !*entry.armed; }),
            heap_.end());
        std::make_heap(heap_.begin(), heap_.end(), EarlierFirst{});
    }

    // dueScratch_ holds the batch being executed; lifetimeToken_ lets
    // guarded handles detect queue destruction (see ScheduledTask::cancel).
    MainThreadTaskQueueOptions options_{};
    std::vector<TaskEntry>     heap_;
    std::vector<TaskEntry>     dueScratch_;
    std::size_t                activeTaskCount_ = 0;
    std::uint64_t              nextSequence_    = 1;
    bool                       ticking_         = false;
    std::shared_ptr<void>      lifetimeToken_   = std::make_shared<unsigned char>(0);
};

inline void ScheduledTask::cancel()
{
    if (armed == nullptr)
    {
        owner = nullptr;
        return;
    }
    if (owner != nullptr && (!lifetimeGuarded || !lifetime.expired()))
    {
        owner->removeTask(armed);
    }
    else
    {
        *armed = false;
    }
    owner = nullptr;
    armed.reset();
    lifetime.reset();
    lifetimeGuarded = false;
}

} // namespace engine

#endif
