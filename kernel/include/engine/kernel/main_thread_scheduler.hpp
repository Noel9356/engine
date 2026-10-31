#ifndef ENGINE_KERNEL_MAIN_THREAD_SCHEDULER_HPP
#define ENGINE_KERNEL_MAIN_THREAD_SCHEDULER_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace engine
{

class MainThreadScheduler;

class ScheduledTask final
{
public:
    ScheduledTask() = default;

    ScheduledTask(
        MainThreadScheduler  *ownerScheduler,
        std::shared_ptr<bool> taskArmed
    )
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
          armed(std::move(other.armed))
    {
        other.owner = nullptr;
    }

    ScheduledTask &operator=(ScheduledTask &&other) noexcept
    {
        if (this != &other)
        {
            cancel();
            owner       = other.owner;
            armed       = std::move(other.armed);
            other.owner = nullptr;
        }
        return *this;
    }

    void cancel();

    bool valid() const
    {
        return armed != nullptr;
    }

private:
    MainThreadScheduler *owner = nullptr;

    std::shared_ptr<bool> armed;
};

class MainThreadScheduler final
{
public:
    MainThreadScheduler() = default;

    MainThreadScheduler(const MainThreadScheduler &)            = delete;
    MainThreadScheduler &operator=(const MainThreadScheduler &) = delete;

    ~MainThreadScheduler()
    {
        clear();
    }

    [[nodiscard]] ScheduledTask scheduleAfter(
        double                delaySeconds,
        std::function<void()> task
    );

    [[nodiscard]] ScheduledTask scheduleEvery(
        double                intervalSeconds,
        std::function<void()> task
    );

    void tick();

    void clear();

    std::size_t taskCount() const;

private:
    friend class ScheduledTask;

    struct TaskEntry
    {
        std::chrono::steady_clock::time_point nextDue;

        double                intervalSeconds = 0.0;
        std::function<void()> callback;

        std::shared_ptr<bool> armed;
    };

    void insertTask(TaskEntry entry);

    ScheduledTask scheduleTaskEntry(
        double                delaySeconds,
        double                intervalSeconds,
        std::function<void()> task
    );

    void removeTask(const std::shared_ptr<bool> &taskArmed);

    static std::chrono::steady_clock::duration secondsAsDuration(double seconds)
    {
        std::chrono::steady_clock::duration converted{};
        if (seconds > 0.0 && std::isfinite(seconds))
        {
            converted = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(seconds)
            );
        }
        if (converted <= std::chrono::steady_clock::duration::zero())
        {
            converted = std::chrono::steady_clock::duration(1);
        }
        return converted;
    }

    std::vector<TaskEntry> tasks;

    std::vector<TaskEntry> dueScratch;
    bool                   ticking = false;
};

inline ScheduledTask MainThreadScheduler::scheduleAfter(
    double                delaySeconds,
    std::function<void()> task
)
{
    return scheduleTaskEntry(delaySeconds, 0.0, std::move(task));
}

inline ScheduledTask MainThreadScheduler::scheduleEvery(
    double                intervalSeconds,
    std::function<void()> task
)
{
    if (intervalSeconds <= 0.0)
    {
        return ScheduledTask{};
    }
    return scheduleTaskEntry(intervalSeconds, intervalSeconds, std::move(task));
}

inline void MainThreadScheduler::tick()
{
    if (ticking || tasks.empty())
    {
        return;
    }
    const auto now = std::chrono::steady_clock::now();

    std::size_t dueCount = 0;
    for (std::size_t index = 0; index < tasks.size(); ++index)
    {
        if (tasks[index].nextDue > now)
        {
            break;
        }
        ++dueCount;
    }
    if (dueCount == 0)
    {
        return;
    }

    dueScratch.clear();
    dueScratch.reserve(dueCount);
    for (std::size_t index = 0; index < dueCount; ++index)
    {
        if (*tasks[index].armed)
        {
            dueScratch.push_back(std::move(tasks[index]));
        }
    }
    tasks.erase(tasks.begin(), tasks.begin() + static_cast<std::ptrdiff_t>(dueCount));

    ticking = true;
    for (TaskEntry &entry : dueScratch)
    {
        if (!*entry.armed)
        {
            continue;
        }
        entry.callback();
        if (entry.intervalSeconds > 0.0 && *entry.armed)
        {
            TaskEntry  recurring   = std::move(entry);
            const auto interval    = secondsAsDuration(recurring.intervalSeconds);
            const auto intervalCnt = interval.count();
            if (recurring.nextDue + interval > now)
            {
                recurring.nextDue += interval;
            } else
            {
                const auto missedCount = (now - recurring.nextDue).count() / intervalCnt;
                if (missedCount > 1000000000)
                {
                    recurring.nextDue = now + interval;
                } else
                {
                    recurring.nextDue += interval * static_cast<std::intmax_t>(missedCount + 1);
                }
            }
            insertTask(std::move(recurring));
        }
    }

    dueScratch.clear();
    ticking = false;
}

inline void MainThreadScheduler::clear()
{
    for (TaskEntry &entry : tasks)
    {
        if (entry.armed)
        {
            *entry.armed = false;
        }
    }
    tasks.clear();
}

inline std::size_t MainThreadScheduler::taskCount() const
{
    return tasks.size();
}

inline void MainThreadScheduler::insertTask(TaskEntry entry)
{
    std::size_t insertPosition = tasks.size();
    for (std::size_t index = 0; index < tasks.size(); ++index)
    {
        if (tasks[index].nextDue > entry.nextDue)
        {
            insertPosition = index;
            break;
        }
    }
    tasks.insert(tasks.begin() + static_cast<std::ptrdiff_t>(insertPosition), std::move(entry));
}

inline void MainThreadScheduler::removeTask(const std::shared_ptr<bool> &taskArmed)
{
    for (std::size_t index = 0; index < tasks.size(); ++index)
    {
        if (tasks[index].armed == taskArmed)
        {
            tasks.erase(tasks.begin() + static_cast<std::ptrdiff_t>(index));
            return;
        }
    }
}

inline void ScheduledTask::cancel()
{
    if (owner == nullptr || armed == nullptr)
    {
        return;
    }
    *armed = false;
    owner->removeTask(armed);
    owner = nullptr;
    armed.reset();
}

inline ScheduledTask MainThreadScheduler::scheduleTaskEntry(
    double                delaySeconds,
    double                intervalSeconds,
    std::function<void()> task
)
{
    TaskEntry entry;
    entry.nextDue                   = std::chrono::steady_clock::now() + secondsAsDuration(std::max(delaySeconds, 0.0));
    entry.intervalSeconds           = intervalSeconds;
    entry.callback                  = std::move(task);
    entry.armed                     = std::make_shared<bool>(true);
    std::shared_ptr<bool> taskArmed = entry.armed;
    insertTask(std::move(entry));
    return ScheduledTask(this, std::move(taskArmed));
}

} // namespace engine

#endif
