// Work-sharing task scheduler with priority queues.
//
// Fixed worker threads pull from per-worker deques (4 priorities), the
// shared injection queue (off-worker submissions) and steal from peers.
// Priority aging prevents starvation; TaskGroup provides fork/join with
// exception propagation and TaskGraph runs acyclic dependency DAGs.

#ifndef ENGINE_KERNEL_TASK_SCHEDULER_HPP
#define ENGINE_KERNEL_TASK_SCHEDULER_HPP

#include <engine/kernel/profiling.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace engine
{

// Lower value = more urgent; indexes the 4 per-worker deques.
enum class TaskPriority : std::uint8_t
{
    Critical = 0,
    High,
    Normal,
    Low
};

// workerCount 0 = hardware concurrency - 1, clamped to maxWorkerCount.
struct TaskSchedulerOptions
{
    std::uint32_t workerCount    = 0;
    std::uint32_t maxWorkerCount = 16;

    // Rejection threshold; 0 disables the bound.
    std::size_t maxOutstandingTasks = 65536;

    // A queued task whose head waited at least this long is dequeued ahead of
    // higher-priority work (starvation guard); 0 disables aging.
    std::chrono::milliseconds priorityAgingThreshold{250};

    // Workers rotate over own deque, injection queue and stealing so no
    // source starves.
    bool queueSourceFairness = true;

    // Yield-retry budget before a worker parks on the condvar. 0 parks
    // immediately (the historical behaviour); higher values trim fork/join
    // wake latency at the cost of extra idle spinning.
    std::uint32_t workerSpinBeforePark = 8;

    // Yield-retry budget in waitIdle() before parking on the condvar. 0 parks
    // immediately; higher values trade idle CPU for shorter join latency.
    std::uint32_t waitIdleSpinIterations = 64;
};

// Point-in-time counters: queuedTasks excludes executing tasks;
// outstandingTasks includes queued + executing; agedTasksDequeued counts
// aging interventions.
struct TaskSchedulerStatistics
{
    std::uint32_t workerCount         = 0;
    std::size_t   queuedTasks         = 0;
    std::size_t   outstandingTasks    = 0;
    std::uint64_t submittedTasks      = 0;
    std::uint64_t completedTasks      = 0;
    std::uint64_t rejectedTasks       = 0;
    std::uint64_t agedTasksDequeued   = 0;
    bool          queueSourceFairness = true;
};

class TaskScheduler;
class TaskGraph;

// Join point for a batch of tasks; wait() helps execute queued work while
// blocked and rethrows the first stored task exception.
class TaskGroup final
{
public:
    TaskGroup()                             = default;
    TaskGroup(const TaskGroup &)            = delete;
    TaskGroup &operator=(const TaskGroup &) = delete;

    // Tasks added but not yet completed.
    [[nodiscard]] std::size_t outstanding() const noexcept
    {
        return outstanding_.load(std::memory_order_acquire);
    }

    // Participates: runs scheduler tasks on the calling thread until the
    // group drains (avoids deadlock when the pool is saturated).
    void wait(TaskScheduler &scheduler);

private:
    friend class TaskScheduler;
    friend class TaskGraph;

    void add() noexcept
    {
        outstanding_.fetch_add(1, std::memory_order_acq_rel);
    }
    void complete() noexcept;
    void setException(std::exception_ptr exception) noexcept;

    std::atomic<std::size_t> outstanding_{0};
    mutable std::mutex       mutex_;
    std::condition_variable  condition_;
    std::exception_ptr       exception_;
};

// start()/stop() own the worker threads; schedule() is safe from any
// thread, including workers (thread-local detection routes to the local
// deque). stop() refuses to run from inside a task.
class TaskScheduler final
{
public:
    using Task             = std::function<void()>;
    using ExceptionHandler = std::function<void(std::exception_ptr)>;

    TaskScheduler() = default;
    explicit TaskScheduler(TaskSchedulerOptions options)
        : options_(normalise(options))
    {}

    TaskScheduler(const TaskScheduler &)            = delete;
    TaskScheduler &operator=(const TaskScheduler &) = delete;

    ~TaskScheduler()
    {
        stop();
    }

    // Throws std::logic_error while running.
    void setOptions(TaskSchedulerOptions options)
    {
        std::lock_guard lifecycleLock(lifecycleMutex_);
        std::lock_guard lock(mutex_);
        if (running_)
            throw std::logic_error("task scheduler options cannot change while running");
        options_ = normalise(options);
    }

    [[nodiscard]] TaskSchedulerOptions options() const
    {
        std::lock_guard lock(mutex_);
        return options_;
    }

    void setProfiler(PerformanceProfiler *profiler) noexcept
    {
        profiler_ = profiler;
    }

    // Invoked for uncaught task exceptions; the default prints to stderr.
    void setExceptionHandler(ExceptionHandler handler)
    {
        std::lock_guard lock(mutex_);
        exceptionHandler_ = std::move(handler);
    }

    // Idempotent; waits out an in-progress stop before restarting. On worker
    // creation failure everything is joined and the exception rethrown.
    void start()
    {
        for (;;)
        {
            std::unique_lock lifecycleLock(lifecycleMutex_);
            {
                std::unique_lock lock(mutex_);
                if (running_)
                {
                    if (!stopInProgress_)
                        return;
                    lifecycleLock.unlock();
                    lifecycleCondition_.wait(lock, [this] { return !stopInProgress_; });
                    continue;
                }
            }

            const std::uint32_t requested   = options_.workerCount == 0 ? defaultWorkerCount() : options_.workerCount;
            const std::uint32_t workerCount = std::clamp(requested, 1u, options_.maxWorkerCount);
            std::vector<std::unique_ptr<WorkerQueue>> queues;
            queues.reserve(workerCount);
            for (std::uint32_t index = 0; index < workerCount; ++index)
                queues.push_back(std::make_unique<WorkerQueue>());
            {
                std::lock_guard lock(mutex_);
                workerQueues_ = std::move(queues);
                stopping_     = false;
                accepting_    = false;
                running_      = true;
            }
            try
            {
                workers_.reserve(workerCount);
                for (std::uint32_t index = 0; index < workerCount; ++index)
                    workers_.emplace_back([this, index] { workerLoop(index); });
                {
                    std::lock_guard lock(mutex_);
                    accepting_ = true;
                }
                // Publish the hint only after the workers exist and tasks are
                // accepted: 0 means "do not submit parallel work".
                runningWorkerCount_.store(workerCount, std::memory_order_release);
                condition_.notify_all();
            }
            catch (...)
            {
                {
                    std::lock_guard lock(mutex_);
                    accepting_ = false;
                    stopping_  = true;
                }
                runningWorkerCount_.store(0, std::memory_order_release);
                condition_.notify_all();
                for (std::thread &worker : workers_)
                    if (worker.joinable())
                        worker.join();
                workers_.clear();
                {
                    std::unique_lock lock(mutex_);
                    helperCondition_.wait(lock, [this] { return activeExternalHelpers_ == 0; });
                    workerQueues_.clear();
                    running_  = false;
                    stopping_ = false;
                }
                throw;
            }
            return;
        }
    }

    // Joins all workers after they drain their queues; also waits for
    // external helpers (tryRunOne callers) to leave before dropping queues.
    void stop()
    {
        if (currentScheduler_ == this)
            throw std::logic_error("a task cannot stop its own scheduler");

        std::vector<std::thread> workers;
        {
            std::unique_lock lifecycleLock(lifecycleMutex_);
            std::unique_lock lock(mutex_);
            if (!running_)
                return;
            if (stopInProgress_)
            {
                lifecycleLock.unlock();
                lifecycleCondition_.wait(lock, [this] { return !stopInProgress_; });
                return;
            }
            stopInProgress_ = true;
            accepting_      = false;
            stopping_       = true;
            // Retract the parallel-work hint first: new parallel triggers see
            // "no workers" and run inline while the drain completes.
            runningWorkerCount_.store(0, std::memory_order_release);
            workers.swap(workers_);
        }
        condition_.notify_all();
        for (std::thread &worker : workers)
            if (worker.joinable())
                worker.join();
        {
            std::lock_guard  lifecycleLock(lifecycleMutex_);
            std::unique_lock lock(mutex_);
            helperCondition_.wait(lock, [this] { return activeExternalHelpers_ == 0; });
            workerQueues_.clear();
            running_        = false;
            stopping_       = false;
            stopInProgress_ = false;
        }
        lifecycleCondition_.notify_all();
    }

    [[nodiscard]] bool isRunning() const noexcept
    {
        std::lock_guard lock(mutex_);
        return running_;
    }

    // Lock-free outstanding-task count (submitted but not finished). Cheap
    // enough for per-frame checks; use statistics() for the exact, locking
    // view that also reports queued vs. executing.
    [[nodiscard]] std::size_t outstandingTaskCount() const noexcept
    {
        return outstandingTasks_.load(std::memory_order_acquire);
    }

    // Lock-free worker-count hint: the live worker count while running, 0
    // otherwise. Published in an atomic at start/stop so per-frame callers
    // (parallel rasterization triggers, parallelFor chunking) never need the
    // scheduler mutexes - statistics() remains the exact, locking variant
    // for diagnostics.
    [[nodiscard]] std::uint32_t runningWorkerCountHint() const noexcept
    {
        return runningWorkerCount_.load(std::memory_order_acquire);
    }

    // Returns false (counting a rejection) when stopping, not running, or the
    // outstanding bound is hit. Worker callers push to their own deque;
    // everyone else to the shared injection queue.
    bool schedule(Task task, TaskPriority priority = TaskPriority::Normal)
    {
        if (!task)
            return false;
        const auto priorityIndex = static_cast<std::size_t>(priority);
        if (priorityIndex >= 4)
            return false;

        WorkItem item{std::move(task), Clock::now()};
        {
            std::lock_guard lock(mutex_);
            if (!canAcceptLocked())
                return rejectLocked();

            if (currentScheduler_ == this && currentWorkerIndex_ < workerQueues_.size())
            {
                WorkerQueue    &queue = *workerQueues_[currentWorkerIndex_];
                std::lock_guard workerLock(queue.mutex);
                queue.tasks[priorityIndex].push_back(std::move(item));
            }
            else
            {
                injectionQueues_[priorityIndex].push_back(std::move(item));
            }
            // queuedApprox_ counts tasks sitting in a queue (pops decrement it
            // after taking the item). The increment happens under mutex_ so the
            // worker wake predicate - which also runs under mutex_ - can never
            // miss it (no lost-wakeup window between check and sleep).
            queuedApprox_.fetch_add(1, std::memory_order_release);
            outstandingTasks_.fetch_add(1, std::memory_order_relaxed);
            submittedTasks_.fetch_add(1, std::memory_order_relaxed);
        }
        // Notify outside the lock: the waiter re-checks under mutex_, so the
        // increment above is already visible; waking without holding mutex_
        // avoids the wake-then-immediately-block-on-mutex_ pessimization.
        condition_.notify_one();
        return true;
    }

    // Wraps the task so the group completes even on exception or scheduling
    // rejection.
    bool schedule(TaskGroup &group, Task task, TaskPriority priority = TaskPriority::Normal)
    {
        if (!task)
            return false;
        group.add();
        bool accepted = false;
        try
        {
            accepted = schedule(
                [task = std::move(task), &group]() mutable
                {
                    try
                    {
                        task();
                    }
                    catch (...)
                    {
                        group.setException(std::current_exception());
                    }
                    group.complete();
                },
                priority);
        }
        catch (...)
        {
            group.complete();
            throw;
        }
        if (!accepted)
            group.complete();
        return accepted;
    }

    // Executes one task if any exist (any queue). Non-worker callers register
    // as external helpers so stop() waits for them; safe inside tasks (work
    // contraction, e.g. TaskGroup::wait).
    bool tryRunOne()
    {
        const bool workerCaller = currentScheduler_ == this;
        if (!workerCaller)
        {
            std::lock_guard lock(mutex_);
            if (!running_)
                return false;
            ++activeExternalHelpers_;
        }

        WorkItem   item;
        const bool found = tryPopTask(invalidWorkerIndex, item);
        if (found)
        {
            TaskScheduler      *previousScheduler   = currentScheduler_;
            const std::uint32_t previousWorkerIndex = currentWorkerIndex_;
            currentScheduler_                       = this;
            currentWorkerIndex_                     = invalidWorkerIndex;
            executeTask(item);
            currentScheduler_   = previousScheduler;
            currentWorkerIndex_ = previousWorkerIndex;
        }

        if (!workerCaller)
        {
            std::lock_guard lock(mutex_);
            --activeExternalHelpers_;
            helperCondition_.notify_all();
        }
        return found;
    }

    // Blocks until every submitted task completed, helping run tasks meanwhile.
    void waitIdle()
    {
        const std::uint32_t spinBudget = options_.waitIdleSpinIterations;
        for (;;)
        {
            if (tryRunOne())
                continue;
            bool idle = false;
            for (std::uint32_t spin = 0; spin < spinBudget; ++spin)
            {
                if (outstandingTasks_.load(std::memory_order_acquire) == 0)
                {
                    idle = true;
                    break;
                }
                engineSpinWaitHint();
            }
            if (idle)
                return;
            std::unique_lock lock(mutex_);
            if (outstandingTasks_.load(std::memory_order_acquire) == 0)
                return;
            idleCondition_.wait(lock, [this] { return outstandingTasks_.load(std::memory_order_acquire) == 0; });
        }
    }

    // Locks both mutexes; fine for diagnostics, not for hot paths.
    [[nodiscard]] TaskSchedulerStatistics statistics() const
    {
        std::lock_guard lifecycleLock(lifecycleMutex_);
        std::size_t     queued = 0;
        {
            std::lock_guard lock(mutex_);
            queued = queuedTasksLocked();
            return TaskSchedulerStatistics{
                static_cast<std::uint32_t>(workers_.size()),
                queued,
                outstandingTasks_.load(std::memory_order_relaxed),
                submittedTasks_.load(std::memory_order_relaxed),
                completedTasks_.load(std::memory_order_relaxed),
                rejectedTasks_.load(std::memory_order_relaxed),
                agedTasksDequeued_.load(std::memory_order_relaxed),
                options_.queueSourceFairness};
        }
    }

    // Splits [0, itemCount) into ~workers+1 contiguous chunks (each at least
    // minItemsPerTask) and runs body(begin, end) in parallel; the caller's
    // chunk runs inline. Falls back to a single body(0, itemCount) call when
    // the scheduler is not running. Rethrows the first body exception after
    // joining.
    template <typename Body>
    void parallelFor(
        std::size_t itemCount, std::size_t minItemsPerTask, Body &&body, TaskPriority priority = TaskPriority::Normal)
    {
        if (itemCount == 0)
            return;
        const std::uint32_t workers = workerCountHint();
        if (workers == 0)
        {
            // 0 = not running (lock-free hint), including the stop-drain
            // window: run inline rather than submitting.
            body(0, itemCount);
            return;
        }

        minItemsPerTask          = std::max<std::size_t>(minItemsPerTask, 1);
        const std::size_t chunks = std::min<std::size_t>(workers + 1u, 1u + (itemCount - 1u) / minItemsPerTask);
        if (chunks <= 1)
        {
            body(0, itemCount);
            return;
        }

        TaskGroup         group;
        const std::size_t chunkSize = itemCount / chunks;
        const std::size_t remainder = itemCount % chunks;
        auto              taskBody  = std::make_shared<std::decay_t<Body>>(std::forward<Body>(body));
        for (std::size_t chunk = 1; chunk < chunks; ++chunk)
        {
            const std::size_t begin = chunk * chunkSize + std::min(chunk, remainder);
            const std::size_t end   = begin + chunkSize + (chunk < remainder ? 1u : 0u);
            if (!schedule(group, [taskBody, begin, end] { (*taskBody)(begin, end); }, priority))
            {
                try
                {
                    (*taskBody)(begin, end);
                }
                catch (...)
                {
                    group.setException(std::current_exception());
                }
            }
        }
        try
        {
            (*taskBody)(0, chunkSize + (remainder > 0 ? 1u : 0u));
        }
        catch (...)
        {
            group.setException(std::current_exception());
        }
        group.wait(*this);
    }

    // Convenience wrapper over parallelFor that iterates a random-access
    // container's [0, container.size()) range; the body receives a reference
    // to each element. Falls back to sequential execution when the scheduler
    // is not running or the container is small.
    template <typename Container, typename Body>
    void parallelForEach(Container &container, std::size_t minItemsPerTask, Body &&body,
        TaskPriority priority = TaskPriority::Normal)
    {
        const std::size_t count = container.size();
        if (count == 0)
            return;
        auto *data = container.data();
        parallelFor(count, minItemsPerTask, [data, &body](std::size_t begin, std::size_t end) {
            for (std::size_t index = begin; index < end; ++index)
                body(data[index]);
        }, priority);
    }

private:
    friend class TaskGroup;

    using Clock                                       = std::chrono::steady_clock;
    static constexpr std::uint32_t invalidWorkerIndex = UINT32_MAX;

    // enqueuedAt feeds the queue-wait metric and priority aging.
    struct WorkItem
    {
        Task              task;
        Clock::time_point enqueuedAt;
    };

    // One worker's deques (index = TaskPriority) plus fairness cursors.
    // alignas(64): each worker's queue gets its own cache line - the mutex
    // is taken by the owner on every pop and by thieves on steal attempts,
    // so adjacent queues sharing a line would have steal attempts evict the
    // owner's mutex line.
    struct alignas(64) WorkerQueue
    {
        std::mutex                          mutex;
        std::array<std::deque<WorkItem>, 4> tasks;

        // Rotates 0..2: own deque, injection queue, steal victim.
        std::uint8_t  nextQueueSource = 0;
        std::uint32_t nextVictim      = 0;
    };

    // hardware_concurrency - 1, at least 1 (leaves a core for the main thread).
    static std::uint32_t defaultWorkerCount() noexcept
    {
        const unsigned int hardware = std::thread::hardware_concurrency();
        return std::max(1u, hardware > 1u ? hardware - 1u : 1u);
    }

    // Clamps options into sane ranges (workerCount 0 keeps "auto").
    static TaskSchedulerOptions normalise(TaskSchedulerOptions options)
    {
        options.maxWorkerCount = std::clamp(options.maxWorkerCount, 1u, 256u);
        if (options.workerCount != 0)
            options.workerCount = std::clamp(options.workerCount, 1u, options.maxWorkerCount);
        if (options.priorityAgingThreshold.count() < 0)
            options.priorityAgingThreshold = std::chrono::milliseconds::zero();
        options.workerSpinBeforePark    = std::clamp(options.workerSpinBeforePark, 0u, 4096u);
        options.waitIdleSpinIterations = std::clamp(options.waitIdleSpinIterations, 0u, 4096u);
        return options;
    }

    bool canAcceptLocked() const noexcept
    {
        return accepting_ && running_
            && (options_.maxOutstandingTasks == 0
                || outstandingTasks_.load(std::memory_order_relaxed) < options_.maxOutstandingTasks);
    }

    bool rejectLocked() noexcept
    {
        rejectedTasks_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }

    std::uint32_t workerCountHint() const noexcept
    {
        // Lock-free: the hint atomic is maintained by start()/stop(); the
        // previous implementation locked lifecycleMutex_ on every call (it
        // runs once per parallelFor - per render-graph frame in this engine).
        return runningWorkerCount_.load(std::memory_order_acquire);
    }

    // Aging pass first: when several priorities are non-empty and the oldest
    // head exceeded the threshold, take it; otherwise strict priority order.
    // fromBack = LIFO for own-queue pops (better cache locality).
    bool popPriorityQueue(std::array<std::deque<WorkItem>, 4> &queues, WorkItem &out, bool fromBack)
    {
        const auto agingThreshold = options_.priorityAgingThreshold;
        if (agingThreshold.count() > 0)
        {
            // Single pass: count non-empty priorities and track the oldest
            // expired head simultaneously, so the common "no aging needed"
            // case costs one scan instead of two.
            std::size_t       nonEmptyPriorities = 0;
            std::size_t       oldestQueue        = queues.size();
            Clock::time_point oldestEnqueue{};
            const auto        now = Clock::now();
            for (std::size_t index = 0; index < queues.size(); ++index)
            {
                const auto &queue = queues[index];
                if (queue.empty())
                {
                    continue;
                }
                ++nonEmptyPriorities;
                const auto enqueuedAt = queue.front().enqueuedAt;
                if (now - enqueuedAt >= agingThreshold
                    && (oldestQueue == queues.size() || enqueuedAt < oldestEnqueue))
                {
                    oldestQueue   = index;
                    oldestEnqueue = enqueuedAt;
                }
            }
            if (nonEmptyPriorities > 1 && oldestQueue != queues.size())
            {
                auto &queue = queues[oldestQueue];
                out         = std::move(queue.front());
                queue.pop_front();
                agedTasksDequeued_.fetch_add(1, std::memory_order_relaxed);
                return true;
            }
        }

        for (auto &queue : queues)
        {
            if (!queue.empty())
            {
                if (fromBack)
                {
                    out = std::move(queue.back());
                    queue.pop_back();
                }
                else
                {
                    out = std::move(queue.front());
                    queue.pop_front();
                }
                return true;
            }
        }
        return false;
    }

    // workerIndex beyond the vector means external helper: injection queue,
    // then steal. Workers use the fairness rotation when enabled; without it:
    // own LIFO, injection, then a steal scan.
    bool tryPopTask(std::uint32_t workerIndex, WorkItem &out)
    {
        if (tryPopTaskInternal(workerIndex, out))
        {
            // The item left a queue; keep the wake-predicate hint exact.
            queuedApprox_.fetch_sub(1, std::memory_order_release);
            return true;
        }
        return false;
    }

    bool tryPopTaskInternal(std::uint32_t workerIndex, WorkItem &out)
    {
        if (workerIndex >= workerQueues_.size())
        {
            {
                std::lock_guard lock(mutex_);
                if (popPriorityQueue(injectionQueues_, out, false))
                    return true;
            }
            for (std::uint32_t victim = 0; victim < workerQueues_.size(); ++victim)
            {
                WorkerQueue     &queue = *workerQueues_[victim];
                std::unique_lock lock(queue.mutex, std::try_to_lock);
                if (lock.owns_lock() && popPriorityQueue(queue.tasks, out, false))
                    return true;
            }
            return false;
        }

        WorkerQueue &own = *workerQueues_[workerIndex];
        if (!options_.queueSourceFairness)
        {
            {
                std::lock_guard lock(own.mutex);
                if (popPriorityQueue(own.tasks, out, true))
                    return true;
            }
            {
                std::lock_guard lock(mutex_);
                if (popPriorityQueue(injectionQueues_, out, false))
                    return true;
            }
            for (std::uint32_t offset = 1; offset < workerQueues_.size(); ++offset)
            {
                const std::uint32_t victim = (workerIndex + offset) % static_cast<std::uint32_t>(workerQueues_.size());
                WorkerQueue        &queue  = *workerQueues_[victim];
                std::unique_lock    lock(queue.mutex, std::try_to_lock);
                if (lock.owns_lock() && popPriorityQueue(queue.tasks, out, false))
                    return true;
            }
            return false;
        }

        const std::uint8_t startSource = own.nextQueueSource;
        for (std::uint8_t attempt = 0; attempt < 3; ++attempt)
        {
            const std::uint8_t source = static_cast<std::uint8_t>((startSource + attempt) % 3u);
            if (source == 0u)
            {
                std::lock_guard lock(own.mutex);
                if (popPriorityQueue(own.tasks, out, true))
                {
                    own.nextQueueSource = 1u;
                    return true;
                }
            }
            else if (source == 1u)
            {
                std::lock_guard lock(mutex_);
                if (popPriorityQueue(injectionQueues_, out, false))
                {
                    own.nextQueueSource = 2u;
                    return true;
                }
            }
            else
            {
                const std::uint32_t count = static_cast<std::uint32_t>(workerQueues_.size());
                for (std::uint32_t offset = 0; offset < count; ++offset)
                {
                    const std::uint32_t victim = (workerIndex + 1u + own.nextVictim + offset) % count;
                    if (victim == workerIndex)
                        continue;
                    WorkerQueue     &queue = *workerQueues_[victim];
                    std::unique_lock lock(queue.mutex, std::try_to_lock);
                    if (lock.owns_lock() && popPriorityQueue(queue.tasks, out, false))
                    {
                        own.nextVictim      = (own.nextVictim + offset + 1u) % std::max(1u, count - 1u);
                        own.nextQueueSource = 0u;
                        return true;
                    }
                }
            }
        }
        return false;
    }

    // Requires mutex_ held; also takes each worker queue's mutex.
    std::size_t queuedTasksLocked() const
    {
        std::size_t count = 0;
        for (const auto &queue : injectionQueues_)
            count += queue.size();
        for (const auto &worker : workerQueues_)
        {
            std::lock_guard workerLock(worker->mutex);
            for (const auto &queue : worker->tasks)
                count += queue.size();
        }
        return count;
    }

    // Records queue-wait/task-duration metrics; exceptions go to the handler
    // (never escape into worker loops); completion bookkeeping wakes waitIdle.
    void executeTask(WorkItem &item) noexcept
    {
        if (profiler_ != nullptr)
        {
            const auto queueWait
                = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - item.enqueuedAt).count();
            profiler_->record(PerformanceMetric::SchedulerQueueWait,
                static_cast<std::uint64_t>(std::max<std::int64_t>(queueWait, 0)));
        }
        const auto start = Clock::now();
        try
        {
            item.task();
        }
        catch (...)
        {
            ExceptionHandler handler;
            {
                std::lock_guard lock(mutex_);
                handler = exceptionHandler_;
            }
            const std::exception_ptr exception = std::current_exception();
            if (handler)
            {
                try
                {
                    handler(exception);
                }
                catch (...)
                {}
            }
            else
            {
                std::fprintf(stderr, "engine task scheduler: unhandled task exception\n");
            }
        }
        if (profiler_ != nullptr)
        {
            const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count();
            profiler_->record(
                PerformanceMetric::SchedulerTask, static_cast<std::uint64_t>(std::max<std::int64_t>(elapsed, 0)));
        }
        // Completion bookkeeping is a single atomic decrement on the hot path;
        // the mutex is taken only for the rare "last task" notification that
        // wakes waitIdle sleepers (holding it closes the lost-wakeup window).
        completedTasks_.fetch_add(1, std::memory_order_relaxed);
        if (outstandingTasks_.fetch_sub(1, std::memory_order_acq_rel) == 1)
        {
            std::lock_guard lock(mutex_);
            idleCondition_.notify_all();
        }
    }

    // Sleeps on condition_ only when no work exists anywhere; on stopping_ it
    // drains the remainder before exiting. Before parking, the loop spins
    // briefly (pop retry + yield) so a task that is mid-handoff between the
    // pop failure and the sleep is picked up without a condvar round trip -
    // this trims the fork/join wake latency that shows up in frame P99s.
    void workerLoop(std::uint32_t workerIndex)
    {
        currentScheduler_   = this;
        currentWorkerIndex_ = workerIndex;
        const std::uint32_t spinBudget = options_.workerSpinBeforePark;
        for (;;)
        {
            WorkItem item;
            if (tryPopTask(workerIndex, item))
            {
                executeTask(item);
                continue;
            }
            bool resumed = false;
            for (std::uint32_t spin = 0; spin < spinBudget; ++spin)
            {
                if (tryPopTask(workerIndex, item))
                {
                    executeTask(item);
                    resumed = true;
                    break;
                }
                if (queuedApprox_.load(std::memory_order_acquire) == 0)
                    break; // nothing is queued anywhere: park now
                engineSpinWaitHint();
            }
            if (resumed)
                continue;
            std::unique_lock lock(mutex_);
            condition_.wait(lock, [this] { return stopping_ || queuedApprox_.load(std::memory_order_acquire) != 0; });
            if (stopping_ && queuedApprox_.load(std::memory_order_acquire) == 0)
            {
                lock.unlock();
                if (!tryPopTask(workerIndex, item))
                    break;
                executeTask(item);
            }
        }
        currentScheduler_   = nullptr;
        currentWorkerIndex_ = invalidWorkerIndex;
    }

    // lifecycleMutex_ serializes start/stop; mutex_ guards queues and
    // counters; helperCondition_ waits for external tryRunOne callers.
    mutable std::mutex      lifecycleMutex_;
    mutable std::mutex      mutex_;
    std::condition_variable condition_;
    std::condition_variable idleCondition_;
    std::condition_variable helperCondition_;
    std::condition_variable lifecycleCondition_;
    // Submissions from threads that are not scheduler workers.
    std::array<std::deque<WorkItem>, 4>       injectionQueues_;
    std::vector<std::unique_ptr<WorkerQueue>> workerQueues_;
    std::vector<std::thread>                  workers_;
    TaskSchedulerOptions                      options_{};
    ExceptionHandler                          exceptionHandler_;
    PerformanceProfiler                      *profiler_ = nullptr;
    // Outstanding = submitted-but-not-finished (drives backpressure and
    // waitIdle). queuedApprox = sitting-in-a-queue-but-not-popped (drives the
    // worker wake predicate without touching any mutex). Both are atomics so
    // the per-task completion path stays lock-free.
    //
    // Hot counters live on separate cache lines (alignas(64)): every task
    // submission read-modify-writes outstandingTasks_/queuedApprox_/
    // submittedTasks_ and every completion writes outstandingTasks_/
    // completedTasks_ - independent writers sharing a line would pay a
    // coherence round trip per task (false sharing). Padding is the remedy
    // for these measured per-task counters only; the cold counters
    // (rejectedTasks_, agedTasksDequeued_) stay packed together.
    alignas(64) std::atomic<std::size_t>   outstandingTasks_{0};
    alignas(64) std::atomic<std::size_t>   queuedApprox_{0};
    alignas(64) std::atomic<std::uint64_t> submittedTasks_{0};
    alignas(64) std::atomic<std::uint64_t> completedTasks_{0};
    std::atomic<std::uint64_t>             rejectedTasks_{0};
    std::atomic<std::uint64_t>             agedTasksDequeued_{0};
    // Lock-free worker-count hint published by start()/stop() (0 = not
    // running / stopping); see runningWorkerCountHint().
    std::atomic<std::uint32_t>             runningWorkerCount_{0};
    std::size_t                            activeExternalHelpers_ = 0;
    bool                                      running_        = false;
    bool                                      accepting_      = false;
    bool                                      stopping_       = false;
    bool                                      stopInProgress_ = false;

    // Thread-local identity: lets schedule() pick the local deque and lets
    // stop() refuse self-termination from inside a task.
    static inline thread_local TaskScheduler *currentScheduler_   = nullptr;
    static inline thread_local std::uint32_t  currentWorkerIndex_ = invalidWorkerIndex;
};

inline void TaskGroup::complete() noexcept
{
    // The mutex must be taken BEFORE the decrement: wait() may observe
    // outstanding_ == 0 through the plain atomic (spin path) and return,
    // destroying the group. Locking first guarantees the final completer's
    // critical section ends before wait()'s own final lock (exception read)
    // can proceed, so the group always outlives every complete() call.
    // Decrementing first and locking after was tried and is a use-after-free
    // (caught by TSan as an invalid-mutex unlock).
    std::lock_guard   lock(mutex_);
    const std::size_t previous = outstanding_.fetch_sub(1, std::memory_order_acq_rel);
    if (previous == 1)
        condition_.notify_all();
}

inline void TaskGroup::setException(std::exception_ptr exception) noexcept
{
    std::lock_guard lock(mutex_);
    if (!exception_)
        exception_ = std::move(exception);
}

// Loop because helpers can complete tasks between the atomic check and
// the condition wait; the exception is cleared so the group is reusable.
inline void TaskGroup::wait(TaskScheduler &scheduler)
{
    while (outstanding_.load(std::memory_order_acquire) != 0)
    {
        if (scheduler.tryRunOne())
            continue;
        std::unique_lock lock(mutex_);
        condition_.wait(lock, [this] { return outstanding_.load(std::memory_order_acquire) == 0; });
    }
    std::exception_ptr exception;
    {
        std::lock_guard lock(mutex_);
        exception  = exception_;
        exception_ = nullptr;
    }
    if (exception)
        std::rethrow_exception(exception);
}

// DAG of tasks with dependency-driven fan-out; execute() blocks the
// caller (helping run tasks) until every node finished.
class TaskGraph final
{
public:
    using Task = TaskScheduler::Task;

    // Returns the node id used for addDependency.
    std::uint32_t addTask(Task task)
    {
        if (!task)
            throw std::invalid_argument("task graph nodes require a callable");
        if (nodes_.size() >= UINT32_MAX)
            throw std::length_error("task graph node limit exceeded");
        nodes_.push_back(Node{std::move(task), {}, 0});
        return static_cast<std::uint32_t>(nodes_.size() - 1u);
    }

    // prerequisite completes before dependent starts; idempotent, self- or
    // out-of-range edges return false (cycles fail at execute()).
    bool addDependency(std::uint32_t prerequisite, std::uint32_t dependent)
    {
        if (prerequisite >= nodes_.size() || dependent >= nodes_.size() || prerequisite == dependent)
            return false;
        auto &dependents = nodes_[prerequisite].dependents;
        if (std::find(dependents.begin(), dependents.end(), dependent) != dependents.end())
            return true;
        dependents.push_back(dependent);
        ++nodes_[dependent].dependencyCount;
        return true;
    }

    [[nodiscard]] std::size_t taskCount() const noexcept
    {
        return nodes_.size();
    }

    // Clears all nodes so the graph object can be rebuilt/reused without
    // releasing its capacity; used by the render graph to keep one graph per
    // cached plan alive across frames instead of rebuilding it every frame.
    void reset() noexcept
    {
        nodes_.clear();
    }

    // Validates acyclicity upfront (throws std::logic_error); a rejected
    // schedule or failing node fails the whole graph via the shared state.
    void execute(TaskScheduler &scheduler) const
    {
        if (nodes_.empty())
            return;
        validateAcyclic();
        // shared_ptr keeps the state alive inside scheduled lambdas after
        // execute()'s locals are gone.
        struct ExecutionState
        {
            const TaskGraph                              *graph     = nullptr;
            TaskScheduler                                *scheduler = nullptr;
            TaskGroup                                    *group     = nullptr;
            std::unique_ptr<std::atomic<std::uint32_t>[]> remaining;
            std::atomic<bool>                             failed{false};
        };
        auto      state = std::make_shared<ExecutionState>();
        TaskGroup group;
        state->graph     = this;
        state->scheduler = &scheduler;
        state->group     = &group;
        state->remaining = std::make_unique<std::atomic<std::uint32_t>[]>(nodes_.size());
        for (std::size_t index = 0; index < nodes_.size(); ++index)
            state->remaining[index].store(nodes_[index].dependencyCount, std::memory_order_relaxed);

        for (std::uint32_t index = 0; index < nodes_.size(); ++index)
            if (nodes_[index].dependencyCount == 0)
                enqueueNode(state, index);
        group.wait(scheduler);
        if (state->failed.load(std::memory_order_acquire))
            throw std::runtime_error("task graph execution was rejected by the scheduler");
    }

private:
    // dependents: adjacency; dependencyCount: unmet prerequisite count.
    struct Node
    {
        Task                       task;
        std::vector<std::uint32_t> dependents;
        std::uint32_t              dependencyCount = 0;
    };

    // Kahn's algorithm; visited < node count means a cycle.
    void validateAcyclic() const
    {
        std::vector<std::uint32_t> remaining(nodes_.size());
        std::deque<std::uint32_t>  ready;
        for (std::uint32_t index = 0; index < nodes_.size(); ++index)
        {
            remaining[index] = nodes_[index].dependencyCount;
            if (remaining[index] == 0)
                ready.push_back(index);
        }
        std::size_t visited = 0;
        while (!ready.empty())
        {
            const auto current = ready.front();
            ready.pop_front();
            ++visited;
            for (const auto dependent : nodes_[current].dependents)
                if (--remaining[dependent] == 0)
                    ready.push_back(dependent);
        }
        if (visited != nodes_.size())
            throw std::logic_error("task graph contains a dependency cycle");
    }

    // Fires a node whose prerequisites hit zero; its completion decrements
    // dependents and enqueues newly ready nodes (atomic remaining counters).
    template <typename State>
    static void enqueueNode(const std::shared_ptr<State> &state, std::uint32_t index)
    {
        if (state->failed.load(std::memory_order_acquire))
            return;
        const Node &node     = state->graph->nodes_[index];
        const bool  accepted = state->scheduler->schedule(*state->group,
            [state, index, task = node.task]
            {
                if (state->failed.load(std::memory_order_acquire))
                    return;
                try
                {
                    task();
                }
                catch (...)
                {
                    state->failed.store(true, std::memory_order_release);
                    state->group->setException(std::current_exception());
                    return;
                }
                for (const std::uint32_t dependent : state->graph->nodes_[index].dependents)
                    if (state->remaining[dependent].fetch_sub(1, std::memory_order_acq_rel) == 1)
                        enqueueNode(state, dependent);
            });
        if (!accepted)
        {
            state->failed.store(true, std::memory_order_release);
            state->group->setException(
                std::make_exception_ptr(std::runtime_error("task scheduler rejected a task graph node")));
        }
    }

    std::vector<Node> nodes_;
};

} // namespace engine

#endif
