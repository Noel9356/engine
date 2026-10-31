#ifndef ENGINE_KERNEL_ASYNC_TASK_DISPATCHER_HPP
#define ENGINE_KERNEL_ASYNC_TASK_DISPATCHER_HPP

#include "./parallel_for.hpp"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <functional>
#include <mutex>
#include <vector>

namespace engine
{

class AsyncTaskDispatcher final
{
public:
    AsyncTaskDispatcher() = default;

    AsyncTaskDispatcher(const AsyncTaskDispatcher &) = delete;

    AsyncTaskDispatcher &operator=(const AsyncTaskDispatcher &) = delete;

    ~AsyncTaskDispatcher()
    {
        stop();
    }

    void start(std::size_t workerCount = defaultWorkerCount())
    {
        if (running_.load())
        {
            return;
        }

        const std::size_t clampedWorkerCount = std::clamp<std::size_t>(workerCount, 1, 32);
        running_.store(true);
        workers_.reserve(clampedWorkerCount);
        for (std::size_t index = 0; index < clampedWorkerCount; ++index)
        {
            workers_.emplace_back(&AsyncTaskDispatcher::workerLoop, this);
        }
    }

    void stop()
    {
        if (!running_.exchange(false))
        {
            return;
        }

        wakeCondition_.notify_all();
        for (std::thread &worker : workers_)
        {
            if (worker.joinable())
            {
                worker.join();
            }
        }
        workers_.clear();
    }

    void post(std::function<void()> work)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            workQueue_.push_back(std::move(work));
            if (sleepingWorkers_.load(std::memory_order_relaxed) > 0) [[unlikely]]
            {
                wakeCondition_.notify_one();
            }
        }
    }

    void postWithCompletion(
        std::function<void()> work,
        std::function<void()> completion
    )
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            workQueue_.push_back([this, work = std::move(work), completion = std::move(completion)]() {
                try
                {
                    work();
                } catch (const std::exception &error)
                {
                    std::fprintf(stderr, "async dispatcher: task failed: %s\n", error.what());
                    return;
                } catch (...)
                {
                    std::fprintf(stderr, "async dispatcher: task failed with an unknown error\n");
                    return;
                }
                std::lock_guard<std::mutex> completionLock(mutex_);
                completionQueue_.push_back(std::move(completion));
                pendingCompletions_.fetch_add(1, std::memory_order_relaxed);
            });

            if (sleepingWorkers_.load(std::memory_order_relaxed) > 0) [[unlikely]]
            {
                wakeCondition_.notify_one();
            }
        }
    }

    void drainCompletedTasks()
    {
        if (pendingCompletions_.load(std::memory_order_relaxed) == 0)
        {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            drainBuffer_.swap(completionQueue_);
            pendingCompletions_.store(0, std::memory_order_relaxed);
        }
        for (std::function<void()> &completion : drainBuffer_)
        {
            completion();
        }
        drainBuffer_.clear();
    }

    std::size_t pendingWorkCount() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return workQueue_.size();
    }

private:
    void workerLoop()
    {
        while (true)
        {
            std::function<void()> work;
            {
                std::unique_lock<std::mutex> lock(mutex_);

                sleepingWorkers_.fetch_add(1, std::memory_order_relaxed);
                wakeCondition_.wait(lock, [this]() { return !running_.load() || !workQueue_.empty(); });
                sleepingWorkers_.fetch_sub(1, std::memory_order_relaxed);
                if (!running_.load() && workQueue_.empty())
                {
                    return;
                }
                if (workQueue_.empty())
                {
                    continue;
                }
                work = std::move(workQueue_.front());
                workQueue_.pop_front();
            }

            try
            {
                work();
            } catch (const std::exception &error)
            {
                std::fprintf(stderr, "async dispatcher: task failed: %s\n", error.what());
            } catch (...)
            {
                std::fprintf(stderr, "async dispatcher: task failed with an unknown error\n");
            }
        }
    }

    mutable std::mutex                 mutex_;
    std::condition_variable            wakeCondition_;
    std::deque<std::function<void()>>  workQueue_;
    std::vector<std::function<void()>> completionQueue_;

    std::vector<std::function<void()>> drainBuffer_;
    std::vector<std::thread>           workers_;
    std::atomic<bool>                  running_{false};

    std::atomic<std::size_t> pendingCompletions_{0};

    std::atomic<std::size_t> sleepingWorkers_{0};
};

} // namespace engine

#endif
