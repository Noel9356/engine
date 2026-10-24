#ifndef ENGINE_KERNEL_ASYNC_TASK_DISPATCHER_HPP
#define ENGINE_KERNEL_ASYNC_TASK_DISPATCHER_HPP

// Off-main-thread work with main-thread completion, as an opt-in utility.
//
// The engine's threading contract is deliberate and documented
// (docs/architecture.md, docs/robustness.md invariant 38): the event bus is
// single threaded, the frame loop is serialized, and nothing shares GPU
// resources across threads. That contract is exactly why this utility exists
// as a KERNEL utility rather than a bus feature: modules occasionally need
// blocking work OFF the frame path (filesystem polling, file IO, decode
// steps), and the safe shape for that is always the same -
//
//   post(work)                 work runs on the worker thread
//   postWithCompletion(work,   work runs on the worker; completion runs on
//     completion)              the MAIN thread at the next drain() call
//   drainCompletedTasks()      the main thread (frame path) drains
//
// - so results re-enter the single-threaded world at a well-defined point
// and no lock but the dispatcher's own queue mutex ever exists. The bus API
// is untouched (the architecture doc reserved this: "a future delivery
// module can add queues without touching the channel API").
//
// One dispatcher, one worker thread: enough for IO-shaped work, no pool
// tuning surface. A module that genuinely needs parallelism across cores
// (image decoding at scale) is a different, explicitly-scoped feature.

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace engine
{

class AsyncTaskDispatcher final
{
public:
    AsyncTaskDispatcher() = default;

    AsyncTaskDispatcher(const AsyncTaskDispatcher &)            = delete;
    AsyncTaskDispatcher &operator=(const AsyncTaskDispatcher &) = delete;

    ~AsyncTaskDispatcher()
    {
        stop();
    }

    // Starts the worker thread. Calling start twice without stop is a
    // contract breach (one dispatcher, one worker); start after stop is
    // legal (a module that re-opens its watcher).
    void start()
    {
        if (running_.load())
        {
            return;
        }
        mainThreadId_       = std::this_thread::get_id();
        running_.store(true);
        worker_ = std::thread(&AsyncTaskDispatcher::workerLoop, this);
    }

    // Stops the worker: pending work still runs (the worker finishes its
    // queue), completions posted but not drained are DROPPED (the main
    // thread chose to stop draining; keeping them would resurrect stale
    // state at an unpredictable later point). Safe to call twice.
    void stop()
    {
        if (!running_.exchange(false))
        {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            workQueue_.clear();
        }
        wakeCondition_.notify_all();
        if (worker_.joinable())
        {
            worker_.join();
        }
    }

    // Posts one unit of work to the worker thread. Work must not touch the
    // bus, GPU resources, or anything the frame loop owns - it should
    // compute and return (captured) values.
    void post(
        std::function<void()> work
    )
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            workQueue_.push_back(std::move(work));
        }
        wakeCondition_.notify_one();
    }

    // Posts work whose RESULT re-enters the main thread: after the work
    // lambda returns on the worker, the completion lambda is queued to run
    // during the next drainCompletedTasks() on the main thread.
    void postWithCompletion(
        std::function<void()> work, std::function<void()> completion
    )
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            workQueue_.push_back(
                [this, work = std::move(work), completion = std::move(completion)]()
                {
                    work();
                    std::lock_guard<std::mutex> completionLock(mutex_);
                    completionQueue_.push_back(std::move(completion));
                }
            );
        }
        wakeCondition_.notify_one();
    }

    // Drains the completion queue on the main thread. Call from the frame
    // path (a FrameBeginEvent handler is the conventional point). Calling
    // from a non-main thread is a contract breach - completions may publish
    // events, and the bus is single threaded.
    void drainCompletedTasks()
    {
        std::vector<std::function<void()>> ready;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ready.swap(completionQueue_);
        }
        for (std::function<void()> &completion : ready)
        {
            completion();
        }
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
                wakeCondition_.wait(
                    lock, [this]() { return !running_.load() || !workQueue_.empty(); }
                );
                if (!running_.load() && workQueue_.empty())
                {
                    return;
                }
                if (workQueue_.empty())
                {
                    continue; // stop() cleared the queue while we waited
                }
                work = std::move(workQueue_.front());
                workQueue_.pop_front();
            }
            work();
        }
    }

    mutable std::mutex                  mutex_;
    std::condition_variable             wakeCondition_;
    std::deque<std::function<void()>>   workQueue_;
    std::vector<std::function<void()>>  completionQueue_;
    std::thread                         worker_;
    std::atomic<bool>                   running_{false};
    std::thread::id                     mainThreadId_{};
};

} // namespace engine

#endif // ENGINE_KERNEL_ASYNC_TASK_DISPATCHER_HPP
