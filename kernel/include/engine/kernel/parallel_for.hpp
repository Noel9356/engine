#ifndef ENGINE_KERNEL_PARALLEL_FOR_HPP
#define ENGINE_KERNEL_PARALLEL_FOR_HPP

#include <algorithm>
#include <cstddef>
#include <exception>
#include <functional>
#include <thread>
#include <vector>

namespace engine
{

inline std::size_t defaultWorkerCount()
{
    const unsigned int hardware = std::thread::hardware_concurrency();
    return std::clamp<std::size_t>(hardware == 0 ? 1 : static_cast<std::size_t>(hardware), 1, 16);
}

template <typename Body>
inline void parallelForDispatch(
    std::size_t itemCount,
    Body       &chunkBody
)
{
    if (itemCount == 0)
    {
        return;
    }
    const std::size_t workerCount = std::min(defaultWorkerCount(), itemCount);
    if (workerCount <= 1)
    {
        chunkBody(0, itemCount);
        return;
    }

    const std::size_t chunkSize = itemCount / workerCount;
    const std::size_t remainder = itemCount % workerCount;

    std::vector<std::exception_ptr> failures(workerCount, nullptr);
    std::vector<std::thread>        workers;
    workers.reserve(workerCount - 1);

    const auto runChunk = [&chunkBody, &failures](std::size_t workerIndex, std::size_t begin, std::size_t end) {
        try
        {
            chunkBody(begin, end);
        } catch (...)
        {
            failures[workerIndex] = std::current_exception();
        }
    };

    for (std::size_t workerIndex = 1; workerIndex < workerCount; ++workerIndex)
    {
        const std::size_t begin = workerIndex * chunkSize + std::min(workerIndex, remainder);
        const std::size_t end   = begin + chunkSize + (workerIndex < remainder ? 1 : 0);
        workers.emplace_back([&runChunk, workerIndex, begin, end]() { runChunk(workerIndex, begin, end); });
    }
    runChunk(0, 0, chunkSize + (remainder > 0 ? 1 : 0));

    for (std::thread &worker : workers)
    {
        worker.join();
    }
    for (std::exception_ptr &failure : failures)
    {
        if (failure != nullptr)
        {
            std::rethrow_exception(failure);
        }
    }
}

template <typename Body>
inline void parallelFor(
    std::size_t itemCount,
    Body      &&chunkBody
)
{
    parallelForDispatch(itemCount, chunkBody);
}

inline void parallelFor(
    std::size_t itemCount,
    const std::function<void(
        std::size_t,
        std::size_t
    )>         &chunkBody
)
{
    if (chunkBody == nullptr)
    {
        return;
    }
    parallelForDispatch(itemCount, chunkBody);
}

} // namespace engine

#endif
