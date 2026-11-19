// Implementation of the draw-list scheduler (draw_scheduler.hpp).

#include "draw_scheduler.hpp"

#include <algorithm>
#include <new>

namespace engine
{
namespace renderer
{

std::span<const RenderDrawCommand> DrawScheduler::orderedCommands(
    const RenderDrawCommand *commands, std::uint32_t commandCount, const RendererSettings &settings,
    PerformanceProfiler *profiler)
{
    if (!settings.drawSortEnabled || commandCount < settings.drawSortMinimumCommands
        || static_cast<std::size_t>(commandCount) > sortedCommandsScratch.max_size())
    {
        sortSignatureValid = false;
        return std::span<const RenderDrawCommand>(commands, commandCount);
    }
    try
    {
        // Percentile-relevant cost: the cache-hit gather is cheap, the
        // rebuild path sorts. Recording both under one metric shows
        // how often and how long the draw-order cache misses.
        const ScopedProfileTimer sortTimer(profiler, PerformanceMetric::RenderDrawSort);
        const std::uint64_t      signature = renderDrawCommandSortSignature(
            std::span<const RenderDrawCommand>(commands, commandCount), settings.drawSortMode);
        if (sortSignatureValid && sortSignature == signature
            && sortPermutation.size() == static_cast<std::size_t>(commandCount))
        {
            // Same key multiset in the same input order as the cached
            // sort: gather through the cached permutation. Matrix data
            // is dereferenced at draw time, so only the pointers must
            // be current - which the gather guarantees.
            sortedCommandsScratch.resize(commandCount);
            for (std::uint32_t index = 0; index < commandCount; ++index)
            {
                sortedCommandsScratch[index] = commands[sortPermutation[index]];
            }
        }
        else
        {
            // Sort {sort key, original index} pairs and gather through
            // the resulting permutation: O(n log n) with no separate
            // origin reconstruction, so the cache-rebuild frame (key
            // set changed: streaming, spawns, material edits) no
            // longer pays the previous O(n^2) search over consumed
            // commands. Ties resolve to the lower original index,
            // which makes the rebuild deterministic and the cached
            // gather identical to a fresh sort.
            sortPermutation.resize(commandCount);
            sortRenderDrawCommandPermutation(std::span<const RenderDrawCommand>(commands, commandCount),
                settings.drawSortMode, std::span<std::uint32_t>(sortPermutation));
            sortedCommandsScratch.resize(commandCount);
            for (std::uint32_t index = 0; index < commandCount; ++index)
            {
                sortedCommandsScratch[index] = commands[sortPermutation[index]];
            }
            // Record the permutation this sorted order came from so
            // later frames with the same keys can skip the sort.
            sortSignature      = signature;
            sortSignatureValid = true;
        }
        return std::span<const RenderDrawCommand>(sortedCommandsScratch.data(), commandCount);
    }
    catch (const std::bad_alloc &)
    {
        sortSignatureValid = false;
        return std::span<const RenderDrawCommand>(commands, commandCount);
    }
}

void DrawScheduler::submit(RenderBackend &backend, bool backendSupportsInstancing, const RenderDrawCommand *commands,
    std::uint32_t commandCount, const RendererSettings &settings, PerformanceProfiler *profiler)
{
    const std::span<const RenderDrawCommand> ordered
        = orderedCommands(commands, commandCount, settings, profiler);

    const bool useInstancing = settings.instanceBatchingEnabled && backendSupportsInstancing
                            && commandCount >= settings.instanceBatchMinimumCount;
    if (!useInstancing)
    {
        // One backend call for the whole frame's mesh draws. The Vulkan
        // backend's default implementation replays the items through
        // drawMesh; the headless backend replaces the replay with a
        // batched tile-parallel pass that preserves per-pixel submission
        // order exactly while spreading the pixel work across every
        // worker (see RenderBackend::drawMeshBatch). Only the scratch
        // build can throw here (nothing has been drawn yet); the backend
        // call itself is allocation-failure safe and falls back to the
        // per-draw paths internally.
        bool batchReady = false;
        try
        {
            batchItemsScratch.clear();
            batchItemsScratch.reserve(ordered.size());
            for (const RenderDrawCommand &command : ordered)
            {
                if (command.modelMatrix == nullptr || command.mesh == invalidMesh)
                {
                    continue;
                }
                batchItemsScratch.push_back(
                    RenderBackend::MeshBatchItem{command.mesh, command.modelMatrix, command.material});
            }
            batchReady = true;
        }
        catch (const std::bad_alloc &)
        {
            batchItemsScratch.clear();
        }
        if (batchReady)
        {
            backend.drawMeshBatch(batchItemsScratch.data(), static_cast<std::uint32_t>(batchItemsScratch.size()));
            return;
        }
        // The scratch copy failed; replay through the per-command path
        // instead (identical output, just unbatched).
        for (const RenderDrawCommand &command : ordered)
        {
            if (command.modelMatrix == nullptr || command.mesh == invalidMesh)
            {
                continue;
            }
            backend.drawMesh(command.mesh, command.modelMatrix, effectiveRenderMaterial(command.material));
        }
        return;
    }

    std::uint32_t index = 0;
    while (index < ordered.size())
    {
        const RenderDrawCommand &first = ordered[index];
        if (first.modelMatrix == nullptr || first.mesh == invalidMesh)
        {
            ++index;
            continue;
        }

        // Same mesh + same material folds into one instanced draw; a run
        // that is too small, overflows the scratch, or fails to allocate
        // falls back to per-command drawMesh calls.
        const MeshMaterial *const material = &effectiveRenderMaterial(first.material);
        std::uint32_t             end      = index + 1u;
        while (end < ordered.size())
        {
            const RenderDrawCommand &next = ordered[end];
            if (next.modelMatrix == nullptr || next.mesh != first.mesh
                || (next.material != first.material
                    && compareRenderMaterialValues(
                        effectiveRenderMaterial(next.material), effectiveRenderMaterial(first.material))
                        != 0))
            {
                break;
            }
            ++end;
        }

        const auto replayRun = [&backend, &ordered](std::uint32_t from, std::uint32_t to)
        {
            for (std::uint32_t fallback = from; fallback < to; ++fallback)
            {
                const RenderDrawCommand &command = ordered[fallback];
                backend.drawMesh(command.mesh, command.modelMatrix, effectiveRenderMaterial(command.material));
            }
        };

        const std::size_t instanceCount = static_cast<std::size_t>(end - index);
        if (instanceCount < settings.instanceBatchMinimumCount)
        {
            replayRun(index, end);
            index = end;
            continue;
        }

        if (instanceCount > instanceMatricesScratch.max_size() / 16u)
        {
            replayRun(index, end);
            index = end;
            continue;
        }

        try
        {
            instanceMatricesScratch.resize(instanceCount * 16u);
        }
        catch (const std::bad_alloc &)
        {
            replayRun(index, end);
            index = end;
            continue;
        }

        float *destination = instanceMatricesScratch.data();
        for (std::uint32_t source = index; source < end; ++source)
        {
            std::copy_n(ordered[source].modelMatrix, 16u, destination);
            destination += 16u;
        }
        backend.drawMeshInstances(first.mesh,
            instanceMatricesScratch.data(),
            static_cast<std::uint32_t>(instanceCount),
            *material);
        index = end;
    }
}

} // namespace renderer
} // namespace engine
