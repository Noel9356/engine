// Draw-list scheduling for drawMeshes: optional sorting with a cached
// permutation, the backend batch submission, and the Vulkan-only instance
// fold. One scheduler lives per renderer; its scratch buffers persist
// between frames, so a steady frame pays no allocation on this path.
//
// The sort cache works because static geometry redraws the same key set
// every frame: the sorted ORDER is cached as a permutation validated by a
// signature over the mesh/material keys (renderDrawCommandSortSignature),
// and only the current frame's matrix pointers are gathered - the
// per-frame copy + std::sort with the field-wise material comparator
// disappears.

#pragma once

#include "render_backend.hpp"
#include "renderer_settings.hpp"

#include <engine/kernel/profiling.hpp>
#include <engine/renderer/capabilities.hpp>
#include <engine/renderer/draw_sort.hpp>

#include <cstdint>
#include <span>
#include <vector>

namespace engine
{
namespace renderer
{

class DrawScheduler
{
public:
    // Orders and submits one frame's mesh-draw list:
    // 1. optional sorting (settings.drawSortMode, from
    //    settings.drawSortMinimumCommands commands up), through the cached
    //    permutation when the key signature matches;
    // 2. the whole list as one RenderBackend::drawMeshBatch call (the
    //    headless backend overrides it with the batched tile-parallel
    //    pass; the Vulkan default replays the items through drawMesh); or,
    //    when the backend supports instancing (Vulkan) and the settings
    //    enable it, runs of equal mesh + material fold into
    //    drawMeshInstances calls, with runs smaller than
    //    settings.instanceBatchMinimumCount falling back to single draws.
    // backendSupportsInstancing gates the fold to backends whose
    // drawMeshInstances is a real instanced draw.
    void submit(RenderBackend &backend, bool backendSupportsInstancing, const RenderDrawCommand *commands,
        std::uint32_t commandCount, const RendererSettings &settings, PerformanceProfiler *profiler);

private:
    // Sorts the commands (or gathers through the cached permutation) and
    // returns the ordered view; on any allocation failure the input order
    // is used unchanged.
    std::span<const RenderDrawCommand> orderedCommands(
        const RenderDrawCommand *commands, std::uint32_t commandCount, const RendererSettings &settings,
        PerformanceProfiler *profiler);

    // The filtered mesh-draw list handed to RenderBackend::drawMeshBatch
    // once per frame; capacity persists across frames.
    std::vector<RenderBackend::MeshBatchItem> batchItemsScratch{};

    // Sort cache: the permutation plus the signature it was built from.
    std::vector<RenderDrawCommand> sortedCommandsScratch{};
    std::vector<std::uint32_t>     sortPermutation{};
    std::uint64_t                  sortSignature      = 0;
    bool                           sortSignatureValid = false;

    // Packed instance matrices for the current fold run.
    std::vector<float> instanceMatricesScratch{};
};

} // namespace renderer
} // namespace engine
