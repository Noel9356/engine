// Instanced-draw batching for the Vulkan backend.
//
// RenderInstanceBatchCache folds a list of draw commands into (mesh,
// material) runs plus a packed column-major instance-matrix stream, and can
// refresh the matrices of just the commands that changed between frames.

#ifndef ENGINE_RENDER_DRAW_BATCH_HPP
#define ENGINE_RENDER_DRAW_BATCH_HPP

#include <engine/render/draw_sort.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <span>
#include <utility>
#include <vector>

namespace engine::render
{

// One batched draw: instanceCount consecutive instances starting at
// firstInstance in the packed instance-matrix stream produced by the cache.
struct RenderInstanceBatch
{
    MeshHandle          mesh          = invalidMesh;
    const MeshMaterial *material      = nullptr;
    std::uint32_t       firstInstance = 0;
    std::uint32_t       instanceCount = 0;
};

// Groups draw commands into batches. The full rebuild sorts commands by
// (mesh, material) — commands with a null mesh/matrix are excluded — and
// records a per-command signature so refreshTransforms can verify that the
// caller is passing the same command list (same mesh/material/inclusion
// pattern) before touching the cached instance stream.
//
// All methods are exception-safe: on allocation failure the cache clears
// itself and reports false. Not thread-safe; treat as frame-local data.
class RenderInstanceBatchCache final
{
public:
    // Rebuilds the batch layout from scratch. Returns false if the command
    // count exceeds uint32 or an allocation fails.
    bool rebuild(
        std::span<const RenderDrawCommand> commands, RenderDrawSortMode sortMode = RenderDrawSortMode::MaterialThenMesh)
    {
        clear();
        if (commands.size() > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()))
            return false;
        try
        {
            std::vector<std::uint32_t> order;
            order.reserve(commands.size());
            signature_.reserve(commands.size());
            commandCount_ = commands.size();
            for (std::uint32_t index = 0; index < commands.size(); ++index)
            {
                const RenderDrawCommand &command  = commands[index];
                const bool               included = command.mesh != invalidMesh && command.modelMatrix != nullptr;
                signature_.push_back({command.mesh, command.material, included});
                if (included)
                    order.push_back(index);
            }
            std::sort(order.begin(),
                order.end(),
                [&](std::uint32_t lhsIndex, std::uint32_t rhsIndex)
                {
                    const RenderDrawCommand &lhs = commands[lhsIndex];
                    const RenderDrawCommand &rhs = commands[rhsIndex];
                    const auto materialLess      = [](const MeshMaterial *left, const MeshMaterial *right) noexcept
                    { return std::less<const MeshMaterial *>{}(left, right); };
                    if (sortMode == RenderDrawSortMode::MeshThenMaterial)
                    {
                        if (lhs.mesh != rhs.mesh)
                            return lhs.mesh < rhs.mesh;
                        return materialLess(lhs.material, rhs.material);
                    }
                    if (lhs.material != rhs.material)
                        return materialLess(lhs.material, rhs.material);
                    if (lhs.mesh != rhs.mesh)
                        return lhs.mesh < rhs.mesh;
                    return lhsIndex < rhsIndex;
                });

            sourceIndices_ = std::move(order);
            sourceToInstance_.assign(commandCount_, invalidInstanceIndex);
            for (std::uint32_t instance = 0; instance < sourceIndices_.size(); ++instance)
                sourceToInstance_[sourceIndices_[instance]] = instance;
            batches_.reserve(sourceIndices_.size());
            for (const std::uint32_t index : sourceIndices_)
            {
                const RenderDrawCommand &command   = commands[index];
                const bool               sameBatch = !batches_.empty() && batches_.back().mesh == command.mesh
                                                  && batches_.back().material == command.material;
                if (!sameBatch)
                {
                    batches_.push_back(
                        {command.mesh,
                            command.material,
                            static_cast<std::uint32_t>(batches_.empty() ? 0u : sourceIndexCount_),
                            0u});
                }
                ++batches_.back().instanceCount;
                ++sourceIndexCount_;
            }
            valid_ = true;
            return true;
        }
        catch (...)
        {
            clear();
            return false;
        }
    }

    // Copies every included command's matrix into outColumnMajorMatrices in
    // instance order (16 floats per instance). Fails unless the command list
    // still matches the one rebuild() saw (same size and same per-command
    // mesh/material/inclusion signature).
    bool refreshTransforms(
        std::span<const RenderDrawCommand> commands, std::vector<float> &outColumnMajorMatrices) const
    {
        outColumnMajorMatrices.clear();
        if (!valid_ || commands.size() != commandCount_)
            return false;
        for (std::size_t index = 0; index < commands.size(); ++index)
        {
            const RenderDrawCommand &command   = commands[index];
            const Signature         &signature = signature_[index];
            const bool               included  = command.mesh != invalidMesh && command.modelMatrix != nullptr;
            if (command.mesh != signature.mesh || command.material != signature.material
                || included != signature.included)
                return false;
        }
        if (sourceIndices_.size() > outColumnMajorMatrices.max_size() / 16u)
            return false;
        try
        {
            outColumnMajorMatrices.resize(sourceIndices_.size() * 16u);
            for (std::size_t instance = 0; instance < sourceIndices_.size(); ++instance)
            {
                const float *matrix = commands[sourceIndices_[instance]].modelMatrix;
                std::copy_n(matrix, 16u, outColumnMajorMatrices.data() + instance * 16u);
            }
            return true;
        }
        catch (...)
        {
            outColumnMajorMatrices.clear();
            return false;
        }
    }

    // Updates only the instances whose source commands are listed in
    // changedSourceIndices, writing into an existing matrix stream sized
    // exactly instanceCount() * 16. noexcept: validates everything up front.
    bool refreshChangedTransforms(std::span<const RenderDrawCommand> commands,
        std::span<const std::uint32_t> changedSourceIndices, std::span<float> inOutColumnMajorMatrices) const noexcept
    {
        if (!valid_ || commands.size() != commandCount_
            || sourceIndices_.size() > std::numeric_limits<std::size_t>::max() / 16u
            || inOutColumnMajorMatrices.size() != sourceIndices_.size() * 16u)
            return false;
        for (const std::uint32_t sourceIndex : changedSourceIndices)
        {
            if (sourceIndex >= commandCount_)
                return false;
            const RenderDrawCommand &command   = commands[sourceIndex];
            const Signature         &signature = signature_[sourceIndex];
            const bool               included  = command.mesh != invalidMesh && command.modelMatrix != nullptr;
            if (command.mesh != signature.mesh || command.material != signature.material
                || included != signature.included
                || (included && sourceToInstance_[sourceIndex] == invalidInstanceIndex))
                return false;
        }
        for (const std::uint32_t sourceIndex : changedSourceIndices)
        {
            const std::uint32_t instanceIndex = sourceToInstance_[sourceIndex];
            if (instanceIndex == invalidInstanceIndex)
                continue;
            std::copy_n(commands[sourceIndex].modelMatrix,
                16u,
                inOutColumnMajorMatrices.data() + static_cast<std::size_t>(instanceIndex) * 16u);
        }
        return true;
    }

    // Maps a command index to its packed instance index, or
    // invalidInstanceIndex when the command was not included in a batch.
    [[nodiscard]] std::uint32_t instanceIndexForSource(std::uint32_t sourceIndex) const noexcept
    {
        return sourceIndex < sourceToInstance_.size() ? sourceToInstance_[sourceIndex] : invalidInstanceIndex;
    }

    static constexpr std::uint32_t invalidInstanceIndex = std::numeric_limits<std::uint32_t>::max();

    // refreshTransforms + a copy of the batches: the one-shot helper for
    // callers that rebuild everything each frame.
    bool update(std::span<const RenderDrawCommand> commands, std::vector<RenderInstanceBatch> &outBatches,
        std::vector<float> &outColumnMajorMatrices) const
    {
        outBatches.clear();
        if (!refreshTransforms(commands, outColumnMajorMatrices))
            return false;
        try
        {
            outBatches = batches_;
            return true;
        }
        catch (...)
        {
            outBatches.clear();
            outColumnMajorMatrices.clear();
            return false;
        }
    }

    [[nodiscard]] std::span<const RenderInstanceBatch> batches() const noexcept
    {
        return valid_ ? std::span<const RenderInstanceBatch>(batches_) : std::span<const RenderInstanceBatch>{};
    }

    void clear() noexcept
    {
        signature_.clear();
        sourceIndices_.clear();
        sourceToInstance_.clear();
        batches_.clear();
        commandCount_     = 0;
        sourceIndexCount_ = 0;
        valid_            = false;
    }

    [[nodiscard]] bool valid() const noexcept
    {
        return valid_;
    }
    [[nodiscard]] std::size_t commandCount() const noexcept
    {
        return commandCount_;
    }
    [[nodiscard]] std::size_t instanceCount() const noexcept
    {
        return sourceIndices_.size();
    }
    [[nodiscard]] std::size_t batchCount() const noexcept
    {
        return batches_.size();
    }

private:
    // Per-command identity recorded at rebuild time; refresh* compares it
    // against the incoming commands to reject stale/reshuffled lists.
    struct Signature
    {
        MeshHandle          mesh     = invalidMesh;
        const MeshMaterial *material = nullptr;
        bool                included = false;
    };

    std::vector<Signature>           signature_;
    std::vector<std::uint32_t>       sourceIndices_;
    std::vector<std::uint32_t>       sourceToInstance_;
    std::vector<RenderInstanceBatch> batches_;
    std::size_t                      commandCount_     = 0;
    std::size_t                      sourceIndexCount_ = 0;
    bool                             valid_            = false;
};

// Convenience wrapper: one-shot rebuild + update for frame-local use (a
// cache is only worth keeping when transforms are refreshed incrementally).
inline bool buildRenderInstanceBatches(std::span<const RenderDrawCommand> commands, RenderDrawSortMode sortMode,
    std::vector<RenderInstanceBatch> &outBatches, std::vector<float> &outColumnMajorMatrices)
{
    RenderInstanceBatchCache cache;
    return cache.rebuild(commands, sortMode) && cache.update(commands, outBatches, outColumnMajorMatrices);
}

// Vulkan VkDrawIndexedIndirectCommand layout, kept here so draw-list code
// can build indirect buffers without including Vulkan headers.
struct IndexedIndirectCommand
{
    std::uint32_t indexCount    = 0;
    std::uint32_t instanceCount = 0;
    std::uint32_t firstIndex    = 0;
    std::int32_t  vertexOffset  = 0;
    std::uint32_t firstInstance = 0;
};
static_assert(sizeof(IndexedIndirectCommand) == 20);
static_assert(std::is_standard_layout_v<IndexedIndirectCommand>);

} // namespace engine::render

#endif
