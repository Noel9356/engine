// Draw-ordering helpers shared by the capability layer and consumers.
//
// Materials are compared field-by-field (bit-cast floats so NaN/-0 order is
// total) so equal materials sort adjacently and drawMeshes can batch runs of
// same mesh + same material commands into instanced draws.

#ifndef ENGINE_RENDERER_DRAW_SORT_HPP
#define ENGINE_RENDERER_DRAW_SORT_HPP

#include <engine/renderer/capabilities.hpp>

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>

namespace engine::renderer
{

static_assert(sizeof(float) == sizeof(std::uint32_t), "render material ordering requires 32-bit float values");

// Three-way compare of two materials over every MeshMaterial field, in
// declaration order. Floats are compared via bit_cast so the ordering is a
// strict total order even for NaN or negative zero; this is what makes the
// equality checks in the batcher stable.
inline int compareRenderMaterialValues(const MeshMaterial &lhs, const MeshMaterial &rhs) noexcept
{
    const auto compareInteger = [](auto left, auto right) noexcept { return left < right ? -1 : left > right ? 1 : 0; };
    const auto compareFloat   = [&compareInteger](float left, float right) noexcept
    { return compareInteger(std::bit_cast<std::uint32_t>(left), std::bit_cast<std::uint32_t>(right)); };
    const auto compareFloatArray = [&compareFloat](const float *left, const float *right, std::size_t count) noexcept
    {
        for (std::size_t index = 0; index < count; ++index)
        {
            const int result = compareFloat(left[index], right[index]);
            if (result != 0)
                return result;
        }
        return 0;
    };

    int result = compareFloatArray(lhs.baseColor, rhs.baseColor, 3);
    if (result != 0)
        return result;
    if ((result = compareInteger(lhs.baseColorTexture, rhs.baseColorTexture)) != 0)
        return result;
    if ((result = compareFloat(lhs.metallicFactor, rhs.metallicFactor)) != 0)
        return result;
    if ((result = compareFloat(lhs.roughnessFactor, rhs.roughnessFactor)) != 0)
        return result;
    if ((result = compareInteger(lhs.metallicRoughnessTexture, rhs.metallicRoughnessTexture)) != 0)
        return result;
    if ((result = compareInteger(lhs.normalTexture, rhs.normalTexture)) != 0)
        return result;
    if ((result = compareFloat(lhs.normalScale, rhs.normalScale)) != 0)
        return result;
    if ((result = compareInteger(lhs.occlusionTexture, rhs.occlusionTexture)) != 0)
        return result;
    if ((result = compareFloat(lhs.occlusionStrength, rhs.occlusionStrength)) != 0)
        return result;
    if ((result = compareFloatArray(lhs.emissiveColor, rhs.emissiveColor, 3)) != 0)
        return result;
    if ((result = compareInteger(lhs.emissiveTexture, rhs.emissiveTexture)) != 0)
        return result;
    return compareInteger(lhs.unlit, rhs.unlit);
}

// The material a draw uses when its MeshMaterial* is null (identity PBR).
inline const MeshMaterial &effectiveRenderMaterial(const MeshMaterial *material) noexcept
{
    static const MeshMaterial defaultMaterial{};
    return material != nullptr ? *material : defaultMaterial;
}

// Sort key layout. MaterialThenMesh groups state changes (best for batching);
// MeshThenMaterial groups geometry. Draw order within one key is not
// guaranteed.
enum class RenderDrawSortMode : std::uint8_t
{
    MeshThenMaterial,
    MaterialThenMesh
};

// FNV-1a over every input a sort depends on: command count, sort mode, each
// command's mesh handle and the full material value tuple (the comparator
// orders by material values, not by pointer identity). Two consecutive frames
// with equal signatures produce the identical sort permutation, so callers can
// cache it and skip re-sorting.
inline std::uint64_t renderDrawCommandSortSignature(
    std::span<const RenderDrawCommand> commands, RenderDrawSortMode mode) noexcept
{
    std::uint64_t hash = 14695981039346656037ull;
    const auto    mix  = [&hash](std::uint64_t value)
    {
        hash ^= value;
        hash *= 1099511628211ull;
    };
    mix(static_cast<std::uint64_t>(commands.size()));
    mix(static_cast<std::uint64_t>(mode));
    const auto mixFloat = [&mix](float value)
    { mix(static_cast<std::uint64_t>(std::bit_cast<std::uint32_t>(value))); };
    for (const RenderDrawCommand &command : commands)
    {
        mix(static_cast<std::uint64_t>(command.mesh));
        const MeshMaterial &material = effectiveRenderMaterial(command.material);
        for (const float component : material.baseColor)
            mixFloat(component);
        mix(static_cast<std::uint64_t>(material.baseColorTexture));
        mixFloat(material.metallicFactor);
        mixFloat(material.roughnessFactor);
        mix(static_cast<std::uint64_t>(material.metallicRoughnessTexture));
        mix(static_cast<std::uint64_t>(material.normalTexture));
        mixFloat(material.normalScale);
        mix(static_cast<std::uint64_t>(material.occlusionTexture));
        mixFloat(material.occlusionStrength);
        for (const float component : material.emissiveColor)
            mixFloat(component);
        mix(static_cast<std::uint64_t>(material.emissiveTexture));
        mix(static_cast<std::uint64_t>(material.unlit));
    }
    return hash;
}

// Three-way compare of two draw commands' sort keys (mesh and material) in
// the requested mode. Materials go through effectiveRenderMaterial, so a null
// MeshMaterial* orders as the default material instead of dereferencing null;
// this matches drawMeshes, which draws null-material commands with the
// default material.
inline int compareRenderDrawCommandKeys(
    const RenderDrawCommand &lhs, const RenderDrawCommand &rhs, RenderDrawSortMode mode) noexcept
{
    const int meshCompare = lhs.mesh == rhs.mesh ? 0 : lhs.mesh < rhs.mesh ? -1 : 1;
    if (mode == RenderDrawSortMode::MeshThenMaterial)
    {
        if (meshCompare != 0)
            return meshCompare;
        return compareRenderMaterialValues(effectiveRenderMaterial(lhs.material), effectiveRenderMaterial(rhs.material));
    }
    const int materialCompare
        = compareRenderMaterialValues(effectiveRenderMaterial(lhs.material), effectiveRenderMaterial(rhs.material));
    if (materialCompare != 0)
        return materialCompare;
    return meshCompare;
}

// Sorts commands in place; null materials compare as the default material.
// Stability is not required because equal keys are interchangeable.
inline void sortRenderDrawCommands(
    std::span<RenderDrawCommand> commands, RenderDrawSortMode mode = RenderDrawSortMode::MaterialThenMesh)
{
    std::sort(commands.begin(),
        commands.end(),
        [&](const RenderDrawCommand &lhs, const RenderDrawCommand &rhs)
        { return compareRenderDrawCommandKeys(lhs, rhs, mode) < 0; });
}

// Builds the permutation that maps each sorted position to the index of the
// command that occupies it: sorts {sort key, original index} pairs directly
// instead of sorting a command copy and then reconstructing its origin with a
// nested search. Rebuilding the sort cache therefore costs O(n log n) rather
// than O(n^2) in the command count, which matters on the frames where the
// draw key set changes (streaming, spawns, material edits) - exactly the
// frames that land in the P99 tail. Ties fall back to the original index so
// the permutation is deterministic for duplicate keys and the cached gather
// reproduces the same order a fresh sort would have produced.
inline void sortRenderDrawCommandPermutation(std::span<const RenderDrawCommand> commands, RenderDrawSortMode mode,
    std::span<std::uint32_t> outPermutation) noexcept
{
    const std::size_t count = commands.size();
    if (count == 0 || outPermutation.size() < count)
        return;
    for (std::size_t index = 0; index < count; ++index)
        outPermutation[index] = static_cast<std::uint32_t>(index);
    const RenderDrawCommand *data = commands.data();
    std::sort(outPermutation.begin(),
        outPermutation.begin() + static_cast<std::ptrdiff_t>(count),
        [data, mode](std::uint32_t lhs, std::uint32_t rhs) noexcept
        {
            const int keyCompare = compareRenderDrawCommandKeys(data[lhs], data[rhs], mode);
            return keyCompare != 0 ? keyCompare < 0 : lhs < rhs;
        });
}

} // namespace engine::renderer

#endif
