// Screen-tile binning policy for the CPU rasterizer's parallel path.
//
// Tile-based rasterisation bins screen-space triangles into a grid of tiles
// and lets each worker own disjoint tiles exclusively, so depth/color writes
// need no per-pixel locks and stay local and predictable (the CPU research
// file's prescribed structure). This header holds the pure policy: grid
// math, pixel-rect -> tile-range mapping, and the two-pass count/scatter
// binning walk. No Vulkan, no backend types - a pure policy header.

#ifndef ENGINE_RENDERER_MODULE_TILE_BINS_HPP
#define ENGINE_RENDERER_MODULE_TILE_BINS_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <new>
#include <vector>

namespace engine
{
namespace renderer
{
namespace tilebins
{

// Tile grid over a framebuffer. Tile dimensions only need to be non-zero.
struct TileGrid
{
    std::uint32_t tileWidth  = 0;
    std::uint32_t tileHeight = 0;
    std::uint32_t tilesX     = 0;
    std::uint32_t tilesY     = 0;
};

inline TileGrid makeGrid(std::uint32_t framebufferWidth, std::uint32_t framebufferHeight, std::uint32_t tileWidth,
    std::uint32_t tileHeight)
{
    TileGrid grid;
    if (tileWidth == 0 || tileHeight == 0)
    {
        return grid; // tilesX/tilesY stay 0: an empty grid, no tiles to own
    }
    const std::uint64_t tilesX = framebufferWidth == 0
        ? 0u
        : (static_cast<std::uint64_t>(framebufferWidth) + tileWidth - 1u) / tileWidth;
    const std::uint64_t tilesY = framebufferHeight == 0
        ? 0u
        : (static_cast<std::uint64_t>(framebufferHeight) + tileHeight - 1u) / tileHeight;
    if (tilesX > std::numeric_limits<std::uint32_t>::max()
        || tilesY > std::numeric_limits<std::uint32_t>::max()
        || tilesX * tilesY > std::numeric_limits<std::uint32_t>::max())
        return grid;
    grid.tileWidth  = tileWidth;
    grid.tileHeight = tileHeight;
    grid.tilesX     = static_cast<std::uint32_t>(tilesX);
    grid.tilesY     = static_cast<std::uint32_t>(tilesY);
    return grid;
}

[[nodiscard]] inline std::uint32_t tileCount(const TileGrid &grid) noexcept
{
    const std::uint64_t count = static_cast<std::uint64_t>(grid.tilesX) * grid.tilesY;
    return count <= std::numeric_limits<std::uint32_t>::max() ? static_cast<std::uint32_t>(count) : 0u;
}

[[nodiscard]] inline std::uint32_t flattenTile(const TileGrid &grid, std::uint32_t tileX, std::uint32_t tileY) noexcept
{
    return tileY * grid.tilesX + tileX;
}

// Maps an inclusive pixel rectangle to an inclusive tile-column/row range.
// Returns false when the rectangle is empty or entirely past the framebuffer
// edge, so callers can skip degenerate spans. Ranges poking past the right /
// bottom edge clamp to the last tile row/column.
[[nodiscard]] inline bool tileRangeForPixelRect(const TileGrid &grid, std::int32_t xBegin, std::int32_t xEnd,
    std::int32_t yBegin, std::int32_t yEnd, std::uint32_t &tileXBegin, std::uint32_t &tileXEnd,
    std::uint32_t &tileYBegin, std::uint32_t &tileYEnd) noexcept
{
    if (grid.tilesX == 0 || grid.tilesY == 0 || grid.tileWidth == 0 || grid.tileHeight == 0
        || xBegin > xEnd || yBegin > yEnd || xEnd < 0 || yEnd < 0)
        return false;

    const std::uint64_t coveredWidth  = static_cast<std::uint64_t>(grid.tilesX) * grid.tileWidth;
    const std::uint64_t coveredHeight = static_cast<std::uint64_t>(grid.tilesY) * grid.tileHeight;
    const std::int64_t  firstX = std::max<std::int64_t>(xBegin, 0);
    const std::int64_t  firstY = std::max<std::int64_t>(yBegin, 0);
    const std::uint64_t lastX  = std::min<std::uint64_t>(static_cast<std::uint64_t>(xEnd), coveredWidth - 1u);
    const std::uint64_t lastY  = std::min<std::uint64_t>(static_cast<std::uint64_t>(yEnd), coveredHeight - 1u);
    if (static_cast<std::uint64_t>(firstX) > lastX || static_cast<std::uint64_t>(firstY) > lastY)
        return false;

    tileXBegin = static_cast<std::uint32_t>(static_cast<std::uint64_t>(firstX) / grid.tileWidth);
    tileXEnd   = static_cast<std::uint32_t>(lastX / grid.tileWidth);
    tileYBegin = static_cast<std::uint32_t>(static_cast<std::uint64_t>(firstY) / grid.tileHeight);
    tileYEnd   = static_cast<std::uint32_t>(lastY / grid.tileHeight);
    return tileXBegin <= tileXEnd && tileYBegin <= tileYEnd && tileXEnd < grid.tilesX && tileYEnd < grid.tilesY;
}

// Two-pass binning over a grid: `ranges(record, ...)` reports, per record,
// the inclusive tile-column/row span the record covers (return false for
// records that touch no tile - they are skipped). Records are visited in
// ascending id order in both passes, so the resulting per-tile lists keep
// ascending record order - the property that keeps per-pixel rasterisation
// order deterministic (equal-depth writes resolve exactly as the
// sequential scan resolves them).
//
// On return: offsets has tileCount(grid)+1 entries (exclusive prefix sums),
// and items holds every (record, tile) pairing as record ids; the ids for
// tile t are items[offsets[t]..offsets[t+1]). Returns the total pairing
// count (items.size()).
template <typename RangeFn>
bool binRecordTileRanges(std::size_t recordCount, const RangeFn &ranges, const TileGrid &grid,
    std::vector<std::uint32_t> &offsets, std::vector<std::uint32_t> &items)
{
    const std::uint32_t tiles = tileCount(grid);
    // std::vector::max_size() is implementation-defined and querying it via
    // a temporary vector is not portable across standard-library versions.
    // ptrdiff_t provides a conservative, allocator-independent element cap.
    constexpr std::uint64_t maximumVectorElements =
        static_cast<std::uint64_t>(std::numeric_limits<std::ptrdiff_t>::max()) / sizeof(std::uint32_t);
    if (recordCount > std::numeric_limits<std::uint32_t>::max()
        || static_cast<std::uint64_t>(tiles) + 1u > maximumVectorElements)
    {
        offsets.clear();
        items.clear();
        return false;
    }

    try
    {
        offsets.assign(static_cast<std::size_t>(tiles) + 1u, 0u);
        if (tiles == 0)
        {
            items.clear();
            return true;
        }

        // Pass 1: per-tile coverage counts.
        for (std::size_t record = 0; record < recordCount; ++record)
        {
            std::uint32_t tileXBegin = 0, tileXEnd = 0, tileYBegin = 0, tileYEnd = 0;
            if (!ranges(record, tileXBegin, tileXEnd, tileYBegin, tileYEnd))
                continue;
            if (tileXBegin > tileXEnd || tileYBegin > tileYEnd || tileXEnd >= grid.tilesX || tileYEnd >= grid.tilesY)
            {
                offsets.clear();
                items.clear();
                return false;
            }
            for (std::uint32_t tileY = tileYBegin;; ++tileY)
            {
                const std::size_t rowBase = static_cast<std::size_t>(flattenTile(grid, 0u, tileY));
                for (std::uint32_t tileX = tileXBegin;; ++tileX)
                {
                    if (offsets[rowBase + tileX + 1u] == std::numeric_limits<std::uint32_t>::max())
                    {
                        offsets.clear();
                        items.clear();
                        return false;
                    }
                    ++offsets[rowBase + tileX + 1u];
                    if (tileX == tileXEnd)
                        break;
                }
                if (tileY == tileYEnd)
                    break;
            }
        }

        // Exclusive prefix sum; the flat item array uses 32-bit offsets.
        std::uint64_t totalPairings = 0;
        for (std::uint32_t tile = 0; tile < tiles; ++tile)
        {
            totalPairings += offsets[tile + 1u];
            if (totalPairings > std::numeric_limits<std::uint32_t>::max()
                || totalPairings > maximumVectorElements)
            {
                offsets.clear();
                items.clear();
                return false;
            }
            offsets[tile + 1u] = static_cast<std::uint32_t>(totalPairings);
        }

        // Pass 2: scatter record ids; cursors consume the prefix sums in place.
        items.resize(static_cast<std::size_t>(totalPairings));
        std::vector<std::uint32_t> cursors(offsets.begin(), offsets.end() - 1);
        for (std::size_t record = 0; record < recordCount; ++record)
        {
            std::uint32_t tileXBegin = 0, tileXEnd = 0, tileYBegin = 0, tileYEnd = 0;
            if (!ranges(record, tileXBegin, tileXEnd, tileYBegin, tileYEnd))
                continue;
            if (tileXBegin > tileXEnd || tileYBegin > tileYEnd || tileXEnd >= grid.tilesX || tileYEnd >= grid.tilesY)
            {
                offsets.clear();
                items.clear();
                return false;
            }
            for (std::uint32_t tileY = tileYBegin;; ++tileY)
            {
                const std::size_t rowBase = static_cast<std::size_t>(flattenTile(grid, 0u, tileY));
                for (std::uint32_t tileX = tileXBegin;; ++tileX)
                {
                    items[cursors[rowBase + tileX]++] = static_cast<std::uint32_t>(record);
                    if (tileX == tileXEnd)
                        break;
                }
                if (tileY == tileYEnd)
                    break;
            }
        }
    }
    catch (const std::bad_alloc &)
    {
        offsets.clear();
        items.clear();
        return false;
    }
    return true;
}
} // namespace tilebins
} // namespace renderer
} // namespace engine

#endif
