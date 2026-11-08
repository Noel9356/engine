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

#include <cstdint>
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
    grid.tileWidth  = tileWidth;
    grid.tileHeight = tileHeight;
    grid.tilesX     = framebufferWidth == 0 ? 0u : (framebufferWidth + tileWidth - 1u) / tileWidth;
    grid.tilesY     = framebufferHeight == 0 ? 0u : (framebufferHeight + tileHeight - 1u) / tileHeight;
    return grid;
}

[[nodiscard]] inline std::uint32_t tileCount(const TileGrid &grid) noexcept
{
    return grid.tilesX * grid.tilesY;
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
    if (grid.tilesX == 0 || grid.tilesY == 0 || xBegin > xEnd || yBegin > yEnd)
    {
        return false;
    }
    if (xBegin >= static_cast<std::int32_t>(grid.tilesX * grid.tileWidth)
        || yBegin >= static_cast<std::int32_t>(grid.tilesY * grid.tileHeight))
    {
        return false; // starts past the framebuffer: no coverage anywhere
    }
    tileXBegin = static_cast<std::uint32_t>(xBegin) / grid.tileWidth;
    tileXEnd   = static_cast<std::uint32_t>(xEnd) / grid.tileWidth;
    tileYBegin = static_cast<std::uint32_t>(yBegin) / grid.tileHeight;
    tileYEnd   = static_cast<std::uint32_t>(yEnd) / grid.tileHeight;
    if (tileXEnd >= grid.tilesX)
    {
        tileXEnd = grid.tilesX - 1u;
    }
    if (tileYEnd >= grid.tilesY)
    {
        tileYEnd = grid.tilesY - 1u;
    }
    return true;
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
std::size_t binRecordTileRanges(std::size_t recordCount, const RangeFn &ranges, const TileGrid &grid,
    std::vector<std::uint32_t> &offsets, std::vector<std::uint32_t> &items)
{
    const std::uint32_t tiles = tileCount(grid);
    offsets.assign(static_cast<std::size_t>(tiles) + 1u, 0u);
    if (tiles == 0)
    {
        items.clear();
        return 0;
    }

    // Pass 1: per-tile coverage counts.
    for (std::size_t record = 0; record < recordCount; ++record)
    {
        std::uint32_t tileXBegin = 0, tileXEnd = 0, tileYBegin = 0, tileYEnd = 0;
        if (!ranges(record, tileXBegin, tileXEnd, tileYBegin, tileYEnd))
        {
            continue;
        }
        for (std::uint32_t tileY = tileYBegin; tileY <= tileYEnd; ++tileY)
        {
            const std::size_t rowBase = static_cast<std::size_t>(flattenTile(grid, 0u, tileY));
            for (std::uint32_t tileX = tileXBegin; tileX <= tileXEnd; ++tileX)
            {
                ++offsets[rowBase + tileX + 1u];
            }
        }
    }

    // Exclusive prefix sum.
    for (std::uint32_t tile = 0; tile < tiles; ++tile)
    {
        offsets[tile + 1u] += offsets[tile];
    }

    // Pass 2: scatter record ids; cursors consume the prefix sums in place.
    items.resize(offsets[tiles]);
    std::vector<std::uint32_t> cursors(offsets.begin(), offsets.end() - 1);
    for (std::size_t record = 0; record < recordCount; ++record)
    {
        std::uint32_t tileXBegin = 0, tileXEnd = 0, tileYBegin = 0, tileYEnd = 0;
        if (!ranges(record, tileXBegin, tileXEnd, tileYBegin, tileYEnd))
        {
            continue;
        }
        for (std::uint32_t tileY = tileYBegin; tileY <= tileYEnd; ++tileY)
        {
            const std::size_t rowBase = static_cast<std::size_t>(flattenTile(grid, 0u, tileY));
            for (std::uint32_t tileX = tileXBegin; tileX <= tileXEnd; ++tileX)
            {
                items[cursors[rowBase + tileX]++] = static_cast<std::uint32_t>(record);
            }
        }
    }
    return items.size();
}

} // namespace tilebins
} // namespace renderer
} // namespace engine

#endif
