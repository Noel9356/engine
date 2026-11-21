// Shared spatial hash for flat-xyz particle systems.
//
// Cloth self-collision and SPH neighbor search both bucket particles by a
// uniform grid cell and walk the 3x3x3 cell neighborhood around a query
// particle. This header keeps one canonical sorted-bucket structure for
// both: rebuild() sorts (cell, particle) pairs, and the cell ranges are
// binary-searchable so findCell() and collectNeighborCandidates() stay
// logarithmic per cell. It is header-only and dependency-free apart from
// the standard library; the caller owns the particle arrays and rebuilds
// once per substep (buffers are reused, so a warmed-up record rebuilds
// without allocating).
//
// The cell identifier mixes the three signed cell coordinates into one
// 64-bit value; the mixing is a SplitMix-style avalanche, so the sorted
// bucket order scatters neighboring lattice cells across the id space
// (only grouping particles of the SAME cell, which is all the bucket
// structure needs - neighborhood queries enumerate the 27 cell ids
// directly and look each one up).

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace engine::util
{

// Sorted-bucket spatial hash over a flat xyz particle array.
class ParticleSpatialHash
{
public:
    // One bucketed particle: the cell id it fell into and its index into
    // the caller's particle arrays. Public so queries can walk a cell's
    // range without copying indices out (see findCell).
    struct Entry
    {
        std::uint64_t cell = 0;
        std::uint32_t particle = 0;
    };

    // The sorted (cell, particle) entries; a cell's particles occupy
    // [start, start + count) as reported by findCell. A public member so
    // solvers walk bucket ranges directly (the cloth self-collision pass
    // iterates one range in a tight loop).
    std::vector<Entry> entries;

    // Lattice coordinate of one scalar position: floor into signed cell
    // space. Non-finite coordinates clamp onto the extreme cell so a
    // diverged particle still hashes somewhere instead of invoking
    // undefined behavior in the cast.
    static std::int64_t cellCoordinate(double latticeCoordinate)
    {
        if (std::isnan(latticeCoordinate))
        {
            return 0;
        }
        const double floored = std::floor(latticeCoordinate);
        if (floored <= -9.2233720368547758e18)
        {
            return INT64_MIN;
        }
        if (floored >= 9.2233720368547758e18)
        {
            return INT64_MAX;
        }
        return static_cast<std::int64_t>(floored);
    }

    // Cell identifier for one lattice cell: the three signed coordinates
    // avalached into a single 64-bit key. Collisions across distinct cells
    // are astronomically unlikely and harmless (two cells merely share a
    // bucket; the neighbor walk then tests both ranges' particles).
    static std::uint64_t cellIdentifier(std::int64_t x, std::int64_t y, std::int64_t z)
    {
        std::uint64_t mixed = 0x9E3779B97F4A7C15ull;
        for (const std::int64_t coordinate : {x, y, z})
        {
            std::uint64_t value = static_cast<std::uint64_t>(coordinate) + 0x165667B19E3779F9ull;
            value ^= value >> 30;
            value *= 0xBF58476D1CE4E5B9ull;
            value ^= value >> 27;
            value *= 0x94D049BB133111EBull;
            value ^= value >> 31;
            mixed ^= value + 0x9E3779B97F4A7C15ull + (mixed << 6) + (mixed >> 2);
        }
        return mixed;
    }

    // Rebuilds the buckets for the current positions (cellSize must be
    // positive; callers pass the interaction distance so a 3x3x3 walk
    // covers every pair within it). Deterministic: identical positions
    // produce identical bucket contents and order.
    void rebuild(const double *positions, std::size_t particleCount, double cellSize)
    {
        entries.clear();
        cellIds_.clear();
        cellStarts_.clear();
        if (positions == nullptr || particleCount == 0 || !(cellSize > 0.0))
        {
            return;
        }
        entries.resize(particleCount);
        const double inverseCellSize = 1.0 / cellSize;
        for (std::size_t particle = 0; particle < particleCount; ++particle)
        {
            const std::size_t offset = particle * 3u;
            const std::int64_t cellX = cellCoordinate(positions[offset] * inverseCellSize);
            const std::int64_t cellY = cellCoordinate(positions[offset + 1u] * inverseCellSize);
            const std::int64_t cellZ = cellCoordinate(positions[offset + 2u] * inverseCellSize);
            entries[particle] = Entry{cellIdentifier(cellX, cellY, cellZ), static_cast<std::uint32_t>(particle)};
        }
        std::sort(entries.begin(), entries.end(), [](const Entry &a, const Entry &b)
        {
            return a.cell < b.cell || (a.cell == b.cell && a.particle < b.particle);
        });
        cellSizeInverse_ = inverseCellSize;
        // Unique cell ids with their entry-range starts (plus the trailing
        // sentinel) - the binary-search index findCell walks.
        cellIds_.reserve(entries.size());
        cellStarts_.reserve(entries.size() + 1u);
        for (std::size_t index = 0; index < entries.size(); ++index)
        {
            if (cellIds_.empty() || cellIds_.back() != entries[index].cell)
            {
                cellIds_.push_back(entries[index].cell);
                cellStarts_.push_back(static_cast<std::uint32_t>(index));
            }
        }
        cellStarts_.push_back(static_cast<std::uint32_t>(entries.size()));
    }

    // Locates one cell's bucket: start/count index into entries (which
    // see for the direct-access pattern the cloth solver uses). Returns
    // false when the cell holds no particle.
    bool findCell(std::uint64_t cellId, std::uint32_t &start, std::uint32_t &count) const
    {
        const std::size_t slot = binarySearchCell(cellId);
        if (slot == cellIds_.size())
        {
            return false;
        }
        start = cellStarts_[slot];
        count = cellStarts_[slot + 1u] - cellStarts_[slot];
        return true;
    }

    // Collects the indices of every particle in the 3x3x3 cell
    // neighborhood of particle `particle` (itself excluded), capped at
    // maxCandidates writes into outCandidates. Returns the number written.
    // The candidates are NOT distance filtered - kernels evaluate the
    // distance themselves - so the list is an over-approximation by
    // design.
    std::uint32_t collectNeighborCandidates(const double *positions, std::size_t particle,
        std::uint32_t *outCandidates, std::uint32_t maxCandidates) const
    {
        if (outCandidates == nullptr || maxCandidates == 0 || entries.empty())
        {
            return 0;
        }
        const std::size_t offset = particle * 3u;
        const double inverseCellSize = cellSizeInverse_;
        if (!(inverseCellSize > 0.0))
        {
            return 0;
        }
        const std::int64_t cellX = cellCoordinate(positions[offset] * inverseCellSize);
        const std::int64_t cellY = cellCoordinate(positions[offset + 1u] * inverseCellSize);
        const std::int64_t cellZ = cellCoordinate(positions[offset + 2u] * inverseCellSize);
        std::uint32_t collected = 0;
        for (std::int64_t offsetX = -1; offsetX <= 1; ++offsetX)
        {
            for (std::int64_t offsetY = -1; offsetY <= 1; ++offsetY)
            {
                for (std::int64_t offsetZ = -1; offsetZ <= 1; ++offsetZ)
                {
                    std::uint32_t start = 0;
                    std::uint32_t count = 0;
                    if (!findCell(cellIdentifier(cellX + offsetX, cellY + offsetY, cellZ + offsetZ), start, count))
                    {
                        continue;
                    }
                    for (std::uint32_t slot = start; slot < start + count; ++slot)
                    {
                        const std::uint32_t other = entries[slot].particle;
                        if (other == particle)
                        {
                            continue;
                        }
                        if (collected >= maxCandidates)
                        {
                            return collected;
                        }
                        outCandidates[collected++] = other;
                    }
                }
            }
        }
        return collected;
    }

private:
    std::size_t binarySearchCell(std::uint64_t cellId) const
    {
        const auto found = std::lower_bound(cellIds_.begin(), cellIds_.end(), cellId);
        if (found == cellIds_.end() || *found != cellId)
        {
            return cellIds_.size();
        }
        return static_cast<std::size_t>(found - cellIds_.begin());
    }

    std::vector<std::uint64_t> cellIds_;
    std::vector<std::uint32_t> cellStarts_; // one longer than cellIds_ (sentinel)
    double cellSizeInverse_ = 0.0;
};

} // namespace engine::util
