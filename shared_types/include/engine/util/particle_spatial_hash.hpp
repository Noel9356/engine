// particle_spatial_hash.hpp - shared sorted-bucket spatial hash for flat xyz
// particle arrays.
//
// The simulation modules that neighbor-search particle pools (cloth
// self-collision, fluid SPH passes) repeat the same three pieces: hashing
// particle positions into integer cell coordinates, bucketing the (cell id,
// particle) pairs in a deterministic sorted structure, and scanning the
// 3x3x3 neighborhood of a query cell. This header keeps one canonical
// implementation of each so the modules search identically where their
// models overlap.
//
// The structure is header-only and depends on the standard library alone. It
// is a value type: the caller owns it inside its record, resizes it through
// rebuild(), and reads the buckets through the lookup helpers. The hash
// function uses the classic spatial-hash prime mix; two distinct cells
// colliding into one id merely merge their candidate lists, which every
// consumer's distance check tolerates deterministically.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <vector>

namespace engine::util
{

// One (cell id, particle) pair of the sorted bucket structure.
struct ParticleHashEntry
{
    std::uint64_t cellId   = 0;
    std::uint32_t particle = 0;
};

// Sorted-bucket spatial hash over a flat xyz particle array.
struct ParticleSpatialHash
{
    // (cell id, particle) pairs sorted by id with a particle tiebreak, so
    // every bucket holds ascending particle indices.
    std::vector<ParticleHashEntry> entries;

    // Unique ascending cell ids and the start offset of each bucket; a
    // bucket ends where the next one starts (the last runs to the end).
    std::vector<std::uint64_t>   cellIds;
    std::vector<std::uint32_t>  cellStarts;

    // Cell size the last rebuild used; lookups must pass the same value.
    double cellSize = 0.0;

    // Cell coordinate for one scaled axis: clamps the scaled position
    // before the integer cast so huge or non-finite positions cannot
    // overflow; a diverging step is rejected by the caller's finite guard,
    // and the hash stays well-defined meanwhile.
    static std::int64_t cellCoordinate(double scaledPosition)
    {
        if (!std::isfinite(scaledPosition))
        {
            return 0;
        }
        constexpr double extent = 1.0e9;
        scaledPosition         = std::min(std::max(scaledPosition, -extent), extent);
        return static_cast<std::int64_t>(std::floor(scaledPosition));
    }

    // Bounds-free cell-id hash (the classic spatial-hash prime mix).
    static std::uint64_t cellIdentifier(std::int64_t cellX, std::int64_t cellY, std::int64_t cellZ)
    {
        return (static_cast<std::uint64_t>(cellX) * 73856093ull)
               ^ (static_cast<std::uint64_t>(cellY) * 19349663ull)
               ^ (static_cast<std::uint64_t>(cellZ) * 83492791ull);
    }

    // Cell id of one particle position under the rebuild's cell size.
    std::uint64_t cellIdentifierForPosition(const double *position) const
    {
        return cellIdentifier(cellCoordinate(position[0] / cellSize),
            cellCoordinate(position[1] / cellSize),
            cellCoordinate(position[2] / cellSize));
    }

    // Rebuilds the buckets for a flat xyz array of particleCount particles.
    // Allocation happens only when the capacities grow; warmed-up records
    // rebuild without touching the allocator.
    void rebuild(const double *positions, std::size_t particleCount, double newCellSize)
    {
        cellSize = newCellSize;
        entries.resize(particleCount);
        for (std::size_t particle = 0; particle < particleCount; ++particle)
        {
            const std::size_t offset  = particle * 3;
            entries[particle].cellId = cellIdentifierForPosition(positions + offset);
            entries[particle].particle = static_cast<std::uint32_t>(particle);
        }
        std::sort(entries.begin(), entries.end(), [](const ParticleHashEntry &left, const ParticleHashEntry &right)
            { return left.cellId != right.cellId ? left.cellId < right.cellId : left.particle < right.particle; });

        cellIds.clear();
        cellStarts.clear();
        for (std::size_t entry = 0; entry < entries.size(); ++entry)
        {
            if (entry == 0 || entries[entry].cellId != cellIds.back())
            {
                cellIds.push_back(entries[entry].cellId);
                cellStarts.push_back(static_cast<std::uint32_t>(entry));
            }
        }
    }

    // Locates one cell id; returns its bucket start and count through the
    // out parameters, false when the cell holds no particles.
    bool findCell(std::uint64_t identifier, std::uint32_t &outStart, std::uint32_t &outCount) const
    {
        const auto slot = std::lower_bound(cellIds.begin(), cellIds.end(), identifier);
        if (slot == cellIds.end() || *slot != identifier)
        {
            return false;
        }
        const std::size_t index = static_cast<std::size_t>(slot - cellIds.begin());
        outStart                = cellStarts[index];
        outCount                = index + 1 < cellStarts.size() ? cellStarts[index + 1] - outStart
                                   : static_cast<std::uint32_t>(entries.size()) - outStart;
        return true;
    }

    // Collects the candidate particles of the 3x3x3 neighborhood of one
    // particle's cell, in fixed cell order and ascending particle order
    // inside each cell. Candidates outside the interaction radius are
    // filtered by the caller; the returned count is capped at
    // maximumCandidates so a degenerately dense cluster cannot grow the
    // caller's per-particle neighbor list without bound.
    std::uint32_t collectNeighborCandidates(
        const double *positions, std::size_t particleIndex, std::uint32_t *outCandidates, std::uint32_t maximumCandidates) const
    {
        std::uint32_t collected = 0;
        const std::size_t offset = particleIndex * 3;
        const std::int64_t cellX = cellCoordinate(positions[offset] / cellSize);
        const std::int64_t cellY = cellCoordinate(positions[offset + 1] / cellSize);
        const std::int64_t cellZ = cellCoordinate(positions[offset + 2] / cellSize);
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
                    for (std::uint32_t entry = 0; entry < count && collected < maximumCandidates; ++entry)
                    {
                        const std::uint32_t candidate = entries[start + entry].particle;
                        if (candidate == particleIndex)
                        {
                            continue;
                        }
                        outCandidates[collected++] = candidate;
                    }
                }
            }
        }
        return collected;
    }
};

} // namespace engine::util
