// Chunk meshing for the planet module (see meshing.hpp).
//
// Each chunk is meshed independently with marching cubes over a (n+3)^3
// padded density grid: the 1-voxel apron lets every surface vertex compute
// its normal from the SAME canonical cell no matter which chunk owns the
// geometry, so same-depth neighbor chunks weld watertight AND shade
// continuously across their shared faces.
//
// Determinism: all sampling positions come from the node's integer lattice
// coordinates times the node step (no accumulated floating point), the
// marching-cubes tables are constants, and the field is a pure function - so
// a chunk's mesh is bit-identical no matter when, where or in what order it
// is built.
//
// LOD seams: same-depth neighbors share the global lattice bit-for-bit and
// weld exactly. Across a depth transition, the octree's 2:1 balance alone
// leaves a T-junction gap: the fine chunk's boundary isoline follows the raw
// field while the coarse neighbor's surface follows its own coarse-lattice
// chords, and the two disagree by whatever high-frequency content the coarse
// lattice cannot represent (visible as black crack dashes at the ring). The
// fix is CONFORMING BOUNDARY SAMPLING: wherever the material beyond a face
// segment is a rendered leaf one level coarser, the fine chunk samples that
// segment's lattice points from the coarse lattice's trilinear interpolant
// restricted to the boundary plane (a bilinear over the four surrounding
// coarse samples) instead of the raw field. The two surfaces then share the
// exact same zero crossings on every coarse grid line, and the residual
// inside coarse cell faces collapses to the bilinear chord sagitta (orders
// of magnitude below a voxel). Which segments conform is decided per coarse
// cell and delivered as a ConformMask, so meshes remain pure functions of
// (params, coord, mask).

#ifndef ENGINE_PLANET_MESHING_HPP
#define ENGINE_PLANET_MESHING_HPP

#include <engine/planet/density.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>

#include <array>
#include <cstdint>
#include <vector>

namespace engine
{
namespace planet
{

// Octree node address: depth (0 = the root cube) + lattice coordinates.
struct ChunkCoord
{
    std::uint32_t depth = 0;
    std::uint32_t x     = 0;
    std::uint32_t y     = 0;
    std::uint32_t z     = 0;
};

inline bool operator==(const ChunkCoord &a, const ChunkCoord &b)
{
    return a.depth == b.depth && a.x == b.x && a.y == b.y && a.z == b.z;
}

// Per-face flags marking which boundary segments of a chunk border a rendered
// leaf one level COARSER (see the LOD seams note above). Face order: 0 = -x,
// 1 = +x, 2 = -y, 3 = +y, 4 = -z, 5 = +z. A chunk is ONE octree cell, so the
// material beyond a face is governed by the single depth-(d-1) cell adjacent
// to the face interior; the +-1 tangential ring covers apron lattice points
// that overshoot the face corner into the diagonal depth-(d-1) cells (their
// values feed the shared gradient blocks, so the diagonal decisions are
// needed as well). Each face stores 3x3 flags over the tangential coarse-cell
// offsets (ou, ov) in {-1, 0, +1} relative to the chunk's own coarse-cell
// block, indexed (ou + 1) + (ov + 1) * 3. The tangential mapping: x faces
// span (y, z), y faces span (x, z), z faces span (x, y).
//
// A set flag does two things on the fine side of the transition:
//   1. the boundary-plane lattice points sample the coarse lattice's
//      trilinear interpolant instead of the raw field, so the fine isoline
//      and the coarse chords share their zero crossings on every coarse grid
//      line; and
//   2. the chunk EXTENDS its surface one cell layer past that face (marching
//      the apron cells beyond it). The extension carries the true surface
//      INTO the coarse neighbor's territory, where the neighbor's own surface
//      - whose chords approximate the same field to within its chord error -
//      overlaps it. The two surfaces interpenetrate along the whole
//      transition, so no viewing ray can slip between them: the sub-voxel
//      mismatch that remains reads as terrain-colored overlap, never as a
//      crack against the background.
struct ConformMask
{
    static constexpr std::uint32_t kFaceCount = 6;

    std::array<std::uint8_t, 9> face[kFaceCount]{};

    bool operator==(const ConformMask &other) const = default;

    bool any() const
    {
        for (std::uint32_t index = 0; index < kFaceCount; ++index)
        {
            for (std::uint32_t slot = 0; slot < 9; ++slot)
            {
                if (face[index][slot] != 0)
                {
                    return true;
                }
            }
        }
        return false;
    }

    std::uint64_t hash64() const
    {
        std::uint64_t value = 0xCBF29CE484222325ull;
        for (std::uint32_t index = 0; index < kFaceCount; ++index)
        {
            for (std::uint32_t slot = 0; slot < 9; ++slot)
            {
                value = (value ^ static_cast<std::uint64_t>(face[index][slot])) * 0x100000001B3ull;
            }
        }
        return value;
    }
};

// Bit helpers shared by the octree (writes) and the mesher (reads).
inline bool conformBit(const ConformMask &mask, std::uint32_t face, int offsetU, int offsetV)
{
    return mask.face[face][static_cast<std::uint32_t>(offsetU + 1) + static_cast<std::uint32_t>(offsetV + 1) * 3u] != 0;
}

inline void setConformBit(ConformMask &mask, std::uint32_t face, int offsetU, int offsetV, bool value)
{
    mask.face[face][static_cast<std::uint32_t>(offsetU + 1) + static_cast<std::uint32_t>(offsetV + 1) * 3u] = value ? 1u : 0u;
}

// Built geometry for one chunk: terrain surface plus (optionally) a water
// surface patch, both in planet-local coordinates (floats, chunk-local
// offsets relative to nodeOrigin are resolved - vertices are absolute
// planet-local positions cast to float at the end).
struct ChunkMesh
{
    std::vector<PlanetVertex>   terrainVertices;
    std::vector<std::uint32_t>  terrainIndices;
    std::vector<PlanetVertex>   waterVertices;
    std::vector<std::uint32_t>  waterIndices;
    std::uint64_t               hash = 0; // FNV-1a over all buffers (self-tests)
    std::uint64_t               triangleCount = 0;
};

// Marches one chunk. params must be validate()d; field must be built from the
// same params. conformMask selects which boundary segments sample the coarse
// neighbor's lattice (see the LOD seams note above); pass a default-constructed
// ConformMask for an unconstrained build. Pure function: thread safe, order
// independent - the mask is an input like the coordinate, never global state.
ChunkMesh buildChunkMesh(const PlanetGenerationParams &params, const DensityField &field, const ChunkCoord &coord,
    const ConformMask &conformMask);

// Convenience overload: unconstrained build (no conforming segments).
ChunkMesh buildChunkMesh(const PlanetGenerationParams &params, const DensityField &field, const ChunkCoord &coord);

// Node geometry helpers shared with the octree and tools.
inline double chunkSizeMeters(const PlanetGenerationParams &params, std::uint32_t depth)
{
    return std::ldexp(params.derivedRootSizeMeters, -static_cast<int>(depth));
}

// Lattice step between adjacent grid samples at this depth. Derived as
// (rootSize / leafGridN) scaled by an exact power of two so every depth's
// step is a bit-exact halving chain; combined with the global-index sampling
// below, any two chunks (same depth or across depths) compute bit-identical
// positions for shared lattice points - the watertightness guarantee.
inline double chunkStepMeters(const PlanetGenerationParams &params, std::uint32_t depth)
{
    return std::ldexp(params.derivedRootSizeMeters / static_cast<double>(params.leafGridN), -static_cast<int>(depth));
}

// Minimum corner of the node's cube in planet-local space.
Vec3 chunkOriginMeters(const PlanetGenerationParams &params, const ChunkCoord &coord);

} // namespace planet
} // namespace engine

#endif // ENGINE_PLANET_MESHING_HPP
