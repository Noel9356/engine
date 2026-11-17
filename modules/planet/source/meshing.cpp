// Chunk meshing implementation (see meshing.hpp).
//
// The marching-cubes lookup tables (Bourke, public domain) live in
// marching_cubes_tables.hpp; the algorithm below consumes them through the
// detail namespace.

#include <cstdint>
#include <engine/planet/meshing.hpp>

#include <engine/planet/density.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>
#include "marching_cubes_tables.hpp"

#include <engine/planet/fbm.hpp>
#include <engine/planet/flat_key_map.hpp>
#include <engine/planet/noise.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <vector>

namespace engine
{
namespace planet
{
namespace
{

// Bourke corner numbering: 0..3 ring the low face counterclockwise, 4..7 the
// high face. Bit c of a cell's cube index refers to this corner.
inline void cornerOffset(std::uint32_t corner, std::uint32_t &dx, std::uint32_t &dy, std::uint32_t &dz)
{
    switch (corner)
    {
    case 0: dx = 0; dy = 0; dz = 0; break;
    case 1: dx = 1; dy = 0; dz = 0; break;
    case 2: dx = 1; dy = 1; dz = 0; break;
    case 3: dx = 0; dy = 1; dz = 0; break;
    case 4: dx = 0; dy = 0; dz = 1; break;
    case 5: dx = 1; dy = 0; dz = 1; break;
    case 6: dx = 1; dy = 1; dz = 1; break;
    default: dx = 0; dy = 1; dz = 1; break; // 7
    }
}

// Dedup key for an edge between two padded-grid points a step apart on
// `axis` (local to one chunk's build).
inline std::uint64_t edgeKey(std::uint32_t ax, std::uint32_t ay, std::uint32_t az, std::uint32_t axis)
{
    return (static_cast<std::uint64_t>(axis) << 60) | (static_cast<std::uint64_t>(ax) << 40)
         | (static_cast<std::uint64_t>(ay) << 20) | static_cast<std::uint64_t>(az);
}

// Canonical endpoints of a marching-cubes edge: lexicographic minimum first.
// Every cell that shares the edge (in this chunk or the neighbor's) sees the
// same (a, b) pair, so interpolated positions and dedup keys agree.
inline void canonicalEdge(std::uint32_t pax, std::uint32_t pay, std::uint32_t paz, std::uint32_t pbx, std::uint32_t pby,
    std::uint32_t pbz, std::uint32_t &ax, std::uint32_t &ay, std::uint32_t &az, std::uint32_t &bx, std::uint32_t &by,
    std::uint32_t &bz, std::uint32_t &axis)
{
    ax = pax; ay = pay; az = paz;
    bx = pbx; by = pby; bz = pbz;
    if (pbx < pax || (pbx == pax && pby < pay) || (pbx == pax && pby == pay && pbz < paz))
    {
        ax = pbx; ay = pby; az = pbz;
        bx = pax; by = pay; bz = paz;
    }
    axis = ax != bx ? 0u : (ay != by ? 1u : 2u);
}

// Shared marching-cubes cell walk for the terrain and water passes below.
// The two passes differ in their field (terrain density vs the signed
// distance to the sea sphere), their cell domain (the terrain pass extends
// into conforming apron cells), their vertex assembly (shaded climate
// albedo vs water depth tint) and their triangle orientation (fixed winding
// vs radial auto-orient); everything else - corner classification, edge
// canonicalization, zero-crossing interpolation, edge-vertex dedup and the
// case-table walk - is this one implementation. The four callables keep the
// shared walk free of per-cell virtual or std::function dispatch:
//   cellActive(cx, cy, cz) -> bool - whether the cell marches at all;
//   skipCell(cubeIndex, buriedMin) -> bool - fully solid/air cells, and
//     (for the water pass, which passes the terrain grid as buriedField)
//     cells buried under the terrain surface;
//   makeVertex(aX, aY, aZ, bX, bY, bZ, edgeAxis, t, point) -> std::uint32_t
//     - assembles the vertex for a newly seen edge crossing and returns its
//       slot in the pass's vertex buffer;
//   emitTriangle(cubeIndex, slotVertex) - pushes the case table's triangles
//     for the cell.
// `buriedField` (optional) is sampled alongside the field so skipCell can
// reason about the other pass's values; nullptr skips the extra reads.
template <typename CellActive, typename SkipCell, typename MakeVertex, typename EmitTriangle>
void marchCells(const std::vector<double> &field, const std::vector<double> *buriedField, std::int64_t minCell,
    std::int64_t maxCell, const std::array<double, 64 + 3> &axisX, const std::array<double, 64 + 3> &axisY,
    const std::array<double, 64 + 3> &axisZ, std::uint32_t paddedN, FlatKeyMap<std::uint32_t> &vertexForEdge,
    CellActive &&cellActive, SkipCell &&skipCell, MakeVertex &&makeVertex, EmitTriangle &&emitTriangle)
{
    const std::size_t strideY = paddedN;
    const std::size_t strideZ = static_cast<std::size_t>(paddedN) * paddedN;
    const auto sampleGrid = [&field, strideY, strideZ](std::uint32_t px, std::uint32_t py, std::uint32_t pz)
    {
        return field[static_cast<std::size_t>(px) + strideY * py + strideZ * pz];
    };
    const auto sampleBuried = [&buriedField, strideY, strideZ](std::uint32_t px, std::uint32_t py, std::uint32_t pz)
    {
        return (*buriedField)[static_cast<std::size_t>(px) + strideY * py + strideZ * pz];
    };

    for (std::int64_t cz = minCell; cz <= maxCell; ++cz)
    {
        for (std::int64_t cy = minCell; cy <= maxCell; ++cy)
        {
            for (std::int64_t cx = minCell; cx <= maxCell; ++cx)
            {
                if (!cellActive(cx, cy, cz))
                {
                    continue;
                }
                double        cornerDensity[8];
                double        minBuried = 1.0e30;
                std::uint32_t cubeIndex = 0;
                for (std::uint32_t corner = 0; corner < 8; ++corner)
                {
                    std::uint32_t dx, dy, dz;
                    cornerOffset(corner, dx, dy, dz);
                    const std::uint32_t px = static_cast<std::uint32_t>(cx + 1 + dx);
                    const std::uint32_t py = static_cast<std::uint32_t>(cy + 1 + dy);
                    const std::uint32_t pz = static_cast<std::uint32_t>(cz + 1 + dz);
                    cornerDensity[corner] = sampleGrid(px, py, pz);
                    if (buriedField != nullptr)
                    {
                        minBuried = std::min(minBuried, sampleBuried(px, py, pz));
                    }
                    if (cornerDensity[corner] > 0.0)
                    {
                        cubeIndex |= 1u << corner;
                    }
                }
                if (skipCell(cubeIndex, minBuried))
                {
                    continue;
                }

                std::uint32_t slotVertex[12];
                for (std::uint32_t slot = 0; slot < 12; ++slot)
                {
                    slotVertex[slot] = 0xFFFFFFFFu;
                }

                const std::uint32_t edges = detail::kEdgeTable[cubeIndex];
                for (std::uint32_t slot = 0; slot < 12; ++slot)
                {
                    if ((edges & (1u << slot)) == 0)
                    {
                        continue;
                    }
                    std::uint32_t adx, ady, adz, bdx, bdy, bdz;
                    cornerOffset(detail::kEdgeCorners[slot][0], adx, ady, adz);
                    cornerOffset(detail::kEdgeCorners[slot][1], bdx, bdy, bdz);
                    std::uint32_t aX, aY, aZ, bX, bY, bZ, edgeAxis;
                    canonicalEdge(cx + 1 + adx, cy + 1 + ady, cz + 1 + adz, cx + 1 + bdx, cy + 1 + bdy, cz + 1 + bdz,
                        aX, aY, aZ, bX, bY, bZ, edgeAxis);

                    const std::uint64_t key = edgeKey(aX, aY, aZ, edgeAxis);
                    const std::uint32_t *vertexSlot = vertexForEdge.find(key);
                    if (vertexSlot != nullptr)
                    {
                        slotVertex[slot] = *vertexSlot;
                    }
                    else
                    {
                        const double densityA = sampleGrid(aX, aY, aZ);
                        const double densityB = sampleGrid(bX, bY, bZ);
                        // The conforming boundary values can make the two
                        // endpoint densities identical (locally flat coarse
                        // field): the raw division would yield inf/NaN and
                        // the degenerate triangle then spans the whole screen
                        // in the software rasterizer. Clamp the crossing into
                        // the edge; legit crossings already sit in [0, 1].
                        const double denominator = densityA - densityB;
                        const double t        = std::abs(denominator) < 1.0e-12
                                                  ? 0.5
                                                  : std::clamp(densityA / denominator, 0.0, 1.0);

                        // aX/aY/aZ can legitimately be 0 for EXTENDED
                        // apron-cell edges; the axis tables are indexed
                        // directly (they include the apron range).
                        const Vec3 pointA{axisX[aX], axisY[aY], axisZ[aZ]};
                        const Vec3 pointB{axisX[bX], axisY[bY], axisZ[bZ]};
                        const Vec3 point = pointA + (pointB - pointA) * t;

                        const std::uint32_t newSlot
                            = makeVertex(aX, aY, aZ, bX, bY, bZ, edgeAxis, t, point);
                        vertexForEdge.findOrInsert(key) = newSlot;
                        slotVertex[slot] = newSlot;
                    }
                }

                emitTriangle(cubeIndex, slotVertex);
            }
        }
    }
}

} // namespace

Vec3 chunkCenterMeters(const PlanetGenerationParams &params, const ChunkCoord &coord)
{
    // Deterministic node center: the min corner shifted by half the node
    // size along every axis (same lattice expression as the min corner, so
    // the placement offset stays bit-identical to the vertex frame).
    const double step = chunkStepMeters(params, coord.depth);
    const double half = params.derivedRootHalfMeters;
    const std::int64_t base = static_cast<std::int64_t>(params.leafGridN) / 2;
    return Vec3{-half + step * static_cast<double>(static_cast<std::int64_t>(coord.x) * static_cast<std::int64_t>(params.leafGridN) + base),
        -half + step * static_cast<double>(static_cast<std::int64_t>(coord.y) * static_cast<std::int64_t>(params.leafGridN) + base),
        -half + step * static_cast<double>(static_cast<std::int64_t>(coord.z) * static_cast<std::int64_t>(params.leafGridN) + base)};
}

ChunkMesh buildChunkMesh(const PlanetGenerationParams &params, const DensityField &field, const ChunkCoord &coord,
    const ConformMask &conformMask)
{
    ChunkMesh mesh;

    const std::uint32_t n        = params.leafGridN;
    const std::uint32_t paddedN  = n + 3; // 1-voxel apron on every side
    const std::uint32_t strideY  = paddedN;
    const std::uint32_t strideZ  = paddedN * paddedN;
    const std::size_t   gridSize = static_cast<std::size_t>(paddedN) * paddedN * paddedN;
    const double        nodeSize = chunkSizeMeters(params, coord.depth);
    const double        step     = chunkStepMeters(params, coord.depth);
    const double        half     = params.derivedRootHalfMeters;

    // Every sample position is derived from ONE global lattice index along
    // each axis: p = -half + step * (coord * n + innerIndex). Neighbor chunks
    // (same depth or across depths) evaluate the exact same expression with
    // the exact same integers for shared points, so their boundary geometry
    // matches bit-for-bit - no floating point drift, no cracks.
    const auto axisCoordinate = [&](std::uint32_t axisIndex, std::int64_t innerIndex)
    {
        const std::int64_t globalIndex = static_cast<std::int64_t>(static_cast<std::size_t>(axisIndex) * n) + innerIndex;
        return half * -1.0 + step * static_cast<double>(globalIndex);
    };

    // Chunk-local origin for the float vertices: the node center. Consumers
    // place chunks with a model matrix; floats stay small at any planet radius.
    const Vec3 center{axisCoordinate(coord.x, n / 2), axisCoordinate(coord.y, n / 2), axisCoordinate(coord.z, n / 2)};

    // ---- water placement ---------------------------------------------------------
    // Nodes whose bounding sphere misses the sea shell have no water - and
    // must not pay for the per-point sea-field grid below, so the test runs
    // BEFORE any sampling. (The sea shell is a thin spherical surface; on
    // inland or high-sky chunks the whole water pass is pure overhead.)
    const bool buildWater = params.waterEnabled;
    bool nodeTouchesSea = false;
    if (buildWater)
    {
        const double centerRadius = length(center);
        const double nodeBound    = 0.5 * nodeSize * std::sqrt(3.0);
        const double seaRadius    = params.derivedSeaRadiusMeters;
        nodeTouchesSea = centerRadius - nodeBound <= seaRadius + step && centerRadius + nodeBound >= seaRadius - step;
    }

    // ---- conforming boundary sampling (LOD seam elimination) --------------------------
    //
    // Wherever the material beyond a boundary-plane segment is a rendered leaf
    // one level coarser (flagged per coarse cell in conformMask), the fine
    // chunk's boundary-plane lattice points sample the COARSE LATTICE's
    // trilinear interpolant restricted to the plane (a bilinear over the (up
    // to) four surrounding on-plane coarse samples) instead of the raw field.
    // The plane sits on a coarse lattice plane, and at coarse lattice points
    // the bilinear degenerates to the coarse sample itself - so the fine
    // isoline and the coarse neighbor's chords share their zero crossings on
    // every coarse grid line, and the two surfaces meet without cracks at
    // those crossings.
    //
    // The residual between the crossings (the coarse chord sagitta) is covered
    // by the EXTENSION: the chunk also marches the apron cells just beyond
    // every conforming face, carrying the true surface INTO the coarse
    // neighbor's territory where the neighbor's own surface overlaps it - the
    // two interpenetrate, so no viewing ray can slip between them (see the
    // ConformMask comment in meshing.hpp).
    //
    // Consistency rules (they make the value a global function of the point):
    //  * the far-side coarse cell is derived from GLOBAL indices (floor of the
    //    tangential position, diagonal ring included), so every chunk touching
    //    the point looks up the same cell and decides identically;
    //  * when several planes touch a point (chunk edges/corners) the lowest
    //    conforming AXIS wins, and the bilinear arithmetic only depends on
    //    that axis - identical for every chunk around the point;
    //  * points at even tangential GLOBAL indices are coarse lattice points -
    //    chunk corners (coord * n) are always such points, which pins every
    //    corner to the same value from both sides (with per-depth band
    //    limiting the value is the coarse side's band-limited sample, see
    //    below).
    const double coarseStep  = coord.depth > 0 ? chunkStepMeters(params, coord.depth - 1) : 0.0;
    const bool   conforming  = coord.depth > 0 && conformMask.any();

    // Per-depth band limiting: a marching-cubes lattice resolves at most ~2
    // samples per wavelength, so the field evaluated FOR this chunk must
    // carry no content finer than its own voxel - content below
    // voxel * lodBandLimitFactor fades smoothly to zero (full strength at
    // twice that wavelength). Without this, coarse LOD chunks sample the
    // sub-voxel detail octaves as uncorrelated random values and marching
    // cubes reconstructs them as random spikes - the "spiky globe".
    // The cutoff follows the lattice, not the chunk: any lattice point at
    // depth d is ALWAYS evaluated with cutoff(d), so shared points agree
    // bit-for-bit no matter which chunk samples them. The coarse side of a
    // depth transition uses cutoff(d-1) = 2 * cutoff(d) (its voxel is twice
    // ours), and the conforming machinery copies exactly those values onto
    // the shared plane - including at coarse lattice points, which is why
    // the even-even skip below moved behind the mask check.
    const double ownCutoff    = step * params.lodBandLimitFactor;
    const double coarseCutoff = coord.depth > 0 ? coarseStep * params.lodBandLimitFactor : 0.0;

    const auto floorDiv = [](std::int64_t value, std::int64_t divisor)
    {
        std::int64_t quotient = value / divisor;
        if ((value % divisor) != 0 && ((value < 0) != (divisor < 0)))
        {
            --quotient;
        }
        return quotient;
    };

    // Bilinear evaluation of the coarse lattice interpolant on one boundary
    // plane. (axis, maxSide) identify the plane; perpGlobalFine is the point's
    // global fine index along that axis (always a multiple of n, hence a
    // coarse lattice plane); tangentialGlobalU/V are the other two axes. At
    // coarse lattice points both weights vanish and the bilinear degenerates
    // to the coarse sample itself.
    //
    // Coarse-corner density memo: every conforming plane (and the apron
    // trilinear) re-evaluates the same coarse lattice corners two to eight
    // times, and each evaluation is a full raw-field pass. The memo keys a
    // mixed hash of the coarse indices and stores the indices alongside the
    // density so a (vanishingly rare) hash collision is detected and simply
    // recomputed instead of reused. Allocated on first use: chunks without a
    // conforming boundary never pay for it.
    struct CoarseDensityEntry
    {
        std::int64_t x       = 0;
        std::int64_t y       = 0;
        std::int64_t z       = 0;
        double       density = 0.0;
    };
    using CoarseDensityMemo = FlatKeyMap<CoarseDensityEntry>;
    std::unique_ptr<CoarseDensityMemo> coarseDensityMemo;
    const auto coarseDensityAt = [&](std::int64_t coarseX, std::int64_t coarseY, std::int64_t coarseZ) -> double
    {
        // splitMix64 mutates its state parameter, so the mixed coordinates are
        // materialized into locals before each pass.
        std::uint64_t mixedU = static_cast<std::uint64_t>(coarseY) * 0xC2B2AE3D27D4EB4Full
                             ^ static_cast<std::uint64_t>(coarseZ) * 0x165667B19E3779F9ull;
        mixedU = splitMix64(mixedU) ^ static_cast<std::uint64_t>(coarseX) * 0x9E3779B97F4A7C15ull;
        const std::uint64_t mixed = splitMix64(mixedU);
        if (coarseDensityMemo != nullptr)
        {
            if (const CoarseDensityEntry *cached = coarseDensityMemo->find(mixed))
            {
                if (cached->x == coarseX && cached->y == coarseY && cached->z == coarseZ)
                    return cached->density;
                // Hash collision with a different lattice point (probability
                // is negligible, but the exact field value matters):
                // recompute without disturbing the cached entry.
                const Vec3 corner{-half + coarseStep * static_cast<double>(coarseX),
                    -half + coarseStep * static_cast<double>(coarseY),
                    -half + coarseStep * static_cast<double>(coarseZ)};
                return field.densityAt(corner, coarseCutoff);
            }
        }
        else
        {
            coarseDensityMemo = std::make_unique<CoarseDensityMemo>(1024);
        }
        CoarseDensityEntry &entry = coarseDensityMemo->findOrInsert(mixed);
        entry.x                   = coarseX;
        entry.y                   = coarseY;
        entry.z                   = coarseZ;
        const Vec3 corner{-half + coarseStep * static_cast<double>(coarseX),
            -half + coarseStep * static_cast<double>(coarseY), -half + coarseStep * static_cast<double>(coarseZ)};
        entry.density = field.densityAt(corner, coarseCutoff);
        return entry.density;
    };
    const auto coarsePlaneDensity = [&](std::uint32_t axis, std::int64_t perpGlobalFine, std::int64_t tangentialGlobalU,
                                        std::int64_t tangentialGlobalV) -> double
    {
        const std::int64_t perpCoarse = perpGlobalFine >> 1;
        const std::int64_t coarseU0   = tangentialGlobalU >> 1;
        const std::int64_t coarseV0   = tangentialGlobalV >> 1;
        const double       weightU    = (tangentialGlobalU & 1) != 0 ? 0.5 : 0.0;
        const double       weightV    = (tangentialGlobalV & 1) != 0 ? 0.5 : 0.0;

        // Per-axis coarse indices of the four surrounding on-plane coarse
        // lattice points: the perpendicular axis sits at perpCoarse; the
        // tangential axes span coarseU0/u1 and coarseV0/v1. A zero weight
        // degenerates to the coarse sample itself.
        const std::int64_t u1 = coarseU0 + ((tangentialGlobalU & 1) != 0 ? 1 : 0);
        const std::int64_t v1 = coarseV0 + ((tangentialGlobalV & 1) != 0 ? 1 : 0);
        const std::uint32_t axisU  = axis == 0 ? 1u : 0u;
        const std::uint32_t axisV  = axis == 2 ? 1u : 2u;
        std::int64_t        cornerCoarse[4][3] = {};
        const auto          fillCornerCoarse   = [&](std::int64_t (&corner)[3], std::int64_t u, std::int64_t v)
        {
            corner[axis]  = perpCoarse;
            corner[axisU] = u;
            corner[axisV] = v;
        };
        fillCornerCoarse(cornerCoarse[0], coarseU0, coarseV0);
        fillCornerCoarse(cornerCoarse[1], u1, coarseV0);
        fillCornerCoarse(cornerCoarse[2], coarseU0, v1);
        fillCornerCoarse(cornerCoarse[3], u1, v1);

        if (weightV == 0.0)
        {
            const double d0 = coarseDensityAt(cornerCoarse[0][0], cornerCoarse[0][1], cornerCoarse[0][2]);
            const double d1 = coarseDensityAt(cornerCoarse[1][0], cornerCoarse[1][1], cornerCoarse[1][2]);
            return d0 + (d1 - d0) * weightU;
        }
        if (weightU == 0.0)
        {
            const double d0 = coarseDensityAt(cornerCoarse[0][0], cornerCoarse[0][1], cornerCoarse[0][2]);
            const double d1 = coarseDensityAt(cornerCoarse[2][0], cornerCoarse[2][1], cornerCoarse[2][2]);
            return d0 + (d1 - d0) * weightV;
        }
        const double d00 = coarseDensityAt(cornerCoarse[0][0], cornerCoarse[0][1], cornerCoarse[0][2]);
        const double d10 = coarseDensityAt(cornerCoarse[1][0], cornerCoarse[1][1], cornerCoarse[1][2]);
        const double d01 = coarseDensityAt(cornerCoarse[2][0], cornerCoarse[2][1], cornerCoarse[2][2]);
        const double d11 = coarseDensityAt(cornerCoarse[3][0], cornerCoarse[3][1], cornerCoarse[3][2]);
        return d00 * (1.0 - weightU) * (1.0 - weightV) + d10 * weightU * (1.0 - weightV)
             + d01 * (1.0 - weightU) * weightV + d11 * weightU * weightV;
    };

    // Trilinear evaluation of the coarse lattice interpolant at an arbitrary
    // planet-local point - the natural continuation of the conforming field
    // BEYOND the boundary plane, used for the apron points of the extension
    // strip. Its far boundary then matches the coarse neighbor's chords to
    // within the chord error of the smooth bilinear field, instead of the
    // (much larger) chord error of the raw field.
    const auto coarseTrilinear = [&](const Vec3 &p) -> double
    {
        const double scaled[3] = {(p.x + half) / coarseStep, (p.y + half) / coarseStep, (p.z + half) / coarseStep};
        const std::int64_t base[3] = {static_cast<std::int64_t>(std::floor(scaled[0])),
            static_cast<std::int64_t>(std::floor(scaled[1])), static_cast<std::int64_t>(std::floor(scaled[2]))};
        const double fraction[3] = {scaled[0] - static_cast<double>(base[0]), scaled[1] - static_cast<double>(base[1]),
            scaled[2] - static_cast<double>(base[2])};
        double sum = 0.0;
        for (std::uint32_t corner = 0; corner < 8; ++corner)
        {
            const std::int64_t offset[3] = {corner & 1u, (corner >> 1) & 1u, (corner >> 2) & 1u};
            const double weight = (offset[0] != 0 ? fraction[0] : 1.0 - fraction[0])
                                * (offset[1] != 0 ? fraction[1] : 1.0 - fraction[1])
                                * (offset[2] != 0 ? fraction[2] : 1.0 - fraction[2]);
            sum += weight * coarseDensityAt(base[0] + offset[0], base[1] + offset[1], base[2] + offset[2]);
        }
        return sum;
    };

    // One padded lattice point: local indices in [-1, n+1] per axis plus the
    // point's position. Conforming planes are probed in ascending face order
    // (0:-x 1:+x 2:-y 3:+y 4:-z 5:+z); the first conforming hit - which is
    // also the lowest conforming axis - defines the value. Apron points one
    // step beyond a conforming face carry the coarse trilinear field: they
    // feed the extension strip whose far boundary must meet the coarse
    // neighbor's chords (see coarseTrilinear below).
    const auto samplePoint = [&](std::int64_t lx, std::int64_t ly, std::int64_t lz, const Vec3 &p) -> double
    {
        if (!conforming)
        {
            return field.densityAt(p, ownCutoff);
        }
        const std::int64_t globals[3] = {static_cast<std::int64_t>(coord.x) * n + lx,
            static_cast<std::int64_t>(coord.y) * n + ly, static_cast<std::int64_t>(coord.z) * n + lz};
        // Face probe order: axis-major, min side before max side.
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            const std::int64_t chunkBase  = static_cast<std::int64_t>(axis == 0 ? coord.x : axis == 1 ? coord.y : coord.z)
                                          * static_cast<std::int64_t>(n);
            const std::int64_t local      = globals[axis] - chunkBase;
            const bool         minPlane   = local == 0;
            const bool         maxPlane   = local == static_cast<std::int64_t>(n);
            const bool         beyond     = local == -1 || local == static_cast<std::int64_t>(n + 1);
            if (!minPlane && !maxPlane && !beyond)
            {
                continue;
            }
            const std::uint32_t face     = axis * 2u + ((maxPlane || local == static_cast<std::int64_t>(n + 1)) ? 1u : 0u);
            // Tangential axes: x planes span (y, z), y planes span (x, z),
            // z planes span (x, y).
            const std::uint32_t axisU    = axis == 0 ? 1u : 0u;
            const std::uint32_t axisV    = axis == 2 ? 1u : 2u;
            const std::int64_t  globalU  = globals[axisU];
            const std::int64_t  globalV  = globals[axisV];
            if (!minPlane && !maxPlane)
            {
                // Apron point one step beyond a conforming face: it lies in
                // the coarse neighbor's territory and feeds the extension
                // strip, so it carries the coarse trilinear field - the
                // extension's far boundary then meets the coarse chords to
                // within the bilinear chord error.
                const std::uint32_t apronFace = face;
                const std::int64_t apronU = globalU;
                const std::int64_t apronV = globalV;
                const std::int64_t apronChunkU = static_cast<std::int64_t>(axisU == 0 ? coord.x : axisU == 1 ? coord.y : coord.z);
                const std::int64_t apronChunkV = static_cast<std::int64_t>(axisV == 0 ? coord.x : axisV == 1 ? coord.y : coord.z);
                const int apronOffsetU = static_cast<int>(floorDiv(apronU, 2 * n) - apronChunkU / 2);
                const int apronOffsetV = static_cast<int>(floorDiv(apronV, 2 * n) - apronChunkV / 2);
                if (apronOffsetU >= -1 && apronOffsetU <= 1 && apronOffsetV >= -1 && apronOffsetV <= 1
                    && conformBit(conformMask, apronFace, apronOffsetU, apronOffsetV))
                {
                    return coarseTrilinear(p);
                }
                continue;
            }
            if ((globalU & 1) != 0 || (globalV & 1) != 0)
            {
                // The depth-(d-1) octree cell beyond this plane containing the
                // point: floor(global / (2n)) per axis, expressed as an offset
                // from the chunk's own coarse-cell block (a depth-(d-1) cell
                // spans 2n fine steps). Offset 0 governs the face interior;
                // +-1 are the diagonal cells reached by apron overshoot.
                const std::int64_t chunkU = static_cast<std::int64_t>(axisU == 0 ? coord.x : axisU == 1 ? coord.y : coord.z);
                const std::int64_t chunkV = static_cast<std::int64_t>(axisV == 0 ? coord.x : axisV == 1 ? coord.y : coord.z);
                const int offsetU = static_cast<int>(floorDiv(globalU, 2 * n) - chunkU / 2);
                const int offsetV = static_cast<int>(floorDiv(globalV, 2 * n) - chunkV / 2);
                if (offsetU < -1 || offsetU > 1 || offsetV < -1 || offsetV > 1)
                {
                    continue; // outside the 3x3 ring (cannot happen for plane points)
                }
                if (!conformBit(conformMask, face, offsetU, offsetV))
                {
                    continue; // material beyond this segment is not a coarse leaf
                }
                return coarsePlaneDensity(axis, globals[axis], globalU, globalV);
            }
            // Coarse lattice point. On a conforming segment this must be the
            // COARSE side's value (the coarse neighbor evaluates its own
            // lattice with cutoff(d-1); with per-depth band limiting that is
            // no longer "raw exactly"), and coarsePlaneDensity degenerates to
            // exactly that sample. On a non-conforming segment the point is
            // shared with same-depth neighbors that evaluate it at cutoff(d)
            // - the fallthrough below does the same.
            const std::int64_t chunkU = static_cast<std::int64_t>(axisU == 0 ? coord.x : axisU == 1 ? coord.y : coord.z);
            const std::int64_t chunkV = static_cast<std::int64_t>(axisV == 0 ? coord.x : axisV == 1 ? coord.y : coord.z);
            const int offsetU = static_cast<int>(floorDiv(globalU, 2 * n) - chunkU / 2);
            const int offsetV = static_cast<int>(floorDiv(globalV, 2 * n) - chunkV / 2);
            if (offsetU >= -1 && offsetU <= 1 && offsetV >= -1 && offsetV <= 1
                && conformBit(conformMask, face, offsetU, offsetV))
            {
                return coarsePlaneDensity(axis, globals[axis], globalU, globalV);
            }
        }
        return field.densityAt(p, ownCutoff);
    };

    // ---- sample the padded grids -------------------------------------------------
    // The grids live in per-thread scratch: a build fully overwrites every
    // element before any read (the loops below cover all gridSize indices),
    // so reuse cannot leak state between builds, and the worker pool stops
    // paying an allocation per chunk for a constant-size buffer.
    thread_local std::vector<double> densitiesScratch;
    thread_local std::vector<double> watersScratch;
    std::vector<double> &densities = densitiesScratch;
    densities.resize(gridSize);
    std::vector<double> &waters = watersScratch;
    waters.clear();
    if (nodeTouchesSea)
    {
        waters.resize(gridSize);
    }
    // Per-axis sample positions, derived once from the same global lattice
    // expression the boundary machinery relies on (bit-identical across
    // chunks at every depth).
    std::array<double, 64 + 3> axisX{};
    std::array<double, 64 + 3> axisY{};
    std::array<double, 64 + 3> axisZ{};
    for (std::uint32_t index = 0; index < paddedN; ++index)
    {
        const std::int64_t inner = static_cast<std::int64_t>(index) - 1;
        axisX[index] = axisCoordinate(coord.x, inner);
        axisY[index] = axisCoordinate(coord.y, inner);
        axisZ[index] = axisCoordinate(coord.z, inner);
    }
    for (std::uint32_t pz = 0; pz < paddedN; ++pz)
    {
        for (std::uint32_t py = 0; py < paddedN; ++py)
        {
            for (std::uint32_t px = 0; px < paddedN; ++px)
            {
                const std::int64_t ix = static_cast<std::int64_t>(px) - 1;
                const std::int64_t iy = static_cast<std::int64_t>(py) - 1;
                const std::int64_t iz = static_cast<std::int64_t>(pz) - 1;
                const Vec3 p{axisX[px], axisY[py], axisZ[pz]};
                const std::size_t index  = px + strideY * py + strideZ * pz;
                densities[index]         = samplePoint(ix, iy, iz, p);
                if (nodeTouchesSea)
                {
                    waters[index] = params.derivedSeaRadiusMeters - length(p);
                }
            }
        }
    }

    const auto sampleGrid = [&](const std::vector<double> &grid, std::uint32_t px, std::uint32_t py, std::uint32_t pz)
    { return grid[static_cast<std::size_t>(px) + strideY * py + strideZ * pz]; };

    // ---- marching cubes over the inner cells --------------------------------------
    std::vector<PlanetVertex>                        &vertices = mesh.terrainVertices;
    std::vector<std::uint32_t>                       &indices  = mesh.terrainIndices;
    // Edge-vertex dedup over the chunk's own lattice points; the flat map is
    // the project's open-addressing table (single cache line per lookup, no
    // node allocations) and the keys are chunk-local. Like the density grids
    // the table lives in per-thread scratch: every slot the build touches is
    // overwritten before any read (findOrInsert seeds new keys), so reuse
    // cannot leak state between builds, and the worker pool stops paying a
    // constant-size allocation per chunk.
    thread_local FlatKeyMap<std::uint32_t> vertexForEdgeScratch(4096);
    FlatKeyMap<std::uint32_t> &vertexForEdge = vertexForEdgeScratch;
    vertexForEdge.clear();

    // Spherical reference frame of the spin axis (see the UV mapping below).
    // With the default upright axis this reduces to east = +x, north = +z.
    const Vec3 axisVector{params.axis[0], params.axis[1], params.axis[2]};
    Vec3 equatorEast = normalized(cross(axisVector, Vec3{0.0, 0.0, 1.0}));
    if (length(equatorEast) < 1.0e-6)
    {
        equatorEast = Vec3{1.0, 0.0, 0.0}; // axis parallel to z: any east works
    }
    const Vec3 equatorNorth = cross(equatorEast, axisVector);

    // Gradient of the trilinear interpolant at an edge, averaged over the (up
    // to) four cells adjacent to the edge on the canonical side. A single-cell
    // gradient is constant per cell: neighboring vertices then carry different
    // constant normals, and every steep crease speckles with dark cell-sized
    // facets (visible as dotted dark lines along ridges). Averaging the four
    // adjacent cells makes the normal field continuous across cell borders.
    // The canonical min-corner bookkeeping keeps the average a pure function
    // of the edge's global position: the sampled density block (2x3x3 around
    // the canonical corner) always lies inside the 1-voxel apron, so two
    // chunks sharing a boundary edge - at the same depth or across depths -
    // evaluate the exact same numbers and carry bit-identical normals.
    const auto edgeGradient = [&](std::uint32_t ax, std::uint32_t ay, std::uint32_t az, std::uint32_t axis, double outGradient[3])
    {
        const std::uint32_t axisB = axis == 0 ? 1u : 0u; // first non-edge axis
        const std::uint32_t axisC = axis == 2 ? 1u : 2u; // second non-edge axis
        double       sum[3]    = {0.0, 0.0, 0.0};
        std::uint32_t cells    = 0;
        for (std::uint32_t cell = 0; cell < 4; ++cell)
        {
            const std::int64_t offsetB = (cell & 1u) != 0u ? 0 : -1;
            const std::int64_t offsetC = (cell & 2u) != 0u ? 0 : -1;
            double corners[2][2][2];
            bool   valid = true;
            for (std::uint32_t i = 0; i < 2 && valid; ++i)
            {
                for (std::uint32_t j = 0; j < 2 && valid; ++j)
                {
                    for (std::uint32_t k = 0; k < 2 && valid; ++k)
                    {
                        const std::int64_t mapped[3] = {static_cast<std::int64_t>(i), static_cast<std::int64_t>(j) + offsetB,
                            static_cast<std::int64_t>(k) + offsetC};
                        std::int64_t address[3] = {static_cast<std::int64_t>(ax), static_cast<std::int64_t>(ay),
                            static_cast<std::int64_t>(az)};
                        address[axis] += mapped[0];
                        address[axisB] += mapped[1];
                        address[axisC] += mapped[2];
                        if (address[0] < 0 || address[1] < 0 || address[2] < 0 || address[0] > std::int64_t(paddedN - 1)
                            || address[1] > std::int64_t(paddedN - 1) || address[2] > std::int64_t(paddedN - 1))
                        {
                            // The cell block reaches past the padded grid
                            // (edges of the own cells sit one step inside the
                            // apron; edges of EXTENDED apron cells sit on the
                            // grid border): skip this neighbor cell. The skip
                            // pattern is a pure function of the canonical
                            // corner, so every chunk evaluating the same edge
                            // skips identically.
                            valid = false;
                            break;
                        }
                        corners[i][j][k] = sampleGrid(densities, static_cast<std::uint32_t>(address[0]),
                            static_cast<std::uint32_t>(address[1]), static_cast<std::uint32_t>(address[2]));
                    }
                }
            }
            if (!valid)
            {
                continue;
            }
            // Central differences of the trilinear interpolant at the cell
            // center (face-averaged directional derivatives).
            double derivative[3] = {0.0, 0.0, 0.0};
            derivative[axis] = 0.25 * ((corners[1][0][0] + corners[1][0][1] + corners[1][1][0] + corners[1][1][1])
                                     - (corners[0][0][0] + corners[0][0][1] + corners[0][1][0] + corners[0][1][1]));
            derivative[axisB] = 0.25 * ((corners[0][1][0] + corners[0][1][1] + corners[1][1][0] + corners[1][1][1])
                                      - (corners[0][0][0] + corners[0][0][1] + corners[1][0][0] + corners[1][0][1]));
            derivative[axisC] = 0.25 * ((corners[0][0][1] + corners[0][1][1] + corners[1][0][1] + corners[1][1][1])
                                      - (corners[0][0][0] + corners[0][1][0] + corners[1][0][0] + corners[1][1][0]));
            sum[axis]   += derivative[axis];
            sum[axisB]  += derivative[axisB];
            sum[axisC]  += derivative[axisC];
            cells       += 1;
        }
        if (cells == 0)
        {
            outGradient[0] = 0.0;
            outGradient[1] = 0.0;
            outGradient[2] = 0.0;
            return;
        }
        outGradient[axis]  = sum[axis] / (static_cast<double>(cells) * step);
        outGradient[axisB] = sum[axisB] / (static_cast<double>(cells) * step);
        outGradient[axisC] = sum[axisC] / (static_cast<double>(cells) * step);
    };

    // ---- marching cubes -------------------------------------------------------------
    //
    // Own cells [0, n)^3 always march. Apron cells (any index at -1 or n)
    // march ONLY when they touch a conforming face: their surface EXTENDS the
    // chunk one cell layer past the transition into the coarse neighbor's
    // territory, where the neighbor's own surface overlaps it - the
    // interpenetration is what hides the sub-voxel boundary mismatch (see the
    // ConformMask comment in meshing.hpp). Apron cells past non-conforming
    // faces stay unmarched: same-depth neighbors already weld bit-identically,
    // and duplicate geometry there would only cost fill rate.
    const auto cellNeedsMarching = [&](std::int64_t cx, std::int64_t cy, std::int64_t cz) -> bool
    {
        if (cx >= 0 && cx < static_cast<std::int64_t>(n) && cy >= 0 && cy < static_cast<std::int64_t>(n) && cz >= 0
            && cz < static_cast<std::int64_t>(n))
        {
            return true; // own cell
        }
        const std::int64_t cell[3] = {cx, cy, cz};
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            const bool minApron = cell[axis] == -1;
            const bool maxApron = cell[axis] == static_cast<std::int64_t>(n);
            if (!minApron && !maxApron)
            {
                continue;
            }
            const std::uint32_t face = axis * 2u + (maxApron ? 1u : 0u);
            const std::uint32_t axisU = axis == 0 ? 1u : 0u;
            const std::uint32_t axisV = axis == 2 ? 1u : 2u;
            // The apron cell lies within ONE coarse-cell segment per
            // tangential axis; use its minimum corner to find which.
            const std::int64_t chunkU = static_cast<std::int64_t>(axisU == 0 ? coord.x : axisU == 1 ? coord.y : coord.z);
            const std::int64_t chunkV = static_cast<std::int64_t>(axisV == 0 ? coord.x : axisV == 1 ? coord.y : coord.z);
            const std::int64_t globalUMin = chunkU * n + cell[axisU];
            const std::int64_t globalVMin = chunkV * n + cell[axisV];
            const int offsetU = static_cast<int>(floorDiv(globalUMin, 2 * n) - chunkU / 2);
            const int offsetV = static_cast<int>(floorDiv(globalVMin, 2 * n) - chunkV / 2);
            if (offsetU >= -1 && offsetU <= 1 && offsetV >= -1 && offsetV <= 1
                && conformBit(conformMask, face, offsetU, offsetV))
            {
                return true;
            }
        }
        return false;
    };

    // Terrain pass: own cells [0, n)^3 always march (cellNeedsMarching
    // above); vertices carry gradient normals and the climate sampler's
    // blended albedo; triangles use the fixed terrain winding.
    auto terrainMakeVertex = [&](std::uint32_t aX, std::uint32_t aY, std::uint32_t aZ, std::uint32_t, std::uint32_t,
        std::uint32_t, std::uint32_t edgeAxis, double, const Vec3 &point) -> std::uint32_t
    {
        double gradient[3] = {0.0, 0.0, 0.0};
        edgeGradient(aX, aY, aZ, edgeAxis, gradient);
        // d > 0 = inside: the gradient points into the body, surface normals
        // point outward against it.
        double nx = -gradient[0], ny = -gradient[1], nz = -gradient[2];
        const double normalLength = std::sqrt(nx * nx + ny * ny + nz * nz);

        const double radius     = std::max(1.0e-9, length(point));
        const Vec3   unitRadial = point * (1.0 / radius);
        if (normalLength > 1.0e-12)
        {
            nx /= normalLength;
            ny /= normalLength;
            nz /= normalLength;
        }
        else
        {
            nx = unitRadial.x;
            ny = unitRadial.y;
            nz = unitRadial.z;
        }

        const double slope01 = std::clamp(1.0 - (nx * unitRadial.x + ny * unitRadial.y + nz * unitRadial.z), 0.0, 1.0);
        // The albedo jitter band-limits at the VERTEX spacing (the resolution
        // the colors are sampled at): octave scales below the spacing alias
        // into uncorrelated per-vertex speckle, so they fade out on coarse
        // LOD chunks exactly like sub-voxel geometry octaves, while the
        // climate classification itself stays exact.
        const ClimateSample climate = field.climate().sample(unitRadial, radius, slope01, step);

        PlanetVertex vertex{};
        vertex.position[0] = static_cast<float>(point.x - center.x);
        vertex.position[1] = static_cast<float>(point.y - center.y);
        vertex.position[2] = static_cast<float>(point.z - center.z);
        vertex.normal[0]   = static_cast<float>(nx);
        vertex.normal[1]   = static_cast<float>(ny);
        vertex.normal[2]   = static_cast<float>(nz);
        // Cloud shadows: the same coverage field the cloud layer renders
        // from, sampled at the vertex's radial direction and band-limited at
        // the chunk's own spacing, darkens the albedo. The banks overhead
        // and their shadows then agree from orbit and from the ground, and
        // the shadow follows the LOD band limit like every other field
        // (coarse chunks get smooth shade, not speckle).
        const double cloudShade = 1.0 - field.cloudShadow(unitRadial, step);
        vertex.color[0]    = climate.color[0] * static_cast<float>(cloudShade);
        vertex.color[1]    = climate.color[1] * static_cast<float>(cloudShade);
        vertex.color[2]    = climate.color[2] * static_cast<float>(cloudShade);
        // Spherical mapping around the SPIN AXIS, not the world Y axis:
        // climate latitude already follows the axis, and the two disagree
        // when the axis is tilted. With the default upright axis the frame
        // reduces to the plain atan2(z, x) mapping.
        const double longitude = std::atan2(dot(unitRadial, equatorNorth), dot(unitRadial, equatorEast)) * 0.15915494309189535 + 0.5;
        const double latitude  = 0.5 - std::asin(std::clamp(dot(unitRadial, axisVector), -1.0, 1.0)) * 0.3183098861837907;
        vertex.uv[0] = static_cast<float>(longitude);
        vertex.uv[1] = static_cast<float>(latitude);

        const std::uint32_t slot = static_cast<std::uint32_t>(vertices.size());
        vertices.push_back(vertex);
        return slot;
    };
    auto terrainEmitTriangle = [&](std::uint32_t cubeIndex, const std::uint32_t *slotVertex)
    {
        for (std::uint32_t triangle = 0; triangle < 5; ++triangle)
        {
            const std::int8_t v0 = detail::kTriTable[cubeIndex][triangle * 3 + 0];
            if (v0 < 0)
            {
                break;
            }
            const std::int8_t v1 = detail::kTriTable[cubeIndex][triangle * 3 + 1];
            const std::int8_t v2 = detail::kTriTable[cubeIndex][triangle * 3 + 2];
            if (!detail::kTerrainFlipWinding)
            {
                indices.insert(indices.end(), {slotVertex[static_cast<std::uint32_t>(v0)],
                    slotVertex[static_cast<std::uint32_t>(v1)], slotVertex[static_cast<std::uint32_t>(v2)]});
            }
            else
            {
                indices.insert(indices.end(), {slotVertex[static_cast<std::uint32_t>(v0)],
                    slotVertex[static_cast<std::uint32_t>(v2)], slotVertex[static_cast<std::uint32_t>(v1)]});
            }
            mesh.triangleCount += 1;
        }
    };
    marchCells(densities, nullptr, -1, static_cast<std::int64_t>(n), axisX, axisY, axisZ, paddedN, vertexForEdge,
        cellNeedsMarching,
        [](std::uint32_t cubeIndex, double) { return cubeIndex == 0u || cubeIndex == 255u; },
        terrainMakeVertex, terrainEmitTriangle);


    // ---- chunk-boundary continuity ----------------------------------------------------
    //
    // No skirt walls: same-depth neighbors sample a shared global lattice, so
    // their boundary vertices are bit-identical and the surface is watertight
    // by construction. Across depth changes, the octree enforces a 2:1 balance
    // (see octree.hpp), which keeps the T-junction residual between the coarse
    // chord and the fine isoline far below one voxel - invisible, and far
    // cheaper than painted-over crack walls.
    // ---- water surface patch --------------------------------------------------------
    if (nodeTouchesSea)
    {
        std::vector<PlanetVertex>  &waterVertices = mesh.waterVertices;
        std::vector<std::uint32_t> &waterIndices  = mesh.waterIndices;
        // Water edge-dedup lives in per-thread scratch like the terrain table.
        thread_local FlatKeyMap<std::uint32_t> waterForEdgeScratch(1024);
        FlatKeyMap<std::uint32_t> &waterForEdge = waterForEdgeScratch;
        waterForEdge.clear();

        // Wave-slope band limit: like every noise term of the module, the
        // wave slope fades out below the chunk's own LOD cutoff, so coarse
        // chunks keep a smooth sea while fine chunks carry the glint
        // texture. The wave seeds derive from the planet seed via the same
        // schedule as every other stream. Two scales ride the same seeds -
        // the configured swell and a half-wavelength chop at a third of the
        // tilt - so close-range seas carry both long undulation and fine
        // glitter, and the sun glint breaks up into sparkle instead of one
        // broad mirror band.
        const std::int64_t seedWaveA = static_cast<std::int64_t>(hashCombine64(params.seed, 0x3A7E51ULL));
        const std::int64_t seedWaveB = static_cast<std::int64_t>(hashCombine64(params.seed, 0x51B0E5ULL));
        const double waveWavelength = std::max(1.0, params.waterWaveWavelengthMeters);
        const double waveFade = lowPassFade(ownCutoff, waveWavelength);
        const double chopFade = lowPassFade(ownCutoff, 0.5 * waveWavelength);
        const double waveAmplitude = params.waterWaveAmplitude * waveFade;
        const double chopAmplitude = 0.33 * params.waterWaveAmplitude * chopFade;
        const double waveFrequency = 1.0 / waveWavelength;
        const double chopFrequency = 2.0 / waveWavelength;

        // Foam seed: the breaker ranks key to the swell's own scale (see the
        // surf block below), whose wavelength stretches with the chunk's
        // sample spacing so coarse chunks keep a smooth line.
        const std::int64_t seedFoam = static_cast<std::int64_t>(hashCombine64(params.seed, 0xF04A1ULL));

        // Per-channel depth tint: water absorbs red fastest, green next,
        // blue last (the physical absorption ordering), so the configured
        // water_depth_tint_meters is the BLUE channel's saturation depth and
        // red/green saturate at fixed fractions of it. The tint then runs
        // shallow turquoise -> green-teal mid shelf -> navy deep instead of
        // one gray-blue mix; the shallow color keeps the configured hue.
        //
        // On coarse chunks the band-limited lattice must still resolve the
        // tint gradient, or per-vertex saturation paints diamond-shaped
        // shallows around every island. The band therefore widens with the
        // chunk's own sample spacing (the same LOD rule every noise term
        // follows) - with an abyssal ocean profile the widening only spreads
        // the genuine shelf around landmasses; open water stays deep.
        const double tintDepthBlue = std::max(1.0, params.waterDepthTintMeters);
        const double tintDepth = std::max(tintDepthBlue, ownCutoff * 0.3);
        const double tintDepths[3] = {0.28 * tintDepth, 0.6 * tintDepth, tintDepth};
        const double shoreWeight = 1.0 - smoothstep(2.0 * params.waterShoreMixMeters, 4.0 * params.waterShoreMixMeters,
            ownCutoff);
        // Brightness of the sand-through-water shore blend (the beach row
        // already carries the sand albedo; sunlit flats under a thin sheet
        // read brighter than the raw row).
        constexpr float kShoreSandBrightness = 1.35f;
        // Shore blend constants (per chunk, not per vertex): whether the
        // blend runs at all and the beach-sand tone transmitted by the
        // water's own hue - the shallow color's channel balance scaled so
        // its brightest channel reads 1.
        const bool shoreBlend = params.waterShoreMixMeters > 0.0;
        float sandHue[3] = {1.0f, 1.0f, 1.0f};
        if (shoreBlend)
        {
            const float brightest = std::max({params.waterColorShallow[0], params.waterColorShallow[1],
                params.waterColorShallow[2], 1.0e-4f});
            for (std::uint32_t channel = 0; channel < 3; ++channel)
            {
                sandHue[channel] = params.waterColorShallow[channel] / brightest;
            }
        }

        // Water pass: own cells only, skip cells fully buried under the
        // terrain surface (the terrain grid rides along as buriedField), and
        // auto-orient every triangle against the true radial direction.
        auto waterMakeVertex = [&](std::uint32_t aX, std::uint32_t aY, std::uint32_t aZ, std::uint32_t bX,
            std::uint32_t bY, std::uint32_t bZ, std::uint32_t edgeAxis, double, const Vec3 &point) -> std::uint32_t
        {
            const double radius = std::max(1.0e-9, length(point));
            const Vec3   radial = point * (1.0 / radius);

            // Water depth = the distance from the sea-surface point down to
            // the terrain surface, straight from the field: the density is
            // the signed distance to the terrain (positive inside rock), so
            // the depth is its NEGATION clamped at the shoreline. The
            // per-chunk trilinear grid would make the tint LOD-dependent:
            // neighboring chunks at different depths disagree, painting
            // straight color edges into open water. The exact field is one
            // pure function - every chunk, at any depth, agrees bit-for-bit.
            //
            // Open ocean skips that exact evaluation: the tint saturates at
            // the blue channel's depth (every channel reads the deep color
            // from there down, and foam and shore blend only live within a
            // few meters of the shoreline), so a vertex whose whole edge
            // already sits that deep gets its depth from the chunk's own
            // lattice - the two endpoint densities the marching-cubes walk
            // itself interpolated. Band-limited fields cannot rise out of a
            // deep edge within one cell, so the probe only fires on truly
            // deep water; anywhere near the shore the exact field takes over
            // and keeps the bit-exact cross-depth agreement.
            double terrainDepth = 0.0;
            {
                const double densityA = densities[static_cast<std::size_t>(aX) + strideY * aY + strideZ * aZ];
                const double densityB = densities[static_cast<std::size_t>(bX) + strideY * bY + strideZ * bZ];
                if (-std::min(densityA, densityB) >= tintDepths[2])
                {
                    terrainDepth = tintDepths[2]; // saturated: every tint channel reads the deep color
                }
                else
                {
                    terrainDepth = std::max(0.0, -field.densityAt(point));
                }
            }

            // Surf: the breaker band lives in the shallow margin of the
            // configured width. Real surf reads as LINES: waves refract
            // until their crests run parallel to the beach, then break in
            // ranks. The shore frame comes from the terrain density
            // gradient at the sea-surface point, read straight from the
            // density lattice the water pass already carries (central
            // differences around the edge's canonical corner - no extra
            // field evaluations): its tangential component points landward,
            // so the negation is offshore, and the crest noise advances
            // offshore while holding along the beach - elongated breaker
            // ranks following the coastline instead of isotropic
            // cotton-wool patches. Between ranks a base wash keeps the band
            // from reading as empty water. The crest wavelength stretches
            // with the chunk's sample spacing so coarse chunks keep a
            // smooth line instead of aliasing the ranks. The band's falloff
            // is cubed: real breakers hug the waterline, and a plain
            // smoothstep over a shallow shelf would wash the whole lagoon
            // white.
            double foam = 0.0;
            if (params.waterFoamStrength > 0.0 && params.waterFoamWidthMeters > 0.0)
            {
                const double depthBand
                    = 1.0 - smoothstep(0.0, params.waterFoamWidthMeters, terrainDepth);
                const double foamBand = depthBand * depthBand * depthBand;
                if (foamBand > 0.0)
                {
                    double crestModulation = 0.30;
                    {
                        // Tangential density gradient from the lattice: the
                        // vertex sits on a lattice edge, so two of its
                        // coordinates are exact grid lines and the central
                        // differences around the canonical corner read the
                        // grid directly (both neighbors always lie inside the
                        // padded grid for own-cell edges).
                        double gradient[3] = {0.0, 0.0, 0.0};
                        if (edgeAxis == 0u)
                        {
                            gradient[1] = densities[static_cast<std::size_t>(aX) + strideY * (aY + 1u) + strideZ * aZ]
                                - densities[static_cast<std::size_t>(aX) + strideY * (aY - 1u) + strideZ * aZ];
                            gradient[2] = densities[static_cast<std::size_t>(aX) + strideY * aY + strideZ * (aZ + 1u)]
                                - densities[static_cast<std::size_t>(aX) + strideY * aY + strideZ * (aZ - 1u)];
                        }
                        else if (edgeAxis == 1u)
                        {
                            gradient[0] = densities[static_cast<std::size_t>(aX + 1u) + strideY * aY + strideZ * aZ]
                                - densities[static_cast<std::size_t>(aX - 1u) + strideY * aY + strideZ * aZ];
                            gradient[2] = densities[static_cast<std::size_t>(aX) + strideY * aY + strideZ * (aZ + 1u)]
                                - densities[static_cast<std::size_t>(aX) + strideY * aY + strideZ * (aZ - 1u)];
                        }
                        else
                        {
                            gradient[0] = densities[static_cast<std::size_t>(aX + 1u) + strideY * aY + strideZ * aZ]
                                - densities[static_cast<std::size_t>(aX - 1u) + strideY * aY + strideZ * aZ];
                            gradient[1] = densities[static_cast<std::size_t>(aX) + strideY * (aY + 1u) + strideZ * aZ]
                                - densities[static_cast<std::size_t>(aX) + strideY * (aY - 1u) + strideZ * aZ];
                        }
                        const Vec3 landwardTangential = Vec3{gradient[0], gradient[1], gradient[2]}
                            - radial * (gradient[0] * radial.x + gradient[1] * radial.y + gradient[2] * radial.z);
                        const double landwardLength = length(landwardTangential);
                        if (landwardLength > 1.0e-9)
                        {
                            const Vec3 offshore = landwardTangential * (-1.0 / landwardLength);
                            const Vec3 alongShore = cross(radial, offshore);
                            const double crestWavelength = std::max(0.5 * waveWavelength, 2.0 * step);
                            const double crestPhase = dot(point, offshore) / crestWavelength
                                + 0.25 * dot(point, alongShore) / (6.0 * crestWavelength);
                            const double crestNoise = noise::simplex3ImproveXY(seedFoam,
                                crestPhase, dot(point, radial) / (24.0 * crestWavelength), 0.0);
                            const double crest = 0.5 + 0.5 * crestNoise;
                            const double ranks = smoothstep(0.30, 0.75, crest);
                            crestModulation = 0.35 + 0.65 * ranks;
                        }
                    }
                    foam = params.waterFoamStrength * foamBand * crestModulation;
                }
            }

            // Sea ice: the water freezes where the sea-level
            // temperature drops below the configured freeze
            // point. The temperature field is the SAME one
            // that drives the land's snow line, so polar
            // oceans ice over where the land turns tundra,
            // with a noisy organic edge instead of a latitude
            // ring. 0 disables.
            double iceWeight = 0.0;
            if (params.seaIceTemperature > 0.0)
            {
                const double freeze = params.seaIceTemperature;
                const double transition = std::max(1.0e-4, params.seaIceTransition);
                iceWeight = 1.0 - smoothstep(freeze - transition, freeze + transition,
                    field.climate().seaLevelTemperature(radial));
            }

            // Wave slope: two deterministic noise streams tilt
            // the radial normal along a local tangential
            // frame, giving the sea surface a sun-glint
            // texture at close range. The tilt rides TWO
            // scales - the configured swell and a
            // half-wavelength chop - so the glint breaks
            // into sparkle; both fade under the chunk's band
            // limit like every other noise term. The frame
            // degenerates gracefully at the poles (the
            // tangent basis picks whichever world axis is
            // least aligned with the radial direction).
            double nx = radial.x, ny = radial.y, nz = radial.z;
            if (waveAmplitude > 0.0 || chopAmplitude > 0.0)
            {
                Vec3 tangentA = cross(radial, std::abs(radial.y) < 0.9 ? Vec3{0.0, 1.0, 0.0} : Vec3{1.0, 0.0, 0.0});
                tangentA = normalized(tangentA);
                const Vec3 tangentB = cross(radial, tangentA);
                double slopeA = 0.0;
                double slopeB = 0.0;
                if (waveAmplitude > 0.0)
                {
                    slopeA = waveAmplitude * noise::simplex3ImproveXY(seedWaveA,
                        point.x * waveFrequency, point.y * waveFrequency, point.z * waveFrequency);
                    slopeB = waveAmplitude * noise::simplex3ImproveXY(seedWaveB,
                        point.x * waveFrequency, point.y * waveFrequency, point.z * waveFrequency);
                }
                if (chopAmplitude > 0.0)
                {
                    slopeA += chopAmplitude * noise::simplex3ImproveXY(seedWaveA,
                        point.x * chopFrequency, point.y * chopFrequency, point.z * chopFrequency);
                    slopeB += chopAmplitude * noise::simplex3ImproveXY(seedWaveB,
                        point.x * chopFrequency, point.y * chopFrequency, point.z * chopFrequency);
                }
                const Vec3 waved = radial + tangentA * slopeA + tangentB * slopeB;
                const double wavedLength = length(waved);
                if (wavedLength > 1.0e-9)
                {
                    nx = waved.x / wavedLength;
                    ny = waved.y / wavedLength;
                    nz = waved.z / wavedLength;
                }
            }

            PlanetVertex vertex{};
            vertex.position[0] = static_cast<float>(point.x - center.x);
            vertex.position[1] = static_cast<float>(point.y - center.y);
            vertex.position[2] = static_cast<float>(point.z - center.z);
            vertex.normal[0]   = static_cast<float>(nx);
            vertex.normal[1]   = static_cast<float>(ny);
            vertex.normal[2]   = static_cast<float>(nz);
            // Cloud shadows ride the water sheet too: the same coverage
            // field keyed to the vertex's radial direction, so a bank's
            // shadow darkens the sea surface beneath it exactly like the
            // terrain beside it.
            const double cloudShade = 1.0 - field.cloudShadow(radial, step);
            for (std::uint32_t channel = 0; channel < 3; ++channel)
            {
                const float shallow = params.waterColorShallow[channel];
                const float deep    = params.waterColorDeep[channel];
                const double channelTint
                    = std::clamp(terrainDepth / tintDepths[channel], 0.0, 1.0);
                float blended = static_cast<float>(shallow + (deep - shallow) * channelTint);
                if (shoreBlend)
                {
                    const double shoreFade
                        = (1.0 - smoothstep(0.0, params.waterShoreMixMeters, terrainDepth)) * shoreWeight;
                    if (shoreFade > 0.0)
                    {
                        // Sand seen through the water: the beach palette row
                        // under the shallow color's transmission tint, lifted
                        // by the sun the flats receive.
                        const float sand = params.biomePalette[static_cast<std::size_t>(Biome::Beach) * 3u + channel];
                        const float shoreTarget = std::clamp(sand * sandHue[channel] * kShoreSandBrightness, 0.0f, 1.0f);
                        blended = static_cast<float>(shoreTarget + (blended - shoreTarget)
                            * smoothstep(0.0, params.waterShoreMixMeters, terrainDepth));
                    }
                }
                blended       = static_cast<float>(blended + (1.0f - blended) * foam);
                const float icy     = params.waterColorIce[channel];
                blended             = static_cast<float>(blended + (icy - blended) * iceWeight);
                vertex.color[channel] = blended * static_cast<float>(cloudShade);
            }
            vertex.uv[0] = 0.0f;
            vertex.uv[1] = 0.0f;

            const std::uint32_t slot = static_cast<std::uint32_t>(waterVertices.size());
            waterVertices.push_back(vertex);
            return slot;
        };
        auto waterEmitTriangle = [&](std::uint32_t cubeIndex, const std::uint32_t *slotVertex)
        {
            for (std::uint32_t triangle = 0; triangle < 5; ++triangle)
            {
                const std::int8_t v0 = detail::kTriTable[cubeIndex][triangle * 3 + 0];
                if (v0 < 0)
                {
                    break;
                }
                const std::int8_t v1 = detail::kTriTable[cubeIndex][triangle * 3 + 1];
                const std::int8_t v2 = detail::kTriTable[cubeIndex][triangle * 3 + 2];
                // The sea surface only ever faces away from the
                // center: auto-orient each triangle radially.
                const PlanetVertex &pv0 = waterVertices[slotVertex[static_cast<std::uint32_t>(v0)]];
                const PlanetVertex &pv1 = waterVertices[slotVertex[static_cast<std::uint32_t>(v1)]];
                const PlanetVertex &pv2 = waterVertices[slotVertex[static_cast<std::uint32_t>(v2)]];
                const Vec3 p0{pv0.position[0], pv0.position[1], pv0.position[2]};
                const Vec3 p1{pv1.position[0], pv1.position[1], pv1.position[2]};
                const Vec3 p2{pv2.position[0], pv2.position[1], pv2.position[2]};
                // Orient against the TRUE radial direction: the
                // centroid in planet-local space (chunk-local
                // vertices alone point no particular way - the
                // water sheet is flat inside its chunk).
                const Vec3 centroidWorld = (p0 + p1 + p2) * (1.0 / 3.0) + center;
                const bool flip = dot(cross(p1 - p0, p2 - p0), normalized(centroidWorld)) < 0.0;
                if (!flip)
                {
                    waterIndices.insert(waterIndices.end(), {slotVertex[static_cast<std::uint32_t>(v0)],
                        slotVertex[static_cast<std::uint32_t>(v1)], slotVertex[static_cast<std::uint32_t>(v2)]});
                }
                else
                {
                    waterIndices.insert(waterIndices.end(), {slotVertex[static_cast<std::uint32_t>(v0)],
                        slotVertex[static_cast<std::uint32_t>(v2)], slotVertex[static_cast<std::uint32_t>(v1)]});
                }
                mesh.triangleCount += 1;
            }
        };
        marchCells(waters, &densities, 0, static_cast<std::int64_t>(n) - 1, axisX, axisY, axisZ, paddedN, waterForEdge,
            [](std::int64_t, std::int64_t, std::int64_t) { return true; },
            [](std::uint32_t cubeIndex, double minBuried)
            {
                if (cubeIndex == 0u || cubeIndex == 255u)
                {
                    return true;
                }
                // Fully buried water: the terrain surface is above the sea
                // everywhere in this cell. The patch here is invisible
                // anyway - except as grazing-angle slivers where it grazes
                // the terrain surface. Skip it.
                return minBuried > 0.0;
            },
            waterMakeVertex, waterEmitTriangle);
    }

    // ---- hash (determinism self-tests + change detection) -----------------------------
    // The combined hash covers all four buffers (the self-test identity);
    // the per-pass hashes let consumers detect a rebuilt terrain or water
    // patch independently (a chunk key can outlive one rebuild).
    std::uint64_t hash = 0xCBF29CE484222325ull;
    hash = fnv1a64(mesh.terrainVertices.data(), mesh.terrainVertices.size() * sizeof(PlanetVertex), hash);
    hash = fnv1a64(mesh.terrainIndices.data(), mesh.terrainIndices.size() * sizeof(std::uint32_t), hash);
    hash = fnv1a64(mesh.waterVertices.data(), mesh.waterVertices.size() * sizeof(PlanetVertex), hash);
    hash = fnv1a64(mesh.waterIndices.data(), mesh.waterIndices.size() * sizeof(std::uint32_t), hash);
    mesh.hash = hash;

    std::uint64_t terrainHash = 0xCBF29CE484222325ull;
    terrainHash = fnv1a64(mesh.terrainVertices.data(), mesh.terrainVertices.size() * sizeof(PlanetVertex), terrainHash);
    mesh.terrainHash = fnv1a64(mesh.terrainIndices.data(), mesh.terrainIndices.size() * sizeof(std::uint32_t), terrainHash);

    std::uint64_t waterHash = 0xCBF29CE484222325ull;
    waterHash = fnv1a64(mesh.waterVertices.data(), mesh.waterVertices.size() * sizeof(PlanetVertex), waterHash);
    mesh.waterHash = fnv1a64(mesh.waterIndices.data(), mesh.waterIndices.size() * sizeof(std::uint32_t), waterHash);
    return mesh;
}

ChunkMesh buildChunkMesh(const PlanetGenerationParams &params, const DensityField &field, const ChunkCoord &coord)
{
    // Unconstrained build: no boundary segment conforms to a coarser lattice.
    return buildChunkMesh(params, field, coord, ConformMask{});
}

} // namespace planet
} // namespace engine
