// Octree LOD selection (see octree.hpp).

#include <cstddef>
#include <cstdint>
#include <engine/planet/density.hpp>
#include <engine/planet/meshing.hpp>
#include <engine/planet/octree.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>

#include <engine/planet/fbm.hpp>

#include "lattice_math.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace engine
{
namespace planet
{
namespace
{

// Minimum/maxmum distance from the planet center to an axis-aligned cube
// (planet-local, cube given by center + half size).
double cubeMinDistanceToOrigin(const Vec3 &center, double halfSize)
{
    const double ax = std::abs(center.x) - halfSize;
    const double ay = std::abs(center.y) - halfSize;
    const double az = std::abs(center.z) - halfSize;
    const double dx = std::max(0.0, ax);
    const double dy = std::max(0.0, ay);
    const double dz = std::max(0.0, az);
    if (dx == 0.0 && dy == 0.0 && dz == 0.0)
    {
        return 0.0; // origin inside the cube
    }
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double cubeMaxDistanceToOrigin(const Vec3 &center, double halfSize)
{
    return length(center) + halfSize * std::sqrt(3.0);
}

// Cube corner offsets shared by the solid/air probe and the horizon test.
constexpr double kCornerDelta[8][3] = {
    {-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, 0.5, -0.5}, {-0.5, 0.5, -0.5},
    {-0.5, -0.5, 0.5}, {0.5, -0.5, 0.5}, {0.5, 0.5, 0.5}, {-0.5, 0.5, 0.5},
};

// Edge-midpoint and face-center offsets completing the probe set below (in
// half-size units, like kCornerDelta). The full set is the cube's 27 lattice
// points (corners, edge midpoints, face centers, center).
constexpr double kEdgeMidDelta[12][3] = {
    {0.0, -0.5, -0.5}, {0.0, 0.5, -0.5}, {0.0, -0.5, 0.5}, {0.0, 0.5, 0.5},
    {-0.5, 0.0, -0.5}, {0.5, 0.0, -0.5}, {-0.5, 0.0, 0.5}, {0.5, 0.0, 0.5},
    {-0.5, -0.5, 0.0}, {0.5, -0.5, 0.0}, {-0.5, 0.5, 0.0}, {0.5, 0.5, 0.0},
};

constexpr double kFaceCenterDelta[7][3] = {
    {0.0, 0.0, 0.0},
    {-0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, -0.5, 0.0},
    {0.0, 0.5, 0.0}, {0.0, 0.0, -0.5}, {0.0, 0.0, 0.5},
};

} // namespace

VisibleLeafSet::VisibleLeafSet()
    : keys_(16384)
{
}

void VisibleLeafSet::assign(const std::vector<ChunkCoord> &sortedLeaves)
{
    keys_.clear();
    keys_.reserve(sortedLeaves.size());
    depthCounts_.fill(0);
    for (const ChunkCoord &leaf : sortedLeaves)
    {
        keys_.findOrInsert(makeChunkKey(leaf.depth, leaf.x, leaf.y, leaf.z)) = 1;
        if (leaf.depth < depthCounts_.size())
        {
            ++depthCounts_[leaf.depth];
        }
    }
}

bool VisibleLeafSet::contains(const ChunkCoord &coord) const noexcept
{
    return keys_.contains(makeChunkKey(coord.depth, coord.x, coord.y, coord.z));
}

PlanetOctree::PlanetOctree(const PlanetGenerationParams &params, const DensityField &field)
    : params_(params)
    , field_(field)
    , nodes_(32768)
{
    // Per-depth probe margins. The corner probe prunes a node as fully
    // solid/air when every probe point sits beyond `margin` on the same side
    // of the isolevel; for that verdict to be sound, the margin must cover
    // the WORST-CASE rise of the density between a probe point and any
    // interior point. The 20-point probe set (corners + edge midpoints)
    // covers the node with radius half the node size, and the density moves
    // at most (1 + relief gradient) per meter - the +1 being the radial -|p|
    // term of the field - so the sound margin is the band-limited relief
    // gradient bound plus one radial, times half the node size, plus the
    // bounded 3D terms (overhang, strata, caves) each weighted by its fade at
    // the node's own probe cutoff. Margins are precomputed per depth: the
    // cutoff follows the node's lattice, so both the gradient bound and the
    // node size are functions of depth alone.
    for (std::uint32_t depth = 0; depth < probeMargins_.size(); ++depth)
    {
        const double nodeSize = chunkSizeMeters(params, depth);
        const double coveringRadius = 0.5 * nodeSize;
        const double probeCutoff = chunkStepMeters(params, depth) * params.lodBandLimitFactor;
        probeMargins_[depth] = (1.0 + bandLimitedReliefGradient(params, probeCutoff)) * coveringRadius
            + bandLimitedBoundedTerms(params, probeCutoff);
        // The node bounding sphere's radius is a pure function of depth.
        boundRadii_[depth] = coveringRadius * std::sqrt(3.0);
    }

    // Occluder for the horizon test: everything below the inner shell radius
    // is guaranteed solid rock, so a sight line passing through that ball is
    // guaranteed blocked by real terrain. The sea sphere (see horizonHidden)
    // adds the waterline occluder for eyes above the water.
    innerOccluderRadius2_ = params.derivedMinShellRadius * params.derivedMinShellRadius;
    seaOccluderRadius2_ = params.derivedSeaRadiusMeters * params.derivedSeaRadiusMeters;
}

bool PlanetOctree::cubeIntersectsShell(double minCubeRadius, double maxCubeRadius) const
{
    return minCubeRadius <= params_.derivedMaxShellRadius && maxCubeRadius >= params_.derivedMinShellRadius;
}

// Horizon tests against occluder spheres (radius r, centered on the planet
// center): a point P is occluded when the segment eye -> P intersects the
// ball before reaching P. The occluded set is a solid circular cone with apex
// at the eye, which is convex, so a node's cube lies fully inside it exactly
// when all 8 cube corners do.
//
// Two occluders, both sound:
//   * the guaranteed-solid inner ball (radius derivedMinShellRadius): a
//     sight line through it passes through rock that provably exists;
//   * the SEA SPHERE (radius derivedSeaRadius) when the eye is above the
//     water: every sight line that crosses the sea sphere is blocked by what
//     the renderer draws there - the water sheet where the sea exists, the
//     land that pokes through it elsewhere - so the whole underwater half of
//     the shell (abyssal floors and submerged slopes, invisible through the
//     opaque sea from above) culls at the waterline instead of at the deep
//     floor, where the old single-occluder horizon reached 3-4x farther.
//     Eyes at or below the sea surface (wading, swimming, underwater
//     cameras) disable the sea-sphere test: sight lines between the surface
//     and the floor do not cross it, and the deep ball stays the only
//     occluder.
//
// (An earlier test used the quadric H(P) = dot(P, P - eye), whose zero set
// is the diameter ball of the eye vector - an occluder much larger than the
// planet itself. Around the limb it culled chunks whose terrain was still
// visible and shaved the horizon into straight chords.)
//
// An eye INSIDE an occluder ball (buried cameras, probes below the inner
// shell) must disable that ball's test: every sight line then intersects it
// trivially and the whole planet would cull away.
namespace
{

// Corner-sight-line occlusion against one centered ball (see the
// horizonHidden comment): true when every corner's segment to the eye
// intersects the ball. eyeRadius2 = squared eye distance from the planet
// center; occluderRadius2 the ball's.
bool cornersOccludedByBall(const Vec3 &nodeCenter, double nodeSize, const Vec3 &eyeLocal, double eyeRadius2,
    double occluderRadius2)
{
    for (std::uint32_t corner = 0; corner < 8; ++corner)
    {
        const Vec3 point{nodeCenter.x + kCornerDelta[corner][0] * nodeSize,
            nodeCenter.y + kCornerDelta[corner][1] * nodeSize,
            nodeCenter.z + kCornerDelta[corner][2] * nodeSize};
        const Vec3   direction = point - eyeLocal;
        const double c = dot(direction, direction);
        if (c <= 1.0e-12)
        {
            return false; // the eye is inside this node: keep it
        }
        const double b = 2.0 * dot(eyeLocal, direction);
        double t = -b / (2.0 * c);
        t = std::clamp(t, 0.0, 1.0);
        const double minDistance2 = eyeRadius2 + b * t + c * t * t;
        if (minDistance2 > occluderRadius2)
        {
            return false; // this corner has an unobstructed sight line
        }
    }
    return true;
}

} // namespace

bool PlanetOctree::horizonHidden(const Vec3 &nodeCenter, double nodeSize, const Vec3 &eyeLocal) const
{
    const double a = dot(eyeLocal, eyeLocal);
    if (innerOccluderRadius2_ > 0.0 && a > innerOccluderRadius2_
        && cornersOccludedByBall(nodeCenter, nodeSize, eyeLocal, a, innerOccluderRadius2_))
    {
        return true;
    }
    // The sea sphere occludes only for an eye above the water (see the
    // function comment); waterEnabled off means there is no sea surface to
    // hide behind.
    if (params_.waterEnabled && seaOccluderRadius2_ > 0.0
        && a > seaOccluderRadius2_ + 2.0 * seaOccluderEyeMarginMeters * std::sqrt(seaOccluderRadius2_)
        && cornersOccludedByBall(nodeCenter, nodeSize, eyeLocal, a, seaOccluderRadius2_))
    {
        return true;
    }
    return false;
}

PlanetOctree::NodeRecord *PlanetOctree::findRecord(ChunkKey key)
{
    return nodes_.find(key);
}

const PlanetOctree::NodeRecord *PlanetOctree::findRecord(ChunkKey key) const
{
    return nodes_.find(key);
}

PlanetOctree::NodeRecord &PlanetOctree::recordFor(ChunkKey key)
{
    return nodes_.findOrInsert(key);
}

void PlanetOctree::setState(const ChunkCoord &coord, ChunkState state)
{
    recordFor(makeChunkKey(coord.depth, coord.x, coord.y, coord.z)).state = state;
}

bool PlanetOctree::rendersMesh(ChunkState state)
{
    switch (state)
    {
        case ChunkState::Wanted:
        case ChunkState::Requested:
        case ChunkState::Building:
        case ChunkState::Ready:
            return true;
        default:
            return false;
    }
}

// Pruning + refinement tests of a single node, shared by the main descent and
// by the balancing pass (which classifies children of force-split nodes).
ChunkState PlanetOctree::classifyNode(const ChunkCoord &coord, const Vec3 &eyeLocal, ChildList *outRefineChildren,
    NodeRecord &record) const
{
    const double nodeSize    = chunkSizeMeters(params_, coord.depth);
    const double halfSize    = 0.5 * nodeSize;
    // One lattice expression for every consumer of the node center (this
    // probe pass, the horizon test, the build requests): chunkCenterMeters
    // is the same expression the mesher pins the vertex frame with, so
    // probe positions and mesh placement cannot drift apart at any depth.
    const Vec3   nodeCenter  = chunkCenterMeters(params_, coord);
    const double boundRadius = boundRadii_[std::min<std::size_t>(coord.depth, boundRadii_.size() - 1)];

    // 1. Shell-band pruning: cubes fully inside the core or fully outside
    //    the terrain band cannot contain surface.
    const double minCubeRadius = cubeMinDistanceToOrigin(nodeCenter, halfSize);
    const double maxCubeRadius = cubeMaxDistanceToOrigin(nodeCenter, halfSize);
    if (!cubeIntersectsShell(minCubeRadius, maxCubeRadius))
    {
        return ChunkState::Empty;
    }

    // 1b. Solid/air probes: when every corner of the cube is on the same
    //     side of the isolevel, no surface passes through it and none can
    //     pass through any descendant - prune the whole subtree. The probe
    //     evaluates the BAND-LIMITED field at this node's own cutoff - the
    //     exact field the node's mesh would be built from - so the corner
    //     signs decide with only a one-voxel safety margin. (Probing the raw
    //     field needed a Lipschitz margin covering every faded octave; with
    //     large-radius amplitude stacks that margin grew to hundreds of
    //     meters, kept thousands of genuinely-empty chunks in the visible
    //     set, and starved the build queue with full-price no-op builds.)
    //     Nodes straddling the sea surface are exempt: the water field
    //     crosses zero there even when every density corner is "air", and
    //     the ocean surface must still render (deep-water holes otherwise
    //     appear at pruning boundaries).
    if (coord.depth >= 2)
    {
        bool straddlesSea = false;
        if (params_.waterEnabled)
        {
            const double probeStep    = chunkStepMeters(params_, coord.depth);
            const double centerRadius = length(nodeCenter);
            straddlesSea = centerRadius - boundRadius <= params_.derivedSeaRadiusMeters + probeStep
                        && centerRadius + boundRadius >= params_.derivedSeaRadiusMeters - probeStep;
        }
        if (!straddlesSea)
        {
            if (record.probe == ProbeVerdict::Unknown)
            {
                const double probeCutoff = chunkStepMeters(params_, coord.depth) * params_.lodBandLimitFactor;
                const double probeMargin = probeMargins_[std::min<std::size_t>(coord.depth, probeMargins_.size() - 1)];
                double minDensity = 0.0;
                double maxDensity = 0.0;
                bool   undecided  = false;
                // The probe samples the node's 27 lattice points (8 corners,
                // 12 edge midpoints, 6 face centers, the center): that set
                // covers the node with radius half the node size (the worst
                // interior points are face centers, nodeSize/2 from their
                // nearest edge midpoint), versus half the DIAGONAL for
                // corners alone - a 0.577x tighter sound margin, so the
                // probe prunes far more of the genuinely empty/solid nodes
                // around the terrain shell.
                for (std::uint32_t sample = 0; sample < 27u && !undecided; ++sample)
                {
                    const double *offset = sample < 8u ? kCornerDelta[sample]
                        : sample < 20u ? kEdgeMidDelta[sample - 8u]
                        : kFaceCenterDelta[sample - 20u];
                    const Vec3 probePoint{nodeCenter.x + offset[0] * nodeSize,
                        nodeCenter.y + offset[1] * nodeSize,
                        nodeCenter.z + offset[2] * nodeSize};
                    const double density = field_.densityAt(probePoint, probeCutoff);
                    if (sample == 0 || density < minDensity)
                    {
                        minDensity = density;
                    }
                    if (sample == 0 || density > maxDensity)
                    {
                        maxDensity = density;
                    }
                    if (minDensity <= probeMargin && maxDensity >= -probeMargin)
                    {
                        undecided = true; // neither fully solid nor fully air
                    }
                }
                record.probe = undecided ? ProbeVerdict::Undecided : ProbeVerdict::Empty;
            }
            if (record.probe == ProbeVerdict::Empty)
            {
                return ChunkState::Empty;
            }
        }
    }

    // 2. Horizon culling (skippable; the demo toggles it from config).
    if (params_.horizonCulling && horizonHidden(nodeCenter, nodeSize, eyeLocal))
    {
        return ChunkState::Hidden;
    }

    // 3. Angular (screen-space) refinement: refine while the node's VOXEL
    //    subtends more than lodAngularVoxelRadians at the eye. Detail follows
    //    apparent size - the whole visible disk of a globe view refines to
    //    the depth whose voxels are a few pixels, surface views keep the
    //    finest depth in the near field, and the rule is independent of
    //    planet radius and viewport. The distance is measured to the node's
    //    bounding-sphere point nearest the eye, so a node stays coarse only
    //    when its ENTIRE volume is far enough away (a node the eye sits
    //    beside always refines).
    const double voxelSize   = chunkStepMeters(params_, coord.depth);
    const double eyeDistance = std::max(voxelSize, length(nodeCenter - eyeLocal) - boundRadius);
    const bool   wantRefine  = coord.depth < params_.derivedMaxDepth
        && voxelSize > eyeDistance * params_.lodAngularVoxelRadians;
    if (wantRefine)
    {
        if (outRefineChildren != nullptr)
        {
            const std::uint32_t x2 = coord.x * 2u;
            const std::uint32_t y2 = coord.y * 2u;
            const std::uint32_t z2 = coord.z * 2u;
            const std::uint32_t nextDepth = coord.depth + 1u;
            for (std::uint32_t child = 0; child < 8; ++child)
            {
                outRefineChildren->items[child]
                    = ChunkCoord{nextDepth, x2 + (child & 1u), y2 + ((child >> 1) & 1u), z2 + ((child >> 2) & 1u)};
            }
            outRefineChildren->count = 8;
        }
        return ChunkState::Split;
    }
    return ChunkState::Wanted;
}

// Splits a rendered leaf outside the normal descent (2:1 balancing). The
// split itself is UNCONDITIONAL - the balance fixpoint has established that
// the adjacent fine material needs this node subdivided regardless of the
// distance-refinement rule. Descendants, however, are classified with the
// ordinary rules (pruning, horizon, refinement), so the subtree shape stays
// consistent with a rule-driven descent.
void PlanetOctree::forceSplit(const ChunkCoord &coord, const Vec3 &eyeLocal, std::vector<ChunkCoord> &outNewLeaves)
{
    const std::uint32_t x2 = coord.x * 2u;
    const std::uint32_t y2 = coord.y * 2u;
    const std::uint32_t z2 = coord.z * 2u;
    const std::uint32_t nextDepth = coord.depth + 1u;

    {
        NodeRecord &record       = recordFor(makeChunkKey(coord.depth, coord.x, coord.y, coord.z));
        record.state             = ChunkState::Split;
        record.lastSelectedUpdate = updateIndex_;
    }
    for (std::uint32_t child = 0; child < 8; ++child)
    {
        const ChunkCoord childCoord{nextDepth, x2 + (child & 1u), y2 + ((child >> 1) & 1u), z2 + ((child >> 2) & 1u)};
        classifyAndRecord(childCoord, eyeLocal, outNewLeaves);
    }
}

// Rule-driven recursive classification used by forceSplit: classifies the
// node, records the state, descends into rule-refined children, and collects
// every newly rendered leaf.
void PlanetOctree::classifyAndRecord(const ChunkCoord &coord, const Vec3 &eyeLocal, std::vector<ChunkCoord> &outNewLeaves)
{
    ChildList  children;
    ChunkState state = ChunkState::Empty;
    {
        // The reference dies before recursing: child insertions may grow the table.
        NodeRecord &record        = recordFor(makeChunkKey(coord.depth, coord.x, coord.y, coord.z));
        state                     = classifyNode(coord, eyeLocal, &children, record);
        record.state              = state;
        record.lastSelectedUpdate = updateIndex_;
    }
    if (state == ChunkState::Split)
    {
        for (std::uint32_t child = 0; child < children.count; ++child)
        {
            classifyAndRecord(children.items[child], eyeLocal, outNewLeaves);
        }
    }
    else if (state == ChunkState::Wanted)
    {
        outNewLeaves.push_back(coord);
    }
}

int PlanetOctree::neighborMaterialDepth(const ChunkCoord &coord, int offsetX, int offsetY, int offsetZ) const
{
    // The neighbor CELL at coord.depth; outside the root means no material.
    const std::int64_t cellCount = static_cast<std::int64_t>(1) << coord.depth;
    const std::int64_t nx = static_cast<std::int64_t>(coord.x) + offsetX;
    const std::int64_t ny = static_cast<std::int64_t>(coord.y) + offsetY;
    const std::int64_t nz = static_cast<std::int64_t>(coord.z) + offsetZ;
    if (nx < 0 || ny < 0 || nz < 0 || nx >= cellCount || ny >= cellCount || nz >= cellCount)
    {
        return -1;
    }

    // Walk the ancestor chain of the neighbor cell from fine to coarse: the
    // first record REFRESHED THIS UPDATE decides. Records left over from
    // earlier updates (different eye positions) describe a tree that no
    // longer exists - trusting them silently skips balancing violations and
    // leaves 2+ depth steps unbalanced, which reads as crack dashes at the
    // rings. A record from this update exists for every node the descent
    // visited (Split and Hidden included), so anything older is stale.
    for (std::uint32_t depth = coord.depth; depth > 0; --depth)
    {
        const std::uint32_t shift = coord.depth - depth;
        const ChunkCoord chain{depth, static_cast<std::uint32_t>(nx >> shift),
            static_cast<std::uint32_t>(ny >> shift), static_cast<std::uint32_t>(nz >> shift)};
        const NodeRecord *record = findRecord(makeChunkKey(chain.depth, chain.x, chain.y, chain.z));
        if (record == nullptr || record->lastSelectedUpdate != updateIndex_)
        {
            continue; // unvisited or stale: covered by a shallower node
        }
        if (record->state == ChunkState::Split)
        {
            return coord.depth; // finer material: balanced from this side
        }
        if (rendersMesh(record->state))
        {
            return static_cast<int>(depth); // the shallowest covering leaf
        }
        return -1; // Empty or Hidden: nothing rendered in that direction
    }
    // The root itself: only when coord.depth chains up to depth 0 with the
    // root being a rendered leaf.
    const NodeRecord *root = findRecord(makeChunkKey(0u, 0u, 0u, 0u));
    if (root != nullptr && rendersMesh(root->state))
    {
        return 0;
    }
    return -1;
}

namespace
{

// Shared conform decision: `visibleContains(probe)` answers membership in the visible leaf list.
//
// Every coarse cell the per-face probes can ask about lies in the 3x3x3 block of depth-(depth-1) cells around the
// chunk's own coarse cell: the far cell of each face is the chunk's coarse cell shifted one along the face axis, and
// the tangential offsets span +-1; the symmetry probe re-reads the chunk's own coarse cell at the same tangential
// offsets. The block is therefore collected ONCE per chunk (27 membership probes) and every face decision reads the
// collected table instead of re-probing the visible set 54+ times.
template <typename Contains>
void computeConformMaskImpl(const PlanetGenerationParams &params, const ChunkCoord &coord, ConformMask &outMask,
    const Contains &visibleContains)
{
    outMask = ConformMask{};
    if (coord.depth == 0)
    {
        return; // nothing coarser than the root
    }

    const std::int64_t n          = static_cast<std::int64_t>(params.leafGridN);
    const std::int64_t coarseSpan = 2 * n; // a depth-(d-1) cell spans 2n fine grid steps


    const std::int64_t chunkCoord[3] = {static_cast<std::int64_t>(coord.x), static_cast<std::int64_t>(coord.y),
        static_cast<std::int64_t>(coord.z)};
    const std::int64_t coarseCellCount = static_cast<std::int64_t>(1) << (coord.depth - 1u); // depth-(d-1) cells per axis

    // The chunk's own coarse cell and the 27-cell block around it (index (du+1) + (dv+1)*3 + (dw+1)*9, du along the
    // chunk's x axis). Cells outside the depth lattice (the root rim) collect as absent.
    std::int64_t blockBase[3] = {0, 0, 0};
    bool         blockAny     = false;
    bool         blockLeaf[27];
    {
        const std::int64_t base[3] = {chunkCoord[0] / 2, chunkCoord[1] / 2, chunkCoord[2] / 2};
        for (std::uint32_t axis = 0; axis < 3; ++axis)
        {
            blockBase[axis] = base[axis];
        }
        for (int dw = -1; dw <= 1; ++dw)
        {
            for (int dv = -1; dv <= 1; ++dv)
            {
                for (int du = -1; du <= 1; ++du)
                {
                    const std::uint32_t slot
                        = static_cast<std::uint32_t>((du + 1) + (dv + 1) * 3 + (dw + 1) * 9);
                    blockLeaf[slot] = false;
                    const std::int64_t cell[3] = {blockBase[0] + du, blockBase[1] + dv, blockBase[2] + dw};
                    if (cell[0] < 0 || cell[1] < 0 || cell[2] < 0 || cell[0] >= coarseCellCount || cell[1] >= coarseCellCount
                        || cell[2] >= coarseCellCount)
                    {
                        continue;
                    }
                    const ChunkCoord probe{coord.depth - 1u, static_cast<std::uint32_t>(cell[0]),
                        static_cast<std::uint32_t>(cell[1]), static_cast<std::uint32_t>(cell[2])};
                    if (visibleContains(probe))
                    {
                        blockLeaf[slot] = true;
                        blockAny        = true;
                    }
                }
            }
        }
    }
    if (!blockAny)
    {
        return; // no rendered leaf one level coarser anywhere around this chunk: nothing can conform
    }

    const auto blockLeafAt = [&blockLeaf](int du, int dv, int dw)
    {
        if (du < -1 || du > 1 || dv < -1 || dv > 1 || dw < -1 || dw > 1)
        {
            return false;
        }
        return blockLeaf[static_cast<std::uint32_t>((du + 1) + (dv + 1) * 3 + (dw + 1) * 9)];
    };

    for (std::uint32_t axis = 0; axis < 3; ++axis)
    {
        // Far-side cell index along the face axis. The chunk spans fine grid
        // steps [C*n, (C+1)*n); beyond the min face the covering depth-(d-1)
        // cell is floor((C*n - 1) / 2n), beyond the max face ((C+1)*n) / 2n.
        const std::int64_t minFaceCell = detail::floorDiv(chunkCoord[axis] * n - 1, coarseSpan);
        const std::int64_t maxFaceCell = ((chunkCoord[axis] + 1) * n) / coarseSpan;

        for (std::uint32_t side = 0; side < 2; ++side)
        {
            const std::uint32_t face  = axis * 2u + side;
            const std::int64_t  perp  = side == 0 ? minFaceCell : maxFaceCell;
            // Tangential axes: x faces span (y, z), y faces span (x, z),
            // z faces span (x, y) - the same mapping the mesher reads.
            const std::uint32_t axisU = axis == 0 ? 1u : 0u;
            const std::uint32_t axisV = axis == 2 ? 1u : 2u;

            // toCoarser: the depth-(d-1) cells around the face segment; the
            // chunk's own coarse-cell block is chunkCoord[t] / 2 (floor).
            const std::int64_t  baseU = chunkCoord[axisU] / 2;
            const std::int64_t  baseV = chunkCoord[axisV] / 2;
            for (int offsetU = -1; offsetU <= 1; ++offsetU)
            {
                for (int offsetV = -1; offsetV <= 1; ++offsetV)
                {
                    bool conform = false;
                    if (perp >= 0 && perp < coarseCellCount)
                    {
                        // The probe cell: the PERPENDICULAR index goes into
                        // the face's own axis, the tangential indices into
                        // the other two. Building it as {perp, u, v} would
                        // describe a wrong cell for every non-x face and
                        // silently corrupt the whole conform mask. Every
                        // probe lands inside the collected 27-cell block:
                        // the perpendicular offset against the chunk's own
                        // coarse cell is always -1 or 0 (min face) or 0 or
                        // +1 (max face), and the tangential offsets are the
                        // loop's +-1 ring.
                        std::int64_t cell[3] = {0, 0, 0};
                        cell[axis]  = perp;
                        cell[axisU] = baseU + offsetU;
                        cell[axisV] = baseV + offsetV;
                        const int blockDu = static_cast<int>(cell[0] - blockBase[0]);
                        const int blockDv = static_cast<int>(cell[1] - blockBase[1]);
                        const int blockDw = static_cast<int>(cell[2] - blockBase[2]);
                        conform = blockLeafAt(blockDu, blockDv, blockDw);

                        // SYMMETRY of the decision: the chunk on the OTHER
                        // side of this face plane sees the mirrored cell
                        // column (the depth-(d-1) cell adjacent to the
                        // plane on OUR side, at the same diagonal
                        // tangential position). Its material can differ
                        // from ours - a coarse leaf below the plane while
                        // fine material splits above it - and then one
                        // side would conform while the other stays raw,
                        // cracking the shared plane at the corners. The
                        // conform decision therefore ORs both adjacent
                        // cells; every chunk touching the same plane point
                        // derives the identical answer from the identical
                        // pair of cells, and the conforming value (the
                        // coarse lattice interpolant) is the same
                        // function either way.
                        std::int64_t nearCell[3] = {0, 0, 0};
                        nearCell[axis]  = chunkCoord[axis] / 2; // the depth-(d-1) cell containing this chunk
                        nearCell[axisU] = baseU + offsetU;
                        nearCell[axisV] = baseV + offsetV;
                        if (nearCell[axis] != cell[axis])
                        {
                            const int nearDu = static_cast<int>(nearCell[0] - blockBase[0]);
                            const int nearDv = static_cast<int>(nearCell[1] - blockBase[1]);
                            const int nearDw = static_cast<int>(nearCell[2] - blockBase[2]);
                            conform = conform || blockLeafAt(nearDu, nearDv, nearDw);
                        }
                    }
                    setConformBit(outMask, face, offsetU, offsetV, conform);
                }
            }
        }
    }
}

} // namespace

void PlanetOctree::computeConformMask(const ChunkCoord &coord, const VisibleLeafSet &visibleLeaves,
    ConformMask &outMask) const
{
    if (coord.depth == 0 || !visibleLeaves.hasDepth(coord.depth - 1u))
    {
        outMask = ConformMask{}; // no rendered leaf one level coarser exists anywhere: nothing can conform
        return;
    }
    computeConformMaskImpl(params_, coord, outMask,
        [&visibleLeaves](const ChunkCoord &probe) { return visibleLeaves.contains(probe); });
}

void PlanetOctree::selectVisible(const Vec3 &eyeLocal, std::vector<ChunkCoord> &outVisible)
{
    ++updateIndex_;
    outVisible.clear();

    std::vector<ChunkCoord> &stack = descentStack_;
    stack.clear();
    stack.push_back(ChunkCoord{0u, 0u, 0u, 0u});

    std::vector<ChunkCoord> &leaves = leavesScratch_;
    leaves.clear();
    while (!stack.empty())
    {
        const ChunkCoord coord = stack.back();
        stack.pop_back();

        ChildList        children;
        NodeRecord      &record   = recordFor(makeChunkKey(coord.depth, coord.x, coord.y, coord.z));
        const ChunkState state    = classifyNode(coord, eyeLocal, &children, record);
        const ChunkState previous = record.state;
        const bool       hadLifecycle = previous == ChunkState::Requested || previous == ChunkState::Building
                                      || previous == ChunkState::Ready;
        record.state              = state;
        record.lastSelectedUpdate = updateIndex_;

        if (state == ChunkState::Split)
        {
            for (std::uint32_t child = 0; child < children.count; ++child)
            {
                stack.push_back(children.items[child]);
            }
        }
        else if (state == ChunkState::Wanted)
        {
            // Preserve lifecycle states from earlier updates (Requested /
            // Building / Ready); the facade filters already-built chunks via
            // its own caches, but keeping the state honest costs nothing.
            if (hadLifecycle)
            {
                record.state = previous;
            }
            leaves.push_back(coord);
        }
    }
    // ---- phase 2: 2:1 balancing -------------------------------------------------
    //
    // Without balancing, the distance-refinement rule can place rendered
    // leaves whose depths differ by 2+ across a shared face/edge/corner. The
    // fine mesh then T-junctions against the coarse one over several voxel
    // steps, which reads as seams at the LOD ring. The fixpoint forces splits
    // along the coarse chain until every pair of adjacent rendered leaves
    // differs by at most one depth level.
    //
    // A force-split only ever makes material FINER: previously balanced
    // leaves keep their balance (their neighborhoods only improved), so the
    // only violations a later pass can find involve the leaves the previous
    // pass CREATED. Each pass therefore walks only the range appended since
    // the previous one - pass 0 covers the whole descent, later passes the
    // force-split children.
    constexpr std::uint32_t kMaxBalancePasses = 12;
    std::size_t             checkBegin = 0;
    for (std::uint32_t pass = 0; pass < kMaxBalancePasses; ++pass)
    {
        bool changed = false;
        const std::size_t passLeafCount = leaves.size();
        for (std::size_t index = checkBegin; index < passLeafCount; ++index)
        {
            const ChunkCoord leaf = leaves[index];
            if (leaf.depth == 0)
            {
                continue; // nothing coarser than the root
            }
            const NodeRecord *leafRecord = findRecord(makeChunkKey(leaf.depth, leaf.x, leaf.y, leaf.z));
            if (leafRecord == nullptr || !rendersMesh(leafRecord->state))
            {
                continue; // force-split earlier in this pass
            }

            // All 26 face/edge/corner neighbor directions. Multiple
            // directions of one leaf may violate; each fix updates the node
            // map live, so later walks in the same pass see the repaired
            // tree.
            for (int dx = -1; dx <= 1; ++dx)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    for (int dz = -1; dz <= 1; ++dz)
                    {
                        if (dx == 0 && dy == 0 && dz == 0)
                        {
                            continue;
                        }
                        const int materialDepth = neighborMaterialDepth(leaf, dx, dy, dz);
                        if (materialDepth < 0 || static_cast<std::uint32_t>(materialDepth) + 1u >= leaf.depth)
                        {
                            continue; // no material, finer, or exactly one level coarser
                        }

                        // Violation: the adjacent material at materialDepth is
                        // leaf-rendered and 2+ levels coarser. Force-split the
                        // chain between materialDepth and leaf.depth - 1.
                        const std::int64_t nx = static_cast<std::int64_t>(leaf.x) + dx;
                        const std::int64_t ny = static_cast<std::int64_t>(leaf.y) + dy;
                        const std::int64_t nz = static_cast<std::int64_t>(leaf.z) + dz;
                        for (std::uint32_t depth = static_cast<std::uint32_t>(materialDepth); depth + 1u < leaf.depth; ++depth)
                        {
                            const std::uint32_t shift = leaf.depth - depth;
                            const ChunkCoord chain{depth, static_cast<std::uint32_t>(nx >> shift),
                                static_cast<std::uint32_t>(ny >> shift), static_cast<std::uint32_t>(nz >> shift)};
                            const NodeRecord *chainRecord = findRecord(makeChunkKey(chain.depth, chain.x, chain.y, chain.z));
                            if (chainRecord == nullptr || !rendersMesh(chainRecord->state))
                            {
                                break; // Split (finer material appeared), Empty, or Hidden
                            }
                            forceSplit(chain, eyeLocal, leaves);
                            changed = true;
                        }
                    }
                }
            }
        }
        if (!changed)
        {
            break;
        }
        // Next pass: only the leaves this pass appended (see the phase note).
        checkBegin = passLeafCount;
    }
    // ---- phase 3: emit ----------------------------------------------------------
    //
    // Only nodes that still render (balancing may have split some) in
    // deterministic (depth, x, y, z) order.
    for (const ChunkCoord &leaf : leaves)
    {
        const NodeRecord *record = findRecord(makeChunkKey(leaf.depth, leaf.x, leaf.y, leaf.z));
        if (record != nullptr && rendersMesh(record->state))
        {
            outVisible.push_back(leaf);
        }
    }
    std::sort(outVisible.begin(), outVisible.end(), [](const ChunkCoord &a, const ChunkCoord &b)
    {
        if (a.depth != b.depth) return a.depth < b.depth;
        if (a.x != b.x) return a.x < b.x;
        if (a.y != b.y) return a.y < b.y;
        return a.z < b.z;
    });

    // ---- phase 4: stale record pruning ----------------------------------------------
    //
    // Nodes the selection has not visited for a while describe obsolete tree
    // shapes; dropping them keeps the node map bounded and guarantees the
    // balancing walks only ever see fresh states (neighborMaterialDepth
    // already skips stale records, this just reclaims the memory).
    // Pruning runs on the configured cadence and keeps records for the
    // configured lifetime: a longer lifetime holds probe verdicts and cached
    // geometry for views that revisit ground (walking cameras, orbits), so
    // the first-visit field probes are paid once per area instead of once
    // per return.
    if (updateIndex_ % params_.nodeRecordPruneIntervalUpdates == 0)
    {
        const std::uint64_t staleThreshold = updateIndex_ > params_.nodeRecordLifetimeUpdates
            ? updateIndex_ - params_.nodeRecordLifetimeUpdates : 1;
        nodes_.eraseIf([staleThreshold](ChunkKey, const NodeRecord &record)
            { return record.lastSelectedUpdate < staleThreshold; });
    }
}

} // namespace planet
} // namespace engine
