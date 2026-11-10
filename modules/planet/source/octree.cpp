// Octree LOD selection (see octree.hpp).

#include <engine/planet/octree.hpp>

#include <algorithm>
#include <cmath>

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

} // namespace

PlanetOctree::PlanetOctree(const PlanetGenerationParams &params, const DensityField &field)
    : params_(params)
    , field_(field)
{
    // Density variation bound of the 3D terms across any node (they are
    // bounded fields, not gradient-limited): overhang and strata contribute
    // at most twice their amplitude, caves at most their carve depth. The
    // radially-shaped relief adds a gradient term (|amp / wavelength| sums,
    // times 3 for safety) times the node diagonal at the probe depth.
    const double boundedTerms = 2.0 * (params.overhangAmplitudeMeters + params.strataAmplitudeMeters) + params.caveCarveMeters;
    const double reliefGradientBound = 3.0
        * (params.continentAmplitudeMeters / params.continentWavelengthMeters
            + params.mountainAmplitudeMeters / params.mountainWavelengthMeters
            + params.mountainAmplitudeMeters / params.derivedMountainMassWavelengthMeters
            + params.mountainAmplitudeMeters / params.rangeClusterWavelengthMeters
            + params.regionalAmplitudeMeters / params.regionalWavelengthMeters
            + params.hillAmplitudeMeters / params.hillWavelengthMeters
            + params.detailAmplitudeMeters / params.detailWavelengthMeters);
    solidMarginMeters_ = boundedTerms + reliefGradientBound * std::sqrt(3.0) * chunkSizeMeters(params, params.derivedMaxDepth);

    // Occluder for the horizon test: everything below the inner shell radius
    // is guaranteed solid rock, so a sight line passing through that ball is
    // guaranteed blocked by real terrain.
    innerOccluderRadius2_ = params.derivedMinShellRadius * params.derivedMinShellRadius;
}

bool PlanetOctree::cubeIntersectsShell(double minCubeRadius, double maxCubeRadius) const
{
    return minCubeRadius <= params_.derivedMaxShellRadius && maxCubeRadius >= params_.derivedMinShellRadius;
}

// Horizon test against the guaranteed-solid inner sphere (radius
// derivedMinShellRadius): a point P is occluded when the segment eye -> P
// intersects that ball before reaching P - the sight line then passes through
// rock that provably exists. The occluded set is a solid circular cone with
// apex at the eye, which is convex, so a node's cube lies fully inside it
// exactly when all 8 cube corners do.
//
// (The previous test used the quadric H(P) = dot(P, P - eye), whose zero set
// is the diameter ball of the eye vector - an occluder much larger than the
// planet itself. Around the limb it culled chunks whose terrain was still
// visible and shaved the horizon into straight chords.)
bool PlanetOctree::horizonHidden(const Vec3 &nodeCenter, double nodeSize, const Vec3 &eyeLocal) const
{
    if (innerOccluderRadius2_ <= 0.0)
    {
        return false;
    }
    const double a = dot(eyeLocal, eyeLocal);
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
        const double minDistance2 = a + b * t + c * t * t;
        if (minDistance2 > innerOccluderRadius2_)
        {
            return false; // this corner has an unobstructed sight line
        }
    }
    return true;
}

PlanetOctree::NodeRecord *PlanetOctree::findRecord(ChunkKey key)
{
    const auto it = std::lower_bound(nodes_.begin(), nodes_.end(), key,
        [](const std::pair<ChunkKey, NodeRecord> &entry, ChunkKey value) { return entry.first < value; });
    if (it != nodes_.end() && it->first == key)
    {
        return &it->second;
    }
    return nullptr;
}

const PlanetOctree::NodeRecord *PlanetOctree::findRecord(ChunkKey key) const
{
    const auto it = std::lower_bound(nodes_.begin(), nodes_.end(), key,
        [](const std::pair<ChunkKey, NodeRecord> &entry, ChunkKey value) { return entry.first < value; });
    if (it != nodes_.end() && it->first == key)
    {
        return &it->second;
    }
    return nullptr;
}

PlanetOctree::NodeRecord &PlanetOctree::recordFor(ChunkKey key)
{
    const auto it = std::lower_bound(nodes_.begin(), nodes_.end(), key,
        [](const std::pair<ChunkKey, NodeRecord> &entry, ChunkKey value) { return entry.first < value; });
    if (it != nodes_.end() && it->first == key)
    {
        return it->second;
    }
    return nodes_.emplace(it, key, NodeRecord{})->second;
}

ChunkState PlanetOctree::stateOf(const ChunkCoord &coord) const
{
    const auto *record = findRecord(makeChunkKey(coord.depth, coord.x, coord.y, coord.z));
    return record != nullptr ? record->state : ChunkState::Wanted;
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
ChunkState PlanetOctree::classifyNode(const ChunkCoord &coord, const Vec3 &eyeLocal,
    std::vector<ChunkCoord> *outRefineChildren) const
{
    const double nodeSize    = chunkSizeMeters(params_, coord.depth);
    const double halfSize    = 0.5 * nodeSize;
    const Vec3   nodeCenter  = Vec3{-params_.derivedRootHalfMeters + nodeSize * (static_cast<double>(coord.x) + 0.5),
        -params_.derivedRootHalfMeters + nodeSize * (static_cast<double>(coord.y) + 0.5),
        -params_.derivedRootHalfMeters + nodeSize * (static_cast<double>(coord.z) + 0.5)};
    const double boundRadius = halfSize * std::sqrt(3.0);

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
    if (coord.depth >= 2 && solidMarginMeters_ > 0.0)
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
            const double probeCutoff = chunkStepMeters(params_, coord.depth) * params_.lodBandLimitFactor;
            const double probeMargin = std::max(solidMarginMeters_, chunkStepMeters(params_, coord.depth));
            double minDensity = 0.0;
            double maxDensity = 0.0;
            bool   undecided  = false;
            for (std::uint32_t corner = 0; corner < 8; ++corner)
            {
                const Vec3 probePoint{nodeCenter.x + kCornerDelta[corner][0] * nodeSize,
                    nodeCenter.y + kCornerDelta[corner][1] * nodeSize,
                    nodeCenter.z + kCornerDelta[corner][2] * nodeSize};
                const double density = field_.densityAt(probePoint, probeCutoff);
                if (corner == 0 || density < minDensity)
                {
                    minDensity = density;
                }
                if (corner == 0 || density > maxDensity)
                {
                    maxDensity = density;
                }
                if (minDensity <= probeMargin && maxDensity >= -probeMargin)
                {
                    undecided = true; // neither fully solid nor fully air
                    break;
                }
            }
            if (!undecided)
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
                outRefineChildren->push_back(ChunkCoord{nextDepth, x2 + (child & 1u), y2 + ((child >> 1) & 1u), z2 + ((child >> 2) & 1u)});
            }
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

    recordFor(makeChunkKey(coord.depth, coord.x, coord.y, coord.z)) = NodeRecord{ChunkState::Split, updateIndex_};
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
    std::vector<ChunkCoord> children;
    const ChunkState state = classifyNode(coord, eyeLocal, &children);
    recordFor(makeChunkKey(coord.depth, coord.x, coord.y, coord.z)) = NodeRecord{state, updateIndex_};
    if (state == ChunkState::Split)
    {
        for (const ChunkCoord &child : children)
        {
            classifyAndRecord(child, eyeLocal, outNewLeaves);
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

void PlanetOctree::computeConformMask(const ChunkCoord &coord, const std::vector<ChunkCoord> &visibleSortedLeaves,
    ConformMask &outMask) const
{
    outMask = ConformMask{};
    if (coord.depth == 0)
    {
        return; // nothing coarser than the root
    }

    // Membership over the sorted visible leaves (depth, x, y, z order, the
    // exact ordering selectVisible emits).
    const auto visibleContains = [&visibleSortedLeaves](const ChunkCoord &probe)
    {
        const auto it = std::lower_bound(visibleSortedLeaves.begin(), visibleSortedLeaves.end(), probe,
            [](const ChunkCoord &a, const ChunkCoord &b)
            {
                if (a.depth != b.depth)
                {
                    return a.depth < b.depth;
                }
                if (a.x != b.x)
                {
                    return a.x < b.x;
                }
                if (a.y != b.y)
                {
                    return a.y < b.y;
                }
                return a.z < b.z;
            });
        return it != visibleSortedLeaves.end() && *it == probe;
    };

    const std::int64_t n          = static_cast<std::int64_t>(params_.leafGridN);
    const std::int64_t coarseSpan = 2 * n; // a depth-(d-1) cell spans 2n fine grid steps

    const auto floorDiv = [](std::int64_t value, std::int64_t divisor)
    {
        std::int64_t quotient = value / divisor;
        if ((value % divisor) != 0 && ((value < 0) != (divisor < 0)))
        {
            --quotient;
        }
        return quotient;
    };

    const std::int64_t chunkCoord[3] = {static_cast<std::int64_t>(coord.x), static_cast<std::int64_t>(coord.y),
        static_cast<std::int64_t>(coord.z)};
    const std::int64_t coarseCellCount = static_cast<std::int64_t>(1) << (coord.depth - 1u); // depth-(d-1) cells per axis

    for (std::uint32_t axis = 0; axis < 3; ++axis)
    {
        // Far-side cell index along the face axis. The chunk spans fine grid
        // steps [C*n, (C+1)*n); beyond the min face the covering depth-(d-1)
        // cell is floor((C*n - 1) / 2n), beyond the max face ((C+1)*n) / 2n.
        const std::int64_t minFaceCell = floorDiv(chunkCoord[axis] * n - 1, coarseSpan);
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
                        const std::int64_t tangentialU = baseU + offsetU;
                        const std::int64_t tangentialV = baseV + offsetV;
                        if (tangentialU >= 0 && tangentialU < coarseCellCount && tangentialV >= 0 && tangentialV < coarseCellCount)
                        {
                            // The probe cell: the PERPENDICULAR index goes into
                            // the face's own axis, the tangential indices into
                            // the other two. Building it as {perp, u, v} would
                            // describe a wrong cell for every non-x face and
                            // silently corrupt the whole conform mask.
                            std::int64_t cell[3] = {0, 0, 0};
                            cell[axis]  = perp;
                            cell[axisU] = tangentialU;
                            cell[axisV] = tangentialV;
                            const ChunkCoord probe{coord.depth - 1u, static_cast<std::uint32_t>(cell[0]),
                                static_cast<std::uint32_t>(cell[1]), static_cast<std::uint32_t>(cell[2])};
                            conform = visibleContains(probe);

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
                            nearCell[axisU] = tangentialU;
                            nearCell[axisV] = tangentialV;
                            if (nearCell[axis] != cell[axis])
                            {
                                const ChunkCoord nearProbe{coord.depth - 1u, static_cast<std::uint32_t>(nearCell[0]),
                                    static_cast<std::uint32_t>(nearCell[1]), static_cast<std::uint32_t>(nearCell[2])};
                                conform = conform || visibleContains(nearProbe);
                            }
                        }
                    }
                    setConformBit(outMask, face, offsetU, offsetV, conform);
                }
            }
        }
    }
}

void PlanetOctree::selectVisible(const Vec3 &eyeLocal, std::vector<ChunkCoord> &outVisible)
{
    ++updateIndex_;
    outVisible.clear();

    // ---- phase 1: descent ------------------------------------------------------
    //
    // Iterative descent with an explicit stack; every visited node is recorded
    // (Split and Hidden included) so the balancing pass can reason about the
    // tree shape afterwards.
    std::vector<ChunkCoord> stack;
    stack.push_back(ChunkCoord{0u, 0u, 0u, 0u});

    std::vector<ChunkCoord> leaves;
    while (!stack.empty())
    {
        const ChunkCoord coord = stack.back();
        stack.pop_back();

        std::vector<ChunkCoord> children;
        const ChunkState state = classifyNode(coord, eyeLocal, &children);
        NodeRecord       &record = recordFor(makeChunkKey(coord.depth, coord.x, coord.y, coord.z));
        const ChunkState  previous = record.state;
        const bool        hadLifecycle = previous == ChunkState::Requested || previous == ChunkState::Building
                                      || previous == ChunkState::Ready;
        record = NodeRecord{state, updateIndex_};

        if (state == ChunkState::Split)
        {
            for (const ChunkCoord &child : children)
            {
                stack.push_back(child);
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
    constexpr std::uint32_t kMaxBalancePasses = 12;
    for (std::uint32_t pass = 0; pass < kMaxBalancePasses; ++pass)
    {
        bool changed = false;
        const std::size_t passLeafCount = leaves.size();
        for (std::size_t index = 0; index < passLeafCount; ++index)
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
    const std::uint64_t staleThreshold = updateIndex_ > 64 ? updateIndex_ - 64 : 1;
    nodes_.erase(std::remove_if(nodes_.begin(), nodes_.end(),
                     [staleThreshold](const std::pair<ChunkKey, NodeRecord> &entry)
                     { return entry.second.lastSelectedUpdate < staleThreshold; }),
        nodes_.end());
}

} // namespace planet
} // namespace engine
