// Octree LOD selection for the planet module (see octree.hpp).
//
// The tree covers one root cube centered on the planet. Descending selects
// ever smaller nodes near the camera; three cheap geometric tests keep the
// working set tiny:
//   * shell-band pruning - cubes entirely inside the solid core or entirely
//     outside the terrain band can never intersect the surface;
//   * horizon culling - nodes hidden behind the planet's horizon from the
//     current eye point are skipped;
//   * angular refinement - a node refines while its voxel subtends more
//     than lodAngularVoxelRadians at the eye, so detail follows apparent
//     size: the whole visible disk of a globe view refines to a few-pixel
//     voxel depth and surface views keep the finest depth in the near field.
//
// Selection output is sorted by (depth, x, y, z) so the visible list itself
// is deterministic; per-chunk MESHES are pure functions anyway (see
// meshing.hpp), which is what makes generation order-independent.

#ifndef ENGINE_PLANET_OCTREE_HPP
#define ENGINE_PLANET_OCTREE_HPP

#include <engine/planet/density.hpp>
#include <engine/planet/meshing.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>

#include <vector>

namespace engine
{
namespace planet
{

enum class ChunkState : std::uint32_t
{
    Empty = 0,   // pruned: the node's cube cannot touch the terrain band
    Wanted,      // selected for display this update, mesh not requested yet
    Requested,   // queued for an async build
    Building,    // build in flight on a worker thread
    Ready,       // mesh built and cached
    Split,       // interior node: its children are the rendered selection
    Hidden       // culled by the horizon test (nothing rendered here)
};

class PlanetOctree final
{
public:
    // The field is used for solid/air corner probes (subtree pruning); it
    // must outlive the octree (Planet owns both).
    PlanetOctree(const PlanetGenerationParams &params, const DensityField &field);

    // Selects the visible node set for this frame's eye position (planet-
    // local, meters). Appends to outVisible in sorted (depth, x, y, z) order
    // and refreshes per-node selection bookkeeping. Cheap: no geometry.
    void selectVisible(const Vec3 &eyeLocal, std::vector<ChunkCoord> &outVisible);

    // Computes the conforming mask for one chunk against the SORTED visible
    // leaf list emitted by selectVisible: a face segment conforms when the
    // depth-(depth-1) cell beyond it (diagonal ring included) is itself a
    // rendered leaf - exactly the situation where the coarse neighbor's own
    // lattice chords define the shared surface, so the fine isoline must
    // follow the coarse interpolant to meet it without cracks. Membership is
    // a binary search over the visible list, which makes the mask a pure
    // function of (visible set, coord) - every chunk touching the same
    // boundary points derives the same decisions.
    void computeConformMask(const ChunkCoord &coord, const std::vector<ChunkCoord> &visibleSortedLeaves,
        ConformMask &outMask) const;

    // Current lifecycle state of a node (Empty when pruned).
    ChunkState stateOf(const ChunkCoord &coord) const;

    void setState(const ChunkCoord &coord, ChunkState state);

private:
    struct NodeRecord
    {
        ChunkState    state = ChunkState::Wanted;
        std::uint64_t lastSelectedUpdate = 0;
    };

    const PlanetGenerationParams &params_;
    const DensityField           &field_;
    double                        solidMarginMeters_ = 0.0;     // density bound for "fully solid/air" probes
    double                        innerOccluderRadius2_ = 0.0;  // squared radius of the guaranteed-solid occluder ball
    std::uint64_t                 updateIndex_ = 0;

    // Sparse node map: only nodes touched by selection (visited + pruned).
    // Keys via makeChunkKey; max depth 20 keeps coordinates in 20 bits.
    std::vector<std::pair<ChunkKey, NodeRecord>> nodes_; // sorted flat map

    const NodeRecord *findRecord(ChunkKey key) const;
    NodeRecord *findRecord(ChunkKey key);
    NodeRecord &recordFor(ChunkKey key);

    // Geometric tests shared with selection.
    bool cubeIntersectsShell(double minCubeRadius, double maxCubeRadius) const;
    bool horizonHidden(const Vec3 &nodeCenter, double nodeSize, const Vec3 &eyeLocal) const;

    // Applies the pruning + refinement tests of one node without touching the
    // node map. Returns the resulting state (Empty, Hidden, Split, or a
    // rendered-leaf state); outRefineChildren receives the children when the
    // state is Split.
    ChunkState classifyNode(const ChunkCoord &coord, const Vec3 &eyeLocal,
        std::vector<ChunkCoord> *outRefineChildren) const;

    // True when the node's state renders a mesh (lifecycle states kept by the
    // facade plus this update's fresh selection).
    static bool rendersMesh(ChunkState state);

    // Splits a rendered leaf immediately (2:1 balancing): records Split and
    // classifies the eight children with the normal rules, appending every
    // newly rendered child leaf to outNewLeaves.
    void forceSplit(const ChunkCoord &coord, const Vec3 &eyeLocal, std::vector<ChunkCoord> &outNewLeaves);

    // Rule-driven classification + recording used by forceSplit on children:
    // classifies the node, stores its state, recursively descends into
    // rule-refined children, and collects newly rendered leaves.
    void classifyAndRecord(const ChunkCoord &coord, const Vec3 &eyeLocal, std::vector<ChunkCoord> &outNewLeaves);

    // Depth of the SHALLOWEST rendered leaf covering the neighbor region of
    // coord along offset (in {-1,0,1}^3 \ {0}, cell units at coord.depth), or
    // -1 when that direction holds no rendered material. This is the adjacent
    // material's LOD level used by the 2:1 balance test.
    int neighborMaterialDepth(const ChunkCoord &coord, int offsetX, int offsetY, int offsetZ) const;
};

} // namespace planet
} // namespace engine

#endif // ENGINE_PLANET_OCTREE_HPP
