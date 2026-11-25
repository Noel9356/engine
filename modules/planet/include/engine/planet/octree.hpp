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

#pragma once

#include <engine/planet/density.hpp>
#include <engine/planet/flat_key_map.hpp>
#include <engine/planet/meshing.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>

#include <array>
#include <cstdint>
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

// Hash index over the sorted visible leaf list: O(1) membership for conform-mask derivation. Rebuilt once whenever
// the visible list changes.
class VisibleLeafSet final
{
public:
    VisibleLeafSet();

    void assign(const std::vector<ChunkCoord> &sortedLeaves);
    [[nodiscard]] bool contains(const ChunkCoord &coord) const noexcept;

    // True when at least one leaf of this depth is rendered (cheap early-out for masks).
    [[nodiscard]] bool hasDepth(std::uint32_t depth) const noexcept
    {
        return depth < depthCounts_.size() && depthCounts_[depth] != 0;
    }

private:
    FlatKeyMap<std::uint8_t>       keys_;
    std::array<std::uint32_t, 32>  depthCounts_{};
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
    // culling (optional, may be null) adds view-frustum rejection: nodes
    // whose bounding sphere lies fully outside any plane are Hidden, so the
    // streamer generates only what the camera can see. The pointer is used
    // for this call only (never stored).
    void selectVisible(const Vec3 &eyeLocal, std::vector<ChunkCoord> &outVisible,
        const PlanetCullingVolume *culling = nullptr);

    // Computes the conforming mask for one chunk against the hash index of
    // the visible leaf set emitted by selectVisible: a face segment conforms
    // when the depth-(depth-1) cell beyond it (diagonal ring included) is
    // itself a rendered leaf - exactly the situation where the coarse
    // neighbor's own lattice chords define the shared surface, so the fine
    // isoline must follow the coarse interpolant to meet it without cracks.
    // Membership is an O(1) hash probe into the prebuilt index, which makes
    // the mask a pure function of (visible set, coord) - every chunk touching
    // the same boundary points derives the same decisions.
    void computeConformMask(const ChunkCoord &coord, const VisibleLeafSet &visibleLeaves, ConformMask &outMask) const;

    void setState(const ChunkCoord &coord, ChunkState state);

private:
    // Outcome of the solid/air corner probe. The probe depends only on the node's coordinate and the planet
    // parameters (never on the eye), so the verdict is computed once per node and kept with its record.
    enum class ProbeVerdict : std::uint8_t
    {
        Unknown = 0,
        Undecided,
        Empty
    };

    struct NodeRecord
    {
        std::uint64_t lastSelectedUpdate = 0;
        ChunkState    state              = ChunkState::Wanted;
        ProbeVerdict  probe              = ProbeVerdict::Unknown;
    };

    struct ChildList
    {
        std::array<ChunkCoord, 8> items{};
        std::uint32_t             count = 0;
    };

    const PlanetGenerationParams &params_;
    const DensityField           &field_;
    // Sound corner-probe margin per depth (see the constructor comment).
    std::array<double, maxSupportedOctreeDepth + 1> probeMargins_{};
    // Node bounding radius per depth (0.5 * nodeSize * sqrt(3)): a pure
    // function of depth alone, so the descent reads it from this table
    // instead of recomputing a sqrt per node visit.
    std::array<double, maxSupportedOctreeDepth + 1> boundRadii_{};
    double                        innerOccluderRadius2_ = 0.0;  // squared radius of the guaranteed-solid occluder ball
    double                        seaOccluderRadius2_   = 0.0;  // squared sea-sphere radius (occludes when the eye is above the water)
    double                        seaOccluderEyeMarginMeters = 0.5; // eye height above the sea sphere the sea-sphere occluder requires
    std::uint64_t                 updateIndex_ = 0;
    // Per-call culling volume (see selectVisible): borrowed, never owned,
    // cleared when the pass ends so a stale pointer cannot survive.
    const PlanetCullingVolume    *culling_ = nullptr;

    // Sparse node map: only nodes touched by selection (visited + pruned).
    // Keys via makeChunkKey; the depth cap keeps coordinates in 20 bits.
    FlatKeyMap<NodeRecord> nodes_;
    std::vector<ChunkCoord> descentStack_;  // scratch buffers reused across updates
    std::vector<ChunkCoord> leavesScratch_;

    const NodeRecord *findRecord(ChunkKey key) const;
    NodeRecord *findRecord(ChunkKey key);
    NodeRecord &recordFor(ChunkKey key);

    // Geometric tests shared with selection.
    bool cubeIntersectsShell(double minCubeRadius, double maxCubeRadius) const;
    bool horizonHidden(const Vec3 &nodeCenter, double nodeSize, const Vec3 &eyeLocal) const;
    // True when the node's bounding sphere lies fully outside any culling
    // plane (see selectVisible): sphere-vs-plane, the negative vertex of the
    // sphere is what the plane test scores.
    bool frustumHidden(const Vec3 &nodeCenter, double boundRadius) const;

    // Applies the pruning + refinement tests of one node without touching the
    // node map. Returns the resulting state (Empty, Hidden, Split, or a
    // rendered-leaf state); outRefineChildren receives the children when the
    // state is Split.
    ChunkState classifyNode(const ChunkCoord &coord, const Vec3 &eyeLocal, ChildList *outRefineChildren,
        NodeRecord &record) const;

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

