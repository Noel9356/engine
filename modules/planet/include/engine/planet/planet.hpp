// Planet: the planet module's generation facade.
//
// Owns the parameters, the density field and the LOD octree, drives chunk
// mesh builds (async through a dedicated worker pool, inline on the calling
// thread when the pool is starved) and hands the current visible chunk set to
// consumers as borrowed views. One Planet instance renders one planet;
// instances are cheap enough for tools to create several (the demo's
// determinism self-test builds three, plus a bare Planet for mask variants).
//
// Threading: update() / copyVisibleChunks() / sampling are MAIN THREAD only
// (standard for capability calls); worker threads only ever touch the pure
// buildChunkMesh path plus two mutex-guarded handoff lists, so mesh output is
// independent of scheduling. Borrowed pointers from copyVisibleChunks stay
// valid until the NEXT update() call (eviction may free their meshes).

#pragma once

#include <engine/planet/climate.hpp>
#include <engine/planet/density.hpp>
#include <engine/planet/meshing.hpp>
#include <engine/planet/octree.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace engine
{
namespace planet
{

// Borrowed view of one drawable chunk mesh (valid until the next update()).
// Fields are append-only across ABI revisions (see capabilities.hpp): ABI 2
// later gained surfaceGroup alongside isWater (the terrain mesh is
// partitioned into surface groups, see meshing.hpp).
struct PlanetChunkView
{
    ChunkKey             key          = 0;
    Vec3                 centerMeters = {};                 // chunk-local vertex offset origin
    const PlanetVertex  *vertices     = nullptr;
    std::uint32_t        vertexCount  = 0;
    const std::uint32_t *indices      = nullptr;
    std::uint32_t        indexCount   = 0;
    bool                 isWater      = false;
    // Content version of the mesh (FNV-1a over the vertex/index buffers),
    // added in ABI 2. A chunk whose geometry was rebuilt (e.g. after a
    // conforming-mask change) keeps its key but changes its hash, so
    // consumers can detect the rebuild and refresh their uploaded copies.
    std::uint64_t        meshHash     = 0;
    // Surface group of a terrain view (ground = 0, rock = 1; see
    // terrainSurfaceGroup* in meshing.hpp). Water views carry 0: consumers
    // distinguish them through isWater.
    std::uint32_t        surfaceGroup = 0;
};

class Planet final
{
public:
    explicit Planet(const PlanetGenerationParams &params);
    ~Planet();

    Planet(const Planet &)            = delete;
    Planet &operator=(const Planet &) = delete;

    const PlanetGenerationParams &params() const;
    const PlanetStatistics       &statistics() const;

    // Advances the LOD selection for this frame's eye (planet-local meters).
    // culling (optional, may be null) narrows the selection to the chunks the
    // camera's frustum can touch (see PlanetCullingVolume): the streamer then
    // generates only visible chunks - nothing behind the camera, nothing off
    // screen - and reselects whenever the volume changes (a panning camera
    // streams its newly revealed ground lazily, never all upfront).
    void update(const Vec3 &eyeLocal, const PlanetCullingVolume *culling = nullptr);

    // Copies borrowed views of every visible, ready chunk (terrain first,
    // then that chunk's water patch), followed by the streaming-fallback
    // ancestors of visible chunks still building (see lodMissingFallback).
    // Returns the count written; call visibleChunkCapacity() to size the
    // buffer.
    std::uint32_t copyVisibleChunks(PlanetChunkView *output, std::uint32_t capacity);
    std::uint32_t visibleChunkCapacity() const;

    // Sampling helpers (pure).
    double        surfaceRadius(const Vec3 &unitDirection) const;
    double        densityAt(const Vec3 &point) const;
    ClimateSample sampleClimate(const Vec3 &unitDirection, double radiusMeters, double slope01) const;
    ReliefSample  sampleRelief(const Vec3 &unitDirection) const;

    // Synchronous pure build of one chunk, hashed - the determinism
    // self-test entry point (same function the workers call). The mask
    // overload lets self-tests exercise conforming builds explicitly.
    std::uint64_t buildChunkHash(const ChunkCoord &coord) const;
    std::uint64_t buildChunkHash(const ChunkCoord &coord, const ConformMask &conformMask) const;

private:
    // Cache eviction scan cadence, in updates.
    static constexpr std::uint64_t kEvictionScanInterval = 8;

    struct BuiltMesh
    {
        ChunkMesh     mesh;
        Vec3          center             = {};
        ConformMask   conformMask        = {}; // mask the mesh was built with (rebuild trigger on change)
        bool          terrainGroupEmpty[terrainSurfaceGroupCount] = {true, true};
        bool          waterEmpty         = true;
        std::uint64_t lastSelectedUpdate = 0;
    };

    struct PendingBuild
    {
        ChunkCoord  coord;
        ConformMask conformMask;
        ChunkMesh   mesh;
    };

    void requestBuild(const ChunkCoord &coord, const ConformMask &conformMask);
    std::uint32_t collectPendingBuilds(); // returns the number of meshes delivered
    void buildOneInline();
    void evictStaleMeshes();
    void buildWorkerLoop();

    // Streaming coverage for one visible chunk without a finished mesh (see
    // lodMissingFallback in params.hpp): records its nearest finished
    // distance-appropriate ancestor as a fallback draw and its coarsest
    // pure ancestor as a scaffold build candidate. Both walks are memoized
    // per pass in readyLeafAncestorStamps_ / ancestorFallbackMemo_, so
    // chunks sharing ancestor chains resolve in near-constant time. Holes
    // under the same parent share the whole chain above it, so the sweep
    // itself runs once per parent (sweepAncestorCoverage through
    // parentCoverage_) and planChunkCoverage applies the result per hole.
    void planChunkCoverage(const ChunkCoord &coord, const Vec3 &eyeLocal);

    // The chain-sweep result for one hole's ancestor chain (see
    // planChunkCoverage): the nearest finished ancestor that may stand in
    // (0 = none finished, kNoCoverageStandIn = one exists but fails the
    // stand-in width bound) and the pure scaffold target.
    struct AncestorCoverage
    {
        ChunkKey      nearestReady = 0;
        bool          pureFound    = false;
        ChunkCoord    pure{};
    };
    AncestorCoverage sweepAncestorCoverage(const ChunkCoord &hole, const Vec3 &eyeLocal);

    // Owned copy: capability callers pass temporaries (stack locals layered
    // over the module config), so a reference would dangle.
    PlanetGenerationParams        params_;
    DensityField                  field_;
    PlanetOctree                  octree_;

    std::vector<ChunkCoord> lastVisible_;

    // Conforming masks of the current visible set, index-aligned with lastVisible_ and recomputed whenever the
    // visible list changes (a chunk's mask is a pure function of the sorted visible list and its own coordinate, so
    // an unchanged list pins the masks).
    std::vector<ConformMask> visibleMasks_;
    VisibleLeafSet           visibleLeafSet_;

    // Scratch for the nearest-first request pass of update() (kept at member
    // scope so streaming frames do not reallocate the candidate buffer).
    // Fine entries carry the INDEX of the visible chunk (and of its
    // conforming mask in visibleMasks_), scaffold entries the ancestor
    // coordinate: the sort moves only the small tuple, and the mask is read
    // once when the request is issued.
    struct BuildCandidate
    {
        double        apparentSize = 0.0;    // chunk width / distance (screen-size priority)
        std::uint32_t visibleIndex = 0;       // fine entries: index into lastVisible_ / visibleMasks_
        ChunkCoord    scaffold{};            // scaffold entries: the ancestor to build
        bool          isScaffold   = false;
    };
    std::vector<BuildCandidate> candidateScratch_;

    // Streaming coverage state (see lodMissingFallback in params.hpp).
    // fallbackDraws_ holds the finished ancestor chunks serving as draw
    // stand-ins this pass (sorted, deduplicated); copyVisibleChunks emits
    // them after the visible set and their eviction stamps refresh in the
    // pass, so reused coverage is not aged out while it is on screen.
    // scaffoldNeeded_ holds the pure ancestors whose builds the queue must
    // keep alive this pass; the stale-entry cancellation keeps visible-set
    // entries plus these. readyLeafAncestorStamps_ marks, per pass, every
    // ancestor of a finished visible chunk (an ancestor of a missing chunk
    // is "shared" while stamped and "pure" once unstamped);
    // ancestorFallbackMemo_ caches the nearest finished ancestor per chain
    // node. All three carry update stamps instead of being cleared, so a
    // streaming pass never reallocates.
    std::vector<ChunkKey>        fallbackDraws_;
    std::unordered_set<ChunkKey> fallbackSeen_;
    std::unordered_set<ChunkKey> scaffoldNeeded_;
    // Visible indices of the current pass's holes (chunks without a mesh of
    // their own): the coverage planning runs only when this is non-empty,
    // so the finished steady state skips the stamping and planning passes.
    std::vector<std::uint32_t>   holeScratch_;
    // Holes of the current pass whose coverage sweep found no stand-in at
    // all (no finished ancestor fit their distance). Non-zero means the
    // visible region still lacks its coverage umbrella, which promotes
    // scaffold builds ahead of fine ones for the pass (see the request
    // loop in update()).
    std::uint32_t coverageDeficitCount_ = 0;
    // Chain-sweep results for the current pass, keyed by the holes' parent
    // (siblings share the sweep); reset with the rest of the pass scratch.
    std::unordered_map<ChunkKey, AncestorCoverage> parentCoverage_;
    std::unordered_map<ChunkKey, std::uint64_t> readyLeafAncestorStamps_;
    struct FallbackMemo
    {
        ChunkKey      nearestReady = 0; // 0 = no finished ancestor on the rest of the chain
        std::uint64_t stamp        = 0;
    };
    std::unordered_map<ChunkKey, FallbackMemo> ancestorFallbackMemo_;

    // Sorted copy of the in-flight key set for one visible pass (see the
    // update() note): keeps the per-chunk in-flight probe off the mutex.
    std::vector<ChunkKey> inflightScratch_;

    // Ready cache: chunk key -> built mesh. Node-based, so mesh storage stays put while the table grows; capacity is
    // reserved up front so streaming never rehashes.
    std::unordered_map<ChunkKey, BuiltMesh> readyMeshes_;

    // Worker -> main handoff.
    std::mutex                                   pendingMutex_;
    std::vector<PendingBuild>                    pendingBuilds_;
    std::vector<ChunkCoord>                      pendingFailures_;
    // In-flight keys (queued or building). Main-thread state: requestBuild
    // and collectPendingBuilds are the only mutators, and both run inside
    // update() on the capability's main thread; workers hand results over
    // through pendingBuilds_ above and never touch this set.
    std::unordered_set<ChunkKey>                 inflightBuild_;

    // DEDICATED BUILD POOL. Chunk builds deliberately do NOT run on the
    // kernel TaskScheduler: the software renderer floods that scheduler with
    // rasterizer tile tasks every frame, and on small machines the ground
    // views' build wave then crawls (one completion per hundreds of updates)
    // no matter what priority the tasks carry. A private pool gives the
    // builds guaranteed progress; results still reach the main thread through
    // the pendingBuilds_ handoff, and mesh contents stay pure functions of
    // (params, coordinate, mask) - order independent.
    std::mutex                                   buildQueueMutex_;
    std::condition_variable                      buildQueueSignal_;
    std::deque<std::pair<ChunkCoord, ConformMask>> buildQueue_;
    std::vector<std::thread>                     buildThreads_;
    bool                                         buildThreadsStopping_ = false;
    // Consecutive updates that had queued work but received no finished mesh. When the pool is starved of CPU for this
    // long, one chunk is built on the calling thread so the visible set still converges (see update()).
    std::uint32_t                                starvedUpdates_ = 0;

    // Last eye the visible set was selected for. Selection is a pure
    // function of (eye, culling volume, params, field), so an unchanged eye
    // AND volume pin the visible list and the update path skips the descent,
    // balancing and mask pass entirely (streaming, stamping and eviction
    // still run).
    bool           hasSelectionEye_ = false;
    Vec3           selectionEye_{};
    bool           hasSelectionCulling_ = false;
    PlanetCullingVolume selectionCulling_{};

    std::uint64_t updateCount_ = 0;
    PlanetStatistics statistics_;

    // Per-pass scratch for the coverage walks (planChunkCoverage).
    std::vector<ChunkKey> coverageChainScratch_;
};

} // namespace planet
} // namespace engine

