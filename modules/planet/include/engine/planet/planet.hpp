// Planet: the planet module's generation facade.
//
// Owns the parameters, the density field and the LOD octree, drives chunk
// mesh builds (async through the kernel TaskScheduler when available, inline
// otherwise) and hands the current visible chunk set to consumers as borrowed
// views. One Planet instance renders one planet; instances are cheap enough
// for tools to create several (the demo's determinism self-test builds two).
//
// Threading: update() / copyVisibleChunks() / sampling are MAIN THREAD only
// (standard for capability calls); worker threads only ever touch the pure
// buildChunkMesh path plus two mutex-guarded handoff lists, so mesh output is
// independent of scheduling. Borrowed pointers from copyVisibleChunks stay
// valid until the NEXT update() call (eviction may free their meshes).

#ifndef ENGINE_PLANET_PLANET_HPP
#define ENGINE_PLANET_PLANET_HPP

#include <engine/planet/climate.hpp>
#include <engine/planet/density.hpp>
#include <engine/planet/meshing.hpp>
#include <engine/planet/octree.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/types.hpp>

#include <engine/kernel/task_scheduler.hpp>

#include <chrono>
#include <cstdint>
#include <atomic>
#include <unordered_map>
#include <thread>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <unordered_set>
#include <vector>

namespace engine
{
namespace planet
{

// Borrowed view of one drawable chunk mesh (valid until the next update()).
struct PlanetChunkView
{
    ChunkKey             key          = 0;
    Vec3                 centerMeters = {};                 // chunk-local vertex offset origin
    const PlanetVertex  *vertices     = nullptr;
    std::uint32_t        vertexCount  = 0;
    const std::uint32_t *indices      = nullptr;
    std::uint32_t        indexCount   = 0;
    bool                 isWater      = false;
};

class Planet final
{
public:
    Planet(const PlanetGenerationParams &params, TaskScheduler *taskScheduler);
    ~Planet();

    Planet(const Planet &)            = delete;
    Planet &operator=(const Planet &) = delete;

    const PlanetGenerationParams &params() const;
    const PlanetStatistics       &statistics() const;

    // Advances the LOD selection for this frame's eye (planet-local meters),
    // collects finished builds and issues new ones.
    void update(const Vec3 &eyeLocal);

    // Copies borrowed views of every visible, ready chunk (terrain first,
    // then that chunk's water patch). Returns the count written; call
    // visibleChunkCapacity() to size the buffer.
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
    // Updates without a delivered mesh (while work is queued) before one chunk is built on the calling thread.
    static constexpr std::uint32_t kStarvedUpdatesBeforeInlineBuild = 30;
    // Cache eviction scan cadence, in updates.
    static constexpr std::uint64_t kEvictionScanInterval = 8;

    struct BuiltMesh
    {
        ChunkMesh     mesh;
        Vec3          center             = {};
        ConformMask   conformMask        = {}; // mask the mesh was built with (rebuild trigger on change)
        bool          terrainEmpty       = true;
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
    void finishBuild(const ChunkCoord &coord, const ConformMask &conformMask); // scheduler-less inline path

    // Owned copy: capability callers pass temporaries (stack locals layered
    // over the module config), so a reference would dangle.
    PlanetGenerationParams        params_;
    DensityField                  field_;
    PlanetOctree                  octree_;
    TaskScheduler                *taskScheduler_;
    TaskGroup                     buildGroup_;

    std::vector<ChunkCoord> lastVisible_;

    // Conforming masks of the current visible set, index-aligned with lastVisible_ and recomputed whenever the
    // visible list changes (a chunk's mask is a pure function of the sorted visible list and its own coordinate, so
    // an unchanged list pins the masks).
    std::vector<ConformMask> visibleMasks_;
    std::vector<ChunkCoord>  previousVisible_;
    VisibleLeafSet           visibleLeafSet_;

    // Ready cache: chunk key -> built mesh. Node-based, so mesh storage stays put while the table grows; capacity is
    // reserved up front so streaming never rehashes.
    std::unordered_map<ChunkKey, BuiltMesh> readyMeshes_;

    // Worker -> main handoff.
    std::mutex                                   pendingMutex_;
    std::vector<PendingBuild>                    pendingBuilds_;
    std::vector<ChunkCoord>                      pendingFailures_;
    std::mutex                                   inflightMutex_;
    std::unordered_set<ChunkKey>                 inflightBuild_; // keys queued or building

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

    std::uint64_t updateCount_ = 0;
    PlanetStatistics statistics_{};
};

} // namespace planet
} // namespace engine

#endif // ENGINE_PLANET_PLANET_HPP
