// Planet: the planet module's generation facade (see planet.hpp).

#include <chrono>
#include <cstdint>
#include <engine/planet/density.hpp>
#include <engine/planet/meshing.hpp>
#include <engine/planet/octree.hpp>
#include <engine/planet/params.hpp>
#include <engine/planet/planet.hpp>
#include <engine/planet/types.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <mutex>
#include <ratio>
#include <thread>
#include <utility>
#include <vector>

namespace engine
{
namespace planet
{

namespace
{

// The ancestor of `coord` at the given (smaller) depth: the chain to the
// root drops one lattice coordinate bit per level, so the ancestor index is
// the coordinate shifted right by the depth difference.
ChunkCoord ancestorOf(const ChunkCoord &coord, std::uint32_t depth)
{
    const std::uint32_t shift = coord.depth - depth;
    return ChunkCoord{depth, coord.x >> shift, coord.y >> shift, coord.z >> shift};
}

} // namespace

Planet::Planet(const PlanetGenerationParams &params)
    : params_(params)
    , field_(params_)
    , octree_(params_, field_)
{
    // Reserve the whole cache capacity up front so streaming never rehashes on the main thread.
    readyMeshes_.reserve(static_cast<std::size_t>(params_.chunkCacheCapacity) + 1024u);

    // The coverage bookkeeping sees every ancestor node the visible set
    // touches; reserving up front keeps the streaming passes rehash-free.
    readyLeafAncestorStamps_.reserve(8192u);
    ancestorFallbackMemo_.reserve(8192u);
    fallbackDraws_.reserve(256u);
    candidateScratch_.reserve(2048u);

    // Dedicated build pool: one thread per permitted concurrent build, at
    // least one (see requestBuild - the pool fully replaces the scheduler
    // path for chunk builds).
    const std::uint32_t threadCount = std::max(1u, std::min(params_.maxConcurrentBuilds,
        static_cast<std::uint32_t>(std::thread::hardware_concurrency())));
    for (std::uint32_t index = 0; index < threadCount; ++index)
    {
        buildThreads_.emplace_back([this]() { buildWorkerLoop(); });
    }
}

Planet::~Planet()
{
    // Stop the pool, then drain whatever the threads still delivered. Mesh
    // contents are pure functions of (params, coordinate, mask), so nothing
    // in-flight can outlive the field/params.
    {
        std::lock_guard<std::mutex> lock(buildQueueMutex_);
        buildThreadsStopping_ = true;
        buildQueue_.clear();
    }
    buildQueueSignal_.notify_all();
    for (std::thread &thread : buildThreads_)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }
    buildThreads_.clear();
    collectPendingBuilds();
}

const PlanetGenerationParams &Planet::params() const
{
    return params_;
}

const PlanetStatistics &Planet::statistics() const
{
    return statistics_;
}

void Planet::update(const Vec3 &eyeLocal)
{
    const auto updateStart = std::chrono::steady_clock::now();
    ++updateCount_;

    // 1. Collect finished builds from the previous frames (workers push into the pending list; the main thread owns
    //    the ready cache).
    const std::uint32_t delivered = collectPendingBuilds();

    // 2. Select the visible set for this eye. The selection is a pure
    //    function of (eye, params, field): an eye that has moved less than
    //    the selection threshold pins the list, so the descent, the 2:1
    //    balancing fixpoint, the conform-mask pass and the per-node stamping
    //    are skipped entirely - a static camera (or a warming-up streamer
    //    that renders nothing) costs an update only what streaming actually
    //    delivered, and a walking camera re-selects only when its movement
    //    since the last selection can change the LOD decision anywhere
    //    (the threshold defaults to the finest voxel: a finer step cannot
    //    change which chunk any point resolves to, because the refinement
    //    rings at every depth are many voxels wide).
    bool selectionChanged = false;
    const double selectionThreshold = params_.selectionUpdateDistanceMeters > 0.0
        ? params_.selectionUpdateDistanceMeters
        : params_.derivedVoxelMeters;
    if (!hasSelectionEye_ || lengthSquared(eyeLocal - selectionEye_) > selectionThreshold * selectionThreshold)
    {
        octree_.selectVisible(eyeLocal, lastVisible_);
        selectionEye_    = eyeLocal;
        hasSelectionEye_ = true;
        selectionChanged = true;

        // 2b. Conforming masks. A chunk's mask is a pure function of the sorted visible list and its coordinate (each
        // face segment conforms when the depth-(depth-1) cell beyond it is itself a rendered leaf - see octree.hpp), so
        // the masks only change when the visible set changes. Chunks whose stored mesh was built under a different mask
        // are rebuilt; the stale mesh keeps rendering until the conforming rebuild lands.
        visibleLeafSet_.assign(lastVisible_);
        visibleMasks_.resize(lastVisible_.size());
        for (std::size_t index = 0; index < lastVisible_.size(); ++index)
        {
            octree_.computeConformMask(lastVisible_[index], visibleLeafSet_, visibleMasks_[index]);
        }

    }

    const std::vector<ChunkCoord> &visible = lastVisible_;

    // The visible-set pass stamps cache entries for eviction, refreshes the
    // statistics and issues new build requests. It only has work when one of
    // those inputs moved: a delivered build (the cache changed), a new
    // selection (visibility and masks changed), or the eviction cadence (the
    // stamps must stay fresh so eviction never ages a visible chunk out).
    // The stamp-refresh and eviction cadence follows the configured
    // eviction age whenever it is shorter than the default scan interval: a
    // threshold of 2..7 updates with an 8-update cadence could let a
    // visible chunk's stamp age past the threshold between scans, evicting
    // meshes the streamer is about to select again.
    const std::uint32_t evictionScanInterval = std::max<std::uint32_t>(1u,
        std::min<std::uint32_t>(kEvictionScanInterval, params_.chunkEvictionUpdates));
    const bool evictionDue = updateCount_ % evictionScanInterval == 0;
    const bool visiblePassNeeded = selectionChanged || delivered > 0 || evictionDue;

    // One sorted copy of the in-flight key set per visible pass: probing the
    // copy is a binary search over a plain vector. The live set is
    // main-thread state (mutated only inside update()), so the copy only has
    // to be an over-approximation - a request issued anyway is filtered by
    // requestBuild's own insertion check.
    std::vector<ChunkKey> &inflightScratch = inflightScratch_;
    std::uint32_t inflightBuilds = 0;
    const auto alreadyInflight = [&inflightScratch](ChunkKey key)
    {
        return std::binary_search(inflightScratch.begin(), inflightScratch.end(), key);
    };
    if (visiblePassNeeded)
    {
        inflightScratch.assign(inflightBuild_.begin(), inflightBuild_.end());
        std::sort(inflightScratch.begin(), inflightScratch.end());
        inflightBuilds = static_cast<std::uint32_t>(inflightScratch.size());
    }
    // Missing (or mask-stale) visible chunks with their eye distances: the
    // request budget is spent in order of APPARENT SIZE (chunk width over
    // distance, descending), which is the same quantity the LOD refinement
    // rule is built on. A coarse ring at range and a fine chunk right under
    // a walking camera then compete on equal, screen-based terms: globe
    // views still fill their disk coarse-first (wide chunks subtend more),
    // while a walking eye gets the chunks it is standing on before anything
    // else. Scaffold candidates (see lodMissingFallback) join the same
    // sort, so their wider coverage competes on the same screen-based terms
    // and the widest gaps stream the widest chunks. The coordinate tuple
    // breaks distance ties, keeping the request order a pure function of
    // (eye, visible set).
    std::vector<BuildCandidate> &candidates = candidateScratch_;
    if (visiblePassNeeded)
    {
        candidates.clear();
        fallbackDraws_.clear();
        fallbackSeen_.clear();
        scaffoldNeeded_.clear();

        std::uint32_t readyVisible = 0;
        std::uint64_t triangles    = 0;
        std::uint64_t vertexCount  = 0;
        std::uint32_t depthUsed    = 0;
        // Visible chunks without a mesh of their own (the holes the coverage
        // planning below fills). Collected in the scan so the stamping and
        // planning runs are skipped entirely on passes where every visible
        // chunk is already served - the finished steady state pays nothing.
        holeScratch_.clear();
        for (std::size_t index = 0; index < visible.size(); ++index)
        {
            const ChunkCoord &coord = visible[index];
            const ChunkKey    key   = makeChunkKey(coord.depth, coord.x, coord.y, coord.z);
            const auto        readyIt = readyMeshes_.find(key);
            const bool        hasMesh = readyIt != readyMeshes_.end();
            depthUsed = std::max(depthUsed, coord.depth);
            if (hasMesh)
            {
                ++readyVisible;
                readyIt->second.lastSelectedUpdate = updateCount_;
                triangles += readyIt->second.mesh.triangleCount;
                vertexCount += readyIt->second.mesh.terrainVertices.size() + readyIt->second.mesh.waterVertices.size();
                if (readyIt->second.conformMask == visibleMasks_[index])
                {
                    continue;
                }
                // Mask-stale: the stored mesh keeps drawing until the
                // conforming rebuild lands, so only the rebuild is queued.
            }
            else
            {
                holeScratch_.push_back(static_cast<std::uint32_t>(index));
            }
            if (alreadyInflight(key))
            {
                continue; // queued or building: the request pass below must not duplicate it
            }
            const Vec3 center = chunkCenterMeters(params_, coord);
            const double dx = center.x - eyeLocal.x;
            const double dy = center.y - eyeLocal.y;
            const double dz = center.z - eyeLocal.z;
            const double distanceSquared = dx * dx + dy * dy + dz * dz;
            const double chunkWidth = chunkSizeMeters(params_, coord.depth);
            candidates.push_back(BuildCandidate{chunkWidth / std::max(std::sqrt(distanceSquared), 1.0e-6),
                static_cast<std::uint32_t>(index), ChunkCoord{}, false});
        }

        // Streaming coverage (see lodMissingFallback in params.hpp): stamp
        // the finished visible chunks' ancestor chains, then plan a fallback
        // draw plus a scaffold build for every hole. The stamps split each
        // hole's ancestor chain into a prefix that already carries finished
        // material and a pure suffix, which planChunkCoverage reads; the
        // walk stops at the first already-stamped node - stamps of one pass
        // form contiguous chains, so everything shallower is stamped too.
        if (!holeScratch_.empty() && params_.lodMissingFallback)
        {
            for (const ChunkCoord &coord : visible)
            {
                if (readyMeshes_.find(makeChunkKey(coord.depth, coord.x, coord.y, coord.z)) == readyMeshes_.end())
                {
                    continue;
                }
                for (std::uint32_t depth = coord.depth; depth-- > 0u;)
                {
                    const ChunkCoord ancestor    = ancestorOf(coord, depth);
                    const ChunkKey   ancestorKey = makeChunkKey(ancestor.depth, ancestor.x, ancestor.y, ancestor.z);
                    std::uint64_t   &stamp      = readyLeafAncestorStamps_[ancestorKey];
                    if (stamp == updateCount_)
                    {
                        break;
                    }
                    stamp = updateCount_;
                }
            }
            for (const std::uint32_t holeIndex : holeScratch_)
            {
                planChunkCoverage(visible[holeIndex], eyeLocal);
            }
        }

        // Requests within this update's budget, apparent-size first (see the
        // note above the pass). Fine chunks and scaffold ancestors share the
        // in-flight capacity; each kind also carries its own per-update
        // request budget so neither can crowd the other out entirely.
        if (!candidates.empty())
        {
            std::sort(candidates.begin(), candidates.end(), [](const BuildCandidate &a, const BuildCandidate &b)
            {
                if (a.apparentSize != b.apparentSize)
                {
                    return a.apparentSize > b.apparentSize;
                }
                if (a.isScaffold != b.isScaffold)
                {
                    return !a.isScaffold; // fine entries first on ties
                }
                if (a.isScaffold)
                {
                    if (a.scaffold.depth != b.scaffold.depth) return a.scaffold.depth < b.scaffold.depth;
                    if (a.scaffold.x != b.scaffold.x) return a.scaffold.x < b.scaffold.x;
                    if (a.scaffold.y != b.scaffold.y) return a.scaffold.y < b.scaffold.y;
                    return a.scaffold.z < b.scaffold.z;
                }
                return a.visibleIndex < b.visibleIndex;
            });
            std::uint32_t fineRequests     = 0;
            std::uint32_t scaffoldRequests = 0;
            for (const BuildCandidate &candidate : candidates)
            {
                if (fineRequests >= params_.maxNewBuildsPerUpdate
                    && scaffoldRequests >= params_.lodScaffoldBuildsPerUpdate)
                {
                    break; // both request budgets spent
                }
                if (inflightBuilds + fineRequests + scaffoldRequests >= params_.maxConcurrentBuilds)
                {
                    break; // no in-flight slot left this update
                }
                if (candidate.isScaffold)
                {
                    if (scaffoldRequests >= params_.lodScaffoldBuildsPerUpdate)
                    {
                        continue;
                    }
                    // The scaffold mesh is built raw (no conforming mask):
                    // it draws as stand-in coverage among whatever tree the
                    // current selection has, and if the node itself is ever
                    // selected as a leaf the masked rebuild replaces it.
                    requestBuild(candidate.scaffold, ConformMask{});
                    ++scaffoldRequests;
                }
                else
                {
                    if (fineRequests >= params_.maxNewBuildsPerUpdate)
                    {
                        continue;
                    }
                    const std::uint32_t visibleIndex = candidate.visibleIndex;
                    requestBuild(visible[visibleIndex], visibleMasks_[visibleIndex]);
                    ++fineRequests;
                }
            }
            inflightBuilds += fineRequests + scaffoldRequests;
        }

        // Emit order of the fallback draws: deterministic by key, and each
        // entry's eviction stamp refreshes so reused coverage is not aged
        // out while it is on screen.
        if (!fallbackDraws_.empty())
        {
            std::sort(fallbackDraws_.begin(), fallbackDraws_.end());
            for (const ChunkKey fallbackKey : fallbackDraws_)
            {
                const auto fallbackIt = readyMeshes_.find(fallbackKey);
                if (fallbackIt != readyMeshes_.end())
                {
                    fallbackIt->second.lastSelectedUpdate = updateCount_;
                }
            }
        }

        statistics_.chunksVisible    = static_cast<std::uint32_t>(visible.size());
        statistics_.chunksReady      = readyVisible;
        statistics_.chunksBuilding   = inflightBuilds;
        statistics_.chunksFallback   = static_cast<std::uint32_t>(fallbackDraws_.size());
        statistics_.chunksReadyTotal = static_cast<std::uint32_t>(readyMeshes_.size());
        statistics_.triangleCount    = triangles;
        statistics_.verticesTotal    = vertexCount;
        statistics_.octreeDepthUsed  = depthUsed;
    }

    // 3b. Cancel stale queue entries: requests whose chunks nobody displays
    //     anymore would occupy build slots with meshes nobody needs, delaying
    //     the chunks the eye actually waits for. A queued (not yet started)
    //     build survives when its chunk is a visible leaf OR one of this
    //     pass's scaffold ancestors (the pure-ancestor coverage of a
    //     still-missing visible chunk, see lodMissingFallback); everything
    //     else is dropped, and builds already running on a worker finish and
    //     land in the cache as usual. A key can also appear twice in the
    //     queue (a re-request issued before the first entry was built) -
    //     cancelling removes every entry for the dropped key and releases
    //     its slot exactly once. The scan runs on the visible-pass cadence,
    //     right after the request loop, so freed slots are picked up by the
    //     next update.
    if (visiblePassNeeded && !buildQueue_.empty())
    {
        std::lock_guard<std::mutex> lock(buildQueueMutex_);
        std::size_t writeIndex = 0;
        for (std::size_t readIndex = 0; readIndex < buildQueue_.size(); ++readIndex)
        {
            const ChunkCoord &queuedCoord = buildQueue_[readIndex].first;
            const ChunkKey    queuedKey   = makeChunkKey(queuedCoord.depth, queuedCoord.x, queuedCoord.y, queuedCoord.z);
            if (visibleLeafSet_.contains(queuedCoord) || scaffoldNeeded_.contains(queuedKey))
            {
                buildQueue_[writeIndex++] = buildQueue_[readIndex];
            }
            else
            {
                inflightBuild_.erase(queuedKey);
            }
        }
        buildQueue_.resize(writeIndex);
    }

    // 3c. Starvation fallback. The dedicated pool threads compete with the host's render workers for cores; on a
    // machine small enough that they receive no CPU at all, nothing would ever finish. Building a chunk on the calling
    // thread costs a full chunk build, so it only happens after the pool has delivered nothing for a sustained run of
    // updates with work queued. Every mesh is a pure function of (params, coordinate, mask), so this changes
    // scheduling only, never contents.
    bool queuedWork = false;
    {
        std::lock_guard<std::mutex> lock(buildQueueMutex_);
        queuedWork = !buildQueue_.empty();
    }
    starvedUpdates_ = (queuedWork && delivered == 0) ? starvedUpdates_ + 1 : 0;
    if (params_.starvedInlineBuildIntervalUpdates > 0
        && starvedUpdates_ >= params_.starvedInlineBuildIntervalUpdates)
    {
        buildOneInline();
        starvedUpdates_ = 0;
    }

    // 4. Eviction scans the whole cache; ready meshes only age out over hundreds of updates, so a coarse cadence
    //    bounds the same set while keeping the scan off most frames. The visible pass above runs on the same cadence,
    //    so eviction never sees a visible chunk older than one scan interval.
    if (evictionDue)
    {
        evictStaleMeshes();
    }

    statistics_.lastUpdateMilliseconds
        = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - updateStart).count();
}

void Planet::buildOneInline()
{
    ChunkCoord  coord{};
    ConformMask conformMask{};
    {
        std::lock_guard<std::mutex> lock(buildQueueMutex_);
        if (buildQueue_.empty())
        {
            return;
        }
        coord       = buildQueue_.front().first;
        conformMask = buildQueue_.front().second;
        buildQueue_.pop_front();
    }
    try
    {
        ChunkMesh mesh = buildChunkMesh(params_, field_, coord, conformMask);
        std::lock_guard<std::mutex> lock(pendingMutex_);
        pendingBuilds_.push_back(PendingBuild{coord, conformMask, std::move(mesh)});
    }
    catch (...)
    {
        std::lock_guard<std::mutex> lock(pendingMutex_);
        pendingFailures_.push_back(coord);
    }
}

void Planet::planChunkCoverage(const ChunkCoord &coord, const Vec3 &eyeLocal)
{
    // One sweep up the ancestor chain (parent first) collects both pieces of
    // streaming coverage for this chunk. Every candidate is checked against
    // the selection's own screen-space rule with a 2x tolerance - the hole
    // itself sits at that rule's edge, so its direct parent always qualifies
    // and coarser ancestors qualify exactly as far as their detail allows
    // (voxel step within distance * lodAngularVoxelRadians * 2):
    //
    // * fallback draw - the nearest finished ancestor that passes. A
    //   stand-in never paints more of the screen than its detail justifies,
    //   so the fill cost stays proportional to the holes actually open, and
    //   the stand-in is replaced by the fine mesh it shadows once that
    //   lands. Finished siblings inside the stand-in's region are re-drawn
    //   at the coarser LOD until then; the overlap is confined to that
    //   region and shrinks as the holes under it fill.
    // * scaffold - the coarsest UNSTAMPED ancestor that passes (pure: no
    //   finished visible chunk lies under it, see readyLeafAncestorStamps_).
    //   Its build covers every hole in its region without overlapping
    //   finished geometry, and because the target rides the same
    //   screen-space rule, coverage arrives coarse at range and fine near
    //   the eye - chunks closer to the viewer stream smaller, like the
    //   selection itself. Stamps split the chain: everything above the first
    //   stamped node shares finished material and cannot be a pure target.
    //
    // The sweep memoizes the fallback answer for every chain node it passes
    // while the search is still open: a node is only reached after all finer
    // chain nodes found nothing finished, so they share the same nearest
    // finished ancestor.
    coverageChainScratch_.clear();
    ChunkKey   nearestReady = 0;
    ChunkCoord pure{};
    bool       pureFound  = false;
    bool       pureClosed = false; // the stamped prefix began: no purer target above
    std::size_t sweepMemoEnd = 0;  // chain nodes whose fallback answer is settled
    for (std::uint32_t depth = coord.depth; depth-- > 0u;)
    {
        const ChunkCoord ancestor    = ancestorOf(coord, depth);
        const ChunkKey   ancestorKey = makeChunkKey(ancestor.depth, ancestor.x, ancestor.y, ancestor.z);
        const Vec3       center      = chunkCenterMeters(params_, ancestor);
        const double     dx = center.x - eyeLocal.x;
        const double     dy = center.y - eyeLocal.y;
        const double     dz = center.z - eyeLocal.z;
        // Distance from the eye to this ancestor's piece of the terrain
        // shell. The shell runs at the eye's own radius, so the cross-radial
        // (tangential) separation to the node center is the honest distance
        // to the surface the ancestor would stand in for - the raw center
        // distance under-measures nothing but reads coarse ancestors whose
        // centers sit buried deep inside the planet as "near", while the
        // node-cube bounding sphere over-measures chunks whose shell patch
        // is simply beside the eye. A node the eye hovers over measures ~0
        // tangentially and can never stand in at coarse detail.
        const double eyeRadius    = std::max(length(eyeLocal), 1.0e-9);
        const double centerDistance = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double radialComponent = (eyeLocal.x * dx + eyeLocal.y * dy + eyeLocal.z * dz) / eyeRadius;
        const double radialSquared   = radialComponent * radialComponent;
        const double shellDistance   = std::max(chunkStepMeters(params_, depth),
            std::sqrt(std::max(0.0, centerDistance * centerDistance - radialSquared)));
        if (chunkStepMeters(params_, depth) > shellDistance * params_.lodAngularVoxelRadians * 2.0)
        {
            break; // too coarse for its distance; coarser ancestors only fall further behind
        }
        if (nearestReady == 0)
        {
            coverageChainScratch_.push_back(ancestorKey);
            sweepMemoEnd = coverageChainScratch_.size();
            const auto readyIt = readyMeshes_.find(ancestorKey);
            if (readyIt != readyMeshes_.end()
                && (!readyIt->second.terrainEmpty || !readyIt->second.waterEmpty))
            {
                nearestReady = ancestorKey;
            }
            else
            {
                const auto memoIt = ancestorFallbackMemo_.find(ancestorKey);
                if (memoIt != ancestorFallbackMemo_.end() && memoIt->second.stamp == updateCount_)
                {
                    nearestReady = memoIt->second.nearestReady;
                }
            }
        }
        if (!pureClosed)
        {
            const auto stampIt = readyLeafAncestorStamps_.find(ancestorKey);
            if (stampIt != readyLeafAncestorStamps_.end() && stampIt->second == updateCount_)
            {
                pureClosed = true; // finished material above: nothing coarser can be pure
            }
            else
            {
                pure      = ancestor; // kept: the coarsest passing pure node wins
                pureFound = true;
            }
        }
        if (nearestReady != 0 && pureClosed)
        {
            break; // fallback found and no better scaffold target above
        }
    }
    for (std::size_t index = 0; index < sweepMemoEnd; ++index)
    {
        ancestorFallbackMemo_[coverageChainScratch_[index]] = FallbackMemo{nearestReady, updateCount_};
    }
    if (nearestReady != 0 && fallbackSeen_.insert(nearestReady).second)
    {
        fallbackDraws_.push_back(nearestReady);
    }
    if (pureFound && params_.lodScaffoldBuildsPerUpdate > 0)
    {
        const ChunkKey pureKey = makeChunkKey(pure.depth, pure.x, pure.y, pure.z);
        if (readyMeshes_.find(pureKey) == readyMeshes_.end()
            && inflightBuild_.find(pureKey) == inflightBuild_.end()
            && scaffoldNeeded_.insert(pureKey).second)
        {
            const Vec3   scaffoldCenter = chunkCenterMeters(params_, pure);
            const double dx = scaffoldCenter.x - eyeLocal.x;
            const double dy = scaffoldCenter.y - eyeLocal.y;
            const double dz = scaffoldCenter.z - eyeLocal.z;
            const double distanceSquared = dx * dx + dy * dy + dz * dz;
            const double chunkWidth = chunkSizeMeters(params_, pure.depth);
            candidateScratch_.push_back(BuildCandidate{chunkWidth / std::max(std::sqrt(distanceSquared), 1.0e-6),
                0u, pure, true});
        }
    }
}

std::uint32_t Planet::copyVisibleChunks(PlanetChunkView *output, std::uint32_t capacity)
{
    if (output == nullptr || capacity == 0)
    {
        return 0;
    }
    std::uint32_t copied = 0;
    const auto emitBuilt = [output, capacity, &copied](ChunkKey key, const BuiltMesh &built)
    {
        if (!built.terrainEmpty && copied < capacity)
        {
            output[copied].key           = key;
            output[copied].centerMeters  = built.center;
            output[copied].vertices      = built.mesh.terrainVertices.data();
            output[copied].vertexCount   = static_cast<std::uint32_t>(built.mesh.terrainVertices.size());
            output[copied].indices       = built.mesh.terrainIndices.data();
            output[copied].indexCount    = static_cast<std::uint32_t>(built.mesh.terrainIndices.size());
            output[copied].isWater       = false;
            output[copied].meshHash      = built.mesh.terrainHash;
            ++copied;
        }
        if (!built.waterEmpty && copied < capacity)
        {
            output[copied].key           = key;
            output[copied].centerMeters  = built.center;
            output[copied].vertices      = built.mesh.waterVertices.data();
            output[copied].vertexCount   = static_cast<std::uint32_t>(built.mesh.waterVertices.size());
            output[copied].indices       = built.mesh.waterIndices.data();
            output[copied].indexCount    = static_cast<std::uint32_t>(built.mesh.waterIndices.size());
            output[copied].isWater       = true;
            output[copied].meshHash      = built.mesh.waterHash;
            ++copied;
        }
    };
    for (const ChunkCoord &coord : lastVisible_)
    {
        if (copied >= capacity)
        {
            break;
        }
        const ChunkKey key = makeChunkKey(coord.depth, coord.x, coord.y, coord.z);
        const auto     it  = readyMeshes_.find(key);
        if (it != readyMeshes_.end())
        {
            emitBuilt(key, it->second);
        }
    }
    // Streaming fallbacks: finished ancestors standing in for visible chunks
    // whose meshes are still building (see lodMissingFallback). They draw
    // after the visible set, so any fine chunk that shares their region
    // wins the depth test.
    for (const ChunkKey fallbackKey : fallbackDraws_)
    {
        if (copied >= capacity)
        {
            break;
        }
        const auto it = readyMeshes_.find(fallbackKey);
        if (it != readyMeshes_.end())
        {
            emitBuilt(fallbackKey, it->second);
        }
    }
    return copied;
}

std::uint32_t Planet::visibleChunkCapacity() const
{
    // Every visible chunk and every fallback ancestor may contribute a
    // terrain AND a water draw.
    return static_cast<std::uint32_t>((lastVisible_.size() + fallbackDraws_.size()) * 2u);
}

double Planet::surfaceRadius(const Vec3 &unitDirection) const
{
    return field_.surfaceRadius(unitDirection);
}

double Planet::densityAt(const Vec3 &point) const
{
    return field_.densityAt(point);
}

ClimateSample Planet::sampleClimate(const Vec3 &unitDirection, double radiusMeters, double slope01) const
{
    return field_.sampleClimate(unitDirection, radiusMeters, slope01);
}

ReliefSample Planet::sampleRelief(const Vec3 &unitDirection) const
{
    return field_.reliefAt(unitDirection);
}

std::uint64_t Planet::buildChunkHash(const ChunkCoord &coord) const
{
    return buildChunkHash(coord, ConformMask{});
}

std::uint64_t Planet::buildChunkHash(const ChunkCoord &coord, const ConformMask &conformMask) const
{
    // Synchronous pure build - the same entry point the async workers use,
    // exposed for determinism self-tests. The conforming mask is an input
    // like the coordinate, never global state.
    const ChunkMesh mesh = buildChunkMesh(params_, field_, coord, conformMask);
    return mesh.hash;
}

void Planet::requestBuild(const ChunkCoord &coord, const ConformMask &conformMask)
{
    const ChunkKey key = makeChunkKey(coord.depth, coord.x, coord.y, coord.z);
    if (!inflightBuild_.insert(key).second)
    {
        return;
    }
    // The two mutations after the insert can allocate (a FlatKeyMap rehash,
    // a deque growth); a failure here must not leave the key recorded as
    // in-flight with no queue entry behind it - nothing would ever deliver
    // or fail it, and one of the maxConcurrentBuilds slots would leak for
    // the planet's lifetime. Roll the insert back before propagating.
    try
    {
        octree_.setState(coord, ChunkState::Requested);

        // Hand the build to the dedicated pool (see the pool comment in the
        // header for why chunk builds do not ride the kernel scheduler).
        {
            std::lock_guard<std::mutex> lock(buildQueueMutex_);
            buildQueue_.emplace_back(coord, conformMask);
        }
    }
    catch (...)
    {
        inflightBuild_.erase(key);
        throw;
    }
    buildQueueSignal_.notify_one();
}

void Planet::buildWorkerLoop()
{
    for (;;)
    {
        ChunkCoord coord{};
        ConformMask conformMask{};
        {
            std::unique_lock<std::mutex> lock(buildQueueMutex_);
            buildQueueSignal_.wait(lock, [this]() { return buildThreadsStopping_ || !buildQueue_.empty(); });
            if (buildThreadsStopping_)
            {
                return;
            }
            coord       = buildQueue_.front().first;
            conformMask = buildQueue_.front().second;
            buildQueue_.pop_front();
        }
        ChunkMesh mesh;
        try
        {
            mesh = buildChunkMesh(params_, field_, coord, conformMask);
        }
        catch (const std::exception &error)
        {
            // Report the failure through the pending channel; never let an
            // exception escape the worker thread.
            std::fprintf(stderr, "  build EXCEPTION d%u(%u,%u,%u): %s\n", coord.depth, coord.x, coord.y,
                coord.z, error.what());
            std::lock_guard<std::mutex> lock(pendingMutex_);
            pendingFailures_.push_back(coord);
            continue;
        }
        catch (...)
        {
            std::fprintf(stderr, "  build EXCEPTION d%u(%u,%u,%u): unknown\n", coord.depth, coord.x, coord.y, coord.z);
            std::lock_guard<std::mutex> lock(pendingMutex_);
            pendingFailures_.push_back(coord);
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(pendingMutex_);
            pendingBuilds_.push_back(PendingBuild{coord, conformMask, std::move(mesh)});
        }
    }
}

std::uint32_t Planet::collectPendingBuilds()
{
    std::vector<PendingBuild> finished;
    std::vector<ChunkCoord>   failures;
    {
        std::lock_guard<std::mutex> lock(pendingMutex_);
        finished.swap(pendingBuilds_);
        failures.swap(pendingFailures_);
    }
    const std::uint32_t deliveredCount = static_cast<std::uint32_t>(finished.size());

    for (PendingBuild &entry : finished)
    {
        const ChunkKey key = makeChunkKey(entry.coord.depth, entry.coord.x, entry.coord.y, entry.coord.z);
        try
        {
            BuiltMesh      built;
            built.mesh         = std::move(entry.mesh);
            built.conformMask  = entry.conformMask;
            built.center       = chunkCenterMeters(params_, entry.coord);
            built.terrainEmpty = built.mesh.terrainVertices.empty();
            built.waterEmpty   = built.mesh.waterVertices.empty();
            built.lastSelectedUpdate = updateCount_;
            readyMeshes_.insert_or_assign(key, std::move(built));
            octree_.setState(entry.coord, ChunkState::Ready);
        }
        catch (...)
        {
            // The mesh payload was already consumed by the move, so this
            // delivery is lost - but the bookkeeping must stay consistent:
            // release the in-flight slot and mark the node wanted again so
            // the next visible pass re-requests it instead of waiting on a
            // key nothing will ever deliver.
            inflightBuild_.erase(key);
            octree_.setState(entry.coord, ChunkState::Wanted);
            continue;
        }
        inflightBuild_.erase(key);
    }
    for (const ChunkCoord &coord : failures)
    {
        octree_.setState(coord, ChunkState::Wanted);
        inflightBuild_.erase(makeChunkKey(coord.depth, coord.x, coord.y, coord.z));
    }
    return deliveredCount;
}

void Planet::evictStaleMeshes()
{
    // Bound the cache: meshes not selected for chunkEvictionUpdates updates
    // age out first, then the oldest survivors keep the cache within
    // chunkCacheCapacity. The stale set is read straight off the cache (a
    // single erase pass, no ordering work); only the capacity trim needs an
    // age order, and std::nth_element partitions just the oldest overflow
    // entries out instead of fully sorting a cache-sized array. Ties break
    // on the chunk key so the trim is deterministic across runs.
    struct Entry
    {
        ChunkKey      key;
        std::uint64_t stamp;
    };
    // Stale meshes erase straight off the cache, no ordering work: any mesh
    // the selection has not stamped for chunkEvictionUpdates updates leaves.
    for (auto it = readyMeshes_.begin(); it != readyMeshes_.end();)
    {
        if (updateCount_ - it->second.lastSelectedUpdate > params_.chunkEvictionUpdates)
        {
            it = readyMeshes_.erase(it);
        }
        else
        {
            ++it;
        }
    }

    const std::size_t survivors = readyMeshes_.size();
    const std::size_t overflow  = survivors > params_.chunkCacheCapacity ? survivors - params_.chunkCacheCapacity : 0;
    if (overflow == 0)
    {
        return;
    }
    std::vector<Entry> byAge;
    byAge.reserve(survivors);
    for (const auto &entry : readyMeshes_)
    {
        byAge.push_back(Entry{entry.first, entry.second.lastSelectedUpdate});
    }
    const auto older = [](const Entry &a, const Entry &b)
    {
        if (a.stamp != b.stamp)
        {
            return a.stamp < b.stamp;
        }
        return a.key < b.key;
    };
    std::nth_element(byAge.begin(), byAge.begin() + static_cast<std::ptrdiff_t>(overflow), byAge.end(), older);
    for (std::size_t index = 0; index < overflow; ++index)
    {
        readyMeshes_.erase(byAge[index].key);
    }
}

} // namespace planet
} // namespace engine

