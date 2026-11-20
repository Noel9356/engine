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
#include <limits>
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

// Streaming-coverage stand-in bound: an ancestor may stand in for a hole
// only when its own chunk width stays within this factor of the hole's
// distance to the terrain shell. The factor bounds the stand-in's on-screen
// extent (about 2 radians wide at worst) so a far hole cannot pull a
// planet-scale ancestor over the screen; coarser ancestors are wider, so
// the first ancestor to fail the bound closes the hole's sweep.
inline constexpr double kCoverageStandInWidthFactor = 2.0;

// Sweep result marker: the nearest finished ancestor exists but fails the
// stand-in width bound, so the hole has no stand-in this pass (coarser
// ancestors are wider and fail too). Never a real chunk key.
inline constexpr ChunkKey kNoCoverageStandIn = std::numeric_limits<ChunkKey>::max();

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
        visibleLeafSet_.assign(lastVisible_);
        visibleMasks_.resize(lastVisible_.size());
        for (std::size_t index = 0; index < lastVisible_.size(); ++index)
        {
            octree_.computeConformMask(lastVisible_[index], visibleLeafSet_, visibleMasks_[index]);
        }
        selectionChanged = true;
        selectionEye_    = eyeLocal;
        hasSelectionEye_ = true;
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
        parentCoverage_.clear();

        std::uint32_t readyVisible = 0;
        std::uint64_t triangles    = 0;
        std::uint64_t vertexCount  = 0;
        std::uint32_t depthUsed    = 0;
        coverageDeficitCount_ = 0;
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
                for (std::uint32_t group = 0; group < terrainSurfaceGroupCount; ++group)
                {
                    vertexCount += readyIt->second.mesh.terrainVertices[group].size();
                }
                vertexCount += readyIt->second.mesh.waterVertices.size();
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
            // Requests within this update's budget, apparent-size first (see the
            // note above the pass). Fine chunks and scaffold ancestors share the
            // in-flight capacity; each kind also carries its own per-update
            // request budget so neither can crowd the other out entirely.
            //
            // When some holes lack any stand-in (no finished ancestor fits
            // their distance yet - the region's coverage umbrella is
            // incomplete), the scaffold builds go FIRST: each umbrella chunk
            // serves every hole in its region at once and turns whole rings
            // of holes into covered ground, while a fine chunk serves the
            // one hole it stands for. The apparent-size sort still applies
            // within each kind. Once every hole finds a stand-in the order
            // reverts to pure apparent size.
            const bool scaffoldFirst = coverageDeficitCount_ > 0;
            std::uint32_t fineRequests     = 0;
            std::uint32_t scaffoldRequests = 0;
            const auto tryRequest = [this, &visible, &inflightBuilds, &fineRequests, &scaffoldRequests](
                const BuildCandidate &candidate) -> bool
            {
                if (fineRequests >= params_.maxNewBuildsPerUpdate
                    && scaffoldRequests >= params_.lodScaffoldBuildsPerUpdate)
                {
                    return false; // both request budgets spent
                }
                if (inflightBuilds + fineRequests + scaffoldRequests >= params_.maxConcurrentBuilds)
                {
                    return false; // no in-flight slot left this update
                }
                if (candidate.isScaffold)
                {
                    if (scaffoldRequests >= params_.lodScaffoldBuildsPerUpdate)
                    {
                        return true; // scaffold budget spent; keep scanning
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
                        return true; // fine budget spent; keep scanning
                    }
                    const std::uint32_t visibleIndex = candidate.visibleIndex;
                    requestBuild(visible[visibleIndex], visibleMasks_[visibleIndex]);
                    ++fineRequests;
                }
                return true;
            };
            if (scaffoldFirst)
            {
                for (const BuildCandidate &candidate : candidates)
                {
                    if (!candidate.isScaffold)
                    {
                        continue;
                    }
                    if (!tryRequest(candidate))
                    {
                        break;
                    }
                }
            }
            for (const BuildCandidate &candidate : candidates)
            {
                if (!tryRequest(candidate))
                {
                    break;
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

Planet::AncestorCoverage Planet::sweepAncestorCoverage(const ChunkCoord &hole, const Vec3 &eyeLocal)
{
    // One sweep up the ancestor chain (parent first) collects both pieces of
    // streaming coverage for the hole. Every hole under the same parent
    // shares the chain above that parent, so the caller runs this once per
    // parent per pass (see parentCoverage_). The two searches gate
    // themselves differently:
    //
    // * fallback draw - the nearest finished ancestor whose chunk width fits
    //   the hole's distance (see kCoverageStandInWidthFactor). The voxel
    //   screen-space rule with its 2x tolerance governs what the streamer
    //   BUILDS (scaffolds below), not what it DRAWS: a stand-in coarser than
    //   the voxel rule would justify still beats a hole to the sky/haze, and
    //   the rule's edge sits exactly where the fine chunks the eye actually
    //   waits for live - stopping the draw search there leaves the far half
    //   of every ground view uncovered (near-eye fine chunks win the build
    //   budget, so the ancestors a far hole's chain would need do not exist
    //   yet). A stand-in never paints more than the hole it covers (the fine
    //   chunks that share its region win the depth test), and it is replaced
    //   by the fine mesh it shadows once that lands. Finished siblings inside
    //   the stand-in's region are re-drawn at the coarser LOD until then; the
    //   overlap is confined to that region and shrinks as the holes under it
    //   fill.
    // * scaffold - the coarsest UNSTAMPED ancestor that still passes the
    //   screen-space rule and the width bound (pure: no finished visible
    //   chunk lies under it, see readyLeafAncestorStamps_). Its build covers
    //   every hole in its region without overlapping finished geometry, and
    //   because the target rides the same screen-space rule, coverage arrives
    //   coarse at range and fine near the eye - chunks closer to the viewer
    //   stream smaller, like the selection itself. Stamps split the chain:
    //   everything above the first stamped node shares finished material and
    //   cannot be a pure target.
    //
    // The sweep memoizes the fallback answer for every chain node it passes
    // while the fallback search is still open: a node is only reached after
    // all finer chain nodes found nothing finished, so they share the same
    // nearest finished ancestor.
    coverageChainScratch_.clear();
    AncestorCoverage coverage{};
    bool             pureClosed   = false; // the stamped prefix began: no purer target above
    bool             scaffoldOpen = true;  // the rule-gated scaffold search is still running
    std::size_t      sweepMemoEnd = 0;     // chain nodes whose fallback answer is settled
    // Reference distance for the width bound: the hole's own piece of the
    // terrain shell. The honest distance from the eye to that patch of
    // ground has a tangential AND a radial component - a hole the eye hovers
    // over is a whole eye-altitude away even though its tangential separation
    // is ~0 (a step-sized floor would read it as "closer than its own voxel"
    // and starve near-eye holes of stand-ins entirely). Every ancestor of
    // the hole stands for the same patch of ground, so one hole-level bound
    // keeps the sweep's stop condition monotone: coarser ancestors are wider
    // and fail a bound the hole's own distance justifies.
    const double eyeRadius = std::max(length(eyeLocal), 1.0e-9);
    double holeWidthBound = 0.0;
    {
        const Vec3   holeCenter        = chunkCenterMeters(params_, hole);
        const double dx = holeCenter.x - eyeLocal.x;
        const double dy = holeCenter.y - eyeLocal.y;
        const double dz = holeCenter.z - eyeLocal.z;
        const double centerDistance    = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double radialComponent   = (eyeLocal.x * dx + eyeLocal.y * dy + eyeLocal.z * dz) / eyeRadius;
        const double tangentialSquared = std::max(0.0,
            centerDistance * centerDistance - radialComponent * radialComponent);
        const double radialSeparation  = std::abs(length(holeCenter) - eyeRadius);
        const double holeShellDistance = std::max(chunkStepMeters(params_, hole.depth),
            std::max(std::sqrt(tangentialSquared), radialSeparation));
        holeWidthBound = holeShellDistance * kCoverageStandInWidthFactor;
    }
    for (std::uint32_t depth = hole.depth; depth-- > 0u;)
    {
        const ChunkCoord ancestor    = ancestorOf(hole, depth);
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
        // tangentially and can never stand in at coarse detail, so the
        // radial separation (the eye's altitude over the node's own shell
        // radius) floors the distance the way it does for the hole bound.
        const double centerDistance = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double radialComponent = (eyeLocal.x * dx + eyeLocal.y * dy + eyeLocal.z * dz) / eyeRadius;
        const double radialSquared   = radialComponent * radialComponent;
        const double shellDistance   = std::max(chunkStepMeters(params_, depth),
            std::max(std::sqrt(std::max(0.0, centerDistance * centerDistance - radialSquared)),
                std::abs(length(center) - eyeRadius)));
        // Screen-size bound for stand-in material: an ancestor may cover a
        // hole only when the ancestor's own chunk width fits the hole's
        // distance (width within kCoverageStandInWidthFactor of it, so the
        // stand-in's on-screen extent stays bounded). Without the bound a
        // planet-scale ancestor covers ground-view holes with whole-planet
        // triangles - unbounded fill and tile-scan cost for one hole.
        // Coarser ancestors are wider, so once an ancestor fails the bound
        // every coarser one fails too.
        const bool fitsDistance = chunkSizeMeters(params_, depth) <= holeWidthBound;
        if (coverage.nearestReady == 0)
        {
            coverageChainScratch_.push_back(ancestorKey);
            sweepMemoEnd = coverageChainScratch_.size();
            const auto readyIt = readyMeshes_.find(ancestorKey);
            if (readyIt != readyMeshes_.end()
                && (std::any_of(std::begin(readyIt->second.terrainGroupEmpty),
                        std::end(readyIt->second.terrainGroupEmpty), [](bool empty) { return !empty; })
                    || !readyIt->second.waterEmpty))
            {
                // The nearest finished ancestor is settled; record whether
                // it may stand in (a too-wide stand-in closes the search -
                // coarser ones are wider still).
                coverage.nearestReady = fitsDistance ? ancestorKey : kNoCoverageStandIn;
            }
            else
            {
                const auto memoIt = ancestorFallbackMemo_.find(ancestorKey);
                if (memoIt != ancestorFallbackMemo_.end() && memoIt->second.stamp == updateCount_)
                {
                    coverage.nearestReady = memoIt->second.nearestReady;
                }
            }
        }
        if (scaffoldOpen)
        {
            if (chunkStepMeters(params_, depth) > shellDistance * params_.lodAngularVoxelRadians * 2.0)
            {
                scaffoldOpen = false; // too coarse for its distance; coarser ancestors only fall further behind
            }
            else if (!fitsDistance)
            {
                scaffoldOpen = false; // wider than the screen-size bound; coarser only falls further behind
            }
            else if (pureClosed)
            {
                scaffoldOpen = false; // finished material above: nothing coarser can be pure
            }
            else
            {
                const auto stampIt = readyLeafAncestorStamps_.find(ancestorKey);
                if (stampIt != readyLeafAncestorStamps_.end() && stampIt->second == updateCount_)
                {
                    pureClosed = true; // finished material above: nothing coarser can be pure
                    scaffoldOpen = false;
                }
                else
                {
                    coverage.pure      = ancestor; // kept: the coarsest passing pure node wins
                    coverage.pureFound = true;
                }
            }
        }
        if ((coverage.nearestReady != 0 || !fitsDistance) && !scaffoldOpen)
        {
            // The fallback answer is settled when a passing stand-in was
            // found, or when the width bound closed the search (the nearest
            // finished ancestor was too wide, so coarser ones fail too). No
            // better scaffold target exists above a closed search either.
            break;
        }
    }
    for (std::size_t index = 0; index < sweepMemoEnd; ++index)
    {
        ancestorFallbackMemo_[coverageChainScratch_[index]] = FallbackMemo{coverage.nearestReady, updateCount_};
    }
    return coverage;
}

void Planet::planChunkCoverage(const ChunkCoord &coord, const Vec3 &eyeLocal)
{
    // Applies one hole's streaming coverage. Every hole under the same
    // parent shares the ancestor chain above that parent, so the sweep runs
    // once per parent per pass (see parentCoverage_) and this function
    // applies its result per hole: the fallback draw (deduplicated across
    // the pass), the coverage-deficit count (per hole - it drives the
    // request loop's scaffold-first rule) and the scaffold build candidate
    // (deduplicated by the needed-scaffold set).
    const ChunkCoord parent       = ancestorOf(coord, coord.depth - 1u);
    const ChunkKey   parentKey    = makeChunkKey(parent.depth, parent.x, parent.y, parent.z);
    const auto       coverageSlot = parentCoverage_.find(parentKey);
    if (coverageSlot == parentCoverage_.end())
    {
        parentCoverage_.emplace(parentKey, sweepAncestorCoverage(coord, eyeLocal));
    }
    const AncestorCoverage &coverage = parentCoverage_.find(parentKey)->second;
    if (coverage.nearestReady != 0 && coverage.nearestReady != kNoCoverageStandIn
        && fallbackSeen_.insert(coverage.nearestReady).second)
    {
        fallbackDraws_.push_back(coverage.nearestReady);
    }
    if (coverage.nearestReady == 0 || coverage.nearestReady == kNoCoverageStandIn)
    {
        // This hole has no stand-in this pass: the region still lacks its
        // coverage umbrella (see the request loop's scaffold-first rule).
        ++coverageDeficitCount_;
    }
    if (coverage.pureFound && params_.lodScaffoldBuildsPerUpdate > 0)
    {
        const ChunkCoord &pure = coverage.pure;
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
        for (std::uint32_t group = 0; group < terrainSurfaceGroupCount && copied < capacity; ++group)
        {
            if (built.terrainGroupEmpty[group])
            {
                continue;
            }
            output[copied].key          = key;
            output[copied].centerMeters = built.center;
            output[copied].vertices     = built.mesh.terrainVertices[group].data();
            output[copied].vertexCount  = static_cast<std::uint32_t>(built.mesh.terrainVertices[group].size());
            output[copied].indices      = built.mesh.terrainIndices[group].data();
            output[copied].indexCount   = static_cast<std::uint32_t>(built.mesh.terrainIndices[group].size());
            output[copied].isWater      = false;
            output[copied].meshHash     = built.mesh.terrainHash;
            output[copied].surfaceGroup = group;
            ++copied;
        }
        if (!built.waterEmpty && copied < capacity)
        {
            output[copied].key          = key;
            output[copied].centerMeters = built.center;
            output[copied].vertices     = built.mesh.waterVertices.data();
            output[copied].vertexCount  = static_cast<std::uint32_t>(built.mesh.waterVertices.size());
            output[copied].indices      = built.mesh.waterIndices.data();
            output[copied].indexCount   = static_cast<std::uint32_t>(built.mesh.waterIndices.size());
            output[copied].isWater      = true;
            output[copied].meshHash     = built.mesh.waterHash;
            output[copied].surfaceGroup = 0;
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
    // Every visible chunk and every fallback ancestor may contribute one
    // draw per terrain surface group plus a water draw.
    return static_cast<std::uint32_t>((lastVisible_.size() + fallbackDraws_.size())
        * (terrainSurfaceGroupCount + 1u));
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
            for (std::uint32_t group = 0; group < terrainSurfaceGroupCount; ++group)
            {
                built.terrainGroupEmpty[group] = built.mesh.terrainVertices[group].empty();
            }
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

