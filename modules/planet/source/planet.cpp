// Planet: the planet module's generation facade (see planet.hpp).

#include <engine/planet/planet.hpp>

#include <algorithm>
#include <cmath>
#include <mutex>

namespace engine
{
namespace planet
{

Planet::Planet(const PlanetGenerationParams &params, TaskScheduler *taskScheduler)
    : params_(params)
    , field_(params_)
    , octree_(params_, field_)
    , taskScheduler_(taskScheduler)
{
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

    // 1. Collect finished builds from the previous frames (workers push into
    //    the pending map; the main thread owns the ready cache).
    collectPendingBuilds();

    // 2. Select the visible set for this eye.
    octree_.selectVisible(eyeLocal, lastVisible_);
    const std::vector<ChunkCoord> &visible = lastVisible_;

    // 2b. Conforming masks. A chunk's mask is a pure function of the sorted
    // visible list and its coordinate (each face segment conforms when the
    // depth-(depth-1) cell beyond it is itself a rendered leaf - see
    // octree.hpp), so the masks only change when the visible set changes.
    // Chunks whose stored mesh was built under a different mask are rebuilt;
    // the stale mesh keeps rendering until the conforming rebuild lands.
    if (visible != previousVisible_)
    {
        conformMasks_.clear();
        for (const ChunkCoord &coord : visible)
        {
            ConformMask mask;
            octree_.computeConformMask(coord, visible, mask);
            conformMasks_.emplace(makeChunkKey(coord.depth, coord.x, coord.y, coord.z), mask);
        }
        previousVisible_ = visible;
    }

    // 3. Request missing meshes (OLD scan order - bisect).
    std::uint32_t newRequests    = 0;
    std::uint32_t inflightBuilds = 0;
    {
        std::lock_guard<std::mutex> lock(inflightMutex_);
        inflightBuilds = static_cast<std::uint32_t>(inflightBuild_.size());
    }
    if (inflightBuilds < params_.maxConcurrentBuilds)
    {
        for (const ChunkCoord &coord : visible)
        {
            const ChunkKey key = makeChunkKey(coord.depth, coord.x, coord.y, coord.z);
            const auto     maskIt = conformMasks_.find(key);
            const ConformMask mask = maskIt != conformMasks_.end() ? maskIt->second : ConformMask{};
            const auto     readyIt = readyMeshes_.find(key);
            if (readyIt != readyMeshes_.end() && readyIt->second.conformMask == mask)
            {
                continue;
            }
            bool alreadyInflight = false;
            {
                std::lock_guard<std::mutex> lock(inflightMutex_);
                alreadyInflight = inflightBuild_.find(key) != inflightBuild_.end();
            }
            if (alreadyInflight)
            {
                continue;
            }
            if (newRequests >= params_.maxNewBuildsPerUpdate || inflightBuilds + newRequests >= params_.maxConcurrentBuilds)
            {
                break;
            }
            requestBuild(coord, mask);
            ++newRequests;
        }
    }
    std::uint32_t lastNewRequests = newRequests;

    // 3b. Inline build slice. The dedicated pool threads compete with the
    // host's render workers for cores; on small machines a heavy frame can
    // starve them to a trickle. Building a bounded slice ON the calling
    // thread guarantees the visible set converges: the slice costs a few
    // milliseconds per update and every mesh is a pure function of
    // (params, coordinate, mask), so this changes scheduling only, never
    // contents. Completed meshes go through the same pending handoff.
    {
        const auto inlineDeadline = std::chrono::steady_clock::now()
            + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(inlineBuildSliceSeconds_));
        while (std::chrono::steady_clock::now() < inlineDeadline)
        {
            ChunkCoord coord{};
            ConformMask conformMask{};
            {
                std::lock_guard<std::mutex> lock(buildQueueMutex_);
                if (buildQueue_.empty())
                {
                    break;
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
    }

    // 4. Refresh selection stamps (used by eviction) and statistics.
    std::uint64_t triangles   = 0;
    std::uint64_t vertexCount = 0;
    std::uint32_t readyVisible = 0;
    for (const ChunkCoord &coord : visible)
    {
        const ChunkKey key = makeChunkKey(coord.depth, coord.x, coord.y, coord.z);
        const auto     it  = readyMeshes_.find(key);
        if (it != readyMeshes_.end())
        {
            ++readyVisible;
            it->second.lastSelectedUpdate = updateCount_;
            triangles += it->second.mesh.triangleCount;
            vertexCount += it->second.mesh.terrainVertices.size() + it->second.mesh.waterVertices.size();
        }
    }

    evictStaleMeshes();

    if (updateCount_ % 100 == 0)
    {
        std::uint32_t depthHistogram[16] = {};
        for (const ChunkCoord &coord : visible)
        {
            if (coord.depth < 16)
            {
                ++depthHistogram[coord.depth];
            }
        }
        std::fprintf(stderr, "planet-diag: update %llu visible %zu readyVisible %u requests %u cache %zu started %u completed %u eye(%.1f,%.1f,%.1f) depths",
            static_cast<unsigned long long>(updateCount_), visible.size(), readyVisible, lastNewRequests, readyMeshes_.size(),
            workerStartedCount_.load(), workerCompletedCount_.load(),
            eyeLocal.x, eyeLocal.y, eyeLocal.z);
        for (std::uint32_t depth = 0; depth < 16; ++depth)
        {
            if (depthHistogram[depth] != 0)
            {
                std::fprintf(stderr, " d%u:%u", depth, depthHistogram[depth]);
            }
        }
        // Max center distance per depth (up to a few leaves each) - verifies
        // where each depth's leaves actually sit.
        double maxDistPerDepth[16] = {};
        for (const ChunkCoord &coord : visible)
        {
            if (coord.depth >= 16)
            {
                continue;
            }
            const double nodeSize = chunkSizeMeters(params_, coord.depth);
            const Vec3 nodeCenter{-params_.derivedRootHalfMeters + nodeSize * (static_cast<double>(coord.x) + 0.5),
                -params_.derivedRootHalfMeters + nodeSize * (static_cast<double>(coord.y) + 0.5),
                -params_.derivedRootHalfMeters + nodeSize * (static_cast<double>(coord.z) + 0.5)};
            const double dist = length(nodeCenter - eyeLocal);
            maxDistPerDepth[coord.depth] = std::max(maxDistPerDepth[coord.depth], dist);
        }
        std::fprintf(stderr, " maxdist");
        for (std::uint32_t depth = 0; depth < 16; ++depth)
        {
            if (depthHistogram[depth] != 0)
            {
                std::fprintf(stderr, " d%u:%.0f", depth, maxDistPerDepth[depth]);
            }
        }
        std::fprintf(stderr, "\n");
    }

    statistics_.chunksVisible    = static_cast<std::uint32_t>(visible.size());
    statistics_.chunksReady      = readyVisible;
    statistics_.chunksBuilding   = inflightBuilds + lastNewRequests;
    statistics_.chunksReadyTotal = static_cast<std::uint32_t>(readyMeshes_.size());
    statistics_.triangleCount    = triangles;
    statistics_.verticesTotal    = vertexCount;
    statistics_.octreeDepthUsed  = params_.derivedMaxDepth;
    statistics_.lastUpdateMilliseconds
        = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - updateStart).count();
}

std::uint32_t Planet::copyVisibleChunks(PlanetChunkView *output, std::uint32_t capacity)
{
    if (output == nullptr || capacity == 0)
    {
        return 0;
    }
    std::uint32_t copied = 0;
    for (const ChunkCoord &coord : lastVisible_)
    {
        if (copied >= capacity)
        {
            break;
        }
        const ChunkKey key  = makeChunkKey(coord.depth, coord.x, coord.y, coord.z);
        const auto     it   = readyMeshes_.find(key);
        if (it == readyMeshes_.end())
        {
            continue;
        }
        const BuiltMesh &built = it->second;
        if (!built.terrainEmpty)
        {
            output[copied].key           = key;
            output[copied].centerMeters  = built.center;
            output[copied].vertices      = built.mesh.terrainVertices.data();
            output[copied].vertexCount   = static_cast<std::uint32_t>(built.mesh.terrainVertices.size());
            output[copied].indices       = built.mesh.terrainIndices.data();
            output[copied].indexCount    = static_cast<std::uint32_t>(built.mesh.terrainIndices.size());
            output[copied].isWater       = false;
            ++copied;
        }
        if (!built.waterEmpty)
        {
            if (copied >= capacity)
            {
                break;
            }
            output[copied].key           = key;
            output[copied].centerMeters  = built.center;
            output[copied].vertices      = built.mesh.waterVertices.data();
            output[copied].vertexCount   = static_cast<std::uint32_t>(built.mesh.waterVertices.size());
            output[copied].indices       = built.mesh.waterIndices.data();
            output[copied].indexCount    = static_cast<std::uint32_t>(built.mesh.waterIndices.size());
            output[copied].isWater       = true;
            ++copied;
        }
    }
    return copied;
}

std::uint32_t Planet::visibleChunkCapacity() const
{
    // Every visible chunk may contribute a terrain AND a water draw.
    return static_cast<std::uint32_t>(lastVisible_.size()) * 2u;
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
    return field_.climate().sample(unitDirection, radiusMeters, slope01);
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
    {
        std::lock_guard<std::mutex> lock(inflightMutex_);
        if (!inflightBuild_.insert(key).second)
        {
            return;
        }
    }
    octree_.setState(coord, ChunkState::Requested);

    // Hand the build to the dedicated pool (see the pool comment in the
    // header for why chunk builds do not ride the kernel scheduler).
    {
        std::lock_guard<std::mutex> lock(buildQueueMutex_);
        buildQueue_.emplace_back(coord, conformMask);
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
        {
            std::lock_guard<std::mutex> lock(pendingMutex_);
            ++workerStartedCount_;
            if (workerStartedCount_.load() <= 24)
            {
                std::fprintf(stderr, "  worker START d%u(%u,%u,%u) at %.3f s\n", coord.depth, coord.x, coord.y, coord.z,
                    std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count());
            }
        }
        const auto buildStart = std::chrono::steady_clock::now();
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
            ++workerCompletedCount_;
            pendingFailures_.push_back(coord);
            continue;
        }
        catch (...)
        {
            std::fprintf(stderr, "  build EXCEPTION d%u(%u,%u,%u): unknown\n", coord.depth, coord.x, coord.y, coord.z);
            std::lock_guard<std::mutex> lock(pendingMutex_);
            ++workerCompletedCount_;
            pendingFailures_.push_back(coord);
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(pendingMutex_);
            ++workerCompletedCount_;
            if (workerCompletedCount_.load() <= 24)
            {
                std::fprintf(stderr, "  worker DONE  d%u(%u,%u,%u) at %.3f s, build %.1f ms\n", coord.depth, coord.x,
                    coord.y, coord.z,
                    std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - buildStart).count());
            }
            pendingBuilds_.push_back(PendingBuild{coord, conformMask, std::move(mesh)});
        }
    }
}

void Planet::collectPendingBuilds()
{
    std::vector<PendingBuild> finished;
    std::vector<ChunkCoord>   failures;
    {
        std::lock_guard<std::mutex> lock(pendingMutex_);
        finished.swap(pendingBuilds_);
        failures.swap(pendingFailures_);
    }

    for (PendingBuild &entry : finished)
    {
        const ChunkKey key = makeChunkKey(entry.coord.depth, entry.coord.x, entry.coord.y, entry.coord.z);
        BuiltMesh      built;
        built.mesh         = std::move(entry.mesh);
        built.conformMask  = entry.conformMask;
        built.center       = Vec3{-params_.derivedRootHalfMeters
                                + chunkStepMeters(params_, entry.coord.depth)
                                    * static_cast<double>(static_cast<std::int64_t>(entry.coord.x) * static_cast<std::int64_t>(params_.leafGridN) + params_.leafGridN / 2),
            -params_.derivedRootHalfMeters
                                + chunkStepMeters(params_, entry.coord.depth)
                                    * static_cast<double>(static_cast<std::int64_t>(entry.coord.y) * static_cast<std::int64_t>(params_.leafGridN) + params_.leafGridN / 2),
            -params_.derivedRootHalfMeters
                                + chunkStepMeters(params_, entry.coord.depth)
                                    * static_cast<double>(static_cast<std::int64_t>(entry.coord.z) * static_cast<std::int64_t>(params_.leafGridN) + params_.leafGridN / 2)};
        built.terrainEmpty = built.mesh.terrainVertices.empty();
        built.waterEmpty   = built.mesh.waterVertices.empty();
        built.lastSelectedUpdate = updateCount_;
        readyMeshes_.insert_or_assign(key, std::move(built));
        octree_.setState(entry.coord, ChunkState::Ready);
        {
            std::lock_guard<std::mutex> lock(inflightMutex_);
            inflightBuild_.erase(key);
        }
    }
    for (const ChunkCoord &coord : failures)
    {
        octree_.setState(coord, ChunkState::Wanted);
        std::lock_guard<std::mutex> lock(inflightMutex_);
        inflightBuild_.erase(makeChunkKey(coord.depth, coord.x, coord.y, coord.z));
    }
}

void Planet::evictStaleMeshes()
{
    // Bound the cache: meshes not selected for chunkEvictionUpdates updates,
    // or beyond chunkCacheCapacity entries (oldest first), are released.
    struct Victim
    {
        ChunkKey      key;
        std::uint64_t stamp;
    };
    std::vector<Victim> victims;
    for (const auto &entry : readyMeshes_)
    {
        const std::uint64_t age = updateCount_ - entry.second.lastSelectedUpdate;
        if (age > params_.chunkEvictionUpdates)
        {
            victims.push_back(Victim{entry.first, entry.second.lastSelectedUpdate});
        }
    }
    if (readyMeshes_.size() > params_.chunkCacheCapacity)
    {
        std::vector<Victim> byAge;
        byAge.reserve(readyMeshes_.size());
        for (const auto &entry : readyMeshes_)
        {
            byAge.push_back(Victim{entry.first, entry.second.lastSelectedUpdate});
        }
        std::sort(byAge.begin(), byAge.end(), [](const Victim &a, const Victim &b) { return a.stamp < b.stamp; });
        const std::size_t overflow = readyMeshes_.size() - params_.chunkCacheCapacity;
        for (std::size_t index = 0; index < overflow && index < byAge.size(); ++index)
        {
            victims.push_back(byAge[index]);
        }
    }
    for (const Victim &victim : victims)
    {
        readyMeshes_.erase(victim.key);
    }
}

void Planet::finishBuild(const ChunkCoord &coord, const ConformMask &conformMask)
{
    // Scheduler-less path: run the pure build here and insert directly into
    // the ready cache (main thread; no worker contention).
    ChunkMesh mesh = buildChunkMesh(params_, field_, coord, conformMask);
    const ChunkKey key = makeChunkKey(coord.depth, coord.x, coord.y, coord.z);
    BuiltMesh built;
    built.mesh         = std::move(mesh);
    built.conformMask  = conformMask;
    built.center       = Vec3{-params_.derivedRootHalfMeters
                           + chunkStepMeters(params_, coord.depth)
                               * static_cast<double>(static_cast<std::int64_t>(coord.x) * static_cast<std::int64_t>(params_.leafGridN) + params_.leafGridN / 2),
        -params_.derivedRootHalfMeters
                           + chunkStepMeters(params_, coord.depth)
                               * static_cast<double>(static_cast<std::int64_t>(coord.y) * static_cast<std::int64_t>(params_.leafGridN) + params_.leafGridN / 2),
        -params_.derivedRootHalfMeters
                           + chunkStepMeters(params_, coord.depth)
                               * static_cast<double>(static_cast<std::int64_t>(coord.z) * static_cast<std::int64_t>(params_.leafGridN) + params_.leafGridN / 2)};
    built.terrainEmpty = built.mesh.terrainVertices.empty();
    built.waterEmpty   = built.mesh.waterVertices.empty();
    built.lastSelectedUpdate = updateCount_;
    readyMeshes_.insert_or_assign(key, std::move(built));
    octree_.setState(coord, ChunkState::Ready);
}

} // namespace planet
} // namespace engine
