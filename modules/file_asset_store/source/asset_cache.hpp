// Bounded, reference-counted cache of whole-file contents.
//
// Owns the record map, the identity-to-handle path index, the resident and
// in-flight byte budgets, and the LRU/FIFO eviction pass. Path
// canonicalization (root confinement, symlink resolution, length cap) lives
// here too because it needs the root configuration and is called on every
// request. The cache does not perform file I/O itself; the module
// orchestrates reserve → load → commit through the cache's budget hooks.

#ifndef ENGINE_FILE_ASSET_STORE_ASSET_CACHE_HPP
#define ENGINE_FILE_ASSET_STORE_ASSET_CACHE_HPP

#include "asset_config.hpp"
#include "asset_record.hpp"
#include <engine/file_asset_store/capabilities.hpp>
#include <engine/file_asset_store/catalog.hpp>
#include <engine/kernel/profiling.hpp>

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace engine::file_asset_store
{

class AssetCache final
{
public:
    AssetCache() = default;

    // Stores pointers to the config, stopping flag, and optional profiler.
    // Called from the module's construct() after parsing.
    void configure(const AssetStoreConfig &config, std::atomic<bool> &stopping,
        PerformanceProfiler *profiler) noexcept
    {
        config_   = &config;
        stopping_ = &stopping;
        profiler_ = profiler;
    }

    // Resolves a request path to a canonical absolute path. Returns "" when
    // the path is unusable, too long, or (with a configured root) escapes the
    // root; relative paths are interpreted against the root directory. When
    // a canonical-path cache is configured, recently resolved inputs are
    // reused to avoid the weakly_canonical syscall on every request.
    std::string canonicalPath(const char *path) const;

    // Finds an existing record by handle (returns nullptr if not found).
    std::shared_ptr<AssetRecord> find(AssetHandle handle);

    // Looks up an existing record by identity. On hit, bumps the reference
    // count and the LRU stamp; returns the handle or invalidAsset.
    AssetHandle findByIdentity(const std::string &identity, AssetError &error);

    // Creates a new record for the given identity/path; returns the handle.
    // Evicts one unreferenced record if the cache is at capacity. On
    // failure, sets error and returns invalidAsset.
    AssetHandle createRecord(const std::filesystem::path &resolvedPath, const std::string &identity,
        const AssetCatalogEntry *cookedEntry, AssetError &error);

    // Reference counting. retain fails on overflow; release erases the
    // record on the last reference unless cacheUnreferencedAssets keeps it.
    bool retain(AssetHandle handle);
    void release(AssetHandle handle);

    // Bumps the record's LRU stamp from the module-wide logical clock.
    void touch(const std::shared_ptr<AssetRecord> &record) noexcept;

    // Evicts unreferenced finished assets until the byte target is met (or
    // all, when purgeAll is true). Returns bytes freed.
    std::uint64_t purgeUnused(std::uint64_t targetBytes, bool purgeAll);

    // Snapshot of cache-wide counters.
    bool statistics(AssetCacheStatistics &output);

    // Budget hooks used by the module's readAsset orchestrator.
    // reserveForLoad: checks + reserves the in-flight and resident budgets,
    // evicting LRU assets first when resident space is short. Returns false
    // (and sets the record's state) when the size does not fit.
    bool reserveForLoad(const std::shared_ptr<AssetRecord> &record, std::uint64_t newSize, AssetError *error);
    // releaseReservation: returns bytes reserved by a failed read.
    void releaseReservation(std::uint64_t bytes);
    // commitLoad: converts the reservation to resident bytes, swaps the
    // buffer in under dataMutex, and bumps the content version.
    void commitLoad(const std::shared_ptr<AssetRecord> &record, std::vector<std::uint8_t> &loaded, std::uint64_t newSize);

    // Drops all records (called by stop()).
    void clear();

    [[nodiscard]] const AssetStoreConfig &config() const noexcept
    {
        return *config_;
    }

private:
    // Internal: evicts unreferenced finished records until the byte and
    // count targets are met. Caller must hold mutex_.
    std::uint64_t evictLocked(std::uint64_t targetBytes, std::size_t targetCount, bool purgeAll = false);
    // Internal: erases one record and returns its byte count. Caller must
    // hold mutex_.
    std::uint64_t eraseRecordLocked(std::unordered_map<AssetHandle, std::shared_ptr<AssetRecord>>::iterator iterator);

    const AssetStoreConfig *config_   = nullptr;
    std::atomic<bool>      *stopping_ = nullptr;
    PerformanceProfiler     *profiler_ = nullptr;
    std::mutex              mutex_;
    std::unordered_map<AssetHandle, std::shared_ptr<AssetRecord>> records_;
    std::unordered_map<std::string, AssetHandle>                   pathIndex_;
    AssetHandle                                                   nextHandle_ = 1;
    std::atomic<std::uint64_t>                                    accessSequence_{0};
    std::uint64_t                                                 residentBytes_   = 0;
    std::uint64_t                                                 reservedBytes_   = 0;
    // Running counters for fast statistics snapshots (maintained under
    // mutex_ alongside residentBytes_).
    std::uint64_t evictableBytes_ = 0;
    std::size_t   evictableCount_ = 0;
    // Canonical-path cache (input → canonical). Bounded FIFO; cleared when
    // full so the common case stays O(1) without a linked list. Guarded by
    // canonicalPathMutex_: requestAsset is callable from any thread, so the
    // cache must not rely on external serialization.
    mutable std::mutex                                    canonicalPathMutex_;
    mutable std::unordered_map<std::string, std::string>  canonicalPathCache_;
    mutable std::vector<std::string>                      canonicalPathCacheOrder_;
};

} // namespace engine::file_asset_store

#endif
