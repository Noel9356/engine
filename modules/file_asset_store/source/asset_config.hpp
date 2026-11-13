// Parsed configuration values shared between the cache, loader, and module.
//
// Built-in defaults match config.toml so the module works with no
// configuration at all. The module's construct() fills this struct from
// ParticipantContext configuration reads and passes it by const reference
// to the cache and loader.

#ifndef ENGINE_FILE_ASSET_STORE_ASSET_CONFIG_HPP
#define ENGINE_FILE_ASSET_STORE_ASSET_CONFIG_HPP

#include <cstdint>
#include <filesystem>

namespace engine::file_asset_store
{

// Eviction order for unreferenced finished records. Lru picks the
// least-recently-used record (lowest lastAccessTick); Fifo picks the
// oldest insertion (lowest handle, which is the allocation order).
enum class EvictionPolicy : std::uint8_t
{
    Lru = 0,
    Fifo
};

struct AssetStoreConfig
{
    // Per-file read cap.
    std::uint64_t maximumFileBytes = 512ull * 1024ull * 1024ull;
    // Live-record cap.
    std::size_t maximumAssets = 8192;
    // Queued asynchronous read cap.
    std::size_t maximumPendingRequests = 8192;
    // Cap on loaded bytes held in memory.
    std::uint64_t maximumResidentBytes = 2ull * 1024ull * 1024ull * 1024ull;
    // Cap on bytes reserved by reads in progress.
    std::uint64_t maximumInflightBytes = 512ull * 1024ull * 1024ull;
    // Keep zero-reference assets as LRU eviction candidates.
    bool cacheUnreferencedAssets = false;
    // Extra bytes evicted beyond the shortfall when making room.
    std::uint64_t cacheEvictionBatchBytes = 64ull * 1024ull * 1024ull;
    // Canonical path length cap.
    std::size_t maximumPathBytes = 4096;
    // Re-hash loaded bytes against the catalog SHA-256 (cooked assets).
    bool verifyCookedAssetHashes = false;
    // Re-hash loaded bytes for loose-file requests too (no catalog digest;
    // when set, a request whose path resolves but whose content cannot be
    // read is still reported as ReadFailed - the hash itself is informational
    // until the caller inspects it via a future debug hook).
    bool verifyLooseAssetHashes = false;
    // Issue posix_fadvise(POSIX_FADV_WILLNEED) before reading (Linux only).
    bool readaheadHint = false;
    // Issue posix_fadvise(POSIX_FADV_DONTNEED) after reading to release page
    // cache pages (Linux only).
    bool dropAfterRead = false;
    // Number of canonical-path resolutions to cache (0 disables). Entries are
    // evicted in FIFO order when the cache fills.
    std::size_t canonicalPathCacheSize = 0;
    // Eviction order for unreferenced finished records.
    EvictionPolicy evictionPolicy = EvictionPolicy::Lru;

    // Optional filesystem root. Relative requests resolve against it and
    // every path is confined to it; if the root does not resolve to an
    // existing directory, all requests are rejected.
    std::filesystem::path rootDirectory;
    bool                  rootConfigured  = false;
    bool                  rootRestricted  = false;
    // Whether requests may address files outside the configured root. With a
    // restricted root this only accepts absolute paths that still resolve
    // inside the root; without a root it rejects absolute paths entirely, so
    // only relative paths (resolved against the process working directory)
    // are served.
    bool allowAbsolutePaths = true;

    // Cooked-asset object directory (set when the catalog loads).
    std::filesystem::path cookedObjectDirectory;
    bool                  cookedCatalogAvailable = false;
};

} // namespace engine::file_asset_store

#endif
