// Public capability surface of the file_asset_store module.
//
// Consumers obtain this C struct via ParticipantContext::acquireCapability and
// use it to request files into a bounded, reference-counted byte cache, wait
// for and copy their contents, and manage cache memory.

#ifndef ENGINE_FILE_ASSET_STORE_CAPABILITIES_HPP
#define ENGINE_FILE_ASSET_STORE_CAPABILITIES_HPP

#include <cstdint>

namespace engine::file_asset_store
{

// ABI version consumers must match before calling any entry point.
inline constexpr std::uint32_t fileAssetStoreCapabilityAbi = 5u;
// Opaque asset id; 0 (invalidAsset) means "none" and is never handed out.
using AssetHandle                         = std::uint64_t;
inline constexpr AssetHandle invalidAsset = 0;

// Lifecycle of one asset: Queued -> Loading -> Ready or Failed. A Failed
// asset stays terminal until reloadAsset succeeds or the record is dropped.
enum class AssetState : std::uint32_t
{
    Queued = 0,
    Loading,
    Ready,
    Failed
};

// Failure reasons reported through out-parameters; None marks success.
// CapacityExceeded covers both record-count and byte-budget limits; Busy
// means the request could not be scheduled or a load is still in flight.
enum class AssetError : std::uint32_t
{
    None = 0,
    InvalidHandle,
    InvalidPath,
    FileNotFound,
    FileTooLarge,
    ReadFailed,
    CapacityExceeded,
    Busy,
    CookedAssetNotFound,
    CatalogUnavailable,
    CookedAssetIntegrityFailed
};

// Scheduler queue hint for asynchronous loads. Default resolves to the
// module's configured default_async_priority; the other values map 1:1 to
// task priorities. Note the list order is not a ranking.
enum class AssetRequestPriority : std::uint8_t
{
    Default = 0,
    Critical,
    High,
    Normal,
    Low
};

// One load request. filePath is UTF-8 and may be relative when a root
// directory is configured; it is copied before the call returns.
struct AssetRequest
{
    const char          *filePath     = nullptr;
    bool                 asynchronous = true; // false: read completes before the call returns
    AssetRequestPriority priority     = AssetRequestPriority::Default;
};

// Point-in-time view of one asset. canonicalPath points into module-owned
// storage and is only valid until the record is evicted or the module stops;
// contentVersion increments on every successful (re)load of the record.
struct AssetInfo
{
    AssetState    state          = AssetState::Queued;
    AssetError    error          = AssetError::None;
    std::uint64_t byteCount      = 0;
    std::uint64_t contentVersion = 0;
    std::uint32_t referenceCount = 0;
    const char   *canonicalPath  = nullptr;
};

// Cache-wide counters; byte fields are bytes, count fields are assets.
// evictableBytes is the subset of residentBytes held by unreferenced,
// finished (Ready/Failed) records.
struct AssetCacheStatistics
{
    std::uint64_t residentBytes          = 0;
    std::uint64_t reservedBytes          = 0;
    std::uint64_t evictableBytes         = 0;
    std::uint64_t assetCount             = 0;
    std::uint64_t unreferencedAssetCount = 0;
};

// Capability vtable handed to consumers. Every function takes `self` (set by
// the module) as its first argument; caller-provided buffers stay
// caller-owned, and calls are safe from any thread.
struct FileAssetStoreCapability
{
    static constexpr const char *capabilityName = "file_asset_store";
    std::uint32_t                abi            = fileAssetStoreCapabilityAbi;
    void                        *self           = nullptr;

    // Resolves the path against the configured root, dedupes by canonical
    // path (an existing asset gains a reference), and starts the load.
    // Returns invalidAsset with *error set on rejection.
    AssetHandle (*requestAsset)(void *, const AssetRequest *, AssetError *);
    // Same contract, but filePath is a cooked-catalog logical key: the record
    // is pinned to the catalog's expected size/digest and read from the
    // object store. Fails with CatalogUnavailable when no catalog is loaded.
    AssetHandle (*requestCookedAsset)(void *, const AssetRequest *, AssetError *);
    // Reference counting: new records start with one reference held by the
    // requester. Releasing the last reference frees the record unless the
    // cache_unreferenced_assets option keeps it as an eviction candidate.
    bool (*retainAsset)(void *, AssetHandle);
    void (*releaseAsset)(void *, AssetHandle);
    bool (*assetInfo)(void *, AssetHandle, AssetInfo *);
    // Blocks up to timeoutMilliseconds for the asset to finish; true only
    // when it reached Ready (false on timeout or Failure).
    bool (*waitForAsset)(void *, AssetHandle, std::uint32_t timeoutMilliseconds);
    // Copies the loaded bytes; always reports the needed size via outRequired
    // and fails (without writing) when capacity is too small. Requires Ready.
    bool (*copyAssetData)(void *, AssetHandle, void *output, std::uint64_t capacity, std::uint64_t *outRequired);
    // Synchronously re-reads the file through the normal budget checks;
    // fails with Busy while the asset is still queued or loading.
    bool (*reloadAsset)(void *, AssetHandle, AssetError *);
    // Copies the canonical identity path; outRequired includes the NUL.
    bool (*copyAssetPath)(void *, AssetHandle, char *output, std::uint32_t capacity, std::uint32_t *outRequired);

    // Evicts unreferenced finished assets LRU-first until the byte target is
    // freed; targetBytesToFree == 0 purges all of them. Returns bytes freed.
    std::uint64_t (*purgeUnusedAssets)(void *, std::uint64_t targetBytesToFree);
    bool (*cacheStatistics)(void *, AssetCacheStatistics *outStatistics);
};

} // namespace engine::file_asset_store

#endif
