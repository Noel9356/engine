# file_asset_store module

file_asset_store gives the engine a bounded, reference-counted cache of whole file contents. Consumers hand it paths (or cooked-catalog keys), get opaque handles back, and can wait for, copy, reload, and evict the cached bytes under explicit memory budgets. Asynchronous reads run on the kernel task scheduler; a companion content-addressed object store plus a strict binary catalog lets builds serve verified, deduplicated "cooked" assets instead of loose files.

## Capability

The module provides the `file_asset_store` capability, ABI constant `fileAssetStoreCapabilityAbi = 5`, declared in `modules/file_asset_store/include/engine/file_asset_store/capabilities.hpp`. A second public header, `modules/file_asset_store/include/engine/file_asset_store/catalog.hpp`, exposes the catalog/cook primitives (built as the static `engine_asset_catalog` library) for offline tooling; the module .so itself is `engine_module_file_asset_store`.

Key capability facts:

- Handles are `std::uint64_t`; `invalidAsset` (`0`) is never handed out and handles stay unique for the module's lifetime (the counter skips 0 on wrap).
- Every function takes the `self` pointer first; callers own all buffers.
- A request for an already-canonicalized path returns the existing handle with one extra reference instead of loading the file twice.
- An asynchronous request whose job cannot be scheduled (queue full, module stopping) still returns a valid handle; the record is left in state `Failed` with error `Busy`, so callers must check `assetInfo`/`waitForAsset`.
- `purgeUnusedAssets(0)` means "purge every unreferenced finished asset"; any other value is a byte target.

## Provided / required / optional capabilities

From `module.toml`:

| Field | Value |
| --- | --- |
| provides | `file_asset_store` |
| requires | (none) |

From the entry table at the bottom of `source/file_asset_store.cpp` (`engineQueryModuleEntryTable`, kernel ABI `engineModuleAbiVersion = 8`):

| Field | Value |
| --- | --- |
| moduleName / version | `file_asset_store` / `0.1.0` |
| providedCapabilities | `file_asset_store` (1) |
| requiredCapabilities | none (0) |
| optionalCapabilities | none (0) |
| declaredPermissions | none (0) |

## Architecture

| File | Role |
| --- | --- |
| `module.toml` | On-disk descriptor: name, version, provided capability, and the declared `config_keys` documenting the configuration surface. Copied next to the built .so by CMake. |
| `config.toml` | Default values for the `[file_asset_store]` section (see `configuration.md`). |
| `include/engine/file_asset_store/capabilities.hpp` | Public capability types: ABI constant, handle/state/error enums, request/info/statistics structs, and the function-pointer vtable. |
| `include/engine/file_asset_store/catalog.hpp` | Catalog/cook API shared with offline tooling: key normalization, binary catalog read/write, object paths, directory cook. |
| `source/asset_record.hpp` | The `AssetRecord` struct shared by the cache and loader: identity, atomic state/error/references, LRU stamp, byte buffer, and the `dataMutex`/`stateMutex` pair that guards them. |
| `source/asset_config.hpp` | Parsed configuration values (`AssetStoreConfig`) shared between the cache, loader, and module. |
| `source/asset_cache.hpp` / `source/asset_cache.cpp` | `AssetCache`: owns the record map and the identity-to-handle path index, manages the resident and in-flight byte budgets, runs LRU/FIFO eviction, canonicalizes request paths (with an optional FIFO cache), and exposes budget hooks (`reserveForLoad`, `commitLoad`, `releaseReservation`) that the module's load orchestrator calls. |
| `source/asset_loader.hpp` / `source/asset_loader.cpp` | `AssetLoader`: stats and reads files into a freshly allocated buffer. On Linux, reads through an opened descriptor with `O_NOFOLLOW` (defensive against symlink replacement after canonicalization) and optional `posix_fadvise` hints (`POSIX_FADV_WILLNEED` before, `POSIX_FADV_DONTNEED` after). On other platforms, uses `std::ifstream`. Also provides SHA-256 verification. |
| `source/asset_catalog.cpp` | Implementation of `catalog.hpp`: little-endian catalog format, canonical key rules, content-addressed object installation, cook walk. Built as the static library `engine_asset_catalog` and linked into the module. Object and catalog installs `fdatasync` the temp file before rename so a crash does not install partially written data. |
| `source/file_asset_store.cpp` | The module: `FileAssetStoreModule` parses configuration, binds the capability vtable, manages async job submission, and orchestrates the load path (stat → reserve → read → verify → commit). Contains the module entry table. |

## How it works

### Request path and identity

1. `requestAsset` canonicalizes the requested path (`AssetCache::canonicalPath`): `weakly_canonical` + `lexically_normal`, resolving relative paths against the configured `root_directory` when one is set. Any violation (empty, too long, escaping the root, unusable root) yields an empty string and the request fails with `InvalidPath`. When a root is configured but does not resolve to a real directory, every request is rejected. When `canonical_path_cache_size` is non-zero, recently resolved inputs are cached so repeated requests for the same path skip the `weakly_canonical` syscall.
2. The canonical path (or `"cooked://<logical key>"` for cooked requests) is the record identity. `pathIndex_` maps identity to handle, so repeat requests return the same record with one extra reference.
3. If the record cache is at `maximum_cached_assets`, one unreferenced asset is evicted to make room; if that fails, the request reports `CapacityExceeded`.

### Loading and budgets

`readAsset` (in `file_asset_store.cpp`) is the single load orchestrator, used by the synchronous path, the scheduler job, and `reloadAsset`:

1. `AssetLoader::stat` stats the file; reject missing files (`FileNotFound`) and files above `maximum_file_bytes` (`FileTooLarge`).
2. Cooked records additionally must match the catalog's expected size, else `CookedAssetIntegrityFailed`.
3. `AssetCache::reserveForLoad` reserves the size against the in-flight budget and then the resident budget under `mutex_`. If resident space is short, LRU/FIFO eviction runs with a target of the shortfall plus `cache_eviction_batch_mb`; if the size still does not fit, the load fails with `CapacityExceeded`. Reserved bytes move to `residentBytes_` on success (`commitLoad`).
4. `AssetLoader::read` reads the bytes from the opened file: on Linux through a descriptor-based read loop (with `O_NOFOLLOW` and optional `posix_fadvise` hints), elsewhere via `std::ifstream`. Both paths verify that the number of bytes read matches the initial file size; empty files are valid.
5. If `verify_cooked_asset_hashes` is set and the record carries a catalog digest, the SHA-256 of the loaded bytes is checked.
6. `AssetCache::commitLoad` swaps the buffer in under `dataMutex` and bumps the record's `contentVersion`, so consumers can detect reloads.

Asynchronous requests reserve a slot under the submission/shutdown mutex (capped by `maximum_pending_requests`) and are submitted to the `ParticipantContext::taskScheduler()` at the request's priority (`AssetRequestPriority::Default` resolves to the configured `default_async_priority`). The job sets `Loading`, runs `readAsset`, then releases the slot and wakes `stop()`.

### Eviction and lifetime

- Eviction candidates are records with zero references that are `Ready` or `Failed`; they are ordered by `lastAccessTick` (LRU) or by handle (FIFO), then re-validated under the lock before removal. Both orders use a min-heap so a partial eviction that meets its target after a few removals never sorts the rest of the cache.
- References are counted with a 32-bit unsigned value; retain operations stop at the maximum value. Releasing a record with no outstanding references has no effect.
- Releasing the last reference deletes the record immediately, unless `cache_unreferenced_assets` keeps it as an eviction candidate.
- `stop()` sets the stopping flag (new submissions fail with `Busy`), drains `pendingJobs_` to zero, then clears the cache.

### Cooked assets and the catalog

`construct()` reads the binary catalog at `cooked_catalog_path` (if set) into an in-memory map from logical key to `{sha256, sizeBytes}`. `requestCookedAsset` normalizes the requested key (see `normalizeAssetKey`: backslashes become slashes, `.`/`..`/absolute/`:`/NUL segments are rejected), looks it up, and loads the object at `<object directory>/<first 2 digest chars>/<digest>`. The object directory defaults to the catalog's parent directory and can be overridden with `cooked_object_directory`. A missing or corrupt catalog only disables cooked requests (stderr note); loose-file requests are unaffected.

The catalog format itself (`asset_catalog.cpp`) is little-endian: 8-byte magic `ENGINEAC`, u32 version (1), u32 entry count, then per entry u32 path size, u64 size, 32 raw SHA-256 bytes, and the key bytes. Readers enforce the magic, version, count, per-entry bounds, canonical and unique keys, and absence of trailing data; writers sort entries by key and install the file via a unique temporary name plus `fdatasync` + rename. `cookAssetDirectory` walks a source tree (skipping symlinks), hashes every regular file, installs each object once (content-addressed dedup, size+hash re-verified after copying, `fdatasync` before rename), and writes the catalog, refusing to place objects or catalog inside the source tree.

## Threading and lifetime

- Capability calls are safe from any thread. `AssetCache::mutex_` guards `records_`, `pathIndex_`, `residentBytes_`, `reservedBytes_`, and the handle counter; each record has a `dataMutex` for its byte buffer and a `stateMutex` + `stateCondition` pair that `waitForAsset` sleeps on.
- Reads run either inline on the caller's thread or on task-scheduler worker threads; the scheduler pointer is captured in `construct()`.
- `AssetInfo::canonicalPath` points into the record's string and is only valid until that record is evicted, purged, or the module stops.
- `copyAssetData` re-checks the record's state under `dataMutex` before copying, guaranteeing the bytes and state are consistent at the time of the copy (a concurrent reload could otherwise swap bytes between an unlocked state check and the data lock).
- Records live until the last reference is released (or `stop()` clears the cache). The module instance itself is created by the kernel's entry table and destroyed by the kernel after `stop()`.
- The catalog/cook helpers in `asset_catalog.cpp` share no mutable state except an atomic temporary-name counter, so concurrent use against different destinations is safe; the final rename makes each file install atomic.

## Configuration

Default values and per-key validation ranges live in `config.toml`; every key is documented in [configuration.md](configuration.md). The module also works with no configuration at all: the built-in defaults equal the `config.toml` values.

## Integration

- Events: none. The module never touches the kernel `EventBus`; it reports progress purely through asset state and the optional `PerformanceMetric::AssetRead` profiler timer around scheduled reads, plus `PerformanceMetric::AssetEviction` around eviction passes.
- Permissions: none declared (`declaredPermissions` is empty in the entry table and `module.toml` declares no permissions).
- Filesystem access is confined to the configured `root_directory` when one is set, and to the cooked object directory for cooked requests. On Linux, file reads use `O_NOFOLLOW` on the final path component as a defensive measure against symlink replacement after canonicalization.

Consumer usage (all signatures taken from `capabilities.hpp`):

```cpp
#include <engine/file_asset_store/capabilities.hpp>

// During a participant's construct phase:
auto& store = context.acquireCapability<engine::file_asset_store::FileAssetStoreCapability>();

engine::file_asset_store::AssetRequest request{};
request.filePath     = "levels/level01.bin"; // relative when root_directory is set
request.asynchronous = true;
request.priority     = engine::file_asset_store::AssetRequestPriority::Normal;

engine::file_asset_store::AssetError error = engine::file_asset_store::AssetError::None;
const engine::file_asset_store::AssetHandle handle = store.requestAsset(store.self, &request, &error);
if (handle == engine::file_asset_store::invalidAsset)
    return; // inspect `error`

if (!store.waitForAsset(store.self, handle, 5000))
    return; // timed out or failed; check assetInfo().state/error

engine::file_asset_store::AssetInfo info{};
store.assetInfo(store.self, handle, &info);

std::vector<std::uint8_t> bytes(info.byteCount);
std::uint64_t required = 0;
if (store.copyAssetData(store.self, handle, bytes.data(), bytes.size(), &required))
{
    // use bytes...
    store.releaseAsset(store.self, handle); // drop our reference
}
```
