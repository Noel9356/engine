# file_asset_store configuration

All keys live in the `[file_asset_store]` section. The module's `config.toml` supplies the defaults below; a runtime profile may override them (see `config/profiles/*.toml` in the engine tree). `module.toml` mirrors every key under `config_keys` for documentation; the kernel validates known keys for engine sections and game-mod manifests only. Keys are listed alphabetically within each table.

## Auto config

| Key | Type | Valid range | Default | Effect |
| --- | --- | --- | --- | --- |
| `file_asset_store.auto_config` | boolean | (boolean) | `false` | Hardware-derived memory shape. When `true`, the cache budgets derive from installed RAM - `file_asset_store.maximum_resident_memory_mb` = 25% clamped to 256..8192, `file_asset_store.maximum_inflight_memory_mb` = 12.5% clamped to 128..4096, `file_asset_store.cache_eviction_batch_mb` = 3% clamped to 32..512, and `file_asset_store.cache_unreferenced_assets` turns on from 8 GiB so burst reloads re-serve from memory instead of re-reading files - and the other tuning keys in `[file_asset_store]` have no effect. The structural keys (`file_asset_store.maximum_path_bytes`, `file_asset_store.root_directory`, `file_asset_store.cooked_catalog_path`, `file_asset_store.cooked_object_directory`, `file_asset_store.verify_cooked_asset_hashes`) describe the store's wiring, not the machine, and stay effective. Unknown RAM keeps the built-in budgets. A one-line stderr notice reports the derived budgets. |

## Manual keys

| Key | Type | Valid range (as validated in `construct()`) | Default (`config.toml`) | Effect |
| --- | --- | --- | --- | --- |
| `file_asset_store.cache_eviction_batch_mb` | integer | 0 - 4096 | `64` | Extra bytes (MiB) evicted beyond the shortfall when a load needs to make resident space. |
| `file_asset_store.cache_unreferenced_assets` | boolean | (boolean) | `false` | Keep zero-reference assets as LRU eviction candidates instead of freeing them on the last `releaseAsset`. |
| `file_asset_store.cooked_catalog_path` | string | non-empty | `""` | Path of the binary cooked catalog to load at construct time; enables `requestCookedAsset`. |
| `file_asset_store.cooked_object_directory` | string | non-empty | `""` | Overrides the directory cooked objects are read from; defaults to the catalog's parent directory. Only consulted when the catalog loads successfully. |
| `file_asset_store.default_async_priority` | string | `critical`, `high`, `normal`, `low` | `"low"` | Task queue used when a request passes `AssetRequestPriority::Default`. |
| `file_asset_store.maximum_cached_assets` | integer | 1 - 1000000 | `8192` | Maximum number of live asset records; when full, one unreferenced asset is evicted before failing with `CapacityExceeded`. |
| `file_asset_store.maximum_file_size_mb` | integer | 1 - 16384 | `512` | Per-file read cap in MiB; larger files fail with `FileTooLarge`. |
| `file_asset_store.maximum_inflight_memory_mb` | integer | 1 - 65536 | `512` | Cap on bytes reserved by reads in progress (MiB); a load larger than the free in-flight budget fails with `CapacityExceeded`. |
| `file_asset_store.maximum_path_bytes` | integer | 64 - 65536 | `4096` | Maximum canonical path length in bytes; longer paths fail with `InvalidPath`. |
| `file_asset_store.maximum_pending_requests` | integer | 1 - 1000000 | `8192` | Cap on queued asynchronous read jobs; further async requests fail with `Busy`. |
| `file_asset_store.maximum_resident_memory_mb` | integer | 1 - 65536 | `2048` | Cap on loaded bytes held in memory (MiB); eviction runs when a load would exceed it. |
| `file_asset_store.root_directory` | string | non-empty | `""` | Filesystem root: relative requests resolve against it, all paths are confined to it. Empty disables restriction. |
| `file_asset_store.verify_cooked_asset_hashes` | boolean | (boolean) | `false` | Re-hash loaded bytes against the catalog SHA-256 and fail with `CookedAssetIntegrityFailed` on mismatch. |

## Invalid and out-of-range values

`construct()` reads each key through `ParticipantContext::configuration*InSection` and applies it only when the value is present, has the expected TOML type, and passes the range check above. A missing, mistyped, or out-of-range value is silently ignored and the built-in default (identical to the `config.toml` value) remains in effect; no error is raised to the caller. Two cases are reported rather than silent:

- `file_asset_store.default_async_priority` with an unknown string prints a note to stderr and keeps `low`.
- `file_asset_store.cooked_catalog_path` set but unreadable/corrupt prints the catalog error to stderr and leaves cooked requests disabled (error `CatalogUnavailable`); loose-file requests continue to work.

Note that configuring a `file_asset_store.root_directory` that does not resolve to an existing directory is stricter: it disables all path requests instead of falling back to unrestricted access.

## Budget model

The three memory keys form the eviction policy: resident memory is the cache high-water mark, in-flight memory bounds concurrent reads before they land in the cache, and the eviction batch controls how far past the shortfall each eviction round goes (batching amortizes the LRU walk at the cost of overshooting). `cache_unreferenced_assets` decides whether the LRU list has candidates at all. With `auto_config` on, all four derive from installed RAM and the manual keys are ignored until the toggle goes back to `false`. See [overview.md](overview.md) for the request path these budgets sit under.
