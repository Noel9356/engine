# file_asset_store configuration

All keys live in the `[file_asset_store]` section. The module's `config.toml` supplies the defaults below; a runtime profile may override them (see `config/profiles/*.toml` in the engine tree). `module.toml` mirrors every key under `config_keys` for documentation; the kernel validates known keys for engine sections and game-mod manifests only.

| Key | Type | Valid range (as validated in `construct()`) | Default (`config.toml`) | Effect |
| --- | --- | --- | --- | --- |
| `maximum_file_size_mb` | integer | 1..16384 | 512 | Per-file read cap in MiB; larger files fail with `FileTooLarge`. |
| `maximum_cached_assets` | integer | 1..1000000 | 8192 | Maximum number of live asset records; when full, one unreferenced asset is evicted before failing with `CapacityExceeded`. |
| `maximum_pending_requests` | integer | 1..1000000 | 8192 | Cap on queued asynchronous read jobs; further async requests fail with `Busy`. |
| `maximum_resident_memory_mb` | integer | 1..65536 | 2048 | Cap on loaded bytes held in memory (MiB); eviction runs when a load would exceed it. |
| `maximum_inflight_memory_mb` | integer | 1..65536 | 512 | Cap on bytes reserved by reads in progress (MiB); a load larger than the free in-flight budget fails with `CapacityExceeded`. |
| `maximum_path_bytes` | integer | 64..65536 | 4096 | Maximum canonical path length in bytes; longer paths fail with `InvalidPath`. |
| `root_directory` | string | non-empty | `""` | Filesystem root: relative requests resolve against it, all paths are confined to it. Empty disables restriction. |
| `cooked_catalog_path` | string | non-empty | `""` | Path of the binary cooked catalog to load at construct time; enables `requestCookedAsset`. |
| `cooked_object_directory` | string | non-empty | `""` | Overrides the directory cooked objects are read from; defaults to the catalog's parent directory. Only consulted when the catalog loads successfully. |
| `verify_cooked_asset_hashes` | boolean | (boolean) | `false` | Re-hash loaded bytes against the catalog SHA-256 and fail with `CookedAssetIntegrityFailed` on mismatch. |
| `default_async_priority` | string | `critical`, `high`, `normal`, `low` | `"low"` | Task queue used when a request passes `AssetRequestPriority::Default`. |
| `cache_unreferenced_assets` | boolean | (boolean) | `false` | Keep zero-reference assets as LRU eviction candidates instead of freeing them on the last `releaseAsset`. |
| `cache_eviction_batch_mb` | integer | 0..4096 | 64 | Extra bytes (MiB) evicted beyond the shortfall when a load needs to make resident space. |

## Invalid and out-of-range values

`construct()` reads each key through `ParticipantContext::configuration*InSection` and applies it only when the value is present, has the expected TOML type, and passes the range check above. A missing, mistyped, or out-of-range value is silently ignored and the built-in default (identical to the `config.toml` value) remains in effect; no error is raised to the caller. Two cases are reported rather than silent:

- `default_async_priority` with an unknown string prints a note to stderr and keeps `low`.
- `cooked_catalog_path` set but unreadable/corrupt prints the catalog error to stderr and leaves cooked requests disabled (error `CatalogUnavailable`); loose-file requests continue to work.

Note that configuring a `root_directory` that does not resolve to an existing directory is stricter: it disables all path requests instead of falling back to unrestricted access.
