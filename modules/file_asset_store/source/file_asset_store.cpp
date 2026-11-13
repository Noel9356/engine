// file_asset_store module implementation.
//
// Wires the "file_asset_store" capability onto an in-memory cache of whole
// file contents: reference-counted records, byte budgets for resident and
// in-flight data, LRU/FIFO eviction, task-scheduler asynchronous reads, and
// optional cooked-asset catalogs backed by content-addressed objects. The
// cache and file-reading concerns live in asset_cache.cpp and
// asset_loader.cpp; this file holds the configuration parsing, capability
// vtable binding, async job plumbing, and the module entry table.

#include "asset_cache.hpp"
#include "asset_config.hpp"
#include "asset_loader.hpp"
#include "asset_record.hpp"
#include <engine/file_asset_store/capabilities.hpp>
#include <engine/file_asset_store/catalog.hpp>
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/kernel/profiling.hpp>
#include <engine/kernel/sha256.hpp>
#include <engine/kernel/task_scheduler.hpp>
#include <engine/util/hardware_info.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine::file_asset_store
{
namespace
{

// Translates an AssetRequestPriority to the kernel TaskPriority; Default
// resolves to the configured default_async_priority.
TaskPriority resolveTaskPriority(AssetRequestPriority requested, TaskPriority defaultPriority) noexcept
{
    switch (requested)
    {
    case AssetRequestPriority::Critical: return TaskPriority::Critical;
    case AssetRequestPriority::High:     return TaskPriority::High;
    case AssetRequestPriority::Normal:   return TaskPriority::Normal;
    case AssetRequestPriority::Low:      return TaskPriority::Low;
    case AssetRequestPriority::Default:  return defaultPriority;
    }
    return defaultPriority;
}

// Parses a priority string ("critical"/"high"/"normal"/"low") into the
// matching TaskPriority; returns false for unknown values.
bool parsePriority(const std::string &text, TaskPriority &out) noexcept
{
    if (text == "critical")
        out = TaskPriority::Critical;
    else if (text == "high")
        out = TaskPriority::High;
    else if (text == "normal")
        out = TaskPriority::Normal;
    else if (text == "low")
        out = TaskPriority::Low;
    else
        return false;
    return true;
}

// Parses an eviction-policy string ("lru"/"fifo") into the matching enum;
// returns false for unknown values.
bool parseEvictionPolicy(const std::string &text, EvictionPolicy &out) noexcept
{
    if (text == "lru")
        out = EvictionPolicy::Lru;
    else if (text == "fifo")
        out = EvictionPolicy::Fifo;
    else
        return false;
    return true;
}

// ModuleInterface implementation: owns the cache, loader, config, and the
// capability vtable. The capability methods are thin forwarders into the
// cache and loader; the orchestrator (readAsset) coordinates budget
// reservation, file reading, hash verification, and the commit step.
class FileAssetStoreModule final : public ModuleInterface
{
public:
    FileAssetStoreModule()
        : manifest_(makeManifest())
    {
        assignCapabilityPointers();
    }

    const ParticipantManifest &manifest() const override
    {
        return manifest_;
    }

    void construct(ParticipantContext &context) override
    {
        scheduler_ = &context.taskScheduler();
        profiler_  = &context.profiler();

        // Built-in defaults first; each recognized key below overrides its
        // default only when present, correctly typed, and inside its range.
        config_.maximumFileBytes        = 512ull * 1024ull * 1024ull;
        config_.maximumAssets           = 8192;
        config_.maximumPendingRequests  = 8192;
        config_.maximumResidentBytes    = 2ull * 1024ull * 1024ull * 1024ull;
        config_.maximumInflightBytes    = 512ull * 1024ull * 1024ull;
        config_.cacheUnreferencedAssets = false;
        config_.cacheEvictionBatchBytes = 64ull * 1024ull * 1024ull;
        defaultAsyncPriority_           = TaskPriority::Low;

        // auto_config = true: the memory-shaped knobs derive from installed
        // RAM and the matching [file_asset_store] tuning keys have no effect.
        // Structural keys (maximum_path_bytes, cooked_catalog_path,
        // cooked_object_directory, verify_cooked_asset_hashes,
        // root_directory) describe the store's wiring, not the machine, and
        // stay effective in both modes below.
        if (context.configurationBooleanInSection("file_asset_store", "auto_config").value_or(false))
        {
            applyHardwareDerivedMemoryBudgets();
        }
        else
        {
            applyManualConfig(context);
        }
        applyStructuralConfig(context);

        cache_.configure(config_, stopping_, profiler_);
        loader_.configure(config_);
    }

    void start() override
    {
        stopping_.store(false, std::memory_order_release);
    }

    void stop() override
    {
        // Refuse new submissions, drain in-flight jobs, then drop the cache.
        stopping_.store(true, std::memory_order_release);
        {
            std::unique_lock lock(jobsMutex_);
            jobsCondition_.wait(lock, [this] { return pendingJobs_.load(std::memory_order_acquire) == 0; });
        }
        cache_.clear();
    }

    void *capabilityPointerFor(const std::string &name) override
    {
        return name == FileAssetStoreCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "file_asset_store",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.1.0",
            .providedCapabilities = {"file_asset_store"}};
    }

    void assignCapabilityPointers()
    {
        capability_.self = this;
        // Forward every vtable entry to the matching member via a variadic lambda.
#define ENGINE_FILE_ASSET_STORE_BIND(field, method)                                                                    \
    capability_.field = [](void *self, auto... args) -> decltype(auto)                                                 \
    { return static_cast<FileAssetStoreModule *>(self)->method(args...); }
        ENGINE_FILE_ASSET_STORE_BIND(requestAsset, requestAssetImpl);
        ENGINE_FILE_ASSET_STORE_BIND(requestCookedAsset, requestCookedAssetImpl);
        ENGINE_FILE_ASSET_STORE_BIND(retainAsset, retainAssetImpl);
        ENGINE_FILE_ASSET_STORE_BIND(releaseAsset, releaseAssetImpl);
        ENGINE_FILE_ASSET_STORE_BIND(assetInfo, assetInfoImpl);
        ENGINE_FILE_ASSET_STORE_BIND(waitForAsset, waitForAssetImpl);
        ENGINE_FILE_ASSET_STORE_BIND(copyAssetData, copyAssetDataImpl);
        ENGINE_FILE_ASSET_STORE_BIND(reloadAsset, reloadAssetImpl);
        ENGINE_FILE_ASSET_STORE_BIND(copyAssetPath, copyAssetPathImpl);
        ENGINE_FILE_ASSET_STORE_BIND(purgeUnusedAssets, purgeUnusedAssetsImpl);
        ENGINE_FILE_ASSET_STORE_BIND(cacheStatistics, cacheStatisticsImpl);
#undef ENGINE_FILE_ASSET_STORE_BIND
    }

    // --- Configuration helpers ---

    // auto_config = true: derive the cache's memory shape from installed RAM
    // (unknown RAM keeps the built-in budgets, which are the conservative
    // fallback). resident gets ~25% of RAM, in-flight ~12.5%, and the
    // eviction overshoot ~3%; on machines with 8+ GiB the zero-reference
    // cache stays warm so burst reloads re-serve from memory instead of
    // re-reading files (fewer IO stalls on tail frames).
    void applyHardwareDerivedMemoryBudgets()
    {
        constexpr std::uint64_t mib = 1024ull * 1024ull;
        constexpr std::uint64_t gib = 1024ull * mib;
        const std::uint64_t     ram  = util::physicalMemoryBytes();
        if (ram == 0u)
        {
            std::fprintf(stderr,
                "file_asset_store: auto_config is on; installed RAM unknown, keeping built-in budgets; "
                "other tuning keys in [file_asset_store] are ignored\n");
            return;
        }
        config_.maximumResidentBytes    = std::clamp<std::uint64_t>(ram / 4u, 256u * mib, 8192u * mib);
        config_.maximumInflightBytes    = std::clamp<std::uint64_t>(ram / 8u, 128u * mib, 4096u * mib);
        config_.cacheEvictionBatchBytes = std::clamp<std::uint64_t>(ram * 3ull / 100ull, 32u * mib, 512u * mib);
        config_.cacheUnreferencedAssets  = ram >= 8ull * gib;
        std::fprintf(stderr,
            "file_asset_store: auto_config is on; budgets derived from %.1f GiB RAM "
            "(resident=%llu MiB, inflight=%llu MiB, eviction_batch=%llu MiB, cache_unreferenced=%s); "
            "other tuning keys in [file_asset_store] are ignored\n",
            static_cast<double>(ram) / static_cast<double>(gib),
            (unsigned long long)(config_.maximumResidentBytes / mib),
            (unsigned long long)(config_.maximumInflightBytes / mib),
            (unsigned long long)(config_.cacheEvictionBatchBytes / mib),
            config_.cacheUnreferencedAssets ? "on" : "off");
    }

    void applyManualConfig(ParticipantContext &context)
    {
        // Per-file read cap; 1..16384 MiB.
        if (const auto value = context.configurationIntegerInSection("file_asset_store", "maximum_file_size_mb");
            value && *value >= 1 && *value <= 16384)
            config_.maximumFileBytes = static_cast<std::uint64_t>(*value) * 1024ull * 1024ull;
        // Live-record cap; 1..1,000,000.
        if (const auto value = context.configurationIntegerInSection("file_asset_store", "maximum_cached_assets");
            value && *value >= 1 && *value <= 1'000'000)
            config_.maximumAssets = static_cast<std::size_t>(*value);
        // Queued asynchronous read cap; 1..1,000,000.
        if (const auto value = context.configurationIntegerInSection("file_asset_store", "maximum_pending_requests");
            value && *value >= 1 && *value <= 1'000'000)
            config_.maximumPendingRequests = static_cast<std::size_t>(*value);
        // Cap on loaded bytes held in memory; 1..65536 MiB.
        if (const auto value = context.configurationIntegerInSection("file_asset_store", "maximum_resident_memory_mb");
            value && *value >= 1 && *value <= 65536)
            config_.maximumResidentBytes = static_cast<std::uint64_t>(*value) * 1024ull * 1024ull;
        // Cap on bytes reserved by reads in progress; 1..65536 MiB.
        if (const auto value = context.configurationIntegerInSection("file_asset_store", "maximum_inflight_memory_mb");
            value && *value >= 1 && *value <= 65536)
            config_.maximumInflightBytes = static_cast<std::uint64_t>(*value) * 1024ull * 1024ull;
        // Keep zero-reference assets instead of freeing them immediately.
        config_.cacheUnreferencedAssets
            = context.configurationBooleanInSection("file_asset_store", "cache_unreferenced_assets").value_or(false);
        // Extra bytes evicted beyond the shortfall when making room; 0..4096 MiB.
        if (const auto value = context.configurationIntegerInSection("file_asset_store", "cache_eviction_batch_mb");
            value && *value >= 0 && *value <= 4096)
            config_.cacheEvictionBatchBytes = static_cast<std::uint64_t>(*value) * 1024ull * 1024ull;
        // Queue used when a request passes AssetRequestPriority::Default.
        if (const auto value = context.configurationStringInSection("file_asset_store", "default_async_priority"))
        {
            if (!parsePriority(*value, defaultAsyncPriority_))
                std::fprintf(
                    stderr, "file_asset_store: unknown default_async_priority '%s'; using low\n", value->c_str());
        }
        // Eviction policy for unreferenced finished records.
        if (const auto value = context.configurationStringInSection("file_asset_store", "eviction_policy"))
        {
            if (!parseEvictionPolicy(*value, config_.evictionPolicy))
                std::fprintf(stderr, "file_asset_store: unknown eviction_policy '%s'; using lru\n", value->c_str());
        }
        // Linux readahead hint before descriptor reads.
        config_.readaheadHint
            = context.configurationBooleanInSection("file_asset_store", "readahead_hint").value_or(false);
        // Linux page-cache drop after descriptor reads.
        config_.dropAfterRead
            = context.configurationBooleanInSection("file_asset_store", "drop_after_read").value_or(false);
        // Canonical-path cache size; 0..65536 entries.
        if (const auto value = context.configurationIntegerInSection("file_asset_store", "canonical_path_cache_size");
            value && *value >= 0 && *value <= 65536)
            config_.canonicalPathCacheSize = static_cast<std::size_t>(*value);
        // Re-hash loose files too (informational; future debug hook).
        config_.verifyLooseAssetHashes
            = context.configurationBooleanInSection("file_asset_store", "verify_loose_asset_hashes").value_or(false);
    }

    // Structural keys honored in both auto_config and manual modes.
    void applyStructuralConfig(ParticipantContext &context)
    {
        // Canonical path length cap; 64..65536 bytes.
        if (const auto value = context.configurationIntegerInSection("file_asset_store", "maximum_path_bytes");
            value && *value >= 64 && *value <= 65536)
            config_.maximumPathBytes = static_cast<std::size_t>(*value);
        // Cooked assets: load the binary catalog and index it by logical key;
        // objects are read from the catalog's directory unless
        // cooked_object_directory overrides it. A missing or corrupt catalog
        // only disables cooked requests (stderr note), never loose files.
        if (const auto value = context.configurationStringInSection("file_asset_store", "cooked_catalog_path");
            value && !value->empty())
        {
            cookedCatalogPath_ = std::filesystem::path(*value);
            std::string                    catalogError;
            std::vector<AssetCatalogEntry> entries;
            if (readAssetCatalog(cookedCatalogPath_, entries, catalogError))
            {
                for (AssetCatalogEntry &entry : entries)
                {
                    std::string key = entry.logicalPath;
                    cookedCatalog_.emplace(std::move(key), std::move(entry));
                }
                config_.cookedCatalogAvailable  = true;
                config_.cookedObjectDirectory  = cookedCatalogPath_.parent_path();
                if (const auto objectDirectory
                    = context.configurationStringInSection("file_asset_store", "cooked_object_directory");
                    objectDirectory && !objectDirectory->empty())
                    config_.cookedObjectDirectory = *objectDirectory;
            }
            else
            {
                std::fprintf(stderr, "file_asset_store: cooked asset catalog unavailable: %s\n", catalogError.c_str());
            }
        }
        // Re-hash loaded bytes against the catalog digest when set.
        config_.verifyCookedAssetHashes
            = context.configurationBooleanInSection("file_asset_store", "verify_cooked_asset_hashes").value_or(false);
        // Optional filesystem root. Relative requests resolve against it and
        // every path is confined to it; if the root does not resolve to an
        // existing directory, all requests are rejected rather than silently
        // falling back to unrestricted filesystem access.
        config_.allowAbsolutePaths
            = context.configurationBooleanInSection("file_asset_store", "allow_absolute_paths").value_or(true);
        if (const auto value = context.configurationStringInSection("file_asset_store", "root_directory");
            value && !value->empty())
        {
            config_.rootConfigured = true;
            std::error_code error;
            config_.rootDirectory = std::filesystem::weakly_canonical(*value, error);
            config_.rootRestricted
                = !error && !config_.rootDirectory.empty() && std::filesystem::is_directory(config_.rootDirectory, error) && !error;
        }
    }

    // --- Async job accounting ---

    bool reservePendingJob()
    {
        std::lock_guard lock(jobsMutex_);
        const std::size_t pending = pendingJobs_.load(std::memory_order_relaxed);
        if (stopping_.load(std::memory_order_acquire) || scheduler_ == nullptr
            || pending >= config_.maximumPendingRequests)
            return false;
        pendingJobs_.store(pending + 1u, std::memory_order_release);
        return true;
    }

    void completePendingJob()
    {
        {
            std::lock_guard lock(jobsMutex_);
            const std::size_t pending = pendingJobs_.load(std::memory_order_relaxed);
            if (pending != 0)
                pendingJobs_.store(pending - 1u, std::memory_order_release);
        }
        jobsCondition_.notify_all();
    }

    // --- Load orchestrator ---

    // Stats the file, reserves budget, reads bytes, verifies hash, commits.
    // Leaves the record Ready or Failed and reports the reason via *error
    // when given.
    bool readAsset(const std::shared_ptr<AssetRecord> &record, AssetError *error)
    {
        const auto [fileSize, statError] = loader_.stat(record->path);
        if (statError != AssetError::None)
        {
            setAssetState(*record, AssetState::Failed, statError);
            if (error != nullptr)
                *error = statError;
            return false;
        }
        // Cooked assets must still match the catalog's recorded size.
        if (record->expectedSize != std::numeric_limits<std::uint64_t>::max() && fileSize != record->expectedSize)
        {
            setAssetState(*record, AssetState::Failed, AssetError::CookedAssetIntegrityFailed);
            if (error != nullptr)
                *error = AssetError::CookedAssetIntegrityFailed;
            return false;
        }
        if (!cache_.reserveForLoad(record, fileSize, error))
            return false;

        AssetError readError = AssetError::None;
        std::vector<std::uint8_t> loaded = loader_.read(record->path, fileSize, &readError);
        if (readError != AssetError::None)
        {
            cache_.releaseReservation(fileSize);
            setAssetState(*record, AssetState::Failed, readError);
            if (error != nullptr)
                *error = readError;
            return false;
        }
        // Optional integrity check for cooked assets (and loose files when
        // verify_loose_asset_hashes is set, though loose files have no
        // expectedSha256 so the check is a no-op for them).
        if (config_.verifyCookedAssetHashes && !record->expectedSha256.empty())
        {
            if (!loader_.verifyHash(loaded, record->expectedSha256))
            {
                cache_.releaseReservation(fileSize);
                setAssetState(*record, AssetState::Failed, AssetError::CookedAssetIntegrityFailed);
                if (error != nullptr)
                    *error = AssetError::CookedAssetIntegrityFailed;
                return false;
            }
        }
        cache_.commitLoad(record, loaded, fileSize);
        setAssetState(*record, AssetState::Ready, AssetError::None);
        if (error != nullptr)
            *error = AssetError::None;
        return true;
    }

    // --- Capability method implementations ---

    static void setError(AssetError *output, AssetError error)
    {
        if (output != nullptr)
            *output = error;
    }

    AssetHandle requestAssetImpl(const AssetRequest *request, AssetError *error)
    {
        setError(error, AssetError::None);
        if (request == nullptr)
        {
            setError(error, AssetError::InvalidPath);
            return invalidAsset;
        }
        const std::string path = cache_.canonicalPath(request->filePath);
        if (path.empty())
        {
            setError(error, AssetError::InvalidPath);
            return invalidAsset;
        }
        return requestResolvedAssetImpl(request, std::filesystem::path(path), path, error);
    }

    // Cooked requests map the logical key to a catalog entry and load its
    // content-addressed object; the "cooked://" identity keeps these entries
    // separate from loose-file paths in the shared path index.
    AssetHandle requestCookedAssetImpl(const AssetRequest *request, AssetError *error)
    {
        setError(error, AssetError::None);
        if (request == nullptr || request->filePath == nullptr)
        {
            setError(error, AssetError::InvalidPath);
            return invalidAsset;
        }
        if (!config_.cookedCatalogAvailable)
        {
            setError(error, AssetError::CatalogUnavailable);
            return invalidAsset;
        }
        const auto *pathTerminator
            = static_cast<const char *>(std::memchr(request->filePath, '\0', config_.maximumPathBytes + 1u));
        if (pathTerminator == nullptr || pathTerminator == request->filePath)
        {
            setError(error, AssetError::InvalidPath);
            return invalidAsset;
        }
        const std::string input(request->filePath, static_cast<std::size_t>(pathTerminator - request->filePath));
        std::string       key;
        if (!normalizeAssetKey(input, key) || key.size() > config_.maximumPathBytes)
        {
            setError(error, AssetError::InvalidPath);
            return invalidAsset;
        }
        const auto found = cookedCatalog_.find(key);
        if (found == cookedCatalog_.end())
        {
            setError(error, AssetError::CookedAssetNotFound);
            return invalidAsset;
        }
        const std::string identity = "cooked://" + key;
        return requestResolvedAssetImpl(
            request, assetObjectPath(config_.cookedObjectDirectory, found->second.sha256), identity, error, &found->second);
    }

    // Shared request path: dedupe by identity string (canonical path or
    // "cooked://key"), make room at the record cap, then either schedule an
    // asynchronous read or run it inline. A handle is returned even when the
    // async submission fails with Busy (the record stays as Failed/Busy).
    AssetHandle requestResolvedAssetImpl(const AssetRequest *request, const std::filesystem::path &resolvedPath,
        const std::string &identity, AssetError *error, const AssetCatalogEntry *cookedEntry = nullptr)
    {
        // Existing asset for this identity: hand the same record out
        // again with an extra reference instead of loading twice.
        AssetError findError = AssetError::None;
        const AssetHandle existing = cache_.findByIdentity(identity, findError);
        if (existing != invalidAsset)
            return existing;
        if (findError != AssetError::None)
        {
            setError(error, findError);
            return invalidAsset;
        }
        AssetError createError = AssetError::None;
        const AssetHandle handle = cache_.createRecord(resolvedPath, identity, cookedEntry, createError);
        if (handle == invalidAsset)
        {
            setError(error, createError);
            return invalidAsset;
        }
        // Fetch the record for the load. The cache guarantees it exists.
        std::shared_ptr<AssetRecord> record = cache_.find(handle);
        if (!record)
        {
            setError(error, AssetError::InvalidHandle);
            return invalidAsset;
        }
        if (request->asynchronous)
        {
            const TaskPriority taskPriority = resolveTaskPriority(request->priority, defaultAsyncPriority_);
            if (!reservePendingJob())
            {
                setAssetState(*record, AssetState::Failed, AssetError::Busy);
                setError(error, AssetError::Busy);
                return handle;
            }
            bool accepted = false;
            try
            {
                accepted = scheduler_->schedule(
                    [this, record]
                    {
                        setAssetState(*record, AssetState::Loading, AssetError::None);
                        try
                        {
                            ScopedProfileTimer timer(*profiler_, PerformanceMetric::AssetRead);
                            readAsset(record, nullptr);
                        }
                        catch (...)
                        {
                            setAssetState(*record, AssetState::Failed, AssetError::ReadFailed);
                        }
                        completePendingJob();
                    },
                    taskPriority);
            }
            catch (...)
            {
                accepted = false;
            }
            if (!accepted)
            {
                completePendingJob();
                setAssetState(*record, AssetState::Failed, AssetError::Busy);
                setError(error, AssetError::Busy);
            }
        }
        else
        {
            setAssetState(*record, AssetState::Loading, AssetError::None);
            readAsset(record, error);
        }
        return handle;
    }

    bool retainAssetImpl(AssetHandle handle)
    {
        return cache_.retain(handle);
    }

    void releaseAssetImpl(AssetHandle handle)
    {
        cache_.release(handle);
    }

    // Snapshot of the record; canonicalPath points into the record's string
    // and is invalidated when the record is evicted or the module stops.
    bool assetInfoImpl(AssetHandle handle, AssetInfo *output)
    {
        auto record = cache_.find(handle);
        if (!record || output == nullptr)
            return false;
        cache_.touch(record);
        output->state          = record->state.load(std::memory_order_acquire);
        output->error          = record->error.load(std::memory_order_acquire);
        output->referenceCount = record->references.load(std::memory_order_acquire);
        output->canonicalPath  = record->canonicalPath.c_str();
        {
            std::lock_guard lock(record->dataMutex);
            output->byteCount      = record->bytes.size();
            output->contentVersion = record->version;
        }
        return true;
    }

    // Sleeps on the record's condition variable until Ready or Failed;
    // returns true only when the asset reached Ready.
    bool waitForAssetImpl(AssetHandle handle, std::uint32_t timeoutMilliseconds)
    {
        auto record = cache_.find(handle);
        if (!record)
            return false;
        std::unique_lock lock(record->stateMutex);
        const bool       completed = record->stateCondition.wait_for(lock,
            std::chrono::milliseconds(timeoutMilliseconds),
            [&record]
            {
                const AssetState state = record->state.load(std::memory_order_acquire);
                return state == AssetState::Ready || state == AssetState::Failed;
            });
        return completed && record->state.load(std::memory_order_acquire) == AssetState::Ready;
    }

    // Always reports the required size; copies only when the caller's buffer
    // is large enough and the asset is Ready. State is re-checked under
    // dataMutex to guarantee the bytes and state are consistent at the time
    // of the copy (a concurrent reload could otherwise swap bytes between
    // the unlocked state check and the dataMutex acquisition).
    bool copyAssetDataImpl(AssetHandle handle, void *output, std::uint64_t capacity, std::uint64_t *outRequired)
    {
        auto record = cache_.find(handle);
        if (!record)
            return false;
        cache_.touch(record);
        std::lock_guard lock(record->dataMutex);
        // Re-check state under the lock so we never copy bytes from a record
        // that has transitioned out of Ready (e.g., a reload that failed).
        if (record->state.load(std::memory_order_acquire) != AssetState::Ready)
            return false;
        if (outRequired != nullptr)
            *outRequired = record->bytes.size();
        if (capacity < record->bytes.size() || (!record->bytes.empty() && output == nullptr))
            return false;
        if (!record->bytes.empty())
            std::memcpy(output, record->bytes.data(), record->bytes.size());
        return true;
    }

    // Rejects assets still queued or loading (Busy); otherwise runs a
    // synchronous read through the normal budget checks.
    bool reloadAssetImpl(AssetHandle handle, AssetError *error)
    {
        auto record = cache_.find(handle);
        if (!record)
        {
            setError(error, AssetError::InvalidHandle);
            return false;
        }
        {
            std::lock_guard  lock(record->stateMutex);
            const AssetState currentState = record->state.load(std::memory_order_acquire);
            if (currentState == AssetState::Queued || currentState == AssetState::Loading)
            {
                setError(error, AssetError::Busy);
                return false;
            }
            record->error.store(AssetError::None, std::memory_order_release);
            record->state.store(AssetState::Loading, std::memory_order_release);
        }
        record->stateCondition.notify_all();
        return readAsset(record, error);
    }

    // outRequired includes the terminating NUL.
    bool copyAssetPathImpl(AssetHandle handle, char *output, std::uint32_t capacity, std::uint32_t *outRequired)
    {
        const auto record = cache_.find(handle);
        if (!record)
            return false;
        cache_.touch(record);
        if (record->canonicalPath.size() >= UINT32_MAX)
            return false;
        const std::uint32_t required = static_cast<std::uint32_t>(record->canonicalPath.size() + 1u);
        if (outRequired != nullptr)
            *outRequired = required;
        if (output == nullptr || capacity < required)
            return false;
        std::memcpy(output, record->canonicalPath.c_str(), required);
        return true;
    }

    // targetBytesToFree == 0 purges every unreferenced finished asset.
    std::uint64_t purgeUnusedAssetsImpl(std::uint64_t targetBytesToFree)
    {
        return cache_.purgeUnused(targetBytesToFree, targetBytesToFree == 0);
    }

    bool cacheStatisticsImpl(AssetCacheStatistics *output)
    {
        if (output == nullptr)
            return false;
        return cache_.statistics(*output);
    }

    // --- Members ---

    ParticipantManifest             manifest_;
    FileAssetStoreCapability        capability_{};
    AssetStoreConfig                config_{};
    AssetCache                      cache_{};
    AssetLoader                     loader_{};
    TaskScheduler                  *scheduler_ = nullptr;
    PerformanceProfiler             *profiler_  = nullptr;
    TaskPriority                    defaultAsyncPriority_ = TaskPriority::Low;
    std::filesystem::path           cookedCatalogPath_;
    std::unordered_map<std::string, AssetCatalogEntry> cookedCatalog_;
    // Async job accounting: stop() sleeps on jobsCondition_ until the count
    // drains; submissions are refused once stopping_ is set.
    std::mutex               jobsMutex_;
    std::condition_variable  jobsCondition_;
    std::atomic<std::size_t> pendingJobs_{0};
    std::atomic<bool>        stopping_{false};
};

} // namespace
} // namespace engine::file_asset_store

// Module entry table: kernel ABI, module identity, and the provided
// capability list. No required, optional, or permission declarations.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const                    provided[] = {"file_asset_store"};
    static const engine::EngineModuleEntryTable entry
        = {.abiVersion     = engine::engineModuleAbiVersion,
            .moduleName    = "file_asset_store",
            .moduleVersion = "0.1.0",
            .createModule  = []() -> engine::ModuleInterface *
            { return new engine::file_asset_store::FileAssetStoreModule(); },
            .providedCapabilities    = provided,
            .providedCapabilityCount = 1,
            .requiredCapabilities    = nullptr,
            .requiredCapabilityCount = 0,
            .optionalCapabilities    = nullptr,
            .optionalCapabilityCount = 0,
            .declaredPermissions     = nullptr,
            .declaredPermissionCount = 0};
    return &entry;
}
