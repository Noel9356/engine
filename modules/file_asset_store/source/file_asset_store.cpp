// file_asset_store module implementation.
//
// Binds the "file_asset_store" capability onto an in-memory cache of whole
// file contents: reference-counted records, byte budgets for resident and
// in-flight data, LRU eviction, task-scheduler asynchronous reads, and
// optional cooked-asset catalogs backed by content-addressed objects.

#include <engine/file_asset_store/capabilities.hpp>
#include <engine/file_asset_store/catalog.hpp>
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/kernel/sha256.hpp>
#include <engine/util/hardware_info.hpp>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

#if defined(__linux__)
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace engine::file_asset_store
{
namespace
{

// One cached asset. Identity (handle, path, canonicalPath) is fixed at
// creation; mutable state is split between dataMutex (the byte buffer) and
// stateMutex + stateCondition (state/error published to waiters).
struct AssetRecord
{
    AssetHandle                handle = invalidAsset;
    std::filesystem::path      path;
    std::string                canonicalPath;
    std::atomic<AssetState>    state{AssetState::Queued};
    std::atomic<AssetError>    error{AssetError::None};
    std::atomic<std::uint32_t> references{1}; // starts at 1: the requester's implicit reference
    // Bumped on every successful data swap so consumers can detect reloads.
    std::uint64_t version = 0;
    // Cooked-asset expectations; the sentinel size means "not verified".
    std::uint64_t expectedSize = std::numeric_limits<std::uint64_t>::max();
    std::string   expectedSha256; // empty disables digest verification
    // LRU stamp from the module-wide logical clock; bumped on every use.
    std::atomic<std::uint64_t> lastAccessTick{0};
    mutable std::mutex         dataMutex;
    mutable std::mutex         stateMutex;
    std::condition_variable    stateCondition;
    std::vector<std::uint8_t>  bytes; // file contents; meaningful only in Ready state
};

// ModuleInterface implementation owning the cache, the cooked catalog index,
// and the capability vtable bound in the constructor.
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
        // Built-in defaults first; each recognized key below overrides its
        // default only when present, correctly typed, and inside its range.
        scheduler_               = &context.taskScheduler();
        profiler_                = &context.profiler();
        maximumFileBytes_        = 512ull * 1024ull * 1024ull;
        maximumAssets_           = 8192;
        maximumPendingRequests_  = 8192;
        maximumResidentBytes_    = 2ull * 1024ull * 1024ull * 1024ull;
        cacheUnreferencedAssets_ = false;
        cacheEvictionBatchBytes_ = 64ull * 1024ull * 1024ull;
        defaultAsyncPriority_    = TaskPriority::Low;

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
        // Per-file read cap; 1..16384 MiB.
        if (const auto value = context.configurationIntegerInSection("file_asset_store", "maximum_file_size_mb");
            value && *value >= 1 && *value <= 16384)
            maximumFileBytes_ = static_cast<std::uint64_t>(*value) * 1024ull * 1024ull;
        // Live-record cap; 1..1,000,000.
        if (const auto value = context.configurationIntegerInSection("file_asset_store", "maximum_cached_assets");
            value && *value >= 1 && *value <= 1'000'000)
            maximumAssets_ = static_cast<std::size_t>(*value);
        // Queued asynchronous read cap; 1..1,000,000.
        if (const auto value = context.configurationIntegerInSection("file_asset_store", "maximum_pending_requests");
            value && *value >= 1 && *value <= 1'000'000)
            maximumPendingRequests_ = static_cast<std::size_t>(*value);
        // Cap on loaded bytes held in memory; 1..65536 MiB.
        if (const auto value = context.configurationIntegerInSection("file_asset_store", "maximum_resident_memory_mb");
            value && *value >= 1 && *value <= 65536)
            maximumResidentBytes_ = static_cast<std::uint64_t>(*value) * 1024ull * 1024ull;
        // Cap on bytes reserved by reads in progress; 1..65536 MiB.
        if (const auto value = context.configurationIntegerInSection("file_asset_store", "maximum_inflight_memory_mb");
            value && *value >= 1 && *value <= 65536)
            maximumInflightBytes_ = static_cast<std::uint64_t>(*value) * 1024ull * 1024ull;
        // Keep zero-reference assets instead of freeing them immediately.
        cacheUnreferencedAssets_
            = context.configurationBooleanInSection("file_asset_store", "cache_unreferenced_assets").value_or(false);
        // Extra bytes evicted beyond the shortfall when making room; 0..4096 MiB.
        if (const auto value = context.configurationIntegerInSection("file_asset_store", "cache_eviction_batch_mb");
            value && *value >= 0 && *value <= 4096)
            cacheEvictionBatchBytes_ = static_cast<std::uint64_t>(*value) * 1024ull * 1024ull;
        // Queue used when a request passes AssetRequestPriority::Default.
        if (const auto value = context.configurationStringInSection("file_asset_store", "default_async_priority"))
        {
            if (*value == "critical")
                defaultAsyncPriority_ = TaskPriority::Critical;
            else if (*value == "high")
                defaultAsyncPriority_ = TaskPriority::High;
            else if (*value == "normal")
                defaultAsyncPriority_ = TaskPriority::Normal;
            else if (*value == "low")
                defaultAsyncPriority_ = TaskPriority::Low;
            else
                std::fprintf(
                    stderr, "file_asset_store: unknown default_async_priority '%s'; using low\n", value->c_str());
        }
        }
        // Canonical path length cap; 64..65536 bytes. Structural: honored in
        // both modes.
        if (const auto value = context.configurationIntegerInSection("file_asset_store", "maximum_path_bytes");
            value && *value >= 64 && *value <= 65536)
            maximumPathBytes_ = static_cast<std::size_t>(*value);
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
                cookedCatalogAvailable_ = true;
                cookedObjectDirectory_  = cookedCatalogPath_.parent_path();
                if (const auto objectDirectory
                    = context.configurationStringInSection("file_asset_store", "cooked_object_directory");
                    objectDirectory && !objectDirectory->empty())
                    cookedObjectDirectory_ = *objectDirectory;
            }
            else
            {
                std::fprintf(stderr, "file_asset_store: cooked asset catalog unavailable: %s\n", catalogError.c_str());
            }
        }
        // Re-hash loaded bytes against the catalog digest when set.
        verifyCookedAssetHashes_
            = context.configurationBooleanInSection("file_asset_store", "verify_cooked_asset_hashes").value_or(false);
        // Optional filesystem root. Relative requests resolve against it and
        // every path is confined to it; if the root does not resolve to an
        // existing directory, all requests are rejected rather than silently
        // falling back to unrestricted filesystem access.
        if (const auto value = context.configurationStringInSection("file_asset_store", "root_directory");
            value && !value->empty())
        {
            rootConfigured_ = true;
            std::error_code error;
            rootDirectory_ = std::filesystem::weakly_canonical(*value, error);
            rootRestricted_
                = !error && !rootDirectory_.empty() && std::filesystem::is_directory(rootDirectory_, error) && !error;
        }
    }

private:
    // auto_config = true: derive the cache's memory shape from installed RAM
    // (unknown RAM keeps the built-in budgets, which are the conservative
    // fallback). resident gets ~25% of RAM, in-flight ~12.5%, and the
    // eviction overshoot ~3%; on machines with 8+ GiB the zero-reference
    // cache stays warm so burst reloads re-serve from memory instead of
    // re-reading files (fewer IO stalls on tail frames).
    void applyHardwareDerivedMemoryBudgets()
    {
        constexpr std::uint64_t mib    = 1024ull * 1024ull;
        constexpr std::uint64_t gib    = 1024ull * mib;
        const std::uint64_t     ram    = util::physicalMemoryBytes();
        if (ram == 0u)
        {
            std::fprintf(stderr,
                "file_asset_store: auto_config is on; installed RAM unknown, keeping built-in budgets; "
                "other tuning keys in [file_asset_store] are ignored\n");
            return;
        }
        maximumResidentBytes_    = std::clamp<std::uint64_t>(ram / 4u, 256u * mib, 8192u * mib);
        maximumInflightBytes_    = std::clamp<std::uint64_t>(ram / 8u, 128u * mib, 4096u * mib);
        cacheEvictionBatchBytes_ = std::clamp<std::uint64_t>(ram * 3ull / 100ull, 32u * mib, 512u * mib);
        cacheUnreferencedAssets_ = ram >= 8ull * gib;
        std::fprintf(stderr,
            "file_asset_store: auto_config is on; budgets derived from %.1f GiB RAM "
            "(resident=%llu MiB, inflight=%llu MiB, eviction_batch=%llu MiB, cache_unreferenced=%s); "
            "other tuning keys in [file_asset_store] are ignored\n",
            static_cast<double>(ram) / static_cast<double>(gib),
            (unsigned long long)(maximumResidentBytes_ / mib),
            (unsigned long long)(maximumInflightBytes_ / mib),
            (unsigned long long)(cacheEvictionBatchBytes_ / mib),
            cacheUnreferencedAssets_ ? "on" : "off");
    }

public:

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
        std::lock_guard lock(recordsMutex_);
        records_.clear();
        pathIndex_.clear();
        residentBytes_ = 0;
        reservedBytes_ = 0;
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

    // Writes an error code through an optional out-parameter.
    static void setError(AssetError *output, AssetError error)
    {
        if (output != nullptr)
            *output = error;
    }

    // Publishes state/error and wakes everyone blocked in waitForAsset.
    static void setState(const std::shared_ptr<AssetRecord> &record, AssetState state, AssetError error)
    {
        {
            std::lock_guard lock(record->stateMutex);
            record->error.store(error, std::memory_order_release);
            record->state.store(state, std::memory_order_release);
        }
        record->stateCondition.notify_all();
    }

    // Resolves a request path to a canonical absolute path. Returns "" when
    // the path is unusable, too long, or (with a configured root) escapes the
    // root; relative paths are interpreted against the root directory.
    std::string canonicalPath(const char *path) const
    {
        if (path == nullptr || (rootConfigured_ && !rootRestricted_))
            return {};
        const auto *terminator = static_cast<const char *>(std::memchr(path, '\0', maximumPathBytes_ + 1u));
        if (terminator == nullptr || terminator == path)
            return {};
        const std::string input(path, static_cast<std::size_t>(terminator - path));
        std::error_code       error;
        std::filesystem::path requested(input);
        if (rootRestricted_ && !requested.is_absolute())
            requested = rootDirectory_ / requested;
        std::filesystem::path value = std::filesystem::weakly_canonical(requested, error);
        if (error)
            value = std::filesystem::absolute(requested, error);
        if (error)
            return {};
        value = value.lexically_normal();
        if (rootRestricted_)
        {
            const std::filesystem::path relative = value.lexically_relative(rootDirectory_);
            if (relative.empty() || relative.is_absolute() || *relative.begin() == "..")
                return {};
        }
        const std::string result = value.generic_string();
        return result.size() <= maximumPathBytes_ ? result : std::string{};
    }

    // Gives back bytes reserved by an in-flight read; saturates at zero.
    void releaseReservation(std::uint64_t bytes)
    {
        if (bytes == 0)
            return;
        std::lock_guard lock(recordsMutex_);
        reservedBytes_ -= std::min(reservedBytes_, bytes);
    }

    std::shared_ptr<AssetRecord> findAsset(AssetHandle handle)
    {
        std::lock_guard lock(recordsMutex_);
        const auto      iterator = records_.find(handle);
        return iterator == records_.end() ? nullptr : iterator->second;
    }

    // Bumps the record's LRU stamp from the module-wide logical clock.
    void touchAsset(const std::shared_ptr<AssetRecord> &record) noexcept
    {
        if (record)
            record->lastAccessTick.store(
                accessSequence_.fetch_add(1, std::memory_order_relaxed) + 1u, std::memory_order_relaxed);
    }

    // Removes a record and its path-index entry, returning its bytes to the
    // resident budget and reporting the byte count actually freed. Caller
    // must hold recordsMutex_.
    std::uint64_t eraseRecordLocked(std::unordered_map<AssetHandle, std::shared_ptr<AssetRecord>>::iterator iterator)
    {
        const std::shared_ptr<AssetRecord> &record = iterator->second;
        std::uint64_t                       byteCount = 0;
        {
            std::lock_guard dataLock(record->dataMutex);
            byteCount = record->bytes.size();
            residentBytes_ -= std::min(residentBytes_, byteCount);
            std::vector<std::uint8_t>().swap(record->bytes);
        }
        pathIndex_.erase(record->canonicalPath);
        records_.erase(iterator);
        return byteCount;
    }

    // LRU eviction over unreferenced records that finished (Ready or Failed).
    // Candidates are popped in ascending (last-access tick, handle) order -
    // the exact order a full sort produced - but via a min-heap, so an
    // eviction that meets its targets after a few removals never orders the
    // rest of the cache: O(k + e log k) instead of O(k log k) while
    // recordsMutex_ is held, which shortens the lock hold callers observe on
    // the request/read paths. Each removal revalidates the record under the
    // lock and returns its bytes through eraseRecordLocked (one dataMutex
    // acquisition per eviction instead of two). Continues until both the
    // byte and count targets are met; purgeAll ignores the targets and evicts
    // every candidate. Caller must hold recordsMutex_.
    std::uint64_t evictUnusedLocked(std::uint64_t targetBytes, std::size_t targetCount = 0, bool purgeAll = false)
    {
        // Percentile-relevant: this runs under recordsMutex_, so its duration
        // is the lock-hold time every concurrent request/read waits behind.
        const ScopedProfileTimer evictionTimer(profiler_, PerformanceMetric::AssetEviction);
        struct Candidate
        {
            std::uint64_t tick;
            AssetHandle   handle;
        };
        std::vector<Candidate> candidates;
        try
        {
            candidates.reserve(records_.size());
            for (const auto &[handle, record] : records_)
            {
                if (record->references.load(std::memory_order_acquire) != 0)
                    continue;
                const AssetState state = record->state.load(std::memory_order_acquire);
                if (state != AssetState::Ready && state != AssetState::Failed)
                    continue;
                candidates.push_back({record->lastAccessTick.load(std::memory_order_relaxed), handle});
            }
        }
        catch (const std::bad_alloc &)
        {
            return 0;
        }
        // Max-heap on the reversed (tick, handle) order: the heap top is the
        // least-recently-used candidate, so pop_heap hands it to the back and
        // processing the back yields ascending (tick, handle) - identical to
        // the previous std::sort order including the handle tie-break.
        const auto leastRecentlyUsedFirst = [](const Candidate &lhs, const Candidate &rhs) noexcept
        { return lhs.tick != rhs.tick ? lhs.tick > rhs.tick : lhs.handle > rhs.handle; };
        std::make_heap(candidates.begin(), candidates.end(), leastRecentlyUsedFirst);

        std::uint64_t freedBytes   = 0;
        std::size_t   evictedCount = 0;
        while (!candidates.empty())
        {
            if (!purgeAll && freedBytes >= targetBytes && evictedCount >= targetCount)
                break;
            std::pop_heap(candidates.begin(), candidates.end(), leastRecentlyUsedFirst);
            const Candidate candidate = candidates.back();
            candidates.pop_back();
            const auto found = records_.find(candidate.handle);
            if (found == records_.end() || found->second->references.load(std::memory_order_acquire) != 0)
                continue;
            const AssetState state = found->second->state.load(std::memory_order_acquire);
            if (state != AssetState::Ready && state != AssetState::Failed)
                continue;
            freedBytes += eraseRecordLocked(found);
            ++evictedCount;
        }
        return freedBytes;
    }

    // Reads the record's file into memory and publishes the bytes. Budgets
    // are accounted under recordsMutex_ (reserve before reading, commit or
    // release after); Linux reads through an opened descriptor and other
    // platforms use std::ifstream. Leaves the record Ready or Failed and
    // reports the reason via *error when given.
    bool readAsset(const std::shared_ptr<AssetRecord> &record, AssetError *error)
    {
        std::error_code      filesystemError;
        const std::uintmax_t fileSize = std::filesystem::file_size(record->path, filesystemError);
        if (filesystemError)
        {
            setState(record, AssetState::Failed, AssetError::FileNotFound);
            setError(error, AssetError::FileNotFound);
            return false;
        }
        if (fileSize > maximumFileBytes_
            || fileSize > static_cast<std::uintmax_t>(std::numeric_limits<std::size_t>::max())
            || fileSize > static_cast<std::uintmax_t>(std::numeric_limits<std::streamsize>::max()))
        {
            setState(record, AssetState::Failed, AssetError::FileTooLarge);
            setError(error, AssetError::FileTooLarge);
            return false;
        }

        const std::uint64_t newSize = static_cast<std::uint64_t>(fileSize);
        // Cooked assets must still match the catalog's recorded size.
        if (record->expectedSize != std::numeric_limits<std::uint64_t>::max() && newSize != record->expectedSize)
        {
            setState(record, AssetState::Failed, AssetError::CookedAssetIntegrityFailed);
            setError(error, AssetError::CookedAssetIntegrityFailed);
            return false;
        }
        {
            // Reserve against the in-flight and resident budgets, evicting
            // LRU assets first when resident space falls short (saturating
            // addition keeps the accounting safe from overflow).
            std::lock_guard lock(recordsMutex_);
            if (!records_.contains(record->handle))
            {
                setState(record, AssetState::Ready, AssetError::None);
                setError(error, AssetError::None);
                return true;
            }
            const std::uint64_t inflightAvailable
                = maximumInflightBytes_ - std::min(maximumInflightBytes_, reservedBytes_);
            if (newSize > inflightAvailable)
            {
                setState(record, AssetState::Failed, AssetError::CapacityExceeded);
                setError(error, AssetError::CapacityExceeded);
                return false;
            }
            const std::uint64_t accounted
                = residentBytes_ > maximumResidentBytes_ - std::min(maximumResidentBytes_, reservedBytes_)
                    ? maximumResidentBytes_
                    : residentBytes_ + reservedBytes_;
            const std::uint64_t residentAvailable = maximumResidentBytes_ - std::min(maximumResidentBytes_, accounted);
            if (newSize > residentAvailable)
            {
                const std::uint64_t shortage = newSize - residentAvailable;
                const std::uint64_t evictionTarget
                    = shortage > std::numeric_limits<std::uint64_t>::max() - cacheEvictionBatchBytes_
                        ? shortage
                        : shortage + cacheEvictionBatchBytes_;
                evictUnusedLocked(evictionTarget);
            }
            const std::uint64_t currentAccounted
                = residentBytes_ > maximumResidentBytes_ - std::min(maximumResidentBytes_, reservedBytes_)
                    ? maximumResidentBytes_
                    : residentBytes_ + reservedBytes_;
            if (newSize > maximumResidentBytes_ - std::min(maximumResidentBytes_, currentAccounted))
            {
                setState(record, AssetState::Failed, AssetError::CapacityExceeded);
                setError(error, AssetError::CapacityExceeded);
                return false;
            }
            reservedBytes_ += newSize;
        }

        std::vector<std::uint8_t> loaded;
        try
        {
            loaded.resize(static_cast<std::size_t>(fileSize));
        }
        catch (const std::bad_alloc &)
        {
            releaseReservation(newSize);
            setState(record, AssetState::Failed, AssetError::CapacityExceeded);
            setError(error, AssetError::CapacityExceeded);
            return false;
        }
#if defined(__linux__)

        // Read from an opened descriptor instead of mapping the file. This
        // handles empty files and reports concurrent truncation as an ordinary
        // read failure rather than risking a fault while touching mapped pages.
        {
            const int fd = ::open(record->path.c_str(), O_RDONLY | O_CLOEXEC);
            if (fd < 0)
            {
                releaseReservation(newSize);
                setState(record, AssetState::Failed, AssetError::FileNotFound);
                setError(error, AssetError::FileNotFound);
                return false;
            }

            struct stat fileStatus{};
            if (::fstat(fd, &fileStatus) != 0 || fileStatus.st_size < 0
                || static_cast<std::uintmax_t>(fileStatus.st_size) != fileSize)
            {
                ::close(fd);
                releaseReservation(newSize);
                setState(record, AssetState::Failed, AssetError::ReadFailed);
                setError(error, AssetError::ReadFailed);
                return false;
            }

            std::size_t offset = 0;
            bool        readFailed = false;
            while (offset < loaded.size())
            {
                const std::size_t remaining = loaded.size() - offset;
                const std::size_t chunkSize = std::min<std::size_t>(
                    remaining, static_cast<std::size_t>(std::numeric_limits<ssize_t>::max()));
                const ssize_t count = ::read(fd, loaded.data() + offset, chunkSize);
                if (count < 0 && errno == EINTR)
                    continue;
                if (count <= 0)
                {
                    readFailed = true;
                    break;
                }
                offset += static_cast<std::size_t>(count);
            }
            if (!readFailed)
            {
                std::uint8_t extraByte = 0;
                ssize_t     count = 0;
                do
                {
                    count = ::read(fd, &extraByte, 1u);
                } while (count < 0 && errno == EINTR);
                readFailed = count != 0;
            }
            ::close(fd);
            if (readFailed)
            {
                releaseReservation(newSize);
                setState(record, AssetState::Failed, AssetError::ReadFailed);
                setError(error, AssetError::ReadFailed);
                return false;
            }
        }
#else
        std::ifstream stream(record->path, std::ios::binary);
        if (!stream)
        {
            releaseReservation(newSize);
            setState(record, AssetState::Failed, AssetError::FileNotFound);
            setError(error, AssetError::FileNotFound);
            return false;
        }
        if (!loaded.empty())
        {
            stream.read(reinterpret_cast<char *>(loaded.data()), static_cast<std::streamsize>(loaded.size()));
            if (stream.gcount() != static_cast<std::streamsize>(loaded.size()))
            {
                releaseReservation(newSize);
                setState(record, AssetState::Failed, AssetError::ReadFailed);
                setError(error, AssetError::ReadFailed);
                return false;
            }
        }
        if (stream.peek() != std::char_traits<char>::eof())
        {
            releaseReservation(newSize);
            setState(record, AssetState::Failed, AssetError::ReadFailed);
            setError(error, AssetError::ReadFailed);
            return false;
        }
#endif
        // Optional integrity check for cooked assets.
        if (verifyCookedAssetHashes_ && !record->expectedSha256.empty())
        {
            const std::string digest = engine::Sha256::hexDigest(loaded.data(), loaded.size());
            if (!engine::sha256HexDigestsEqual(digest, record->expectedSha256))
            {
                releaseReservation(newSize);
                setState(record, AssetState::Failed, AssetError::CookedAssetIntegrityFailed);
                setError(error, AssetError::CookedAssetIntegrityFailed);
                return false;
            }
        }
        {
            // Commit: convert the reservation to resident bytes, swap the
            // buffer in under dataMutex, and bump the content version.
            std::lock_guard recordsLock(recordsMutex_);
            reservedBytes_ -= std::min(reservedBytes_, newSize);
            if (records_.contains(record->handle))
            {
                std::lock_guard     dataLock(record->dataMutex);
                const std::uint64_t oldSize = record->bytes.size();
                residentBytes_ -= std::min(residentBytes_, oldSize);
                residentBytes_ += newSize;
                record->bytes.swap(loaded);
                ++record->version;
            }
        }
        setState(record, AssetState::Ready, AssetError::None);
        setError(error, AssetError::None);
        return true;
    }

    AssetHandle requestAssetImpl(const AssetRequest *request, AssetError *error)
    {
        setError(error, AssetError::None);
        if (request == nullptr)
        {
            setError(error, AssetError::InvalidPath);
            return invalidAsset;
        }
        const std::string path = canonicalPath(request->filePath);
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
        if (!cookedCatalogAvailable_)
        {
            setError(error, AssetError::CatalogUnavailable);
            return invalidAsset;
        }
        const auto *pathTerminator
            = static_cast<const char *>(std::memchr(request->filePath, '\0', maximumPathBytes_ + 1u));
        if (pathTerminator == nullptr || pathTerminator == request->filePath)
        {
            setError(error, AssetError::InvalidPath);
            return invalidAsset;
        }
        const std::string input(request->filePath, static_cast<std::size_t>(pathTerminator - request->filePath));
        std::string       key;
        if (!normalizeAssetKey(input, key) || key.size() > maximumPathBytes_)
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
            request, assetObjectPath(cookedObjectDirectory_, found->second.sha256), identity, error, &found->second);
    }

    TaskPriority resolveTaskPriority(AssetRequestPriority requested) const noexcept
    {
        switch (requested)
        {
        case AssetRequestPriority::Critical:
            return TaskPriority::Critical;
        case AssetRequestPriority::High:
            return TaskPriority::High;
        case AssetRequestPriority::Normal:
            return TaskPriority::Normal;
        case AssetRequestPriority::Low:
            return TaskPriority::Low;
        case AssetRequestPriority::Default:
            return defaultAsyncPriority_;
        }
        return defaultAsyncPriority_;
    }

    bool reservePendingJob()
    {
        std::lock_guard lock(jobsMutex_);
        const std::size_t pending = pendingJobs_.load(std::memory_order_relaxed);
        if (stopping_.load(std::memory_order_acquire) || scheduler_ == nullptr
            || pending >= maximumPendingRequests_)
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

    // Shared request path: dedupe by identity string (canonical path or
    // "cooked://key"), make room at the record cap, then either schedule an
    // asynchronous read or run it inline. A handle is returned even when the
    // async submission fails with Busy (the record stays as Failed/Busy).
    AssetHandle requestResolvedAssetImpl(const AssetRequest *request, const std::filesystem::path &resolvedPath,
        const std::string &identity, AssetError *error, const AssetCatalogEntry *cookedEntry = nullptr)
    {
        std::shared_ptr<AssetRecord> record;
        {
            // Existing asset for this identity: hand the same record out
            // again with an extra reference instead of loading twice.
            std::lock_guard lock(recordsMutex_);
            const auto      existing = pathIndex_.find(identity);
            if (existing != pathIndex_.end())
            {
                const auto found = records_.find(existing->second);
                if (found != records_.end())
                {
                    const std::uint32_t references
                        = found->second->references.load(std::memory_order_relaxed);
                    if (references == std::numeric_limits<std::uint32_t>::max())
                    {
                        setError(error, AssetError::CapacityExceeded);
                        return invalidAsset;
                    }
                    found->second->references.store(references + 1u, std::memory_order_relaxed);
                    touchAsset(found->second);
                    return found->first;
                }
            }
            if (records_.size() >= maximumAssets_)
                evictUnusedLocked(0, 1);
            if (records_.size() >= maximumAssets_)
            {
                setError(error, AssetError::CapacityExceeded);
                return invalidAsset;
            }
            try
            {
                record = std::make_shared<AssetRecord>();
                // Probe live handles after counter wrap; 0 remains reserved.
                AssetHandle candidate = nextHandle_;
                bool        handleAvailable = false;
                for (std::size_t attempt = 0; attempt <= records_.size(); ++attempt)
                {
                    if (candidate == invalidAsset)
                        candidate = 1;
                    if (!records_.contains(candidate))
                    {
                        handleAvailable = true;
                        break;
                    }
                    candidate = candidate == std::numeric_limits<AssetHandle>::max() ? 1 : candidate + 1;
                }
                if (!handleAvailable)
                {
                    setError(error, AssetError::CapacityExceeded);
                    return invalidAsset;
                }
                record->handle = candidate;
                nextHandle_ = candidate == std::numeric_limits<AssetHandle>::max() ? 1 : candidate + 1;
                record->path          = resolvedPath;
                record->canonicalPath = identity;
                touchAsset(record);
                if (cookedEntry != nullptr)
                {
                    record->expectedSize   = cookedEntry->sizeBytes;
                    record->expectedSha256 = cookedEntry->sha256;
                }
                records_.emplace(record->handle, record);
                try
                {
                    pathIndex_.emplace(identity, record->handle);
                }
                catch (...)
                {
                    records_.erase(record->handle);
                    throw;
                }
            }
            catch (const std::bad_alloc &)
            {
                setError(error, AssetError::CapacityExceeded);
                return invalidAsset;
            }
        }
        if (request->asynchronous)
        {
            const TaskPriority taskPriority = resolveTaskPriority(request->priority);
            if (!reservePendingJob())
            {
                setState(record, AssetState::Failed, AssetError::Busy);
                setError(error, AssetError::Busy);
                return record->handle;
            }

            bool accepted = false;
            try
            {
                accepted = scheduler_->schedule(
                    [this, record]
                    {
                        setState(record, AssetState::Loading, AssetError::None);
                        try
                        {
                            if (profiler_ != nullptr)
                            {
                                ScopedProfileTimer timer(*profiler_, PerformanceMetric::AssetRead);
                                readAsset(record, nullptr);
                            }
                            else
                            {
                                readAsset(record, nullptr);
                            }
                        }
                        catch (...)
                        {
                            setState(record, AssetState::Failed, AssetError::ReadFailed);
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
                setState(record, AssetState::Failed, AssetError::Busy);
                setError(error, AssetError::Busy);
            }
        }
        else
        {
            setState(record, AssetState::Loading, AssetError::None);
            readAsset(record, error);
        }
        return record->handle;
    }

    // Reference counting under recordsMutex_. Releasing the last reference
    // frees the record unless cache_unreferenced_assets keeps it as an
    // eviction candidate.
    bool retainAssetImpl(AssetHandle handle)
    {
        std::lock_guard lock(recordsMutex_);
        const auto      iterator = records_.find(handle);
        if (iterator == records_.end())
            return false;
        const std::uint32_t references = iterator->second->references.load(std::memory_order_relaxed);
        if (references == std::numeric_limits<std::uint32_t>::max())
            return false;
        iterator->second->references.store(references + 1u, std::memory_order_relaxed);
        touchAsset(iterator->second);
        return true;
    }

    void releaseAssetImpl(AssetHandle handle)
    {
        std::lock_guard lock(recordsMutex_);
        const auto      iterator = records_.find(handle);
        if (iterator == records_.end())
            return;
        const std::uint32_t current = iterator->second->references.load(std::memory_order_acquire);
        if (current == 0)
            return;
        const std::uint32_t previous = iterator->second->references.fetch_sub(1, std::memory_order_acq_rel);
        if (previous > 1)
            return;
        if (cacheUnreferencedAssets_)
        {
            touchAsset(iterator->second);
            return;
        }
        eraseRecordLocked(iterator);
    }

    // Snapshot of the record; canonicalPath points into the record's string
    // and is invalidated when the record is evicted or the module stops.
    bool assetInfoImpl(AssetHandle handle, AssetInfo *output)
    {
        auto record = findAsset(handle);
        if (!record || output == nullptr)
            return false;
        touchAsset(record);
        output->state          = record->state.load(std::memory_order_acquire);
        output->error          = record->error.load(std::memory_order_acquire);
        output->referenceCount = record->references.load(std::memory_order_relaxed);
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
        auto record = findAsset(handle);
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
    // is large enough and the asset is Ready.
    bool copyAssetDataImpl(AssetHandle handle, void *output, std::uint64_t capacity, std::uint64_t *outRequired)
    {
        auto record = findAsset(handle);
        if (!record || record->state.load(std::memory_order_acquire) != AssetState::Ready)
            return false;
        touchAsset(record);
        std::lock_guard lock(record->dataMutex);
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
        auto record = findAsset(handle);
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
        const auto record = findAsset(handle);
        if (!record)
            return false;
        touchAsset(record);
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
        std::lock_guard lock(recordsMutex_);
        return evictUnusedLocked(targetBytesToFree, 0, targetBytesToFree == 0);
    }

    // Consistent snapshot under recordsMutex_; the evictable byte count
    // additionally locks each record's dataMutex.
    bool cacheStatisticsImpl(AssetCacheStatistics *output)
    {
        if (output == nullptr)
            return false;
        std::lock_guard      lock(recordsMutex_);
        AssetCacheStatistics result{};
        result.residentBytes = residentBytes_;
        result.reservedBytes = reservedBytes_;
        result.assetCount    = records_.size();
        for (const auto &[handle, record] : records_)
        {
            (void)handle;
            if (record->references.load(std::memory_order_acquire) == 0)
            {
                const AssetState state = record->state.load(std::memory_order_acquire);
                if (state == AssetState::Ready || state == AssetState::Failed)
                {
                    ++result.unreferencedAssetCount;
                    std::lock_guard dataLock(record->dataMutex);
                    result.evictableBytes += record->bytes.size();
                }
            }
        }
        *output = result;
        return true;
    }

    // Cache state. recordsMutex_ guards records_, pathIndex_, residentBytes_,
    // reservedBytes_, and nextHandle_; the *_mb configuration keys are
    // converted to byte units in construct().
    ParticipantManifest                                           manifest_;
    FileAssetStoreCapability                                      capability_{};
    std::mutex                                                    recordsMutex_;
    std::unordered_map<AssetHandle, std::shared_ptr<AssetRecord>> records_;
    std::unordered_map<std::string, AssetHandle>                  pathIndex_;
    AssetHandle                                                   nextHandle_ = 1;
    std::atomic<std::uint64_t>                                    accessSequence_{0};
    bool                                                          cacheUnreferencedAssets_ = false;
    std::uint64_t                                                 cacheEvictionBatchBytes_ = 64ull * 1024ull * 1024ull;
    std::size_t                                                   maximumAssets_           = 8192;
    std::size_t                                                   maximumPendingRequests_  = 8192;
    TaskPriority                                                  defaultAsyncPriority_    = TaskPriority::Low;
    std::uint64_t                                                 maximumFileBytes_        = 512ull * 1024ull * 1024ull;
    std::uint64_t                                      maximumResidentBytes_ = 2ull * 1024ull * 1024ull * 1024ull;
    std::uint64_t                                      maximumInflightBytes_ = 512ull * 1024ull * 1024ull;
    std::uint64_t                                      residentBytes_        = 0;
    std::uint64_t                                      reservedBytes_        = 0;
    std::size_t                                        maximumPathBytes_     = 4096;
    std::filesystem::path                              rootDirectory_;
    std::filesystem::path                              cookedCatalogPath_;
    std::filesystem::path                              cookedObjectDirectory_;
    std::unordered_map<std::string, AssetCatalogEntry> cookedCatalog_;
    bool                                               cookedCatalogAvailable_  = false;
    bool                                               verifyCookedAssetHashes_ = false;
    bool                                               rootConfigured_          = false;
    bool                                               rootRestricted_          = false;
    TaskScheduler                                     *scheduler_               = nullptr;
    PerformanceProfiler                               *profiler_                = nullptr;
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
