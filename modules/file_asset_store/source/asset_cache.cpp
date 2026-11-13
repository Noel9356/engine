// AssetCache implementation: see asset_cache.hpp for the contract.

#include "asset_cache.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine::file_asset_store
{

void setAssetState(AssetRecord &record, AssetState state, AssetError error) noexcept
{
    {
        std::lock_guard lock(record.stateMutex);
        record.error.store(error, std::memory_order_release);
        record.state.store(state, std::memory_order_release);
    }
    record.stateCondition.notify_all();
}

// --- Path canonicalization ---

std::string AssetCache::canonicalPath(const char *path) const
{
    if (path == nullptr || config_ == nullptr || (config_->rootConfigured && !config_->rootRestricted))
        return {};
    const auto *terminator = static_cast<const char *>(std::memchr(path, '\0', config_->maximumPathBytes + 1u));
    if (terminator == nullptr || terminator == path)
        return {};
    const std::string input(path, static_cast<std::size_t>(terminator - path));

    // Consult the canonical-path cache when configured. The cache maps the
    // raw input string to its canonical form; entries are reused as long
    // as the filesystem does not change the resolution. A stale entry
    // would at worst cause a duplicate record, never a correctness bug.
    if (config_->canonicalPathCacheSize > 0)
    {
        std::lock_guard lock(canonicalPathMutex_);
        const auto      cached = canonicalPathCache_.find(input);
        if (cached != canonicalPathCache_.end())
            return cached->second;
    }

    std::error_code             error;
    std::filesystem::path requested(input);
    // Without a restricted root, allow_absolute_paths = false serves only
    // relative requests (resolved against the process working directory).
    if (!config_->rootRestricted && !config_->allowAbsolutePaths && requested.is_absolute())
        return {};
    if (config_->rootRestricted && !requested.is_absolute())
        requested = config_->rootDirectory / requested;
    std::filesystem::path value = std::filesystem::weakly_canonical(requested, error);
    if (error)
        value = std::filesystem::absolute(requested, error);
    if (error)
        return {};
    value = value.lexically_normal();
    if (config_->rootRestricted)
    {
        const std::filesystem::path relative = value.lexically_relative(config_->rootDirectory);
        if (relative.empty() || relative.is_absolute() || *relative.begin() == "..")
            return {};
    }
    const std::string result = value.generic_string();
    if (result.size() > config_->maximumPathBytes)
        return {};

    if (config_->canonicalPathCacheSize > 0)
    {
        std::lock_guard lock(canonicalPathMutex_);
        if (canonicalPathCache_.size() >= config_->canonicalPathCacheSize && !canonicalPathCacheOrder_.empty())
        {
            // FIFO eviction: drop the oldest entry to make room.
            const std::string &oldest = canonicalPathCacheOrder_.front();
            canonicalPathCache_.erase(oldest);
            canonicalPathCacheOrder_.erase(canonicalPathCacheOrder_.begin());
        }
        try
        {
            canonicalPathCache_.emplace(input, result);
            canonicalPathCacheOrder_.push_back(input);
        }
        catch (...)
        {
            // If the cache allocation fails we silently skip caching;
            // correctness is unaffected.
        }
    }
    return result;
}

// --- Record lookup ---

std::shared_ptr<AssetRecord> AssetCache::find(AssetHandle handle)
{
    std::lock_guard lock(mutex_);
    const auto      iterator = records_.find(handle);
    return iterator == records_.end() ? nullptr : iterator->second;
}

AssetHandle AssetCache::findByIdentity(const std::string &identity, AssetError &error)
{
    error = AssetError::None;
    std::lock_guard lock(mutex_);
    const auto      existing = pathIndex_.find(identity);
    if (existing != pathIndex_.end())
    {
        const auto found = records_.find(existing->second);
        if (found != records_.end())
        {
            const std::uint32_t references = found->second->references.load(std::memory_order_relaxed);
            if (references == std::numeric_limits<std::uint32_t>::max())
            {
                error = AssetError::CapacityExceeded;
                return invalidAsset;
            }
            found->second->references.store(references + 1u, std::memory_order_relaxed);
            touch(found->second);
            return found->first;
        }
    }
    return invalidAsset;
}

AssetHandle AssetCache::createRecord(const std::filesystem::path &resolvedPath, const std::string &identity,
    const AssetCatalogEntry *cookedEntry, AssetError &error)
{
    error = AssetError::None;
    std::shared_ptr<AssetRecord> record;
    {
        std::lock_guard lock(mutex_);
        // Two threads can pass the findByIdentity miss window concurrently;
        // the loser of the race adopts the winner's record (with an extra
        // reference) instead of storing a second record for one identity.
        const auto raced = pathIndex_.find(identity);
        if (raced != pathIndex_.end())
        {
            const auto found = records_.find(raced->second);
            if (found != records_.end())
            {
                const std::uint32_t references = found->second->references.load(std::memory_order_relaxed);
                if (references != std::numeric_limits<std::uint32_t>::max())
                {
                    found->second->references.store(references + 1u, std::memory_order_relaxed);
                    touch(found->second);
                    error = AssetError::None;
                    return raced->second;
                }
                error = AssetError::CapacityExceeded;
                return invalidAsset;
            }
        }
        if (records_.size() >= config_->maximumAssets)
            evictLocked(0, 1);
        if (records_.size() >= config_->maximumAssets)
        {
            error = AssetError::CapacityExceeded;
            return invalidAsset;
        }
        try
        {
            record          = std::make_shared<AssetRecord>();
            // Probe live handles after counter wrap; 0 remains reserved.
            AssetHandle candidate      = nextHandle_;
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
                error = AssetError::CapacityExceeded;
                return invalidAsset;
            }
            record->handle = candidate;
            nextHandle_    = candidate == std::numeric_limits<AssetHandle>::max() ? 1 : candidate + 1;
            record->path          = resolvedPath;
            record->canonicalPath = identity;
            touch(record);
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
            error = AssetError::CapacityExceeded;
            return invalidAsset;
        }
    }
    return record->handle;
}

// --- Reference counting ---

bool AssetCache::retain(AssetHandle handle)
{
    std::lock_guard lock(mutex_);
    const auto      iterator = records_.find(handle);
    if (iterator == records_.end())
        return false;
    const std::uint32_t references = iterator->second->references.load(std::memory_order_relaxed);
    if (references == std::numeric_limits<std::uint32_t>::max())
        return false;
    iterator->second->references.store(references + 1u, std::memory_order_relaxed);
    touch(iterator->second);
    return true;
}

void AssetCache::release(AssetHandle handle)
{
    std::lock_guard lock(mutex_);
    const auto      iterator = records_.find(handle);
    if (iterator == records_.end())
        return;
    const std::uint32_t current = iterator->second->references.load(std::memory_order_acquire);
    if (current == 0)
        return;
    const std::uint32_t previous = iterator->second->references.fetch_sub(1, std::memory_order_acq_rel);
    if (previous > 1)
        return;
    if (config_->cacheUnreferencedAssets)
    {
        touch(iterator->second);
        return;
    }
    eraseRecordLocked(iterator);
}

void AssetCache::touch(const std::shared_ptr<AssetRecord> &record) noexcept
{
    if (record)
        record->lastAccessTick.store(
            accessSequence_.fetch_add(1, std::memory_order_relaxed) + 1u, std::memory_order_relaxed);
}

// --- Eviction ---

std::uint64_t AssetCache::eraseRecordLocked(
    std::unordered_map<AssetHandle, std::shared_ptr<AssetRecord>>::iterator iterator)
{
    const std::shared_ptr<AssetRecord> &record   = iterator->second;
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

std::uint64_t AssetCache::evictLocked(std::uint64_t targetBytes, std::size_t targetCount, bool purgeAll)
{
    // Percentile-relevant: this runs under mutex_, so its duration is the
    // lock-hold time every concurrent request/read waits behind.
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
            // For FIFO eviction, use the handle (allocation order) instead of
            // the LRU tick. Lower handle = older = evict first.
            const std::uint64_t sortKey = config_->evictionPolicy == EvictionPolicy::Fifo
                                              ? record->handle
                                              : record->lastAccessTick.load(std::memory_order_relaxed);
            candidates.push_back({sortKey, handle});
        }
    }
    catch (const std::bad_alloc &)
    {
        return 0;
    }
    // Max-heap on the reversed sort key so pop_heap hands the
    // oldest/least-recently-used candidate to the back, matching the order
    // a full ascending sort would produce.
    const auto oldestFirst = [](const Candidate &lhs, const Candidate &rhs) noexcept
    { return lhs.tick != rhs.tick ? lhs.tick > rhs.tick : lhs.handle > rhs.handle; };
    std::make_heap(candidates.begin(), candidates.end(), oldestFirst);

    std::uint64_t freedBytes   = 0;
    std::size_t   evictedCount = 0;
    while (!candidates.empty())
    {
        if (!purgeAll && freedBytes >= targetBytes && evictedCount >= targetCount)
            break;
        std::pop_heap(candidates.begin(), candidates.end(), oldestFirst);
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

std::uint64_t AssetCache::purgeUnused(std::uint64_t targetBytes, bool purgeAll)
{
    std::lock_guard lock(mutex_);
    return evictLocked(targetBytes, 0, purgeAll);
}

// --- Statistics ---

bool AssetCache::statistics(AssetCacheStatistics &output)
{
    output = AssetCacheStatistics{};
    std::lock_guard lock(mutex_);
    output.residentBytes = residentBytes_;
    output.reservedBytes = reservedBytes_;
    output.assetCount    = records_.size();
    for (const auto &[handle, record] : records_)
    {
        (void)handle;
        if (record->references.load(std::memory_order_acquire) == 0)
        {
            const AssetState state = record->state.load(std::memory_order_acquire);
            if (state == AssetState::Ready || state == AssetState::Failed)
            {
                ++output.unreferencedAssetCount;
                std::lock_guard dataLock(record->dataMutex);
                output.evictableBytes += record->bytes.size();
            }
        }
    }
    return true;
}

// --- Budget hooks ---

void AssetCache::releaseReservation(std::uint64_t bytes)
{
    if (bytes == 0)
        return;
    std::lock_guard lock(mutex_);
    reservedBytes_ -= std::min(reservedBytes_, bytes);
}

bool AssetCache::reserveForLoad(const std::shared_ptr<AssetRecord> &record, std::uint64_t newSize, AssetError *error)
{
    std::lock_guard lock(mutex_);
    if (!records_.contains(record->handle))
    {
        // The record was evicted while we were not holding the lock; treat
        // this as a successful no-op (the caller still has the shared_ptr
        // so the bytes will be swapped in, but the cache no longer tracks
        // them until the next request re-creates the identity).
        setAssetState(*record, AssetState::Ready, AssetError::None);
        if (error != nullptr)
            *error = AssetError::None;
        return true;
    }
    const std::uint64_t inflightAvailable
        = config_->maximumInflightBytes - std::min(config_->maximumInflightBytes, reservedBytes_);
    if (newSize > inflightAvailable)
    {
        setAssetState(*record, AssetState::Failed, AssetError::CapacityExceeded);
        if (error != nullptr)
            *error = AssetError::CapacityExceeded;
        return false;
    }
    const std::uint64_t accounted
        = residentBytes_ > config_->maximumResidentBytes - std::min(config_->maximumResidentBytes, reservedBytes_)
              ? config_->maximumResidentBytes
              : residentBytes_ + reservedBytes_;
    const std::uint64_t residentAvailable
        = config_->maximumResidentBytes - std::min(config_->maximumResidentBytes, accounted);
    if (newSize > residentAvailable)
    {
        const std::uint64_t shortage = newSize - residentAvailable;
        const std::uint64_t evictionTarget
            = shortage > std::numeric_limits<std::uint64_t>::max() - config_->cacheEvictionBatchBytes
                  ? shortage
                  : shortage + config_->cacheEvictionBatchBytes;
        evictLocked(evictionTarget, 0);
    }
    const std::uint64_t currentAccounted
        = residentBytes_ > config_->maximumResidentBytes - std::min(config_->maximumResidentBytes, reservedBytes_)
              ? config_->maximumResidentBytes
              : residentBytes_ + reservedBytes_;
    if (newSize > config_->maximumResidentBytes - std::min(config_->maximumResidentBytes, currentAccounted))
    {
        setAssetState(*record, AssetState::Failed, AssetError::CapacityExceeded);
        if (error != nullptr)
            *error = AssetError::CapacityExceeded;
        return false;
    }
    reservedBytes_ += newSize;
    return true;
}

void AssetCache::commitLoad(
    const std::shared_ptr<AssetRecord> &record, std::vector<std::uint8_t> &loaded, std::uint64_t newSize)
{
    std::lock_guard recordsLock(mutex_);
    reservedBytes_ -= std::min(reservedBytes_, newSize);
    if (records_.contains(record->handle))
    {
        std::lock_guard dataLock(record->dataMutex);
        const std::uint64_t oldSize = record->bytes.size();
        residentBytes_ -= std::min(residentBytes_, oldSize);
        residentBytes_ += newSize;
        record->bytes.swap(loaded);
        ++record->version;
    }
}

void AssetCache::clear()
{
    std::lock_guard lock(mutex_);
    records_.clear();
    pathIndex_.clear();
    residentBytes_ = 0;
    reservedBytes_ = 0;
    canonicalPathCache_.clear();
    canonicalPathCacheOrder_.clear();
}

} // namespace engine::file_asset_store
