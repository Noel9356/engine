// One cached asset record: identity, state, reference count, byte buffer.
//
// Identity (handle, path, canonicalPath) is fixed at creation; mutable state
// is split between dataMutex (the byte buffer) and stateMutex +
// stateCondition (state/error published to waiters). Shared between the
// cache, loader, and module orchestrator.

#ifndef ENGINE_FILE_ASSET_STORE_ASSET_RECORD_HPP
#define ENGINE_FILE_ASSET_STORE_ASSET_RECORD_HPP

#include <engine/file_asset_store/capabilities.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace engine::file_asset_store
{

// One cached asset. The handle and identity strings are immutable after
// creation; state/error/references are atomic for lock-free reads. The byte
// buffer is guarded by dataMutex; the state/error pair and the condition
// variable are guarded by stateMutex so waitForAsset can sleep until a
// terminal state is published.
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

// Publishes state/error under stateMutex and wakes everyone blocked in
// waitForAsset. The caller must keep the record alive for the duration of
// the call (a shared_ptr or a stack reference suffices).
void setAssetState(AssetRecord &record, AssetState state, AssetError error) noexcept;

} // namespace engine::file_asset_store

#endif
