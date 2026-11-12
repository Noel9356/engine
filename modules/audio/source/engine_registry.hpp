// Engine registry: the module's handle-to-engine-record index.
//
// Owns the live engine records, the engine handle counter, the shared
// resource manager, and the two registry-level locks (lifecycle and registry);
// per-engine state and locking stay on the records themselves. Only compiled
// when the miniaudio backend is enabled.

#ifndef ENGINE_AUDIO_ENGINE_REGISTRY_HPP
#define ENGINE_AUDIO_ENGINE_REGISTRY_HPP

#include "audio_backend.hpp"

#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)

#include <mutex>
#include <unordered_map>

namespace engine
{
namespace audio
{
namespace backend
{

class EngineRegistry
{
public:
    // Registry lookup; the returned shared_ptr keeps the record alive after
    // registry mutations. Callers that also need to touch the record must use
    // lock()/lockSound()/lockBus() instead.
    std::shared_ptr<EngineRecord> find(AudioEngineHandle handle);

    // Resolves and locks an engine; the returned guard holds the record's
    // mutex until it goes out of scope. The lease is empty when the handle is
    // unknown or the record is closed.
    LockedEngine lock(AudioEngineHandle handle);

    // Resolves and locks an engine and one of its sounds; the sound pointer
    // is null when either handle is unknown, in which case the engine lease
    // is released.
    LockedSound lockSound(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle);

    // Resolves and locks an engine and one of its buses, like lockSound.
    LockedBus lockBus(AudioEngineHandle engineHandle, AudioBusHandle busHandle);

    // Serializes engine creation, destruction, and module shutdown. Callers
    // hold the returned lock for the whole operation so registry state cannot
    // change between their capacity checks and insertions.
    std::unique_lock<std::mutex> lifecycleLock();

    // Fail-fast creation check: false while the module is stopping or the
    // configured engine budget is exhausted (0 = unbounded).
    bool canCreate(std::uint32_t maximumEngines);

    // Allocates the next engine handle; invalidAudioEngine when the handle
    // space is exhausted.
    AudioEngineHandle allocateEngineHandle();

    // Publishes a fully initialized record. Re-checks stopping and capacity
    // under the registry lock, so callers that passed canCreate() earlier can
    // race with concurrent shutdown and still fail cleanly.
    AudioError insert(AudioEngineHandle handle, const std::shared_ptr<EngineRecord> &record,
        std::uint32_t maximumEngines);

    // Unpublishes a record, moving its lease to outRecord; false and an empty
    // lease when the handle is unknown.
    bool remove(AudioEngineHandle handle, std::shared_ptr<EngineRecord> &outRecord);

    // The lazily created module-wide resource manager for share_resources
    // mode; nullptr when it cannot be created. Owned and released by
    // closeAll().
    ma_resource_manager *sharedResourceManager(const AudioConfiguration &configuration);

    // Shuts every engine down and releases the shared resource manager;
    // called from module stop() under the lifecycle lock. After closeAll(),
    // canCreate() stays false until the module is constructed again.
    void closeAll();

    // Prepares the registry for use (initial bucket reservation, clear the
    // stopping flag); called from module construct().
    void open(std::uint32_t engineCapacity);

private:
    // Lifecycle operations (create/destroy/stop) serialize on lifecycleMutex_;
    // ordinary operations take enginesMutex_ only long enough to acquire or
    // publish a lease. The two are never held in the opposite order.
    std::mutex lifecycleMutex_;
    std::mutex enginesMutex_;
    std::unordered_map<AudioEngineHandle, std::shared_ptr<EngineRecord>> engines_;
    std::mutex sharedResourceManagerMutex_;
    ma_resource_manager sharedResourceManager_{};
    bool sharedResourceManagerInitialized_ = false;
    bool stopping_ = false;
    std::atomic<std::uint64_t> nextEngine_{1};
};

} // namespace backend
} // namespace audio
} // namespace engine

#endif
#endif
