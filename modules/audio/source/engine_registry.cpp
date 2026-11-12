// Engine registry implementation: handle lookup, lifecycle serialization,
// engine handle allocation, and the shared resource manager. Every engine
// registry operation needs miniaudio state, so the whole unit compiles empty
// without the backend.

#include "engine_registry.hpp"

#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)

namespace engine
{
namespace audio
{
namespace backend
{

std::shared_ptr<EngineRecord> EngineRegistry::find(AudioEngineHandle handle)
{
    std::lock_guard lock(enginesMutex_);
    const auto iterator = engines_.find(handle);
    return iterator != engines_.end() ? iterator->second : nullptr;
}

LockedEngine EngineRegistry::lock(AudioEngineHandle handle)
{
    return LockedEngine(find(handle));
}

LockedSound EngineRegistry::lockSound(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle)
{
    LockedSound locked;
    locked.engine = lock(engineHandle);
    EngineRecord *record = locked.engine.get();
    if (record == nullptr)
    {
        return locked;
    }
    const auto iterator = record->sounds.find(soundHandle);
    if (iterator == record->sounds.end())
    {
        locked.engine.release();
        return locked;
    }
    locked.sound = &iterator->second;
    return locked;
}

LockedBus EngineRegistry::lockBus(AudioEngineHandle engineHandle, AudioBusHandle busHandle)
{
    LockedBus locked;
    locked.engine = lock(engineHandle);
    EngineRecord *record = locked.engine.get();
    if (record == nullptr)
    {
        return locked;
    }
    const auto iterator = record->buses.find(busHandle);
    if (iterator == record->buses.end())
    {
        locked.engine.release();
        return locked;
    }
    locked.bus = &iterator->second;
    return locked;
}

std::unique_lock<std::mutex> EngineRegistry::lifecycleLock()
{
    return std::unique_lock<std::mutex>(lifecycleMutex_);
}

bool EngineRegistry::canCreate(std::uint32_t maximumEngines)
{
    std::lock_guard lock(enginesMutex_);
    return !stopping_ && (maximumEngines == 0 || engines_.size() < maximumEngines);
}

AudioEngineHandle EngineRegistry::allocateEngineHandle()
{
    return allocateHandle(nextEngine_);
}

AudioError EngineRegistry::insert(AudioEngineHandle handle, const std::shared_ptr<EngineRecord> &record,
    std::uint32_t maximumEngines)
{
    std::lock_guard lock(enginesMutex_);
    if (stopping_ || (maximumEngines != 0 && engines_.size() >= maximumEngines))
    {
        return AudioError::CapacityExceeded;
    }
    try
    {
        engines_.emplace(handle, record);
    }
    catch (...)
    {
        return AudioError::CreationFailed;
    }
    return AudioError::None;
}

bool EngineRegistry::remove(AudioEngineHandle handle, std::shared_ptr<EngineRecord> &outRecord)
{
    std::lock_guard lock(enginesMutex_);
    const auto iterator = engines_.find(handle);
    if (iterator == engines_.end())
    {
        return false;
    }
    outRecord = std::move(iterator->second);
    engines_.erase(iterator);
    return true;
}

ma_resource_manager *EngineRegistry::sharedResourceManager(const AudioConfiguration &configuration)
{
    std::lock_guard lock(sharedResourceManagerMutex_);
    if (sharedResourceManagerInitialized_)
    {
        return &sharedResourceManager_;
    }
    const ma_resource_manager_config config = makeResourceManagerConfig(configuration.resourceManagerThreads,
        configuration.decodedFormatFloat32, configuration.decodedChannels, configuration.decodedSampleRate);
    if (ma_resource_manager_init(&config, &sharedResourceManager_) != MA_SUCCESS)
    {
        return nullptr;
    }
    sharedResourceManagerInitialized_ = true;
    return &sharedResourceManager_;
}

void EngineRegistry::closeAll()
{
    std::unordered_map<AudioEngineHandle, std::shared_ptr<EngineRecord>> records;
    {
        std::lock_guard registryLock(enginesMutex_);
        stopping_ = true;
        records.swap(engines_);
    }
    for (const auto &entry : records)
    {
        const auto &record = entry.second;
        std::lock_guard engineLock(record->mutex);
        record->close();
    }
    {
        std::lock_guard resourceLock(sharedResourceManagerMutex_);
        if (sharedResourceManagerInitialized_)
        {
            ma_resource_manager_uninit(&sharedResourceManager_);
            sharedResourceManagerInitialized_ = false;
        }
    }
}

void EngineRegistry::open(std::uint32_t engineCapacity)
{
    std::lock_guard lock(enginesMutex_);
    engines_.reserve(engineCapacity);
    stopping_ = false;
}

} // namespace backend
} // namespace audio
} // namespace engine

#endif
