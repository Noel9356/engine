// audio module implementation.
//
// Wraps miniaudio (compiled in only when ENGINE_AUDIO_BACKEND_MINIAUDIO is
// defined) behind the "audio" capability: engines, mix buses, and sounds with
// optional 3D spatialization, plus manual rendering for device-less engines.
// Without the backend every entry fails with AudioError::BackendUnavailable.

#include <engine/audio/capabilities.hpp>
#include "lua_bridge.hpp"
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
#include <miniaudio.h>

#include <algorithm>
#include <mutex>
#include <unordered_map>
#include <utility>
#endif

namespace engine
{
namespace audio
{
namespace
{

// Writes an error code through an optional out-parameter.
void setError(AudioError *outError, AudioError error)
{
    if (outError != nullptr)
    {
        *outError = error;
    }
}

std::uint32_t maximumSupportedListenerCount() noexcept
{
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
    return MA_ENGINE_MAX_LISTENERS;
#else
    return 4u;
#endif
}

#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)

// Validation guards for capability floats: finite and non-negative, finite and
// positive, and finite in all three components respectively.
bool finiteNonNegative(float value)
{
    return std::isfinite(value) && value >= 0.0f;
}

bool finitePositive(float value)
{
    return std::isfinite(value) && value > 0.0f;
}

bool finiteVector(const float values[3])
{
    return values != nullptr && std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
}

bool finiteNonZeroVector(const float values[3])
{
    if (!finiteVector(values))
        return false;
    const double lengthSquared = static_cast<double>(values[0]) * values[0]
        + static_cast<double>(values[1]) * values[1] + static_cast<double>(values[2]) * values[2];
    return lengthSquared > 1.0e-16 && std::isfinite(lengthSquared);
}

void normalizeVector(float values[3])
{
    const double lengthSquared = static_cast<double>(values[0]) * values[0]
        + static_cast<double>(values[1]) * values[1] + static_cast<double>(values[2]) * values[2];
    const float inverseLength = static_cast<float>(1.0 / std::sqrt(lengthSquared));
    for (std::uint32_t axis = 0; axis < 3; ++axis)
        values[axis] *= inverseLength;
}

bool validSoundCone(float innerAngleRadians, float outerAngleRadians, float outerGain)
{
    return finiteNonNegative(innerAngleRadians) && finiteNonNegative(outerAngleRadians)
        && innerAngleRadians <= outerAngleRadians && outerAngleRadians <= 2.0f * std::numbers::pi_v<float>
        && finiteNonNegative(outerGain);
}

bool secondsToFrameOffset(double seconds, std::uint32_t sampleRate, ma_uint64 &output)
{
    if (!std::isfinite(seconds) || seconds < 0.0 || sampleRate == 0)
        return false;
    const long double frames = std::ceil(static_cast<long double>(seconds) * sampleRate);
    if (!std::isfinite(frames) || frames > static_cast<long double>(std::numeric_limits<ma_uint64>::max()))
        return false;
    output = static_cast<ma_uint64>(frames);
    return true;
}

// One loaded sound. Records are stored directly in the engine's node-based
// hash map, avoiding one heap allocation per sound while keeping ma_sound's
// address stable for its lifetime.
struct SoundRecord
{
    ma_sound       sound{};
    AudioBusHandle bus         = invalidAudioBus;
    bool           initialized = false;

    ~SoundRecord()
    {
        if (initialized)
        {
            ma_sound_stop(&sound);
            ma_sound_uninit(&sound);
        }
    }

    SoundRecord() = default;
    SoundRecord(const SoundRecord &) = delete;
    SoundRecord &operator=(const SoundRecord &) = delete;
};

// A mix group also caches its direct-child and attached-sound counts. This
// makes destroyBus a constant-time ownership check rather than a scan of all
// buses and sounds.
struct BusRecord
{
    ma_sound_group group{};
    AudioBusHandle parent          = invalidAudioBus;
    std::uint32_t  childBusCount   = 0;
    std::uint32_t  attachedSounds  = 0;
    float          volume          = 1.0f;
    bool           muted           = false;
    bool           initialized     = false;

    ~BusRecord()
    {
        if (initialized)
        {
            ma_sound_group_uninit(&group);
        }
    }

    BusRecord() = default;
    BusRecord(const BusRecord &) = delete;
    BusRecord &operator=(const BusRecord &) = delete;
};

// Each engine owns its sound/bus records and has an independent API mutex.
// The engine registry mutex is held only long enough to acquire/release a
// shared_ptr, so operations on unrelated engines can proceed concurrently.
struct EngineRecord
{
    ma_engine                      engine{};
    ma_resource_manager            ownedResourceManager{};
    ma_resource_manager           *resourceManager = nullptr;
    std::mutex                     mutex;
    std::unordered_map<AudioSoundHandle, SoundRecord> sounds;
    std::unordered_map<AudioBusHandle, BusRecord>    buses;
    std::vector<AudioBusHandle>                      busCreationOrder;
    bool initialized                  = false;
    bool ownsResourceManager          = false;
    bool noDevice                     = false;
    bool closed                       = false;

    ~EngineRecord()
    {
        close();
    }

    // Must be called while mutex is held once the record has been published.
    // A child bus can only be created after its parent, so reverse creation
    // order is a valid linear-time leaf-first destruction order.
    void close()
    {
        if (closed)
        {
            return;
        }

        sounds.clear();
        for (auto iterator = busCreationOrder.rbegin(); iterator != busCreationOrder.rend(); ++iterator)
        {
            buses.erase(*iterator);
        }
        buses.clear();
        busCreationOrder.clear();

        if (initialized)
        {
            ma_engine_uninit(&engine);
            initialized = false;
        }
        if (ownsResourceManager)
        {
            ma_resource_manager_uninit(&ownedResourceManager);
            ownsResourceManager = false;
        }
        resourceManager = nullptr;
        closed = true;
    }
};

// Maps the public attenuation enum onto miniaudio's model; unknown values
// fall back to inverse.
ma_attenuation_model nativeAttenuation(AudioAttenuationModel model)
{
    switch (model)
    {
    case AudioAttenuationModel::None:
        return ma_attenuation_model_none;
    case AudioAttenuationModel::Inverse:
        return ma_attenuation_model_inverse;
    case AudioAttenuationModel::Linear:
        return ma_attenuation_model_linear;
    case AudioAttenuationModel::Exponential:
        return ma_attenuation_model_exponential;
    }
    return ma_attenuation_model_inverse;
}

#endif

// Module singleton backing the "audio" capability. The registry lock protects
// only engine lookup/insertion; each engine serializes its own miniaudio objects.
class AudioModule final : public ModuleInterface
{
public:
    AudioModule()
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
        // Start from the built-in defaults, then overlay [audio] values that
        // validate; out-of-range keys are ignored.
        configuredSampleRate_               = 0;
        configuredChannels_                 = 0;
        configuredListenerCount_            = 1;
        configuredMasterVolume_             = 1.0f;
        configuredSharedResources_          = false;
        configuredResourceManagerThreads_   = 1;
        configuredDecodedFormatFloat32_     = false;
        configuredDecodedChannels_          = 0;
        configuredDecodedSampleRate_        = 0;
        configuredInitialEngineCapacity_    = 4;
        configuredInitialSoundCapacity_     = 64;
        configuredInitialBusCapacity_       = 8;
        configuredMaximumEngines_            = 0;
        configuredMaximumSoundsPerEngine_   = 0;
        configuredMaximumBusesPerEngine_    = 0;
        configuredPeriodSizeInFrames_       = 0;
        configuredGainSmoothTimeInFrames_   = 0;
        configuredDefaultVolumeSmoothFrames_ = 0;
        configuredPreMixStackSizeInBytes_   = 0;

        // A zero sample rate/channel count asks the selected playback device
        // for its native format. Device-less engines must set both explicitly.
        if (const auto value = context.configurationIntegerInSection("audio", "sample_rate");
            value && *value >= 0 && *value <= std::numeric_limits<std::uint32_t>::max())
        {
            configuredSampleRate_ = static_cast<std::uint32_t>(*value);
        }
        if (const auto value = context.configurationIntegerInSection("audio", "channels");
            value && *value >= 0 && *value <= 254)
        {
            configuredChannels_ = static_cast<std::uint32_t>(*value);
        }
        if (const auto value = context.configurationIntegerInSection("audio", "listener_count");
            value && *value >= 1 && static_cast<std::uint64_t>(*value) <= maximumSupportedListenerCount())
        {
            configuredListenerCount_ = static_cast<std::uint32_t>(*value);
        }
        if (const auto value = context.configurationFloatingInSection("audio", "master_volume");
            value && std::isfinite(*value) && *value >= 0.0)
        {
            configuredMasterVolume_ = static_cast<float>(*value);
        }

        if (const auto value = context.configurationBooleanInSection("audio", "share_resources"); value)
        {
            configuredSharedResources_ = *value;
        }
        if (const auto value = context.configurationIntegerInSection("audio", "resource_manager_job_threads");
            value && *value >= 1 && *value <= 64)
        {
            configuredResourceManagerThreads_ = static_cast<std::uint32_t>(*value);
        }
        if (const auto value = context.configurationStringInSection("audio", "decoded_format"); value)
        {
            if (*value == "f32" || *value == "float32")
            {
                configuredDecodedFormatFloat32_ = true;
            }
            else if (*value == "native")
            {
                configuredDecodedFormatFloat32_ = false;
            }
        }
        if (const auto value = context.configurationIntegerInSection("audio", "decoded_channels");
            value && *value >= 0 && *value <= 32)
        {
            configuredDecodedChannels_ = static_cast<std::uint32_t>(*value);
        }
        if (const auto value = context.configurationIntegerInSection("audio", "decoded_sample_rate");
            value && *value >= 0 && (*value == 0 || (*value >= 8000 && *value <= 384000)))
        {
            configuredDecodedSampleRate_ = static_cast<std::uint32_t>(*value);
        }

        const auto readCapacity = [&context](const char *key, std::uint32_t &target, std::uint32_t maximum)
        {
            if (const auto value = context.configurationIntegerInSection("audio", key);
                value && *value >= 0 && *value <= maximum)
            {
                target = static_cast<std::uint32_t>(*value);
            }
        };
        readCapacity("initial_engine_capacity", configuredInitialEngineCapacity_, 65536);
        readCapacity("initial_sound_capacity", configuredInitialSoundCapacity_, 1048576);
        readCapacity("initial_bus_capacity", configuredInitialBusCapacity_, 65536);
        readCapacity("max_engines", configuredMaximumEngines_, 1048576);
        readCapacity("max_sounds_per_engine", configuredMaximumSoundsPerEngine_, 1048576);
        readCapacity("max_buses_per_engine", configuredMaximumBusesPerEngine_, 65536);
        readCapacity("period_size_in_frames", configuredPeriodSizeInFrames_, std::numeric_limits<std::uint32_t>::max());
        readCapacity("gain_smooth_time_in_frames", configuredGainSmoothTimeInFrames_, std::numeric_limits<std::uint32_t>::max());
        readCapacity("default_volume_smooth_time_in_frames", configuredDefaultVolumeSmoothFrames_, std::numeric_limits<std::uint32_t>::max());
        readCapacity("pre_mix_stack_size_in_bytes", configuredPreMixStackSizeInBytes_, std::numeric_limits<std::uint32_t>::max());

#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        std::lock_guard lock(enginesMutex_);
        engines_.reserve(configuredInitialEngineCapacity_);
        stopping_ = false;
#endif

        scriptRuntime_ = context.tryAcquireCapability<script_runtime::ScriptRuntimeCapability>();
        if (scriptRuntime_ != nullptr && scriptRuntime_->abi == script_runtime::scriptRuntimeCapabilityAbi
            && scriptRuntime_->registerRuntimeInitializer != nullptr)
        {
            luaBridgeContext_ = AudioLuaBridgeContext{scriptRuntime_, &capability_};
            runtimeInitializer_ = scriptRuntime_->registerRuntimeInitializer(
                scriptRuntime_->self, initializeAudioLuaRuntime, &luaBridgeContext_);
        }
    }

    void start() override
    {}

    void stop() override
    {
        if (scriptRuntime_ != nullptr && runtimeInitializer_ != script_runtime::invalidScriptRuntimeInitializer
            && scriptRuntime_->unregisterRuntimeInitializer != nullptr)
        {
            scriptRuntime_->unregisterRuntimeInitializer(scriptRuntime_->self, runtimeInitializer_);
            runtimeInitializer_ = script_runtime::invalidScriptRuntimeInitializer;
        }
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        std::lock_guard lifecycleLock(lifecycleMutex_);
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
#endif
    }

    void *capabilityPointerFor(const std::string &capabilityName) override
    {
        return capabilityName == AudioCapability::capabilityName ? &capability_ : nullptr;
    }

private:
    static ParticipantManifest makeManifest()
    {
        return ParticipantManifest{
            .name                 = "audio",
            .kind                 = ParticipantKind::EngineModule,
            .version              = "0.3.0",
            .providedCapabilities = {"audio"},
            .optionalCapabilities = {"script_runtime"}};
    }

    // Binds each capability field to the same-named Impl member through a
    // forwarding lambda; the void* self is this AudioModule instance.
    void assignCapabilityPointers()
    {
        capability_.self = this;
#define ENGINE_AUDIO_BIND(field, method)                                                                               \
    capability_.field                                                                                                  \
        = [](void *self, auto... args) -> decltype(auto) { return static_cast<AudioModule *>(self)->method(args...); }
        ENGINE_AUDIO_BIND(createEngine, createEngineImpl);
        ENGINE_AUDIO_BIND(destroyEngine, destroyEngineImpl);
        ENGINE_AUDIO_BIND(engineInfo, engineInfoImpl);
        ENGINE_AUDIO_BIND(startEngine, startEngineImpl);
        ENGINE_AUDIO_BIND(stopEngine, stopEngineImpl);
        ENGINE_AUDIO_BIND(setMasterVolume, setMasterVolumeImpl);
        ENGINE_AUDIO_BIND(createBus, createBusImpl);
        ENGINE_AUDIO_BIND(destroyBus, destroyBusImpl);
        ENGINE_AUDIO_BIND(setBusVolume, setBusVolumeImpl);
        ENGINE_AUDIO_BIND(setBusMuted, setBusMutedImpl);
        ENGINE_AUDIO_BIND(createSound, createSoundImpl);
        ENGINE_AUDIO_BIND(destroySound, destroySoundImpl);
        ENGINE_AUDIO_BIND(play, playImpl);
        ENGINE_AUDIO_BIND(pause, pauseImpl);
        ENGINE_AUDIO_BIND(stop, stopImpl);
        ENGINE_AUDIO_BIND(seekSeconds, seekSecondsImpl);
        ENGINE_AUDIO_BIND(isPlaying, isPlayingImpl);
        ENGINE_AUDIO_BIND(atEnd, atEndImpl);
        ENGINE_AUDIO_BIND(soundTimes, soundTimesImpl);
        ENGINE_AUDIO_BIND(setSoundVolume, setSoundVolumeImpl);
        ENGINE_AUDIO_BIND(setSoundPitch, setSoundPitchImpl);
        ENGINE_AUDIO_BIND(setSoundPan, setSoundPanImpl);
        ENGINE_AUDIO_BIND(setSoundLooping, setSoundLoopingImpl);
        ENGINE_AUDIO_BIND(setSoundPosition, setSoundPositionImpl);
        ENGINE_AUDIO_BIND(setSoundVelocity, setSoundVelocityImpl);
        ENGINE_AUDIO_BIND(setListener, setListenerImpl);
        ENGINE_AUDIO_BIND(readFrames, readFramesImpl);
        ENGINE_AUDIO_BIND(fadeSoundGain, fadeSoundGainImpl);
        ENGINE_AUDIO_BIND(schedulePlay, schedulePlayImpl);
        ENGINE_AUDIO_BIND(scheduleStop, scheduleStopImpl);
        ENGINE_AUDIO_BIND(setSoundDirection, setSoundDirectionImpl);
        ENGINE_AUDIO_BIND(setSoundCone, setSoundConeImpl);
        ENGINE_AUDIO_BIND(setSoundDopplerFactor, setSoundDopplerFactorImpl);
        ENGINE_AUDIO_BIND(setSoundListener, setSoundListenerImpl);
        ENGINE_AUDIO_BIND(setSoundSpatializationEnabled, setSoundSpatializationEnabledImpl);
        ENGINE_AUDIO_BIND(setSoundGainRange, setSoundGainRangeImpl);
        ENGINE_AUDIO_BIND(setSoundAttenuation, setSoundAttenuationImpl);
#undef ENGINE_AUDIO_BIND
    }

#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
    std::shared_ptr<EngineRecord> findEngine(AudioEngineHandle handle)
    {
        std::lock_guard lock(enginesMutex_);
        const auto iterator = engines_.find(handle);
        return iterator != engines_.end() ? iterator->second : nullptr;
    }

    static ma_resource_manager_config makeResourceManagerConfig(std::uint32_t jobThreadCount,
        bool decodedFormatFloat32, std::uint32_t decodedChannels, std::uint32_t decodedSampleRate)
    {
        ma_resource_manager_config config = ma_resource_manager_config_init();
        config.jobThreadCount = jobThreadCount;
        config.decodedFormat = decodedFormatFloat32 ? ma_format_f32 : ma_format_unknown;
        config.decodedChannels = decodedChannels;
        config.decodedSampleRate = decodedSampleRate;
        return config;
    }

    bool initializeSharedResourceManager()
    {
        std::lock_guard lock(sharedResourceManagerMutex_);
        if (sharedResourceManagerInitialized_)
        {
            return true;
        }
        const ma_resource_manager_config config = makeResourceManagerConfig(configuredResourceManagerThreads_,
            configuredDecodedFormatFloat32_, configuredDecodedChannels_, configuredDecodedSampleRate_);
        if (ma_resource_manager_init(&config, &sharedResourceManager_) != MA_SUCCESS)
        {
            return false;
        }
        sharedResourceManagerInitialized_ = true;
        return true;
    }

    static std::uint32_t allocateHandle(std::atomic<std::uint64_t> &counter)
    {
        std::uint64_t value = counter.load(std::memory_order_relaxed);
        for (;;)
        {
            if (value == 0 || value > std::numeric_limits<std::uint32_t>::max())
            {
                return 0;
            }
            if (counter.compare_exchange_weak(
                    value, value + 1, std::memory_order_relaxed, std::memory_order_relaxed))
            {
                return static_cast<std::uint32_t>(value);
            }
        }
    }

    struct LockedEngine
    {
        std::shared_ptr<EngineRecord> record;
        std::unique_lock<std::mutex>   lock;

        LockedEngine() = default;
        explicit LockedEngine(std::shared_ptr<EngineRecord> value)
            : record(std::move(value))
        {
            if (record)
            {
                lock = std::unique_lock<std::mutex>(record->mutex);
                if (record->closed || !record->initialized)
                {
                    lock.unlock();
                    record.reset();
                }
            }
        }

        EngineRecord *get() const
        {
            return record.get();
        }
    };

    LockedEngine lockEngine(AudioEngineHandle handle)
    {
        return LockedEngine(findEngine(handle));
    }

#endif

    // Creates an engine. Description fields left unset (0, or a negative
    // master volume) fall back to the [audio] configuration; noDevice engines
    // require an explicit sample rate and channel count. Fails with
    // InvalidDescription for bad input, CreationFailed when miniaudio cannot
    // initialize the engine or the 32-bit handle space is exhausted.
    AudioEngineHandle createEngineImpl(const AudioEngineDescription *description, AudioError *error)
    {
        setError(error, AudioError::None);
        const AudioEngineDescription  defaults{};
        const AudioEngineDescription &resolved = description != nullptr ? *description : defaults;
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        const std::uint32_t sampleRate = resolved.sampleRate != 0 ? resolved.sampleRate : configuredSampleRate_;
        const std::uint32_t channels = resolved.channels != 0 ? resolved.channels : configuredChannels_;
        const std::uint32_t listenerCount
            = resolved.listenerCount != 0 ? resolved.listenerCount : configuredListenerCount_;
        const std::uint32_t periodSizeInFrames = resolved.periodSizeInFrames != 0
            ? resolved.periodSizeInFrames : configuredPeriodSizeInFrames_;
        const std::uint32_t gainSmoothTimeInFrames = resolved.gainSmoothTimeInFrames != 0
            ? resolved.gainSmoothTimeInFrames : configuredGainSmoothTimeInFrames_;
        const std::uint32_t defaultVolumeSmoothTimeInFrames = resolved.defaultVolumeSmoothTimeInFrames != 0
            ? resolved.defaultVolumeSmoothTimeInFrames : configuredDefaultVolumeSmoothFrames_;
        const std::uint32_t preMixStackSizeInBytes = resolved.preMixStackSizeInBytes != 0
            ? resolved.preMixStackSizeInBytes : configuredPreMixStackSizeInBytes_;
        if (!std::isfinite(resolved.masterVolume))
        {
            setError(error, AudioError::InvalidDescription);
            return invalidAudioEngine;
        }
        const float masterVolume = resolved.masterVolume >= 0.0f ? resolved.masterVolume : configuredMasterVolume_;
        if (!finiteNonNegative(masterVolume) || listenerCount == 0 || listenerCount > MA_ENGINE_MAX_LISTENERS
            || (resolved.noDevice && (sampleRate == 0 || channels == 0)))
        {
            setError(error, AudioError::InvalidDescription);
            return invalidAudioEngine;
        }

        std::lock_guard lifecycleLock(lifecycleMutex_);
        {
            std::lock_guard registryLock(enginesMutex_);
            if (stopping_ || (configuredMaximumEngines_ != 0 && engines_.size() >= configuredMaximumEngines_))
            {
                setError(error, AudioError::CapacityExceeded);
                return invalidAudioEngine;
            }
        }

        std::shared_ptr<EngineRecord> record;
        try
        {
            record = std::make_shared<EngineRecord>();
            record->sounds.reserve(configuredInitialSoundCapacity_);
            record->buses.reserve(configuredInitialBusCapacity_);
            record->busCreationOrder.reserve(configuredInitialBusCapacity_);
        }
        catch (...)
        {
            setError(error, AudioError::CreationFailed);
            return invalidAudioEngine;
        }

        ma_resource_manager *resourceManager = nullptr;
        if (configuredSharedResources_)
        {
            if (!initializeSharedResourceManager())
            {
                setError(error, AudioError::CreationFailed);
                return invalidAudioEngine;
            }
            resourceManager = &sharedResourceManager_;
        }
        else
        {
            const ma_resource_manager_config resourceConfig = makeResourceManagerConfig(
                configuredResourceManagerThreads_, configuredDecodedFormatFloat32_, configuredDecodedChannels_,
                configuredDecodedSampleRate_);
            if (ma_resource_manager_init(&resourceConfig, &record->ownedResourceManager) != MA_SUCCESS)
            {
                setError(error, AudioError::CreationFailed);
                return invalidAudioEngine;
            }
            record->ownsResourceManager = true;
            resourceManager = &record->ownedResourceManager;
        }
        record->resourceManager = resourceManager;

        ma_engine_config config = ma_engine_config_init();
        config.sampleRate = sampleRate;
        config.channels = channels;
        config.listenerCount = listenerCount;
        config.noDevice = resolved.noDevice ? MA_TRUE : MA_FALSE;
        config.noAutoStart = resolved.noAutoStart ? MA_TRUE : MA_FALSE;
        config.pResourceManager = resourceManager;
        config.periodSizeInFrames = periodSizeInFrames;
        config.gainSmoothTimeInFrames = gainSmoothTimeInFrames;
        config.defaultVolumeSmoothTimeInPCMFrames = defaultVolumeSmoothTimeInFrames;
        config.preMixStackSizeInBytes = preMixStackSizeInBytes;

        const ma_result result = ma_engine_init(&config, &record->engine);
        if (result != MA_SUCCESS)
        {
            record->close();
            setError(error, AudioError::CreationFailed);
            std::fprintf(stderr, "audio module: ma_engine_init failed (%d)\\n", static_cast<int>(result));
            return invalidAudioEngine;
        }
        record->initialized = true;
        record->noDevice = resolved.noDevice;
        ma_engine_set_volume(&record->engine, masterVolume);

        const AudioEngineHandle handle = allocateHandle(nextEngine_);
        if (handle == invalidAudioEngine)
        {
            record->close();
            setError(error, AudioError::CreationFailed);
            return invalidAudioEngine;
        }
        {
            std::lock_guard registryLock(enginesMutex_);
            if (stopping_ || (configuredMaximumEngines_ != 0 && engines_.size() >= configuredMaximumEngines_))
            {
                record->close();
                setError(error, AudioError::CapacityExceeded);
                return invalidAudioEngine;
            }
            try
            {
                engines_.emplace(handle, record);
            }
            catch (...)
            {
                record->close();
                setError(error, AudioError::CreationFailed);
                return invalidAudioEngine;
            }
        }
        return handle;
#else
        (void)resolved;
        setError(error, AudioError::BackendUnavailable);
        return invalidAudioEngine;
#endif
    }

    void destroyEngineImpl(AudioEngineHandle handle)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        std::lock_guard lifecycleLock(lifecycleMutex_);
        std::shared_ptr<EngineRecord> record;
        {
            std::lock_guard registryLock(enginesMutex_);
            const auto iterator = engines_.find(handle);
            if (iterator == engines_.end())
            {
                return;
            }
            record = std::move(iterator->second);
            engines_.erase(iterator);
        }
        std::lock_guard engineLock(record->mutex);
        record->close();
#else
        (void)handle;
#endif
    }

    // Reports the engine's actual format (queried from miniaudio), not the
    // creation request.
    bool engineInfoImpl(AudioEngineHandle handle, AudioEngineInfo *outInfo)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (outInfo == nullptr)
        {
            return false;
        }
        LockedEngine engineLock = lockEngine(handle);
        EngineRecord *record = engineLock.get();
        if (record == nullptr)
        {
            return false;
        }
        outInfo->backendName = "miniaudio";
        outInfo->sampleRate  = ma_engine_get_sample_rate(&record->engine);
        outInfo->channels    = ma_engine_get_channels(&record->engine);
        outInfo->hasDevice   = !record->noDevice;
        return true;
#else
        (void)handle;
        (void)outInfo;
        return false;
#endif
    }

    // Starts device output; unsupported for noDevice engines.
    bool startEngineImpl(AudioEngineHandle handle, AudioError *error)
    {
        setError(error, AudioError::None);
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        LockedEngine engineLock = lockEngine(handle);
        EngineRecord *record = engineLock.get();
        if (record == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        if (record->noDevice)
        {
            setError(error, AudioError::Unsupported);
            return false;
        }
        const ma_result result = ma_engine_start(&record->engine);
        if (result != MA_SUCCESS)
        {
            setError(error, AudioError::CreationFailed);
            return false;
        }
        return true;
#else
        (void)handle;
        setError(error, AudioError::BackendUnavailable);
        return false;
#endif
    }

    // Mirrors startEngineImpl; unsupported for noDevice engines.
    bool stopEngineImpl(AudioEngineHandle handle, AudioError *error)
    {
        setError(error, AudioError::None);
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        LockedEngine engineLock = lockEngine(handle);
        EngineRecord *record = engineLock.get();
        if (record == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        if (record->noDevice)
        {
            setError(error, AudioError::Unsupported);
            return false;
        }
        const ma_result result = ma_engine_stop(&record->engine);
        if (result != MA_SUCCESS)
        {
            setError(error, AudioError::CreationFailed);
            return false;
        }
        return true;
#else
        (void)handle;
        setError(error, AudioError::BackendUnavailable);
        return false;
#endif
    }

    // Master gain applied on top of bus/sound gains; must be finite and >= 0.
    bool setMasterVolumeImpl(AudioEngineHandle handle, float volume)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (!finiteNonNegative(volume))
        {
            return false;
        }
        LockedEngine engineLock = lockEngine(handle);
        EngineRecord *record = engineLock.get();
        if (record == nullptr)
        {
            return false;
        }
        ma_engine_set_volume(&record->engine, volume);
        return true;
#else
        (void)handle;
        (void)volume;
        return false;
#endif
    }

    // Adds a mix group under parent (invalidAudioBus = engine master output).
    AudioBusHandle createBusImpl(
        AudioEngineHandle engineHandle, const AudioBusDescription *description, AudioError *error)
    {
        setError(error, AudioError::None);
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (description == nullptr || !finiteNonNegative(description->volume))
        {
            setError(error, AudioError::InvalidDescription);
            return invalidAudioBus;
        }
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return invalidAudioBus;
        }
        if (configuredMaximumBusesPerEngine_ != 0
            && engine->buses.size() >= configuredMaximumBusesPerEngine_)
        {
            setError(error, AudioError::CapacityExceeded);
            return invalidAudioBus;
        }

        ma_sound_group *parentGroup = nullptr;
        BusRecord *parentRecord = nullptr;
        if (description->parent != invalidAudioBus)
        {
            const auto parent = engine->buses.find(description->parent);
            if (parent == engine->buses.end())
            {
                setError(error, AudioError::InvalidHandle);
                return invalidAudioBus;
            }
            parentRecord = &parent->second;
            parentGroup = &parentRecord->group;
        }

        const AudioBusHandle handle = allocateHandle(nextBus_);
        if (handle == invalidAudioBus)
        {
            setError(error, AudioError::CreationFailed);
            return invalidAudioBus;
        }

        try
        {
            auto [iterator, inserted] = engine->buses.try_emplace(handle);
            if (!inserted)
            {
                setError(error, AudioError::CreationFailed);
                return invalidAudioBus;
            }
            BusRecord &bus = iterator->second;
            const ma_result result = ma_sound_group_init(&engine->engine, 0, parentGroup, &bus.group);
            if (result != MA_SUCCESS)
            {
                engine->buses.erase(iterator);
                setError(error, AudioError::CreationFailed);
                return invalidAudioBus;
            }
            bus.initialized = true;
            bus.parent = description->parent;
            bus.volume = description->volume;
            bus.muted = description->muted;
            ma_sound_group_set_volume(&bus.group, bus.muted ? 0.0f : bus.volume);
            engine->busCreationOrder.push_back(handle);
            if (parentRecord != nullptr)
            {
                ++parentRecord->childBusCount;
            }
        }
        catch (...)
        {
            engine->buses.erase(handle);
            setError(error, AudioError::CreationFailed);
            return invalidAudioBus;
        }
        return handle;
#else
        (void)engineHandle;
        (void)description;
        setError(error, AudioError::BackendUnavailable);
        return invalidAudioBus;
#endif
    }

    // Refuses to delete a bus that still has child buses or attached sounds
    // (Busy); callers must reparent or delete those first.
    bool destroyBusImpl(AudioEngineHandle engineHandle, AudioBusHandle busHandle, AudioError *error)
    {
        setError(error, AudioError::None);
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        const auto bus = engine->buses.find(busHandle);
        if (bus == engine->buses.end())
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        if (bus->second.childBusCount != 0 || bus->second.attachedSounds != 0)
        {
            setError(error, AudioError::Busy);
            return false;
        }
        const AudioBusHandle parentHandle = bus->second.parent;
        engine->buses.erase(bus);
        if (parentHandle != invalidAudioBus)
        {
            const auto parent = engine->buses.find(parentHandle);
            if (parent != engine->buses.end() && parent->second.childBusCount > 0)
            {
                --parent->second.childBusCount;
            }
        }
        if (engine->busCreationOrder.size() > engine->buses.size() * 2 + 64)
        {
            auto &order = engine->busCreationOrder;
            order.erase(std::remove_if(order.begin(), order.end(), [engine](AudioBusHandle value)
                { return engine->buses.find(value) == engine->buses.end(); }), order.end());
        }
        return true;
#else
        (void)engineHandle;
        (void)busHandle;
        setError(error, AudioError::BackendUnavailable);
        return false;
#endif
    }

    bool setBusVolumeImpl(AudioEngineHandle engineHandle, AudioBusHandle busHandle, float volume)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (!finiteNonNegative(volume))
        {
            return false;
        }
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            return false;
        }
        const auto bus = engine->buses.find(busHandle);
        if (bus == engine->buses.end())
        {
            return false;
        }
        bus->second.volume = volume;
        ma_sound_group_set_volume(&bus->second.group, bus->second.muted ? 0.0f : volume);
        return true;
#else
        (void)engineHandle;
        (void)busHandle;
        (void)volume;
        return false;
#endif
    }

    bool setBusMutedImpl(AudioEngineHandle engineHandle, AudioBusHandle busHandle, bool muted)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            return false;
        }
        const auto bus = engine->buses.find(busHandle);
        if (bus == engine->buses.end())
        {
            return false;
        }
        bus->second.muted = muted;
        ma_sound_group_set_volume(&bus->second.group, muted ? 0.0f : bus->second.volume);
        return true;
#else
        (void)engineHandle;
        (void)busHandle;
        (void)muted;
        return false;
#endif
    }

    // Loads a sound from filePath after validating the whole description
    // (streaming/decode are mutually exclusive; pan is in [-1, 1]; distances
    // positive with max >= min). ResourceFailed covers missing or undecodable
    // files; the sound is attached to the requested bus, if any.
    AudioSoundHandle createSoundImpl(
        AudioEngineHandle engineHandle, const AudioSoundDescription *description, AudioError *error)
    {
        setError(error, AudioError::None);
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (description == nullptr || description->filePath == nullptr || description->filePath[0] == '\0'
            || !finiteNonNegative(description->volume) || !finitePositive(description->pitch)
            || !std::isfinite(description->pan) || description->pan < -1.0f || description->pan > 1.0f
            || !finiteVector(description->position) || !finiteVector(description->velocity)
            || !finitePositive(description->minDistance) || !finitePositive(description->maxDistance)
            || description->maxDistance < description->minDistance || !finiteNonNegative(description->rolloff)
            || !finiteNonNegative(description->minimumGain) || !finiteNonNegative(description->maximumGain)
            || description->maximumGain < description->minimumGain || !finiteNonNegative(description->dopplerFactor)
            || (description->spatialized && !finiteNonZeroVector(description->direction))
            || (description->directionalCone && !validSoundCone(description->coneInnerAngleRadians,
                description->coneOuterAngleRadians, description->coneOuterGain))
            || (!description->spatialized
                && (description->directionalCone || description->listenerIndex != invalidAudioListener))
            || (description->streaming && description->decode)
            || static_cast<std::uint32_t>(description->attenuation)
                   > static_cast<std::uint32_t>(AudioAttenuationModel::Exponential))
        {
            setError(error, AudioError::InvalidDescription);
            return invalidAudioSound;
        }
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return invalidAudioSound;
        }
        if (description->listenerIndex != invalidAudioListener
            && description->listenerIndex >= ma_engine_get_listener_count(&engine->engine))
        {
            setError(error, AudioError::InvalidDescription);
            return invalidAudioSound;
        }
        if (configuredMaximumSoundsPerEngine_ != 0
            && engine->sounds.size() >= configuredMaximumSoundsPerEngine_)
        {
            setError(error, AudioError::CapacityExceeded);
            return invalidAudioSound;
        }

        ma_sound_group *group = nullptr;
        auto bus = engine->buses.end();
        if (description->bus != invalidAudioBus)
        {
            bus = engine->buses.find(description->bus);
            if (bus == engine->buses.end())
            {
                setError(error, AudioError::InvalidHandle);
                return invalidAudioSound;
            }
            group = &bus->second.group;
        }

        ma_uint32 flags = 0;
        if (description->streaming)
            flags |= MA_SOUND_FLAG_STREAM;
        if (description->decode)
            flags |= MA_SOUND_FLAG_DECODE;
        if (description->asynchronous)
            flags |= MA_SOUND_FLAG_ASYNC;
        if (!description->spatialized)
            flags |= MA_SOUND_FLAG_NO_SPATIALIZATION;
        if (description->disablePitch)
            flags |= MA_SOUND_FLAG_NO_PITCH;

        const AudioSoundHandle handle = allocateHandle(nextSound_);
        if (handle == invalidAudioSound)
        {
            setError(error, AudioError::CreationFailed);
            return invalidAudioSound;
        }

        try
        {
            auto [iterator, inserted] = engine->sounds.try_emplace(handle);
            if (!inserted)
            {
                setError(error, AudioError::CreationFailed);
                return invalidAudioSound;
            }
            SoundRecord &sound = iterator->second;
            const ma_result result = ma_sound_init_from_file(
                &engine->engine, description->filePath, flags, group, nullptr, &sound.sound);
            if (result != MA_SUCCESS)
            {
                engine->sounds.erase(iterator);
                setError(error, AudioError::ResourceFailed);
                std::fprintf(stderr, "audio module: failed to load '%s' (%d)\\n", description->filePath,
                    static_cast<int>(result));
                return invalidAudioSound;
            }
            sound.initialized = true;
            sound.bus = description->bus;
            ma_sound_set_volume(&sound.sound, description->volume);
            ma_sound_set_pitch(&sound.sound, description->pitch);
            ma_sound_set_pan(&sound.sound, description->pan);
            ma_sound_set_looping(&sound.sound, description->looping ? MA_TRUE : MA_FALSE);
            if (description->spatialized)
            {
                ma_sound_set_position(&sound.sound, description->position[0], description->position[1],
                    description->position[2]);
                ma_sound_set_velocity(&sound.sound, description->velocity[0], description->velocity[1],
                    description->velocity[2]);
                ma_sound_set_min_distance(&sound.sound, description->minDistance);
                ma_sound_set_max_distance(&sound.sound, description->maxDistance);
                ma_sound_set_rolloff(&sound.sound, description->rolloff);
                ma_sound_set_attenuation_model(&sound.sound, nativeAttenuation(description->attenuation));
                float direction[3] = {description->direction[0], description->direction[1], description->direction[2]};
                normalizeVector(direction);
                ma_sound_set_direction(&sound.sound, direction[0], direction[1], direction[2]);
                ma_sound_set_min_gain(&sound.sound, description->minimumGain);
                ma_sound_set_max_gain(&sound.sound, description->maximumGain);
                ma_sound_set_doppler_factor(&sound.sound, description->dopplerFactor);
                if (description->directionalCone)
                {
                    ma_sound_set_cone(&sound.sound, description->coneInnerAngleRadians,
                        description->coneOuterAngleRadians, description->coneOuterGain);
                }
                if (description->listenerIndex != invalidAudioListener)
                    ma_sound_set_pinned_listener_index(&sound.sound, description->listenerIndex);
            }
            if (bus != engine->buses.end())
            {
                ++bus->second.attachedSounds;
            }
        }
        catch (...)
        {
            engine->sounds.erase(handle);
            setError(error, AudioError::CreationFailed);
            return invalidAudioSound;
        }
        return handle;
#else
        (void)engineHandle;
        (void)description;
        setError(error, AudioError::BackendUnavailable);
        return invalidAudioSound;
#endif
    }

    void destroySoundImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            return;
        }
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
        {
            return;
        }
        const AudioBusHandle busHandle = sound->second.bus;
        engine->sounds.erase(sound);
        if (busHandle != invalidAudioBus)
        {
            const auto bus = engine->buses.find(busHandle);
            if (bus != engine->buses.end() && bus->second.attachedSounds > 0)
            {
                --bus->second.attachedSounds;
            }
        }
#else
        (void)engineHandle;
        (void)soundHandle;
#endif
    }

#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
    // Shared runner for play/pause-style actions: resolve both handles, then
    // apply a single ma_sound call. Backend failures map to Unsupported.
    bool soundAction(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle, AudioError *error,
        ma_result (*action)(ma_sound *))
    {
        setError(error, AudioError::None);
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        const ma_result result = action(&sound->second.sound);
        if (result != MA_SUCCESS)
        {
            setError(error, AudioError::Unsupported);
            return false;
        }
        return true;
    }
#endif

    bool playImpl(AudioEngineHandle engine, AudioSoundHandle sound, AudioError *error)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        return soundAction(engine, sound, error, ma_sound_start);
#else
        (void)engine;
        (void)sound;
        setError(error, AudioError::BackendUnavailable);
        return false;
#endif
    }

    // Pause is ma_sound_stop without rewinding; play resumes from the cursor.
    bool pauseImpl(AudioEngineHandle engine, AudioSoundHandle sound, AudioError *error)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        return soundAction(engine, sound, error, ma_sound_stop);
#else
        (void)engine;
        (void)sound;
        setError(error, AudioError::BackendUnavailable);
        return false;
#endif
    }

    bool stopImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle, AudioError *error)
    {
        setError(error, AudioError::None);
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        // Unlike pause, stop also rewinds to PCM frame 0.
        const ma_result stopResult = ma_sound_stop(&sound->second.sound);
        const ma_result seekResult = ma_sound_seek_to_pcm_frame(&sound->second.sound, 0);
        if (stopResult != MA_SUCCESS || seekResult != MA_SUCCESS)
        {
            setError(error, AudioError::Unsupported);
            return false;
        }
        return true;
#else
        (void)engineHandle;
        (void)soundHandle;
        setError(error, AudioError::BackendUnavailable);
        return false;
#endif
    }

    // Looks up the sound's data-format sample rate, then converts seconds to
    // a PCM frame index (long double so huge values overflow-check cleanly).
    bool seekSecondsImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle, double seconds)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (!std::isfinite(seconds) || seconds < 0.0)
        {
            return false;
        }
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            return false;
        }
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
        {
            return false;
        }
        ma_format format     = ma_format_unknown;
        ma_uint32 channels   = 0;
        ma_uint32 sampleRate = 0;
        if (ma_sound_get_data_format(&sound->second.sound, &format, &channels, &sampleRate, nullptr, 0) != MA_SUCCESS
            || sampleRate == 0)
        {
            return false;
        }
        const long double frame = static_cast<long double>(seconds) * static_cast<long double>(sampleRate);
        if (frame > static_cast<long double>(std::numeric_limits<ma_uint64>::max()))
        {
            return false;
        }
        return ma_sound_seek_to_pcm_frame(&sound->second.sound, static_cast<ma_uint64>(frame)) == MA_SUCCESS;
#else
        (void)engineHandle;
        (void)soundHandle;
        (void)seconds;
        return false;
#endif
    }

    bool isPlayingImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
            return false;
        const auto sound = engine->sounds.find(soundHandle);
        return sound != engine->sounds.end() && ma_sound_is_playing(&sound->second.sound) == MA_TRUE;
#else
        (void)engineHandle;
        (void)soundHandle;
        return false;
#endif
    }

    bool atEndImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
            return false;
        const auto sound = engine->sounds.find(soundHandle);
        return sound != engine->sounds.end() && ma_sound_at_end(&sound->second.sound) == MA_TRUE;
#else
        (void)engineHandle;
        (void)soundHandle;
        return false;
#endif
    }

    // Reports cursor and total length in seconds; fails when the sound's
    // data format is unknown.
    bool soundTimesImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle, double *outCursorSeconds,
        double *outLengthSeconds)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (outCursorSeconds == nullptr || outLengthSeconds == nullptr)
            return false;
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
            return false;
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
            return false;
        ma_format format     = ma_format_unknown;
        ma_uint32 channels   = 0;
        ma_uint32 sampleRate = 0;
        ma_uint64 cursor     = 0;
        ma_uint64 length     = 0;
        if (ma_sound_get_data_format(&sound->second.sound, &format, &channels, &sampleRate, nullptr, 0) != MA_SUCCESS
            || sampleRate == 0 || ma_sound_get_cursor_in_pcm_frames(&sound->second.sound, &cursor) != MA_SUCCESS
            || ma_sound_get_length_in_pcm_frames(&sound->second.sound, &length) != MA_SUCCESS)
        {
            return false;
        }
        const double rate = static_cast<double>(sampleRate);
        *outCursorSeconds = static_cast<double>(cursor) / rate;
        *outLengthSeconds = static_cast<double>(length) / rate;
        return true;
#else
        (void)engineHandle;
        (void)soundHandle;
        (void)outCursorSeconds;
        (void)outLengthSeconds;
        return false;
#endif
    }

    bool setSoundVolumeImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle, float volume)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (!finiteNonNegative(volume))
            return false;
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
            return false;
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
            return false;
        ma_sound_set_volume(&sound->second.sound, volume);
        return true;
#else
        (void)engineHandle;
        (void)soundHandle;
        (void)volume;
        return false;
#endif
    }

    bool setSoundPitchImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle, float pitch)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (!finitePositive(pitch))
            return false;
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
            return false;
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
            return false;
        ma_sound_set_pitch(&sound->second.sound, pitch);
        return true;
#else
        (void)engineHandle;
        (void)soundHandle;
        (void)pitch;
        return false;
#endif
    }

    bool setSoundPanImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle, float pan)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (!std::isfinite(pan) || pan < -1.0f || pan > 1.0f)
            return false;
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
            return false;
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
            return false;
        ma_sound_set_pan(&sound->second.sound, pan);
        return true;
#else
        (void)engineHandle;
        (void)soundHandle;
        (void)pan;
        return false;
#endif
    }

    bool setSoundLoopingImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle, bool looping)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
            return false;
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
            return false;
        ma_sound_set_looping(&sound->second.sound, looping ? MA_TRUE : MA_FALSE);
        return true;
#else
        (void)engineHandle;
        (void)soundHandle;
        (void)looping;
        return false;
#endif
    }

    bool setSoundPositionImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle, const float position[3])
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (!finiteVector(position))
            return false;
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
            return false;
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
            return false;
        ma_sound_set_position(&sound->second.sound, position[0], position[1], position[2]);
        return true;
#else
        (void)engineHandle;
        (void)soundHandle;
        (void)position;
        return false;
#endif
    }

    bool setSoundVelocityImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle, const float velocity[3])
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (!finiteVector(velocity))
            return false;
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
            return false;
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
            return false;
        ma_sound_set_velocity(&sound->second.sound, velocity[0], velocity[1], velocity[2]);
        return true;
#else
        (void)engineHandle;
        (void)soundHandle;
        (void)velocity;
        return false;
#endif
    }

    bool setListenerImpl(
        AudioEngineHandle engineHandle, std::uint32_t listenerIndex, const AudioListenerDescription *description)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (description == nullptr || !finiteVector(description->position) || !finiteVector(description->direction)
            || !finiteVector(description->worldUp) || !finiteVector(description->velocity))
        {
            return false;
        }
        // Listener index must be below the engine's listener count.
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr || listenerIndex >= ma_engine_get_listener_count(&engine->engine))
            return false;
        ma_engine_listener_set_position(&engine->engine,
            listenerIndex,
            description->position[0],
            description->position[1],
            description->position[2]);
        ma_engine_listener_set_direction(&engine->engine,
            listenerIndex,
            description->direction[0],
            description->direction[1],
            description->direction[2]);
        ma_engine_listener_set_world_up(
            &engine->engine, listenerIndex, description->worldUp[0], description->worldUp[1], description->worldUp[2]);
        ma_engine_listener_set_velocity(&engine->engine,
            listenerIndex,
            description->velocity[0],
            description->velocity[1],
            description->velocity[2]);
        return true;
#else
        (void)engineHandle;
        (void)listenerIndex;
        (void)description;
        return false;
#endif
    }

    // Manual rendering is only allowed for noDevice engines; engines with a
    // real device render on their own audio thread instead.
    bool readFramesImpl(AudioEngineHandle engineHandle, float *output, std::uint64_t frameCount,
        std::uint64_t *outFramesRead, AudioError *error)
    {
        setError(error, AudioError::None);
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (output == nullptr || frameCount == 0)
        {
            setError(error, AudioError::InvalidDescription);
            return false;
        }
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        if (!engine->noDevice)
        {
            setError(error, AudioError::Unsupported);
            return false;
        }
        ma_uint64       framesRead = 0;
        const ma_result result
            = ma_engine_read_pcm_frames(&engine->engine, output, static_cast<ma_uint64>(frameCount), &framesRead);
        if (outFramesRead != nullptr)
            *outFramesRead = static_cast<std::uint64_t>(framesRead);
        if (result != MA_SUCCESS)
        {
            setError(error, AudioError::ResourceFailed);
            return false;
        }
        return true;
#else
        (void)engineHandle;
        (void)output;
        (void)frameCount;
        (void)outFramesRead;
        setError(error, AudioError::BackendUnavailable);
        return false;
#endif
    }

#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
    bool fadeSoundGainImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle, float targetGain,
        double durationSeconds, AudioError *error)
    {
        setError(error, AudioError::None);
        if (!finiteNonNegative(targetGain) || !std::isfinite(durationSeconds) || durationSeconds < 0.0)
        {
            setError(error, AudioError::InvalidDescription);
            return false;
        }
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        ma_uint64 fadeFrames = 0;
        if (!secondsToFrameOffset(durationSeconds, ma_engine_get_sample_rate(&engine->engine), fadeFrames))
        {
            setError(error, AudioError::InvalidDescription);
            return false;
        }
        ma_sound_set_fade_in_pcm_frames(&sound->second.sound, -1.0f, targetGain, fadeFrames);
        return true;
    }

    bool schedulePlayImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle,
        double delaySeconds, AudioError *error)
    {
        setError(error, AudioError::None);
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        ma_uint64 delayFrames = 0;
        if (!secondsToFrameOffset(delaySeconds, ma_engine_get_sample_rate(&engine->engine), delayFrames))
        {
            setError(error, AudioError::InvalidDescription);
            return false;
        }
        const ma_uint64 currentTime = ma_engine_get_time_in_pcm_frames(&engine->engine);
        if (delayFrames > std::numeric_limits<ma_uint64>::max() - currentTime)
        {
            setError(error, AudioError::InvalidDescription);
            return false;
        }
        ma_sound_set_start_time_in_pcm_frames(&sound->second.sound, currentTime + delayFrames);
        if (ma_sound_start(&sound->second.sound) != MA_SUCCESS)
        {
            setError(error, AudioError::Unsupported);
            return false;
        }
        return true;
    }

    bool scheduleStopImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle,
        double delaySeconds, double fadeSeconds, AudioError *error)
    {
        setError(error, AudioError::None);
        if (!std::isfinite(delaySeconds) || delaySeconds < 0.0 || !std::isfinite(fadeSeconds)
            || fadeSeconds < 0.0 || fadeSeconds > delaySeconds)
        {
            setError(error, AudioError::InvalidDescription);
            return false;
        }
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        const std::uint32_t sampleRate = ma_engine_get_sample_rate(&engine->engine);
        ma_uint64 delayFrames = 0;
        ma_uint64 fadeFrames = 0;
        if (!secondsToFrameOffset(delaySeconds, sampleRate, delayFrames)
            || !secondsToFrameOffset(fadeSeconds, sampleRate, fadeFrames))
        {
            setError(error, AudioError::InvalidDescription);
            return false;
        }
        const ma_uint64 currentTime = ma_engine_get_time_in_pcm_frames(&engine->engine);
        if (delayFrames > std::numeric_limits<ma_uint64>::max() - currentTime)
        {
            setError(error, AudioError::InvalidDescription);
            return false;
        }
        ma_sound_set_stop_time_with_fade_in_pcm_frames(
            &sound->second.sound, currentTime + delayFrames, fadeFrames);
        return true;
    }

    bool setSoundDirectionImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle,
        const float directionInput[3], AudioError *error)
    {
        setError(error, AudioError::None);
        if (!finiteNonZeroVector(directionInput))
        {
            setError(error, AudioError::InvalidDescription);
            return false;
        }
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        float direction[3] = {directionInput[0], directionInput[1], directionInput[2]};
        normalizeVector(direction);
        ma_sound_set_direction(&sound->second.sound, direction[0], direction[1], direction[2]);
        return true;
    }

    bool setSoundConeImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle,
        float innerAngleRadians, float outerAngleRadians, float outerGain, AudioError *error)
    {
        setError(error, AudioError::None);
        if (!validSoundCone(innerAngleRadians, outerAngleRadians, outerGain))
        {
            setError(error, AudioError::InvalidDescription);
            return false;
        }
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        ma_sound_set_cone(&sound->second.sound, innerAngleRadians, outerAngleRadians, outerGain);
        return true;
    }

    bool setSoundDopplerFactorImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle,
        float factor, AudioError *error)
    {
        setError(error, AudioError::None);
        if (!finiteNonNegative(factor))
        {
            setError(error, AudioError::InvalidDescription);
            return false;
        }
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        ma_sound_set_doppler_factor(&sound->second.sound, factor);
        return true;
    }

    bool setSoundListenerImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle,
        std::uint32_t listenerIndex, AudioError *error)
    {
        setError(error, AudioError::None);
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        if (listenerIndex == invalidAudioListener)
        {
            // miniaudio permits pinning to an explicit listener but exposes no
            // public operation to restore closest-listener selection afterward.
            setError(error, AudioError::Unsupported);
            return false;
        }
        if (listenerIndex >= ma_engine_get_listener_count(&engine->engine))
        {
            setError(error, AudioError::InvalidDescription);
            return false;
        }
        ma_sound_set_pinned_listener_index(&sound->second.sound, static_cast<ma_uint32>(listenerIndex));
        return true;
    }

    bool setSoundSpatializationEnabledImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle,
        bool enabled, AudioError *error)
    {
        setError(error, AudioError::None);
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        ma_sound_set_spatialization_enabled(&sound->second.sound, enabled ? MA_TRUE : MA_FALSE);
        return true;
    }

    bool setSoundGainRangeImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle,
        float minimumGain, float maximumGain, AudioError *error)
    {
        setError(error, AudioError::None);
        if (!finiteNonNegative(minimumGain) || !finiteNonNegative(maximumGain) || maximumGain < minimumGain)
        {
            setError(error, AudioError::InvalidDescription);
            return false;
        }
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        ma_sound_set_min_gain(&sound->second.sound, minimumGain);
        ma_sound_set_max_gain(&sound->second.sound, maximumGain);
        return true;
    }

    bool setSoundAttenuationImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle,
        float minimumDistance, float maximumDistance, float rolloff, AudioAttenuationModel model,
        AudioError *error)
    {
        setError(error, AudioError::None);
        if (!finitePositive(minimumDistance) || !finitePositive(maximumDistance)
            || maximumDistance < minimumDistance || !finiteNonNegative(rolloff)
            || static_cast<std::uint32_t>(model) > static_cast<std::uint32_t>(AudioAttenuationModel::Exponential))
        {
            setError(error, AudioError::InvalidDescription);
            return false;
        }
        LockedEngine engineLock = lockEngine(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        const auto sound = engine->sounds.find(soundHandle);
        if (sound == engine->sounds.end())
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        ma_sound_set_min_distance(&sound->second.sound, minimumDistance);
        ma_sound_set_max_distance(&sound->second.sound, maximumDistance);
        ma_sound_set_rolloff(&sound->second.sound, rolloff);
        ma_sound_set_attenuation_model(&sound->second.sound, nativeAttenuation(model));
        return true;
    }
#else
    bool fadeSoundGainImpl(AudioEngineHandle, AudioSoundHandle, float, double, AudioError *error)
    { setError(error, AudioError::BackendUnavailable); return false; }
    bool schedulePlayImpl(AudioEngineHandle, AudioSoundHandle, double, AudioError *error)
    { setError(error, AudioError::BackendUnavailable); return false; }
    bool scheduleStopImpl(AudioEngineHandle, AudioSoundHandle, double, double, AudioError *error)
    { setError(error, AudioError::BackendUnavailable); return false; }
    bool setSoundDirectionImpl(AudioEngineHandle, AudioSoundHandle, const float[3], AudioError *error)
    { setError(error, AudioError::BackendUnavailable); return false; }
    bool setSoundConeImpl(AudioEngineHandle, AudioSoundHandle, float, float, float, AudioError *error)
    { setError(error, AudioError::BackendUnavailable); return false; }
    bool setSoundDopplerFactorImpl(AudioEngineHandle, AudioSoundHandle, float, AudioError *error)
    { setError(error, AudioError::BackendUnavailable); return false; }
    bool setSoundListenerImpl(AudioEngineHandle, AudioSoundHandle, std::uint32_t, AudioError *error)
    { setError(error, AudioError::BackendUnavailable); return false; }
    bool setSoundSpatializationEnabledImpl(AudioEngineHandle, AudioSoundHandle, bool, AudioError *error)
    { setError(error, AudioError::BackendUnavailable); return false; }
    bool setSoundGainRangeImpl(AudioEngineHandle, AudioSoundHandle, float, float, AudioError *error)
    { setError(error, AudioError::BackendUnavailable); return false; }
    bool setSoundAttenuationImpl(AudioEngineHandle, AudioSoundHandle, float, float, float,
        AudioAttenuationModel, AudioError *error)
    { setError(error, AudioError::BackendUnavailable); return false; }
#endif

    ParticipantManifest manifest_;
    AudioCapability capability_{};
    script_runtime::ScriptRuntimeCapability *scriptRuntime_ = nullptr;
    script_runtime::ScriptRuntimeInitializerHandle runtimeInitializer_ = script_runtime::invalidScriptRuntimeInitializer;
    AudioLuaBridgeContext luaBridgeContext_{};

    // Module-level defaults captured once in construct(). Zero capacities are
    // unbounded; initial capacities reserve hash-map buckets without imposing
    // a hard limit.
    std::uint32_t configuredSampleRate_             = 0;
    std::uint32_t configuredChannels_               = 0;
    std::uint32_t configuredListenerCount_          = 1;
    float         configuredMasterVolume_           = 1.0f;
    bool          configuredSharedResources_        = false;
    std::uint32_t configuredResourceManagerThreads_ = 1;
    bool          configuredDecodedFormatFloat32_   = false;
    std::uint32_t configuredDecodedChannels_        = 0;
    std::uint32_t configuredDecodedSampleRate_      = 0;
    std::uint32_t configuredInitialEngineCapacity_  = 4;
    std::uint32_t configuredInitialSoundCapacity_   = 64;
    std::uint32_t configuredInitialBusCapacity_     = 8;
    std::uint32_t configuredMaximumEngines_         = 0;
    std::uint32_t configuredMaximumSoundsPerEngine_ = 0;
    std::uint32_t configuredMaximumBusesPerEngine_  = 0;
    std::uint32_t configuredPeriodSizeInFrames_ = 0;
    std::uint32_t configuredGainSmoothTimeInFrames_ = 0;
    std::uint32_t configuredDefaultVolumeSmoothFrames_ = 0;
    std::uint32_t configuredPreMixStackSizeInBytes_ = 0;

#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
    // Lifecycle operations serialize create/destroy/stop. Registry and
    // per-engine mutexes are not held simultaneously by ordinary operations.
    std::mutex lifecycleMutex_;
    std::mutex enginesMutex_;
    std::unordered_map<AudioEngineHandle, std::shared_ptr<EngineRecord>> engines_;
    std::mutex sharedResourceManagerMutex_;
    ma_resource_manager sharedResourceManager_{};
    bool sharedResourceManagerInitialized_ = false;
    bool stopping_ = false;

    // 64-bit counters permanently exhaust instead of wrapping into reused
    // 32-bit public handles.
    std::atomic<std::uint64_t> nextEngine_{1};
    std::atomic<std::uint64_t> nextSound_{1};
    std::atomic<std::uint64_t> nextBus_{1};
#endif
};

} // namespace
} // namespace audio

} // namespace engine

// Module entry table: the one exported symbol the kernel loads. Declares the
// module name/version and the provided "audio" capability; script_runtime is
// an optional capability used only for automatic Lua bindings.
extern "C" ENGINE_MODULE_EXPORT const engine::EngineModuleEntryTable *engineQueryModuleEntryTable()
{
    static const char *const providedCapabilities[] = {"audio"};
    static const char *const optionalCapabilities[] = {"script_runtime"};
    static const engine::EngineModuleEntryTable entry = {
        .abiVersion              = engine::engineModuleAbiVersion,
        .moduleName              = "audio",
        .moduleVersion           = "0.3.0",
        .createModule            = []() -> engine::ModuleInterface                             *{ return new engine::audio::AudioModule(); },
        .providedCapabilities    = providedCapabilities,
        .providedCapabilityCount = 1,
        .requiredCapabilities    = nullptr,
        .requiredCapabilityCount = 0,
        .optionalCapabilities    = optionalCapabilities,
        .optionalCapabilityCount = 1,
        .declaredPermissions     = nullptr,
        .declaredPermissionCount = 0,
    };
    return &entry;
}
