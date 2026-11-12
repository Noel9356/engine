// audio module implementation.
//
// Wraps miniaudio (compiled in only when ENGINE_AUDIO_BACKEND_MINIAUDIO is
// defined) behind the "audio" capability: engines, mix buses, and sounds with
// optional 3D spatialization, plus manual rendering for device-less engines.
// This file owns configuration, capability binding, and the module lifecycle;
// the record types and lock helpers live in source/audio_backend.hpp and the
// engine registry lives in source/engine_registry.*. Without the backend
// every entry fails with AudioError::BackendUnavailable.

#include <engine/audio/capabilities.hpp>
#include "audio_backend.hpp"
#include "lua_bridge.hpp"
#include <engine/kernel/module_api.hpp>
#include <engine/kernel/participant_interface.hpp>
#include <engine/kernel/participant_manifest.hpp>
#include <engine/util/hardware_info.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>

#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
#include "engine_registry.hpp"

#include <memory>
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

// Backend helpers and state types; the configuration struct exists in every
// build, the miniaudio-dependent types only with the backend.
using namespace backend;

// Module singleton backing the "audio" capability. Owns the resolved
// configuration, the engine registry, and the optional script bridge; the
// per-engine state and its locking stay on the records and the registry.
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
        configuration_ = AudioConfiguration{};

        // auto_config = true: the module derives its tuning from the host
        // hardware and the other [audio] tuning keys have no effect. The
        // content keys (master_volume, listener_count) describe the mix, not
        // the machine, and stay effective in both modes. Hardware inputs
        // today: the logical processor count shapes the resource-manager
        // worker pool; the output/decode format keys resolve to their
        // device-native defaults (0 / "native") because the module cannot
        // know the playback device better than miniaudio does.
        const bool autoConfig = context.configurationBooleanInSection("audio", "auto_config").value_or(false);

        if (const auto value = context.configurationIntegerInSection("audio", "listener_count");
            value && *value >= 1 && static_cast<std::uint64_t>(*value) <= maximumSupportedListenerCount())
        {
            configuration_.listenerCount = static_cast<std::uint32_t>(*value);
        }
        if (const auto value = context.configurationFloatingInSection("audio", "master_volume");
            value && std::isfinite(*value) && *value >= 0.0)
        {
            configuration_.masterVolume = static_cast<float>(*value);
        }

        if (autoConfig)
        {
            const std::uint32_t cores = util::logicalProcessorCount();
            configuration_.resourceManagerThreads = cores >= 16u ? 4u : cores >= 8u ? 3u : cores >= 4u ? 2u : 1u;
            std::fprintf(stderr,
                "audio module: auto_config is on; resource_manager_job_threads=%u derived from %u logical "
                "processor(s), device-native output/decode formats; other tuning keys in [audio] are ignored\n",
                configuration_.resourceManagerThreads,
                cores);
        }
        else
        {
            // Manual path: overlay [audio] values that validate; out-of-range
            // keys are ignored.

            // A zero sample rate/channel count asks the selected playback
            // device for its native format. Device-less engines must set both
            // explicitly.
            if (const auto value = context.configurationIntegerInSection("audio", "sample_rate");
                value && *value >= 0 && *value <= std::numeric_limits<std::uint32_t>::max())
            {
                configuration_.sampleRate = static_cast<std::uint32_t>(*value);
            }
            if (const auto value = context.configurationIntegerInSection("audio", "channels");
                value && *value >= 0 && *value <= 254)
            {
                configuration_.channels = static_cast<std::uint32_t>(*value);
            }

            if (const auto value = context.configurationBooleanInSection("audio", "share_resources"); value)
            {
                configuration_.sharedResources = *value;
            }
            if (const auto value = context.configurationIntegerInSection("audio", "resource_manager_job_threads");
                value && *value >= 1 && *value <= 64)
            {
                configuration_.resourceManagerThreads = static_cast<std::uint32_t>(*value);
            }
            if (const auto value = context.configurationStringInSection("audio", "decoded_format"); value)
            {
                if (*value == "f32" || *value == "float32")
                {
                    configuration_.decodedFormatFloat32 = true;
                }
                else if (*value == "native")
                {
                    configuration_.decodedFormatFloat32 = false;
                }
            }
            if (const auto value = context.configurationIntegerInSection("audio", "decoded_channels");
                value && *value >= 0 && *value <= 32)
            {
                configuration_.decodedChannels = static_cast<std::uint32_t>(*value);
            }
            if (const auto value = context.configurationIntegerInSection("audio", "decoded_sample_rate");
                value && *value >= 0 && (*value == 0 || (*value >= 8000 && *value <= 384000)))
            {
                configuration_.decodedSampleRate = static_cast<std::uint32_t>(*value);
            }

            const auto readCapacity = [&context, this](const char *key, std::uint32_t &target, std::uint32_t maximum)
            {
                if (const auto value = context.configurationIntegerInSection("audio", key);
                    value && *value >= 0 && *value <= maximum)
                {
                    target = static_cast<std::uint32_t>(*value);
                }
            };
            readCapacity("initial_engine_capacity", configuration_.initialEngineCapacity, 65536);
            readCapacity("initial_sound_capacity", configuration_.initialSoundCapacity, 1048576);
            readCapacity("initial_bus_capacity", configuration_.initialBusCapacity, 65536);
            readCapacity("max_engines", configuration_.maximumEngines, 1048576);
            readCapacity("max_sounds_per_engine", configuration_.maximumSoundsPerEngine, 1048576);
            readCapacity("max_buses_per_engine", configuration_.maximumBusesPerEngine, 65536);
            readCapacity("period_size_in_frames", configuration_.periodSizeInFrames,
                std::numeric_limits<std::uint32_t>::max());
            readCapacity("gain_smooth_time_in_frames", configuration_.gainSmoothTimeInFrames,
                std::numeric_limits<std::uint32_t>::max());
            readCapacity("default_volume_smooth_time_in_frames", configuration_.defaultVolumeSmoothTimeInFrames,
                std::numeric_limits<std::uint32_t>::max());
            readCapacity("pre_mix_stack_size_in_bytes", configuration_.preMixStackSizeInBytes,
                std::numeric_limits<std::uint32_t>::max());
        }

#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        registry_.open(configuration_.initialEngineCapacity);
#endif

        scriptRuntime_ = context.tryAcquireCapability<script_runtime::ScriptRuntimeCapability>();
        if (script_runtime::canRegisterRuntimeInitializer(scriptRuntime_))
        {
            luaBridgeContext_ = AudioLuaBridgeContext{scriptRuntime_, &capability_};
            runtimeInitializer_ = script_runtime::tryRegisterRuntimeInitializer(
                scriptRuntime_, initializeAudioLuaRuntime, &luaBridgeContext_);
        }
    }

    void start() override
    {}

    void stop() override
    {
        script_runtime::unregisterRuntimeInitializer(scriptRuntime_, runtimeInitializer_);
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        const auto lifecycleLock = registry_.lifecycleLock();
        registry_.closeAll();
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
            .version              = "0.1.0",
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
        ENGINE_AUDIO_BIND(engineStatistics, engineStatisticsImpl);
        ENGINE_AUDIO_BIND(startEngine, startEngineImpl);
        ENGINE_AUDIO_BIND(stopEngine, stopEngineImpl);
        ENGINE_AUDIO_BIND(isEngineStarted, isEngineStartedImpl);
        ENGINE_AUDIO_BIND(setMasterVolume, setMasterVolumeImpl);
        ENGINE_AUDIO_BIND(getMasterVolume, getMasterVolumeImpl);
        ENGINE_AUDIO_BIND(createBus, createBusImpl);
        ENGINE_AUDIO_BIND(destroyBus, destroyBusImpl);
        ENGINE_AUDIO_BIND(setBusVolume, setBusVolumeImpl);
        ENGINE_AUDIO_BIND(setBusMuted, setBusMutedImpl);
        ENGINE_AUDIO_BIND(busInfo, busInfoImpl);
        ENGINE_AUDIO_BIND(createSound, createSoundImpl);
        ENGINE_AUDIO_BIND(destroySound, destroySoundImpl);
        ENGINE_AUDIO_BIND(play, playImpl);
        ENGINE_AUDIO_BIND(pause, pauseImpl);
        ENGINE_AUDIO_BIND(stop, stopImpl);
        ENGINE_AUDIO_BIND(seekSeconds, seekSecondsImpl);
        ENGINE_AUDIO_BIND(isPlaying, isPlayingImpl);
        ENGINE_AUDIO_BIND(atEnd, atEndImpl);
        ENGINE_AUDIO_BIND(soundTimes, soundTimesImpl);
        ENGINE_AUDIO_BIND(soundInfo, soundInfoImpl);
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
        const std::uint32_t sampleRate = resolved.sampleRate != 0 ? resolved.sampleRate : configuration_.sampleRate;
        const std::uint32_t channels = resolved.channels != 0 ? resolved.channels : configuration_.channels;
        const std::uint32_t listenerCount
            = resolved.listenerCount != 0 ? resolved.listenerCount : configuration_.listenerCount;
        const std::uint32_t periodSizeInFrames = resolved.periodSizeInFrames != 0
            ? resolved.periodSizeInFrames : configuration_.periodSizeInFrames;
        const std::uint32_t gainSmoothTimeInFrames = resolved.gainSmoothTimeInFrames != 0
            ? resolved.gainSmoothTimeInFrames : configuration_.gainSmoothTimeInFrames;
        const std::uint32_t defaultVolumeSmoothTimeInFrames = resolved.defaultVolumeSmoothTimeInFrames != 0
            ? resolved.defaultVolumeSmoothTimeInFrames : configuration_.defaultVolumeSmoothTimeInFrames;
        const std::uint32_t preMixStackSizeInBytes = resolved.preMixStackSizeInBytes != 0
            ? resolved.preMixStackSizeInBytes : configuration_.preMixStackSizeInBytes;
        if (!std::isfinite(resolved.masterVolume))
        {
            setError(error, AudioError::InvalidDescription);
            return invalidAudioEngine;
        }
        const float masterVolume = resolved.masterVolume >= 0.0f ? resolved.masterVolume : configuration_.masterVolume;
        // An explicit channel count shares the configured 254-channel bound;
        // zero keeps meaning "the selected device's native layout".
        if (!finiteNonNegative(masterVolume) || listenerCount == 0 || listenerCount > MA_ENGINE_MAX_LISTENERS
            || (channels != 0 && channels > 254)
            || (resolved.noDevice && (sampleRate == 0 || channels == 0)))
        {
            setError(error, AudioError::InvalidDescription);
            return invalidAudioEngine;
        }

        const auto lifecycleLock = registry_.lifecycleLock();
        if (!registry_.canCreate(configuration_.maximumEngines))
        {
            setError(error, AudioError::CapacityExceeded);
            return invalidAudioEngine;
        }

        std::shared_ptr<EngineRecord> record;
        try
        {
            record = std::make_shared<EngineRecord>();
            record->sounds.reserve(configuration_.initialSoundCapacity);
            record->buses.reserve(configuration_.initialBusCapacity);
            record->busCreationOrder.reserve(configuration_.initialBusCapacity);
        }
        catch (...)
        {
            setError(error, AudioError::CreationFailed);
            return invalidAudioEngine;
        }

        ma_resource_manager *resourceManager = nullptr;
        if (configuration_.sharedResources)
        {
            resourceManager = registry_.sharedResourceManager(configuration_);
            if (resourceManager == nullptr)
            {
                setError(error, AudioError::CreationFailed);
                return invalidAudioEngine;
            }
        }
        else
        {
            const ma_resource_manager_config resourceConfig = makeResourceManagerConfig(
                configuration_.resourceManagerThreads, configuration_.decodedFormatFloat32,
                configuration_.decodedChannels, configuration_.decodedSampleRate);
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
            std::fprintf(stderr, "audio module: ma_engine_init failed (%d)\n", static_cast<int>(result));
            return invalidAudioEngine;
        }
        record->initialized = true;
        record->noDevice = resolved.noDevice;
        // A device engine starts itself unless noAutoStart was requested;
        // device-less engines have no transport at all.
        record->started = !resolved.noDevice && !resolved.noAutoStart;
        ma_engine_set_volume(&record->engine, masterVolume);

        const AudioEngineHandle handle = registry_.allocateEngineHandle();
        if (handle == invalidAudioEngine)
        {
            record->close();
            setError(error, AudioError::CreationFailed);
            return invalidAudioEngine;
        }
        const AudioError insertError = registry_.insert(handle, record, configuration_.maximumEngines);
        if (insertError != AudioError::None)
        {
            record->close();
            setError(error, insertError);
            return invalidAudioEngine;
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
        const auto lifecycleLock = registry_.lifecycleLock();
        std::shared_ptr<EngineRecord> record;
        if (!registry_.remove(handle, record))
        {
            return;
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
        const LockedEngine engineLock = registry_.lock(handle);
        const EngineRecord *record = engineLock.get();
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

    // Reports the engine's actual format, not the creation request.
    bool engineStatisticsImpl(AudioEngineHandle handle, AudioEngineStatistics *outStatistics)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (outStatistics == nullptr)
        {
            return false;
        }
        const LockedEngine engineLock = registry_.lock(handle);
        const EngineRecord *record = engineLock.get();
        if (record == nullptr)
        {
            return false;
        }
        outStatistics->soundCount = static_cast<std::uint32_t>(record->sounds.size());
        outStatistics->busCount   = static_cast<std::uint32_t>(record->buses.size());
        return true;
#else
        (void)handle;
        (void)outStatistics;
        return false;
#endif
    }

#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
    // Shared runner for the device transport: resolve the engine, apply one
    // ma_engine call, and track the resulting started state. Device-less
    // engines have no transport; backend failures map to CreationFailed.
    bool engineTransport(AudioEngineHandle handle, bool start, AudioError *error)
    {
        setError(error, AudioError::None);
        const LockedEngine engineLock = registry_.lock(handle);
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
        const ma_result result = start ? ma_engine_start(&record->engine) : ma_engine_stop(&record->engine);
        if (result != MA_SUCCESS)
        {
            setError(error, AudioError::CreationFailed);
            return false;
        }
        record->started = start;
        return true;
    }
#endif

    // Starts device output; unsupported for noDevice engines.
    bool startEngineImpl(AudioEngineHandle handle, AudioError *error)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        return engineTransport(handle, true, error);
#else
        (void)handle;
        setError(error, AudioError::BackendUnavailable);
        return false;
#endif
    }

    // Mirrors startEngineImpl; unsupported for noDevice engines.
    bool stopEngineImpl(AudioEngineHandle handle, AudioError *error)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        return engineTransport(handle, false, error);
#else
        (void)handle;
        setError(error, AudioError::BackendUnavailable);
        return false;
#endif
    }

    // True while the engine's device transport runs. Device-less engines
    // have no transport and always report false.
    bool isEngineStartedImpl(AudioEngineHandle handle)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        const LockedEngine engineLock = registry_.lock(handle);
        const EngineRecord *record = engineLock.get();
        return record != nullptr && record->started;
#else
        (void)handle;
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
        const LockedEngine engineLock = registry_.lock(handle);
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

    // Reads the engine's current master gain.
    bool getMasterVolumeImpl(AudioEngineHandle handle, float *outVolume)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (outVolume == nullptr)
        {
            return false;
        }
        const LockedEngine engineLock = registry_.lock(handle);
        EngineRecord *record = engineLock.get();
        if (record == nullptr)
        {
            return false;
        }
        *outVolume = ma_engine_get_volume(&record->engine);
        return true;
#else
        (void)handle;
        (void)outVolume;
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
        const LockedEngine engineLock = registry_.lock(engineHandle);
        EngineRecord *engine = engineLock.get();
        if (engine == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return invalidAudioBus;
        }
        if (configuration_.maximumBusesPerEngine != 0
            && engine->buses.size() >= configuration_.maximumBusesPerEngine)
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
        const LockedEngine engineLock = registry_.lock(engineHandle);
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
        const LockedBus locked = registry_.lockBus(engineHandle, busHandle);
        if (locked.bus == nullptr)
        {
            return false;
        }
        locked.bus->volume = volume;
        ma_sound_group_set_volume(&locked.bus->group, locked.bus->muted ? 0.0f : volume);
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
        const LockedBus locked = registry_.lockBus(engineHandle, busHandle);
        if (locked.bus == nullptr)
        {
            return false;
        }
        locked.bus->muted = muted;
        ma_sound_group_set_volume(&locked.bus->group, muted ? 0.0f : locked.bus->volume);
        return true;
#else
        (void)engineHandle;
        (void)busHandle;
        (void)muted;
        return false;
#endif
    }

    // Reports the bus's own volume setting (kept while muted), its muted
    // state, and its parent.
    bool busInfoImpl(AudioEngineHandle engineHandle, AudioBusHandle busHandle, AudioBusInfo *outInfo)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (outInfo == nullptr)
        {
            return false;
        }
        const LockedBus locked = registry_.lockBus(engineHandle, busHandle);
        if (locked.bus == nullptr)
        {
            return false;
        }
        outInfo->parent = locked.bus->parent;
        outInfo->volume = locked.bus->volume;
        outInfo->muted  = locked.bus->muted;
        return true;
#else
        (void)engineHandle;
        (void)busHandle;
        (void)outInfo;
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
        const LockedEngine engineLock = registry_.lock(engineHandle);
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
        if (configuration_.maximumSoundsPerEngine != 0
            && engine->sounds.size() >= configuration_.maximumSoundsPerEngine)
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
                std::fprintf(stderr, "audio module: failed to load '%s' (%d)\n", description->filePath,
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
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        EngineRecord *engine = locked.engine.get();
        if (locked.sound == nullptr)
        {
            return;
        }
        const AudioBusHandle busHandle = locked.sound->bus;
        engine->sounds.erase(soundHandle);
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
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        if (locked.sound == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        const ma_result result = action(&locked.sound->sound);
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
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        if (locked.sound == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        // Unlike pause, stop also rewinds to PCM frame 0.
        const ma_result stopResult = ma_sound_stop(&locked.sound->sound);
        const ma_result seekResult = ma_sound_seek_to_pcm_frame(&locked.sound->sound, 0);
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

    // Uses the sound's cached data-format sample rate to convert seconds to
    // a PCM frame index (long double so huge values overflow-check cleanly).
    bool seekSecondsImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle, double seconds)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (!std::isfinite(seconds) || seconds < 0.0)
        {
            return false;
        }
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        if (locked.sound == nullptr)
        {
            return false;
        }
        const std::uint32_t sampleRate = locked.sound->resolvedDataSampleRate();
        if (sampleRate == 0)
        {
            return false;
        }
        const long double frame = static_cast<long double>(seconds) * static_cast<long double>(sampleRate);
        if (frame > static_cast<long double>(std::numeric_limits<ma_uint64>::max()))
        {
            return false;
        }
        return ma_sound_seek_to_pcm_frame(&locked.sound->sound, static_cast<ma_uint64>(frame)) == MA_SUCCESS;
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
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        return locked.sound != nullptr && ma_sound_is_playing(&locked.sound->sound) == MA_TRUE;
#else
        (void)engineHandle;
        (void)soundHandle;
        return false;
#endif
    }

    bool atEndImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        return locked.sound != nullptr && ma_sound_at_end(&locked.sound->sound) == MA_TRUE;
#else
        (void)engineHandle;
        (void)soundHandle;
        return false;
#endif
    }

    // Reports cursor and total length in seconds using the sound's cached
    // data-format sample rate; fails while that rate is unknown.
    bool soundTimesImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle, double *outCursorSeconds,
        double *outLengthSeconds)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (outCursorSeconds == nullptr || outLengthSeconds == nullptr)
        {
            return false;
        }
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        if (locked.sound == nullptr)
        {
            return false;
        }
        const std::uint32_t sampleRate = locked.sound->resolvedDataSampleRate();
        ma_uint64 cursor = 0;
        ma_uint64 length = 0;
        if (sampleRate == 0 || ma_sound_get_cursor_in_pcm_frames(&locked.sound->sound, &cursor) != MA_SUCCESS
            || ma_sound_get_length_in_pcm_frames(&locked.sound->sound, &length) != MA_SUCCESS)
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

    // Reads the sound's current settings back from the backend.
    bool soundInfoImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle, AudioSoundInfo *outInfo)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (outInfo == nullptr)
        {
            return false;
        }
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        if (locked.sound == nullptr)
        {
            return false;
        }
        outInfo->volume      = ma_sound_get_volume(&locked.sound->sound);
        outInfo->pitch       = ma_sound_get_pitch(&locked.sound->sound);
        outInfo->pan         = ma_sound_get_pan(&locked.sound->sound);
        outInfo->bus         = locked.sound->bus;
        outInfo->looping     = ma_sound_is_looping(&locked.sound->sound) == MA_TRUE;
        outInfo->spatialized = ma_sound_is_spatialization_enabled(&locked.sound->sound) == MA_TRUE;
        return true;
#else
        (void)engineHandle;
        (void)soundHandle;
        (void)outInfo;
        return false;
#endif
    }

    bool setSoundVolumeImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle, float volume)
    {
#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
        if (!finiteNonNegative(volume))
        {
            return false;
        }
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        if (locked.sound == nullptr)
        {
            return false;
        }
        ma_sound_set_volume(&locked.sound->sound, volume);
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
        {
            return false;
        }
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        if (locked.sound == nullptr)
        {
            return false;
        }
        ma_sound_set_pitch(&locked.sound->sound, pitch);
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
        {
            return false;
        }
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        if (locked.sound == nullptr)
        {
            return false;
        }
        ma_sound_set_pan(&locked.sound->sound, pan);
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
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        if (locked.sound == nullptr)
        {
            return false;
        }
        ma_sound_set_looping(&locked.sound->sound, looping ? MA_TRUE : MA_FALSE);
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
        {
            return false;
        }
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        if (locked.sound == nullptr)
        {
            return false;
        }
        ma_sound_set_position(&locked.sound->sound, position[0], position[1], position[2]);
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
        {
            return false;
        }
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        if (locked.sound == nullptr)
        {
            return false;
        }
        ma_sound_set_velocity(&locked.sound->sound, velocity[0], velocity[1], velocity[2]);
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
        // Direction and world up must be nonzero so the listener basis stays
        // well-defined; position and velocity may be anything finite.
        if (description == nullptr || !finiteVector(description->position) || !finiteNonZeroVector(description->direction)
            || !finiteNonZeroVector(description->worldUp) || !finiteVector(description->velocity))
        {
            return false;
        }
        // Listener index must be below the engine's listener count.
        const LockedEngine engineLock = registry_.lock(engineHandle);
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
        const LockedEngine engineLock = registry_.lock(engineHandle);
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
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        const EngineRecord *engine = locked.engine.get();
        if (locked.sound == nullptr)
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
        ma_sound_set_fade_in_pcm_frames(&locked.sound->sound, -1.0f, targetGain, fadeFrames);
        return true;
    }
    bool schedulePlayImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle,
        double delaySeconds, AudioError *error)
    {
        setError(error, AudioError::None);
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        const EngineRecord *engine = locked.engine.get();
        if (locked.sound == nullptr)
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
        ma_sound_set_start_time_in_pcm_frames(&locked.sound->sound, currentTime + delayFrames);
        if (ma_sound_start(&locked.sound->sound) != MA_SUCCESS)
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
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        const EngineRecord *engine = locked.engine.get();
        if (locked.sound == nullptr)
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
            &locked.sound->sound, currentTime + delayFrames, fadeFrames);
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
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        if (locked.sound == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        float direction[3] = {directionInput[0], directionInput[1], directionInput[2]};
        normalizeVector(direction);
        ma_sound_set_direction(&locked.sound->sound, direction[0], direction[1], direction[2]);
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
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        if (locked.sound == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        ma_sound_set_cone(&locked.sound->sound, innerAngleRadians, outerAngleRadians, outerGain);
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
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        if (locked.sound == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        ma_sound_set_doppler_factor(&locked.sound->sound, factor);
        return true;
    }

    bool setSoundListenerImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle,
        std::uint32_t listenerIndex, AudioError *error)
    {
        setError(error, AudioError::None);
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        if (locked.sound == nullptr)
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
        if (listenerIndex >= ma_engine_get_listener_count(&locked.engine.get()->engine))
        {
            setError(error, AudioError::InvalidDescription);
            return false;
        }
        ma_sound_set_pinned_listener_index(&locked.sound->sound, static_cast<ma_uint32>(listenerIndex));
        return true;
    }

    bool setSoundSpatializationEnabledImpl(AudioEngineHandle engineHandle, AudioSoundHandle soundHandle,
        bool enabled, AudioError *error)
    {
        setError(error, AudioError::None);
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        if (locked.sound == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        ma_sound_set_spatialization_enabled(&locked.sound->sound, enabled ? MA_TRUE : MA_FALSE);
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
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        if (locked.sound == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        ma_sound_set_min_gain(&locked.sound->sound, minimumGain);
        ma_sound_set_max_gain(&locked.sound->sound, maximumGain);
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
        const LockedSound locked = registry_.lockSound(engineHandle, soundHandle);
        if (locked.sound == nullptr)
        {
            setError(error, AudioError::InvalidHandle);
            return false;
        }
        ma_sound_set_min_distance(&locked.sound->sound, minimumDistance);
        ma_sound_set_max_distance(&locked.sound->sound, maximumDistance);
        ma_sound_set_rolloff(&locked.sound->sound, rolloff);
        ma_sound_set_attenuation_model(&locked.sound->sound, nativeAttenuation(model));
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

    // Module-level defaults captured once in construct().
    AudioConfiguration configuration_{};

#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
    // Handle-to-record index plus the lifecycle and shared-resource locks.
    EngineRegistry registry_;

    // Sound and bus handles are allocated from module-wide counters; see
    // allocateHandle for the exhaustion rules.
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
        .moduleVersion           = "0.1.0",
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
