// Private backend surface shared by the audio module's translation units.
//
// Owns the miniaudio-facing state types: the validation helpers, the engine/
// bus/sound records, the resolved [audio] configuration, and the lock guards
// the capability implementations use to hold an engine while operating on it.
// Only compiled when the miniaudio backend is enabled; consumers include it
// unconditionally and guard their backend-specific code the same way.

#ifndef ENGINE_AUDIO_BACKEND_HPP
#define ENGINE_AUDIO_BACKEND_HPP

#include <engine/audio/capabilities.hpp>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <numbers>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine
{
namespace audio
{
namespace backend
{

// The [audio] configuration section resolved once in construct(). Zero
// capacities are unbounded; initial capacities reserve hash-map buckets
// without imposing a hard limit.
struct AudioConfiguration
{
    std::uint32_t sampleRate                      = 0;
    std::uint32_t channels                        = 0;
    std::uint32_t listenerCount                   = 1;
    float         masterVolume                    = 1.0f;
    bool          sharedResources                 = false;
    std::uint32_t resourceManagerThreads          = 1;
    bool          decodedFormatFloat32            = false;
    std::uint32_t decodedChannels                 = 0;
    std::uint32_t decodedSampleRate               = 0;
    std::uint32_t initialEngineCapacity           = 4;
    std::uint32_t initialSoundCapacity            = 64;
    std::uint32_t initialBusCapacity              = 8;
    std::uint32_t maximumEngines                  = 0;
    std::uint32_t maximumSoundsPerEngine          = 0;
    std::uint32_t maximumBusesPerEngine           = 0;
    std::uint32_t periodSizeInFrames              = 0;
    std::uint32_t gainSmoothTimeInFrames          = 0;
    std::uint32_t defaultVolumeSmoothTimeInFrames = 0;
    std::uint32_t preMixStackSizeInBytes          = 0;
};

} // namespace backend
} // namespace audio
} // namespace engine

#if defined(ENGINE_AUDIO_BACKEND_MINIAUDIO)
#include <miniaudio.h>

namespace engine
{
namespace audio
{
namespace backend
{

// Validation guards for capability floats: finite and non-negative, finite and
// positive, and finite in all three components respectively.
inline bool finiteNonNegative(float value)
{
    return std::isfinite(value) && value >= 0.0f;
}

inline bool finitePositive(float value)
{
    return std::isfinite(value) && value > 0.0f;
}

inline bool finiteVector(const float values[3])
{
    return values != nullptr && std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
}

inline bool finiteNonZeroVector(const float values[3])
{
    if (!finiteVector(values))
        return false;
    const double lengthSquared = static_cast<double>(values[0]) * values[0]
        + static_cast<double>(values[1]) * values[1] + static_cast<double>(values[2]) * values[2];
    return lengthSquared > 1.0e-16 && std::isfinite(lengthSquared);
}

inline void normalizeVector(float values[3])
{
    const double lengthSquared = static_cast<double>(values[0]) * values[0]
        + static_cast<double>(values[1]) * values[1] + static_cast<double>(values[2]) * values[2];
    const float inverseLength = static_cast<float>(1.0 / std::sqrt(lengthSquared));
    for (std::uint32_t axis = 0; axis < 3; ++axis)
        values[axis] *= inverseLength;
}

inline bool validSoundCone(float innerAngleRadians, float outerAngleRadians, float outerGain)
{
    return finiteNonNegative(innerAngleRadians) && finiteNonNegative(outerAngleRadians)
        && innerAngleRadians <= outerAngleRadians && outerAngleRadians <= 2.0f * std::numbers::pi_v<float>
        && finiteNonNegative(outerGain);
}

// Converts a non-negative duration to a whole frame count; false for
// non-finite or negative input or an overflow of the frame type.
inline bool secondsToFrameOffset(double seconds, std::uint32_t sampleRate, ma_uint64 &output)
{
    if (!std::isfinite(seconds) || seconds < 0.0 || sampleRate == 0)
        return false;
    const long double frames = std::ceil(static_cast<long double>(seconds) * sampleRate);
    if (!std::isfinite(frames) || frames > static_cast<long double>(std::numeric_limits<ma_uint64>::max()))
        return false;
    output = static_cast<ma_uint64>(frames);
    return true;
}

// Maps the public attenuation enum onto miniaudio's model; unknown values
// fall back to inverse.
inline ma_attenuation_model nativeAttenuation(AudioAttenuationModel model)
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

// Allocates the next 32-bit handle from a 64-bit counter. Counters count up
// from 1 and permanently exhaust instead of wrapping into reused handles, so
// a stale handle can never alias a live object.
inline std::uint32_t allocateHandle(std::atomic<std::uint64_t> &counter)
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

// One loaded sound. Records are stored directly in the engine's node-based
// hash map, avoiding one heap allocation per sound while keeping ma_sound's
// address stable for its lifetime.
struct SoundRecord
{
    ma_sound       sound{};
    AudioBusHandle bus          = invalidAudioBus;
    bool           initialized  = false;

    // The sound's data-format sample rate (its own rate, not the engine's) is
    // needed to convert between seconds and the sound's PCM frame indexes. It
    // never changes after creation, so it is queried once and cached.
    // Asynchronously decoded sounds may not know it yet at creation, so the
    // cache fills in on the first successful query.
    std::uint32_t dataSampleRate       = 0;
    bool          dataSampleRateKnown  = false;

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

    // Returns the sound's data-format sample rate, or 0 while it is unknown.
    std::uint32_t resolvedDataSampleRate()
    {
        if (dataSampleRateKnown)
        {
            return dataSampleRate;
        }
        ma_format format     = ma_format_unknown;
        ma_uint32 channels   = 0;
        ma_uint32 sampleRate = 0;
        if (ma_sound_get_data_format(&sound, &format, &channels, &sampleRate, nullptr, 0) == MA_SUCCESS
            && sampleRate != 0)
        {
            dataSampleRate      = sampleRate;
            dataSampleRateKnown = true;
        }
        return sampleRate;
    }
};

// A mix group also caches its direct-child and attached-sound counts. This
// makes destroyBus a constant-time ownership check rather than a scan of all
// buses and sounds.
struct BusRecord
{
    ma_sound_group group{};
    AudioBusHandle parent         = invalidAudioBus;
    std::uint32_t  childBusCount  = 0;
    std::uint32_t  attachedSounds = 0;
    float          volume         = 1.0f;
    bool           muted          = false;
    bool           initialized    = false;

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
    ma_engine            engine{};
    ma_resource_manager  ownedResourceManager{};
    ma_resource_manager *resourceManager = nullptr;
    std::mutex           mutex;
    std::unordered_map<AudioSoundHandle, SoundRecord> sounds;
    std::unordered_map<AudioBusHandle, BusRecord>     buses;
    std::vector<AudioBusHandle>                       busCreationOrder;
    bool initialized         = false;
    bool ownsResourceManager = false;
    bool noDevice            = false;
    bool started             = false;
    bool closed              = false;

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
        started = false;
        closed = true;
    }
};

// The [audio] configuration section is declared above the backend guard;
// the helpers below this point need miniaudio types.

inline ma_resource_manager_config makeResourceManagerConfig(std::uint32_t jobThreadCount,
    bool decodedFormatFloat32, std::uint32_t decodedChannels, std::uint32_t decodedSampleRate)
{
    ma_resource_manager_config config = ma_resource_manager_config_init();
    config.jobThreadCount = jobThreadCount;
    config.decodedFormat = decodedFormatFloat32 ? ma_format_f32 : ma_format_unknown;
    config.decodedChannels = decodedChannels;
    config.decodedSampleRate = decodedSampleRate;
    return config;
}

// An engine lease: holds a shared_ptr acquired from the registry plus the
// engine's own mutex, released together. Construction locks the engine and
// yields an empty lease when the record is already closed.
struct LockedEngine
{
    std::shared_ptr<EngineRecord> record;
    std::unique_lock<std::mutex>  lock;

    LockedEngine() = default;
    explicit LockedEngine(std::shared_ptr<EngineRecord> value)
        : record(std::move(value))
    {
        if (record)
        {
            lock = std::unique_lock<std::mutex>(record->mutex);
            if (record->closed || !record->initialized)
            {
                release();
            }
        }
    }

    void release()
    {
        if (lock.owns_lock())
        {
            lock.unlock();
        }
        record.reset();
    }

    EngineRecord *get() const
    {
        return record.get();
    }
};

// An engine lease plus one resolved member record; the sound/bus pointer is
// null when either handle is unknown, in which case the engine lease is
// already released.
struct LockedSound
{
    LockedEngine engine;
    SoundRecord *sound = nullptr;
};

struct LockedBus
{
    LockedEngine engine;
    BusRecord   *bus = nullptr;
};

} // namespace backend
} // namespace audio
} // namespace engine

#endif
#endif
