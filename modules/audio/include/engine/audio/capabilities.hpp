// Public capability surface of the audio module.
//
// Declares the POD types and function-pointer table consumers receive through
// ParticipantContext::acquireCapability for the "audio" capability (ABI 3).
// The implementation lives in source/audio.cpp (miniaudio backend).

#ifndef ENGINE_AUDIO_CAPABILITIES_HPP
#define ENGINE_AUDIO_CAPABILITIES_HPP

#include <cstdint>
#include <limits>

namespace engine
{
namespace audio
{

// ABI version of this capability struct; consumers must match it.
inline constexpr std::uint32_t audioCapabilityAbi = 3u;

// Opaque handle kinds; 0 is the invalid sentinel for each (see constants below).
using AudioEngineHandle = std::uint32_t;
using AudioSoundHandle  = std::uint32_t;
using AudioBusHandle    = std::uint32_t;

// "None" sentinels; handles handed out by the module start at 1.
inline constexpr AudioEngineHandle invalidAudioEngine = 0;
inline constexpr AudioSoundHandle  invalidAudioSound  = 0;
inline constexpr AudioBusHandle    invalidAudioBus    = 0;
inline constexpr std::uint32_t     invalidAudioListener = std::numeric_limits<std::uint32_t>::max(); // Use the closest listener.

// Error codes written through optional AudioError out-parameters: no backend
// compiled in (BackendUnavailable), object could not be created (CreationFailed),
// unknown handle (InvalidHandle), malformed description (InvalidDescription),
// file load/decode or read failure (ResourceFailed), bus still in use (Busy),
// operation not meaningful for this engine (Unsupported), or a configured hard
// resource budget was reached (CapacityExceeded).
enum class AudioError : std::uint32_t
{
    None = 0,
    BackendUnavailable,
    CreationFailed,
    InvalidHandle,
    InvalidDescription,
    ResourceFailed,
    Busy,
    Unsupported,
    CapacityExceeded
};

// Distance-attenuation curves for spatialized sounds, applied between
// minDistance and maxDistance: constant (None), 1/d (Inverse), straight fade
// to zero at maxDistance (Linear), or d^-rolloff (Exponential).
enum class AudioAttenuationModel : std::uint32_t
{
    None = 0,
    Inverse,
    Linear,
    Exponential
};

// Engine creation parameters. Zero-valued members (and a negative masterVolume)
// mean "use the module's configured default" (see construct() in source/audio.cpp).
struct AudioEngineDescription
{
    // 0 = configured or backend default device format.
    std::uint32_t sampleRate = 0;
    std::uint32_t channels   = 0;

    // 0 = configured default; the module validates 1-4.
    std::uint32_t listenerCount = 0;

    // Negative = use the configured master volume.
    float masterVolume = -1.0f;

    // noDevice: no output device; audio is pulled manually via readFrames
    // (sampleRate and channels must resolve to nonzero values). noAutoStart: engine stays
    // stopped until startEngine is called.
    bool noDevice    = false;
    bool noAutoStart = false;

    // Mixer scheduling and CPU/latency controls. Zero selects the corresponding
    // [audio] setting; a zero configured value leaves miniaudio's default.
    std::uint32_t periodSizeInFrames = 0;
    std::uint32_t gainSmoothTimeInFrames = 0;
    std::uint32_t defaultVolumeSmoothTimeInFrames = 0;
    std::uint32_t preMixStackSizeInBytes = 0;
};

// Observed engine facts; backendName is a static string ("miniaudio").
struct AudioEngineInfo
{
    const char   *backendName = nullptr;
    std::uint32_t sampleRate  = 0;
    std::uint32_t channels    = 0;
    bool          hasDevice   = false;
};

// Live engine object counts for budget monitoring and leak triage.
struct AudioEngineStatistics
{
    std::uint32_t soundCount = 0;
    std::uint32_t busCount   = 0;
};

// Mix-group parameters; parent = invalidAudioBus attaches the bus directly to
// the engine's master output. A muted bus keeps its volume setting and plays
// at gain 0 until unmuted.
struct AudioBusDescription
{
    AudioBusHandle parent = invalidAudioBus;
    float          volume = 1.0f;
    bool           muted  = false;
};

// Observed bus facts; volume is the bus's own setting (a muted bus keeps it)
// and parent identifies the owning bus (invalidAudioBus = master output).
struct AudioBusInfo
{
    AudioBusHandle parent = invalidAudioBus;
    float          volume = 1.0f;
    bool           muted  = false;
};

// Sound creation parameters. filePath is required and only needs to stay
// valid for the call. position/velocity/minDistance/maxDistance/rolloff and
// attenuation are only applied when spatialized is true (world units,
// seconds-based distances).
struct AudioSoundDescription
{
    // Required, non-empty; decoded by miniaudio per its extension.
    const char *filePath = nullptr;

    AudioBusHandle bus = invalidAudioBus;

    float volume = 1.0f;
    float pitch  = 1.0f;
    float pan    = 0.0f;

    float position[3] = {0.0f, 0.0f, 0.0f};
    float velocity[3] = {0.0f, 0.0f, 0.0f};

    // Distances in world units; both must be positive and max >= min.
    float minDistance = 1.0f;
    float maxDistance = 100.0f;
    float rolloff     = 1.0f;

    // Optional emitter directivity and motion controls; applied when spatialized is true.
    float direction[3] = {0.0f, 0.0f, -1.0f};
    float minimumGain = 0.0f;
    float maximumGain = 1.0f;
    float dopplerFactor = 1.0f;
    float coneInnerAngleRadians = 0.0f;
    float coneOuterAngleRadians = 0.0f;
    float coneOuterGain = 1.0f;
    std::uint32_t listenerIndex = invalidAudioListener;
    bool directionalCone = false;

    AudioAttenuationModel attenuation = AudioAttenuationModel::Inverse;

    // Load/behaviour flags. streaming and decode are mutually exclusive load
    // modes; asynchronous decodes on a worker thread; disablePitch skips the
    // pitch shifter entirely.
    bool looping      = false;
    bool spatialized  = false;
    bool streaming    = false;
    bool decode       = false;
    bool asynchronous = false;
    bool disablePitch = false;
};

// Listener pose used to spatialize sounds (world units; velocity in units per
// second feeds motion effects).
struct AudioListenerDescription
{
    float position[3]  = {0.0f, 0.0f, 0.0f};
    float direction[3] = {0.0f, 0.0f, -1.0f};
    float worldUp[3]   = {0.0f, 1.0f, 0.0f};
    float velocity[3]  = {0.0f, 0.0f, 0.0f};
};

// Observed sound facts; volume/pitch/pan/looping/spatialized are read back
// from the backend and bus identifies the mix group the sound feeds
// (invalidAudioBus = master output).
struct AudioSoundInfo
{
    float          volume      = 1.0f;
    float          pitch       = 1.0f;
    float          pan         = 0.0f;
    AudioBusHandle bus         = invalidAudioBus;
    bool           looping     = false;
    bool           spatialized = false;
};

// Function-pointer table for the "audio" capability. Every call takes the
// module's self pointer first; AudioError out-parameters are optional and
// always written on failing paths that report errors. Calls targeting the same
// engine are serialized; calls targeting different engines can run concurrently.
struct AudioCapability
{
    static constexpr const char *capabilityName = "audio";

    std::uint32_t abi  = audioCapabilityAbi;
    void         *self = nullptr;

    // Creating an engine succeeds even with a null description (all defaults);
    // destroying an engine destroys its buses and sounds with it.
    AudioEngineHandle (*createEngine)(void *self, const AudioEngineDescription *description, AudioError *error);
    void (*destroyEngine)(void *self, AudioEngineHandle engine);
    bool (*engineInfo)(void *self, AudioEngineHandle engine, AudioEngineInfo *outInfo);
    // Live object counts for budget monitoring and leak triage.
    bool (*engineStatistics)(void *self, AudioEngineHandle engine, AudioEngineStatistics *outStatistics);
    bool (*startEngine)(void *self, AudioEngineHandle engine, AudioError *error);
    bool (*stopEngine)(void *self, AudioEngineHandle engine, AudioError *error);
    // True while the engine's device transport runs; false for stopped and
    // device-less engines.
    bool (*isEngineStarted)(void *self, AudioEngineHandle engine);
    bool (*setMasterVolume)(void *self, AudioEngineHandle engine, float volume);
    // Reads the engine's current master gain into outVolume.
    bool (*getMasterVolume)(void *self, AudioEngineHandle engine, float *outVolume);

    AudioBusHandle (*createBus)(
        void *self, AudioEngineHandle engine, const AudioBusDescription *description, AudioError *error);
    bool (*destroyBus)(void *self, AudioEngineHandle engine, AudioBusHandle bus, AudioError *error);
    bool (*setBusVolume)(void *self, AudioEngineHandle engine, AudioBusHandle bus, float volume);
    bool (*setBusMuted)(void *self, AudioEngineHandle engine, AudioBusHandle bus, bool muted);
    // Observed bus state: parent, volume, and muted flag.
    bool (*busInfo)(void *self, AudioEngineHandle engine, AudioBusHandle bus, AudioBusInfo *outInfo);

    AudioSoundHandle (*createSound)(
        void *self, AudioEngineHandle engine, const AudioSoundDescription *description, AudioError *error);
    void (*destroySound)(void *self, AudioEngineHandle engine, AudioSoundHandle sound);
    bool (*play)(void *self, AudioEngineHandle engine, AudioSoundHandle sound, AudioError *error);
    bool (*pause)(void *self, AudioEngineHandle engine, AudioSoundHandle sound, AudioError *error);
    bool (*stop)(void *self, AudioEngineHandle engine, AudioSoundHandle sound, AudioError *error);
    bool (*seekSeconds)(void *self, AudioEngineHandle engine, AudioSoundHandle sound, double seconds);
    bool (*isPlaying)(void *self, AudioEngineHandle engine, AudioSoundHandle sound);
    bool (*atEnd)(void *self, AudioEngineHandle engine, AudioSoundHandle sound);
    bool (*soundTimes)(void *self, AudioEngineHandle engine, AudioSoundHandle sound, double *outCursorSeconds,
        double *outLengthSeconds);
    // Observed sound state: base volume, pitch, pan, looping, spatialization,
    // and the bus the sound feeds.
    bool (*soundInfo)(void *self, AudioEngineHandle engine, AudioSoundHandle sound, AudioSoundInfo *outInfo);
    bool (*setSoundVolume)(void *self, AudioEngineHandle engine, AudioSoundHandle sound, float volume);
    bool (*setSoundPitch)(void *self, AudioEngineHandle engine, AudioSoundHandle sound, float pitch);
    bool (*setSoundPan)(void *self, AudioEngineHandle engine, AudioSoundHandle sound, float pan);
    bool (*setSoundLooping)(void *self, AudioEngineHandle engine, AudioSoundHandle sound, bool looping);
    bool (*setSoundPosition)(void *self, AudioEngineHandle engine, AudioSoundHandle sound, const float position[3]);
    bool (*setSoundVelocity)(void *self, AudioEngineHandle engine, AudioSoundHandle sound, const float velocity[3]);

    bool (*setListener)(
        void *self, AudioEngineHandle engine, std::uint32_t listenerIndex, const AudioListenerDescription *description);

    // Manual rendering for noDevice engines: fills output with interleaved
    // frames in the engine's format (frameCount frames attempted).
    bool (*readFrames)(void *self, AudioEngineHandle engine, float *output, std::uint64_t frameCount,
        std::uint64_t *outFramesRead, AudioError *error);

    // Gain envelope is independent of the sound's base volume and fades from
    // the current envelope value to targetGain over durationSeconds.
    bool (*fadeSoundGain)(void *self, AudioEngineHandle engine, AudioSoundHandle sound, float targetGain,
        double durationSeconds, AudioError *error);

    // Schedules playback relative to the engine's current graph clock. A
    // positive start delay still starts the sound object immediately so the
    // backend can begin it at the scheduled frame.
    bool (*schedulePlay)(void *self, AudioEngineHandle engine, AudioSoundHandle sound,
        double delaySeconds, AudioError *error);

    // Schedules stop relative to the current graph clock. fadeSeconds must not
    // exceed delaySeconds; zero produces an immediate stop at the scheduled time.
    bool (*scheduleStop)(void *self, AudioEngineHandle engine, AudioSoundHandle sound,
        double delaySeconds, double fadeSeconds, AudioError *error);

    bool (*setSoundDirection)(void *self, AudioEngineHandle engine, AudioSoundHandle sound,
        const float direction[3], AudioError *error);
    bool (*setSoundCone)(void *self, AudioEngineHandle engine, AudioSoundHandle sound,
        float innerAngleRadians, float outerAngleRadians, float outerGain, AudioError *error);
    bool (*setSoundDopplerFactor)(void *self, AudioEngineHandle engine, AudioSoundHandle sound,
        float factor, AudioError *error);
    // Pins to an explicit listener. invalidAudioListener selects closest on
    // creation; miniaudio has no public API to restore closest after pinning.
    bool (*setSoundListener)(void *self, AudioEngineHandle engine, AudioSoundHandle sound,
        std::uint32_t listenerIndex, AudioError *error);
    bool (*setSoundSpatializationEnabled)(void *self, AudioEngineHandle engine, AudioSoundHandle sound,
        bool enabled, AudioError *error);
    bool (*setSoundGainRange)(void *self, AudioEngineHandle engine, AudioSoundHandle sound,
        float minimumGain, float maximumGain, AudioError *error);
    bool (*setSoundAttenuation)(void *self, AudioEngineHandle engine, AudioSoundHandle sound,
        float minimumDistance, float maximumDistance, float rolloff, AudioAttenuationModel model,
        AudioError *error);
};

} // namespace audio
} // namespace engine

#endif
