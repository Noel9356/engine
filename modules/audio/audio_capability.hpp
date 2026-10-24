#ifndef ENGINE_MODULES_AUDIO_AUDIO_CAPABILITY_HPP
#define ENGINE_MODULES_AUDIO_AUDIO_CAPABILITY_HPP

// The published capability surface of the audio module. Every consumer -
// other modules, native mods, the host - includes THIS header, from the
// module that provides the capability. Like every published capability header
// this stays implementation-, platform- and vendor-agnostic: file paths,
// handles and volumes.
//
// Provider model: the family member in this module is "audio.miniaudio"
// (miniaudio, vendored as one pinned header). A different backend declares
// its own member of the family and wins through [providers]; mods keep
// demanding "audio".

#include <cstdint>
#include <string>

namespace engine
{

// Published when the audio device failed to open (headless machines, no
// sound hardware): every later playback call degrades to a logged no-op.
// Consumers that only fire-and-forget sounds never need to look at this.
struct AudioDeviceUnavailableEvent
{
    std::string reason;
};

// The fire-and-forget playback request: any component may publish this on
// the bus WITHOUT acquiring the capability - the module loads (caching by
// path) and plays the file. For fine control (stop, volumes, looping) use
// the capability instead.
struct AudioPlaybackRequestEvent
{
    std::string path;
    float       volume  = 1.0f;
    bool        looping = false;
};

// Capability "audio": sound and music playback.
//
// Sounds are addressed by handle, loaded once and played many times; the
// implementation decodes through its own resource pipeline, so WAV/MP3/FLAC/
// OGG come with the provider (miniaudio's decoder set). Every call is legal
// even without a device: the provider degrades to precise, once-logged
// no-ops so a headless CI run plays silence instead of failing.
class AudioCapability
{
public:
    static constexpr const char *capabilityName = "audio";

    using SoundHandle = std::uint32_t;

    static constexpr SoundHandle invalidSound = 0;

    virtual ~AudioCapability() = default;

    // Loads (or returns the already-loaded handle for) one sound file.
    // looping starts the sound again at its end until stopped. Returns
    // invalidSound with a precise log when the file is unreadable or not a
    // known format.
    virtual SoundHandle loadSound(
        const std::string &path, bool looping
    ) = 0;

    // Starts (or restarts) one loaded sound.
    virtual void play(
        SoundHandle handle
    ) = 0;

    virtual void stop(
        SoundHandle handle
    ) = 0;

    // Per-sound volume (1.0 = the mix level the file was authored at).
    virtual void setVolume(
        SoundHandle handle, float volume
    ) = 0;

    // Master output volume; every sound's effective gain is sound * master.
    virtual void setMasterVolume(
        float volume
    ) = 0;

    // Whether at least one playback of this sound is currently running.
    virtual bool isPlaying(
        SoundHandle handle
    ) = 0;

    // Releases one loaded sound. The handle becomes invalid; playing
    // instances stop.
    virtual void unload(
        SoundHandle handle
    ) = 0;
};

} // namespace engine

#endif // ENGINE_MODULES_AUDIO_AUDIO_CAPABILITY_HPP
