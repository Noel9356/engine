#ifndef ENGINE_MODULES_AUDIO_HPP
#define ENGINE_MODULES_AUDIO_HPP

#include "audio_capability.hpp"
#include "logging_capability.hpp"
#include "engine/kernel/component_interface.hpp"
#include "engine/kernel/event_bus.hpp"

#include <filesystem>
#include <string>
#include <unordered_map>

// miniaudio: vendored, pinned single header (third_party/miniaudio). The
// implementation lives in audio.cpp (MINIAUDIO_IMPLEMENTATION); this header
// only needs the opaque struct types - the audio backend never leaks into
// any consumer of the capability.
struct ma_engine;
struct ma_resource_manager;
struct ma_sound;

namespace engine
{

// Module "audio": the miniaudio-backed provider (family member
// "audio.miniaudio").
//
// Two surfaces:
// 1. the AudioCapability (fine control: load, play, stop, volumes),
// 2. the event-driven path: any component may publish an
//    AudioPlaybackRequestEvent on the bus - no capability acquisition, no
//    handle bookkeeping - and the module plays the file fire-and-forget.
//    That is the SFX path for modules that just want a click when something
//    happens.
//
// Headless degradation: without a device (CI containers, servers) the engine
// initializes a NULL device - the whole pipeline runs, everything is legal,
// nothing is audible - and the module logs once. A run never fails because
// the machine has no sound hardware.
class AudioModule final : public ModuleInterface, public AudioCapability
{
public:
    AudioModule();
    ~AudioModule() override;

    const ComponentManifest &manifest() const override
    {
        return componentManifest;
    }

    void construct(ComponentContext &context) override;
    void start() override;
    void stop() override;

    void *capabilityPointerFor(const std::string &capabilityName) override;

    // AudioCapability.
    SoundHandle   loadSound(const std::string &path, bool looping) override;
    void          play(SoundHandle handle) override;
    void          stop(SoundHandle handle) override;
    void          setVolume(SoundHandle handle, float volume) override;
    void          setMasterVolume(float volume) override;
    bool          isPlaying(SoundHandle handle) override;
    void          unload(SoundHandle handle) override;

private:
    struct LoadedSound
    {
        std::uint32_t handle    = invalidSound;
        std::string   resolvedPath;
        bool          looping    = false;
        bool          loaded     = false;          // the heap ma_sound below is initialized
        // Heap anchored (opaque here): the module's translation unit owns the
        // full miniaudio types; the header stays backend-free.
        struct ma_sound *sound = nullptr;
    };

    // Resolves and validates one path (relative to the executable directory,
    // the engine's path convention).
    std::filesystem::path resolvePath(
        const std::string &path
    ) const;
    LoadedSound           *findSound(
        SoundHandle handle
    );
    void publishDeviceUnavailable(
        const std::string &reason
    );

    ComponentManifest componentManifest;
    ComponentContext *boundContext = nullptr;
    LoggerCapability *logger       = nullptr;

    // The miniaudio engine (resource manager + mixer + device). Null when
    // initialization failed: every call then degrades to a logged no-op.
    ma_engine    *audioEngine = nullptr;
    ma_resource_manager *resourceManager = nullptr;
    bool          deviceUnavailableLogged = false;
    float         masterVolume            = 1.0f;

    std::unordered_map<std::uint32_t, LoadedSound> sounds;
    std::unordered_map<std::string, std::uint32_t> soundsByPath;
    std::uint32_t                                  nextHandle = invalidSound + 1;

    EventSubscription playbackRequestSubscription;

    // The fire-and-forget path's consumer (event channel subscription above).
    void handlePlaybackRequest(const AudioPlaybackRequestEvent &request);
};

} // namespace engine

#endif // ENGINE_MODULES_AUDIO_HPP
