#include "audio.hpp"

// The miniaudio implementation, in this translation unit exactly once
// (module_plugin.cpp includes the header only, which carries declarations).
#define MINIAUDIO_IMPLEMENTATION
#include <miniaudio.h>

#include "engine/kernel/filesystem_utilities.hpp"

#include <algorithm>
#include <cmath>

namespace engine
{

AudioModule::AudioModule()
{
    componentManifest.name = "audio";
    componentManifest.kind = ComponentKind::Module;
    componentManifest.providedCapabilities = {"audio.miniaudio"};
    // Reading content files is the module's whole job; no other capability
    // dependency - audio works in any assembly, renderer or not.
    componentManifest.requiredCapabilities = {"logging"};
    componentManifest.requiredPermissions  = {"capability.logging", "filesystem.read"};
    componentManifest.knownConfigKeys      = {
        "audio.master_volume",
        "audio.start_muted",
    };
}

AudioModule::~AudioModule()
{
    stop();
}

void AudioModule::construct(
    ComponentContext &context
)
{
    boundContext = &context;
    logger       = &context.acquireCapability<LoggerCapability>();

    masterVolume = static_cast<float>(context.configurationFloating("master_volume").value_or(1.0));
    if (context.configurationBoolean("start_muted").value_or(false))
    {
        masterVolume = 0.0f;
    }

    // The fire-and-forget path: one subscription, any component can request.
    EventBus                           &bus               = context.eventBus();
    EventChannel<AudioPlaybackRequestEvent> &requestChannel = bus.channel<AudioPlaybackRequestEvent>();
    playbackRequestSubscription                            = EventSubscription(
        &bus,
        typeid(AudioPlaybackRequestEvent),
        requestChannel.subscribe<AudioModule, &AudioModule::handlePlaybackRequest>(this)
    );
}

void AudioModule::start()
{
    // Resource manager first (decoders, file streaming), then the engine
    // (mixer + device). ma_engine_init builds a context over every available
    // backend - including the NULL backend, so a success here means real OR
    // silent output, both fully functional API-wise. Failure means even the
    // NULL backend could not open: the degraded mode below keeps every call
    // legal (ghost handles, precise no-ops) so a headless CI run plays
    // silence instead of failing startup.
    ma_resource_manager_config resourceConfig = ma_resource_manager_config_init();
    resourceManager                           = new ma_resource_manager();
    if (ma_resource_manager_init(&resourceConfig, resourceManager) != MA_SUCCESS)
    {
        delete resourceManager;
        resourceManager = nullptr;
        publishDeviceUnavailable("miniaudio resource manager failed to initialize");
        logger->logMessage(LogLevel::Warning, "audio module started DEGRADED (no decoder pipeline; playback is a no-op)");
        return;
    }

    ma_engine_config engineConfig = ma_engine_config_init();
    engineConfig.pResourceManager = resourceManager;
    audioEngine                   = new ma_engine();
    const ma_result result        = ma_engine_init(&engineConfig, audioEngine);
    if (result != MA_SUCCESS)
    {
        delete audioEngine;
        audioEngine = nullptr;
        publishDeviceUnavailable("miniaudio engine failed to initialize (error "
                                 + std::to_string(static_cast<int>(result)) + ")");
        logger->logMessage(LogLevel::Warning, "audio module started DEGRADED (no device; playback is a no-op)");
        return;
    }
    ma_engine_set_volume(audioEngine, masterVolume);

    logger->logMessage(LogLevel::Information, "audio module started");
}

void AudioModule::stop()
{
    playbackRequestSubscription.release();
    for (auto &entry : sounds)
    {
        if (entry.second.loaded && audioEngine != nullptr)
        {
            ma_sound_uninit(entry.second.sound);
        }
        delete entry.second.sound;
        entry.second.sound = nullptr;
    }
    sounds.clear();
    soundsByPath.clear();
    if (audioEngine != nullptr)
    {
        ma_engine_uninit(audioEngine);
        delete audioEngine;
        audioEngine = nullptr;
    }
    if (resourceManager != nullptr)
    {
        ma_resource_manager_uninit(resourceManager);
        delete resourceManager;
        resourceManager = nullptr;
    }
    boundContext = nullptr;
    logger       = nullptr;
}

void *AudioModule::capabilityPointerFor(
    const std::string &capabilityName
)
{
    if (capabilityName == "audio.miniaudio")
    {
        return static_cast<AudioCapability *>(this);
    }
    return nullptr;
}

std::filesystem::path AudioModule::resolvePath(
    const std::string &path
) const
{
    const std::filesystem::path candidate(path);
    return candidate.is_relative() ? (executableDirectory() / candidate) : candidate;
}

AudioModule::LoadedSound *AudioModule::findSound(
    SoundHandle handle
)
{
    const auto found = sounds.find(handle);
    return found == sounds.end() ? nullptr : &found->second;
}

void AudioModule::publishDeviceUnavailable(
    const std::string &reason
)
{
    if (deviceUnavailableLogged)
    {
        return;
    }
    deviceUnavailableLogged = true;
    if (boundContext != nullptr)
    {
        boundContext->eventBus().publish(AudioDeviceUnavailableEvent{.reason = reason});
    }
}

AudioCapability::SoundHandle AudioModule::loadSound(
    const std::string &path, bool looping
)
{
    // Same path + same looping = the same handle (load-once semantics); a
    // different looping mode is a different sound by contract (the flag is
    // fixed at load).
    const std::string cacheKey = path + (looping ? "#loop" : "#once");
    const auto        cached   = soundsByPath.find(cacheKey);
    if (cached != soundsByPath.end())
    {
        return cached->second;
    }
    if (audioEngine == nullptr)
    {
        // Degraded run: hand out a stable handle so scripts keep working,
        // the sound simply never plays.
        const std::uint32_t ghostHandle = nextHandle++;
        soundsByPath.emplace(cacheKey, ghostHandle);
        LoadedSound ghost;
        ghost.handle       = ghostHandle;
        ghost.resolvedPath = path;
        ghost.looping      = looping;
        ghost.loaded       = false;
        sounds.emplace(ghostHandle, std::move(ghost));
        return ghostHandle;
    }
    const std::filesystem::path resolved = resolvePath(path);
    std::error_code             fileError;
    if (!std::filesystem::is_regular_file(resolved, fileError))
    {
        logger->logMessage(
            LogLevel::Error, "audio module: '" + path + "' does not name a readable file (resolved '" + resolved.string() + "')"
        );
        return invalidSound;
    }

    LoadedSound sound;
    sound.handle       = nextHandle++;
    sound.resolvedPath = resolved.string();
    sound.looping      = looping;
    sound.sound        = new ma_sound();
    const ma_result result = ma_sound_init_from_file(
        audioEngine, sound.resolvedPath.c_str(), MA_SOUND_FLAG_STREAM, nullptr, nullptr, sound.sound
    );
    if (result != MA_SUCCESS)
    {
        delete sound.sound;
        logger->logMessage(
            LogLevel::Error,
            "audio module: failed to decode '" + path + "' (miniaudio error " + std::to_string(static_cast<int>(result))
                + "; WAV/MP3/FLAC/OGG are the supported container set)"
        );
        return invalidSound;
    }
    sound.loaded = true;
    ma_sound_set_looping(sound.sound, sound.looping ? MA_TRUE : MA_FALSE);
    const std::uint32_t handle = sound.handle;
    sounds.emplace(handle, std::move(sound));
    soundsByPath.emplace(cacheKey, handle);
    return handle;
}

void AudioModule::play(
    SoundHandle handle
)
{
    LoadedSound *sound = findSound(handle);
    if (sound == nullptr || !sound->loaded || audioEngine == nullptr)
    {
        return; // degraded or unknown: precise no-op
    }
    ma_sound_seek_to_pcm_frame(sound->sound, 0);
    ma_sound_start(sound->sound);
}

void AudioModule::stop(
    SoundHandle handle
)
{
    LoadedSound *sound = findSound(handle);
    if (sound == nullptr || !sound->loaded || audioEngine == nullptr)
    {
        return;
    }
    ma_sound_stop(sound->sound);
}

void AudioModule::setVolume(
    SoundHandle handle, float volume
)
{
    LoadedSound *sound = findSound(handle);
    if (sound == nullptr || !sound->loaded || audioEngine == nullptr)
    {
        return;
    }
    ma_sound_set_volume(sound->sound, std::max(volume, 0.0f));
}

void AudioModule::setMasterVolume(
    float volume
)
{
    masterVolume = std::max(volume, 0.0f);
    if (audioEngine != nullptr)
    {
        ma_engine_set_volume(audioEngine, masterVolume);
    }
}

bool AudioModule::isPlaying(
    SoundHandle handle
)
{
    LoadedSound *sound = findSound(handle);
    if (sound == nullptr || !sound->loaded || audioEngine == nullptr)
    {
        return false;
    }
    return ma_sound_is_playing(sound->sound) == MA_TRUE;
}

void AudioModule::unload(
    SoundHandle handle
)
{
    LoadedSound *sound = findSound(handle);
    if (sound == nullptr)
    {
        return;
    }
    if (sound->loaded && audioEngine != nullptr)
    {
        ma_sound_stop(sound->sound);
        ma_sound_uninit(sound->sound);
    }
    for (auto iterator = soundsByPath.begin(); iterator != soundsByPath.end(); ++iterator)
    {
        if (iterator->second == handle)
        {
            soundsByPath.erase(iterator);
            break;
        }
    }
    sounds.erase(handle);
}

void AudioModule::handlePlaybackRequest(
    const AudioPlaybackRequestEvent &request
)
{
    const SoundHandle handle = loadSound(request.path, request.looping);
    if (handle == invalidSound)
    {
        return;
    }
    setVolume(handle, request.volume);
    play(handle);
}

} // namespace engine
