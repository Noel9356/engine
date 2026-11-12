// Lua bindings for the "audio" capability.
//
// One native function per capability entry, registered by
// initializeAudioLuaRuntime when the module acquires the optional
// script_runtime capability. Bindings map Lua values onto the capability's
// plain argument lists: handles are non-negative integers, strings only for
// file paths, and the rest are numbers or booleans. Every binding reports
// usage problems (wrong argument count or type, missing result capacity) by
// returning false; capability failures are reported through the results.

#include "lua_bridge.hpp"

#include <engine/script_runtime/native_values.hpp>

#include <cstdint>
#include <limits>

namespace engine::audio
{
namespace
{

using script_runtime::ScriptNativeFunction;
using script_runtime::ScriptRuntimeHandle;
using script_runtime::ScriptValue;
using script_runtime::native::floatValue;
using script_runtime::native::number;
using script_runtime::native::optionalBoolean;
using script_runtime::native::resultBoolean;
using script_runtime::native::resultNil;
using script_runtime::native::resultNumber;
using script_runtime::native::resultString;
using script_runtime::native::stringValue;
using script_runtime::native::uint32Value;

// The bridge context every native function receives as userData.
AudioCapability &audio(void *userData)
{
    return *static_cast<AudioLuaBridgeContext *>(userData)->audio;
}

// Writes an error code into a result slot.
void resultError(ScriptValue *results, std::uint32_t index, AudioError error)
{
    resultNumber(results, index, static_cast<double>(static_cast<std::uint32_t>(error)));
}

// Reads the (engine, sound) handle pair shared by every sound binding.
bool parseEngineSound(const ScriptValue *args, std::uint32_t count, AudioEngineHandle &engine, AudioSoundHandle &sound)
{
    std::uint32_t engineValue = 0;
    std::uint32_t soundValue = 0;
    if (!uint32Value(args, count, 0, engineValue) || !uint32Value(args, count, 1, soundValue))
        return false;
    engine = engineValue;
    sound = soundValue;
    return true;
}

bool createEngine(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    if (count > 6 || capacity < 2)
        return false;
    auto &api = audio(userData);
    AudioEngineDescription description{};
    float master = -1.0f;
    if (!uint32Value(args, count, 0, description.sampleRate, 0, false)
        || !uint32Value(args, count, 1, description.channels, 0, false)
        || !uint32Value(args, count, 2, description.listenerCount, 0, false)
        || !floatValue(args, count, 3, master, -1.0f, false)
        || !optionalBoolean(args, count, 4, description.noDevice)
        || !optionalBoolean(args, count, 5, description.noAutoStart))
        return false;
    description.masterVolume = master;
    AudioError error = AudioError::None;
    const AudioEngineHandle handle = api.createEngine(api.self, &description, &error);
    if (handle == invalidAudioEngine)
        resultNil(out, 0);
    else
        resultNumber(out, 0, handle);
    resultError(out, 1, error);
    *outCount = 2;
    return true;
}

bool destroyEngine(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *, std::uint32_t, std::uint32_t *outCount)
{
    std::uint32_t engine = 0;
    if (count != 1 || !uint32Value(args, count, 0, engine))
        return false;
    auto &api = audio(userData);
    api.destroyEngine(api.self, engine);
    *outCount = 0;
    return true;
}

bool engineInfo(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    std::uint32_t engine = 0;
    if (count != 1 || capacity < 4 || !uint32Value(args, count, 0, engine))
        return false;
    AudioEngineInfo info{};
    auto &api = audio(userData);
    if (!api.engineInfo(api.self, engine, &info))
    {
        resultNil(out, 0);
        *outCount = 1;
        return true;
    }
    resultNumber(out, 0, info.sampleRate);
    resultNumber(out, 1, info.channels);
    resultBoolean(out, 2, info.hasDevice);
    resultString(out, 3, info.backendName);
    *outCount = 4;
    return true;
}

bool engineStatistics(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    std::uint32_t engine = 0;
    if (count != 1 || capacity < 2 || !uint32Value(args, count, 0, engine))
        return false;
    AudioEngineStatistics statistics{};
    auto &api = audio(userData);
    if (!api.engineStatistics(api.self, engine, &statistics))
    {
        resultNil(out, 0);
        resultNil(out, 1);
    }
    else
    {
        resultNumber(out, 0, statistics.soundCount);
        resultNumber(out, 1, statistics.busCount);
    }
    *outCount = 2;
    return true;
}

// Shared body of startEngine/stopEngine.
bool engineAction(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount, bool start)
{
    std::uint32_t engine = 0;
    if (count != 1 || capacity < 2 || !uint32Value(args, count, 0, engine))
        return false;
    auto &api = audio(userData);
    AudioError error = AudioError::None;
    const bool ok = start ? api.startEngine(api.self, engine, &error) : api.stopEngine(api.self, engine, &error);
    resultBoolean(out, 0, ok);
    resultError(out, 1, error);
    *outCount = 2;
    return true;
}

bool startEngine(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    return engineAction(userData, args, count, out, capacity, outCount, true);
}

bool stopEngine(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    return engineAction(userData, args, count, out, capacity, outCount, false);
}

bool isEngineStarted(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    std::uint32_t engine = 0;
    if (count != 1 || capacity < 1 || !uint32Value(args, count, 0, engine))
        return false;
    auto &api = audio(userData);
    resultBoolean(out, 0, api.isEngineStarted(api.self, engine));
    *outCount = 1;
    return true;
}

bool setMasterVolume(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    std::uint32_t engine = 0;
    float volume = 0.0f;
    if (count != 2 || capacity < 1 || !uint32Value(args, count, 0, engine) || !floatValue(args, count, 1, volume))
        return false;
    auto &api = audio(userData);
    resultBoolean(out, 0, api.setMasterVolume(api.self, engine, volume));
    *outCount = 1;
    return true;
}

bool getMasterVolume(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    std::uint32_t engine = 0;
    if (count != 1 || capacity < 1 || !uint32Value(args, count, 0, engine))
        return false;
    auto &api = audio(userData);
    float volume = 0.0f;
    if (!api.getMasterVolume(api.self, engine, &volume))
        resultNil(out, 0);
    else
        resultNumber(out, 0, volume);
    *outCount = 1;
    return true;
}

bool createBus(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    if (count > 4 || capacity < 2)
        return false;
    std::uint32_t engine = 0;
    std::uint32_t parent = 0;
    AudioBusDescription description{};
    if (!uint32Value(args, count, 0, engine) || !uint32Value(args, count, 1, parent, 0, false)
        || !floatValue(args, count, 2, description.volume, 1.0f, false)
        || !optionalBoolean(args, count, 3, description.muted))
        return false;
    description.parent = parent;
    auto &api = audio(userData);
    AudioError error = AudioError::None;
    const AudioBusHandle handle = api.createBus(api.self, engine, &description, &error);
    if (handle == invalidAudioBus)
        resultNil(out, 0);
    else
        resultNumber(out, 0, handle);
    resultError(out, 1, error);
    *outCount = 2;
    return true;
}

bool destroyBus(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    std::uint32_t engine = 0;
    std::uint32_t bus = 0;
    if (count != 2 || capacity < 2 || !uint32Value(args, count, 0, engine) || !uint32Value(args, count, 1, bus))
        return false;
    auto &api = audio(userData);
    AudioError error = AudioError::None;
    resultBoolean(out, 0, api.destroyBus(api.self, engine, bus, &error));
    resultError(out, 1, error);
    *outCount = 2;
    return true;
}

bool setBusVolume(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    std::uint32_t engine = 0;
    std::uint32_t bus = 0;
    float volume = 0.0f;
    if (count != 3 || capacity < 1 || !uint32Value(args, count, 0, engine) || !uint32Value(args, count, 1, bus)
        || !floatValue(args, count, 2, volume))
        return false;
    auto &api = audio(userData);
    resultBoolean(out, 0, api.setBusVolume(api.self, engine, bus, volume));
    *outCount = 1;
    return true;
}

bool setBusMuted(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    std::uint32_t engine = 0;
    std::uint32_t bus = 0;
    bool muted = false;
    if (count != 3 || capacity < 1 || !uint32Value(args, count, 0, engine) || !uint32Value(args, count, 1, bus)
        || !optionalBoolean(args, count, 2, muted, false, true))
        return false;
    auto &api = audio(userData);
    resultBoolean(out, 0, api.setBusMuted(api.self, engine, bus, muted));
    *outCount = 1;
    return true;
}

bool busInfo(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    std::uint32_t engine = 0;
    std::uint32_t bus = 0;
    if (count != 2 || capacity < 3 || !uint32Value(args, count, 0, engine) || !uint32Value(args, count, 1, bus))
        return false;
    AudioBusInfo info{};
    auto &api = audio(userData);
    if (!api.busInfo(api.self, engine, bus, &info))
    {
        resultNil(out, 0);
        resultNil(out, 1);
        resultNil(out, 2);
    }
    else
    {
        resultNumber(out, 0, info.parent == invalidAudioBus ? 0.0 : static_cast<double>(info.parent));
        resultNumber(out, 1, info.volume);
        resultBoolean(out, 2, info.muted);
    }
    *outCount = 3;
    return true;
}

bool createSound(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    if (count < 2 || count > 12 || capacity < 2)
        return false;
    std::uint32_t engine = 0;
    std::uint32_t bus = 0;
    const char *path = nullptr;
    AudioSoundDescription description{};
    if (!uint32Value(args, count, 0, engine) || !stringValue(args, count, 1, path)
        || !uint32Value(args, count, 2, bus, 0, false)
        || !floatValue(args, count, 3, description.volume, 1.0f, false)
        || !floatValue(args, count, 4, description.pitch, 1.0f, false)
        || !floatValue(args, count, 5, description.pan, 0.0f, false)
        || !optionalBoolean(args, count, 6, description.looping)
        || !optionalBoolean(args, count, 7, description.spatialized)
        || !optionalBoolean(args, count, 8, description.streaming)
        || !optionalBoolean(args, count, 9, description.decode)
        || !optionalBoolean(args, count, 10, description.asynchronous)
        || !optionalBoolean(args, count, 11, description.disablePitch))
        return false;
    description.filePath = path;
    description.bus = bus;
    auto &api = audio(userData);
    AudioError error = AudioError::None;
    const AudioSoundHandle handle = api.createSound(api.self, engine, &description, &error);
    if (handle == invalidAudioSound)
        resultNil(out, 0);
    else
        resultNumber(out, 0, handle);
    resultError(out, 1, error);
    *outCount = 2;
    return true;
}

bool destroySound(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *, std::uint32_t, std::uint32_t *outCount)
{
    AudioEngineHandle engine = 0;
    AudioSoundHandle sound = 0;
    if (count != 2 || !parseEngineSound(args, count, engine, sound))
        return false;
    auto &api = audio(userData);
    api.destroySound(api.self, engine, sound);
    *outCount = 0;
    return true;
}

enum class SoundAction
{
    Play,
    Pause,
    Stop
};

// Shared body of play/pause/stop.
bool soundAction(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount, SoundAction action)
{
    AudioEngineHandle engine = 0;
    AudioSoundHandle sound = 0;
    if (count != 2 || capacity < 2 || !parseEngineSound(args, count, engine, sound))
        return false;
    auto &api = audio(userData);
    AudioError error = AudioError::None;
    bool ok = false;
    switch (action)
    {
    case SoundAction::Play:
        ok = api.play(api.self, engine, sound, &error);
        break;
    case SoundAction::Pause:
        ok = api.pause(api.self, engine, sound, &error);
        break;
    case SoundAction::Stop:
        ok = api.stop(api.self, engine, sound, &error);
        break;
    }
    resultBoolean(out, 0, ok);
    resultError(out, 1, error);
    *outCount = 2;
    return true;
}

bool play(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    return soundAction(userData, args, count, out, capacity, outCount, SoundAction::Play);
}

bool pause(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    return soundAction(userData, args, count, out, capacity, outCount, SoundAction::Pause);
}

bool stop(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    return soundAction(userData, args, count, out, capacity, outCount, SoundAction::Stop);
}

bool seekSeconds(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    AudioEngineHandle engine = 0;
    AudioSoundHandle sound = 0;
    double seconds = 0.0;
    if (count != 3 || capacity < 1 || !parseEngineSound(args, count, engine, sound) || !number(args, count, 2, seconds))
        return false;
    auto &api = audio(userData);
    resultBoolean(out, 0, api.seekSeconds(api.self, engine, sound, seconds));
    *outCount = 1;
    return true;
}

// Shared body of isPlaying/atEnd.
bool querySound(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount, bool end)
{
    AudioEngineHandle engine = 0;
    AudioSoundHandle sound = 0;
    if (count != 2 || capacity < 1 || !parseEngineSound(args, count, engine, sound))
        return false;
    auto &api = audio(userData);
    resultBoolean(out, 0, end ? api.atEnd(api.self, engine, sound) : api.isPlaying(api.self, engine, sound));
    *outCount = 1;
    return true;
}

bool isPlaying(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    return querySound(userData, args, count, out, capacity, outCount, false);
}

bool atEnd(void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    return querySound(userData, args, count, out, capacity, outCount, true);
}

bool soundTimes(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    AudioEngineHandle engine = 0;
    AudioSoundHandle sound = 0;
    if (count != 2 || capacity < 2 || !parseEngineSound(args, count, engine, sound))
        return false;
    double cursor = 0.0;
    double length = 0.0;
    auto &api = audio(userData);
    if (!api.soundTimes(api.self, engine, sound, &cursor, &length))
    {
        resultNil(out, 0);
        resultNil(out, 1);
    }
    else
    {
        resultNumber(out, 0, cursor);
        resultNumber(out, 1, length);
    }
    *outCount = 2;
    return true;
}

bool soundInfo(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    AudioEngineHandle engine = 0;
    AudioSoundHandle sound = 0;
    if (count != 2 || capacity < 6 || !parseEngineSound(args, count, engine, sound))
        return false;
    AudioSoundInfo info{};
    auto &api = audio(userData);
    if (!api.soundInfo(api.self, engine, sound, &info))
    {
        for (std::uint32_t index = 0; index < 6; ++index)
            resultNil(out, index);
    }
    else
    {
        resultNumber(out, 0, info.volume);
        resultNumber(out, 1, info.pitch);
        resultNumber(out, 2, info.pan);
        resultNumber(out, 3, info.bus == invalidAudioBus ? 0.0 : static_cast<double>(info.bus));
        resultBoolean(out, 4, info.looping);
        resultBoolean(out, 5, info.spatialized);
    }
    *outCount = 6;
    return true;
}

enum class FloatSetter
{
    Volume,
    Pitch,
    Pan,
    Doppler
};

// Shared body of the single-float sound setters.
bool floatSoundSetter(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount, FloatSetter which)
{
    AudioEngineHandle engine = 0;
    AudioSoundHandle sound = 0;
    float value = 0.0f;
    if (count != 3 || capacity < 1 || !parseEngineSound(args, count, engine, sound)
        || !floatValue(args, count, 2, value))
        return false;
    auto &api = audio(userData);
    bool ok = false;
    switch (which)
    {
    case FloatSetter::Volume:
        ok = api.setSoundVolume(api.self, engine, sound, value);
        break;
    case FloatSetter::Pitch:
        ok = api.setSoundPitch(api.self, engine, sound, value);
        break;
    case FloatSetter::Pan:
        ok = api.setSoundPan(api.self, engine, sound, value);
        break;
    case FloatSetter::Doppler:
        ok = api.setSoundDopplerFactor(api.self, engine, sound, value, nullptr);
        break;
    }
    resultBoolean(out, 0, ok);
    *outCount = 1;
    return true;
}

bool setSoundVolume(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    return floatSoundSetter(userData, args, count, out, capacity, outCount, FloatSetter::Volume);
}

bool setSoundPitch(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    return floatSoundSetter(userData, args, count, out, capacity, outCount, FloatSetter::Pitch);
}

bool setSoundPan(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    return floatSoundSetter(userData, args, count, out, capacity, outCount, FloatSetter::Pan);
}

bool setSoundDoppler(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    return floatSoundSetter(userData, args, count, out, capacity, outCount, FloatSetter::Doppler);
}

bool setSoundLooping(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    AudioEngineHandle engine = 0;
    AudioSoundHandle sound = 0;
    bool looping = false;
    if (count != 3 || capacity < 1 || !parseEngineSound(args, count, engine, sound)
        || !optionalBoolean(args, count, 2, looping, false, true))
        return false;
    auto &api = audio(userData);
    resultBoolean(out, 0, api.setSoundLooping(api.self, engine, sound, looping));
    *outCount = 1;
    return true;
}

enum class VectorSetter
{
    Position,
    Velocity,
    Direction
};

// Shared body of the 3-float sound setters.
bool vectorSoundSetter(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount, VectorSetter which)
{
    AudioEngineHandle engine = 0;
    AudioSoundHandle sound = 0;
    float values[3]{};
    if (count != 5 || capacity < 1 || !parseEngineSound(args, count, engine, sound)
        || !floatValue(args, count, 2, values[0]) || !floatValue(args, count, 3, values[1])
        || !floatValue(args, count, 4, values[2]))
        return false;
    auto &api = audio(userData);
    bool ok = false;
    switch (which)
    {
    case VectorSetter::Position:
        ok = api.setSoundPosition(api.self, engine, sound, values);
        break;
    case VectorSetter::Velocity:
        ok = api.setSoundVelocity(api.self, engine, sound, values);
        break;
    case VectorSetter::Direction:
        ok = api.setSoundDirection(api.self, engine, sound, values, nullptr);
        break;
    }
    resultBoolean(out, 0, ok);
    *outCount = 1;
    return true;
}

bool setSoundPosition(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    return vectorSoundSetter(userData, args, count, out, capacity, outCount, VectorSetter::Position);
}

bool setSoundVelocity(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    return vectorSoundSetter(userData, args, count, out, capacity, outCount, VectorSetter::Velocity);
}

bool setSoundDirection(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    return vectorSoundSetter(userData, args, count, out, capacity, outCount, VectorSetter::Direction);
}

bool setSoundCone(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    AudioEngineHandle engine = 0;
    AudioSoundHandle sound = 0;
    float innerAngle = 0.0f;
    float outerAngle = 0.0f;
    float outerGain = 0.0f;
    if (count != 5 || capacity < 1 || !parseEngineSound(args, count, engine, sound)
        || !floatValue(args, count, 2, innerAngle) || !floatValue(args, count, 3, outerAngle)
        || !floatValue(args, count, 4, outerGain))
        return false;
    auto &api = audio(userData);
    resultBoolean(out, 0, api.setSoundCone(api.self, engine, sound, innerAngle, outerAngle, outerGain, nullptr));
    *outCount = 1;
    return true;
}

bool setSoundListener(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    AudioEngineHandle engine = 0;
    AudioSoundHandle sound = 0;
    std::uint32_t listener = 0;
    if (count != 3 || capacity < 1 || !parseEngineSound(args, count, engine, sound)
        || !uint32Value(args, count, 2, listener, std::numeric_limits<std::uint32_t>::max()))
        return false;
    auto &api = audio(userData);
    resultBoolean(out, 0, api.setSoundListener(api.self, engine, sound, listener, nullptr));
    *outCount = 1;
    return true;
}

bool setSoundSpatialization(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    AudioEngineHandle engine = 0;
    AudioSoundHandle sound = 0;
    bool enabled = false;
    if (count != 3 || capacity < 1 || !parseEngineSound(args, count, engine, sound)
        || !optionalBoolean(args, count, 2, enabled, false, true))
        return false;
    auto &api = audio(userData);
    resultBoolean(out, 0, api.setSoundSpatializationEnabled(api.self, engine, sound, enabled, nullptr));
    *outCount = 1;
    return true;
}

bool setSoundGainRange(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    AudioEngineHandle engine = 0;
    AudioSoundHandle sound = 0;
    float minimum = 0.0f;
    float maximum = 0.0f;
    if (count != 4 || capacity < 1 || !parseEngineSound(args, count, engine, sound)
        || !floatValue(args, count, 2, minimum) || !floatValue(args, count, 3, maximum))
        return false;
    auto &api = audio(userData);
    resultBoolean(out, 0, api.setSoundGainRange(api.self, engine, sound, minimum, maximum, nullptr));
    *outCount = 1;
    return true;
}

bool setSoundAttenuation(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    AudioEngineHandle engine = 0;
    AudioSoundHandle sound = 0;
    float minimumDistance = 0.0f;
    float maximumDistance = 0.0f;
    float rolloff = 0.0f;
    std::uint32_t model = 0;
    if (count != 6 || capacity < 1 || !parseEngineSound(args, count, engine, sound)
        || !floatValue(args, count, 2, minimumDistance) || !floatValue(args, count, 3, maximumDistance)
        || !floatValue(args, count, 4, rolloff) || !uint32Value(args, count, 5, model))
        return false;
    if (model > static_cast<std::uint32_t>(AudioAttenuationModel::Exponential))
        return false;
    auto &api = audio(userData);
    resultBoolean(out, 0,
        api.setSoundAttenuation(api.self, engine, sound, minimumDistance, maximumDistance, rolloff,
            static_cast<AudioAttenuationModel>(model), nullptr));
    *outCount = 1;
    return true;
}

bool fadeSoundGain(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    AudioEngineHandle engine = 0;
    AudioSoundHandle sound = 0;
    float gain = 0.0f;
    double seconds = 0.0;
    if (count != 4 || capacity < 2 || !parseEngineSound(args, count, engine, sound)
        || !floatValue(args, count, 2, gain) || !number(args, count, 3, seconds))
        return false;
    auto &api = audio(userData);
    AudioError error = AudioError::None;
    const bool ok = api.fadeSoundGain(api.self, engine, sound, gain, seconds, &error);
    resultBoolean(out, 0, ok);
    resultError(out, 1, error);
    *outCount = 2;
    return true;
}

bool schedulePlay(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    AudioEngineHandle engine = 0;
    AudioSoundHandle sound = 0;
    double delay = 0.0;
    if (count != 3 || capacity < 2 || !parseEngineSound(args, count, engine, sound) || !number(args, count, 2, delay))
        return false;
    auto &api = audio(userData);
    AudioError error = AudioError::None;
    const bool ok = api.schedulePlay(api.self, engine, sound, delay, &error);
    resultBoolean(out, 0, ok);
    resultError(out, 1, error);
    *outCount = 2;
    return true;
}

bool scheduleStop(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    AudioEngineHandle engine = 0;
    AudioSoundHandle sound = 0;
    double delay = 0.0;
    double fade = 0.0;
    if (count != 4 || capacity < 2 || !parseEngineSound(args, count, engine, sound)
        || !number(args, count, 2, delay) || !number(args, count, 3, fade))
        return false;
    auto &api = audio(userData);
    AudioError error = AudioError::None;
    const bool ok = api.scheduleStop(api.self, engine, sound, delay, fade, &error);
    resultBoolean(out, 0, ok);
    resultError(out, 1, error);
    *outCount = 2;
    return true;
}

bool setListener(
    void *userData, const ScriptValue *args, std::uint32_t count, ScriptValue *out, std::uint32_t capacity,
    std::uint32_t *outCount)
{
    std::uint32_t engine = 0;
    std::uint32_t index = 0;
    AudioListenerDescription description{};
    if (count != 14 || capacity < 1 || !uint32Value(args, count, 0, engine) || !uint32Value(args, count, 1, index))
        return false;
    std::uint32_t next = 2;
    for (float &value : description.position)
    {
        if (!floatValue(args, count, next++, value))
            return false;
    }
    for (float &value : description.direction)
    {
        if (!floatValue(args, count, next++, value))
            return false;
    }
    for (float &value : description.worldUp)
    {
        if (!floatValue(args, count, next++, value))
            return false;
    }
    for (float &value : description.velocity)
    {
        if (!floatValue(args, count, next++, value))
            return false;
    }
    auto &api = audio(userData);
    resultBoolean(out, 0, api.setListener(api.self, engine, index, &description));
    *outCount = 1;
    return true;
}

struct Binding
{
    const char          *name;
    ScriptNativeFunction function;
};

constexpr Binding bindings[] = {
    {"audio.create_engine",                    createEngine           },
    {"audio.destroy_engine",                   destroyEngine          },
    {"audio.engine_info",                      engineInfo             },
    {"audio.engine_statistics",                engineStatistics       },
    {"audio.start_engine",                     startEngine            },
    {"audio.stop_engine",                      stopEngine             },
    {"audio.is_engine_started",                isEngineStarted        },
    {"audio.set_master_volume",                setMasterVolume        },
    {"audio.get_master_volume",                getMasterVolume        },
    {"audio.create_bus",                       createBus              },
    {"audio.destroy_bus",                      destroyBus             },
    {"audio.set_bus_volume",                   setBusVolume           },
    {"audio.set_bus_muted",                    setBusMuted            },
    {"audio.bus_info",                         busInfo                },
    {"audio.create_sound",                     createSound            },
    {"audio.destroy_sound",                    destroySound           },
    {"audio.play",                             play                   },
    {"audio.pause",                            pause                  },
    {"audio.stop",                             stop                   },
    {"audio.seek_seconds",                     seekSeconds            },
    {"audio.is_playing",                       isPlaying              },
    {"audio.at_end",                           atEnd                  },
    {"audio.sound_times",                      soundTimes             },
    {"audio.sound_info",                       soundInfo              },
    {"audio.set_sound_volume",                 setSoundVolume         },
    {"audio.set_sound_pitch",                  setSoundPitch          },
    {"audio.set_sound_pan",                    setSoundPan            },
    {"audio.set_sound_looping",                setSoundLooping        },
    {"audio.set_sound_position",               setSoundPosition       },
    {"audio.set_sound_velocity",               setSoundVelocity       },
    {"audio.set_sound_direction",              setSoundDirection      },
    {"audio.set_sound_cone",                   setSoundCone           },
    {"audio.set_sound_doppler_factor",         setSoundDoppler        },
    {"audio.set_sound_listener",               setSoundListener       },
    {"audio.set_sound_spatialization_enabled", setSoundSpatialization },
    {"audio.set_sound_gain_range",             setSoundGainRange      },
    {"audio.set_sound_attenuation",            setSoundAttenuation    },
    {"audio.fade_sound_gain",                  fadeSoundGain          },
    {"audio.schedule_play",                    schedulePlay           },
    {"audio.schedule_stop",                    scheduleStop           },
    {"audio.set_listener",                     setListener            },
};

} // namespace

bool initializeAudioLuaRuntime(void *userData, ScriptRuntimeHandle runtime) noexcept
{
    auto *context = static_cast<AudioLuaBridgeContext *>(userData);
    if (context == nullptr || context->scriptRuntime == nullptr || context->audio == nullptr)
        return false;

    // Every closure retains this same context pointer so script_runtime can
    // disable it safely when the module unregisters the initializer.
    try
    {
        for (const Binding &binding : bindings)
        {
            if (!context->scriptRuntime->registerNativeFunction(
                    context->scriptRuntime->self, runtime, binding.name, binding.function, context))
                return false;
        }
        return true;
    }
    catch (...)
    {
        return false;
    }
}

} // namespace engine::audio
