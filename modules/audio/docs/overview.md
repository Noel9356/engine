# audio module

audio gives the engine sound playback: it wraps the miniaudio library behind the `audio` capability so consumers can create audio engines, mix buses, and sounds (optionally 3D-spatialized), play/pause/seek them, and drive a device-less engine manually. When the module is built without `ENGINE_AUDIO_BACKEND_MINIAUDIO` every capability entry fails gracefully with `AudioError::BackendUnavailable`.

## Capability

The module provides the `audio` capability (ABI version constant `audioCapabilityAbi = 2`, see `include/engine/audio/capabilities.hpp`). All calls take the capability's `self` pointer first; handles are `std::uint32_t` and never reused, with `0` (`invalidAudioEngine` / `invalidAudioSound` / `invalidAudioBus`) meaning "none". Optional `AudioError*` out-parameters are always written on failing paths that report errors.

Main entries: `createEngine`/`destroyEngine`/`engineInfo`/`startEngine`/ `stopEngine`/`setMasterVolume`, bus management (`createBus`, `destroyBus`, `setBusVolume`, `setBusMuted`), sound management (`createSound`, `destroySound`, `play`, `pause`, `stop`, `seekSeconds`, `isPlaying`, `atEnd`, `soundTimes`, per-sound setters for volume/pitch/pan/looping/position/velocity/direction/cone/Doppler/listener/spatialization/gain range/attenuation), plus gain fades and clock-scheduled play/stop, `setListener`, and `readFrames` for manually pulling rendered audio from a device-less engine.

## Lua bridge

When `script_runtime` is available, the module automatically installs an `audio` Lua table into every subsequently created runtime before its script executes. The bridge is optional; native consumers can use the C++ capability without Lua. Handles are numeric, booleans remain booleans, and error values are the numeric `AudioError` enum values. Creation functions return `(handleOrNil, errorCode)`; operations with explicit backend errors return `(success, errorCode)`.

Available functions cover engine, bus, and sound creation/destruction; engine start/stop and info; playback, seeking, and time queries; sound and listener spatialization; gain/attenuation controls; fades; and scheduled playback. Manual `readFrames` remains native-only because the primitive-value Lua ABI does not transport interleaved sample buffers efficiently.

```lua
local engine, error_code = audio.create_engine(48000, 2, 1)
assert(engine ~= nil, error_code)
local sound, sound_error = audio.create_sound(engine, "assets/audio/impact.wav")
assert(sound ~= nil, sound_error)
assert(audio.play(engine, sound))
```

The bridge uses the `script_runtime` runtime-initializer extension. `script_runtime` is optional, so the C++ audio capability remains available without Lua. The complete function reference lives in [lua_bridge.md](lua_bridge.md).

## Provided / required / optional capabilities

- `module.toml`: `provides = ["audio"]`, `requires = []`, `optional_capabilities = ["script_runtime"]`.
- The entry table declares the same optional capability. No additional permissions are declared.
## Architecture

| File | Role |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capability, declared `config_keys`. Read by kernel module discovery. |
| `config.toml` | Default values for the `[audio]` configuration section (see `configuration.md`). |
| `include/engine/audio/capabilities.hpp` | Public capability types: ABI, handles, error/attenuation enums, engine/bus/sound/listener descriptions, function-pointer table. |
| `source/audio.cpp` | Validation helpers, per-engine state and locking, miniaudio resource-manager ownership, capability binding, configuration, and module entry table. |
| `source/lua_bridge.cpp` / `source/lua_bridge.hpp` | Automatic Lua namespace registration and primitive-value wrappers for engine, bus, sound, and listener operations. |

## How it works

- **Records.** `EngineRecord` owns one `ma_engine` plus hash maps that store `SoundRecord` and `BusRecord` values directly. This removes one heap allocation per sound/bus while node-based hash-map storage keeps each embedded miniaudio object at a stable address. Sounds and buses are strictly owned by their engine. Buses form a tree via `AudioBusDescription::parent`; direct child/sound counts make `destroyBus` ownership checks constant-time, and reverse creation order releases buses leaf-first in linear time.
- **Defaults resolution.** `createEngine` treats description fields of `0` (and a negative `masterVolume`) as "unset" and substitutes the values read from `[audio]` configuration in `construct()`; `noDevice` engines require nonzero sample rate and channel count after resolving the per-engine description against configured defaults.
- **Validation.** Small helpers (`finiteNonNegative`, `finitePositive`, `finiteVector`) gate every float that crosses the capability boundary; `createSound` additionally checks pan in [-1, 1], positive distances with max >= min, mutually exclusive streaming/decode, and a known attenuation model. Invalid input yields `AudioError::InvalidDescription`, backend failures `CreationFailed` / `ResourceFailed`.
- **Sound playback.** `play`/`pause` go through a shared `soundAction` runner that resolves both handles and applies one `ma_sound` call. `pause` is `ma_sound_stop` without rewinding (play resumes from the cursor), while `stop` also seeks back to PCM frame 0. `seekSeconds` looks up the sound's data-format sample rate and converts seconds to a PCM frame index in `long double` so huge values overflow-check cleanly. Creation flags map directly onto miniaudio load flags (`MA_SOUND_FLAG_STREAM`, `MA_SOUND_FLAG_DECODE`, `MA_SOUND_FLAG_ASYNC`, `MA_SOUND_FLAG_NO_SPATIALIZATION`, `MA_SOUND_FLAG_NO_PITCH`).
- **Device-less rendering.** Engines created with `noDevice` produce no output themselves; `readFrames` pulls interleaved frames from `ma_engine_read_pcm_frames`. Calling `readFrames` on an engine that owns a real device returns `Unsupported` (that engine renders on its own audio thread).
- **Resource management.** Each engine can own a resource manager, or the module can lazily create one shared manager for all engines. Worker count and decoded output format/channels/sample rate are configurable. Choosing float32 and a common decoded sample rate can move format conversion/resampling to loading rather than the mixer callback. Shared mode allows overlapping assets to reuse resource-manager data; per-engine mode isolates resource pools.
- **Capacity planning.** Initial map capacities can be reserved from configuration to reduce rehashes. Optional maximum engine, sound, and bus counts provide hard resource budgets and report `AudioError::CapacityExceeded`. The engine, sound, and bus handle counters are atomic and never wrap into reused handles.
- **Graceful no-backend build.** Every Impl checks `ENGINE_AUDIO_BACKEND_MINIAUDIO`; without it the entry fails with `BackendUnavailable` (or `false`) and no backend state is touched.

## Threading and lifetime

The module registry has a short-lived mutex for engine lookup/insertion. Every engine has its own mutex, so calls on different engines can execute concurrently while calls touching the same engine are serialized. Lifecycle changes use a separate module mutex; it is not taken by normal per-engine operations. Engine lookup returns a `shared_ptr` lease: destroying an engine removes it from the registry, waits for in-flight operations, then closes it; operations that acquired a lease before destruction observe the closed state and do not touch released miniaudio objects. The miniaudio device callback runs on miniaudio's own audio thread, independent of these API locks. Per-engine resource managers create their own worker pool; shared mode reuses one module-level pool. Engines, buses, and sounds live until explicitly destroyed; destroying an engine releases sounds, buses, engine, then its owned resource manager. Shared resource management remains alive until module `stop()` has closed all engines. `destroyBus` refuses buses with children or attached sounds (`AudioError::Busy`). Handles are allocated from 64-bit atomic counters, exposed as monotonically increasing 32-bit values, and permanently exhaust rather than wrapping and reusing an earlier handle.

## Configuration

Configurable — see [configuration.md](configuration.md) for output defaults, resource-manager policy, capacity planning, and compile-time miniaudio backend selection.

## Integration

The module does not use the kernel event bus at all: it publishes no events and subscribes to none (no `eventBus` usage in the source). It declares no permissions (`declaredPermissions = nullptr` in the entry table).

Consumer usage with the real API signatures:

```cpp
// In a module's construct(): throws ParticipantAccessError when the
// permission system blocks "audio" for this participant.
engine::audio::AudioCapability &audio
    = context.acquireCapability<engine::audio::AudioCapability>();

engine::audio::AudioError error{};
engine::audio::AudioEngineDescription engineDescription{};
engineDescription.sampleRate = 48000;
engineDescription.channels   = 2;
const engine::audio::AudioEngineHandle engine
    = audio.createEngine(audio.self, &engineDescription, &error);

engine::audio::AudioSoundDescription soundDescription{};
soundDescription.filePath     = "assets/audio/impact.wav";
soundDescription.spatialized  = true;
soundDescription.attenuation  = engine::audio::AudioAttenuationModel::Inverse;
const engine::audio::AudioSoundHandle sound
    = audio.createSound(audio.self, engine, &soundDescription, &error);

audio.play(audio.self, engine, sound, &error);

// Later: double cursor/length, then tear down.
double cursorSeconds = 0.0;
double lengthSeconds = 0.0;
audio.soundTimes(audio.self, engine, sound, &cursorSeconds, &lengthSeconds);
audio.destroySound(audio.self, engine, sound);
audio.destroyEngine(audio.self, engine);
```
