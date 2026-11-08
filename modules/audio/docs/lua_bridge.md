# audio lua bridge

The audio module installs an `audio.*` Lua function namespace into every script runtime the engine hosts. Scripts use it to create audio engines, buses, and sounds; to drive playback (play/pause/stop/seek, fades, scheduled start/stop); and to control spatialization (positions, cones, attenuation, listeners). The bridge is optional: when `script_runtime` is absent the C++ `audio` capability works exactly as before, and scripts simply never see the `audio` table.

## Installation

The module declares `optional_capabilities = ["script_runtime"]` (module.toml and the entry table). During `construct()` it acquires the capability when present and calls `registerRuntimeInitializer(initializer, userData)` with `initializeAudioLuaRuntime` (source/lua_bridge.cpp). script_runtime then runs that initializer once for every runtime - runtimes created before registration included - so each runtime receives the same function set before its script executes. Unregistering the initializer (module shutdown) prevents future applications; existing closures hold the same context pointer, which lets script_runtime disable them safely.

## Values and errors

The bridge crosses the script boundary with script_runtime's four primitive value types only (`nil`, boolean, number, string):

- Handles (`AudioEngineHandle`, `AudioBusHandle`, `AudioSoundHandle`) are numbers. `0` means "none"; creation functions return `nil` in the handle slot when creation failed.
- Numbers must be finite; `inf`/`nan` arguments make the call fail. Integer-typed arguments must be integral and inside the C++ type's range. Optional trailing arguments may be omitted - an absent argument falls back to the same default the C++ description would use.
- Strings are borrowed for the duration of the call; paths are passed through to the backend unchanged.
- Functions that report backend errors return `(value, error_code)` where `error_code` is the numeric `AudioError` value: 0 `None`, 1 `BackendUnavailable`, 2 `CreationFailed`, 3 `InvalidHandle`, 4 `InvalidDescription`, 5 `ResourceFailed`, 6 `Busy`, 7 `Unsupported`, 8 `CapacityExceeded`. Functions without a dedicated error path return a plain success boolean.
- A malformed call (wrong argument count, wrong types, non-finite values) returns `false` or `nil` from the binding without touching audio state.

`readFrames` (pulling rendered samples from a device-less engine) is intentionally native-only: interleaved PCM buffers do not fit the primitive-value ABI.

## Function reference

Functions are listed alphabetically. `engine`/`bus`/`sound` are handles, `?` marks optional trailing arguments with their fallback.

| Function | Arguments | Returns | Notes |
| --- | --- | --- | --- |
| `audio.at_end` | engine, sound | boolean | True when the sound reached its end. |
| `audio.create_bus` | engine, parent? (=0), volume? (=1.0), muted? (=false) | handle or nil, error | Parent 0 creates a top-level bus. |
| `audio.create_engine` | sample_rate? (=0), channels? (=0), listeners? (=0), master_volume? (=-1), no_device? (=false), no_auto_start? (=false) | handle or nil, error | `0` sample rate/channels/listeners and negative master volume mean "use `[audio]` configuration defaults". `no_device = true` creates a manual-render engine (see `readFrames` note above). |
| `audio.create_sound` | engine, path, bus? (=0), volume? (=1.0), pitch? (=1.0), pan? (=0.0), looping? (=false), spatialized? (=false), streaming? (=false), decode? (=false), asynchronous? (=false), disable_pitch? (=false) | handle or nil, error | Flags map onto miniaudio load flags; `streaming` and `decode` are mutually exclusive. |
| `audio.destroy_bus` | engine, bus | (none) | Refuses buses with children or attached sounds (`Busy`). |
| `audio.destroy_engine` | engine | (none) | Releases all sounds and buses of the engine. |
| `audio.destroy_sound` | engine, sound | (none) | |
| `audio.engine_info` | engine | sample_rate, channels, has_device, backend_name | Returns `nil` for an unknown engine. |
| `audio.fade_sound_gain` | engine, sound, gain, seconds | ok, error | Frame-clock-based gain envelope on the engine graph. |
| `audio.is_playing` | engine, sound | boolean | |
| `audio.pause` | engine, sound | ok, error | Stops without rewinding; `play` resumes from the cursor. |
| `audio.play` | engine, sound | ok, error | |
| `audio.schedule_play` | engine, sound, delay_seconds | ok, error | Armed on the engine graph clock. |
| `audio.schedule_stop` | engine, sound, delay_seconds, fade_seconds | ok, error | Fade duration must not exceed the delay. |
| `audio.seek_seconds` | engine, sound, seconds | boolean | Converted to a PCM frame index using the sound's data-format sample rate. |
| `audio.set_bus_muted` | engine, bus, muted | boolean | |
| `audio.set_bus_volume` | engine, bus, volume | boolean | |
| `audio.set_listener` | engine, index, px, py, pz, dx, dy, dz, ux, uy, uz, vx, vy, vz | boolean | Position, direction, world-up, velocity (3 floats each); 14 arguments total. |
| `audio.set_master_volume` | engine, volume | boolean | |
| `audio.set_sound_attenuation` | engine, sound, min_distance, max_distance, rolloff, model | boolean | `model`: 0 `None`, 1 `Inverse`, 2 `Linear`, 3 `Exponential`. Distances must satisfy `0 < min <= max`. |
| `audio.set_sound_cone` | engine, sound, inner_radians, outer_radians, outer_gain | boolean | Requires `0 <= inner <= outer <= 2*pi`. |
| `audio.set_sound_direction` | engine, sound, x, y, z | boolean | |
| `audio.set_sound_doppler_factor` | engine, sound, factor | boolean | |
| `audio.set_sound_gain_range` | engine, sound, minimum, maximum | boolean | |
| `audio.set_sound_listener` | engine, sound, listener | boolean | Pins the sound to a listener index; miniaudio has no unpin operation. |
| `audio.set_sound_looping` | engine, sound, looping | boolean | |
| `audio.set_sound_pan` | engine, sound, pan | boolean | Pan must be in [-1, 1]. |
| `audio.set_sound_pitch` | engine, sound, pitch | boolean | |
| `audio.set_sound_position` | engine, sound, x, y, z | boolean | |
| `audio.set_sound_spatialization_enabled` | engine, sound, enabled | boolean | |
| `audio.set_sound_velocity` | engine, sound, x, y, z | boolean | |
| `audio.set_sound_volume` | engine, sound, volume | boolean | |
| `audio.sound_times` | engine, sound | cursor_seconds, length_seconds | Returns `nil, nil` for an unknown sound. |
| `audio.start_engine` | engine | ok, error | |
| `audio.stop` | engine, sound | ok, error | Stops and rewinds to PCM frame 0. |
| `audio.stop_engine` | engine | ok, error | |

## Limits

- script_runtime caps apply per runtime: at most `script_runtime.maximum_native_functions` (default 256) registered functions - the bridge registers 35 - at most `maximum_native_arguments` (default 32) per call (the widest call is `audio.set_listener` with 14) and at most `maximum_native_results` (default 16) per call (the widest is `audio.engine_info` with 4).
- The bridge runs on the caller's thread and serializes on the owning engine's mutex like the native capability; per-engine calls from scripts and native consumers are mutually exclusive but calls on different engines proceed concurrently.
- Object budgets (`audio.max_engines`, `audio.max_sounds_per_engine`, `audio.max_buses_per_engine`) surface as error code 8 (`CapacityExceeded`) in the second result.

## Example

```lua
local engine, error_code = audio.create_engine(48000, 2, 1)
assert(engine ~= nil, error_code)

local music, music_error = audio.create_sound(engine, "assets/audio/theme.ogg",
    0, 0.8, 1.0, 0.0, true)  -- default bus, 80% volume, looping
assert(music ~= nil, music_error)
audio.play(engine, music)

local shot, shot_error = audio.create_sound(engine, "assets/audio/impact.wav")
assert(shot ~= nil, shot_error)
audio.set_sound_attenuation(engine, shot, 1.0, 40.0, 1.0, 1)  -- inverse falloff
audio.set_sound_position(engine, shot, 3.0, 0.0, -2.0)
audio.play(engine, shot)

local cursor, length = audio.sound_times(engine, music)
if cursor ~= nil then
    print(string.format("music at %.1f / %.1f s", cursor, length))
end
```

See [configuration.md](configuration.md) for the `[audio]` keys behind the defaults and [overview.md](overview.md) for the native capability.
