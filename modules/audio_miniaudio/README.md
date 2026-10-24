# audio_miniaudio module - the miniaudio-backed provider

## What this module provides

Capability `audio` (implementation-bound member `audio.miniaudio`, published in `audio_capability.hpp`): sound and music playback. The backend is miniaudio, vendored as one pinned header (`third_party/miniaudio/`, v0.11.21) - the dependency ships with the repository, so audio works in an offline build exactly like every other module.

Two surfaces:

1. **The capability** (fine control): `loadSound(path, looping)` returns a stable handle (load-once semantics: the same path + looping mode returns the same handle), then `play` / `stop` / `setVolume` / `isPlaying` / `unload` / `setMasterVolume`. WAV, MP3, FLAC and OGG come with the decoder set.
2. **The event path** (fire-and-forget): any component may publish an `AudioPlaybackRequestEvent { path, volume, looping }` on the bus WITHOUT acquiring the capability - the module loads (caching by path) and plays it. That is the SFX path for modules that just want a click when something happens.

The provider is a 3D mixer: one listener (position + orientation, `setListenerPosition` / `setListenerOrientation`) and per-sound world positions - distance attenuates, panning follows the listener's orientation. `playSoundAt(path, position, volume)` is the positional one-shot (spatialized immediately at the given point); the event path accepts the same placement through `AudioPlaybackRequestEvent.position`. The default listener sits at the origin looking down -Z.

```lua
engine.play_sound("mods/my_mod/assets/impact.wav", 0.8)
engine.play_sound_3d("mods/my_mod/assets/portal.ogg", 3.0, 1.0, -2.0, 0.9)
engine.set_master_volume(0.5)
```

## Headless degradation

Without a device (CI containers, servers, a muted desktop), `ma_engine_init` falls through every backend and the module starts DEGRADED: every call stays legal, loads hand out ghost handles, plays are precise no-ops, one log line (and one `AudioDeviceUnavailableEvent`) says so. A run never fails because the machine has no sound hardware - automated runs play silence, interactive runs play sound, and neither has to branch on the difference.

miniaudio loads the platform device APIs (ALSA, PulseAudio) at runtime via `dlopen`: no audio dev packages are needed at build time, only the client library at play time on the player's machine.

## Configuration

```toml
[audio]
# master_volume = 1.0    # output volume at startup (0..1+)
# start_muted = false    # convenience for CI runs
```

## Files

- `audio_capability.hpp` - the published capability surface
- `audio.hpp` / `audio.cpp` - the module (the miniaudio implementation lives in the .cpp; the header carries only opaque types)
- `module_plugin.cpp` - the static plugin descriptor
- `CMakeLists.txt` - target wiring (`ENGINE_BUILD_MODULE_AUDIO_MINIAUDIO`)
- `third_party/miniaudio/miniaudio.h` - the vendored backend (pinned)
