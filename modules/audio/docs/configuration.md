# audio configuration

All keys live in the `[audio]` section. Defaults ship in `config.toml`; the application config or a profile may override them. The `config_keys` list in `module.toml` mirrors this table. Values are read once in `construct()`. Invalid values are ignored and the built-in default remains active.

## Output defaults

| Key | Type | Valid range | Default | Effect |
| --- | --- | --- | --- | --- |
| `audio.sample_rate` | integer | 0 – `UINT32_MAX` | `0` | Default engine sample rate. `0` uses the selected device's native rate. |
| `audio.channels` | integer | 0 – 254 | `0` | Default engine channel count. `0` uses the selected device's native channel count. |
| `audio.listener_count` | integer | 1 – `MA_ENGINE_MAX_LISTENERS` | `1` | Default number of spatialization listeners. The accepted maximum follows the backend build rather than a module-specific listener limit. |
| `audio.master_volume` | finite float | `>= 0` | `1.0` | Default linear master gain. Values above `1.0` amplify. |

A device-less engine requires nonzero sample rate and channel count after resolving its `AudioEngineDescription` against these defaults.

## Mixer tuning

| Key | Type | Valid range | Default | Effect |
| --- | --- | --- | --- | --- |
| `audio.period_size_in_frames` | integer | 0 – `UINT32_MAX` | `0` | Requested device period size. Zero leaves selection to miniaudio/backend; smaller periods may reduce latency at the cost of increased scheduling pressure. |
| `audio.gain_smooth_time_in_frames` | integer | 0 – `UINT32_MAX` | `0` | Mixer gain smoothing interval in frames. Zero uses miniaudio's default. |
| `audio.default_volume_smooth_time_in_frames` | integer | 0 – `UINT32_MAX` | `0` | Default volume smoothing interval in PCM frames. Zero uses miniaudio's default. |
| `audio.pre_mix_stack_size_in_bytes` | integer | 0 – `UINT32_MAX` | `0` | Stack budget for the pre-mix node graph. Increase it for deeper node graphs; zero uses miniaudio's default. |

Each field can also be overridden for an individual engine through `AudioEngineDescription`. These are tuning controls, not guaranteed latency or CPU targets; validate them on the target backend and workload.

## Resource manager

| Key | Type | Valid range | Default | Effect |
| --- | --- | --- | --- | --- |
| `audio.share_resources` | boolean | `true` / `false` | `false` | Use one module-owned resource manager for all engines. Identical assets can then share loaded data across engines. When disabled, every engine owns an isolated manager. |
| `audio.resource_manager_job_threads` | integer | 1 – 64 | `1` | Number of miniaudio resource-manager workers used for asynchronous loading and streaming jobs. In per-engine mode, each engine creates this many workers; shared mode uses one worker pool for the module. |
| `audio.decoded_format` | string | `native`, `f32`, `float32` | `native` | `native` preserves the source format. `f32`/`float32` converts decoded resources to 32-bit float during loading. |
| `audio.decoded_channels` | integer | 0 – 32 | `0` | Decoded channel count. `0` preserves each source's channel count. |
| `audio.decoded_sample_rate` | integer | `0` or 8000 – 384000 | `0` | Decoded sample rate. `0` preserves each source's sample rate. A common decoded rate can move resampling away from the mix callback. |

`share_resources = true` is useful when multiple engines load overlapping asset sets and avoids creating a separate resource-manager worker pool for every engine. It also couples their resource-manager configuration and lifetime: the manager is created lazily on the first engine and released by module `stop()` after all engines are closed. Choose per-engine managers when separate resource pools or independent manager lifetimes are more important.

For a high-throughput game mixer, a typical starting point is `decoded_format = "f32"`, `decoded_sample_rate` equal to the engine output rate, and `share_resources = true` when engines use overlapping assets. `decoded_channels = 0` preserves source channel layouts, which can be important for spatialized multichannel material. These are workload-dependent settings, not universally optimal defaults.

## Capacity and allocation policy

| Key | Type | Valid range | Default | Effect |
| --- | --- | --- | --- | --- |
| `audio.initial_engine_capacity` | integer | 0 – 65536 | `4` | Initial engine-registry bucket reservation. |
| `audio.initial_sound_capacity` | integer | 0 – 1048576 | `64` | Initial per-engine sound-map bucket reservation. |
| `audio.initial_bus_capacity` | integer | 0 – 65536 | `8` | Initial per-engine bus-map and teardown-order reservation. |
| `audio.max_engines` | integer | 0 – 1048576 | `0` | Maximum live engines. `0` is unbounded. |
| `audio.max_sounds_per_engine` | integer | 0 – 1048576 | `0` | Maximum live sounds per engine. `0` is unbounded. |
| `audio.max_buses_per_engine` | integer | 0 – 65536 | `0` | Maximum live buses per engine. `0` is unbounded. |

Initial capacities reserve hash-map buckets; they do not preallocate sound or bus objects and are not limits. Set them near the expected peak object counts to reduce rehashing during gameplay. The `max_*` values enforce hard limits and failures report `AudioError::CapacityExceeded`.

## Build-time backend selection

The audio module CMake options are:

| Option | Default | Effect |
| --- | --- | --- |
| `ENGINE_AUDIO_ENABLE_MINIAUDIO` | `ON` | Build the miniaudio playback/mixer backend. `OFF` builds the capability shell, whose operations report `BackendUnavailable`. |
| `ENGINE_AUDIO_MINIAUDIO_BACKENDS` | `DEFAULT` | Build all platform-supported miniaudio backends, or specify a semicolon-separated allow-list such as `WASAPI` or `ALSA;PULSEAUDIO`. Supported names: `WASAPI`, `DSOUND`, `WINMM`, `ALSA`, `PULSEAUDIO`, `JACK`, `COREAUDIO`, `SNDIO`, `AUDIO4`, `OSS`, `AAudio`, `OPENSL`, `WEBAUDIO`, `CUSTOM`, `NULL`. |
| `ENGINE_AUDIO_MINIAUDIO_DISABLE_RUNTIME_LINKING` | `OFF` | Define `MA_NO_RUNTIME_LINKING`; useful on platforms with restricted dynamic loading. Requires an explicit backend allow-list so required system libraries can be linked. On Unix, ALSA/PulseAudio/JACK/sndio development packages are resolved through pkg-config. |

Backend selection is compile-time. Select only backends supported by the target platform; the selected macros are applied to both miniaudio and the module so their miniaudio structure layouts agree.

## Sound envelopes and spatial controls

The capability exposes `fadeSoundGain`, `schedulePlay`, and `scheduleStop` for frame-clock-based envelopes and transport scheduling. Delays/durations are expressed in seconds at the API boundary and converted using the owning engine's actual sample rate. A scheduled start is armed on the engine graph clock; a scheduled stop can include a fade whose duration does not exceed its delay.

When `spatialized = true`, sound descriptions can set emitter direction, minimum/maximum gain, Doppler factor, listener index, and a directional cone. Supplying a cone or explicit listener index while spatialization is disabled is rejected. Angles are radians, cone angles must satisfy `0 <= inner <= outer <= 2*pi`, and `coneOuterGain` is linear gain. `invalidAudioListener` leaves a newly created sound on the backend's closest listener. Runtime `setSoundListener` only supports pinning to a valid listener; miniaudio does not expose a public unpin operation. The runtime setters allow these values to change without recreating a sound. `setSoundGainRange` changes minimum/maximum gain; `setSoundAttenuation` changes minimum/maximum distance, rolloff, and attenuation model together.
