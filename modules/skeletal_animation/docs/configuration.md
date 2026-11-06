# skeletal_animation configuration

All keys live in the `[skeletal_animation]` section. Defaults ship in `config.toml`; the application config (or a profile) may override them. Every key is read once in `construct()`.

| Key | Type | Valid range / values | Default | Effect |
| --- | --- | --- | --- | --- |
| `maximum_joints` | integer | 1 – 65536 | `4096` | Largest single skeleton, joint-index limit for clip tracks, and maximum joint count accepted by pose blending and additive operations. |
| `maximum_clips` | integer | 1 – 65536 | `4096` | Maximum concurrently existing clips. |
| `maximum_players` | integer | 1 – 65536 | `4096` | Maximum concurrently existing players. |
| `maximum_skeletons` | integer | 1 – 65536 | `4096` | Maximum concurrently existing skeletons. |
| `maximum_tracks_per_clip` | integer | 1 – 65536 | `4096` | Maximum tracks accepted for one clip. |
| `maximum_keyframes_per_track` | integer | 1 – 1000000 | `65536` | Maximum keyframes accepted in one track. |
| `maximum_total_keyframes` | integer | 1 – 100000000 | `8000000` | Module-wide keyframe budget shared by all clips. |
| `maximum_total_joints` | integer | 1 – 10000000 | `1000000` | Module-wide joint budget shared by all skeletons. |
| `maximum_blend_poses` | integer | 1 – 256 | `16` | Maximum input poses accepted by `blendPoses`. |
| `sampling_cache_enabled` | boolean | `true` / `false` | `true` | Enables per-player track cursors in `evaluatePlayer`. Sequential playback can reuse the previous key interval instead of binary-searching every track. `sampleClip` is stateless and always uses binary search. |
| `maximum_total_sampling_cache_entries` | integer | 1 – 100000000 | `1000000` | Global limit on cached track cursors. Each entry is one 32-bit key index (4 bytes); players that exceed the remaining budget transparently use binary-search sampling. |
| `maximum_cursor_walk` | integer | 0 – 1000000 | `8` | Maximum adjacent key intervals walked before falling back to binary search. `0` always uses binary search; larger values favor sequential playback with short seeks. |
| `default_interpolation` | string | `slerp` / `nlerp` / `step` | `slerp` | Default rotation interpolation for tracks whose `AnimationTrackDescription::interpolation` is `Default`. `nlerp` is cheaper; `slerp` follows spherical interpolation more closely; `step` holds the previous key until the next key. Each track may override this default. |

Out-of-range integer values, missing values, and invalid strings are ignored and the built-in default stays active. Budgets are enforced when objects or sampling caches are created. Lowering a configured limit affects new allocations; it does not evict existing objects. `stop()` releases all records and resets usage counters.
