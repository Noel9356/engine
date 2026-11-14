# fur configuration

All keys live in the `[fur]` section of the module configuration
(`runtime/config/modules/fur/config.toml` holds the defaults); live reloads
are picked up through ConfigurationReloadedEvent like every module. Invalid
or out-of-range values are clamped or fall back to their defaults with a
stderr warning.

| key | default | meaning |
| --- | ------- | ------- |
| `default_preset` | `"plush"` | Preset the demo/UI defaults to: `plush`, `shaggy` or `velvety`. Unknown names warn and use `plush`. |
| `texture_size` | `256` | Default strand-texture edge length in pixels (16..2048) when a consumer does not pin one. |
| `max_layers` | `16` | Hard shell-layer ceiling applied by the LOD evaluator (1..64). The renderer additionally clamps with `[renderer] fur_max_layers`, so this key bounds the policy while that one bounds the worst case. |
| `lod_layer_counts` | `[14, 10, 6, 3, 1]` | Shell layer budget per LOD band, most detailed first (5 values). |
| `lod_screen_sizes` | `[0.32, 0.16, 0.08, 0.03]` | Angular-size band edges (worldRadius / distance), descending; band 5 catches everything below the smallest. |
| `lod_fade_fraction` | `0.35` | Fraction of each band (from its lower edge) over which the outermost layer's alpha dissolves before the next band's layer-count drop. |
| `lod_height_scales` | `[1.0, 1.0, 0.9, 0.8, 0.7]` | Strand-height multiplier per band; shrinking the coat at distance keeps reduced stacks from reading as sparse spikes. |

Tuning guidance:

- Lower `lod_layer_counts` band 0 (e.g. 10) on fill-rate-limited targets; the
  soft alpha ramp hides a good deal of layer reduction before it reads as
  quality loss.
- `lod_fade_fraction` trades smoothness against how long the full stack stays
  visible: larger values start fading earlier.
- The bands interact with `lod_screen_sizes`: keep them strictly descending
  (the module rejects a non-descending table wholesale and keeps the
  defaults, warning on stderr).
