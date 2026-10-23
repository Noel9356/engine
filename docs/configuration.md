# Configuration reference

Default file: `config/engine.toml`, resolved relative to the executable; `--config <path>` overrides. Parsed with toml++, so type errors report file and line. Run `./engine --validate` after every edit — it performs the full load/resolve/permission pass without opening a window, and warns about keys that no component declares.

## The layer stack

The effective configuration is assembled from layers, in decreasing precedence:

| Layer | Provided by | Merge rule |
|---|---|---|
| command line (`--set`) | the `--set key=value` flags, in the order given | replaces; highest authority |
| engine configuration | `config/engine.toml` (or `--config <path>`) | replaces |
| scene file | the world source: `--scene <path>`, or a content mod's declared scene file when the assembly defines no `[scene]` of its own (see below) | replaces the whole `[scene]` section, nothing else |
| module defaults | `config/modules/<name>/module.toml`, when a module ships one | fills gaps only |
| mod defaults | `<mod package>/config.toml`, per active package | fills gaps only |
| code defaults | each component's `value_or(...)` in source | reached when no layer above provided the key |

Rules the stack guarantees:

- **The engine configuration always wins over shipped files.** A `module.toml` or a mod's `config.toml` can never override a value the assembly provides; defaults only FILL keys nobody above them set. Between two defaults files providing the same key, the first merge wins — module defaults before mod defaults, each group in sorted name order.
- **Tables deep-merge; arrays and `[[scene.objects]]` rows replace wholesale.** A higher layer that sets `[[scene.objects]]` replaces the entire object list, never appends to it. That is what keeps scene authoring predictable: the objects a run uses are the objects the winning layer lists.
- **The scene file owns `[scene]`, and the world has exactly one source.** Precedence: `--scene` (explicit intent) > the assembly's own `[scene]` > an installed content mod's declared scene file (`scene = ...` in `mod.toml`, applied with the same wholesale swap) > code defaults. `--scene` swaps the world section out from under the assembly configuration wholesale: a key the assembly's `[scene]` had is gone unless the scene file lists it too. A scene file may define ONLY `[scene]` (sub-tables and array-of-tables rows included); any other top-level key, or a missing/`[scene]`-as-value, is a precise startup error. This is why `--scene` is useful with the shipped demo assembly: the content mod's demo objects do not leak into your world. Content mods may coexist freely: with an explicit world source their claims are simply not applied (`--validate` warns; an explicit `--scene` stays quiet), and only claims that would decide the world must be unambiguous — two or more active claims with no explicit source fail startup with a report naming every claim. World content never arrives through a defaults layer: a `config.toml` or `module.toml` that defines `[scene]` fails startup with a precise error, because per-key default merging would drip scene values into configurations that did not ask for them (a sealed-room test whose zero sky must stay zero no matter what content is installed).
- **The command line is absolute.** `--set` is parsed before any backend starts (a malformed assignment is a usage error, exit code 2) and applied after every other layer. Keys are dotted paths of bare segments (`scene.sun.intensity`, `test_scene`'s `renderer.texture_directory`); values use TOML syntax — numbers, booleans, quoted strings, arrays, inline tables — and any raw value TOML rejects becomes a literal string, so `--set window.title=My Room` works without quotes.
- **No hot reload.** The stack is assembled once, before any component constructs.

### Shipped defaults

A module directory may carry `module.toml` and a mod package may carry `config.toml` next to its `mod.toml`; the build copies both into the runtime layout and the kernel merges them as the two defaults layers above. Defaults files may only define sections, and never the assembly- or host-owned ones (`[engine]`, `[providers]`, `[permissions]`, `[diagnostics]`, `[leak_test]`) — a package that could grow the module list or deny permissions would break the authority model — and never `[scene]` (world content has a dedicated channel, see the layer rules above). Any such file fails startup with a precise error. The in-tree example is the content package's own: `mods/test_scene/config.toml` ships `renderer.texture_directory`, so the renderer finds the package's texture sets in every configuration that does not set the key itself; the assembly can override it from `engine.toml`, and `--set` overrides both.

### Provenance

Every effective value records the layer and source that provided it. `./engine --provenance` validates and then prints the full map:

```
configuration provenance (the layer that provided each effective value):
  renderer.texture_directory <- mod defaults 'test_scene'
  scene.camera.fov <- command line (--set)
  scene.objects <- scene file '/opt/engine/runtime/mods/test_scene/scene.toml'
  window.title <- engine configuration '/opt/engine/runtime/config/engine.toml'
  ...
```

The same records power the unknown-key warnings: when `--validate` reports `unknown configuration key 'renderer.exposur' (from 'config/engine.toml')`, the source is the provenance of the offending value.

### Known keys

Every component declares the configuration keys it reads — modules in their manifest (`knownConfigKeys`), mods as `config_keys` in `mod.toml` — and the kernel adds its own assembly/host keys (`[engine]`, `[providers]`, `[permissions]`, `[diagnostics]`, `[leak_test]`). `--validate` walks the effective table and reports anything nobody declares:

```
warning: unknown configuration key 'renderer.exposur' (from 'config/engine.toml')
warning: unknown configuration key 'windwo.title' (from 'config/engine.toml'; no component declares keys in '[windwo]')
```

Warnings never fail validation — dead keys are noise, not breakage. Keys that only a shipped defaults file provided are exempt: a package owns the keys it ships. Arrays of tables are declared at the array's path (`scene.objects`), since rows are authored wholesale. The per-module key tables in this documentation set are exactly the declared surfaces.

This document covers the GLOBAL and SHARED sections. Each module owns the documentation for its own settings, in its `README.md`, next to its code:

| Section | Documented in |
|---|---|
| `[engine]`, `[providers]`, `[permissions]` | here (assembly: graph wiring and grants) |
| `[logging]` | `../modules/logging/README.md` |
| `[window]` | `../modules/window/README.md` |
| `[input]` | `../modules/input/README.md` |
| `[renderer]` | `../modules/renderer/README.md` |
| `[screenshot]` | `../modules/screenshot/README.md` |
| `[diagnostics]` | here (host-owned opt-in reporting; see also `../host/README.md`) |
| `[lighting]` (shared hand-off key `max_roughness`), `[lighting.pbr]` | `../modules/pbr/README.md` |
| `[lighting.radiance_cascades]` | `../modules/radiance_cascades/README.md` |
| `[lighting.ray_reflections]` | `../modules/ray_reflections/README.md` |
| `[scene]` | here (shared world content — read by several modules) |
| `[leak_test]` | `../host/README.md` (host-only, consumed by `--leak-check`) |

Sections other than a component's own are read-only shared configuration by convention: the module that owns a section reads it, everyone else treats it as data.

## [engine]

| Key | Type | Default | Meaning |
|---|---|---|---|
| `required_capabilities` | string array | `[]` | capability demand of the assembly: every named capability pulls its providers into the active set, transitively |
| `mod_directory` | string | `"mods"` | directory scanned for mod packages |
| `modules_directory` | string | `"modules"` | directory scanned for module plugin binaries |
| `disabled_mods` | string array | `[]` | mod ids to skip at discovery |

There is no module list. Modules are activated purely by capability demand: a module loads exactly when a mod's manifest, or this `required_capabilities` list, requires a capability it provides (and an activated module's own requirements pull its dependencies transitively — the renderer brings `window` and `logging` with it). A capability nothing requires stays dormant: its plugin is never instantiated and its binary is released before any construct phase. A demanded capability with no available provider fails startup with a precise diagnostic naming the consumer and the alternatives; the composition root's demand lives here because it is assembly data, not engine code.

## [providers]

Picks one implementation when several provide the same capability, e.g. `window = "glfw_window"`. Keys are capability names, values are component names. Two providers with no selection fails startup, naming the candidates and the key that resolves them.

## [permissions]

| Key | Type | Meaning |
|---|---|---|
| `denied` | array of `[component, permission]` pairs | deny one declared permission of one component |

Modules and mods are both subject to denials; a denied permission fails startup precisely. An acquisition attempt without a grant is blocked at runtime with a `ComponentAccessError` naming the offender and the missing grant. The kernel also prints one structural advisory per mod that runs native machine code, requests permissions outside the `capability.*` namespace, or acquires a broad set of capabilities — see `architecture.md`, "Mod risk advisories".

## [diagnostics]

| Key | Type | Default | Meaning |
|---|---|---|---|
| `latency_report` | bool | `false` | record event dispatch latencies and print the per-channel report plus frame-time percentiles at shutdown |

Diagnostics are opt-in: with recording disabled the event bus never reads a clock, so a normal run pays nothing and stays quiet. The same report is available per run with `--latency-report`.

## [scene]

Shared world content, read by the renderer (rasterization) and the lighting modules (the exact analytic SDF the GI traces — one geometry truth). Object and light details also live in the technique docs: `../modules/pbr/README.md` (light seeding), `../modules/radiance_cascades/README.md` and `../modules/ray_reflections/README.md` (how the world is traced).

| Key | Type | Default | Meaning |
|---|---|---|---|
| `sky_radiance` | float[3] | `[0, 0, 0]` | base sky radiance; zero closes the scene completely |
| `sky.zenith` / `sky.horizon` / `sky.ground` | float[3] | `sky_radiance` | vertical gradient radiance (components left out fall back to the flat `sky_radiance`, so legacy configurations render unchanged) |
| `sky.sun_disk` | bool | `true` | draw the sun disk on the direct view and in reflections (never in the GI transport field) |
| `sky.sun_disk_size_degrees` | float | `0.6` | angular disk diameter; the disk radiance is the sun's irradiance divided by its solid angle, so it adds no energy |
| `camera.position` / `camera.look_at` | float[3] | see source | camera placement (the free-fly STARTING pose) |
| `camera.fov` / `camera.near` / `camera.far` | float | `60` / `0.05` / `500` | vertical field of view and clip planes |
| `camera.free_fly` | bool | `true` | free-fly controls: WASD flies, mouse looks while the cursor is captured (Esc releases, F or left-click re-captures), Space/E rises, Ctrl/Q sinks, Shift boosts, the wheel scales `fly_speed` |
| `camera.fly_speed` / `camera.fly_boost` | float | `4` / `4` | base speed (world units per second) and the Shift boost multiplier (>= 1) |
| `camera.look_sensitivity` | float | `0.0022` | yaw/pitch radians per cursor pixel |
| `sun.direction` / `sun.color` / `sun.intensity` | float[3]/float[3]/float | off | seeds the directional light and the sky's sun disk (`direction` points towards the scene) |
| `objects` | array of tables | `[]` | scene objects (max 32) |

Each `[[scene.objects]]` row: `type` (`box`, `sphere`, or `plane` - authoring sugar for a thin box so rasterized and traced geometry stay the same primitive), `position`, `size` (box half extents / sphere radius), `rotation_y` (degrees), `albedo`, `roughness`, `metallic`, `emissive_strength` (emissive radiance is albedo times this value), `texture` (name of a PBR texture set under `[renderer].texture_directory` as `<name>/` - a folder with `Color.png`, `NormalGL.png`, `Roughness.png` and optional `Metalness.png`; all sets used by one scene must share one resolution), and `uv_scale` (world meters per texture tile, default 1). An object with a texture set takes albedo/roughness/metallic/normal from its maps; the flat values are the fallback. Point lights can be seeded with `[[scene.point_lights]]` rows (`position`, `color`, `intensity`, `radius`).

Where the world itself comes from is a configuration-source question, not a `[scene]` key: the shipped demo world lives in the content mod (`mods/test_scene/scene.toml`, declared through the manifest's `scene` key), and an assembly or `--scene` file that defines `[scene]` replaces it wholesale (precedence in "The layer stack"). The content package also ships the texture sets and the `renderer.texture_directory` default pointing at them; a scene naming texture sets with no directory configured fails with a precise error instead of rendering flat materials.

The demo scene ships five CC0 texture sets from [ambientCG](https://ambientcg.com): `WoodFloor055`, `Plaster001`, `Bricks070`, `Concrete034`, `Metal049A`.

## Minimal example

```toml
[engine]
required_capabilities = ["window", "input", "renderer"]

[window]
title = "my demo"

[renderer]
enable_validation = false
```

This assembly demands the window/input/renderer capabilities directly; their providers pull `logging` (and, through the shipped content mod's `scripting` requirement, the script host) in transitively. The assembly defines no `[scene]`, so the installed content mod's world renders (the shipped `test_scene`). Mod packages still load from the default `mods/` directory; add `mod_directory`/`disabled_mods` under `[engine]` to control them — disabling the content mod also removes its world, and with it the `scripting` demand, so the script host module stays dormant too. Remove `input` from the demand list and the input module never loads: startup gets faster and the camera stays at its configured pose.

## Layered one-liners

The same assembly, without touching any file:

```sh
# A different world: the scene file owns [scene] wholesale.
./engine --scene my_world.toml

# Command-line tweaks on top of any assembly - repeatable, highest layer.
./engine --set window.title="My Room" --set scene.sun.intensity=5.0 --set scene.camera.fov=90

# Which layer provided what (validates first).
./engine --validate --provenance
```

A `--scene` file holds exactly the `[scene]` section — copy any assembly-side key (the camera pose, for instance) into it that the swapped world should keep:

```toml
[scene]
sky_radiance = [0.05, 0.05, 0.08]

[scene.camera]
position = [0.0, 1.5, 3.0]
look_at = [0.0, 0.5, 0.0]

[[scene.objects]]
type = "plane"
position = [0.0, 0.0, 0.0]
size = [3.0, 0.01, 3.0]
albedo = [0.6, 0.6, 0.6]
```
