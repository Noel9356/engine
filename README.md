# engine

A modular C++20 engine kernel. The kernel owns three things only: component lifecycle, the event bus and the permission system. Every feature — windows, rendering, input, screenshots, scripting — lives in an interchangeable module behind a capability it publishes, and mods (Lua scripts or C++ shared libraries) consume the same capabilities.

Capabilities have names, and names come in two shapes: implementation-agnostic families (`renderer`) and implementation-bound members (`renderer.vulkan`). Each module publishes its capability vocabulary from its own directory (`window_capability.hpp` lives in `modules/window/`), so the provider and its types are one unit: consumers include the provider's header directly, and there is no central contract directory that every new capability has to touch. Nothing is registered anywhere — not modules, not capabilities: acquisition is a pull that asks the declaring provider for its implementation at the moment of the call.

The kernel is a pure mediator. It never names a module and never interprets a capability: modules are anonymous capability providers that describe themselves through static plugin descriptors, mods are anonymous consumers that name only the capabilities they require, and a module is instantiated exactly when an active mod's manifest requires a capability it provides — transitively, through the providers' own requirements. Nothing registers a module anywhere; a run with no mods loads no modules at all. The kernel also enforces permissions: a component that touches a capability it did not declare is blocked, and every mod is risk-assessed for the user before its code runs.

## Quick start

Requirements: CMake 3.24+, a C++20 compiler, a Vulkan loader, and `glslangValidator`. A complete Vulkan SDK (headers incl. `vulkan.hpp` plus loader) is detected and used as-is; the pinned fallback fetches only when something is missing. See `docs/building.md`.

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
./build/runtime/engine --validate  # dry run: resolve, check, report, exit
./build/runtime/engine             # textured 3D PBR scene with radiance-cascades GI
                                   # and ray-traced reflections; F12 screenshot
                                   # (to ./screenshots), Esc quit
```

The run is driven by the one shipped content mod, `mods/test_scene`: its manifest requires every capability the demo pipeline consumes (script host, logging, window, input, renderer, lighting, the three radiance layers, screenshot), and that demand — nothing else — decides which module plugins load. The world comes from the same package: it declares `scene.toml` (the Cornell-style room, camera, sun) and carries the PBR texture sets it references. The demo camera is a free-fly camera (starts at the configured `[scene.camera]` pose): **WASD** fly, **mouse** look while the cursor is captured (**Esc** releases; **F** or left-click re-captures), **Space/E** rise, **Ctrl/Q** sink, **Shift** speed boost, **mouse wheel** fly speed. Set `[scene.camera] free_fly = false` for a fixed tripod camera.

Runs stay quiet at shutdown. Developers can opt into a communication latency report (per-channel event dispatch mean and percentiles) plus frame-time percentiles with `--latency-report` or `[diagnostics] latency_report = true`.

### The dynamic world (expansion modules)

The demo run now exercises the full content stack end to end: the script spawns an emissive orb and a physics-driven ball at startup, the orb bobs on an animation tween, and the ball falls and bounces under the builtin physics provider. The pipeline behind that:

- **scene** — the runtime object registry (`engine.spawn_object` and friends): the static `[scene]` world is pushed into it at startup, scripts and modules mutate it at runtime, and the renderer rebuilds its GPU data incrementally (content-only re-upload of fixed-capacity buffers; no pipeline or descriptor ever invalidates).
- **animation** — tween drivers over the registry and the light state (`engine.animate_object` / `engine.animate_light`): channels, easing curves, loop/ping-pong modes, delay.
- **physics** — the builtin rigid body provider (`engine.add_physics_body`): gravity, restitution, sphere/box contacts, a ground plane; collision pair culling through the kernel's BVH. A heavier provider (e.g. Jolt) slots in as another `[providers]` member without touching mods.
- **asset** — glTF 2.0 import into the registry (`engine.import_gltf`): node hierarchy, per-primitive bounding boxes, material factors. `.gltf` JSON; bounding-box proxies per the analytic renderer contract (the module README documents the v1 mapping).
- **audio** — the miniaudio provider (`engine.play_sound`), vendored pinned single header; degrades to silent no-ops on device-less machines.
- **hot_reload** — the development-loop watcher (add `hot_reload` to a mod's required capabilities): recompiled shaders re-create the renderer's pipelines, edited mod scripts reload live, config/manifest changes log.
- **input** now covers gamepads (polled buttons/axes + connect/disconnect events), text input (`TextInputEvent`, `engine.subscribe_text_input`), and action mapping (`[input.actions]` in configuration, `engine.action_strength("jump")`).
- **renderer** grew bloom (opt-in `[renderer] bloom = true`: HDR bright pass + separable blur added back pre-tonemap) and script-addressable camera pose (`engine.set_camera_position` / `camera_look_at`).
- **lua_scripting** grew input polling, timers (`engine.create_timer`), per-call permission re-checking across every capability-backed bridge call.

Kernel additions: `engine/kernel/async_dispatcher.hpp` (the sanctioned worker-thread → main-thread bridge) and `engine/kernel/bounding_volume_hierarchy.hpp` (CPU-side spatial queries; unit tested against brute force).

Dependencies are never fetched when a usable copy already exists: existing target, system package, vendored copy, populated build tree, then pinned download. Every configure prints an origin report showing what came from where.

## Documentation

Documentation lives NEXT TO THE CODE IT DESCRIBES: every module and the content package carry their own `README.md`, and `docs/` keeps only the cross-cutting guides.

| Document | Contents |
|---|---|
| `docs/architecture.md` | system map, capability surfaces and families, capability demand, pure-pull acquisition, components and mods, event bus, permissions, lifecycle, conventions |
| `docs/building.md` | configure/build, dependency tiers, `_deps` layout, offline controls, module options |
| `docs/configuration.md` | global and shared `engine.toml` sections ([engine], [providers], [permissions], [scene], [diagnostics]); per-module keys are documented by their owning module |
| `mods/README.md` | mod packages: the `mod.toml` schema, capability demand, world content, script and native authoring, portability rules |
| `modules/<name>/README.md` | one guide per module (sixteen total): capability wiring, its configuration keys with shipped defaults, implementation notes |
| `docs/robustness.md` | the engine's robustness problem log: the synchronization and lifetime invariants code comments cite by number |
| `.github/workflows/ci.yml` | continuous integration: build, kernel unit tests, headless validation, smoke frames under Xvfb + lavapipe |
| `mods/test_scene/README.md` | the content package: what it ships and how the world reaches the renderer |

## Layout

- `kernel/` — lifecycle, event bus, permission system, dependency resolution, configuration, mod discovery, the async task dispatcher (worker-thread work with main-thread completions), and the bounding volume hierarchy (CPU-side spatial queries); it includes no capability header and interprets no capability name
- `modules/<name>/` — sixteen modules, each with its own `README.md`: logging, window, input, renderer (with its own `shaders/` — module code, compiled at build time), screenshot, lua_scripting, plus the lighting technique modules pbr, radiance_cascades and ray_reflections (named after their algorithms), and the expansion modules **scene** (the runtime object registry), **animation** (tween drivers), **asset** (glTF 2.0 import), **physics** (the builtin rigid body provider), **audio** (the miniaudio provider, vendored single header), and **hot_reload** (the development-loop filesystem watcher). Each module also publishes its capability vocabulary from its own directory (`logging_capability.hpp`, `window_capability.hpp`, ...) — the single source of truth every consumer includes; the CMake `engine_capability_<name>` INTERFACE targets carry those include paths without linking any module implementation
- `host/` — composition root and frame loop; the CLI (`--validate`, `--scene`, `--set`, `--smoke-frames`, `--leak-check`, `--latency-report`)
- `mods/<id>/` — mod packages, discovered at runtime from `mod.toml`; `mods/README.md` is the authoring guide. The tree ships exactly one: `test_scene/`, the content package that owns the demo world and its texture assets
- `cmake/` — dependency acquisition, shader compilation, availability header
- `config/` — `engine.toml` (deliberately without `[scene]`: world content belongs to the content mod, or to an assembly/`--scene` that defines its own)

The engine ships no content assets: the engine tree is code only (kernel, modules, host, build scripts), and content lives in mod packages (`mods/test_scene/assets/`). Shaders are not assets either — they are the build output of the modules that implement the techniques, landing in `build/runtime/shaders/`.

`third_party/` is not part of the tree: create it only when vendoring dependency sources (see `docs/building.md`).
