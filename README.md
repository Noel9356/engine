# engine

A modular C++20 engine kernel. The kernel owns four things only: component lifecycle, the event bus, the capability registry and the permission system. Every feature — windows, rendering, input, screenshots, scripting — lives in an interchangeable module behind a capability contract, and mods (Lua scripts or C++ shared libraries) talk to modules and to each other through exactly the same contracts.

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

Dependencies are never fetched when a usable copy already exists: existing target, system package, vendored copy, populated build tree, then pinned download. Every configure prints an origin report showing what came from where.

## Documentation

Documentation lives NEXT TO THE CODE IT DESCRIBES: every module and the content package carry their own `README.md`, and `docs/` keeps only the cross-cutting guides.

| Document | Contents |
|---|---|
| `docs/architecture.md` | system map, contracts, capability demand, components and mods, event bus, permissions, lifecycle, conventions |
| `docs/building.md` | configure/build, dependency tiers, `_deps` layout, offline controls, module options |
| `docs/configuration.md` | global and shared `engine.toml` sections ([engine], [providers], [permissions], [scene], [diagnostics]); per-module keys are documented by their owning module |
| `mods/README.md` | mod packages: the `mod.toml` schema, capability demand, world content, script and native authoring, portability rules |
| `modules/<name>/README.md` | one guide per module (nine total): capability wiring, its configuration keys with shipped defaults, implementation notes |
| `mods/test_scene/README.md` | the content package: what it ships and how the world reaches the renderer |

## Layout

- `kernel/` — lifecycle, event bus, capability registry, permission system, dependency resolution, configuration, mod discovery; it links no capability contract and interprets no capability name
- `contracts/` — the capability contracts (`engine/capabilities/*.hpp`), the shared vocabulary of modules and mods; a separate target the kernel does not link
- `modules/<name>/` — nine modules, each with its own `README.md`: logging, window, input, renderer (with its own `shaders/` — module code, compiled at build time), screenshot, lua_scripting, plus the lighting technique modules pbr, radiance_cascades and ray_reflections (named after their algorithms)
- `host/` — composition root and frame loop; the CLI (`--validate`, `--scene`, `--set`, `--smoke-frames`, `--leak-check`, `--latency-report`)
- `mods/<id>/` — mod packages, discovered at runtime from `mod.toml`; `mods/README.md` is the authoring guide. The tree ships exactly one: `test_scene/`, the content package that owns the demo world and its texture assets
- `cmake/` — dependency acquisition, shader compilation, availability header
- `config/` — `engine.toml` (deliberately without `[scene]`: world content belongs to the content mod, or to an assembly/`--scene` that defines its own)

The engine ships no content assets: the engine tree is code only (kernel, contracts, modules, host, build scripts), and content lives in mod packages (`mods/test_scene/assets/`). Shaders are not assets either — they are the build output of the modules that implement the techniques, landing in `build/runtime/shaders/`.

`third_party/` is not part of the tree: create it only when vendoring dependency sources (see `docs/building.md`).
