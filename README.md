# engine

A modular C++20 engine kernel. The kernel owns four things only: capability contracts, component lifecycle, the event bus, and the permission system. Every feature — windows, rendering, input, screenshots, scripting — lives in an interchangeable module behind a capability contract, and mods (C++ shared libraries or Lua scripts) talk to modules and to each other through exactly the same contracts. Nothing in the kernel knows that GLFW, Vulkan, or a window exist. Script mods are implementation-, platform-, and vendor-agnostic by construction — the renderer can be made from Vulkan or OpenGL without a script being able to tell (`mods/README.md`, section "Portability").

## Quick start

Requirements: CMake 3.24+, a C++20 compiler, a Vulkan loader, and `glslangValidator`. A complete Vulkan SDK (headers incl. `vulkan.hpp` plus loader) is detected and used as-is; the pinned fallback fetches only when something is missing. See `docs/building.md`.

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build             # 67 unit test cases + headless smoke + light leak check
./build/runtime/engine --validate  # dry run: resolve, check, report, exit
./build/runtime/engine             # textured 3D PBR scene with radiance-cascades GI
                                   # and ray-traced reflections; F12 screenshot
                                   # (to ./screenshots), Esc quit
```

The world comes from the one shipped content mod: `mods/test_scene` declares `scene.toml` (the Cornell-style room, camera, sun) and carries the PBR texture sets it references. The demo camera is a free-fly camera (starts at the configured `[scene.camera]` pose): **WASD** fly, **mouse** look while the cursor is captured (**Esc** releases; **F** or left-click re-captures), **Space/E** rise, **Ctrl/Q** sink, **Shift** speed boost, **mouse wheel** fly speed. Set `[scene.camera] free_fly = false` for a fixed tripod camera.

Runs stay quiet at shutdown. Developers can opt into a communication latency report (per-channel event dispatch mean and percentiles) plus frame-time percentiles with `--latency-report` or `[diagnostics] latency_report = true`.

Dependencies are never fetched when a usable copy already exists: existing target, system package, vendored copy, populated build tree, then pinned download. Every configure prints an origin report showing what came from where.

## Documentation

Documentation lives NEXT TO THE CODE IT DESCRIBES: every module, the kernel, the host and the mods tree carry their own `README.md`, and `docs/` keeps only the cross-cutting guides.

| Document | Contents |
|---|---|
| `docs/architecture.md` | system map, contracts, components and mods, event bus, communication latency model, lifecycle, conventions |
| `docs/building.md` | configure/build/test, dependency tiers, `_deps` layout, offline controls, module options |
| `docs/configuration.md` | global and shared `engine.toml` sections ([engine], [providers], [permissions], [scene], [diagnostics]); per-module keys are documented by their owning module |
| `docs/robustness.md` | problems found and fixed, how runtime regressions are caught, known limits |
| `docs/performance_changes.md` | the performance pass: architecture verdict, the five per-frame cost fixes, verification |
| `kernel/README.md` | the kernel concerns file by file, the six contracts, lifecycle, mod loading, startup diagnostics |
| `host/README.md` | composition root, frame loop, CLI flags, exit codes, the leak-check harness |
| `mods/README.md` | mod packages, the `mod.toml` schema, world content (`scene = ...`), native and Lua authoring guides, mod portability rules |
| `modules/<name>/README.md` | one guide per module (nine total): capability wiring, its configuration keys with shipped defaults, implementation deep-dives, files, tests |
| `mods/test_scene/README.md` | the content package: what it ships and how the world reaches the renderer |

## Layout

- `kernel/` — the four kernel concerns, capability contracts, mod manifest loading; `kernel/README.md` documents it file by file
- `modules/<name>/` — nine modules, each with its own `README.md` (wiring, configuration keys, implementation notes): logging, window, input, renderer (with its own `shaders/` — module code, compiled at build time), screenshot, lua_scripting, plus the lighting technique modules pbr, radiance_cascades and ray_reflections (named after their algorithms)
- `host/` — composition root and frame loop; `host/README.md` documents the CLI and exit codes
- `mods/<id>/` — mod packages, discovered at runtime from `mod.toml`; `mods/README.md` is the authoring guide. The tree ships exactly one: `test_scene/`, the content package that owns the demo world and its texture assets
- `tests/` — Catch2 suite plus the native-mod loading fixture (a real shared library the tests dlopen through the kernel); optional (`-DENGINE_BUILD_TESTS=OFF` removes it and the Catch2 dependency)
- `scripts/` — `run_headless_smoke.sh`, the CTest smoke run over the assembled runtime, plus `qa/` (the QA capture mods and pose harness used during visual verification)
- `cmake/` — dependency acquisition, shader compilation, availability header
- `config/` — `engine.toml` (deliberately without `[scene]`: world content belongs to the content mod, or to an assembly/`--scene` that defines its own)
- `docs/` — the cross-cutting guides (architecture, building, configuration, robustness)

The engine ships no content assets: the engine tree is code only (kernel, modules, host, tests, build scripts), and content lives in mod packages (`mods/test_scene/assets/`). Shaders are not assets either — they are the build output of the modules that implement the techniques, landing in `build/runtime/shaders/`.

`third_party/` is not part of the tree: create it only when vendoring dependency sources (see `docs/building.md`).
