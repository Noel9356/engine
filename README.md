# engine

A modular C++20 engine kernel. The kernel owns three things only: component lifecycle, the event bus and the permission system. Every feature — windows, rendering, input, screenshots, scripting — lives in an interchangeable module behind a capability it publishes, and mods (Lua scripts or C++ shared libraries) consume the same capabilities.

Capabilities have names, and names come in two shapes: implementation-agnostic families (`renderer`) and implementation-bound members (`renderer.vulkan`). Each module publishes its capability vocabulary from its own directory (`window_capability.hpp` lives in `modules/window_glfw/`), so the provider and its types are one unit: consumers include the provider's header directly, and there is no central contract directory that every new capability has to touch. Nothing is registered anywhere — not modules, not capabilities: acquisition is a pull that asks the declaring provider for its implementation at the moment of the call.

The kernel is a pure mediator. It never names a module and never interprets a capability: modules are anonymous capability providers that describe themselves through static plugin descriptors, mods are anonymous consumers that name only the capabilities they require, and a module is instantiated exactly when an active mod's manifest requires a capability it provides — transitively, through the providers' own requirements. Nothing registers a module anywhere; a run with no mods loads no modules at all. The kernel also enforces permissions: a component that touches a capability it did not declare is blocked, and every mod is risk-assessed for the user before its code runs.

## Quick start

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
./build/runtime/engine
```

## Documentation

Documentation lives NEXT TO THE CODE IT DESCRIBES: every module and the content package carry their own `README.md`, and `docs/` keeps only the cross-cutting guides.

| Document | Contents |
|---|---|
| `docs/architecture.md` | system map, capability surfaces and families, capability demand, pure-pull acquisition, components and mods, event bus, permissions, lifecycle, conventions |
| `docs/building.md` | configure/build, dependency tiers, `_deps` layout, offline controls, module options |
| `docs/configuration.md` | global and shared `engine.toml` sections ([engine], [providers], [permissions], [scene], [diagnostics]); per-module keys are documented by their owning module |
| `mods/README.md` | mod packages: the `mod.toml` schema, capability demand, world content, script and native authoring, portability rules |
| `modules/<name>/README.md` | one guide per module (twenty total): capability wiring, its configuration keys with shipped defaults, implementation notes |