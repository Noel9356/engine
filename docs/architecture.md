# Architecture

The kernel owns four concerns: capability contracts, component lifecycle, the event bus, and the permission system. Everything else is a module behind a contract, and mods use exactly the same contracts. No kernel file knows GLFW, Vulkan, or windows exist - and no kernel file knows any module by name: modules are anonymous capability providers that describe themselves through static plugin descriptors, and the kernel only mediates between consumers and providers. Nothing is registered anywhere; a module exists in a run exactly when something requires a capability it provides. This document is the system map and the cross-cutting models; every module's internals live next to its code in `modules/<name>/README.md` (see the index below).

## Documentation index

| Document | Contents |
|---|---|
| `../kernel/README.md` | the kernel concerns file by file, contracts, lifecycle, mod loading, startup diagnostics |
| `../host/README.md` | composition root, frame loop, CLI flags, exit codes, the leak-check harness |
| `../mods/README.md` | mod packages, the `mod.toml` schema, authoring guides, portability rules |
| `../modules/logging/README.md` | the logging service and its contract |
| `../modules/window/README.md` | windowing backend, presentation surface factory, resize events |
| `../modules/input/README.md` | input aggregation, consumed deltas, capture mode, the free-fly consumer reference |
| `../modules/renderer/README.md` | the frame pipeline, G-buffer, materials and textures, camera, synchronization |
| `../modules/screenshot/README.md` | capture paths and the correctness model |
| `../modules/lua_scripting/README.md` | the script host, the `engine` bridge, script isolation |
| `../modules/pbr/README.md` | analytic direct lighting, the `LightingCapability`, SDF soft shadows |
| `../modules/radiance_cascades/README.md` | the GI technique: cascades, anchoring, gather, leak guard, cost model |
| `../modules/ray_reflections/README.md` | ray-traced glossy reflections, self-hit handling, specular partition |
| `configuration.md` | global and shared configuration sections ([engine], [providers], [permissions], [scene], [diagnostics]); per-module keys live in the module docs |
| `building.md` | configure/build/test, dependency tiers, `_deps` layout, offline controls, module options |
| `robustness.md` | problems found and fixed, how runtime regressions are caught, known limits |

## Capability contracts

A capability is a pure virtual interface plus a stable name, e.g. `LoggerCapability` with `capabilityName == "logging"`. Contract headers live in the kernel's include tree (`kernel/include/engine/capabilities/`), so both sides compile against them even when the provider is excluded from the build. Consumers ask their `ComponentContext` for an interface and never downcast to concrete module types. When several components require one capability, the registry hands every consumer the same shared instance.

Capability names are opaque to the kernel: it matches requirement strings against provider declarations and never interprets what a capability means. The semantics live entirely in the contract header and its consumers.

Contracts are the mod-facing stability surface and are held to a portability rule, enforced mechanically at configure time: `kernel/CMakeLists.txt` fails the configure if a contract header includes a graphics, windowing, or platform header (Vulkan, OpenGL, GLFW, X11, Win32). Module implementations keep their specifics behind opaque handles — the window contract deals in `void*` presentation surfaces and extension-name strings, so a renderer made from Vulkan and one made from OpenGL are indistinguishable from behind the contracts. Script mods see nothing else at all (`../mods/README.md`, section "Portability"). The six contracts and the registry mechanics are detailed in `../kernel/README.md`.

## Capability demand: how modules get activated

There is no module list and no registration anywhere - not in the host, not in the kernel, not in a configuration file. Activation is demand-driven, computed once at startup from static declarations:

1. **Module plugins describe themselves.** Every plugin binary in the modules directory exports one descriptor (name, factory, static provided/required capability lists). The kernel reads descriptors without creating instances.
2. **Mods are the demand roots.** Every active mod package becomes a component; its `capabilities` list states what it consumes (a script mod's `"scripting"` entry pulls its host in).
3. **The assembly states its own demand as data.** `[engine].required_capabilities` names the capabilities the composition root consumes directly (the frame loop needs `window` and `renderer`; the demo also demands the lighting layers and `screenshot`).
4. **The closure runs transitively.** Each demanded capability activates the plugin(s) that provide it (honoring a `[providers]` selection when one applies), and an activated plugin's own requirements demand further providers - the renderer brings `window` and `logging` with it, for example.
5. **Only then are instances created.** Plugins nobody required are never instantiated and their binaries are released before any construct phase; an instance manifest must agree with its static descriptor (drift is a precise startup error), and the registry receives each provider's implementation pointers as it constructs.

A requirement no available component satisfies fails startup with one precise diagnostic naming the consumer, the capability and the available alternatives. A run with no mods and an empty demand list loads zero modules: the engine kernel starts, publishes nothing, and exits cleanly. Startup work scales with what the run actually uses - a GI-only test run never loads the reflection module at all.

## Components, modules, mods

Modules and mods are both *components* in one dependency graph. Each declares a `ComponentManifest`: provided capabilities, required capabilities, required permissions. The resolver treats every component uniformly, so all four dependency directions work and are unit tested:

- module depends on module (screenshot requires renderer)
- mod depends on module (the C++ example mod requires logging and input)
- module depends on mod (a module may require a capability only a mod provides)
- mod depends on mod (two mods sharing one capability)

A mod is therefore not a second-class plugin mechanism: it gets the same construction order, permission enforcement, and diagnostics as a compiled-in module. Packages and the `mod.toml` schema: `../mods/README.md`.

## Event bus

`EventChannel<Event>` stores handlers as cached function-pointer + user-data pairs in one contiguous vector; publishing is a linear scan with one indirect call per handler. No `std::function`, no queue, no allocation once the handler set is stable. Components cache a `channel<Event>()` reference once and publish with zero lookups.

The bus is single threaded by contract. Unsubscribing during dispatch is deferred to the end of the scan; `EventSubscription` RAII guards guarantee a stopped component never receives events through a dangling handler. A future delivery module can add queues without touching the channel API.

## Communication latency model

The engine prioritises communication latency over decoupling, by measured design:

- **Module-to-module, mod-to-module and mod-to-mod calls.** Capability calls are direct virtual calls on a pointer acquired once (one vtable indirection, no lookup, no locking). The event bus is the only fan-out medium and dispatches synchronously on the caller's stack. Both paths are the minimum achievable for in-process, same-thread communication.
- **Measured, not assumed - and opt-in.** Recording is off by default (a normal run publishes thousands of events per second and its user does not want a data dump on shutdown; disabled, the bus never reads a clock, so measurement costs nothing). With `[diagnostics] latency_report = true` or the `--latency-report` flag, every `publish()` records its dispatch duration into a per-channel fixed-size ring (no heap growth) and the host prints a per-channel mean/p50/p95/p99/max report plus the frame-time percentiles at shutdown.
- **Evaluated and rejected alternatives.** Message queues add queueing latency and memory for decoupling no current feature needs; a threaded or async bus adds synchronization on every publish and re-opens the data-race surface the single-thread contract closed; an ECS would change data layout, not communication latency. The architecture stays.

## Lifecycle

After resolving the graph, the kernel runs `construct` (acquire capabilities, subscribe to events — no backends), `start` (initialize backends, dependency order), and `stop` (reverse order). `validate()` is a pure dry run: configuration load, manifest build (including every `mod.toml` on disk), graph resolution, permission check — no backend is touched, which makes it CI-suitable.

Destructors are defensive: `stop()` no-ops when `construct` never ran, and the kernel destructor stops everything exactly once.

## Permissions

A component may only acquire capabilities it declared (`capability.<name>`) and may only write files with `filesystem.write`. The configuration can deny individual permissions per component; denial fails startup with a precise diagnostic. The host composition root is trusted; mods are subject to the same declared-grant model as modules. An acquisition without a grant is **blocked**: the engine refuses the access with a `ComponentAccessError` naming the component, the capability and the missing permission - during startup that fails the construct phase with the diagnostic, and at runtime it surfaces through the frame loop's error handling instead of ever handing out the capability.

### Mod risk advisories

The permission system enforces what it can; what it cannot enforce structurally, it makes visible. At discovery the kernel prints one warning per mod whose request is structurally sensitive - the signals are generic, never capability semantics: the mod runs native machine code, it requests permissions outside the `capability.*` namespace (direct environment access such as `filesystem.write`), or it acquires a broad set of capabilities. The advisories print on every normal run and in `--validate`, so a user installs third-party mods knowing exactly what they asked for.

## Startup diagnostics

All checks accumulate into one report before failing. Diagnosed cases, each naming the relevant components:

- unknown capability (with the list of available ones)
- ambiguous provider (with candidates and the `[providers]` key that resolves it)
- dependency cycles (with the full participant chain)
- permission denials
- module manifest/descriptor drift (a rebuilt module disagreeing with its static declaration)
- mod interface version mismatches, duplicate mod ids, missing mod libraries
- script mods with no active `scripting` provider, Lua script load failures

`engine --validate` exits 0 and prints `engine configuration is valid` on success.

## Extending

- New capability: add a contract header under `kernel/include/engine/capabilities/`, implement it in a module, acquire it where needed.
- New module: create `modules/<name>` with its own `README.md` (plus `shaders/` if it owns GLSL), add a CMake option and subdirectory, and declare the module's name, factory and static capability lists in its `module_plugin.cpp` descriptor. No registration exists anywhere: consumers (mods or `[engine].required_capabilities`) activate it by requiring its capabilities.
- New backend: implement the same contract and ship both providers; `[providers]` picks one at runtime.
- New mod: create `mods/<id>/` with a `mod.toml`; see `../mods/README.md`.

## Conventions

Functions and variables `camelCase`, types `PascalCase`, files `snake_case`, Allman braces (every brace on its own line). Module sources drop the `_module` suffix - the directory already names the module (`modules/renderer/renderer.cpp`, `modules/pbr/pbr.cpp`). Members are declared in destruction-safe order (see `robustness.md`, problem 3); Vulkan objects inside a pass destroy in strict reverse dependency order (sets, pools, pipelines, layouts, descriptor set layouts - problem 25).

### Documentation structure

Documentation lives next to the code it describes, in exactly two shapes: **one `README.md` per component** (each module, the kernel, the host, `mods/`, the content package) and **`docs/` for the cross-cutting guides** (architecture, building, configuration, robustness). A `README.md` documents identity, wiring, configuration keys with their shipped defaults, files and tests; it stays a single-file guide (a component whose docs outgrow one file would add a `doc/` directory and shrink the README to an index - no component currently needs that, and the uniform shape keeps the tree navigable). Content that serves several components at once never goes into a component README; it belongs in `docs/` where the other components can cite it.

### Configuration format: TOML, deliberately

Every configuration surface - `config/engine.toml`, `--scene` files, mod scene files, module `module.toml`, mod manifests and mod `config.toml` - is TOML, and it stays TOML. The reasons are structural, not taste: comments carry load-bearing reasoning in the shipped configs (every demo knob documents why its value is what it is - JSON has no comment syntax); the dotted-path addressing (`--set scene.sun.intensity=5.0`, the in-kernel `stringAtPath` readers) maps one-to-one onto TOML tables; array-of-tables rows (`[[scene.objects]]`) are authored wholesale and replaced wholesale, which the merge semantics are built on; and the kernel carries exactly one parser (toml++) whose type errors report file and line. JSON would forfeit the comments, fork the configuration languages between configs and manifests, and buy nothing the engine needs.

### Mod system policy: the restriction set is the correctness minimum

Mods face as few restrictions as possible, and every restriction that exists must earn its place by protecting a correctness property - never by policy taste. The rules that survive that bar: the ABI version match (a mismatched binary would crash, not complain), unique ids and their restricted charset (ids become component names and configuration sections), package-contained scene paths (one package cannot claim content outside itself), the defaults-file ban on authority sections (`[engine]`, `[providers]`, `[permissions]`, `[diagnostics]`, `[scene]` - a package that could grow the module list or deny permissions would break the authority model), and world-ambiguity failure (a world is never picked silently by filesystem order). Everything else is deliberately open: packages coexist freely, several may ship worlds side by side as long as the run names which one it wants, script mods may order after any component through `requires`, one `capabilities` entry provides both the graph edge and the acquisition grant, and unknown manifest keys are ignored (forward compatibility, not an error). The script bridge is small by design, and that is not a restriction on mods: native mods are the unrestricted path - C++ against engine headers, no bridge vocabulary, no sandbox - so the engine always offers full power next to the portable path.

### Asset ownership: the engine ships no content

The engine tree (kernel, modules, host, tests, build scripts) is code only. Content lives in mod packages, and the split is enforced structurally, not by convention alone:

- **Content assets belong to mods.** Texture sets, world definitions and the defaults that point the renderer at them live in `mods/<id>/` (`assets/`, `scene.toml`, `config.toml`). The build copies each package's content into the runtime layout next to its entry; there is no engine-owned asset directory at all. A build without packages is a working engine that renders whatever the configuration defines - an empty sky by default.
- **Shaders are not assets.** A shader is the implementation of a rendering technique: it is written next to the module that owns the algorithm, compiled by CMake at build time, and fails the build loudly when broken. The runtime layout reflects this: SPIR-V lands in `runtime/shaders/` (build output, like the executable), not under any `assets` name.
- **The world has exactly one source.** A run's `[scene]` comes from `--scene`, from the assembly, or from the single active content mod's declared scene file - never merged across sources (see `docs/configuration.md`, "The layer stack"), and never from a defaults file, which cannot define `[scene]` at all. Content mods coexist freely; the one-world rule fails startup only when no explicit source exists and two or more active packages claim the world - ambiguity is reported, never resolved silently.

The shipped `test_scene` package is the worked example: it owns the demo world, its texture sets, and the `renderer.texture_directory` default, and nothing in `modules/` or `kernel/` names it.

### Code hygiene: no hardcoding, no dead code, no god objects

Three standing rules, applied as a full-tree audit and kept as the bar every change is held to:

- **No hardcoding.** No baked-in world content (the engine's own fallback is an empty scene - content belongs to mods, see above); no unnamed magic numbers (timeouts, budgets, binding numbers, formats and thresholds are named constants at the top of their file, each with the reason for its value); no repeated path or name literals (the default configuration path is named once and quoted by the usage text; the "mods" and "shaders" directories live in one constant each). A number or a path that appears twice without sharing a name is a defect, because the two sites can drift silently.
- **No dead code.** Unused API surface, leftover compatibility mappings and stale references are deleted, not kept "just in case". The 2026-10 audit removed the deprecated `[lighting].direct` / `[lighting].gi` fallback mapping and retired the renderer's private copies of the shared Vulkan helpers (`findMemoryType`, shader-byte reading) that `vulkan_utilities.hpp` already provided - the shared copy now carries the SPIR-V word-alignment check the private one had. The same audit pass later removed the module registration API, the `[engine].modules` list and the `[lighting].techniques` selector when capability demand replaced them (see `REFACTORING.md` at the tree root). Removing a compatibility shim is announced in the docs the same way adding one is: the capability demand lists are the only spelling of what a run loads.
- **No god objects.** A type or file that accretes several concerns is split by concern before it hardens. `RendererState` (one struct, ~3.1k lines covering device creation, scene resources, pipelines, recording and the frame loop) is now one state across five translation units split by concern - `renderer_device.cpp`, `renderer_scene.cpp`, `renderer_pipelines.cpp`, `renderer_frames.cpp`, and a slim `renderer.cpp` for the module lifecycle - with the module-private `renderer_state.hpp` as the only place they meet; the kernel's mod-package discovery (directory scan, native library loading) moved out of `engine_kernel.cpp` into `mod_discovery.cpp`, leaving the kernel's gather phase policy-only; the host's leak-check policy moved out of `main.cpp` into `leak_check.cpp`, leaving the composition root with wiring and reporting. The split is structural, not architectural churn: one state struct, zero behavior change, and every concern owns a file whose name says what it does.
