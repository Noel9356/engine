# Architecture

The kernel owns three concerns: component lifecycle, the event bus and the permission system. Everything else is a module behind a capability it publishes, and mods consume exactly the same capabilities. No kernel file knows GLFW, Vulkan, or windows exist — and no kernel file knows any module by name or any capability by meaning: modules are anonymous capability providers that describe themselves through static plugin descriptors, mods are anonymous consumers, and the kernel only mediates between them. Nothing is registered anywhere — neither modules nor capabilities; a module exists in a run exactly when an active mod requires a capability it provides. This document is the system map and the cross-cutting models; every module's internals live next to its code in `modules/<name>/README.md`.

## Capability surfaces

A capability is a pure virtual interface plus a stable name, e.g. `LoggerCapability` with `capabilityName == "logging"`. The capability's vocabulary lives in a header published by the module that PROVIDES it — `modules/logging_stdout/logging_capability.hpp`, `modules/window_glfw/window_capability.hpp`, and so on. There is no central contract directory: the provider owns its types, consumers include the provider's header directly, and the header is the single source of truth on both sides. Adding a capability means touching the module that provides it, never a shared tree. In CMake each module declares an `engine_capability_<name>` INTERFACE target that carries its include path and nothing else: a consumer links that target to compile against the capability without linking the provider's implementation binary, so the include graph in the build files IS the capability graph. A consumer compiles against a capability even when the provider that implements it is excluded from the build — the demand then fails at startup with a precise diagnostic instead of a compile error.

### Families and implementations

Capability names come in two shapes. A **family name** (`renderer`) is implementation-agnostic demand: "I need frame rendering, whichever implementation is installed". A **family member** (`renderer.vulkan`) is one implementation of that family, named after it. A requirement matches exactly first; a family requirement with no exact provider is satisfied by the single active member of that family, and two active members are an ambiguity that only a `[providers]` selection resolves. A requirement that names a member (`renderer.vulkan`) matches that implementation exactly — that is how a mod declares it NEEDS one specific backend. The shipped tree uses both shapes: `window.glfw`, `input.glfw`, `renderer.vulkan` and `scripting.lua` are the implementation-bound members their modules provide, while mods require the families — which is why the demo mod never names GLFW or Vulkan even though both run underneath.

This is also the answer to "does the engine expose platform-specific capabilities?": yes — as family members. The portable surface every consumer is encouraged to use is the family; the implementation-bound member is visible, nameable, and picked either by installing only one implementation or by a `[providers]` selection. Script mods never see any of it: their surface is the Lua bridge (`../mods/README.md`, section "Portability").

### Names are opaque; acquisition is a pull

Capability names are opaque to the kernel: it matches requirement strings against provider declarations and never interprets what a capability means. The semantics live entirely in the provider's published header and its consumers. The one exception the kernel could have had — resolving a script mod's host — is not one: the package declares `host_capability` (one of its own requirements), and the kernel resolves that provider like any other requirement.

Acquisition stores nothing. `ComponentContext::acquireCapability` checks the permission (`capability.<name>`, derived from the requirement that pulled the component in) and then PULLS: the kernel walks the constructed components, finds the provider of the exact name — or of the family member that satisfies it — and calls that provider's `capabilityPointerFor` hook at the moment of the call, every time. There is no capability registry, no stored implementation table, and no construction-time population step; the construct phase only VERIFIES that every declared capability returns an implementation. Consumers cast the returned pointer to the interface they compiled against (the provider's published header); the kernel stays type-blind end to end. When several components consume one capability they pull the same provider object, so sharing costs nothing — there is simply nothing to duplicate.

Published capability headers are held to a portability rule, enforced mechanically at configure time: each module's `CMakeLists.txt` runs `engine_guard_capability_header` (from `cmake/capability_header_guard.cmake`) on the headers it publishes, and a header that starts including a graphics, windowing, or platform header (Vulkan, OpenGL, GLFW, X11, Win32) fails the configure. Module implementations keep their specifics behind opaque handles — the window capability deals in `void*` presentation surfaces and extension-name strings, so a renderer made from Vulkan and one made from OpenGL are indistinguishable behind the capability. Script mods see nothing else at all (`../mods/README.md`, section "Portability").

## Capability demand: how modules get activated

There is no module list and no registration anywhere — not in the app, not in the kernel, not in a configuration file. Activation is demand-driven, computed once at startup from static declarations:

1. **Module plugins describe themselves.** Every plugin binary in the modules directory exports one descriptor (name, factory, static provided/required capability lists). The kernel reads descriptors without creating instances.
2. **Mods are the sole demand roots.** Every active mod package becomes a component; its `capabilities` list states what it consumes, and a script mod's `host_capability` names which requirement its host must satisfy.
3. **The closure runs transitively.** Each demanded capability activates the plugin(s) that provide it — exact name matches first, then the members of the demanded family (honoring a `[providers]` selection when one applies) — and an activated plugin's own requirements demand further providers — the renderer brings `window` and `logging` with it, for example. A family demand satisfied through a member also marks the member as demanded, so a provider consulting the closure about its own capability sees the demand that activated it.
4. **Only then are instances created.** Plugins nobody required are never instantiated and their binaries are released before any construct phase; an instance manifest must agree with its static descriptor (drift is a precise startup error), and the construct phase verifies each provided capability has an implementation behind it — nothing is stored, because acquisition pulls from the provider at call time.

A requirement no available component satisfies fails startup with one precise diagnostic naming the consumer, the capability and the available alternatives. A run with no active mods loads zero modules: the engine kernel starts, publishes nothing, and exits cleanly. Startup work scales with what the run actually uses — a content mod that does not require `indirect_specular` never loads the reflection module at all.

## Components, modules, mods

Modules and mods are both *components* in one dependency graph. Each declares a `ComponentManifest`: provided capabilities, required capabilities, optional capabilities, required permissions. The resolver treats every component uniformly, so all four dependency directions work:

- module depends on module (screenshot requires renderer)
- mod depends on module (a native mod requiring logging and input)
- module depends on mod (a module may require a capability only a mod provides)
- mod depends on mod (two mods sharing one capability)

**Optional capabilities** (`optionalCapabilities`, `optional_capabilities` in `mod.toml`) state a *use-if-present* dependency: the component functions without the capability. They resolve through the same machinery as requirements — one provider, or the configuration's `[providers]` choice among several, with an ambiguous optional capability being the same startup error (a silent pick would depend on scan order) — and an existing provider is ordered before the consumer exactly like a requirement's. The differences: zero providers is a valid outcome (no edge, no diagnostic), the capability never joins the demand closure (an optional use never pulls its provider into a run that has no required use for it), and the derived `capability.<name>` permission still covers the acquisition. The consumer probes availability at construct time with `ComponentContext::isCapabilityAvailable` and degrades gracefully when the answer is no. The shipped worked example: the lighting technique modules list `lighting` as optional and render emissive + sky transport when no light provider is active.

A mod is therefore not a second-class plugin mechanism: it gets the same construction order, permission enforcement, and diagnostics as a compiled-in module. Packages and the `mod.toml` schema: `../mods/README.md`.

## Event bus

`EventChannel<Event>` stores handlers as cached function-pointer + user-data pairs in one contiguous vector; publishing is a linear scan with one indirect call per handler. No `std::function`, no queue, no allocation once the handler set is stable. Components cache a `channel<Event>()` reference once and publish with zero lookups.

The bus is single threaded by contract. Unsubscribing during dispatch is deferred to the end of the scan; `EventSubscription` RAII guards guarantee a stopped component never receives events through a dangling handler. A future delivery module can add queues without touching the channel API.

## Communication latency model

The engine prioritises communication latency over decoupling, by measured design:

- **Module-to-module, mod-to-module and mod-to-mod calls.** Capability calls are direct virtual calls on a pointer acquired from the provider (one vtable indirection, no lookup, no locking). The event bus is the only fan-out medium and dispatches synchronously on the caller's stack. Both paths are the minimum achievable for in-process, same-thread communication.
- **Measured, not assumed - and opt-in.** Recording is off by default (a normal run publishes thousands of events per second and its user does not want a data dump on shutdown; disabled, the bus never reads a clock, so measurement costs nothing). With `[diagnostics] latency_report = true`, every `publish()` records its dispatch duration into a per-channel fixed-size ring (no heap growth) and the app prints a per-channel mean/p50/p95/p99/max report plus the frame-time percentiles at shutdown.
- **Evaluated and rejected alternatives.** Message queues add queueing latency and memory for decoupling no current feature needs; a threaded or async bus adds synchronization on every publish and re-opens the data-race surface the single-thread contract closed; an ECS would change data layout, not communication latency. The architecture stays.

## Lifecycle

After resolving the graph, the kernel runs `construct` (acquire capabilities, subscribe to events — no backends), `start` (initialize backends, dependency order), and `stop` (reverse order). `validate()` is a pure dry run: configuration load, manifest build (including every `mod.toml` on disk), graph resolution, permission check — no backend is touched, which makes it CI-suitable.

Destructors are defensive: `stop()` no-ops when `construct` never ran, and the kernel destructor stops everything exactly once.

## Permissions

A component may only acquire capabilities it declared (`capability.<name>`) and may only write files with `filesystem.write`. The configuration can deny individual permissions per component; denial fails startup with a precise diagnostic. The app composition root is trusted; mods are subject to the same declared-grant model as modules. An acquisition without a grant is **blocked**: the engine refuses the access with a `ComponentAccessError` naming the component, the capability and the missing permission - during startup that fails the construct phase with the diagnostic, and at runtime it surfaces through the frame loop's error handling instead of ever handing out the capability.

### Mod risk advisories

The permission system enforces what it can; what it cannot enforce structurally, it makes visible. At discovery the kernel prints one warning per mod whose request is structurally sensitive - the signals are generic, never capability semantics: the mod runs native machine code, it requests permissions outside the `capability.*` namespace (direct environment access such as `filesystem.write`), or it acquires a broad set of capabilities. The advisories print on every run, so a user installs third-party mods knowing exactly what they asked for.

## Extending

- New capability: publish its header from the module that provides it (`modules/<name>/<name>_capability.hpp`), implement it, and let consumers include that header. If several implementations of one family will exist, provide the implementation-bound member name in the plugin descriptor and let consumers demand the family.
- New module: create `modules/<name>` with its own `README.md`, its `CMakeLists.txt` (plus `shaders/` if it owns GLSL), and declare the module's name, factory and static capability lists in its `module_plugin.cpp` descriptor. That is the whole change: the build discovers every module directory (the module's CMakeLists brings its own `ENGINE_BUILD_MODULE_<NAME>` build switch), and the kernel discovers the built plugin at runtime by scanning the modules directory and reading the descriptor. No shared file lists modules, and no registration exists anywhere: mods activate the module by requiring its capabilities. The descriptor's module name must equal the name the module's manifest reports - the kernel verifies the agreement at startup.
- New backend for an existing family: implement the same published interface, provide the family member name in the descriptor, ship both providers; `[providers]` picks one at runtime (installing only one implementation needs no selection at all).
- New mod: create `mods/<id>/` with a `mod.toml`; see `../mods/README.md`. A native mod adds its own self-contained `CMakeLists.txt` to the package directory - the build discovers it the same way module directories are discovered, and the kernel loads the built library from the package through the same plugin contract a module uses.

### Documentation structure

Documentation lives next to the code it describes, in exactly two shapes: **one `README.md` per component** (each module and the content package) plus the mod authoring guide (`mods/README.md`), and **`docs/` for the cross-cutting guides** (architecture, building, configuration). A `README.md` documents identity, wiring, configuration keys with their shipped defaults and files; it stays a single-file guide. Content that serves several components at once never goes into a component README; it belongs in `docs/` where the other components can cite it.

### Configuration format: TOML, deliberately

Every configuration surface - `config/engine.toml`, scene files, mod manifests and mod `config.toml` - is TOML, and it stays TOML. The reasons are structural, not taste: comments carry load-bearing reasoning in the shipped configs (every demo knob documents why its value is what it is - JSON has no comment syntax); the dotted-path addressing (the in-kernel `stringAtPath` readers) maps one-to-one onto TOML tables; array-of-tables rows (`[[scene.objects]]`) are authored wholesale and replaced wholesale, which the merge semantics are built on; and the kernel carries exactly one parser (toml++) whose type errors report file and line. JSON would forfeit the comments, fork the configuration languages between configs and manifests, and buy nothing the engine needs.

### Mod system policy: the restriction set is the correctness minimum

Mods face as few restrictions as possible, and every restriction that exists must earn its place by protecting a correctness property - never by policy taste. The rules that survive that bar: the ABI version match (a mismatched binary would crash, not complain), unique ids and their restricted charset (ids become component names and configuration sections), package-contained scene paths (one package cannot claim content outside itself), the defaults-file ban on authority sections (`[engine]`, `[providers]`, `[permissions]`, `[diagnostics]`, `[scene]` - a package that could deny permissions or drip world content would break the authority model), and world-ambiguity failure (a world is never picked silently by filesystem order). Everything else is deliberately open: packages coexist freely, several may ship worlds side by side as long as the run names which one it wants, script mods may order after any component through `requires`, one `capabilities` entry provides both the graph edge and the acquisition grant, and unknown manifest keys are ignored (forward compatibility, not an error). The script bridge is small by design, and that is not a restriction on mods: native mods are the unrestricted path - C++ against engine headers, no bridge vocabulary, no sandbox - so the engine always offers full power next to the portable path.

### Asset ownership: the engine ships no content

The engine tree (kernel, modules, app, build scripts) is code only. Content lives in mod packages, and the split is enforced structurally, not by convention alone:

- **Content assets belong to mods.** Texture sets, world definitions and the defaults that point the renderer at them live in `mods/<id>/` (`assets/`, `scene.toml`, `config.toml`). The build copies each package's content into the runtime layout next to its entry; there is no engine-owned asset directory at all. A build without packages is a working engine that renders whatever the configuration defines - an empty sky by default.
- **Shaders are not assets.** A shader is the implementation of a rendering technique: it is written next to the module that owns the algorithm, compiled by CMake at build time, and fails the build loudly when broken. The runtime layout reflects this: SPIR-V lands in `runtime/shaders/` (build output, like the executable), not under any `assets` name.
- **The world has exactly one source.** A run's `[scene]` comes from an explicitly staged scene file, from the assembly, or from the single active content mod's declared scene file - never merged across sources (see `docs/configuration.md`, "The layer stack"), and never from a defaults file, which cannot define `[scene]` at all. Content mods coexist freely; the one-world rule fails startup only when no explicit source exists and two or more active packages claim the world - ambiguity is reported, never resolved silently.

The shipped `test_scene` package is the worked example: it owns the demo world, its texture sets, and the `renderer.texture_directory` default, and nothing in `modules/` or `kernel/` names it.