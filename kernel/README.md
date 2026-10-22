# kernel

The engine kernel owns four concerns and nothing else: capability contracts, component lifecycle, the event bus, and the permission system. Every feature — windows, rendering, input, screenshots, scripting — lives in an interchangeable module behind a capability contract, and mods (C++ shared libraries or Lua scripts) talk to modules and to each other through exactly the same contracts. No kernel file knows that GLFW, Vulkan, or a window exist.

## The four concerns, file by file

All kernel headers live under `kernel/include/engine/kernel/`; the capability contracts one level up in `kernel/include/engine/capabilities/`.

| File | Concern |
|---|---|
| `capability_registry.hpp` | registration and lookup of live capabilities: `registerCapability` rejects duplicates by name (two providers of one capability must be disambiguated through `[providers]`), `acquireCapability` returns the shared instance pointer, `providerNameFor` feeds diagnostics |
| `component_manifest.hpp` | `ComponentManifest` — the uniform identity and wiring record for modules AND mods: name, kind, version, provided capabilities, required capabilities, required permissions, and the known configuration keys (`knownConfigKeys`) that feed the unknown-key warnings of `--validate` |
| `component_interface.hpp` | the lifecycle interfaces: `ComponentInterface` (construct/start/stop), the `ComponentContext` handed to construct (permission-checked capability acquisition, event channels), `ModInterface` for native mods, `ScriptModHost` for script hosting |
| `dependency_resolver.hpp` | resolves every component into one dependency graph from manifests alone; rejects cycles with the full participant chain, reports unknown capabilities with the available ones, reports ambiguous providers with the candidates and the `[providers]` key that resolves them |
| `event_bus.hpp` | `EventBus` / `EventChannel<Event>` / `EventSubscription` — synchronous single-threaded publish/subscribe with per-channel latency statistics (see `docs/architecture.md`) |
| `permission_system.hpp` | the declared-grant model: a component may only acquire capabilities it declared (`capability.<name>`) and may only write files with `filesystem.write`; configuration can deny individual grants per component and denial fails startup precisely |
| `configuration.hpp` | `ConfigurationStore` — the layered configuration (toml++): assembly file (`config/engine.toml` or `--config`), shipped defaults (`module.toml` / mod `config.toml`, fill-gaps only), the world source (`--scene` or a content mod's declared scene file — a wholesale `[scene]` swap with one source), typed `--set` overrides, per-key provenance records and the known-key registry; typed access with per-section and dotted-path readers |
| `mod_manifest.hpp` / `mod_api.hpp` / `dynamic_library.hpp` | mod packages: `mod.toml` parsing and validation (`engine_api` handshake, entry naming per platform), the C ABI entry table contract (`engine_query_mod_entry_table`, `engineModInterfaceVersion`), and dynamic library loading/unloading |
| `engine_kernel.hpp` | orchestration: `registerModuleFactory`, graph resolution, the construct/start/stop phases, `validate()` |

## Capability contracts

The six contracts live in `kernel/include/engine/capabilities/`: `logging_contract.hpp` (`LoggerCapability`, `logging`), `window_contract.hpp` (`WindowCapability`, `window`), `input_contract.hpp` (`InputCapability`, `input`, plus `KeyPressedEvent`/`KeyReleasedEvent`), `renderer_contract.hpp` (`RendererCapability`, `renderer`), `screenshot_contract.hpp` (`ScreenshotCapability`, `screenshot`), and `lighting_contract.hpp` (`LightingCapability`, `lighting`). Consumers ask their `ComponentContext` for an interface and never downcast to concrete module types; when several components require one capability, the registry hands every consumer the same shared instance.

Contracts are the mod-facing stability surface and are held to a portability rule, enforced mechanically at configure time: `kernel/CMakeLists.txt` fails the configure if a contract header includes a graphics, windowing, or platform header (Vulkan, OpenGL, GLFW, X11, Win32) — a rule born from a real violation (problem 16 in `docs/robustness.md`). Module implementations keep their specifics behind opaque handles (`requiredInstanceExtensions`, `createPresentationSurface`, `nativeWindowHandle`), so a renderer made from Vulkan and one made from OpenGL are indistinguishable from behind the contracts. The renderer's technique attach point (`modules/renderer/render_interface.hpp`) is deliberately NOT a contract — it is Vulkan-typed, in-tree, and shared only by graphics code.

## Lifecycle

After resolving the graph, the kernel runs `construct` (acquire capabilities, subscribe to events — no backends), `start` (initialize backends, dependency order), and `stop` (reverse order). `validate()` is a pure dry run: layered configuration load, manifest build (including every `mod.toml` on disk), graph resolution, permission check — no backend is touched, which makes it CI-suitable. Unknown configuration keys are reported as warnings and never flip the result. Destructors are defensive: `stop()` no-ops when `construct` never ran, and the kernel destructor stops everything exactly once. Each component receives its own `ComponentContext` (named after its manifest id) that stays alive for the component's lifetime — an earlier shared-context design dangled after construct (problem 21 in `docs/robustness.md`).

## Mod loading

The kernel discovers mod packages on disk (every subdirectory of `[engine].mod_directory` containing a `mod.toml`), validates each manifest precisely, and loads native mods through `dynamic_library.hpp`: the entry symbol is resolved as a FUNCTION and called — never read as data (problem 2) — the library handle outlives the mod instance and unloads strictly after destruction through the mod's own `destroyMod` (problem 3), and the compiled manifest's name is cross-checked against `mod.toml` so the binary cannot quietly claim more than the package declares. Script mods are handed to their host module through `ScriptModHost`. A package may declare world content (`scene = "scene.toml"`): the kernel collects the claims during discovery and `finishConfiguration` decides — an explicit world source (`--scene` or the assembly's own `[scene]`) makes the claims irrelevant, a single claim provides the world wholesale, and two or more claims with no explicit source are a startup error naming every claim (the world would be ambiguous; packages otherwise coexist freely). Permission grants are the declared list plus one `capability.<name>` grant per required capability, with configuration denials winning over both (precedence and the defaults-file ban in `mods/README.md` and `docs/configuration.md`). The native loading path is covered end to end by the test suite's own fixture library (`tests/native_mod_fixture/`, dlopened from a temporary package), so shipping content packages does not trade away that coverage. The full package schema and authoring walkthroughs live in `mods/README.md`.

## Startup diagnostics

All checks accumulate into one report before failing — an early return once masked a denied permission behind a wrong module name (problem 6). Diagnosed cases, each naming the relevant components: unknown capability (with the available ones), ambiguous provider (with candidates and the `[providers]` key), dependency cycles (with the full participant chain), permission denials, modules requested but compiled out (the availability header makes the compile-time/runtime mismatch precise — problem 5), mod interface version mismatches, duplicate mod ids, missing mod libraries, compiled-manifest name disagreements, script mods whose host module is missing, and Lua script load failures. `engine --validate` exits 0 and prints `engine configuration is valid` on success.

## Files

- `kernel/include/engine/kernel/*.hpp` — the headers in the table above
- `kernel/include/engine/capabilities/*.hpp` — the six capability contracts
- `kernel/source/*.cpp` — the kernel implementation (configuration store, orchestration, graph resolver, manifest, permission and mod-package discovery plumbing; `mod_discovery.cpp` owns the filesystem scan and native-library loading so the kernel's gather phase stays policy-only)
- `kernel/CMakeLists.txt` — the kernel library target and the contract portability guard

## Tests

The kernel is the best-tested part of the tree: `tests/dependency_resolver_tests.cpp` (all four dependency directions, cycles, ambiguity), `tests/event_bus_tests.cpp` (dispatch, unsubscription during dispatch, latency statistics), `tests/permission_system_tests.cpp` (declared grants, denials), `tests/mod_manifest_tests.cpp` (schema, failure diagnostics), `tests/configuration_tests.cpp` (layer stack, merge semantics, `--set` typing, provenance, unknown-key warnings, kernel integration), and `tests/kernel_integration_tests.cpp` (full graphs with real components, mods included).
