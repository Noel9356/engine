# Writing mods

A mod is a package: a directory under `mods/` containing a `mod.toml` manifest plus its entry point (a shared library or a Lua script). The manifest is authoritative for identity and wiring; the kernel validates it before touching any code, and `--validate` checks every package without executing anything. Mods are discovered on disk, never listed in `engine.toml`. TOML rather than JSON keeps one configuration language and one parser dependency; the schema mirrors what a `mod.json` convention would carry.

Mods are the engine's content carriers. The engine tree itself ships no content assets — kernel, modules and host are code only — so everything a run renders that is not configuration comes from a package: the world (`scene = ...`), the texture sets it references (`assets/`), and the defaults that point the renderer at them (`config.toml`). The tree ships exactly one package, `test_scene/`, the demo world; this guide is the authoring manual for adding more.

## Discovery

Every subdirectory of `[engine].mod_directory` (default `mods`) containing a `mod.toml` is a package; directories without one are ignored. Ids in `[engine].disabled_mods` are skipped (installed but inactive). Every failure is diagnosed precisely:

| Condition | Diagnostic names |
|---|---|
| duplicate `id` | both package directories |
| `engine_api` mismatch | manifest value and engine's expected version |
| unknown `type`, missing keys, malformed TOML | offending field and location |
| script mod whose `host` module is not in the build | both names |
| native package without its built library | expected file names per platform |
| compiled mod manifest name disagrees with `mod.toml` | both names |
| two or more active packages declare a `scene` and the run names no world | every claiming mod id and scene file |
| the applied `scene` file missing from the package | mod id and resolved path |

Packages may coexist freely; the only failure condition involving several packages at once is world ambiguity (next section).

## The mod.toml schema

| Field | Required | Meaning |
|---|---|---|
| `id` | yes | unique component name, `[a-zA-Z0-9_]` |
| `version` | no | informational, defaults to `0.0.0` |
| `engine_api` | yes | must equal the engine's `engineModInterfaceVersion` |
| `type` | yes | `"native"` or `"script"` |
| `entry` | yes | native: library base name (`lib<entry>.so` / `<entry>.dll`); script: file relative to the package |
| `host` | script mods | hosting module name, default `"lua_scripting"` |
| `display_name` | no | defaults to `id` |
| `description`, `author` | no | free text |
| `capabilities` | no | required capabilities; these become dependency graph edges, and each also grants its `capability.<name>` permission automatically |
| `permissions` | no | extra permission grants beyond the derived capability ones (`filesystem.write`, or a lazily acquired capability), deniable in configuration |
| `requires` | no | component names to order after, without consuming a capability; any mod type may declare it (script mods additionally order after their host) |
| `config_keys` | no | configuration keys the mod reads, `"<section>.<dotted path>"` each; feeds the known-key registry so `--validate` recognizes them instead of warning about unknown keys |
| `scene` | no | package-relative path to a `[scene]`-only TOML file the mod ships as world content; see the next section |

`capabilities` pulls in a shared provider instance and fails if none exists; `requires` only orders construction. A mod that lists a capability needs no `permissions` line for it — requiring a capability and being authorized to acquire it are the same statement, so the kernel derives the grant; `permissions` remains for grants that are not capability acquisitions, and a `[permissions]` denial in the configuration wins over every grant, derived or declared.

## World content (scene files)

A package may ship the world: `scene = "scene.toml"` in `mod.toml` declares a file that may define only `[scene]` (the same rule a `--scene` file follows), resolved against the package directory. The file is applied WHOLESALE — the same store operation `--scene` uses — so a partial configuration can never merge with a mod scene.

The world source precedence, highest first:

1. `--scene <path>` — the composition root's explicit world, overriding everything.
2. the assembly's own `[scene]` section (`config/engine.toml` or `--config`) — an assembly either owns the world or delegates it; when it owns it, the installed content mod's scene is not applied and `--validate` says so in a warning.
3. the single active content mod's declared scene file, when exactly one package claims the world.
4. the components' built-in defaults (an empty sky).

Content mods may coexist freely — several packages can ship worlds side by side. The one-world rule is enforced exactly where the world would be ambiguous: with no explicit world source and two or more active claims, startup fails with a report naming every claim and how to resolve it (`--scene`, a `[scene]` section, or disabling all but one) — never a silent pick of whichever the filesystem listed first. With an explicit source the claims are simply not applied (`--validate` prints a warning per unapplied claim; an explicit `--scene` stays quiet — it is deliberate intent). A disabled package's claim is not counted, and the applied scene file missing from its package is a precise startup failure naming the path.

The dedicated channel is also enforced against the other direction: a shipped defaults file (`config.toml`) may NOT define `[scene]` — a defaults file drips its keys into every configuration that does not define the same keys, which would silently change other configurations' worlds (a sealed-room test config whose zero-sky guarantee depends on an untouched scene). The conventional file name is `scene.toml`, and the build's packaging rule copies exactly that name into the runtime package.

The shipped example is `test_scene/`: its `scene.toml` defines the demo room, camera, sun and lights, and its texture sets live in the package's `assets/textures/`.

## Shipped defaults (config.toml)

A package may carry a `config.toml` next to its `mod.toml`: the kernel merges it as the LOWEST configuration layer when the package is active. Defaults FILL — a key the assembly (`config/engine.toml`), a `--scene` file or a `--set` flag provides is never touched — so a mod can ship sensible defaults for the sections it reads without ever overriding the assembly owner's choices. The full precedence rules live in `docs/configuration.md` ("The layer stack"); `--provenance` shows, per value, which layer won.

The rules that keep the mechanism honest:

- A defaults file may only define sections, and never the assembly- or host-owned ones (`[engine]`, `[providers]`, `[permissions]`, `[diagnostics]`, `[leak_test]`) — a package that could grow the module list or deny permissions would break the authority model. `[scene]` is rejected for the same authority reason (see the previous section). Violations fail startup with a precise error.
- Keys the mod reads from the assembly's side should be declared as `config_keys` in `mod.toml`: the declaration tells `--validate` that the key is real, so an assembly override is recognized instead of warned about as unknown.
- A broken defaults file (parse error, forbidden section) drops the package with a precise diagnostic, exactly like a broken `mod.toml` does.

The in-tree example is the content package's own default: `test_scene/config.toml` ships `renderer.texture_directory = "mods/test_scene/assets/textures"`, so the renderer finds the package's texture sets in every configuration that does not set the key itself, and the assembly or `--set` overrides it when wanted. Script mods read configuration through their host module today; the Lua bridge does not expose configuration reads yet, so a script mod's `config.toml` is most useful for defaults aimed at sections the engine's modules read.

## Native (C++) mod

Implement `engine::ModInterface`, export the C ABI entry table, and build as a `MODULE` library that outputs into the package directory:

```cpp
class ExampleMod final : public engine::ModInterface
{
public:
    const engine::ComponentManifest& manifest() const override; // name must match mod.toml id
    void construct(engine::ComponentContext& context) override; // acquire, subscribe
    void start() override;
    void stop() override;
    void* capabilityPointerFor(const std::string&) override { return nullptr; }
};

extern "C" const EngineModEntryTable* engine_query_mod_entry_table()
{
    static const EngineModEntryTable table{ engine::engineModInterfaceVersion, &create, &destroy };
    return &table;
}
```

The kernel resolves the entry symbol as a function and calls it (the symbol is never read as data), unloads the library strictly after destroying the mod instance through the mod's own `destroyMod`, and cross-checks the compiled manifest's name against `mod.toml` — capabilities and permissions come from the file, so the binary cannot quietly claim more than the package declares. Build with the same compiler family and standard library as the engine: the entry table is C ABI, but the lifecycle interfaces use C++ virtual tables. The complete working reference is the test fixture `../tests/native_mod_fixture/fixture_mod.cpp`, which the unit suite builds as a real shared library and loads through the kernel's full discovery path.

The package must contain the library under the name the loader derives from `entry`: `lib<entry>.so` (or `<entry>.so`) on Linux, `lib<entry>.dylib` on macOS, and `<entry>.dll` (or the MinGW-convention `lib<entry>.dll`) on Windows. MinGW prefixes module DLLs with `lib` by default, so set `PREFIX ""` on Windows to get the canonical name; the loader accepts the prefixed spelling as a fallback so mods built with toolchain defaults still load. Out-of-tree mod builds should also define `WIN32_LEAN_AND_MEAN` and `NOMINMAX` on Windows, as the engine's own build does for the whole tree.

## Script (Lua) mod

A manifest plus a Lua 5.4 file, handed to its host module through the kernel's `ScriptModHost` contract. Each script runs in an isolated state with a permission-checked `engine` table:

```lua
engine.log_information("loaded")
engine.subscribe_key_pressed(function(key_name)
    if key_name == "F11" then engine.request_screenshot() end
end)
local width, height = engine.window_size()
```

| Function | Effect |
|---|---|
| `engine.log_trace/debug/information/warning/error(msg)` | write through the logging capability |
| `engine.subscribe_key_pressed(fn)` / `subscribe_key_released(fn)` | call `fn(key_name)` on key events |
| `engine.subscribe_frame_begin(fn)` | call `fn(frame_index)` once per rendered frame, before the frame is recorded - the hook for per-frame script logic (capture probes, animated light values through a future bridge) |
| `engine.window_size()` | returns `width, height` |
| `engine.request_screenshot()` | queues a screenshot |

Script permissions are validated at startup (denied fails the run); per-call re-checking inside the bridge is a roadmap item. The shipped `test_scene/` is the minimal live example (load + log in every run, including the headless smoke); the QA packages under `../scripts/qa/` exercise the frame and capture paths end to end.

## Mod-to-mod dependencies

Mods are ordinary components: declaring another component's capability in `capabilities` resolves both in dependency order with the shared-instance rule, and cycles are rejected with the full participant chain.

## Portability

Script mods are implementation-, platform-, and vendor-agnostic by construction. The `engine` bridge is the entire script ABI, and every function speaks capability-level concepts — log lines, key names, window pixel sizes, capture requests. No script can name Vulkan, OpenGL, GLFW, Win32, or X11, because the bridge never exposes them. A script author can rely on the following:

- The renderer module can be made from Vulkan, OpenGL, or any other graphics API — scripts neither know nor care; they observe frames only through neutral contracts and events.
- Switching operating system or windowing backend changes nothing in a script package.
- GPU vendor differences are absorbed by the graphics implementation behind the contract; cross-vendor behavior is the module's problem, never the mod's.

The rule is structural, not aspirational: a configure-time guard in `kernel/CMakeLists.txt` fails the configure if any capability contract header includes a graphics, windowing, or platform header (Vulkan, OpenGL, GLFW, X11, Win32). The surface that scripts and mods compile against therefore cannot silently grow implementation vocabulary, and the renderer and window modules keep their implementation details behind opaque handles (`requiredInstanceExtensions`, `createPresentationSurface`, `nativeWindowHandle`).

Native (C++) mods are not held to the same bar. They are C++ compiled against engine headers and may reach into module implementations or third-party APIs directly — at the cost of giving up the guarantees above. Keeping a native mod on the capability contracts only makes it as portable as a script; that is a recommendation, not an enforcement.

## Lighting from mods

C++ mods acquire the `LightingCapability` (capability `lighting`, provided by the pbr module — see `../modules/pbr/README.md`) to read and mutate the scene light state at runtime: `setSunLight`, `addPointLight`, `removePointLight`, `clearPointLights`. The state is consumed once per frame by the lighting passes; `[scene.sun]` and `[[scene.point_lights]]` in the configuration only seed it (section "[scene]" in `../docs/configuration.md`). Light descriptions are plain primitive values, so the contract stays implementation-, platform- and vendor-agnostic like every other contract. Live point lights are capped at 16; `addPointLight` returns the invalid handle when the cap is reached, so a mod can react instead of silently overwriting an existing light. Script mods interact with lighting through configuration today; the Lua bridge is the entire script ABI and grows deliberately, so it does not expose this capability yet.

## Input from mods

The `InputCapability` (capability `input`, provided by the input module — see `../modules/input/README.md`) exposes keyboard state (`isKeyDown`), mouse buttons (`isMouseButtonDown`), and continuous devices as consumed deltas: `consumeMouseDelta` / `consumeScrollDelta` drain the motion accumulated since the last call, exactly once per frame per consumer. `setCursorCaptured(true)` locks and hides the cursor for mouse-look schemes (raw motion where the platform supports it); the renderer's free-fly camera is the reference consumer (`../modules/renderer/camera_controller.hpp` — header only, driven entirely through this contract, unit tested with a stub input). Key presses and releases are also published as `KeyPressedEvent` / `KeyReleasedEvent` on the event bus for event-driven mods; the screenshot module's F12 trigger subscribes that way. A mod that acquires the capability declares `input` under `capabilities` in `mod.toml` — that single entry provides the graph edge and the acquisition grant; the explicit spellings (`requires` for the edge, `capability.input` under `permissions` for the grant) still work.
