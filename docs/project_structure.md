# Project structure

## Runtime layers

- `kernel/` owns engine startup/shutdown, configuration, module discovery and loading, plugin discovery, platform library loading, and trust checks. Its public API is under `kernel/include/engine/kernel/`; implementation details are grouped under `kernel/source/`.
- `application/` builds the application host, command-line processing, frame timing, and frame pacing on top of the kernel.
- `shared_types/` contains data contracts shared across capability boundaries.
- `modules/` contains independently selectable engine capabilities. Each module owns its public headers, private implementation, manifest, configuration defaults, and module documentation.
- `plugins/` contains plugin packages discovered by the runtime. Package directories remain direct children of `plugins/`.
- `demos/` contains standalone applications that exercise combinations of modules.

## Implementation organization

The kernel groups implementation by responsibility: `engine_kernel/` contains lifecycle and module-loading orchestration; `configuration/` handles settings; `module_system/` handles module descriptors and capabilities; `plugins/` handles plugin packages; `platform/` provides OS services; `security/` contains hashing and trust checks; and `internal/` holds private cross-translation-unit headers.

The renderer keeps one module target but separates `source/core/` from backend implementations under `source/backend/`. Backend-private headers remain within their implementation area; the module target adds private include directories for shared internal headers. The models and world modules similarly group implementation by asset pipeline and world/ECS responsibilities.

## Public versus private interfaces

Public module APIs stay under each module's `include/engine/<module>/` directory. Private implementation headers belong under `source/` and are not part of the supported external API. Directory moves should not change public include paths, capability names, module manifests, or runtime package discovery conventions.

## Build discovery

The root CMake project discovers modules through `modules/*/CMakeLists.txt` and plugins through direct children of `plugins/`. Those conventions are preserved. Demos are explicitly registered from `demos/<name>/` when `ENGINE_BUILD_DEMO` is enabled.

## Demo source layout

- `demos/character_customisation/source/demo/benchmark_scene.*` owns benchmark-only textures, imported meshes, static batches, and their teardown. `character_stage.*` owns the static floor/sky/sun resources; `character_screen/` separates screen lifecycle, assembly editing, frame/input handling, rendering, panels, and automated self-tests. `character_content.cpp` remains a single authored-part/rig catalog because its definitions are tightly coupled data.
- `demos/lumen_gi/source/demo/` contains command-line option parsing and deterministic scene construction; the entry point owns engine startup, capability wiring, and the render loop.
- `demos/planet/source/demo/` contains option parsing, camera/viewpoint selection, generation determinism tests, window-to-camera input, and the GPU chunk-mesh cache. The entry point owns application startup, mode selection, and the frame loop; the camera-input and chunk-rendering helpers own their respective responsibilities.

Large module implementation files are not split solely by line count. Backend implementations and stateful module cores should be divided only when a stable responsibility boundary exists; otherwise extra translation units make navigation and maintenance worse.


## Interactive demo controls and runtime assets

- The planet demo uses a Fly camera in interactive windowed runs: mouse look while the cursor is captured, `W/A/S/D` movement, `Space`/`Ctrl` vertical movement, `Shift` sprint, arrow-key look, and `Left Alt` to toggle cursor capture. `--orbit` retains the deterministic automated camera path and disables manual camera input.
- The character customisation demo ships its authored Wavefront part meshes under `demos/character_customisation/assets/character_parts/`; CMake stages the complete `assets/` directory beside the demo executable at runtime. Its `character_lab` script plugin is enabled by default for this demo, unless plugin selection is explicitly overridden on the command line.
