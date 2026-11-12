# Engine

A modular C++20 real-time engine with a shared kernel, application layer, dynamically loaded modules, package-style plugins, and standalone demonstration applications.

## Repository map

- `kernel/` — engine lifecycle, module discovery/loading, configuration, platform services, and trust utilities.
- `application/` — application host, command-line processing, frame clock, and frame pacing.
- `shared_types/` — stable shared data types used across module boundaries.
- `modules/` — independently built engine capabilities; each module owns its public headers, implementation, configuration, manifest, and documentation.
- `plugins/` — plugin packages discovered by the runtime.
- `demos/` — standalone sample applications that exercise the engine and its modules.
- `config/` — engine-wide defaults and runtime profiles.
- `cmake/` — reusable CMake functions for modules, applications, shaders, and dependencies.
- `docs/` — repository-wide navigation and architecture notes.

## Build

Configure and build with CMake using the toolchain and dependencies appropriate for your platform. The root `CMakeLists.txt` controls module selection and the `ENGINE_BUILD_DEMO` option. Each demo has its own `CMakeLists.txt` and can be built as an individual target.

## Where to start

For architectural boundaries and source organization, see [`docs/project_structure.md`](docs/project_structure.md). For module-specific APIs and configuration, start in the relevant module's `docs/` directory and `include/engine/<module>/` directory.


## Interactive demos

- `demo_planet`: windowed runs use mouse-look, `W/A/S/D`, `Space`/`Ctrl`, `Shift`, and arrow keys. Press `Left Alt` to release or recapture the cursor. `--orbit` selects the automated camera path instead.
- `demo_character_customisation`: includes its character-part OBJ assets and enables the `character_lab` example plugin by default; explicit plugin-selection command-line options still take precedence.
