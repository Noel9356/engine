# Building

Requirements: CMake 3.24+, a C++20 compiler, a Vulkan loader, and `glslangValidator` (Vulkan SDK, or `apt install glslang-tools`). Linux needs X11 development packages (GLFW uses the X11 backend). No presets are used.

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build
./build/runtime/engine --validate
```

If the Vulkan loader is not on the default linker path, add `-DCMAKE_PREFIX_PATH=/your/prefix`. On Windows the build defines `WIN32_LEAN_AND_MEAN` and `NOMINMAX` once for the whole tree (engine targets and in-tree mods alike), so no source file defines them itself.

## Testing

`ctest --test-dir build` runs three tiers:

- `engine_tests` — the kernel and module unit tests; headless by design, no window, no Vulkan instance.
- `engine_headless_smoke` — boots the assembled runtime and renders 30 frames of the 3D scene (`config/smoke.toml`: small resolution, reduced GI cost) with the validation layers on, failing on crashes, exceptions and any validation error. Registered on Linux when the demo host is built.
- `engine_leak_check` — runs `config/leak_test.toml` (a sealed room with one light inside and a window opening) through `engine --leak-check`, which evaluates the captured frame against the configured sealed/lit regions and exits 4 on a light leak or a dead GI path. This is the automated no-light-leak guard for the radiance cascades pass.

The smoke test needs a Vulkan ICD plus `Xvfb` on display-less machines; it uses the system GPU driver when one is present. In a container, any software ICD works, for example Mesa lavapipe (`mesa-vulkan-drivers`) with the Khronos validation layer (`vulkan-validationlayers`), discovered through the standard loader variables:

```bash
export VK_DRIVER_FILES=/path/to/lvp_icd.json   # or VK_ICD_FILENAMES on older loaders
export VK_LAYER_PATH=/path/to/explicit_layer.d
sh scripts/run_headless_smoke.sh build/runtime/engine 30
```

## Smoke runs without a display

`engine --smoke-frames <n>` renders exactly `n` frames, queues one screenshot through the screenshot module halfway through (exercising the readback and PNG write), shuts down cleanly and reports through exit codes: `0` clean, `1` the frame loop threw, `2` bad usage, `3` the graphics backend observed validation errors during the run, `4` the leak check failed (`--leak-check`). The renderer observes backend errors through the debug messenger, so a broken pipeline or a disabled feature fails the run even when the driver would have carried on. Communication latency and frame-time reporting is opt-in (`[diagnostics] latency_report = true` or `--latency-report`); a normal run stays quiet at shutdown. See `docs/robustness.md`, section "How runtime regressions are caught".

Screenshots are written to a `screenshots` folder in the process working directory by default - the project root for the documented quick start (`./build/runtime/engine` from the engine tree) - so captures land where the user actually looks; `[screenshot] output_directory` overrides this (relative paths also resolve against the working directory, absolute paths as-is; see `modules/screenshot/README.md`).

## Dependencies

Resolution order per dependency — nothing is fetched when a usable copy already exists:

1. Existing target: a superproject-provided target, or a system package (`find_package` probes GLFW 3.4+, Catch2 3.16+, and a complete Vulkan SDK).
2. Vendored copy: `third_party/<name>/` if you create it (full CMake project or plain header/source layout).
3. Populated build tree: a `<name>-src` directory from an earlier configure is reused; no network access.
4. Pinned download: release tags for git dependencies, SHA256-verified tarballs for URL downloads.

Every configure prints an origin report:

```text
-- engine dependency origins:
--   vulkan: already available (Vulkan SDK: C:/VulkanSDK/1.4.363.0/Include)
--   glfw: already available (existing target from a superproject or find_package)
--   lua: downloaded (pinned https://www.lua.org/ftp/lua-5.4.8.tar.gz (SHA256 checked))
--   stb: cached single header reused from build/deps/stb (no download)
```

The Vulkan SDK is used whenever it ships `vulkan.hpp` next to `vulkan.h` (the SDK always does — both come from the same Vulkan-Headers release). Only an incomplete install falls back to the pinned Vulkan-Headers + Vulkan-Hpp pair; `-DENGINE_FORCE_PINNED_VULKAN=ON` ignores the SDK entirely. Lua is pinned to 5.4 because sol2 requires it exactly.

FetchContent declarations use plain names, so populated trees are `_deps/<name>-src` and the built-in local-source override is `FETCHCONTENT_SOURCE_DIR_<NAME>` (e.g. `-DFETCHCONTENT_SOURCE_DIR_GLFW=/path/to/glfw`). Engine-internal wrapper targets keep the `engine_` prefix (`engine_lua_library`, `engine_sol2_bindings`, ...) to avoid collisions.

### Where fetched dependencies live

FetchContent separates each dependency automatically:

| Directory | Contents |
|---|---|
| `build/_deps/<name>-src` | source tree, downloaded once, reused forever |
| `build/_deps/<name>-build` | its own CMake build tree (only GLFW and Catch2 ship one) |
| `build/_deps/<name>-subbuild` | FetchContent's populate bookkeeping |

Header-only dependencies the engine wraps itself (Vulkan-Hpp, sol2, toml++) are populate-only: a `-src` directory, no `-build`. stb is a single file cached in `build/deps/stb/`; the Lua tarball is cached in `build/deps/`.

### Offline and air-gapped builds

- `-DENGINE_ALLOW_FETCHCONTENT=OFF` never touches the network: configure fails naming the missing dependency unless it comes from a target, a vendored copy, a populated tree, or the cached Lua tarball.
- `FETCHCONTENT_SOURCE_DIR_<NAME>` points any dependency at a local source tree.
- One online configure, then offline reconfigures forever after. To vendor, drop the source into `third_party/<name>/` (directory does not exist in a fresh checkout; create it).

### TLS failures on downloads

MinGW CMake bundles curl/GnuTLS without a CA trust store, so HTTPS downloads can fail with `SSL peer certificate ... not OK` while git clones succeed (git uses the OS trust store). Hash-checked downloads (Lua tarball, stb header) retry automatically without certificate verification — integrity is guaranteed by the SHA256 check, not by TLS. Alternative fixes: install `curl-ca-bundle.crt` next to your `cmake.exe`, or vendor the dependencies.

## Options

Each module has an option: `ENGINE_BUILD_MODULE_LOGGING`, `..._WINDOW`, `..._INPUT`, `..._RENDERER`, `..._SCREENSHOT`, `..._LUA_SCRIPTING`, `..._PBR`, `..._RADIANCE_CASCADES`, `..._RAY_REFLECTIONS`. Excluding a module removes its code, its third-party dependencies, and its registration; a configuration still listing it fails with a precise diagnostic, never a link error. The compiled set is generated into `engine_module_availability.h`. Each module's build surface (its option, its dependencies, its shaders) is described in its own `modules/<name>/README.md`.

`ENGINE_BUILD_TESTS=OFF` removes the test suite and the Catch2 dependency entirely. `ENGINE_BUILD_DEMO=OFF` removes the host executable and the runtime layout assembly (configs and mod packages).

## Runtime layout

After building, `build/runtime/` is relocatable: `engine` (executable), `config/engine.toml`, `shaders/*.spv` (module build output), and `mods/<id>/` holding each package's `mod.toml`, its entry (library or script), its shipped defaults (`config.toml`), its world content (`scene.toml`, when the manifest declares one) and its content assets (`assets/`, when it ships any). Configuration paths resolve relative to the executable directory. The engine ships no content of its own: everything under `mods/` comes from the packages, and the tree's one package (`test_scene`) carries the demo world and its textures.

## Shaders

Shaders are module code, not content: the renderer keeps GLSL in `modules/renderer/shaders/` and compiles it via the shared `engine_compile_module_shaders` helper into `runtime/shaders/`. There is no project-root shader directory and no engine-side shader asset tree. A missing `glslangValidator` is a precise configure-time error.
