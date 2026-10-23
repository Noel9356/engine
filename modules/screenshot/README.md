# screenshot module

PNG capture of the presented frame. The module exposes the `ScreenshotCapability` (capability name `screenshot`, published in `screenshot_capability.hpp` next to this module) — a single `requestScreenshot()` method — and owns the whole path from the request to a file on disk: queue, copy, readback, encode, write.

## Request paths

All of them end in the same queue, so captures are serialized and never interleave:

- the configured hotkey (`[screenshot].key`, default `F12`) — the module subscribes `KeyPressedEvent` on the event bus, which is why the key names it accepts are the same names the input module publishes;
- script mods calling `engine.request_screenshot()` through the Lua bridge;
- C++ consumers calling `requestFrameCapture()` on the `RendererCapability` (the screenshot module forwards it into the same queue);
- the host's `--smoke-frames <n>` run, which queues one capture halfway through to exercise the readback and PNG write.

## Correctness model

The capture is recorded inside the same command buffer as the frame — the swapchain image is copied before present, synchronized with the frame's fence — then swizzled to RGBA8 on the CPU and written once the GPU signals. The result is never torn and exactly one frame old; a capture recorded after present would race the compositor and produce mixed frames, which is why the copy lives inside the frame's submission. The readback waits on its own fence with a dedicated budget: reusing the presentation fence budget made slow software-rasterizer captures time out the frame. Writing the file requires the `filesystem.write` permission, which the module declares; denying it fails startup precisely instead of losing captures silently.

## Wiring

| Manifest field | Value |
|---|---|
| provided capabilities | `screenshot` |
| required capabilities | `logging`, `input`, `renderer` |
| required permissions | `capability.logging`, `capability.input`, `capability.renderer`, `filesystem.write` |

The dependency on `input` is for the hotkey subscription; a build without the input module cannot resolve this module's requirement, so a screenshot-only headless setup should queue captures through the renderer capability or the Lua bridge instead.

## Configuration

| Key | Type | Default | Meaning |
|---|---|---|---|
| `key` | string | `"F12"` | key that queues a capture (same names the event bus reports) |
| `output_directory` | string | `"screenshots"` | where PNG files are written |

Relative paths (including the default) resolve against the process working directory — the project root for the documented quick start (`./build/runtime/engine` from the engine tree), so captures land in `<project root>/screenshots` where the user actually looks; absolute paths are used as-is.

## Files

- `screenshot_capability.hpp` — the published capability surface every consumer includes
- `screenshot.hpp` / `screenshot.cpp` — the module, the queue, the readback and PNG writing
- `module_plugin.cpp` — the static plugin descriptor (name, factory, provided capabilities)
- `CMakeLists.txt` — target and option wiring (`ENGINE_BUILD_MODULE_SCREENSHOT`)
