# screenshot module

PNG capture of the presented frame. The module exposes the `ScreenshotCapability` (capability name `screenshot`, contract in `kernel/include/engine/capabilities/screenshot_contract.hpp`) — a single `requestScreenshot()` method — and owns the whole path from the request to a file on disk: queue, copy, readback, encode, write.

## Request paths

All of them end in the same queue, so captures are serialized and never interleave:

- the configured hotkey (`[screenshot].key`, default `F12`) — the module subscribes `KeyPressedEvent` on the event bus, which is why the key names it accepts are the same names the input module publishes;
- script mods calling `engine.request_screenshot()` through the Lua bridge;
- C++ consumers calling `requestFrameCapture()` on the `RendererCapability` (the screenshot module forwards it into the same queue);
- the QA harness (`scripts/qa/qa_capture/`, a script mod) and the headless smoke run, which queues one capture halfway through to exercise the readback and PNG write every run.

## Correctness model

The capture is recorded inside the same command buffer as the frame — the swapchain image is copied before present, synchronized with the frame's fence — then swizzled to RGBA8 on the CPU and written once the GPU signals. The result is never torn and exactly one frame old; capturing after present once raced the compositor and produced mixed frames (problem 7 in `docs/robustness.md`). The readback waits on its own fence with a dedicated budget: reusing the 1 s presentation fence budget once made slow software-rasterizer captures time out the frame (problem 30). Writing the file requires the `filesystem.write` permission, which the module declares; denying it fails startup precisely instead of losing captures silently.

## Wiring

| Manifest field | Value |
|---|---|
| provided capabilities | `screenshot` |
| required capabilities | `logging`, `input`, `renderer` |
| required permissions | `capability.logging`, `capability.input`, `capability.renderer`, `filesystem.write` |

The dependency on `input` is for the hotkey subscription; a build without the input module cannot resolve this module's requirement, so a screenshot-only headless setup should queue captures through the renderer contract or the Lua bridge instead.

## Configuration

| Key | Type | Default | Meaning |
|---|---|---|---|
| `key` | string | `"F12"` | key that queues a capture (same names the event bus reports) |
| `output_directory` | string | `"screenshots"` | where PNG files are written |

Relative paths (including the default) resolve against the process working directory — the project root for the documented quick start (`./build/runtime/engine` from the engine tree), so captures land in `<project root>/screenshots` where the user actually looks; absolute paths are used as-is.

## Files

- `screenshot.hpp` / `screenshot.cpp` — the module, the queue, the readback and PNG writing
- `CMakeLists.txt` — target and option wiring (`ENGINE_BUILD_MODULE_SCREENSHOT`)

## Tests

The headless smoke run queues one capture per run ( exercising readback + PNG write + permission grant) and fails on any error; the QA harness (`scripts/qa/`) builds pixel-level verification on top of the same path.
