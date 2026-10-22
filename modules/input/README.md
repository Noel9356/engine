# input module

Keyboard and mouse aggregation for the whole engine. The module attaches GLFW key, cursor-position and scroll callbacks to the windows created by the window module and exposes the `InputCapability` (capability name `input`, contract in `kernel/include/engine/capabilities/input_contract.hpp`). The contract stays implementation-agnostic — no GLFW types cross it — so consumers are written once and survive a windowing backend swap.

## Contract

Two consumption styles, both frame-friendly. Discrete devices are polled: `isKeyDown(KeyCode)` and `isMouseButtonDown(MouseButton)` report current state at any point in the frame. Continuous devices are consumed deltas: `consumeMouseDelta(double& deltaX, double& deltaY)` and `consumeScrollDelta(double& deltaX, double& deltaY)` drain the motion accumulated since the last call — exactly once per frame per consumer — which decouples consumers from event ordering and from the platform's event batching. `setCursorCaptured(bool)` switches the platform cursor to disabled + raw motion where available, and `isCursorCaptured()` reports the mode; around every capture transition the module clears pending deltas so no accumulated jump is reported on the first frame after capture or release.

The module also publishes `KeyPressedEvent` / `KeyReleasedEvent` (defined in `input_contract.hpp`) on the event bus for event-driven consumers: the screenshot module subscribes for its configured trigger key, the Lua bridge exposes `engine.subscribe_key_pressed` / `subscribe_key_released` to script mods, and the free-fly camera uses polling instead because continuous motion belongs in the frame loop, not in event handlers.

## Wiring

| Manifest field | Value |
|---|---|
| provided capabilities | `input` |
| required capabilities | `logging`, `window` |
| required permissions | `capability.logging`, `capability.window` |

## Configuration

Reserved — the input module takes no configuration. The `[input]` section exists in `config/engine.toml` as a placeholder; behavior (which keys, what sensitivity, what the cursor mode is) belongs to consumers, and the renderer's camera settings under `[scene.camera]` are the shipped example of that split.

## Consumer reference: the free-fly camera

The renderer integrates a fly controller every frame from the input capability when present (`modules/renderer/camera_controller.hpp` — header-only, driven entirely through this contract, unit tested with a stub input in `tests/camera_controller_tests.cpp`): **WASD** flies, the mouse looks while the cursor is captured (**Esc** releases; **F** or left-click re-captures), **Space**/**E** rise, **Ctrl**/**Q** sink, **Shift** multiplies speed, and the wheel scales the base speed. The controller is the reference consumer for the delta discipline — one drain per frame, no polling of raw motion, no assumptions about event counts.

## Files

- `input.hpp` / `input.cpp` — the module, its capability implementation, the GLFW callbacks and the event publishing
- `CMakeLists.txt` — target and option wiring (`ENGINE_BUILD_MODULE_INPUT`)

## Tests

`tests/camera_controller_tests.cpp` unit tests the dominant consumer against a stub capability (capture transitions, boost scaling, delta draining). The module itself is exercised by the kernel integration tests and by the headless smoke run, which boots input together with window and renderer under Xvfb or a real display.
