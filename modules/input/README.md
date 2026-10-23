# input module

Keyboard and mouse aggregation for the whole engine. The module attaches GLFW key, cursor-position and scroll callbacks to the windows created by the window module and exposes the `InputCapability` (capability name `input`, published in `input_capability.hpp` next to this module; the plugin descriptor carries the implementation-bound member name `input.glfw`). The capability stays implementation-agnostic — no GLFW types cross it — so consumers are written once and survive a windowing backend swap.

## Contract

Two consumption styles, both frame-friendly. Discrete devices are polled: `isKeyDown(KeyCode)` and `isMouseButtonDown(MouseButton)` report current state at any point in the frame. Continuous devices are consumed deltas: `consumeMouseDelta(double& deltaX, double& deltaY)` and `consumeScrollDelta(double& deltaX, double& deltaY)` drain the motion accumulated since the last call — exactly once per frame per consumer — which decouples consumers from event ordering and from the platform's event batching. `setCursorCaptured(bool)` switches the platform cursor to disabled + raw motion where available, and `isCursorCaptured()` reports the mode; around every capture transition the module clears pending deltas so no accumulated jump is reported on the first frame after capture or release.

The module also publishes `KeyPressedEvent` / `KeyReleasedEvent` (defined in `input_capability.hpp`) on the event bus for event-driven consumers: the screenshot module subscribes for its configured trigger key, the Lua bridge exposes `engine.subscribe_key_pressed` / `subscribe_key_released` to script mods, and the free-fly camera uses polling instead because continuous motion belongs in the frame loop, not in event handlers. The stable key name table (`keyCodeName` / `keyCodeFromName`) is capability metadata and lives in the same published header as inline functions, so every consumer embeds the table it uses and no plugin depends on another plugin's symbols at load time.

## Wiring

| Manifest field | Value |
|---|---|
| provided capabilities | `input` |
| required capabilities | `logging`, `window` |
| required permissions | `capability.logging`, `capability.window` |

## Configuration

Reserved — the input module takes no configuration. The `[input]` section exists in `config/engine.toml` as a placeholder; behavior (which keys, what sensitivity, what the cursor mode is) belongs to consumers, and the renderer's camera settings under `[scene.camera]` are the shipped example of that split.

## Consumer reference: the free-fly camera

The renderer integrates a fly controller every frame from the input capability when present (`modules/renderer/camera_controller.hpp` — header-only, driven entirely through this capability): **WASD** flies, the mouse looks while the cursor is captured (**Esc** releases; **F** or left-click re-captures), **Space**/**E** rise, **Ctrl**/**Q** sink, **Shift** multiplies speed, and the wheel scales the base speed. The controller is the reference consumer for the delta discipline — one drain per frame, no polling of raw motion, no assumptions about event counts.

## Files

- `input_capability.hpp` — the published capability surface every consumer includes (with the stable key name table)
- `input.hpp` / `input.cpp` — the module, its capability implementation, the GLFW callbacks and the event publishing
- `module_plugin.cpp` — the static plugin descriptor (name, factory, provided capabilities)
- `CMakeLists.txt` — target and option wiring (`ENGINE_BUILD_MODULE_INPUT`)
