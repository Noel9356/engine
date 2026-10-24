# input_glfw module

Keyboard, mouse and gamepad aggregation for the whole engine, plus action mapping and text input. The module attaches GLFW key, cursor-position, scroll and joystick callbacks to the windows created by the window_glfw module and exposes the `InputCapability` (capability name `input`, published in `input_capability.hpp` next to this module; the plugin descriptor carries the implementation-bound member name `input.glfw`). The capability stays implementation-agnostic — no GLFW types cross it — so consumers are written once and survive a windowing backend swap.

## Contract

Two consumption styles, both frame-friendly. Discrete devices are polled: `isKeyDown(KeyCode)` and `isMouseButtonDown(MouseButton)` report current state at any point in the frame. Continuous devices are consumed deltas: `consumeMouseDelta(double& deltaX, double& deltaY)` and `consumeScrollDelta(double& deltaX, double& deltaY)` drain the motion accumulated since the last call — exactly once per frame per consumer — which decouples consumers from event ordering and from the platform's event batching. `setCursorCaptured(bool)` switches the platform cursor to disabled + raw motion where available, and `isCursorCaptured()` reports the mode; around every capture transition the module clears pending deltas so no accumulated jump is reported on the first frame after capture or release.

The module also publishes events on the event bus for event-driven consumers: `KeyPressedEvent` / `KeyReleasedEvent` (the screenshot_png module subscribes for its configured trigger key; the Lua bridge exposes `engine.subscribe_key_pressed` / `subscribe_key_released`), `TextInputEvent` (text-shaped input — the Lua bridge exposes `engine.subscribe_text_input`), and `GamepadConnectedEvent` / `GamepadDisconnectedEvent` (polled buttons and axes live on the capability: `isGamepadButtonDown`, `gamepadAxis`). The free-fly camera uses polling instead of events because continuous motion belongs in the frame loop. The stable key name table (`keyCodeName` / `keyCodeFromName`) is capability metadata and lives in the same published header as inline functions, so every consumer embeds the table it uses and no plugin depends on another plugin's symbols at load time.

## Wiring

| Manifest field | Value |
|---|---|
| provided capabilities | `input` |
| required capabilities | `logging`, `window` |
| required permissions | `capability.logging`, `capability.window` |

## Configuration

The module reads one thing: the `[input.actions]` map in `config/engine.toml` (declared as `input.actions.*`, a wildcard because action names are user-authored). One action = one comma-separated binding string; each binding is a key by its stable name, `mouse:<button>`, or `gamepad:<control>` (buttons and axes: `gamepad:A`, `gamepad:LeftTrigger`, ...). `actionStrength(name)` returns the strongest currently-held binding — keys and buttons contribute 1.0 when held, axes contribute their clamped 0..1 value — and an unconfigured action reads 0, so rebinding is a configuration edit and consumers query by meaning (`engine.action_strength("jump")`). The shipped example binds `boost` and `release_cursor`.

Everything else on the `[input]` section belongs to consumers: the renderer's camera tuning lives under `[scene.camera]`, and behavior (which keys, what sensitivity, what the cursor mode is) is the consumer's decision, not the device module's.

## Consumer reference: the free-fly camera

The renderer integrates a fly controller every frame from the input capability when present (`modules/renderer_vulkan/camera_controller.hpp` — header-only, driven entirely through this capability): **WASD** flies, the mouse looks while the cursor is captured (**Esc** releases; **F** or left-click re-captures), **Space**/**E** rise, **Ctrl**/**Q** sink, **Shift** multiplies speed, and the wheel scales the base speed. The controller is the reference consumer for the delta discipline — one drain per frame, no polling of raw motion, no assumptions about event counts.

## Files

- `input_capability.hpp` — the published capability surface every consumer includes (with the stable key name table)
- `input.hpp` / `input.cpp` — the module, its capability implementation, the GLFW callbacks and the event publishing
- `module_plugin.cpp` — the static plugin descriptor (name, factory, provided capabilities)
- `CMakeLists.txt` — target and option wiring (`ENGINE_BUILD_MODULE_INPUT_GLFW`)
