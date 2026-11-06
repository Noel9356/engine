# ui module

ui gives the engine an immediate-mode user interface layer. It wraps Dear ImGui behind the engine's capability convention: consumers acquire a C struct of function pointers, feed it input events, drive a `beginFrame` / widgets / `endFrame` lifecycle each rendered frame, and receive the resulting draw data in plain structs that any renderer can turn into GPU work without knowing that ImGui exists underneath.

## Capability

The module provides the `ui` capability (ABI 1, constant `uiCapabilityAbi`, see `include/engine/ui/capabilities.hpp`). All calls take the capability's `self` pointer as their first argument. Handles: `UiContextHandle` (0 = `invalidUiContext`) and `UiFontHandle` (0 = the built-in default font).

Main function groups:

| Group | Functions |
| --- | --- |
| Context lifetime | `createContext`, `destroyContext` |
| Frame lifecycle | `beginFrame`, `endFrame`, `drawData`, `captureState` |
| Input sink | `addMousePosition`, `addMouseButton`, `addMouseWheel`, `addKey`, `addGamepadButton`, `addGamepadAxis`, `addText`, `addFocus` |
| Windows and widgets | `beginWindow` / `endWindow`, `text`, `textColored`, `separator`, `sameLine`, `spacing`, `button`, `checkbox`, `sliderFloat`, `inputText`, `progressBar`, `image` |
| Fonts | `addFontFromFile`, `pushFont` / `popFont`, `fontAtlas`, `setFontTexture` |
| Style | `pushStyleColor` / `popStyleColor`, `pushStyleVariable` / `popStyleVariable`, `setNextWindowPosition`, `setNextWindowSize` |

Error convention: fallible calls report a `UiError` through an out-parameter and return a handle or boolean. `drawData` returns null for an unknown or foreign-thread context. Widget calls made outside an active frame are silently ignored (they return false / do nothing rather than reporting an error).

## Provided / required / optional capabilities

From `module.toml` and the entry table at the bottom of `source/ui.cpp` (they agree):

- Provided: `ui`
- Required: none
- Optional: none
- Declared permissions: none

Consumers still need the kernel's `capability.ui` permission to acquire the capability through `ParticipantContext::acquireCapability`.

## Architecture

| File | Purpose |
| --- | --- |
| `module.toml` | Module descriptor (name, version, provided capability). Read by kernel module discovery. |
| `config.toml` | Default `[ui]` layout-persistence setting (see `configuration.md`). |
| `include/engine/ui/capabilities.hpp` | Public capability types: ABI, handles, error/key/style enums, vertex/command/draw-data structs, `UiCapability` function-pointer table. |
| `source/ui.cpp` | The whole implementation: ImGui context records, key/flag/color mapping, frame lifecycle, draw-data flattening, module entry table. |

The module is conditionally functional: without the `ENGINE_UI_BACKEND_IMGUI` build definition the ImGui library is not linked and every capability entry fails with `UiError::BackendUnavailable`.

## How it works

- **Contexts.** `createContext` validates the description (finite positive scale; zero-valued reserve/maximum fields fall back to the built-in defaults; reserve above maximum is rejected), creates an `ImGuiContext`, applies `StyleColorsDark` scaled by `globalScale`, enables keyboard/gamepad navigation flags as requested, and installs the default font. Handles are handed out monotonically starting at 1. The first context fixes the owning thread for all contexts.
- **Frame lifecycle.** `beginFrame` validates the `UiFrameDescription`, clamps `deltaSeconds` into [1 ms, 0.25 s] for ImGui's internal timers, clears the previous frame's draw data, and calls `ImGui::NewFrame()`. Consumers then issue widget calls (immediate mode: each call appends to the ImGui frame) and call `endFrame`, which runs `ImGui::Render()`.
- **Draw data extraction.** `endFrame` flattens ImGui's draw lists into `UiDrawData`: vertices and indices from all lists are concatenated, each `UiDrawCommand`'s offsets are rebased into the combined buffers, clip rectangles and texture ids are copied through, and commands carrying custom callbacks are dropped (only the reset-render-state marker survives as `resetRenderState = true`). If any total exceeds the description's `maximumVertices` / `maximumIndices` / `maximumCommands`, the buffers are cleared and the frame fails with `CapacityExceeded`.
- **Input.** The `add*` functions forward events into ImGui's IO queue (`AddMousePosEvent`, `AddKeyEvent`, and friends); they take effect on the next `beginFrame`. `addGamepadAxis` splits a stick axis into ImGui's positive/negative analog keys and clamps to [-1, 1]; triggers drive only the positive key. `addText` accepts single Unicode codepoints and rejects surrogates. `captureState` exposes ImGui's `WantCaptureMouse` / `WantCaptureKeyboard` / `WantTextInput` flags so the host can suppress game input while the UI is being used.
- **Fonts.** `fontAtlas` returns the built atlas as RGBA pixels (uploaded by the renderer), and `setFontTexture` stores the renderer's texture handle so draw commands can reference it. `addFontFromFile` loads a TTF and rebuilds the atlas, so it must be called outside an active frame.

## Threading and lifetime

- A context belongs to the thread that created it. Every call checks the calling thread; foreign-thread use finds no context (`InvalidHandle`), and `createContext` on a foreign thread reports `WrongThread` explicitly. All contexts share one owner thread: the first `createContext` fixes it, and the last `destroyContext` releases it.
- There is no internal locking; the capability is designed for a single UI thread.
- `drawData` points into context-owned buffers and stays valid until the next `beginFrame` on that context. The font-atlas pixels stay valid until fonts change.
- `stop()` destroys every context; ImGui state does not survive a module restart. Context handles are never reused.

## Configuration

The module reads `ui.save_window_layout` once during `construct()`. It is disabled by default to avoid synchronous ini persistence in frame execution; see `configuration.md`.

## Integration

- Events: the module publishes nothing and subscribes to nothing — `construct` does not touch the event bus.
- Permissions: none declared in `module.toml` or the entry table.
- Consumer example (host side, inside a `construct(ParticipantContext&)`):

```cpp
auto &ui = context.acquireCapability<engine::ui::UiCapability>();

engine::ui::UiError error{};
engine::ui::UiContextHandle context =
    ui.createContext(ui.self, nullptr, &error);          // defaults

engine::ui::UiFrameDescription frame{};
frame.displayWidth  = 1280.0f;
frame.displayHeight = 720.0f;
frame.deltaSeconds  = 1.0f / 60.0f;

if (ui.beginFrame(ui.self, context, &frame, &error))
{
    if (ui.beginWindow(ui.self, context, "Debug", nullptr,
                       engine::ui::UiWindowFlags_None))
    {
        float value = 0.5f;
        ui.sliderFloat(ui.self, context, "speed", &value, 0.0f, 1.0f, "%.2f");
        ui.endWindow(ui.self, context);                  // always pair
    }
    ui.endFrame(ui.self, context, &error);

    const engine::ui::UiDrawData *draw = ui.drawData(ui.self, context);
    if (draw != nullptr && draw->commandCount != 0)
    {
        // render draw->vertices / draw->indices / draw->commands
    }
}
```
