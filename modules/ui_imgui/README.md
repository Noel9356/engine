# ui_imgui module - the immediate mode debug overlay (Dear ImGui)

## What this module provides

Capability `ui` (implementation-bound member `ui.imgui`, published in `ui_capability.hpp`): the in-frame debug overlay. One key (config `[ui].key`, default F1) toggles a window with frame statistics, the scene census, the camera pose and run actions (frame capture, scene save/load). The overlay draws INSIDE the frame through the renderer's overlay seam (`renderer.overlay`, `renderer_overlay_capability.hpp`), after the composite pass and before the present transition - so a screenshot taken while the overlay is visible shows the overlay.

```toml
[ui]
key = "F1"                  # any key name the input capability knows
font_scale_percent = 100    # 100 = 1.0x, clamped to 50..400
```

## The modal rule

While the overlay is visible it owns the cursor. That is one sentence with real consequences:

- The module drains the input capability's cursor and wheel accumulators every visible frame, so even a consumer that ignores this contract reads a settled accumulator (0 motion).
- The scripting bridge additionally gates itself on `wantsMouse()` before handing deltas to a script: camera scripts stop receiving pointer motion while a debug window is up, deterministically, regardless of event subscription order.
- The run's cursor capture state is saved when the overlay shows and restored when it hides: a free-fly camera that captured the pointer gets it back exactly as it left it.

## The renderer seam (how the overlay draws)

The renderer publishes `renderer.vulkan.overlay` (the seam is deliberately an implementation-bound MEMBER, not a family - its contract is the Vulkan handle bundle): one registration, one per-frame virtual call. The ui_imgui module registers itself as the recorder during its first visible frame (the renderer's backend handle bundle is complete then - instance, physical device, logical device, graphics queue, swapchain counts and color format, all as documented opaque handles), and the renderer calls `recordOverlay` between its composite pass and the present transition. The module speaks Vulkan only there: descriptor setup and the font atlas upload for the ImGui Vulkan backend, dynamic rendering against the swapchain's color format. With no ui_imgui module in the run the recorder pointer stays null and the seam costs one branch per frame.

The ui_imgui module is therefore honest about coupling: it is the Vulkan implementation of the `ui` family, and it only works where the `renderer.vulkan` provider does - exactly like `input.glfw` only works where `window.glfw` does. A software renderer would publish the same `renderer.overlay` family with its own bundle semantics.

## Input feeding

The module does not install platform hooks. It consumes the input capability like every other consumer: cursor position (the absolute query, added to `input` for exactly this purpose), button state, key state through the engine's `KeyCode` table, modifiers, wheel deltas and the text-input events (queued between frames, flushed into the UI library's io once per frame). One input source of truth; the overlay is a consumer, never a second poller.

## Panels

- Frame statistics: delta time and frames per second, straight from the UI library's io.
- Scene census: live object count against the registry capacity (`scene`).
- Camera pose: position and look-at, read-only, from the renderer capability.
- Actions: `screenshot` (renderer frame capture), `save scene` / `load scene` (persistence_toml module, resolved lazily - the buttons degrade to one precise warning per press when the module is absent).

## Wiring

| Manifest field | Value |
|---|---|
| provided capabilities | `ui.imgui` (the `ui` family's implementation member) |
| required capabilities | `logging`, `renderer.vulkan.overlay` (the seam is a graph edge: no overlay-seam provider, no module), `window`, `input`, `scene` |
| required permissions | `capability.logging`, `capability.window`, `capability.input`, `capability.scene`, plus `capability.renderer` and `capability.persistence` declared as PERMISSIONS without graph edges |

`capability.renderer` and `capability.persistence` follow the screenshot-capability pattern: a run without those modules still starts, and the affected widgets fail precisely per use.

## Files

- `ui_capability.hpp` - the published capability surface
- `ui.hpp` / `ui.cpp` - the module, the frame handler, the widget tree
- `imgui_backend.cpp` - the Vulkan backend glue and the key mapping table
- `module_plugin.cpp` - the static plugin descriptor
- `CMakeLists.txt` - target wiring (`ENGINE_BUILD_MODULE_UI_IMGUI`)
