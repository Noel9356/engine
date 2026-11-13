# ui module

ui gives the engine an immediate-mode user interface layer. It wraps Dear ImGui behind the engine's capability convention: consumers acquire a C struct of function pointers, feed it input events, drive a `beginFrame` / widgets / `endFrame` lifecycle each rendered frame, and receive the resulting draw data in plain structs that any renderer can turn into GPU work without knowing that ImGui exists underneath.

## Capability

The module provides the `ui` capability (ABI 4, constant `uiCapabilityAbi`, see `include/engine/ui/capabilities.hpp`). All calls take the capability's `self` pointer as their first argument. Handles: `UiContextHandle` (0 = `invalidUiContext`) and `UiFontHandle` (0 = the built-in default font).

Main function groups:

| Group | Functions |
| --- | --- |
| Context lifetime | `createContext`, `destroyContext` |
| Frame lifecycle | `beginFrame`, `endFrame`, `drawData`, `captureState` |
| Input sink | `addMousePosition`, `addMouseButton`, `addTouch`, `addMouseWheel`, `addKey`, `addGamepadButton`, `addGamepadAxis`, `addText`, `addFocus` |
| Windows and widgets | `beginWindow` / `endWindow`, `text`, `textColored`, `bulletText`, `separator`, `sameLine`, `spacing`, `newLine`, `dummy`, `button`, `checkbox`, `sliderFloat`, `inputText`, `inputInt`, `inputFloat`, `sliderInt`, `dragFloat`, `combo`, `selectable` / `selectableEx`, `radioButton`, `progressBar`, `image` |
| Layout | `beginChild` / `endChild`, `beginGroup` / `endGroup`, `indent`, `unindent` |
| Tree | `treeNode`, `treeNodeEx`, `treePop`, `setNextItemOpen`, `collapsingHeader`, `collapsingHeaderBool` |
| Disabled sections | `beginDisabled`, `endDisabled` |
| Popups | `openPopup`, `openPopupId`, `beginPopup`, `beginPopupModal`, `beginPopupContextItem`, `beginPopupContextWindow`, `beginPopupContextVoid`, `endPopup`, `closeCurrentPopup`, `isPopupOpen` |
| Menus | `beginMenuBar` / `endMenuBar`, `beginMenu` / `endMenu`, `menuItem`, `menuItemActivated` |
| Tabs | `beginTabBar` / `endTabBar`, `beginTabItem` / `endTabItem`, `setTabItemClosed` |
| Tables | `beginTable` / `endTable`, `tableNextRow`, `tableNextColumn`, `tableSetColumnIndex`, `tableSetupColumn`, `tableSetupScrollFreeze`, `tableHeadersRow`, `tableHeader`, `tableColumnCount`, `tableColumnIndex`, `tableRowIndex`, `tableColumnName`, `tableColumnIsEnabled`, `tableColumnIsSorted`, `tableSetBgColor` |
| Tooltips | `setTooltip`, `beginTooltip` / `endTooltip` |
| Vector sliders/drags | `sliderFloat2` / `sliderFloat3` / `sliderFloat4`, `sliderInt2` / `sliderInt3` / `sliderInt4`, `dragFloat2` / `dragFloat3`, `dragInt2` / `dragInt3` |
| Color edits | `colorEdit3`, `colorEdit4`, `colorButton` |
| Plots | `plotLines`, `plotHistogram` |
| Item/window queries | `isItemHovered`, `isItemActive`, `isItemFocused`, `isItemClicked`, `isItemVisible`, `isItemActivated`, `isItemDeactivated`, `isItemDeactivatedAfterEdit`, `isAnyItemHovered`, `isAnyItemActive`, `isWindowAppearing`, `isWindowCollapsed`, `isWindowFocused`, `isWindowHovered` |
| Window placement | `windowWidth`, `windowHeight`, `windowPositionX`, `windowPositionY`, `setNextWindowFocus`, `setNextWindowCollapsed`, `setWindowFocus`, `setWindowCollapsedNamed` |
| Mouse cursor | `setMouseCursor`, `mouseCursor` |
| Fonts | `addFontFromFile`, `addFontFromMemoryTTF`, `pushFont` / `popFont`, `fontAtlas`, `setFontTexture` |
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
| `module.toml` | Module descriptor (name, version, provided capability, declared config keys). Read by kernel module discovery. |
| `config.toml` | Default `[ui]` settings (see `configuration.md`). |
| `include/engine/ui/capabilities.hpp` | Public capability types: ABI, handles, error/key/style enums, flag enums, vertex/command/draw-data structs, `UiCapability` function-pointer table. |
| `source/ui_backend.hpp` | Internal header: `UiContextRecord`, `UiModuleState`, mapping and helper function declarations shared by every `ui_*` TU. |
| `source/ui.cpp` | Module entry: `UiModule` (manifest, capability binding, `construct`/`start`/`stop`), `*Impl` forwarders that delegate to `backend::` free functions, and `engineQueryModuleEntryTable`. |
| `source/ui_mappings.cpp` | Enum -> ImGui mapping tables (keys, gamepad, window flags, colors, style variables, child/tree/selectable/popup/tab/table/color-edit flags, mouse cursor, table bg target, style preset). |
| `source/ui_context.cpp` | Context lifetime (`createContext`, `destroyContext`), frame lifecycle (`beginFrame`, `endFrame`), draw-data extraction (`copyDrawData`, `clearDrawData`), `drawData`, `captureState`, description validation. |
| `source/ui_input.cpp` | Input event sink: mouse, touch, wheel, key, gamepad, text, focus. |
| `source/ui_widgets.cpp` | Windows, text, spacing, buttons, selects, radio, progress, image, numeric widgets, combo, vector sliders/drags, color edits, plots, item/window state queries, window placement queries. |
| `source/ui_layout.cpp` | Child windows, groups, indent, dummy, newLine, tree nodes, collapsing headers, disabled sections. |
| `source/ui_popup.cpp` | Popups, modal dialogs, context-menu popups, menu bars, menu items, tooltips. |
| `source/ui_table.cpp` | Tab bars/items and tables (setup, headers, row/column navigation, query helpers, background color). |
| `source/ui_fonts_style.cpp` | Font loading (file + memory), font atlas, font texture, style stack (colors and variables), next-window placement, mouse cursor. |

The module is conditionally functional: without the `ENGINE_UI_BACKEND_IMGUI` build definition the ImGui library is not linked and every capability entry fails with `UiError::BackendUnavailable`. Every backend TU is guarded by the macro so the same source set compiles in both configurations; the non-backend build keeps the `UiModuleState` field set empty and the `*Impl` methods short-circuit to the unavailable error.

## How it works

- **Contexts.** `createContext` validates the description (finite positive scale; finite non-negative default font size; zero-valued reserve/maximum fields fall back to the built-in defaults; reserve above maximum is rejected), creates an `ImGuiContext`, applies the requested `initialStyle` (or the module-level default style when the description did not pin one), scales every style size by `globalScale`, enables keyboard/gamepad navigation flags as requested, and installs the default font at `defaultFontSize` when non-zero. Handles are handed out monotonically starting at 1. The first context fixes the owning thread for all contexts.
- **Frame lifecycle.** `beginFrame` validates the `UiFrameDescription`, clamps `deltaSeconds` into [1 ms, 0.25 s] for ImGui's internal timers, clears the previous frame's draw data, and calls `ImGui::NewFrame()`. Consumers then issue widget calls (immediate mode: each call appends to the ImGui frame) and call `endFrame`, which runs `ImGui::Render()`.
- **Draw data extraction.** `endFrame` flattens ImGui's draw lists into `UiDrawData`: vertices and indices from all lists are concatenated, each `UiDrawCommand`'s offsets are rebased into the combined buffers, clip rectangles and texture ids are copied through, and commands carrying custom callbacks are dropped (only the reset-render-state marker survives as `resetRenderState = true`). The first pass counts every list's vertices/indices/commands so the second pass can copy into reserved buffers without reallocation; custom-callback commands are skipped in both passes so the totals match the surviving commands. If any total exceeds the description's `maximumVertices` / `maximumIndices` / `maximumCommands`, the buffers are cleared and the frame fails with `CapacityExceeded`.
- **Input.** The `add*` functions forward events into ImGui's IO queue (`AddMousePosEvent`, `AddKeyEvent`, and friends); they take effect on the next `beginFrame`. `addGamepadAxis` splits a stick axis into ImGui's positive/negative analog keys and clamps to [-1, 1]; triggers drive only the positive key. `addTouch` maps the first active contact to ImGui's left mouse button and pointer; concurrent contacts are ignored by the UI sink, while `input_actions` retains full multitouch state for gestures. Touch coordinates are UI display pixels, not normalized window coordinates. A cancelled contact or focus loss releases the pointer button. `addText` accepts single Unicode codepoints and rejects surrogates. `captureState` exposes ImGui's `WantCaptureMouse` / `WantCaptureKeyboard` / `WantTextInput` flags so the host can suppress game input while the UI is being used.
- **Widgets.** Numeric controls include unconstrained integer/float inputs, bounded integer/float sliders, and a bounded float drag control. `dragFloat` requires finite, strictly increasing bounds; a zero-width range is rejected because Dear ImGui interprets `minimum >= maximum` as an unbounded drag. `combo` accepts a caller-owned array of non-null item strings and a selected index in `[-1, itemCount)`; `-1` represents no current selection. `selectable` toggles the pointed-to boolean when activated; `selectableEx` takes a read-only selected state and a `UiSelectableFlags` value (use it when the caller wants to control selection externally, e.g. for highlight-only rows). `radioButton` updates a caller-owned integer selection. Vector sliders/drags accept 2/3/4 component arrays; the integer variants copy into a temporary native array because ImGui's `SliderIntN` takes `int*`. All widget buffers and values remain caller-owned and must stay valid for the duration of the call.
- **Layout, popups, menus, tabs, tables, tooltips, tree.** These mirror the ImGui Begin/End pairs. Every `begin*` returning false still needs the matching `end*` (the implementation tolerates the spurious end call so consumers can keep the bracket structure unconditional). `treePop` pairs with `treeNode` / `treeNodeEx` only when the begin returned true; `collapsingHeader` never needs a pop because the underlying ImGui call sets `NoTreePushOnOpen` internally. `beginDisabled`/`endDisabled` bracket a section whose widgets render greyed-out and reject input. `openPopup` queues an open request that the matching `beginPopup` consumes; `closeCurrentPopup` closes the popup that owns the current item. `beginTable` requires a column count greater than zero; column setup (`tableSetupColumn`) must precede the first `tableNextRow`. `tableColumnIsEnabled` and `tableColumnIsSorted` read ImGui's column status flags via `TableGetColumnFlags`; they are meaningful while a table is active.
- **Color edits.** `colorEdit3` and `colorEdit4` accept float components in 0..1 range; `colorButton` renders a swatch from a 4-component float array. The `UiColorEditFlags` enum mirrors the subset of `ImGuiColorEditFlags` the engine exposes; `UiColorEditFlags::DefaultOptions` matches ImGui's recommended defaults.
- **Plots.** `plotLines` and `plotHistogram` accept a caller-owned sample array plus a count; the overlay string (may be null) is drawn in the top-left corner of the graph. Values are passed straight to ImGui; the caller is responsible for the array's lifetime for the duration of the call.
- **Item/window queries.** `isItem*` and `isWindow*` reflect the state of the most recently submitted widget or the current window. The boolean returns are only meaningful inside an active frame; outside one they return false rather than touching ImGui.
- **Window placement.** `windowWidth` / `windowHeight` / `windowPositionX` / `windowPositionY` query the current window. `setNextWindowFocus` and `setNextWindowCollapsed` apply to the next `beginWindow`; `setWindowFocus` and `setWindowCollapsedNamed` act on a named window.
- **Mouse cursor.** `setMouseCursor` sets ImGui's desired cursor shape (the host window backend is responsible for translating it to an OS cursor); `mouseCursor` returns the current shape, or `UiMouseCursor::None` outside a frame.
- **Fonts.** `addFontFromFile` loads a TTF from disk and `addFontFromMemoryTTF` loads one from a caller-owned buffer that ImGui copies into the atlas (the caller may free the buffer once the call returns). Both rebuild the atlas, so they must be called outside an active frame. `fontAtlas` returns the built atlas as RGBA pixels (uploaded by the renderer), and `setFontTexture` stores the renderer's texture handle so draw commands can reference it.

## Threading and lifetime

- A context belongs to the thread that created it. Every call checks the calling thread; foreign-thread use finds no context (`InvalidHandle`), and `createContext` on a foreign thread reports `WrongThread` explicitly. All contexts share one owner thread: the first `createContext` fixes it, and the last `destroyContext` releases it.
- There is no internal locking; the capability is designed for a single UI thread.
- `drawData` points into context-owned buffers and stays valid until the next `beginFrame` on that context. The font-atlas pixels stay valid until fonts change.
- `stop()` destroys every context and resets the owner-thread pin and the next-context counter; ImGui state does not survive a module restart. Context handles are never reused within one run.

## Configuration

The module reads `[ui]` keys once during `construct()` (`save_window_layout`, `ini_filename`, `default_style`, `default_font_size`); see `configuration.md` for the values and the host override paths.

## Integration

- Events: the module publishes nothing and subscribes to nothing — `construct` does not touch the event bus. The configuration values are frozen into each `UiContextRecord` at creation, so a live reload would only affect later contexts; the host can already override every field per-context via `UiContextDescription`.
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
        engine::ui::UiWindowFlags::None))
    {
        if (ui.collapsingHeader(ui.self, context, "Rendering",
            engine::ui::UiTreeNodeFlags::DefaultOpen))
        {
            float speed = 0.5f;
            ui.sliderFloat(ui.self, context, "speed", &speed, 0.0f, 1.0f, "%.2f");
            float roughness = 0.5f;
            ui.dragFloat(ui.self, context, "roughness", &roughness, 0.01f, 0.0f, 1.0f, "%.2f");
            float color[3] = {0.8f, 0.6f, 0.4f};
            ui.colorEdit3(ui.self, context, "tint", color, engine::ui::UiColorEditFlags::DefaultOptions);
        }

        if (ui.beginTabBar(ui.self, context, "modes",
            engine::ui::UiTabBarFlags::Reorderable))
        {
            if (ui.beginTabItem(ui.self, context, "Quality", nullptr,
                engine::ui::UiTabItemFlags::None))
            {
                std::int32_t quality = 1;
                const char *qualityItems[] = {"Low", "Medium", "High"};
                ui.combo(ui.self, context, "quality", &quality, qualityItems, 3);
                ui.endTabItem(ui.self, context);
            }
            if (ui.beginTabItem(ui.self, context, "Stats", nullptr,
                engine::ui::UiTabItemFlags::None))
            {
                if (ui.beginTable(ui.self, context, "stats", 2,
                    engine::ui::UiTableFlags::Borders | engine::ui::UiTableFlags::RowBackground,
                    0.0f, 0.0f, 0.0f))
                {
                    ui.tableSetupColumn(ui.self, context, "Metric",
                        engine::ui::UiTableColumnFlags::WidthStretch, 0.0f, 0);
                    ui.tableSetupColumn(ui.self, context, "Value",
                        engine::ui::UiTableColumnFlags::WidthFixed, 80.0f, 0);
                    ui.tableHeadersRow(ui.self, context);
                    ui.tableNextRow(ui.self, context, engine::ui::UiTableRowFlags::None, 0.0f);
                    ui.tableNextColumn(ui.self, context);
                    ui.text(ui.self, context, "frame time");
                    ui.tableNextColumn(ui.self, context);
                    ui.text(ui.self, context, "16.6 ms");
                    ui.endTable(ui.self, context);
                }
                ui.endTabItem(ui.self, context);
            }
            ui.endTabBar(ui.self, context);
        }
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
