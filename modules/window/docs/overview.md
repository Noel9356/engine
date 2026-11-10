# window module

window owns the engine's OS windows: it creates and destroys them, pumps the OS event queue into per-window event queues, reports keyboard, mouse, standardized gamepad, and forwarded touch input, scroll and text-input events, and gives render modules what they need to present (Vulkan instance extensions and native surface handles). A headless backend with a CPU-side surface backs the same capability set so tests and CI can run without a display server.

## Capability

The module provides four capabilities, all sharing ABI 14 (constant `windowCapabilityAbi`, see `include/engine/window/capabilities.hpp`):

| Capability | Role |
| --- | --- |
| `window` | Primary: window lifecycle (`createWindow` / `destroyWindow`), event pumping (`pollEvents`, `popEvent`, `popEvents`), size/title/position, keyboard (`isKeyDown`, `keyStateMask`), mouse buttons (`isMouseButtonDown`), cursor (`setCursorMode`, `cursorPosition`), native touch ingress (`submitTouchEvent`), clipboard text (`setClipboardText`, `copyClipboardText`), file-drop batches (`copyDroppedFiles`, `releaseDroppedFiles`, `submitFileDropEvent`), and headless-only pixel access (`lockSurfacePixels`). |
| `window.vulkan` | Secondary: `requiredInstanceExtensions` (what a `VkInstance` must enable) and `nativeSurfaceHandles` (Win32/X11 handles for `VkSurfaceKHR` creation). |
| `window.glfw` | Secondary: `glfwVersion` string and the raw `GLFWwindow` pointer for consumers that talk to GLFW directly. |
| `window.headless` | Secondary: `exportSurfacePpm` writes the software surface as a binary PPM (P6) file. |

Error convention: fallible calls report a `WindowError` through an out-parameter. Unknown window handles make `shouldClose` return `true` (so main loops terminate) and the other calls fail quietly. A `WindowHandle` of 0 (`invalidWindow`) means "none".

### Clipboard text

The primary `window` capability provides UTF-8 clipboard text access. `setClipboardText` writes the text to the active window system clipboard; `copyClipboardText` copies the current text into a caller-owned buffer. The required size includes the terminating NUL and is reported through `outRequired`; too-small buffers are left untouched. Text must be valid UTF-8 and is limited to 1 MiB. GLFW delegates to the operating-system clipboard, while the headless backend keeps an in-memory clipboard for deterministic tests. Invalid window handles, unsupported backends, and backend failures return `false`.

```cpp
std::uint32_t required = 0;
window.copyClipboardText(window.self, windowHandle, nullptr, 0, &required);
std::vector<char> clipboard(required);
if (required != 0 && window.copyClipboardText(
        window.self, windowHandle, clipboard.data(), required, &required))
{
    // clipboard.data() is a NUL-terminated UTF-8 string.
}
```


### Dropped files

The GLFW backend emits one `WindowEventKind::FilesDropped` event for each OS drop operation. The event contains a batch ID and file count; paths are copied into module-owned storage because GLFW's callback strings are only valid during the callback. Retrieve the paths using `copyDroppedFiles`: the output is a sequence of UTF-8 paths, each followed by NUL, and `outRequired` includes every terminator. The copy is all-or-nothing. Call `releaseDroppedFiles` when finished. The event queue can evict an older drop event when full; its retained path batch is released with it.

The storage is bounded to 256 paths and 8 MiB per batch, 32 KiB per path, and 64 outstanding batches per window. A batch that exceeds a limit or cannot be retained is not published. `submitFileDropEvent` exposes the same path for SDL/mobile/platform adapters and headless tests; it validates and copies the complete batch before queuing the event.

```cpp
if (event.kind == engine::window::WindowEventKind::FilesDropped)
{
    std::uint32_t required = 0;
    // A null buffer is intentionally too small; the call reports the required size.
    window.copyDroppedFiles(window.self, windowHandle, event.fileDropBatchId,
            nullptr, 0, &required);
    std::vector<char> paths(required);
    if (required != 0 && window.copyDroppedFiles(window.self, windowHandle,
            event.fileDropBatchId, paths.data(), required, &required))
    {
        for (std::size_t offset = 0; offset < required; offset += std::char_traits<char>::length(paths.data() + offset) + 1)
        {
            // Import or inspect paths.data() + offset.
        }
    }
    window.releaseDroppedFiles(window.self, windowHandle, event.fileDropBatchId);
}
```

For a two-call size query, the first `copyDroppedFiles` call intentionally returns `false` when the output buffer is null or too small; `outRequired` still receives the required byte count.

## Provided / required / optional capabilities

From `module.toml` and the entry table at the bottom of `source/window.cpp` (they agree):

- Provided: `window`, `window.vulkan`, `window.glfw`, `window.headless`
- Required: none
- Optional: none
- Declared permissions: none

Consumers need the kernel permission `capability.window` (or `capability.window.vulkan` etc.) to acquire a capability through `ParticipantContext::acquireCapability`.

## Architecture

| File | Purpose |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capabilities, declared `config_keys`. |
| `config.toml` | Defaults for the `[window]` section (see `configuration.md`). |
| `include/engine/window/capabilities.hpp` | Public capability types: ABI, errors, keys, events, descriptions, and the four capability structs. |
| `source/window.cpp` | Module implementation: configuration, backend selection, capability tables, facade over the active backend; headless PPM export uses `engine/util/ppm.hpp`. |
| `source/window_backend.hpp` | Internal shared interface: `WindowRecord` (per-window state), `queueWindowEvent` (queueing/coalescing rules), abstract `WindowBackend`. |
| `source/glfw_backend.hpp` / `.cpp` | GLFW backend: real OS windows, GLFW callback wiring, key mapping, cursor modes, Vulkan surface support. Compiled only with `ENGINE_WINDOW_BACKEND_GLFW`. |
| `source/headless_window_backend.hpp` / `.cpp` | Headless backend: CPU RGBA "framebuffer", size changes, optional headless Vulkan (`ENGINE_WINDOW_HEADLESS_VULKAN`). |

## Input events

The GLFW backend queues keyboard and mouse-button transitions, cursor movement, scroll offsets, Unicode text-input code points, and standardized gamepad events. It polls GLFW gamepads from `pollEvents`; only devices for which `glfwJoystickIsGamepad` reports a standardized mapping are exposed. The button and axis indices match `GamepadButton` / `GamepadAxis` and GLFW's standardized layout. `gamepadId` is the GLFW joystick id (0..15). Connect/disconnect, button-edge, and axis-change events are broadcast to every live window queue. Stick-axis events carry values in [-1, 1]; trigger-axis events are normalized to [0, 1] with zero meaning released. GLFW itself reports every standardized axis in [-1, 1], so the backend converts its trigger axes before publishing the engine event.

Mouse button indices match GLFW's eight-button ordering (`Left`, `Right`, `Middle`, `Back`, `Forward`, `Button6`, `Button7`, `Button8`). Cursor coordinates and relative deltas use logical window units with a top-left origin. The first cursor event after creation or focus change has a zero delta; adjacent cursor events coalesce to the latest position while summing their deltas. Scroll values retain GLFW's horizontal/vertical offsets. Text events carry Unicode scalar values rather than layout-dependent key codes. Applications should drain events regularly.

`WindowEventKind::CursorMoved`, `MouseScrolled`, and `TextInput` carry `cursorX/cursorY` plus `cursorDeltaX/cursorDeltaY`, `scrollX/scrollY`, and `codepoint`, respectively. Forward these events to UI pointer, wheel, and text-input handling as appropriate.

### Touch ingress

`WindowCapability::submitTouchEvent` is a platform-neutral ingress for native multitouch events. The platform host (for example, an Android or iOS application layer) submits one `TouchEventDescription` for each contact phase. Coordinates are normalized to [0, 1] relative to the window's drawable area, with the origin at the top-left; pressure is normalized to [0, 1]. A `(device, touchId)` pair identifies a contact from `Began` until `Ended` or `Cancelled`. Each window tracks at most `maximumTouchContactsPerWindow` (256) active contacts; invalid values, duplicate begins, excess contacts, and moves/ends for unknown contacts are rejected. Touch-move events for the same contact coalesce while preserving accumulated normalized deltas. The current desktop GLFW backend has no native touch callback; the same ingress is available to a mobile host.

This is an input bridge, not a native mobile window backend: the current window module does not create Android/iOS windows or read OS touch events itself. A mobile host must provide the native window/event loop and call `submitTouchEvent`; on Android, the GLFW backend is disabled by the module build configuration. To drive the UI, convert normalized touch coordinates to UI display pixels and forward the same contact phases through `ui.addTouch`; the UI sink uses one primary contact as its pointer, while `input_actions` retains all contacts for multitouch gestures.

## How it works

- **Backend selection.** On the first `createWindow` the module initializes a backend: GLFW when allowed (`window.backend` is `"auto"` or `"glfw"` and the GLFW build is available), otherwise the headless backend (a forced `"glfw"` that fails reports `BackendUnavailable`; `"auto"` falls back with a stderr note). The choice lasts until `stop()`.
- **Window creation.** The module resolves the effective description first: configured values fill the fields the caller left unset (the `has*` flags distinguish "unset"), sizes are clamped into the effective minimum/maximum limits, and the pixel area is overflow-checked. The backend then creates the OS window (GLFW hints, size limits, callbacks) or allocates the CPU surface (headless). Windows get monotonic ids starting at 1; ids are never reused.
- **Event flow.** `pollEvents` pumps GLFW once per frame and polls standardized gamepads. GLFW callbacks resolve the window's `WindowRecord` and append events to its deque; the file-drop callback copies complete UTF-8 path batches into bounded per-window storage before publishing `FilesDropped`; gamepad transitions are broadcast to every live window. Adjacent resize events replace one another, cursor events keep the latest position and accumulate relative deltas, consecutive scroll events accumulate offsets, and adjacent moves for one touch contact keep the latest position and accumulate deltas. When the queue is full, high-frequency state events (resize, cursor motion, scrolling, gamepad-axis motion, and touch motion) are discarded before other events; if none exists, the oldest event is evicted. Evicting a `FilesDropped` event also releases its path batch. Consumers drain the queue with `popEvent` / `popEvents` after polling.
- **Keyboard and mouse state.** The module keeps a 64-bit key mask and an eight-bit mouse-button mask per GLFW window. Both are rebuilt from GLFW's live state on focus gain and cleared on focus loss. `keyStateMask` returns the key mask ANDed with the caller's requested mask; `isMouseButtonDown` reads the mouse mask. Cursor capture maps to `GLFW_CURSOR_DISABLED` and enables raw mouse motion where GLFW/platform support it.
- **Headless surface.** Headless windows own a zeroed RGBA byte buffer sized `width * height * 4`. `setSize` resizes it (copying the overlapping region), queues resize events, and `lockSurfacePixels` maps it for inspection; `exportSurfacePpm` writes it out as P6 RGB. GLFW windows cannot be locked or exported (`Unsupported`).

## Threading and lifetime

- All capability calls and GLFW callbacks happen on the main thread; the module has no internal locks. GLFW requires its events to be pumped from the main thread, which the `pollEvents` contract reflects.
- Dropped-file paths remain owned by their window until `releaseDroppedFiles`, event-queue eviction, or window destruction. All drop-batch APIs are main-thread-only. The module owns all windows and destroys them in `stop()` before shutting the backend down (`glfwTerminate`) and unsubscribing from the event bus. The GLFW backend and touch submission API are main-thread-only.
- `lockSurfacePixels` output stays valid until the window is destroyed or resized; `unlockSurfacePixels` is currently a no-op placeholder.
- GLFW is process-global: `glfwInit`/`glfwTerminate` are tracked by one flag in the GLFW backend, so only one window backend may run at a time.

## Configuration

Defaults live in `[window]`; see `configuration.md` for every key, its validated range, and its effect.

## Integration

- Events subscribed: `ConfigurationReloadedEvent` (subscribed in `construct`, unsubscribed in `stop`). When a reload touches any key starting with `window.`, the module re-reads the `[window]` section; already-created windows keep their current settings.
- Events published: none. The module's output is its event *queues* (popped via the capability), not the kernel event bus. The application layer is expected to publish `WindowKeyEvent` from pumped key events.
- Permissions: none declared in `module.toml` or the entry table.
- Consumer example (host side, inside a `construct(ParticipantContext&)`):

```cpp
auto &window = context.acquireCapability<engine::window::WindowCapability>();

engine::window::WindowError error{};
engine::window::WindowDescription description{};
description.title  = "game";
description.width  = 1280;
description.height = 720;

engine::window::WindowHandle win =
    window.createWindow(window.self, description, &error);

while (!window.shouldClose(window.self, win))
{
    window.pollEvents(window.self);

    engine::window::WindowEvent event{};
    while (window.popEvent(window.self, win, &event))
    {
        // handle event.kind (Resized, KeyPressed, GamepadAxisMoved, TouchBegan, ...)
    }

    if (window.isKeyDown(window.self, win, engine::window::Key::Escape))
        window.requestClose(window.self, win);
}
window.destroyWindow(window.self, win);
```
