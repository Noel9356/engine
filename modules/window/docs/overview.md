# window module

window owns the engine's OS windows: it creates and destroys them, pumps the OS event queue into per-window event queues, reports keyboard state and cursor position, and gives render modules what they need to present (Vulkan instance extensions and native surface handles). A headless backend with a CPU-side surface backs the same capability set so tests and CI can run without a display server.

## Capability

The module provides four capabilities, all sharing ABI 9 (constant `windowCapabilityAbi`, see `include/engine/window/capabilities.hpp`):

| Capability | Role |
| --- | --- |
| `window` | Primary: window lifecycle (`createWindow` / `destroyWindow`), event pumping (`pollEvents`, `popEvent`, `popEvents`), size/title/position, keyboard (`isKeyDown`, `keyStateMask`), cursor (`setCursorMode`, `cursorPosition`), and headless-only pixel access (`lockSurfacePixels`). |
| `window.vulkan` | Secondary: `requiredInstanceExtensions` (what a `VkInstance` must enable) and `nativeSurfaceHandles` (Win32/X11 handles for `VkSurfaceKHR` creation). |
| `window.glfw` | Secondary: `glfwVersion` string and the raw `GLFWwindow` pointer for consumers that talk to GLFW directly. |
| `window.headless` | Secondary: `exportSurfacePpm` writes the software surface as a binary PPM (P6) file. |

Error convention: fallible calls report a `WindowError` through an out-parameter. Unknown window handles make `shouldClose` return `true` (so main loops terminate) and the other calls fail quietly. A `WindowHandle` of 0 (`invalidWindow`) means "none".

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
| `source/window.cpp` | Module implementation: configuration, backend selection, capability tables, facade over the active backend. |
| `source/window_backend.hpp` | Internal shared interface: `WindowRecord` (per-window state), `queueWindowEvent` (queueing/coalescing rules), abstract `WindowBackend`. |
| `source/glfw_backend.hpp` / `.cpp` | GLFW backend: real OS windows, GLFW callback wiring, key mapping, cursor modes, Vulkan surface support. Compiled only with `ENGINE_WINDOW_BACKEND_GLFW`. |
| `source/headless_window_backend.hpp` / `.cpp` | Headless backend: CPU RGBA "framebuffer", size changes, optional headless Vulkan (`ENGINE_WINDOW_HEADLESS_VULKAN`). |

## How it works

- **Backend selection.** On the first `createWindow` the module initializes a backend: GLFW when allowed (`window.backend` is `"auto"` or `"glfw"` and the GLFW build is available), otherwise the headless backend (a forced `"glfw"` that fails reports `BackendUnavailable`; `"auto"` falls back with a stderr note). The choice lasts until `stop()`.
- **Window creation.** The module resolves the effective description first: configured values fill the fields the caller left unset (the `has*` flags distinguish "unset"), sizes are clamped into the effective minimum/maximum limits, and the pixel area is overflow-checked. The backend then creates the OS window (GLFW hints, size limits, callbacks) or allocates the CPU surface (headless). Windows get monotonic ids starting at 1; ids are never reused.
- **Event flow.** `pollEvents` pumps GLFW once per frame; GLFW callbacks fire inside it, resolve the window's `WindowRecord` from the GLFW user pointer, and append `WindowEvent`s to the record's deque. Resize events are coalesced: a trailing same-kind `Resized`/`FramebufferResized` event is replaced in place, and when the queue is full a queued resize is dropped before the oldest event is evicted. Consumers drain the queue with `popEvent` / `popEvents` after polling.
- **Keyboard state.** The module keeps a 64-bit mask per window (bit `1 << Key` set while pressed), updated from key press/release callbacks. Because events are missed while a window is unfocused, the mask is rebuilt from GLFW's live key state on focus gain and cleared on focus loss. `keyStateMask` returns the mask ANDed with the caller's requested mask.
- **Headless surface.** Headless windows own a zeroed RGBA byte buffer sized `width * height * 4`. `setSize` resizes it (copying the overlapping region), queues resize events, and `lockSurfacePixels` maps it for inspection; `exportSurfacePpm` writes it out as P6 RGB. GLFW windows cannot be locked or exported (`Unsupported`).

## Threading and lifetime

- All capability calls and GLFW callbacks happen on the main thread; the module has no internal locks. GLFW requires its events to be pumped from the main thread, which the `pollEvents` contract reflects.
- The module owns all windows and destroys them in `stop()` before shutting the backend down (`glfwTerminate`) and unsubscribing from the event bus.
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
        // handle event.kind (Resized, KeyPressed, CloseRequested, ...)
    }

    if (window.isKeyDown(window.self, win, engine::window::Key::Escape))
        window.requestClose(window.self, win);
}
window.destroyWindow(window.self, win);
```
