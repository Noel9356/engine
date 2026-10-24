# window_glfw module

The windowing backend: creates platform windows, pumps the OS event queue, and exposes the `WindowCapability` (capability name `window`, published in `window_capability.hpp` next to this module). The plugin descriptor and manifest carry the implementation-bound member name `window.glfw`: consumers demand the family and a mod that NEEDS this backend names the member exactly (`docs/architecture.md`, "Capability surfaces"). The shipped implementation is GLFW 3.4, but the capability is written so the renderer neither knows nor cares — the window surface deals in opaque identifiers, `void*` presentation surfaces, and extension-name strings, which is what keeps a Vulkan renderer and a hypothetical OpenGL renderer indistinguishable from behind the capability.

## Contract

The capability splits into two groups. The first group is what a generic host loop needs: `createMainWindow(const WindowDescription&)` (title, size, resizability — seeded from `[window]`), `windowCloseRequested(WindowIdentifier)` for the loop's quit condition, `pollEvents()` to pump the platform queue once per frame, `framebufferWidth`/`framebufferHeight` for resize-aware consumers, and `windowIdentifiers()` to enumerate. The second group is what a rendering backend needs, and it is deliberately platform-neutral: `requiredInstanceExtensions(graphicsApi)` returns the instance extensions the platform demands for presentation, `createPresentationSurface(graphicsApi, graphicsInstance)` returns the backend-specific surface as an opaque `void*` (for Vulkan this is the `VkSurfaceKHR` produced by `glfwCreateWindowSurface`), and `nativeWindowHandle`/`windowIdentifierFromNativeHandle` map between engine identifiers and raw platform handles. A renderer made from a different graphics API calls the same two methods with its own API tag — the window_glfw module absorbs the difference.

The module publishes `WindowResizedEvent` on the event bus when a framebuffer size changes; the renderer subscribes and rebuilds its swapchain-dependent state, which is why the present semaphores involved in that path are owned per swapchain image, not per frame slot (see `modules/renderer_vulkan/README.md`).

## Wiring

| Manifest field | Value |
|---|---|
| provided capabilities | `window` |
| required capabilities | `logging` |
| required permissions | `capability.logging` |

## Configuration

The module reads its `[window]` section during startup; missing keys fall back to the defaults.

| Key | Type | Default | Meaning |
|---|---|---|---|
| `title` | string | `"engine"` | window title |
| `width` | integer | `1280` | initial width |
| `height` | integer | `720` | initial height |
| `resizable` | bool | `true` | resize events reach subscribers |

## Implementation

GLFW is initialized in the module's `start()` (`glfwInit` failure is a precise thrown diagnostic) and terminated in `stop()`, so a validated-but-never-started run touches no platform state — the kernel's `validate()` stays side-effect free. Each `GLFWwindow*` is created from the `WindowDescription` and hidden behind a `WindowIdentifier`, and the frame loop calls `pollEvents()` exactly once per frame; resize callbacks publish `WindowResizedEvent` through the module's bound `ComponentContext` event bus. The module keeps all GLFW types inside its translation unit — no consumer header sees them, which is the mechanical half of the portability rule the published-header guard enforces at configure time (`cmake/capability_header_guard.cmake`, called from this module's `CMakeLists.txt`).

The scene's geometry and the window's framebuffer are independent unit systems. The renderer's projection derives from the framebuffer, while the analytic SDF the lighting modules trace derives from `[scene]` — the two agree because the camera's aspect ratio tracks the framebuffer, which the resize event chain guarantees.

## Files

- `window_capability.hpp` — the published capability surface every consumer includes
- `window.hpp` / `window.cpp` — the module, its capability implementation and GLFW lifecycle
- `module_plugin.cpp` — the static plugin descriptor (name, factory, provided capabilities)
- `CMakeLists.txt` — target and option wiring (`ENGINE_BUILD_MODULE_WINDOW_GLFW`)
