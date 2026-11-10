# camera module

camera provides handle-based cameras for the engine: each camera is a pose (position, yaw/pitch orientation derived from an up vector) plus a projection (perspective or orthographic), driven by a per-frame input sample through one of three control modes (fixed, orbit, fly). Consumers call `viewState` with their viewport size and receive cached view/projection matrices ready for rendering. `worldToViewport` projects world positions to pixel coordinates; `viewportToWorldRay` unprojects a pixel into a normalized world-space ray. Defaults for new cameras come from the `[camera]` configuration section and hot-reload when the kernel publishes a configuration-reloaded event.

## Capability

The module provides the `camera` capability (ABI version constant `cameraCapabilityAbi = 10`, see `include/engine/camera/capabilities.hpp`). Handles are `std::uint32_t` starting at 1; `0` (`invalidCamera`) means "none". Only `createCamera` reports a `CameraError` (optional out-parameter). Mutators with no result fail silently on invalid input; boolean-returning operations report success or failure directly.

Entries: `createCamera`/`destroyCamera`, `setLookAt`, `setControlMode`, `setOrbitDistance`, `setMovementSpeed`, `setLookSpeed`, `setMouseSensitivity`, `setControlSettings`, `setProjection`, `update`, `viewState`, `worldToViewport`, and `viewportToWorldRay`.

## Provided / required / optional capabilities

- `module.toml`: `provides = ["camera"]`, `requires = []`, `optional_capabilities = ["script_runtime"]`.
- Entry table (bottom of `source/camera.cpp`): `providedCapabilities = {"camera"}`, `requiredCapabilities = nullptr` (count 0), `optionalCapabilities = {"script_runtime"}` (count 1), `declaredPermissions = nullptr` (count 0).

The module optionally acquires `script_runtime` to register its Lua bridge. The camera capability remains usable without a scripting backend and declares no permissions.

## Architecture

| File | Role |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capability, declared `config_keys` (27 keys). Read by kernel module discovery. |
| `config.toml` | Default values for the `[camera]` configuration section (see `configuration.md`). |
| `include/engine/camera/capabilities.hpp` | Public capability types: ABI, handle/error enums, control modes and projection types, input/settings/parameters/view-state structs, function-pointer table. |
| `source/camera.cpp` | The implementation: capacity-limited camera records, configuration and hot-reload, control-mode integration, cached matrix building with glm, world/viewport projection and ray construction, optional Lua bridge registration, capability binding, module entry table. |
| `source/lua_bridge.cpp`, `source/lua_bridge.hpp` | Optional `script_runtime` bridge for creation, destruction, pose/projection changes, per-frame updates, eye-position queries, projection, and viewport-ray construction. |

## How it works

- **Per-camera record.** `CameraRecord` stores pose (`position`, `up`, yaw/pitch in degrees) and an orbit pivot (`target`, `orbitDistance`). Orientation is expressed in a reference frame derived from the up vector: `referenceFrame()` projects +Z onto the plane perpendicular to `up` (falling back to +X when they are nearly parallel) and crosses it to get `referenceRight`. Yaw/pitch and direction vectors convert back and forth in that frame, so a changed up vector does not corrupt the pose.
- **Creation.** `createCamera` starts from the configuration defaults, then adopts every valid field from `CameraParameters` (vectors are accepted when flagged with `hasPosition`/`hasTarget`/`hasUp` or non-zero; invalid scalars keep the default). A near-zero-length up vector is rejected back to +Y, the initial yaw/pitch is seeded from the position -> target direction, and the projection values are coerced through `clampProjection` (hard FOV range [0.1, 179] degrees, far plane forced above near). For orbit cameras the radius comes from the explicit parameter, else from the |position - target| distance when a pose was given.
- **Integration (`update`).** Input axes are clamped to [-1, 1] with non-finite values counting as 0; mouse deltas are magnitude-clamped to 10000 px; `deltaSeconds` is clamped to `maxDeltaSeconds` so a pause or debugger break cannot teleport the camera. Orbit mode treats the move axes as rotation input and zoom as radius change (floor 0.5), re-placing the eye on the orbit sphere after any change; fly mode moves along the view/right/ up directions and rotates from axis or mouse input, with yaw wrapped mod 360 and pitch clamped to `pitchLimitDegrees`.
- **Matrix caching (`viewState`).** Each record caches the `ViewState` (column-major 4x4 view and projection, plus eye position). The view matrix rebuilds when the pose changed (`viewDirty`); the projection rebuilds when its inputs changed (`projectionDirty`) or when the viewport size differs from the cached one. A zero viewport falls back to aspect 1. `setProjection` applies changes on a copy and commits only when something actually changed, so equal calls do not invalidate the cache.
- **Mode transitions.** Entering orbit from fly re-derives the orbit radius from the current pose; leaving orbit for fly intentionally keeps the pose.
- **Viewport queries.** `worldToViewport` projects world coordinates through the same cached view/projection matrices used by rendering. It returns top-left-origin pixel coordinates and maps OpenGL NDC depth from [-1, 1] to [0, 1]; projected X/Y are not clamped, so off-screen points remain distinguishable. `viewportToWorldRay` unprojects the requested pixel at the near/far clip planes. Perspective rays start at the camera eye; orthographic rays start on the near plane. Both APIs reject zero-sized viewports and invalid inputs.

## Lua bridge

When `script_runtime` is present, each Lua runtime receives the `camera.*` functions: `create`, `destroy`, `set_mode`, `look_at`, `set_projection`, `update`, `view_position`, `world_to_viewport`, `viewport_to_world_ray`, and `set_control_settings`. Camera handles and viewport dimensions are numeric arguments; modes are `"fixed"`, `"orbit"`, or `"fly"`, and projection names are `"perspective"` or `"orthographic"`. The bridge serializes its own camera calls, but direct C++ capability calls must still obey the camera module's single-thread rule. The bridge is omitted when `script_runtime` is unavailable. The complete function reference lives in [lua_bridge.md](lua_bridge.md).

## Threading and lifetime

There is no internal locking: capability calls touch the camera map and the records (including their mutable caches) directly, so they should be routed through a single thread — the demo calls `update`/`viewState` from its frame loop. Cameras live until `destroyCamera` or module `stop()` (which clears all of them). Configuration changes only affect the defaults for *new* cameras; existing cameras keep their current settings.

## Configuration

Configurable — see [configuration.md](configuration.md) for the `[camera]` keys (mode, speeds, projection defaults, key bindings, and their validated ranges).

## Integration

Events: the module subscribes to `ConfigurationReloadedEvent` (kernel event bus) in `construct()` and unsubscribes in `stop()`. The handler re-reads the `[camera]` section when any changed key starts with `"camera."`. It publishes no events of its own. Permissions: none declared (`declaredPermissions = nullptr` in the entry table).

Consumer usage with the real API signatures (as the demo app does in `demo_character_customisation/source/main.cpp`):

```cpp
camera::CameraCapability &camera = context.acquireCapability<camera::CameraCapability>();
if (camera.abi != camera::cameraCapabilityAbi) { /* ABI mismatch: refuse */ }

camera::CameraError      error{};
camera::CameraParameters parameters{};
parameters.hasPosition   = true;
parameters.position[1]   = 2.0f;
parameters.fovYDegrees   = 55.0f;
const camera::CameraHandle handle = camera.createCamera(camera.self, &parameters, &error);

camera.setControlMode(camera.self, handle, camera::CameraControlMode::Orbit);

camera::CameraInputState input{};
input.moveForward  = 1.0f;              // clamped to [-1, 1] by the module
camera.update(camera.self, handle, &input, deltaSeconds);

camera::ViewState viewState{};
camera.viewState(camera.self, handle, framebufferWidth, framebufferHeight, &viewState);
// viewState.view / viewState.projection are column-major float[16].
```
