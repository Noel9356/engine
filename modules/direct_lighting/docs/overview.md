# direct_lighting module

direct_lighting maintains the engine's registry of direct lights (directional, point, and spot) and hands renderers a stable, priority-sorted snapshot of the currently active lights each time they ask. It also resolves a per-device light budget from a `render::DeviceProfile`, so renderers can size their uniform buffers without knowing the hardware. The module is intentionally stateless beyond the registry: it does no lighting math itself.

## Capability

The module provides the `direct_lighting` capability (ABI version constant `directLightingCapabilityAbi = 1`, see `include/engine/direct_lighting/capabilities.hpp`). Light handles are `std::uint32_t`, never reused, with `0` (`invalidDirectLight`) meaning "none". The renderer-facing per-draw limit is `maxDirectLights = 32`; the CPU-side registry has a separate configurable capacity and can retain more lights for external culling/selection. Optional `DirectLightingError*` out-parameters are written on every failing path of `createLight`/`updateLight`.

Entries: `createLight`, `updateLight`, `destroyLight`, `setEnabled`, `copyLights` (snapshot read side with a change-version out-parameter), and `resolveProfile` (device-based light budget).

## Provided / required / optional capabilities

- `module.toml`: `provides = ["direct_lighting"]`, `requires = []`.
- Entry table (bottom of `source/direct_lighting.cpp`): `providedCapabilities = {"direct_lighting"}`, `requiredCapabilities = nullptr` (count 0), `optionalCapabilities = nullptr` (count 0), `declaredPermissions = nullptr` (count 0).

The module has no capability dependencies and declares no permissions.

## Architecture

| File | Role |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capability, declared `config_keys` (`direct_lighting.max_lights` and `direct_lighting.maximum_registered_lights`). Read by kernel module discovery. |
| `config.toml` | Default value for the `[direct_lighting]` configuration section (see `configuration.md`). |
| `include/engine/direct_lighting/capabilities.hpp` | Public capability types: ABI, light handle/type/error, light description and snapshot entry, profile struct, function-pointer table. Includes `engine/render/device_profile.hpp` for the profile resolver's input type. |
| `source/direct_lighting.cpp` | The whole implementation: description validation and normalization, mutex-guarded registry and snapshot, version counter, device budget resolution, capability binding, module entry table. |

## How it works

- **Registry and snapshot.** `lights_` and `snapshot_` are vectors reserved to the configured registry budget. An `unordered_map` maps stable handles to registry indices, so handle updates and lookups stay constant-time as the registry grows. `lights_` is the authoritative registry (insertion/swap-remove order, irrelevant to consumers), `snapshot_` is the renderer-facing view rebuilt by `rebuildSnapshotLocked()`. The snapshot contains only *active* lights — `enabled` with radiance, i.e. positive intensity and at least one non-zero color channel — sorted by descending `priority`, then ascending handle for a stable order.
- **Validation and normalization.** `validDescription` requires every float to be finite, `range > 0`, `0 <= innerConeRadians <= outerConeRadians <= pi/2`, non-negative intensity and color, and a non-degenerate direction for directional/spot lights (length checked in double precision). `normalizeDescription` then normalizes that direction so consumers always see unit vectors.
- **Update strategy.** `updateLight` compares the new (normalized) description with the old one: an exact match is a no-op; a change that alters activity (enabled/intensity/color) or priority triggers a snapshot rebuild, while purely positional changes patch the light in place inside the existing snapshot. Every accepted mutation bumps the atomic `version_` counter.
- **Copy protocol.** `copyLights(output, capacity, outVersion)` copies the snapshot truncated to `capacity` and reports the current version. Called with a null/zero-capacity buffer it only reports the version, which lets a renderer detect changes cheaply every frame and copy only when the version moved. This is exactly the pattern the render module uses.
- **Device budget (`resolveProfile`).** For Vulkan devices the automatic budget is 32 (discrete GPU), 24 (integrated), 16 (virtual), 8 otherwise; without compute shaders it is capped at 16. It is then capped by the uniform-buffer budget `(maxUniformBufferRange - 176) / 64` (176 bytes of fixed block data plus 64 bytes per light; a range below 176 means zero lights). Non-Vulkan devices get the base budget of 8. A configured `max_lights` of 0 selects the automatic value; an explicit value is capped by the hardware limit.

## Threading and lifetime

One `std::mutex` guards the registry and the snapshot; all mutators and `copyLights` take it. The `version_` counter is atomic with release/acquire ordering so consumers can poll it safely (the null-buffer probe reads it without the lock). Lights live from `createLight` until `destroyLight`; the swap-remove in `destroyLight` reorders the registry but not the snapshot. Module `stop()` clears both vectors and the handle index, then bumps the version so pollers observe the change. Handles are never reused and are also refused once the 32-bit handle space would wrap.

## Configuration

Configurable — see [configuration.md](configuration.md) for the `[direct_lighting]` key (`direct_lighting.max_lights` and `direct_lighting.maximum_registered_lights`).

## Integration

The module does not touch the kernel event bus: it publishes no events and subscribes to none (no `eventBus` usage in the source). It declares no permissions (`declaredPermissions = nullptr` in the entry table).

Consumer usage with the real API signatures (as the render module does in `modules/render/source/render.cpp`):

```cpp
// tryAcquireCapability returns nullptr when the module is absent.
direct_lighting::DirectLightingCapability *lights
    = context.tryAcquireCapability<direct_lighting::DirectLightingCapability>();

if (lights != nullptr && lights->copyLights != nullptr)
{
    std::uint64_t version = 0;
    // Version probe: no buffer, no copy — cheap per-frame change detection.
    lights->copyLights(lights->self, nullptr, 0, &version);
    if (cachedVersion != version)
    {
        std::array<direct_lighting::DirectLight, direct_lighting::maxDirectLights> buffer{};
        const std::uint32_t count = lights->copyLights(lights->self, buffer.data(), buffer.size(), &version);
        // buffer[0 .. count) holds the active lights, priority-sorted.
        cachedVersion = version;
    }
}
```
