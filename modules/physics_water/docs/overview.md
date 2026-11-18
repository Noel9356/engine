# physics_water module

physics_water provides ocean surfaces and local water interaction over a physics_rigid world: a Gerstner wave sum with per-wave steepness (deep-water dispersion, horizontal crest displacement, surface height and normal recovered through a fixed inverse iteration), a uniform water current, localized ripple rings that propagate and decay across the surface (impacts, splashes, automatic wakes from fast registered bodies), and a buoyancy pass that registers rigid bodies and applies submersion-sampled buoyancy forces, current-relative quadratic water drag, and angular damping through the rigid module's apply entry points.

## Capability

The module provides the `physics_water` capability (ABI version constant `physicsWaterCapabilityAbi = 2`, see `include/engine/physics_water/capabilities.hpp`) and requires the `physics_rigid` capability; the kernel constructs the rigid core first and grants the permission from the manifest's `requires` list. Ocean handles are `std::uint32_t`, start at 1, are never reused, and `0` (`invalidOcean`) means "none". Positions are world-space meters and time is the consumer's fixed-step clock in seconds; an ocean carries up to `maximumWavesPerOcean` (= 8) wave components, and each ocean's ripple slots and registered-body set are bounded by configuration. Consumers check `abi >= 2` before touching the additions (wave steepness, `sampleSurfaceDisplaced`, wake emission, `OceanDescription.current`, `setCurrentVelocity`); everything from ABI 1 keeps its slot and meaning.

### Time and stepping contract

The wave surface is a pure function of the consumer's clock: `sampleSurface` evaluates the Gerstner-displaced wave sum plus every active ripple at the `timeSeconds` argument, so rendering and physics can sample any time deterministically and repeat the same surface. Ripples advance in whole fixed steps only: `stepRipples(ocean, fixedDeltaSeconds)` expands and decays the ring field and runs the automatic wake emission for registered bodies, whose travelled-distance accumulators likewise advance only inside these fixed steps. The module has no time accumulator of its own — the consumer calls `applyBuoyancy` once per fixed update, immediately before stepping the rigid world, which is where every water force lands.

| Group | Functions | Purpose |
| --- | --- | --- |
| Lifetime | `createOcean(world, description, outError)`, `destroyOcean(ocean)`, `setOceanDescription(ocean, description)` | Creates an ocean over the given rigid world from a mean water level, a wave set, and a uniform current; re-tunes the wave set, level, and current later (existing ripples survive). |
| Surface queries | `sampleSurface(ocean, x, z, timeSeconds, outHeight, outNormal)`, `sampleSurfaceDisplaced(ocean, position, timeSeconds, outDisplacement)` | Writes the surface height at a world (x, z) and the unit surface normal, computed by central differences of the displaced wave sum plus every active ripple; the displaced variant writes the full surface offset (horizontal Gerstner shift plus height over the seed) a renderer applies to a grid vertex. |
| Ripples | `addRipple(ocean, center, strength)`, `stepRipples(ocean, fixedDeltaSeconds)` | Plants an expanding, decaying ring at a surface point (the oldest ripple is recycled once the configured budget is full) and advances the ring field one fixed step, emitting wake ripples for fast registered bodies. |
| Currents | `setCurrentVelocity(ocean, current)` | Re-tunes the ocean's uniform horizontal current at runtime; the next buoyancy pass drags against the body velocity relative to it. |
| Buoyancy registration | `registerBody(ocean, body, outError)`, `unregisterBody(ocean, body)` | Enrolls a dynamic body for the buoyancy pass (duplicate registration is an accepted no-op); a registration goes dormant when the body dies in the rigid world and is freed at the next pass. Registrations are kept in ascending body-handle order and also feed the automatic wake emission. |
| Submersion | `querySubmersion(ocean, body, timeSeconds, outFraction, outDepth)` | Writes the estimated submerged volume fraction in [0, 1] and the deepest sample point's depth below the surface (negative when fully above water). |
| Forces | `applyBuoyancy(ocean, timeSeconds, fixedDeltaSeconds)` | The per-fixed-update pass: buoyancy at the submerged centroid, quadratic water drag against the ocean's uniform current, and angular damping for every registered awake body, through the rigid module's apply entry points. |
| Statistics | `waterStatistics(outStatistics)` | Ocean count, registered body count, active ripple count, and the bodies the last buoyancy pass touched. |

**Error convention:** `createOcean` and `registerBody` fill the `WaterError` out-parameter (`InvalidDescription` for null pointers, non-finite values, wrong wave counts, or non-finite wave or current values, `CapacityExceeded` beyond the ocean or registered-body budgets, `InvalidHandle` for unknown oceans or bodies the rigid world does not know). The boolean entry points (`setOceanDescription`, `sampleSurface`, `sampleSurfaceDisplaced`, `addRipple`, `stepRipples`, `unregisterBody`, `querySubmersion`, `applyBuoyancy`, `setCurrentVelocity`) report unknown handles and invalid inputs through their `false` return; `SimulationFailure` is the enum value reserved for a force application rejected by the rigid world.

## Simulation model

**Wave sum.** Each ocean is a mean `waterLevel` plus up to eight traveling Gerstner wave components. A wave's direction normalizes on entry, its wave number is `k = 2 x pi / wavelength`, and its angular frequency follows the deep-water dispersion relation `omega = sqrt(g x k)` against a fixed reference gravity of 9.81, so longer wavelengths travel faster. Each component carries a steepness in [0, 1] (clamped on entry; 0 keeps it purely sinusoidal) that sharpens crests horizontally: with the GPU Gems steepness factor `Q_i = steepness_i / (k_i x amplitude_i x waveCount)`, the displaced surface for a horizontal seed point p0 is `P(p0) = p0 + sum(Q_i x amplitude_i x D_i x cos(k_i x dot(D_i, p0) - omega_i x time))` horizontally along the wave direction `D_i` and `h(p0) = waterLevel + sum(amplitude_i x sin(k_i x dot(D_i, p0) - omega_i x time))` vertically. The per-wave slope factors `Q_i x k_i x amplitude_i = steepness_i / waveCount` sum to at most 1 across the set, so crests sharpen without looping; zero-amplitude components contribute no displacement. `setOceanDescription` re-prepares the wave set in place while ripples continue undisturbed.

**Ripples.** `addRipple` plants a ring source (the center's x and z, an initial amplitude `strength`) into the ocean's fixed slot array, reusing the first inactive slot or recycling the oldest when the budget is full. Each active ripple contributes a cosine ring: the front expands at the configured `ripple_speed`, crests repeat every `ripple_wavelength` meters, and the amplitude carries an exponential decay `exp(-age x ripple_decay)` plus a radial attenuation `exp(-radius x ripple_attenuation)`, windowed to zero at the band edge. `stepRipples` ages every active ripple one fixed step and retires it once its envelope has decayed below a thousandth of its initial amplitude.

**Surface queries.** `sampleSurface` evaluates the full surface (Gerstner-displaced waves plus ripples) and derives the normal by central differences (0.05 m epsilon) of the displaced height field, normalized and pointing out of the water, so consumers render the exact surface the buoyancy pass samples. `sampleSurfaceDisplaced` writes the same inverse-mapped surface point as an offset relative to the horizontal seed, with the ripple field sampled at the seed exactly as `sampleSurface` adds it.

**Buoyancy.** Registered bodies are sampled per shape: spheres at the center plus six axis points, boxes at the eight rotated corners (0.7 x half extents) plus the center, capsules at five stations along the rotated axis plus four lateral offsets; each sample compares against the local surface height. The submerged fraction drives three applications per awake body, all through the rigid module: a gravity-opposing buoyancy force `water_density x displacedVolume x gravity` at the submerged centroid (displacedVolume is the closed-form shape volume times the fraction times `buoyancy_scale`; the gravity magnitude is read once from the world), a quadratic drag `0.5 x water_density x linear_drag_coefficient x dragArea x fraction x speed^2` opposite the velocity relative to the ocean's uniform current (dragArea is the shape volume's two-thirds power), and an angular damping torque `angular_drag_coefficient x fraction x mass x angularSpeed x 0.01` opposing the spin. Bodies entirely above water receive nothing; shapes without sample points (planes) are never buoyed.

## Provided / required / optional capabilities

From `module.toml` and the entry table at the bottom of `source/physics_water.cpp` (they agree):

- **Provided:** `physics_water`.
- **Required:** `physics_rigid`.
- **Optional:** none.
- **Declared permissions:** none.

## Architecture

| File | Purpose |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capability, required capability, declared `config_keys`. |
| `config.toml` | Default values for the `[physics_water]` configuration section (see `configuration.md`). |
| `CMakeLists.txt` | Builds `engine_module_physics_water` (a `.so` under `runtime/modules/physics_water/`) and copies `module.toml` next to it. |
| `include/engine/physics_water/capabilities.hpp` | Public capability types: ABI constant and history, ocean handle, wave and ocean descriptions, statistics, capability struct. |
| `source/physics_water.cpp` | Module class (lifecycle, configuration, wave preparation, ripple field, wake emission, submersion sampling, buoyancy pass, capability binding) and entry table. |
| `include/engine/physics_rigid/math.hpp` | The rigid module's public math vocabulary used for rotated shape sample points. |

## How it works

**Creation.** `createOcean` validates the wave set (count within `maximumWavesPerOcean`, finite and positive wavelengths, non-negative amplitudes, finite steepness and current values), copies the description, prepares the wave list (normalized directions, wave numbers, dispersion frequencies, steepness-clamped horizontal displacement amplitudes), derives the effective current with its y component forced to zero, allocates the ripple slot array from the configured budget, and captures the world's gravity magnitude through `copyWorldGravity` for the buoyancy pass.

**Gerstner displacement.** The horizontal crest displacement moves sample points, so the height at a fixed world (x, z) is recovered by inverse iteration: the seed starts at (x, z), the displaced position is evaluated, and the seed is corrected by `seed += (x, z) - P.xz` a fixed four times before a final evaluation returns the height that surface queries and buoyancy sampling share. Each correction contracts the seed error by at most the total steepness factor `sum(Q_i x k_i x amplitude_i) <= 1`, so oceans below the looping limit converge monotonically; purely sinusoidal sets (all steepness 0) resolve exactly on the first evaluation, and sets near steepness 1 leave a proportionally small residual that stays deterministic because the iteration count is fixed.

**Wakes.** Inside `stepRipples` (before the ring field ages), each registered body — walked in ascending body-handle order — accumulates the horizontal distance travelled since its last wake. When the accumulated distance crosses `wake_spacing_meters` and the body's horizontal speed is at least `wake_speed_threshold`, a ripple is planted at the body's position with amplitude `speed x wake_strength_scale` (meters) and the accumulator resets, discarding any excess distance. Fully submerged bodies emit nothing: the same shape-sample scheme `querySubmersion` uses must report a submerged fraction below 0.95, evaluated against the wave surface at the ocean's last buoyancy clock reading (0 before the first `applyBuoyancy`), so sunken hulls and underwater movers stay silent while anything riding at the surface keeps trailing rings. `wake_strength_scale = 0` disables the emission entirely.

**Currents.** Every ocean carries a uniform horizontal current (`OceanDescription.current`, meters per second, y component forced to 0). `applyBuoyancy` computes its quadratic drag against `bodyVelocity - current`, so a body adrift in the current feels no horizontal drag while the water moves, and `setCurrentVelocity` re-tunes the field at runtime without touching the wave set or registrations.

**Buoyancy pass.** `applyBuoyancy` walks the registration list: bodies the rigid world no longer knows are unregistered on the spot, sleeping bodies are skipped, and each awake body receives the three applications described above. The registration list shrinks automatically, so a consumer never needs to unregister a destroyed body, and `buoyancyApplications` reports how many bodies the last pass touched.

**Statistics.** `waterStatistics` copies the `WaterStatistics` struct: the live `oceanCount` and `registeredBodies` (maintained on create, destroy, register, and unregister), the `activeRipples` count from the last `stepRipples` call, and the `buoyancyApplications` count of the last pass.

## Threading and lifetime

The module has no locks; serialize calls from one thread. Ocean records (wave lists, ripple slot arrays, registration lists, wake bookkeeping) live in the module and die with it: `destroyOcean` erases the record (swap-with-back keeps the vector dense); `stop()` clears every ocean, resets the handle counter, and zeroes the statistics. Handles are monotonic and never reused. The buoyancy pass reads body state and description copies from the rigid world and writes forces through its apply entry points, so it belongs in the same fixed update as the rigid world's own step, sequenced before it.

A consumer builds a two-wave ocean, floats a hull on it, and splashes a ripple at an impact point:

```cpp
engine::physics_water::WaterCapability &waterCapability =
    context.acquireCapability<engine::physics_water::WaterCapability>();
if (waterCapability.abi != engine::physics_water::physicsWaterCapabilityAbi)
{
    return;
}
engine::physics_water::OceanDescription oceanDescription{};
oceanDescription.waveCount = 2;
oceanDescription.waves[1].wavelength = 11.0;
oceanDescription.waves[1].amplitude = 0.2;
oceanDescription.waves[1].directionX = 0.0;
oceanDescription.waves[1].directionZ = 1.0;
const engine::physics_water::OceanHandle ocean =
    waterCapability.createOcean(waterCapability.self, rigidWorld, &oceanDescription, nullptr);
waterCapability.registerBody(waterCapability.self, ocean, hullBody, nullptr);
const double splash[3] = {impactPoint.x, oceanDescription.waterLevel, impactPoint.z};
waterCapability.addRipple(waterCapability.self, ocean, splash, 0.3);
// per fixed update:
waterCapability.stepRipples(waterCapability.self, ocean, fixedDeltaSeconds);
waterCapability.applyBuoyancy(waterCapability.self, ocean, simulationTimeSeconds, fixedDeltaSeconds);
```
