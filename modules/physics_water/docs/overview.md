# physics_water module

physics_water provides ocean surfaces and local water interaction over a physics_rigid world: an analytic deep-water wave sum with per-wave dispersion sampled in closed form for height and normal, localized ripple rings that propagate and decay across the surface (impacts, splashes, wakes), and a buoyancy pass that registers rigid bodies and applies submersion-sampled buoyancy forces, quadratic water drag, and angular damping through the rigid module's apply entry points.

## Capability

The module provides the `physics_water` capability (ABI version constant `physicsWaterCapabilityAbi = 1`, see `include/engine/physics_water/capabilities.hpp`) and requires the `physics_rigid` capability; the kernel constructs the rigid core first and grants the permission from the manifest's `requires` list. Ocean handles are `std::uint32_t`, start at 1, are never reused, and `0` (`invalidOcean`) means "none". Positions are world-space meters and time is the consumer's fixed-step clock in seconds; an ocean carries up to `maximumWavesPerOcean` (= 8) wave components, and each ocean's ripple slots and registered-body set are bounded by configuration.

### Time and stepping contract

The wave surface is a pure function of the consumer's clock: `sampleSurface` evaluates the wave sum plus every active ripple at the `timeSeconds` argument, so rendering and physics can sample any time deterministically and repeat the same surface. Ripples advance in whole fixed steps only: `stepRipples(ocean, fixedDeltaSeconds)` expands and decays the ring field (values in (0, 0.1] are accepted). The module has no accumulator of its own — the consumer calls `applyBuoyancy` once per fixed update, immediately before stepping the rigid world, which is where every water force lands.

| Group | Functions | Purpose |
| --- | --- | --- |
| Lifetime | `createOcean(world, description, outError)`, `destroyOcean(ocean)`, `setOceanDescription(ocean, description)` | Creates an ocean over the given rigid world from a mean water level and a wave set; re-tunes the wave set and level later (existing ripples survive). |
| Surface queries | `sampleSurface(ocean, x, z, timeSeconds, outHeight, outNormal)` | Writes the surface height at a world (x, z) and the unit surface normal, computed by central differences of the wave sum plus every active ripple. |
| Ripples | `addRipple(ocean, center, strength)`, `stepRipples(ocean, fixedDeltaSeconds)` | Plants an expanding, decaying ring at a surface point (the oldest ripple is recycled once the configured budget is full) and advances the ring field one fixed step. |
| Buoyancy registration | `registerBody(ocean, body, outError)`, `unregisterBody(ocean, body)` | Enrolls a dynamic body for the buoyancy pass (duplicate registration is an accepted no-op); a registration goes dormant when the body dies in the rigid world and is freed at the next pass. |
| Submersion | `querySubmersion(ocean, body, timeSeconds, outFraction, outDepth)` | Writes the estimated submerged volume fraction in [0, 1] and the deepest sample point's depth below the surface (negative when fully above water). |
| Forces | `applyBuoyancy(ocean, timeSeconds, fixedDeltaSeconds)` | The per-fixed-update pass: buoyancy at the submerged centroid, quadratic water drag, and angular damping for every registered awake body, through the rigid module's apply entry points. |
| Statistics | `waterStatistics(outStatistics)` | Ocean count, registered body count, active ripple count, and the bodies the last buoyancy pass touched. |

**Error convention:** `createOcean` and `registerBody` fill the `WaterError` out-parameter (`InvalidDescription` for null pointers, non-finite values, wrong wave counts, or out-of-range wave parameters, `CapacityExceeded` beyond the ocean or registered-body budgets, `InvalidHandle` for unknown oceans or bodies the rigid world does not know). The boolean entry points (`setOceanDescription`, `sampleSurface`, `addRipple`, `stepRipples`, `unregisterBody`, `querySubmersion`, `applyBuoyancy`) report unknown handles and invalid inputs through their `false` return; `SimulationFailure` is the enum value reserved for a force application rejected by the rigid world.

## Simulation model

**Wave sum.** Each ocean is a mean `waterLevel` plus up to eight traveling wave components. A wave's direction normalizes on entry, its wave number is `k = 2 x pi / wavelength`, and its angular frequency follows the deep-water dispersion relation `omega = sqrt(g x k)` against a fixed reference gravity of 9.81, so longer wavelengths travel faster. The height at a world point and time is `waterLevel + sum(amplitude x sin(k x dot(direction, point) - omega x time))`; wave descriptions carry direction, wavelength, amplitude, and steepness, and the evaluated height field is this sinusoidal sum. `setOceanDescription` re-prepares the wave set in place while ripples continue undisturbed.

**Ripples.** `addRipple` plants a ring source (the center's x and z, an initial amplitude `strength`) into the ocean's fixed slot array, reusing the first inactive slot or recycling the oldest when the budget is full. Each active ripple contributes a cosine ring: the front expands at the configured `ripple_speed`, crests repeat every `ripple_wavelength` meters, and the amplitude carries an exponential decay `exp(-age x ripple_decay)` plus a radial attenuation `exp(-radius x ripple_attenuation)`, windowed to zero at the band edge. `stepRipples` ages every active ripple one fixed step and retires it once its envelope has decayed below a thousandth of its initial amplitude.

**Surface queries.** `sampleSurface` evaluates the full surface (waves plus ripples) and derives the normal by central differences (0.05 m epsilon) of the height field, normalized and pointing out of the water, so consumers render the exact surface the buoyancy pass samples.

**Buoyancy.** Registered bodies are sampled per shape: spheres at the center plus six axis points, boxes at the eight rotated corners (0.7 x half extents) plus the center, capsules at five stations along the rotated axis plus four lateral offsets; each sample compares against the local surface height. The submerged fraction drives three applications per awake body, all through the rigid module: a gravity-opposing buoyancy force `water_density x displacedVolume x gravity` at the submerged centroid (displacedVolume is the closed-form shape volume times the fraction times `buoyancy_scale`; the gravity magnitude is read once from the world), a quadratic drag `0.5 x water_density x linear_drag_coefficient x dragArea x fraction x speed^2` opposite the velocity (dragArea is the shape volume's two-thirds power), and an angular damping torque `angular_drag_coefficient x fraction x mass x angularSpeed x 0.01` opposing the spin. Bodies entirely above water receive nothing; shapes without sample points (planes) are never buoyed.

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
| `include/engine/physics_water/capabilities.hpp` | Public capability types: ABI constant, ocean handle, wave and ocean descriptions, statistics, capability struct. |
| `source/physics_water.cpp` | Module class (lifecycle, configuration, wave preparation, ripple field, submersion sampling, buoyancy pass, capability binding) and entry table. |
| `include/engine/physics_rigid/math.hpp` | The rigid module's public math vocabulary used for rotated shape sample points. |

## How it works

**Creation.** `createOcean` validates the wave set (count within `maximumWavesPerOcean`, finite and positive wavelengths, non-negative amplitudes, steepness below 1), copies the description, prepares the wave list (normalized directions, wave numbers, dispersion frequencies), allocates the ripple slot array from the configured budget, and captures the world's gravity magnitude through `copyWorldGravity` for the buoyancy pass.

**Buoyancy pass.** `applyBuoyancy` walks the registration list: bodies the rigid world no longer knows are unregistered on the spot, sleeping bodies are skipped, and each awake body receives the three applications described above. The registration list shrinks automatically, so a consumer never needs to unregister a destroyed body, and `buoyancyApplications` reports how many bodies the last pass touched.

**Statistics.** `waterStatistics` copies the `WaterStatistics` struct: the live `oceanCount` and `registeredBodies` (maintained on create, destroy, register, and unregister), the `activeRipples` count from the last `stepRipples` call, and the `buoyancyApplications` count of the last pass.

## Threading and lifetime

The module has no locks; serialize calls from one thread. Ocean records (wave lists, ripple slot arrays, registration lists) live in the module and die with it: `destroyOcean` erases the record (swap-with-back keeps the vector dense); `stop()` clears every ocean, resets the handle counter, and zeroes the statistics. Handles are monotonic and never reused. The buoyancy pass reads body state and description copies from the rigid world and writes forces through its apply entry points, so it belongs in the same fixed update as the rigid world's own step, sequenced before it.

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
