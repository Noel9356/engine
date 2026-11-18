# physics_particles module

physics_particles provides deterministic particle systems for ambient and gameplay effects: rain, snow, dust, and fire emitters over a physics_rigid world, each holding a fixed pool of particles whose parameters derive from an integer hash of (slot, generation) so identical emitters replay identically with no random-number state. Particles integrate per fixed step under the world gravity plus a consumer-supplied wind acceleration, sway with per-slot phase, and optionally collide against the rigid world through the sphere-overlap query (rain dies on impact, snow settles, dust drifts through, fire rises and flickers).

## Capability

The module provides the `physics_particles` capability (ABI version constant `physicsParticlesCapabilityAbi = 1`, see `include/engine/physics_particles/capabilities.hpp`) and requires the `physics_rigid` capability; the kernel constructs the rigid core first and grants the permission from the manifest's `requires` list. Emitter handles are `std::uint32_t`, start at 1, are never reused, and `0` (`invalidEmitter`) means "none". Positions and velocities are world-space meters and meters per second; `copyParticleStates` fills `ParticleState` snapshots (position, velocity, age, lifetime, size, seed, alive flag) in pool order, and the seed gives renderers a stable per-particle value for color and texture variation.

### Frame-rate independence contract

Like the rigid module, an emitter advances in whole fixed steps only. `step(emitter, fixedDeltaSeconds)` performs exactly one step (values in (0, 0.1] are accepted) and must be called with a constant delta; `simulate(emitter, elapsedSeconds)` accumulates wall time and runs whole fixed steps at the configured `fixed_rate_hz`, dropping backlog beyond `maximum_steps_per_simulate`. Spawn debt accumulates fractionally per step, so the emission cadence is frame-rate independent as well.

| Group | Functions | Purpose |
| --- | --- | --- |
| Lifetime | `createEmitter(world, description, outError)`, `destroyEmitter(emitter)` | Validates the description, adopts the target world's gravity, and registers the emitter with its pool reserved; destruction frees the pool. |
| Retuning | `setEmitterDescription(emitter, description)` | Re-applies a validated description in place; the pool resizes (a shrink truncates the tail slots and resets the recycle cursor) and live particles keep their state. |
| Wind | `setWindAcceleration(emitter, wind)` | Uniform wind acceleration (meters per second squared) blended by the description's `airDrag`; consumers can bridge a wind field's local sample per step. |
| Burst | `burst(emitter, position, count, speed)` | One-shot isotropic burst from a point (explosion debris, dust kicks, splash droplets); expired slots are reused first, then the pool grows or recycles through the FIFO cursor. |
| Inspection | `copyParticleStates`, `particlesStatistics` | Per-particle snapshots up to capacity (with the filled count and the total pool size) and the module-wide emitter, spawned, expired, and alive counters. |
| Stepping | `step(emitter, fixedDeltaSeconds)`, `simulate(emitter, elapsedSeconds)` | The fixed-step primitive and the accumulator convenience; see the contract above. |

**Error convention:** `createEmitter` fills the `ParticlesError` out-parameter (`InvalidDescription` for a null description, an unavailable rigid capability, non-finite fields, negative half-extents, emit rate, drag, or radius, an inverted lifetime or size range, `flicker` outside [0, 2], or a pool above `maximum_particles_per_emitter`; `CapacityExceeded` beyond `maximum_emitters`). The remaining functions report through their `bool` returns; `step` returns `false` for deltas outside (0, 0.1] and when any particle position becomes non-finite.

## Simulation model

**Deterministic spawn.** Every slot's parameters derive from an integer hash of (slot, generation): the spawn position inside the axis-aligned box around `origin` scaled by the description's `halfExtents`, the lifetime drawn from `[minimumLifetime, maximumLifetime]`, the per-axis velocity jitter `initialVelocity x (1 +- velocityJitter)`, the rendered size from `[minimumSize, maximumSize]`, the sway phase, and a per-slot jitter triple — each from its own hash stream, with the derived seed stored for consumer renderers. Emission accumulates `emitRate` fractionally per step: the pool grows while below `min(maximumParticles, maximum_particles_per_emitter)` and then recycles slots in FIFO order through a cursor, with surplus whole spawns dropped once the pool is saturated.

**Integration.** Per fixed step, each living particle ages and expires at its lifetime. The remaining ones accelerate under the world gravity (captured at creation) scaled by `gravityScale` — a negative scale makes particles rise — plus the wind acceleration, plus a horizontal sway oscillation at `swayFrequency` hertz with the slot's phase and amplitude `swayAmplitude`; velocity then damps by `exp(-airDrag x dt)` toward the wind target before positions advance. `airDrag` is the knob that separates the kinds: rain barely reacts and keeps its ballistic fall, while snow and dust carry a high drag and track the wind.

**Kinds.** `EmitterKind` selects the behavior class. Rain falls fast and dies on world contact or at the ground-plane fallback (`useGroundPlane` with `groundHeight` ends particles at the floor even without colliders). Snow settles: on contact it is projected out of the surface, its velocity is zeroed, and it shrinks toward half size while holding the contact until its lifetime ends. Dust drifts with the wind inside its volume and ignores world impact entirely. Fire rises buoyantly and flickers: the rendered size oscillates through two phase-shifted sine terms scaled by `flicker` (0 = steady, accepted range [0, 2]) and burns out by shrinking to zero over the final third of the lifetime.

**Collision.** When `collideWithWorld` is set and the rigid capability is available, every moving non-dust particle queries `collideSphere` with `particleRadius` and is projected out along the push-out normal by the overlap depth; rain expires on contact and snow enters its settled state at the contact point. Without the flag (the description default) particles fly through the world and only the ground-plane fallback applies.

## Provided / required / optional capabilities

From `module.toml` and the entry table at the bottom of `source/physics_particles.cpp` (they agree):

- **Provided:** `physics_particles`.
- **Required:** `physics_rigid`.
- **Optional:** none.
- **Declared permissions:** none.

## Architecture

| File | Purpose |
| --- | --- |
| `module.toml` | Module descriptor: name, version, provided capability, required capability, declared `config_keys`. |
| `config.toml` | Default values for the `[physics_particles]` configuration section (see `configuration.md`). |
| `CMakeLists.txt` | Builds `engine_module_physics_particles` (a `.so` under `runtime/modules/physics_particles/`) and copies `module.toml` next to it. |
| `include/engine/physics_particles/capabilities.hpp` | Public capability types: ABI constant, emitter handle, error and kind enums, emitter description, particle snapshot and statistics, capability struct. |
| `source/physics_particles.cpp` | Module class (lifecycle, configuration, deterministic hash spawning, per-kind integration, world collision, bursts, capability binding), the emitter record and slot state, and entry table. |
| `include/engine/physics_rigid/math.hpp` | The rigid module's public math header included by the module source alongside the rigid capability. |

## How it works

**Creation.** `createEmitter` validates finiteness and field relationships (half-extents non-negative, `emitRate` non-negative, `minimumLifetime` positive with `maximumLifetime` at or above it, `airDrag` and `particleRadius` non-negative, `minimumSize` positive with `maximumSize` at or above it, `flicker` in [0, 2], and `maximumParticles` within the configured `maximum_particles_per_emitter`), checks the emitter budget, reads the target world's gravity through `copyWorldGravity` (falling back to (0, -9.81, 0) if the world does not answer), and reserves the pool capacity before the handle is published.

**Stepping.** `step` runs the spawn, integrate, and collision passes described above, validates the delta ((0, 0.1], finite), and reports failure when any particle position becomes non-finite. `simulate` mirrors the rigid module's accumulator: whole steps at the configured rate, budget clamp, and zero-budget reset when the step cap trips so a stalled consumer never accumulates debt. `particlesStatistics` walks every pool to report the module-wide spawned, expired, and alive totals alongside the emitter count.

**Determinism.** No random-number state exists anywhere: every per-slot value comes from a 64-bit integer hash mix seeded from (slot, generation) with per-stream offsets, slots recycle in FIFO cursor order, and bursts probe from the same cursor, so replaying the same creation and step sequence reproduces the same particles. That makes emitter behavior reproducible across runs and machines without serialization.

## Threading and lifetime

The module has no locks; serialize calls from one thread. The slot pool lives in the emitter record and dies with it: `destroyEmitter` releases the pool and updates the module statistics; `stop()` clears every emitter and resets the handle counters and statistics. Handles are monotonic and never reused. The `collideSphere` queries are read-only against the rigid world, so emitter steps may run concurrently with rigid `step` calls of a different world only if the consumer guarantees the worlds are distinct; in the common single-world case the consumer steps the rigid world and every emitter sequentially within one fixed update.

A consumer creates a snow emitter that settles on the rigid world and applies wind:

```cpp
engine::physics_particles::ParticlesCapability &particlesCapability =
    context.acquireCapability<engine::physics_particles::ParticlesCapability>();
if (particlesCapability.abi != engine::physics_particles::physicsParticlesCapabilityAbi)
{
    return;
}
engine::physics_particles::EmitterDescription description{};
description.kind = engine::physics_particles::EmitterKind::Snow;
description.collideWithWorld = true; // snow settles on the rigid world
const engine::physics_particles::EmitterHandle emitter =
    particlesCapability.createEmitter(particlesCapability.self, rigidWorld, &description, nullptr);

const double wind[3] = {1.5, 0.0, 0.4};
particlesCapability.setWindAcceleration(particlesCapability.self, emitter, wind);
// per fixed step:
particlesCapability.step(particlesCapability.self, emitter, fixedDeltaSeconds);
```
