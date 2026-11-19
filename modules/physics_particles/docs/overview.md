# physics_particles module

physics_particles provides deterministic particle systems for ambient and gameplay effects: rain, snow, dust, fire, hail, ash, leaves, pollen, and insect emitters over a physics_rigid world, each holding a fixed pool of particles whose parameters derive from an integer hash of (slot, generation) so identical emitters replay identically with no random-number state. Particles integrate per fixed step under the world gravity plus a consumer-supplied wind acceleration, sway with per-slot phase, and optionally collide against the rigid world through the sphere-overlap query (rain and hail die on impact, snow settles, dust and ash drift through, fire rises and flickers, leaves flutter and settle briefly, pollen rides the wind, insects hover and wander).

## Capability

The module provides the `physics_particles` capability (ABI version constant `physicsParticlesCapabilityAbi = 3`; the header's ABI history comment records what each bump appended, the `Hail`, `Ash`, `Leaves`, `Pollen`, and `Insects` kinds arrived in ABI 2, the simulation-LOD surface (`setEmitterImportance` and the `lodSkippedSteps` counter) arrived in ABI 3, and consumers check `abi >= 3` before touching those additions) and requires the `physics_rigid` capability; the kernel constructs the rigid core first and grants the permission from the manifest's `requires` list. Emitter handles are `std::uint32_t`, start at 1, are never reused, and `0` (`invalidEmitter`) means "none". Positions and velocities are world-space meters and meters per second; `copyParticleStates` fills `ParticleState` snapshots (position, velocity, age, lifetime, size, seed, alive flag) in pool order, and the seed gives renderers a stable per-particle value for color and texture variation.

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

| Simulation LOD | `setEmitterImportance(emitter, importance)` | Writes the emitter's importance score in [0, 1]; the module resolves the fidelity band (with demotion hysteresis, see the shared ladder the `physics_scheduling` module also implements) and updates the staggered cadence from it. Non-finite scores report failure. |

**Error convention:** `createEmitter` fills the `ParticlesError` out-parameter (`InvalidDescription` for a null description, an unavailable rigid capability, non-finite fields, negative half-extents, emit rate, drag, or radius, an inverted lifetime or size range, `flicker` outside [0, 2], or a pool above `maximum_particles_per_emitter`; `CapacityExceeded` beyond `maximum_emitters`). The remaining functions report through their `bool` returns; `step` returns `false` for deltas outside (0, 0.1] and when any particle position becomes non-finite.

## Simulation model

**Deterministic spawn.** Every slot's parameters derive from an integer hash of (slot, generation): the spawn position inside the axis-aligned box around `origin` scaled by the description's `halfExtents`, the lifetime drawn from `[minimumLifetime, maximumLifetime]`, the per-axis velocity jitter `initialVelocity x (1 +- velocityJitter)`, the rendered size from `[minimumSize, maximumSize]`, the sway phase, and a per-slot jitter triple — each from its own hash stream, with the derived seed stored for consumer renderers. Emission accumulates `emitRate` fractionally per step: the pool grows while below `min(maximumParticles, maximum_particles_per_emitter)` and then recycles slots in FIFO order through a cursor, with surplus whole spawns dropped once the pool is saturated.

**Integration.** Per fixed step, each living particle ages and expires at its lifetime. The remaining ones accelerate under the world gravity (captured at creation) scaled by `gravityScale` — a negative scale makes particles rise — plus the wind acceleration, plus a horizontal sway oscillation at `swayFrequency` hertz with the slot's phase and amplitude `swayAmplitude`; velocity then damps by `exp(-airDrag x dt)` toward the wind target before positions advance. `airDrag` is the knob that separates the kinds: rain and hail barely react and keep their ballistic fall, while snow, dust, ash, leaves, and pollen carry a high drag and track the wind.

**Kinds.** `EmitterKind` selects the behavior class. Rain falls fast and dies on world contact or at the ground-plane fallback (`useGroundPlane` with `groundHeight` ends particles at the floor even without colliders). Snow settles: on contact it is projected out of the surface, its velocity is zeroed, and it shrinks toward half size while holding the contact until its lifetime ends. Dust drifts with the wind inside its volume and ignores world impact entirely. Fire rises buoyantly and flickers: the rendered size oscillates through two phase-shifted sine terms scaled by `flicker` (0 = steady, accepted range [0, 2]) and burns out by shrinking to zero over the final third of the lifetime. Hail falls like rain on a steeper effective descent (about 1.6x rain's default drop with the documented profile: gravity scale 1.0, drag 0.15, small spawn jitter, slightly larger sizes) and dies on world contact without bouncing. Ash descends slowly (gravity scale 0.08) on a long lifetime, drifts buoyantly with the wind through high drag, sways weaker than snow, shrinks to zero over the final 35% of its lifetime, and ignores world impact like dust. Leaves fall slowly (gravity scale 0.35) through strong horizontal drag, flutter on a large slow sway, and on contact reuse snow's settle mechanics for a brief configurable hold (`leaf_settle_seconds`) before expiring. Pollen is nearly weightless (gravity scale 0.02), rides the wind through the highest drag on a long lifetime, and simply fades at the end of its lifetime. Insects hover with no gravity: their acceleration is a stateless wander controller that steers the velocity toward a pseudo-noise target derived from the slot hash and a slow 0.5 hertz emitter-clock phase, with a soft pull that keeps each insect inside a configurable wander radius (`insect_wander_radius`) around its spawn point, and bursts re-anchor the wander to the burst point.

**Collision.** When `collideWithWorld` is set and the rigid capability is available, every moving particle that reacts to impact (all kinds except dust, ash, pollen, and insects) queries `collideSphere` with `particleRadius` and is projected out along the push-out normal by the overlap depth; rain and hail expire on contact, snow enters its settled state at the contact point, and leaves hold that settle briefly before expiring. Without the flag (the description default) particles fly through the world and only the ground-plane fallback applies.

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

**Determinism.** No random-number state exists anywhere: every per-slot value comes from a 64-bit integer hash mix seeded from (slot, generation) with per-stream offsets, the insect wander target is a stateless function of the slot hash and the emitter clock, slots recycle in FIFO cursor order, and bursts probe from the same cursor, so replaying the same creation and step sequence reproduces the same particles. That makes emitter behavior reproducible across runs and machines without serialization.

## Simulation LOD and collision batching

Every emitter carries an importance score the consumer writes through `setEmitterImportance`; the module maps the score onto the five fidelity bands (Cached, Animation, Procedural, Simplified, Full) with the same demotion-hysteresis ladder the `physics_scheduling` module exposes, and derives a per-band update interval from configuration (`lod_interval_*`; Full always runs every step, and a `0` interval freezes that band). With `lod_enabled` on, emitters below Full advance on a staggered cadence — the phase comes from the emitter's registration ordinal, so emitters sharing an interval spread their work evenly across steps — and one due step covers the whole interval of wall time: spawn debt, lifetime aging, sway, and collision all integrate with interval x the caller's fixed delta, so the emission rate and pacing stay wall-time exact while the per-step cost drops in proportion. Cached emitters freeze until the score rises again. Without `lod_enabled` every emitter runs Full every step, which is the default. The `lodSkippedSteps` statistic counts the skipped emitter-steps so a consumer can watch the LOD do its job.

World collision resolves through one batched sphere-overlap query per step against the rigid capability's `collideSpheres` surface instead of one call per particle: the rigid module pre-filters candidates through each body's shape bounds and returns the same deepest-overlap results the per-sphere query gives, in input order. Settled snow and leaf slots take part in the pass, so a surface that moves away releases them back into free fall. A particle whose state turns non-finite during a step retires immediately (the step still reports failure through `SimulationFailure`), so one diverging particle cannot poison its pool.

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
