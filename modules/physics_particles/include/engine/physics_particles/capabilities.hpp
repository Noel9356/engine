// capabilities.hpp - public C capability surface of the physics_particles
// module.
//
// Deterministic particle systems for ambient and gameplay effects: rain,
// snow, dust, fire, hail, ash, leaves, pollen, and insect emitters over a
// physics_rigid world. Every emitter keeps a fixed pool of particles whose
// parameters derive from an integer hash of (slot, generation), so
// identical emitters produce identical results without random-number
// state. Particles integrate per fixed step, respond to a uniform wind
// acceleration (consumers can bridge a wind field's local sample), and
// optionally collide against the rigid world through the batched
// sphere-overlap query (rain and hail die on impact, snow settles, leaves
// settle briefly before expiring, ash/pollen/insects pass through).
// Simulation LOD: every emitter carries an importance score; emitters below
// the Full fidelity band update on a staggered cadence whose due steps cover
// the whole interval of wall time, and Cached emitters freeze. Consumers
// acquire it via ParticipantContext::acquireCapability
// <ParticlesCapability>(); the module declares physics_rigid as a required
// capability.

#pragma once

#include <engine/physics_rigid/capabilities.hpp>

#include <cstdint>

namespace engine::physics_particles
{

// Increment when any public capability structure or function-table layout
// changes; consumers verify it after acquisition.
//
// ABI history:
//   1 - initial surface (Rain/Snow/Dust/Fire kinds, the emitter
//       description, bursts, wind, statistics, and the step/simulate
//       entry points).
//   2 - appended EmitterKind::Hail, Ash, Leaves, Pollen, and Insects with
//       their deterministic behaviors. Everything from ABI 1 keeps its
//       slot and meaning; consumers check `abi >= 2` before touching the
//       additions.
//   3 - appended the simulation-LOD surface: the setEmitterImportance entry
//       point (an importance score in [0, 1] that drives the emitter's
//       fidelity band) and the ParticlesStatistics.lodSkippedSteps counter.
//       Everything from ABI 2 keeps its slot and meaning; consumers check
//       `abi >= 3` before touching the additions.
inline constexpr std::uint32_t physicsParticlesCapabilityAbi = 3u;

using EmitterHandle = std::uint32_t;

inline constexpr EmitterHandle invalidEmitter = 0;

enum class ParticlesError : std::uint32_t
{
    None = 0,

    InvalidHandle,      // unknown emitter handle or particle index
    InvalidDescription, // null pointers, non-finite values, negative extents
    CapacityExceeded,   // configured emitter or particle budget exhausted
    SimulationFailure   // non-finite state entered during a step; the diverging particles are retired
};

// Particle behavior classes. Rain falls fast and dies at the ground; snow
// falls slowly, sways, and settles on contact; dust drifts with the wind
// inside its volume and respawns; fire rises buoyantly and flickers.
// Hail falls fast like rain (about 1.6x rain's effective descent) and dies
// on impact without bouncing.
// Ash descends slowly and drifts buoyantly with the wind, shrinks toward
// the end of its life, and ignores impacts.
// Leaves flutter on a strong slow sway with heavy horizontal drag and
// settle briefly on impact before expiring.
// Pollen rides the wind nearly weightless on a long drift and fades out at
// the end of its lifetime without impact death.
// Insects hover with no gravity and wander deterministically around their
// spawn point, ignoring impacts.
enum class EmitterKind : std::uint32_t
{
    Rain = 0,
    Snow,
    Dust,
    Fire,
    Hail,   // appended in ABI 2: fast ballistic fall, dies on impact like rain
    Ash,    // appended in ABI 2: slow buoyant drift, shrinks late, passes through
    Leaves, // appended in ABI 2: slow heavy flutter, settles briefly on impact
    Pollen, // appended in ABI 2: near-weightless wind drift, fades at lifetime
    Insects // appended in ABI 2: no gravity, deterministic hover-and-wander
};

struct EmitterDescription
{
    EmitterKind kind = EmitterKind::Rain;

    // Spawn region: particles spawn at random points inside the axis-aligned
    // box around `origin` (derived per slot from the deterministic hash).
    double origin[3]          = {0.0, 8.0, 0.0};
    double halfExtents[3]     = {8.0, 0.5, 8.0};

    // Particles per second; the pool fills to maximumParticles and then
    // recycles slots FIFO (the oldest slot respawns regardless of whether
    // its particle is still alive).
    double emitRate = 400.0;

    std::uint32_t maximumParticles = 2000;

    // Lifetime range in seconds; the per-slot value derives from the hash.
    double minimumLifetime = 1.5;
    double maximumLifetime = 2.5;

    // Base initial velocity (meters per second) and per-slot jitter fraction
    // (derived from the hash, applied to each axis).
    double initialVelocity[3] = {0.0, -9.0, 0.0};
    double velocityJitter     = 0.1;

    // Gravity scale against the world gravity (rain and hail 1.0, leaves
    // 0.35, ash 0.08, pollen 0.02, fire -0.15 for buoyant rise; a negative
    // scale makes particles rise, and the insect wander controller replaces
    // gravity entirely).
    double gravityScale = 1.0;

    // Air drag: exponential velocity damping per second toward the wind
    // acceleration target (snow, dust, ash, leaves, and pollen carry a
    // high drag so they track the wind; rain and hail barely react, and
    // insects blend their wander target at a moderate drag).
    double airDrag = 0.5;

    // Sway amplitude (meters per second of horizontal drift oscillation)
    // and sway frequency (hertz), phase-derived from the slot hash. Snow,
    // dust, and ash use visible sway, leaves flutter hard on a slow phase;
    // rain, hail, and fire keep it small.
    double swayAmplitude     = 0.8;
    double swayFrequency     = 1.5;

    // Particle radius for world collision and consumer rendering.
    double particleRadius = 0.05;

    // Size range for the consumer renderer (visual units, usually meters).
    double minimumSize = 0.05;
    double maximumSize = 0.1;

    // Fire flicker: how strongly the rendered size oscillates over the
    // particle lifetime (0 = steady).
    double flicker = 0.5;

    // World collision through the rigid capability: rain and hail die on
    // impact, snow settles on impact (velocity zeroed, slides with friction 0.2,
    // resumes when the surface moves away), leaves settle briefly and then
    // expire, ash/pollen/insects ignore impact like dust.
    bool collideWithWorld = false;

    // Ground plane fallback: particles below this height die when they have
    // no world collision (rain and hail end at the floor even without colliders).
    bool   useGroundPlane     = false;
    double groundHeight       = 0.0;
};

// One particle snapshot for consumer renderers.
struct ParticleState
{
    double position[3]  = {0.0, 0.0, 0.0};
    double velocity[3]  = {0.0, 0.0, 0.0};
    double age          = 0.0;
    double lifetime     = 1.0;
    double size         = 0.1;
    std::uint32_t seed  = 0;
    bool alive          = false;
};

struct ParticlesStatistics
{
    std::uint32_t emitterCount   = 0;
    std::uint64_t spawnedTotal   = 0;
    std::uint64_t expiredTotal   = 0;
    std::uint32_t aliveParticles = 0;

    // Fixed steps emitters skipped through simulation LOD: emitters below
    // the Full fidelity band update every Nth step, and this counter tracks
    // the skipped emitter-steps across the module.
    std::uint64_t lodSkippedSteps = 0;
};

// The "physics_particles" capability. Calls are main-thread only; the
// module takes no internal locks. Positions and velocities are world-space
// meters and meters per second; stepping happens in whole fixed steps, so
// results depend on the step count only.
struct ParticlesCapability
{
    static constexpr const char *capabilityName = "physics_particles";

    std::uint32_t abi = physicsParticlesCapabilityAbi;

    void *self = nullptr;

    EmitterHandle (*createEmitter)(void *self, engine::physics_rigid::WorldHandle world,
        const EmitterDescription *description, ParticlesError *error);
    void (*destroyEmitter)(void *self, EmitterHandle emitter);

    // Re-tunes an emitter (the pool resizes; live particles keep their state).
    bool (*setEmitterDescription)(void *self, EmitterHandle emitter, const EmitterDescription *description);

    // Uniform wind acceleration (meters per second squared) blended by the
    // air drag; consumers can bridge a wind field's local sample per step.
    bool (*setWindAcceleration)(void *self, EmitterHandle emitter, const double wind[3]);

    // One-shot burst: spawns `count` particles from the point with the
    // given speed and isotropic spread (explosion debris, dust kicks,
    // splash droplets). Bursts reuse expired slots first; when the pool is
    // at capacity, the FIFO cursor slot respawns.
    bool (*burst)(void *self, EmitterHandle emitter, const double position[3], std::uint32_t count, double speed);

    // Particle state copies: fills up to capacity ParticleState records in
    // pool order and writes how many were filled; outRequired receives the
    // total pool size.
    bool (*copyParticleStates)(void *self, EmitterHandle emitter, ParticleState *outStates, std::uint32_t capacity,
        std::uint32_t *outFilled, std::uint32_t *outRequired);

    // Advances the emitter by exactly one fixed step of fixedDeltaSeconds.
    // Keep the delta constant across a running simulation.
    bool (*step)(void *self, EmitterHandle emitter, double fixedDeltaSeconds);

    // Accumulator convenience mirroring the rigid module's simulate: adds
    // elapsedSeconds and runs whole fixed steps at the configured fixed rate,
    // returning the number of steps executed.
    std::uint32_t (*simulate)(void *self, EmitterHandle emitter, double elapsedSeconds);

    bool (*particlesStatistics)(void *self, ParticlesStatistics *outStatistics);

    // ---- ABI 3 additions -------------------------------------------------------

    // Simulation LOD: writes the emitter's importance score in [0, 1]. The
    // module maps the score onto the five fidelity bands (Cached, Animation,
    // Procedural, Simplified, Full) with demotion hysteresis; emitters below
    // Full update every Nth fixed step (the interval per band is module
    // configuration) and one due step then covers the whole interval of wall
    // time, so emission rate and lifetime pacing stay correct while the
    // per-step cost drops. Cached emitters freeze until the score rises
    // again. Non-finite scores resolve to Cached.
    bool (*setEmitterImportance)(void *self, EmitterHandle emitter, double importance);
};

} // namespace engine::physics_particles
