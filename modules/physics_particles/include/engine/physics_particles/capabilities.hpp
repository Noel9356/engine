// capabilities.hpp - public C capability surface of the physics_particles
// module.
//
// Deterministic particle systems for ambient and gameplay effects: rain,
// snow, dust, and fire emitters over a physics_rigid world. Every emitter
// keeps a fixed pool of particles whose parameters derive from an integer
// hash of (slot, generation), so identical emitters produce identical
// results without random-number state. Particles integrate per fixed step,
// respond to a uniform wind acceleration (consumers can bridge a wind
// field's local sample), and optionally collide against the rigid world
// through the sphere-overlap query (rain dies on impact, snow settles).
// Consumers acquire it via ParticipantContext::acquireCapability
// <ParticlesCapability>(); the module declares physics_rigid as a required
// capability.

#pragma once

#include <engine/physics_rigid/capabilities.hpp>

#include <cstdint>

namespace engine::physics_particles
{

// Increment when any public capability structure or function-table layout
// changes; consumers verify it after acquisition.
inline constexpr std::uint32_t physicsParticlesCapabilityAbi = 1u;

using EmitterHandle = std::uint32_t;

inline constexpr EmitterHandle invalidEmitter = 0;

enum class ParticlesError : std::uint32_t
{
    None = 0,

    InvalidHandle,      // unknown emitter handle or particle index
    InvalidDescription, // null pointers, non-finite values, negative extents
    CapacityExceeded,   // configured emitter or particle budget exhausted
    SimulationFailure   // non-finite state entered during a step; the emitter is left unmodified
};

// Particle behavior classes. Rain falls fast and dies at the ground; snow
// falls slowly, sways, and settles on contact; dust drifts with the wind
// inside its volume and respawns; fire rises buoyantly and flickers.
enum class EmitterKind : std::uint32_t
{
    Rain = 0,
    Snow,
    Dust,
    Fire
};

struct EmitterDescription
{
    EmitterKind kind = EmitterKind::Rain;

    // Spawn region: particles spawn at random points inside the axis-aligned
    // box around `origin` (derived per slot from the deterministic hash).
    double origin[3]          = {0.0, 8.0, 0.0};
    double halfExtents[3]     = {8.0, 0.5, 8.0};

    // Particles per second; the pool fills to maximumParticles and recycles
    // the oldest expired slot.
    double emitRate = 400.0;

    std::uint32_t maximumParticles = 2000;

    // Lifetime range in seconds; the per-slot value derives from the hash.
    double minimumLifetime = 1.5;
    double maximumLifetime = 2.5;

    // Base initial velocity (meters per second) and per-slot jitter fraction
    // (derived from the hash, applied to each axis).
    double initialVelocity[3] = {0.0, -9.0, 0.0};
    double velocityJitter     = 0.1;

    // Gravity scale against the world gravity (rain 1.0, fire -0.15 for
    // buoyant rise; a negative scale makes particles rise).
    double gravityScale = 1.0;

    // Air drag: exponential velocity damping per second toward the wind
    // acceleration target (snow and dust carry a high drag so they track
    // the wind; rain barely reacts).
    double airDrag = 0.5;

    // Sway amplitude (meters per second of horizontal drift oscillation)
    // and sway frequency (hertz), phase-derived from the slot hash. Snow
    // and dust use visible sway; rain and fire keep it small.
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

    // World collision through the rigid capability: rain dies on impact,
    // snow settles on impact (velocity zeroed, slides with friction 0.2,
    // resumes when the surface moves away), dust ignores impact.
    bool collideWithWorld = false;

    // Ground plane fallback: particles below this height die when they have
    // no world collision (rain ends at the floor even without colliders).
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
    // splash droplets). Bursts reuse expired slots first.
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
};

} // namespace engine::physics_particles
