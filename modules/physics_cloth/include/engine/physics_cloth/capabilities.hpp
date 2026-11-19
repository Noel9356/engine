// capabilities.hpp - public C capability surface of the physics_cloth module.
//
// XPBD cloth: a grid of particles held together by structural, shear, and
// bending distance constraints, stepped with fixed-timestep substepping.
// Constraints tear when stretched past a configurable strain tolerance, and
// particles repel each other through a deterministic self-collision pass.
// Cloth particles collide against a physics_rigid world's bodies and planes
// through that module's sphere-overlap query, so cloth drapes over static
// scenery and moving rigid bodies alike, and the deformed surface is
// sampleable through bilinear grid interpolation for rendering. Consumers
// acquire it via ParticipantContext::acquireCapability<ClothCapability>();
// the module declares physics_rigid as a required capability.

#pragma once

#include <engine/physics_rigid/capabilities.hpp>

#include <cstdint>

namespace engine::physics_cloth
{

// Increment when any public capability structure or function-table layout
// changes; consumers verify it after acquisition.
//
// ABI history:
//   1 - initial surface (cloth grid creation with structural, shear, and
//       bend constraints, pinning, dragging, wind, fixed-step stepping with
//       internal XPBD substeps, and statistics).
//   2 - appended ClothDescription.tearStrain,
//       ClothDescription.selfCollisionRadius,
//       ClothStatistics.brokenConstraintCount, and the sampleClothSurface
//       entry point. Everything from ABI 1 keeps its slot and meaning;
//       consumers check `abi >= 2` before touching the additions.
//   3 - appended the simulation-LOD and sleep surfaces: the
//       setClothImportance entry point, the ClothStatistics.lodSkippedSteps
//       and ClothStatistics.sleeping counters, and the settled-cloth
//       behavior of ClothDescription.sleepingEnabled. Everything from ABI 2
//       keeps its slot and meaning; consumers check `abi >= 3` before
//       touching the additions.
inline constexpr std::uint32_t physicsClothCapabilityAbi = 3u;

using ClothHandle = std::uint32_t;

inline constexpr ClothHandle invalidCloth = 0;

enum class ClothError : std::uint32_t
{
    None = 0,

    InvalidHandle,      // unknown cloth handle or particle index
    InvalidDescription, // null pointers, non-finite values, degenerate grid
    CapacityExceeded,   // configured cloth or particle budget exhausted
    SimulationFailure   // non-finite state entered during a step; the step is rolled back to its start state
};

struct ClothDescription
{
    // Grid dimensions in particles; both must be at least 2.
    std::uint32_t columns = 16;
    std::uint32_t rows    = 16;

    // Rest distance between adjacent particles (meters).
    double spacing = 0.25;

    // Placement: the grid lies in the local XZ plane around the transform,
    // with the first particle at the transform position and the grid extending
    // along local +X (columns) and local +Z (rows).
    engine::physics_rigid::RigidTransform transform;

    double particleMass = 0.2;

    // XPBD compliances (inverse stiffness; 0 = perfectly rigid). Stretch
    // edges keep the cloth's size, shear edges keep its in-plane shape, and
    // bend edges (two-apart neighbors) control how easily it folds.
    double stretchCompliance = 0.0;
    double shearCompliance   = 0.00002;
    double bendCompliance    = 0.0001;

    // Global velocity damping per second (0 = none).
    double damping = 0.02;

    // Contact radius of one particle against rigid geometry; 0 derives
    // spacing x 0.25.
    double particleRadius = 0.0;

    // Tangential velocity kept per second while a particle touches geometry
    // (0 = full stick, 1 = frictionless).
    double friction = 0.6;

    // Particles pinned to their creation positions (or moved through
    // pinParticleToPosition); pinned particles carry no mass.
    const std::uint32_t *pinnedParticles = nullptr;
    std::uint32_t        pinnedCount     = 0;

    // Settled cloth: when enabled, a cloth whose free particles all stay
    // below the module's configured sleep velocity for the configured time
    // stops integrating (and wakes on drag, pin, or wind changes). Enabled
    // by default; the thresholds are module configuration.
    bool sleepingEnabled = true;

    // ABI 2 additions (consumers check `abi >= 2` before touching them).

    // Maximum tolerated stretch ratio beyond rest length before a distance
    // constraint breaks permanently (structural, shear, and bend alike);
    // a constraint breaks when its length exceeds restLength x (1 + this)
    // while stretched. 0 disables tearing.
    double tearStrain = 0.0;

    // Self-collision sphere radius per particle (meters); particle pairs
    // closer than twice this radius repel each other after the constraint
    // iterations of every substep. 0 disables self-collision.
    double selfCollisionRadius = 0.0;
};

struct ClothInfo
{
    std::uint32_t columns         = 0;
    std::uint32_t rows            = 0;
    std::uint32_t particleCount   = 0;
    std::uint32_t constraintCount = 0;
};

struct ClothStatistics
{
    std::uint64_t stepsSimulated       = 0;
    std::uint64_t substepsExecuted     = 0;
    std::uint32_t lastContactCount     = 0;
    double        lastStepMilliseconds = 0.0;

    // ABI 2: distance constraints torn since creation (0 when tearing is
    // disabled); see ClothDescription.tearStrain.
    std::uint32_t brokenConstraintCount = 0;

    // ABI 3: fixed steps this cloth skipped through simulation LOD, and
    // whether the cloth is currently settled (asleep).
    std::uint64_t lodSkippedSteps = 0;
    bool          sleeping        = false;
};

struct ClothParticleDrag
{
    std::uint32_t particleIndex       = 0;
    double        stiffness           = 400.0; // spring acceleration per meter of error
    double        damping             = 8.0;   // velocity damping coefficient
    double        maximumAcceleration = 200.0;
};

// The "physics_cloth" capability. Calls are main-thread only; the module
// takes no internal locks. Positions and velocities are world-space meters
// and meters per second; all stepping happens in whole fixed steps with
// internal XPBD substeps, so results depend on the step count only.
struct ClothCapability
{
    static constexpr const char *capabilityName = "physics_cloth";

    std::uint32_t abi = physicsClothCapabilityAbi;

    void *self = nullptr;

    ClothHandle (*createCloth)(
        void *self, engine::physics_rigid::WorldHandle world, const ClothDescription *description, ClothError *error);
    void (*destroyCloth)(void *self, ClothHandle cloth);

    bool (*clothInfo)(void *self, ClothHandle cloth, ClothInfo *outInfo);

    // Particle state copies: positions and velocities are packed x,y,z per
    // particle in grid order (row-major over rows of columns).
    bool (*copyParticlePositions)(
        void *self, ClothHandle cloth, double *outPositions, std::uint32_t capacity, std::uint32_t *outRequired);
    bool (*copyParticleVelocities)(
        void *self, ClothHandle cloth, double *outVelocities, std::uint32_t capacity, std::uint32_t *outRequired);

    // Pinning: a pinned particle keeps its pin position (its creation
    // position until pinParticleToPosition moves it).
    bool (*setParticlePinned)(void *self, ClothHandle cloth, std::uint32_t particleIndex, bool pinned);
    bool (*pinParticleToPosition)(void *self, ClothHandle cloth, std::uint32_t particleIndex, const double position[3]);
    bool (*isParticlePinned)(void *self, ClothHandle cloth, std::uint32_t particleIndex, bool *outPinned);

    // Mouse-style grabbing: a soft spring drags the particle toward a target
    // point each substep. endParticleDrag releases it.
    bool (*beginParticleDrag)(void *self, ClothHandle cloth, const ClothParticleDrag *drag);
    bool (*setParticleDragTarget)(void *self, ClothHandle cloth, const double target[3]);
    bool (*endParticleDrag)(void *self, ClothHandle cloth);

    // Uniform wind acceleration (meters per second squared) added to every
    // particle's velocity each substep.
    bool (*setWindAcceleration)(void *self, ClothHandle cloth, const double wind[3]);

    // Advances the cloth by exactly one fixed step of fixedDeltaSeconds
    // (internal XPBD substeps per the module configuration). Keep the delta
    // constant across a running simulation.
    bool (*step)(void *self, ClothHandle cloth, double fixedDeltaSeconds);

    // Accumulator convenience mirroring the rigid module's simulate: adds
    // elapsedSeconds and runs whole fixed steps at the configured fixed rate,
    // returning the number of steps executed.
    std::uint32_t (*simulate)(void *self, ClothHandle cloth, double elapsedSeconds);

    bool (*clothStatistics)(void *self, ClothHandle cloth, ClothStatistics *outStatistics);

    // ABI 2: render-mesh sampling. Bilinear interpolation of the particle
    // grid at normalized coordinates (u, v clamped into [0, 1]; u runs along
    // the grid's first axis (columns), v along the second (rows)). outNormal
    // comes from the cross product of the two grid-direction tangents (the
    // central differences of the surrounding particles; +Y for degenerate
    // cases). Returns false for unknown handles or non-finite results.
    bool (*sampleClothSurface)(
        void *self, ClothHandle cloth, double u, double v, double outPosition[3], double outNormal[3]);

    // ---- ABI 3 additions -------------------------------------------------------

    // Simulation LOD: writes the cloth's importance score in [0, 1]. The
    // module maps the score onto the five fidelity bands (Cached, Animation,
    // Procedural, Simplified, Full) with demotion hysteresis; bands below
    // Full keep the cloth's fixed step size and reduce the internal XPBD
    // substep count instead (the small-steps schedule tolerates the coarser
    // solve), and Cached cloths freeze until the score rises again.
    // Non-finite scores resolve to Cached.
    bool (*setClothImportance)(void *self, ClothHandle cloth, double importance);
};

} // namespace engine::physics_cloth
