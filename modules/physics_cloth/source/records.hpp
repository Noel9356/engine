// records.hpp - the cloth module's runtime records: one cloth world per
// physics_rigid world, one cloth record with particles and XPBD constraints.

#pragma once

#include <engine/physics_cloth/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/util/particle_spatial_hash.hpp>
#include <engine/util/simulation_lod.hpp>
#include <engine/util/step_accumulator.hpp>

#include <cstdint>
#include <vector>

namespace engine::physics_cloth
{

// One XPBD distance constraint between two particles.
struct DistanceConstraint
{
    std::uint32_t particleA  = 0;
    std::uint32_t particleB  = 0;
    double        restLength = 0.0;
    double        compliance = 0.0;
    bool          broken     = false; // torn constraint; skipped from then on

    // XPBD Lagrange multiplier accumulator for the current substep. It resets
    // when each substep begins and accumulates across the solver iterations,
    // so repeated passes soften the constraint through the compliance term
    // exactly as the XPBD formulation prescribes instead of re-solving from
    // scratch every pass.
    double lambda = 0.0;
};

// One quad of the cloth grid (four particle indices in row-major order),
// walked per substep by the aerodynamic wind model. Faces are stored
// explicitly so the wind walk is a flat, cache-friendly loop instead of a
// per-cell index computation.
struct ClothFace
{
    std::uint32_t particleA = 0;
    std::uint32_t particleB = 0;
    std::uint32_t particleC = 0;
    std::uint32_t particleD = 0;
};

// One live cloth.
struct ClothRecord
{
    ClothHandle handle = invalidCloth;

    engine::physics_rigid::WorldHandle world = engine::physics_rigid::invalidWorld;

    std::uint32_t columns = 0;
    std::uint32_t rows    = 0;
    double        spacing = 0.0;

    std::vector<double>       positions;  // x,y,z per particle
    std::vector<double>       velocities; // x,y,z per particle
    std::vector<double>       previousPositions;
    std::vector<double>       inverseMasses;
    std::vector<std::uint8_t> pinned;
    std::vector<double>       pinPositions; // x,y,z per pinned particle target

    std::vector<DistanceConstraint> constraints;

    // Grid quads (one entry per (row, column) cell), walked by the
    // aerodynamic wind model; empty when that model is not in use.
    std::vector<ClothFace> faces;

    double particleMass = 0.2;

    double gravity[3] = {0.0, -9.81, 0.0}; // read from the rigid world at creation

    double particleRadius    = 0.0;
    double friction          = 0.6;
    double damping           = 0.02;
    double stretchCompliance = 0.0;
    double shearCompliance   = 0.0;
    double bendCompliance    = 0.0;

    double tearStrain          = 0.0; // stretch ratio beyond rest that permanently breaks a constraint (0 = off)
    double selfCollisionRadius = 0.0; // self-collision sphere radius per particle (0 = off)

    double windVelocity[3] = {0.0, 0.0, 0.0};

    // When true the wind vector acts as a far-field wind velocity for the
    // per-face drag-and-lift model; when false it is a uniform per-particle
    // acceleration (the ClothDescription.aerodynamicWind surface).
    bool aerodynamicWind = false;

    ClothParticleDrag drag;
    double            dragTarget[3] = {0.0, 0.0, 0.0}; // per-cloth grab target (two cloths never share one)
    bool              dragActive    = false;

    // Settled-cloth state: sleepTimer accumulates while every free particle
    // stays below the sleep velocity; sleeping cloths hold their state until
    // a drag, pin, or wind change wakes them.
    bool          sleepingEnabled = true;
    bool          sleeping        = false;
    double        sleepTimer      = 0.0;

    // Simulation LOD: importance-driven fidelity band and staggered cadence
    // (the phase derives from the registration ordinal).
    double                           importance = 1.0;
    engine::util::SimulationFidelity lodBand    = engine::util::SimulationFidelity::Full;
    engine::util::SimulationCadence  cadence{};

    // Fixed-step budget bookkeeping for simulate().
    engine::util::StepAccumulator accumulator;

    std::uint64_t stepsSimulated       = 0;
    std::uint64_t substepsExecuted     = 0;
    std::uint32_t lastContactCount     = 0;
    std::uint32_t brokenConstraintCount = 0;
    double        lastStepMilliseconds = 0.0;
    std::uint64_t lodSkippedSteps      = 0;

    // Per-step contact scratch (one entry per particle): 0 none, 1 contact,
    // plus the push-out normal.
    std::vector<std::uint8_t> contactFlags;
    std::vector<double>       contactNormals;

    // Pre-step copies of the particle state, restored when a step diverges
    // into non-finite values (reused buffers; capture is one copy per step).
    std::vector<double> preStepPositions;
    std::vector<double> preStepVelocities;

    // Self-collision spatial hash: the shared sorted-bucket structure rebuilt
    // per substep while enabled (reused buffers; a warmed-up record rebuilds
    // without allocating).
    engine::util::ParticleSpatialHash selfCollisionHash;
};

// Module-wide budgets read from the [physics_cloth] section.
struct ClothSettings
{
    std::uint32_t maximumCloths            = 64;
    std::uint32_t maximumParticles         = 200000;
    std::uint32_t maximumParticlesPerCloth = 65536;

    double        fixedRateHz             = 60.0;
    std::uint32_t substepsPerStep         = 8;
    std::uint32_t maximumStepsPerSimulate = 8;

    // Constraint solve passes per XPBD substep (the Gauss-Seidel walk over
    // the constraint list runs this many times per substep; 1 = the small
    // steps schedule, where stiffness comes from the substep count).
    std::uint32_t solverIterations = 1;

    // Aerodynamic wind response coefficients (the per-cloth flag is the
    // ClothDescription.aerodynamicWind surface): a1 scales the face drag
    // along the face normal, a2 = a3 the two lift shares along the face's
    // in-plane cross products (the Baraff-Witkin decomposition).
    double windDragCoefficient = 1.5;
    double windLiftCoefficient = 0.2;

    // Velocity clamp applied to the derived per-substep particle velocities;
    // keeps solver noise from escalating.
    double maximumVelocity = 30.0;

    // Settled cloth: a cloth whose free particles all stay below
    // sleepVelocityThreshold for timeToSleep seconds stops integrating until
    // an interaction wakes it.
    double sleepVelocityThreshold = 0.05;
    double timeToSleep            = 0.6;

    // Simulation LOD: fidelity bands keep the fixed step size and reduce the
    // internal substep count instead (Full = substepsPerStep, Simplified =
    // half, Procedural = quarter, Animation = one); the interval table says
    // how many fixed steps each band updates on (0 freezes that band, which
    // is the Cached default).
    engine::util::SimulationLodSettings lod{};
};

} // namespace engine::physics_cloth
