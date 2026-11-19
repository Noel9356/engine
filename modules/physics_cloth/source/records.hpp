// records.hpp - the cloth module's runtime records: one cloth world per
// physics_rigid world, one cloth record with particles and XPBD constraints.

#pragma once

#include <engine/physics_cloth/capabilities.hpp>
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
};

// One (cell id, particle) pair for the self-collision spatial hash.
struct SelfCollisionEntry
{
    std::uint64_t cellId   = 0;
    std::uint32_t particle = 0;
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

    double windAcceleration[3] = {0.0, 0.0, 0.0};

    ClothParticleDrag drag;
    bool              dragActive = false;

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

    // Self-collision scratch (rebuilt per substep while enabled; reused so
    // stepping stays allocation-free after the first pass): the sorted (cell
    // id, particle) pairs, the unique ascending cell ids, and each bucket's
    // start offset into the sorted pairs.
    std::vector<SelfCollisionEntry> selfCollisionEntries;
    std::vector<std::uint64_t>      selfCollisionCells;
    std::vector<std::uint32_t>      selfCollisionCellStarts;
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
    bool                              lodEnabled   = false;
    engine::util::SimulationFidelityThresholds lodThresholds{};
    std::uint32_t                     lodIntervals[engine::util::kSimulationFidelityBandCount] = {0u, 8u, 4u, 2u, 1u};
};

} // namespace engine::physics_cloth
