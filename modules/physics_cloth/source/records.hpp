// records.hpp - the cloth module's runtime records: one cloth world per
// physics_rigid world, one cloth record with particles and XPBD constraints.

#pragma once

#include <engine/physics_cloth/capabilities.hpp>

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

    double windAcceleration[3] = {0.0, 0.0, 0.0};

    ClothParticleDrag drag;
    bool              dragActive = false;

    double simulatedTimeBudget = 0.0;

    std::uint64_t stepsSimulated       = 0;
    std::uint64_t substepsExecuted     = 0;
    std::uint32_t lastContactCount     = 0;
    double        lastStepMilliseconds = 0.0;

    // Per-step contact scratch (one entry per particle): 0 none, 1 contact,
    // plus the push-out normal.
    std::vector<std::uint8_t> contactFlags;
    std::vector<double>       contactNormals;
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
};

} // namespace engine::physics_cloth
