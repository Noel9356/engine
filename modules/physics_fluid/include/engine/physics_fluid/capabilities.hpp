// capabilities.hpp - public C capability surface of the physics_fluid module.
//
// Smoothed-particle hydrodynamics (SPH): a particle fluid with the classic
// Muller 2003 force set (poly6 density estimation, spiky-gradient pressure,
// Laplacian viscosity) over a spatial hash, stepped with fixed-timestep
// substeps. Fluid particles collide against a physics_rigid world's bodies
// and planes through that module's sphere-overlap query and can be confined
// to an axis-aligned bounds box (containers, tanks). Consumers acquire it
// via ParticipantContext::acquireCapability<FluidCapability>(); the module
// declares physics_rigid as a required capability.

#pragma once

#include <engine/physics_rigid/capabilities.hpp>

#include <cstdint>

namespace engine::physics_fluid
{

// Increment when any public capability structure or function-table layout
// changes; consumers verify it after acquisition.
inline constexpr std::uint32_t physicsFluidCapabilityAbi = 1u;

using FluidHandle = std::uint32_t;

inline constexpr FluidHandle invalidFluid = 0;

enum class FluidError : std::uint32_t
{
    None = 0,

    InvalidHandle,      // unknown fluid handle or particle index
    InvalidDescription, // null pointers, non-finite values, non-positive scales
    CapacityExceeded,   // configured fluid or particle budget exhausted
    SimulationFailure   // non-finite state entered during a step; the fluid is left unmodified
};

struct FluidDescription
{
    // Kernel radius (meters): the smoothing length h. Neighbors within h
    // exchange pressure and viscosity; roughly twice the particle spacing.
    double interactionRadius = 0.35;

    // Collision radius of one particle against rigid geometry (meters).
    double particleRadius = 0.12;

    double particleMass = 0.2; // kilograms

    // Rest density (kilograms per cubic meter) and gas stiffness k of the
    // equation of state p = k * (density - restDensity).
    double restDensity  = 1000.0;
    double gasStiffness = 8.0;

    // Dynamic viscosity coefficient of the Laplacian viscosity force.
    double viscosity = 3.5;

    // Global velocity damping per second (0 = none).
    double damping = 0.01;

    // Optional axis-aligned confinement box; particles inside are pushed
    // back when they cross a wall (a tank or container). useBounds = false
    // leaves the fluid free to collide with the rigid world only.
    bool   useBounds      = false;
    double boundsMinimum[3] = {-4.0, 0.0, -4.0};
    double boundsMaximum[3] = {4.0, 8.0, 4.0};
};

struct FluidInfo
{
    std::uint32_t particleCount = 0;
    double        interactionRadius = 0.0;
};

struct FluidStatistics
{
    std::uint64_t stepsSimulated       = 0;
    std::uint64_t substepsExecuted     = 0;
    std::uint32_t lastContactCount     = 0;
    double        lastAverageDensity   = 0.0;
    double        lastStepMilliseconds = 0.0;
};

// The "physics_fluid" capability. Calls are main-thread only; the module
// takes no internal locks. Positions and velocities are world-space meters
// and meters per second; all stepping happens in whole fixed steps with
// internal SPH substeps, so results depend on the step count only.
struct FluidCapability
{
    static constexpr const char *capabilityName = "physics_fluid";

    std::uint32_t abi = physicsFluidCapabilityAbi;

    void *self = nullptr;

    FluidHandle (*createFluid)(
        void *self, engine::physics_rigid::WorldHandle world, const FluidDescription *description, FluidError *error);
    void (*destroyFluid)(void *self, FluidHandle fluid);

    bool (*fluidInfo)(void *self, FluidHandle fluid, FluidInfo *outInfo);

    // Appends particles (positions and optional velocities packed x,y,z per
    // particle; a null velocities pointer starts them at rest). Returns the
    // first appended particle index.
    std::uint32_t (*addParticles)(void *self, FluidHandle fluid, const double *positions, const double *velocities,
        std::uint32_t count, FluidError *error);

    // Particle state copies: packed x,y,z per particle in insertion order.
    bool (*copyParticlePositions)(
        void *self, FluidHandle fluid, double *outPositions, std::uint32_t capacity, std::uint32_t *outRequired);
    bool (*copyParticleVelocities)(
        void *self, FluidHandle fluid, double *outVelocities, std::uint32_t capacity, std::uint32_t *outRequired);

    // Advances the fluid by exactly one fixed step of fixedDeltaSeconds
    // (internal SPH substeps per the module configuration). Keep the delta
    // constant across a running simulation.
    bool (*step)(void *self, FluidHandle fluid, double fixedDeltaSeconds);

    // Accumulator convenience mirroring the rigid module's simulate: adds
    // elapsedSeconds and runs whole fixed steps at the configured fixed rate,
    // returning the number of steps executed.
    std::uint32_t (*simulate)(void *self, FluidHandle fluid, double elapsedSeconds);

    bool (*fluidStatistics)(void *self, FluidHandle fluid, FluidStatistics *outStatistics);
};

} // namespace engine::physics_fluid
