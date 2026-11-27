// records.hpp - the physics_water module's plain-data runtime records: one
// ocean record (prepared Gerstner waves, ripple slots, registered bodies and
// their wake/volume bookkeeping) plus the shared WaterSettings.

#pragma once

#include <engine/physics_water/capabilities.hpp>
#include <engine/physics_rigid/capabilities.hpp>
#include <engine/physics_rigid/math.hpp>
#include <engine/util/simulation_lod.hpp>

#include <cstdint>
#include <vector>

namespace engine::physics_water
{
namespace math = engine::physics_rigid::math;

struct PreparedWave
{
    math::Vec3 direction;           // unit, XZ plane
    double     waveNumber           = 0.0;  // k = 2*pi / wavelength
    double     amplitude            = 0.0;
    double     horizontalAmplitude  = 0.0;  // Q * amplitude: Gerstner horizontal crest displacement (meters)
    double     omega                = 0.0;  // deep-water dispersion: omega = sqrt(g * k)
};

struct Ripple
{
    bool   active    = false;
    double centerX   = 0.0;
    double centerZ   = 0.0;
    double age       = 0.0;
    double strength  = 0.0; // initial ring amplitude (meters)
};

// Per-registration wake bookkeeping, kept parallel to the ocean's
// registeredBodies vector: the body's last seen position and the horizontal
// distance travelled since its last emitted wake ripple.
struct WakeState
{
    double lastPositionX     = 0.0;
    double lastPositionZ     = 0.0;
    double travelledDistance = 0.0; // meters since the last wake ripple
};
struct OceanRecord
{
    OceanHandle handle = invalidOcean;

    engine::physics_rigid::WorldHandle world = engine::physics_rigid::invalidWorld;

    OceanDescription description{};

    // Effective uniform current (meters per second); the y component is
    // always 0. Re-derived from the description on create and on
    // setOceanDescription unless the consumer set an explicit current
    // through setCurrentVelocity (which setOceanDescription then keeps).
    math::Vec3 currentVelocity{};
    bool       currentExplicit = false;

    // Simulated time of this ocean: the ripple pass advances it by the
    // interval-scaled step delta, so the wake submersion gate samples the
    // wave surface at the ocean's own clock even when the consumer never
    // calls applyBuoyancy.
    double simulatedTime = 0.0;

    std::vector<PreparedWave> waves;

    std::vector<Ripple> ripples;

    // Ascending body-handle order; wakeStates and bodyVolumes stay
    // index-parallel to it. bodyVolumes caches each registered body's shape
    // volume (shapes are immutable per body handle; a negative value marks
    // "not yet computed"), so the buoyancy pass does not re-derive it from
    // the description every step.
    std::vector<engine::physics_rigid::BodyHandle> registeredBodies;

    std::vector<WakeState> wakeStates;
    std::vector<double>    bodyVolumes;

    // Gravity of this ocean's rigid world, captured at creation (the buoyancy
    // pass pushes with -rho * V * g - the full VECTOR, so a tilted gravity
    // field pushes floats opposite its actual direction and scales with the
    // true magnitude; gravityMagnitude is |g| precomputed for the passes
    // that want the scalar).
    math::Vec3 gravity{0.0, -9.81, 0.0};
    double     gravityMagnitude = 9.81;

    // Simulation LOD: importance-driven fidelity band shared by the two
    // passes, each with its own staggered cadence (the phases derive from
    // the registration ordinal). A due ripple pass ages the ripples by the
    // interval delta; a due buoyancy pass scales its forces by the
    // interval, so the impulse per wall time matches the full-rate
    // schedule.
    double                           importance = 1.0;
    engine::util::SimulationFidelity lodBand    = engine::util::SimulationFidelity::Full;
    engine::util::SimulationCadence  rippleCadence{};
    engine::util::SimulationCadence  buoyancyCadence{};
    std::uint64_t                    lodSkippedSteps = 0;
};

struct WaterSettings
{
    std::uint32_t maximumOceans           = 16;
    std::uint32_t maximumRipplesPerOcean  = 64;
    std::uint32_t maximumRegisteredBodies = 4096;
    double        waterDensity            = 1000.0; // kilograms per cubic meter
    double        rippleSpeed             = 4.0;    // ring expansion (meters per second)
    double        rippleDecay             = 1.2;    // exponential decay per second
    double        rippleAttenuation       = 0.35;   // amplitude falloff per meter of radius
    double        rippleWavelength        = 1.6;    // ring crest spacing (meters)
    double        buoyancyScale           = 1.0;    // global buoyancy multiplier
    double        quadraticDragCoefficient = 1.8;   // quadratic water drag per square meter of drag area
    double        angularDragCoefficient  = 3.0;    // angular damping torque per (rad/s) of submerged body
    double        wakeSpeedThreshold      = 1.2;    // minimum horizontal speed for wake emission (meters per second)
    double        wakeSpacingMeters       = 1.5;    // travelled distance between wake ripples (meters)
    double        wakeStrengthScale       = 0.35;   // wake ripple amplitude per (meter per second) of speed
    double        wakeSubmergedFraction   = 0.95;   // bodies at or above this submersion emit no wakes

    // A ripple retires when its exponential envelope decays below this
    // amplitude fraction of its initial strength.
    double        rippleRetireEnvelope    = 1.0e-3;

    // Deep-water dispersion reference gravity (omega = sqrt(g k)): Earth's
    // 9.81 by default, independent of the scene gravity the buoyancy pass
    // reads per world.
    double        waveGravity            = 9.81;

    // Fixed-point iterations for the displaced-height query (more iterations
    // sharpen the inverse mapping of steep Gerstner surfaces).
    std::uint32_t gerstnerIterations    = 4;

    // Central-difference offset for the surface normal (meters): small
    // relative to the wave curvature, large enough to stay above the height
    // function's own precision.
    double        normalSampleEpsilon   = 0.05;

    // Simulation LOD: oceans below the Full band run their passes on a
    // staggered cadence; the per-band interval says how many fixed steps one
    // due pass covers (0 freezes that band, which is the Cached default).
    engine::util::SimulationLodSettings lod{};
};

} // namespace engine::physics_water
