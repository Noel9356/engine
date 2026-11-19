// capabilities.hpp - public C capability surface of the physics_fields module.
//
// Environmental and external force fields over a physics_rigid world: a
// wind field (base direction and speed, gusts, spatial turbulence) sampled
// for effects and applied to bodies as aerodynamic drag, radial impulse
// blasts, and placed force volumes (uniform directional acceleration and
// radial magnetic-style attraction/repulsion). Consumers acquire it via
// ParticipantContext::acquireCapability<FieldsCapability>(); the module
// declares physics_rigid as a required capability and drives forces through
// that module's apply* entry points, so fields compose with any other force
// source and never integrate state themselves.

#pragma once

#include <engine/physics_rigid/capabilities.hpp>

#include <cstdint>

namespace engine::physics_fields
{

// Increment when any public capability structure or function-table layout
// changes; consumers verify it after acquisition.
// ABI history:
//   1 - initial surface (wind fields with gust/turbulence evaluation, wind
//       drag, force volumes, radial impulse blasts, statistics).
//   2 - appended the simulation-LOD surface: the setWindFieldImportance /
//       setForceVolumeImportance entry points and the
//       FieldsStatistics.lodSkippedSteps counter. Everything from ABI 1
//       keeps its slot and meaning; consumers check `abi >= 2` before
//       touching the additions.
inline constexpr std::uint32_t physicsFieldsCapabilityAbi = 2u;

using WindHandle        = std::uint32_t;
using ForceVolumeHandle = std::uint32_t;

inline constexpr WindHandle        invalidWind        = 0;
inline constexpr ForceVolumeHandle invalidForceVolume = 0;

enum class FieldsError : std::uint32_t
{
    None = 0,

    InvalidHandle,      // unknown wind, volume, or world handle
    InvalidDescription, // null pointers, non-finite values, negative extents
    CapacityExceeded,   // configured wind or volume budget exhausted
    SimulationFailure   // reserved; force rejections report through the return value
};

// One wind field. The velocity at a point and time is the base direction and
// speed modulated by a global gust (a slow oscillation plus per-point noise
// so gust fronts travel across the field) and spatial turbulence (two
// higher-frequency noise octaves sampled at the point). Everything derives
// from position and time only, so sampling is deterministic and repeatable.
struct WindFieldDescription
{
    // Unit direction the wind blows toward (normalized on entry).
    double direction[3] = {1.0, 0.0, 0.0};

    // Mean wind speed (meters per second) along the direction.
    double baseSpeed = 5.0;

    // Gust strength as a fraction of baseSpeed (0 = steady wind): the gust
    // cycle swings the local speed between (1 - gustStrength) and
    // (1 + gustStrength) times the base.
    double gustStrength = 0.3;

    // Gust cycle length in seconds; gust fronts additionally drift along the
    // wind direction at baseSpeed.
    double gustPeriodSeconds = 6.0;

    // Turbulence wavelength (meters) of the spatial noise octaves and their
    // speed amplitude as a fraction of baseSpeed. Zero turbulence keeps a
    // perfectly uniform field.
    double turbulenceWavelength = 24.0;
    double turbulenceStrength   = 0.15;
};

// One placed force volume. A Uniform volume accelerates every affected
// dynamic body by `acceleration` while its center stays inside the
// axis-aligned box around `center` (gravity wells, magnetic pads, anti-grav
// zones). A Radial volume pulls (negative strength) or pushes (positive
// strength) bodies with an inverse-square falloff bounded by `radius`
// (magnetic attraction, repulsors). `acceleration` is meters per second
// squared; forces scale with body mass inside the module.
struct ForceVolumeDescription
{
    enum class Kind : std::uint32_t
    {
        Uniform = 0,
        Radial
    };

    Kind kind = Kind::Uniform;

    double center[3] = {0.0, 0.0, 0.0};

    // Uniform: positive half extents of the affected box. Radial: unused
    // (the volume covers its radius sphere).
    double halfExtents[3] = {4.0, 4.0, 4.0};

    // Radial: influence radius (meters) around the center; 0 keeps the
    // volume inactive.
    double radius = 0.0;

    double acceleration[3] = {0.0, 0.0, 0.0};

    // Radial: signed strength (meters per second squared at one meter of
    // distance, falling off with the inverse square). Positive pushes.
    double radialStrength = 0.0;

    bool enabled = true;
};

struct FieldsStatistics
{
    std::uint32_t windFieldCount     = 0;
    std::uint32_t forceVolumeCount   = 0;
    std::uint64_t dragApplications   = 0; // bodies that received wind drag force
    std::uint64_t volumeApplications = 0; // bodies that received volume force

    // ABI 2: field passes skipped through simulation LOD.
    std::uint64_t lodSkippedSteps = 0;
};

// The "physics_fields" capability. Calls are main-thread only; the module
// takes no internal locks. Positions are world-space meters; wind velocities
// are meters per second; accelerations are meters per second squared.
struct FieldsCapability
{
    static constexpr const char *capabilityName = "physics_fields";

    std::uint32_t abi = physicsFieldsCapabilityAbi;

    void *self = nullptr;

    // ---- wind fields -------------------------------------------------------

    WindHandle (*createWindField)(void *self, engine::physics_rigid::WorldHandle world,
        const WindFieldDescription *description, FieldsError *error);
    void (*destroyWindField)(void *self, WindHandle wind);

    // Writes the wind velocity (meters per second) at the world point and
    // simulation time. `timeSeconds` is the consumer's fixed-step clock.
    bool (*sampleWindVelocity)(
        void *self, WindHandle wind, const double position[3], double timeSeconds, double outVelocity[3]);

    // Applies aerodynamic drag from this wind field to every awake dynamic
    // body of the field's world: each body receives a quadratic drag force
    // from the relative air speed at its center, scaled by its shape-derived
    // cross-section and drag coefficient. Call once per fixed update before
    // stepping the rigid world.
    bool (*applyWindDrag)(void *self, WindHandle wind, double timeSeconds, double fixedDeltaSeconds);

    // ---- force volumes ------------------------------------------------------

    ForceVolumeHandle (*createForceVolume)(void *self, engine::physics_rigid::WorldHandle world,
        const ForceVolumeDescription *description, FieldsError *error);
    void (*destroyForceVolume)(void *self, ForceVolumeHandle volume);

    // Re-tunes an existing volume (same validation as creation).
    bool (*setForceVolume)(void *self, ForceVolumeHandle volume, const ForceVolumeDescription *description);

    // Applies every enabled volume of its world to the dynamic bodies
    // currently inside the region. Call once per fixed update before
    // stepping the rigid world.
    bool (*applyForceVolumes)(void *self, double fixedDeltaSeconds);

    // ---- blasts -------------------------------------------------------------

    // Instantaneous radial impulse: every awake dynamic body of the world
    // within `radius` of the point receives an impulse along the away-from-
    // center direction with linear falloff (1 - distance/radius) scaled by
    // `impulseMagnitude` (newton-seconds at the center) and biased along
    // the world's up axis (+Y, the engine's gravity convention) by the
    // `upwardBias` fraction of its magnitude.
    bool (*applyRadialImpulse)(void *self, engine::physics_rigid::WorldHandle world, const double position[3],
        double impulseMagnitude, double radius, double upwardBias);

    bool (*fieldsStatistics)(void *self, FieldsStatistics *outStatistics);

    // ---- ABI 2 additions -------------------------------------------------------

    // Simulation LOD: writes a wind field's importance score in [0, 1]. The
    // module maps the score onto the five fidelity bands (Cached, Animation,
    // Procedural, Simplified, Full) with demotion hysteresis; fields below
    // Full run their drag pass on a staggered cadence whose due steps scale
    // the applied force by the interval (same total impulse per wall time),
    // and Cached fields apply nothing until the score rises again.
    // Non-finite scores resolve to Cached.
    bool (*setWindFieldImportance)(void *self, WindHandle wind, double importance);

    // Simulation LOD for force volumes, with the same band ladder and
    // interval-scaled application as the wind fields.
    bool (*setForceVolumeImportance)(void *self, ForceVolumeHandle volume, double importance);
};

} // namespace engine::physics_fields
