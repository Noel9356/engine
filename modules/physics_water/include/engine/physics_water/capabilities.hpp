// capabilities.hpp - public C capability surface of the physics_water module.
//
// Ocean and large water surface simulation plus local interactions over a
// physics_rigid world: a Gerstner wave sum (deep-water dispersion, per-wave
// steepness with horizontal crest displacement, inverse-sampled surface
// height and normal), a uniform water current, localized ripple sources that
// propagate and decay over the surface (impacts, splashes, automatic wakes
// from fast registered bodies), and a buoyancy pass that registers rigid
// bodies and applies submersion-sampled buoyancy forces, current-relative
// water drag, and angular damping through the rigid module's apply entry
// points. Consumers acquire it via
// ParticipantContext::acquireCapability<WaterCapability>(); the module
// declares physics_rigid as a required capability.

#pragma once

#include <engine/physics_rigid/capabilities.hpp>

#include <cstdint>

namespace engine::physics_water
{

// Increment when any public capability structure or function-table layout
// changes; consumers verify it after acquisition.
//
// ABI history:
//   1 - initial surface (ocean and wave descriptions, the sinusoidal wave
//       sum with surface sampling, ripples, buoyancy registration and the
//       buoyancy pass, statistics).
//   2 - appended per-wave steepness with Gerstner horizontal displacement
//       and inverse height sampling behind sampleSurface, the
//       sampleSurfaceDisplaced entry point, automatic wake emission for fast
//       registered bodies inside stepRipples, OceanDescription.current with
//       the setCurrentVelocity entry point (current-relative drag in the
//       buoyancy pass), and the wake_speed_threshold / wake_spacing_meters /
//       wake_strength_scale configuration keys. Everything from ABI 1 keeps
//       its slot and meaning; consumers check `abi >= 2` before touching the
//       additions.
inline constexpr std::uint32_t physicsWaterCapabilityAbi = 2u;

using OceanHandle = std::uint32_t;

inline constexpr OceanHandle invalidOcean = 0;

enum class WaterError : std::uint32_t
{
    None = 0,

    InvalidHandle,      // unknown ocean handle or unregistered body
    InvalidDescription, // null pointers, non-finite values, wrong wave counts
    CapacityExceeded,   // configured ocean, wave, or ripple budget exhausted
    SimulationFailure   // a force application rejected by the rigid world
};

inline constexpr std::uint32_t maximumWavesPerOcean = 8;

// One Gerstner wave component: a trochoidal deep-water wave. The wavelength
// sets the dispersion (phase speed follows the deep-water relation from
// gravity), the amplitude the crest height, and the steepness the horizontal
// sharpening of the crests.
struct WaveDescription
{
    // Horizontal travel direction in the XZ plane (normalized on entry).
    double directionX = 1.0;
    double directionZ = 0.0;

    double wavelength = 30.0; // meters, > 0
    double amplitude  = 0.5;  // meters, >= 0

    // Gerstner steepness, clamped to [0, 1] on entry (out-of-range values
    // clamp instead of invalidating the description). The per-wave factor
    // Q = steepness / (k * amplitude * waveCount) contributes Q * k *
    // amplitude = steepness / waveCount of horizontal crest sharpening, so
    // the sum over the wave set stays <= 1 and displaced crests never loop;
    // 0 keeps the component purely sinusoidal.
    double steepness = 0.0;
};

struct OceanDescription
{
    // Mean surface height (meters); waves displace around it.
    double waterLevel = 0.0;

    WaveDescription waves[maximumWavesPerOcean];
    std::uint32_t   waveCount = 3;

    // Uniform horizontal current velocity (meters per second); the y
    // component is forced to 0. The buoyancy pass drags against the body
    // velocity relative to this current.
    double current[3] = {0.0, 0.0, 0.0};
};

struct WaterStatistics
{
    std::uint32_t oceanCount       = 0;
    std::uint32_t registeredBodies = 0;
    std::uint32_t activeRipples    = 0;
    std::uint64_t buoyancyApplications = 0; // bodies the last buoyancy pass touched
};

// The "physics_water" capability. Calls are main-thread only; the module
// takes no internal locks. Positions are world-space meters, time is the
// consumer's fixed-step clock in seconds.
struct WaterCapability
{
    static constexpr const char *capabilityName = "physics_water";

    std::uint32_t abi = physicsWaterCapabilityAbi;

    void *self = nullptr;

    OceanHandle (*createOcean)(
        void *self, engine::physics_rigid::WorldHandle world, const OceanDescription *description, WaterError *error);
    void (*destroyOcean)(void *self, OceanHandle ocean);

    // Re-tunes the wave set, level, and current (existing ripples survive).
    bool (*setOceanDescription)(void *self, OceanHandle ocean, const OceanDescription *description);

    // Surface query: writes the surface height at the world (x, z) and the
    // surface normal (unit, pointing out of the water) computed by central
    // differences of the Gerstner-displaced wave sum plus every active
    // ripple. Steepened crests displace sample points horizontally, so the
    // height at a fixed position is recovered through a fixed inverse
    // iteration (see the module overview).
    bool (*sampleSurface)(void *self, OceanHandle ocean, double x, double z, double timeSeconds, double *outHeight,
        double outNormal[3]);

    // Adds a local ripple: a ring that expands from the center at the
    // module's configured propagation speed and decays exponentially. The
    // oldest ripple is recycled once the configured budget is full.
    bool (*addRipple)(void *self, OceanHandle ocean, const double center[3], double strength);

    // Advances the ripple field by one fixed step (expansion and decay) and
    // runs the automatic wake emission: registered bodies moving faster than
    // the configured wake threshold shed ripples once per wake_spacing_meters
    // of travelled distance unless fully submerged.
    bool (*stepRipples)(void *self, OceanHandle ocean, double fixedDeltaSeconds);

    // Buoyancy registration: dynamic bodies only. Registered bodies receive
    // forces in applyBuoyancy until unregistered; destroying the body in the
    // rigid world makes the registration dormant (it is skipped and freed at
    // the next pass). Registrations are kept in ascending body-handle order,
    // and registered bodies also emit automatic wake ripples from stepRipples
    // while they move fast at the surface.
    bool (*registerBody)(void *self, OceanHandle ocean, engine::physics_rigid::BodyHandle body, WaterError *error);
    bool (*unregisterBody)(void *self, OceanHandle ocean, engine::physics_rigid::BodyHandle body);

    // Submersion query for one registered body: writes the estimated
    // submerged volume fraction in [0, 1] and the deepest sample point's
    // depth below the surface (meters, negative when fully above water).
    bool (*querySubmersion)(
        void *self, OceanHandle ocean, engine::physics_rigid::BodyHandle body, double timeSeconds, double *outFraction,
        double *outDepth);

    // The buoyancy pass: for every registered awake dynamic body, samples
    // the surface under the shape, applies the buoyancy force at the
    // submerged centroid, quadratic water drag against the ocean's uniform
    // current, and angular damping through the rigid module. Call once per
    // fixed update before stepping the rigid world.
    bool (*applyBuoyancy)(void *self, OceanHandle ocean, double timeSeconds, double fixedDeltaSeconds);

    bool (*waterStatistics)(void *self, WaterStatistics *outStatistics);

    // ---- ABI 2 additions ----------------------------------------------------

    // Displaced-surface query for renderers. position carries the horizontal
    // seed (position[0], position[2]; position[1] is validated but ignored),
    // and outDisplacement receives the total surface offset relative to the
    // seed: outDisplacement[0] = P.x - x, outDisplacement[1] = P.y +
    // rippleHeight, outDisplacement[2] = P.z - z, where P is the
    // Gerstner-displaced surface point found by the same fixed inverse
    // iteration sampleSurface uses and the ripple field is sampled at the
    // seed exactly as sampleSurface adds it. Adding the written offsets to a
    // grid vertex at (x, y, z) moves it onto the rendered surface.
    bool (*sampleSurfaceDisplaced)(
        void *self, OceanHandle ocean, const double position[3], double timeSeconds, double outDisplacement[3]);

    // Re-tunes the ocean's uniform horizontal current at runtime; values
    // must be finite and the y component is forced to 0. Takes effect in the
    // next applyBuoyancy pass (current-relative drag).
    bool (*setCurrentVelocity)(void *self, OceanHandle ocean, const double current[3]);
};

} // namespace engine::physics_water
